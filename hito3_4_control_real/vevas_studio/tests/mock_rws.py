"""IRC5 falso (RWS 1.0) para probar vevas/rws_cliente.py, vevas/backend_rws.py y el lanzador sin robot.

Emula lo que usa el cliente:
  - lecturas de modo, motores, ejecución RAPID, módulos, puntero de programa (con la errata «modulemame»)
    y juntas (rax_1..6 por mechunits y j1..j6 por motion);
  - mastership (HTTP 415 si el Content-Type no es application/x-www-form-urlencoded exacto; 403 si se
    libera sin tenerlo), carga de módulos (PUT fileservice + loadmod), set-module-text, set-pp-routine,
    start/stop/resetpp, motores ON/OFF (403 con la cadena de seguridad abierta);
  - variables PERS de VEVAS_RWS (GET y POST ?action=set) con RMMP obligatorio en manual reducido;
  - el servidor RAPID VevasServidor: Validar() igual que en el .mod (códigos -1/-2/-3/-4), vvRbCodigo 1→2,
    vvRbSeq, y las juntas se mueven hacia el objetivo a `vel_deg_s` (o con el tiempo \\T si perfil_tiempo);
  - movimiento continuo (vvPcZona = 1): cola de movimiento de 2 tramos como el pre-fetch de RAPID (el tramo en
    curso y uno planificado). Una orden con zona se acusa (vvRbCodigo 3, vvRbSeq) apenas entra en la cola;
    si la cola está llena, RAPID queda «bloqueado» en el MoveAbsJ hasta que termine el tramo en curso. Δq y
    el paso se validan contra el objetivo anterior. Si al terminar un tramo con zona no hay otro encolado,
    el robot para ahí (corner path failure: aviso 50024 en el registro, `fallas_esquina` += 1);
  - digest opcional (auth=("usuario","clave")) con cookie de sesión como el IRC5.

Uso:  python3 tests/mock_rws.py [puerto]   (queda corriendo)
"""
import hashlib
import json
import os
import re
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

LIMITES = ((-165, 165), (-110, 110), (-110, 70), (-160, 160), (-120, 120), (-400, 400))
MARGEN = 2.0
MAX_PASO = 45.0
FORM = "application/x-www-form-urlencoded"
SIMBOLOS = "/rw/rapid/symbol/data/RAPID/T_ROB1/VEVAS_RWS/"
_PERS = re.compile(r"^([ \t]*PERS[ \t]+(jointtarget|num)[ \t]+(\w+)[ \t]*:=[ \t]*)[^;\n]*;", flags=re.M)


def estado(o):
    return json.dumps({"_embedded": {"_state": [o]}})


def fmt6(v):
    return float(f"{v:.6g}")


class MockIRC5:
    def __init__(self, opmode="AUTO", vel_deg_s=400.0, confirma_pendant=True, auth=None, q=None,
                 mechunits=True, acepta_home_var=True, modulos=None, perfil_tiempo=False, escala_tiempo=1.0):
        self.lock = threading.RLock()
        self.opmode = opmode
        self.ctrlstate = "motoroff"
        self.exec = "stopped"
        self.modulos = list(modulos or ["BASE", "user", "Module1"])
        self.textos = {}
        self.elog = []
        self.niega_motoron = False   # True: como el IRC5 tras reiniciar en AUTO (403 a motores ON)
        self.fallas_texto = 0   # >0: contesta HTTP 500 «Error registering callback» esas veces
        self.fileservice = {}
        self.pp = ("Module1", "main")
        self.q = list(q or [0.0, -20.0, 30.0, 0.0, 40.0, 0.0])
        self.vel = vel_deg_s
        # perfil_tiempo: imita MoveAbsJ \T como el RAPID C02 (t = max(0.5, kPico·|Δq|/vLim), perfil 3s²−2s³,
        # pico = 1.5 × media). escala_tiempo < 1 simula un controlador que NO respeta el tiempo pedido.
        self.perfil_tiempo = perfil_tiempo
        self.escala_tiempo = escala_tiempo
        self._cola = []            # tramos en ejecución/planificados (máx. 2, como el pre-fetch de RAPID)
        self._pendiente = None     # orden validada cuyo MoveAbsJ espera lugar en la cola
        self._fine_seq = None      # RAPID bloqueado en un MoveAbsJ fine hasta la llegada
        self._previo = None        # objetivo anterior (referencia de Δq mientras hay zona)
        self._en_zona = False
        self._quieto = True
        self._t_paso = None
        self.movimientos = []      # {"seq", "q", "zona", "T"} de cada orden aceptada
        self.limites = LIMITES     # las pruebas pueden achicarlos para simular un rechazo del RAPID
        self.fallas_esquina = 0
        self.confirma_pendant = confirma_pendant
        self.mechunits = mechunits
        self.acepta_home_var = acepta_home_var
        self.seguridad_abierta = False
        self.mastership = False
        self.rmmp = "NONE"
        self.rmmp_auto = True
        self.pers = {"vvPcTarget": None, "vvPcSeq": 0, "vvPcFin": 0, "vvPcZona": 0, "vvRbSeq": 0, "vvRbCodigo": 0,
                     "vvRbActivo": 0, "vvRbTiempo": 0}
        self.ordenes = []      # objetivos recibidos y ejecutados con llegada
        self.rechazos = []     # (codigo, objetivo)
        self.log = []
        self.auth = auth
        self.sesiones = set()
        self.nonces = set()
        self.srv_estado = "libre"   # libre | confirmando | activo
        self.congelado = False       # True: RAPID «colgado» (no atiende vvPcSeq); sirve para probar órdenes en curso
        self._t_conf = 0.0
        self._vivo = True
        self.srv = None
        self.puerto = None

    # ------------------------------------------------------------------ simulación del robot y de RAPID
    def _bucle(self):
        dt = 0.02
        while self._vivo:
            with self.lock:
                self._paso(dt)
            time.sleep(dt)

    def _modo_permitido(self):
        return self.opmode in ("AUTO", "MANR")  # permitirAuto := TRUE

    def _paso(self, dt):
        ahora = time.monotonic()
        dt = dt if self._t_paso is None else min(0.2, ahora - self._t_paso)   # solo avanza con RAPID en marcha
        self._t_paso = ahora
        if self.congelado:
            return
        en_servidor = (self.exec == "running" and self.pp == ("VEVAS_RWS", "VevasServidor") and "VEVAS_RWS" in self.modulos)
        if not en_servidor:
            if self.srv_estado != "libre" and self.exec != "running":
                # RAPID detenido: la rutina queda donde estaba (una orden pendiente se retoma al volver a Start)
                if self.srv_estado == "confirmando":
                    self.srv_estado = "libre"
            return
        p = self.pers
        if self.srv_estado == "libre":
            p["vvRbActivo"] = 0
            if not self._modo_permitido():
                self.exec = "stopped"
                return
            self.srv_estado = "confirmando"
            self._t_conf = time.monotonic()
            return
        if self.srv_estado == "confirmando":
            if time.monotonic() - self._t_conf < 0.2:
                return
            if not self.confirma_pendant:  # el operador no pulsó «Iniciar»: VevasServidor termina
                self.exec = "stopped"
                self.srv_estado = "libre"
                return
            p["vvPcFin"] = 0
            if self._soporta_zona():
                p["vvPcZona"] = 0
            p["vvRbCodigo"] = 0
            p["vvRbSeq"] = p["vvPcSeq"]
            p["vvRbActivo"] = 1
            self._reiniciar_movimiento()
            self.srv_estado = "activo"
            return
        # activo: primero el robot avanza, después el programa RAPID
        self._mover(dt)
        if self._pendiente is not None:
            self._encolar()
            return
        if self._fine_seq is not None:
            return
        if p["vvPcFin"] != 0:
            p["vvRbActivo"] = 0
            self.exec = "stopped"
            self.srv_estado = "libre"
            return
        if p["vvPcSeq"] != p["vvRbSeq"]:
            seq = p["vvPcSeq"]
            objetivo = list(p["vvPcTarget"]) if p["vvPcTarget"] else None
            zona = self._soporta_zona() and p["vvPcZona"] == 1
            ref = self._previo if self._en_zona else list(self.q)
            cod = self._validar(objetivo, ref)
            if cod != 0:
                p["vvRbCodigo"] = cod
                self.rechazos.append((cod, objetivo))
                p["vvRbSeq"] = seq
                return
            if self.perfil_tiempo:
                from vevas.rws_cliente import T_MIN_S, limites_c02, tiempo_movimiento, tiempo_zona
                txt = self.textos.get("VEVAS_RWS")
                vlim, k = limites_c02(txt) if txt and "vLim{6}" in txt else limites_c02(None)
                T = tiempo_movimiento(objetivo, ref, vlim, k, (tiempo_zona(txt) or 0.2) if zona else T_MIN_S)
                p["vvRbTiempo"] = round(T, 6)
                Tm = T * self.escala_tiempo
            else:
                mayor = max(abs(a - b) for a, b in zip(objetivo, ref))
                Tm = mayor / self.vel if self.vel > 0 else float("inf")
            self._previo, self._en_zona = list(objetivo), zona
            if not zona:
                p["vvRbCodigo"] = 1
            self.movimientos.append({"seq": seq, "q": list(objetivo), "zona": zona, "T": Tm})
            self._pendiente = {"seq": seq, "qf": list(objetivo), "Tm": Tm, "zona": zona, "q0": None, "trans": 0.0}
            self._encolar()

    def _soporta_zona(self):
        txt = self.textos.get("VEVAS_RWS") or ""
        return "vvPcZona" in txt and "tZona" in txt

    def _reiniciar_movimiento(self):
        """Mover el PP (o arrancar VevasServidor) borra la cola de movimiento."""
        self._cola, self._pendiente, self._fine_seq = [], None, None
        self._previo, self._en_zona, self._quieto = None, False, True

    def _encolar(self):
        """MoveAbsJ: entra en la cola si hay lugar (tramo en curso + 1 planificado)."""
        s = self._pendiente
        if len(self._cola) >= 2:
            return
        self._cola.append(s)
        self._pendiente = None
        if s["zona"]:
            self.pers["vvRbCodigo"] = 3
            self.pers["vvRbSeq"] = s["seq"]
        else:
            self._fine_seq = s["seq"]

    def _mover(self, dt):
        resto = dt
        while self._cola and resto > 0:
            s = self._cola[0]
            if s["q0"] is None:
                s["q0"], s["reposo"] = list(self.q), self._quieto
                self._quieto = False
            avance = min(resto, max(0.0, s["Tm"] - s["trans"]))
            s["trans"] += avance
            resto -= avance
            u = 1.0 if s["Tm"] <= 0 else min(1.0, s["trans"] / s["Tm"])
            # desde reposo: 3u²−2u³ (pico 1.5 × media, como el MoveAbsJ \T aislado); en cadena con zona: lineal
            f = 3 * u * u - 2 * u * u * u if (self.perfil_tiempo and s["reposo"]) else u
            self.q = [a + (b - a) * f for a, b in zip(s["q0"], s["qf"])]
            if u < 1.0:
                return
            self.q = list(s["qf"])
            self._cola.pop(0)
            self.ordenes.append(list(s["qf"]))
            if not s["zona"]:
                self.pers["vvRbCodigo"] = 2
                self.pers["vvRbSeq"] = s["seq"]
                self._fine_seq = None
                self._quieto = True
                return
            if not self._cola and self._pendiente is None:
                # el punto siguiente no llegó a tiempo: el robot para en este punto (no es un error)
                self.fallas_esquina += 1
                self._quieto = True
                self.elog.insert(0, {"code": 50024, "msgtype": "2", "tstamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                                     "title": "Corner path failure", "desc": f"Orden {s['seq']}: punto de zona convertido en punto de paro."})
                return

    def _validar(self, q, ref=None):
        if not self._modo_permitido():
            return -4
        if q is None or any(abs(v) > 1000 for v in q):
            return -1
        for v, (lo, hi) in zip(q, self.limites):
            if v < lo + MARGEN or v > hi - MARGEN:
                return -2
        if any(abs(a - b) > MAX_PASO for a, b in zip(q, self.q if ref is None else ref)):
            return -3
        return 0

    # ------------------------------------------------------------------ texto del módulo como lo devuelve el IRC5
    def texto_modulo(self, nombre):
        fuente = self.textos[nombre]

        def sub(m):
            v = self.pers.get(m.group(3))
            if m.group(2) == "num" and v is not None:
                return f"{m.group(1)}{v};"
            if m.group(2) == "jointtarget":
                q = self.pers["vvPcTarget"]
                return m.group(1) + (self.fmt_jt(q) if q else "[[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]") + ";"
            return m.group(0)

        return _PERS.sub(sub, fuente).replace("\n", "\r\n")

    @staticmethod
    def fmt_jt(q):
        return "[[" + ",".join(repr(fmt6(v)) for v in q) + "],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]"

    # ------------------------------------------------------------------ servidor HTTP
    def iniciar(self, puerto=0):
        mock = self

        class H(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.0"

            def log_message(self, *a):
                pass

            def _r(self, code=200, body="", ctype="application/json", extra=None):
                b = body.encode()
                self.send_response(code)
                self.send_header("Content-Length", str(len(b)))
                self.send_header("Content-Type", ctype)
                for k, v in (extra or {}).items():
                    self.send_header(k, v)
                self.end_headers()
                self.wfile.write(b)

            def _err(self, code, msg, cod=-1073445880):
                self._r(code, json.dumps({"status": {"code": cod, "msg": msg}}))

            def _autenticado(self):
                if not mock.auth:
                    return True
                ck = self.headers.get("Cookie", "")
                m = re.search(r"-http-session-=([0-9a-f]+)", ck)
                if m and m.group(1) in mock.sesiones:
                    return True
                h = self.headers.get("Authorization", "")
                if h.startswith("Digest "):
                    d = dict(re.findall(r'(\w+)="?([^",]*)"?', h[7:]))
                    ha1 = hashlib.md5(f"{mock.auth[0]}:{d.get('realm')}:{mock.auth[1]}".encode()).hexdigest()
                    ha2 = hashlib.md5(f"{self.command}:{d.get('uri')}".encode()).hexdigest()
                    if d.get("qop") == "auth":
                        esperado = hashlib.md5(f"{ha1}:{d.get('nonce')}:{d.get('nc')}:{d.get('cnonce')}:auth:{ha2}".encode()).hexdigest()
                    else:
                        esperado = hashlib.md5(f"{ha1}:{d.get('nonce')}:{ha2}".encode()).hexdigest()
                    if d.get("username") == mock.auth[0] and d.get("response") == esperado and d.get("nonce") in mock.nonces:
                        sid = uuid.uuid4().hex
                        mock.sesiones.add(sid)
                        self._cookie = f"-http-session-={sid}; Path=/"
                        return True
                nonce = uuid.uuid4().hex
                mock.nonces.add(nonce)
                self._r(401, "", extra={"WWW-Authenticate": f'Digest realm="validusers@robapi.abb", qop="auth", nonce="{nonce}", opaque="x"'})
                return False

            # ---------------- GET
            def do_GET(self):
                if not self._autenticado():
                    return
                u = urlparse(self.path)
                p, q = u.path, parse_qs(u.query)
                with mock.lock:
                    mock.log.append("GET " + self.path)
                    return self._get(p, q)

            def _get(self, p, q):
                S = mock
                if p == "/logout":
                    return self._r(200)
                if p == "/rw/panel/opmode":
                    return self._r(200, estado({"_type": "pnl-opmode", "opmode": S.opmode}))
                if p == "/rw/elog/0":
                    return self._r(200, json.dumps({"_embedded": {"_state": S.elog}}))
                if p == "/rw/panel/ctrlstate":
                    return self._r(200, estado({"_type": "pnl-ctrlstate", "ctrlstate": S.ctrlstate}))
                if p == "/rw/rapid/execution":
                    return self._r(200, estado({"_type": "rap-execution", "ctrlexecstate": S.exec, "cycle": "once"}))
                if p == "/rw/rapid/modules" and q.get("task") == ["T_ROB1"]:
                    return self._r(200, json.dumps({"_embedded": {"_state": [{"_type": "rap-module-info-li", "_title": m, "name": m} for m in S.modulos]}}))
                if p.startswith("/rw/rapid/modules/") and q.get("resource") == ["module-text"] and S.fallas_texto > 0:
                    S.fallas_texto -= 1
                    return self._err(500, "rws_resource_rapid_module.cpp[2017] Error registering callback")
                if p.startswith("/rw/rapid/modules/") and q.get("resource") == ["module-text"]:
                    n = p.rsplit("/", 1)[1]
                    if n not in S.textos:
                        return self._err(404, "no existe el módulo")
                    return self._r(200, estado({"_type": "rap-module", "module-text": S.texto_modulo(n)}))
                if p == "/rw/rapid/tasks/T_ROB1/pcp":
                    return self._r(200, estado({"_type": "pcp-info", "modulemame": S.pp[0], "routinename": S.pp[1]}))
                if p == "/rw/motionsystem/mechunits/ROB_1/jointtarget":
                    if not S.mechunits:
                        return self._r(404, "")
                    return self._r(200, estado({"_type": "ms-jointtarget", **{f"rax_{i + 1}": f"{v:.6g}" for i, v in enumerate(S.q)}}))
                if p == "/rw/rapid/tasks/T_ROB1/motion" and q.get("resource") == ["jointtarget"]:
                    return self._r(200, estado({"_type": "rapid-jointtarget", **{f"j{i + 1}": f"{v:.6g}" for i, v in enumerate(S.q)},
                                                **{f"ej{i + 1}": "9E+09" for i in range(6)}}))
                if p.startswith(SIMBOLOS):
                    n = p[len(SIMBOLOS):]
                    if ("VEVAS_RWS" not in S.modulos or n not in S.pers
                            or (S.textos.get("VEVAS_RWS") and n not in S.textos["VEVAS_RWS"])):
                        return self._err(404, "símbolo no encontrado")
                    if n == "vvPcTarget":
                        v = S.fmt_jt(S.pers[n]) if S.pers[n] else "[[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]"
                    else:
                        v = str(S.pers[n])
                    return self._r(200, estado({"_type": "rap-data", "_title": f"RAPID/T_ROB1/VEVAS_RWS/{n}", "value": v}))
                if p == "/users/rmmp/poll":
                    return self._r(200, estado({"_type": "user-rmmp-poll", "status": S.rmmp}))
                self._r(404, "")

            # ---------------- PUT
            def do_PUT(self):
                if not self._autenticado():
                    return
                n = int(self.headers.get("Content-Length", 0))
                txt = self.rfile.read(n).decode("utf-8")
                with mock.lock:
                    mock.log.append("PUT " + self.path)
                    if self.path.startswith("/fileservice/"):
                        mock.fileservice[os.path.basename(self.path)] = txt
                        return self._r(201, "")
                self._r(404, "")

            # ---------------- POST
            def do_POST(self):
                if not self._autenticado():
                    return
                u = urlparse(self.path)
                p, q = u.path, parse_qs(u.query)
                n = int(self.headers.get("Content-Length", 0))
                body = parse_qs(self.rfile.read(n).decode("utf-8"), keep_blank_values=True)
                if self.headers.get("Content-Type", "") != FORM:
                    return self._err(415, "Unsupported Media Type", -1073414140)
                with mock.lock:
                    mock.log.append(f"POST {self.path} {sorted(body)}")
                    return self._post(p, q, body)

            def _post(self, p, q, body):
                S = mock
                a = (q.get("action") or [""])[0]
                if p == "/rw/mastership/rapid":
                    if a == "request":
                        S.mastership = True
                        return self._r(204, "")
                    if a == "release":
                        if not S.mastership:
                            return self._err(403, "The resource is not held by caller")
                        S.mastership = False
                        return self._r(204, "")
                if p == "/rw/panel/ctrlstate" and a == "setctrlstate":
                    est = body.get("ctrl-state", [""])[0]
                    if est == "motoron":
                        if S.seguridad_abierta:
                            return self._err(403, "cadena de seguridad abierta")
                        if S.niega_motoron:
                            return self._err(403, "rws_component_panel.cpp[116]: org_code: -4507")
                        S.ctrlstate = "motoron"
                    else:
                        S.ctrlstate = "motoroff"
                        if S.exec == "running":
                            S.exec = "stopped"
                    return self._r(204, "")
                if p == "/rw/rapid/execution":
                    if a == "start":
                        if S.ctrlstate != "motoron":
                            return self._err(403, "motores apagados")
                        if body.get("cycle") != ["once"]:
                            return self._err(400, "falta cycle=once")
                        S.exec = "running"
                        return self._r(204, "")
                    if a == "stop":
                        S.exec = "stopped"
                        return self._r(204, "")
                    if a == "resetpp":
                        if S.exec == "running" or not S.mastership:
                            return self._err(403, "no permitido")
                        S.pp = ("Module1", "main")
                        return self._r(204, "")
                if p == "/rw/rapid/tasks/T_ROB1/pcp" and a == "set-pp-routine":
                    if not S.mastership or S.exec == "running":
                        return self._err(403, "mastership o ejecución")
                    m, r = body.get("module", [""])[0], body.get("routine", [""])[0]
                    if m not in S.modulos:
                        return self._err(400, "módulo desconocido")
                    if any("ERROR_RAPID" in t for t in S.textos.values()):
                        S.elog.insert(0, {"code": 40192, "msgtype": "3", "tstamp": "2026-11-19 18:18:00",
                                          "title": "Error de referencia", "desc": "T_ROB1/VEVAS_RWS line 47: nombre ambiguo"})
                        return self._err(400, "programa con errores")
                    S.pp = (m, r)
                    S.srv_estado = "libre"   # la rutina empieza de nuevo (y vuelve a pedir confirmación)
                    S._reiniciar_movimiento()
                    return self._r(204, "")
                if p == "/rw/rapid/tasks/T_ROB1" and a == "unloadmod":
                    if not S.mastership or S.exec == "running":
                        return self._err(403, "mastership o ejecución")
                    m = body.get("module", [""])[0]
                    if m in S.modulos:
                        S.modulos.remove(m); S.textos.pop(m, None)
                    return self._r(204, "")
                if p == "/rw/rapid/tasks/T_ROB1" and a == "loadmod":
                    path = body.get("modulepath", [""])[0]
                    if not S.mastership or S.exec == "running" or S.opmode != "AUTO":
                        return self._err(403, "mastership, ejecución o modo")
                    base = os.path.basename(path.replace("HOME:", ""))
                    if path.startswith("$HOME/") and not S.acepta_home_var:
                        return self._err(400, "ruta no válida")
                    if not (path.startswith("$HOME/") or path.startswith("HOME:/")) or base not in S.fileservice:
                        return self._err(400, "archivo no encontrado")
                    nombre = base[:-4]
                    if nombre in S.modulos and body.get("replace", ["false"])[0] != "true":
                        return self._err(400, "el módulo ya existe")
                    if nombre not in S.modulos:
                        S.modulos.append(nombre)
                    S.textos[nombre] = S.fileservice[base]
                    return self._r(204, "")
                if p.startswith("/rw/rapid/modules/") and a == "set-module-text":
                    nombre = p.rsplit("/", 1)[1]
                    if not S.mastership or S.exec == "running" or S.opmode != "AUTO" or nombre not in S.textos:
                        return self._err(403, "mastership, ejecución, modo o módulo")
                    S.textos[nombre] = body.get("text", [""])[0][:5000]   # el IRC5 real corta en 5000 caracteres
                    return self._r(200, "")
                if p.startswith(SIMBOLOS) and a == "set":
                    n = p[len(SIMBOLOS):]
                    if ("VEVAS_RWS" not in S.modulos or n not in ("vvPcTarget", "vvPcSeq", "vvPcFin", "vvPcZona")
                            or (S.textos.get("VEVAS_RWS") and n not in S.textos["VEVAS_RWS"])):
                        return self._err(404, "símbolo")
                    if S.opmode in ("MANR", "MANF") and S.rmmp != "GRANTED":
                        return self._err(403, "RMMP necesario en manual")
                    v = body.get("value", [""])[0]
                    try:
                        if n == "vvPcTarget":
                            j = json.loads(v)
                            S.pers[n] = None if all(x >= 8e9 for g in j for x in g) else [fmt6(x) for x in j[0]]
                        else:
                            S.pers[n] = int(float(v))
                    except (ValueError, TypeError, IndexError):
                        return self._err(400, "valor no válido")
                    return self._r(204, "")
                if p == "/users/rmmp":
                    S.rmmp = "PENDING"
                    if S.rmmp_auto:
                        threading.Timer(0.3, lambda: setattr(S, "rmmp", "GRANTED")).start()
                    return self._r(204, "")
                self._r(404, "")

            def end_headers(self):
                c = getattr(self, "_cookie", None)
                if c:
                    self.send_header("Set-Cookie", c)
                super().end_headers()

        self.srv = ThreadingHTTPServer(("127.0.0.1", puerto), H)
        self.puerto = self.srv.server_address[1]
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        threading.Thread(target=self._bucle, daemon=True).start()
        return self.puerto

    def detener(self):
        self._vivo = False
        if self.srv:
            self.srv.shutdown()
            self.srv.server_close()
            self.srv = None


def arrancar(puerto, **kw):
    """Compatibilidad: arranca un IRC5 falso y devuelve el objeto."""
    m = MockIRC5(**kw)
    m.iniciar(puerto)
    return m


if __name__ == "__main__":
    kw = {}
    for a in sys.argv[2:]:
        if a == "--manr":
            kw["opmode"] = "MANR"
        if a == "--auth":
            kw["auth"] = ("Default User", "robotics")
    m = arrancar(int(sys.argv[1]) if len(sys.argv) > 1 else 8099, **kw)
    print(f"IRC5 falso en 127.0.0.1:{m.puerto}", flush=True)
    while True:
        time.sleep(1)
