"""IRC5 falso (RWS 1.0) para probar vevas/rws_cliente.py, vevas/backend_rws.py y el lanzador sin robot.

Emula lo que usa el cliente:
  - lecturas de modo, motores, ejecución RAPID, módulos, puntero de programa (con la errata «modulemame»)
    y juntas (rax_1..6 por mechunits y j1..j6 por motion);
  - mastership (HTTP 415 si el Content-Type no es application/x-www-form-urlencoded exacto; 403 si se
    libera sin tenerlo), carga de módulos (PUT fileservice + loadmod), set-module-text, set-pp-routine,
    start/stop/resetpp, motores ON/OFF (403 con la cadena de seguridad abierta);
  - variables PERS de VEVAS_RWS (GET y POST ?action=set) con RMMP obligatorio en manual reducido;
  - el servidor RAPID VevasServidor: Validar() igual que en el .mod (códigos -1/-2/-3/-4), vvRbCodigo 1→2,
    vvRbSeq, y las juntas se mueven hacia el objetivo a `vel_deg_s`;
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
        self._mov = None
        self.confirma_pendant = confirma_pendant
        self.mechunits = mechunits
        self.acepta_home_var = acepta_home_var
        self.seguridad_abierta = False
        self.mastership = False
        self.rmmp = "NONE"
        self.rmmp_auto = True
        self.pers = {"vvPcTarget": None, "vvPcSeq": 0, "vvPcFin": 0, "vvRbSeq": 0, "vvRbCodigo": 0, "vvRbActivo": 0, "vvRbTiempo": 0}
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
            p["vvRbCodigo"] = 0
            p["vvRbSeq"] = p["vvPcSeq"]
            p["vvRbActivo"] = 1
            self.srv_estado = "activo"
            return
        # activo
        if p["vvPcFin"] != 0:
            p["vvRbActivo"] = 0
            self.exec = "stopped"
            self.srv_estado = "libre"
            return
        if p["vvPcSeq"] != p["vvRbSeq"]:
            seq = p["vvPcSeq"]
            objetivo = list(p["vvPcTarget"]) if p["vvPcTarget"] else None
            if p["vvRbCodigo"] != 1:
                p["vvRbCodigo"] = self._validar(objetivo)
                if p["vvRbCodigo"] != 0:
                    self.rechazos.append((p["vvRbCodigo"], objetivo))
                    p["vvRbSeq"] = seq
                    return
                p["vvRbCodigo"] = 1
            if self.perfil_tiempo:
                ahora = time.monotonic()
                if self._mov is None or self._mov[0] != seq:
                    from vevas.rws_cliente import limites_c02, tiempo_movimiento
                    txt = self.textos.get("VEVAS_RWS")
                    vlim, k = limites_c02(txt) if txt and "vLim{6}" in txt else limites_c02(None)
                    T = tiempo_movimiento(objetivo, self.q, vlim, k)
                    p["vvRbTiempo"] = round(T, 6)
                    self._mov = (seq, ahora, list(self.q), list(objetivo), T * self.escala_tiempo)
                _, t0, q0, qf, T = self._mov
                u = min(1.0, (ahora - t0) / T)
                f = 3 * u * u - 2 * u * u * u
                self.q = [a + (b - a) * f for a, b in zip(q0, qf)]
                if u >= 1.0:
                    self.q = list(qf)
                    p["vvRbCodigo"] = 2
                    self.ordenes.append(list(qf))
                    p["vvRbSeq"] = seq
                    self._mov = None
                return
            # mover (straight line en el espacio articular)
            dist = [t - a for t, a in zip(objetivo, self.q)]
            mayor = max(abs(d) for d in dist)
            paso = self.vel * dt
            if mayor <= paso:
                self.q = list(objetivo)
                p["vvRbCodigo"] = 2
                self.ordenes.append(list(objetivo))
                p["vvRbSeq"] = seq
            else:
                self.q = [a + d * paso / mayor for a, d in zip(self.q, dist)]

    def _validar(self, q):
        if not self._modo_permitido():
            return -4
        if q is None or any(abs(v) > 1000 for v in q):
            return -1
        for v, (lo, hi) in zip(q, LIMITES):
            if v < lo + MARGEN or v > hi - MARGEN:
                return -2
        if any(abs(a - b) > MAX_PASO for a, b in zip(q, self.q)):
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
                    if "VEVAS_RWS" not in S.modulos or n not in S.pers:
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
                    if "VEVAS_RWS" not in S.modulos or n not in ("vvPcTarget", "vvPcSeq", "vvPcFin"):
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
