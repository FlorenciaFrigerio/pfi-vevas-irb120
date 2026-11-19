"""Cliente RWS (Robot Web Services 1.0) para mover el IRB 120 desde la PC con el módulo VEVAS_RWS.

Solo biblioteca estándar (urllib con digest y cookies). Sin ROS, sin numpy.

Adaptado de rws_mover.py / rws_j6.py del repo robotica-IMT342, rama proyectoHito3y4
(equipo/proyecto, ros2_ws/src/irb120_control/irb120_control/). Se copia el protocolo, las
rutas y las comprobaciones; no se copia la interfaz (panel, RViz).

Protocolo (idéntico al de LabJointRws V3):

    vvPcTarget  <- objetivo J1..J6 en grados (se lee de vuelta y se compara)
    vvPcSeq     <- vvPcSeq + 1       (ordena ejecutar el objetivo ya escrito)
    vvRbCodigo  -> 2 llegada, 1 moviendo, 0 listo, negativo = rechazo (-1 vacío, -2 límites,
                 -3 paso grande, -4 modo no permitido)
    vvRbSeq     -> copia de vvPcSeq cuando el robot terminó o rechazó

Rutas RWS usadas y su origen (R = copiada tal cual de la referencia, H = de herramientas/rws_irc5.py
ya probado en este proyecto, S = sin verificar con el robot, se prueba con lecturas de vuelta):

    GET  /rw/panel/opmode, /rw/panel/ctrlstate, /rw/rapid/execution       R/H
    GET  /rw/rapid/tasks/T_ROB1/pcp  (campo mal escrito «modulemame»)     R/H
    GET  /rw/rapid/modules?task=T_ROB1                                     R/H
    GET  /rw/motionsystem/mechunits/ROB_1/jointtarget  (rax_1..6)          H
    GET  /rw/rapid/tasks/T_ROB1/motion?resource=jointtarget (j1..j6)       R (respaldo)
    GET  /rw/rapid/symbol/data/RAPID/T_ROB1/VEVAS_RWS/<PERS>               R (con otro módulo)
    POST ...<PERS>?action=set  value=...                                   R
    POST /rw/mastership/rapid?action=request|release                       R (415 → Content-Type)
    POST /rw/rapid/modules/VEVAS_RWS?task=T_ROB1&action=set-module-text    R
    GET  /rw/rapid/modules/VEVAS_RWS?resource=module-text&task=T_ROB1      R
    POST /rw/rapid/tasks/T_ROB1/pcp?action=set-pp-routine                  R
    POST /rw/rapid/execution?action=start (cycle=once)                     R
    POST /rw/rapid/execution?action=stop | resetpp                         H
    POST /rw/panel/ctrlstate?action=setctrlstate  ctrl-state=motoron|off  H
    PUT  /fileservice/$HOME/VEVAS_RWS.mod                                  H
    POST /rw/rapid/tasks/T_ROB1?action=loadmod modulepath=...&replace=true H
    POST /users/rmmp, GET /users/rmmp/poll  (solo manual reducido)         R, sin probar aquí
"""
from __future__ import annotations

import contextlib
import hashlib
import http.cookiejar
import json
import math
import os
import re
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET

try:  # solo POSIX; en otros sistemas no hay candado entre procesos
    import fcntl
except ImportError:  # pragma: no cover
    fcntl = None

TAREA = "T_ROB1"
MODULO = "VEVAS_RWS"
RUTINA = "VevasServidor"
BASE_SIMBOLO = f"/rw/rapid/symbol/data/RAPID/{TAREA}/{MODULO}/"
SIMBOLOS_NUM = ("vvPcSeq", "vvPcFin", "vvRbSeq", "vvRbCodigo", "vvRbActivo")
RUTA_MODULO = f"/rw/rapid/modules/{MODULO}"
MASTERSHIP = "/rw/mastership/rapid"
PCP = f"/rw/rapid/tasks/{TAREA}/pcp"
FORM = "application/x-www-form-urlencoded"  # con «;v=2.0» el IRC5 contesta HTTP 415
START_FORM = {"regain": "continue", "execmode": "continue", "cycle": "once", "condition": "none",
              "stopatbp": "disabled", "alltaskbytsp": "false"}

# Límites nominales del IRB 120 (igual que Validar() del RAPID) y margen del módulo.
LIMITES_DEG = ((-165, 165), (-110, 110), (-110, 70), (-160, 160), (-120, 120), (-400, 400))
MARGEN_DEG = 2.0
PASO_RAPID_DEG = 45.0    # maxPasoDeg del módulo
PASO_PC_MAX_DEG = 40.0   # tope de la PC: 5° de margen para el error residual de llegada
PASO_TOL_DEG = 0.05      # ruido de medida entre el cálculo del tramo y su envío
DECIMALES = 3            # el IRC5 devuelve 6 cifras: con ejes ≥ 100° solo 3 decimales sobreviven
LECTURA_TOL_DEG = 2e-3
CODIGOS = {0: "listo", 1: "moviendo", 2: "llegada", -1: "objetivo vacío o no válido",
           -2: "fuera de límites con margen de 2°", -3: "algún eje cambia más de 45° en una orden",
           -4: "modo no permitido por el módulo"}

_PERS_VALOR = re.compile(r"^([ \t]*PERS[ \t]+(jointtarget|num)[ \t]+\w+[ \t]*:=[ \t]*)[^;\n]*;", flags=re.M)
_PERS_INICIAL = {"jointtarget": "[[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]]", "num": "0"}


class Falla(Exception):
    """Error de comunicación o de estado: el mensaje ya está pensado para mostrarse al usuario."""


# ----------------------------------------------------------------------------- utilidades de texto
def _buscar_valor(txt: str, clave: str):
    """Primer valor de `clave` en una respuesta RWS (JSON o XHTML)."""
    try:
        obj = json.loads(txt)
    except ValueError:
        m = re.search(rf'class="{re.escape(clave)}"[^>]*>\s*([^<]*)', txt)
        return m.group(1).strip() if m else None

    def rec(o):
        if isinstance(o, dict):
            if clave in o:
                return o[clave]
            for v in o.values():
                r = rec(v)
                if r is not None:
                    return r
        elif isinstance(o, list):
            for v in o:
                r = rec(v)
                if r is not None:
                    return r
        return None

    return rec(obj)


def _nombres(txt: str) -> list[str]:
    """Nombres de una lista RWS (módulos)."""
    try:
        obj = json.loads(txt)
    except ValueError:
        return re.findall(r'class="name"[^>]*>\s*([^<\s]+)', txt)
    out = []

    def rec(o):
        if isinstance(o, dict):
            if "name" in o and isinstance(o["name"], str):
                out.append(o["name"])
            elif "_title" in o and isinstance(o["_title"], str) and "_type" in o:
                out.append(o["_title"])
            for v in o.values():
                rec(v)
        elif isinstance(o, list):
            for v in o:
                rec(v)

    rec(obj)
    return out


def detalle_error(txt: str) -> str:
    """Código y mensaje del error ABB (sin cabeceras ni cookies)."""
    cod = _buscar_valor(txt, "code")
    msg = _buscar_valor(txt, "msg") or _buscar_valor(txt, "message")
    if cod is None and msg is None:
        return " ".join(txt.split())[:160]
    return f"código {cod}: {msg}"[:300]


def huella_modulo(fuente: str) -> str:
    """Huella del código del módulo. El IRC5 devuelve el texto con el VALOR ACTUAL de cada PERS en su
    declaración (p. ej. un vvPcTarget pendiente): ese valor es dato, no código, y se restituye el inicial."""
    fuente = fuente.replace("\r\n", "\n").replace("\r", "\n").strip()
    fuente = _PERS_VALOR.sub(lambda m: m.group(1) + _PERS_INICIAL[m.group(2)] + ";", fuente)
    return hashlib.sha256(fuente.encode()).hexdigest()


def diferencias(local: str, controlador: str) -> list[str]:
    """Diff unificado (código normalizado igual que la huella) entre el archivo local y lo que guardó el IRC5."""
    import difflib
    def norm(t):
        t = t.replace("\r\n", "\n").replace("\r", "\n").strip()
        return _PERS_VALOR.sub(lambda m: m.group(1) + _PERS_INICIAL[m.group(2)] + ";", t).splitlines()
    return list(difflib.unified_diff(norm(local), norm(controlador), "local", "IRC5", n=1, lineterm=""))


def arreglar_texto(t):
    """El IRC5 a veces manda UTF-8 que llega doblemente codificado («MÃ³dulo»)."""
    if isinstance(t, str) and "Ã" in t:
        try:
            return t.encode("latin-1").decode("utf-8")
        except (UnicodeEncodeError, UnicodeDecodeError):
            return t
    return t


def literal(q_deg) -> str:
    """jointtarget RAPID con ejes externos sin usar: [[q1..q6],[9E9 x6]] (15 cifras, como la referencia)."""
    q = [round(float(v), DECIMALES) for v in q_deg]
    return "[[" + ",".join(format(v, ".15g") for v in q) + "],[" + ",".join(format(9e9, ".15g") for _ in range(6)) + "]]"


def parsear_jointtarget(txt: str, permitir_vacio: bool = False):
    """'[[a..f],[ej]]' → lista de 6 grados (o None si es el objetivo vacío)."""
    try:
        v = json.loads(txt.strip().strip('"'))
        if not (isinstance(v, list) and len(v) == 2 and all(isinstance(g, list) and len(g) == 6 for g in v)):
            raise ValueError("formato jointtarget")
        if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for g in v for x in g):
            raise ValueError("valor no numérico o no finito")
    except (ValueError, TypeError) as e:
        raise Falla(f"jointtarget ilegible ({e}): {txt[:80]}") from e
    if all(x >= 8e9 for g in v for x in g):
        if permitir_vacio:
            return None
        raise Falla("El objetivo está vacío.")
    return [float(x) for x in v[0]]


def rechazo_local(objetivo_deg, actual_deg, paso_max_deg: float = PASO_PC_MAX_DEG):
    """Comprobación de la PC antes de escribir (la del robot se repite en RAPID).
    Devuelve None si está bien o (codigo, mensaje) con los mismos códigos que vvRbCodigo."""
    if len(objetivo_deg) != 6 or any(not math.isfinite(float(v)) for v in objetivo_deg):
        return -1, "El objetivo no tiene seis valores finitos."
    if paso_max_deg > PASO_RAPID_DEG:
        return -3, f"El paso máximo de la PC ({paso_max_deg}°) no puede superar el del RAPID ({PASO_RAPID_DEG}°)."
    for i, (q, (lo, hi)) in enumerate(zip(objetivo_deg, LIMITES_DEG), 1):
        if not lo + MARGEN_DEG <= q <= hi - MARGEN_DEG:
            return -2, f"J{i}={q:.3f}° fuera de [{lo + MARGEN_DEG}, {hi - MARGEN_DEG}] (límite con margen de {MARGEN_DEG}°)."
    peor = max(abs(a - b) for a, b in zip(objetivo_deg, actual_deg))
    if peor > paso_max_deg + PASO_TOL_DEG:
        return -3, f"Cambio de {peor:.3f}° mayor que el paso permitido de {paso_max_deg}°."
    return None


def subdividir(q_actual_deg, trayectoria_deg, paso_deg: float, minimo_deg: float = 0.005):
    """Waypoints (grados) para recorrer `trayectoria_deg` desde `q_actual_deg` con saltos de a lo sumo
    `paso_deg` por eje. Siempre incluye el último punto. Si dos muestras consecutivas distan más que el
    paso, se interpola linealmente entre ellas."""
    if not 0 < paso_deg <= PASO_PC_MAX_DEG:
        raise Falla(f"El paso debe estar entre 0 y {PASO_PC_MAX_DEG}° (el RAPID admite {PASO_RAPID_DEG}°).")
    pts = [list(map(float, p)) for p in trayectoria_deg]
    if not pts:
        raise Falla("La trayectoria está vacía.")

    def dist(a, b):
        return max(abs(x - y) for x, y in zip(a, b))

    out = []
    ultimo = list(map(float, q_actual_deg))
    i, n = 0, len(pts)
    while i < n:
        j = i
        while j < n and dist(ultimo, pts[j]) <= paso_deg:
            j += 1
        if j == i:  # la siguiente muestra ya queda lejos: interpolar hasta ella
            k = math.ceil(dist(ultimo, pts[i]) / paso_deg)
            for m in range(1, k + 1):
                out.append([a + (b - a) * m / k for a, b in zip(ultimo, pts[i])])
            ultimo = pts[i]
            i += 1
        else:
            out.append(pts[j - 1])
            ultimo = pts[j - 1]
            i = j
    # sin saltos mínimos intermedios, pero el último siempre queda
    filtrado = []
    previo = list(map(float, q_actual_deg))
    for k, p in enumerate(out):
        if k < len(out) - 1 and dist(previo, p) < minimo_deg:
            continue
        filtrado.append(p)
        previo = p
    return filtrado


# ----------------------------------------------------------------------------- cliente
class Cliente:
    def __init__(self, ip: str = "192.168.125.1", usuario: str = "Default User", clave: str = "robotics",
                 puerto: int = 80, timeout: float = 5.0, ruta_candado: str | None = None):
        self.base = f"http://{ip}:{puerto}"
        gestor = urllib.request.HTTPPasswordMgrWithDefaultRealm()
        gestor.add_password(None, self.base, usuario, clave)
        self._abridor = urllib.request.build_opener(
            urllib.request.HTTPDigestAuthHandler(gestor),
            urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()),
        )
        self.timeout = timeout
        self._http = threading.Lock()  # una operación HTTP a la vez: el IRC5 admite pocas sesiones
        self.on_joints = None          # callback(list[6] grados) cada vez que se lee la postura
        self._ruta_q = None
        self.rmmp_concedido = False
        self._rmmp_t = 0.0
        seguro = re.sub(r"[^A-Za-z0-9_.-]", "_", f"{ip}_{puerto}")
        self.ruta_candado = ruta_candado or os.path.join(tempfile.gettempdir(), f"vevas_rws_escritor_{seguro}.lock")
        self._esc_lock = threading.Lock()
        self._esc_n = 0
        self._esc_fd = None

    # ---- HTTP
    def _pedir(self, metodo, ruta, datos=None, tipo=FORM, params=None):
        url = self.base + ruta
        if params:
            url += ("&" if "?" in url else "?") + urllib.parse.urlencode(params)
        cuerpo = None
        if datos is not None:
            cuerpo = datos if isinstance(datos, bytes) else (
                urllib.parse.urlencode(datos) if isinstance(datos, dict) else str(datos)).encode("utf-8")
        req = urllib.request.Request(url, data=cuerpo, method=metodo)
        if cuerpo is not None:
            req.add_header("Content-Type", tipo)
        with self._http:
            try:
                with self._abridor.open(req, timeout=self.timeout) as r:
                    return r.status, r.read().decode("utf-8", errors="replace")
            except urllib.error.HTTPError as e:
                return e.code, e.read().decode("utf-8", errors="replace")
            except (urllib.error.URLError, OSError) as e:
                raise Falla(f"No hay respuesta de RWS en {self.base}: {e}") from e

    def get(self, ruta, **params) -> str:
        est, txt = self._pedir("GET", ruta, params={"json": "1", **params})
        if est != 200:
            raise Falla(f"GET {ruta} → HTTP {est}: {detalle_error(txt)}")
        return txt

    def post(self, ruta, params=None, datos=None, esperados=(200, 204), que="") -> tuple[int, str]:
        est, txt = self._pedir("POST", ruta, datos={} if datos is None else datos, params=params)
        if est not in esperados:
            raise Falla(f"{que or 'POST ' + ruta}: HTTP {est}; {detalle_error(txt)}")
        return est, txt

    def cerrar(self):
        with contextlib.suppress(Exception):
            self._pedir("GET", "/logout")

    # ---- candado entre herramientas (VEVAS y el nodo de MoveIt no pueden escribir a la vez)
    @contextlib.contextmanager
    def escritor(self):
        with self._esc_lock:
            if self._esc_n == 0 and fcntl is not None:
                fd = os.open(self.ruta_candado, os.O_CREAT | os.O_RDWR, 0o666)
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except OSError:
                    os.close(fd)
                    raise Falla("Otra herramienta (VEVAS o el nodo de MoveIt) está moviendo el robot por RWS. "
                                "Usá una sola a la vez.") from None
                self._esc_fd = fd
            self._esc_n += 1
        try:
            yield
        finally:
            with self._esc_lock:
                self._esc_n -= 1
                if self._esc_n == 0 and self._esc_fd is not None:
                    with contextlib.suppress(OSError):
                        fcntl.flock(self._esc_fd, fcntl.LOCK_UN)
                    os.close(self._esc_fd)
                    self._esc_fd = None

    # ---- lecturas
    def joints_deg(self) -> list[float]:
        """Ejes del robot en grados. Primero /rw/motionsystem/mechunits/ROB_1/jointtarget (rax_1..6) y, si
        no existe, /rw/rapid/tasks/T_ROB1/motion?resource=jointtarget (j1..j6, ruta de la referencia)."""
        rutas = [("/rw/motionsystem/mechunits/ROB_1/jointtarget", {}, "rax_{}"),
                 (f"/rw/rapid/tasks/{TAREA}/motion", {"resource": "jointtarget"}, "j{}")]
        if self._ruta_q is not None:
            rutas = [rutas[self._ruta_q]]
            idx = [self._ruta_q]
        else:
            idx = [0, 1]
        ultimo = None
        for k, (ruta, params, campo) in zip(idx, rutas):
            try:
                txt = self.get(ruta, **params)
                q = []
                for i in range(1, 7):
                    v = _buscar_valor(txt, campo.format(i))
                    if v is None:
                        raise Falla(f"La lectura de juntas no trae {campo.format(i)}")
                    q.append(float(v))
                if not all(math.isfinite(x) for x in q):
                    raise Falla("La lectura de juntas no es finita")
                self._ruta_q = k
                if self.on_joints is not None:
                    self.on_joints(list(q))
                return q
            except (Falla, ValueError) as e:
                ultimo = e
        raise Falla(str(ultimo))

    def modulos(self) -> list[str]:
        return _nombres(self.get("/rw/rapid/modules", task=TAREA))

    def leer_estado(self, con_juntas: bool = True) -> dict:
        e = {}
        e["opmode"] = str(_buscar_valor(self.get("/rw/panel/opmode"), "opmode") or "?").upper()
        e["ctrlstate"] = str(_buscar_valor(self.get("/rw/panel/ctrlstate"), "ctrlstate") or "?").lower()
        e["exec"] = str(_buscar_valor(self.get("/rw/rapid/execution"), "ctrlexecstate") or "?").lower()
        mods = self.modulos()
        e["modulos"] = mods
        e["modulo_vevas"] = MODULO in mods
        try:
            pcp = self.get(PCP)
            e["pp_rutina"] = str(_buscar_valor(pcp, "routinename") or "?")
            # RW6 trae la errata «modulemame»; se aceptan las variantes
            e["pp_modulo"] = str(_buscar_valor(pcp, "modulemame") or _buscar_valor(pcp, "modulename")
                                 or _buscar_valor(pcp, "module-name") or "?")
        except Falla:
            e["pp_rutina"] = e["pp_modulo"] = "?"
        if con_juntas:
            e["q_deg"] = self.joints_deg()
        return e

    def pp_en_servidor(self, e: dict) -> bool:
        return e["pp_rutina"].lower() == RUTINA.lower() and e["pp_modulo"] in (MODULO, "?")

    def _num(self, nombre: str) -> float:
        v = _buscar_valor(self.get(BASE_SIMBOLO + nombre), "value")
        try:
            return float(v)
        except (TypeError, ValueError) as e:
            raise Falla(f"{nombre} no es numérico: {v}") from e

    def estado_servidor(self) -> dict:
        """vvPcSeq, vvPcFin, vvRbSeq, vvRbCodigo, vvRbActivo (enteros). Falla si el módulo no está cargado."""
        return {n: int(self._num(n)) for n in SIMBOLOS_NUM}

    def _leer_objetivo(self):
        return parsear_jointtarget(str(_buscar_valor(self.get(BASE_SIMBOLO + "vvPcTarget"), "value")), permitir_vacio=True)

    # ---- mastership
    def _mastership(self, accion: str):
        """request → 200/204; release → también 403 «not held by caller» (RW6 puede liberarlo solo)."""
        ok = (200, 204) if accion == "request" else (200, 204, 403)
        est, txt = self._pedir("POST", MASTERSHIP, datos={}, params={"action": accion})
        if est in ok:
            return
        if accion == "request" and est in (400, 404):  # variantes de otras versiones (rws_irc5.py)
            for ruta in (f"/rw/mastership?action={accion}", f"/rw/mastership/edit?action={accion}"):
                est2, _ = self._pedir("POST", ruta, datos={})
                if est2 in ok:
                    return
        raise Falla(f"mastership {accion}: HTTP {est}; {detalle_error(txt)}")

    @contextlib.contextmanager
    def mastership(self):
        self._mastership("request")
        try:
            yield
        finally:
            with contextlib.suppress(Falla):
                self._mastership("release")

    # ---- registro de eventos del IRC5 (para mostrar el error real del controlador)
    def eventos(self, n: int = 8) -> list[dict]:
        """Últimos mensajes del registro de eventos (dominio 0). Cada uno: codigo, tipo, fecha, titulo, desc, causas, acciones."""
        txt = None
        for lang in ("es", "en"):
            try:
                txt = self.get("/rw/elog/0", lang=lang)
                break
            except Falla:
                continue
        if txt is None:
            return []
        try:
            d = json.loads(txt)
        except json.JSONDecodeError:
            return []
        estados = (d.get("_embedded") or {}).get("_state") or []
        out = []
        for e in estados[:n]:
            if not isinstance(e, dict):
                continue
            out.append({"codigo": e.get("code"), "tipo": {"1": "info", "2": "aviso", "3": "ERROR"}.get(str(e.get("msgtype")), str(e.get("msgtype"))),
                        "fecha": e.get("tstamp"), "titulo": arreglar_texto(e.get("title")), "desc": arreglar_texto(e.get("desc")),
                        "causas": arreglar_texto(e.get("causes")), "acciones": arreglar_texto(e.get("actions"))})
        return out

    def texto_modulo(self, modulo: str = MODULO) -> str:
        ruta = f"/rw/rapid/modules/{modulo}"
        txt = self.get(ruta, resource="module-text", task=TAREA)
        v = _buscar_valor(txt, "module-text")
        return v if isinstance(v, str) else ""

    # ---- módulo
    def _texto_modulo(self) -> str:
        txt = self.get(RUTA_MODULO, resource="module-text", task=TAREA)
        v = _buscar_valor(txt, "module-text")
        if isinstance(v, str) and v:
            return v
        ruta = _buscar_valor(txt, "file-path")
        if not ruta:
            raise Falla("La respuesta del módulo no trae su texto ni un temporal.")
        ruta = urllib.parse.unquote(str(ruta)).strip('"')
        if not re.fullmatch(r"/hd0a/TEMP[A-Za-z0-9_.-]+", ruta):  # nunca seguir rutas arbitrarias
            raise Falla("Ruta de temporal no reconocida.")
        for r in ("/fileservice" + ruta, "/fileservice/" + ruta):
            est, cuerpo = self._pedir("GET", r)
            if est == 404:
                continue
            if est != 200:
                raise Falla(f"GET del temporal RAPID: HTTP {est}")
            if len(cuerpo) > 100000 or not re.search(r"\bMODULE\s+" + MODULO + r"\b", cuerpo, flags=re.I):
                raise Falla("El temporal no contiene el módulo esperado.")
            return cuerpo
        raise Falla("No se pudo leer el temporal del módulo.")

    def instalar_modulo(self, ruta_mod: str, forzar: bool = False, eco=print) -> str:
        """Deja VEVAS_RWS cargado y con el texto de `ruta_mod`. Solo en AUTO, motores apagados y RAPID
        detenido (como la referencia: no se pide RMMP ni se cambia el modo desde la PC).
        Devuelve 'ya_instalado', 'cargado' o 'actualizado'."""
        with open(ruta_mod, encoding="utf-8") as f:
            fuente = f.read()
        if len(fuente) > 4800:
            raise Falla(f"{os.path.basename(ruta_mod)} tiene {len(fuente)} caracteres: RWS corta el texto del módulo en 5000 "
                        "(19/11: quedó cortado y RAPID marcó error). Acortalo (comentarios) por debajo de 4800.")
        if not re.search(r"^\s*MODULE\s+" + MODULO + r"\b", fuente, flags=re.M):
            raise Falla(f"{ruta_mod} no define MODULE {MODULO}.")
        e = self.leer_estado(con_juntas=False)
        if e["opmode"] != "AUTO":
            raise Falla(f"El controlador está en {e['opmode']}: la instalación por RWS exige AUTO, motores apagados "
                        "y RAPID detenido (no se cambia el modo desde la PC). Girá la llave a AUTO, o cargá "
                        f"{os.path.basename(ruta_mod)} a mano desde el FlexPendant.")
        if e["exec"] == "running":
            raise Falla("RAPID está en ejecución: detenelo antes de instalar el módulo.")
        if e["ctrlstate"] == "motoron":
            # RAPID está detenido: apagar motores para poder reemplazar el módulo (no mueve nada)
            self.post("/rw/panel/ctrlstate", {"action": "setctrlstate"}, {"ctrl-state": "motoroff"},
                      esperados=(200, 204), que="motores OFF")
            for _ in range(20):
                if self.leer_estado(con_juntas=False)["ctrlstate"] == "motoroff":
                    break
                time.sleep(0.25)
            e = self.leer_estado(con_juntas=False)
            eco("  Motores OFF para poder cargar el módulo.")
        if e["ctrlstate"] != "motoroff":
            raise Falla(f"Motores {e['ctrlstate']}: tienen que estar apagados para instalar el módulo.")
        try:
            lista = self.get("/rw/rapid/modules", task=TAREA)
        except Falla:
            lista = ""
        otros = [m for m in ("LabJointRws", "EGM_ROS") if f'"{m}"' in lista]
        if otros:
            eco(f"  Aviso: en T_ROB1 también está {', '.join(otros)} (de otro grupo). VEVAS_RWS usa nombres propios y no choca con ellos.")
        mia = huella_modulo(fuente)
        if e["modulo_vevas"] and not forzar:
            try:
                actual = huella_modulo(self._texto_modulo())
            except Falla as ex:
                raise Falla(f"{MODULO} ya está cargado pero no pude leer su texto para compararlo ({ex}). "
                            "Reintentá con --recargar-modulo.") from ex
            if actual == mia:
                eco(f"  {MODULO} ya está cargado y coincide con {os.path.basename(ruta_mod)}.")
                return "ya_instalado"
            eco(f"  {MODULO} está cargado pero su texto difiere: se reemplaza (set-module-text).")
            with self.mastership():
                self.post(RUTA_MODULO, {"task": TAREA, "action": "set-module-text"}, {"text": fuente},
                          esperados=(200, 204), que="set-module-text")
            leido = self._texto_modulo()
            if huella_modulo(leido) != mia:
                # No se reintenta (cada intento es otra edición en el controlador): se muestra la diferencia.
                eco("  El texto que guardó el controlador no es idéntico al archivo local. Diferencias:")
                for l in diferencias(fuente, leido)[:30]:
                    eco("    " + l)
                return "actualizado_con_diferencias"
            return "actualizado"
        # no está cargado (o se fuerza la recarga): subir el archivo y cargarlo
        est, txt = self._pedir("PUT", f"/fileservice/$HOME/{MODULO}.mod", datos=fuente, tipo="text/plain")
        if est not in (200, 201, 204):
            raise Falla(f"No se pudo subir {MODULO}.mod al controlador (HTTP {est}): {detalle_error(txt)}")
        eco(f"  Archivo subido a $HOME/{MODULO}.mod")
        intentos = []
        cargado = False
        with self.mastership():
            if forzar and e["modulo_vevas"]:
                # descargar la versión vieja (o con errores) antes de cargar la nueva
                est, txt = self._pedir("POST", f"/rw/rapid/tasks/{TAREA}", datos={"module": MODULO},
                                       params={"action": "unloadmod"})
                eco(f"  Descarga de la versión anterior de {MODULO}: HTTP {est}"
                    + ("" if est in (200, 204) else f" ({detalle_error(txt)})"))
            for path in (f"$HOME/{MODULO}.mod", f"HOME:/{MODULO}.mod"):
                est, txt = self._pedir("POST", f"/rw/rapid/tasks/{TAREA}", datos={"modulepath": path, "replace": "true"},
                                       params={"action": "loadmod"})
                intentos.append(f"loadmod [{path}] → HTTP {est}" + ("" if est in (200, 204) else f": {detalle_error(txt)}"))
                if est in (200, 204) and MODULO in self.modulos():
                    cargado = True
                    break
        if not cargado:
            raise Falla("No se pudo cargar el módulo por RWS. Intentos:\n  " + "\n  ".join(intentos) +
                        f"\nAlternativa: FlexPendant → Editor de programas → Módulos → Cargar módulo → {MODULO}.mod.")
        with contextlib.suppress(Falla):
            if huella_modulo(self._texto_modulo()) != mia:
                raise Falla("El módulo cargado no coincide con el archivo local.")
        return "actualizado" if e["modulo_vevas"] else "cargado"

    # ---- arranque / parada
    def _motores(self, estado: str, esperas: int = 20):
        self.post("/rw/panel/ctrlstate", {"action": "setctrlstate"}, {"ctrl-state": estado}, que=f"motores {estado}")
        for _ in range(esperas):
            if self.leer_estado(con_juntas=False)["ctrlstate"] == estado:
                return
            time.sleep(0.25)
        raise Falla(f"Los motores no pasaron a {estado}.")

    def preparar_y_arrancar(self, eco=print) -> dict:
        """Solo en AUTO: motores ON si hace falta, PP a VEVAS_RWS/VevasServidor (verificado) y Start (ciclo único).
        Devuelve el estado final. Si todo falla después de encender motores, los vuelve a apagar."""
        e = self.leer_estado(con_juntas=False)
        if e["opmode"] != "AUTO":
            raise Falla(f"El controlador está en {e['opmode']}, no en AUTO. La PC no cambia el modo (es la llave). "
                        f"En manual reducido: Depurar → PP a rutina → {RUTINA}, y Start con el habilitador.")
        if e["ctrlstate"] in ("guardstop", "emergencystop", "emergencystopreset", "sysfail"):
            raise Falla(f"Cadena de seguridad abierta o en falla (estado: {e['ctrlstate']}).")
        if not e["modulo_vevas"]:
            raise Falla(f"{MODULO} no está cargado: instalarlo primero (instalar_modulo).")
        if e["exec"] == "running":
            if self.pp_en_servidor(e):
                eco("  RAPID ya está corriendo en VevasServidor; no se vuelve a arrancar.")
                return e
            raise Falla(f"RAPID está ejecutando otro programa ({e['pp_modulo']}/{e['pp_rutina']}): detenerlo antes.")
        encendi = False
        try:
            if e["ctrlstate"] != "motoron":
                self._motores("motoron")
                encendi = True
                eco("  Motores ON.")
            with self.mastership():
                self.post(PCP, {"action": "set-pp-routine"}, {"module": MODULO, "routine": RUTINA, "userlevel": "false"},
                          que="set-pp-routine")
                e2 = self.leer_estado(con_juntas=False)
                if not self.pp_en_servidor(e2) or e2["pp_modulo"] == "?":
                    raise Falla(f"El PP no quedó en {MODULO}/{RUTINA} (está en {e2['pp_modulo']}/{e2['pp_rutina']}); NO se arranca.")
                eco(f"  PP confirmado en {MODULO}/{RUTINA}.")
                self.post("/rw/rapid/execution", {"action": "start"}, START_FORM, que="start RAPID")
            for _ in range(20):
                if self.leer_estado(con_juntas=False)["exec"] == "running":
                    return self.leer_estado(con_juntas=False)
                time.sleep(0.25)
            raise Falla("RAPID no pasó a running.")
        except BaseException:
            if encendi:
                with contextlib.suppress(Exception):
                    self._motores("motoroff", esperas=4)
            raise

    def esperar_servidor(self, timeout: float = 40.0, pausa_inicial: float = 1.0) -> bool:
        """Espera vvRbActivo = 1 (con confirmarEnPendant := TRUE el operador pulsa «Iniciar» antes de 30 s).
        vvRbActivo es una PERS: tras detener RAPID se queda en 1. La pausa inicial deja que VevasServidor
        arranque y lo ponga en 0 antes de mirarlo, para no darlo por activo con un valor viejo."""
        time.sleep(pausa_inicial)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            try:
                s = self.estado_servidor()
                if s["vvRbActivo"] == 1 and s["vvPcFin"] == 0:
                    return True
                if self.leer_estado(con_juntas=False)["exec"] != "running":
                    return False  # la rutina terminó (confirmación vencida o modo no permitido)
            except Falla:
                pass
            time.sleep(0.5)
        return False

    def terminar_servidor(self):
        """vvPcFin := 1: VevasServidor termina solo (el robot no se mueve)."""
        self._escribir("vvPcFin", "1")

    def parar(self) -> dict:
        """Stop de RAPID y motores OFF. No lanza: devuelve el estado final y los errores."""
        errores = []
        try:
            self.post("/rw/rapid/execution", {"action": "stop"}, {"stopmode": "stop"}, que="stop RAPID")
        except Falla as ex:
            errores.append(str(ex))
        try:
            self.post("/rw/panel/ctrlstate", {"action": "setctrlstate"}, {"ctrl-state": "motoroff"}, que="motores OFF")
        except Falla as ex:
            errores.append(str(ex))
        time.sleep(0.3)
        try:
            e = self.leer_estado(con_juntas=False)
        except Falla as ex:
            return {"ok": False, "errores": errores + [str(ex)]}
        return {"ok": e["exec"] != "running", "exec": e["exec"], "ctrlstate": e["ctrlstate"], "errores": errores}

    def pp_main(self):
        """Puntero de programa de vuelta a main (resetpp)."""
        e = self.leer_estado(con_juntas=False)
        if e["exec"] == "running":
            raise Falla("RAPID está corriendo; detenelo antes de mover el puntero.")
        with self.mastership():
            self.post("/rw/rapid/execution", {"action": "resetpp"}, None, que="resetpp")

    # ---- RMMP (solo manual reducido; sin probar con el robot en este proyecto)
    def _estado_rmmp(self) -> str:
        return str(_buscar_valor(self.get("/users/rmmp/poll"), "status"))

    def solicitar_rmmp(self, espera: float = 90.0, eco=print):
        self.post("/users/rmmp", {"json": "1"}, {"privilege": "modify"}, esperados=(200, 201, 202, 204), que="solicitud RMMP")
        eco(">>> En el FlexPendant aparece una solicitud de acceso remoto: el operador debe ACEPTARLA.")
        t0 = time.monotonic()
        while time.monotonic() - t0 < espera:
            st = self._estado_rmmp()
            if st == "GRANTED":
                self.rmmp_concedido = True
                self._rmmp_t = time.monotonic()
                return
            if st != "PENDING":
                raise Falla(f"RMMP no concedido (estado {st}): el operador lo rechazó o caducó.")
            time.sleep(0.25)
        raise Falla("RMMP no concedido a tiempo.")

    def mantener_rmmp(self, forzar: bool = False):
        if not self.rmmp_concedido:
            return
        if not forzar and time.monotonic() - self._rmmp_t < 2.0:
            return
        st = self._estado_rmmp()
        self._rmmp_t = time.monotonic()
        if st != "GRANTED":
            self.rmmp_concedido = False
            raise Falla(f"Se perdió RMMP (estado {st}). Volver a pedirlo con el operador.")

    # ---- órdenes
    def _escribir(self, nombre: str, valor: str):
        if nombre not in ("vvPcTarget", "vvPcSeq", "vvPcFin"):
            raise Falla("Solo se escriben vvPcTarget, vvPcSeq y vvPcFin.")
        self.mantener_rmmp()
        est, txt = self._pedir("POST", BASE_SIMBOLO + nombre, datos={"value": valor}, params={"action": "set"})
        if est not in (200, 204):
            pista = ""
            if est == 403:
                pista = " En manual hace falta RMMP aceptado en el FlexPendant."
                self.rmmp_concedido = False
            raise Falla(f"POST {nombre}: HTTP {est}; {detalle_error(txt)}.{pista}")

    def problemas_para_enviar(self, e: dict | None = None, s: dict | None = None) -> list[str]:
        """Lo que impide enviar órdenes ahora (lista vacía = listo)."""
        e = e or self.leer_estado(con_juntas=False)
        p = []
        if not e["modulo_vevas"]:
            p.append(f"módulo {MODULO} no cargado")
        if e["opmode"] == "MANF":
            p.append("modo manual 100 %: usar manual reducido o AUTO")
        elif e["opmode"] not in ("AUTO", "MANR"):
            p.append(f"modo {e['opmode']} no permitido")
        if e["ctrlstate"] != "motoron":
            p.append(f"motores {e['ctrlstate']}")
        if e["exec"] != "running":
            p.append("RAPID detenido")
        if not e["modulo_vevas"]:
            return p
        try:
            s = s or self.estado_servidor()
        except Falla as ex:
            return p + [f"no se leen las variables del servidor ({ex})"]
        if s["vvRbActivo"] != 1:
            p.append("VevasServidor no está activo (vvRbActivo=0): ¿falta «Iniciar» en el FlexPendant?")
        if s["vvPcFin"] != 0:
            p.append("vvPcFin=1: el servidor terminó; arrancarlo de nuevo")
        if s["vvPcSeq"] != s["vvRbSeq"]:
            p.append("hay una orden en curso (vvPcSeq≠vvRbSeq)")
        return p

    def enviar_objetivo(self, q_deg, timeout: float = 120.0, cancelar: threading.Event | None = None,
                        al_aceptar=None, eco=None, paso_max_deg: float = PASO_PC_MAX_DEG,
                        intervalo: float = 0.25) -> tuple[bool, int | None, str]:
        """Un objetivo articular absoluto (grados). Devuelve (ok, codigo, mensaje).

        codigo = vvRbCodigo del robot (2 llegada, negativos = rechazo); -1/-2/-3 también cuando la PC lo
        rechaza antes de escribir; None si la orden no llegó a decidirse (no listo, sin respuesta...).
        `al_aceptar(codigo)` se llama una vez cuando el robot ya la tomó (1 moviendo) o contestó.
        `cancelar` no interrumpe una orden ya escrita (el MoveAbsJ en curso termina): solo lo usa el
        llamador entre órdenes."""
        q_deg = [float(v) for v in q_deg]
        try:
            with self.escritor():
                return self._enviar(q_deg, timeout, al_aceptar, eco, paso_max_deg, intervalo)
        except Falla as ex:
            return False, None, str(ex)

    def _enviar(self, q_deg, timeout, al_aceptar, eco, paso_max_deg, intervalo):
        e = self.leer_estado(con_juntas=False)
        s = self.estado_servidor() if e["modulo_vevas"] else None
        problemas = self.problemas_para_enviar(e, s)
        if problemas:
            return False, None, "No listo: " + "; ".join(problemas)
        actual = self.joints_deg()
        rech = rechazo_local(q_deg, actual, paso_max_deg)
        if rech is not None:
            return False, rech[0], "Rechazado por la PC: " + rech[1]
        if e["opmode"] == "MANR" and not self.rmmp_concedido:
            self.solicitar_rmmp(eco=eco or print)
        q_deg = [round(v, DECIMALES) for v in q_deg]
        self._escribir("vvPcTarget", literal(q_deg))
        vuelta = self._leer_objetivo()
        if vuelta is None or max(abs(a - b) for a, b in zip(vuelta, q_deg)) > LECTURA_TOL_DEG:
            return False, None, "La lectura de vuelta de vvPcTarget no coincide; no se ordena el movimiento."
        seq = s["vvPcSeq"] + 1
        self._escribir("vvPcSeq", str(seq))
        t0 = time.monotonic()
        avisado = False
        detenido = 0
        while time.monotonic() - t0 < timeout:
            self.mantener_rmmp()
            q = self.joints_deg()
            hecho = int(self._num("vvRbSeq")) == seq
            codigo = int(self._num("vvRbCodigo"))
            falta = max(abs(a - b) for a, b in zip(q, q_deg))
            if eco:
                eco("  " + " ".join(f"J{i}={v:8.3f}" for i, v in enumerate(q, 1)) +
                    f" | falta {falta:7.3f}° | robot: {CODIGOS.get(codigo, codigo)}")
            if not avisado and (codigo == 1 or hecho):
                avisado = True
                if al_aceptar is not None:
                    al_aceptar(codigo)
            if hecho:
                if codigo != 2:
                    return False, codigo, f"El robot rechazó la orden {seq}: código {codigo} ({CODIGOS.get(codigo, '?')})."
                return True, 2, f"orden {seq}: llegada confirmada (falta {falta:.3f}°)"
            if self.leer_estado(con_juntas=False)["exec"] != "running":
                detenido += 1
                if detenido >= 2:
                    return False, None, (f"RAPID se detuvo con la orden {seq} sin terminar (¿paro, habilitador, error?). "
                                         "Queda pendiente (vvPcSeq≠vvRbSeq): no se envían más órdenes hasta resolverlo en el FlexPendant.")
            else:
                detenido = 0
            time.sleep(intervalo)
        return False, None, (f"Sin respuesta del robot a la orden {seq} en {timeout:.0f} s. "
                             "No enviar otra: revisar el FlexPendant.")
