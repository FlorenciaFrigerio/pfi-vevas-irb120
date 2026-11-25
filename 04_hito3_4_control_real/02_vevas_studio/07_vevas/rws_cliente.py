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

Movimiento continuo (streaming, módulo con vvPcZona y tZona):

    vvPcZona    <- 1 antes de los puntos intermedios de un recorrido, 0 antes del último (y en órdenes sueltas)
    con vvPcZona = 1 el RAPID hace MoveAbsJ ...\\T:=t,z1 y contesta enseguida vvRbCodigo = 3 («tomada en
    zona») + vvRbSeq: la PC escribe el punto siguiente mientras el robot recorre el actual. Δq, el paso
    máximo y t se calculan contra el objetivo ANTERIOR; el piso de t baja de 0.5 s a tZona (0.2 s). Si el
    punto siguiente no llega a tiempo, el robot para en ese punto (aviso 50024 «corner path failure» del
    IRC5): no es un error. El último punto va con fine (vvRbCodigo 1 → 2 al llegar).

Rutas RWS usadas y su origen (R = copiada tal cual de la referencia, H = de 02_herramientas/rws_irc5.py
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
SIMBOLO_ZONA = "vvPcZona"      # 1: vienen más puntos (MoveAbsJ con z1); 0: fine (último u orden suelta)
SIMBOLO_TIEMPO = "vvRbTiempo"  # tiempo (s) que usó el último MoveAbsJ \\T
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
CODIGOS = {0: "listo", 1: "moviendo", 2: "llegada", 3: "tomada en zona (vienen más puntos)", -1: "objetivo vacío o no válido",
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


# C02 · el RAPID mueve con MoveAbsJ \T y calcula el tiempo con estas constantes (5 % de la ficha ABB del
# IRB 120-3/0.6: 250,250,250,320,320,420 °/s). La PC las lee del mismo .mod para no duplicarlas.
VLIM_DEG_S_DEFECTO = (12.5, 12.5, 12.5, 16.0, 16.0, 21.0)
K_PICO_DEFECTO = 2.5
T_MIN_S = 0.5
# Piso de tiempo de los tramos con zona (streaming). Con 0.5 s un recorrido de 60 puntos chicos tardaría
# 30 s aunque cada tramo sea de décimas de grado; 0.2 s sigue siendo lento para la junta (Δq ≤ 0.08·vLim·t
# en esos tramos) y deja a la PC ~0.2 s para escribir el punto siguiente. El tope kPico·|Δq|/vLim no cambia.
T_ZONA_DEFECTO = 0.2


def limites_c02(fuente: str | None = None) -> tuple[tuple[float, ...], float]:
    """(vLim por eje en °/s, kPico) declarados en el módulo RAPID. Falla si el texto no los trae."""
    if fuente is None:
        return VLIM_DEG_S_DEFECTO, K_PICO_DEFECTO
    mv = re.search(r"CONST\s+num\s+vLim\{6\}\s*:=\s*\[([^\]]*)\]", fuente)
    mk = re.search(r"CONST\s+num\s+kPico\s*:=\s*([0-9.]+)", fuente)
    if not mv or not mk:
        raise Falla("El módulo no declara vLim{6}/kPico: no es la versión con MoveAbsJ \\T (C02).")
    vlim = tuple(float(x) for x in mv.group(1).split(","))
    if len(vlim) != 6 or any(v <= 0 for v in vlim):
        raise Falla("vLim{6} del módulo no tiene seis valores positivos.")
    return vlim, float(mk.group(1))


def tiempo_zona(fuente: str | None) -> float | None:
    """Piso tZona del módulo si soporta movimiento continuo (vvPcZona + tZona); None si es la versión vieja."""
    if not fuente:
        return None
    m = re.search(r"CONST\s+num\s+tZona\s*:=\s*([0-9.]+)", fuente)
    if not m or not re.search(r"PERS\s+num\s+vvPcZona\b", fuente):
        return None
    return float(m.group(1))


def tiempo_movimiento(objetivo_deg, actual_deg, vlim=VLIM_DEG_S_DEFECTO, k_pico=K_PICO_DEFECTO,
                      t_min: float = T_MIN_S) -> float:
    """Mismo cálculo que Tiempo() del RAPID: t = max(t_min, kPico·|Δq_i|/vLim_i); t_min = 0.5 s (fine) o
    tZona (tramo con zona de un recorrido continuo)."""
    t = t_min
    for a, b, v in zip(objetivo_deg, actual_deg, vlim):
        t = max(t, k_pico * abs(float(a) - float(b)) / v)
    return t


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


def subdividir(q_actual_deg, trayectoria_deg, paso_deg: float):
    """Conserva los vértices y cambios de sentido. Solo fusiona puntos colineales ordenados."""
    if not 0 < paso_deg <= PASO_PC_MAX_DEG:
        raise Falla(f"El paso debe estar entre 0 y {PASO_PC_MAX_DEG}°.")
    origen = list(map(float, q_actual_deg))
    if len(origen) != 6 or not all(math.isfinite(x) for x in origen):
        raise Falla("El origen necesita seis valores finitos.")
    vertices = [origen]
    final = None
    for muestra in trayectoria_deg:
        final = list(map(float, muestra))
        if len(final) != 6 or not all(math.isfinite(x) for x in final):
            raise Falla("Cada waypoint necesita seis valores finitos.")
        if final == vertices[-1]:
            continue
        if len(vertices) >= 2:
            a, b = vertices[-2:]
            d = [z - x for x, z in zip(a, final)]
            eje = max(range(6), key=lambda i: abs(d[i]))
            if d[eje] != 0:
                u = (b[eje] - a[eje]) / d[eje]
                if 0 <= u <= 1 and all(abs(x + u * dx - y) <= 1e-10 for x, dx, y in zip(a, d, b)):
                    vertices.pop()
        vertices.append(final)
    if final is None:
        raise Falla("La trayectoria está vacía.")
    out = []
    for a, b in zip(vertices, vertices[1:]):
        n = math.ceil(max(abs(y - x) for x, y in zip(a, b)) / paso_deg)
        for k in range(1, n):
            out.append([x + (y - x) * k / n for x, y in zip(a, b)])
        out.append(b)
    return out or [final]


def muestrear_recorrido(q_actual_deg, trayectoria_deg, paso_max_deg: float = 10.0, tol_deg: float = 0.1):
    """Puntos para el movimiento continuo. Recorre las muestras (densas) de la trayectoria y deja un punto solo
    donde la cuerda desde el último punto se apartaría más de `tol_deg` (en cualquier junta) de alguna muestra
    intermedia, o donde el tramo pasaría de `paso_max_deg`. Así los tramos rectos en el espacio articular se
    fusionan (un punto cada ≤ paso_max_deg) y las curvas quedan con un punto cada ~0.5–2° (cuerda ~√(8·R·tol));
    los vértices y cambios de sentido se conservan. Un salto entre muestras mayor que el paso se subdivide."""
    if not 0 < paso_max_deg <= PASO_PC_MAX_DEG:
        raise Falla(f"El paso debe estar entre 0 y {PASO_PC_MAX_DEG}°.")
    origen = [float(x) for x in q_actual_deg]
    if len(origen) != 6 or not all(math.isfinite(x) for x in origen):
        raise Falla("El origen necesita seis valores finitos.")
    pts = [origen]
    for muestra in trayectoria_deg:
        q = [float(x) for x in muestra]
        if len(q) != 6 or not all(math.isfinite(x) for x in q):
            raise Falla("Cada waypoint necesita seis valores finitos.")
        if q != pts[-1]:
            pts.append(q)
    if len(trayectoria_deg) == 0:
        raise Falla("La trayectoria está vacía.")
    if len(pts) == 1:
        return [list(pts[0])]
    def desvio(a, d, dd, p):
        u = 0.0 if dd <= 0 else min(1.0, max(0.0, sum((x - y) * z for x, y, z in zip(p, a, d)) / dd))
        return max(abs(x - (y + u * z)) for x, y, z in zip(p, a, d))

    n = len(pts)
    elegidos = []
    i = 0
    while i < n - 1:
        a = pts[i]
        mejor = i + 1
        for j in range(i + 2, n):
            d = [y - x for x, y in zip(a, pts[j])]
            if max(abs(x) for x in d) > paso_max_deg:
                break
            dd = sum(x * x for x in d)
            if any(desvio(a, d, dd, pts[m]) > tol_deg for m in range(i + 1, j)):
                break
            mejor = j
        elegidos.append(mejor)
        i = mejor
    out = []
    previo = pts[0]
    for j in elegidos:
        b = pts[j]
        m = max(1, math.ceil(max(abs(y - x) for x, y in zip(previo, b)) / paso_max_deg - 1e-9))
        for k in range(1, m):
            out.append([x + (y - x) * k / m for x, y in zip(previo, b)])
        out.append(list(b))
        previo = b
    return out


def muestrear_con_paradas(q_actual_deg, trayectoria_deg, paradas=(), paso_max_deg: float = 10.0,
                          tol_deg: float = 0.1):
    """muestrear_recorrido por trozos, cortando en las `paradas` (índices de muestras de la trayectoria con
    velocidad nula en el plan: esquinas del cuadrado/triángulo, punta del corazón). Cada parada queda como
    punto propio y va con fine, así la zona no redondea la esquina. Devuelve (puntos, fines): `fines` son
    los índices de `puntos` que van con fine; siempre incluye el último."""
    tray = list(trayectoria_deg)
    if not tray:
        raise Falla("La trayectoria está vacía.")
    cortes = sorted({int(k) for k in paradas if 0 <= int(k) < len(tray) - 1})
    puntos, fines = [], []
    desde, inicio = [float(x) for x in q_actual_deg], 0
    for fin in cortes + [len(tray) - 1]:
        trozo = muestrear_recorrido(desde, tray[inicio:fin + 1], paso_max_deg, tol_deg)
        if not (len(trozo) == 1 and trozo[0] == desde):   # trozo quieto: no agrega un tramo nulo
            puntos += trozo
        if puntos and (not fines or fines[-1] != len(puntos) - 1):
            fines.append(len(puntos) - 1)
        desde, inicio = [float(x) for x in tray[fin]], fin + 1
    if not puntos:
        return [desde], [0]
    return puntos, fines


class CancelacionOrden(threading.Event):
    """Serializa la cancelación con la entrega de vvPcSeq al transporte."""
    def __init__(self):
        super().__init__()
        self.compromiso = threading.Lock()

    def set(self):
        with self.compromiso:
            super().set()


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
        self.max_edad_juntas_s = None
        self._http = threading.Lock()  # una operación HTTP a la vez: el IRC5 admite pocas sesiones
        self.on_joints = None          # callback(list[6] grados) cada vez que se lee la postura
        self._ruta_q = None
        self.rmmp_concedido = False
        self._rmmp_t = 0.0
        seguro = re.sub(r"[^A-Za-z0-9_.-]", "_", f"{ip}_{puerto}")
        self.ruta_candado = ruta_candado or os.path.join(tempfile.gettempdir(), f"vevas_rws_escritor_{seguro}.lock")
        self._esc_lock = threading.Lock()
        self._esc_propietario = None
        self.ruta_modulo = os.path.join(os.path.dirname(__file__), "..", "05_rapid", MODULO + ".mod")
        self._esc_n = 0
        self._esc_fd = None

    # ---- HTTP
    def _pedir(self, metodo, ruta, datos=None, tipo=FORM, params=None, cancelar=None, antes_de_enviar=None):
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
            if antes_de_enviar is not None:
                antes_de_enviar()
            try:
                with getattr(cancelar, "compromiso", contextlib.nullcontext()):
                    if cancelar is not None and cancelar.is_set():
                        raise Falla("Orden cancelada antes de emitir la escritura RWS.")
                    respuesta = self._abridor.open(req, timeout=self.timeout)
                with respuesta as r:
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
            if self._esc_n and self._esc_propietario != threading.get_ident():
                raise Falla("Otra herramienta o hilo ya posee el candado de escritura RWS.")
            if fcntl is None:
                raise Falla("El sistema no ofrece el candado entre emisores RWS.")
            if self._esc_n == 0:
                fd = os.open(self.ruta_candado, os.O_CREAT | os.O_RDWR, 0o666)
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except OSError:
                    os.close(fd)
                    raise Falla("Otra herramienta (VEVAS o el nodo de MoveIt) está moviendo el robot por RWS. "
                                "Usá una sola a la vez.") from None
                self._esc_fd = fd
                self._esc_propietario = threading.get_ident()
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
                    self._esc_propietario = None

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
                t_lectura = time.monotonic()
                txt = self.get(ruta, **params)
                if self.max_edad_juntas_s is not None and time.monotonic() - t_lectura > self.max_edad_juntas_s:
                    raise Falla("La lectura de postura superó el tiempo de frescura permitido.")
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
        return e["pp_rutina"].lower() == RUTINA.lower() and e["pp_modulo"] == MODULO

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
        """Texto del módulo en el controlador. El IRC5 a veces contesta HTTP 500 «Error registering callback»
        (visto el 24/11 con RAPID corriendo, tras varias sesiones RWS cortadas con Ctrl+C): se reintenta
        cerrando la sesión HTTP para liberar los recursos del controlador."""
        ultimo = None
        for intento in range(4):
            try:
                return self._texto_modulo_una_vez()
            except Falla as e:
                ultimo = e
                if "Error registering callback" not in str(e) and "HTTP 500" not in str(e):
                    raise
                self.cerrar()                   # /logout: libera la sesión y sus suscripciones
                time.sleep(1.0 + intento)
        raise Falla(f"{ultimo} — el IRC5 no entregó el texto del módulo tras 4 intentos. Esperá un minuto "
                    "(o reiniciá el controlador con un «Reiniciar» normal si persiste) y reintentá.")

    def _texto_modulo_una_vez(self) -> str:
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

    def verificar_modulo(self, ruta=None):
        with open(ruta or self.ruta_modulo, encoding="utf-8") as f:
            esperado = huella_modulo(f.read())
        if huella_modulo(self._texto_modulo()) != esperado:
            raise Falla("El módulo del controlador difiere del archivo revisado; se bloquea el arranque y el envío.")

    def instalar_modulo(self, ruta_mod: str, forzar: bool = False, eco=print) -> str:
        """Deja VEVAS_RWS cargado y con el texto de `ruta_mod`. Solo en AUTO, motores apagados y RAPID
        detenido (como la referencia: no se pide RMMP ni se cambia el modo desde la PC).
        Devuelve 'ya_instalado', 'cargado' o 'actualizado'."""
        with open(ruta_mod, encoding="utf-8") as f:
            fuente = f.read()
        if len(fuente.encode("utf-8")) > 4800:
            raise Falla(f"{os.path.basename(ruta_mod)} tiene {len(fuente.encode('utf-8'))} bytes: RWS corta el texto del módulo en 5000 "
                        "(19/11: quedó cortado y RAPID marcó error). Acortalo (comentarios) por debajo de 4800.")
        if not re.search(r"^\s*MODULE\s+" + MODULO + r"\b", fuente, flags=re.M):
            raise Falla(f"{ruta_mod} no define MODULE {MODULO}.")
        e = self.leer_estado(con_juntas=False)
        if e["opmode"] != "AUTO":
            raise Falla(f"El controlador está en {e['opmode']}: la instalación por RWS exige AUTO, motores apagados "
                        "y RAPID detenido (no se cambia el modo desde la PC). Girá la llave a AUTO, o cargá "
                        f"{os.path.basename(ruta_mod)} a mano desde el FlexPendant.")
        if e["exec"] != "stopped":
            raise Falla("RAPID debe estar confirmado en stopped antes de instalar el módulo.")
        try:
            lista = self.get("/rw/rapid/modules", task=TAREA)
        except Falla:
            lista = ""
        otros = [m for m in ("LabJointRws", "EGM_ROS") if f'"{m}"' in lista]
        if otros:
            eco(f"  Aviso: en T_ROB1 también está {', '.join(otros)} (de otro grupo). VEVAS_RWS usa nombres propios y no choca con ellos.")
        mia = huella_modulo(fuente)

        def motores_off_para_escribir():
            # Solo si de verdad hay que escribir el módulo (24/11: apagar sin necesidad dejaba el robot
            # sin poder encenderse por RWS: el IRC5 contestó 403 a motores ON después de un reinicio).
            nonlocal e
            if e["ctrlstate"] == "motoron":
                self.post("/rw/panel/ctrlstate", {"action": "setctrlstate"}, {"ctrl-state": "motoroff"},
                          esperados=(200, 204), que="motores OFF")
                for _ in range(20):
                    if self.leer_estado(con_juntas=False)["ctrlstate"] == "motoroff":
                        break
                    time.sleep(0.25)
                e = self.leer_estado(con_juntas=False)
                eco("  Motores OFF para poder cargar el módulo.")
            if e["ctrlstate"] != "motoroff":
                raise Falla(f"Motores {e['ctrlstate']}: tienen que estar apagados para instalar el módulo "
                            "(soltá y rearmá el paro de emergencia si dice emergencystop).")

        if e["modulo_vevas"] and not forzar:
            try:
                actual = huella_modulo(self._texto_modulo())
            except Falla as ex:
                raise Falla(f"{MODULO} ya está cargado pero no pude leer su texto para compararlo ({ex}). "
                            "Reintentá con --recargar-modulo.") from ex
            if actual == mia:
                eco(f"  {MODULO} ya está cargado y coincide con {os.path.basename(ruta_mod)}.")
                self.ruta_modulo = ruta_mod
                return "ya_instalado"
            eco(f"  {MODULO} está cargado pero su texto difiere: se reemplaza (set-module-text).")
            motores_off_para_escribir()
            with self.mastership():
                self.post(RUTA_MODULO, {"task": TAREA, "action": "set-module-text"}, {"text": fuente},
                          esperados=(200, 204), que="set-module-text")
            leido = self._texto_modulo()
            if huella_modulo(leido) != mia:
                # No se reintenta (cada intento es otra edición en el controlador): se muestra la diferencia.
                eco("  El texto que guardó el controlador no es idéntico al archivo local. Diferencias:")
                for l in diferencias(fuente, leido)[:30]:
                    eco("    " + l)
                raise Falla("La lectura de vuelta del módulo difiere del archivo revisado.")
            self.ruta_modulo = ruta_mod
            return "actualizado"
        # no está cargado (o se fuerza la recarga): subir el archivo y cargarlo
        motores_off_para_escribir()
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
        self.verificar_modulo(ruta_mod)
        self.ruta_modulo = ruta_mod
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
        if e["ctrlstate"] not in ("motoron", "motoroff") or e["exec"] not in ("running", "stopped"):
            raise Falla(f"Estado del controlador incompatible para arrancar: {e['ctrlstate']}, RAPID {e['exec']}.")
        if not e["modulo_vevas"]:
            raise Falla(f"{MODULO} no está cargado: instalarlo primero (instalar_modulo).")
        self.verificar_modulo()
        if e["exec"] == "running":
            if self.pp_en_servidor(e):
                eco("  RAPID ya está corriendo en VevasServidor; no se vuelve a arrancar.")
                return e
            raise Falla(f"RAPID está ejecutando otro programa ({e['pp_modulo']}/{e['pp_rutina']}): detenerlo antes.")
        encendi = False
        try:
            if e["ctrlstate"] != "motoron":
                try:
                    self._motores("motoron")
                    encendi = True
                    eco("  Motores ON.")
                except Falla as ex:
                    if "403" not in str(ex):
                        raise
                    # Tras un reinicio en AUTO el IRC5 puede negar «motores ON» por RWS: lo hace el operador.
                    eco("  >>> El controlador no deja encender los motores desde la PC: pulsá el botón blanco "
                        "MOTORS ON del controlador (tenés 60 s).")
                    for _ in range(120):
                        if self.leer_estado(con_juntas=False)["ctrlstate"] == "motoron":
                            break
                        time.sleep(0.5)
                    else:
                        raise Falla("Motores sin encender: pulsá MOTORS ON en el controlador y reintentá.") from ex
                    eco("  Motores ON (operador).")
            with self.mastership():
                self.post(PCP, {"action": "set-pp-routine"}, {"module": MODULO, "routine": RUTINA, "userlevel": "false"},
                          que="set-pp-routine")
                e2 = self.leer_estado(con_juntas=False)
                if not self.pp_en_servidor(e2):
                    raise Falla(f"El PP no quedó en {MODULO}/{RUTINA} (está en {e2['pp_modulo']}/{e2['pp_rutina']}); NO se arranca.")
                eco(f"  PP confirmado en {MODULO}/{RUTINA}.")
                self.post("/rw/rapid/execution", {"action": "start"}, START_FORM, que="start RAPID")
            for _ in range(20):
                e2 = self.leer_estado(con_juntas=False)
                if e2["exec"] == "running" and self.pp_en_servidor(e2):
                    return e2
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
                e = self.leer_estado(con_juntas=False)
                if e["exec"] != "running" or not self.pp_en_servidor(e):
                    return False  # la rutina terminó o el puntero está en otro programa
                if s["vvRbActivo"] == 1 and s["vvPcFin"] == 0:
                    return True
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
        return {"ok": e["exec"] == "stopped" and e["ctrlstate"] == "motoroff", "exec": e["exec"], "ctrlstate": e["ctrlstate"], "errores": errores}

    def pp_main(self):
        """Puntero de programa de vuelta a main (resetpp)."""
        e = self.leer_estado(con_juntas=False)
        if e["exec"] != "stopped":
            raise Falla("RAPID debe estar confirmado en stopped antes de mover el puntero.")
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
    def _escribir(self, nombre: str, valor: str, *, cancelar=None, antes_de_enviar=None):
        if nombre not in ("vvPcTarget", "vvPcSeq", "vvPcFin", SIMBOLO_ZONA):
            raise Falla("Solo se escriben vvPcTarget, vvPcSeq, vvPcFin y vvPcZona.")
        self.mantener_rmmp()
        est, txt = self._pedir("POST", BASE_SIMBOLO + nombre, datos={"value": valor}, params={"action": "set"},
                               cancelar=cancelar, antes_de_enviar=antes_de_enviar)
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
        if not self.pp_en_servidor(e):
            p.append("el puntero de programa no está confirmado en VEVAS_RWS/VevasServidor")
        if s["vvRbActivo"] != 1:
            p.append("VevasServidor no está activo (vvRbActivo=0): ¿falta «Iniciar» en el FlexPendant?")
        if s["vvPcFin"] != 0:
            p.append("vvPcFin=1: el servidor terminó; arrancarlo de nuevo")
        if s["vvPcSeq"] != s["vvRbSeq"]:
            p.append("hay una orden en curso (vvPcSeq≠vvRbSeq)")
        return p

    def enviar_objetivo(self, q_deg, timeout: float = 120.0, cancelar: threading.Event | None = None,
                        al_aceptar=None, eco=None, paso_max_deg: float = PASO_PC_MAX_DEG,
                        intervalo: float = 0.25, origen_deg=None, al_comprometer=None) -> tuple[bool, int | None, str]:
        """Un objetivo articular absoluto (grados). Devuelve (ok, codigo, mensaje).

        codigo = vvRbCodigo del robot (2 llegada, negativos = rechazo); -1/-2/-3 también cuando la PC lo
        rechaza antes de escribir; None si la orden no llegó a decidirse (no listo, sin respuesta...).
        `al_aceptar(codigo)` se llama una vez cuando el robot ya la tomó (1 moviendo) o contestó.
        La cancelación previa a vvPcSeq impide iniciar ese tramo. Una orden ya emitida espera su acuse."""
        q_deg = [float(v) for v in q_deg]
        try:
            with self.escritor():
                return self._enviar(q_deg, timeout, al_aceptar, eco, paso_max_deg, intervalo, cancelar, origen_deg, al_comprometer)
        except Falla as ex:
            return False, None, str(ex)

    def _enviar(self, q_deg, timeout, al_aceptar, eco, paso_max_deg, intervalo, cancelar, origen_deg, al_comprometer):
        def comprobar_cancelacion():
            if cancelar is not None and cancelar.is_set():
                raise Falla("Orden cancelada antes de incrementar vvPcSeq.")

        comprobar_cancelacion()
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
        self.verificar_modulo()
        comprobar_cancelacion()
        if self.tiempo_zona_local() is not None and int(self._num(SIMBOLO_ZONA)) != 0:
            self._escribir(SIMBOLO_ZONA, "0", cancelar=cancelar)   # orden suelta: siempre fine
        self._escribir("vvPcTarget", literal(q_deg), cancelar=cancelar)
        vuelta = self._leer_objetivo()
        if vuelta is None or max(abs(a - b) for a, b in zip(vuelta, q_deg)) > LECTURA_TOL_DEG:
            return False, None, "La lectura de vuelta de vvPcTarget no coincide; no se ordena el movimiento."
        e_actual = self.leer_estado(con_juntas=False)
        s_actual = self.estado_servidor()
        problemas = self.problemas_para_enviar(e_actual, s_actual)
        if problemas or s_actual["vvPcSeq"] != s["vvPcSeq"]:
            return False, None, "Cambió el estado antes de la orden: " + ("; ".join(problemas) or "secuencia distinta")
        seq = s["vvPcSeq"] + 1
        # El origen del tramo puede cambiar durante RMMP o durante la lectura de vuelta.
        t_origen = time.monotonic()
        actual = self.joints_deg()
        if origen_deg is not None and max(abs(a - b) for a, b in zip(actual, origen_deg)) > PASO_TOL_DEG:
            raise Falla("La postura medida cambió desde el origen revisado. Recalculá y revisá el plan.")
        if rechazo_local(q_deg, actual, paso_max_deg):
            raise Falla("El tramo ya no cumple los límites desde la postura medida.")
        def confirmar_orden():
            if al_comprometer is not None:
                al_comprometer()
            if self.max_edad_juntas_s is not None and time.monotonic() - t_origen > self.max_edad_juntas_s:
                raise Falla("El estado del robot caducó antes de emitir vvPcSeq.")
        self._escribir("vvPcSeq", str(seq), cancelar=cancelar, antes_de_enviar=confirmar_orden)
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
                # Leer después del acuse: la lectura anterior puede preceder a la llegada.
                falta = max(abs(a - b) for a, b in zip(self.joints_deg(), q_deg))
                if falta > PASO_TOL_DEG:
                    return False, codigo, f"Acuse sin llegada medida: faltan {falta:.3f}°."
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

    # ---- movimiento continuo (streaming con zonas)
    def tiempo_zona_local(self) -> float | None:
        """tZona del módulo revisado (05_rapid/VEVAS_RWS.mod); None si esa versión no soporta streaming.
        verificar_modulo() garantiza que el controlador corre ese mismo texto."""
        try:
            with open(self.ruta_modulo, encoding="utf-8") as f:
                return tiempo_zona(f.read())
        except OSError:
            return None

    def _exec(self) -> str:
        return str(_buscar_valor(self.get("/rw/rapid/execution"), "ctrlexecstate") or "?").lower()

    def _esperar_acuse(self, seq: int, timeout: float, intervalo: float) -> int:
        """Espera vvRbSeq = seq y devuelve vvRbCodigo. Lee la postura cada 0.1 s (frescura para el watchdog de
        Consola) y el estado de RAPID cada 0.5 s (un Stop/paro corta la espera)."""
        t0 = time.monotonic()
        t_q = t_exec = 0.0
        detenido = 0
        while True:
            ahora = time.monotonic()
            if int(self._num("vvRbSeq")) == seq:
                return int(self._num("vvRbCodigo"))
            if ahora - t0 > timeout:
                raise Falla(f"Sin acuse del robot para la orden {seq} en {timeout:.0f} s. No enviar otra: revisar el FlexPendant.")
            if ahora - t_q >= 0.1:
                t_q = ahora
                self.mantener_rmmp()
                self.joints_deg()
            if ahora - t_exec >= 0.5:
                t_exec = ahora
                if self._exec() != "running":
                    detenido += 1
                    if detenido >= 2:
                        raise Falla(f"RAPID se detuvo con la orden {seq} pendiente (¿PARAR, paro, habilitador, error?). "
                                    "No se envían más puntos; resolverlo en el FlexPendant.")
                else:
                    detenido = 0
            time.sleep(intervalo)

    def _esperar_quieto(self, objetivo_deg, timeout: float) -> list[float] | None:
        """Tras cortar un recorrido: espera a que el robot termine lo ya encolado (llega al último punto tomado o
        se queda quieto) o a que RAPID esté detenido. Devuelve la última postura leída."""
        t0 = time.monotonic()
        q_prev, t_quieto, q = None, None, None
        while time.monotonic() - t0 < timeout:
            try:
                q = self.joints_deg()
                if objetivo_deg is not None and max(abs(a - b) for a, b in zip(q, objetivo_deg)) <= PASO_TOL_DEG:
                    return q
                if q_prev is not None and max(abs(a - b) for a, b in zip(q, q_prev)) < 0.005:
                    t_quieto = t_quieto or time.monotonic()
                    if time.monotonic() - t_quieto > 0.6:
                        return q
                else:
                    t_quieto = None
                q_prev = q
                if self._exec() != "running":
                    return q
            except Falla:
                pass
            time.sleep(0.05)
        return q

    def enviar_recorrido(self, puntos_deg, timeout: float = 120.0, cancelar: threading.Event | None = None,
                         al_aceptar=None, al_progreso=None, origen_deg=None, al_comprometer=None,
                         intervalo: float = 0.02, eco=None, fines=()) -> tuple[bool, int | None, str]:
        """Recorrido continuo: los puntos intermedios van con zona (vvPcZona = 1, el robot no para) y el último
        con fine. El punto k+1 se escribe apenas el robot contestó que tomó el k (vvRbCodigo = 3).
        `fines`: índices de puntos intermedios que también van con fine (paradas del plan: esquinas); el robot
        se detiene ahí (vvRbCodigo 1 → 2) y el recorrido sigue desde la postura medida.

        Antes de mover nada se validan TODOS los puntos (límites y paso ≤ 40° contra el punto anterior).
        Si el robot rechaza un punto a mitad, no se escribe ninguno más: el robot frena en el último tomado.
        Cancelar (PARAR) corta el envío entre puntos; el robot termina lo ya encolado (≤ 2 tramos) y para.
        Devuelve (ok, codigo, mensaje); el detalle queda en self.ultimo_recorrido."""
        puntos = [[float(v) for v in p] for p in puntos_deg]
        self.ultimo_recorrido = {"puntos": len(puntos), "tomados": 0, "codigos": [], "espera_acuse_max_s": 0.0}
        try:
            with self.escritor():
                return self._recorrido(puntos, timeout, cancelar, al_aceptar, al_progreso, origen_deg,
                                       al_comprometer, intervalo, eco, fines)
        except Falla as ex:
            return False, None, str(ex)

    def _recorrido(self, puntos, timeout, cancelar, al_aceptar, al_progreso, origen_deg, al_comprometer, intervalo, eco,
                   fines=()):
        info = self.ultimo_recorrido
        n = len(puntos)
        if n == 0:
            return False, None, "El recorrido está vacío."
        fine = {int(k) for k in (fines or ()) if 0 <= int(k) < n} | {n - 1}   # el último siempre con fine
        info["fines"] = sorted(fine)
        if cancelar is not None and cancelar.is_set():
            raise Falla("Recorrido cancelado antes de empezar.")
        tz = self.tiempo_zona_local()
        if tz is None:
            return False, None, "El módulo RAPID no soporta movimiento continuo (falta vvPcZona/tZona)."
        with open(self.ruta_modulo, encoding="utf-8") as f:
            vlim, kp = limites_c02(f.read())
        e = self.leer_estado(con_juntas=False)
        s = self.estado_servidor() if e["modulo_vevas"] else None
        problemas = self.problemas_para_enviar(e, s)
        if problemas:
            return False, None, "No listo: " + "; ".join(problemas)
        actual = self.joints_deg()
        if origen_deg is not None and max(abs(a - b) for a, b in zip(actual, origen_deg)) > PASO_TOL_DEG:
            raise Falla("La postura medida cambió desde el origen revisado. Recalculá y revisá el plan.")
        puntos = [[round(v, DECIMALES) for v in p] for p in puntos]
        previo = actual
        tiempos = []
        for i, p in enumerate(puntos, 1):
            rech = rechazo_local(p, previo)
            if rech is not None:
                return False, rech[0], f"Rechazado por la PC (punto {i}/{n}, no se movió nada): {rech[1]}"
            tiempos.append(tiempo_movimiento(p, previo, vlim, kp, T_MIN_S if i - 1 in fine else tz))
            previo = p
        if e["opmode"] == "MANR" and not self.rmmp_concedido:
            self.solicitar_rmmp(eco=eco or print)
        self.verificar_modulo()
        zona_ctrl = int(self._num(SIMBOLO_ZONA))
        seq = s["vvPcSeq"]
        tomados, ultimo_tomado = 0, None
        try:
            for i, p in enumerate(puntos):
                con_fine = i in fine
                if cancelar is not None and cancelar.is_set():
                    break
                zona = 0 if con_fine else 1
                if zona_ctrl != zona:
                    self._escribir(SIMBOLO_ZONA, str(zona), cancelar=cancelar)
                    zona_ctrl = zona
                self._escribir("vvPcTarget", literal(p), cancelar=cancelar)
                vuelta = self._leer_objetivo()
                if vuelta is None or max(abs(a - b) for a, b in zip(vuelta, p)) > LECTURA_TOL_DEG:
                    raise Falla(f"La lectura de vuelta de vvPcTarget (punto {i + 1}/{n}) no coincide; no se ordena.")
                confirmar = None
                if i == 0:
                    e1, s1 = self.leer_estado(con_juntas=False), self.estado_servidor()
                    problemas = self.problemas_para_enviar(e1, s1)
                    if problemas or s1["vvPcSeq"] != seq:
                        return False, None, "Cambió el estado antes de la orden: " + ("; ".join(problemas) or "secuencia distinta")
                    t_origen = time.monotonic()
                    q1 = self.joints_deg()
                    if origen_deg is not None and max(abs(a - b) for a, b in zip(q1, origen_deg)) > PASO_TOL_DEG:
                        raise Falla("La postura medida cambió desde el origen revisado. Recalculá y revisá el plan.")
                    if rechazo_local(p, q1):
                        raise Falla("El primer punto ya no cumple los límites desde la postura medida.")

                    def confirmar():
                        if al_comprometer is not None:
                            al_comprometer()
                        if self.max_edad_juntas_s is not None and time.monotonic() - t_origen > self.max_edad_juntas_s:
                            raise Falla("El estado del robot caducó antes de emitir vvPcSeq.")
                self._escribir("vvPcSeq", str(seq + 1), cancelar=cancelar, antes_de_enviar=confirmar)
                seq += 1
                # el acuse de un punto con zona llega cuando el RAPID pudo encolarlo (a lo sumo cuando termina el
                # tramo anterior); el de un punto con fine, al llegar (detrás de hasta dos tramos encolados).
                espera = sum(tiempos[max(0, i - 2):i + 1]) + 10.0
                t_a = time.monotonic()
                codigo = self._esperar_acuse(seq, espera, intervalo)
                info["espera_acuse_max_s"] = max(info["espera_acuse_max_s"], round(time.monotonic() - t_a, 3))
                info["codigos"].append(codigo)
                if i == 0 and al_aceptar is not None:
                    al_aceptar(codigo)
                if codigo < 0:
                    if ultimo_tomado is not None:
                        self._esperar_quieto(ultimo_tomado, sum(tiempos[max(0, i - 2):i]) + 5.0)
                    donde = f"el punto {tomados}" if tomados else "el origen (no se movió)"
                    return False, codigo, (f"El robot rechazó el punto {i + 1}/{n}: código {codigo} "
                                           f"({CODIGOS.get(codigo, '?')}). No se enviaron más; frenó en {donde}.")
                if codigo != (2 if con_fine else 3):
                    self._esperar_quieto(ultimo_tomado, sum(tiempos[max(0, i - 2):i + 1]) + 5.0)
                    return False, codigo, f"Respuesta inesperada al punto {i + 1}/{n}: código {codigo} ({CODIGOS.get(codigo, '?')})."
                tomados, ultimo_tomado = i + 1, p
                info["tomados"] = tomados
                if al_progreso is not None:
                    al_progreso(tomados, n)
            else:
                falta = max(abs(a - b) for a, b in zip(self.joints_deg(), puntos[-1]))
                if falta > PASO_TOL_DEG:
                    return False, 2, f"Acuse sin llegada medida: faltan {falta:.3f}°."
                return True, 2, (f"recorrido continuo de {n} puntos: llegada confirmada (falta {falta:.3f}°, "
                                 f"espera máx. de acuse {info['espera_acuse_max_s']:.2f} s)")
        except Falla:
            if not (cancelar is not None and cancelar.is_set()):
                # falla a mitad (HTTP, RAPID detenido...): no se mandan más puntos; antes de soltar el candado se
                # espera a que el robot termine lo encolado, así nadie ordena desde una postura en movimiento
                with contextlib.suppress(Falla):
                    self._esperar_quieto(ultimo_tomado, sum(tiempos[max(0, tomados - 2):tomados]) + 5.0)
                raise
        finally:
            if zona_ctrl == 1:   # que una orden futura no herede la zona (el RAPID la ignora sin vvPcSeq nuevo)
                with contextlib.suppress(Falla):
                    self._escribir(SIMBOLO_ZONA, "0")
        # cancelado (PARAR) entre puntos: el robot termina lo encolado y para en el último punto tomado
        self._esperar_quieto(ultimo_tomado, sum(tiempos[max(0, tomados - 2):tomados]) + 5.0)
        return False, None, (f"Recorrido cancelado tras {tomados}/{n} puntos: no se enviaron más; el robot "
                             "termina el tramo encolado y para en el último punto tomado.")
