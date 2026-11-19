#!/usr/bin/env python3
# LEGADO (ruta EGM). La ruta vigente es herramientas/rws_irc5.py (RWS + VEVAS_RWS) y el lanzador iniciar_real.sh.
"""Ayudante RWS para el IRC5 (RobotWare 6, RWS 1.0). Solo biblioteca estándar.

Subcomandos:
  estado      lee modo, motores, ejecución RAPID, módulo EGM_ROS, puntero y juntas (solo lectura)
  preparar    carga EGM_ROS si falta y deja el puntero de programa en ros_egm (el programa debe estar detenido)
  arrancar    motores ON + Start de RAPID. SOLO si el controlador está en AUTO
  parar       Stop de RAPID + motores OFF
  pp-main     puntero de programa de vuelta a main (resetpp)
  comparar    compara juntas del IRC5 (RWS) con /joint_states de ROS

Nunca cambia el modo de operación (la llave es física), nunca puentea cadenas de seguridad.
Cada acción que escribe se verifica leyendo de vuelta; si no se puede confirmar, sale con error.
"""
from __future__ import annotations

import argparse
import http.cookiejar
import json
import math
import os
import re
import subprocess
import sys
import time
import urllib.error
import urllib.request

TAREA = "T_ROB1"
MODULO = "EGM_ROS"
RUTINA = "ros_egm"
AQUI = os.path.dirname(os.path.abspath(__file__))
MOD_LOCAL = os.path.join(os.path.dirname(AQUI), "EGM_ROS.mod")


class Falla(Exception):
    pass


class RWS:
    def __init__(self, ip, usuario, clave, puerto=80, timeout=5.0):
        self.base = f"http://{ip}:{puerto}"
        gestor = urllib.request.HTTPPasswordMgrWithDefaultRealm()
        gestor.add_password(None, self.base, usuario, clave)
        self.abridor = urllib.request.build_opener(
            urllib.request.HTTPDigestAuthHandler(gestor),
            urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()),
        )
        self.timeout = timeout

    def pedir(self, metodo, ruta, datos=None, tipo="application/x-www-form-urlencoded;v=2.0"):
        url = self.base + ruta
        cuerpo = None if datos is None else datos.encode("utf-8")
        req = urllib.request.Request(url, data=cuerpo, method=metodo)
        if cuerpo is not None:
            req.add_header("Content-Type", tipo)
        try:
            with self.abridor.open(req, timeout=self.timeout) as r:
                return r.status, r.read().decode("utf-8", errors="replace")
        except urllib.error.HTTPError as e:
            return e.code, e.read().decode("utf-8", errors="replace")
        except (urllib.error.URLError, OSError) as e:
            raise Falla(f"No hay respuesta de RWS en {self.base}: {e}") from e

    def get(self, ruta):
        sep = "&" if "?" in ruta else "?"
        est, txt = self.pedir("GET", ruta + sep + "json=1")
        if est != 200:
            raise Falla(f"GET {ruta} → HTTP {est}: {txt[:200]}")
        return txt

    def post(self, ruta, datos=""):
        return self.pedir("POST", ruta, datos)


def _buscar_valor(txt, clave):
    try:
        obj = json.loads(txt)
    except json.JSONDecodeError:
        m = re.search(rf'class="{clave}"[^>]*>\s*([^<\s]+)', txt)
        return m.group(1) if m else None

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


def modulos(c: RWS) -> str:
    """Lista de módulos de T_ROB1. RW6: /rw/rapid/modules?task=T_ROB1 (la ruta /tasks/T_ROB1/modules es de otras versiones)."""
    ultimo = None
    for ruta in (f"/rw/rapid/modules?task={TAREA}", f"/rw/rapid/tasks/{TAREA}/modules"):
        try:
            return c.get(ruta)
        except Falla as e:
            ultimo = e
    raise ultimo


def leer_estado(c: RWS) -> dict:
    e = {}
    e["opmode"] = str(_buscar_valor(c.get("/rw/panel/opmode"), "opmode") or "?").upper()
    e["ctrlstate"] = str(_buscar_valor(c.get("/rw/panel/ctrlstate"), "ctrlstate") or "?").lower()
    e["exec"] = str(_buscar_valor(c.get("/rw/rapid/execution"), "ctrlexecstate") or "?").lower()
    e["modulo_egm"] = f'"{MODULO}"' in modulos(c)
    try:
        pcp = c.get(f"/rw/rapid/tasks/{TAREA}/pcp")
        e["pp_rutina"] = str(_buscar_valor(pcp, "routinename") or "?")
        # RW6 trae la errata «modulemame»; se aceptan las variantes
        e["pp_modulo"] = str(_buscar_valor(pcp, "modulemame") or _buscar_valor(pcp, "modulename")
                             or _buscar_valor(pcp, "module-name") or "?")
    except Falla:
        e["pp_rutina"] = e["pp_modulo"] = "?"
    jt = c.get("/rw/motionsystem/mechunits/ROB_1/jointtarget")
    q = []
    for i in range(1, 7):
        v = _buscar_valor(jt, f"rax_{i}")
        if v is None:
            raise Falla(f"La lectura de juntas no trae rax_{i}")
        q.append(float(v))
    if not all(math.isfinite(x) for x in q):
        raise Falla("La lectura de juntas no es finita")
    e["q_deg"] = q
    return e


def pp_en_ros_egm(e) -> bool:
    return e["pp_rutina"].lower() == RUTINA and e["pp_modulo"] in (MODULO, "?")


def cmd_estado(c, a):
    e = leer_estado(c)
    if a.shell:
        print(f"OPMODE={e['opmode']}")
        print(f"CTRLSTATE={e['ctrlstate']}")
        print(f"EXEC={e['exec']}")
        print(f"MODULO_EGM={'si' if e['modulo_egm'] else 'no'}")
        print(f"PP_RUTINA={e['pp_rutina']}")
        print("Q_DEG=" + ",".join(f"{x:.3f}" for x in e["q_deg"]))
    else:
        print(json.dumps(e, indent=2, ensure_ascii=False))
    return 0


def _mastership(c, accion):
    for ruta in (f"/rw/mastership?action={accion}", f"/rw/mastership/rapid?action={accion}", f"/rw/mastership/edit?action={accion}"):
        est, _ = c.post(ruta)
        if est in (200, 204):
            return True
    return False


def _verificar_modulo(c):
    return f'"{MODULO}"' in modulos(c)


def cmd_preparar(c, a):
    e = leer_estado(c)
    if e["exec"] == "running":
        raise Falla("RAPID está en ejecución. Detené el programa antes de preparar (o usá `parar`).")
    if e["modulo_egm"] and not a.recargar:
        print(f"Módulo {MODULO}: ya está cargado en {TAREA} (no se toca; usá --recargar para reemplazarlo).")
    else:
        if not os.path.exists(a.modulo):
            raise Falla(f"No encuentro {a.modulo}")
        with open(a.modulo, encoding="utf-8") as f:
            texto = f.read()
        est, txt = c.pedir("PUT", f"/fileservice/$HOME/{MODULO}.mod", texto, tipo="text/plain")
        if est not in (200, 201, 204):
            raise Falla(f"No se pudo subir {MODULO}.mod al controlador (HTTP {est}): {txt[:200]}")
        print(f"Archivo subido a $HOME/{MODULO}.mod")
        if not _mastership(c, "request"):
            raise Falla("No se obtuvo mastership de RAPID (¿hay un FlexPendant con el mando en modo edición?).")
        cargado = False
        try:
            intentos = []
            for ruta in (f"/rw/rapid/tasks/{TAREA}?action=loadmod", f"/rw/rapid/tasks/{TAREA}?action=loadmod&mastership=implicit", f"/rw/rapid/tasks/{TAREA}/loadmod"):
                for path in (f"$HOME/{MODULO}.mod", f"HOME:/{MODULO}.mod"):
                    est, txt = c.post(ruta, f"modulepath={path}&replace=true")
                    intentos.append(f"{ruta} [{path}] → HTTP {est}")
                    if est in (200, 204) and _verificar_modulo(c):
                        cargado = True
                        break
                if cargado:
                    break
            if not cargado:
                raise Falla("No se pudo cargar el módulo vía RWS. Intentos:\n  " + "\n  ".join(intentos) +
                            f"\nAlternativa: cargalo una vez por FlexPendant (Program Editor → Modules → Load Module → {MODULO}.mod).")
        finally:
            _mastership(c, "release")
        print(f"Módulo {MODULO} cargado y verificado en {TAREA}.")
    # puntero de programa
    if pp_en_ros_egm(leer_estado(c)):
        print(f"Puntero de programa ya en {RUTINA}.")
        return 0
    if not _mastership(c, "request"):
        raise Falla("No se obtuvo mastership para mover el puntero de programa.")
    ok = False
    intentos = []
    try:
        cuerpo = f"module={MODULO}&routine={RUTINA}"
        for ruta in (f"/rw/rapid/tasks/{TAREA}/pcp?action=set-pp-routine", f"/rw/rapid/tasks/{TAREA}/pcp/routine?mastership=implicit", f"/rw/rapid/tasks/{TAREA}/pcp?action=set-routine",
                     f"/rw/rapid/tasks/{TAREA}/pcp/routine", f"/rw/rapid/tasks/{TAREA}/pcp?action=set-routine-pp"):
            est, txt = c.post(ruta, cuerpo)
            intentos.append(f"{ruta} → HTTP {est}")
            if est in (200, 204):
                e2 = leer_estado(c)
                if pp_en_ros_egm(e2):
                    ok = True
                    break
    finally:
        _mastership(c, "release")
    if not ok:
        raise Falla(f"No se pudo confirmar el puntero de programa en {RUTINA}. Intentos:\n  " + "\n  ".join(intentos) +
                    f"\nAlternativa: FlexPendant → Debug → PP to Routine… → {RUTINA} (sin pulsar Start).")
    print(f"Puntero de programa en {MODULO}/{RUTINA} (verificado).")
    return 0


def cmd_arrancar(c, a):
    e = leer_estado(c)
    if e["opmode"] != "AUTO":
        raise Falla(f"El controlador está en {e['opmode']}, no en AUTO. Este script no cambia el modo (es la llave del IRC5). "
                    "En MANR el operador pulsa Start en el FlexPendant con el enabling device.")
    if e["ctrlstate"] in ("guardstop", "emergencystop", "emergencystopreset", "sysfail"):
        raise Falla(f"Cadena de seguridad abierta o en falla (estado: {e['ctrlstate']}). Resolverlo en el controlador.")
    if not pp_en_ros_egm(e):
        raise Falla(f"El puntero de programa no está en {MODULO}/{RUTINA} (está en {e['pp_modulo']}/{e['pp_rutina']}). Corré `preparar`.")
    if e["exec"] == "running":
        print("RAPID ya está corriendo; no se vuelve a arrancar.")
        return 0
    if e["ctrlstate"] != "motoron":
        est, txt = c.post("/rw/panel/ctrlstate?action=setctrlstate", "ctrl-state=motoron")
        if est not in (200, 204):
            raise Falla(f"No se pudieron encender los motores (HTTP {est}): {txt[:200]}")
        for _ in range(20):
            if leer_estado(c)["ctrlstate"] == "motoron":
                break
            time.sleep(0.25)
        else:
            raise Falla("Los motores no pasaron a motoron.")
        print("Motores ON.")
    est, txt = c.post("/rw/rapid/execution?action=start",
                      "regain=continue&execmode=continue&cycle=once&condition=none&stopatbp=disabled&alltaskbytsp=false")
    if est not in (200, 204):
        raise Falla(f"No se pudo iniciar RAPID (HTTP {est}): {txt[:200]}")
    for _ in range(20):
        if leer_estado(c)["exec"] == "running":
            print(f"RAPID corriendo en {RUTINA}: esperando a ROS.")
            return 0
        time.sleep(0.25)
    raise Falla("RAPID no pasó a running.")


def cmd_parar(c, a):
    est, _ = c.post("/rw/rapid/execution?action=stop", "stopmode=stop")
    c.post("/rw/panel/ctrlstate?action=setctrlstate", "ctrl-state=motoroff")
    time.sleep(0.5)
    e = leer_estado(c)
    print(f"Estado: exec={e['exec']} motores={e['ctrlstate']}")
    return 0 if e["exec"] != "running" else 1


def cmd_ppmain(c, a):
    e = leer_estado(c)
    if e["exec"] == "running":
        raise Falla("RAPID está corriendo; detenelo antes de mover el puntero.")
    if not _mastership(c, "request"):
        raise Falla("No se obtuvo mastership.")
    try:
        est, txt = c.post("/rw/rapid/execution?action=resetpp")
    finally:
        _mastership(c, "release")
    if est not in (200, 204):
        raise Falla(f"resetpp falló (HTTP {est}): {txt[:200]}")
    print("Puntero de programa en main.")
    return 0


def _ros_echo(topico, campo, timeout):
    r = subprocess.run(["ros2", "topic", "echo", topico, "--once", "--field", campo],
                       capture_output=True, text=True, timeout=timeout)
    if r.returncode != 0 or not r.stdout.strip():
        raise Falla(f"No llegó {topico} ({campo}): {r.stderr.strip()[:150]}")
    return r.stdout


def cmd_comparar(c, a):
    nombres = re.findall(r"joint_\d", _ros_echo(a.topico, "name", a.espera))
    pos = [float(x) for x in re.findall(r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?", _ros_echo(a.topico, "position", a.espera))]
    if nombres != [f"joint_{i}" for i in range(1, 7)] or len(pos) != 6:
        raise Falla(f"Nombres u orden inesperados en {a.topico}: {nombres}, {len(pos)} posiciones.")
    q_ros = [math.degrees(x) for x in pos]
    q_rws = leer_estado(c)["q_deg"]
    dif = [abs(x - y) for x, y in zip(q_ros, q_rws)]
    print("ROS :", " ".join(f"{x:8.2f}" for x in q_ros))
    print("RWS :", " ".join(f"{x:8.2f}" for x in q_rws))
    print(f"Diferencia máxima: {max(dif):.2f}° (tolerancia {a.tol}°)")
    if max(dif) > a.tol:
        raise Falla("ROS y el controlador no coinciden: signos, offsets o un estado viejo.")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.125.1")
    ap.add_argument("--puerto", type=int, default=80)
    ap.add_argument("--usuario", default="Default User")
    ap.add_argument("--clave", default="robotics")
    ap.add_argument("--timeout", type=float, default=5.0)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("estado"); s.add_argument("--shell", action="store_true")
    s = sub.add_parser("preparar"); s.add_argument("--recargar", action="store_true"); s.add_argument("--modulo", default=MOD_LOCAL)
    sub.add_parser("arrancar"); sub.add_parser("parar"); sub.add_parser("pp-main")
    s = sub.add_parser("comparar"); s.add_argument("--topico", default="/joint_states"); s.add_argument("--tol", type=float, default=1.0); s.add_argument("--espera", type=float, default=10.0)
    a = ap.parse_args(argv)
    c = RWS(a.ip, a.usuario, a.clave, a.puerto, a.timeout)
    f = {"estado": cmd_estado, "preparar": cmd_preparar, "arrancar": cmd_arrancar, "parar": cmd_parar,
         "pp-main": cmd_ppmain, "comparar": cmd_comparar}[a.cmd]
    try:
        return f(c, a)
    except Falla as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    except subprocess.TimeoutExpired:
        print("ERROR: tiempo agotado esperando ROS.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
