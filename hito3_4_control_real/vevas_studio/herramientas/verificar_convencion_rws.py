#!/usr/bin/env python3
"""Compara la FK del grupo con la pose que reporta el controlador IRC5 (SOLO LECTURA).

Es la prueba de Gate 1 / nivel S1: el controlador dice "estoy en estas juntas" y
"mi tool0 está en esta pose"; nosotros calculamos la FK con esas mismas juntas y
comparamos. Si coincide, nuestra tabla D-H, el orden, los signos y los offsets de
las juntas son los mismos que los del robot real.

Solo hace pedidos GET a Robot Web Services (RWS). No escribe nada, no pide
mastership, no enciende motores y no mueve el robot.

Uso (PC con IP 192.168.125.10, cable en el puerto SERVICE del IRC5):
    python3 herramientas/verificar_convencion_rws.py                 # una postura
    python3 herramientas/verificar_convencion_rws.py --posturas 3    # pide mover con el FlexPendant entre medidas
    python3 herramientas/verificar_convencion_rws.py --prueba        # sin robot, con datos inventados (para ensayar)

Credenciales: usuario y clave por defecto de RWS ("Default User" / "robotics"),
los mismos que usa el driver abb_ros2. Si el laboratorio los cambió, usar --usuario y --clave.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import http.cookiejar
import itertools
import json
import math
import os
import re
import sys
import time
import urllib.request

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
sys.path.insert(0, RAIZ)

from vevas.robot_irb120 import Robot, cuaternion_a_matriz, error_orientacion, matriz_a_cuaternion  # noqa: E402
from vevas.seguridad import cargar_config  # noqa: E402

RUTA_JUNTAS = "/rw/motionsystem/mechunits/ROB_1/jointtarget"
RUTA_POSE = "/rw/motionsystem/mechunits/ROB_1/robtarget?tool=tool0&wobj=wobj0&coordinate=Base"


class ClienteRWS:
    """Cliente mínimo de RWS 1.0 (RobotWare 6) con autenticación digest. Solo GET."""

    def __init__(self, ip: str, usuario: str, clave: str, puerto: int = 80, timeout: float = 4.0):
        self.base = f"http://{ip}:{puerto}"
        gestor = urllib.request.HTTPPasswordMgrWithDefaultRealm()
        gestor.add_password(None, self.base, usuario, clave)
        self.abridor = urllib.request.build_opener(
            urllib.request.HTTPDigestAuthHandler(gestor), urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar())
        )
        self.timeout = timeout

    def get(self, ruta: str) -> str:
        sep = "&" if "?" in ruta else "?"
        with self.abridor.open(self.base + ruta + sep + "json=1", timeout=self.timeout) as r:
            return r.read().decode("utf-8", errors="replace")


def _buscar(o, claves):
    if isinstance(o, dict):
        if all(k in o for k in claves):
            return o
        for v in o.values():
            r = _buscar(v, claves)
            if r is not None:
                return r
    elif isinstance(o, list):
        for v in o:
            r = _buscar(v, claves)
            if r is not None:
                return r
    return None


def _valores(texto: str, claves: list[str]) -> dict:
    """Lee los campos de la respuesta, sea JSON (?json=1) o XHTML (por si acaso)."""
    try:
        d = _buscar(json.loads(texto), claves[:1])
        if d is not None:
            return {k: float(d[k]) for k in claves}
    except (json.JSONDecodeError, KeyError, ValueError):
        pass
    out = {}
    for k in claves:
        m = re.search(rf'class="{k}"[^>]*>\s*([-+0-9.eE]+)\s*<', texto)
        if not m:
            raise ValueError(f"No encontré '{k}' en la respuesta de RWS: {texto[:200]}")
        out[k] = float(m.group(1))
    return out


def leer_juntas(cli) -> np.ndarray:
    v = _valores(cli.get(RUTA_JUNTAS), [f"rax_{i}" for i in range(1, 7)])
    return np.array([v[f"rax_{i}"] for i in range(1, 7)])


def leer_pose(cli):
    v = _valores(cli.get(RUTA_POSE), ["x", "y", "z", "q1", "q2", "q3", "q4"])
    return np.array([v["x"], v["y"], v["z"]]), np.array([v["q1"], v["q2"], v["q3"], v["q4"]])


class ClientePrueba:
    """Respuestas inventadas con la forma de RWS, para ensayar sin robot."""

    def __init__(self):
        self.robot = Robot()
        self.q = np.array([12.34, -18.9, 27.65, 41.2, 36.7, -77.1])

    def get(self, ruta: str) -> str:
        if "jointtarget" in ruta:
            estado = {"_type": "ms-jointtarget", "_title": "ROB_1", **{f"rax_{i + 1}": f"{v:.2f}" for i, v in enumerate(self.q)}}
        else:
            T = self.robot.fk_tool0(np.radians(self.q))
            q = matriz_a_cuaternion(T[:3, :3])
            estado = {"_type": "ms-robtargets", "_title": "ROB_1", "x": f"{T[0, 3] * 1000:.2f}", "y": f"{T[1, 3] * 1000:.2f}", "z": f"{T[2, 3] * 1000:.2f}", "q1": f"{q[0]:.6f}", "q2": f"{q[1]:.6f}", "q3": f"{q[2]:.6f}", "q4": f"{q[3]:.6f}"}
        return json.dumps({"_embedded": {"_state": [estado]}})


def comparar(robot, q_deg, p_mm, cuat, signos, offsets_deg):
    q = np.radians(np.asarray(signos) * q_deg + np.asarray(offsets_deg))
    T = robot.fk_tool0(q)
    e_p = float(np.linalg.norm(T[:3, 3] * 1000 - p_mm))
    e_R = math.degrees(error_orientacion(T[:3, :3], cuaternion_a_matriz(*cuat)))
    return e_p, e_R, T


def main():
    ap = argparse.ArgumentParser(description="FK propia vs controlador IRC5 (solo lectura por RWS)")
    ap.add_argument("--ip", default="192.168.125.1")
    ap.add_argument("--puerto", type=int, default=80)
    ap.add_argument("--usuario", default="Default User")
    ap.add_argument("--clave", default="robotics")
    ap.add_argument("--posturas", type=int, default=1)
    ap.add_argument("--prueba", action="store_true", help="sin robot: datos inventados con la forma de RWS")
    ap.add_argument("--config", default=os.path.join(RAIZ, "config.json"))
    a = ap.parse_args()

    cfg = cargar_config(a.config)
    signos = cfg["robot"]["signos"]
    offsets = cfg["robot"]["offsets_deg"]
    robot = Robot()
    cli = ClientePrueba() if a.prueba else ClienteRWS(a.ip, a.usuario, a.clave, a.puerto)

    if not a.prueba:
        try:
            tipo = cli.get("/rw/system/robottype")
            modo = cli.get("/rw/panel/opmode")
            estado = cli.get("/rw/panel/ctrlstate")
            print("Robot:", re.findall(r'"robot-type"\s*:\s*"([^"]+)"', tipo) or tipo[:120])
            print("Modo:", re.findall(r'"opmode"\s*:\s*"([^"]+)"', modo) or modo[:120], "  Estado:", re.findall(r'"ctrlstate"\s*:\s*"([^"]+)"', estado) or estado[:120])
        except Exception as e:  # noqa: BLE001
            print(f"No se pudo leer RWS en {a.ip}: {e}")
            print("Revisa: cable en SERVICE, IP 192.168.125.10/24, ping 192.168.125.1, usuario y clave.")
            sys.exit(1)

    salida = os.path.join(RAIZ, "01_docs", "datos")
    os.makedirs(salida, exist_ok=True)
    ruta_csv = os.path.join(salida, "validacion_rws.csv")
    nuevo = not os.path.exists(ruta_csv)
    filas = []
    for k in range(a.posturas):
        if k > 0 and not a.prueba:
            input(f"\nMueve el robot con el FlexPendant a otra postura (lejos de q5 = 0), deja el robot QUIETO y pulsa Enter ({k + 1}/{a.posturas})... ")
        if a.prueba and k > 0:
            cli.q = cli.q + np.array([7.1, 5.3, -6.2, 11.0, 9.4, 15.5])
        for intento in range(5):
            q1 = leer_juntas(cli)
            p, cuat = leer_pose(cli)
            q2 = leer_juntas(cli)
            if np.max(np.abs(q1 - q2)) < 0.01:
                break
            print("El robot se estaba moviendo entre lecturas (inconcluso por tiempo); reintento...")
            time.sleep(0.5)
        else:
            print("No se logró una lectura con el robot quieto. Resultado: INCONCLUSO POR TIEMPO.")
            continue
        e_p, e_R, T = comparar(robot, q1, p, cuat, signos, offsets)
        ok = e_p < 0.5 and e_R < 0.05
        print(f"\nPostura {k + 1}: q del controlador [°] = {np.round(q1, 2).tolist()}")
        print(f"  tool0 según el IRC5 : p = {np.round(p, 2).tolist()} mm   q = {np.round(cuat, 5).tolist()}")
        print(f"  tool0 según nuestra FK: p = {np.round(T[:3, 3] * 1000, 2).tolist()} mm   q = {np.round(matriz_a_cuaternion(T[:3, :3]), 5).tolist()}")
        print(f"  e_p = {e_p:.3f} mm   e_R = {e_R:.4f}°   -> {'COINCIDE' if ok else 'NO COINCIDE'}")
        if not ok:
            mejor = min(
                (comparar(robot, q1, p, cuat, s, offsets)[:2] + (s,) for s in itertools.product([1, -1], repeat=6)),
                key=lambda x: x[0] + x[1],
            )
            print(f"  Pista: con signos {list(mejor[2])} el error sería e_p = {mejor[0]:.3f} mm, e_R = {mejor[1]:.4f}°.")
            print("  Si eso coincide, corrige 'signos' en config.json. Si nada coincide: revisa base, tool0 (no un TCP) y offsets.")
        filas.append([dt.datetime.now().isoformat(timespec="seconds"), "prueba" if a.prueba else a.ip, *np.round(q1, 3), *np.round(p, 3), *np.round(cuat, 6), *np.round(T[:3, 3] * 1000, 3), round(e_p, 4), round(e_R, 5), "PASS" if ok else "FAIL", "S1 RWS jointtarget+robtarget tool0/wobj0/Base"])

    with open(ruta_csv, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if nuevo:
            w.writerow(["fecha", "fuente", *[f"q{i}_deg" for i in range(1, 7)], "x_ref_mm", "y_ref_mm", "z_ref_mm", "q1_ref", "q2_ref", "q3_ref", "q4_ref", "x_fk_mm", "y_fk_mm", "z_fk_mm", "e_p_mm", "e_R_deg", "resultado", "nivel"])
        w.writerows(filas)
    print(f"\nGuardado en {ruta_csv}. Umbral usado: e_p < 0.5 mm y e_R < 0.05° (propuesto por el grupo: el controlador redondea a 0.01 mm y 1e-6).")


if __name__ == "__main__":
    main()
