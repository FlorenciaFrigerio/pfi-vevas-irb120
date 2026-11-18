#!/usr/bin/env python3
"""Consola VEVAS: cálculos en tiempo real y control del ABB IRB 120.

Ejemplos:
    # Ensayo sin ROS (cualquier computadora): simulador interno
    python3 consola_vevas.py --modo sim

    # Con ROS 2 y hardware simulado (use_fake_hardware:=true en otra terminal)
    source /opt/ros/jazzy/setup.bash && source ~/abb_ws/install/setup.bash
    python3 consola_vevas.py --modo ros

    # Con el robot real (driver abb_ros2 con use_fake_hardware:=false y EGM corriendo)
    python3 consola_vevas.py --modo ros --real --go GO_VEVAS.json

Después abre http://127.0.0.1:8000 en el navegador.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import webbrowser

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)

from vevas.backend_sim import BackendSim  # noqa: E402
from vevas.consola import Consola  # noqa: E402
from vevas.registro import Registro  # noqa: E402
from vevas.seguridad import cargar_config  # noqa: E402
from vevas.servidor import crear_servidor  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="Consola VEVAS para el ABB IRB 120")
    ap.add_argument("--modo", choices=["sim", "ros"], default="sim", help="sim = simulador interno; ros = ROS 2")
    ap.add_argument("--real", action="store_true", help="permite mover un robot REAL (exige --go y el checklist)")
    ap.add_argument("--solo-lectura", action="store_true", help="ROS sin interfaces de comando")
    ap.add_argument("--topico-estado", help="tópico JointState, por ejemplo /irb120/real/joint_states")
    ap.add_argument("--go", default=None, help="archivo GO_VEVAS.json lleno por el grupo")
    ap.add_argument("--config", default=os.path.join(AQUI, "config.json"))
    ap.add_argument("--puerto", type=int, default=None)
    ap.add_argument("--sesiones", default=os.path.join(AQUI, "sesiones"))
    ap.add_argument("--abrir", action="store_true", help="abrir el navegador automáticamente")
    args = ap.parse_args()

    if args.solo_lectura and (args.modo != "ros" or args.real):
        ap.error("--solo-lectura requiere --modo ros y es incompatible con --real")
    if args.topico_estado and args.modo != "ros":
        ap.error("--topico-estado requiere --modo ros")

    cfg = cargar_config(args.config)
    if args.topico_estado:
        cfg["ros"]["topico_estado"] = args.topico_estado
    puerto = args.puerto or cfg["servidor"]["puerto"]
    go = None
    if args.go:
        with open(args.go, encoding="utf-8") as f:
            go = json.load(f)

    registro = Registro(args.sesiones, args.modo + ("_real" if args.real else ""))
    avisos_previos = []

    if args.modo == "sim":
        q0 = [math.radians(v) for v in cfg["postura_inicial_deg"]]
        backend = BackendSim(q0)
    else:
        try:
            from vevas.backend_ros import BackendROS
        except ImportError as e:
            print(f"No se pudo importar rclpy ({e}). ¿Hiciste 'source /opt/ros/jazzy/setup.bash'?")
            sys.exit(1)
        backend = BackendROS(cfg, lambda t: avisos_previos.append(t), solo_lectura=args.solo_lectura)

    consola = Consola(cfg, backend, registro, permitir_real=args.real, go=go)
    if args.modo == "ros":
        backend._avisar = lambda t: consola.avisar(t, "info")
    for t in avisos_previos:
        consola.avisar(t)

    backend.iniciar()
    consola.iniciar()
    registro.evento("inicio", modo=args.modo, real=args.real, config=cfg, go=go, solo_lectura=args.solo_lectura)
    consola.avisar(f"Consola iniciada en modo {args.modo.upper()}. Registro en {registro.carpeta}", "ok")

    servidor = crear_servidor(consola, os.path.join(AQUI, "web"), cfg["servidor"]["host"], puerto, cfg["servidor"]["hz_estado"])
    url = f"http://{cfg['servidor']['host']}:{puerto}"
    print(f"\n  Consola VEVAS lista: abre {url} en el navegador.")
    print(f"  Modo: {args.modo}{' (ROBOT REAL PERMITIDO)' if args.real else ''}.  Registro: {registro.carpeta}")
    print("  Ctrl+C para cerrar y guardar el registro.\n")

    def salir(*_):
        print("\nCerrando la consola y guardando el registro.")
        try:
            consola.parar("cierre de la consola")
        finally:
            consola.detener()
            backend.detener()
            registro.cerrar()
            threading.Thread(target=servidor.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, salir)
    signal.signal(signal.SIGTERM, salir)
    if args.abrir:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    servidor.serve_forever()


if __name__ == "__main__":
    main()
