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

    # Con el robot real por RWS (sin ROS ni EGM): módulo RAPID VEVAS_RWS ya corriendo (VevasServidor)
    python3 consola_vevas.py --modo rws --real --go GO_VEVAS.json
    python3 consola_vevas.py --modo rws --real --go GO_VEVAS.json --preparar-rapid   # además instala y arranca el servidor (AUTO)
    python3 consola_vevas.py --modo rws --solo-lectura                               # solo mira la postura

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
from vevas.consola import Consola, validar_go  # noqa: E402
from vevas.registro import Registro  # noqa: E402
from vevas.seguridad import cargar_config  # noqa: E402
from vevas.servidor import crear_servidor  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="Consola VEVAS para el ABB IRB 120")
    ap.add_argument("--modo", choices=["sim", "ros", "rws"], default="sim",
                    help="sim = simulador interno; ros = ROS 2; rws = IRC5 por RWS (módulo RAPID VEVAS_RWS)")
    ap.add_argument("--real", action="store_true", help="permite mover un robot REAL (exige --go y el checklist)")
    ap.add_argument("--solo-lectura", action="store_true", help="ROS sin interfaces de comando")
    ap.add_argument("--topico-estado", help="tópico JointState, por ejemplo /irb120/real/joint_states")
    ap.add_argument("--go", default=None, help="archivo GO_VEVAS.json lleno por el grupo")
    ap.add_argument("--ip", help="IP del IRC5 (solo --modo rws; por defecto la de config.json)")
    ap.add_argument("--rws-puerto", type=int, help="puerto HTTP de RWS (solo --modo rws; por defecto 80)")
    ap.add_argument("--usuario", help="usuario RWS (solo --modo rws)")
    ap.add_argument("--clave", help="clave RWS (solo --modo rws; también se lee VEVAS_RWS_CLAVE)")
    ap.add_argument("--preparar-rapid", action="store_true",
                    help="--modo rws: instala VEVAS_RWS si falta, pone el PP en VevasServidor y arranca RAPID (solo AUTO)")
    ap.add_argument("--config", default=os.path.join(AQUI, "config.json"))
    ap.add_argument("--puerto", type=int, default=None)
    ap.add_argument("--sesiones", default=os.path.join(AQUI, "sesiones"))
    ap.add_argument("--datos", default=os.path.join(AQUI, "datos"),
                    help="carpeta de datos locales (poses_guardadas.json); no se versiona")
    ap.add_argument("--abrir", action="store_true", help="abrir el navegador automáticamente")
    args = ap.parse_args()
    if args.real and args.modo not in ("ros", "rws"):
        ap.error("--real requiere --modo ros o --modo rws")
    if args.real and not args.go:
        ap.error("--real exige --go GO_VEVAS.json (archivo completo y del día)")

    if args.solo_lectura and (args.modo not in ("ros", "rws") or args.real):
        ap.error("--solo-lectura requiere --modo ros o rws y es incompatible con --real")
    if args.modo == "rws" and not args.real and not args.solo_lectura:
        print("Aviso: --modo rws sin --real: se abre en SOLO LECTURA (para mover hace falta --real y --go).")
    if (args.ip or args.usuario or args.clave or args.rws_puerto or args.preparar_rapid) and args.modo != "rws":
        ap.error("--ip, --usuario, --clave y --preparar-rapid requieren --modo rws")
    if args.preparar_rapid and (not args.real or args.solo_lectura):
        ap.error("--preparar-rapid arranca RAPID: exige --real y --go (y no vale con --solo-lectura)")
    if args.topico_estado and args.modo != "ros":
        ap.error("--topico-estado requiere --modo ros")

    cfg = cargar_config(args.config)
    if args.topico_estado:
        cfg["ros"]["topico_estado"] = args.topico_estado
    if args.modo == "rws":
        for k, v in (("ip", args.ip), ("puerto", args.rws_puerto), ("usuario", args.usuario), ("clave", args.clave or os.environ.get("VEVAS_RWS_CLAVE"))):
            if v:
                cfg["rws"][k] = v
        # La postura llega por HTTP a ~10 Hz: el watchdog de 0.5 s se sube al valor de cfg["rws"] (nunca se baja).
        cfg["seguridad"]["watchdog_estado_s"] = max(cfg["seguridad"]["watchdog_estado_s"], cfg["rws"]["watchdog_estado_s"])
    puerto = cfg["servidor"]["puerto"] if args.puerto is None else args.puerto
    go = None
    if args.go:
        with open(args.go, encoding="utf-8") as f:
            go = json.load(f)
    if args.real:
        ok_go, mensaje_go = validar_go(go)
        if not ok_go:
            ap.error("GO rechazado antes de crear interfaces o preparar RAPID: " + mensaje_go)
        if "entorno" in go:
            if not isinstance(go["entorno"], dict):
                ap.error("El entorno de sesión debe ser un objeto JSON")
            from copy import deepcopy
            cfg["entorno"] = deepcopy(go["entorno"])

    registro = Registro(args.sesiones, args.modo + ("_real" if args.real else ""))
    avisos_previos = []

    if args.modo == "sim":
        q0 = [math.radians(v) for v in cfg["postura_inicial_deg"]]
        backend = BackendSim(q0)
    elif args.modo == "rws":
        from vevas.backend_rws import BackendRWS
        solo = args.solo_lectura or not args.real
        backend = BackendRWS(cfg, lambda t: avisos_previos.append(t), solo_lectura=solo)
    else:
        try:
            from vevas.backend_ros import BackendROS
        except ImportError as e:
            print(f"No se pudo importar rclpy ({e}). ¿Hiciste 'source /opt/ros/jazzy/setup.bash'?")
            sys.exit(1)
        backend = BackendROS(cfg, lambda t: avisos_previos.append(t), solo_lectura=args.solo_lectura)

    consola = Consola(cfg, backend, registro, permitir_real=args.real, go=go, ruta_go=args.go if args.real else None,
                      ruta_poses=os.path.join(args.datos, "poses_guardadas.json"))
    if args.modo in ("ros", "rws"):
        backend._avisar = lambda t: consola.avisar(t, "info")
    for t in avisos_previos:
        consola.avisar(t)

    backend.iniciar()
    if args.preparar_rapid:
        from vevas.rws_cliente import Falla
        ruta_mod = cfg["rws"]["modulo_rapid"]
        ruta_mod = ruta_mod if os.path.isabs(ruta_mod) else os.path.join(AQUI, ruta_mod)
        print(f"Preparando RAPID en {cfg['rws']['ip']} (módulo {ruta_mod})...")
        try:
            backend.preparar_rapid(ruta_mod)
        except Falla as e:
            print(f"\nERROR al preparar RAPID: {e}\nNo se abre la consola. Resolvelo (llave en AUTO, motores apagados, RAPID detenido) y reintentá.")
            backend.detener()
            registro.cerrar()
            sys.exit(1)
        print("RAPID listo: VevasServidor activo.")
    consola.iniciar()
    from copy import deepcopy
    cfg_registro = deepcopy(cfg)
    cfg_registro.get("rws", {}).pop("clave", None)
    registro.evento("inicio", modo=args.modo, real=args.real, config=cfg_registro, go=go, solo_lectura=args.solo_lectura)
    consola.avisar(f"Consola iniciada en modo {args.modo.upper()}. Registro en {registro.carpeta}", "ok")

    servidor = crear_servidor(consola, os.path.join(AQUI, "08_web"), cfg["servidor"]["host"], puerto, cfg["servidor"]["hz_estado"])
    url = f"http://{cfg['servidor']['host']}:{servidor.server_address[1]}"
    print(f"\n  Consola VEVAS lista: abre {url} en el navegador.", flush=True)
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
