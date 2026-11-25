#!/usr/bin/env python3
"""Ayudante RWS para el IRC5 (RobotWare 6, RWS 1.0), ruta VEVAS_RWS. Solo biblioteca estándar.

Es una CLI fina sobre vevas/rws_cliente.py (una sola implementación del protocolo).

Subcomandos:
  estado      modo, motores, RAPID, módulo VEVAS_RWS, puntero y juntas, más las variables del servidor (solo lectura)
  instalar    carga VEVAS_RWS desde rapid/VEVAS_RWS.mod si falta (o lo actualiza si el texto difiere). Solo AUTO, motores OFF
  arrancar    motores ON + PP a VEVAS_RWS/VevasServidor (verificado) + Start. SOLO si el controlador está en AUTO
  servidor    comprueba que VevasServidor esté activo (vvRbActivo=1, listo para recibir); sale con 0 si sí
  fin         vvPcFin := 1 (VevasServidor termina solo)
  parar       Stop de RAPID + motores OFF
  pp-main     puntero de programa de vuelta a main (resetpp)
  comparar    compara las juntas del IRC5 (RWS) con /joint_states de ROS

Nunca cambia el modo de operación (la llave es física), nunca puentea cadenas de seguridad.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
sys.path.insert(0, RAIZ)

from vevas.rws_cliente import MODULO, RUTINA, Cliente, Falla  # noqa: E402

MOD_LOCAL = os.path.join(RAIZ, "rapid", "VEVAS_RWS.mod")


def cmd_estado(c: Cliente, a):
    e = c.leer_estado()
    srv = None
    if e["modulo_vevas"]:
        try:
            srv = c.estado_servidor()
        except Falla:
            srv = None
    if a.shell:
        print(f"OPMODE={e['opmode']}")
        print(f"CTRLSTATE={e['ctrlstate']}")
        print(f"EXEC={e['exec']}")
        print(f"MODULO_VEVAS={'si' if e['modulo_vevas'] else 'no'}")
        print(f"PP_MODULO={e['pp_modulo']}")
        print(f"PP_RUTINA={e['pp_rutina']}")
        print("Q_DEG=" + ",".join(f"{x:.3f}" for x in e["q_deg"]))
        if srv:
            print(f"RB_ACTIVO={srv['vvRbActivo']}")
            print(f"RB_CODIGO={srv['vvRbCodigo']}")
            print(f"PC_SEQ={srv['vvPcSeq']}")
            print(f"RB_SEQ={srv['vvRbSeq']}")
    else:
        e.pop("modulos", None)
        print(json.dumps({**e, "servidor": srv}, indent=2, ensure_ascii=False))
    return 0


def exigir_go(a):
    from vevas.consola import validar_go
    if not getattr(a, "go", None):
        raise Falla("Esta operación requiere --go con un registro explícito de la sesión. No se genera automáticamente.")
    try:
        with open(a.go, encoding="utf-8") as archivo:
            go = json.load(archivo)
    except (OSError, ValueError) as error:
        raise Falla("No se pudo leer el GO: " + str(error)) from error
    ok, mensaje = validar_go(go)
    if not ok:
        raise Falla("GO rechazado: " + mensaje)
    return go


def cmd_instalar(c: Cliente, a):
    exigir_go(a)
    r = c.instalar_modulo(a.modulo, forzar=a.recargar)
    print(f"Módulo {MODULO}: {r}.")
    return 0


def cmd_arrancar(c: Cliente, a):
    exigir_go(a)
    c.preparar_y_arrancar()
    print(f"RAPID corriendo en {RUTINA}.")
    return 0


def cmd_servidor(c: Cliente, a):
    if c.esperar_servidor(a.espera, pausa_inicial=a.pausa):
        problemas = c.problemas_para_enviar()
        if problemas:
            print("Servidor activo, pero no listo: " + "; ".join(problemas))
            return 1
        print("VevasServidor activo y listo (vvRbActivo=1, vvRbCodigo=0).")
        return 0
    print("VevasServidor NO está activo (vvRbActivo=0): con el .mod por defecto hay que pulsar «Iniciar» en el FlexPendant antes de 30 s.")
    return 1


def cmd_fin(c: Cliente, a):
    c.terminar_servidor()
    print("vvPcFin := 1 escrito: VevasServidor termina.")
    return 0


def cmd_parar(c: Cliente, a):
    r = c.parar()
    for err in r.get("errores", []):
        print("AVISO:", err, file=sys.stderr)
    print(f"Estado: exec={r.get('exec')} motores={r.get('ctrlstate')}")
    return 0 if r.get("ok") else 1


def cmd_ppmain(c: Cliente, a):
    c.pp_main()
    print("Puntero de programa en main.")
    return 0


def _ros_echo(topico, campo, timeout):
    r = subprocess.run(["ros2", "topic", "echo", topico, "--once", "--field", campo],
                       capture_output=True, text=True, timeout=timeout)
    if r.returncode != 0 or not r.stdout.strip():
        raise Falla(f"No llegó {topico} ({campo}): {r.stderr.strip()[:150]}")
    return r.stdout


def cmd_comparar(c: Cliente, a):
    nombres = re.findall(r"joint_\d", _ros_echo(a.topico, "name", a.espera))
    pos = [float(x) for x in re.findall(r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?", _ros_echo(a.topico, "position", a.espera))]
    if nombres != [f"joint_{i}" for i in range(1, 7)] or len(pos) != 6:
        raise Falla(f"Nombres u orden inesperados en {a.topico}: {nombres}, {len(pos)} posiciones.")
    q_ros = [math.degrees(x) for x in pos]
    q_rws = c.joints_deg()
    dif = [abs(x - y) for x, y in zip(q_ros, q_rws)]
    print("ROS :", " ".join(f"{x:8.2f}" for x in q_ros))
    print("RWS :", " ".join(f"{x:8.2f}" for x in q_rws))
    print(f"Diferencia máxima: {max(dif):.2f}° (tolerancia {a.tol}°)")
    if max(dif) > a.tol:
        raise Falla("ROS y el controlador no coinciden: signos, offsets o un estado viejo.")
    return 0


def cmd_diagnostico(c: Cliente, a):
    """Muestra los últimos mensajes del IRC5 y, si nombran una línea, esa parte del módulo en el controlador."""
    import re as _re
    evs = c.eventos(a.n)
    if not evs:
        print("No se pudo leer el registro de eventos del controlador.")
    lineas = set()
    for e in evs:
        if a.solo_errores and e["tipo"] not in ("ERROR", "aviso"):
            continue
        print(f"  [{e['tipo']}] {e['codigo']} {e['fecha'] or ''}  {e['titulo'] or ''}")
        for k in ("desc", "causas", "acciones"):
            if e.get(k):
                print(f"      {k}: {str(e[k]).strip()[:400]}")
        for m in _re.finditer(r"(?:line|l[ií]nea|row|fila)\D{0,3}(\d+)", " ".join(str(e.get(k) or "") for k in ("titulo", "desc", "causas")), _re.I):
            lineas.add(int(m.group(1)))
    if lineas:
        try:
            texto = c.texto_modulo().splitlines()
        except Falla:
            texto = []
        for ln in sorted(lineas):
            if texto and 1 <= ln <= len(texto):
                print(f"  Módulo VEVAS_RWS en el controlador, alrededor de la línea {ln}:")
                for i in range(max(1, ln - 2), min(len(texto), ln + 2) + 1):
                    print(f"    {'>' if i == ln else ' '}{i:4d}: {texto[i - 1]}")
    return 0


def cmd_texto(c: Cliente, a):
    """Imprime el módulo VEVAS_RWS tal como está en el controlador y lo compara con el archivo local."""
    from vevas.rws_cliente import diferencias
    t = c.texto_modulo()
    if not t:
        t = c._texto_modulo()
    for i, l in enumerate(t.replace("\r\n", "\n").split("\n"), 1):
        if a.desde <= i <= a.hasta:
            print(f"{i:4d}: {l}")
    with open(a.modulo, encoding="utf-8") as f:
        dif = diferencias(f.read(), t)
    print("\n== Diferencias con el archivo local ==" if dif else "\n== Idéntico al archivo local (salvo valores de PERS) ==")
    for l in dif[:60]:
        print(l)
    return 0


def cmd_mover(c: Cliente, a):
    """La CLI no salta el bloqueo de velocidad articular del controlador."""
    exigir_go(a)
    raise Falla(
        "Movimiento RWS bloqueado: speeddata vPc limita TCP/orientación, no demuestra el tope de cada junta del GO. "
        "Se requiere implementar y validar ese control antes de mandar objetivos. "
        "No se instaló, arrancó ni envió una secuencia. Estado/diagnóstico y modo solo lectura siguen disponibles."
    )


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.125.1")
    ap.add_argument("--puerto", type=int, default=80)
    ap.add_argument("--usuario", default="Default User")
    ap.add_argument("--clave", default=os.environ.get("VEVAS_RWS_CLAVE", "robotics"))
    ap.add_argument("--timeout", type=float, default=5.0)
    ap.add_argument("--go", help="registro explícito y válido de esta sesión, obligatorio para instalar/arrancar/mover")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("estado"); s.add_argument("--shell", action="store_true")
    s = sub.add_parser("instalar"); s.add_argument("--recargar", action="store_true"); s.add_argument("--modulo", default=MOD_LOCAL)
    sub.add_parser("arrancar")
    s = sub.add_parser("servidor"); s.add_argument("--espera", type=float, default=40.0); s.add_argument("--pausa", type=float, default=1.0)
    sub.add_parser("fin"); sub.add_parser("parar"); sub.add_parser("pp-main")
    s = sub.add_parser("mover"); s.add_argument("q", nargs=6, type=float, metavar="J"); s.add_argument("--rel", action="store_true")
    s.add_argument("--paso", type=float, default=10.0); s.add_argument("--timeout", type=float, default=120.0); s.add_argument("-y", "--si", action="store_true")
    s = sub.add_parser("texto"); s.add_argument("--desde", type=int, default=1); s.add_argument("--hasta", type=int, default=9999); s.add_argument("--modulo", default=MOD_LOCAL)
    s = sub.add_parser("diagnostico"); s.add_argument("-n", type=int, default=8); s.add_argument("--solo-errores", action="store_true")
    s = sub.add_parser("comparar"); s.add_argument("--topico", default="/joint_states"); s.add_argument("--tol", type=float, default=1.0); s.add_argument("--espera", type=float, default=10.0)
    a = ap.parse_args(argv)
    c = Cliente(a.ip, a.usuario, a.clave, a.puerto, a.timeout)
    f = {"estado": cmd_estado, "instalar": cmd_instalar, "arrancar": cmd_arrancar, "servidor": cmd_servidor, "fin": cmd_fin,
         "parar": cmd_parar, "pp-main": cmd_ppmain, "comparar": cmd_comparar,
         "diagnostico": cmd_diagnostico, "texto": cmd_texto, "mover": cmd_mover}[a.cmd]
    try:
        return f(c, a)
    except Falla as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    except subprocess.TimeoutExpired:
        print("ERROR: tiempo agotado esperando ROS.", file=sys.stderr)
        return 1
    finally:
        c.cerrar()


if __name__ == "__main__":
    sys.exit(main())
