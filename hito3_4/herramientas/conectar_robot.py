#!/usr/bin/env python3
"""Diagnostica desde Ubuntu la conexión al IRC5 y toma una captura RWS en solo lectura."""

from __future__ import annotations

import argparse
import datetime as dt
import ipaddress
import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ))

from vevas.robot_irb120 import Robot  # noqa: E402
from vevas.rws_cliente import Cliente, Falla  # noqa: E402
from vevas.seguridad import cargar_config  # noqa: E402
from vevas.verificacion_pendant import ComparacionInvalida, comparar_con_controlador  # noqa: E402

REDES_PRIVADAS = tuple(map(ipaddress.IPv4Network, ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16")))
ESTADOS_PANEL = {
    "opmode": {"AUTO", "MANR", "MANF", "INIT", "UNDEF"},
    "ctrlstate": {"init", "motoron", "motoroff", "guardstop", "emergencystop", "emergencystopreset", "sysfail"},
    "exec": {"running", "stopped"},
}


def _configuracion(archivo, ip):
    if not archivo.is_file():
        raise ValueError("No se encontró el archivo de configuración.")
    try:
        cfg = cargar_config(archivo)
        rws = cfg["rws"]
        destino = ip if ip is not None else rws["ip"]
        puerto = rws["puerto"]
        usuario = rws["usuario"]
        clave = os.environ.get("VEVAS_RWS_CLAVE") or rws["clave"]
        timeout = float(rws["timeout_http_s"])
        limites = (float(cfg["seguridad"]["watchdog_estado_s"]), float(rws["watchdog_estado_s"]))
        convencion = cfg["robot"]
        signos = tuple(map(float, convencion["signos"]))
        offsets = tuple(map(float, convencion["offsets_deg"]))
    except (OSError, ValueError, TypeError, AttributeError, KeyError, OverflowError):
        raise ValueError("No se pudo cargar la configuración. Revisá el archivo y el perfil que referencia.") from None
    try:
        if not isinstance(destino, str):
            raise ValueError
        direccion = ipaddress.IPv4Address(destino)
    except ValueError:
        raise ValueError("Indicá la dirección IPv4 privada del IRC5 como una IP, sin URL ni puerto.") from None
    if not (direccion.is_loopback or any(direccion in red for red in REDES_PRIVADAS)):
        raise ValueError("Usá la IPv4 privada del IRC5 o una dirección de loopback para un controlador virtual local.")
    if type(puerto) is not int or not 1 <= puerto <= 65535:
        raise ValueError("El puerto RWS debe ser un entero entre 1 y 65535.")
    if not isinstance(usuario, str) or not usuario.strip() or not isinstance(clave, str):
        raise ValueError("Revisá los campos de autenticación RWS de la configuración.")
    if not all(math.isfinite(t) and t > 0 for t in (timeout, *limites)):
        raise ValueError("El tiempo de espera y los límites de frescura RWS deben ser positivos y finitos.")
    if len(signos) != 6 or any(abs(s) != 1 for s in signos) or len(offsets) != 6 or not all(map(math.isfinite, offsets)):
        raise ValueError("La convención articular debe tener seis signos +1 o −1 y seis offsets finitos.")
    return direccion, puerto, usuario, clave, timeout, max(limites), signos, offsets


def _ip_json(*args):
    try:
        proceso = subprocess.run(["ip", "-j", *args], capture_output=True, text=True, timeout=3, check=False)
    except FileNotFoundError:
        raise ValueError("Falta el comando ip de iproute2 en Ubuntu.") from None
    except subprocess.TimeoutExpired:
        raise ValueError("Venció el tiempo de espera al consultar la red con ip.") from None
    except OSError:
        raise ValueError("No se pudo ejecutar ip para consultar la red.") from None
    if proceso.returncode:
        raise ValueError("Ubuntu no encontró una ruta al IRC5 o no pudo consultar las interfaces.")
    try:
        salida = json.loads(proceso.stdout)
        if not isinstance(salida, list) or not all(isinstance(item, dict) for item in salida):
            raise ValueError
    except ValueError:
        raise ValueError("La salida de ip no contiene el formato JSON esperado.") from None
    return salida


def _diagnosticar_red(direccion, red):
    rutas = _ip_json("route", "get", str(direccion))
    interfaces = _ip_json("address")
    if len(rutas) != 1:
        raise ValueError("Ubuntu no devolvió una ruta única hacia el IRC5.")
    ruta = rutas[0]
    interfaz = ruta.get("dev")
    origen = ruta.get("prefsrc") or ruta.get("src")
    if not isinstance(interfaz, str) or not interfaz or not interfaz.isprintable():
        raise ValueError("La ruta al IRC5 no indica una interfaz válida.")
    red["interfaz"] = interfaz
    try:
        red["origen"] = str(ipaddress.IPv4Address(origen))
    except ipaddress.AddressValueError:
        raise ValueError("La ruta al IRC5 no indica una IPv4 de origen válida.") from None
    simulador_local = direccion.is_loopback and interfaz == "lo" and ruta.get("type") == "local"
    if ruta.get("gateway") or (not simulador_local and (ruta.get("type", "unicast") != "unicast" or interfaz == "lo")):
        raise ValueError("La ruta al IRC5 debe salir directamente por la interfaz de la red del laboratorio.")
    if "linkdown" in (ruta.get("flags") or []):
        raise ValueError("El enlace elegido por Ubuntu está caído. Revisá el cable y el puerto SERVICE.")
    for dispositivo in interfaces:
        if dispositivo.get("ifname") != interfaz:
            continue
        if dispositivo.get("operstate") in ("DOWN", "LOWERLAYERDOWN"):
            raise ValueError("El enlace elegido por Ubuntu está caído. Revisá el cable y el puerto SERVICE.")
        for dato in dispositivo.get("addr_info") or []:
            if dato.get("family") != "inet" or dato.get("local") != origen:
                continue
            try:
                local = ipaddress.IPv4Interface(f"{origen}/{dato['prefixlen']}")
            except (ValueError, KeyError):
                continue
            red["origen_cidr"] = str(local)
            if (
                (simulador_local or direccion != local.ip) and direccion in local.network
                and (local.network.prefixlen >= 31 or direccion not in
                     (local.network.network_address, local.network.broadcast_address))
            ):
                return
    raise ValueError("La interfaz elegida no tiene una IPv4 de origen compatible con la subred del IRC5.")


def _ping(direccion):
    inicio = time.monotonic()
    try:
        prueba = subprocess.run(
            ["ping", "-n", "-c", "1", "-W", "2", str(direccion)],
            capture_output=True, text=True, timeout=4, check=False,
        )
        resultado = {0: "respuesta", 1: "sin respuesta"}.get(prueba.returncode, "error de ping")
    except subprocess.TimeoutExpired:
        resultado = "tiempo de espera agotado"
    except OSError:
        resultado = "comando no disponible"
    return {"resultado": resultado, "duracion_s": round(time.monotonic() - inicio, 3)}


def _error_rws(exc):
    # Los cuerpos y errores de transporte pueden incluir datos sensibles. Conservar solo el diagnóstico.
    detalle = str(exc)
    codigo = re.match(r"^GET /[^ ]+ → HTTP (\d{3}):", detalle)
    if codigo:
        numero = int(codigo[1])
        mensaje = {
            401: "Revisá las credenciales RWS con el responsable del laboratorio.",
            403: "La cuenta RWS no puede consultar ese recurso.",
            404: "El recurso consultado no está disponible en este controlador.",
        }.get(numero, "El controlador rechazó la consulta.")
        return {"http": numero, "mensaje": mensaje}
    if "tiempo de frescura" in detalle:
        return {"mensaje": "La respuesta de juntas superó el límite de frescura."}
    if "No hay respuesta de RWS" in detalle:
        return {"mensaje": "Sin respuesta HTTP de RWS. Revisá el cable, la IP y el puerto RWS."}
    return {"mensaje": "RWS devolvió una lectura incompleta o ilegible."}


def _error_comparacion(exc):
    if isinstance(exc.__cause__, Falla):
        return _error_rws(exc.__cause__)
    if str(exc).startswith("La captura es inestable."):
        return {"mensaje": "La captura fue inestable. Repetila con el brazo detenido."}
    return {"mensaje": "No se pudo comparar la pose y las juntas de esa captura de tool0 en Base."}


def _capturar(cliente, signos, offsets, rws):
    try:
        panel = cliente.leer_estado(con_juntas=False)
        rws["panel"] = {
            campo: panel.get(campo) if panel.get(campo) in valores else "?"
            for campo, valores in ESTADOS_PANEL.items()
        }
    except Falla as exc:
        rws["error_panel"] = _error_rws(exc)

    inicio = time.monotonic()
    try:
        rws["juntas_deg"] = cliente.joints_deg()
        rws["lectura_juntas_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    except Falla as exc:
        rws["error_juntas"] = _error_rws(exc)
    rws["duracion_lectura_juntas_s"] = round(time.monotonic() - inicio, 3)

    inicio = time.monotonic()
    try:
        rws["comparacion_fk"] = comparar_con_controlador(cliente, Robot(), signos=signos, offsets_deg=offsets)
        rws["captura_fk_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    except ComparacionInvalida as exc:
        rws["error_comparacion_fk"] = _error_comparacion(exc)
    rws["duracion_captura_fk_s"] = round(time.monotonic() - inicio, 3)


def _diagnosticar(args, datos):
    try:
        ip, puerto, usuario, clave, timeout, frescura, signos, offsets = _configuracion(args.config, args.ip)
    except ValueError as exc:
        datos["error_config"] = str(exc)
        return 2
    red = datos["red"] = {"destino": str(ip)}
    try:
        _diagnosticar_red(ip, red)
    except ValueError as exc:
        red["error"] = str(exc)
        return 2

    if args.ping:
        red["ping"] = _ping(ip)

    rws = datos["rws"] = {"puerto": puerto, "limite_frescura_juntas_s": frescura}
    cliente = Cliente(str(ip), usuario, clave, puerto, timeout)
    cliente.max_edad_juntas_s = frescura
    try:
        _capturar(cliente, signos, offsets, rws)
    finally:
        cliente.cerrar()  # El cliente cierra la sesión con GET /logout.
    comparacion = rws.get("comparacion_fk")
    if comparacion is None:
        return 2
    return 0 if comparacion["coincide"] else 1


def _mostrar(datos):
    print("Diagnóstico IRC5 · SOLO LECTURA")
    if "error_config" in datos:
        print(datos["error_config"], file=sys.stderr)
        return
    red = datos["red"]
    origen = red.get("origen_cidr", red.get("origen", "sin detectar"))
    print(f'IRC5 {red["destino"]} · interfaz {red.get("interfaz", "sin detectar")} · origen {origen}')
    if "error" in red:
        print(red["error"], file=sys.stderr)
        return
    if "ping" in red:
        ping = red["ping"]
        print(f'Ping: {ping["resultado"]} ({ping["duracion_s"]:.3f} s).')
    rws = datos["rws"]
    print(f'RWS HTTP · puerto {rws["puerto"]} · límite de frescura por lectura de juntas {rws["limite_frescura_juntas_s"]:g} s')
    if "panel" in rws:
        panel = rws["panel"]
        print(f'Panel · modo {panel["opmode"]} · controlador {panel["ctrlstate"]} · RAPID {panel["exec"]}')
    if "juntas_deg" in rws:
        print(f'Juntas J1–J6 [°] {rws["juntas_deg"]} · lectura {rws["duracion_lectura_juntas_s"]:.3f} s '
              f'· recibida {rws["lectura_juntas_utc"]}')
    for etapa in ("panel", "juntas", "comparacion_fk"):
        error = rws.get(f"error_{etapa}")
        if error:
            http = f'HTTP {error["http"]} · ' if "http" in error else ""
            print(f'{etapa.replace("_", " ").capitalize()} · {http}{error["mensaje"]}', file=sys.stderr)
    if "comparacion_fk" in rws:
        fk = rws["comparacion_fk"]
        veredicto = "COINCIDE" if fk["coincide"] else "DIFIERE"
        print(f'FK tool0 · wobj0 · Base · {veredicto} · error XYZ {fk["error_posicion_mm"]:.4f} mm '
              f'· error angular {fk["error_orientacion_deg"]:.4f}° · captura {rws["duracion_captura_fk_s"]:.3f} s')
        print("Calibración sin verificar. Consultá el FlexPendant.")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", help="IPv4 del IRC5. Por defecto se usa la configuración (192.168.125.1).")
    parser.add_argument("--config", type=Path, default=RAIZ / "config.json", help="Archivo de configuración de VEVAS.")
    parser.add_argument("--ping", action="store_true", help="Sondea ICMP antes de RWS.")
    parser.add_argument("--salida", type=Path, help="Guarda el diagnóstico en un archivo JSON nuevo.")
    args = parser.parse_args(argv)

    datos = {"solo_lectura": True, "inicio_utc": dt.datetime.now(dt.timezone.utc).isoformat()}
    codigo = _diagnosticar(args, datos)
    datos["codigo_salida"] = codigo
    _mostrar(datos)
    if args.salida is not None:
        try:
            with args.salida.open("x", encoding="utf-8") as archivo:
                archivo.write(json.dumps(datos, ensure_ascii=False, allow_nan=False, indent=2) + "\n")
        except OSError:
            print("No se pudo guardar el diagnóstico. Usá una carpeta existente y un nombre de archivo nuevo.", file=sys.stderr)
            return 2
    return codigo


if __name__ == "__main__":
    sys.exit(main())
