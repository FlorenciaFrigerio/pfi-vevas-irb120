#!/usr/bin/env python3
"""Lee la postura del IRC5 y la compara con VEVAS sin enviar órdenes al robot."""

import argparse
import json
import os
import sys
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ))

from vevas.robot_irb120 import Robot  # noqa: E402
from vevas.rws_cliente import Cliente  # noqa: E402
from vevas.seguridad import cargar_config  # noqa: E402
from vevas.verificacion_pendant import ComparacionInvalida, comparar_con_controlador  # noqa: E402


def _mostrar(datos):
    marco = datos["frame"]
    print(f'{marco["unidad_mecanica"]} · {marco["herramienta"]} · {marco["objeto"]} · {marco["coordenadas"]}')
    print("Juntas IRC5 [°]", datos["juntas_deg"])
    print("Juntas del modelo [°]", datos["juntas_modelo_deg"])
    for fuente in ("controlador", "modelo"):
        pose = datos[fuente]
        print(f'Posición {fuente} XYZ [mm]', pose["xyz_mm"])
        print(f'Cuaternión {fuente} ABB [w, x, y, z]', pose["cuaternion_abb_wxyz"])
        print(f'Euler {fuente} ZYX [°]', pose["euler_zyx_deg"],
              "(cerca del bloqueo)" if pose["gimbal_zyx"] else "")
        print(f'Euler {fuente} ZXY [°]', pose["euler_zxy_deg"],
              "(cerca del bloqueo)" if pose["gimbal_zxy"] else "")
    resultado = "COINCIDE" if datos["coincide"] else "DIFIERE"
    print(f'{resultado} · error XYZ {datos["error_posicion_mm"]:.4f} mm · '
          f'error de orientación {datos["error_orientacion_rad"]:.6g} rad ({datos["error_orientacion_deg"]:.4f}°)')
    calibracion = datos["calibracion"]
    print(f'Estado de calibración {calibracion["estado"]}. Consultá el {calibracion["fuente"]}.')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path, default=RAIZ / "config.json")
    ap.add_argument("--ip")
    ap.add_argument("--rws-puerto", "--puerto", type=int)
    ap.add_argument("--usuario")
    ap.add_argument("--clave", default=os.environ.get("VEVAS_RWS_CLAVE"))
    ap.add_argument("--json", action="store_true", help="emite el resultado en JSON")
    a = ap.parse_args(argv)
    try:
        cfg = cargar_config(a.config)
        rws, convencion = cfg["rws"], cfg["robot"]
    except (OSError, ValueError, KeyError) as exc:
        print(f"No se pudo cargar la configuración. {exc}", file=sys.stderr)
        return 2

    cliente = Cliente(a.ip or rws["ip"], a.usuario or rws["usuario"], a.clave or rws["clave"],
                      a.rws_puerto if a.rws_puerto is not None else rws["puerto"], rws["timeout_http_s"])
    try:
        datos = comparar_con_controlador(cliente, Robot(), signos=convencion["signos"],
                                         offsets_deg=convencion["offsets_deg"])
    except ComparacionInvalida as exc:
        if a.json:
            print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        else:
            print(str(exc), file=sys.stderr)
        return 2
    finally:
        cliente.cerrar()
    if a.json:
        print(json.dumps(datos, ensure_ascii=False, allow_nan=False, indent=2))
    else:
        _mostrar(datos)
    return 0 if datos["coincide"] else 1


if __name__ == "__main__":
    sys.exit(main())
