"""Compara la pose de la brida del IRC5 con la FK de VEVAS por RWS de solo lectura."""

import json
import math
import xml.etree.ElementTree as ET

import numpy as np

from vevas.robot_irb120 import cuaternion_a_matriz, error_orientacion, matriz_a_cuaternion
from vevas.rws_cliente import Falla

RUTA_POSE = "/rw/motionsystem/mechunits/ROB_1/robtarget"
PARAMETROS_POSE = {"tool": "tool0", "wobj": "wobj0", "coordinate": "Base"}
MARCO_POSE = {"unidad_mecanica": "ROB_1", "herramienta": "tool0", "objeto": "wobj0", "coordenadas": "Base"}
CAMPOS_POSE = ("x", "y", "z", "q1", "q2", "q3", "q4")
TOL_JUNTAS_DEG = 0.01
TOL_POSICION_MM = 0.5
TOL_ORIENTACION_DEG = 0.05
TOL_GIMBAL = 1e-6


class ComparacionInvalida(ValueError):
    """La captura carece de datos coherentes para una comparación."""


def _registro_pose(texto):
    """Extrae un único registro con todos los campos de la pose RWS 1.0."""
    try:
        doc = json.loads(texto)
    except json.JSONDecodeError:
        try:
            raiz = ET.fromstring(texto)
        except ET.ParseError as exc:
            raise ComparacionInvalida("RWS devolvió un formato de pose inválido.") from exc
        registros = []
        for elemento in raiz.iter():
            if elemento.tag.rsplit("}", 1)[-1] == "li":
                datos = {n.get("class"): (n.text or "").strip()
                         for n in elemento if n.get("class")}
                registros.append(datos)
    else:
        embebido = doc.get("_embedded") if isinstance(doc, dict) else None
        estados = embebido.get("_state") if isinstance(embebido, dict) else None
        registros = estados if isinstance(estados, list) else []
    poses = [r for r in registros if isinstance(r, dict) and all(k in r for k in CAMPOS_POSE)]
    if len(poses) != 1:
        raise ComparacionInvalida("RWS debe devolver una única pose completa con XYZ y q1–q4.")
    return poses[0]


def _validar_metadatos(datos):
    # Algunas respuestas no repiten los parámetros de la solicitud.
    for campo, esperado in PARAMETROS_POSE.items():
        if campo in datos and str(datos[campo]).lower() != esperado.lower():
            raise ComparacionInvalida(f"Se esperaba {campo}={esperado}, pero RWS devolvió {datos[campo]}.")
    for campo in ("unit", "position-unit"):
        if campo in datos and str(datos[campo]).lower() != "mm":
            raise ComparacionInvalida("La posición RWS debe estar expresada en milímetros.")


def _vectores_pose(texto):
    datos = _registro_pose(texto)
    _validar_metadatos(datos)
    try:
        valores = np.array([float(datos[k]) for k in CAMPOS_POSE])
    except (ValueError, TypeError, OverflowError) as exc:
        raise ComparacionInvalida("RWS devolvió campos de pose no numéricos.") from exc
    if any(isinstance(datos[k], bool) for k in CAMPOS_POSE) or not np.all(np.isfinite(valores)):
        raise ComparacionInvalida("RWS devolvió campos de pose no numéricos o no finitos.")
    posicion, cuaternion = valores[:3], valores[3:]
    if not math.isclose(math.hypot(*cuaternion), 1.0, abs_tol=1e-3):
        raise ComparacionInvalida("El cuaternión ABB recibido no tiene norma unitaria.")
    return posicion, cuaternion


def _resumen_pose(xyz_mm, cuaternion, R):
    # Euler ZYX: Rz(z) Ry(y) Rx(x). Euler ZXY: Rz(z) Rx(x) Ry(y).
    cy = math.hypot(R[0, 0], R[1, 0])
    cx = math.hypot(R[2, 0], R[2, 2])
    y_zyx = math.atan2(-R[2, 0], cy)
    x_zxy = math.atan2(R[2, 1], cx)
    # Fija en cero el último ángulo solo en la singularidad numérica.
    if cy < 1e-12:
        z_zyx, x_zyx = math.atan2(-R[0, 1], R[1, 1]), 0.0
    else:
        z_zyx, x_zyx = math.atan2(R[1, 0], R[0, 0]), math.atan2(R[2, 1], R[2, 2])
    if cx < 1e-12:
        z_zxy, y_zxy = math.atan2(R[1, 0], R[0, 0]), 0.0
    else:
        z_zxy, y_zxy = math.atan2(-R[0, 1], R[1, 1]), math.atan2(-R[2, 0], R[2, 2])
    return {
        "xyz_mm": xyz_mm.tolist(),
        "cuaternion_abb_wxyz": list(cuaternion),
        "euler_zyx_deg": [math.degrees(a) for a in (z_zyx, y_zyx, x_zyx)],
        "gimbal_zyx": cy < TOL_GIMBAL,
        "euler_zxy_deg": [math.degrees(a) for a in (z_zxy, x_zxy, y_zxy)],
        "gimbal_zxy": cx < TOL_GIMBAL,
    }


def _seis_valores(valores, nombre):
    try:
        vector = np.asarray(valores, dtype=float)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ComparacionInvalida(f"{nombre} debe contener seis valores numéricos.") from exc
    if vector.shape != (6,) or not np.all(np.isfinite(vector)):
        raise ComparacionInvalida(f"{nombre} debe contener seis valores numéricos finitos.")
    return vector


def comparar_con_controlador(cliente, robot, *, signos=(1,) * 6, offsets_deg=(0,) * 6):
    """Lee juntas, pose de tool0/Base y juntas otra vez. Devuelve un dict serializable a JSON."""
    signo = _seis_valores(signos, "signos")
    offset = _seis_valores(offsets_deg, "offsets_deg")
    if not np.all(np.abs(signo) == 1):
        raise ComparacionInvalida("Los signos articulares deben ser +1 o −1.")
    try:
        antes = _seis_valores(cliente.joints_deg(), "La primera lectura de juntas")
        pose = cliente.get(RUTA_POSE, **PARAMETROS_POSE)
        despues = _seis_valores(cliente.joints_deg(), "La segunda lectura de juntas")
    except Falla as exc:
        raise ComparacionInvalida(f"No se pudo completar la lectura RWS. {exc}") from exc
    variacion = float(np.max(np.abs(despues - antes)))
    if variacion >= TOL_JUNTAS_DEG:
        raise ComparacionInvalida(f"La captura es inestable. Una junta cambió {variacion:.3f}°. "
                                  "Repetí la medición con el brazo detenido.")

    punto_ref, cuat_ref = _vectores_pose(pose)
    rot_ref = cuaternion_a_matriz(*cuat_ref)
    q_deg = (antes + despues) * 0.5
    q_modelo = q_deg * signo + offset
    T = robot.fk_tool0(np.radians(q_modelo))
    punto, rot = T[:3, 3] * 1000.0, T[:3, :3]
    ep = math.dist(punto, punto_ref)
    er = error_orientacion(rot, rot_ref)
    if not math.isfinite(ep) or not math.isfinite(er):
        raise ComparacionInvalida("El modelo o RWS devolvió una pose fuera del rango numérico.")
    return {
        "ok": True,
        "coincide": ep < TOL_POSICION_MM and math.degrees(er) < TOL_ORIENTACION_DEG,
        "juntas_deg": q_deg.tolist(),
        "juntas_modelo_deg": q_modelo.tolist(),
        "variacion_juntas_deg": variacion,
        "frame": dict(MARCO_POSE),
        "controlador": _resumen_pose(punto_ref, cuat_ref.tolist(), rot_ref),
        "modelo": _resumen_pose(punto, matriz_a_cuaternion(rot), rot),
        "error_posicion_mm": ep,
        "error_orientacion_rad": er,
        "error_orientacion_deg": math.degrees(er),
        "tolerancia_posicion_mm": TOL_POSICION_MM,
        "tolerancia_orientacion_deg": TOL_ORIENTACION_DEG,
        "calibracion": {"estado": "no verificado", "fuente": "FlexPendant"},
    }
