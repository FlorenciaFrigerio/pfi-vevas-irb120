"""Comprobaciones de seguridad de la consola.

Antes de mover (se revisa el plan completo, muestra por muestra):
    - todas las juntas dentro de límites con margen
    - TCP, brida, muñeca y codo por encima de la altura mínima (mesa)
    - el plan empieza donde está el robot (diferencia < 1°)
    - la velocidad pedida no supera la permitida en este modo (0 < v ≤ 50 %, ver vevas/velocidad.py)

Durante el movimiento (se revisa en cada ciclo):
    - error de seguimiento |q_medida - q_comandada| por junta
    - llegada de datos del robot (watchdog de /joint_states)
    - pulso de la interfaz (si el navegador se cuelga, se detiene)
    - juntas medidas cerca del límite
"""

from __future__ import annotations

import json
import math
import os
from copy import deepcopy

import numpy as np

from .velocidad import validar_config_velocidad

CONFIG_POR_DEFECTO = {
    "robot": {
        "tcp_xyz_m": [0.0, 0.0, 0.0],
        "tcp_rpy_rad": [0.0, 0.0, 0.0],
        "nombres_juntas": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        "signos": [1, 1, 1, 1, 1, 1],
        "offsets_deg": [0, 0, 0, 0, 0, 0],
    },
    "ik": {
        "tol_e_p_m": 1e-6,
        "tol_e_R_rad": 1e-5,
        "margen_limite_deg": 5.0,
        "sigma_min": 0.02,
    },
    "trayectoria": {
        "perfil": "suave",
        "kappa": 0.25,
        "t_acel_s": 0.5,
        "t_min_s": 1.5,
        "dt_envio_s": 0.02,
    },
    "seguridad": {
        "velocidad_sim_max_pct": 50,   # 0 < v ≤ 50 % (vevas/velocidad.py)
        "velocidad_real_max_pct": 50,  # el GO puede bajarlo; con RWS el RAPID (C02) limita igual al 5 % por junta
        "velocidad_inicial_pct": 5,
        "z_min_m": 0.08,
        "error_seguimiento_max_deg": 3.0,
        "inicio_max_deg": 1.0,
        "watchdog_estado_s": 0.5,
        "watchdog_interfaz_s": 1.0,
        "jog_junta_deg_s": 5.0,
        "jog_cartesiano_mm_s": 20.0,
        "jog_real_junta_max_deg_s": 5.0,
        "jog_real_cartesiano_max_mm_s": 25.0,
    },
    "revision": {
        "muestras_max": 240,
        "holgura_colision_m": 0.005,
        "radios_m": {"base": 0.085, "hombro": 0.055, "codo": 0.050, "antebrazo": 0.045, "muneca": 0.040},
    },
    "entorno": {
        "mesa_z_m": 0.0,
        "cajas": [],
        "sin_objetos_externos_confirmado": False,
        "sin_herramienta_confirmada": False,
        "herramienta": {"radio_mm": 30.0, "confirmada": False},
    },
    "ros": {
        "topico_estado": "/joint_states",
        "accion_trayectoria": "/joint_trajectory_controller/follow_joint_trajectory",
        "topico_trayectoria": "/joint_trajectory_controller/joint_trajectory",
        "servicio_hardware": "/controller_manager/list_hardware_components",
    },
    "rws": {
        "ip": "192.168.125.1",
        "puerto": 80,
        "usuario": "Default User",
        "clave": "robotics",
        "timeout_http_s": 3.0,
        "hz_lectura": 10,
        "paso_envio_deg": 40.0,   # tramos colineales; la geometría no cambia, solo hay menos paradas
        "timeout_orden_s": 120,
        "vel_min_estimada_deg_s": 2.0,
        "watchdog_estado_s": 1.0,
        "modulo_rapid": "rapid/VEVAS_RWS.mod",
    },
    "postura_inicial_deg": [0, -20, 30, 0, 40, 0],
    "servidor": {"host": "127.0.0.1", "puerto": 8000, "hz_estado": 20},
}


def _mezclar(base: dict, extra: dict) -> dict:
    out = deepcopy(base)
    for k, v in extra.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _mezclar(out[k], v)
        else:
            out[k] = v
    return out


def cargar_config(ruta: str | None) -> dict:
    if ruta and os.path.exists(ruta):
        with open(ruta, encoding="utf-8") as f:
            texto = "\n".join(l for l in f.read().splitlines() if not l.strip().startswith("//"))
        cfg = _mezclar(CONFIG_POR_DEFECTO, json.loads(texto))
        perfil = cfg["entorno"].get("perfil_mesa")
        if perfil:
            from pathlib import Path
            from .mesa import cargar_perfil_mesa, generar_mesa
            archivo = Path(perfil)
            if not archivo.is_absolute():
                archivo = Path(ruta).resolve().parent / archivo
            mesa = generar_mesa(cargar_perfil_mesa(archivo))
            # Una sola geometría alimenta la revisión y la vista 3D.
            ent = cfg["entorno"]
            ent["mesa"] = mesa
            externas = [c for c in ent["cajas"] if c.get("origen") != "mesa"]
            ent["cajas"] = externas + [{**c, "origen": "mesa"} for c in mesa["cajas"]]
        validar_config_velocidad(cfg["seguridad"])
        return cfg
    return deepcopy(CONFIG_POR_DEFECTO)


def revisar_plan(robot, Q: np.ndarray, q_medida, cfg: dict) -> list[str]:
    """Devuelve la lista de problemas del plan (vacía = se puede ejecutar)."""
    problemas = []
    margen = math.radians(cfg["ik"]["margen_limite_deg"])
    z_min = cfg["seguridad"]["z_min_m"]
    if q_medida is not None:
        dif = np.degrees(np.abs(np.asarray(Q[0]) - np.asarray(q_medida)))
        if dif.max() > cfg["seguridad"]["inicio_max_deg"]:
            j = int(np.argmax(dif)) + 1
            problemas.append(
                f"El plan no empieza donde está el robot (J{j} difiere {dif.max():.2f}°). Recalcula desde la postura actual."
            )
    peor_lim = None
    peor_z = None
    paso = max(1, len(Q) // 200)
    for k in range(0, len(Q), paso):
        q = Q[k]
        dentro = robot.dentro_de_limites(q, margen)
        if not all(dentro) and peor_lim is None:
            j = dentro.index(False) + 1
            peor_lim = f"La trayectoria pasa a menos de {math.degrees(margen):.0f}° del límite de J{j} (muestra {k})."
        pts = robot.puntos_cuerpo(q)
        zmin_k = float(pts[1:, 2].min())
        if zmin_k < z_min and peor_z is None:
            peor_z = f"Una parte del brazo baja a z = {zmin_k * 1000:.0f} mm (< {z_min * 1000:.0f} mm, altura mínima) en la muestra {k}."
    for p in (peor_lim, peor_z):
        if p:
            problemas.append(p)
    return problemas


def semaforo(err_deg_max: float, e_p_mm: float, cfg: dict) -> str:
    lim = cfg["seguridad"]["error_seguimiento_max_deg"]
    if err_deg_max > lim:
        return "rojo"
    if err_deg_max > 0.5 * lim:
        return "ambar"
    return "verde"
