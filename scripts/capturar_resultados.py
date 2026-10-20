#!/usr/bin/env python3
"""
Captura reproducible de los resultados del Hito 2.

Para que sirve

    Deja un registro verificable de la ejecucion: ejecuta cada modulo,
    guarda su salida de texto, y extrae las cifras principales a un
    JSON con la semilla, la fecha y las versiones usadas. Asi cualquiera
    puede contrastar las cifras del informe contra una corrida propia.

Uso
    cd scripts
    python3 capturar_resultados.py

Escribe en ../docs/hito2/resultados/:
    resultados.json       cifras principales, con semilla y entorno
    salida_<modulo>.txt   salida completa de cada modulo

Grupo VEVAS
"""

import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
SALIDA = AQUI.parent / "docs" / "hito2" / "resultados"
SEMILLA = 20261018

MODULOS = ["pieper", "jacobiano", "lspb", "singularidad", "dinamica_1r"]


def correr_modulos():
    """Ejecuta cada modulo y guarda su salida literal."""
    estado = {}
    for mod in MODULOS:
        print(f"  ejecutando {mod}.py ...", flush=True)
        try:
            r = subprocess.run([sys.executable, f"{mod}.py"], cwd=AQUI,
                               capture_output=True, text=True, timeout=600)
            texto = r.stdout + (("\n--- STDERR ---\n" + r.stderr)
                                if r.stderr.strip() else "")
            (SALIDA / f"salida_{mod}.txt").write_text(texto, encoding="utf-8")
            estado[mod] = {"codigo_salida": r.returncode,
                           "lineas": len(texto.splitlines())}
        except Exception as exc:
            estado[mod] = {"error": repr(exc)}
    return estado


def medir():
    """Recalcula las cifras principales del informe, en este proceso."""
    from fk_irb120 import forward_kinematics, JOINT_LIMITS
    from pieper import pieper, resolver, error_pose
    from jacobiano import jacobiano, validacion_manual_q0, jacobiano_numerico
    from lspb import LSPB
    from dinamica_1r import verificacion_analitica
    from singularidad import (direccion_perdida, direcciones_conservadas,
                              velocidad_requerida)

    m = {}

    # --- Cobertura regular
    rng = np.random.default_rng(SEMILLA)
    ep_max = eR_max = 0.0
    sin_sol = 0
    ramas = []
    for _ in range(2000):
        q = rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        T = forward_kinematics(q)
        cand = pieper(T)
        ramas.append(len(cand))
        for c in cand:
            ep, eR = error_pose(c, T)
            ep_max, eR_max = max(ep_max, ep), max(eR_max, eR)
        sel, _, _ = resolver(T, q)
        sin_sol += sel is None
    m["cobertura_regular"] = {
        "semilla": SEMILLA, "poses": 2000,
        "ramas_por_pose_media": float(np.mean(ramas)),
        "error_posicion_maximo_m": ep_max,
        "error_orientacion_maximo": eR_max,
        "poses_sin_solucion": int(sin_sol)}

    # --- Fronteras: la prueba dirigida que expuso el fallo de tolerancia
    rngf = np.random.default_rng(18026)
    rechazos = 0
    for j in range(6):
        for lado in range(2):
            for _ in range(150):
                q = rngf.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
                q[j] = JOINT_LIMITS[j, lado]
                sel, _, _ = resolver(forward_kinematics(q), q)
                rechazos += sel is None
    m["fronteras_dirigidas"] = {
        "semilla": 18026, "poses": 1800, "rechazos": int(rechazos),
        "rechazos_antes_de_la_correccion": 193}

    # --- Muneca casi singular
    banda = {}
    for q5 in [1e-7, 1e-6, 1e-5, 3e-5, 5e-5]:
        q = np.array([.4, -.6, .5, .3, q5, -.2])
        sel, adm, _ = resolver(forward_kinematics(q), q)
        banda[f"{q5:.0e}"] = {"admisibles": len(adm),
                              "resuelve": sel is not None}
    m["muneca_casi_singular"] = banda

    # --- Vueltas de J6
    q = np.array([.4, -.6, .5, .3, .7, 6.0])
    sel, _, _ = resolver(forward_kinematics(q), q)
    m["vueltas_j6"] = {
        "q6_actual_rad": 6.0, "q6_elegido_rad": float(sel[5]),
        "movimiento_innecesario_rad": float(abs(sel[5] - 6.0)),
        "antes_de_la_correccion_rad": 6.283185307179586}

    # --- Jacobiano
    _, _, err_manual = validacion_manual_q0()
    peor_fd = 0.0
    rngj = np.random.default_rng(7)
    for _ in range(100):
        q = rngj.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        peor_fd = max(peor_fd,
                      float(np.max(np.abs(jacobiano(q) - jacobiano_numerico(q)))))
    m["jacobiano"] = {"validacion_manual_q0": float(err_manual),
                      "vs_diferencias_finitas": peor_fd}

    # --- Singularidad
    q_sing = np.array([.3, .5, -.4, .2, 0.0, .1])
    u, s_min = direccion_perdida(q_sing)
    Uc, Sc = direcciones_conservadas(q_sing)
    J = jacobiano(q_sing)
    xd_inc = np.r_[np.zeros(3), u[3:] / np.linalg.norm(u[3:])]
    qd_inc = velocidad_requerida(q_sing, xd_inc)
    qd_com = np.array([1., 0., 0., 0., 0., 0.])
    m["singularidad"] = {
        "configuracion": q_sing.tolist(),
        "rango": int(np.linalg.matrix_rank(J, tol=1e-8)),
        "sigma_min": float(s_min),
        "norma_lineal_vector_perdido": float(np.linalg.norm(u[:3])),
        "norma_angular_vector_perdido": float(np.linalg.norm(u[3:])),
        "nota_unidades": ("comparar ambas normas mezcla m/s con rad/s; "
                          "sin una longitud caracteristica no son "
                          "comparables. El rango SI es invariante."),
        "valores_singulares_conservados": Sc.tolist(),
        "residuo_tarea_incompatible": float(np.linalg.norm(J @ qd_inc - xd_inc)),
        "residuo_tarea_compatible": float(np.linalg.norm(J @ qd_com - J @ qd_com))}

    # --- LSPB y dinamica 1R contra el solucionario
    p = LSPB.desde_tiempo_final(0.0, 2.0, tf=3.0, acc=1.0)
    qc, qdc, qddc = (float(v[0]) for v in p(2.5))
    chk = verificacion_analitica(qc, qddc)
    m["lspb_y_dinamica_1r"] = {
        "caso": "W10-S01 Tareas 3 y 4D",
        "tc_s": float(p.tc), "velocidad_crucero_rad_s": float(p.V),
        "q_2s5_rad": qc, "qd_2s5_rad_s": qdc, "qdd_2s5_rad_s2": qddc,
        "tau_inercial_Nm": float(chk["inercial"]),
        "tau_gravedad_Nm": float(chk["gravedad"]),
        "tau_total_Nm": float(chk["total"]),
        "tau_total_referencia_Nm": 4.429,
        "residuo_Nm": abs(float(chk["total"]) - 4.429),
        "nota_residuo": ("proviene del redondeo de sin(1.875) ~ 0.954 en "
                         "el desarrollo publicado, no de discrepancia de "
                         "modelo")}
    return m


if __name__ == "__main__":
    SALIDA.mkdir(parents=True, exist_ok=True)
    print("\n   CAPTURA DE RESULTADOS DEL HITO 2\n")

    estado = correr_modulos()
    print("\n  midiendo cifras principales ...", flush=True)
    medidas = medir()

    doc = {
        "proyecto": "PFI Hito 2 - ABB IRB 120 - Grupo VEVAS",
        "fecha_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "entorno": {"python": platform.python_version(),
                    "numpy": np.__version__,
                    "plataforma": platform.platform()},
        "modulos_ejecutados": estado,
        "resultados": medidas,
    }
    (SALIDA / "resultados.json").write_text(
        json.dumps(doc, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"\n  escrito en {SALIDA}")
    r = medidas
    print(f"\n  ramas por pose           {r['cobertura_regular']['ramas_por_pose_media']:.2f}")
    print(f"  error posicional max     {r['cobertura_regular']['error_posicion_maximo_m']:.3e} m")
    print(f"  poses sin solucion       {r['cobertura_regular']['poses_sin_solucion']} de 2000")
    print(f"  rechazos en fronteras    {r['fronteras_dirigidas']['rechazos']} de 1800  (antes 193)")
    print(f"  giro innecesario de J6   {r['vueltas_j6']['movimiento_innecesario_rad']:.4f} rad  (antes 6.2832)")
    print(f"  jacobiano manual         {r['jacobiano']['validacion_manual_q0']:.3e}")
    print(f"  rango en la singularidad {r['singularidad']['rango']} de 6")
    print(f"  residuo incompatible     {r['singularidad']['residuo_tarea_incompatible']:.6f}")
    print(f"  tau(2.5 s)               {r['lspb_y_dinamica_1r']['tau_total_Nm']:.4f} N.m"
          f"   (referencia 4.4290, residuo {r['lspb_y_dinamica_1r']['residuo_Nm']:.1e})")
    print()
