#!/usr/bin/env python3
"""Validación offline (nivel S0) de FK, IK y Jacobiano con el protocolo de W12.

Referencia independiente: la cadena del URDF de ROS-Industrial (abb_irb120_support),
que NO usa D-H sino orígenes y ejes de cada junta. Si las dos FK coinciden, entonces
la tabla D-H del grupo y el ángulo de cada junta en ROS (/joint_states) son la misma
convención (signo +1, offset 0) y la comparación con el robot real es directa.

Casos (los nombres son los del protocolo):
    FK-N1   postura nominal (la inicial segura)
    FK-N2   otra postura
    FK-L    cerca de un límite documentado (J3 a 2° de +70°)
    FK-D-TCP  comparación MAL hecha a propósito: brida contra un TCP de 120 mm
    FK-D-ORD  comparación MAL hecha a propósito: orden de juntas 4 y 6 intercambiado
    IK-1, IK-2  dos poses objetivo, con todas sus ramas
    IK-S    pose cerca de la singularidad de muñeca (q5 = 0.5°)
    J-N     Jacobiano nominal contra diferencias finitas centrales, curva error vs h
    SING    barrido de q5 hacia 0: sigma_min y velocidad articular para 0.1 m/s

Uso:
    python3 02_herramientas/validacion_offline.py            (salida en 01_docs/01_datos/01_validacion_offline/)
"""

from __future__ import annotations

import csv
import datetime as dt
import hashlib
import math
import os
import sys

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
sys.path.insert(0, RAIZ)

from vevas.jacobiano import analisis_singularidad, curva_error_vs_h, errores_jacobiano, jacobiano  # noqa: E402
from vevas.pieper import elegir_rama, resolver  # noqa: E402
from vevas.robot_irb120 import (  # noqa: E402
    Robot,
    error_orientacion,
    error_posicion,
    homogenea,
    pose_desde_xyz_euler,
    rot_x,
    rot_y,
    rot_z,
)

# Cadena del URDF de ROS-Industrial para el IRB 120 (irb120_3_58): origen y eje de cada junta
URDF_ROSI = [((0, 0, 0), "z"), ((0, 0, 0.29), "y"), ((0, 0, 0.27), "y"), ((0, 0, 0.07), "x"), ((0.302, 0, 0), "y"), ((0.072, 0, 0), "x")]
T_LINK6_TOOL0 = homogenea(rot_y(math.pi / 2), [0, 0, 0])
TOL_P, TOL_R = 1e-6, 1e-5


def fk_referencia(q, tcp=np.eye(4)):
    T = np.eye(4)
    for (o, eje), qi in zip(URDF_ROSI, q):
        R = {"x": rot_x, "y": rot_y, "z": rot_z}[eje](qi)
        T = T @ homogenea(np.eye(3), o) @ homogenea(R, [0, 0, 0])
    return T @ T_LINK6_TOOL0 @ tcp


def main():
    salida = os.path.join(RAIZ, "01_docs", "01_datos", "01_validacion_offline")
    os.makedirs(salida, exist_ok=True)
    robot = Robot()
    commit = os.popen(f"git -C {RAIZ} rev-parse --short HEAD 2>/dev/null").read().strip() or "sin_git"
    with open(os.path.join(RAIZ, "07_vevas", "robot_irb120.py"), "rb") as f:
        hash_modelo = hashlib.sha256(f.read()).hexdigest()[:12]
    filas_fk = []

    def caso_fk(nombre, q_deg, propio, referencia, nota):
        q = np.radians(q_deg)
        Tp, Tr = propio(q), referencia(q)
        e_p, e_R = error_posicion(Tp, Tr), error_orientacion(Tp[:3, :3], Tr[:3, :3])
        ok = e_p <= TOL_P and e_R <= TOL_R
        filas_fk.append([nombre, " ".join(f"{v:.3f}" for v in q_deg), "base", "tool0", f"vevas {hash_modelo}", "URDF ROS-I irb120_3_58", f"{e_p:.3e}", f"{e_R:.3e}", "PASS" if ok else "FAIL", nota])
        return e_p, e_R

    nominal = [0, -20, 30, 0, 40, 0]
    caso_fk("FK-N1", nominal, robot.fk_tool0, fk_referencia, "postura inicial segura")
    caso_fk("FK-N2", [35.5, 12.3, -27.8, 61.2, -44.4, 123.9], robot.fk_tool0, fk_referencia, "postura con números feos")
    caso_fk("FK-L", [-160.0, -105.0, 68.0, 155.0, -115.0, 390.0], robot.fk_tool0, fk_referencia, "todas las juntas a 5° o menos de su límite")
    tcp120 = homogenea(np.eye(3), [0, 0, 0.120])
    caso_fk("FK-D-TCP", nominal, robot.fk_tool0, lambda q: fk_referencia(q, tcp120), "MAL A PROPÓSITO: brida vs TCP de 120 mm -> error = 120 mm")
    caso_fk("FK-D-ORD", nominal[:3] + [10, 40, 50], robot.fk_tool0, lambda q: fk_referencia(np.array([q[0], q[1], q[2], q[5], q[4], q[3]])), "MAL A PROPÓSITO: J4 y J6 intercambiadas")
    rng = np.random.default_rng(7)
    peor = 0.0
    for _ in range(5000):
        q = np.array([rng.uniform(lo, hi) for lo, hi in robot.limites])
        T1, T2 = robot.fk_tool0(q), fk_referencia(q)
        peor = max(peor, error_posicion(T1, T2), error_orientacion(T1[:3, :3], T2[:3, :3]))
    filas_fk.append(["FK-MC5000", "5000 q aleatorias dentro de límites", "base", "tool0", f"vevas {hash_modelo}", "URDF ROS-I", f"{peor:.3e}", "", "PASS" if peor < TOL_P else "FAIL", "peor caso de 5000 (posición u orientación)"])

    with open(os.path.join(salida, "casos_fk.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["case_id", "q_deg", "base_frame", "tcp_frame", "model_version", "reference_version", "position_error_m", "orientation_error_rad", "acceptance_result", "notes"])
        w.writerows(filas_fk)

    # IK
    filas_ik = []
    poses = {
        "IK-1": pose_desde_xyz_euler(350, 150, 400, 0, 180, 0),
        "IK-2": robot.fk_tcp(np.radians([35.5, 12.3, -27.8, 61.2, -44.4, 123.9])),
        "IK-S": robot.fk_tcp(np.radians([10, -10, 20, 30, 0.5, 10])),
    }
    resumen_ik = {}
    for nombre, T in poses.items():
        out = resolver(robot, T, np.radians(nominal))
        mejor, razon = elegir_rama(out["ramas"], TOL_P, TOL_R, math.radians(5), 0.02)
        resumen_ik[nombre] = (sum(r.existe for r in out["ramas"]), sum(r.valida for r in out["ramas"]), mejor.indice if mejor else None)
        for r in out["ramas"]:
            if not r.existe:
                clas = "sin solución encontrada"
            elif r.singular_muneca or (r.sigma_min < 0.02):
                clas = "singular numérica"
            elif all(r.en_limites) and r.e_p <= TOL_P and r.e_R <= TOL_R:
                clas = "válida en modelo"
            else:
                clas = "no válida (límites)"
            filas_ik.append([nombre, r.indice, f"{r.hombro}/{r.codo}/{r.muneca}", " ".join(f"{math.degrees(v):.3f}" for v in r.q) if r.q else "", f"{r.e_p:.3e}" if r.existe else "", f"{r.e_R:.3e}" if r.existe else "", f"{r.sigma_min:.4f}" if r.existe else "", clas, "ELEGIDA" if mejor and r.indice == mejor.indice else ""])
    with open(os.path.join(salida, "casos_ik.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["case_id", "rama", "hombro/codo/muneca", "q_deg", "e_p_m", "e_R_rad", "sigma_min", "clasificacion", "eleccion"])
        w.writerows(filas_ik)

    # Jacobiano y curva error vs h
    qJ = np.radians([35.5, 12.3, -27.8, 61.2, -44.4, 123.9])
    eJ = errores_jacobiano(robot, qJ, 1e-6)
    curva = curva_error_vs_h(robot, qJ)
    with open(os.path.join(salida, "jacobiano_error_vs_h.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["h", "e_Jv_m_por_rad", "e_Jw_rad_por_rad"])
        w.writerows(curva)
    qd = np.radians([5, -3, 4, 10, -6, 8])
    J = jacobiano(robot, qJ)
    dt_ = 1e-6
    T1, T0 = robot.fk_tcp(qJ + qd * dt_ / 2), robot.fk_tcp(qJ - qd * dt_ / 2)
    v_num = (T1[:3, 3] - T0[:3, 3]) / dt_
    e_v = float(np.linalg.norm(J[:3] @ qd - v_num))

    # Barrido de singularidad
    barrido = []
    for q5 in [20, 10, 5, 2, 1, 0.5, 0.2, 0.1]:
        q = np.radians([10, -10, 20, 30, q5, 10])
        a = analisis_singularidad(robot, q)
        Jq = jacobiano(robot, q)
        xd = np.array([0, 0, 0, 0.0, 0.1, 0.0])  # girar la herramienta a 0.1 rad/s alrededor de y
        qd_n = np.linalg.solve(Jq, xd) if a["rango"] == 6 else np.full(6, np.nan)
        barrido.append((q5, a["sigma_min"], a["condicion"], float(np.degrees(np.linalg.norm(qd_n))), a["rango"]))
    with open(os.path.join(salida, "singularidad_barrido.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["q5_deg", "sigma_min", "condicion", "norma_qd_deg_s_para_0.1rad_s", "rango"])
        w.writerows(barrido)
    pend = np.polyfit(np.log([b[0] for b in barrido]), np.log([b[3] for b in barrido]), 1)[0]

    # Gráficas (si hay matplotlib)
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(6.4, 4))
        hs = [c[0] for c in curva]
        ax.loglog(hs, [c[1] for c in curva], "o-", label="e_Jv [m/rad]")
        ax.loglog(hs, [c[2] for c in curva], "s-", label="e_Jw [rad/rad]")
        ax.set_xlabel("paso h [rad]")
        ax.set_ylabel("||J_analítico - J_numérico||_F")
        ax.set_title("Diferencias finitas centrales: redondeo (izq.) y truncamiento O(h²) (der.)")
        ax.grid(True, which="both", alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(salida, "jacobiano_error_vs_h.png"), dpi=130)
        plt.close(fig)
        fig, ax = plt.subplots(figsize=(6.4, 4))
        ax.loglog([b[0] for b in barrido], [b[3] for b in barrido], "o-", label="||q̇|| [°/s]")
        ax.loglog([b[0] for b in barrido], [b[1] * 100 for b in barrido], "s-", label="σ_min × 100")
        ax.set_xlabel("q5 [°] (hacia la singularidad de muñeca)")
        ax.set_title(f"Cerca de q5 = 0: ||q̇|| ~ q5^{pend:.2f}")
        ax.grid(True, which="both", alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(salida, "singularidad_barrido.png"), dpi=130)
        plt.close(fig)
        graficas = True
    except Exception:
        graficas = False

    hmin = min(curva, key=lambda c: c[1])
    md = [
        "# Validación offline (S0) de FK, IK y Jacobiano",
        "",
        f"Generado: {dt.datetime.now().isoformat(timespec='seconds')} · commit {commit} · modelo 07_vevas/robot_irb120.py sha256 {hash_modelo}",
        "",
        "Nivel de evidencia: S0 (modelo). Demuestra coherencia matemática interna y que la tabla D-H del grupo",
        "coincide con la cadena del URDF de ROS-Industrial. No demuestra nada del robot real.",
        "",
        "Condiciones de comparabilidad: base = base del robot ABB; punto comparado = tool0 (brida) en los dos modelos;",
        "orden de juntas joint_1 ... joint_6; ángulos en rad; longitudes en m; signos +1 y offsets 0 (convención ABB).",
        "",
        "## Cinemática directa",
        "",
        "| Caso | e_p [m] | e_R [rad] | Resultado | Nota |",
        "|---|---|---|---|---|",
    ]
    for r in filas_fk:
        md.append(f"| {r[0]} | {r[6]} | {r[7]} | {r[8]} | {r[9]} |")
    md += [
        "",
        "Los dos casos FK-D salen FAIL a propósito: muestran que el protocolo detecta una comparación mal armada",
        "(brida contra TCP, u orden de juntas cambiado). Con los marcos alineados el error es del orden de 1e-16.",
        "",
        "## Cinemática inversa (todas las ramas)",
        "",
        "| Caso | Ramas que existen | Ramas válidas | Rama elegida |",
        "|---|---|---|---|",
    ]
    for k, (ex, va, el) in resumen_ik.items():
        md.append(f"| {k} | {ex} | {va} | {el} |")
    md += [
        "",
        "Detalle por rama en casos_ik.csv. Dos ramas distintas pueden ser válidas para la misma pose: no se exige q_propia = q_ref.",
        "",
        "## Jacobiano",
        "",
        f"Jacobiano geométrico contra diferencias finitas centrales con h = 1e-6: e_Jv = {eJ['e_Jv']:.2e} m/rad, e_Jw = {eJ['e_Jw']:.2e} rad/rad.",
        f"Mejor paso de la curva: h = {hmin[0]:.0e} con e_Jv = {hmin[1]:.2e}. A la derecha domina el truncamiento O(h²), a la izquierda el redondeo.",
        f"Verificación de velocidad: ||J_v q̇ - dp/dt|| = {e_v:.2e} m/s.",
        "",
        "## Singularidad de muñeca",
        "",
        "| q5 [°] | σ_min | condición | ||q̇|| [°/s] para girar 0.1 rad/s | rango |",
        "|---|---|---|---|---|",
    ]
    for b in barrido:
        md.append(f"| {b[0]} | {b[1]:.4f} | {b[2]:.1f} | {b[3]:.1f} | {b[4]} |")
    md += [
        "",
        f"Ajuste log-log: ||q̇|| crece como q5^{pend:.2f} (esperado -1): al acercarse a q5 = 0 las juntas tendrían que girar cada vez más rápido.",
        "Pérdida de rango no es torque infinito: τ_ext = Jᵀ F (estática) no es el torque del actuador M q̈ + C q̇ + g.",
    ]
    if graficas:
        md += ["", "![error vs h](jacobiano_error_vs_h.png)", "", "![singularidad](singularidad_barrido.png)"]
    with open(os.path.join(salida, "01_resumen.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    print("\n".join(md))
    print(f"\nArchivos en {salida}")


if __name__ == "__main__":
    main()
