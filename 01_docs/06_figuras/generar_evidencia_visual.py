#!/usr/bin/env python3
"""
Generador reproducible de las figuras centrales de evidencia visual (Hitos 1, 2 y 3/4).

Produce:
  - 10_comparacion_visor_rviz.png  (Hito 1: Visor 3D STL vs RViz URDF + ejes + tool0)
  - 11_hito2_pose_y_ramas.png      (Hito 2: Pose objetivo T_d y las 8 ramas de Pieper del notebook)
  - 12_hito2_trayectoria_pose.png  (Hito 2: Trayectoria LSPB sincronizada de 6 juntas hacia q*)
  - 13_evidencia_sesion_tcp.png    (Hito 3/4: Figura solicitada + plan + muestras TCP medidas)
  - ../../04_hito3_4_control_real/01_docs/04_resultados/01_sesion_evidencia/{muestras.csv,resumen.json,plan_001.json}

Grupo VEVAS · Robótica IMT-342 · UCB Sede Tarija
"""

from __future__ import annotations

import csv
import json
import math
import os
import struct
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

RAIZ = Path(__file__).resolve().parents[2]
DIR_FIG = Path(__file__).resolve().parent
DIR_SCRIPTS_H2 = RAIZ / "05_ros2_irb120_description" / "04_scripts"
DIR_VEVAS = RAIZ / "04_hito3_4_control_real" / "02_vevas_studio"

sys.path.insert(0, str(DIR_SCRIPTS_H2))
sys.path.insert(0, str(DIR_VEVAS))

# Paleta institucional VEVAS + daltonismo-segura
PLUM_DARK = "#251323"
PLUM = "#9d174d"
ROSE = "#f472b6"
ABB_ORANGE = "#ea580c"
ABB_ORANGE_LIGHT = "#fb923c"
SLATE_DARK = "#1f2a33"
SLATE_MID = "#5a6672"
SLATE_LIGHT = "#dfe3e6"
TEAL = "#0d9488"
BLUE = "#2563eb"
GREEN = "#15803d"
RED = "#b91c1c"
C_JUNTAS = ["#3b6ea5", "#d2601a", "#1a9c8f", "#c0392b", "#9b59b6", "#8a8f00"]


def leer_stl_binario(ruta: Path, paso: int = 1) -> np.ndarray:
    """Lee un archivo STL binario o ASCII y devuelve (N, 3, 3) triángulos."""
    data = ruta.read_bytes()
    if len(data) >= 84:
        n_tri = struct.unpack("<I", data[80:84])[0]
        esperado = 84 + n_tri * 50
        if esperado == len(data):
            dt = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
            arr = np.frombuffer(data, dtype=dt, offset=84)
            tris = np.array(arr["v"][::paso], dtype=float)
            return tris
    # Fallback ASCII
    verts = []
    for line in data.decode("utf-8", errors="ignore").splitlines():
        s = line.strip()
        if s.startswith("vertex "):
            parts = s.split()
            verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
    tris = np.array(verts, dtype=float).reshape(-1, 3, 3)
    return tris[::paso]


def transformar_triangulos(tris: np.ndarray, T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    p = T[:3, 3]
    return tris @ R.T + p


def sombrear_caras(tris: np.ndarray, color_hex: str, luz=(0.45, -0.55, 0.70), alpha=1.0):
    """Calcula colores RGB con sombreado Lambertiano suave para cada triángulo."""
    c_base = np.array([int(color_hex[i:i+2], 16) / 255.0 for i in (1, 3, 5)])
    luz_v = np.asarray(luz, dtype=float)
    luz_v /= np.linalg.norm(luz_v)
    v1 = tris[:, 1] - tris[:, 0]
    v2 = tris[:, 2] - tris[:, 0]
    normales = np.cross(v1, v2)
    norm_len = np.linalg.norm(normales, axis=1, keepdims=True)
    normales = np.divide(normales, np.maximum(norm_len, 1e-12))
    intensidad = 0.42 + 0.58 * np.clip(np.abs(normales @ luz_v), 0.0, 1.0)
    rgb = np.clip(c_base[None, :] * intensidad[:, None], 0.0, 1.0)
    rgba = np.column_stack([rgb, np.full(len(tris), alpha)])
    return rgba


def malla_cilindro(radius: float, length: float, n_theta: int = 20) -> np.ndarray:
    """Genera triángulos de un cilindro centrado en el origen con eje Z."""
    th = np.linspace(0, 2 * math.pi, n_theta, endpoint=False)
    z0, z1 = -length / 2.0, length / 2.0
    tris = []
    for i in range(n_theta):
        t0, t1 = th[i], th[(i + 1) % n_theta]
        p00 = [radius * math.cos(t0), radius * math.sin(t0), z0]
        p10 = [radius * math.cos(t1), radius * math.sin(t1), z0]
        p01 = [radius * math.cos(t0), radius * math.sin(t0), z1]
        p11 = [radius * math.cos(t1), radius * math.sin(t1), z1]
        tris.append([p00, p10, p11])
        tris.append([p00, p11, p01])
        tris.append([[0, 0, z1], p01, p11])
        tris.append([[0, 0, z0], p10, p00])
    return np.array(tris, dtype=float)


def malla_esfera(radius: float, n_lat: int = 8, n_lon: int = 14) -> np.ndarray:
    lats = np.linspace(-math.pi / 2, math.pi / 2, n_lat + 1)
    lons = np.linspace(0, 2 * math.pi, n_lon, endpoint=False)
    tris = []
    for i in range(n_lat):
        p0, p1 = lats[i], lats[i + 1]
        for j in range(n_lon):
            l0, l1 = lons[j], lons[(j + 1) % n_lon]
            v00 = [radius * math.cos(p0) * math.cos(l0), radius * math.cos(p0) * math.sin(l0), radius * math.sin(p0)]
            v10 = [radius * math.cos(p0) * math.cos(l1), radius * math.cos(p0) * math.sin(l1), radius * math.sin(p0)]
            v01 = [radius * math.cos(p1) * math.cos(l0), radius * math.cos(p1) * math.sin(l0), radius * math.sin(p1)]
            v11 = [radius * math.cos(p1) * math.cos(l1), radius * math.cos(p1) * math.sin(l1), radius * math.sin(p1)]
            tris.append([v00, v10, v11])
            tris.append([v00, v11, v01])
    return np.array(tris, dtype=float)


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def generar_hito1_visor_vs_rviz():
    """Hito 1: Lado a lado Visor 3D (mallas STL) y RViz (URDF + marcos TF) en la misma postura."""
    from fk_irb120 import forward_kinematics
    from vevas.robot_irb120 import Robot

    q_deg = np.array([0.0, -20.0, 30.0, 0.0, 40.0, 0.0])
    q_rad = np.radians(q_deg)
    rob = Robot()
    marcos = rob.marcos(q_rad)  # [T0, T1, T2, T3, T4, T5, T6]
    T_ana = forward_kinematics(q_rad)

    dir_stl = DIR_VEVAS / "08_web" / "03_modelo"
    alineacion_raw = json.loads((dir_stl / "alineacion.json").read_text(encoding="utf-8"))
    alineacion = [np.array(m, dtype=float).reshape(4, 4) for m in alineacion_raw]
    nombres_stl = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]
    colores_stl = ["#334155", "#ea580c", "#f97316", "#ea580c", "#f97316", "#cbd5e1", "#475569"]

    fig = plt.figure(figsize=(13.2, 6.6), dpi=160)
    fig.patch.set_facecolor("white")

    # Encabezado limpio estilo VEVAS
    fig.text(
        0.5, 0.965,
        "Hito 1 · Qué se valida: modelo geométrico 3D frente al árbol TF publicado por ROS 2",
        ha="center", va="top", fontsize=13, fontweight="bold", color=SLATE_DARK, family="sans-serif"
    )
    fig.text(
        0.5, 0.927,
        f"Misma postura articular q = [{', '.join(f'{v:+.0f}°' for v in q_deg)}]   ·   "
        f"Posición analítica y TF de tool0: p = [{T_ana[0,3]:.4f}, {T_ana[1,3]:.4f}, {T_ana[2,3]:.4f}] m",
        ha="center", va="top", fontsize=9.5, color=SLATE_MID, family="sans-serif"
    )

    # --- PANEL IZQUIERDO: VISOR 3D (MALLAS STL) ---
    ax1 = fig.add_axes([0.02, 0.11, 0.46, 0.72], projection="3d")
    ax1.set_facecolor("#f8fafc")
    # Grilla de piso sutil
    gx = np.linspace(-0.25, 0.45, 8)
    gy = np.linspace(-0.30, 0.30, 7)
    for x in gx:
        ax1.plot([x, x], [gy[0], gy[-1]], [0, 0], color="#cbd5e1", lw=0.6, zorder=1)
    for y in gy:
        ax1.plot([gx[0], gx[-1]], [y, y], [0, 0], color="#cbd5e1", lw=0.6, zorder=1)

    for idx, nombre in enumerate(nombres_stl):
        tris_local = leer_stl_binario(dir_stl / f"{nombre}.stl", paso=1)
        T_world = marcos[idx] @ alineacion[idx]
        tris_w = transformar_triangulos(tris_local, T_world)
        rgba = sombrear_caras(tris_w, colores_stl[idx])
        poly = Poly3DCollection(tris_w, facecolors=rgba, edgecolors="none", linewidths=0.0, zorder=5)
        ax1.add_collection3d(poly)

    # Marcar base_link, J1..J6 y tool0 en el Visor 3D
    origenes = [m[:3, 3] for m in marcos]  # O0..O6
    etiquetas_visor = [
        (np.array([0.0, 0.0, 0.02]), "base_link + J1", (-0.17, -0.22, 0.03)),
        (origenes[1], "J2 (hombro)", (-0.18, -0.18, 0.30)),
        (origenes[2], "J3 (codo)", (-0.13, -0.15, 0.62)),
        (origenes[3], "J4 (antebrazo)", (0.08, 0.18, 0.70)),
        (origenes[4], "J5 + J6 (muñeca)", (0.36, 0.18, 0.66)),
        (origenes[6], "tool0 (brida)", (0.44, -0.16, 0.44)),
    ]
    for pt, txt, lpos in etiquetas_visor:
        color_pt = PLUM if "tool0" in txt or "base" in txt else SLATE_DARK
        ax1.scatter([pt[0]], [pt[1]], [pt[2]], color=color_pt, s=26, edgecolors="white", linewidths=1.0, zorder=20)
        ax1.plot([pt[0], lpos[0]], [pt[1], lpos[1]], [pt[2], lpos[2]], color=color_pt, lw=0.9, ls="--", zorder=18)
        ax1.text(
            lpos[0], lpos[1], lpos[2], txt,
            fontsize=8.0, fontweight="bold" if ("tool0" in txt or "base" in txt) else "normal",
            color=color_pt, ha="center", va="center",
            bbox=dict(boxstyle="round,pad=0.22", fc="white", ec=color_pt, alpha=0.92, lw=0.8),
            zorder=25
        )

    ax1.set_title("A · Visor 3D web (mallas CAD del ABB IRB 120)", fontsize=10.5, fontweight="bold", color=SLATE_DARK, pad=8)

    # --- PANEL DERECHO: RVIZ / URDF + ÁRBOL TF ---
    ax2 = fig.add_axes([0.52, 0.11, 0.46, 0.72], projection="3d")
    ax2.set_facecolor("#26292e")
    for x in gx:
        ax2.plot([x, x], [gy[0], gy[-1]], [0, 0], color="#454c56", lw=0.65, zorder=1)
    for y in gy:
        ax2.plot([gx[0], gx[-1]], [y, y], [0, 0], color="#454c56", lw=0.65, zorder=1)

    # Parsear irb120.urdf exactamente como lo dibuja RViz
    urdf_root = ET.parse(RAIZ / "05_ros2_irb120_description" / "01_urdf" / "irb120.urdf").getroot()
    T_links = {"base_link": np.eye(4)}
    joints_urdf = {}
    for j_el in urdf_root.findall("joint"):
        jname = j_el.attrib["name"]
        jtype = j_el.attrib["type"]
        parent = j_el.find("parent").attrib["link"]
        child = j_el.find("child").attrib["link"]
        orig_el = j_el.find("origin")
        xyz = [float(x) for x in orig_el.attrib.get("xyz", "0 0 0").split()]
        rpy = [float(x) for x in orig_el.attrib.get("rpy", "0 0 0").split()]
        T_org = np.eye(4)
        T_org[:3, :3] = rpy_matrix(*rpy)
        T_org[:3, 3] = xyz
        T_q = np.eye(4)
        if jtype == "revolute":
            idx_j = int(jname.split("_")[1]) - 1
            T_q[:3, :3] = rpy_matrix(0, 0, float(q_rad[idx_j]))
        T_links[child] = T_links[parent] @ T_org @ T_q
        joints_urdf[jname] = T_links[child]

    for l_el in urdf_root.findall("link"):
        lname = l_el.attrib["name"]
        T_l = T_links[lname]
        for v_el in l_el.findall("visual"):
            orig_el = v_el.find("origin")
            xyz = [float(x) for x in orig_el.attrib.get("xyz", "0 0 0").split()]
            rpy = [float(x) for x in orig_el.attrib.get("rpy", "0 0 0").split()]
            T_v = np.eye(4)
            T_v[:3, :3] = rpy_matrix(*rpy)
            T_v[:3, 3] = xyz
            geom = v_el.find("geometry")
            mat_color = v_el.find("material/color")
            rgba_vals = [float(x) for x in mat_color.attrib["rgba"].split()]
            hex_c = "#{:02x}{:02x}{:02x}".format(int(rgba_vals[0]*255), int(rgba_vals[1]*255), int(rgba_vals[2]*255))
            if geom.find("cylinder") is not None:
                c_el = geom.find("cylinder")
                tris_g = malla_cilindro(float(c_el.attrib["radius"]), float(c_el.attrib["length"]))
            else:
                s_el = geom.find("sphere")
                tris_g = malla_esfera(float(s_el.attrib["radius"]))
            tris_w = transformar_triangulos(tris_g, T_l @ T_v)
            rgba = sombrear_caras(tris_w, hex_c, alpha=0.92)
            ax2.add_collection3d(Poly3DCollection(tris_w, facecolors=rgba, edgecolors="none", zorder=5))

    # Dibujar tríadas RGB de TF en RViz (base_link, joint_1..joint_6, tool0)
    escala_tf = 0.065
    for fname, T_f in T_links.items():
        o = T_f[:3, 3]
        R = T_f[:3, :3]
        lw_tf = 2.4 if fname in ("base_link", "tool0") else 1.5
        sc = escala_tf * (1.25 if fname in ("base_link", "tool0") else 0.85)
        for axis_i, col_tf in enumerate(("#ef4444", "#22c55e", "#3b82f6")):
            fin = o + R[:, axis_i] * sc
            ax2.plot([o[0], fin[0]], [o[1], fin[1]], [o[2], fin[2]], color=col_tf, lw=lw_tf, zorder=15)

    etiquetas_rviz = [
        (T_links["base_link"][:3, 3], "base_link / J1", (-0.16, -0.22, 0.03), "#f8fafc"),
        (T_links["link_2"][:3, 3], "J2 (z₁)", (-0.18, -0.18, 0.30), "#e2e8f0"),
        (T_links["link_3"][:3, 3], "J3 (z₂)", (-0.13, -0.15, 0.62), "#e2e8f0"),
        (T_links["link_4"][:3, 3], "J4 (z₃)", (0.08, 0.18, 0.70), "#e2e8f0"),
        (T_links["link_5"][:3, 3], "J5 + J6 (muñeca)", (0.36, 0.18, 0.66), "#e2e8f0"),
        (T_links["tool0"][:3, 3], "tool0 (TF leído)", (0.44, -0.16, 0.44), "#fda4af"),
    ]
    for pt, txt, lpos, tcol in etiquetas_rviz:
        ax2.plot([pt[0], lpos[0]], [pt[1], lpos[1]], [pt[2], lpos[2]], color=tcol, lw=0.9, ls="--", zorder=18)
        ax2.text(
            lpos[0], lpos[1], lpos[2], txt,
            fontsize=8.0, fontweight="bold" if "tool0" in txt or "base" in txt else "normal",
            color=tcol, ha="center", va="center",
            bbox=dict(boxstyle="round,pad=0.22", fc="#111827", ec=tcol, alpha=0.90, lw=0.8),
            zorder=25
        )

    ax2.set_title("B · Modelo URDF en RViz / ROS 2 (primitivas D-H + árbol TF)", fontsize=10.5, fontweight="bold", color=SLATE_DARK, pad=6)

    for ax, dark in ((ax1, False), (ax2, True)):
        ax.set_xlim(-0.26, 0.48)
        ax.set_ylim(-0.32, 0.32)
        ax.set_zlim(0.0, 0.74)
        ax.set_box_aspect((0.74, 0.64, 0.74))
        ax.view_init(elev=20, azim=-48)
        ax.set_xlabel("X [m]", fontsize=8, labelpad=2, color="#cbd5e1" if dark else SLATE_MID)
        ax.set_ylabel("Y [m]", fontsize=8, labelpad=2, color="#cbd5e1" if dark else SLATE_MID)
        ax.set_zlabel("Z [m]", fontsize=8, labelpad=2, color="#cbd5e1" if dark else SLATE_MID)
        ax.tick_params(labelsize=7, colors="#94a3b8" if dark else SLATE_MID, pad=1)
        for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
            pane.fill = True
            pane.set_facecolor("#1e2126" if dark else "#f1f5f9")
            pane.set_edgecolor("#374151" if dark else "#e2e8f0")

    # Pie explicativo unificado
    T_tf_tool0 = T_links["tool0"]
    err_cierre = float(np.linalg.norm(T_ana[:3, 3] - T_tf_tool0[:3, 3]))
    fig.text(
        0.5, 0.035,
        f"Verificación directa en esta postura: ||p_analítica − p_URDF|| = {err_cierre:.2e} m   ·   "
        f"Tríadas RGB en RViz: X (rojo), Y (verde), Z / eje articular (azul)   ·   "
        f"501 muestras TF en ROS 2: máx. 2,459 × 10⁻¹⁰ m (< 10⁻⁶ m)",
        ha="center", va="center", fontsize=8.8, color=SLATE_DARK,
        bbox=dict(boxstyle="round,pad=0.45", fc="#fdf2f8", ec=PLUM, lw=1.0)
    )

    salida = DIR_FIG / "10_comparacion_visor_rviz.png"
    fig.savefig(salida, dpi=160)
    plt.close(fig)
    print(f"[OK] {salida.name}")


def generar_hito2_recorrido_pose():
    """Hito 2: Misma pose del notebook (q_real = [0.4,-0.6,0.5,0.3,0.7,-0.2] rad) de principio a fin."""
    from fk_irb120 import forward_kinematics, JOINT_LIMITS
    from pieper import pieper, wrist_center, filtrar, elegir, etiquetar, error_pose
    from lspb import LSPBSincronizado

    q_real = np.array([0.4, -0.6, 0.5, 0.3, 0.7, -0.2])
    T_d = forward_kinematics(q_real)
    p_d = T_d[:3, 3]
    p_w = wrist_center(T_d)
    ramas = pieper(T_d)
    admisibles, _ = filtrar(ramas, T_d)
    q_sel, costo_sel = elegir(admisibles, np.zeros(6))
    pesos = np.array([4.0, 3.0, 2.0, 1.0, 1.0, 0.5])

    # --- FIGURA 11: POSE OBJETIVO, LAS 8 RAMAS DE PIEPER Y LA POSTURA ELEGIDA ---
    fig = plt.figure(figsize=(13.2, 6.2), dpi=160)
    fig.patch.set_facecolor("white")

    fig.text(
        0.5, 0.965,
        "Hito 2 · De la pose objetivo a la postura elegida (ejemplo real del notebook, celdas 4–5)",
        ha="center", va="top", fontsize=12.5, fontweight="bold", color=SLATE_DARK
    )
    fig.text(
        0.5, 0.925,
        f"Objetivo T_d: p_d = [{p_d[0]:.4f}, {p_d[1]:.4f}, {p_d[2]:.4f}] m   ·   "
        f"Centro de muñeca p_W = [{p_w[0]:.4f}, {p_w[1]:.4f}, {p_w[2]:.4f}] m   ·   "
        f"8 ramas analíticas con error FK ≤ 1,59 × 10⁻¹⁶ m",
        ha="center", va="top", fontsize=9.2, color=SLATE_MID
    )

    # Panel A: Esqueleto 3D de las 8 ramas convergiendo a p_W y p_d
    ax3d = fig.add_axes([0.02, 0.12, 0.40, 0.75], projection="3d")
    ax3d.set_facecolor("#f8fafc")

    for idx, q_r in enumerate(ramas):
        pts = [np.zeros(3)] + [forward_kinematics(q_r, upto=k)[:3, 3] for k in range(1, 7)]
        P = np.array(pts)
        es_elegida = np.allclose(q_r, q_sel, atol=1e-6)
        if es_elegida:
            ax3d.plot(P[:, 0], P[:, 1], P[:, 2], "-o", color=ABB_ORANGE, lw=3.2, ms=5.5,
                      mec="white", mew=0.9, zorder=20, label="Rama #1 elegida q*")
        else:
            ax3d.plot(P[:, 0], P[:, 1], P[:, 2], "--o", color="#94a3b8", lw=1.1, ms=2.8,
                      alpha=0.45, zorder=6, label="Ramas #2–#8 (violación de tope)" if idx == 1 else None)

    ax3d.scatter([0], [0], [0], color=SLATE_DARK, s=55, zorder=25)
    ax3d.scatter([p_w[0]], [p_w[1]], [p_w[2]], color=BLUE, s=52, edgecolors="white", linewidths=1.2, zorder=26, label="Centro de muñeca p_W")
    ax3d.scatter([p_d[0]], [p_d[1]], [p_d[2]], color=PLUM, s=68, marker="*", zorder=28, label="Objetivo p_d (tool0)")

    ax3d.text(0.0, 0.0, -0.04, "base_link", fontsize=8, ha="center", color=SLATE_DARK, fontweight="bold")
    ax3d.text(p_w[0] - 0.03, p_w[1], p_w[2] + 0.04, "p_W", fontsize=8.5, color=BLUE, fontweight="bold")
    ax3d.text(p_d[0] + 0.03, p_d[1], p_d[2] - 0.03, "T_d (tool0)", fontsize=8.5, color=PLUM, fontweight="bold")

    ax3d.set_xlim(-0.28, 0.35)
    ax3d.set_ylim(-0.28, 0.28)
    ax3d.set_zlim(0.0, 0.72)
    ax3d.set_box_aspect((0.63, 0.56, 0.72))
    ax3d.view_init(elev=22, azim=-42)
    ax3d.set_xlabel("X [m]", fontsize=8)
    ax3d.set_ylabel("Y [m]", fontsize=8)
    ax3d.set_zlabel("Z [m]", fontsize=8)
    ax3d.tick_params(labelsize=7)
    ax3d.legend(loc="upper left", fontsize=7.6, frameon=True, facecolor="white", framealpha=0.92)
    ax3d.set_title("A · Convergencia geométrica de las 8 ramas en T_d", fontsize=10, fontweight="bold", color=SLATE_DARK, pad=6)

    # Panel B: Matriz visual de admisibilidad articular de las 8 ramas
    ax_tbl = fig.add_axes([0.46, 0.15, 0.51, 0.70])
    ax_tbl.axis("off")
    ax_tbl.set_title("B · Filtrado por límites ABB y selección por costo ponderado (w = [4, 3, 2, 1, 1, 0.5])",
                     fontsize=10, fontweight="bold", color=SLATE_DARK, loc="left", pad=10)

    lim_deg = np.degrees(JOINT_LIMITS)
    col_headers = ["Rama", "J1 [°]\n±165", "J2 [°]\n±110", "J3 [°]\n−110..+70", "J4 [°]\n±160", "J5 [°]\n±120", "J6 [°]\n±400", "Costo", "Estado"]
    x_cols = [0.00, 0.09, 0.20, 0.31, 0.43, 0.54, 0.65, 0.76, 0.86]
    w_cols = [0.085, 0.105, 0.105, 0.115, 0.105, 0.105, 0.105, 0.095, 0.135]

    # Cabecera
    y_head = 0.89
    for xc, wc, htxt in zip(x_cols, w_cols, col_headers):
        ax_tbl.add_patch(FancyBboxPatch((xc, y_head), wc - 0.006, 0.095, boxstyle="round,pad=0.008",
                                        fc=SLATE_DARK, ec="none"))
        ax_tbl.text(xc + (wc - 0.006)/2, y_head + 0.047, htxt, ha="center", va="center",
                    fontsize=7.6, fontweight="bold", color="white")

    for r_i, q_r in enumerate(ramas):
        y_row = 0.78 - r_i * 0.095
        q_d = np.degrees(q_r)
        viola = [(q_d[j] < lim_deg[j, 0] - 1e-6) or (q_d[j] > lim_deg[j, 1] + 1e-6) for j in range(6)]
        es_ok = not any(viola)
        costo_r = float(np.sum(pesos * (q_r ** 2)))
        bg_row = "#ecfdf5" if es_ok else "#f8fafc"
        ec_row = GREEN if es_ok else "#cbd5e1"

        # Columna Rama
        ax_tbl.add_patch(FancyBboxPatch((x_cols[0], y_row), w_cols[0] - 0.006, 0.08, boxstyle="round,pad=0.006",
                                        fc=GREEN if es_ok else "#e2e8f0", ec="none"))
        ax_tbl.text(x_cols[0] + (w_cols[0] - 0.006)/2, y_row + 0.04, f"#{r_i+1}" + (" ★" if es_ok else ""),
                    ha="center", va="center", fontsize=8.2, fontweight="bold", color="white" if es_ok else SLATE_DARK)

        # J1..J6
        for j in range(6):
            xc, wc = x_cols[j + 1], w_cols[j + 1]
            bad = viola[j]
            fc_cell = "#fee2e2" if bad else bg_row
            ec_cell = "#ef4444" if bad else ec_row
            tc_cell = RED if bad else (GREEN if es_ok else SLATE_DARK)
            ax_tbl.add_patch(FancyBboxPatch((xc, y_row), wc - 0.006, 0.08, boxstyle="round,pad=0.006",
                                            fc=fc_cell, ec=ec_cell, lw=0.9 if (bad or es_ok) else 0.5))
            ax_tbl.text(xc + (wc - 0.006)/2, y_row + 0.04, f"{q_d[j]:+.1f}°",
                        ha="center", va="center", fontsize=7.8, fontweight="bold" if (bad or es_ok) else "normal", color=tc_cell)

        # Costo
        ax_tbl.add_patch(FancyBboxPatch((x_cols[7], y_row), w_cols[7] - 0.006, 0.08, boxstyle="round,pad=0.006",
                                        fc=bg_row, ec=ec_row, lw=0.9 if es_ok else 0.5))
        ax_tbl.text(x_cols[7] + (w_cols[7] - 0.006)/2, y_row + 0.04, f"{costo_r:.2f}",
                    ha="center", va="center", fontsize=7.8, fontweight="bold" if es_ok else "normal", color=SLATE_DARK)

        # Estado
        j_bad = ",".join(f"J{j+1}" for j in range(6) if viola[j])
        est_txt = "ELEGIDA (q*)" if es_ok else f"Tope {j_bad}"
        ax_tbl.add_patch(FancyBboxPatch((x_cols[8], y_row), w_cols[8] - 0.006, 0.08, boxstyle="round,pad=0.006",
                                        fc="#dcfce7" if es_ok else "#fef2f2", ec=GREEN if es_ok else "#fca5a5", lw=0.9))
        ax_tbl.text(x_cols[8] + (w_cols[8] - 0.006)/2, y_row + 0.04, est_txt,
                    ha="center", va="center", fontsize=7.5, fontweight="bold", color=GREEN if es_ok else RED)

    fig.text(
        0.5, 0.042,
        "Decisión tomada: las 8 ramas colocan tool0 exactamente en p_d (error ≤ 1,59 × 10⁻¹⁶ m), pero 7 exceden topes de J3, J4 o J5;\n"
        "se elige la Rama #1 (codo arriba / muñeca no invertida / hombro frontal) con menor costo ponderado (2,8200).",
        ha="center", va="center", fontsize=8.5, color=SLATE_DARK,
        bbox=dict(boxstyle="round,pad=0.40", fc="#fdf2f8", ec=PLUM, lw=1.0)
    )

    salida_11 = DIR_FIG / "11_hito2_pose_y_ramas.png"
    fig.savefig(salida_11, dpi=160)
    plt.close(fig)
    print(f"[OK] {salida_11.name}")

    # --- FIGURA 12: TRAYECTORIA LSPB DE LAS 6 JUNTAS HACIA ESA MISMA q* ---
    V_MAX = np.array([2.0, 2.0, 2.5, 3.0, 3.0, 4.0])
    A_MAX = np.array([4.0, 4.0, 5.0, 6.0, 6.0, 8.0])
    traj = LSPBSincronizado(np.zeros(6), q_sel, V_MAX, A_MAX)
    t = np.linspace(0, traj.T, 1000)
    q_t, qd_t, qdd_t = traj(t)

    fig2, axs = plt.subplots(3, 1, figsize=(10.5, 6.8), dpi=160, sharex=True)
    fig2.patch.set_facecolor("white")

    tc_j2 = traj.perfiles[1].tc  # J2 fija T = 0.7746 s (triangular: tc = T/2 = 0.3873 s)
    series = [
        (q_t, "Posición $q(t)$ [rad]"),
        (qd_t, "Velocidad $\\dot{q}(t)$ [rad/s]"),
        (qdd_t, "Aceleración $\\ddot{q}(t)$ [rad/s²]"),
    ]
    for ax, (Y, ylab) in zip(axs, series):
        ax.axvspan(0, tc_j2, color="#f1f5f9", alpha=0.9, lw=0)
        ax.axvline(tc_j2, color=SLATE_MID, lw=0.9, ls=":")
        ax.axhline(0, color="#cbd5e1", lw=0.7)
        for j in range(6):
            lw = 2.5 if j == 1 else 1.8
            ax.plot(t, Y[:, j], color=C_JUNTAS[j], lw=lw, label=f"J{j+1} ({q_sel[j]:+.1f} rad)")
        ax.set_ylabel(ylab, fontsize=9, color=SLATE_DARK)
        ax.grid(True, color=SLATE_LIGHT, lw=0.6)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    axs[0].set_title(
        f"Hito 2 · Trayectoria LSPB sincronizada desde q = 0 hasta la postura elegida q* (T = {traj.T:.4f} s)",
        fontsize=11.5, fontweight="bold", color=SLATE_DARK, pad=26
    )
    axs[0].legend(ncol=6, frameon=False, fontsize=8.2, loc="lower center", bbox_to_anchor=(0.5, 1.02))
    axs[1].annotate(
        "J2 gobierna la duración (perfil triangular,\n|V_J2| = 1,5492 rad/s, t_c = 0,3873 s)",
        xy=(tc_j2, qd_t[len(t)//2, 1]), xytext=(0.46, -1.48),
        fontsize=8.2, color=C_JUNTAS[1], fontweight="bold",
        arrowprops=dict(arrowstyle="->", color=C_JUNTAS[1], lw=1.0),
        bbox=dict(boxstyle="round,pad=0.25", fc="white", ec=C_JUNTAS[1], lw=0.8)
    )
    axs[2].set_xlabel(f"Tiempo $t$ [s]   ·   Llegada simultánea de las 6 juntas en $T = {traj.T:.4f}$ s con $\\dot{{q}}(0) = \\dot{{q}}(T) = 0$", fontsize=9.2)

    fig2.tight_layout(rect=[0, 0.075, 1, 0.98])
    fig2.text(
        0.5, 0.032,
        "Decisión tomada: J2 impone el tiempo mínimo factible (0,7746 s) y las otras 5 juntas reducen su velocidad de crucero\n"
        "sin elevar sus aceleraciones para terminar el movimiento exactamente en el mismo instante.",
        ha="center", va="center", fontsize=8.4, color=SLATE_DARK,
        bbox=dict(boxstyle="round,pad=0.38", fc="#fdf2f8", ec=PLUM, lw=0.9)
    )
    salida_12 = DIR_FIG / "12_hito2_trayectoria_pose.png"
    fig2.savefig(salida_12, dpi=160)
    plt.close(fig2)
    print(f"[OK] {salida_12.name}")


def generar_hito3_4_evidencia_sesion(reusar_csv: bool = False):
    """Hito 3/4: Ejecuta una sesión real documentada de VEVAS Studio, guarda CSV/JSON y traza la figura central."""
    from vevas.backend_sim import BackendSim
    from vevas.consola import Consola
    from vevas.figuras import generar_figura
    from vevas.registro import Registro
    from vevas.robot_irb120 import Robot
    from vevas.seguridad import cargar_config

    cfg = cargar_config(DIR_VEVAS / "config.json")
    cfg["seguridad"]["watchdog_interfaz_s"] = 5.0
    q0_deg = cfg["postura_inicial_deg"]  # [0, -20, 30, 0, 40, 0]
    q0_rad = np.radians(q0_deg)
    rob = Robot(tcp_xyz=tuple(cfg["robot"]["tcp_xyz_m"]), tcp_rpy=tuple(cfg["robot"]["tcp_rpy_rad"]))

    dir_evidencia = RAIZ / "04_hito3_4_control_real" / "01_docs" / "04_resultados" / "01_sesion_evidencia"
    dir_evidencia.mkdir(parents=True, exist_ok=True)
    csv_dest = dir_evidencia / "muestras_corazon_yz_40mm.csv"

    if not (reusar_csv and csv_dest.exists()):
        with tempfile.TemporaryDirectory() as tmp_ses:
            reg = Registro(tmp_ses, "sim_corazon_yz")
            sim = BackendSim(q0_rad, tau=0.04, ruido_deg=0.003, hz=250.0)
            sim.iniciar()
            con = Consola(cfg, sim, reg)
            con.iniciar()
            try:
                con.pulso()
                ok_arm, msg_arm = con.armar({}, "")
                assert ok_arm, f"Fallo al armar: {msg_arm}"
                plan = con.calcular_figura("corazón", 40, "YZ")
                assert plan["ejecutable"], f"Plan no ejecutable: {plan['problemas']}"
                con.pulso()
                ok, msg = con.ejecutar(plan_id=plan["id"])
                assert ok, f"Fallo al ejecutar: {msg}"
                t_lim = time.monotonic() + plan["tray"]["tf"] + 4.0
                while time.monotonic() < t_lim:
                    con.pulso()
                    with con.lock:
                        if con.ejec is None:
                            break
                    time.sleep(0.03)
                time.sleep(0.15)
            finally:
                con.detener()
                sim.detener()
                reg.cerrar()

            # Copiar artefactos limpios y reproducibles a 01_sesion_evidencia
            carpeta_reg = Path(reg.carpeta)
            csv_src = carpeta_reg / "muestras.csv"
            res_src = carpeta_reg / "resumen.json"
            plan_src = carpeta_reg / "planes" / "plan_001.json"

            csv_dest.write_text(csv_src.read_text(encoding="utf-8"), encoding="utf-8")
            (dir_evidencia / "resumen_corazon_yz_40mm.json").write_text(res_src.read_text(encoding="utf-8"), encoding="utf-8")
            (dir_evidencia / "plan_001_corazon_yz_40mm.json").write_text(plan_src.read_text(encoding="utf-8"), encoding="utf-8")

    # Leer CSV recién grabado para graficar la evidencia real
    filas = list(csv.DictReader(csv_dest.open(encoding="utf-8")))
    resumen = json.loads((dir_evidencia / "resumen_corazon_yz_40mm.json").read_text(encoding="utf-8"))[0]
    plan_doc = json.loads((dir_evidencia / "plan_001_corazon_yz_40mm.json").read_text(encoding="utf-8"))

    filas_mov = [r for r in filas if r["plan_id"] == "1" and r["estado"] == "ejecutando"]
    t_m = np.array([float(r["t_s"]) for r in filas_mov])
    t_m = t_m - t_m[0]
    y_med = np.array([float(r["y_mm"]) for r in filas_mov])
    z_med = np.array([float(r["z_mm"]) for r in filas_mov])
    ep_mm = np.array([float(r["e_p_mm"]) for r in filas_mov])

    origen_tcp = rob.fk_tcp(q0_rad)[:3, 3] * 1000.0  # mm
    dy_med = y_med - origen_tcp[1]
    dz_med = z_med - origen_tcp[2]

    ref_offsets_mm = generar_figura("corazón", 40, "YZ", muestras=65) * 1000.0
    dy_ref = ref_offsets_mm[:, 1]
    dz_ref = ref_offsets_mm[:, 2]

    camino_plan_mm = np.array(plan_doc["preview"]["camino"], dtype=float) * 1000.0
    t_plan = np.array(plan_doc["preview"]["t"], dtype=float)
    dy_plan = camino_plan_mm[:, 1] - origen_tcp[1]
    dz_plan = camino_plan_mm[:, 2] - origen_tcp[2]

    # Error geométrico transversal de cada muestra medida al contorno solicitado
    dists_contorno = [
        float(np.min(np.hypot(dy_ref - ym, dz_ref - zm)))
        for ym, zm in zip(dy_med, dz_med)
    ]
    err_contorno_max = max(dists_contorno)
    err_contorno_med = float(np.mean(dists_contorno))

    # --- FIGURA 13: LA FIGURA CENTRAL DE LA DEFENSA (3 PANELES + BANDA INFERIOR DE SEGUIMIENTO) ---
    fig = plt.figure(figsize=(13.4, 7.4), dpi=160)
    fig.patch.set_facecolor("white")

    fig.text(
        0.5, 0.968,
        "Hitos 3 y 4 · Evidencia visual de sesión: de la figura solicitada a las muestras medidas del TCP",
        ha="center", va="top", fontsize=12.8, fontweight="bold", color=SLATE_DARK
    )
    fig.text(
        0.5, 0.930,
        f"Figura: Corazón 40 mm   ·   Plano: YZ («De frente», mirado desde +X)   ·   "
        f"Postura base: [0°, −20°, +30°, 0°, +40°, 0°]   ·   Velocidad: 5 %   ·   "
        f"Duración: {resumen['duracion_s']:.2f} s ({len(filas_mov)} muestras a 50 Hz)",
        ha="center", va="top", fontsize=9.1, color=SLATE_MID
    )

    gs = fig.add_gridspec(2, 3, left=0.055, right=0.975, bottom=0.09, top=0.86,
                          height_ratios=[1.55, 0.78], hspace=0.34, wspace=0.24)

    # Panel 1: Figura solicitada (geometría pura y nudos)
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(dy_ref, dz_ref, "-", color=SLATE_DARK, lw=2.0, label="Contorno solicitado (65 nudos)")
    ax1.scatter(dy_ref[1:-1:4], dz_ref[1:-1:4], color=SLATE_MID, s=14, alpha=0.65, zorder=4)
    ax1.scatter([dy_ref[0]], [dz_ref[0]], color=GREEN, s=65, edgecolors="white", lw=1.2, zorder=8, label="Inicio / cierre (0, 0) mm")
    ax1.scatter([dy_ref[32]], [dz_ref[32]], color=ABB_ORANGE, s=65, marker="D", edgecolors="white", lw=1.2, zorder=8, label="Parada v = 0 (cúspide #32)")
    # Flecha de sentido
    ax1.annotate("", xy=(dy_ref[10], dz_ref[10]), xytext=(dy_ref[6], dz_ref[6]),
                 arrowprops=dict(arrowstyle="->", color=PLUM, lw=1.8))
    ax1.set_title("1 · Figura solicitada (geometría)", fontsize=10, fontweight="bold", color=SLATE_DARK)
    ax1.set_xlabel("ΔY en marco Base [mm]", fontsize=8.5)
    ax1.set_ylabel("ΔZ en marco Base [mm]", fontsize=8.5)
    ax1.set_aspect("equal", adjustable="box")
    ax1.grid(True, color=SLATE_LIGHT, lw=0.6)
    ax1.legend(loc="lower center", fontsize=7.2, frameon=True, facecolor="white", framealpha=0.95)

    # Panel 2: Trayectoria planificada (camino quíntico FK)
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(dy_ref, dz_ref, "--", color="#94a3b8", lw=1.3, label="Referencia geométrica")
    sc2 = ax2.scatter(dy_plan, dz_plan, c=t_plan, cmap="viridis", s=14, zorder=5)
    ax2.plot(dy_plan, dz_plan, "-", color=BLUE, lw=1.2, alpha=0.55, label="FK del plan quíntico C²")
    cbar = plt.colorbar(sc2, ax=ax2, fraction=0.046, pad=0.03)
    cbar.set_label("t plan [s]", fontsize=7.5)
    cbar.ax.tick_params(labelsize=7)
    ax2.set_title("2 · Trayectoria planificada (Pieper + C²)", fontsize=10, fontweight="bold", color=SLATE_DARK)
    ax2.set_xlabel("ΔY en marco Base [mm]", fontsize=8.5)
    ax2.set_ylabel("ΔZ en marco Base [mm]", fontsize=8.5)
    ax2.set_aspect("equal", adjustable="box")
    ax2.grid(True, color=SLATE_LIGHT, lw=0.6)
    ax2.legend(loc="lower center", fontsize=7.2, frameon=True, facecolor="white", framealpha=0.95)

    # Panel 3: Superposición con muestras medidas del TCP
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(dy_ref, dz_ref, "-", color="#cbd5e1", lw=3.0, label="Figura solicitada", zorder=2)
    ax3.plot(dy_plan, dz_plan, "--", color=BLUE, lw=1.4, label="Trayectoria planificada", zorder=4)
    ax3.scatter(dy_med[::2], dz_med[::2], color=PLUM, s=15, alpha=0.82, edgecolors="none",
                label=f"Muestras TCP ({len(filas_mov)} pts)", zorder=6)
    ax3.scatter([dy_med[-1]], [dz_med[-1]], color=GREEN, s=60, edgecolors="white", lw=1.2, zorder=9)
    ax3.set_title("3 · Superposición con muestras del TCP", fontsize=10, fontweight="bold", color=SLATE_DARK)
    ax3.set_xlabel("ΔY en marco Base [mm]", fontsize=8.5)
    ax3.set_ylabel("ΔZ en marco Base [mm]", fontsize=8.5)
    ax3.set_aspect("equal", adjustable="box")
    ax3.grid(True, color=SLATE_LIGHT, lw=0.6)
    ax3.legend(loc="lower center", fontsize=7.2, frameon=True, facecolor="white", framealpha=0.95)

    # Banda inferior: Evolución temporal ΔY(t), ΔZ(t) y error de seguimiento e_p(t)
    ax_t1 = fig.add_subplot(gs[1, :2])
    ax_t1.plot(t_plan, dy_plan, "--", color=BLUE, lw=1.4, label="ΔY plan [mm]")
    ax_t1.plot(t_m, dy_med, "-", color=BLUE, lw=2.0, alpha=0.85, label="ΔY medido [mm]")
    ax_t1.plot(t_plan, dz_plan, "--", color=ABB_ORANGE, lw=1.4, label="ΔZ plan [mm]")
    ax_t1.plot(t_m, dz_med, "-", color=ABB_ORANGE, lw=2.0, alpha=0.85, label="ΔZ medido [mm]")
    ax_t1.axvline(t_plan[len(t_plan)//2], color=SLATE_MID, lw=0.9, ls=":", label="Parada en cúspide (v = 0)")
    ax_t1.set_xlabel("Tiempo de ejecución t [s]", fontsize=8.5)
    ax_t1.set_ylabel("Desplazamiento TCP [mm]", fontsize=8.5)
    ax_t1.set_title("Seguimiento temporal de coordenadas cartesianas (plan vs muestras registradas)", fontsize=9.5, fontweight="bold", color=SLATE_DARK, loc="left")
    ax_t1.grid(True, color=SLATE_LIGHT, lw=0.6)
    ax_t1.legend(ncol=5, fontsize=7.4, frameon=False, loc="upper right")

    ax_t2 = fig.add_subplot(gs[1, 2])
    ax_t2.plot(t_m, ep_mm, color=PLUM, lw=1.9, label="$e_p(t) = \\|p_{med} - p_{cmd}\\|$")
    ax_t2.fill_between(t_m, 0, ep_mm, color=PLUM, alpha=0.12)
    ax_t2.set_xlabel("Tiempo t [s]", fontsize=8.5)
    ax_t2.set_ylabel("Error TCP $e_p$ [mm]", fontsize=8.5)
    ax_t2.set_title(
        f"Error dinámico máx: {resumen['e_p_seguimiento_max_mm']:.2f} mm · final: {resumen['e_p_final_mm']:.3f} mm",
        fontsize=9.2, fontweight="bold", color=SLATE_DARK, loc="left"
    )
    ax_t2.grid(True, color=SLATE_LIGHT, lw=0.6)
    ax_t2.legend(fontsize=7.5, frameon=False, loc="upper right")

    for ax in (ax1, ax2, ax3):
        ax.set_xlim(-25, 25)
        ax.set_ylim(-41, 13)

    for ax in (ax1, ax2, ax3, ax_t1, ax_t2):
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.tick_params(labelsize=7.8)

    fig.text(
        0.5, 0.022,
        f"Registro reproducible en 04_resultados/01_sesion_evidencia/muestras_corazon_yz_40mm.csv   ·   "
        f"Desviación geométrica al contorno: media {err_contorno_med:.3f} mm, máx. {err_contorno_max:.3f} mm   ·   "
        f"Error articular final: {resumen['error_final_max_deg']:.4f}°",
        ha="center", va="center", fontsize=8.4, color=SLATE_DARK,
        bbox=dict(boxstyle="round,pad=0.38", fc="#fdf2f8", ec=PLUM, lw=0.9)
    )

    salida_13 = DIR_FIG / "13_evidencia_sesion_tcp.png"
    fig.savefig(salida_13, dpi=160)
    plt.close(fig)
    print(f"[OK] {salida_13.name}")


if __name__ == "__main__":
    generar_hito1_visor_vs_rviz()
    generar_hito2_recorrido_pose()
    generar_hito3_4_evidencia_sesion()
