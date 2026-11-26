#!/usr/bin/env python3
"""
Genera dos animaciones GIF suaves y ligeras desde los modelos y datos reales del repositorio:
  - 14_animacion_corazon_3d_2d.gif   (Brazo 3D STL trazando el corazón YZ + lienzo 2D TCP en vivo)
  - 15_animacion_hito2_pieper_lspb.gif (Transición 3D q=0 -> q* de Pieper + barrido LSPB 6 juntas)

Grupo VEVAS · Robótica IMT-342 · UCB Sede Tarija
"""

from __future__ import annotations

import csv
import io
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from PIL import Image

RAIZ = Path(__file__).resolve().parents[2]
DIR_FIG = Path(__file__).resolve().parent
sys.path.insert(0, str(DIR_FIG))
sys.path.insert(0, str(RAIZ / "05_ros2_irb120_description" / "04_scripts"))
sys.path.insert(0, str(RAIZ / "04_hito3_4_control_real" / "02_vevas_studio"))

from generar_evidencia_visual import (
    ABB_ORANGE,
    BLUE,
    C_JUNTAS,
    GREEN,
    PLUM,
    SLATE_DARK,
    SLATE_LIGHT,
    SLATE_MID,
    leer_stl_binario,
    sombrear_caras,
    transformar_triangulos,
)


def _cargar_mallas_stl(paso: int = 4):
    dir_stl = RAIZ / "04_hito3_4_control_real" / "02_vevas_studio" / "08_web" / "03_modelo"
    alineacion_raw = json.loads((dir_stl / "alineacion.json").read_text(encoding="utf-8"))
    alineacion = [np.array(m, dtype=float).reshape(4, 4) for m in alineacion_raw]
    nombres = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]
    colores = ["#334155", "#ea580c", "#f97316", "#ea580c", "#f97316", "#cbd5e1", "#475569"]
    tris_locales = [leer_stl_binario(dir_stl / f"{n}.stl", paso=paso) for n in nombres]
    return tris_locales, alineacion, colores


def _fig_a_pil(fig: plt.Figure) -> Image.Image:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=105)
    buf.seek(0)
    return Image.open(buf).convert("RGB")


def _guardar_gif(frames_rgb: list[Image.Image], ruta: Path, duracion_ms: int = 85):
    paleta_base = frames_rgb[len(frames_rgb) // 2].quantize(colors=128, method=Image.Quantize.MEDIANCUT)
    cuantizados = [im.quantize(palette=paleta_base, dither=Image.Dither.NONE) for im in frames_rgb]
    duraciones = [duracion_ms] * len(cuantizados)
    duraciones[0] = 350
    duraciones[-1] = 650
    cuantizados[0].save(
        ruta,
        save_all=True,
        append_images=cuantizados[1:],
        duration=duraciones,
        loop=0,
        optimize=True,
    )


def generar_gif_corazon_3d_2d():
    from vevas.figuras import generar_figura
    from vevas.robot_irb120 import Robot

    rob = Robot()
    tris_locales, alineacion, colores = _cargar_mallas_stl(paso=4)

    csv_path = (
        RAIZ
        / "04_hito3_4_control_real"
        / "01_docs"
        / "04_resultados"
        / "01_sesion_evidencia"
        / "muestras_corazon_yz_40mm.csv"
    )
    filas = [
        r for r in csv.DictReader(csv_path.open(encoding="utf-8"))
        if r["plan_id"] == "1" and r["estado"] == "ejecutando"
    ]

    q0_rad = np.radians([0, -20, 30, 0, 40, 0])
    origen_mm = rob.fk_tcp(q0_rad)[:3, 3] * 1000.0
    ref_mm = generar_figura("corazón", 40, "YZ", muestras=65) * 1000.0
    dy_ref, dz_ref = ref_mm[:, 1], ref_mm[:, 2]

    t_all = np.array([float(r["t_s"]) for r in filas])
    t_all -= t_all[0]
    dy_all = np.array([float(r["y_mm"]) for r in filas]) - origen_mm[1]
    dz_all = np.array([float(r["z_mm"]) for r in filas]) - origen_mm[2]
    ep_all = np.array([float(r["e_p_mm"]) for r in filas])
    Q_med_rad = np.radians([[float(r[f"q{j}_med_deg"]) for j in range(1, 7)] for r in filas])
    P_tcp_m = np.array([[float(r["x_mm"]), float(r["y_mm"]), float(r["z_mm"])] for r in filas]) / 1000.0

    indices = np.linspace(0, len(filas) - 1, 34, dtype=int)
    frames = []

    gx = np.linspace(-0.20, 0.42, 7)
    gy = np.linspace(-0.26, 0.26, 6)

    for k in indices:
        fig = plt.figure(figsize=(9.2, 4.35), dpi=105)
        fig.patch.set_facecolor("white")

        fig.text(
            0.5, 0.96,
            "VEVAS Studio · Trazado continuo en vivo: Corazón 40 mm en plano frontal YZ (5 % velocidad)",
            ha="center", va="top", fontsize=10.6, fontweight="bold", color=SLATE_DARK
        )

        ax3d = fig.add_axes([0.02, 0.08, 0.48, 0.81], projection="3d")
        ax3d.set_facecolor("#0f172a")
        for x in gx:
            ax3d.plot([x, x], [gy[0], gy[-1]], [0, 0], color="#334155", lw=0.6)
        for y in gy:
            ax3d.plot([gx[0], gx[-1]], [y, y], [0, 0], color="#334155", lw=0.6)

        marcos = rob.marcos(Q_med_rad[k])
        for idx_m in range(7):
            tris_w = transformar_triangulos(tris_locales[idx_m], marcos[idx_m] @ alineacion[idx_m])
            rgba = sombrear_caras(tris_w, colores[idx_m])
            ax3d.add_collection3d(Poly3DCollection(tris_w, facecolors=rgba, edgecolors="none"))

        # Estela 3D del TCP
        ax3d.plot(
            origen_mm[0]/1000 + ref_mm[:, 0]/1000,
            origen_mm[1]/1000 + dy_ref/1000,
            origen_mm[2]/1000 + dz_ref/1000,
            "--", color="#64748b", lw=1.0
        )
        ax3d.plot(P_tcp_m[:k+1, 0], P_tcp_m[:k+1, 1], P_tcp_m[:k+1, 2], "-", color="#f472b6", lw=2.4)
        ax3d.scatter([P_tcp_m[k, 0]], [P_tcp_m[k, 1]], [P_tcp_m[k, 2]], color="#4ade80", s=34, zorder=20)

        ax3d.set_xlim(-0.20, 0.42)
        ax3d.set_ylim(-0.26, 0.26)
        ax3d.set_zlim(0.0, 0.68)
        ax3d.set_box_aspect((0.62, 0.52, 0.68))
        ax3d.view_init(elev=18, azim=-36)
        ax3d.set_axis_off()
        ax3d.set_title("Gemelo 3D ABB IRB 120 (vista frontal-lateral)", fontsize=9.2, fontweight="bold", color=SLATE_DARK, pad=2)

        # Panel derecho: Vista 2D milimetrada en vivo
        ax2d = fig.add_axes([0.56, 0.14, 0.40, 0.72])
        ax2d.plot(dy_ref, dz_ref, "--", color="#cbd5e1", lw=2.0, label="Referencia solicitada")
        ax2d.plot(dy_all[:k+1], dz_all[:k+1], "-", color=PLUM, lw=2.4, label="Rastro TCP medido")
        ax2d.scatter([0], [0], color=GREEN, s=42, zorder=6)
        ax2d.scatter([dy_ref[32]], [dz_ref[32]], color=ABB_ORANGE, marker="D", s=40, zorder=6)
        ax2d.scatter([dy_all[k]], [dz_all[k]], color="#22c55e", edgecolors=SLATE_DARK, lw=1.2, s=58, zorder=10)

        ax2d.set_xlim(-24, 24)
        ax2d.set_ylim(-33, 12)
        ax2d.set_aspect("equal", adjustable="box")
        ax2d.grid(True, color=SLATE_LIGHT, lw=0.6)
        ax2d.spines["top"].set_visible(False)
        ax2d.spines["right"].set_visible(False)
        ax2d.set_xlabel("ΔY en marco Base [mm]", fontsize=8.5)
        ax2d.set_ylabel("ΔZ en marco Base [mm]", fontsize=8.5)
        ax2d.tick_params(labelsize=7.5)
        ax2d.set_title(
            f"t = {t_all[k]:4.2f} / {t_all[-1]:.2f} s   ·   e_p = {ep_all[k]:.2f} mm",
            fontsize=9.2, fontweight="bold", color=PLUM
        )
        ax2d.legend(loc="lower right", fontsize=7.2, frameon=True, facecolor="white")

        frames.append(_fig_a_pil(fig))
        plt.close(fig)

    out = DIR_FIG / "14_animacion_corazon_3d_2d.gif"
    _guardar_gif(frames, out, duracion_ms=85)
    print(f"[OK] {out.name} ({out.stat().st_size / 1024:.0f} KB)")


def generar_gif_hito2_pieper_lspb():
    from lspb import LSPBSincronizado
    from vevas.robot_irb120 import Robot

    rob = Robot()
    tris_locales, alineacion, colores = _cargar_mallas_stl(paso=4)

    q_sel = np.array([0.4, -0.6, 0.5, 0.3, 0.7, -0.2])
    V_MAX = np.array([2.0, 2.0, 2.5, 3.0, 3.0, 4.0])
    A_MAX = np.array([4.0, 4.0, 5.0, 6.0, 6.0, 8.0])
    traj = LSPBSincronizado(np.zeros(6), q_sel, V_MAX, A_MAX)

    t_dense = np.linspace(0, traj.T, 300)
    _, qd_dense, _ = traj(t_dense)

    t_frames = np.linspace(0, traj.T, 28)
    q_frames, qd_frames, _ = traj(t_frames)
    tcp_frames = np.array([rob.fk_tcp(qf)[:3, 3] for qf in q_frames])

    gx = np.linspace(-0.24, 0.44, 7)
    gy = np.linspace(-0.26, 0.26, 6)
    frames = []

    for idx_f, tk in enumerate(t_frames):
        fig = plt.figure(figsize=(9.2, 4.2), dpi=105)
        fig.patch.set_facecolor("white")

        fig.text(
            0.5, 0.96,
            "Hito 2 · Transición sincronizada LSPB desde q = 0 hasta la Rama #1 de Pieper (T = 0,7746 s)",
            ha="center", va="top", fontsize=10.5, fontweight="bold", color=SLATE_DARK
        )

        ax3d = fig.add_axes([0.02, 0.07, 0.45, 0.82], projection="3d")
        ax3d.set_facecolor("#f8fafc")
        for x in gx:
            ax3d.plot([x, x], [gy[0], gy[-1]], [0, 0], color="#cbd5e1", lw=0.6)
        for y in gy:
            ax3d.plot([gx[0], gx[-1]], [y, y], [0, 0], color="#cbd5e1", lw=0.6)

        marcos = rob.marcos(q_frames[idx_f])
        for idx_m in range(7):
            tris_w = transformar_triangulos(tris_locales[idx_m], marcos[idx_m] @ alineacion[idx_m])
            rgba = sombrear_caras(tris_w, colores[idx_m])
            ax3d.add_collection3d(Poly3DCollection(tris_w, facecolors=rgba, edgecolors="none"))

        ax3d.plot(tcp_frames[:idx_f+1, 0], tcp_frames[:idx_f+1, 1], tcp_frames[:idx_f+1, 2], "-", color=PLUM, lw=2.5)
        ax3d.scatter([tcp_frames[-1, 0]], [tcp_frames[-1, 1]], [tcp_frames[-1, 2]], color=PLUM, marker="*", s=65, zorder=20)

        ax3d.set_xlim(-0.24, 0.44)
        ax3d.set_ylim(-0.26, 0.26)
        ax3d.set_zlim(0.0, 0.70)
        ax3d.set_box_aspect((0.68, 0.52, 0.70))
        ax3d.view_init(elev=20, azim=-44)
        ax3d.set_axis_off()
        ax3d.set_title("Movimiento coordinado hacia T_d", fontsize=9.2, fontweight="bold", color=SLATE_DARK, pad=2)

        ax_v = fig.add_axes([0.53, 0.15, 0.43, 0.70])
        ax_v.axhline(0, color="#cbd5e1", lw=0.8)
        for j in range(6):
            lw = 2.4 if j == 1 else 1.6
            ax_v.plot(t_dense, qd_dense[:, j], color=C_JUNTAS[j], lw=lw, alpha=0.32)
            mask = t_dense <= tk + 1e-9
            ax_v.plot(t_dense[mask], qd_dense[mask, j], color=C_JUNTAS[j], lw=lw, label=f"J{j+1}")
            ax_v.scatter([tk], [qd_frames[idx_f, j]], color=C_JUNTAS[j], s=22, zorder=8)

        ax_v.axvline(tk, color=PLUM, lw=1.3, ls="--")
        ax_v.set_xlim(0, traj.T)
        ax_v.set_ylim(-1.75, 1.45)
        ax_v.grid(True, color=SLATE_LIGHT, lw=0.6)
        ax_v.spines["top"].set_visible(False)
        ax_v.spines["right"].set_visible(False)
        ax_v.set_xlabel("Tiempo t [s]", fontsize=8.5)
        ax_v.set_ylabel("Velocidad q̇(t) [rad/s]", fontsize=8.5)
        ax_v.tick_params(labelsize=7.5)
        ax_v.legend(ncol=6, frameon=False, fontsize=7.2, loc="lower center", bbox_to_anchor=(0.5, 1.01))
        ax_v.set_title(f"Perfiles de velocidad sincronizados · t = {tk:.3f} s", fontsize=9.0, fontweight="bold", color=SLATE_DARK, pad=16)

        frames.append(_fig_a_pil(fig))
        plt.close(fig)

    out = DIR_FIG / "15_animacion_hito2_pieper_lspb.gif"
    _guardar_gif(frames, out, duracion_ms=85)
    print(f"[OK] {out.name} ({out.stat().st_size / 1024:.0f} KB)")


if __name__ == "__main__":
    generar_gif_corazon_3d_2d()
    generar_gif_hito2_pieper_lspb()
