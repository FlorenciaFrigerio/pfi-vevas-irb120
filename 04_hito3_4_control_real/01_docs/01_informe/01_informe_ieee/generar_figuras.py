#!/usr/bin/env python3
"""Figuras y cifras calculadas del informe IEEE del Hito 3/4 (Grupo VEVAS).

Usa SOLO los módulos del proyecto (07_vevas/) y los valores medidos del 24/11/2026
(HECHOS_24NOV.md). No se inventa ninguna medición: lo medido se copia tal cual
(rangos de velocidad por grupo de juntas) y todo lo demás se calcula aquí.

    python3 04_hito3_4_control_real/01_docs/01_informe/01_informe_ieee/generar_figuras.py     (desde la raíz del repo)

Salida: 01_img/fig_c02.pdf, 01_img/fig_sigma.pdf y un resumen numérico por consola.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, "..", "..", "..", "02_vevas_studio"))
sys.path.insert(0, RAIZ)

from vevas.jacobiano import analisis_singularidad  # noqa: E402
from vevas.revision import revisar_trayectoria  # noqa: E402
from vevas.robot_irb120 import VEL_MAX, VEL_MAX_DEG, Robot  # noqa: E402
from vevas.rws_cliente import (K_PICO_DEFECTO, VLIM_DEG_S_DEFECTO,  # noqa: E402
                               limites_c02, subdividir, tiempo_movimiento)
from vevas.seguridad import cargar_config  # noqa: E402
from vevas.trayectoria import planificar_suave  # noqa: E402

IMG = os.path.join(AQUI, "01_img")
os.makedirs(IMG, exist_ok=True)

# Paleta categórica fija (orden: azul, naranja, aqua) y tinta de texto
C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"
TINTA, GRIS = "#222222", "#777777"

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Liberation Serif", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 8, "axes.labelsize": 8, "axes.titlesize": 8,
    "xtick.labelsize": 7, "ytick.labelsize": 7, "legend.fontsize": 7,
    "axes.edgecolor": GRIS, "axes.labelcolor": TINTA, "xtick.color": TINTA, "ytick.color": TINTA,
    "axes.grid": True, "grid.color": "#e3e3e3", "grid.linewidth": 0.5,
    "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 1.4, "pdf.fonttype": 42,
})
ANCHO = 3.45  # pulgadas: una columna IEEE


def coma(v):
    return f"{v:g}".replace(".", ",")

# ------------------------------------------------------------------ C02
with open(os.path.join(RAIZ, "05_rapid", "VEVAS_RWS.mod"), encoding="utf-8") as f:
    texto_mod = f.read()
vlim, kpico = limites_c02(texto_mod)
assert tuple(vlim) == tuple(VLIM_DEG_S_DEFECTO) and kpico == K_PICO_DEFECTO
print("Módulo:", len(texto_mod), "caracteres; vLim =", vlim, "kPico =", kpico)
print("vLim / ficha [%]:", [round(100 * v / m, 2) for v, m in zip(vlim, VEL_MAX_DEG)])

# Factor pico/medio de perfiles suaves (justificación de kPico)
u = np.linspace(0, 1, 200001)
quintico = np.max(30 * u**2 - 60 * u**3 + 30 * u**4)          # s'(u) de 10u^3-15u^4+6u^5
cubico = np.max(6 * u - 6 * u**2)                              # s'(u) de 3u^2-2u^3
print(f"pico/medio: quíntico {quintico:.4f}, cúbico {cubico:.4f}")

DELTA = 5.0  # grados, valor por defecto de 02_herramientas/verificar_c02.py
grupos = [("J1–J3", 0, (5.2, 5.4)), ("J4–J5", 3, (6.7, 6.9)), ("J6", 5, (8.8, 8.9))]  # HECHOS 24/11
for nombre, j, (lo, hi) in grupos:
    q = [0.0] * 6
    q[j] = DELTA
    t = tiempo_movimiento(q, [0.0] * 6, vlim, kpico)
    medio = DELTA / t
    print(f"{nombre}: t({DELTA}°) = {t:.3f} s, v_media = {medio:.2f} °/s, medido {lo}-{hi} °/s "
          f"-> pico/medio {lo / medio:.3f}-{hi / medio:.3f}, medido/vLim {lo / vlim[j]:.3f}-{hi / vlim[j]:.3f}, "
          f"medido/ficha {100 * hi / VEL_MAX_DEG[j]:.2f} %")

fig, (a1, a2) = plt.subplots(2, 1, figsize=(ANCHO, 3.55), gridspec_kw={"height_ratios": [1.15, 1]})
dq = np.linspace(0, 40, 401)
for (nombre, j, _), c in zip(grupos, (C1, C2, C3)):
    t = [tiempo_movimiento([d if k == j else 0 for k in range(6)], [0] * 6, vlim, kpico) for d in dq]
    a1.plot(dq, t, color=c, label=f"{nombre} ($v_{{\\lim}}$ = {coma(vlim[j])} °/s)")
    t5 = tiempo_movimiento([DELTA if k == j else 0 for k in range(6)], [0] * 6, vlim, kpico)
    a1.plot([DELTA], [t5], "o", ms=4.5, color=c, mec="white", mew=0.8, zorder=5)
a1.axhline(0.5, color=GRIS, ls=":", lw=0.9)
a1.text(39.5, 0.62, "piso 0,5 s", ha="right", va="bottom", fontsize=7, color=GRIS)
a1.annotate(r"ensayo $\delta$ = 5°", xy=(5, 1.0), xytext=(0.8, 3.0), fontsize=7, color=TINTA,
            arrowprops=dict(arrowstyle="-", color=GRIS, lw=0.6))
a1.set_xlabel(r"$|\Delta q_i|$ del tramo [°]")
a1.set_ylabel(r"$t$ impuesto por RAPID [s]")
a1.set_xlim(0, 40)
a1.set_ylim(0, 8.5)
a1.legend(loc="upper left", frameon=False, handlelength=1.6)
a1.set_title("(a) Ley de tiempo C02 (una sola junta en movimiento)", loc="left")

x = np.arange(len(grupos))
for i, ((nombre, j, (lo, hi)), c) in enumerate(zip(grupos, (C1, C2, C3))):
    a2.bar(i, vlim[j], width=0.55, color="white", edgecolor=c, lw=1.0, zorder=2)
    a2.bar(i, hi, width=0.55, color=c, alpha=0.85, zorder=3)
    a2.plot([i - 0.32, i + 0.32], [vlim[j] / kpico] * 2, color=TINTA, lw=0.9, ls="--", zorder=4)
    a2.text(i, vlim[j] + 0.4, coma(vlim[j]), ha="center", va="bottom", fontsize=7, color=TINTA)
    a2.text(i, hi + 1.1, f"{coma(lo)}–{coma(hi)}", ha="center", va="bottom", fontsize=7, color=TINTA)
a2.set_xticks(x, [g[0] for g in grupos])
a2.set_ylabel("velocidad [°/s]")
a2.set_ylim(0, 34)
a2.grid(axis="x", visible=False)
from matplotlib.patches import Patch  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
a2.legend(handles=[Patch(facecolor="white", edgecolor=GRIS, label=r"tope $v_{\lim}$ (5 % ficha)"),
                   Patch(facecolor=GRIS, alpha=0.85, label=r"$v_{\max}$ medida (ventana 0,1 s)"),
                   Line2D([], [], color=TINTA, ls="--", lw=0.9, label=r"media nominal $v_{\lim}/k_{pico}$")],
          loc="upper left", frameon=False, ncol=1)
a2.set_title("(b) Velocidad máxima medida en el IRB 120 real (24/11)", loc="left")
fig.tight_layout(h_pad=0.6)
fig.savefig(os.path.join(IMG, "fig_c02.pdf"), bbox_inches="tight", pad_inches=0.02)
plt.close(fig)

# ------------------------------------------------------------------ singularidad
robot = Robot()
cfg = cargar_config(os.path.join(RAIZ, "config.json"))
umbral = float(cfg["ik"]["sigma_min"])
tc = cfg["trayectoria"]
q_ini = np.radians(cfg["postura_inicial_deg"])
casos = [
    ("«Volver a cero» desde la postura inicial", q_ini, np.zeros(6), "articular_real", C1),
    ("«Preparar J5 a +30°» desde $q$ = 0", np.zeros(6), np.radians([0, 0, 0, 0, 30, 0]), False, C2),
]
fig, ax = plt.subplots(figsize=(ANCHO, 2.05))
for nombre, qa, qb, permitir, c in casos:
    tr = planificar_suave(qa, qb, VEL_MAX, 0.05, tc["t_acel_s"], t_min=tc["t_min_s"])
    ts, Q, QD, _ = tr.muestrear(0.02)
    sig = np.array([analisis_singularidad(robot, q)["sigma_min"] for q in Q])
    inf = revisar_trayectoria(robot, ts, Q, QD, qa, cfg, 5, exige_go=True, permitir_singular_destino=permitir)
    chk = next(ch for ch in inf["chequeos"] if ch["id"] == "singularidad")
    t_c02 = tiempo_movimiento(np.degrees(qb), np.degrees(qa), vlim, kpico)
    n_tramos = len(subdividir(np.degrees(qa), np.degrees(Q), 40.0))
    n_tramos10 = len(subdividir(np.degrees(qa), np.degrees(Q), 10.0))
    print(f"{nombre}: σ ini {sig[0]:.4f} fin {sig[-1]:.2e} mín {sig.min():.2e}; revisión «{chk['estado']}»; "
          f"t C02 = {t_c02:.3f} s; tramos 40° = {n_tramos}, tramos 10° = {n_tramos10}")
    ax.plot(ts / ts[-1], sig, color=c, label=nombre)
    # mismo plan sin C02 (sin excepción articular): ¿qué dice la revisión?
    inf2 = revisar_trayectoria(robot, ts, Q, QD, qa, cfg, 5, exige_go=True, permitir_singular_destino=False)
    chk2 = next(ch for ch in inf2["chequeos"] if ch["id"] == "singularidad")
    print("   sin excepción articular_real:", chk2["estado"])
ax.axhspan(0, umbral, color="#d9d9d9", alpha=0.6, lw=0)
ax.axhline(2 * umbral, color=GRIS, ls=":", lw=0.9)
ax.text(0.5, umbral * 0.45, r"singular: $\sigma_{\min}<0{,}02$", ha="center", va="center", fontsize=7, color=TINTA)
ax.text(0.13, 2 * umbral + 0.013, r"«cerca»: $\sigma_{\min}<0{,}04$", ha="center", va="bottom", fontsize=7, color=GRIS)
ax.set_xlabel(r"tiempo normalizado $t/t_f$")
ax.set_ylabel(r"$\sigma_{\min}(J_L)$")
ax.set_xlim(0, 1)
ax.set_ylim(0, 0.30)
ax.legend(loc="upper center", frameon=False, bbox_to_anchor=(0.5, 1.22), ncol=1)
fig.tight_layout()
fig.savefig(os.path.join(IMG, "fig_sigma.pdf"), bbox_inches="tight", pad_inches=0.02)
plt.close(fig)

# σ en q = 0 y en la postura inicial
print("σmin(q=0) =", analisis_singularidad(robot, np.zeros(6))["sigma_min"],
      " σmin(postura inicial) =", analisis_singularidad(robot, q_ini)["sigma_min"])
# Subdivisión: tiempo nominal total con tramos de 10° vs 40° (Δq5 = 40°)
for paso in (10.0, 40.0):
    w = subdividir([0, -20, 30, 0, 40, 0], [[0, 0, 0, 0, 0, 0]], paso)
    prev, tot = [0, -20, 30, 0, 40, 0], 0.0
    for p in w:
        tot += tiempo_movimiento(p, prev, vlim, kpico)
        prev = p
    print(f"paso {paso:g}°: {len(w)} tramos con fine, tiempo nominal de movimiento {tot:.3f} s")
