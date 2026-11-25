#!/usr/bin/env python3
"""Analiza una sesión grabada por la consola y arma un informe con diagnóstico.

Uso:
    python3 02_herramientas/analizar_sesion.py sesiones/2026-11-18_10-30-00_ros_real
    python3 02_herramientas/analizar_sesion.py --ultima

Crea dentro de la carpeta de la sesión:
    informe/informe.md          tablas por movimiento, diagnóstico y entradas de bitácora listas
    informe/mov_<id>.png        q comandada vs medida, error por junta, e_p y e_R
    informe/resumen_errores.png errores máximos de todos los movimientos

Diagnóstico automático (reglas simples y explicables):
    - retardo: cuánto va el robot detrás de lo comandado (se busca el corrimiento en
      el tiempo que hace mínima la diferencia entre q_comandada y q_medida)
    - signo invertido: si la junta se mueve al revés de lo comandado (correlación negativa)
    - offset: si al final queda una diferencia constante (calibración o convención)
    - error alto: si se acercó al límite de seguimiento (bajar velocidad)
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np


def leer_muestras(carpeta):
    with open(os.path.join(carpeta, "muestras.csv"), encoding="utf-8") as f:
        filas = list(csv.DictReader(f))
    if not filas:
        return None
    d = {k: [] for k in filas[0]}
    for r in filas:
        for k, v in r.items():
            d[k].append(v)
    out = {"estado": np.array(d["estado"]), "plan_id": np.array([int(x) for x in d["plan_id"]])}
    for k, v in d.items():
        if k in ("estado", "plan_id"):
            continue
        out[k] = np.array([float(x) if x not in ("", "nan") else np.nan for x in v])
    return out


def leer_jsonl(ruta):
    if not os.path.exists(ruta):
        return []
    with open(ruta, encoding="utf-8") as f:
        return [json.loads(l) for l in f if l.strip()]


def retardo_ms(t, cmd, med):
    """Corrimiento temporal (ms) que mejor alinea q_medida con q_comandada."""
    if len(t) < 20 or np.ptp(cmd) < 0.05:
        return None
    dt = np.median(np.diff(t))
    mejor, err_min = 0, np.inf
    for k in range(0, min(40, len(t) // 3)):
        e = np.nanmean(np.abs(med[k:] - cmd[: len(cmd) - k]))
        if e < err_min:
            mejor, err_min = k, e
    return mejor * dt * 1000.0


def analizar(carpeta):
    m = leer_muestras(carpeta)
    resumen = []
    if os.path.exists(os.path.join(carpeta, "resumen.json")):
        with open(os.path.join(carpeta, "resumen.json"), encoding="utf-8") as f:
            resumen = json.load(f)
    eventos = leer_jsonl(os.path.join(carpeta, "eventos.jsonl"))
    salida = os.path.join(carpeta, "informe")
    os.makedirs(salida, exist_ok=True)
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        plt = None

    inicio = next((e for e in eventos if e.get("evento") == "inicio"), {})
    seg = (inicio.get("config") or {}).get("seguridad", {})
    lim = seg.get("error_seguimiento_max_deg", 3.0)
    md = [f"# Informe de la sesión {os.path.basename(carpeta)}", ""]
    md.append(f"Modo: {inicio.get('modo', '?')}{' (robot real permitido)' if inicio.get('real') else ''}. Movimientos registrados: {len(resumen)}. Límite de seguimiento: {lim}°.")
    if inicio.get("modo") == "sim" or not inicio.get("real"):
        md.append("Nivel de evidencia: S0 si fue simulación interna o hardware simulado; S2/S3 solo si el registro muestra hardware real (ver eventos).")
    md.append("")
    abortos = [e for e in eventos if e.get("evento") == "aborto"]
    if abortos:
        md += ["## Abortos", ""] + [f"- {e['t']}: {e.get('motivo')}" for e in abortos] + [""]

    md += ["## Movimientos", "", "| Plan | Descripción | Estado | Duración [s] | Err. seguim. máx [°] | e_p máx [mm] | Error final [°] | e_p vs pose pedida [mm] | Retardo est. [ms] | Diagnóstico |", "|---|---|---|---|---|---|---|---|---|---|"]
    diagnosticos_globales = []
    for r in resumen:
        pid = r["plan_id"]
        diag = []
        ret = None
        if m is not None:
            sel = (m["plan_id"] == pid) & (m["estado"] == "ejecutando")
            if sel.sum() > 5:
                t = m["t_s"][sel] - m["t_s"][sel][0]
                Qc = np.column_stack([m[f"q{i}_cmd_deg"][sel] for i in range(1, 7)])
                Qm = np.column_stack([m[f"q{i}_med_deg"][sel] for i in range(1, 7)])
                E = Qm - Qc
                rets = []
                for j in range(6):
                    dq_c = np.diff(Qc[:, j])
                    dq_m = np.diff(Qm[:, j])
                    if np.ptp(Qc[:, j]) > 0.5:
                        c = np.corrcoef(dq_c, dq_m)[0, 1] if np.std(dq_m) > 0 and np.std(dq_c) > 0 else 1
                        if c < -0.3:
                            diag.append(f"J{j + 1} se mueve AL REVÉS de lo comandado (corr {c:.2f}): revisar signo")
                        rr = retardo_ms(t, Qc[:, j], Qm[:, j])
                        if rr is not None:
                            rets.append(rr)
                    fin = np.nanmean(E[-5:, j])
                    if abs(fin) > 0.1:
                        diag.append(f"J{j + 1} termina con {fin:+.2f}° de diferencia: posible offset o no convergió")
                if rets:
                    ret = float(np.median(rets))
                    if ret > 150:
                        diag.append(f"retardo alto ({ret:.0f} ms): revisar red/EGM o bajar velocidad")
                emax = np.nanmax(np.abs(E))
                if emax > 0.5 * lim:
                    j = int(np.nanargmax(np.nanmax(np.abs(E), axis=0))) + 1
                    diag.append(f"error alto en J{j} ({emax:.2f}°, más de la mitad del límite): bajar velocidad")
                if plt is not None:
                    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
                    for j in range(6):
                        ax[0].plot(t, Qc[:, j], "--", lw=1, color=f"C{j}")
                        ax[0].plot(t, Qm[:, j], lw=1.4, color=f"C{j}", label=f"J{j + 1}")
                        ax[1].plot(t, E[:, j], lw=1.2, color=f"C{j}", label=f"J{j + 1}")
                    ax[0].set_ylabel("q [°] (- - comandada)")
                    ax[0].legend(ncol=6, fontsize=8)
                    ax[1].axhline(lim, color="r", ls=":")
                    ax[1].axhline(-lim, color="r", ls=":")
                    ax[1].set_ylabel("e_q = medida - comandada [°]")
                    ax[1].legend(ncol=6, fontsize=8)
                    ax[2].plot(t, m["e_p_mm"][sel], label="e_p [mm]")
                    ax[2].plot(t, m["e_R_deg"][sel], label="e_R [°]")
                    ax[2].set_ylabel("error TCP")
                    ax[2].set_xlabel("t [s]")
                    ax[2].legend()
                    for a in ax:
                        a.grid(alpha=0.3)
                    fig.suptitle(f"Plan {pid}: {r.get('descripcion', '')}")
                    fig.tight_layout()
                    fig.savefig(os.path.join(salida, f"mov_{pid:03d}.png"), dpi=110)
                    plt.close(fig)
        if r.get("estado") == "abortado":
            diag.append("abortado: " + str(r.get("motivo")))
        diagnosticos_globales += diag
        f = lambda k, d=3: (f"{r[k]:.{d}f}" if isinstance(r.get(k), (int, float)) else "—")  # noqa: E731
        md.append(
            f"| {pid} | {r.get('descripcion', '')} | {r.get('estado')} | {f('duracion_s', 2)} | {f('err_seguimiento_max_deg')} | {f('e_p_seguimiento_max_mm', 2)} | "
            f"{f('error_final_max_deg', 4)} | {f('e_p_vs_pose_pedida_mm', 3)} | {'—' if ret is None else f'{ret:.0f}'} | {'; '.join(diag) or 'sin observaciones'} |"
        )

    if plt is not None and resumen:
        fig, ax = plt.subplots(figsize=(9, 3.6))
        ids = [str(r["plan_id"]) for r in resumen]
        ax.bar(ids, [r.get("err_seguimiento_max_deg", 0) or 0 for r in resumen], label="error de seguimiento máx [°]")
        ax.axhline(lim, color="r", ls=":", label="límite")
        ax.set_xlabel("plan")
        ax.legend()
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(salida, "resumen_errores.png"), dpi=110)
        plt.close(fig)
        md += ["", "![errores](resumen_errores.png)"]

    md += ["", "## Qué hacer con esto", ""]
    if not diagnosticos_globales:
        md.append("- No hubo observaciones: el seguimiento fue bueno en todos los movimientos.")
    else:
        for d in dict.fromkeys(diagnosticos_globales):
            md.append(f"- {d}")
    md += [
        "",
        "## Entradas de bitácora (copiar en 04_hito3_4_control_real/01_docs/05_bitacora/06_bitacora.md y completar)",
        "",
    ]
    for r in resumen:
        md += [
            "```",
            f"ID: MOV-{os.path.basename(carpeta)}-{r['plan_id']:03d}",
            f"fecha/hora: {r.get('inicio', '')}",
            "equipo y responsables: Grupo VEVAS (completar)",
            f"hipotesis: el robot sigue el plan '{r.get('descripcion', '')}' con error de seguimiento < {lim}°",
            "objetivo de la prueba: ejecutar el plan y medir el error en tiempo real",
            "software/hardware y versiones: consola VEVAS, ROS 2 Jazzy, abb_ros2 (completar commit)",
            f"modo: {inicio.get('modo', '?')}{' / movimiento autorizado' if inicio.get('real') else ''}",
            "condiciones / frames / parametros: base ABB, tool0, velocidad en eventos.jsonl",
            "prueba ejecutada: plan enviado desde la consola",
            f"resultado observable: estado {r.get('estado')}, error máx {r.get('err_seguimiento_max_deg', float('nan')):.3f}°, final {r.get('error_final_max_deg', float('nan')) if r.get('error_final_max_deg') is not None else float('nan'):.4f}°",
            f"evidencia: {os.path.basename(carpeta)}/informe/mov_{r['plan_id']:03d}.png, muestras.csv, video",
            "estado funcional: (functional / partially_verified / blocked / not verified)",
            "limitaciones / hipotesis descartadas: (completar)",
            "proxima accion y responsable: (completar)",
            "```",
            "",
        ]
    ruta = os.path.join(salida, "informe.md")
    with open(ruta, "w", encoding="utf-8") as fh:
        fh.write("\n".join(md) + "\n")
    print("\n".join(md[: 12 + len(resumen)]))
    print(f"\nInforme: {ruta}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("carpeta", nargs="?")
    ap.add_argument("--ultima", action="store_true", help="analizar la última sesión de ./sesiones")
    a = ap.parse_args()
    carpeta = a.carpeta
    if a.ultima or not carpeta:
        base = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sesiones")
        todas = sorted(glob.glob(os.path.join(base, "*")))
        if not todas:
            print("No hay sesiones guardadas.")
            sys.exit(1)
        carpeta = todas[-1]
    analizar(carpeta)


if __name__ == "__main__":
    main()
