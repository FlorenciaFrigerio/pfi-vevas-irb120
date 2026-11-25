"""Registro de cada sesión de la consola (evidencia para la bitácora).

Cada vez que se abre la consola se crea una carpeta:

    sesiones/AAAA-MM-DD_HH-MM-SS_<modo>/
        eventos.jsonl   un evento por línea (cálculos, planes, envíos, abortos, jog, armado)
        muestras.csv    estado cada 20 ms durante el movimiento y cada 0.5 s en reposo
        planes/         el cálculo completo de cada plan (8 ramas, pasos, LSPB) en JSON
        resumen.json    métricas de cada movimiento ejecutado (error máximo, final, etc.)

02_herramientas/analizar_sesion.py lee esta carpeta y arma el informe con gráficas.
"""

from __future__ import annotations

import csv
import datetime as dt
import json
import math
import os
import threading


def _json_seguro(o):
    if hasattr(o, "item") and not hasattr(o, "__len__"):
        o = o.item()
    elif hasattr(o, "tolist"):
        o = o.tolist()
    if isinstance(o, float) and not math.isfinite(o):
        return None
    if isinstance(o, dict):
        return {k: _json_seguro(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_json_seguro(v) for v in o]
    return o


class Registro:
    def __init__(self, carpeta_base: str, modo: str):
        stamp = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.carpeta = os.path.join(carpeta_base, f"{stamp}_{modo}")
        os.makedirs(os.path.join(self.carpeta, "planes"), exist_ok=True)
        self._lock = threading.Lock()
        self._ev = open(os.path.join(self.carpeta, "eventos.jsonl"), "a", encoding="utf-8")
        self._csv_f = open(os.path.join(self.carpeta, "muestras.csv"), "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._csv_f)
        cab = ["t_s", "estado", "plan_id"]
        cab += [f"q{i}_cmd_deg" for i in range(1, 7)]
        cab += [f"q{i}_med_deg" for i in range(1, 7)]
        cab += [f"e{i}_deg" for i in range(1, 7)]
        cab += ["e_p_mm", "e_R_deg", "x_mm", "y_mm", "z_mm", "sigma_min", "vel_pct"]
        cab += [f"v{i}_med_deg_s" for i in range(1, 7)]   # velocidad medida (regresión ~0.3 s), la misma de la interfaz
        self._csv.writerow(cab)
        self.resumenes = []
        self._n = 0

    def evento(self, evento: str, **datos):
        linea = {"t": dt.datetime.now().isoformat(timespec="milliseconds"), "evento": evento, **datos}
        with self._lock:
            self._ev.write(json.dumps(_json_seguro(linea), ensure_ascii=False) + "\n")
            self._ev.flush()

    def muestra(self, fila: list):
        with self._lock:
            self._csv.writerow(fila)
            self._n += 1
            if self._n % 25 == 0:
                self._csv_f.flush()

    def guardar_plan(self, plan_id: int, datos: dict):
        ruta = os.path.join(self.carpeta, "planes", f"plan_{plan_id:03d}.json")
        with open(ruta, "w", encoding="utf-8") as f:
            json.dump(_json_seguro(datos), f, ensure_ascii=False, indent=1)

    def agregar_resumen(self, r: dict):
        with self._lock:
            self.resumenes.append(_json_seguro(r))
            with open(os.path.join(self.carpeta, "resumen.json"), "w", encoding="utf-8") as f:
                json.dump(self.resumenes, f, ensure_ascii=False, indent=1)

    def cerrar(self):
        with self._lock:
            for f in (self._ev, self._csv_f):
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass
