"""Velocidad de las juntas: tope del slider y velocidad MEDIDA a partir de las posiciones recibidas.

Tope del slider («Velocidad del plan», % de la hoja de datos: 250/250/250/320/320/420 °/s)
    0 < v ≤ VEL_PCT_MAX (50 %) en SIM y con robot real; por defecto 5 %. config.json, el GO y la consola
    usan la misma validación (pct_valido).
    Con el robot real por RWS el RAPID (C02) limita cada junta a vLim = 5 % de la ficha
    (12.5/12.5/12.5/16/16/21 °/s) con t = max(piso, 2.5·|Δq_i|/vLim_i): aunque el plan pida más, el robot
    no va más rápido; solo tarda más. Por encima de VEL_PCT_VERIFICADO (10 %) la interfaz lo marca como
    «solo simulación / fuera de lo verificado».

Velocidad medida
    Diferencia finita robusta: pendiente por mínimos cuadrados de q(t) sobre las muestras de los últimos
    ~0.3 s, con la marca de tiempo de cada muestra. Las muestras repetidas (misma marca de tiempo: el bucle de
    la consola corre a 50 Hz y RWS entrega ~10 Hz) se descartan; las irregulares pesan por su tiempo real.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

VEL_PCT_MAX = 50.0          # tope absoluto del slider, en SIM y con robot real
VEL_PCT_DEFECTO = 5.0       # valor inicial y Enter de crear_go
VEL_PCT_VERIFICADO = 10.0   # lo ensayado con el robot real; por encima: solo simulación / fuera de lo verificado

CLAVES_PCT = ("velocidad_sim_max_pct", "velocidad_real_max_pct", "velocidad_inicial_pct")


def pct_valido(v) -> bool:
    """0 < v ≤ 50 (número finito, no booleano)."""
    return (not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v)
            and 0.0 < float(v) <= VEL_PCT_MAX)


def validar_config_velocidad(seguridad: dict) -> None:
    """Falla con un mensaje claro si config.json pide un tope fuera de 0 < v ≤ 50 %."""
    for k in CLAVES_PCT:
        if not pct_valido(seguridad.get(k)):
            raise ValueError(f"config.json: seguridad.{k} = {seguridad.get(k)!r}; debe ser un número 0 < v ≤ {VEL_PCT_MAX:g} %.")
    if seguridad["velocidad_inicial_pct"] > min(seguridad["velocidad_sim_max_pct"], seguridad["velocidad_real_max_pct"]):
        raise ValueError("config.json: seguridad.velocidad_inicial_pct no puede superar los topes de SIM ni de robot real.")


class MedidorVelocidad:
    """Velocidad articular medida [°/s] por regresión lineal en una ventana corta, y máximo del último movimiento."""

    def __init__(self, ventana_s: float = 0.3, edad_max_s: float = 1.0, span_min_s: float = 0.04):
        self.ventana_s = float(ventana_s)
        self.edad_max_s = float(edad_max_s)    # si la última muestra es más vieja, la velocidad es desconocida
        self.span_min_s = float(span_min_s)    # menos tiempo entre muestras amplifica el ruido: no se estima
        self._t: deque = deque()
        self._q: deque = deque()
        self.max_mov = np.zeros(6)
        self.ultima = None

    def agregar(self, t, q_deg) -> bool:
        """Agrega una muestra (t [s], q [°]). Devuelve False si es repetida, desordenada o inválida."""
        if t is None or q_deg is None:
            return False
        t = float(t)
        q = np.asarray(q_deg, dtype=float).reshape(-1)
        if not math.isfinite(t) or q.shape != (6,) or not np.all(np.isfinite(q)):
            return False
        if self._t and t <= self._t[-1] + 1e-9:
            return False
        self._t.append(t)
        self._q.append(q)
        # se conservan al menos dos muestras para estimar aunque lleguen espaciadas
        while len(self._t) > 2 and self._t[0] < t - self.ventana_s:
            self._t.popleft()
            self._q.popleft()
        v = self._estimar()
        self.ultima = v
        if v is not None:
            self.max_mov = np.maximum(self.max_mov, np.abs(v))
        return True

    def _estimar(self):
        if len(self._t) < 2:
            return None
        t = np.fromiter(self._t, dtype=float)
        if t[-1] - t[0] < self.span_min_s or t[-1] - t[0] > self.ventana_s + self.edad_max_s:
            return None
        Q = np.vstack(self._q)
        tc = t - t.mean()
        den = float(tc @ tc)
        if den <= 0.0:
            return None
        return (tc @ (Q - Q.mean(axis=0))) / den

    def velocidad(self, ahora: float | None = None):
        """Última velocidad estimada [°/s] (array de 6) o None si no hay datos suficientes o frescos."""
        if self.ultima is None or not self._t:
            return None
        if ahora is not None and ahora - self._t[-1] > self.edad_max_s:
            return None
        return self.ultima.copy()

    def reiniciar_maximos(self) -> None:
        self.max_mov = np.zeros(6)

    def reiniciar(self) -> None:
        self._t.clear()
        self._q.clear()
        self.ultima = None
