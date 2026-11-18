"""Simulador interno del robot (no necesita ROS).

Imita lo que hace el joint_trajectory_controller + el robot:
    - recibe una trayectoria (tiempos y posiciones) y la interpola en el tiempo
    - recibe puntos sueltos (jog) y va hacia ellos
    - la posición "medida" sigue a la comandada con un retardo de primer orden
      (tau = 40 ms) y un ruido muy pequeño, para que el error en vivo se vea real

Sirve para ensayar la presentación en cualquier computadora (incluso Windows)
y para probar la consola antes de usar ROS. Todo lo que muestra es S0 (simulación).
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np


class BackendSim:
    tipo = "sim"

    def __init__(self, q_inicial_rad, tau: float = 0.04, ruido_deg: float = 0.003, hz: float = 250.0):
        self._q = np.asarray(q_inicial_rad, dtype=float).copy()
        self._ref = self._q.copy()
        self._tau = tau
        self._ruido = math.radians(ruido_deg)
        self._dt = 1.0 / hz
        self._tray = None  # (t0, ts, Q)
        self._jog = None  # (t0, q0, q1, horizonte)
        self._lock = threading.Lock()
        self._activo = False
        self._t_estado = time.monotonic()
        self._rng = np.random.default_rng(7)
        self.ultimo_resultado = None

    def info(self) -> dict:
        return {"tipo": "sim", "hardware": "simulador interno", "real": False, "texto": "SIMULACIÓN INTERNA (sin ROS)"}

    def iniciar(self):
        self._activo = True
        threading.Thread(target=self._bucle, daemon=True).start()

    def detener(self):
        self._activo = False

    def _bucle(self):
        while self._activo:
            t = time.monotonic()
            with self._lock:
                if self._tray is not None:
                    t0, ts, Q = self._tray
                    tt = t - t0
                    if tt >= ts[-1]:
                        self._ref = Q[-1].copy()
                        self._tray = None
                        self.ultimo_resultado = "terminada"
                    else:
                        k = int(np.searchsorted(ts, tt))
                        k = max(1, min(k, len(ts) - 1))
                        a = (tt - ts[k - 1]) / (ts[k] - ts[k - 1])
                        self._ref = Q[k - 1] + a * (Q[k] - Q[k - 1])
                elif self._jog is not None:
                    t0, q0, q1, h = self._jog
                    a = min(1.0, (t - t0) / h)
                    self._ref = q0 + a * (q1 - q0)
                alfa = 1.0 - math.exp(-self._dt / self._tau)
                self._q = self._q + alfa * (self._ref - self._q)
                self._t_estado = t
            time.sleep(self._dt)

    def leer(self):
        with self._lock:
            q = self._q + self._rng.normal(0.0, self._ruido, 6)
            return q, self._t_estado

    def ejecutar_trayectoria(self, ts, Q, QD=None) -> tuple[bool, str]:
        with self._lock:
            self._jog = None
            self._tray = (time.monotonic(), np.asarray(ts, dtype=float), np.asarray(Q, dtype=float))
            self.ultimo_resultado = None
        return True, "trayectoria aceptada por el simulador"

    def enviar_punto(self, q, horizonte: float) -> None:
        with self._lock:
            self._tray = None
            self._jog = (time.monotonic(), self._ref.copy(), np.asarray(q, dtype=float), max(horizonte, 1e-3))

    def mantener(self, q=None) -> None:
        """Parar y quedarse quieto (lo que hace el controlador al cancelar)."""
        with self._lock:
            self._tray = None
            self._jog = None
            self._ref = self._q.copy() if q is None else np.asarray(q, dtype=float)
            self.ultimo_resultado = "cancelada"

    def en_movimiento(self) -> bool:
        with self._lock:
            return self._tray is not None or self._jog is not None
