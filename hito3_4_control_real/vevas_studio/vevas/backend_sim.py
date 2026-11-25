"""Simulador interno del robot (no necesita ROS).

Imita lo que hace el joint_trajectory_controller + el robot:
    - interpola trayectorias por tiempo con las derivadas disponibles
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
        self._tray = None  # (t0, ts, destino, coeficientes)
        self._jog = None  # (t0, q0, q1, horizonte)
        self._lock = threading.Lock()
        self._activo = False
        self._t_estado = time.monotonic()
        self._t_integracion = self._t_estado
        self._rng = np.random.default_rng(7)
        self.ultimo_resultado = None

    def info(self) -> dict:
        return {"tipo": "sim", "hardware": "simulador interno", "real": False, "texto": "SIMULACIÓN INTERNA (sin ROS)"}

    def iniciar(self):
        self._t_integracion = time.monotonic()
        self._activo = True
        threading.Thread(target=self._bucle, daemon=True).start()

    def detener(self):
        self._activo = False

    def _bucle(self):
        while self._activo:
            with self._lock:
                t = time.monotonic()
                if self._tray is not None:
                    t0, ts, destino, coeficientes = self._tray
                    tt = t - t0
                    if tt >= ts[-1]:
                        self._ref = destino
                        self._tray = None
                        self.ultimo_resultado = "terminada"
                    else:
                        tt = max(0.0, tt)
                        k = int(np.searchsorted(ts, tt, side="right")) - 1
                        u = (tt - ts[k]) / (ts[k + 1] - ts[k])
                        referencia = coeficientes[-1][k]
                        for coeficiente in reversed(coeficientes[:-1]):
                            referencia = referencia * u + coeficiente[k]
                        self._ref = referencia
                elif self._jog is not None:
                    t0, q0, q1, h = self._jog
                    a = min(1.0, (t - t0) / h)
                    self._ref = q0 + a * (q1 - q0)
                # La referencia avanza con el reloj real. El filtro debe usar
                # el mismo tiempo transcurrido, incluso si el sistema retrasa
                # este hilo; usar siempre _dt acumulaba un retraso ficticio.
                transcurrido = max(0.0, t - self._t_integracion)
                self._t_integracion = t
                alfa = 1.0 - math.exp(-transcurrido / self._tau)
                self._q = self._q + alfa * (self._ref - self._q)
                self._t_estado = t
            time.sleep(self._dt)

    def leer(self):
        with self._lock:
            q = self._q + self._rng.normal(0.0, self._ruido, 6)
            return q, self._t_estado

    def ejecutar_trayectoria(self, ts, Q, QD=None, *, QDD=None) -> tuple[bool, str]:
        """Interpola linealmente o por Hermite con velocidad y aceleración."""
        try:
            ts = np.array(ts, dtype=float)
            Q = np.asarray(Q, dtype=float)
            QD = np.asarray(QD, dtype=float) if QD is not None else None
            QDD = np.asarray(QDD, dtype=float) if QDD is not None else None
        except (TypeError, ValueError, OverflowError):
            return False, "Muestras de trayectoria inválidas."
        if (ts.ndim != 1 or not ts.size or not np.all(np.isfinite(ts))
                or ts[0] < 0 or np.any(np.diff(ts) <= 0)):
            return False, "La trayectoria necesita tiempos finitos, crecientes y no negativos."
        muestras = (Q, QD, QDD)
        if any(x is not None and (x.shape != (ts.size, self._q.size) or not np.all(np.isfinite(x)))
               for x in muestras) or (QDD is not None and QD is None):
            return False, "Las posiciones y derivadas de la trayectoria no coinciden con sus tiempos."

        with self._lock:
            if ts[0] > 0:
                # Los llamadores pueden enviar solo puntos futuros. El intervalo
                # hasta el primero comienza en la referencia actual, en reposo.
                ts = np.concatenate(([0.0], ts))
                Q = np.vstack((self._ref, Q))
                if QD is not None:
                    QD = np.vstack((np.zeros_like(self._ref), QD))
                if QDD is not None:
                    QDD = np.vstack((np.zeros_like(self._ref), QDD))
            with np.errstate(over="ignore", invalid="ignore"):
                coeficientes = _coeficientes(ts, Q, QD, QDD)
            if not all(np.all(np.isfinite(c)) for c in coeficientes):
                return False, "La interpolación de la trayectoria produjo valores no finitos."
            self._jog = None
            self._tray = (time.monotonic(), ts, Q[-1].copy(), coeficientes)
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


def _coeficientes(ts, Q, QD, QDD):
    """Coeficientes en potencias crecientes del tiempo normalizado de cada intervalo."""
    posicion = Q[:-1].copy()
    delta = Q[1:] - posicion
    if QD is None:
        return posicion, delta
    h = np.diff(ts)[:, None]
    v0, v1 = h * QD[:-1], h * QD[1:]
    if QDD is None:
        return posicion, v0, 3 * delta - 2 * v0 - v1, -2 * delta + v0 + v1
    a0, a1 = h**2 * QDD[:-1], h**2 * QDD[1:]
    resta_q = delta - v0 - a0 / 2
    resta_v = v1 - v0 - a0
    resta_a = a1 - a0
    return (posicion, v0, a0 / 2,
            10 * resta_q - 4 * resta_v + resta_a / 2,
            -15 * resta_q + 7 * resta_v - resta_a,
            6 * resta_q - 3 * resta_v + resta_a / 2)
