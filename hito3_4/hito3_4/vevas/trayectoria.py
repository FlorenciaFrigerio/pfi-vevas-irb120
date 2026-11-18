"""Trayectorias LSPB (perfil trapezoidal) sincronizadas para las 6 juntas.

Para cada junta i con desplazamiento D_i = q_f,i - q_0,i:

    tramo 1 (0 <= t < tc):        q = q0 + (A/2) t^2            (acelera)
    tramo 2 (tc <= t < tf - tc):  q = q0 + V (t - tc/2)         (velocidad constante)
    tramo 3 (tf - tc <= t <= tf): q = qf - (A/2) (tf - t)^2     (frena)

    con V = D / (tf - tc)  y  A = V / tc.

Sincronización: todas las juntas usan el MISMO tf y el MISMO tc, así llegan juntas
y el TCP no hace "eses". Se fija tc = kappa · tf (kappa = 0.25 por defecto), entonces

    V_i = D_i / ((1 - kappa) tf)   <= V_max,i   ->  tf >= |D_i| / ((1 - kappa) V_max,i)
    A_i = V_i / (kappa tf)          <= A_max,i   ->  tf >= sqrt(|D_i| / (kappa (1 - kappa) A_max,i))

y tf es el mayor de todos esos mínimos (la junta más exigida manda), con un mínimo
absoluto t_min para que los movimientos cortos no sean bruscos.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class LSPB:
    q0: np.ndarray
    qf: np.ndarray
    tf: float
    tc: float
    V: np.ndarray
    A: np.ndarray
    junta_limitante: int
    motivo_tf: str

    def evaluar(self, t: float):
        """(q, qd, qdd) en el instante t [s]."""
        t = min(max(t, 0.0), self.tf)
        tf, tc = self.tf, self.tc
        if tf <= 0:
            return self.qf.copy(), np.zeros(6), np.zeros(6)
        if t < tc:
            q = self.q0 + 0.5 * self.A * t * t
            qd = self.A * t
            qdd = self.A.copy()
        elif t <= tf - tc:
            q = self.q0 + self.V * (t - tc / 2.0)
            qd = self.V.copy()
            qdd = np.zeros(6)
        else:
            dt = tf - t
            q = self.qf - 0.5 * self.A * dt * dt
            qd = self.A * dt
            qdd = -self.A
        return q, qd, qdd

    def muestrear(self, dt: float = 0.02):
        n = max(2, int(math.ceil(self.tf / dt)) + 1)
        ts = np.linspace(0.0, self.tf, n)
        Q = np.zeros((n, 6))
        QD = np.zeros((n, 6))
        QDD = np.zeros((n, 6))
        for k, t in enumerate(ts):
            Q[k], QD[k], QDD[k] = self.evaluar(t)
        return ts, Q, QD, QDD

    def a_dict(self, dt_muestras: float = 0.05) -> dict:
        ts, Q, QD, QDD = self.muestrear(dt_muestras)
        return {
            "tf": self.tf,
            "tc": self.tc,
            "V_deg_s": np.degrees(self.V).tolist(),
            "A_deg_s2": np.degrees(self.A).tolist(),
            "D_deg": np.degrees(self.qf - self.q0).tolist(),
            "junta_limitante": self.junta_limitante,
            "motivo_tf": self.motivo_tf,
            "t": ts.tolist(),
            "q_deg": np.degrees(Q).tolist(),
            "qd_deg_s": np.degrees(QD).tolist(),
            "qdd_deg_s2": np.degrees(QDD).tolist(),
        }


def planificar(q0, qf, vel_max, factor_vel: float, t_acel: float = 0.5, kappa: float = 0.25, t_min: float = 1.0) -> LSPB:
    """LSPB sincronizado.

    vel_max:    velocidades máximas de hoja de datos [rad/s]
    factor_vel: fracción de esas velocidades que se permite (0.05 = 5 %)
    t_acel:     tiempo para llegar a la velocidad máxima permitida -> A_max = V_max / t_acel
    """
    q0 = np.asarray(q0, dtype=float)
    qf = np.asarray(qf, dtype=float)
    D = qf - q0
    vmax = np.asarray(vel_max, dtype=float) * factor_vel
    amax = vmax / t_acel
    candidatos = [(t_min, -1, f"tiempo mínimo {t_min:.1f} s")]
    for i in range(6):
        d = abs(D[i])
        if d < 1e-12:
            continue
        t_v = d / ((1.0 - kappa) * vmax[i])
        t_a = math.sqrt(d / (kappa * (1.0 - kappa) * amax[i]))
        candidatos.append((t_v, i, f"velocidad de J{i + 1}"))
        candidatos.append((t_a, i, f"aceleración de J{i + 1}"))
    tf, limitante, motivo = max(candidatos, key=lambda c: c[0])
    tc = kappa * tf
    V = D / (tf - tc)
    A = V / tc
    return LSPB(q0, qf, tf, tc, V, A, limitante + 1 if limitante >= 0 else 0, motivo)


def comprobar_continuidad(tray: LSPB, dt: float = 0.001) -> dict:
    """Saltos máximos entre muestras: q y qd deben ser continuas; qdd salta en tc y tf - tc."""
    ts, Q, QD, QDD = tray.muestrear(dt)
    dq = np.max(np.abs(np.diff(Q, axis=0)), axis=0)
    dqd = np.max(np.abs(np.diff(QD, axis=0)), axis=0)
    return {
        "salto_q_max_deg": float(np.degrees(dq.max())),
        "salto_qd_max_deg_s": float(np.degrees(dqd.max())),
        "llega_a_qf_deg": float(np.degrees(np.max(np.abs(Q[-1] - tray.qf)))),
        "qd_final_deg_s": float(np.degrees(np.max(np.abs(QD[-1])))),
    }
