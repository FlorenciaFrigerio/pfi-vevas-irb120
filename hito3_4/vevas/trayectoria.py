"""Trayectorias LSPB y quínticas para seis juntas.

Definición del perfil trapezoidal LSPB:

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

    perfil = "lspb"

    def a_dict(self, dt_muestras: float = 0.05) -> dict:
        return {**_serializar(self, dt_muestras), "perfil": self.perfil}


def _serializar(tray, dt_muestras):
    ts, Q, QD, QDD = tray.muestrear(dt_muestras)
    return {
        "tf": tray.tf,
        "tc": tray.tc,
        "V_deg_s": np.degrees(tray.V).tolist(),
        "A_deg_s2": np.degrees(tray.A).tolist(),
        "D_deg": np.degrees(tray.qf - tray.q0).tolist(),
        "junta_limitante": tray.junta_limitante,
        "motivo_tf": tray.motivo_tf,
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


@dataclass
class TrayectoriaQuintica:
    """Hermite de grado cinco por tramo; V y A son picos absolutos en rad/s y rad/s²."""
    puntos: np.ndarray
    _tiempos: np.ndarray
    tiempos_nudos: np.ndarray
    _coeficientes: tuple
    V: np.ndarray
    A: np.ndarray
    junta_limitante: int
    motivo_tf: str
    perfil: str
    tc = None

    @property
    def q0(self):
        return self.puntos[0]

    @property
    def qf(self):
        return self.puntos[-1]

    @property
    def tf(self):
        return float(self._tiempos[-1])

    def _evaluar(self, tiempos):
        t = np.clip(tiempos, 0.0, self.tf)
        tramo = np.minimum(np.searchsorted(self._tiempos, t, side="right") - 1, len(self.puntos) - 2)
        h = (self._tiempos[tramo + 1] - self._tiempos[tramo])[..., None]
        u = (t - self._tiempos[tramo])[..., None] / h
        d0, c3, c4, c5 = (c[tramo] for c in self._coeficientes)
        q = self.puntos[tramo] + u * (d0 + u**2 * (c3 + u * (c4 + u * c5)))
        qd = (d0 + u**2 * (3*c3 + u * (4*c4 + u * 5*c5))) / h
        qdd = u * (1 - u) * (6*c3 - 20*c5*u) / h**2
        final = (t == self.tf)[..., None]
        return np.where(final, self.qf, q), np.where(final, 0.0, qd), qdd

    def evaluar(self, t: float):
        return self._evaluar(np.asarray(t, dtype=float))

    def muestrear(self, dt: float = 0.02):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("El período de muestreo debe ser finito y positivo.")
        ts = [np.array([0.0])]
        for inicio, fin in zip(self._tiempos, self._tiempos[1:]):
            pasos = max(1, math.ceil((fin - inicio) / dt))
            ts.append(np.linspace(inicio, fin, pasos + 1)[1:])
        tiempos = np.concatenate(ts)
        return tiempos, *self._evaluar(tiempos)

    def a_dict(self, dt_muestras: float = 0.05):
        return {**_serializar(self, dt_muestras), "perfil": self.perfil, "tiempos_nudos": self.tiempos_nudos.tolist()}


def _coeficientes_quinticos(delta, duracion, paradas):
    h = duracion[:, None]
    secantes = delta / h
    tangentes = np.zeros((len(delta) + 1, 6))
    # Cada eje que cambia de sentido tiene tangente cero. Los demás atraviesan
    # el nudo con la misma velocidad física por ambos lados.
    antes, despues = secantes[:-1], secantes[1:]
    central = (h[1:] * antes + h[:-1] * despues) / (h[1:] + h[:-1])
    limite = 1.25 * np.minimum(abs(antes), abs(despues))
    tangentes[1:-1] = np.where(np.sign(antes) == np.sign(despues), np.clip(central, -limite, limite), 0.0)
    tangentes[paradas] = 0.0
    d0, d1 = h * tangentes[:-1], h * tangentes[1:]
    # p(u) = q0 + d0*u + c3*u³ + c4*u⁴ + c5*u⁵, u entre 0 y 1.
    # p''(0)=p''(1)=0 da continuidad C2 al unir los segmentos.
    c3 = 10*delta - 6*d0 - 4*d1
    c4 = -15*delta + 8*d0 + 7*d1
    c5 = 6*delta - 3*d0 - 3*d1
    return d0, c3, c4, c5


def _raiz_en_intervalo(numerador, denominador):
    raiz = np.divide(numerador, denominador, out=np.zeros_like(numerador), where=denominador != 0)
    return np.clip(raiz, 0.0, 1.0)


def _picos_quinticos(coef, duracion):
    """Extremos analíticos por tramo y eje, incluidos los extremos del intervalo."""
    d0, c3, c4, c5 = coef
    h = duracion[:, None]
    # q''(u) = u*(1-u)*(6*c3 - 20*c5*u).
    u = _raiz_en_intervalo(6*c3, 20*c5)
    derivada = d0 + u**2 * (3*c3 + u * (4*c4 + u * 5*c5))
    velocidad = np.maximum.reduce([abs(d0), abs(d0 + 3*c3 + 4*c4 + 5*c5), abs(derivada)]) / h
    # Los extremos de aceleración están en las raíces de la tercera derivada.
    a, b, c = 60*c5, 24*c4, 6*c3
    discriminante = np.maximum(b*b - 4*a*c, 0.0)
    numerador = -0.5 * (b + np.copysign(np.sqrt(discriminante), b))
    aceleracion = np.zeros_like(velocidad)
    for u in (_raiz_en_intervalo(numerador, a), _raiz_en_intervalo(c, numerador)):
        aceleracion = np.maximum(aceleracion, abs(u * (1-u) * (6*c3 - 20*c5*u)))
    return velocidad, aceleracion / h**2


def _plan_quintico(Q, vel_max, factor_vel, t_acel, t_min, perfil, paradas=()):
    puntos = np.asarray(Q, dtype=float)
    limites = np.asarray(vel_max, dtype=float)
    if puntos.ndim != 2 or puntos.shape[0] < 2 or puntos.shape[1] != 6 or not np.isfinite(puntos).all():
        raise ValueError("El camino necesita al menos dos posturas de seis valores finitos.")
    if limites.shape != (6,) or not np.isfinite(limites).all() or np.any(limites <= 0):
        raise ValueError("Se necesitan seis límites de velocidad positivos y finitos.")
    if not 0 < factor_vel <= 1 or not all(math.isfinite(t) and t > 0 for t in (t_acel, t_min)):
        raise ValueError("El factor debe estar en (0, 1] y los tiempos deben ser positivos y finitos.")
    vmax = limites * factor_vel
    amax = vmax / t_acel

    # Omitir duplicados consecutivos no añade una pausa. Los tiempos públicos
    # conservan la correspondencia con todos los puntos de la entrada.
    distintos = np.r_[True, np.any(np.diff(puntos, axis=0) != 0, axis=1)]
    indice_original = np.cumsum(distintos) - 1
    vertices = puntos[distintos]
    estacionaria = len(vertices) == 1
    if estacionaria:
        vertices = np.repeat(vertices, 2, axis=0)
    detener = np.zeros(len(vertices), dtype=bool)
    for k in (() if paradas is None else paradas):
        if isinstance(k, (bool, np.bool_)) or not isinstance(k, (int, np.integer)) or not 0 <= k < len(puntos):
            raise ValueError("Cada parada debe ser un índice válido de las posturas del camino.")
        detener[indice_original[k]] = True

    delta = np.diff(vertices, axis=0)
    # Medimos los tramos en el espacio de juntas normalizado por vmax. Las
    # pasadas desde el principio y el final reservan tiempo para acelerar y
    # frenar en las paradas. Se usa un solo escalado final para respetar los
    # picos analíticos sin alterar la forma ni crear cambios bruscos de ritmo.
    delta_normalizada = delta / vmax
    longitudes = np.linalg.norm(delta_normalizada, axis=1)
    ritmo2 = np.ones(len(vertices))
    # Una curva cerrada o una cúspide puede exigir aceleración aunque la rapidez
    # sea constante. Reservar tiempo localmente según el cambio de dirección
    # evita que el punto de mayor curvatura imponga su ritmo a todo el trazado.
    direccion = np.divide(delta_normalizada, longitudes[:, None], out=np.zeros_like(delta),
                          where=longitudes[:, None] > 0)
    giro = np.max(np.abs(np.diff(direccion, axis=0)), axis=1)
    distancia_giro = np.minimum(longitudes[:-1], longitudes[1:])
    ritmo2[1:-1] = np.minimum(1.0, np.divide(distancia_giro, t_acel * giro,
                                             out=np.ones_like(giro), where=giro > 0))
    ritmo2[detener] = 0.0
    ritmo2[[0, -1]] = 0.0
    for i, largo in enumerate(longitudes):
        ritmo2[i + 1] = min(ritmo2[i + 1], ritmo2[i] + 2*largo/t_acel)
    for i in range(len(longitudes) - 1, -1, -1):
        ritmo2[i] = min(ritmo2[i], ritmo2[i + 1] + 2*longitudes[i]/t_acel)
    ritmo = np.sqrt(ritmo2)
    suma = ritmo[:-1] + ritmo[1:]
    desde_reposo = np.max(np.abs(delta_normalizada), axis=1)
    duracion = np.divide(2*longitudes, suma, out=desde_reposo, where=suma > 0)
    duracion = np.maximum(duracion, 1e-8)
    coef = _coeficientes_quinticos(delta, duracion, detener)
    vel, acel = _picos_quinticos(coef, duracion)
    razon_vel = vel.max(axis=0) / vmax
    razon_acel = np.sqrt(acel.max(axis=0) / amax)
    por_eje = np.maximum(razon_vel, razon_acel)
    eje = int(np.argmax(por_eje))
    escala = max(1.0, float(por_eje[eje]))
    if t_min >= duracion.sum() * escala:
        escala = t_min / duracion.sum()
        limitante, motivo = 0, f"tiempo mínimo {t_min:g} s"
    else:
        limite = "velocidad" if razon_vel[eje] >= razon_acel[eje] else "aceleración"
        limitante, motivo = eje + 1, f"{limite} de J{eje + 1}"
    tiempos = np.r_[0.0, np.cumsum(duracion * escala)]
    tiempos_originales = np.linspace(0, tiempos[-1], len(puntos)) if estacionaria else tiempos[indice_original]
    return TrayectoriaQuintica(vertices, tiempos, tiempos_originales, coef, vel.max(axis=0)/escala,
                              acel.max(axis=0)/escala**2, limitante, motivo, perfil)


def planificar_suave(q0, qf, vel_max, factor_vel: float, t_acel: float = 0.5,
                     kappa: float = 0.25, t_min: float = 1.0) -> TrayectoriaQuintica:
    """Movimiento punto a punto s(u)=10u³-15u⁴+6u⁵. kappa solo se usa en el perfil LSPB."""
    return _plan_quintico([q0, qf], vel_max, factor_vel, t_acel, t_min, "quintico")


def planificar_camino(Q, vel_max, factor_vel: float, t_acel: float = 0.5,
                     t_min: float = 1.0, *, paradas=None) -> TrayectoriaQuintica:
    """Interpola posturas IK en radianes; paradas contiene los índices de esquinas con reposo exacto.

    Cada junta se mantiene dentro del intervalo de sus dos valores consecutivos.
    En forma Bézier los puntos de control son q0, q0+d0/5, q0+2d0/5,
    q1-2d1/5, q1-d1/5, q1. El límite de tangente conserva su orden.
    """
    return _plan_quintico(Q, vel_max, factor_vel, t_acel, t_min, "camino_quintico", paradas)


def comprobar_continuidad(tray: LSPB | TrayectoriaQuintica, dt: float = 0.001) -> dict:
    """Máxima diferencia entre muestras consecutivas y errores al finalizar."""
    ts, Q, QD, QDD = tray.muestrear(dt)
    dq = np.max(np.abs(np.diff(Q, axis=0)), axis=0)
    dqd = np.max(np.abs(np.diff(QD, axis=0)), axis=0)
    return {
        "salto_q_max_deg": float(np.degrees(dq.max())),
        "salto_qd_max_deg_s": float(np.degrees(dqd.max())),
        "llega_a_qf_deg": float(np.degrees(np.max(np.abs(Q[-1] - tray.qf)))),
        "qd_final_deg_s": float(np.degrees(np.max(np.abs(QD[-1])))),
    }
