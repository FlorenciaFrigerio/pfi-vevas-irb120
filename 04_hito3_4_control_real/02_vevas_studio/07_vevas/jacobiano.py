"""Jacobiano geométrico del IRB 120, verificación por diferencias finitas y singularidades.

Convención (protocolo W12): twist con la parte lineal arriba y la angular abajo,
calculado en el marco base {0} y para el punto TCP.

    J_i = [ z_{i-1} x (p_TCP - o_{i-1}) ]      (junta de revolución)
          [ z_{i-1}                     ]

Las tres singularidades del IRB 120 (todas salen del determinante):
    muñeca  : sin(q5) = 0            (ejes 4 y 6 alineados)
    codo    : brazo y antebrazo efectivo alineados (cos(delta) = ±1 en Pieper)
    hombro  : el centro de muñeca cae sobre el eje 1 (r_w = 0)
"""

from __future__ import annotations

import math

import numpy as np

from .robot_irb120 import A2, A3, D4, D6, Robot

L3 = math.hypot(A3, D4)  # longitud efectiva junta 3 -> centro de muñeca
GAMMA = math.atan2(A3, D4)  # ángulo del antebrazo efectivo


def jacobiano(robot: Robot, q) -> np.ndarray:
    """J(q) 6x6 geométrico en la base, para el TCP."""
    M = robot.marcos(q)
    p = robot.fk_tcp(q)[:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z = M[i][:3, 2]
        o = M[i][:3, 3]
        J[:3, i] = np.cross(z, p - o)
        J[3:, i] = z
    return J


def _vee_log(R: np.ndarray) -> np.ndarray:
    """Vector de rotación (eje·ángulo) de una matriz de rotación pequeña."""
    cos_t = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    t = math.acos(cos_t)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if t < 1e-12:
        return 0.5 * w
    return t / (2.0 * math.sin(t)) * w


def jacobiano_numerico(robot: Robot, q, h: float = 1e-6) -> np.ndarray:
    """Jacobiano por diferencias finitas centrales.

    Parte lineal:  (p(q + h e_i) - p(q - h e_i)) / (2h)
    Parte angular: log(R(q + h e_i) R(q - h e_i)^T)^vee / (2h)
    """
    q = np.asarray(q, dtype=float)
    J = np.zeros((6, 6))
    for i in range(6):
        e = np.zeros(6)
        e[i] = h
        Tp = robot.fk_tcp(q + e)
        Tm = robot.fk_tcp(q - e)
        J[:3, i] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * h)
        J[3:, i] = _vee_log(Tp[:3, :3] @ Tm[:3, :3].T) / (2 * h)
    return J


def errores_jacobiano(robot: Robot, q, h: float = 1e-6) -> dict:
    """e_Jv [m/rad] y e_Jw [rad/rad] por separado (no se mezclan unidades)."""
    Ja = jacobiano(robot, q)
    Jn = jacobiano_numerico(robot, q, h)
    return {
        "e_Jv": float(np.linalg.norm(Ja[:3] - Jn[:3], ord="fro")),
        "e_Jw": float(np.linalg.norm(Ja[3:] - Jn[3:], ord="fro")),
    }


def curva_error_vs_h(robot: Robot, q, hs=None) -> list[tuple[float, float, float]]:
    """Curva error vs paso h: truncamiento O(h^2) a la derecha, redondeo a la izquierda."""
    if hs is None:
        hs = [10.0 ** (-k) for k in np.arange(1.0, 12.5, 0.5)]
    salida = []
    for h in hs:
        e = errores_jacobiano(robot, q, h)
        salida.append((float(h), e["e_Jv"], e["e_Jw"]))
    return salida


def analisis_singularidad(robot: Robot, q, L: float = 0.3) -> dict:
    """Valores singulares, rango, condición y diagnóstico de las 3 singularidades.

    L [m] es la longitud característica para escalar la parte lineal (J_v / L) y
    poder comparar filas de unidades distintas al calcular la condición.
    """
    J = jacobiano(robot, q)
    Js = J.copy()
    Js[:3] /= L
    s = np.linalg.svd(Js, compute_uv=False)
    eps = np.finfo(float).eps
    tol = max(Js.shape) * eps * s[0]
    rango = int(np.sum(s > tol))
    cond = float(s[0] / s[-1]) if s[-1] > 0 else float("inf")

    # Indicadores geométricos de cada singularidad
    M = robot.marcos(q)
    p6 = M[6][:3, 3]
    z6 = M[6][:3, 2]
    pw = p6 - D6 * z6
    r_w = math.hypot(pw[0], pw[1])
    cos_delta = math.cos(q[2] - GAMMA + math.pi / 2)
    ind_muneca = abs(math.sin(q[4]))
    ind_codo = math.sqrt(max(0.0, 1.0 - cos_delta**2))  # = |sin(delta)|
    ind_hombro = r_w
    return {
        "sigma": [float(v) for v in s],
        "sigma_min": float(s[-1]),
        "rango": rango,
        "tolerancia_rango": float(tol),
        "condicion": cond,
        "det_J": float(np.linalg.det(J)),
        "L_m": L,
        "muneca_sin_q5": ind_muneca,
        "codo_sin_delta": ind_codo,
        "hombro_r_w_m": ind_hombro,
        "J": J.tolist(),
    }


def determinante_analitico(q) -> float:
    """det J (para la brida, sin herramienta) en forma cerrada.

    det J = -a2 · L3 · sin(delta) · r_w · sin(q5)   (con el signo de esta tabla D-H)
    donde delta = q3 - gamma + pi/2 y r_w es la distancia del centro de muñeca al eje 1.
    Se verifica numéricamente en los tests (el signo depende de la convención).
    """
    q2, q3, q5 = q[1], q[2], q[4]
    delta = q3 - GAMMA + math.pi / 2
    r_w = A2 * math.sin(q2) + L3 * math.cos(q2 + q3 - GAMMA)
    return -A2 * L3 * math.sin(delta) * r_w * math.sin(q5)
