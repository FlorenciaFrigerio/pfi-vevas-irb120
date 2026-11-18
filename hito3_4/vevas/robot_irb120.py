"""Modelo cinemático del ABB IRB 120-3/0.6 (Grupo VEVAS).

Tabla D-H estándar (convención Truc & Lam, la misma del Hito 1):

    i |  a_i [m] | alpha_i  | d_i [m] | theta_i
    1 |  0       | -pi/2    | 0.290   | q1
    2 |  0.270   |  0       | 0       | q2 - pi/2   (offset: con q2 = 0 el brazo queda vertical)
    3 |  0.070   | -pi/2    | 0       | q3
    4 |  0       | +pi/2    | 0.302   | q4
    5 |  0       | -pi/2    | 0       | q5
    6 |  0       |  0       | 0.072   | q6

Con q = 0 la brida queda en p = [0.374, 0, 0.630] m (coincide con el manual de ABB).

Marcos que se usan:
    {0}     base del robot (la del controlador ABB, "Base")
    {6}     marco 6 de la tabla D-H (en la brida)
    {tool0} marco de la brida según ABB. tool0 = {6} girado 180° en z6.
            Así, con q = 0, tool0 tiene z hacia +x de la base y x hacia abajo,
            igual que el robtarget [0.7071, 0, 0.7071, 0] del FlexPendant.
    {tcp}   punta de la herramienta. Por defecto tcp = tool0 (sin herramienta).

Matriz de cada eslabón (D-H estándar):
    A_i = Rz(theta_i) · Tz(d_i) · Tx(a_i) · Rx(alpha_i)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

PI = math.pi

# Tabla D-H: (a [m], alpha [rad], d [m], offset de theta [rad])
DH = (
    (0.000, -PI / 2, 0.290, 0.0),
    (0.270, 0.0, 0.000, -PI / 2),
    (0.070, -PI / 2, 0.000, 0.0),
    (0.000, PI / 2, 0.302, 0.0),
    (0.000, -PI / 2, 0.000, 0.0),
    (0.000, 0.0, 0.072, 0.0),
)

D1, A2, A3, D4, D6 = 0.290, 0.270, 0.070, 0.302, 0.072

# Límites articulares de la hoja de datos del IRB 120 [grados]
LIMITES_DEG = (
    (-165.0, 165.0),
    (-110.0, 110.0),
    (-110.0, 70.0),
    (-160.0, 160.0),
    (-120.0, 120.0),
    (-400.0, 400.0),
)
LIMITES = tuple((math.radians(a), math.radians(b)) for a, b in LIMITES_DEG)

# Velocidades máximas de la hoja de datos [grados/s]
VEL_MAX_DEG = (250.0, 250.0, 250.0, 320.0, 320.0, 420.0)
VEL_MAX = tuple(math.radians(v) for v in VEL_MAX_DEG)

NOMBRES_JUNTAS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")


def rot_z(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_x(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def homogenea(R: np.ndarray, p) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(p, dtype=float)
    return T


def inversa_rapida(T: np.ndarray) -> np.ndarray:
    """T^-1 = [R^T, -R^T p; 0 0 0 1] (no es la transpuesta de T)."""
    R, p = T[:3, :3], T[:3, 3]
    return homogenea(R.T, -R.T @ p)


def matriz_dh(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """A_i = Rz(theta) Tz(d) Tx(a) Rx(alpha), escrita término a término."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


# Marco tool0 (ABB) respecto del marco 6 de la tabla D-H: giro de 180° en z6.
T_6_TOOL0 = homogenea(rot_z(PI), [0.0, 0.0, 0.0])


def euler_zyx_a_matriz(rz: float, ry: float, rx: float) -> np.ndarray:
    """R = Rz(rz) · Ry(ry) · Rx(rx), la misma convención que el FlexPendant (EulerZYX)."""
    return rot_z(rz) @ rot_y(ry) @ rot_x(rx)


def matriz_a_euler_zyx(R: np.ndarray) -> tuple[float, float, float]:
    """Inversa de euler_zyx_a_matriz. Devuelve (rz, ry, rx) en rad.

    En ry = ±90° hay bloqueo de cardán (gimbal lock): se fija rx = 0.
    """
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    ry = math.asin(sy)
    if abs(math.cos(ry)) > 1e-9:
        rz = math.atan2(R[1, 0], R[0, 0])
        rx = math.atan2(R[2, 1], R[2, 2])
    else:
        rx = 0.0
        rz = math.atan2(-R[0, 1], R[1, 1])
    return rz, ry, rx


def matriz_a_cuaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    """Cuaternión [w, x, y, z] (mismo orden q1..q4 que usa ABB), con w >= 0."""
    tr = float(np.trace(R))
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    q /= np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    return tuple(float(v) for v in q)


def cuaternion_a_matriz(w: float, x: float, y: float, z: float) -> np.ndarray:
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def error_posicion(T_a: np.ndarray, T_b: np.ndarray) -> float:
    """e_p = ||p_a - p_b||_2 [m]."""
    return float(np.linalg.norm(T_a[:3, 3] - T_b[:3, 3]))


def error_orientacion(R_a: np.ndarray, R_ref: np.ndarray) -> float:
    """Ángulo de R_e = R_ref^T R_a [rad] (protocolo W12).

    Fórmula del protocolo: e_R = arccos(clip((tr(R_e) - 1)/2, -1, 1)).
    Es la misma cantidad que atan2(sin, cos) con sin = ||vee(R_e - R_e^T)||/2.
    Se usa atan2 porque arccos cerca de 1 solo resuelve ~1e-8 rad (la pendiente
    de arccos es infinita en 1); atan2 llega a ~1e-16 rad. Ver test_cinematica.py.
    """
    Re = R_ref.T @ R_a
    c = (float(np.trace(Re)) - 1.0) / 2.0
    w = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    s = float(np.linalg.norm(w)) / 2.0
    return math.atan2(s, c)


def error_orientacion_arccos(R_a: np.ndarray, R_ref: np.ndarray) -> float:
    """Versión literal del protocolo (arccos con clip), para comparar."""
    Re = R_ref.T @ R_a
    u = (float(np.trace(Re)) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, u)))


def envolver(angulo: float) -> float:
    """Lleva un ángulo a (-pi, pi]."""
    a = math.fmod(angulo + PI, 2 * PI)
    if a <= 0:
        a += 2 * PI
    return a - PI


@dataclass
class Robot:
    """IRB 120 con una herramienta (TCP) opcional montada en tool0.

    tcp_xyz y tcp_rpy describen el TCP respecto de tool0 (m, rad, rpy = Rz·Ry·Rx).
    """

    tcp_xyz: tuple = (0.0, 0.0, 0.0)
    tcp_rpy: tuple = (0.0, 0.0, 0.0)
    limites: tuple = LIMITES
    vel_max: tuple = VEL_MAX
    T_tool0_tcp: np.ndarray = field(init=False)

    def __post_init__(self):
        rz, ry, rx = self.tcp_rpy[2], self.tcp_rpy[1], self.tcp_rpy[0]
        self.T_tool0_tcp = homogenea(euler_zyx_a_matriz(rz, ry, rx), self.tcp_xyz)

    # Cinemática directa
    def matrices_eslabon(self, q) -> list[np.ndarray]:
        """Las 6 matrices A_i(q_i) de la tabla D-H."""
        return [matriz_dh(a, al, d, qi + off) for (a, al, d, off), qi in zip(DH, q)]

    def marcos(self, q) -> list[np.ndarray]:
        """[T_0^0, T_0^1, ..., T_0^6]: los 7 marcos D-H en la base."""
        T = np.eye(4)
        salida = [T.copy()]
        for A in self.matrices_eslabon(q):
            T = T @ A
            salida.append(T.copy())
        return salida

    def fk6(self, q) -> np.ndarray:
        """T_0^6 (marco 6 de la tabla D-H)."""
        return self.marcos(q)[-1]

    def fk_tool0(self, q) -> np.ndarray:
        return self.fk6(q) @ T_6_TOOL0

    def fk_tcp(self, q) -> np.ndarray:
        return self.fk_tool0(q) @ self.T_tool0_tcp

    def tcp_a_marco6(self, T_tcp: np.ndarray) -> np.ndarray:
        """Pose deseada del TCP -> pose deseada del marco 6 D-H (para Pieper)."""
        return T_tcp @ inversa_rapida(self.T_tool0_tcp) @ inversa_rapida(T_6_TOOL0)

    # Límites
    def dentro_de_limites(self, q, margen: float = 0.0) -> list[bool]:
        return [bool(lo + margen <= qi <= hi - margen) for qi, (lo, hi) in zip(q, self.limites)]

    def distancia_a_limites(self, q) -> list[float]:
        """Distancia [rad] de cada junta a su límite más cercano (negativa si lo pasa)."""
        return [float(min(qi - lo, hi - qi)) for qi, (lo, hi) in zip(q, self.limites)]

    def puntos_cuerpo(self, q) -> np.ndarray:
        """Puntos del brazo (hombro, codo, muñeca, brida, TCP) para revisar alturas."""
        M = self.marcos(q)
        tcp = self.fk_tcp(q)[:3, 3]
        return np.array([M[1][:3, 3], M[2][:3, 3], M[3][:3, 3], M[4][:3, 3], M[6][:3, 3], tcp])


def pose_desde_xyz_euler(x_mm, y_mm, z_mm, rz_deg, ry_deg, rx_deg) -> np.ndarray:
    """Pose de entrada de la interfaz (mm, grados, EulerZYX) -> T homogénea (m)."""
    R = euler_zyx_a_matriz(math.radians(rz_deg), math.radians(ry_deg), math.radians(rx_deg))
    return homogenea(R, [x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0])


def describir_pose(T: np.ndarray) -> dict:
    """Pose en unidades de la interfaz: mm, grados (EulerZYX) y cuaternión ABB."""
    rz, ry, rx = matriz_a_euler_zyx(T[:3, :3])
    return {
        "x_mm": T[0, 3] * 1000.0,
        "y_mm": T[1, 3] * 1000.0,
        "z_mm": T[2, 3] * 1000.0,
        "rz_deg": math.degrees(rz),
        "ry_deg": math.degrees(ry),
        "rx_deg": math.degrees(rx),
        "cuaternion": matriz_a_cuaternion(T[:3, :3]),
    }
