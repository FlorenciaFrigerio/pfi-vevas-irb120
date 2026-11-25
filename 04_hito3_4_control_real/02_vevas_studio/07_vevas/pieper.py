"""Cinemática inversa analítica del IRB 120 por desacople de Pieper (8 ramas).

Idea: los ejes 4, 5 y 6 se cruzan en un punto (muñeca esférica). Entonces:
    1) la POSICIÓN del centro de muñeca depende solo de q1, q2, q3;
    2) la ORIENTACIÓN restante depende solo de q4, q5, q6.

Pasos (los mismos que se muestran en la consola):
    P1  T_d (TCP) -> T_0^6 deseada (se quita la herramienta y el giro tool0)
    P2  centro de muñeca  p_w = p_6 - d6 · z_6
    P3  q1 = atan2(y_w, x_w)  o  q1 + pi   (hombro adelante / atrás)
    P4  plano del brazo: r = ±sqrt(x_w^2 + y_w^2), s = z_w - d1
        ley de cosenos: cos(delta) = (r^2 + s^2 - a2^2 - L3^2) / (2 a2 L3)
        delta = ±acos(...)                  (codo arriba / codo abajo)
        q3 = delta + gamma - pi/2,  gamma = atan2(a3, d4),  L3 = sqrt(a3^2 + d4^2)
        q2 = atan2(r, s) - atan2(L3 sin(delta), a2 + L3 cos(delta))
    P5  R_3^6 = (R_0^3)^T · R_0^6
    P6  muñeca (dos soluciones, s5 > 0 o s5 < 0):
        q5 = atan2(±sqrt(r13^2 + r23^2), r33)
        q4 = atan2(-r23 / s5, -r13 / s5)
        q6 = atan2(-r32 / s5,  r31 / s5)
        Si s5 ~ 0 (singularidad de muñeca) solo se conoce q4 + q6: se fija q4 = q4_actual.
    P7  cada rama se verifica con FK: e_p y e_R deben ser ~0.
    P8  se elige la mejor rama (ver elegir_rama).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .jacobiano import GAMMA, L3, analisis_singularidad
from .robot_irb120 import A2, D1, D6, PI, Robot, envolver, error_orientacion, error_posicion

UMBRAL_SINGULAR_MUNECA = 1e-6  # |sin q5| por debajo de esto se trata como singular


@dataclass
class Rama:
    indice: int
    hombro: str  # "adelante" / "atrás"
    codo: str  # "arriba" / "abajo"
    muneca: str  # "sin giro" / "girada"
    q: list | None = None  # rad, None si no existe (fuera de alcance)
    existe: bool = True
    motivo_no_existe: str = ""
    en_limites: list = field(default_factory=list)
    margen_min_deg: float = 0.0
    e_p: float = float("nan")
    e_R: float = float("nan")
    sigma_min: float = float("nan")
    distancia: float = float("nan")
    puntaje: float = float("inf")
    valida: bool = False
    motivos: list = field(default_factory=list)
    singular_muneca: bool = False

    def a_dict(self) -> dict:
        d = {
            "indice": self.indice,
            "nombre": f"{self.hombro} · codo {self.codo} · muñeca {self.muneca}",
            "hombro": self.hombro,
            "codo": self.codo,
            "muneca": self.muneca,
            "existe": self.existe,
            "motivo_no_existe": self.motivo_no_existe,
            "valida": self.valida,
            "motivos": self.motivos,
            "singular_muneca": self.singular_muneca,
        }
        if self.q is not None:
            d.update(
                {
                    "q_rad": [float(v) for v in self.q],
                    "q_deg": [math.degrees(v) for v in self.q],
                    "en_limites": self.en_limites,
                    "margen_min_deg": self.margen_min_deg,
                    "e_p_m": self.e_p,
                    "e_R_rad": self.e_R,
                    "sigma_min": self.sigma_min,
                    "distancia": self.distancia,
                    "puntaje": self.puntaje if math.isfinite(self.puntaje) else None,
                }
            )
        return d


def _representante_cercano(angulo: float, referencia: float, lo: float, hi: float) -> float:
    """De angulo + 2k·pi, el que esté dentro de [lo, hi] y más cerca de la referencia.

    Sirve para q4 (±160°) y sobre todo q6 (±400°): evita giros completos innecesarios.
    """
    candidatos = [angulo + 2 * PI * k for k in range(-3, 4)]
    dentro = [c for c in candidatos if lo - 1e-9 <= c <= hi + 1e-9]
    if not dentro:
        return envolver(angulo)
    return min(dentro, key=lambda c: abs(c - referencia))


def _rot03(q1: float, q2: float, q3: float, robot: Robot) -> np.ndarray:
    M = robot.marcos([q1, q2, q3, 0.0, 0.0, 0.0])
    return M[3][:3, :3]


def resolver(robot: Robot, T_tcp: np.ndarray, q_actual=None) -> dict:
    """Resuelve las 8 ramas y devuelve todos los pasos intermedios.

    T_tcp: pose deseada del TCP en la base (4x4, metros).
    q_actual: configuración actual (rad); se usa para desempates y continuidad.
    """
    if q_actual is None:
        q_actual = [0.0] * 6
    q_actual = [float(v) for v in q_actual]

    pasos = {}
    T06 = robot.tcp_a_marco6(T_tcp)
    R06 = T06[:3, :3]
    p6 = T06[:3, 3]
    z6 = R06[:, 2]
    pasos["T_tcp"] = T_tcp.tolist()
    pasos["T06"] = T06.tolist()

    # P2: centro de muñeca
    pw = p6 - D6 * z6
    pasos["p6"] = p6.tolist()
    pasos["z6"] = z6.tolist()
    pasos["p_w"] = pw.tolist()

    # P3: dos opciones de q1
    r_xy = math.hypot(pw[0], pw[1])
    singular_hombro = r_xy < 1e-9
    q1a = math.atan2(pw[1], pw[0]) if not singular_hombro else q_actual[0]
    opciones_q1 = [("adelante", q1a, r_xy), ("atrás", envolver(q1a + PI), -r_xy)]
    s = pw[2] - D1
    pasos["r_xy"] = r_xy
    pasos["s"] = s
    pasos["singular_hombro"] = singular_hombro
    pasos["q1_opciones"] = [o[1] for o in opciones_q1]

    ramas: list[Rama] = []
    detalle_brazo = []
    idx = 0
    for nombre_h, q1, r in opciones_q1:
        cos_d = (r * r + s * s - A2 * A2 - L3 * L3) / (2 * A2 * L3)
        info = {"hombro": nombre_h, "q1": q1, "r": r, "s": s, "cos_delta": cos_d}
        alcanzable = -1.0 - 1e-12 <= cos_d <= 1.0 + 1e-12
        cos_d = max(-1.0, min(1.0, cos_d))
        sub = []
        for nombre_c, signo in (("arriba", 1.0), ("abajo", -1.0)):
            delta = signo * math.acos(cos_d)
            q3 = envolver(delta + GAMMA - PI / 2)
            q2 = envolver(math.atan2(r, s) - math.atan2(L3 * math.sin(delta), A2 + L3 * math.cos(delta)))
            sub.append({"codo": nombre_c, "delta": delta, "q2": q2, "q3": q3})
            for nombre_m in ("sin giro", "girada"):
                rama = Rama(idx, nombre_h, nombre_c, nombre_m)
                idx += 1
                if not alcanzable:
                    rama.existe = False
                    rama.motivo_no_existe = f"fuera de alcance: |cos δ| = {abs(info['cos_delta']):.3f} > 1"
                    ramas.append(rama)
                    continue
                R03 = _rot03(q1, q2, q3, robot)
                R36 = R03.T @ R06
                s5 = math.hypot(R36[0, 2], R36[1, 2])
                if nombre_m == "girada":
                    s5 = -s5
                if abs(s5) < UMBRAL_SINGULAR_MUNECA:
                    rama.singular_muneca = True
                    q5 = 0.0 if R36[2, 2] > 0 else PI
                    suma = math.atan2(R36[1, 0], R36[0, 0])
                    q4 = q_actual[3]
                    q6 = suma - q4
                    if nombre_m == "girada":
                        rama.existe = False
                        rama.motivo_no_existe = "singularidad de muñeca: la rama girada coincide con la otra"
                        ramas.append(rama)
                        continue
                else:
                    q5 = math.atan2(s5, R36[2, 2])
                    q4 = math.atan2(-R36[1, 2] / s5, -R36[0, 2] / s5)
                    q6 = math.atan2(-R36[2, 1] / s5, R36[2, 0] / s5)
                lo4, hi4 = robot.limites[3]
                lo6, hi6 = robot.limites[5]
                q4 = _representante_cercano(q4, q_actual[3], lo4, hi4)
                q6 = _representante_cercano(q6, q_actual[5], lo6, hi6)
                rama.q = [q1, q2, q3, q4, q5, q6]
                ramas.append(rama)
        info["codos"] = sub
        info["alcanzable"] = alcanzable
        detalle_brazo.append(info)
    pasos["brazo"] = detalle_brazo

    # Primera rama existente: guardar R_0^3 y R_3^6 para mostrar el paso P5
    for rama in ramas:
        if rama.existe and rama.q is not None:
            R03 = _rot03(*rama.q[:3], robot)
            pasos["ejemplo_R03"] = R03.tolist()
            pasos["ejemplo_R36"] = (R03.T @ R06).tolist()
            pasos["ejemplo_rama"] = rama.indice
            break

    # P7: verificar cada rama con FK
    for rama in ramas:
        if not rama.existe:
            continue
        T = robot.fk_tcp(rama.q)
        rama.e_p = error_posicion(T, T_tcp)
        rama.e_R = error_orientacion(T[:3, :3], T_tcp[:3, :3])
        rama.en_limites = robot.dentro_de_limites(rama.q)
        rama.margen_min_deg = math.degrees(min(robot.distancia_a_limites(rama.q)))
        rama.sigma_min = analisis_singularidad(robot, rama.q)["sigma_min"]
        rama.distancia = distancia_ponderada(rama.q, q_actual)

    return {"pasos": pasos, "ramas": ramas}


PESOS_DISTANCIA = (1.0, 1.0, 1.0, 0.6, 0.6, 0.4)


def distancia_ponderada(q, q_ref) -> float:
    """sum_i w_i |q_i - q_ref,i|: mover menos las juntas grandes (1, 2, 3) es mejor."""
    return float(sum(w * abs(a - b) for w, a, b in zip(PESOS_DISTANCIA, q, q_ref)))


def elegir_rama(ramas: list[Rama], tol_p: float, tol_R: float, margen_lim: float, sigma_min: float):
    """Marca qué ramas son válidas y elige la mejor.

    Una rama es VÁLIDA si:
        existe, e_p <= tol_p, e_R <= tol_R, todas las juntas dentro de límites con
        margen, y no está cerca de una singularidad (sigma_min >= umbral).
    Entre las válidas gana la de MENOR puntaje:
        puntaje = distancia ponderada a q_actual  (+ penalización pequeña si el margen es poco)
    El error FK ya es ~1e-16 en todas las válidas, así que no decide; la cercanía sí.
    """
    for r in ramas:
        r.motivos = []
        if not r.existe:
            r.valida = False
            r.motivos.append(r.motivo_no_existe)
            continue
        ok = True
        if not (r.e_p <= tol_p):
            ok = False
            r.motivos.append(f"e_p = {r.e_p:.2e} m > {tol_p:.0e}")
        if not (r.e_R <= tol_R):
            ok = False
            r.motivos.append(f"e_R = {r.e_R:.2e} rad > {tol_R:.0e}")
        fuera = [i + 1 for i, ok_i in enumerate(r.en_limites) if not ok_i]
        if fuera:
            ok = False
            r.motivos.append("fuera de límites en J" + ", J".join(str(i) for i in fuera))
        elif r.margen_min_deg < math.degrees(margen_lim):
            ok = False
            r.motivos.append(f"a {r.margen_min_deg:.1f}° de un límite (margen pedido {math.degrees(margen_lim):.0f}°)")
        if r.sigma_min < sigma_min:
            ok = False
            r.motivos.append(f"cerca de singularidad: σ_min = {r.sigma_min:.3f} < {sigma_min}")
        r.valida = ok
        if ok:
            penal = max(0.0, math.radians(20.0) - math.radians(r.margen_min_deg))
            r.puntaje = r.distancia + 0.5 * penal
            r.motivos.append("válida")
        else:
            r.puntaje = float("inf")
    validas = [r for r in ramas if r.valida]
    if not validas:
        return None, "Ninguna rama es válida: la pose no se puede alcanzar con seguridad."
    mejor = min(validas, key=lambda r: r.puntaje)
    razon = (
        f"Rama {mejor.indice} ({mejor.hombro}, codo {mejor.codo}, muñeca {mejor.muneca}): "
        f"de {len(validas)} ramas válidas es la que menos mueve el robot desde la postura actual "
        f"(distancia ponderada {mejor.distancia:.3f} rad), con error FK e_p = {mejor.e_p:.1e} m, "
        f"e_R = {mejor.e_R:.1e} rad y margen a límites {mejor.margen_min_deg:.1f}°."
    )
    return mejor, razon
