"""Trazados cartesianos relativos al TCP, expresados en metros."""

import math
from numbers import Integral, Real

import numpy as np


PLANOS = {"XY": (0, 1), "XZ": (0, 2), "YZ": (1, 2)}
NOMBRES = {"circulo": "círculo", "círculo": "círculo", "corazon": "corazón",
           "corazón": "corazón", "cuadrado": "cuadrado", "triangulo": "triángulo",
           "triángulo": "triángulo", "saludo": "saludo"}

# Saludo: la mano va y viene en arco tres veces, como un antebrazo que pivota en el codo (∩).
SALUDO_AMPLITUD_DEG = 40.0
SALUDO_VAIVENES = 2  # 0 → +A → −A → +A → −A → 0


def generar_figura(nombre, tamano_mm=25, plano="XY", *, muestras=65) -> np.ndarray:
    """Devuelve `muestras` desplazamientos XYZ en metros, con origen y cierre en cero.

    El tamaño es el ancho en el primer eje del plano. Equivale al diámetro del
    círculo y al lado del cuadrado y del triángulo equilátero. El corazón conserva
    su proporción original. Los puntos del cuadrado y del triángulo incluyen sus
    vértices y solo describen la geometría.
    """
    figura = NOMBRES.get(nombre.strip().lower()) if isinstance(nombre, str) else None
    if figura is None:
        raise ValueError("Figura desconocida. Elegí círculo, corazón, cuadrado, triángulo o saludo.")
    ejes = PLANOS.get(plano.strip().upper()) if isinstance(plano, str) else None
    if ejes is None:
        raise ValueError("Elegí un plano válido (XY, XZ o YZ).")
    if (isinstance(tamano_mm, bool) or not isinstance(tamano_mm, Real)
            or not 5 <= tamano_mm <= 100 or not math.isfinite(tamano_mm)):
        raise ValueError("El tamaño debe ser un número entre 5 y 100 mm.")
    if not isinstance(muestras, Integral) or not 65 <= muestras <= 129:
        raise ValueError("La cantidad de muestras debe ser un entero entre 65 y 129.")

    # Incluir los tramos exactos (cuatro cuartos; tres lados en el triángulo)
    # aunque el número de intervalos no sea divisible. Así se conservan
    # vértices y ancho.
    tramos = 3 if figura == "triángulo" else 4
    por_tramo, resto = divmod(muestras - 1, tramos)
    partes = [np.linspace(i, i + 1, por_tramo + (i < resto), endpoint=False) for i in range(tramos)]
    u = np.concatenate([*partes, [float(tramos)]])

    if figura == "saludo":
        # Arco de radio R sobre un pivote imaginario debajo del TCP (el «codo»): 0 → +A → −A → +A → −A → 0.
        # Ancho total = tamaño → R = tamaño / (2 sen A). Empieza y termina en el TCP actual.
        # Ángulo repartido parejo a lo largo del arco (misma separación entre puntos) y con los
        # extremos justo en muestras: ahí la mano se detiene un instante, como al saludar.
        amp = math.radians(SALUDO_AMPLITUD_DEG)
        giros = [0.0] + [(-1.0) ** k for k in range(2 * SALUDO_VAIVENES)] + [0.0]
        hitos = np.concatenate([[0.0], np.cumsum(np.abs(np.diff(giros)))])  # recorrido acumulado, en unidades de A
        u_s = np.linspace(0.0, hitos[-1], muestras)
        th = amp * np.interp(u_s, hitos, giros)
        radio = 1.0 / (2.0 * math.sin(amp))
        a = radio * np.sin(th)
        b = -radio * (1.0 - np.cos(th))
        puntos = np.zeros((muestras, 3))
        puntos[:, ejes] = np.column_stack((a, b)) * (tamano_mm / 1000)
        puntos[[0, -1]] = 0.0
        return puntos

    if figura == "triángulo":
        # Equilátero de lado = tamaño, con un vértice en el origen.
        a = np.interp(u, range(4), [0, 1, 0.5, 0])
        b = np.interp(u, range(4), [0, 0, math.sqrt(3) / 2, 0])
    elif figura == "cuadrado":
        a = np.interp(u, range(5), [0, 1, 1, 0, 0])
        b = np.interp(u, range(5), [0, 0, 1, 1, 0])
    else:
        t = u * (math.pi / 2)
        if figura == "círculo":
            a = (np.cos(t) - 1) / 2
            b = np.sin(t) / 2
        else:
            a = 16 * np.sin(t)**3 / 32
            b = (13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t) - 5) / 32

    puntos = np.zeros((muestras, 3))
    puntos[:, ejes] = np.column_stack((a, b)) * (tamano_mm / 1000)
    puntos[[0, -1]] = 0.0
    return puntos


def nombre_canonico(nombre) -> str | None:
    """«círculo», «circulo» → «círculo»; «saludo» → «saludo»; otro → None."""
    if not isinstance(nombre, str):
        return None
    n = nombre.strip().lower()
    return NOMBRES.get(n)


def paradas_figura(nombre, puntos, limite=None) -> list[int]:
    """Índices de las muestras donde el recorrido se detiene (v = 0).

    Corazón: la punta (cúspide). Cuadrado y triángulo: cada esquina (cambio de
    dirección de más de ~37°). Saludo: cada extremo del vaivén (vuelta atrás). ``limite`` es la cantidad de muestras que llegan a
    resolverse (si la IK falla antes del final, solo cuentan las previas).
    """
    figura = nombre_canonico(nombre)
    puntos = np.asarray(puntos, dtype=float)
    n = len(puntos)
    limite = n if limite is None else int(limite)
    paradas = []
    if figura == "corazón" and limite > n // 2:
        paradas.append(n // 2)
    if figura in ("cuadrado", "triángulo", "saludo"):
        d = np.diff(puntos, axis=0)
        for k in range(1, min(limite - 1, len(d))):
            if np.dot(d[k - 1], d[k]) < 0.8 * np.linalg.norm(d[k - 1]) * np.linalg.norm(d[k]):
                paradas.append(k)
    return paradas


def plano_de_vista(offsets, plano="XY") -> str:
    """Plano en el que conviene dibujar el recorrido en 2D.

    Para las figuras es su plano de dibujo. Si el recorrido no es plano (saludo
    articular), se elige el plano base con mayor extensión.
    """
    offsets = np.asarray(offsets, dtype=float)
    plano = plano.strip().upper() if isinstance(plano, str) else "XY"
    if plano in PLANOS:
        normal = ({0, 1, 2} - set(PLANOS[plano])).pop()
        if offsets.size == 0 or np.ptp(offsets[:, normal]) < 1e-9:
            return plano
    rango = np.ptp(offsets, axis=0) if offsets.size else np.zeros(3)
    return max(PLANOS, key=lambda p: float(rango[PLANOS[p][0]] * rango[PLANOS[p][1]] + rango[PLANOS[p][0]] + rango[PLANOS[p][1]]))
