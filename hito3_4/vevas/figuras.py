"""Trazados cartesianos relativos al TCP, expresados en metros."""

import math
from numbers import Integral, Real

import numpy as np


PLANOS = {"XY": (0, 1), "XZ": (0, 2), "YZ": (1, 2)}
NOMBRES = {"circulo": "círculo", "círculo": "círculo", "corazon": "corazón",
           "corazón": "corazón", "cuadrado": "cuadrado", "triangulo": "triángulo",
           "triángulo": "triángulo"}


def generar_figura(nombre, tamano_mm=25, plano="XY", *, muestras=65) -> np.ndarray:
    """Devuelve `muestras` desplazamientos XYZ en metros, con origen y cierre en cero.

    El tamaño es el ancho en el primer eje del plano. Equivale al diámetro del
    círculo y al lado del cuadrado y del triángulo equilátero. El corazón conserva
    su proporción original. Los puntos del cuadrado y del triángulo incluyen sus
    vértices y solo describen la geometría.
    """
    figura = NOMBRES.get(nombre.strip().lower()) if isinstance(nombre, str) else None
    if figura is None:
        raise ValueError("Figura desconocida. Elegí círculo, corazón, cuadrado o triángulo.")
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
