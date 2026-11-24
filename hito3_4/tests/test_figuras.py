"""Pruebas geométricas de los trazados relativos al TCP."""

import numpy as np
import pytest

from vevas.figuras import generar_figura


@pytest.mark.parametrize("nombre", ["círculo", "corazón", "cuadrado", "triángulo"])
@pytest.mark.parametrize(("plano", "ejes", "normal"), [("XY", (0, 1), 2), ("XZ", (0, 2), 1), ("YZ", (1, 2), 0)])
def test_trazado_cerrado_en_su_plano_y_con_el_ancho_pedido(nombre, plano, ejes, normal):
    puntos = generar_figura(nombre, 42, plano, muestras=98)

    assert puntos.shape == (98, 3)
    assert np.all(np.isfinite(puntos))
    np.testing.assert_array_equal(puntos[[0, -1]], 0.0)
    np.testing.assert_array_equal(puntos[:, normal], 0.0)
    np.testing.assert_allclose(puntos[:, ejes], generar_figura(nombre, 42, muestras=98)[:, :2])
    assert np.ptp(puntos[:, ejes[0]]) == pytest.approx(0.042)
    assert np.all(np.linalg.norm(np.diff(puntos, axis=0), axis=1) > 0)


def test_circulo_usa_diametro_y_empieza_en_su_borde():
    puntos = generar_figura("Circulo")
    radio = 0.025 / 2
    centro = [-radio, 0, 0]
    assert puntos.shape == (65, 3)
    np.testing.assert_allclose(np.linalg.norm(puntos - centro, axis=1), radio)
    assert np.ptp(puntos[:, 1]) == pytest.approx(0.025)


def test_corazon_simetrico_anclado_en_la_hendidura():
    puntos = generar_figura("corazon", 32)
    np.testing.assert_allclose(puntos[:, :2], puntos[::-1, :2] * [-1, 1], atol=1e-15)
    assert puntos[:, 1].min() == pytest.approx(-0.022)
    assert puntos[1, 1] > 0
    assert puntos[-2, 1] > 0


def test_cuadrado_incluye_los_cuatro_vertices_y_recorridos_rectos():
    puntos = generar_figura("cuadrado", 100, muestras=129)
    n = range(0, 129, 32)
    esquinas = [[0, 0], [0.1, 0], [0.1, 0.1], [0, 0.1], [0, 0]]
    np.testing.assert_array_equal(puntos[n, :2], esquinas)
    delta = np.diff(puntos[:, :2], axis=0)
    assert np.all(np.count_nonzero(delta, axis=1) == 1)
    assert np.linalg.norm(delta, axis=1).sum() == pytest.approx(0.4)


@pytest.mark.parametrize("muestras", [65, 98, 129])
def test_triangulo_equilatero_cerrado_con_sus_tres_vertices(muestras):
    lado = 0.03
    puntos = generar_figura("triangulo", 30, muestras=muestras)
    np.testing.assert_allclose(puntos, generar_figura("Triángulo", 30, muestras=muestras))
    assert puntos.shape == (muestras, 3)
    np.testing.assert_array_equal(puntos[[0, -1]], 0.0)
    por_lado, resto = divmod(muestras - 1, 3)
    k1 = por_lado + (resto > 0)
    k2 = k1 + por_lado + (resto > 1)
    esquinas = [[0, 0], [lado, 0], [lado / 2, lado * np.sqrt(3) / 2], [0, 0]]
    np.testing.assert_allclose(puntos[[0, k1, k2, muestras - 1], :2], esquinas, atol=1e-15)
    lados = np.linalg.norm(np.diff(np.asarray(esquinas), axis=0), axis=1)
    np.testing.assert_allclose(lados, lado)
    assert np.linalg.norm(np.diff(puntos[:, :2], axis=0), axis=1).sum() == pytest.approx(3 * lado)
    # recorridos rectos: cada punto queda sobre el lado que le toca
    for a, b, tramo in ((0, k1, puntos[:k1 + 1]), (k1, k2, puntos[k1:k2 + 1]), (k2, muestras - 1, puntos[k2:])):
        d = puntos[b, :2] - puntos[a, :2]
        rel = tramo[:, :2] - puntos[a, :2]
        np.testing.assert_allclose(rel[:, 0] * d[1] - rel[:, 1] * d[0], 0.0, atol=1e-15)


@pytest.mark.parametrize(("nombre", "datos", "error"), [
    ("estrella", {}, "Figura desconocida"),
    (42, {}, "Figura desconocida"),
    ("circulo", {"plano": "ZY"}, "plano válido"),
    ("circulo", {"tamano_mm": 4.99}, "tamaño"),
    ("circulo", {"tamano_mm": 100.01}, "tamaño"),
    ("circulo", {"tamano_mm": np.nan}, "tamaño"),
    ("circulo", {"tamano_mm": np.inf}, "tamaño"),
    ("circulo", {"tamano_mm": "25"}, "tamaño"),
    ("circulo", {"muestras": 64}, "muestras"),
    ("circulo", {"muestras": 130}, "muestras"),
    ("circulo", {"muestras": 65.0}, "muestras"),
])
def test_rechaza_entradas_invalidas(nombre, datos, error):
    with pytest.raises(ValueError, match=error):
        generar_figura(nombre, **datos)
