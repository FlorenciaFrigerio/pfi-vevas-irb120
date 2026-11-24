"""Interpolación del simulador con reloj determinista, sin hilos ni hardware."""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vevas.backend_sim as modulo  # noqa: E402
from vevas.backend_sim import BackendSim  # noqa: E402


@pytest.fixture
def sim(monkeypatch):
    reloj = SimpleNamespace(ahora=0.0)
    monkeypatch.setattr(modulo, "time", SimpleNamespace(monotonic=lambda: reloj.ahora))
    backend = BackendSim(np.zeros(6), ruido_deg=0)
    modulo.time.sleep = lambda _: setattr(backend, "_activo", False)

    def paso(t):
        reloj.ahora = t
        backend._activo = True
        backend._bucle()
        q_medida, sello = backend.leer()
        assert sello == t
        return backend._ref.copy(), q_medida

    return backend, paso


def test_un_objetivo_futuro_interpola_desde_el_origen_hasta_llegar(sim):
    backend, paso = sim
    destino = np.full(6, 0.2)
    assert backend.ejecutar_trayectoria([1], [destino], [np.zeros(6)])[0]
    assert np.array_equal(paso(0)[0], np.zeros(6))
    assert np.allclose(paso(0.1)[0], destino * (3 * 0.1**2 - 2 * 0.1**3))
    referencia, medida = paso(0.5)
    assert np.allclose(referencia, destino / 2)
    assert np.all(medida < referencia)  # conserva la dinámica medida y su retardo
    assert np.array_equal(paso(1)[0], destino)
    assert not backend.en_movimiento()
    assert backend.ultimo_resultado == "terminada"


def test_muestras_sin_derivadas_conservan_interpolacion_lineal(sim):
    backend, paso = sim
    assert backend.ejecutar_trayectoria([0.25, 0.75], np.array([[1] * 6, [2] * 6]))[0]
    assert np.allclose(paso(0.125)[0], 0.5)
    assert np.allclose(paso(0.5)[0], 1.5)


@pytest.mark.parametrize("grado", [3, 5])
def test_hermite_reproduce_posicion_y_derivadas_a_traves_de_los_nodos(sim, grado):
    backend, paso = sim
    tiempos = np.array([0, 0.31, 0.69, 1])
    escala = np.array([1, -2, 3, -4, 5, -6]) / 10
    q = tiempos[:, None] ** grado * escala
    qd = grado * tiempos[:, None] ** (grado - 1) * escala
    derivadas = {}
    if grado == 5:
        derivadas["QDD"] = grado * (grado - 1) * tiempos[:, None] ** (grado - 2) * escala
    assert backend.ejecutar_trayectoria(tiempos, q, qd, **derivadas)[0]

    for t in [0, 0.13, 0.309999, 0.31, 0.310001, 0.5, 0.689999, 0.69, 0.690001, 0.92, 1]:
        referencia, _ = paso(t)
        assert np.allclose(referencia, t**grado * escala, atol=1e-12)


def test_el_destino_permanece_fijo_y_la_medida_converge_tras_un_salto_de_reloj(sim):
    backend, paso = sim
    destino = np.full(6, 0.2)
    assert backend.ejecutar_trayectoria([0, 0.25], [np.zeros(6), destino])[0]
    paso(0.1)
    referencia, medida = paso(1.5)
    assert np.array_equal(referencia, destino)
    assert np.allclose(medida, destino, atol=1e-12)
    assert not backend.en_movimiento()


@pytest.mark.parametrize("ts,Q,QD,QDD", [
    ([], [], None, None),
    ([-1], [[1] * 6], None, None),
    ([0.5, 0.5], [[1] * 6] * 2, None, None),
    ([float("nan")], [[1] * 6], None, None),
    ([1], [[float("inf")] * 6], None, None),
    ([1], [[1] * 5], None, None),
    ([1], [[1] * 6], [[0] * 5], None),
    ([1], [[1] * 6], None, [[0] * 6]),
    ([0, 1], [[-1e308] * 6, [1e308] * 6], None, None),
])
def test_muestras_invalidas_no_reemplazan_una_trayectoria_activa(sim, ts, Q, QD, QDD):
    backend, paso = sim
    destino = np.full(6, 0.2)
    assert backend.ejecutar_trayectoria([1], [destino])[0]
    assert not backend.ejecutar_trayectoria(ts, Q, QD, QDD=QDD)[0]
    assert np.allclose(paso(0.5)[0], destino / 2)
