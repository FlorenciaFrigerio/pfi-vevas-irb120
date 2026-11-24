"""El simulador no pierde tiempo físico cuando el scheduler retrasa su hilo."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vevas.backend_sim import BackendSim
import vevas.backend_sim as modulo


def respuesta(monkeypatch, instantes):
    reloj = iter([0.0, *instantes])
    monkeypatch.setattr(modulo.time, "monotonic", lambda: next(reloj))
    backend = BackendSim(np.zeros(6), tau=0.04, ruido_deg=0)
    backend._ref = np.ones(6)
    backend._activo = True
    contador = 0

    def siguiente(_):
        nonlocal contador
        contador += 1
        if contador == len(instantes):
            backend._activo = False

    monkeypatch.setattr(modulo.time, "sleep", siguiente)
    backend._bucle()
    return backend._q.copy(), backend._t_estado


@pytest.mark.parametrize("pasos", [1, 5, 50])
def test_respuesta_depende_del_tiempo_transcurrido_no_del_numero_de_ciclos(monkeypatch, pasos):
    q, sello = respuesta(monkeypatch, np.linspace(0.2 / pasos, 0.2, pasos).tolist())
    # Respuesta física de un primer orden después de cinco constantes de tiempo.
    assert np.allclose(q, np.full(6, 1 - math.exp(-5)), atol=1e-12)
    assert sello == 0.2


def test_sin_tiempo_transcurrido_no_se_simula_un_avance(monkeypatch):
    q, sello = respuesta(monkeypatch, [0.0])
    assert np.array_equal(q, np.zeros(6))
    assert sello == 0.0
