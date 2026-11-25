"""Interpolación articular C2 en simulación; sin transporte ni controlador."""
import math

import numpy as np
import pytest

from vevas.trayectoria import planificar, planificar_camino, planificar_suave


def test_quintica_punto_a_punto_respeta_solucion_analitica():
    q0 = np.zeros(6)
    qf = np.array([1, -.4, 0, .05, 0, .6])
    vmax = np.array([2., .8, 1, .3, 2, 3])
    for factor, t_acel, t_min in [(1., .5, .01), (.05, .05, .1), (.8, .3, 12.)]:
        p = planificar_suave(q0, qf, vmax, factor, t_acel, t_min=t_min)
        v = vmax * factor
        d = abs(qf - q0)
        # Picos del smoothstep de quinto grado: s'(1/2) y |s''((3±√3)/6)|.
        tf = max(t_min, (1.875*d/v).max(), np.sqrt((10/math.sqrt(3))*d*t_acel/v).max())
        assert p.tf == pytest.approx(tf)
        np.testing.assert_allclose(p.V, 1.875*d/tf)
        np.testing.assert_allclose(p.A, (10/math.sqrt(3))*d/tf**2)
        for t, posicion in [(-1, q0), (0, q0), (p.tf, qf), (p.tf + 1, qf)]:
            q, qd, qdd = p.evaluar(t)
            np.testing.assert_array_equal(q, posicion)
            np.testing.assert_array_equal(qd, 0.)
            np.testing.assert_array_equal(qdd, 0.)
        u = .3
        np.testing.assert_allclose(p.evaluar(u*tf)[0], q0 + (qf - q0) * (10*u**3-15*u**4+6*u**5))
        assert p.a_dict()["perfil"] == "quintico" and p.tc is None
    simultanea = planificar_suave(q0, np.ones(6), np.ones(6), .2, t_acel=.01, t_min=.01)
    assert simultanea.tf == pytest.approx(1.875 / .2)
    clasica = planificar(q0, qf, vmax, .05)
    assert clasica.perfil == clasica.a_dict()["perfil"] == "lspb"
    assert clasica.tc == pytest.approx(.25*clasica.tf)


def test_camino_atraviesa_nudos_sin_parar_y_es_c2():
    fase = np.linspace(.1, 2.8, 17)
    puntos = np.column_stack([.2*np.cos(fase), .3*np.sin(fase), .04*fase,
                              np.zeros((len(fase), 3))])
    p = planificar_camino(puntos, np.ones(6), .25, t_acel=.5)
    tiempos, Q, QD, QDD = p.muestrear(.005)
    assert np.all(np.diff(tiempos) > 0)
    assert np.max(np.diff(tiempos)) <= .005 + 1e-12
    np.testing.assert_array_equal(Q[np.searchsorted(tiempos, p.tiempos_nudos)], puntos)
    for i, t in enumerate(p.tiempos_nudos[1:-1], 1):
        q, qd, qdd = p.evaluar(t)
        assert np.linalg.norm(qd) > 1e-3
        np.testing.assert_array_equal(qdd, 0.)
        h = min(t-p.tiempos_nudos[i-1], p.tiempos_nudos[i+1]-t) * 1e-6
        for lado in (-1, 1):
            q_l, v_l, a_l = p.evaluar(t + lado*h)
            np.testing.assert_allclose((q_l-q)/(lado*h), qd, atol=2e-7)
            np.testing.assert_allclose(v_l, qd, atol=1e-8)
            np.testing.assert_allclose(a_l, qdd, atol=2e-5)
    np.testing.assert_allclose(np.max(abs(QD), axis=0), p.V, atol=5e-5)
    np.testing.assert_allclose(np.max(abs(QDD), axis=0), p.A, atol=.001)


def test_camino_con_esquinas_y_repeticiones():
    eje = np.array([1., -.5, 0, .4, 0, 0])
    puntos = np.outer([0, .2, .2, .4, .5, .6], eje)
    p = planificar_camino(puntos, np.ones(6), .5, paradas=[1, 4])
    assert len(p.tiempos_nudos) == len(puntos)
    assert p.tiempos_nudos[1] == p.tiempos_nudos[2]
    assert p.perfil == "camino_quintico"
    for i in (0, 1, 2, 4, 5):
        q, qd, qdd = p.evaluar(p.tiempos_nudos[i])
        np.testing.assert_array_equal(q, puntos[i])
        np.testing.assert_array_equal(qd, 0.)
        np.testing.assert_array_equal(qdd, 0.)
    assert np.linalg.norm(p.evaluar(p.tiempos_nudos[3])[1]) > 1e-3
    quieta = planificar_camino(np.tile(eje, (4, 1)), np.ones(6), .05, t_min=.75)
    assert quieta.tf == pytest.approx(.75)
    np.testing.assert_array_equal(quieta.V, 0.)
    np.testing.assert_array_equal(quieta.A, 0.)
    np.testing.assert_array_equal(quieta.evaluar(.3)[0], eje)


def test_camino_no_sobrepasa_cotas_entre_nudos_irregulares():
    rng = np.random.default_rng(22)
    vmax = np.array([.15, 4., 1., .3, 2., 3.])
    for _ in range(8):
        delta = rng.normal(size=(12, 6)) * rng.uniform(.001, .3, size=(12, 1))
        puntos = np.vstack((np.zeros(6), np.cumsum(delta, axis=0)))
        p = planificar_camino(puntos, vmax, .4, t_acel=.7, paradas=[5])
        assert np.all(p.V <= .4*vmax * (1+1e-10))
        assert np.all(p.A <= .4*vmax/.7 * (1+1e-10))
        for i, (inicio, fin) in enumerate(zip(p.tiempos_nudos, p.tiempos_nudos[1:])):
            valores = [p.evaluar(t) for t in np.linspace(inicio, fin, 41)]
            q, v, a = np.moveaxis(np.asarray(valores), 1, 0)
            assert np.all(q >= puntos[i:i+2].min(axis=0) - 1e-12)
            assert np.all(q <= puntos[i:i+2].max(axis=0) + 1e-12)
            assert np.all(abs(v) <= p.V + 1e-10)
            assert np.all(abs(a) <= p.A + 1e-10)


@pytest.mark.parametrize("opciones", [{"factor_vel": 0}, {"factor_vel": float("nan")},
                                     {"t_acel": -1}, {"t_min": 0}, {"paradas": [-1]}, {"paradas": [True]}])
def test_rechaza_parametros_invalidos(opciones):
    args = {"factor_vel": .2, **opciones}
    with pytest.raises(ValueError):
        planificar_camino(np.zeros((2, 6)), np.ones(6), **args)


def test_cuspide_frena_localmente_al_refinar_un_corazon():
    duraciones = []
    for n in (65, 129):
        u = np.linspace(0, 2*np.pi, n)
        x = .04 * np.sin(u)**3
        y = .04/16 * (13*np.cos(u) - 5*np.cos(2*u) - 2*np.cos(3*u) - np.cos(4*u))
        camino = np.column_stack([x, .2*x+y, x-y, y, np.zeros(n), -x])
        limites = np.ones(6)
        p = planificar_camino(camino, limites, .08, t_acel=.5, t_min=.01)
        duraciones.append(p.tf)
        # El interior de la cúspide tiene tramos muy cortos. Su velocidad local
        # baja, pero el resto del ciclo puede recorrer los segmentos más largos.
        _, velocidad_punta, _ = p.evaluar(p.tiempos_nudos[n//2])
        assert np.max(abs(velocidad_punta)) < .05 * np.max(p.V)
        assert np.max(p.V) > .25 * .08
        assert np.all(p.V <= .08 + 1e-12)
        assert np.all(p.A <= .08/.5 + 1e-12)
    assert duraciones[1] == pytest.approx(duraciones[0], rel=.1)
