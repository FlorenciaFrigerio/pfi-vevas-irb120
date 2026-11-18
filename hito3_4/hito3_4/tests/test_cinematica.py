"""Pruebas del núcleo de cálculo. Correr con:  python3 -m pytest -q tests/"""

import math
import os
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vevas.jacobiano import analisis_singularidad, curva_error_vs_h, determinante_analitico, errores_jacobiano, jacobiano  # noqa: E402
from vevas.pieper import elegir_rama, resolver  # noqa: E402
from vevas.robot_irb120 import (  # noqa: E402
    LIMITES,
    Robot,
    error_orientacion,
    error_orientacion_arccos,
    matriz_a_cuaternion,
    matriz_a_euler_zyx,
    euler_zyx_a_matriz,
    pose_desde_xyz_euler,
)
from vevas.seguridad import CONFIG_POR_DEFECTO, revisar_plan  # noqa: E402
from vevas.trayectoria import comprobar_continuidad, planificar  # noqa: E402

R = Robot()


def q_aleatorio(rng):
    return np.array([rng.uniform(lo, hi) for lo, hi in LIMITES])


def test_fk_en_cero_coincide_con_manual_abb():
    T = R.fk_tool0([0] * 6)
    assert np.allclose(T[:3, 3], [0.374, 0.0, 0.630], atol=1e-12)
    # Orientación de tool0 en la posición de calibración: [0.7071, 0, 0.7071, 0]
    assert np.allclose(matriz_a_cuaternion(T[:3, :3]), [math.sqrt(0.5), 0, math.sqrt(0.5), 0], atol=1e-12)


def test_centro_de_muneca_en_cero():
    M = R.marcos([0] * 6)
    assert np.allclose(M[4][:3, 3], [0.302, 0, 0.630], atol=1e-12)


def test_rotaciones_validas():
    rng = np.random.default_rng(0)
    for _ in range(200):
        Rm = R.fk_tcp(q_aleatorio(rng))[:3, :3]
        assert np.allclose(Rm.T @ Rm, np.eye(3), atol=1e-12)
        assert abs(np.linalg.det(Rm) - 1.0) < 1e-12


def test_euler_ida_y_vuelta():
    rng = np.random.default_rng(1)
    for _ in range(200):
        a = rng.uniform(-math.pi, math.pi, 3)
        a[1] = rng.uniform(-1.5, 1.5)
        Rm = euler_zyx_a_matriz(*a)
        assert np.allclose(euler_zyx_a_matriz(*matriz_a_euler_zyx(Rm)), Rm, atol=1e-12)


def test_pieper_reproduce_la_configuracion_original():
    """Para q aleatorias: alguna de las 8 ramas devuelve exactamente q, y todas tienen error FK ~ 0."""
    rng = np.random.default_rng(2)
    peor = 0.0
    for _ in range(2000):
        q = q_aleatorio(rng)
        q[5] = rng.uniform(-math.pi, math.pi)
        out = resolver(R, R.fk_tcp(q), q)
        existentes = [r for r in out["ramas"] if r.existe]
        assert existentes, "debe existir al menos una rama"
        assert min(np.max(np.abs(np.array(r.q) - q)) for r in existentes) < 1e-7
        peor = max(peor, max(max(r.e_p, r.e_R) for r in existentes))
    assert peor < 1e-9


def test_pieper_da_8_ramas_en_pose_comoda():
    q = np.radians([10, -10, 20, 30, 50, 10])
    out = resolver(R, R.fk_tcp(q), q)
    assert sum(r.existe for r in out["ramas"]) == 8
    for r in out["ramas"]:
        assert r.e_p < 1e-12 and r.e_R < 1e-9


def test_eleccion_de_rama_es_la_mas_cercana_y_valida():
    q = np.radians([0, -20, 30, 0, 40, 0])
    out = resolver(R, R.fk_tcp(q), q)
    mejor, razon = elegir_rama(out["ramas"], 1e-6, 1e-5, math.radians(5), 0.02)
    assert mejor is not None
    assert np.allclose(mejor.q, q, atol=1e-9)
    assert "válidas" in razon


def test_pose_fuera_de_alcance_no_tiene_ramas():
    T = pose_desde_xyz_euler(900, 0, 400, 0, 180, 0)
    out = resolver(R, T, [0] * 6)
    assert not any(r.existe for r in out["ramas"])
    mejor, _ = elegir_rama(out["ramas"], 1e-6, 1e-5, 0.08, 0.02)
    assert mejor is None


def test_jacobiano_contra_diferencias_finitas():
    rng = np.random.default_rng(3)
    for _ in range(50):
        e = errores_jacobiano(R, q_aleatorio(rng), 1e-6)
        assert e["e_Jv"] < 1e-8 and e["e_Jw"] < 1e-8


def test_curva_error_vs_h_tiene_forma_de_v():
    curva = curva_error_vs_h(R, np.radians([10, -20, 30, 40, 50, 60]))
    errores = [c[1] for c in curva]
    k = int(np.argmin(errores))
    assert 0 < k < len(errores) - 1  # mínimo en el medio: truncamiento a la derecha, redondeo a la izquierda


def test_determinante_analitico():
    rng = np.random.default_rng(4)
    for _ in range(200):
        q = q_aleatorio(rng)
        assert abs(np.linalg.det(jacobiano(R, q)) - determinante_analitico(q)) < 1e-12


def test_postura_cero_es_singular():
    a = analisis_singularidad(R, [0] * 6)
    assert a["rango"] == 5
    assert a["muneca_sin_q5"] < 1e-12


def test_error_orientacion_atan2_es_mas_fino_que_arccos():
    Ra = euler_zyx_a_matriz(0.3, 0.2, 0.1)
    Rb = Ra @ euler_zyx_a_matriz(1e-9, 0, 0)
    assert abs(error_orientacion(Rb, Ra) - 1e-9) < 1e-15
    assert abs(error_orientacion_arccos(Rb, Ra) - 1e-9) > 1e-10  # arccos no resuelve 1e-9


def test_lspb_sincronizado_continuo_y_dentro_de_velocidad():
    q0 = np.radians([0, -20, 30, 0, 40, 0])
    qf = np.radians([40, 10, -10, 60, 20, 90])
    vmax = R.vel_max
    t = planificar(q0, qf, vmax, 0.05)
    _, Q, QD, _ = t.muestrear(0.001)
    assert np.allclose(Q[0], q0) and np.allclose(Q[-1], qf, atol=1e-12)
    assert np.all(np.abs(QD) <= np.asarray(vmax) * 0.05 + 1e-9)
    c = comprobar_continuidad(t)
    assert c["salto_q_max_deg"] < 0.05 and c["salto_qd_max_deg_s"] < 0.1
    assert abs(t.tc - 0.25 * t.tf) < 1e-12


def test_revisar_plan_detecta_altura_minima():
    q0 = np.radians([0, -20, 30, 0, 40, 0])
    qf = np.radians([0, 90, 60, 0, 40, 0])  # brazo hacia la mesa
    t = planificar(q0, qf, R.vel_max, 0.05)
    _, Q, _, _ = t.muestrear(0.02)
    problemas = revisar_plan(R, Q, q0, CONFIG_POR_DEFECTO)
    assert any("altura" in p or "límite" in p for p in problemas)


def test_consola_en_simulacion_ejecuta_un_plan(tmp_path):
    from vevas.backend_sim import BackendSim
    from vevas.consola import Consola
    from vevas.registro import Registro

    cfg = dict(CONFIG_POR_DEFECTO)
    q0 = np.radians(cfg["postura_inicial_deg"])
    b = BackendSim(q0)
    reg = Registro(str(tmp_path), "test")
    c = Consola(cfg, b, reg)
    b.iniciar()
    c.iniciar()
    try:
        time.sleep(0.2)
        c.fijar_velocidad(20)
        plan = c.calcular_pose({"x": 350, "y": -150, "z": 400, "rz": 0, "ry": 180, "rx": 0})
        assert plan["ejecutable"], plan["problemas"]
        assert plan["verificacion"]["e_p_m"] < 1e-9
        ok, msg = c.armar({}, "")
        assert ok, msg
        ok, msg = c.ejecutar()
        assert ok, msg
        t_fin = time.time() + plan["tray"]["tf"] + 3
        while time.time() < t_fin and c.ejec is not None:
            c.pulso()
            time.sleep(0.1)
        assert c.ejec is None
        r = c.ultimo_resumen
        assert r["estado"] == "completado"
        assert r["e_p_vs_pose_pedida_mm"] < 0.5
    finally:
        c.detener()
        b.detener()
        reg.cerrar()
    assert os.path.exists(os.path.join(reg.carpeta, "muestras.csv"))


def test_consola_aborta_si_la_interfaz_no_responde(tmp_path):
    from vevas.backend_sim import BackendSim
    from vevas.consola import Consola
    from vevas.registro import Registro

    cfg = dict(CONFIG_POR_DEFECTO)
    b = BackendSim(np.radians(cfg["postura_inicial_deg"]))
    reg = Registro(str(tmp_path), "test")
    c = Consola(cfg, b, reg)
    b.iniciar()
    c.iniciar()
    try:
        time.sleep(0.2)
        c.calcular_articular([30, -20, 30, 0, 40, 0])
        c.armar({}, "")
        c.pulso()
        ok, _ = c.ejecutar()
        assert ok
        time.sleep(1.6)  # sin pulsos de la interfaz
        assert c.ejec is None and not c.armado
        assert "interfaz" in c.motivo_desarmado
    finally:
        c.detener()
        b.detener()
        reg.cerrar()


def test_robot_real_sin_go_queda_en_solo_lectura(tmp_path):
    from vevas.consola import Consola
    from vevas.registro import Registro

    class FalsoReal:
        tipo = "ros"

        def info(self):
            return {"tipo": "ros", "hardware": "abb_hardware_interface/ABBSystemHardware", "real": True, "texto": "real"}

        def leer(self):
            return np.zeros(6), time.monotonic()

    reg = Registro(str(tmp_path), "test")
    c = Consola(dict(CONFIG_POR_DEFECTO), FalsoReal(), reg, permitir_real=False)
    ok, msg = c.armar({"supervisor": True, "paro": True, "area": True, "velocidad": True}, "MOVER ROBOT REAL")
    assert not ok and "SOLO LECTURA" in msg
    c2 = Consola(dict(CONFIG_POR_DEFECTO), FalsoReal(), reg, permitir_real=True, go=None)
    ok, msg = c2.armar({"supervisor": True, "paro": True, "area": True, "velocidad": True}, "MOVER ROBOT REAL")
    assert not ok and "GO" in msg
    reg.cerrar()
