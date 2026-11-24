"""Slider de velocidad hasta 50 % (config, GO, consola), tope físico C02 con RWS y velocidad MEDIDA por junta."""
import csv
import datetime as dt
import os
import time
from pathlib import Path

import numpy as np
import pytest

from vevas.backend_rws import BackendRWS
from vevas.backend_sim import BackendSim
from vevas.consola import CHECKLIST_GO, Consola, validar_go
from vevas.registro import Registro
from vevas.revision import revisar_trayectoria
from vevas.robot_irb120 import VEL_MAX_DEG, Robot
from vevas.rws_cliente import VLIM_DEG_S_DEFECTO
from vevas.seguridad import cargar_config
from vevas.trayectoria import planificar_suave
from vevas.velocidad import VEL_PCT_MAX, MedidorVelocidad, pct_valido

RAIZ = Path(__file__).resolve().parents[1]
CHECK = {k: True for k in ("supervisor", "paro", "area", "velocidad")}
FRASE = "MOVER ROBOT REAL"


def _cfg():
    return cargar_config(str(RAIZ / "config.json"))


def _go(pct):
    return {"fecha": dt.date.today().isoformat(), "decision": "GO", "autoriza": "Docente", "operador_pendant": "A",
            "responsable_consola": "B", "supervisor": "C", "alcance": "ensayo", "velocidad_max_pct": pct,
            "checklist": {k: True for k in CHECKLIST_GO}}


# ---------------------------------------------------------------- topes 50 % en config, GO y consola
def test_config_permite_50_y_arranca_en_5():
    s = _cfg()["seguridad"]
    assert s["velocidad_sim_max_pct"] == 50 and s["velocidad_real_max_pct"] == 50 and s["velocidad_inicial_pct"] == 5
    assert VEL_PCT_MAX == 50


@pytest.mark.parametrize("clave,valor", [("velocidad_real_max_pct", 60), ("velocidad_sim_max_pct", 100),
                                         ("velocidad_inicial_pct", 0), ("velocidad_real_max_pct", "10")])
def test_config_fuera_de_rango_se_rechaza(tmp_path, clave, valor):
    import json
    ruta = tmp_path / "config.json"
    ruta.write_text(json.dumps({"seguridad": {clave: valor}}), encoding="utf-8")
    with pytest.raises(ValueError, match="0 < v ≤ 50"):
        cargar_config(str(ruta))


def test_validacion_coherente_0_a_50():
    assert pct_valido(50) and pct_valido(0.5) and not pct_valido(0) and not pct_valido(50.01)
    assert not pct_valido(True) and not pct_valido(float("nan")) and not pct_valido("5")
    assert validar_go(_go(50))[0]
    assert not validar_go(_go(51))[0] and "≤ 50" in validar_go(_go(51))[1]
    assert not validar_go(_go(0))[0]


def _consola_sim(tmp_path):
    cfg = _cfg()
    b = BackendSim(np.radians(cfg["postura_inicial_deg"]), ruido_deg=0)
    reg = Registro(str(tmp_path), "test_vel")
    return Consola(cfg, b, reg), reg


def test_consola_sim_slider_hasta_50(tmp_path):
    con, reg = _consola_sim(tmp_path)
    try:
        m = con.modo()
        assert m["vel_max_pct"] == 50 and m["vel_pct_max_abs"] == 50 and m["vel_pct_verificado"] == 10
        assert con.vel_pct == 5
        assert con.fijar_velocidad(30) == 30
        assert con.fijar_velocidad(80) == 50
        with pytest.raises(ValueError):
            con.fijar_velocidad(float("nan"))
        plan = con.calcular_junta(1, 20.0, True)
        assert plan["ejecutable"] and plan["vel_pct"] == 50
        v = next(c for c in plan["revision"]["chequeos"] if c["id"] == "velocidad")
        assert v["estado"] == "aprobado"
    finally:
        reg.cerrar()


# ---------------------------------------------------------------- robot real por RWS con C02
class FalsoRWSC02(BackendSim):
    """IRC5 sin red con la interfaz de RWS y los topes vLim del módulo; el tiempo C02 lo calcula BackendRWS."""
    tipo = "rws"
    velocidad_articular_verificada = True
    seguimiento_por_llegada = True
    vlim_c02 = VLIM_DEG_S_DEFECTO
    k_c02 = 2.5
    t_zona_s = 0.2
    paso_deg = 40.0
    paso_zona_deg = 10.0
    tol_zona_deg = 0.1
    signos = np.ones(6)
    offsets = np.zeros(6)
    duracion_c02_s = BackendRWS.duracion_c02_s
    plan_recorrido = BackendRWS.plan_recorrido
    paradas_de = staticmethod(BackendRWS.paradas_de)
    _a_rws_deg = BackendRWS._a_rws_deg

    def info(self):
        return {"tipo": "rws", "hardware": "IRC5 falso", "real": True, "texto": "RWS falso", "solo_lectura": False,
                "velocidad_articular_verificada": True, "vlim_c02_deg_s": list(self.vlim_c02)}

    def leer(self):
        q, _ = super().leer()
        return q, time.monotonic()

    def ejecutar_trayectoria(self, ts, Q, QD=None, **opciones):
        self.enviados.append((ts, Q, opciones))
        return True, "falso"


def _consola_rws(tmp_path, go_pct=50):
    cfg = _cfg()
    cfg["entorno"].update({"cajas": [], "sin_objetos_externos_confirmado": True, "mesa_z_m": 0.0,
                           "sin_herramienta_confirmada": True, "herramienta": {"radio_mm": 30.0, "confirmada": False}})
    b = FalsoRWSC02(np.radians([0, -20, 30, 0, 40, 0]), ruido_deg=0)
    b.enviados = []
    reg = Registro(str(tmp_path), "test_vel_rws")
    return Consola(cfg, b, reg, permitir_real=True, go=_go(go_pct)), reg


def test_real_rws_slider_30_no_rechaza_y_estima_tiempo_c02(tmp_path):
    con, reg = _consola_rws(tmp_path)
    try:
        m = con.modo()
        assert m["vel_max_pct"] == 50 and m["tope_fisico_c02"] is True
        assert con.armar(CHECK, FRASE)[0]
        assert con.fijar_velocidad(30) == 30
        plan = con.calcular_junta(1, 20.0, True)
        assert plan["ejecutable"], plan["problemas"]
        assert next(c for c in plan["revision"]["chequeos"] if c["id"] == "velocidad")["estado"] == "aprobado"
        fis = next(c for c in plan["revision"]["chequeos"] if c["id"] == "velocidad_fisica")
        assert fis["estado"] == "aviso" and fis["juntas"] == [1]
        assert "RAPID limita J1 a 12.5°/s" in fis["detalle"] and "tardará ≈" in fis["detalle"]
        # 20° en J1 con vLim 12.5°/s y kPico 2.5: el RAPID tarda ≥ 4 s (+ piso de 0.5 s del último tramo
        # fine si queda corto); el plan al 30 % tarda mucho menos
        t_c02 = plan["t_c02_s"]
        assert 4.0 - 1e-9 <= t_c02 <= 4.5 + 1e-9
        assert t_c02 > 2 * plan["tray"]["tf"]
        assert plan["vlim_c02_deg_s"] == list(VLIM_DEG_S_DEFECTO)
        ok, msg = con.ejecutar(plan["id"])
        assert ok, msg
        e = con.ejec
        assert e["t_c02_s"] == pytest.approx(t_c02, abs=1e-9)
        assert e["limite_s"] >= 2 * e["t_c02_s"] + 30 - 1e-9        # la vigilancia de llegada usa el tiempo C02
        assert any(f"≈ {t_c02:.1f} s" in ev["texto"] for ev in con.eventos)
        # velocidad comandada estimada: el plan escalado al tiempo C02 nunca supera vLim
        e["t0"] = time.monotonic() - t_c02 / 2                          # mitad del recorrido real
        vel, fuente = con._vel_plan(None, time.monotonic(), True)
        assert "C02" in fuente and 0 < vel[0] <= 12.5 + 1e-6 and max(abs(x) for x in vel[1:]) < 1e-9
        tope, fuente_tope = con._tope_vel_juntas()
        assert tope == list(VLIM_DEG_S_DEFECTO) and "C02" in fuente_tope
    finally:
        reg.cerrar()


def test_real_rws_al_5_por_ciento_el_tope_fisico_queda_aprobado(tmp_path):
    con, reg = _consola_rws(tmp_path, go_pct=5)
    try:
        assert con.modo()["vel_max_pct"] == 5
        plan = con.calcular_junta(1, 10.0, True)
        assert plan["ejecutable"], plan["problemas"]
        fis = next(c for c in plan["revision"]["chequeos"] if c["id"] == "velocidad_fisica")
        assert fis["estado"] == "aprobado" and "tardará ≈" in fis["detalle"] and 2.0 <= plan["t_c02_s"] <= 2.5
    finally:
        reg.cerrar()


def test_duracion_c02_de_backend_rws_replica_el_rapid():
    cfg = _cfg()
    cfg["rws"].update({"ip": "127.0.0.1", "puerto": 9, "evidencia_c02": "/no/existe.json"})
    be = BackendRWS(cfg)                                           # sin red: solo lee el módulo local
    assert be.vlim_c02 == tuple(VLIM_DEG_S_DEFECTO)
    q0 = np.radians([0, -20, 30, 0, 40, 0])
    qf = q0 + np.radians([10, 0, 0, 0, 0, -21])
    tray = planificar_suave(q0, qf, Robot().vel_max, 0.30)
    ts, Q, QD, _ = tray.muestrear(0.02)
    t = be.duracion_c02_s(Q, QD)
    # recorrido proporcional: manda J6 (2.5·21/21 = 2.5 s) más, a lo sumo, el piso fine (0.5 s) del último tramo corto
    assert 2.5 - 1e-9 <= t <= 3.0 + 1e-9
    assert t > tray.tf
    assert be.duracion_estimada_s(Q[1:]) >= t                       # la cota de vigilancia es más holgada


def test_revision_ros_real_por_encima_del_10_avisa_fuera_de_lo_verificado():
    cfg = _cfg()
    robot = Robot()
    q0 = np.radians([0, -20, 30, 0, 40, 0])
    tray = planificar_suave(q0, q0 + np.radians([10, 0, 0, 0, 0, 0]), robot.vel_max, 0.30)
    ts, Q, QD, _ = tray.muestrear(0.02)
    inf = revisar_trayectoria(robot, ts, Q, QD, q0, cfg, 50, exige_go=True, vel_pct_plan=30)
    c = next(c for c in inf["chequeos"] if c["id"] == "velocidad_verificada")
    assert c["estado"] == "aviso" and "fuera de lo verificado" in c["detalle"]
    inf = revisar_trayectoria(robot, ts, Q, QD, q0, cfg, 50, exige_go=False, vel_pct_plan=30)
    assert not any(c["id"] == "velocidad_verificada" for c in inf["chequeos"])  # en SIM no hace falta


# ---------------------------------------------------------------- velocidad medida
def test_rampa_conocida_da_la_velocidad_correcta_con_muestras_irregulares():
    med = MedidorVelocidad(ventana_s=0.3)
    pend = np.array([10.0, -5.0, 0.0, 2.5, 0.0, 30.0])
    rng = np.random.default_rng(1)
    t = 100.0
    for _ in range(40):
        t += 0.1 + rng.uniform(-0.03, 0.03)                       # ~10 Hz con jitter
        assert med.agregar(t, pend * t)
    np.testing.assert_allclose(med.velocidad(t), pend, atol=1e-9)
    np.testing.assert_allclose(med.max_mov, np.abs(pend), atol=1e-9)


def test_muestras_repetidas_no_cambian_la_velocidad():
    med = MedidorVelocidad()
    pend = np.array([12.5, 0, 0, 0, 0, 0])
    for k in range(10):
        t = 0.1 * k
        assert med.agregar(t, pend * t)
        for _ in range(4):                                         # el bucle de 50 Hz relee la misma muestra RWS
            assert med.agregar(t, pend * t) is False
    assert med.agregar(0.05, pend * 0.05) is False                 # desordenada
    assert med.agregar(1.0, [np.nan] * 6) is False                 # inválida
    np.testing.assert_allclose(med.velocidad(0.9), pend, atol=1e-9)


def test_sin_muestras_recientes_la_velocidad_es_desconocida():
    med = MedidorVelocidad(edad_max_s=1.0)
    assert med.velocidad() is None
    med.agregar(0.0, np.zeros(6))
    assert med.velocidad(0.0) is None                              # una sola muestra
    med.agregar(0.1, np.ones(6))
    assert med.velocidad(0.5) is not None
    assert med.velocidad(1.5) is None                              # el estado dejó de llegar


def test_maximos_y_reinicio():
    med = MedidorVelocidad()
    t, q = 0.0, np.zeros(6)
    for v in (5.0, 20.0, 5.0):                                     # sube y baja: el máximo queda en 20
        for _ in range(6):
            t += 0.1
            q = q + np.array([v, 0, 0, 0, 0, -v]) * 0.1
            med.agregar(t, q)
    assert med.max_mov[0] == pytest.approx(20.0, abs=1e-6) and med.max_mov[5] == pytest.approx(20.0, abs=1e-6)
    assert med.velocidad(t)[0] == pytest.approx(5.0, abs=1e-6)
    med.reiniciar_maximos()
    assert not med.max_mov.any()
    t += 0.1
    q = q + np.array([5.0, 0, 0, 0, 0, 0]) * 0.1
    med.agregar(t, q)
    assert med.max_mov[0] == pytest.approx(5.0, abs=1e-6)


class SimRampa(BackendSim):
    """Estado sintético: J2 en rampa a 8 °/s, nueva muestra cada 0.1 s (lo demás quieto)."""

    def __init__(self, q0):
        super().__init__(q0, ruido_deg=0)
        self.q0 = np.asarray(q0, dtype=float)
        self.t = time.monotonic()

    def leer(self):
        dt_ = self.t - self.t_inicio
        return self.q0 + np.radians([0, 8.0 * dt_, 0, 0, 0, 0]), self.t


def test_consola_manda_y_registra_la_velocidad_medida(tmp_path):
    cfg = _cfg()
    b = SimRampa(np.radians(cfg["postura_inicial_deg"]))
    b.t_inicio = b.t
    reg = Registro(str(tmp_path), "test_vel_ciclo")
    con = Consola(cfg, b, reg)
    try:
        con.pulso()
        for k in range(8):
            b.t = b.t_inicio + 0.1 * k
            for _ in range(5):                                     # 50 Hz de consola sobre 10 Hz de estado
                con._t_muestra = 0.0
                con._ciclo(b.t + 0.01, 0.02)
        e = con.estado()
        np.testing.assert_allclose(e["vel_medida_deg_s"], [0, 8, 0, 0, 0, 0], atol=1e-6)
        assert e["vel_max_mov_deg_s"][1] == pytest.approx(8.0, abs=1e-6)
        assert e["vel_tope_deg_s"] == pytest.approx([v * 0.05 for v in VEL_MAX_DEG])   # SIM: ficha × slider
        assert e["vel_plan_deg_s"] is None                         # sin ejecución en curso
        con.reiniciar_maximos_velocidad()
        assert not any(con.estado()["vel_max_mov_deg_s"])
    finally:
        reg.cerrar()
    with open(os.path.join(reg.carpeta, "muestras.csv"), encoding="utf-8") as f:
        filas = list(csv.DictReader(f))
    assert "v2_med_deg_s" in filas[0]
    assert float(filas[-1]["v2_med_deg_s"]) == pytest.approx(8.0, abs=1e-3)
