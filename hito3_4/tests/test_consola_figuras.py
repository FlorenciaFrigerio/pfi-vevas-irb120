"""Integración de figuras y perfiles con la consola SIM, sin hilos de movimiento."""

import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from vevas import pieper
from vevas.backend_sim import BackendSim
from vevas.consola import Consola
from vevas.figuras import generar_figura
from vevas.registro import Registro
from vevas.robot_irb120 import error_orientacion
from vevas.seguridad import cargar_config


@pytest.fixture
def consola(tmp_path):
    cfg = cargar_config(Path(__file__).resolve().parents[1] / "config.json")
    q0 = np.radians(cfg["postura_inicial_deg"])
    registro = Registro(tmp_path, "test_sim")
    try:
        yield Consola(cfg, BackendSim(q0, ruido_deg=0), registro)
    finally:
        registro.cerrar()


@pytest.mark.parametrize(("nombre", "plano"), [
    ("círculo", "XY"), ("corazón", "XZ"), ("cuadrado", "YZ"), ("triángulo", "XZ"), ("saludo", "XY"),
])
def test_figura_revisada_con_fk_en_los_nudos_y_en_el_trayecto(consola, nombre, plano):
    q0 = np.radians([0, -20, 30, 0, 40, 0])
    origen = consola.robot.fk_tcp(q0)
    assert consola.cfg["entorno"]["mesa"]["cajas"]
    plan = consola.calcular_figura(nombre, 25, plano)

    assert plan["tipo"] == "figura"
    assert plan["ejecutable"], plan["problemas"]
    assert plan["figura"]["n_puntos"] == 65
    assert plan["figura"]["falla_punto"] is None
    assert plan["tray"]["perfil"] == "camino_quintico"
    assert plan["tray"]["tc"] is None
    np.testing.assert_allclose(plan["q0_deg"], np.degrees(q0), atol=1e-10)
    np.testing.assert_allclose(plan["q_obj_rad"], q0, atol=1e-12)

    tray = consola.plan["_tray"]
    tiempos = np.asarray(plan["tray"]["tiempos_nudos"])
    puntos = np.asarray(plan["preview"]["puntos"])
    camino = np.asarray(plan["preview"]["camino"])
    assert tiempos.shape == (65,)
    assert np.all(np.diff(tiempos) > 0)
    assert puntos.shape == (65, 3)
    assert camino.ndim == 2 and camino.shape[1] == 3
    assert len(camino) == len(plan["preview"]["t"]) == len(plan["preview"]["frames"])
    np.testing.assert_allclose(camino[[0, -1]], np.tile(origen[:3, 3], (2, 1)), atol=1e-12)
    assert np.linalg.norm(np.ptp(camino, axis=0)) > 0.005

    Q = np.array([tray.evaluar(t)[0] for t in tiempos])
    fk_nudos = np.array([consola.robot.fk_tcp(q) for q in Q])
    np.testing.assert_allclose(Q[[0, -1]], np.tile(q0, (2, 1)), atol=1e-12)
    assert np.max(np.abs(np.degrees(np.diff(Q, axis=0)))) < 2.0
    tol_pos = consola.cfg["ik"]["tol_e_p_m"]
    tol_rot = consola.cfg["ik"]["tol_e_R_rad"]
    assert np.linalg.norm(fk_nudos[:, :3, 3] - puntos, axis=1).max() < tol_pos
    assert plan["verificacion"]["e_p_m"] < tol_pos
    assert plan["verificacion"]["e_R_rad"] < tol_rot
    if nombre == "saludo":
        assert plan["figura"]["metodo"] == "articular"
        assert plan["figura"]["ramas"] == []
        np.testing.assert_allclose(Q[:, :3], np.tile(q0[:3], (65, 1)), atol=1e-12)
        assert np.ptp(Q[:, 5]) > np.radians(5)
    else:
        assert len(plan["figura"]["ramas"]) == 64
        np.testing.assert_allclose(puntos, origen[:3, 3] + generar_figura(nombre, 25, plano))
        assert max(error_orientacion(T[:3, :3], origen[:3, :3]) for T in fk_nudos) < tol_rot

    medios = (tiempos[:-1] + tiempos[1:]) / 2
    fk_medios = np.array([consola.robot.fk_tcp(tray.evaluar(t)[0]) for t in medios])
    segmentos = np.diff(puntos, axis=0)
    desplazamiento = fk_medios[:, :3, 3] - puntos[:-1]
    avance = np.sum(desplazamiento * segmentos, axis=1) / np.sum(segmentos**2, axis=1)
    residual = np.linalg.norm(desplazamiento - avance[:, None] * segmentos, axis=1)
    assert np.min(avance) > 0.2 and np.max(avance) < 0.8
    # Tolerancia geométrica intermedia: las uniones se interpolan en juntas.
    assert np.max(residual) < 0.0001
    if nombre != "saludo":
        assert max(error_orientacion(T[:3, :3], origen[:3, :3]) for T in fk_medios) < 0.00025

    for previo, nudo, siguiente in zip(tiempos, tiempos[1:-1], tiempos[2:]):
        eps = min(nudo - previo, siguiente - nudo) * 1e-7
        izquierda = tray.evaluar(nudo - eps)
        derecha = tray.evaluar(nudo + eps)
        for antes, despues, tol in zip(izquierda, derecha, (1e-6, 1e-5, 1e-4)):
            np.testing.assert_allclose(antes, despues, rtol=0.0, atol=tol)

    paradas = plan["figura"]["paradas"]
    if nombre == "cuadrado":
        assert paradas == [16, 32, 48]
        assert np.max(np.abs(tray.evaluar(tiempos[8])[1])) > 1e-3
    elif nombre == "triángulo":
        # 64 intervalos en tres lados (22 + 21 + 21): paradas en las dos esquinas intermedias
        assert paradas == [22, 43]
        np.testing.assert_allclose(puntos[22] - puntos[0], generar_figura("triangulo", 25, plano)[22])
        assert np.max(np.abs(tray.evaluar(tiempos[11])[1])) > 1e-3
    elif nombre == "corazón":
        assert paradas == [32]
    else:
        assert paradas == []
    for k in [0, *paradas, len(tiempos) - 1]:
        _, velocidad, aceleracion = tray.evaluar(tiempos[k])
        np.testing.assert_allclose(velocidad, 0.0, atol=1e-10)
        np.testing.assert_allclose(aceleracion, 0.0, atol=1e-10)


def test_perfil_invalida_plan_y_solo_lspb_tiene_fases_trapezoidales(consola):
    primero = consola.calcular_junta(1, 3, True)
    assert primero["tray"]["perfil"] == "quintico"
    assert primero["tray"]["tc"] is None
    assert consola.fijar_perfil("lspb") == "lspb"
    assert consola.plan is None
    siguiente = consola.calcular_junta(1, 3, True)
    assert siguiente["tray"]["perfil"] == "lspb"
    assert siguiente["tray"]["tc"] > 0
    figura = consola.calcular_figura("cuadrado")
    assert figura["tray"]["perfil"] == "camino_quintico"
    assert figura["tray"]["tc"] is None


@pytest.mark.parametrize("ocupacion", ["ejec", "jog", "envio_pendiente"])
def test_cambio_de_perfil_rechazado_durante_movimiento_o_envio(consola, ocupacion):
    original = consola.plan = {"id": 42}
    setattr(consola, ocupacion, True)
    with pytest.raises(RuntimeError, match="termine el movimiento"):
        consola.fijar_perfil("lspb")
    assert consola.plan is original
    assert consola.cfg["trayectoria"]["perfil"] == "suave"


@pytest.mark.parametrize("perfil", [None, 3, True, [], {}, "lineal"])
def test_tipo_de_perfil_invalido_no_modifica_el_plan(consola, perfil):
    original = consola.plan = {"id": 42}
    with pytest.raises(ValueError, match="suave o lspb"):
        consola.fijar_perfil(perfil)
    assert consola.plan is original
    assert consola.cfg["trayectoria"]["perfil"] == "suave"


@pytest.mark.parametrize("datos", [
    {"tipo": "ros"},
    {"tipo": "ros", "velocidad_articular_verificada": True, "seguimiento_por_llegada": True},
    {"tipo": "rws"},
    {"tipo": "rws", "velocidad_articular_verificada": False, "seguimiento_por_llegada": True},
    {"tipo": "rws", "velocidad_articular_verificada": True, "seguimiento_por_llegada": False},
])
def test_figuras_y_j5_rechazados_sin_rws_c02_sin_consultar_backend(consola, datos):
    backend = SimpleNamespace(leer=Mock(), info=Mock(return_value={"tipo": datos["tipo"], "real": True}), **datos)
    consola.backend = backend
    for figura in ("círculo", "triángulo", "saludo"):
        with pytest.raises(RuntimeError, match="C02"):
            consola.calcular_figura(figura)
    with pytest.raises(ValueError, match="C02"):
        consola.calcular_home("j5")
    backend.leer.assert_not_called()
    assert consola.plan is None
    modo = consola.modo()
    assert modo["figuras_permitidas"] is False and "C02" in modo["figuras_motivo"]


# ---- robot real por RWS con C02 verificado (sin red: simulador con la interfaz de RWS)
from test_revision import CHECK, FRASE, go_valido  # noqa: E402


class FalsoRWS(BackendSim):
    tipo = "rws"
    velocidad_articular_verificada = True
    seguimiento_por_llegada = True

    def info(self):
        return {"tipo": "rws", "hardware": "IRC5 falso", "real": True, "texto": "RWS falso", "solo_lectura": False,
                "velocidad_articular_verificada": self.velocidad_articular_verificada, "vlim_c02_deg_s": None}

    def leer(self):
        q, _ = super().leer()
        return q, time.monotonic()

    def ejecutar_trayectoria(self, ts, Q, QD=None, **opciones):
        self.enviados.append((ts, Q, opciones))
        return True, "falso"


def _consola_rws(tmp_path, q0_deg, verificada=True):
    cfg = cargar_config(Path(__file__).resolve().parents[1] / "config.json")
    cfg["entorno"].update({"cajas": [], "sin_objetos_externos_confirmado": True, "mesa_z_m": 0.0,
                           "sin_herramienta_confirmada": True, "herramienta": {"radio_mm": 30.0, "confirmada": False}})
    backend = FalsoRWS(np.radians(q0_deg), ruido_deg=0)
    backend.velocidad_articular_verificada = verificada
    backend.enviados = []
    registro = Registro(tmp_path, "test_rws_c02")
    return Consola(cfg, backend, registro, permitir_real=True, go=go_valido()), registro


@pytest.fixture
def consola_rws(tmp_path):
    con, registro = _consola_rws(tmp_path, [0, -20, 30, 0, 40, 0])
    try:
        yield con
    finally:
        registro.cerrar()


def test_rws_c02_habilita_figuras_con_revision_real(consola_rws):
    modo = consola_rws.modo()
    assert modo["exige_go"] and modo["figuras_permitidas"] and modo["figura_max_mm"] == 80
    for nombre in ("círculo", "corazón", "cuadrado", "triángulo", "saludo"):
        plan = consola_rws.calcular_figura(nombre, 40, "XY")
        assert plan["ejecutable"], (nombre, plan["problemas"])
        assert plan["revision"]["estado"] in ("aprobado", "aviso")
        assert next(c for c in plan["revision"]["chequeos"] if c["id"] == "limites")["estado"] == "aprobado"
        assert plan["vel_pct"] <= modo["vel_max_pct"]
        if nombre == "triángulo":
            assert plan["figura"]["paradas"] == [22, 43]
            assert plan["descripcion"].startswith("Triángulo")


def test_rws_c02_limita_el_tamano_de_la_figura(consola_rws):
    with pytest.raises(ValueError, match="80 mm"):
        consola_rws.calcular_figura("cuadrado", 81)
    consola_rws.backend.leer = Mock(side_effect=AssertionError("no debe leer"))
    with pytest.raises(ValueError, match="80 mm"):
        consola_rws.calcular_figura("triángulo", 100)
    assert consola_rws.plan is None


def test_rws_c02_envio_de_figura_conserva_las_compuertas(consola_rws):
    plan = consola_rws.calcular_figura("triángulo", 30)
    ok, mensaje = consola_rws.ejecutar(plan["id"])
    assert not ok and "ARMAR" in mensaje
    assert consola_rws.backend.enviados == []
    assert not consola_rws.armar({}, "")[0]                 # sin checklist ni frase no arma
    assert consola_rws.armar(CHECK, FRASE)[0]
    ok, mensaje = consola_rws.ejecutar(plan["id"])
    assert ok, mensaje
    (ts, Q, opciones), = consola_rws.backend.enviados
    assert {"origen_revisado", "cancelar", "comprobar_go"} <= set(opciones)
    np.testing.assert_allclose(Q[-1], plan["q_obj_rad"], atol=1e-12)


def test_rws_c02_figura_rechaza_si_la_postura_cambio(consola_rws, monkeypatch):
    plan = consola_rws.calcular_figura("cuadrado", 30)
    q = consola_rws.plan["_tray"].q0.copy()
    q[0] += np.radians(consola_rws.cfg["seguridad"]["inicio_max_deg"] + 0.5)
    monkeypatch.setattr(consola_rws.backend, "leer", lambda: (q, time.monotonic()))
    assert consola_rws.armar(CHECK, FRASE)[0]
    ok, mensaje = consola_rws.ejecutar(plan["id"])
    assert not ok and "Recalculá" in mensaje
    assert consola_rws.backend.enviados == []


@pytest.mark.parametrize("q0_deg", [[0, -20, 30, 0, 0, 0], [0, 0, -25.2, 0, -0.05, 0]])
def test_rws_c02_preparar_j5_sale_de_la_singularidad(tmp_path, q0_deg):
    con, registro = _consola_rws(tmp_path, q0_deg)
    try:
        with pytest.raises(RuntimeError, match="Preparar J5 a \\+30°"):
            con.calcular_figura("círculo")
        assert con.plan is None
        plan = con.calcular_home("j5")
        assert plan["ejecutable"], plan["problemas"]
        assert plan["preparar_j5"]
        assert plan["q_obj_deg"][4] == pytest.approx(30.0)
        for i in (0, 1, 2, 3, 5):
            assert plan["q_obj_deg"][i] == plan["q0_deg"][i]
        assert next(c for c in plan["revision"]["chequeos"] if c["id"] == "singularidad")["estado"] == "aviso"
    finally:
        registro.cerrar()


def test_rws_sin_c02_rechaza_j5_y_figuras(tmp_path):
    con, registro = _consola_rws(tmp_path, [0, -20, 30, 0, 40, 0], verificada=False)
    try:
        assert con.modo()["figuras_permitidas"] is False
        with pytest.raises(ValueError, match="C02"):
            con.calcular_home("j5")
        with pytest.raises(RuntimeError, match="C02"):
            con.calcular_figura("triángulo")
        assert con.plan is None
    finally:
        registro.cerrar()


def test_cambio_de_origen_no_convierte_figura_en_un_punto_a_punto(consola, monkeypatch):
    plan = consola.calcular_figura("círculo")
    tray = consola.plan["_tray"]
    q_desplazada = tray.q0.copy()
    q_desplazada[0] += np.radians(consola.cfg["seguridad"]["inicio_max_deg"] + 0.5)
    monkeypatch.setattr(consola.backend, "leer", lambda: (q_desplazada, time.monotonic()))
    envio = Mock()
    recalcular = Mock()
    monkeypatch.setattr(consola.backend, "ejecutar_trayectoria", envio)
    monkeypatch.setattr(consola, "_planificar_punto", recalcular)
    assert consola.armar({}, "")[0]

    ok, mensaje = consola.ejecutar(plan_id=plan["id"])

    assert not ok and "postura inicial cambió" in mensaje
    assert consola.plan["_tray"] is tray
    recalcular.assert_not_called()
    envio.assert_not_called()


def test_envio_sim_preserva_el_recorrido_y_sus_derivadas(consola, monkeypatch):
    plan = consola.calcular_figura("cuadrado")
    tray = consola.plan["_tray"]
    monkeypatch.setattr(consola.backend, "leer", lambda: (tray.q0, time.monotonic()))
    envio = Mock(wraps=consola.backend.ejecutar_trayectoria)
    monkeypatch.setattr(consola.backend, "ejecutar_trayectoria", envio)
    assert consola.armar({}, "")[0]

    ok, mensaje = consola.ejecutar(plan_id=plan["id"])

    assert ok, mensaje
    envio.assert_called_once()
    ts, Q, QD = envio.call_args.args
    QDD = envio.call_args.kwargs["QDD"]
    esperado = tray.muestrear(consola.cfg["trayectoria"]["dt_envio_s"])
    for real, previsto in zip((ts, Q, QD, QDD), esperado):
        np.testing.assert_array_equal(real, previsto)
    assert consola.backend.en_movimiento()


def test_falla_de_ik_bloquea_envio_de_la_figura_aunque_el_prefijo_sea_valido(consola, monkeypatch):
    resolver = pieper.resolver
    n_soluciones = 0

    def resolver_hasta_fallar(*args):
        nonlocal n_soluciones
        n_soluciones += 1
        return resolver(*args) if n_soluciones < 5 else {"ramas": []}

    monkeypatch.setattr(pieper, "resolver", resolver_hasta_fallar)
    plan = consola.calcular_figura("círculo")
    assert len(plan["figura"]["ramas"]) == 4
    assert plan["figura"]["falla_punto"] == 5
    assert not plan["ejecutable"]
    assert plan["revision"]["estado"] == "rechazado"
    assert plan["revision"]["bloquea_envio"]
    chequeo = next(c for c in plan["revision"]["chequeos"] if c["id"] == "ik_figura")
    assert chequeo["estado"] == "rechazado"
    assert "Punto 6/65" in chequeo["detalle"]

    q0 = consola.plan["_tray"].q0
    monkeypatch.setattr(consola.backend, "leer", lambda: (q0, time.monotonic()))
    envio = Mock(return_value=(True, "sim"))
    monkeypatch.setattr(consola.backend, "ejecutar_trayectoria", envio)
    assert consola.armar({}, "")[0]

    ok, mensaje = consola.ejecutar(plan_id=plan["id"])

    assert not ok and "rechazado" in mensaje
    envio.assert_not_called()


def test_sim_en_j5_cero_la_figura_sugiere_preparar_j5(tmp_path):
    cfg = cargar_config(Path(__file__).resolve().parents[1] / "config.json")
    registro = Registro(tmp_path, "test_sim_j5")
    try:
        con = Consola(cfg, BackendSim(np.radians([0, -20, 30, 0, 0, 0]), ruido_deg=0), registro)
        plan = con.calcular_figura("triángulo", 25)
        assert not plan["ejecutable"]
        assert "Preparar J5 a +30°" in plan["problemas"][0]
    finally:
        registro.cerrar()


# ---- vista previa de puntos (no calcula plan ni mueve)
@pytest.mark.parametrize(("nombre", "plano", "paradas", "ejes"), [
    ("cuadrado", "XY", [16, 32, 48], ["X", "Y"]),
    ("triángulo", "XZ", [22, 43], ["X", "Z"]),
    ("corazón", "YZ", [32], ["Y", "Z"]),
    ("círculo", "XY", [], ["X", "Y"]),
])
def test_vista_figura_devuelve_puntos_y_paradas_iguales_al_plan(consola, nombre, plano, paradas, ejes):
    from vevas.servidor import ORDENES
    r = ORDENES["vista_figura"](consola, {"figura": nombre, "tamano_mm": 30, "plano": plano})
    assert r["ok"], r
    vista = r["vista"]
    assert consola.plan is None                         # no crea plan
    assert vista["paradas"] == paradas and vista["ejes"] == ejes and vista["plano_vista"] == plano
    assert vista["n_puntos"] == 65 and vista["inicio"] == 0 and vista["fin"] == 64
    esperado = generar_figura(nombre, 30, plano) * 1000
    np.testing.assert_allclose(vista["offsets_mm"], esperado, atol=1e-9)
    i, j = {"XY": (0, 1), "XZ": (0, 2), "YZ": (1, 2)}[plano]
    np.testing.assert_allclose(vista["puntos_2d_mm"], esperado[:, [i, j]], atol=1e-9)
    # ancho igual al tamaño en el primer eje del plano
    assert np.ptp(np.asarray(vista["puntos_2d_mm"])[:, 0]) == pytest.approx(30, abs=1e-9)
    origen = consola.robot.fk_tcp(consola.backend.leer()[0])[:3, 3]
    np.testing.assert_allclose(vista["origen_m"], origen, atol=1e-12)
    # misma geometría y mismas paradas que el plan calculado
    plan = consola.calcular_figura(nombre, 30, plano)
    np.testing.assert_allclose(vista["puntos_m"], plan["figura"]["puntos_objetivo_m"], atol=1e-12)
    assert plan["figura"]["paradas"] == vista["paradas"]
    assert plan["figura"]["plano_vista"] == plano


def test_vista_figura_saludo_usa_la_postura_actual(consola):
    vista = consola.vista_figura("saludo", 25)
    plan = consola.calcular_figura("saludo", 25)
    np.testing.assert_allclose(vista["puntos_m"], plan["figura"]["puntos_objetivo_m"], atol=1e-12)
    assert vista["paradas"] == [] and vista["metodo"] == "articular"
    assert vista["plano_vista"] == plan["figura"]["plano_vista"]
    assert np.ptp(np.asarray(vista["puntos_2d_mm"]), axis=0).max() > 1.0


def test_vista_figura_valida_sin_registrar_avisos(consola):
    from vevas.servidor import ORDENES
    n = consola.n_evento
    for cuerpo in ({"figura": "estrella"}, {"figura": "circulo", "tamano_mm": 4},
                   {"figura": "circulo", "tamano_mm": 101}, {"figura": "circulo", "plano": "XX"}):
        r = ORDENES["vista_figura"](consola, cuerpo)
        assert r["ok"] is False and r["mensaje"]
    assert consola.n_evento == n


def test_vista_figura_en_rws_sin_c02_se_ve_pero_avisa(tmp_path):
    con, registro = _consola_rws(tmp_path, [0, -20, 30, 0, 40, 0], verificada=False)
    try:
        vista = con.vista_figura("cuadrado", 90)
        assert vista["permitida"] is False and vista["excede_tope"] is True and vista["tope_mm"] == 80
        assert con.plan is None and con.backend.enviados == []
    finally:
        registro.cerrar()
