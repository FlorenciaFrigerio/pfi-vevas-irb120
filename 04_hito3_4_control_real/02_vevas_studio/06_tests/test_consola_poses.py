"""Poses guardadas: almacén JSON local, validaciones y «Ir a pose» con sus compuertas."""

import json
import os
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from vevas.backend_sim import BackendSim
from vevas.consola import Consola
from vevas.poses_guardadas import AlmacenPoses, validar_nombre
from vevas.registro import Registro
from vevas.robot_irb120 import LIMITES_DEG
from vevas.seguridad import cargar_config
from vevas.servidor import ORDENES

from test_consola_figuras import _consola_rws  # noqa: E402
from test_revision import CHECK, FRASE  # noqa: E402

CFG = Path(__file__).resolve().parents[1] / "config.json"
Q_A = [10.0, -15.0, 25.0, 5.0, 40.0, -20.0]


class SimFresco(BackendSim):
    """Simulador sin hilo cuya lectura siempre es reciente (la postura se fija con .q)."""

    @property
    def q(self):
        return self._q

    @q.setter
    def q(self, valor):
        self._q = np.asarray(valor, dtype=float).copy()

    def leer(self):
        return self._q.copy(), time.monotonic()


@pytest.fixture
def consola(tmp_path):
    cfg = cargar_config(CFG)
    registro = Registro(tmp_path / "sesiones", "test_poses")
    con = Consola(cfg, SimFresco(np.radians(Q_A), ruido_deg=0), registro,
                  ruta_poses=str(tmp_path / "01_datos" / "poses_guardadas.json"))
    try:
        yield con
    finally:
        registro.cerrar()


def _ruta(con):
    return Path(con.poses.ruta)


# ---- almacén
def test_guardar_listar_renombrar_y_borrar(consola):
    pose = consola.guardar_pose_actual("  Sobre   la mesa ")
    assert pose["nombre"] == "Sobre la mesa"
    np.testing.assert_allclose(pose["q_deg"], Q_A, atol=1e-6)
    T = consola.robot.fk_tcp(np.radians(Q_A))
    assert pose["tcp"]["x_mm"] == pytest.approx(T[0, 3] * 1000, abs=1e-3)
    assert pose["tcp"]["z_mm"] == pytest.approx(T[2, 3] * 1000, abs=1e-3)
    assert set(pose["tcp"]) == {"x_mm", "y_mm", "z_mm", "rz_deg", "ry_deg", "rx_deg"}
    assert pose["fecha"][:4].isdigit() and "T" in pose["fecha"]

    datos = json.loads(_ruta(consola).read_text(encoding="utf-8"))
    assert datos["version"] == 1 and [p["nombre"] for p in datos["poses"]] == ["Sobre la mesa"]
    # escritura atómica: no quedan temporales en la carpeta
    assert sorted(os.listdir(_ruta(consola).parent)) == ["poses_guardadas.json"]

    with pytest.raises(ValueError, match="Ya existe"):
        consola.guardar_pose_actual("sobre LA mesa")
    consola.guardar_pose_actual("Otra")
    lista = consola.listar_poses()
    assert [p["nombre"] for p in lista["poses"]] == ["Sobre la mesa", "Otra"] and lista["aviso"] is None

    consola.renombrar_pose("sobre la mesa", "Mesa (2)")
    with pytest.raises(ValueError, match="Ya existe"):
        consola.renombrar_pose("Mesa (2)", "otra")
    with pytest.raises(ValueError, match="No existe"):
        consola.renombrar_pose("Inexistente", "X")
    assert [p["nombre"] for p in consola.listar_poses()["poses"]] == ["Mesa (2)", "Otra"]

    with pytest.raises(ValueError, match="Confirmá"):
        consola.borrar_pose("Otra")
    with pytest.raises(ValueError, match="Confirmá"):
        consola.borrar_pose("Otra", confirmar="si")
    consola.borrar_pose("otra", confirmar=True)
    assert [p["nombre"] for p in consola.listar_poses()["poses"]] == ["Mesa (2)"]
    with pytest.raises(ValueError, match="No existe"):
        consola.borrar_pose("Otra", confirmar=True)


@pytest.mark.parametrize("nombre", ["", "   ", "x" * 41, "../etc", "a/b", 'con "comillas"', "<b>", "-guion", 12, None,
                                    "nul\x00"])
def test_nombres_invalidos(consola, nombre):
    with pytest.raises(ValueError):
        consola.guardar_pose_actual(nombre)
    assert not _ruta(consola).exists()


@pytest.mark.parametrize("nombre", ["Home", "Pose 1", "Sobre la mesa (alta)", "J5 +30°", "Ñandú #2", "x" * 40])
def test_nombres_validos(nombre):
    assert validar_nombre(nombre) == nombre


@pytest.mark.parametrize("q", [
    [0, 0, 0, 0, 0],                                   # 5 juntas
    [0, 0, 0, 0, 0, float("nan")],
    [0, 0, 0, 0, 0, float("inf")],
    [0, 0, 0, "1", 0, 0],
    [0, 0, 0, True, 0, 0],
    [0, 0, 0, 0, LIMITES_DEG[4][1] + 0.5, 0],          # fuera de límites
])
def test_valores_invalidos(tmp_path, q):
    almacen = AlmacenPoses(str(tmp_path / "p.json"), LIMITES_DEG)
    with pytest.raises(ValueError):
        almacen.guardar("Mala", q, {})
    assert not (tmp_path / "p.json").exists()


def test_postura_medida_fuera_de_limites_no_se_guarda(consola, monkeypatch):
    q = np.radians([0, 0, 0, 0, LIMITES_DEG[4][1] + 1, 0])
    monkeypatch.setattr(consola.backend, "leer", lambda: (q, time.monotonic()))
    with pytest.raises(ValueError, match="J5"):
        consola.guardar_pose_actual("Límite")


def test_no_guarda_con_estado_viejo_ni_en_movimiento(consola, monkeypatch):
    monkeypatch.setattr(consola.backend, "leer", lambda: (np.radians(Q_A), time.monotonic() - 10))
    with pytest.raises(RuntimeError, match="desactualizado"):
        consola.guardar_pose_actual("Vieja")
    monkeypatch.setattr(consola.backend, "leer", lambda: (np.radians(Q_A), time.monotonic()))
    consola.ejec = {"plan_id": 1}
    with pytest.raises(RuntimeError, match="quieto"):
        consola.guardar_pose_actual("Moviendo")
    consola.ejec = None
    assert consola.listar_poses()["poses"] == []


@pytest.mark.parametrize("contenido", ["{ esto no es json", '{"poses": 3}', "[]", ""])
def test_archivo_corrupto_avisa_y_no_rompe(consola, contenido):
    ruta = _ruta(consola)
    ruta.parent.mkdir(parents=True, exist_ok=True)
    ruta.write_text(contenido, encoding="utf-8")
    lista = consola.listar_poses()
    assert lista["poses"] == [] and "se ignora" in lista["aviso"]
    # Guardar no pierde el archivo dañado: queda una copia aparte.
    consola.guardar_pose_actual("Nueva")
    assert [p["nombre"] for p in consola.listar_poses()["poses"]] == ["Nueva"]
    copias = [n for n in os.listdir(ruta.parent) if n.startswith("poses_guardadas.json.danado-")]
    assert len(copias) == 1
    assert (ruta.parent / copias[0]).read_text(encoding="utf-8") == contenido


def test_entradas_invalidas_del_archivo_se_descartan_con_aviso(consola):
    ruta = _ruta(consola)
    ruta.parent.mkdir(parents=True, exist_ok=True)
    buenas = {"nombre": "Buena", "q_deg": Q_A, "tcp": {"x_mm": 1.0}, "fecha": "2026-11-24T10:00:00"}
    ruta.write_text(json.dumps({"version": 1, "poses": [
        buenas,
        {"nombre": "buena", "q_deg": Q_A},                      # repetida
        {"nombre": "Sin juntas"},
        {"nombre": "NaN", "q_deg": [0, 0, 0, 0, 0, None]},
        {"nombre": "Fuera", "q_deg": [0, 0, 0, 0, 500, 0]},
        {"nombre": "a/b", "q_deg": Q_A},
        "texto suelto",
    ]}), encoding="utf-8")
    lista = consola.listar_poses()
    assert [p["nombre"] for p in lista["poses"]] == ["Buena"]
    assert "6 poses inválidas" in lista["aviso"]
    assert consola.poses.obtener("BUENA")["q_deg"] == Q_A


def test_sin_carpeta_de_datos(tmp_path):
    cfg = cargar_config(CFG)
    registro = Registro(tmp_path, "test")
    try:
        con = Consola(cfg, BackendSim(np.radians(Q_A), ruido_deg=0), registro)
        assert con.listar_poses()["disponible"] is False
        with pytest.raises(RuntimeError, match="--datos"):
            con.guardar_pose_actual("X")
    finally:
        registro.cerrar()


# ---- «Ir a pose»
def test_ir_a_pose_en_sim_pasa_por_revision_y_compuertas(consola, monkeypatch):
    consola.guardar_pose_actual("Destino")
    consola.backend.q = np.radians([0, -20, 30, 0, 40, 0])
    plan = consola.calcular_pose_guardada("destino")
    assert plan["tipo"] == "articular" and plan["descripcion"] == "Ir a pose «Destino»"
    assert plan["pose_guardada"]["nombre"] == "Destino"
    np.testing.assert_allclose(plan["q_obj_deg"], Q_A, atol=1e-6)
    assert plan["ejecutable"], plan["problemas"]
    assert {c["id"] for c in plan["revision"]["chequeos"]} >= {"limites", "singularidad"}
    ok, msg = consola.ejecutar(plan["id"])
    assert not ok and "ARMAR" in msg
    # la postura cambió desde el cálculo: no se recalcula solo, se exige recalcular
    assert consola.armar({}, "")[0]
    q = consola.plan["_tray"].q0.copy()
    q[0] += np.radians(consola.cfg["seguridad"]["inicio_max_deg"] + 0.5)
    monkeypatch.setattr(consola.backend, "leer", lambda: (q, time.monotonic()))
    ok, msg = consola.ejecutar(plan["id"])
    assert not ok and "Recalculá" in msg


def test_ir_a_pose_inexistente(consola):
    with pytest.raises(ValueError, match="No existe"):
        consola.calcular_pose_guardada("Nada")
    assert consola.plan is None


def test_ir_a_pose_fuera_de_limites_por_archivo_editado_no_es_ejecutable(consola):
    consola.guardar_pose_actual("Editada")
    ruta = _ruta(consola)
    datos = json.loads(ruta.read_text(encoding="utf-8"))
    datos["poses"][0]["q_deg"][1] = 500
    ruta.write_text(json.dumps(datos), encoding="utf-8")
    with pytest.raises(ValueError, match="No existe"):       # la entrada inválida se descarta al leer
        consola.calcular_pose_guardada("Editada")


def _con_poses_rws(tmp_path, verificada):
    con, registro = _consola_rws(tmp_path, [0, -20, 30, 0, 40, 0], verificada=verificada)
    con.poses = AlmacenPoses(str(tmp_path / "01_datos" / "poses_guardadas.json"), LIMITES_DEG)
    con.poses.guardar("Real", [5, -15, 25, 0, 35, 10], {})
    return con, registro


def test_ir_a_pose_rws_con_c02_exige_go_armado_y_frase(tmp_path):
    con, registro = _con_poses_rws(tmp_path, verificada=True)
    try:
        assert con.modo()["ir_a_pose_permitido"] is True
        plan = con.calcular_pose_guardada("Real")
        assert plan["ejecutable"], plan["problemas"]
        assert plan["vel_pct"] <= con.modo()["vel_max_pct"]
        assert con.plan["_permitir_singular_destino"] == "articular_real"
        ok, msg = con.ejecutar(plan["id"])
        assert not ok and "ARMAR" in msg
        assert not con.armar({}, "")[0]
        assert not con.armar(CHECK, "mover")[0]
        assert con.armar(CHECK, FRASE)[0]
        ok, msg = con.ejecutar(plan["id"])
        assert ok, msg
        (ts, Q, opciones), = con.backend.enviados
        assert {"origen_revisado", "cancelar", "comprobar_go"} <= set(opciones)
        np.testing.assert_allclose(np.degrees(Q[-1]), [5, -15, 25, 0, 35, 10], atol=1e-9)
    finally:
        registro.cerrar()


def test_ir_a_pose_rws_con_c02_rechaza_si_la_postura_cambio(tmp_path, monkeypatch):
    con, registro = _con_poses_rws(tmp_path, verificada=True)
    try:
        plan = con.calcular_pose_guardada("Real")
        q = con.plan["_tray"].q0.copy()
        q[2] += np.radians(con.cfg["seguridad"]["inicio_max_deg"] + 0.5)
        monkeypatch.setattr(con.backend, "leer", lambda: (q, time.monotonic()))
        assert con.armar(CHECK, FRASE)[0]
        ok, msg = con.ejecutar(plan["id"])
        assert not ok and "Recalculá" in msg
        assert con.backend.enviados == []
    finally:
        registro.cerrar()


def test_ir_a_pose_rws_sin_c02_rechazado_sin_leer_el_robot(tmp_path):
    con, registro = _con_poses_rws(tmp_path, verificada=False)
    try:
        con.backend.leer = Mock(side_effect=AssertionError("no debe leer"))
        modo = con.modo()
        assert modo["ir_a_pose_permitido"] is False and "C02" in modo["ir_a_pose_motivo"]
        with pytest.raises(RuntimeError, match="C02"):
            con.calcular_pose_guardada("Real")
        assert con.plan is None and con.backend.enviados == []
    finally:
        registro.cerrar()


def test_ir_a_pose_con_ros_sigue_la_regla_articular_del_real(consola):
    consola.guardar_pose_actual("Ros")
    q0 = np.radians([0, -20, 30, 0, 40, 0])
    consola.backend = SimpleNamespace(tipo="ros", leer=lambda: (q0, time.monotonic()),
                                      info=lambda: {"tipo": "ros", "real": True, "texto": "ROS falso"})
    assert consola.modo()["ir_a_pose_permitido"] is True
    plan = consola.calcular_pose_guardada("Ros")
    assert consola.plan["_permitir_singular_destino"] is False
    assert "punto_intermedio_deg" not in plan            # sin búsqueda de retorno con el real
    np.testing.assert_allclose(plan["q_obj_deg"], Q_A, atol=1e-6)


# ---- API
def test_ordenes_de_poses_en_el_servidor(consola):
    r = ORDENES["pose_guardar"](consola, {"nombre": "API"})
    assert r["ok"] and [p["nombre"] for p in r["poses"]] == ["API"] and r["pose"]["nombre"] == "API"
    r = ORDENES["pose_renombrar"](consola, {"nombre": "API", "nuevo": "API 2"})
    assert [p["nombre"] for p in r["poses"]] == ["API 2"]
    r = ORDENES["calcular_pose_guardada"](consola, {"nombre": "API 2"})
    assert r["ok"] and r["plan"]["pose_guardada"]["nombre"] == "API 2"
    with pytest.raises(ValueError, match="Confirmá"):
        ORDENES["pose_borrar"](consola, {"nombre": "API 2"})
    r = ORDENES["pose_borrar"](consola, {"nombre": "API 2", "confirmar": True})
    assert r["poses"] == []
    r = ORDENES["poses_listar"](consola, {})
    assert r["ok"] and r["disponible"] and r["ruta"].endswith("poses_guardadas.json")
