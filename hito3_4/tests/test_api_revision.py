"""Pruebas HTTP con consola y cliente RWS falsos."""
from http.client import HTTPConnection
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vevas import verificacion_pendant  # noqa: E402
from vevas.servidor import crear_servidor  # noqa: E402


class ConsolaFalsa:
    def __init__(self):
        self.lock = threading.Lock()
        self.cfg = {
            "servidor": {"hz_estado": 20},
            "rws": {"clave": "secreto-de-prueba", "hz_lectura": 8},
            "robot": {"signos": [1] * 6, "offsets_deg": [0] * 6},
        }
        self.backend = SimpleNamespace(tipo="rws", cli=Mock(spec_set=["estado"]))
        self.robot = object()
        self.reg = Mock(spec_set=["evento"])
        self.ejec = self.jog = None
        self.envio_pendiente = False
        self.id_actual = 42
        self.envios = []
        self.paradas = []

    def pulso(self):
        pass

    def ejecutar(self, *, plan_id):
        with self.lock:
            self.envios.append(plan_id)
            if plan_id != self.id_actual:
                return False, "El plan cambió"
            return True, "Plan recibido"

    def parar(self, motivo):
        self.paradas.append(motivo)


@pytest.fixture
def api(tmp_path):
    consola = ConsolaFalsa()
    servidor = crear_servidor(consola, str(tmp_path), "127.0.0.1", 0, 10)
    hilo = threading.Thread(target=servidor.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
    hilo.start()

    def pedir(ruta, cuerpo=None):
        conexion = HTTPConnection(*servidor.server_address, timeout=2)
        try:
            if cuerpo is None:
                conexion.request("GET", f"/api/{ruta}")
            else:
                conexion.request("POST", f"/api/{ruta}", json.dumps(cuerpo), {"Content-Type": "application/json"})
            respuesta = conexion.getresponse()
            contenido = json.load(respuesta)
            assert respuesta.status == 200, contenido
            return contenido
        finally:
            conexion.close()

    try:
        yield consola, pedir
    finally:
        servidor.shutdown()
        servidor.server_close()
        hilo.join(timeout=2)


def test_config_no_publica_ni_elimina_la_clave(api):
    consola, pedir = api
    config = pedir("config")
    assert config["cfg"]["rws"] == {"hz_lectura": 8}
    assert consola.cfg["rws"]["clave"] == "secreto-de-prueba"


@pytest.mark.parametrize("cuerpo", [{}, {"plan_id": True}, {"plan_id": 0}, {"plan_id": -1},
                                    {"plan_id": 42.0}, {"plan_id": "42"}, {"plan_id": None}])
def test_ejecutar_rechaza_id_ausente_o_invalido(api, cuerpo):
    consola, pedir = api
    assert pedir("ejecutar", cuerpo)["ok"] is False
    assert consola.envios == []


def test_ejecutar_delega_id_y_decision_en_una_llamada(api):
    consola, pedir = api
    assert pedir("ejecutar", {"plan_id": 41}) == {"ok": False, "mensaje": "El plan cambió"}
    assert pedir("ejecutar", {"plan_id": 42}) == {"ok": True, "mensaje": "Plan recibido"}
    assert consola.envios == [41, 42]


def test_frecuencia_modifica_solo_la_vista(api):
    consola, pedir = api
    for hz in (5, 10, 20):
        assert pedir("frecuencia", {"hz": hz}) == {"ok": True, "hz_vista": hz, "hz_fuente": 8}
        assert consola.cfg["servidor"]["hz_estado"] == hz
        assert consola.cfg["rws"]["hz_lectura"] == 8
    assert consola.backend.cli.mock_calls == []


@pytest.mark.parametrize("valor", [True, False, 0, 8, 15.5, "20", None])
def test_frecuencia_invalida_no_altera_vista_ni_fuente(api, valor):
    consola, pedir = api
    assert pedir("frecuencia", {"hz": valor})["ok"] is False
    assert consola.cfg["servidor"]["hz_estado"] == 10
    assert consola.cfg["rws"]["hz_lectura"] == 8
    assert consola.backend.cli.mock_calls == []


def test_verificacion_pendant_exige_rws(api, monkeypatch):
    consola, pedir = api
    consola.backend.tipo = "sim"
    comparar = Mock()
    monkeypatch.setattr(verificacion_pendant, "comparar_con_controlador", comparar)
    respuesta = pedir("verificacion_pendant")
    assert respuesta["ok"] is False and "RWS" in respuesta["mensaje"]
    comparar.assert_not_called()
    consola.reg.evento.assert_not_called()


@pytest.mark.parametrize("ocupado", ["ejec", "jog", "envio_pendiente"])
def test_verificacion_pendant_rechazada_si_hay_movimiento(api, monkeypatch, ocupado):
    consola, pedir = api
    setattr(consola, ocupado, True)
    comparar = Mock()
    monkeypatch.setattr(verificacion_pendant, "comparar_con_controlador", comparar)
    respuesta = pedir("verificacion_pendant")
    assert respuesta["ok"] is False and "movimiento" in respuesta["mensaje"]
    comparar.assert_not_called()
    consola.reg.evento.assert_not_called()


def test_verificacion_pendant_rws_libre_devuelve_y_registra_comparacion(api, monkeypatch):
    consola, pedir = api
    resultado = {"ok": True, "comparacion": {"error_juntas_deg": [0.1] * 6}}
    comparar = Mock(return_value=resultado)
    monkeypatch.setattr(verificacion_pendant, "comparar_con_controlador", comparar)

    assert pedir("verificacion_pendant") == resultado
    comparar.assert_called_once_with(consola.backend.cli, consola.robot, signos=[1] * 6, offsets_deg=[0] * 6)
    consola.reg.evento.assert_called_once_with("verificacion_pendant", datos=resultado)
    assert consola.backend.cli.mock_calls == []


def test_parar_rws_informa_solicitud_sin_afirmar_quietud(api):
    consola, pedir = api
    respuesta = pedir("parar", {"motivo": "prueba de cancelación"})
    assert consola.paradas == ["prueba de cancelación"]
    assert respuesta["ok"] is True
    assert "Cancelación solicitada" in respuesta["mensaje"]
    assert "tramo físico puede seguir" in respuesta["mensaje"]
