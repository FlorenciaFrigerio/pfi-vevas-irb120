"""Pruebas del diagnóstico de laboratorio sin ejecutar comandos de red ni contactar al IRC5."""

import json
import subprocess
from unittest.mock import Mock, call

import pytest

from herramientas import conectar_robot as conectar
from vevas.robot_irb120 import Robot, matriz_a_cuaternion
from vevas.rws_cliente import Cliente, Falla
from vevas.verificacion_pendant import RUTA_POSE


@pytest.fixture
def laboratorio(monkeypatch):
    interfaz = "enx010203040506"
    ruta = {"dst": "192.168.125.1", "dev": interfaz, "prefsrc": "192.168.125.43"}
    direcciones = [{"ifname": interfaz, "addr_info": [{"family": "inet",
                    "local": ruta["prefsrc"], "prefixlen": 24}]}]
    ping = Mock(return_value=subprocess.CompletedProcess([], 1))

    def ejecutar(comando, **opciones):
        assert opciones == {"check": False, "capture_output": True, "text": True,
                             "timeout": 4 if comando[0] == "ping" else 3}
        match comando:
            case ["ip", "-j", "route", "get", _]:
                return subprocess.CompletedProcess(comando, 0, json.dumps([ruta]))
            case ["ip", "-j", "address"]:
                return subprocess.CompletedProcess(comando, 0, json.dumps(direcciones))
            case ["ping", "-n", "-c", "1", "-W", "2", _]:
                return ping(comando)
            case _:
                pytest.fail(f"Comando inesperado: {comando}")

    ejecutar = Mock(side_effect=ejecutar)
    monkeypatch.setattr(conectar.subprocess, "run", ejecutar)

    robot = Robot()
    T = robot.fk_tool0([0.0] * 6)
    campos = zip(("x", "y", "z", "q1", "q2", "q3", "q4"), (*T[:3, 3] * 1000, *matriz_a_cuaternion(T[:3, :3])))
    pose = dict(campos)
    cliente = Mock(spec=Cliente)
    cliente.leer_estado.return_value = {"opmode": "MANR", "ctrlstate": "guardstop", "exec": "stopped",
                                         "modulos": ["dato_no_necesario"]}
    cliente.joints_deg.return_value = [0.0] * 6
    cliente.get.side_effect = lambda *_args, **_params: json.dumps({"_embedded": {"_state": [pose]}})
    crear = Mock(return_value=cliente)
    monkeypatch.setattr(conectar, "Cliente", crear)
    return ruta, direcciones, ping, ejecutar, cliente, crear, pose


def test_manual_y_guardstop_solo_consultan_aunque_no_haya_ping(laboratorio, monkeypatch, tmp_path, capsys):
    _, _, _, ejecutar, cliente, crear, _ = laboratorio
    monkeypatch.setenv("VEVAS_RWS_CLAVE", "clave_que_no_sale")
    salida = tmp_path / "diagnostico.json"

    assert conectar.main(["--ping", "--salida", str(salida)]) == 0

    crear.assert_called_once_with("192.168.125.1", "Default User", "clave_que_no_sale", 80, 3.0)
    assert ejecutar.call_count == 3
    assert [args.args[0] for args in ejecutar.call_args_list] == [
        ["ip", "-j", "route", "get", "192.168.125.1"],
        ["ip", "-j", "address"],
        ["ping", "-n", "-c", "1", "-W", "2", "192.168.125.1"],
    ]
    assert cliente.method_calls == [
        call.leer_estado(con_juntas=False), call.joints_deg(), call.joints_deg(),
        call.get(RUTA_POSE, tool="tool0", wobj="wobj0", coordinate="Base"), call.joints_deg(), call.cerrar(),
    ]
    assert cliente.max_edad_juntas_s == 1.0
    texto = salida.read_text(encoding="utf-8")
    datos = json.loads(texto)
    assert datos["solo_lectura"]
    assert datos["red"]["origen_cidr"] == "192.168.125.43/24"
    assert datos["red"]["ping"]["resultado"] == "sin respuesta"
    assert datos["rws"]["panel"] == {"opmode": "MANR", "ctrlstate": "guardstop", "exec": "stopped"}
    assert datos["rws"]["comparacion_fk"]["coincide"]
    assert "guardstop" in capsys.readouterr().out
    assert all(secreto not in texto for secreto in ("clave_que_no_sale", "Default User", "dato_no_necesario"))


def test_ip_alternativa_y_ping_con_timeout_seguido_de_fk_distinta(laboratorio, tmp_path):
    ruta, direcciones, ping, _, _, crear, pose = laboratorio
    ruta.update(dst="10.1.0.9", prefsrc="10.1.0.4")
    direcciones[0]["addr_info"][0]["local"] = "10.1.0.4"
    ping.side_effect = subprocess.TimeoutExpired("ping", 4)
    pose["x"] += 1.5
    salida = tmp_path / "diagnostico.json"

    assert conectar.main(["--ip", "10.1.0.9", "--ping", "--salida", str(salida)]) == 1

    datos = json.loads(salida.read_text(encoding="utf-8"))
    assert crear.call_args.args[0] == "10.1.0.9"
    assert datos["red"]["ping"]["resultado"] == "tiempo de espera agotado"
    assert datos["rws"]["comparacion_fk"]["error_posicion_mm"] == pytest.approx(1.5)


def test_ruta_via_gateway_rechaza_incluso_ping(laboratorio, capsys):
    ruta, _, _, ejecutar, _, crear, _ = laboratorio
    ruta["gateway"] = "192.168.125.254"

    assert conectar.main(["--ping"]) == 2

    assert ejecutar.call_count == 2
    crear.assert_not_called()
    assert "ruta al IRC5" in capsys.readouterr().err


def test_loopback_explicito_para_controlador_virtual(laboratorio):
    ruta, direcciones, _, _, _, crear, _ = laboratorio
    ruta.update(dst="127.0.0.1", dev="lo", prefsrc="127.0.0.1", type="local")
    direcciones[0] = {"ifname": "lo", "addr_info": [{"family": "inet", "local": "127.0.0.1", "prefixlen": 8}]}

    assert conectar.main(["--ip", "127.0.0.1"]) == 0

    assert crear.call_args.args[0] == "127.0.0.1"


@pytest.mark.parametrize("ip", ["https://192.168.125.1", "8.8.8.8", "169.254.0.1", "192.168.125.1:80"])
def test_ip_invalida_antes_de_usar_red(laboratorio, ip):
    _, _, _, ejecutar, _, crear, _ = laboratorio

    assert conectar.main(["--ip", ip]) == 2

    ejecutar.assert_not_called()
    crear.assert_not_called()


def test_error_http_no_registra_cuerpo_y_conserva_juntas(laboratorio, tmp_path, capsys):
    _, _, _, _, cliente, _, _ = laboratorio
    cliente.get.side_effect = Falla("GET /rw/pose → HTTP 403: secreto_reflejado")
    salida = tmp_path / "diagnostico.json"

    assert conectar.main(["--salida", str(salida)]) == 2

    texto = salida.read_text(encoding="utf-8")
    datos = json.loads(texto)
    terminal = capsys.readouterr()
    assert datos["rws"]["juntas_deg"] == [0.0] * 6
    assert datos["rws"]["error_comparacion_fk"]["http"] == 403
    assert "secreto_reflejado" not in texto + terminal.out + terminal.err
    cliente.cerrar.assert_called_once_with()


def test_config_rara_omite_valores_sensibles_y_no_sondea(laboratorio, tmp_path, capsys):
    _, _, _, ejecutar, _, crear, _ = laboratorio
    config = tmp_path / "config.json"
    config.write_text('{"rws":{"puerto":"clave_en_campo_incorrecto"}}', encoding="utf-8")

    assert conectar.main(["--config", str(config)]) == 2

    ejecutar.assert_not_called()
    crear.assert_not_called()
    assert "clave_en_campo_incorrecto" not in capsys.readouterr().err
