"""Pruebas de la comparación FlexPendant con datos sintéticos y sin red."""

import json
import math
from unittest.mock import Mock, call

import numpy as np
import pytest

from herramientas import comparar_pendant
from vevas.robot_irb120 import Robot, matriz_a_cuaternion, rot_x, rot_y, rot_z
from vevas.rws_cliente import Falla
from vevas.verificacion_pendant import ComparacionInvalida, comparar_con_controlador


RAIZ_MEDIA = math.sqrt(0.5)
POSE_CERO = {
    "x": "374", "y": "0", "z": "630", "q1": str(RAIZ_MEDIA), "q2": "0",
    "q3": str(RAIZ_MEDIA), "q4": "0",
}


def cliente_irc5(datos=None, *, antes=None, despues=None, cuerpo=None):
    if antes is None:
        antes = [0.0] * 6
    cliente = Mock(spec=["get", "joints_deg", "cerrar"])
    cliente.joints_deg.side_effect = [antes, antes if despues is None else despues]
    cliente.get.return_value = cuerpo if cuerpo is not None else json.dumps(
        {"_embedded": {"_state": [{**POSE_CERO, **(datos or {})}]}}, ensure_ascii=False)
    return cliente


def test_pose_tool0_usa_grados_mm_y_cuaternion_abb_sin_inferir_calibracion():
    cliente = cliente_irc5({"q1": -RAIZ_MEDIA, "q3": -RAIZ_MEDIA})
    # Un TCP cargado en la consola no cambia la FK de la brida.
    datos = comparar_con_controlador(cliente, Robot(tcp_xyz=(0, 0, 0.12)))

    assert cliente.mock_calls == [
        call.joints_deg(),
        call.get("/rw/motionsystem/mechunits/ROB_1/robtarget",
                 tool="tool0", wobj="wobj0", coordinate="Base"),
        call.joints_deg(),
    ]
    assert datos["ok"] and datos["coincide"]
    assert datos["juntas_deg"] == [0.0] * 6
    assert datos["frame"] == {
        "unidad_mecanica": "ROB_1", "herramienta": "tool0", "objeto": "wobj0", "coordenadas": "Base",
    }
    assert datos["modelo"]["xyz_mm"] == pytest.approx([374, 0, 630], abs=1e-9)
    assert datos["error_posicion_mm"] == pytest.approx(0.0, abs=1e-9)
    assert datos["error_orientacion_rad"] == pytest.approx(0.0, abs=1e-12)
    assert datos["controlador"]["gimbal_zyx"]
    assert not datos["controlador"]["gimbal_zxy"]
    assert datos["calibracion"] == {"estado": "no verificado", "fuente": "FlexPendant"}
    json.dumps(datos, allow_nan=False)


def test_configuracion_convierte_juntas_crudas_a_la_convencion_del_programa():
    q_deg = [35, -15, 25, 10, 40, 20]
    signos = [-1, 1, 1, 1, 1, 1]
    offsets = [5, 0, 0, 0, 0, 0]
    robot = Robot()
    q_modelo = [-30, -15, 25, 10, 40, 20]
    T = robot.fk_tool0(np.radians(q_modelo))
    pose = dict(zip(POSE_CERO, [*(T[:3, 3] * 1000), *matriz_a_cuaternion(T[:3, :3])]))
    datos = comparar_con_controlador(cliente_irc5(pose, antes=q_deg), robot,
                                     signos=signos, offsets_deg=offsets)
    assert datos["coincide"]
    assert datos["juntas_deg"] == q_deg
    assert datos["juntas_modelo_deg"] == q_modelo


def test_comparacion_valida_detecta_diferencia_mm_y_rad():
    giro = math.radians(10)
    q = matriz_a_cuaternion(rot_y(math.pi / 2) @ rot_z(giro))
    pose = {"x": 375, **dict(zip(("q1", "q2", "q3", "q4"), q))}
    datos = comparar_con_controlador(cliente_irc5(pose), Robot())
    assert datos["ok"] and not datos["coincide"]
    assert datos["error_posicion_mm"] == pytest.approx(1.0)
    assert datos["error_orientacion_rad"] == pytest.approx(giro)


def test_euler_zxy_en_bloqueo_representa_la_misma_rotacion():
    R = rot_z(0.4) @ rot_x(math.pi / 2) @ rot_y(-0.9)
    pose = dict(zip(("q1", "q2", "q3", "q4"), matriz_a_cuaternion(R)))
    datos = comparar_con_controlador(cliente_irc5(pose), Robot())["controlador"]
    z, x, y = np.radians(datos["euler_zxy_deg"])
    assert datos["gimbal_zxy"]
    assert np.allclose(rot_z(z) @ rot_x(x) @ rot_y(y), R)


def test_rechaza_captura_si_las_juntas_cambian():
    cliente = cliente_irc5(despues=[0.0, 0.0, 0.04, 0.0, 0.0, 0.0])
    with pytest.raises(ComparacionInvalida, match="inestable"):
        comparar_con_controlador(cliente, Robot())


@pytest.mark.parametrize(("campo", "valor", "mensaje"), [
    ("coordinate", "World", "coordinate=Base"),
    ("tool", "tool1", "tool=tool0"),
    ("wobj", "wobj2", "wobj=wobj0"),
    ("position-unit", "m", "milímetros"),
    ("x", "NaN", "no finitos"),
    ("y", True, "no numéricos"),
    ("q2", "abc", "no numéricos"),
    ("q3", 0, "norma unitaria"),
])
def test_rechaza_pose_con_marco_unidad_o_formato_invalido(campo, valor, mensaje):
    with pytest.raises(ComparacionInvalida, match=mensaje):
        comparar_con_controlador(cliente_irc5({campo: valor}), Robot())


def test_no_compone_un_target_a_partir_de_registros_parciales():
    registros = [{"x": 374, "y": 0, "z": 630}, {"q1": RAIZ_MEDIA, "q2": 0, "q3": RAIZ_MEDIA, "q4": 0}]
    cliente = cliente_irc5(cuerpo=json.dumps({"_embedded": {"_state": registros}}))
    with pytest.raises(ComparacionInvalida, match="pose completa"):
        comparar_con_controlador(cliente, Robot())


def test_acepta_una_pose_xhtml_de_rws():
    campos = "".join(f'<span class="{k}">{v}</span>' for k, v in POSE_CERO.items())
    cuerpo = f'<html xmlns="http://www.w3.org/1999/xhtml"><body><li class="ms-robtargets">{campos}</li></body></html>'
    assert comparar_con_controlador(cliente_irc5(cuerpo=cuerpo), Robot())["coincide"]


def test_rechaza_lectura_http_fallida():
    cliente = cliente_irc5()
    cliente.get.side_effect = Falla("HTTP 404")
    with pytest.raises(ComparacionInvalida, match="HTTP 404"):
        comparar_con_controlador(cliente, Robot())


def test_cli_solo_lectura_emite_informe_json(monkeypatch, capsys):
    cliente = cliente_irc5()
    monkeypatch.setattr(comparar_pendant, "Cliente", lambda *args: cliente)
    assert comparar_pendant.main(["--json"]) == 0
    informe = json.loads(capsys.readouterr().out)
    assert informe["coincide"]
    assert informe["frame"]["herramienta"] == "tool0"
    cliente.cerrar.assert_called_once()
