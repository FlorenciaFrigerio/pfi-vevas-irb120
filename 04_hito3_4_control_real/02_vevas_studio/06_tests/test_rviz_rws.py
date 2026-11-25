"""Regresiones de la visualización RWS sin ROS ni conexión al robot."""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from herramientas import nodo_rws_ros
from vevas.robot_irb120 import Robot, error_orientacion, error_posicion, euler_zyx_a_matriz, homogenea


def transformada_urdf(modelo, q):
    """Compone las transformaciones del URDF desde la base hasta tool0."""
    por_hijo = {j.find("child").attrib["link"]: j for j in modelo.findall("joint")}
    cadena = []
    hijo = "tool0"
    while hijo != "base_link":
        junta = por_hijo[hijo]
        cadena.append(junta)
        hijo = junta.find("parent").attrib["link"]

    T = np.eye(4)
    for junta in reversed(cadena):
        origen = junta.find("origin")
        xyz = [float(v) for v in origen.attrib["xyz"].split()]
        roll, pitch, yaw = map(float, origen.attrib["rpy"].split())
        T = T @ homogenea(euler_zyx_a_matriz(yaw, pitch, roll), xyz)
        if junta.attrib["type"] == "revolute":
            eje = np.fromstring(junta.find("axis").attrib["xyz"], sep=" ")
            eje /= np.linalg.norm(eje)
            x, y, z = eje
            cruz = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            angulo = q[junta.attrib["name"]]
            T[:3, :3] = T[:3, :3] @ (
                np.eye(3) + math.sin(angulo) * cruz + (1 - math.cos(angulo)) * (cruz @ cruz))
        else:
            assert junta.attrib["type"] == "fixed"
    return T


def posturas_urdf(robot):
    inferior, superior = np.asarray(robot.limites).T
    aleatorias = np.random.default_rng(120).uniform(inferior, superior, (99, 6))
    return np.vstack((np.zeros(6), aleatorias))


def test_tool0_urdf_coincide_con_fk_en_100_posturas():
    ruta = Path(__file__).resolve().parents[1] / "03_modelo" / "irb120_rws.urdf"
    modelo = ET.parse(ruta).getroot()
    robot = Robot()
    for q in posturas_urdf(robot):
        T = transformada_urdf(modelo, dict(zip(nodo_rws_ros.NOMBRES, q)))
        esperado = robot.fk_tool0(q)
        assert error_posicion(T, esperado) < 1e-8
        assert error_orientacion(T[:3, :3], esperado[:3, :3]) < 1e-8


@pytest.mark.parametrize("argumentos", [[], ["--solo-lectura"]])
def test_nodo_solo_lectura_publica_sin_dependencias_de_accion(monkeypatch, argumentos):
    pub = Mock()
    sello = object()

    class NodoSimulado:
        def __init__(self, nombre):
            assert nombre == "nodo_rws_ros"

        create_publisher = Mock(return_value=pub)
        create_timer = Mock()
        get_logger = Mock()
        get_clock = Mock()
        destroy_node = Mock()

    NodoSimulado.get_clock.return_value.now.return_value.to_msg.return_value = sello

    def joint_state():
        return SimpleNamespace(header=SimpleNamespace())

    def spin(nodo):
        assert isinstance(nodo, NodoSimulado)
        publicar = NodoSimulado.create_timer.call_args.args[1]
        publicar()
        pub.publish.assert_not_called()
        publicar()

    rclpy = SimpleNamespace(init=Mock(), spin=spin, ok=Mock(return_value=True), shutdown=Mock())
    modulos_ros = {
        "rclpy": rclpy,
        "rclpy.action": None,
        "rclpy.node": SimpleNamespace(Node=NodoSimulado),
        "sensor_msgs.msg": SimpleNamespace(JointState=joint_state),
        "control_msgs.action": None,
    }
    for nombre, modulo in modulos_ros.items():
        monkeypatch.setitem(sys.modules, nombre, modulo)

    q_deg = [0, -20, 30, 0, 40, 0]
    cliente = Mock()
    lector = Mock(error=None)
    lector.reciente.side_effect = [None, q_deg]
    monkeypatch.setattr(nodo_rws_ros, "Cliente", Mock(return_value=cliente))
    monkeypatch.setattr(nodo_rws_ros, "Lector", Mock(return_value=lector))
    ejecutor = Mock()
    monkeypatch.setattr(nodo_rws_ros, "Ejecutor", ejecutor)

    assert nodo_rws_ros.main(argumentos) == 0

    NodoSimulado.create_publisher.assert_called_once_with(joint_state, "/joint_states", 10)
    ejecutor.assert_not_called()
    pub.publish.assert_called_once()
    msg = pub.publish.call_args.args[0]
    assert msg.header.stamp == sello
    assert msg.name == [f"joint_{i}" for i in range(1, 7)]
    assert msg.position == [math.radians(q) for q in q_deg]


def test_nodo_rechaza_control_por_accion(capsys):
    with pytest.raises(SystemExit) as error:
        nodo_rws_ros.main(["--accion", "/joint_trajectory_controller/follow_joint_trajectory"])
    assert error.value.code == 2
    assert "El control por acción está deshabilitado" in capsys.readouterr().err
