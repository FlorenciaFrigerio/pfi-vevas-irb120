"""Regresión del modo de lectura con interfaces ROS sustituidas en la prueba."""

import copy
import json
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vevas.backend_ros import BackendROS
from vevas.consola import Consola
from vevas.registro import Registro
from vevas.seguridad import CONFIG_POR_DEFECTO


def test_lectura_ros_recibe_estado_y_no_crea_comandos(monkeypatch, tmp_path):
    creadas = []

    class Nodo:
        def __init__(self, nombre):
            pass

        def create_subscription(self, tipo, topico, callback, qos):
            creadas.append(topico)

        def create_publisher(self, *args):
            pytest.fail("Solo lectura intentó crear un publicador")

        def create_client(self, *args):
            pytest.fail("Solo lectura intentó crear un cliente")

    class Executor:
        def add_node(self, node):
            pass

        def spin(self):
            pass

    for nombre, campos in {
        'rclpy': {'ok': lambda: True},
        'rclpy.executors': {'MultiThreadedExecutor': Executor},
        'rclpy.node': {'Node': Nodo},
        'rclpy.qos': {'qos_profile_sensor_data': object()},
        'sensor_msgs': {},
        'sensor_msgs.msg': {'JointState': object},
    }.items():
        modulo = ModuleType(nombre)
        modulo.__dict__.update(campos)
        monkeypatch.setitem(sys.modules, nombre, modulo)
    # El constructor de lectura debe funcionar sin las dependencias de comando.
    monkeypatch.setitem(sys.modules, 'rclpy.action', None)
    monkeypatch.setitem(sys.modules, 'control_msgs.action', None)
    cfg = copy.deepcopy(CONFIG_POR_DEFECTO)
    cfg['ros']['topico_estado'] = '/irb120/real/joint_states'
    backend = BackendROS(cfg, lambda texto: None, solo_lectura=True)
    assert creadas == ['/irb120/real/joint_states']
    backend.iniciar()
    nombres = cfg['robot']['nombres_juntas']
    backend._cb_estado(SimpleNamespace(name=nombres, position=[0.1] * 6))
    q, instante = backend.leer()
    assert np.allclose(q, 0.1)
    assert time.monotonic() - instante < 1
    backend._cb_estado(SimpleNamespace(name=nombres, position=[float('nan')] * 6))
    backend._cb_estado(SimpleNamespace(name=nombres, position=[]))
    assert np.array_equal(backend.leer()[0], q)
    assert backend.info()['solo_lectura']
    assert backend.ejecutar_trayectoria([], [])[0] is False
    backend.enviar_punto(q, 0.25)
    backend.mantener()
    assert backend._ac is None and backend._pub is None
    reg = Registro(str(tmp_path), 'lectura')
    consola = Consola(cfg, backend, reg, permitir_real=True)
    assert consola.armar({}, '') == (False, 'La consola se abrió en SOLO LECTURA.')
    consola.parar('prueba de PARAR')
    reg.cerrar()


CASOS = json.loads(
    (Path(__file__).resolve().parents[1] / 'docs/datos/referencia_marcelo/posturas_historicas.json').read_text()
)['casos']


@pytest.mark.parametrize('caso', CASOS, ids=[c['id'] for c in CASOS])
def test_fk_vevas_con_capturas_historicas(caso):
    from vevas.robot_irb120 import Robot, cuaternion_a_matriz, error_orientacion

    T = Robot().fk_tool0(np.radians(caso['q_deg']))
    posicion = np.linalg.norm(T[:3, 3] * 1000 - caso['xyz_mm'])
    orientacion = error_orientacion(T[:3, :3], cuaternion_a_matriz(*caso['cuaternion_wxyz']))
    assert posicion < caso['tol_p_mm']
    assert orientacion < caso['tol_R_rad']
