"""Casos del lanzador con ejecutables locales falsos y sin acceso al IRC5."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from test_revision import go_valido

RAIZ = Path(__file__).resolve().parents[1]


@pytest.fixture
def lanzador(tmp_path):
    # El lanzador usa su propia carpeta para encontrar el código y guardar logs.
    proyecto = tmp_path / "vevas"
    proyecto.mkdir()
    shutil.copy2(RAIZ / "iniciar_real.sh", proyecto)
    (proyecto / "vevas").symlink_to(RAIZ / "vevas", target_is_directory=True)
    ejecutables = tmp_path / "bin"
    ejecutables.mkdir()
    python = ejecutables / "python3"
    python.write_text(
        r"""#!/usr/bin/env bash
printf 'python3 %s\n' "$*" >> "$PRUEBA_LOG"
case "$1" in
  */rws_irc5.py)
    if [[ "$*" != *' estado --shell' ]]; then
      echo "Escritura inesperada" >&2
      exit 99
    fi
    printf '%s\n' OPMODE=MANR CTRLSTATE=guardstop EXEC=running MODULO_VEVAS=no \
      PP_MODULO=OtroModulo PP_RUTINA=main Q_DEG=0,0,0,0,20,0
    ;;
  */nodo_rws_ros.py) exec sleep 60 ;;
  */consola_vevas.py) exit 0 ;;
  -c)
    if [[ "${PRUEBA_VELOCIDAD:-no}" == si ]]; then
      exit 0
    fi
    exec "$PRUEBA_PYTHON" "$@"
    ;;
  *) exec "$PRUEBA_PYTHON" "$@" ;;
esac
""",
        encoding="utf-8",
    )
    ping = ejecutables / "ping"
    ping.write_text(
        '#!/usr/bin/env bash\nprintf "ping %s\\n" "$*" >> "$PRUEBA_LOG"\n',
        encoding="utf-8",
    )
    ros = ejecutables / "ros2"
    ros.write_text(
        r"""#!/usr/bin/env bash
printf 'ros2 %s\n' "$*" >> "$PRUEBA_LOG"
case "$*" in
  'pkg prefix robot_state_publisher'|'pkg prefix rviz2') exit 0 ;;
  'topic echo --once /joint_states sensor_msgs/msg/JointState') exit 0 ;;
  launch*"rviz_rws.launch.py") exit 0 ;;
  *) exit 99 ;;
esac
""",
        encoding="utf-8",
    )
    for programa in (python, ping, ros):
        programa.chmod(0o755)
    registro = tmp_path / "comandos.log"
    go = proyecto / "GO_VEVAS.json"

    def ejecutar(*args, velocidad_verificada=False):
        registro.touch()
        env = {
            **os.environ,
            "PATH": str(ejecutables) + os.pathsep + os.environ["PATH"],
            "PRUEBA_LOG": str(registro),
            "PRUEBA_PYTHON": sys.executable,
            "PRUEBA_VELOCIDAD": "si" if velocidad_verificada else "no",
            "ROS_SETUP": str(tmp_path / "sin_setup.bash"),
        }
        resultado = subprocess.run(
            ["bash", str(proyecto / "iniciar_real.sh"), "--ip", "127.0.0.1", *args],
            env=env,
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            timeout=20,
        )
        return resultado, registro.read_text(encoding="utf-8")

    return ejecutar, go


def test_chequeo_lee_en_manual_con_cadena_abierta(lanzador):
    ejecutar, _ = lanzador
    resultado, comandos = ejecutar("--solo-chequeo")
    assert resultado.returncode == 0, resultado.stderr
    assert "Modo=MANR" in resultado.stdout and "estado=guardstop" in resultado.stdout
    assert comandos.count(" estado --shell") == 1
    assert "ros2" not in comandos and "consola_vevas.py" not in comandos


def test_lectura_web_no_exige_go_ni_arranca_rapid(lanzador):
    ejecutar, _ = lanzador
    resultado, comandos = ejecutar("--solo-lectura", "--sin-rviz")
    assert resultado.returncode == 0, resultado.stderr
    assert comandos.count(" estado --shell") == 1
    consola = next(linea for linea in comandos.splitlines() if "consola_vevas.py" in linea)
    assert "--solo-lectura" in consola
    assert "--real" not in consola and "--go" not in consola
    assert "instalar" not in comandos and "arrancar" not in comandos


def test_rviz_espera_joint_states_y_usa_nodo_de_lectura(lanzador):
    ejecutar, _ = lanzador
    resultado, comandos = ejecutar("--solo-lectura", "--sin-vevas")
    assert resultado.returncode == 0, resultado.stderr
    assert "ros2 pkg prefix robot_state_publisher" in comandos
    assert "ros2 pkg prefix rviz2" in comandos
    nodo = next(linea for linea in comandos.splitlines() if "nodo_rws_ros.py" in linea)
    assert "--solo-lectura" in nodo
    assert "topic echo --once /joint_states" in comandos
    assert "rviz_rws.launch.py" in comandos
    assert "abb_bringup" not in comandos and "action list" not in comandos


@pytest.mark.parametrize("decision", [None, "NO-GO"])
def test_y_no_crea_go_ni_sobrescribe_un_no_go(lanzador, decision):
    ejecutar, go = lanzador
    contenido = None
    if decision:
        contenido = json.dumps(dict(go_valido(), decision=decision))
        go.write_text(contenido, encoding="utf-8")
    resultado, comandos = ejecutar("-y")
    assert resultado.returncode != 0
    if contenido is None:
        assert not go.exists()
    else:
        assert go.read_text(encoding="utf-8") == contenido
    assert "rws_irc5.py" not in comandos and "ping " not in comandos
    assert "crear_go.py" not in comandos


def test_go_valido_no_habilita_control_con_velocidad_no_verificada(lanzador):
    ejecutar, go = lanzador
    go.write_text(json.dumps(go_valido()), encoding="utf-8")
    resultado, comandos = ejecutar("-y")
    assert resultado.returncode != 0
    assert "velocidad articular real" in resultado.stderr
    assert "--solo-lectura" in resultado.stderr
    assert "rws_irc5.py" not in comandos and "ping " not in comandos


def test_eof_cancela_antes_de_contactar_al_controlador(lanzador):
    ejecutar, go = lanzador
    go.write_text(json.dumps(go_valido()), encoding="utf-8")
    resultado, comandos = ejecutar(velocidad_verificada=True)
    assert resultado.returncode != 0
    assert "Entrada cerrada" in resultado.stderr
    assert "rws_irc5.py" not in comandos and "ping " not in comandos
