"""Generación interactiva del registro de sesión con archivos temporales."""
from io import StringIO
import json
from pathlib import Path
import shutil
import sys

import pytest

RAIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ))

from herramientas import crear_go  # noqa: E402
from vevas.consola import CHECKLIST_GO, validar_go  # noqa: E402


@pytest.fixture
def archivos(tmp_path):
    config = tmp_path / "config.json"
    shutil.copy2(RAIZ / "config.json", config)
    perfil_mesa = tmp_path / "modelo" / "mesa_medidas.json"
    perfil_mesa.parent.mkdir()
    shutil.copy2(RAIZ / "modelo" / "mesa_medidas.json", perfil_mesa)
    go = tmp_path / "GO_VEVAS.json"
    return config, go, ["--config", str(config), "--salida", str(go)]


def respuestas(monkeypatch, *, rechazo=None):
    datos = ["Docente, autorización 1", "Operadora", "Responsable consola", "Supervisor", "Ensayo en simulación"]
    datos.append("1")
    datos.extend("n" if clave == rechazo else "s" for clave in CHECKLIST_GO)
    datos.extend(["s", "s"])  # sin objetos externos y brida libre
    monkeypatch.setattr(sys, "stdin", StringIO("\n".join(datos) + "\n"))


def test_automatico_rechaza_sin_crear_archivo(archivos, capsys):
    config, go, args = archivos
    original = config.read_bytes()
    with pytest.raises(SystemExit) as error:
        crear_go.main([*args, "--automatico"])
    assert error.value.code == 2
    assert "respuestas explícitas" in capsys.readouterr().err
    assert not go.exists()
    assert config.read_bytes() == original


def test_archivo_existente_se_conserva_incluso_no_go(archivos, capsys):
    config, go, args = archivos
    original = config.read_bytes()
    go.write_text('{"decision":"NO-GO"}\n', encoding="utf-8")
    existente = go.read_bytes()
    with pytest.raises(SystemExit) as error:
        crear_go.main(args)
    assert error.value.code == 2
    assert "ya existe" in capsys.readouterr().err
    assert go.read_bytes() == existente
    assert config.read_bytes() == original


def test_eof_parcial_no_guarda_go(archivos, monkeypatch):
    config, go, args = archivos
    original = config.read_bytes()
    monkeypatch.setattr(sys, "stdin", StringIO("Docente\nOperadora\n"))
    assert crear_go.main(args) == 130
    assert not go.exists()
    assert config.read_bytes() == original


def test_checklist_negativo_registra_no_go(archivos, monkeypatch):
    config, go, args = archivos
    original = config.read_bytes()
    rechazo = "5_zona_despejada"
    respuestas(monkeypatch, rechazo=rechazo)
    assert crear_go.main(args) == 1
    registro = json.loads(go.read_text(encoding="utf-8"))
    assert registro["decision"] == "NO-GO"
    assert registro["checklist"] == {clave: clave != rechazo for clave in CHECKLIST_GO}
    assert validar_go(registro)[0] is False
    assert config.read_bytes() == original


def test_confirmaciones_explicitas_generan_go_y_entorno_de_sesion(archivos, monkeypatch):
    config, go, args = archivos
    original = config.read_bytes()
    respuestas(monkeypatch)
    assert crear_go.main(args) == 0
    registro = json.loads(go.read_text(encoding="utf-8"))
    assert registro["decision"] == "GO"
    assert registro["checklist"] == dict.fromkeys(CHECKLIST_GO, True)
    assert registro["velocidad_max_pct"] == 1
    assert registro["entorno"]["sin_objetos_externos_confirmado"] is True
    assert registro["entorno"]["sin_herramienta_confirmada"] is True
    assert registro["entorno"]["herramienta"]["confirmada"] is False
    assert validar_go(registro)[0] is True
    assert config.read_bytes() == original
