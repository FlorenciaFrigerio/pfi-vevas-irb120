"""La mesa declarada usa la misma geometría para revisión y publicación web."""
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vevas.mesa import generar_mesa
from vevas.seguridad import cargar_config

RAIZ = Path(__file__).resolve().parents[1]


def test_config_carga_las_cuatro_barras_sin_confirmar_el_resto_del_entorno():
    cfg = cargar_config(str(RAIZ / "config.json"))
    ent = cfg["entorno"]
    mesa = generar_mesa()
    assert ent["mesa"] == mesa
    assert len(mesa["cajas"]) == 4
    assert ent["cajas"] == [{**c, "origen": "mesa"} for c in mesa["cajas"]]
    assert mesa["arista_superior_z_mm"] == 190
    assert ent["sin_objetos_externos_confirmado"] is False
    assert ent["sin_herramienta_confirmada"] is False


def test_perfil_se_resuelve_desde_config_y_conserva_obstaculos_externos(tmp_path):
    perfil = tmp_path / "03_modelo" / "mesa.json"
    perfil.parent.mkdir()
    perfil.write_bytes((RAIZ / "03_modelo" / "mesa_medidas.json").read_bytes())
    caja = {"nombre": "pared", "min_mm": [800, -400, 0], "max_mm": [900, 400, 900]}
    config = tmp_path / "config.json"
    config.write_text(json.dumps({"entorno": {"perfil_mesa": "03_modelo/mesa.json", "cajas": [caja]}}))
    cfg = cargar_config(str(config))
    assert cfg["entorno"]["cajas"][0] == caja
    assert len(cfg["entorno"]["cajas"]) == 5
    assert len(cfg["entorno"]["mesa"]["cajas"]) == 4
