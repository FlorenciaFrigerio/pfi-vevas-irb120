"""Geometría del perfil medido del marco y límites de su representación."""
import pytest

from vevas.mesa import cargar_perfil_mesa, generar_mesa


def contiene(caja, punto):
    return all(lo <= valor <= hi for lo, valor, hi in zip(caja["min_mm"], punto, caja["max_mm"]))


def test_mesa_medida_respeta_cotas_y_referencia_del_robot():
    mesa = generar_mesa()

    assert mesa["confirmado"] is True
    assert mesa["arista_superior_z_mm"] == 190
    assert mesa["cilindro_base_z_mm"] == 180
    assert mesa["referencia"] == {
        "frente_eje_base": "+X",
        "centro_huella_xy_mm": [0, 0],
        "plano_montaje_z_mm": 0,
    }
    assert mesa["huella_brazo_mm"] == {"profundidad": 215, "ancho": 210}
    assert mesa["cajas"] == [
        {"nombre": "Mesa · aro delantero", "min_mm": [502.5, -400, 140], "max_mm": [552.5, 400, 190]},
        {"nombre": "Mesa · aro trasero", "min_mm": [-247.5, -400, 140], "max_mm": [-197.5, 400, 190]},
        {"nombre": "Mesa · aro izquierdo", "min_mm": [-197.5, 350, 140], "max_mm": [502.5, 400, 190]},
        {"nombre": "Mesa · aro derecho", "min_mm": [-197.5, -400, 140], "max_mm": [502.5, -350, 190]},
    ]


def test_marco_abierto_y_apoyos_inferiores_solo_visuales():
    mesa = generar_mesa()

    for punto in ([0, 0, 165], [0, 0, 190], [0, 0, 0], [100, 0, -20]):
        assert not any(contiene(caja, punto) for caja in mesa["cajas"])

    apoyos = mesa["geometria_visual"]
    assert len(apoyos) == 2
    assert all(apoyo["confirmado"] is False for apoyo in apoyos)
    assert any(contiene(apoyo, [100, 0, -20]) for apoyo in apoyos)
    assert not any(contiene(apoyo, [0, 0, -20]) for apoyo in apoyos)


def test_perfil_parametrizable_con_giro_y_otras_dimensiones():
    perfil = cargar_perfil_mesa()
    perfil["confirmado"] = False
    perfil["referencia"].update(frente_eje_base="+Y", centro_huella_local_mm=[5, 10], plano_montaje_z_mm=20)
    perfil["medidas_declaradas"]["base_brazo_a_superficie"] = 25
    perfil["geometria"]["seccion_aro_mm"]["alto"] = 40

    mesa = generar_mesa(perfil)

    assert mesa["confirmado"] is False
    assert mesa["arista_superior_z_mm"] == 270
    assert mesa["referencia"]["centro_huella_xy_mm"] == [-10, 5]
    assert mesa["cajas"][0]["min_mm"] == [-410, 507.5, 230]
    assert mesa["cajas"][0]["max_mm"] == [390, 557.5, 270]


def test_mesa_rechaza_hipotesis_marcadas_como_confirmadas():
    perfil = cargar_perfil_mesa()
    perfil["hipotesis"] = ["Sin confirmar la referencia del frente."]

    with pytest.raises(ValueError, match="resuelve las hipótesis"):
        generar_mesa(perfil)
