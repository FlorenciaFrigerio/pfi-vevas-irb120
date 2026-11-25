"""Geometría parametrizable del bastidor abierto, en el marco base del robot."""

from __future__ import annotations

import json
from math import isfinite
from pathlib import Path

RUTA_PERFIL_MESA = Path(__file__).resolve().parents[1] / "03_modelo" / "mesa_medidas.json"
FACTORES_MM = {"mm": 1, "cm": 10, "m": 1000}
GIROS_CARDINALES = {"+X": (1, 0), "+Y": (0, 1), "-X": (-1, 0), "-Y": (0, -1)}


def cargar_perfil_mesa(ruta: str | Path | None = None) -> dict:
    """Carga el perfil de medidas; por defecto usa el perfil de este proyecto."""
    with Path(ruta or RUTA_PERFIL_MESA).open(encoding="utf-8") as archivo:
        return json.load(archivo)


def generar_mesa(perfil: dict | None = None) -> dict:
    """Devuelve el aro como cajas de entorno y los apoyos inferiores ilustrativos.

    En el marco local de la mesa, X apunta al frente, Y a la izquierda y Z hacia
    arriba. Se aplica un giro cardinal para llevar las cajas al marco ABB.
    """
    if perfil is None:
        perfil = cargar_perfil_mesa()
    unidad = perfil["unidad_para_modelo"]
    factor = FACTORES_MM.get(unidad)
    if factor is None:
        raise ValueError("La unidad de la mesa debe ser mm, cm o m.")
    confirmado = perfil["confirmado"]
    hipotesis = perfil["hipotesis"]
    if not isinstance(confirmado, bool):
        raise ValueError("El estado de confirmación de la mesa debe ser booleano.")
    if confirmado and (perfil["unidad_confirmada"] != unidad or hipotesis):
        raise ValueError("Confirma la unidad y resuelve las hipótesis antes de confirmar la mesa.")

    medidas = {clave: valor * factor for clave, valor in perfil["medidas_declaradas"].items()}
    if any(not isfinite(valor) or valor < 0 for valor in medidas.values()):
        raise ValueError("Las medidas de la mesa deben ser finitas y no negativas.")
    largo = medidas["largo"]
    ancho = medidas["ancho"]
    desnivel = medidas["base_brazo_a_superficie"]
    altura_cilindro = medidas["base_brazo_a_superior_cilindro"]
    frente = medidas["frente_mesa_a_frente_brazo"]
    atras = medidas["atras_mesa_a_espalda_brazo"]
    izquierda = medidas["lado_izquierdo"]
    derecha = medidas["lado_derecho"]
    profundidad_huella = largo - frente - atras
    ancho_huella = ancho - izquierda - derecha
    if min(profundidad_huella, ancho_huella, desnivel) <= 0:
        raise ValueError("Las medidas deben dejar una huella positiva y un aro sobre el montaje.")

    referencia = perfil["referencia"]
    giro = GIROS_CARDINALES.get(referencia["frente_eje_base"])
    if giro is None:
        raise ValueError("El frente de la mesa debe corresponder a +X, +Y, -X o -Y.")
    cx, cy = referencia["centro_huella_local_mm"]
    z_montaje = referencia["plano_montaje_z_mm"]
    if not all(isfinite(valor) for valor in (cx, cy, z_montaje)):
        raise ValueError("La referencia de montaje debe usar coordenadas finitas.")
    coseno, seno = giro

    geometria = perfil["geometria"]
    ancho_aro = geometria["seccion_aro_mm"]["ancho"]
    alto_aro = geometria["seccion_aro_mm"]["alto"]
    ancho_montaje = geometria["seccion_montaje_mm"]["ancho"]
    alto_montaje = geometria["seccion_montaje_mm"]["alto"]
    secciones = (ancho_aro, alto_aro, ancho_montaje, alto_montaje)
    if any(not isfinite(valor) or valor <= 0 for valor in secciones):
        raise ValueError("Las secciones de la mesa deben tener dimensiones finitas positivas.")
    if 2 * ancho_aro >= min(largo, ancho) or 2 * ancho_montaje >= profundidad_huella:
        raise ValueError("Las secciones de los travesaños deben dejar el bastidor abierto.")

    x_huella_min = cx - profundidad_huella / 2
    x_huella_max = cx + profundidad_huella / 2
    x_min = x_huella_min - atras
    x_max = x_huella_max + frente
    y_min = cy - ancho_huella / 2 - derecha
    y_max = cy + ancho_huella / 2 + izquierda
    z_aro = z_montaje + desnivel

    def caja(nombre, x0, x1, y0, y1, z0, z1):
        puntos = [(coseno * x - seno * y, seno * x + coseno * y)
                  for x in (x0, x1) for y in (y0, y1)]
        xs, ys = zip(*puntos)
        return {
            "nombre": f"Mesa · {nombre}",
            "min_mm": [min(xs), min(ys), z0],
            "max_mm": [max(xs), max(ys), z1],
        }

    cajas = [
        caja("aro delantero", x_max - ancho_aro, x_max, y_min, y_max, z_aro - alto_aro, z_aro),
        caja("aro trasero", x_min, x_min + ancho_aro, y_min, y_max, z_aro - alto_aro, z_aro),
        caja("aro izquierdo", x_min + ancho_aro, x_max - ancho_aro,
             y_max - ancho_aro, y_max, z_aro - alto_aro, z_aro),
        caja("aro derecho", x_min + ancho_aro, x_max - ancho_aro,
             y_min, y_min + ancho_aro, z_aro - alto_aro, z_aro),
    ]
    apoyos = [
        caja("montaje delantero (no medido)", x_huella_max - ancho_montaje, x_huella_max,
             y_min, y_max, z_montaje - alto_montaje, z_montaje),
        caja("montaje trasero (no medido)", x_huella_min, x_huella_min + ancho_montaje,
             y_min, y_max, z_montaje - alto_montaje, z_montaje),
    ]
    return {
        "cajas": cajas,
        "geometria_visual": [{**apoyo, "confirmado": False} for apoyo in apoyos],
        "arista_superior_z_mm": z_aro,
        "cilindro_base_z_mm": z_montaje + altura_cilindro,
        "confirmado": confirmado,
        "hipotesis": list(hipotesis),
        "hipotesis_visual": list(perfil["hipotesis_visual"]),
        "referencia": {
            "frente_eje_base": referencia["frente_eje_base"],
            "centro_huella_xy_mm": [coseno * cx - seno * cy, seno * cx + coseno * cy],
            "plano_montaje_z_mm": z_montaje,
        },
        "huella_brazo_mm": {"profundidad": profundidad_huella, "ancho": ancho_huella},
        "unidad_confirmada": perfil["unidad_confirmada"],
        "unidad_para_modelo": unidad,
        "medidas_declaradas": dict(perfil["medidas_declaradas"]),
        "geometria": {clave: dict(seccion) for clave, seccion in geometria.items()},
    }
