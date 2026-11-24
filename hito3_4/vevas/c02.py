"""C02 · evidencia de que ninguna junta supera su tope de velocidad en el robot real.

Cómo se cierra C02 en esta versión (24/11/2026):

1. El RAPID (rapid/VEVAS_RWS.mod) mueve con ``MoveAbsJ ... \\T:=t`` y calcula t él mismo:
   ``t = max(0.5, kPico·|Δq_i| / vLim_i)`` con vLim = 5 % de la ficha ABB (12.5, 12.5, 12.5, 16, 16, 21 °/s)
   y kPico = 2.5. La PC no puede pedir más velocidad: el tope está en el controlador.
2. ``herramientas/verificar_c02.py`` mueve cada junta ±δ en el robot real, lee la postura por RWS lo más
   rápido posible, estima la velocidad por diferencias con ventana de 0.1 s y guarda la evidencia
   (``evidencia_c02.json``) ligada a la huella del módulo, al modo de operación y a los topes.
3. El backend RWS solo habilita el envío si la evidencia existe, pasó, corresponde al MISMO texto de
   módulo (local y en el controlador) y al modo actual. Cambiar el .mod invalida la evidencia.

Limitaciones (documentadas también en docs/CONTROL_FISICO_PENDIENTE.md): la velocidad se estima desde
posiciones leídas por RWS (no es una traza interna del controlador) y una diferencia finita promedia en
su ventana. Por eso el tope del RAPID usa kPico = 2.5 de margen sobre la velocidad media.
"""
from __future__ import annotations

import json
import os

from .rws_cliente import huella_modulo, limites_c02  # noqa: F401 (huella_modulo se reexporta)

VERSION_EVIDENCIA = 1
VENTANA_S = 0.10          # ventana de la diferencia finita
TOL_TIEMPO = 0.02         # |vvRbTiempo − t esperado| / t esperado


def ruta_por_defecto(raiz: str) -> str:
    return os.path.join(raiz, "evidencia_c02.json")


def velocidades_max(muestras: list[tuple[float, list[float]]], ventana_s: float = VENTANA_S) -> list[float]:
    """Máximo |Δq/Δt| por junta (°/s) con Δt ≥ ventana_s entre muestras."""
    vmax = [0.0] * 6
    j = 0
    for i in range(len(muestras)):
        ti, qi = muestras[i]
        if j <= i:
            j = i + 1
        while j < len(muestras) and muestras[j][0] - ti < ventana_s:
            j += 1
        if j >= len(muestras):
            break
        tj, qj = muestras[j]
        dt = tj - ti
        for k in range(6):
            vmax[k] = max(vmax[k], abs(qj[k] - qi[k]) / dt)
    return vmax


def evaluar(ruta: str, texto_modulo_local: str, opmode: str | None = None,
            texto_modulo_controlador: str | None = None) -> tuple[bool, str, dict | None]:
    """¿La evidencia guardada habilita el envío? Devuelve (ok, motivo, evidencia)."""
    if not os.path.exists(ruta):
        return False, ("C02 sin evidencia: corré «python3 herramientas/verificar_c02.py» con el robot en AUTO "
                       "y una persona en el paro de emergencia."), None
    try:
        with open(ruta, encoding="utf-8") as f:
            ev = json.load(f)
    except (OSError, ValueError) as e:
        return False, f"C02: evidencia ilegible ({e}).", None
    if ev.get("version") != VERSION_EVIDENCIA:
        return False, "C02: la evidencia es de otra versión del protocolo; repetí la verificación.", ev
    if not ev.get("ok"):
        return False, "C02: la última verificación NO pasó; revisá evidencia_c02.json y repetila.", ev
    try:
        vlim, k = limites_c02(texto_modulo_local)
    except Exception as e:  # noqa: BLE001
        return False, f"C02: {e}", ev
    if ev.get("huella_modulo") != huella_modulo(texto_modulo_local):
        return False, "C02: el módulo RAPID cambió desde la verificación; repetila.", ev
    if list(ev.get("vlim_deg_s", [])) != list(vlim) or float(ev.get("k_pico", -1)) != k:
        return False, "C02: los topes del módulo no coinciden con los verificados.", ev
    probadas = {int(m["junta"]) for m in ev.get("movimientos", []) if m.get("ok")}
    if probadas != {1, 2, 3, 4, 5, 6}:
        return False, f"C02: faltan juntas verificadas ({sorted({1, 2, 3, 4, 5, 6} - probadas)}).", ev
    if opmode is not None and opmode.upper() not in [m.upper() for m in ev.get("modos", [])]:
        return False, (f"C02: verificado en {', '.join(ev.get('modos', []))}; el controlador está en {opmode}. "
                       "Una verificación acredita solo su modo."), ev
    if texto_modulo_controlador is not None and huella_modulo(texto_modulo_controlador) != ev["huella_modulo"]:
        return False, "C02: el módulo cargado en el IRC5 no es el verificado (reinstalalo o repetí la verificación).", ev
    return True, f"C02 verificado el {ev.get('fecha_utc', '?')[:10]} en {', '.join(ev.get('modos', []))}.", ev
