#!/usr/bin/env python3
"""Registra declaraciones explícitas de la sesión; nunca autoriza por -y ni modifica config.json."""
from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import math
import os
from pathlib import Path
import sys
import tempfile

RAIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ))
from vevas.consola import CHECKLIST_GO, validar_go
from vevas.seguridad import cargar_config

TEXTO = {
    '1_identidad_controlador_manual': 'Controlador correcto y estado/calibración revisados',
    '2_autorizacion_acceso_robot': 'Autorización del responsable para esta sesión',
    '3_operador_y_supervisor_presentes': 'Operador y supervisor presentes',
    '4_modo_operativo_permitido': 'Modo operativo autorizado para el ensayo',
    '5_zona_despejada': 'Zona despejada y acceso controlado',
    '6_paro_emergencia_y_parada_normal_probados': 'Procedimientos de parada comprobados',
    '7_trayectoria_revisada_en_simulacion': 'Trayectoria revisada en simulación',
    '8_limites_velocidad_tool_base_revisados': 'Límites, velocidad, herramienta y base revisados',
    '9_sin_pieza_ni_herramienta_peligrosa': 'Sin pieza ni herramienta peligrosa',
    '10_registro_y_aborto_funcionando': 'Registro y cancelación de software comprobados (no sustituyen paro físico)',
    '11_condiciones_de_aborto_y_recuperacion_acordadas': 'Condiciones de aborto y recuperación acordadas',
}

def texto(pregunta):
    while True:
        valor = input(pregunta + ': ').strip()
        if valor:
            return valor
        print('Se requiere una respuesta explícita.')


def si(pregunta):
    while True:
        valor = texto(pregunta + ' [s/n]').lower()
        if valor in ('s', 'si', 'sí'):
            return True
        if valor in ('n', 'no'):
            return False


def velocidad(maximo):
    while True:
        try:
            valor = float(texto(f'Tope del plan en % (mayor que 0, máximo {maximo:g})').replace(',', '.'))
            if math.isfinite(valor) and 0 < valor <= maximo:
                return valor
        except ValueError:
            pass
        print('Número fuera de rango. Este porcentaje no certifica la velocidad articular ejecutada por RAPID.')


def guardar(ruta, datos):
    ruta.parent.mkdir(parents=True, exist_ok=True)
    fd, temporal = tempfile.mkstemp(prefix='.go_', suffix='.tmp', dir=ruta.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as salida:
            json.dump(datos, salida, ensure_ascii=False, indent=2, allow_nan=False)
            salida.write('\n')
        os.replace(temporal, ruta)
    finally:
        if os.path.exists(temporal):
            os.unlink(temporal)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--salida', type=Path, default=RAIZ / 'GO_VEVAS.json')
    ap.add_argument('--config', type=Path, default=RAIZ / 'config.json')
    ap.add_argument('--reemplazar', action='store_true', help='permite reemplazar explícitamente un registro de sesión existente')
    ap.add_argument('--automatico', action='store_true', help=argparse.SUPPRESS)
    args = ap.parse_args(argv)
    if args.automatico:
        ap.error('No se genera un GO automático. Las declaraciones requieren respuestas explícitas del responsable.')
    if args.salida.exists() and not args.reemplazar:
        ap.error('El archivo ya existe; se conserva, incluso si dice NO-GO. Revisalo o elegí otra --salida.')
    cfg = cargar_config(str(args.config))
    try:
        print('Registro de sesión: responder solo por condiciones verificadas. No habilita ni mueve el controlador.')
        go = {'fecha': dt.date.today().isoformat(), 'decision': 'NO-GO'}
        for campo, pregunta in (
            ('autoriza', 'Nombre y referencia de la autorización del responsable'),
            ('operador_pendant', 'Nombre del operador del FlexPendant'),
            ('responsable_consola', 'Nombre del responsable de consola'),
            ('supervisor', 'Nombre del supervisor'),
            ('alcance', 'Alcance concreto de los ensayos autorizados'),
        ):
            go[campo] = texto(pregunta)
        go['velocidad_max_pct'] = velocidad(float(cfg['seguridad']['velocidad_real_max_pct']))
        go['checklist'] = {k: si(TEXTO.get(k, k)) for k in CHECKLIST_GO}
        entorno = copy.deepcopy(cfg['entorno'])
        entorno['sin_objetos_externos_confirmado'] = si('¿Se verificó que NO hay objetos externos, aparte de la mesa modelada?')
        entorno['sin_herramienta_confirmada'] = si('¿Se verificó la brida SIN herramienta?')
        if entorno['sin_herramienta_confirmada']:
            entorno.setdefault('herramienta', {})['confirmada'] = False
        if not entorno['sin_herramienta_confirmada']:
            entorno.setdefault('herramienta', {})['confirmada'] = si('¿La geometría de herramienta y el TCP de config.json están medidos y verificados?')
        go['entorno'] = entorno
        if all(go['checklist'].values()):
            go['decision'] = 'GO'
        ok, mensaje = validar_go(go)
        if not ok:
            go['decision'] = 'NO-GO'
        guardar(args.salida, go)
        print(f"{go['decision']} registrado en {args.salida}: {mensaje}")
        print('config.json no cambió. Las declaraciones de entorno se aplican solamente con este GO de sesión.')
        return 0 if ok else 1
    except (EOFError, KeyboardInterrupt):
        print('\nInterrumpido: no se guardó ni se autorizó la sesión.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
