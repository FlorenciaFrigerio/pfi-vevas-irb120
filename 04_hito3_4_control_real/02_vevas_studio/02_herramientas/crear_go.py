#!/usr/bin/env python3
"""Registra el GO de la sesión con dos respuestas explícitas; nunca autoriza por -y ni modifica config.json."""
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
from vevas.velocidad import VEL_PCT_DEFECTO, VEL_PCT_MAX, pct_valido

TOPE_DEFECTO = VEL_PCT_DEFECTO   # Enter = 5 %
TOPE_MAXIMO = VEL_PCT_MAX        # 50 %; con RWS el RAPID (C02) limita igual cada junta al 5 % de la ficha
# Campos de registro que antes se preguntaban uno por uno; se completan solos para no demorar la sesión.
AUTOMATICOS = {
    'autoriza': 'Docente IMT-342 (ensayos del PFI)',
    'operador_pendant': 'Grupo VEVAS',
    'responsable_consola': 'Grupo VEVAS',
    'supervisor': 'Grupo VEVAS',
    'alcance': 'Ensayos VEVAS del Hito 3/4 con revisión previa',
}

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


def velocidad(maximo, defecto):
    while True:
        try:
            crudo = input(f'Tope de velocidad en % [Enter = {defecto:g}, máximo {maximo:g}]: ').strip()
            valor = float(crudo.replace(',', '.')) if crudo else defecto
            if math.isfinite(valor) and 0 < valor <= maximo and pct_valido(valor):
                return valor
        except ValueError:
            pass
        print(f'Número fuera de rango (0 < v ≤ {maximo:g}). Con RWS el RAPID (C02) limita igual cada junta al 5 % de la ficha.')


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
        print('GO de la sesión (vale hasta el final del día). Responder solo lo verificado en persona.')
        go = {'fecha': dt.date.today().isoformat(), 'decision': 'NO-GO'}
        go.update(AUTOMATICOS)
        listo = si('¿Zona despejada, brida sin herramienta, nada fuera de la mesa y paro de emergencia a mano?')
        maximo = min(TOPE_MAXIMO, float(cfg['seguridad']['velocidad_real_max_pct']))
        go['velocidad_max_pct'] = velocidad(maximo, min(TOPE_DEFECTO, maximo))
        go['checklist'] = {k: listo for k in CHECKLIST_GO}
        entorno = copy.deepcopy(cfg['entorno'])
        entorno['sin_objetos_externos_confirmado'] = listo
        entorno['sin_herramienta_confirmada'] = listo
        entorno.setdefault('herramienta', {})['confirmada'] = False
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
