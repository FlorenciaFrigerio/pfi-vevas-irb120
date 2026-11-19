#!/usr/bin/env python3
"""Asistente: crea GO_VEVAS.json (del día) y completa la sección «entorno» de config.json preguntando todo.
Uso:  python3 herramientas/crear_go.py
Solo respondé «s» a lo que comprobaste en persona: el GO es la constancia de la sesión."""
import datetime as dt, json, os, sys
AQUI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, AQUI)
from vevas.consola import CHECKLIST_GO, validar_go  # noqa: E402

TEXTO = {
    "1_identidad_controlador_manual": "¿Es el IRC5 / IRB 120 correcto y revisaste su estado (calibración, sin alarmas)?",
    "2_autorizacion_acceso_robot": "¿Tenés autorización para usar el robot hoy?",
    "3_operador_y_supervisor_presentes": "¿Están presentes el operador y el supervisor?",
    "4_modo_operativo_permitido": "¿El modo (AUTO) está permitido para esta prueba?",
    "5_zona_despejada": "¿La zona del robot está despejada (nadie dentro del alcance)?",
    "6_paro_emergencia_y_parada_normal_probados": "¿Probaste el paro de emergencia y alguien tiene la mano en él?",
    "7_trayectoria_revisada_en_simulacion": "¿Revisaste el movimiento en simulación antes?",
    "8_limites_velocidad_tool_base_revisados": "¿Revisaste límites, velocidad baja, tool y base?",
    "9_sin_pieza_ni_herramienta_peligrosa": "¿No hay pieza ni herramienta peligrosa montada?",
    "10_registro_y_aborto_funcionando": "¿El registro y el botón PARAR funcionan?",
    "11_condiciones_de_aborto_y_recuperacion_acordadas": "¿Acordaron cuándo abortar y cómo recuperar?",
}

def pedir(t, defecto=""):
    while True:
        r = input(f"{t}{f' [{defecto}]' if defecto else ''}: ").strip() or defecto
        if r: return r

def numero(t, defecto, lo, hi):
    while True:
        r = input(f"{t} [{defecto}]: ").strip().replace(",", ".") or str(defecto)
        try:
            v = float(r)
            if lo < v <= hi: return v
        except ValueError:
            pass
        print(f"  Escribí un número entre {lo} y {hi} (o Enter para {defecto}).")

def si(t):
    while True:
        r = input(f"{t} (s/n): ").strip().lower()
        if r in ("s", "si", "sí"): return True
        if r in ("n", "no"): return False

AUTOMATICO = "--automatico" in sys.argv
if AUTOMATICO:
    _respuestas = iter([])
    import getpass
    quien = os.environ.get("VEVAS_RESPONSABLE") or getpass.getuser()
    vel = float(os.environ.get("VEVAS_VEL_PCT", "5"))
    go = {"fecha": dt.date.today().isoformat(), "decision": "GO", "autoriza": f"{quien} (confirmado en iniciar_real.sh)",
          "operador_pendant": quien, "responsable_consola": quien, "supervisor": quien,
          "alcance": f"Pruebas articulares y cartesianas cortas a {vel:g} % de velocidad",
          "velocidad_max_pct": vel, "checklist": {k: True for k in CHECKLIST_GO}}
else:
 print("== GO del día (rápido) ==")
 nombre = pedir("Nombre del docente/responsable que autoriza y supervisa")
 operador = pedir("Quién tiene la mano en el paro de emergencia", nombre)
 vel = numero("Velocidad máxima en % (recomendado 5)", 5, 0, 25)
 print("\nChecklist (confirmás todo junto):")
 for k in CHECKLIST_GO:
     print("  · " + TEXTO.get(k, k))
 todo = si("¿Está TODO lo de arriba verificado en persona?")
 go = {"fecha": dt.date.today().isoformat(), "decision": "GO", "autoriza": nombre, "operador_pendant": operador,
       "responsable_consola": nombre, "supervisor": nombre,
       "alcance": f"Pruebas articulares y cartesianas cortas a {vel:g} % de velocidad",
       "velocidad_max_pct": vel, "checklist": {k: todo for k in CHECKLIST_GO}}
ok, msg = validar_go(go)
ruta = sys.argv[sys.argv.index("--salida") + 1] if "--salida" in sys.argv else os.path.join(AQUI, "GO_VEVAS.json")
if not ok:
    print(f"\nNO se guarda el GO: {msg}\nSi algún punto es «no», hay que resolverlo antes de mover el robot.")
    sys.exit(1)

print("" if AUTOMATICO else "\n== Entorno (config.json) ==")
cfgp = os.path.join(AQUI, "config.json")
import re
texto = open(cfgp, encoding="utf-8").read()   # tiene comentarios //: se edita el texto, no se reescribe el JSON

def poner(clave, valor):
    global texto
    v = json.dumps(valor)
    nuevo, n = re.subn(rf'("{clave}"\s*:\s*)(true|false|[-0-9.]+)', lambda m: m.group(1) + v, texto, count=1)
    if n == 0:
        raise SystemExit(f"No encontré «{clave}» en config.json; editalo a mano.")
    texto = nuevo

sin_obj = True if AUTOMATICO else si("¿Confirmás que NO hay objetos dentro del alcance del robot (aparte de la mesa)?")
poner("sin_objetos_externos_confirmado", sin_obj)
if not sin_obj:
    print("  Describí las cajas en config.json → entorno.cajas (mm, ejes de la base). Mientras tanto VEVAS no enviará al robot real.")
sin_herr = True if AUTOMATICO else si("¿Confirmás que NO hay herramienta en la brida?")
poner("sin_herramienta_confirmada", sin_herr)
if not sin_herr:
    poner("radio_mm", float(pedir("Radio aproximado de la herramienta en mm", "30")))
    poner("confirmada", si("¿Medida y TCP (robot.tcp_xyz_m) revisados?"))
open(cfgp, "w", encoding="utf-8").write(texto)
from vevas.seguridad import cargar_config  # noqa: E402
e = cargar_config(cfgp)["entorno"]
print(f"config.json actualizado (sin objetos={e['sin_objetos_externos_confirmado']}, sin herramienta={e['sin_herramienta_confirmada']}).")
json.dump(go, open(ruta, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
print(f"GO guardado en {ruta}")
print("El script iniciar_real.sh detecta el GO solo y sigue.")
