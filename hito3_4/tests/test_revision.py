"""Pruebas de la revisión previa y de los defectos C01–C04. Se corren con:  python3 -m pytest tests -q
(o directamente: python3 tests/test_revision.py)."""
import datetime as dt
import math
import os
import sys
import tempfile
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vevas.backend_sim import BackendSim  # noqa: E402
from vevas.consola import CHECKLIST_GO, Consola, validar_go  # noqa: E402
from vevas.registro import Registro  # noqa: E402
from vevas.robot_irb120 import Robot  # noqa: E402
from vevas.revision import revisar_trayectoria  # noqa: E402
from vevas.seguridad import cargar_config  # noqa: E402
from vevas.trayectoria import planificar  # noqa: E402

CFG_RUTA = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config.json")
HOME = [0, -20, 30, 0, 40, 0]


def cfg():
    return cargar_config(CFG_RUTA)


def nueva(q0=HOME, backend_cls=BackendSim, **kw):
    c = cfg()
    c.update(kw.pop("cfg_extra", {}))
    b = backend_cls(np.radians(q0))
    reg = Registro(tempfile.mkdtemp(), "test")
    con = Consola(c, b, reg, **kw)
    b.iniciar()
    con.iniciar()
    time.sleep(0.15)
    return con, b


def cerrar(con, b):
    con.detener()
    b.detener()
    time.sleep(0.1)
    con.reg.cerrar()


def chequeo(plan, id_):
    return next(c for c in plan["revision"]["chequeos"] if c["id"] == id_)


# ---- revisión
def test_junta_chica_aprobada_en_simulacion():
    con, b = nueva()
    try:
        p = con.calcular_junta(1, 5.0, True)
        assert p["ejecutable"], p["problemas"]
        for id_ in ("limites", "velocidad", "singularidad", "altura", "autocolision"):
            assert chequeo(p, id_)["estado"] == "aprobado", chequeo(p, id_)
        # sin objetos descritos ni herramienta confirmada: NO es «aprobado»
        assert chequeo(p, "entorno")["estado"] == "no_verificado"
        assert chequeo(p, "herramienta")["estado"] == "no_verificado"
        assert p["revision"]["estado"] == "no_verificado"
    finally:
        cerrar(con, b)


def test_destino_singular_rechazado_con_falla_localizada():
    con, b = nueva()
    try:
        p = con.calcular_articular([0, -20, 30, 0, 0, 0])  # q5 = 0: singularidad de muñeca
        assert not p["ejecutable"]
        c = chequeo(p, "singularidad")
        assert c["estado"] == "rechazado" and "muñeca" in c["detalle"]
        f = p["revision"]["falla"]
        assert f and f["check"] == "singularidad" and len(f["marcos"]) == 8
        assert p["preview"]["frames"][f["k_preview"]] == f["marcos"]
    finally:
        cerrar(con, b)


def test_parte_de_singular_y_no_la_empeora_es_aviso():
    con, b = nueva(q0=[0, 0, 0, 0, 0, 0])
    try:
        p = con.calcular_junta(1, 5.0, True)
        assert chequeo(p, "singularidad")["estado"] == "aviso"
        assert p["ejecutable"]
    finally:
        cerrar(con, b)


def test_altura_minima_rechaza_y_marca_tiempo():
    con, b = nueva()
    try:
        p = con.calcular_articular([0, 100, 0, 0, 40, 0])
        assert not p["ejecutable"]
        c = chequeo(p, "altura")
        assert c["estado"] == "rechazado" and c["t"] is not None
        assert any(x == 2 for x in p["revision"]["cinta"])
    finally:
        cerrar(con, b)


def test_caja_en_el_camino_rechaza_y_fuera_del_camino_no():
    ent_en_camino = {"entorno": {"cajas": [{"nombre": "tablero", "min_mm": [250, -50, 250], "max_mm": [450, 50, 600]}],
                                 "sin_objetos_externos_confirmado": False, "mesa_z_m": 0.0,
                                 "sin_herramienta_confirmada": False, "herramienta": {"radio_mm": 30.0, "confirmada": False}}}
    con, b = nueva(cfg_extra=ent_en_camino)
    try:
        p = con.calcular_articular([0, 30, 30, 0, 40, 0])
        c = chequeo(p, "entorno")
        assert c["estado"] == "rechazado" and "tablero" in c["detalle"], c
    finally:
        cerrar(con, b)
    lejos = {"entorno": {"cajas": [{"nombre": "pared", "min_mm": [-900, -900, 0], "max_mm": [-800, 900, 800]}],
                         "sin_objetos_externos_confirmado": False, "mesa_z_m": 0.0,
                         "sin_herramienta_confirmada": False, "herramienta": {"radio_mm": 30.0, "confirmada": False}}}
    con, b = nueva(cfg_extra=lejos)
    try:
        p = con.calcular_junta(1, 5.0, True)
        assert chequeo(p, "entorno")["estado"] == "aprobado"
    finally:
        cerrar(con, b)


def test_autocolision_detectada():
    con, b = nueva()
    try:
        # postura con el antebrazo/muñeca atravesando la base, con z > altura mínima
        p = con.calcular_articular([158, 51, 59, -5, 59, -31])
        c = chequeo(p, "autocolision")
        assert c["estado"] == "rechazado" and "base" in c["detalle"], c
        assert not p["ejecutable"] and p["revision"]["falla"]["check"] in ("autocolision", "limites", "altura", "entorno")
        # y una postura normal no genera falsos positivos
        assert chequeo(con.calcular_articular([30, -10, 20, 0, 40, 0]), "autocolision")["estado"] == "aprobado"
    finally:
        cerrar(con, b)


def test_tope_de_velocidad_por_junta_c02():
    c = cfg()
    R = Robot()
    q0 = np.radians(HOME)
    qf = np.radians([60, -20, 30, 0, 40, 0])
    tray = planificar(q0, qf, R.vel_max, 1.0, 0.5, 0.25, 1.0)  # al 100 %
    ts, Q, QD, _ = tray.muestrear(0.02)
    inf = revisar_trayectoria(R, ts, Q, QD, q0, c, 100.0)
    assert next(x for x in inf["chequeos"] if x["id"] == "velocidad")["estado"] == "aprobado"
    inf = revisar_trayectoria(R, ts, Q, QD, q0, c, 10.0)  # tope real del 10 %
    v = next(x for x in inf["chequeos"] if x["id"] == "velocidad")
    assert v["estado"] == "rechazado" and v["juntas"] == [1]
    assert inf["bloquea_envio"]


# ---- C01: GO
def go_valido():
    return {"fecha": dt.date.today().isoformat(), "decision": "GO", "autoriza": "Ing. X en persona",
            "operador_pendant": "A", "responsable_consola": "B", "supervisor": "C", "alcance": "J1 +5° a 3°/s",
            "velocidad_max_pct": 5, "checklist": {k: True for k in CHECKLIST_GO}}


def test_go_vacio_o_incompleto_no_vale():
    assert not validar_go(None)[0]
    assert not validar_go({})[0]
    assert validar_go(go_valido())[0]
    for borrar in ("fecha", "autoriza", "supervisor", "alcance", "checklist", "velocidad_max_pct"):
        g = go_valido()
        g.pop(borrar)
        assert not validar_go(g)[0], borrar
    g = go_valido(); g["checklist"] = {}; assert not validar_go(g)[0]
    g = go_valido(); g["checklist"].pop(CHECKLIST_GO[3]); assert not validar_go(g)[0]
    g = go_valido(); g["checklist"][CHECKLIST_GO[0]] = "true"; assert not validar_go(g)[0]
    g = go_valido(); g["supervisor"] = "COMPLETAR: quien supervisa"; assert not validar_go(g)[0]
    g = go_valido(); g["decision"] = "NO-GO"; assert not validar_go(g)[0]
    g = go_valido(); g["fecha"] = "2020-01-01"; assert not validar_go(g)[0]
    g = go_valido(); g["velocidad_max_pct"] = 0; assert not validar_go(g)[0]


# ---- robot "real" falso para las compuertas
class FalsoReal(BackendSim):
    tipo = "ros"
    retardo_envio = 0.0
    antiguedad = 0.0

    def info(self):
        return {"tipo": "ros", "hardware": "falso", "real": True, "texto": "ROS 2 · ROBOT REAL (falso)", "solo_lectura": False}

    def leer(self):
        q, t = super().leer()
        return q, t - self.antiguedad

    def ejecutar_trayectoria(self, ts, Q, QD=None):
        time.sleep(self.retardo_envio)
        self.enviado = True
        return super().ejecutar_trayectoria(ts, Q, QD)


def real_listo(go=None, extra=None):
    e = {"entorno": {"cajas": [], "sin_objetos_externos_confirmado": True, "mesa_z_m": 0.0,
                     "sin_herramienta_confirmada": True, "herramienta": {"radio_mm": 30.0, "confirmada": False}}}
    if extra:
        e["entorno"].update(extra)
    con, b = nueva(backend_cls=FalsoReal, permitir_real=True, go=go or go_valido(), cfg_extra=e)
    b.enviado = False
    return con, b


CHECK = {k: True for k in ("supervisor", "paro", "area", "velocidad")}
FRASE = "MOVER ROBOT REAL"


def test_real_sin_go_no_arma_y_con_go_si():
    con, b = nueva(backend_cls=FalsoReal, permitir_real=True, go={})
    try:
        assert not con.armar(CHECK, FRASE)[0]
    finally:
        cerrar(con, b)
    con, b = real_listo()
    try:
        assert not con.armar(CHECK, "otra frase")[0]
        assert con.armar(CHECK, FRASE)[0]
        assert con.vel_pct <= 5  # el tope del GO (5 %) manda sobre el 10 % de config
    finally:
        cerrar(con, b)


def test_real_bloquea_si_entorno_o_herramienta_no_verificados():
    con, b = nueva(backend_cls=FalsoReal, permitir_real=True, go=go_valido())
    try:
        con.armar(CHECK, FRASE)
        p = con.calcular_junta(1, 2.0, True)
        assert not p["ejecutable"] and p["revision"]["bloquea_envio"]
        ok, msg = con.ejecutar()
        assert not ok and "No se envía" in msg
    finally:
        cerrar(con, b)


def test_real_envia_un_plan_aprobado_y_el_tope_de_velocidad_manda():
    con, b = real_listo()
    try:
        assert con.armar(CHECK, FRASE)[0]
        p = con.calcular_junta(1, 2.0, True)
        assert p["ejecutable"], p["problemas"]
        assert p["vel_pct"] <= 5
        ok, msg = con.ejecutar()
        assert ok, msg
        assert b.enviado
    finally:
        cerrar(con, b)


def test_c04_estado_viejo_no_se_envia():
    con, b = real_listo()
    try:
        con.armar(CHECK, FRASE)
        p = con.calcular_junta(1, 2.0, True)
        assert p["ejecutable"]
        b.antiguedad = 2.0  # el estado quedó viejo después del cálculo
        ok, msg = con.ejecutar()
        assert not ok and "estado" in msg.lower() and not b.enviado
        b.antiguedad = 0.0
        # tampoco se deja armar ni empezar un jog con estado viejo
        con.desarmar()
        b.antiguedad = 2.0
        assert not con.armar(CHECK, FRASE)[0]
    finally:
        cerrar(con, b)


def test_c03_desarmar_durante_el_envio_cancela():
    con, b = real_listo()
    try:
        con.armar(CHECK, FRASE)
        con.calcular_junta(1, 2.0, True)
        b.retardo_envio = 0.4
        resultado = {}
        t = threading.Thread(target=lambda: resultado.update(r=con.ejecutar()))
        t.start()
        time.sleep(0.1)
        con.desarmar("prueba")
        t.join()
        ok, msg = resultado["r"]
        assert not ok and "cancel" in msg.lower()
        assert con.ejec is None and not con.armado
    finally:
        cerrar(con, b)


def test_c03_parar_durante_el_envio_cancela():
    con, b = real_listo()
    try:
        con.armar(CHECK, FRASE)
        con.calcular_junta(1, 2.0, True)
        b.retardo_envio = 0.4
        resultado = {}
        t = threading.Thread(target=lambda: resultado.update(r=con.ejecutar()))
        t.start()
        time.sleep(0.1)
        con.parar("prueba")
        t.join()
        assert not resultado["r"][0] and con.ejec is None
    finally:
        cerrar(con, b)


def test_c02_plan_calculado_a_mas_velocidad_que_el_tope_se_rechaza():
    con, b = real_listo()
    try:
        con.armar(CHECK, FRASE)
        p = con.calcular_junta(1, 2.0, True)
        con.plan["vel_pct"] = 50.0  # como si se hubiera calculado antes de bajar el tope
        ok, msg = con.ejecutar()
        assert not ok and "tope" in msg
    finally:
        cerrar(con, b)


def test_go_se_vuelve_a_comprobar_al_enviar():
    con, b = real_listo()
    try:
        con.armar(CHECK, FRASE)
        con.calcular_junta(1, 2.0, True)
        con.go["checklist"][CHECKLIST_GO[4]] = False
        ok, msg = con.ejecutar()
        assert not ok and "GO" in msg or "checklist" in msg.lower()
    finally:
        cerrar(con, b)


if __name__ == "__main__":
    fallas = 0
    for nombre, f in sorted(globals().items()):
        if nombre.startswith("test_"):
            try:
                f()
                print("ok   ", nombre)
            except Exception as e:  # noqa: BLE001
                fallas += 1
                import traceback
                traceback.print_exc()
                print("FALLA", nombre, e)
    sys.exit(1 if fallas else 0)


def test_salir_de_singularidad_de_muneca_en_robot_real():
    """24/11: el robot real quedó en J5 = 0 y la revisión rechazaba incluso J5 → 30°. Salir alejándose es aviso;
    entrar o quedarse en la singularidad sigue rechazado."""
    c = cfg()
    R = Robot()

    def rev(qa_deg, qb_deg):
        qa, qb = np.radians(qa_deg), np.radians(qb_deg)
        tray = planificar(qa, qb, R.vel_max, 0.05, 0.5, 0.25, 1.0)
        ts, Q, QD, _ = tray.muestrear(0.02)
        inf = revisar_trayectoria(R, ts, Q, QD, qa, c, 5.0, exige_go=True)
        return next(x for x in inf["chequeos"] if x["id"] == "singularidad")

    assert rev([0, 0, -25.2, 0, 0, 0], [0, 0, -25.2, 0, 30, 0])["estado"] == "aviso"
    assert rev([0, 0, -25.2, 0, 30, 0], [0, 0, -25.2, 0, 0, 0])["estado"] == "rechazado"
    assert rev([0, 0, -25.2, 0, 0, 0], [10, 0, -25.2, 0, 0, 0])["estado"] == "rechazado"     # se queda singular
    assert rev([0, 0, -25.2, 0, 0, 0], [0, 0, -25.2, 0, 1, 0])["estado"] == "rechazado"      # no se aleja lo suficiente


def test_salir_de_singularidad_cruzando_cero():
    """J5 leído por RWS apenas negativo (−0.3°) y destino +30°: el camino cruza q5 = 0 al principio y después se aleja."""
    c = cfg()
    R = Robot()
    for q5 in (-0.05, -0.3):
        qa, qb = np.radians([0, 0, -25.2, 0, q5, 0]), np.radians([0, 0, -25.2, 0, 30, 0])
        ts, Q, QD, _ = planificar(qa, qb, R.vel_max, 0.05, 0.5, 0.25, 1.0).muestrear(0.02)
        inf = revisar_trayectoria(R, ts, Q, QD, qa, c, 5.0, exige_go=True)
        assert next(x for x in inf["chequeos"] if x["id"] == "singularidad")["estado"] == "aviso", q5
    # cruzar 0 desde lejos (σ inicial fuera de la zona singular) sigue rechazado
    qa, qb = np.radians([0, 0, -25.2, 0, -30, 0]), np.radians([0, 0, -25.2, 0, 30, 0])
    ts, Q, QD, _ = planificar(qa, qb, R.vel_max, 0.05, 0.5, 0.25, 1.0).muestrear(0.02)
    inf = revisar_trayectoria(R, ts, Q, QD, qa, c, 5.0, exige_go=True)
    assert next(x for x in inf["chequeos"] if x["id"] == "singularidad")["estado"] == "rechazado"
