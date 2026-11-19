"""Pruebas de la ruta RWS: cliente, módulo RAPID simulado, BackendRWS dentro de Consola y nodo ROS (helpers).

Se corren con:  python3 tests/test_rws.py   (o python3 -m pytest tests -q)
Usan tests/mock_rws.py: un IRC5 falso con digest, mastership, PERS y un VevasServidor simulado.
"""
import os
import re
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
sys.path.insert(0, RAIZ)
sys.path.insert(0, AQUI)

from mock_rws import MockIRC5  # noqa: E402
import test_revision as tr  # noqa: E402
from vevas.backend_rws import BackendRWS  # noqa: E402
from vevas.consola import Consola  # noqa: E402
from vevas.registro import Registro  # noqa: E402
from vevas.rws_cliente import Cliente, Falla, huella_modulo, literal, rechazo_local, subdividir  # noqa: E402

MOD = os.path.join(RAIZ, "rapid", "VEVAS_RWS.mod")
HOME = [0, -20, 30, 0, 40, 0]
CHECK = tr.CHECK
FRASE = tr.FRASE


# ---------------------------------------------------------------- utilidades
class Escenario:
    def __init__(self, listo=True, **kw):
        self.mock = MockIRC5(**kw)
        self.puerto = self.mock.iniciar()
        self.lock = os.path.join(tempfile.mkdtemp(), "escritor.lock")
        self.cli = self.nuevo_cliente()
        if listo:
            self.preparar()

    def nuevo_cliente(self, usuario="Default User", clave="robotics"):
        return Cliente("127.0.0.1", usuario, clave, self.puerto, timeout=3.0, ruta_candado=self.lock)

    def preparar(self):
        self.cli.instalar_modulo(MOD, eco=lambda *_: None)
        self.cli.preparar_y_arrancar(eco=lambda *_: None)
        assert self.cli.esperar_servidor(10, pausa_inicial=0.1), "VevasServidor no quedó activo"

    def cerrar(self):
        self.mock.detener()


def con_escenario(f):
    def envoltura():
        e = Escenario()
        try:
            f(e)
        finally:
            e.cerrar()
    envoltura.__name__ = f.__name__
    return envoltura


def cfg_rws(e, **extra):
    c = tr.cfg()
    c["rws"].update({"ip": "127.0.0.1", "puerto": e.puerto, "hz_lectura": 20, "timeout_orden_s": 30})
    c["seguridad"]["watchdog_estado_s"] = 1.0
    c["entorno"].update({"sin_objetos_externos_confirmado": True, "sin_herramienta_confirmada": True})
    c.update(extra)
    return c


def consola_rws(e, go=None):
    c = cfg_rws(e)
    b = BackendRWS(c, lambda t: None, cliente=e.cli)
    con = Consola(c, b, Registro(tempfile.mkdtemp(), "test_rws"), permitir_real=True, go=go or tr.go_valido())
    b.iniciar()
    con.iniciar()
    # el navegador manda un pulso periódico; sin él Consola aborta cualquier movimiento (watchdog de la interfaz)
    def pulsos():
        while con.activo:
            con.pulso()
            time.sleep(0.2)
    threading.Thread(target=pulsos, daemon=True).start()
    t0 = time.monotonic()
    while b.leer()[0] is None and time.monotonic() - t0 < 3:
        time.sleep(0.05)
    time.sleep(0.15)
    return con, b


def cerrar_consola(con, b):
    con.detener()
    b.detener()
    time.sleep(0.1)
    con.reg.cerrar()


def esperar(cond, t=30.0, paso=0.05):
    t0 = time.monotonic()
    while time.monotonic() - t0 < t:
        if cond():
            return True
        time.sleep(paso)
    return False


# ---------------------------------------------------------------- helpers puros
def test_subdividir_respeta_paso_e_incluye_el_final():
    q0 = [0, -20, 30, 0, 40, 0]
    Q = [[a + k * 0.5 * s for a, s in zip(q0, (1, 0, -1, 0, 2, 0))] for k in range(1, 61)]  # J1 +30°
    wps = subdividir(q0, Q, 10.0)
    assert wps[-1] == Q[-1]
    previo = q0
    for w in wps:
        assert max(abs(a - b) for a, b in zip(w, previo)) <= 10.0 + 1e-9
        previo = w
    assert len(wps) >= 3
    # un salto de 25° entre muestras se interpola
    wps = subdividir([0] * 6, [[25, 0, 0, 0, 0, 0]], 10.0)
    assert len(wps) == 3 and wps[-1] == [25, 0, 0, 0, 0, 0]
    assert max(abs(wps[i][0] - (wps[i - 1][0] if i else 0)) for i in range(3)) <= 10.0 + 1e-9
    try:
        subdividir([0] * 6, [[1] * 6], 45.0)
        assert False
    except Falla:
        pass
    # trayectoria sin movimiento: igual se envía el punto final
    assert subdividir(HOME, [HOME], 10.0) == [[float(v) for v in HOME]]


def test_rechazo_local_limites_y_paso():
    assert rechazo_local(HOME, HOME) is None
    assert rechazo_local([164, 0, 0, 0, 0, 0], HOME)[0] == -2  # dentro del límite pero sin margen de 2°
    assert rechazo_local([0, 0, 0, 0, 0, 0], [50, 0, 0, 0, 0, 0])[0] == -3
    assert rechazo_local([0, 0, 71, 0, 0, 0], [0] * 6)[0] == -2
    assert rechazo_local([float("nan")] * 6, HOME)[0] == -1
    assert literal([1.23456, 2, 3, 4, 5, 6]) == "[[1.235,2,3,4,5,6],[9000000000,9000000000,9000000000,9000000000,9000000000,9000000000]]"


def test_el_mod_define_el_servidor_y_la_huella_ignora_los_valores_pers():
    t = open(MOD, encoding="utf-8").read()
    assert t.startswith("MODULE VEVAS_RWS") and "PROC VevasServidor()" in t and not re.search(r"^\s*PROC\s+main\b", t, flags=re.M)
    assert "CONST bool permitirAuto := TRUE;" in t and "CONST speeddata vPc := [20,5,20,5];" in t
    t2 = t.replace("PERS num vvPcSeq := 0;", "PERS num vvPcSeq := 7;").replace("\n", "\r\n")
    assert huella_modulo(t) == huella_modulo(t2)
    assert huella_modulo(t) != huella_modulo(t.replace("maxPasoDeg := 45", "maxPasoDeg := 90"))


# ---------------------------------------------------------------- cliente: instalar y arrancar
def test_instalar_y_arrancar_en_auto():
    e = Escenario(listo=False)
    try:
        c, m = e.cli, e.mock
        assert not c.leer_estado()["modulo_vevas"]
        assert c.instalar_modulo(MOD, eco=lambda *_: None) == "cargado"
        assert "VEVAS_RWS" in m.modulos and huella_modulo(m.textos["VEVAS_RWS"]) == huella_modulo(open(MOD).read())
        assert not m.mastership  # se liberó
        assert c.instalar_modulo(MOD, eco=lambda *_: None) == "ya_instalado"
        c.preparar_y_arrancar(eco=lambda *_: None)
        est = c.leer_estado()
        assert est["exec"] == "running" and est["ctrlstate"] == "motoron" and est["pp_modulo"] == "VEVAS_RWS" and est["pp_rutina"] == "VevasServidor"
        assert c.esperar_servidor(10, pausa_inicial=0.1)
        s = c.estado_servidor()
        assert s["vvRbActivo"] == 1 and s["vvRbCodigo"] == 0 and s["vvPcSeq"] == s["vvRbSeq"]
        assert c.problemas_para_enviar() == []
        # ya corriendo: no se vuelve a arrancar
        c.preparar_y_arrancar(eco=lambda *_: None)
        # parar: RAPID detenido y motores OFF; pp_main devuelve el puntero
        r = c.parar()
        assert r["ok"] and r["ctrlstate"] == "motoroff" and r["exec"] == "stopped"
        c.pp_main()
        assert m.pp == ("Module1", "main")
    finally:
        e.cerrar()


def test_instalar_prueba_la_segunda_ruta_de_loadmod_y_actualiza_con_set_module_text():
    e = Escenario(listo=False, acepta_home_var=False)  # el IRC5 falso rechaza «$HOME/...»: se prueba «HOME:/...»
    try:
        assert e.cli.instalar_modulo(MOD, eco=lambda *_: None) == "cargado"
        assert "VEVAS_RWS" in e.mock.modulos
        # texto distinto en el controlador → set-module-text
        e.mock.textos["VEVAS_RWS"] = open(MOD).read().replace("maxPasoDeg := 45", "maxPasoDeg := 90")
        e.mock.log.clear()
        assert e.cli.instalar_modulo(MOD, eco=lambda *_: None) == "actualizado"
        assert any("set-module-text" in l for l in e.mock.log)
        assert "maxPasoDeg := 45" in e.mock.textos["VEVAS_RWS"]
        assert not e.mock.mastership
        e.mock.log.clear()
        assert e.cli.instalar_modulo(MOD, eco=lambda *_: None) == "ya_instalado"
        assert not any(l.startswith("POST") or l.startswith("PUT") for l in e.mock.log)
    finally:
        e.cerrar()


def test_rechazo_en_manual_reducido():
    e = Escenario(listo=False, opmode="MANR")
    try:
        for f in (lambda: e.cli.instalar_modulo(MOD, eco=lambda *_: None), lambda: e.cli.preparar_y_arrancar(eco=lambda *_: None)):
            try:
                f()
                assert False, "debía negarse en MANR"
            except Falla as ex:
                assert "AUTO" in str(ex) or "MANR" in str(ex)
        assert "VEVAS_RWS" not in e.mock.modulos and e.mock.exec == "stopped"
        e.mock.opmode = "MANF"
        try:
            e.cli.preparar_y_arrancar(eco=lambda *_: None)
            assert False
        except Falla:
            pass
    finally:
        e.cerrar()


def test_no_arranca_con_la_cadena_de_seguridad_abierta_y_apaga_motores_si_falla():
    e = Escenario(listo=False)
    try:
        e.cli.instalar_modulo(MOD, eco=lambda *_: None)
        e.mock.seguridad_abierta = True
        try:
            e.cli.preparar_y_arrancar(eco=lambda *_: None)
            assert False
        except Falla:
            pass
        assert e.mock.ctrlstate == "motoroff" and e.mock.exec == "stopped"
        e.mock.seguridad_abierta = False
        e.mock.modulos.remove("VEVAS_RWS")  # sin módulo no arranca
        try:
            e.cli.preparar_y_arrancar(eco=lambda *_: None)
            assert False
        except Falla as ex:
            assert "no está cargado" in str(ex)
    finally:
        e.cerrar()


def test_sin_confirmacion_en_el_pendant_el_servidor_no_queda_activo():
    e = Escenario(listo=False, confirma_pendant=False)
    try:
        e.cli.instalar_modulo(MOD, eco=lambda *_: None)
        e.cli.preparar_y_arrancar(eco=lambda *_: None)
        assert not e.cli.esperar_servidor(3, pausa_inicial=0.1)
        assert e.cli.problemas_para_enviar()
        ok, cod, msg = e.cli.enviar_objetivo([5, -20, 30, 0, 40, 0])
        assert not ok and cod is None and "No listo" in msg
    finally:
        e.cerrar()


def test_digest_y_cookie_con_el_irc5_falso():
    e = Escenario(listo=False, auth=("Default User", "robotics"))
    try:
        assert e.cli.leer_estado()["opmode"] == "AUTO"
        mal = e.nuevo_cliente(clave="otra")
        try:
            mal.leer_estado()
            assert False
        except Falla as ex:
            assert "401" in str(ex)
        assert e.cli.instalar_modulo(MOD, eco=lambda *_: None) == "cargado"  # POST/PUT también pasan por el digest
    finally:
        e.cerrar()


def test_lectura_de_juntas_cae_a_la_ruta_de_la_referencia():
    e = Escenario(listo=False, mechunits=False)
    try:
        q = e.cli.joints_deg()
        assert max(abs(a - b) for a, b in zip(q, HOME)) < 1e-3
        assert e.cli._ruta_q == 1
    finally:
        e.cerrar()


# ---------------------------------------------------------------- cliente: órdenes
@con_escenario
def test_enviar_objetivo_llega_y_actualiza_el_robot(e):
    visto = []
    ok, cod, msg = e.cli.enviar_objetivo([10, -20, 30, 0, 40, 0], al_aceptar=visto.append)
    assert ok and cod == 2 and "llegada" in msg, msg
    assert visto and visto[0] in (1, 2)
    assert abs(e.mock.q[0] - 10) < 1e-6 and e.mock.ordenes == [[10, -20, 30, 0, 40, 0]]
    s = e.cli.estado_servidor()
    assert s["vvRbSeq"] == s["vvPcSeq"] == 1 and s["vvRbCodigo"] == 2
    # una segunda orden incrementa la secuencia; el valor con 6 cifras (≥100°) también se comprueba bien
    ok, cod, msg = e.cli.enviar_objetivo([40, -20, 30, 0, 40, 0])
    assert ok, msg
    ok, cod, msg = e.cli.enviar_objetivo([70, -20, 30, 0, 40, 0])
    ok, cod, msg = e.cli.enviar_objetivo([100.123, -20, 30, 0, 40, 0])
    assert ok, msg
    assert e.cli.estado_servidor()["vvPcSeq"] == 4


@con_escenario
def test_enviar_objetivo_rechaza_limites_y_pasos_grandes_sin_mover(e):
    ok, cod, msg = e.cli.enviar_objetivo([170, -20, 30, 0, 40, 0])
    assert not ok and cod == -2 and "rechazado por la pc" in msg.lower()
    ok, cod, msg = e.cli.enviar_objetivo([60, -20, 30, 0, 40, 0])
    assert not ok and cod == -3
    assert e.mock.ordenes == [] and e.cli.estado_servidor()["vvPcSeq"] == 0
    # el robot repite la validación aunque la PC se la salte (RAPID: -2 límites, -3 paso, -4 modo)
    def forzar(q):
        e.cli._escribir("vvPcTarget", literal(q))
        seq = e.cli.estado_servidor()["vvPcSeq"] + 1
        e.cli._escribir("vvPcSeq", str(seq))
        assert esperar(lambda: e.cli.estado_servidor()["vvRbSeq"] == seq, 5)
        return e.cli.estado_servidor()["vvRbCodigo"]
    assert forzar([170, -20, 30, 0, 40, 0]) == -2
    assert forzar([60, -20, 30, 0, 40, 0]) == -3
    e.mock.opmode, e.mock.rmmp = "MANF", "GRANTED"  # en manual hace falta RMMP para escribir; MANF no está permitido
    assert forzar([1, -20, 30, 0, 40, 0]) == -4
    e.mock.opmode = "AUTO"
    assert e.mock.ordenes == [] and e.cli.problemas_para_enviar() == []
    ok, cod, msg = e.cli.enviar_objetivo([5, -20, 30, 0, 40, 0])  # sigue funcionando después de los rechazos
    assert ok, msg


@con_escenario
def test_no_se_envia_si_hay_una_orden_en_curso_o_el_servidor_terminó(e):
    e.mock.congelado = True  # RAPID sin atender: la orden «en curso» (vvPcSeq ≠ vvRbSeq) escrita por otro sigue pendiente
    e.mock.pers["vvPcSeq"] += 1
    ok, cod, msg = e.cli.enviar_objetivo([5, -20, 30, 0, 40, 0])
    assert not ok and "en curso" in msg
    e.mock.pers["vvRbSeq"] = e.mock.pers["vvPcSeq"]
    e.mock.congelado = False
    e.cli.terminar_servidor()
    assert esperar(lambda: e.mock.exec == "stopped", 3)
    ok, cod, msg = e.cli.enviar_objetivo([5, -20, 30, 0, 40, 0])
    assert not ok and "No listo" in msg


@con_escenario
def test_dos_escritores_no_pueden_mover_a_la_vez(e):
    otro = e.nuevo_cliente()
    with e.cli.escritor():
        ok, cod, msg = otro.enviar_objetivo([5, -20, 30, 0, 40, 0])
        assert not ok and cod is None and "Otra herramienta" in msg
        # el mismo proceso/cliente sí puede anidar (la trayectoria mantiene el candado entre waypoints)
        ok, cod, msg = e.cli.enviar_objetivo([5, -20, 30, 0, 40, 0])
        assert ok, msg
    ok, cod, msg = otro.enviar_objetivo([10, -20, 30, 0, 40, 0])
    assert ok, msg


def test_manual_reducido_pide_rmmp_y_envia():
    e = Escenario(listo=False, opmode="MANR")
    try:
        m = e.mock
        # el operador deja todo listo en el pendant: módulo cargado, motores, PP en VevasServidor y Start
        m.modulos.append("VEVAS_RWS")
        m.textos["VEVAS_RWS"] = open(MOD).read()
        m.ctrlstate, m.pp, m.exec = "motoron", ("VEVAS_RWS", "VevasServidor"), "running"
        assert e.cli.esperar_servidor(10, pausa_inicial=0.1)
        ok, cod, msg = e.cli.enviar_objetivo([5, -20, 30, 0, 40, 0], eco=lambda *_: None)
        assert ok, msg
        assert m.rmmp == "GRANTED" and e.cli.rmmp_concedido
        # sin RMMP la escritura da 403 y el cliente lo explica
        m.rmmp, e.cli.rmmp_concedido = "NONE", False
        try:
            e.cli._escribir("vvPcSeq", "9")
            assert False
        except Falla as ex:
            assert "403" in str(ex) and "RMMP" in str(ex)
        m.rmmp_auto = False
        try:
            e.cli.solicitar_rmmp(espera=0.6, eco=lambda *_: None)
            assert False
        except Falla as ex:
            assert "RMMP" in str(ex)
    finally:
        e.cerrar()


# ---------------------------------------------------------------- BackendRWS dentro de Consola
def test_consola_ejecuta_una_trayectoria_multiwaypoint_sin_abortar_por_seguimiento():
    e = Escenario(vel_deg_s=6.0)  # robot MÁS LENTO que el plan LSPB: por tiempo habría abortado (> 3° de error)
    con, b = consola_rws(e)
    try:
        assert b.tipo == "rws" and b.seguimiento_por_llegada
        assert con.modo()["exige_go"] and con.armar(CHECK, FRASE)[0]
        p = con.calcular_junta(1, 25.0, True)
        assert p["ejecutable"], p["problemas"]
        ok, msg = con.ejecutar()
        assert ok, msg
        assert con.ejec is not None  # no bloquea: sigue ejecutándose
        assert esperar(lambda: con.ejec is None, 40), "no terminó"
        r = con.ultimo_resumen
        assert r["estado"] == "completado", r
        assert r["error_final_max_deg"] < 0.1, r
        assert abs(e.mock.q[0] - 25.0) < 0.01
        assert len(e.mock.ordenes) >= 3 and all(
            max(abs(x - y) for x, y in zip(a, c)) <= 10.0 + 1e-6 for a, c in zip([HOME] + e.mock.ordenes, e.mock.ordenes))
        assert e.mock.ordenes[-1][0] == 25.0
        assert con.armado  # terminó bien: sigue armado
        assert b.resultado_trayectoria()[0] is True and "llegada" in b.ultimo_resultado
        assert e.mock.rechazos == []
    finally:
        cerrar_consola(con, b)
        e.cerrar()


def test_parar_durante_la_ejecucion_corta_los_waypoints_siguientes():
    e = Escenario(vel_deg_s=15.0)
    con, b = consola_rws(e)
    try:
        con.armar(CHECK, FRASE)
        p = con.calcular_junta(1, 60.0, True)
        assert p["ejecutable"], p["problemas"]
        ok, msg = con.ejecutar()
        assert ok, msg
        assert esperar(lambda: len(e.mock.ordenes) >= 1, 10)
        con.parar("prueba")
        assert con.ejec is None and not con.armado
        assert "prueba" in con.motivo_desarmado
        assert b.resultado_trayectoria()[0] is False
        # el MoveAbsJ en curso termina solo; luego el hilo no manda nada más
        assert esperar(lambda: not b.en_movimiento(), 15)
        n = len(e.mock.ordenes)
        time.sleep(1.0)
        assert len(e.mock.ordenes) == n < 6
        assert e.mock.q[0] < 59.9
        assert con.ultimo_resumen["estado"] == "abortado"
    finally:
        cerrar_consola(con, b)
        e.cerrar()


def test_el_backend_rechaza_una_trayectoria_nueva_mientras_la_anterior_corre():
    e = Escenario(vel_deg_s=10.0)
    con, b = consola_rws(e)
    try:
        con.armar(CHECK, FRASE)
        con.calcular_junta(1, 40.0, True)
        ok, _ = con.ejecutar()
        assert ok
        q, _ = b.leer()
        ok, msg = b.ejecutar_trayectoria([0.0], np.radians([[20, -20, 30, 0, 40, 0]]))
        assert not ok and "sigue ejecutándose" in msg
        con.parar("fin de la prueba")
        assert esperar(lambda: not b.en_movimiento(), 20)
    finally:
        cerrar_consola(con, b)
        e.cerrar()


def test_el_robot_rechaza_el_primer_waypoint_y_el_envio_falla_de_inmediato():
    e = Escenario()
    con, b = consola_rws(e)
    try:
        con.armar(CHECK, FRASE)
        con.calcular_junta(1, 20.0, True)
        e.mock.opmode = "MANF"  # el robot dice -4 / el cliente dice «no listo»
        ok, msg = con.ejecutar()
        assert not ok and ("manual 100" in msg or "-4" in msg or "modo" in msg), msg
        assert con.ejec is None and e.mock.ordenes == []
    finally:
        cerrar_consola(con, b)
        e.cerrar()


def test_jog_rechazado_en_rws():
    e = Escenario()
    con, b = consola_rws(e)
    try:
        con.armar(CHECK, FRASE)
        ok, msg = con.orden_jog("mover", "junta", 1, 1)
        assert not ok and "RWS" in msg and con.jog is None
        assert con.orden_jog("soltar")[0]  # soltar sigue siendo inofensivo
        assert e.mock.ordenes == []
    finally:
        cerrar_consola(con, b)
        e.cerrar()


def test_solo_lectura_no_ejecuta_y_lee_la_postura():
    e = Escenario()
    c = cfg_rws(e)
    b = BackendRWS(c, lambda t: None, solo_lectura=True, cliente=e.cli)
    b.iniciar()
    try:
        assert esperar(lambda: b.leer()[0] is not None, 3)
        q, t = b.leer()
        assert np.allclose(np.degrees(q), HOME, atol=1e-3)
        i = b.info()
        assert i["tipo"] == "rws" and i["real"] is True and i["solo_lectura"] and "SOLO LECTURA" in i["texto"] and "RWS" in i["hardware"]
        assert b.ejecutar_trayectoria([0.1], np.radians([HOME]))[0] is False
    finally:
        b.detener()
        e.cerrar()


def test_estado_viejo_se_nota_en_leer_y_aborta_la_ejecucion():
    e = Escenario(vel_deg_s=10.0)
    con, b = consola_rws(e)
    try:
        q, t = b.leer()
        assert time.monotonic() - t < 0.5
        con.armar(CHECK, FRASE)
        con.calcular_junta(1, 40.0, True)
        ok, msg = con.ejecutar()
        assert ok, msg
        e.mock.detener()  # el IRC5 «desaparece»: la lectura falla y la marca de tiempo envejece
        assert esperar(lambda: con.ejec is None, 8), "Consola no abortó con estado viejo"
        assert not con.armado
        assert "Sin datos" in con.motivo_desarmado or "no completó" in con.motivo_desarmado, con.motivo_desarmado
        assert esperar(lambda: time.monotonic() - b.leer()[1] > 0.5, 5)
        assert b.ultimo_error
    finally:
        cerrar_consola(con, b)
        e.cerrar()


def test_tiempo_limite_global_si_el_robot_nunca_informa():
    e = Escenario(vel_deg_s=0.0001)  # el robot «se traba»: nunca llega
    con, b = consola_rws(e)
    try:
        con.armar(CHECK, FRASE)
        con.calcular_junta(1, 5.0, True)
        ok, msg = con.ejecutar()
        assert ok, msg
        con.ejec["limite_s"] = 1.0  # el límite real es tf*6+30: se acorta para la prueba
        assert esperar(lambda: con.ejec is None, 6)
        assert "Tiempo límite" in con.motivo_desarmado, con.motivo_desarmado
    finally:
        cerrar_consola(con, b)
        e.cerrar()


def test_consola_vevas_modo_rws_exige_real_o_solo_lectura_y_arranca_en_solo_lectura():
    e = Escenario(listo=False)
    try:
        base = [sys.executable, os.path.join(RAIZ, "consola_vevas.py"), "--modo", "rws", "--ip", "127.0.0.1",
                "--rws-puerto", str(e.puerto), "--sesiones", tempfile.mkdtemp(), "--puerto", "0"]
        r = subprocess.run(base + ["--preparar-rapid"], capture_output=True, text=True, timeout=20)
        assert r.returncode == 2 and "--real" in r.stderr
        r = subprocess.run([sys.executable, os.path.join(RAIZ, "consola_vevas.py"), "--modo", "sim", "--ip", "1.2.3.4"],
                           capture_output=True, text=True, timeout=20)
        assert r.returncode == 2 and "rws" in r.stderr
        # arranca (solo lectura), imprime la URL y se cierra con SIGINT
        p = subprocess.Popen(base + ["--solo-lectura"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        salida = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15:
            l = p.stdout.readline()
            salida.append(l)
            if "Consola VEVAS lista" in l:
                break
        p.send_signal(2)
        p.wait(timeout=15)
        assert any("Consola VEVAS lista" in l for l in salida), "".join(salida)
    finally:
        e.cerrar()


def test_consola_vevas_preparar_rapid_instala_y_arranca():
    e = Escenario(listo=False)
    try:
        d = tempfile.mkdtemp()
        go = os.path.join(d, "go.json")
        import json
        json.dump(tr.go_valido(), open(go, "w"))
        base = [sys.executable, os.path.join(RAIZ, "consola_vevas.py"), "--modo", "rws", "--ip", "127.0.0.1",
                "--rws-puerto", str(e.puerto), "--sesiones", d, "--puerto", "0", "--real", "--go", go, "--preparar-rapid"]
        p = subprocess.Popen(base, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        salida = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < 30:
            l = p.stdout.readline()
            salida.append(l)
            if "Consola VEVAS lista" in l or not l:
                break
        p.send_signal(2)
        p.wait(timeout=15)
        txt = "".join(salida)
        assert "Consola VEVAS lista" in txt and "RAPID listo" in txt, txt
        assert e.mock.exec == "running" and e.mock.pers["vvRbActivo"] == 1 and "VEVAS_RWS" in e.mock.modulos
        # en MANR: error claro y salida con código 1
        e.mock.opmode = "MANR"
        e.mock.exec = "stopped"
        r = subprocess.run(base, capture_output=True, text=True, timeout=30)
        assert r.returncode == 1 and "ERROR al preparar RAPID" in r.stdout and "AUTO" in r.stdout, r.stdout + r.stderr
    finally:
        e.cerrar()


# ---------------------------------------------------------------- nodo ROS (solo los ayudantes, sin rclpy)
def test_nodo_ros_importa_sin_ros_y_sus_ayudantes_validan():
    sys.path.insert(0, os.path.join(RAIZ, "herramientas"))
    import nodo_rws_ros as n
    assert "rclpy" not in sys.modules
    nombres = ["joint_3", "joint_1", "joint_2", "joint_6", "joint_5", "joint_4"]
    pos = [np.radians(v) for v in (30, 10, -20, 0, 40, 0)]
    q = n.ordenar_juntas(nombres, pos)
    assert np.allclose(q, np.radians([10, -20, 30, 0, 40, 0]))
    try:
        n.ordenar_juntas(nombres[:5], pos[:5])
        assert False
    except ValueError:
        pass
    pts = [np.radians([k * 4.0, -20, 30, 0, 40, 0]) for k in range(1, 11)]  # J1 hasta 40°
    wps = n.waypoints_desde_puntos(pts, HOME, 10.0)
    assert abs(wps[-1][0] - 40.0) < 1e-9 and max(abs(a - b) for a, b in zip(wps[0], HOME)) <= 10.0
    assert n.validar_waypoints(wps, HOME, 40.0) is None
    assert "fuera de" in n.validar_waypoints([[170, 0, 0, 0, 0, 0]], HOME, 40.0)
    assert "paso" in n.validar_waypoints([[50, -20, 30, 0, 40, 0]], HOME, 40.0)
    assert n.validar_waypoints([], HOME, 40.0)


@con_escenario
def test_nodo_ros_ejecutor_ejecuta_cancela_y_rechaza(e):
    sys.path.insert(0, os.path.join(RAIZ, "herramientas"))
    import nodo_rws_ros as n
    ej = n.Ejecutor(e.cli, paso_deg=10.0, timeout_orden_s=30)
    nombres = list(n.NOMBRES)
    pts = [np.radians([k * 2.0, -20, 30, 0, 40, 0]) for k in range(1, 16)]  # J1 0→30°
    vistos = []
    estado, cod, txt = ej.ejecutar(nombres, pts, progreso=lambda k, tot, w: vistos.append(k))
    assert estado == "ok" and cod == n.SUCCESSFUL, txt
    assert abs(e.mock.q[0] - 30.0) < 1e-3 and vistos == list(range(1, len(vistos) + 1)) and len(vistos) >= 3
    # nombres incompletos / fuera de límites / paso inalcanzable: se rechazan sin mover
    antes = len(e.mock.ordenes)
    assert ej.ejecutar(nombres[:5], [p[:5] for p in pts])[:2] == ("falla", n.INVALID_JOINTS)
    assert ej.ejecutar(nombres, [np.radians([170, 0, 0, 0, 0, 0])])[:2] == ("falla", n.INVALID_GOAL)
    assert len(e.mock.ordenes) == antes
    # cancelar después del primer waypoint: no se manda ninguno más
    pts2 = [np.radians([30 - k * 2.0, -20, 30, 0, 40, 0]) for k in range(1, 16)]
    cuenta = {"n": 0}
    estado, cod, txt = ej.ejecutar(nombres, pts2, cancelado=lambda: cuenta["n"] >= 1,
                                   progreso=lambda k, tot, w: cuenta.update(n=k))
    assert estado == "cancelado" and cuenta["n"] == 1 and e.mock.q[0] > 15
    # otro escritor con el candado → falla con mensaje claro
    otro = n.Ejecutor(e.nuevo_cliente(), 10.0, 30)
    with e.cli.escritor():
        estado, cod, txt = otro.ejecutar(nombres, [np.radians([20, -20, 30, 0, 40, 0])])
    assert estado == "falla" and "Otra herramienta" in txt or "no se" in txt.lower() or estado == "falla"


if __name__ == "__main__":
    fallas = 0
    n_ok = 0
    for nombre, f in sorted(globals().items()):
        if nombre.startswith("test_") and callable(f):
            try:
                f()
                n_ok += 1
                print("ok   ", nombre, flush=True)
            except Exception as ex:  # noqa: BLE001
                fallas += 1
                import traceback
                traceback.print_exc()
                print("FALLA", nombre, ex, flush=True)
    print(f"\n{n_ok} ok, {fallas} con falla")
    sys.exit(1 if fallas else 0)


def test_programa_con_error_diagnostico_y_recarga():
    """1ra prueba real (19/11): VEVAS_RWS cargado con un error RAPID → set-pp-routine 400.
    Se lee el error del registro de eventos y la recarga desde cero lo corrige."""
    import io, contextlib, mock_rws
    from vevas.rws_cliente import Cliente, Falla
    sys.path.insert(0, os.path.join(RAIZ, "herramientas"))
    import rws_irc5
    m = mock_rws.arrancar(8297)
    try:
        m.modulos.append("VEVAS_RWS"); m.textos["VEVAS_RWS"] = "MODULE VEVAS_RWS\n ERROR_RAPID\nENDMODULE\n"
        c = Cliente("127.0.0.1", "Default User", "robotics", puerto=8297)
        try:
            c.preparar_y_arrancar(eco=lambda *_: None)
            assert False, "debía fallar"
        except Falla as ex:
            assert "set-pp-routine" in str(ex)
        b = io.StringIO()
        with contextlib.redirect_stdout(b):
            rws_irc5.main(["--ip", "127.0.0.1", "--puerto", "8297", "diagnostico"])
        assert "line 47" in b.getvalue(), b.getvalue()
        c.parar()
        r = c.instalar_modulo(os.path.join(RAIZ, "rapid", "VEVAS_RWS.mod"), forzar=True, eco=lambda *_: None)
        assert r in ("cargado", "actualizado")
        assert "ERROR_RAPID" not in m.textos["VEVAS_RWS"]
        c.preparar_y_arrancar(eco=lambda *_: None)
        assert m.pp == ("VEVAS_RWS", "VevasServidor")
    finally:
        m.detener() if hasattr(m, "detener") else None


def test_modulo_entra_en_el_limite_de_rws():
    """RWS corta set-module-text en 5000 caracteres (visto en el IRC5 real el 19/11)."""
    with open(os.path.join(RAIZ, "rapid", "VEVAS_RWS.mod"), encoding="utf-8") as f:
        t = f.read()
    assert len(t) <= 4800, len(t)
    assert t.rstrip().endswith("ENDMODULE")
