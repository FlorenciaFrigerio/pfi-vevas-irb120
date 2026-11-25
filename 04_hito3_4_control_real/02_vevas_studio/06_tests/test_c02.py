"""C02: módulo RAPID con MoveAbsJ \\T, evidencia medida y habilitación del envío por RWS."""
import io
import contextlib
import json
import os
import re
import sys
import tempfile

import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
sys.path.insert(0, os.path.join(RAIZ, "06_tests"))
sys.path.insert(0, os.path.join(RAIZ, "02_herramientas"))

from mock_rws import MockIRC5  # noqa: E402
from vevas import c02  # noqa: E402
from vevas.backend_rws import BackendRWS  # noqa: E402
from vevas.rws_cliente import huella_modulo, limites_c02, tiempo_movimiento, tiempo_zona  # noqa: E402
from vevas.seguridad import cargar_config  # noqa: E402
import verificar_c02  # noqa: E402

MOD = os.path.join(RAIZ, "05_rapid", "VEVAS_RWS.mod")
TEXTO = open(MOD, encoding="utf-8").read()


def test_modulo_rapid_c02():
    assert len(TEXTO) < 4800, "RWS corta el módulo en 5000 caracteres"
    assert TEXTO.rstrip().endswith("ENDMODULE")
    assert "MoveAbsJ objetivo,vPc\\T:=t,fine,tool0;" in TEXTO
    assert re.search(r"MoveAbsJ objetivo,vPc,fine", TEXTO) is None, "no debe quedar un MoveAbsJ sin \\T"
    vlim, k = limites_c02(TEXTO)
    assert vlim == (12.5, 12.5, 12.5, 16.0, 16.0, 21.0)   # 5 % de 250,250,250,320,320,420 °/s
    assert k == 2.5
    for nombre in ("pcTarget", "ServidorPC", "rbSeq"):   # nombres de LabJointRws: no deben repetirse
        assert re.search(rf"\b{nombre}\b", TEXTO) is None
    # movimiento continuo: intermedios con zona chica y el mismo \T; piso más bajo solo en zona
    assert "MoveAbsJ objetivo,vPc\\T:=t,z1,tool0;" in TEXTO
    assert len(re.findall(r"^\s*MoveAbsJ ", TEXTO, flags=re.M)) == 2
    assert tiempo_zona(TEXTO) == 0.2
    assert re.search(r"^\s*PERS\s+num\s+vvPcZona\s*:=\s*0;", TEXTO, flags=re.M)
    # Δq contra el objetivo anterior mientras hay zona; CJointT solo con el robot parado (tras fine)
    assert "Validar(objetivo.robax, previo.robax)" in TEXTO and "Tiempo(objetivo.robax, previo.robax, t)" in TEXTO
    assert re.search(r"IF enZona = FALSE THEN\s+previo := CJointT\(\);", TEXTO)
    # nombres globales: solo VevasServidor y las PERS vv*; el resto LOCAL
    globales = re.findall(r"^ {2}(?:PROC|FUNC\s+\w+|CONST\s+\w+|VAR\s+\w+)\s+(\w+)", TEXTO, flags=re.M)
    assert globales == ["VevasServidor"], globales


def test_tiempo_de_zona_como_el_rapid():
    v, k = limites_c02(TEXTO)
    tz = tiempo_zona(TEXTO)
    assert tiempo_movimiento([0] * 6, [0] * 6, v, k, tz) == 0.2
    assert abs(tiempo_movimiento([0, 0, 0, 0, 0, 2.1], [0] * 6, v, k, tz) - 0.25) < 1e-12
    assert abs(tiempo_movimiento([10, 0, 0, 0, 0, 0], [0] * 6, v, k, tz) - 2.0) < 1e-12   # el tope no cambia
    assert tiempo_zona(TEXTO.replace("PERS num vvPcZona := 0;", "")) is None             # módulo viejo


def test_tiempo_como_el_rapid():
    v, k = limites_c02(TEXTO)
    assert tiempo_movimiento([0] * 6, [0] * 6, v, k) == 0.5
    assert abs(tiempo_movimiento([10, 0, 0, 0, 0, 0], [0] * 6, v, k) - 2.0) < 1e-12
    assert abs(tiempo_movimiento([0, 0, 0, 10, 0, 21], [0] * 6, v, k) - 2.5) < 1e-12   # J6: 2.5·21/21


def test_velocidades_max_por_ventana():
    ts = np.arange(0, 2.0, 0.01)
    ms = [(t, [5.0 * t, 0, 0, 0, 0, -2.0 * t]) for t in ts]
    v = c02.velocidades_max(ms)
    assert abs(v[0] - 5.0) < 1e-6 and abs(v[5] - 2.0) < 1e-6 and v[1] == 0


def _evidencia(tmp, **cambios):
    ev = {"version": c02.VERSION_EVIDENCIA, "ok": True, "fecha_utc": "2026-11-24T10:00:00+00:00", "modos": ["AUTO"],
          "huella_modulo": huella_modulo(TEXTO), "vlim_deg_s": list(limites_c02(TEXTO)[0]), "k_pico": 2.5,
          "movimientos": [{"junta": j, "ok": True} for j in range(1, 7)], "streaming": {"junta": 6, "ok": True}}
    ev.update(cambios)
    ruta = os.path.join(tmp, "ev.json")
    with open(ruta, "w") as f:
        json.dump(ev, f)
    return ruta


def test_evaluar_evidencia():
    tmp = tempfile.mkdtemp()
    assert not c02.evaluar(os.path.join(tmp, "no.json"), TEXTO)[0]
    assert c02.evaluar(_evidencia(tmp), TEXTO)[0]
    assert c02.evaluar(_evidencia(tmp), TEXTO, opmode="AUTO", texto_modulo_controlador=TEXTO)[0]
    ok, motivo, _ = c02.evaluar(_evidencia(tmp), TEXTO, opmode="MANR")
    assert not ok and "modo" in motivo
    assert not c02.evaluar(_evidencia(tmp), TEXTO + "\n! cambio")[0]
    assert not c02.evaluar(_evidencia(tmp), TEXTO, texto_modulo_controlador=TEXTO.replace("2.5;", "1;"))[0]
    assert not c02.evaluar(_evidencia(tmp, ok=False), TEXTO)[0]
    assert not c02.evaluar(_evidencia(tmp, movimientos=[{"junta": 1, "ok": True}]), TEXTO)[0]
    # el módulo mueve en continuo: sin la prueba de streaming aprobada no habilita
    ok, motivo, _ = c02.evaluar(_evidencia(tmp, streaming=None), TEXTO)
    assert not ok and "streaming" in motivo
    assert not c02.evaluar(_evidencia(tmp, streaming={"ok": False}), TEXTO)[0]
    assert not c02.evaluar(_evidencia(tmp, version=1), TEXTO)[0]


def _verificar(mock, salida):
    b = io.StringIO()
    with contextlib.redirect_stdout(b):
        rc = verificar_c02.main(["--ip", "127.0.0.1", "--puerto", str(mock.puerto), "--delta", "2", "-y",
                                 "--salida", salida, "--timeout", "30"])
    return rc, b.getvalue()


def _cfg(salida, puerto):
    cfg = cargar_config(os.path.join(RAIZ, "config.json"))
    cfg["rws"].update({"ip": "127.0.0.1", "puerto": puerto, "evidencia_c02": salida})
    return cfg


def test_verificacion_completa_habilita_el_envio():
    m = MockIRC5(perfil_tiempo=True)
    m.iniciar()
    try:
        salida = os.path.join(tempfile.mkdtemp(), "evidencia_c02.json")
        rc, out = _verificar(m, salida)
        assert rc == 0, out
        ev = json.load(open(salida))
        assert ev["ok"] and ev["modos"] == ["AUTO"] and ev["huella_modulo"] == huella_modulo(TEXTO)
        for mv in ev["movimientos"]:
            if mv.get("resumen"):
                continue
            assert all(v <= lim for v, lim in zip(mv["vmax_deg_s"], ev["vlim_deg_s"])), mv
            assert mv["ok_tiempo_rapid"]
        st = ev["streaming"]
        assert st["ok"], st
        assert st["puntos"] == 8 and st["codigos"] == [3] * 7 + [2]
        assert all(v <= lim for v, lim in zip(st["vmax_deg_s"], ev["vlim_deg_s"])), st
        assert st["vmax_deg_s"][5] > 0.5 and st["duracion_observada_s"] >= 0.9 * st["t_esperado_total_s"]
        assert m.fallas_esquina == 0, "la PC no llegó a tiempo con el punto siguiente"
        assert [mv["zona"] for mv in m.movimientos[-8:]] == [True] * 7 + [False]
        # el backend lee la evidencia y habilita el envío
        be = BackendRWS(_cfg(salida, m.puerto))
        assert be.velocidad_articular_verificada, be.c02_motivo
        ok, motivo = be.comprobar_c02_en_controlador()
        assert ok, motivo
        m.opmode = "MANR"                       # verificado en AUTO: en manual no habilita
        assert not be.comprobar_c02_en_controlador()[0]
        m.opmode = "AUTO"
        be.iniciar()
        try:
            import time
            time.sleep(0.4)
            q, _ = be.leer()
            Q = np.array([q, q + np.radians([2, 0, 0, 0, 0, 0])])
            ok, msg = be.ejecutar_trayectoria(np.array([0, 1.0]), Q, origen_revisado=q)
            assert ok, msg
            t0 = time.monotonic()
            while be.resultado_trayectoria()[0] is None and time.monotonic() - t0 < 20:
                time.sleep(0.1)
            assert be.resultado_trayectoria()[0] is True, be.resultado_trayectoria()
        finally:
            be.detener()
    finally:
        m.detener() if hasattr(m, "detener") else None


def test_controlador_mas_rapido_que_lo_pedido_no_pasa():
    m = MockIRC5(perfil_tiempo=True, escala_tiempo=0.4)   # el «IRC5» ignora el tiempo y va 2.5× más rápido
    m.iniciar()
    try:
        salida = os.path.join(tempfile.mkdtemp(), "evidencia_c02.json")
        rc, out = _verificar(m, salida)
        assert rc == 1, out
        assert json.load(open(salida))["ok"] is False
        be = BackendRWS(_cfg(salida, m.puerto))
        assert not be.velocidad_articular_verificada
        ok, msg = be.ejecutar_trayectoria(np.array([0, 1.0]), np.zeros((2, 6)), origen_revisado=np.zeros(6))
        assert not ok and "C02" in msg
    finally:
        m.detener() if hasattr(m, "detener") else None


def test_sin_evidencia_sigue_bloqueado():
    salida = os.path.join(tempfile.mkdtemp(), "no_existe.json")
    be = BackendRWS(_cfg(salida, 9))
    assert not be.velocidad_articular_verificada and "verificar_c02" in be.c02_motivo
    assert be.info()["c02_motivo"] == be.c02_motivo


def test_reemplaza_un_servidor_viejo_en_ejecucion():
    m = MockIRC5(perfil_tiempo=True)
    m.iniciar()
    try:
        viejo = TEXTO.replace("vPc\\T:=t,fine", "vPc,fine")
        with m.lock:
            m.modulos.append("VEVAS_RWS"); m.textos["VEVAS_RWS"] = viejo
            m.pp = ("VEVAS_RWS", "VevasServidor"); m.ctrlstate = "motoron"; m.exec = "running"
        salida = os.path.join(tempfile.mkdtemp(), "evidencia_c02.json")
        rc, out = _verificar(m, salida)
        assert rc == 0, out
        assert "VevasServidor viejo" in out
        assert huella_modulo(m.textos["VEVAS_RWS"]) == huella_modulo(TEXTO)
    finally:
        m.detener()


def test_reintenta_error_registering_callback():
    """24/11: el IRC5 contestó HTTP 500 «Error registering callback» al pedir el texto del módulo."""
    from vevas.rws_cliente import Cliente
    m = MockIRC5()
    m.iniciar()
    try:
        with m.lock:
            m.modulos.append("VEVAS_RWS"); m.textos["VEVAS_RWS"] = TEXTO
            m.fallas_texto = 2
        c = Cliente("127.0.0.1", "Default User", "robotics", m.puerto, 3.0)
        assert huella_modulo(c._texto_modulo()) == huella_modulo(TEXTO)
        assert m.fallas_texto == 0
    finally:
        m.detener()


def test_no_apaga_motores_si_el_modulo_ya_esta_y_espera_motors_on_del_operador():
    """24/11 tras reiniciar: instalar apagaba los motores aunque el módulo ya estaba, y el IRC5 negaba
    «motores ON» por RWS (403). Ahora no se apagan sin necesidad y, si hace falta, se espera al operador."""
    import threading, time
    from vevas.rws_cliente import Cliente
    m = MockIRC5()
    m.iniciar()
    try:
        with m.lock:
            m.modulos.append("VEVAS_RWS"); m.textos["VEVAS_RWS"] = TEXTO
            m.ctrlstate = "motoron"; m.niega_motoron = True
        c = Cliente("127.0.0.1", "Default User", "robotics", m.puerto, 3.0)
        assert c.instalar_modulo(MOD, eco=lambda *_: None) == "ya_instalado"
        assert m.ctrlstate == "motoron"
        with m.lock:
            m.ctrlstate = "motoroff"
        threading.Timer(1.0, lambda: setattr(m, "ctrlstate", "motoron")).start()   # el operador pulsa MOTORS ON
        c.preparar_y_arrancar(eco=lambda *_: None)
        assert m.exec == "running" and m.pp == ("VEVAS_RWS", "VevasServidor")
    finally:
        m.detener()


if __name__ == "__main__":
    for n, f in sorted(globals().items()):
        if n.startswith("test_"):
            f()
            print("ok", n)
