"""Pruebas de /api/ik_vivo, /api/calcular_home y sig.nivel. Correr con:
    python3 06_tests/test_ik_vivo.py     o     python3 -m pytest -q 06_tests/test_ik_vivo.py"""
import json
import math
import os
import socket
import sys
import tempfile
import threading
import time
import urllib.request

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vevas.backend_sim import BackendSim  # noqa: E402
from vevas.consola import Consola  # noqa: E402
from vevas.registro import Registro  # noqa: E402
from vevas.robot_irb120 import describir_pose  # noqa: E402
from vevas.seguridad import cargar_config  # noqa: E402
from vevas.servidor import ORDENES, crear_servidor  # noqa: E402

CFG_RUTA = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config.json")
HOME = [0, -20, 30, 0, 40, 0]


def nueva(q0=HOME):
    b = BackendSim(np.radians(q0))
    con = Consola(cargar_config(CFG_RUTA), b, Registro(tempfile.mkdtemp(), "test"))
    b.iniciar()
    con.iniciar()
    time.sleep(0.15)
    return con, b


def cerrar(con, b):
    con.detener()
    b.detener()
    time.sleep(0.1)
    con.reg.cerrar()


def pose_de(con, q_deg):
    d = describir_pose(con.robot.fk_tcp(np.radians(q_deg)))
    return {"x": d["x_mm"], "y": d["y_mm"], "z": d["z_mm"], "rz": d["rz_deg"], "ry": d["ry_deg"], "rx": d["rx_deg"]}


def chequeo(plan, id_):
    return next(c for c in plan["revision"]["chequeos"] if c["id"] == id_)


def test_ik_vivo_pose_conocida_admisible():
    con, b = nueva()
    try:
        q = [20, 10, 20, 15, 50, -30]
        ik = con.ik_vivo_rapido(pose_de(con, q), q_ref_deg=q)
        assert ik["alcanzable"] and ik["admisible"], ik["motivos"]
        assert len(ik["ramas"]) == 8 and len(ik["marcos"]) == 8 and len(ik["marcos"][0]) == 16
        assert ik["verificacion"]["e_p_mm"] < 1e-3 and ik["verificacion"]["e_R_deg"] < 1e-3
        assert np.allclose(ik["q_deg"], q, atol=1e-4)
        assert ik["delta_max_deg"] < 1e-3 and ik["singular"]["tipo"] is None and not ik["singular"]["cerca"]
        # q_ref por defecto = postura medida
        ik2 = con.ik_vivo_rapido(pose_de(con, HOME))
        assert ik2["admisible"] and np.allclose(ik2["q_deg"], HOME, atol=1e-3)
    finally:
        cerrar(con, b)


def test_ik_vivo_inalcanzable():
    con, b = nueva()
    try:
        ik = con.ik_vivo_rapido({"x": 2000, "y": 0, "z": 300, "rz": 0, "ry": 0, "rx": 0})
        assert ik["alcanzable"] is False and ik["admisible"] is False and ik["motivos"]
        assert ik["elegida"] is None and ik["q_deg"] is None and ik["marcos"] is None and ik["verificacion"] is None
        assert len(ik["ramas"]) == 8
    finally:
        cerrar(con, b)


def test_ik_vivo_cerca_singular_muneca():
    con, b = nueva()
    try:
        q = [0, -20, 30, 0, 0.5, 0]
        ik = con.ik_vivo_rapido(pose_de(con, q), q_ref_deg=q)
        assert ik["admisible"] is False and ik["singular"]["tipo"] == "muñeca" and ik["singular"]["cerca"]
        assert any("singularidad" in m for m in ik["motivos"])
    finally:
        cerrar(con, b)


def test_ik_vivo_tiempo():
    con, b = nueva()
    try:
        p = pose_de(con, [20, 10, 20, 15, 50, -30])
        con.ik_vivo_rapido(p)
        t = time.perf_counter()
        for _ in range(200):
            con.ik_vivo_rapido(p)
        ms = (time.perf_counter() - t) * 1000 / 200
        print(f"ik_vivo: {ms:.2f} ms por llamada")
        assert ms < 15.0, ms
    finally:
        cerrar(con, b)


def test_calcular_home_cero_es_aviso_y_articular_normal_rechaza():
    con, b = nueva()
    try:
        p = con.calcular_home("cero")
        assert p["descripcion"] == "Volver a cero" and p["ejecutable"], p["problemas"]
        c = chequeo(p, "singularidad")
        assert c["estado"] == "aviso" and "q = 0 es singular de muñeca" in c["detalle"]
        for id_ in ("limites", "velocidad", "altura", "autocolision", "inicio"):
            assert chequeo(p, id_)["estado"] == "aprobado", chequeo(p, id_)
        assert np.allclose(p["q_obj_deg"], 0)
        # un plan articular normal al mismo destino sigue rechazado
        p2 = con.calcular_articular([0, 0, 0, 0, 0, 0])
        assert chequeo(p2, "singularidad")["estado"] == "rechazado" and not p2["ejecutable"]
        p3 = con.calcular_home("inicial")
        assert p3["descripcion"] == "Volver a la postura inicial"
        assert chequeo(p3, "singularidad")["estado"] in ("aprobado", "aviso")
    finally:
        cerrar(con, b)


def test_sing_nivel_en_estado_vivo():
    con, b = nueva()
    try:
        assert con.estado()["sing"]["nivel"] == "ok"
    finally:
        cerrar(con, b)
    con, b = nueva([0, 0, 0, 0, 0, 0])
    try:
        assert con.estado()["sing"]["nivel"] == "singular"
    finally:
        cerrar(con, b)


def _post(puerto, ruta, cuerpo):
    req = urllib.request.Request(f"http://127.0.0.1:{puerto}{ruta}", json.dumps(cuerpo).encode(),
                                 {"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=10).read())


def test_http():
    con, b = nueva()
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    puerto = s.getsockname()[1]
    s.close()
    srv = crear_servidor(con, os.path.join(os.path.dirname(CFG_RUTA), "08_web"), "127.0.0.1", puerto, 20)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        assert "ik_vivo" in ORDENES and "calcular_home" in ORDENES
        r = _post(puerto, "/api/ik_vivo", pose_de(con, HOME))
        assert r["ok"] and r["ik"]["admisible"] and set(r["ik"]) >= {
            "alcanzable", "ramas", "elegida", "q_deg", "marcos", "verificacion", "sigma_min", "singular",
            "delta_max_deg", "admisible", "motivos"}
        r = _post(puerto, "/api/ik_vivo", {"x": 3000, "y": 0, "z": 0, "rz": 0, "ry": 0, "rx": 0, "q_ref_deg": HOME})
        assert r["ok"] and r["ik"]["alcanzable"] is False
        r = _post(puerto, "/api/calcular_home", {"destino": "cero"})
        assert r["ok"] and r["plan"]["ejecutable"] and r["plan"]["descripcion"] == "Volver a cero"
        r = _post(puerto, "/api/calcular_home", {"destino": "xx"})
        assert r["ok"] is False
    finally:
        srv.shutdown()
        cerrar(con, b)


def test_j5_sim_conserva_las_otras_juntas_y_verifica_llegada():
    con, backend = nueva()
    try:
        assert con.armar({}, "")[0]
        plan = con.calcular_home("j5")
        assert plan["ejecutable"]
        assert math.isclose(plan["q_obj_deg"][4], 30.0)
        for indice in (0, 1, 2, 3, 5):
            assert plan["q_obj_deg"][indice] == plan["q0_deg"][indice]
        assert con.ejec is None
        assert con.ejecutar(plan["id"])[0]
        plazo = time.monotonic() + plan["tray"]["tf"] + 6.0
        while con.ejec is not None and time.monotonic() < plazo:
            con.pulso()
            time.sleep(0.03)
        assert con.ultimo_resumen["llegada_verificada"]
        assert con.ultimo_resumen["error_final_max_deg"] <= 0.25
    finally:
        cerrar(con, backend)


def test_velocidad_invalida_plan_sim_y_llegada_fuera_de_tolerancia():
    con, backend = nueva()
    try:
        con.calcular_home("j5")
        con.fijar_velocidad(10)
        assert con.plan is None
        plan = con.calcular_home("j5")
        assert plan["vel_pct"] == 10
        destino = np.radians(plan["q_obj_deg"])
        assert con._llegada_sim_verificada({"q_obj": destino}, destino)
        assert not con._llegada_sim_verificada({"q_obj": destino}, None)
        assert not con._llegada_sim_verificada({"q_obj": destino}, np.radians(HOME))
    finally:
        cerrar(con, backend)


if __name__ == "__main__":
    n = 0
    for nombre, f in list(globals().items()):
        if nombre.startswith("test_") and callable(f):
            f()
            n += 1
            print("ok", nombre)
    print(f"{n} pruebas OK")
