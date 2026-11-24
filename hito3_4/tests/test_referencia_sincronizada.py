"""La latencia se vigila aparte; el seguimiento compara instantes iguales."""
import time
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vevas.backend_sim import BackendSim
from vevas.consola import Consola
from vevas.registro import Registro
from vevas.seguridad import cargar_config


def preparar(tmp_path, desfase_deg=0, edad=0.2):
    cfg = cargar_config(None)
    q0 = np.radians(cfg["postura_inicial_deg"])
    b = BackendSim(q0, ruido_deg=0)
    c = Consola(cfg, b, Registro(str(tmp_path), "test_referencia"))
    c.fijar_velocidad(20)
    qf = np.degrees(q0)
    qf[0] += 30
    c.calcular_articular(qf)
    tray = c.plan["_tray"]
    origen = time.monotonic()
    now = origen + 0.8
    sello = now - edad
    medida = tray.evaluar(sello - origen)[0].copy()
    medida[0] += np.radians(desfase_deg)
    b.leer = lambda: (medida.copy(), sello)
    c.armado = True
    c.pulso_t = now
    c.ejec = {"tray": tray, "t0": origen, "plan_id": c.plan["id"], "q_obj": tray.qf,
              "descripcion": "Trayectoria sintética para comparar referencias",
              "err_max_deg": 0, "e_p_max_mm": 0, "e_R_max_deg": 0}
    return c, now


def test_muestra_retrasada_valida_no_crea_error_ficticio(tmp_path):
    c, now = preparar(tmp_path)
    try:
        c._ciclo(now, 0.02)
        assert c.ejec is not None
        assert max(abs(v) for v in c.vivo["err_deg"]) < 1e-8
        assert c.vivo["edad_estado_s"] == pytest.approx(0.2)
        assert c.vivo["referencia_sincronizada_con_estado"] is True
        assert np.max(np.abs(np.degrees(c.q_ref) - c.vivo["q_ref_deg"])) > 3
    finally:
        c.reg.cerrar()


def test_error_real_aun_aborta_con_el_mismo_umbral(tmp_path):
    c, now = preparar(tmp_path, desfase_deg=5)
    try:
        c._ciclo(now, 0.02)
        assert c.ejec is None
        assert "Error de seguimiento" in c.ultimo_resumen["motivo"]
        assert c.cfg["seguridad"]["error_seguimiento_max_deg"] == 3
    finally:
        c.reg.cerrar()


def test_estado_caducado_sigue_bloqueado_por_watchdog(tmp_path):
    c, now = preparar(tmp_path, edad=0.6)
    try:
        c._ciclo(now, 0.02)
        assert c.ejec is None
        assert "watchdog" in c.ultimo_resumen["motivo"]
    finally:
        c.reg.cerrar()
