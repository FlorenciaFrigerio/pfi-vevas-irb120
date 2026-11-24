#!/usr/bin/env python3
"""Verificación presencial de C02: mide la velocidad real de cada junta del IRB 120 y guarda la evidencia.

Uso (llave en AUTO, célula despejada, una persona con la mano en el paro de emergencia):

    python3 herramientas/verificar_c02.py            # ±5° en J1…J6 desde la postura actual
    python3 herramientas/verificar_c02.py --delta 3  # pasos más chicos

Qué hace:
  1. Lee el estado (AUTO, cadena de seguridad, RAPID libre) y carga rapid/VEVAS_RWS.mod si falta o cambió.
  2. Arranca VevasServidor (el FlexPendant pide «Iniciar»).
  3. Por cada junta: va +δ y vuelve (o −δ si +δ se sale de los límites), leyendo la postura por RWS tan rápido
     como responda, y estima la velocidad con diferencias de ventana 0.1 s.
  4. Pasa si en TODOS los movimientos ninguna junta supera vLim (5 % de la ficha) y el RAPID usó el tiempo
     esperado (vvRbTiempo). Guarda evidencia_c02.json (huella del módulo, modo, topes, trazas resumidas).

Solo con esa evidencia la consola VEVAS habilita el envío por RWS. Si falla, no habilita nada.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import shutil
import sys
import threading
import time

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
sys.path.insert(0, RAIZ)

from vevas import c02  # noqa: E402
from vevas.rws_cliente import (LIMITES_DEG, MARGEN_DEG, Cliente, Falla, huella_modulo,  # noqa: E402
                               limites_c02, rechazo_local, tiempo_movimiento)
from vevas.seguridad import cargar_config  # noqa: E402


class Muestreador:
    def __init__(self, cli: Cliente):
        self.cli = cli
        self.muestras: list[tuple[float, list[float]]] = []
        self._fin = threading.Event()
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self.errores = 0

    def _bucle(self):
        while not self._fin.is_set():
            try:
                q = self.cli.joints_deg()
                self.muestras.append((time.monotonic(), q))
            except Falla:
                self.errores += 1
                time.sleep(0.01)

    def __enter__(self):
        self._hilo.start()
        return self

    def __exit__(self, *a):
        self._fin.set()
        self._hilo.join(2)


def destino(q0, j, delta):
    lo, hi = LIMITES_DEG[j - 1]
    q = list(q0)
    if q0[j - 1] + delta <= hi - MARGEN_DEG - 1:
        q[j - 1] = q0[j - 1] + delta
    else:
        q[j - 1] = q0[j - 1] - delta
    return q


def un_movimiento(cli, cli_lectura, objetivo, vlim, k, timeout):
    q0 = cli.joints_deg()
    r = rechazo_local(objetivo, q0)
    if r is not None:
        raise Falla(f"La PC rechaza el objetivo: {r[1]}")
    t_esp = tiempo_movimiento(objetivo, q0, vlim, k)
    with Muestreador(cli_lectura) as m:
        time.sleep(0.3)
        ok, cod, msg = cli.enviar_objetivo(objetivo, timeout=timeout)
        time.sleep(0.5)
    if not ok:
        raise Falla(f"El robot no llegó: {msg}")
    try:
        t_rapid = cli._num("vvRbTiempo")
    except Falla:
        t_rapid = None
    ms = m.muestras
    vmax = c02.velocidades_max(ms)
    dts = [b[0] - a[0] for a, b in zip(ms, ms[1:])]
    moviendo = [t for (t, q), (_, qa) in zip(ms[1:], ms) if max(abs(x - y) for x, y in zip(q, qa)) > 0.005]
    dur = (moviendo[-1] - moviendo[0]) if len(moviendo) >= 2 else None
    pasa_v = all(v <= lim for v, lim in zip(vmax, vlim))
    pasa_t = t_rapid is not None and abs(t_rapid - t_esp) <= c02.TOL_TIEMPO * t_esp
    return {"desde_deg": [round(x, 4) for x in q0], "hasta_deg": [round(x, 4) for x in objetivo],
            "t_esperado_s": round(t_esp, 4), "t_rapid_s": t_rapid, "duracion_observada_s": dur and round(dur, 3),
            "vmax_deg_s": [round(v, 3) for v in vmax], "muestras": len(ms), "errores_lectura": m.errores,
            "periodo_medio_s": round(sum(dts) / len(dts), 4) if dts else None,
            "periodo_max_s": round(max(dts), 4) if dts else None,
            "ok_velocidad": pasa_v, "ok_tiempo_rapid": pasa_t, "ok": pasa_v and pasa_t}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(RAIZ, "config.json"))
    ap.add_argument("--ip"); ap.add_argument("--puerto", type=int); ap.add_argument("--usuario")
    ap.add_argument("--clave", default=os.environ.get("VEVAS_RWS_CLAVE"))
    ap.add_argument("--delta", type=float, default=5.0, help="grados por junta (máx. 10)")
    ap.add_argument("--juntas", default="1,2,3,4,5,6")
    ap.add_argument("--salida", default=None, help="por defecto evidencia_c02.json en la raíz del código")
    ap.add_argument("--modulo", default=os.path.join(RAIZ, "rapid", "VEVAS_RWS.mod"))
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("-y", "--si", action="store_true", help="no preguntar (solo para pruebas automáticas)")
    a = ap.parse_args(argv)
    if not 0.5 <= a.delta <= 10:
        print("--delta tiene que estar entre 0.5 y 10 grados.")
        return 2
    cfg = cargar_config(a.config)
    r = cfg["rws"]
    ip, puerto = a.ip or r["ip"], a.puerto or int(r["puerto"])
    usuario, clave = a.usuario or r["usuario"], a.clave or r["clave"]
    cli = Cliente(ip, usuario, clave, puerto, float(r["timeout_http_s"]))
    cli_lect = Cliente(ip, usuario, clave, puerto, float(r["timeout_http_s"]))
    salida = a.salida or c02.ruta_por_defecto(RAIZ)
    juntas = [int(x) for x in a.juntas.split(",")]
    with open(a.modulo, encoding="utf-8") as f:
        texto = f.read()
    try:
        vlim, k = limites_c02(texto)
        e = cli.leer_estado()
        print(f"Controlador: modo {e['opmode']}, motores {e['ctrlstate']}, RAPID {e['exec']}, PP {e['pp_modulo']}/{e['pp_rutina']}")
        print("Juntas ahora:", " ".join(f"{x:8.2f}" for x in e["q_deg"]))
        if e["opmode"] != "AUTO":
            print("Para esta verificación el controlador tiene que estar en AUTO (la llave la gira una persona).")
            return 2
        print(f"\nSe moverá cada junta {', '.join(f'J{j}' for j in juntas)} {a.delta:g}° y de vuelta, una por vez.")
        print(f"Topes del RAPID (5 %): {vlim} °/s, kPico {k}. Tiempo por tramo ≈ {tiempo_movimiento([a.delta] + [0]*5, [0]*6, vlim, k):.1f} s o más.")
        if not a.si:
            resp = input("Zona despejada y mano en el paro de emergencia. ¿Mover el robot real? (s/n): ").strip().lower()
            if resp not in ("s", "si", "sí"):
                print("Cancelado: no se movió nada.")
                return 1
        e = cli.leer_estado(con_juntas=False)
        if e["exec"] == "running":
            if not cli.pp_en_servidor(e):
                print(f"RAPID está ejecutando otro programa ({e['pp_modulo']}/{e['pp_rutina']}): detenelo antes. No se movió nada.")
                return 1
            if huella_modulo(cli._texto_modulo()) != huella_modulo(texto):
                print("VevasServidor viejo en ejecución: lo detengo (Stop + motores OFF) para cargar la versión C02.")
                cli.parar()
        print(f"Módulo: {cli.instalar_modulo(a.modulo)}")
        e = cli.leer_estado(con_juntas=False)
        if e["exec"] != "running" or not cli.pp_en_servidor(e):
            cli.preparar_y_arrancar()
        print(">>> Si el FlexPendant pide «Aceptar objetivos enviados por la PC?», pulsá «Iniciar» (30 s).")
        if not cli.esperar_servidor(60, pausa_inicial=1.0):
            print("VevasServidor no quedó activo (¿no se pulsó «Iniciar»?). No se movió nada.")
            return 1
        txt_ctrl = cli._texto_modulo()
        if huella_modulo(txt_ctrl) != huella_modulo(texto):
            print("El módulo del controlador no coincide con rapid/VEVAS_RWS.mod: no se verifica.")
            return 1
        movs = []
        for j in juntas:
            q0 = cli.joints_deg()
            ida = destino(q0, j, a.delta)
            for nombre, obj in (("ida", ida), ("vuelta", q0)):
                res = un_movimiento(cli, cli_lect, obj, vlim, k, a.timeout)
                res.update({"junta": j, "tramo": nombre})
                movs.append(res)
                marca = "OK " if res["ok"] else "MAL"
                print(f"  {marca} J{j} {nombre:6s} t={res['t_esperado_s']:.2f}s (RAPID {res['t_rapid_s']}) "
                      f"vmax={res['vmax_deg_s'][j-1]:.2f}°/s (tope {vlim[j-1]}) · peor otra junta "
                      f"{max(v for i, v in enumerate(res['vmax_deg_s']) if i != j-1):.2f}°/s · {res['muestras']} muestras")
    except Falla as ex:
        print(f"ERROR: {ex}")
        return 1
    ok = all(m["ok"] for m in movs) and {m["junta"] for m in movs} == set(juntas)
    por_junta = [{"junta": j, "ok": all(m["ok"] for m in movs if m["junta"] == j)} for j in juntas]
    ev = {"version": c02.VERSION_EVIDENCIA, "ok": ok and set(juntas) == {1, 2, 3, 4, 5, 6},
          "fecha_utc": dt.datetime.now(dt.timezone.utc).isoformat(), "ip": ip, "modos": [e["opmode"]],
          "huella_modulo": huella_modulo(texto), "vlim_deg_s": list(vlim), "k_pico": k, "delta_deg": a.delta,
          "ventana_s": c02.VENTANA_S, "metodo": "posiciones RWS /rw/motionsystem/mechunits/ROB_1/jointtarget, diferencia finita",
          "movimientos": [dict(m, ok=m["ok"]) for m in movs], "juntas": por_junta}
    # una entrada por junta (ok si ida y vuelta pasaron) para evaluar()
    ev["movimientos"] = movs + [{"junta": p["junta"], "ok": p["ok"], "resumen": True} for p in por_junta]
    ev["movimientos"] = [m for m in ev["movimientos"] if m.get("resumen")] + [m for m in ev["movimientos"] if not m.get("resumen")]
    if os.path.exists(salida):
        shutil.copy(salida, salida + ".anterior")
    with open(salida, "w", encoding="utf-8") as f:
        json.dump(ev, f, ensure_ascii=False, indent=2)
    print(f"\nEvidencia guardada en {salida}")
    if ev["ok"]:
        print("C02 VERIFICADO para este módulo y modo: la consola VEVAS ya puede enviar por RWS.")
        return 0
    print("C02 NO pasó (o faltan juntas): la consola sigue bloqueada. Revisá la tabla de arriba.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
