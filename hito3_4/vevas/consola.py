"""Lógica central de la consola VEVAS.

Flujo que pidió el docente (y que se ve en pantalla):

    1. el usuario da una pose (o el valor de una junta, o las 6 juntas)
    2. se calcula TODO y se muestra: T_d, centro de muñeca, las 8 ramas de Pieper,
       el error FK de cada rama, límites, singularidad
    3. se elige la mejor rama (error mínimo + dentro de límites + menor movimiento)
    4. se planifica la trayectoria LSPB y se revisa muestra por muestra
    5. RECIÉN AHÍ se manda al robot
    6. mientras se mueve se calcula el error en tiempo real:
         e_q,i = q_medida,i - q_comandada,i                     [grados]
         e_p   = || p(q_medida) - p(q_comandada) ||              [mm]
         e_R   = ángulo de R(q_comandada)^T R(q_medida)          [grados]
       y si algo se sale de lo permitido, se aborta y el robot se queda quieto.
"""

from __future__ import annotations

import datetime as dt
import math
import threading
import time
from collections import deque

import numpy as np

from . import pieper as pp
from .jacobiano import analisis_singularidad
from .robot_irb120 import (
    VEL_MAX_DEG,
    Robot,
    describir_pose,
    error_orientacion,
    error_posicion,
    pose_desde_xyz_euler,
)
from .secuencias import pasos_demo
from .seguridad import revisar_plan, semaforo
from .trayectoria import comprobar_continuidad, planificar

FRASE_REAL = "MOVER ROBOT REAL"
GRAD = 180.0 / math.pi


def _m16(T) -> list:
    return [round(float(v), 6) for v in np.asarray(T).reshape(-1)]


class Consola:
    def __init__(self, cfg: dict, backend, registro, permitir_real: bool = False, go: dict | None = None):
        self.cfg = cfg
        rc = cfg["robot"]
        self.robot = Robot(tcp_xyz=tuple(rc["tcp_xyz_m"]), tcp_rpy=tuple(rc["tcp_rpy_rad"]))
        self.backend = backend
        self.reg = registro
        self.permitir_real = permitir_real
        self.go = go
        self.lock = threading.RLock()
        s = cfg["seguridad"]
        self.vel_pct = float(s["velocidad_inicial_pct"])
        self.armado = False
        self.motivo_desarmado = "Todavía no se armó el movimiento."
        self.plan = None
        self.n_planes = 0
        self.ejec = None
        self.jog = None
        self.q_ref = None
        self.pulso_t = time.monotonic()
        self.eventos = deque(maxlen=80)
        self.n_evento = 0
        self.vivo = {}
        self.ik_vivo = None
        self.secuencia = None
        self._sec_cmd = None
        self._ciclo_n = 0
        self._t_muestra = 0.0
        self.t0 = time.monotonic()
        self.activo = False
        self.ultimo_resumen = None

    # Utilidades
    def avisar(self, texto: str, nivel: str = "info", **datos):
        with self.lock:
            self.n_evento += 1
            self.eventos.append(
                {"n": self.n_evento, "hora": dt.datetime.now().strftime("%H:%M:%S"), "nivel": nivel, "texto": texto}
            )
        self.reg.evento("aviso", nivel=nivel, texto=texto, **datos)

    def modo(self) -> dict:
        info = self.backend.info()
        real = info.get("real")
        es_real = (self.backend.tipo == "ros") and (real is True or real is None)
        info["exige_go"] = es_real
        info["permitir_real"] = self.permitir_real
        info["vel_max_pct"] = (
            self.cfg["seguridad"]["velocidad_real_max_pct"] if es_real else self.cfg["seguridad"]["velocidad_sim_max_pct"]
        )
        return info

    def _q_medida(self):
        q, t = self.backend.leer()
        return q, t

    def pulso(self):
        self.pulso_t = time.monotonic()

    # Bucle principal (50 Hz)
    def iniciar(self):
        self.activo = True
        threading.Thread(target=self._bucle, daemon=True).start()

    def detener(self):
        self.activo = False

    def _bucle(self):
        periodo = 0.02
        while self.activo:
            t_ini = time.monotonic()
            try:
                self._ciclo(t_ini, periodo)
            except Exception as e:  # noqa: BLE001
                self.avisar(f"Error interno en el ciclo: {e}", "error")
            time.sleep(max(0.0, periodo - (time.monotonic() - t_ini)))

    def _ciclo(self, now: float, dt_c: float):
        s = self.cfg["seguridad"]
        q_med, t_est = self._q_medida()
        with self.lock:
            self._ciclo_n += 1
            edad = (now - t_est) if q_med is not None else float("inf")
            moviendo = self.ejec is not None or self.jog is not None

            if moviendo and edad > s["watchdog_estado_s"]:
                self._abortar(f"Sin datos del robot hace {edad:.2f} s (watchdog {s['watchdog_estado_s']} s).")
            elif moviendo and now - self.pulso_t > s["watchdog_interfaz_s"]:
                self._abortar("La interfaz dejó de responder (watchdog del navegador). Se detuvo por seguridad.")

            if self.ejec is not None:
                e = self.ejec
                t = now - e["t0"]
                q_cmd, _, _ = e["tray"].evaluar(t)
                self.q_ref = q_cmd
                e["t"] = t
                if t >= e["tray"].tf + 0.4:
                    self._terminar_ejecucion(q_med)
            elif self.jog is not None:
                if now - self.jog["t_ultimo"] > 0.3:
                    self._parar_jog("se soltó la tecla (hombre muerto)")
                else:
                    self._paso_jog(dt_c)

            v = {"edad_estado_s": edad, "edad_pulso_s": now - self.pulso_t}
            if q_med is not None:
                M = self.robot.marcos(q_med)
                T_med = self.robot.fk_tcp(q_med)
                sing = analisis_singularidad(self.robot, q_med)
                v["q_med_deg"] = [float(a) * GRAD for a in q_med]
                v["tcp_med"] = describir_pose(T_med)
                v["marcos_med"] = [_m16(T) for T in M] + [_m16(T_med)]
                v["sing"] = {k: sing[k] for k in ("sigma", "sigma_min", "rango", "condicion", "det_J", "muneca_sin_q5", "codo_sin_delta", "hombro_r_w_m")}
                if self._ciclo_n % 10 == 0 or "J" not in (self.vivo.get("sing") or {}):
                    v["sing"]["J"] = [[round(x, 5) for x in fila] for fila in sing["J"]]
                else:
                    v["sing"]["J"] = self.vivo["sing"]["J"]
                v["lim_dist_deg"] = [d * GRAD for d in self.robot.distancia_a_limites(q_med)]
                if self._ciclo_n % 10 == 0:
                    v["fk_vivo"] = {"A": [_m16(A) for A in self.robot.matrices_eslabon(q_med)], "T06": _m16(M[6])}
                else:
                    v["fk_vivo"] = self.vivo.get("fk_vivo")
                if self.q_ref is not None:
                    T_ref = self.robot.fk_tcp(self.q_ref)
                    err = (np.asarray(q_med) - np.asarray(self.q_ref)) * GRAD
                    e_p = error_posicion(T_med, T_ref) * 1000.0
                    e_R = error_orientacion(T_med[:3, :3], T_ref[:3, :3]) * GRAD
                    v["q_ref_deg"] = [float(a) * GRAD for a in self.q_ref]
                    v["err_deg"] = [float(x) for x in err]
                    v["e_p_mm"] = e_p
                    v["e_R_deg"] = e_R
                    v["tcp_ref"] = describir_pose(T_ref)
                    v["marcos_ref"] = [_m16(T) for T in self.robot.marcos(self.q_ref)] + [_m16(T_ref)]
                    v["semaforo"] = semaforo(float(np.max(np.abs(err))), e_p, self.cfg)
                    if moviendo:
                        emax = float(np.max(np.abs(err)))
                        if self.ejec is not None:
                            self.ejec["err_max_deg"] = max(self.ejec["err_max_deg"], emax)
                            self.ejec["e_p_max_mm"] = max(self.ejec["e_p_max_mm"], e_p)
                            self.ejec["e_R_max_deg"] = max(self.ejec["e_R_max_deg"], e_R)
                        if emax > s["error_seguimiento_max_deg"]:
                            j = int(np.argmax(np.abs(err))) + 1
                            self._abortar(
                                f"Error de seguimiento en J{j}: {emax:.2f}° > {s['error_seguimiento_max_deg']}° permitido."
                            )
                    if moviendo and min(self.robot.distancia_a_limites(q_med)) < math.radians(1.0):
                        self._abortar("Una junta medida quedó a menos de 1° de su límite.")
                # muestra para el CSV
                periodo_m = 0.02 if moviendo else 0.5
                if now - self._t_muestra >= periodo_m - 1e-4:
                    self._t_muestra = now
                    self._guardar_muestra(now, q_med, v, moviendo)
            self.vivo = v

    def _guardar_muestra(self, now, q_med, v, moviendo):
        q_ref = v.get("q_ref_deg") or [float("nan")] * 6
        err = v.get("err_deg") or [float("nan")] * 6
        tcp = v["tcp_med"]
        estado = "ejecutando" if self.ejec else ("jog" if self.jog else "reposo")
        pid = self.ejec["plan_id"] if self.ejec else (self.plan["id"] if self.plan else 0)
        fila = [round(now - self.t0, 4), estado, pid]
        fila += [round(x, 5) for x in q_ref]
        fila += [round(x, 5) for x in v["q_med_deg"]]
        fila += [round(x, 5) for x in err]
        fila += [
            round(v.get("e_p_mm", float("nan")), 5),
            round(v.get("e_R_deg", float("nan")), 5),
            round(tcp["x_mm"], 3),
            round(tcp["y_mm"], 3),
            round(tcp["z_mm"], 3),
            round(v["sing"]["sigma_min"], 5),
            self.vel_pct,
        ]
        self.reg.muestra(fila)

    # Armado y GO
    def armar(self, checklist: dict, frase: str) -> tuple[bool, str]:
        with self.lock:
            m = self.modo()
            if m.get("solo_lectura"):
                return False, "La consola se abrió en SOLO LECTURA."
            if m["exige_go"]:
                if not self.permitir_real:
                    return False, (
                        "La consola está conectada a un robot REAL (o a un hardware que no se pudo identificar) y "
                        "se abrió en SOLO LECTURA. Para mover, ciérrala y ábrela con --real y el archivo GO lleno."
                    )
                ok, msg = validar_go(self.go)
                if not ok:
                    return False, msg
                faltan = [k for k in ("supervisor", "paro", "area", "velocidad") if not checklist.get(k)]
                if faltan:
                    return False, "Falta marcar en la lista: " + ", ".join(faltan)
                if (frase or "").strip().upper() != FRASE_REAL:
                    return False, f'Escribe exactamente la frase "{FRASE_REAL}" para armar el robot real.'
            self.armado = True
            self.vel_pct = min(self.vel_pct, m["vel_max_pct"])
            self.avisar(f"Movimiento ARMADO ({m['texto']}). Velocidad {self.vel_pct:.0f} %.", "ok", checklist=checklist)
            return True, "Armado."

    def desarmar(self, motivo: str = "desarmado por el usuario"):
        with self.lock:
            if self.ejec is not None or self.jog is not None:
                self._abortar(motivo)
            self.armado = False
            self.motivo_desarmado = motivo
            self.avisar(f"Movimiento DESARMADO: {motivo}", "aviso")

    def fijar_velocidad(self, pct: float) -> float:
        with self.lock:
            m = self.modo()
            self.vel_pct = float(max(1.0, min(pct, m["vel_max_pct"])))
            return self.vel_pct

    # Cálculo de planes
    def _q_inicio(self):
        q_med, _ = self._q_medida()
        if q_med is None:
            raise RuntimeError("Todavía no llega la posición del robot (/joint_states).")
        return np.asarray(q_med, dtype=float)

    def _factor(self, vel_cap_deg_s: float | None) -> float:
        f = self.vel_pct / 100.0
        if vel_cap_deg_s is not None:
            f = min(f, vel_cap_deg_s / min(VEL_MAX_DEG))
        return f

    def _armar_plan(self, tipo, descripcion, q0, q_obj, extra: dict, vel_cap_deg_s=None) -> dict:
        tcfg = self.cfg["trayectoria"]
        tray = planificar(
            q0, q_obj, self.robot.vel_max, self._factor(vel_cap_deg_s), tcfg["t_acel_s"], tcfg["kappa"], tcfg["t_min_s"]
        )
        ts, Q, QD, _ = tray.muestrear(0.02)
        problemas = revisar_plan(self.robot, Q, q0, self.cfg)
        T_obj = self.robot.fk_tcp(q_obj)
        sing = analisis_singularidad(self.robot, q_obj)
        # vista previa: marcos de ~40 muestras y el camino del TCP
        idx = np.linspace(0, len(ts) - 1, min(len(ts), 45)).astype(int)
        frames = []
        camino = []
        for k in idx:
            M = self.robot.marcos(Q[k])
            T = self.robot.fk_tcp(Q[k])
            frames.append([_m16(A) for A in M] + [_m16(T)])
            camino.append([float(v) for v in T[:3, 3]])
        self.n_planes += 1
        plan = {
            "id": self.n_planes,
            "tipo": tipo,
            "descripcion": descripcion,
            "creado": dt.datetime.now().strftime("%H:%M:%S"),
            "q0_deg": [float(a) * GRAD for a in q0],
            "q_obj_deg": [float(a) * GRAD for a in q_obj],
            "q_obj_rad": [float(a) for a in q_obj],
            "T_obj": _m16(T_obj),
            "pose_obj": describir_pose(T_obj),
            "tray": tray.a_dict(0.05),
            "continuidad": comprobar_continuidad(tray),
            "sing_obj": {k: sing[k] for k in ("sigma", "sigma_min", "rango", "condicion", "det_J", "muneca_sin_q5", "codo_sin_delta", "hombro_r_w_m")},
            "J_obj": sing["J"],
            "fk_obj": {"A": [_m16(A) for A in self.robot.matrices_eslabon(q_obj)], "T06": _m16(self.robot.fk6(q_obj)), "T_tcp": _m16(T_obj)},
            "preview": {"frames": frames, "camino": camino, "t": [float(ts[k]) for k in idx]},
            "problemas": problemas,
            "ejecutable": not problemas,
            "vel_pct": self.vel_pct,
            "vel_cap_deg_s": vel_cap_deg_s,
        }
        plan.update(extra)
        plan["_tray"] = tray
        plan["_T_obj"] = T_obj
        return plan

    def _guardar_plan(self, plan):
        datos = {k: v for k, v in plan.items() if not k.startswith("_")}
        self.reg.guardar_plan(plan["id"], datos)
        self.reg.evento(
            "plan",
            id=plan["id"],
            tipo=plan["tipo"],
            descripcion=plan["descripcion"],
            q_obj_deg=plan["q_obj_deg"],
            problemas=plan["problemas"],
            tf=plan["tray"]["tf"],
        )

    def calcular_pose(self, pose: dict, vel_cap_deg_s=None) -> dict:
        """pose: x, y, z [mm] y rz, ry, rx [°] del TCP en la base."""
        T_d = pose_desde_xyz_euler(pose["x"], pose["y"], pose["z"], pose["rz"], pose["ry"], pose["rx"])
        with self.lock:
            q0 = self._q_inicio()
            ref = self.q_ref if (self.q_ref is not None and self.ejec is None) else q0
            t_calc = time.perf_counter()
            res = pp.resolver(self.robot, T_d, ref)
            ik = self.cfg["ik"]
            mejor, razon = pp.elegir_rama(
                res["ramas"], ik["tol_e_p_m"], ik["tol_e_R_rad"], math.radians(ik["margen_limite_deg"]), ik["sigma_min"]
            )
            t_calc = (time.perf_counter() - t_calc) * 1000.0
            ramas = []
            for r in res["ramas"]:
                d = r.a_dict()
                if r.existe and r.q is not None:
                    d["marcos"] = [_m16(T) for T in self.robot.marcos(r.q)] + [_m16(self.robot.fk_tcp(r.q))]
                ramas.append(d)
            pasos = res["pasos"]
            pasos["q_actual_deg"] = [float(a) * GRAD for a in ref]
            if mejor is not None:
                R03 = self.robot.marcos(mejor.q)[3][:3, :3]
                R06 = np.asarray(pasos["T06"])[:3, :3]
                pasos["R03_elegida"] = R03.tolist()
                pasos["R36_elegida"] = (R03.T @ R06).tolist()
            extra = {
                "pose_pedida": pose,
                "T_d": _m16(T_d),
                "ik": {"pasos": pasos, "ramas": ramas, "elegida": mejor.indice if mejor else None, "razon": razon, "ms": t_calc},
            }
            if mejor is None:
                self.n_planes += 1
                plan = {
                    "id": self.n_planes,
                    "tipo": "pose",
                    "descripcion": f"Pose {pose_texto(pose)}",
                    "creado": dt.datetime.now().strftime("%H:%M:%S"),
                    "problemas": [razon],
                    "ejecutable": False,
                    **extra,
                }
                self.plan = plan
                self._guardar_plan_simple(plan)
                self.avisar(f"Plan {plan['id']}: {razon}", "error")
                return publico(plan)
            plan = self._armar_plan("pose", f"Pose {pose_texto(pose)}", q0, np.array(mejor.q), extra, vel_cap_deg_s)
            T_fin = plan["_T_obj"]
            plan["verificacion"] = {
                "e_p_m": error_posicion(T_fin, T_d),
                "e_R_rad": error_orientacion(T_fin[:3, :3], T_d[:3, :3]),
            }
            plan["_T_d"] = T_d
            self.plan = plan
            self._guardar_plan(plan)
            self.avisar(
                f"Plan {plan['id']}: pose resuelta con la rama {mejor.indice} en {t_calc:.1f} ms; "
                f"{'listo para ejecutar' if plan['ejecutable'] else 'NO ejecutable: ' + '; '.join(plan['problemas'])}.",
                "ok" if plan["ejecutable"] else "aviso",
            )
            return publico(plan)

    def _guardar_plan_simple(self, plan):
        self.reg.guardar_plan(plan["id"], {k: v for k, v in plan.items() if not k.startswith("_")})
        self.reg.evento("plan", id=plan["id"], tipo=plan["tipo"], problemas=plan["problemas"])

    def elegir_rama_manual(self, indice: int) -> dict:
        """El usuario prefiere otra rama válida (por ejemplo, para mostrar codo abajo)."""
        with self.lock:
            if not self.plan or self.plan.get("tipo") != "pose":
                raise RuntimeError("No hay un plan de pose calculado.")
            ramas = self.plan["ik"]["ramas"]
            r = ramas[indice]
            if not r["valida"]:
                raise RuntimeError("Esa rama no es válida: " + "; ".join(r["motivos"]))
            q0 = self._q_inicio()
            extra = {k: self.plan[k] for k in ("pose_pedida", "T_d", "ik")}
            extra["ik"] = dict(extra["ik"])
            extra["ik"]["elegida"] = indice
            extra["ik"]["razon"] = f"Rama {indice} elegida a mano por el usuario (la automática era {self.plan['ik']['elegida']})."
            plan = self._armar_plan("pose", self.plan["descripcion"], q0, np.array(r["q_rad"]), extra, self.plan.get("vel_cap_deg_s"))
            T_fin = plan["_T_obj"]
            T_d = self.plan["_T_d"]
            plan["verificacion"] = {"e_p_m": error_posicion(T_fin, T_d), "e_R_rad": error_orientacion(T_fin[:3, :3], T_d[:3, :3])}
            plan["_T_d"] = T_d
            self.plan = plan
            self._guardar_plan(plan)
            self.avisar(f"Plan {plan['id']}: rama {indice} elegida a mano.", "info")
            return publico(plan)

    def calcular_articular(self, q_deg, descripcion=None, vel_cap_deg_s=None) -> dict:
        with self.lock:
            q0 = self._q_inicio()
            q_obj = np.radians(np.asarray(q_deg, dtype=float))
            fuera = [i + 1 for i, ok in enumerate(self.robot.dentro_de_limites(q_obj)) if not ok]
            desc = descripcion or "Juntas " + ", ".join(f"{v:.1f}" for v in q_deg) + " °"
            plan = self._armar_plan("articular", desc, q0, q_obj, {}, vel_cap_deg_s)
            if fuera:
                plan["problemas"].insert(0, "Fuera de límites en J" + ", J".join(map(str, fuera)) + ".")
                plan["ejecutable"] = False
            self.plan = plan
            self._guardar_plan(plan)
            self.avisar(
                f"Plan {plan['id']}: {desc} -> {'listo' if plan['ejecutable'] else 'NO ejecutable'} (tf = {plan['tray']['tf']:.2f} s).",
                "ok" if plan["ejecutable"] else "aviso",
            )
            return publico(plan)

    def calcular_junta(self, junta: int, valor_deg: float, relativo: bool) -> dict:
        with self.lock:
            q0 = self._q_inicio()
            q = [float(a) * GRAD for a in q0]
            j = int(junta) - 1
            q[j] = q[j] + valor_deg if relativo else valor_deg
            txt = f"Solo J{junta} {'+' if relativo and valor_deg >= 0 else ''}{valor_deg:.1f}°" + (" (relativo)" if relativo else " (absoluto)")
            return self.calcular_articular(q, txt)

    # Ejecución
    def ejecutar(self) -> tuple[bool, str]:
        with self.lock:
            if not self.armado:
                return False, "Primero hay que ARMAR el movimiento (botón Armar)."
            if self.ejec is not None or self.jog is not None:
                return False, "El robot ya se está moviendo."
            plan = self.plan
            if plan is None or "_tray" not in plan:
                return False, "No hay un plan calculado."
            q_med, _ = self._q_medida()
            if q_med is None:
                return False, "No llega la posición del robot."
            dif = np.degrees(np.abs(np.asarray(q_med) - plan["_tray"].q0)).max()
            if dif > self.cfg["seguridad"]["inicio_max_deg"]:
                self.avisar(f"El robot se movió {dif:.2f}° desde que se calculó el plan: se recalcula desde la postura actual.", "aviso")
                tcfg = self.cfg["trayectoria"]
                plan["_tray"] = planificar(
                    q_med, plan["q_obj_rad"], self.robot.vel_max, self._factor(plan.get("vel_cap_deg_s")),
                    tcfg["t_acel_s"], tcfg["kappa"], tcfg["t_min_s"],
                )
            tray = plan["_tray"]
            ts, Q, QD, _ = tray.muestrear(self.cfg["trayectoria"]["dt_envio_s"])
            problemas = revisar_plan(self.robot, Q, q_med, self.cfg)
            if plan["tipo"] == "pose" and plan.get("ik", {}).get("elegida") is None:
                problemas.append("No hay rama válida.")
            if problemas:
                return False, "No se envía: " + " ".join(problemas)
        ok, msg = self.backend.ejecutar_trayectoria(ts[1:], Q[1:], QD[1:])
        with self.lock:
            if not ok:
                self.avisar(f"Plan {plan['id']}: el envío falló: {msg}", "error")
                return False, msg
            self.ejec = {
                "plan_id": plan["id"],
                "tray": tray,
                "t0": time.monotonic(),
                "t": 0.0,
                "err_max_deg": 0.0,
                "e_p_max_mm": 0.0,
                "e_R_max_deg": 0.0,
                "descripcion": plan["descripcion"],
                "q_obj": np.array(plan["q_obj_rad"]),
                "T_d": plan.get("_T_d"),
                "inicio": dt.datetime.now().isoformat(timespec="seconds"),
            }
            self.q_ref = tray.q0.copy()
            self.reg.evento("envio", plan_id=plan["id"], tf=tray.tf, n_puntos=len(ts) - 1, vel_pct=self.vel_pct, mensaje=msg)
            self.avisar(f"Plan {plan['id']} ENVIADO al robot ({msg}). Duración {tray.tf:.2f} s.", "ok")
            return True, msg

    def _terminar_ejecucion(self, q_med):
        e = self.ejec
        self.ejec = None
        self.q_ref = e["q_obj"].copy()
        r = {"plan_id": e["plan_id"], "descripcion": e["descripcion"], "inicio": e["inicio"], "duracion_s": e["tray"].tf}
        if q_med is not None:
            err_fin = np.degrees(np.asarray(q_med) - e["q_obj"])
            T_fin = self.robot.fk_tcp(q_med)
            r["error_final_juntas_deg"] = [float(x) for x in err_fin]
            r["error_final_max_deg"] = float(np.max(np.abs(err_fin)))
            T_obj = self.robot.fk_tcp(e["q_obj"])
            r["e_p_final_mm"] = error_posicion(T_fin, T_obj) * 1000.0
            r["e_R_final_deg"] = error_orientacion(T_fin[:3, :3], T_obj[:3, :3]) * GRAD
            if e.get("T_d") is not None:
                r["e_p_vs_pose_pedida_mm"] = error_posicion(T_fin, e["T_d"]) * 1000.0
        r["err_seguimiento_max_deg"] = e["err_max_deg"]
        r["e_p_seguimiento_max_mm"] = e["e_p_max_mm"]
        r["e_R_seguimiento_max_deg"] = e["e_R_max_deg"]
        r["resultado_controlador"] = getattr(self.backend, "ultimo_resultado", None)
        r["estado"] = "completado"
        self.ultimo_resumen = r
        self.reg.agregar_resumen(r)
        self.reg.evento("fin", **r)
        self.avisar(
            f"Plan {e['plan_id']} terminado. Error de seguimiento máx. {e['err_max_deg']:.3f}°, "
            f"e_p máx. {e['e_p_max_mm']:.2f} mm, error final {r.get('error_final_max_deg', float('nan')):.3f}°.",
            "ok",
        )

    def _abortar(self, motivo: str):
        """Detener todo, dejar el robot quieto y desarmar. Se registra el motivo."""
        en_ejec = self.ejec
        self.ejec = None
        self.jog = None
        try:
            self.backend.mantener(None)
        except Exception as ex:  # noqa: BLE001
            motivo += f" (además falló mantener: {ex})"
        q_med, _ = self.backend.leer()
        if q_med is not None:
            self.q_ref = np.asarray(q_med, dtype=float)
        self.armado = False
        self.motivo_desarmado = motivo
        if en_ejec is not None:
            r = {
                "plan_id": en_ejec["plan_id"],
                "descripcion": en_ejec["descripcion"],
                "estado": "abortado",
                "motivo": motivo,
                "t_aborto_s": en_ejec.get("t", 0.0),
                "err_seguimiento_max_deg": en_ejec["err_max_deg"],
                "e_p_seguimiento_max_mm": en_ejec["e_p_max_mm"],
            }
            self.ultimo_resumen = r
            self.reg.agregar_resumen(r)
        self.reg.evento("aborto", motivo=motivo)
        self.avisar("ABORTO: " + motivo, "error")
        if self.secuencia is not None and self.secuencia.get("estado") == "corriendo":
            self.secuencia["estado"] = "detenida"
            self.secuencia["texto"] = "Detenida por aborto: " + motivo

    def parar(self, motivo="botón PARAR"):
        with self.lock:
            if self.secuencia is not None and self.secuencia.get("estado") in ("corriendo", "esperando"):
                self._sec_cmd = "cancelar"
            if self.ejec is not None or self.jog is not None:
                self._abortar(motivo)
            else:
                self.backend.mantener(None)
                self.avisar(f"PARAR ({motivo}): no había movimiento en curso.", "aviso")

    # Jog con flechas
    def orden_jog(self, accion: str, modo: str = "junta", eje: int = 1, direccion: int = 1) -> tuple[bool, str]:
        with self.lock:
            now = time.monotonic()
            self.pulso_t = now
            if accion == "soltar":
                if self.jog is not None:
                    self._parar_jog("tecla soltada")
                return True, "ok"
            if not self.armado:
                return False, "Para mover con flechas primero hay que ARMAR."
            if self.ejec is not None:
                return False, "Espera a que termine la trayectoria."
            if self.secuencia is not None and self.secuencia.get("estado") in ("corriendo", "esperando"):
                return False, "Hay una secuencia en curso."
            mismo = self.jog is not None and self.jog["modo"] == modo and self.jog["eje"] == eje and self.jog["dir"] == direccion
            if mismo:
                self.jog["t_ultimo"] = now
                return True, "ok"
            q_med, _ = self._q_medida()
            if q_med is None:
                return False, "No llega la posición del robot."
            base = np.asarray(q_med, dtype=float) if self.jog is None else self.q_ref
            self.q_ref = base.copy()
            self.jog = {
                "modo": modo,
                "eje": int(eje),
                "dir": int(direccion),
                "t_ultimo": now,
                "t_inicio": now,
                "T": self.robot.fk_tcp(self.q_ref),
                "n": 0,
            }
            nombre = (
                f"J{eje} {'+' if direccion > 0 else '−'}"
                if modo == "junta"
                else f"{'XYZ'[eje - 1]} {'+' if direccion > 0 else '−'}"
            )
            self.reg.evento("jog_inicio", modo=modo, eje=eje, dir=direccion)
            self.avisar(f"Jog {modo}: {nombre}", "info")
            return True, "ok"

    def _limites_jog(self):
        s = self.cfg["seguridad"]
        m = self.modo()
        vj = s["jog_junta_deg_s"]
        vc = s["jog_cartesiano_mm_s"]
        if m["exige_go"]:
            vj = min(vj, s["jog_real_junta_max_deg_s"])
            vc = min(vc, s["jog_real_cartesiano_max_mm_s"])
        f = self.vel_pct / max(1.0, self.cfg["seguridad"]["velocidad_inicial_pct"])
        f = min(1.0, max(0.2, f))
        return vj * f, vc * f

    def _paso_jog(self, dt_c):
        j = self.jog
        vj, vc = self._limites_jog()
        margen = math.radians(self.cfg["ik"]["margen_limite_deg"])
        q = self.q_ref.copy()
        if j["modo"] == "junta":
            i = j["eje"] - 1
            q[i] += j["dir"] * math.radians(vj) * dt_c
            lo, hi = self.robot.limites[i]
            if not (lo + margen <= q[i] <= hi - margen):
                self._parar_jog(f"J{i + 1} llegó al margen de su límite")
                return
        else:
            T = j["T"].copy()
            T[j["eje"] - 1, 3] += j["dir"] * vc / 1000.0 * dt_c
            if T[2, 3] < self.cfg["seguridad"]["z_min_m"]:
                self._parar_jog("el TCP llegó a la altura mínima")
                return
            res = pp.resolver(self.robot, T, q)
            ik = self.cfg["ik"]
            mejor, _ = pp.elegir_rama(res["ramas"], ik["tol_e_p_m"], ik["tol_e_R_rad"], margen, ik["sigma_min"])
            if mejor is None:
                self._parar_jog("la pose siguiente no tiene rama válida (límite, alcance o singularidad)")
                return
            q_new = np.array(mejor.q)
            vel = np.degrees(np.abs(q_new - q)) / dt_c
            if vel.max() > 45.0:
                self._parar_jog(
                    f"para seguir en línea recta J{int(np.argmax(vel)) + 1} tendría que girar a {vel.max():.0f}°/s: "
                    "estás cerca de una singularidad"
                )
                return
            j["T"] = T
            q = q_new
            if j["n"] % 10 == 0:
                self.ik_vivo = {
                    "ramas": [r.a_dict() for r in res["ramas"]],
                    "elegida": mejor.indice,
                    "pose": describir_pose(T),
                }
        if min(self.robot.distancia_a_limites(q)) < margen or self.robot.puntos_cuerpo(q)[1:, 2].min() < self.cfg["seguridad"]["z_min_m"]:
            self._parar_jog("se llegó a un límite de seguridad (junta o altura)")
            return
        self.q_ref = q
        j["n"] += 1
        if j["n"] % 2 == 0:
            self.backend.enviar_punto(self.q_ref, 0.12)

    def _parar_jog(self, motivo):
        if self.jog is None:
            return
        self.jog = None
        self.backend.mantener(self.q_ref)
        self.reg.evento("jog_fin", motivo=motivo)
        self.avisar(f"Jog detenido: {motivo}.", "info")

    # Secuencia de demostración
    def iniciar_secuencia(self, pausar_entre_pasos: bool) -> tuple[bool, str]:
        with self.lock:
            if not self.armado:
                return False, "Primero hay que ARMAR."
            if self.ejec is not None or self.jog is not None:
                return False, "El robot se está moviendo."
            if self.secuencia is not None and self.secuencia.get("estado") in ("corriendo", "esperando"):
                return False, "Ya hay una secuencia en curso."
            pasos = pasos_demo()
            self.secuencia = {
                "nombre": "Secuencia de demostración",
                "paso": 0,
                "total": len(pasos),
                "estado": "corriendo",
                "texto": "Iniciando",
                "titulo": "",
                "explicacion": "",
                "pausar": pausar_entre_pasos,
                "resultados": [],
            }
            self._sec_cmd = None
        threading.Thread(target=self._correr_secuencia, args=(pasos,), daemon=True).start()
        return True, "Secuencia iniciada."

    def continuar_secuencia(self):
        with self.lock:
            self._sec_cmd = "continuar"

    def _esperar_fin(self):
        while True:
            time.sleep(0.05)
            with self.lock:
                if self.ejec is None:
                    return
                if self._sec_cmd == "cancelar":
                    return

    def _correr_secuencia(self, pasos):
        sec = self.secuencia
        for i, paso in enumerate(pasos):
            with self.lock:
                if self._sec_cmd == "cancelar" or sec["estado"] == "detenida":
                    sec["estado"] = "detenida"
                    sec["texto"] = "Secuencia cancelada."
                    self.avisar("Secuencia cancelada.", "aviso")
                    return
                sec.update({"paso": i + 1, "titulo": paso["titulo"], "explicacion": paso["explicacion"], "texto": "Calculando"})
            try:
                if paso["tipo"] == "pose":
                    plan = self.calcular_pose(paso["pose"], paso.get("vel_cap_deg_s"))
                else:
                    plan = self.calcular_articular(paso["q_deg"], paso["titulo"], paso.get("vel_cap_deg_s"))
            except Exception as e:  # noqa: BLE001
                with self.lock:
                    sec["estado"] = "detenida"
                    sec["texto"] = f"Error al calcular: {e}"
                return
            if not plan["ejecutable"]:
                with self.lock:
                    sec["estado"] = "detenida"
                    sec["texto"] = "El paso no pasó la revisión: " + "; ".join(plan["problemas"])
                    self.avisar(sec["texto"], "error")
                return
            if sec["pausar"]:
                with self.lock:
                    sec["estado"] = "esperando"
                    sec["texto"] = "Revisa el cálculo y pulsa CONTINUAR (o la tecla C)."
                    self._sec_cmd = None
                while True:
                    time.sleep(0.05)
                    with self.lock:
                        if self._sec_cmd in ("continuar", "cancelar"):
                            break
                with self.lock:
                    if self._sec_cmd == "cancelar":
                        sec["estado"] = "detenida"
                        sec["texto"] = "Secuencia cancelada."
                        return
                    sec["estado"] = "corriendo"
            else:
                time.sleep(1.2)
            ok, msg = self.ejecutar()
            if not ok:
                with self.lock:
                    sec["estado"] = "detenida"
                    sec["texto"] = "No se pudo ejecutar: " + msg
                    self.avisar(sec["texto"], "error")
                return
            with self.lock:
                sec["texto"] = "Moviendo"
            self._esperar_fin()
            with self.lock:
                if sec["estado"] == "detenida":
                    return
                r = dict(self.ultimo_resumen or {})
                r["titulo"] = paso["titulo"]
                r["grupo"] = paso.get("grupo", "")
                sec["resultados"].append(r)
            time.sleep(0.8)
        with self.lock:
            sec["estado"] = "terminada"
            sec["texto"] = "Secuencia completa."
            self.reg.evento("secuencia_fin", resultados=sec["resultados"])
            self.avisar("Secuencia de demostración COMPLETA.", "ok")

    # Estado para la interfaz
    def estado(self) -> dict:
        with self.lock:
            v = dict(self.vivo)
            v["modo"] = self.modo()
            v["armado"] = self.armado
            v["motivo_desarmado"] = self.motivo_desarmado
            v["vel_pct"] = self.vel_pct
            v["plan_id"] = self.plan["id"] if self.plan else None
            if self.ejec is not None:
                e = self.ejec
                v["ejec"] = {
                    "plan_id": e["plan_id"],
                    "t": e["t"],
                    "tf": e["tray"].tf,
                    "progreso": min(1.0, e["t"] / max(e["tray"].tf, 1e-6)),
                    "err_max_deg": e["err_max_deg"],
                    "e_p_max_mm": e["e_p_max_mm"],
                }
            else:
                v["ejec"] = None
            v["jog"] = None if self.jog is None else {k: self.jog[k] for k in ("modo", "eje", "dir")}
            v["ik_vivo"] = self.ik_vivo if self.jog is not None and self.jog["modo"] == "cartesiano" else None
            v["secuencia"] = None if self.secuencia is None else {k: val for k, val in self.secuencia.items()}
            v["eventos"] = list(self.eventos)[-25:]
            v["ultimo_resumen"] = self.ultimo_resumen
            v["t"] = time.monotonic() - self.t0
            v["sesion"] = self.reg.carpeta
            return v


def pose_texto(p: dict) -> str:
    return f"({p['x']:.0f}, {p['y']:.0f}, {p['z']:.0f}) mm, Rz {p['rz']:.0f}° Ry {p['ry']:.0f}° Rx {p['rx']:.0f}°"


def publico(plan: dict) -> dict:
    return {k: v for k, v in plan.items() if not k.startswith("_")}


def validar_go(go: dict | None) -> tuple[bool, str]:
    """El archivo GO lo llena el grupo. La consola solo comprueba que esté completo y sea de hoy."""
    if not go:
        return False, "No se cargó ningún archivo GO (usa --go GO_VEVAS.json)."
    hoy = dt.date.today().isoformat()
    if go.get("fecha") != hoy:
        return False, f"El GO es de la fecha {go.get('fecha')} y hoy es {hoy}: llénalo de nuevo."
    for campo in ("autoriza", "operador_pendant", "responsable_consola", "alcance"):
        if not str(go.get(campo, "")).strip() or "COMPLETAR" in str(go.get(campo, "")):
            return False, f"Falta completar el campo '{campo}' del GO."
    pend = [k for k, v in go.get("checklist", {}).items() if v is not True]
    if pend:
        return False, "Ítems del checklist GO sin confirmar: " + ", ".join(pend)
    if go.get("decision") != "GO":
        return False, "La decisión del archivo GO no es 'GO'."
    return True, "GO válido."
