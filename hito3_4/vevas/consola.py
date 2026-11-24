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
import json
import math
import sys
import threading
import time
from collections import deque

import numpy as np

from . import pieper as pp
from .jacobiano import analisis_singularidad
from .robot_irb120 import (
    LIMITES_DEG,
    VEL_MAX_DEG,
    Robot,
    describir_pose,
    error_orientacion,
    error_posicion,
    pose_desde_xyz_euler,
)
from .poses_guardadas import AlmacenPoses
from .secuencias import POSTURA_INICIAL_DEG, pasos_demo
from .revision import nivel_singularidad, problemas_de, revisar_trayectoria
from .rws_cliente import CancelacionOrden
from .seguridad import semaforo
from .trayectoria import comprobar_continuidad, planificar, planificar_suave, planificar_camino

FRASE_REAL = "MOVER ROBOT REAL"
GRAD = 180.0 / math.pi
FIGURA_REAL_MAX_MM = 80.0   # tamaño máximo de una figura con el robot real (se puede bajar en config: seguridad.figura_real_max_mm)
FIGURAS = ("circulo", "corazon", "cuadrado", "triangulo", "saludo")
MOTIVO_IR_A_POSE = ("Ir a una pose guardada con el robot real por RWS exige C02 verificado "
                    "(velocidad por junta acotada en el RAPID).")
MOTIVO_SOLO_SIM = ("Disponible en SIM y, con el robot real, solo por RWS con C02 verificado "
                   "(velocidad por junta acotada en el RAPID).")


def _m16(T) -> list:
    return [round(float(v), 6) for v in np.asarray(T).reshape(-1)]


class Consola:
    def __init__(self, cfg: dict, backend, registro, permitir_real: bool = False, go: dict | None = None, ruta_go=None,
                 ruta_poses: str | None = None):
        self.cfg = cfg
        rc = cfg["robot"]
        self.robot = Robot(tcp_xyz=tuple(rc["tcp_xyz_m"]), tcp_rpy=tuple(rc["tcp_rpy_rad"]))
        self.backend = backend
        self.reg = registro
        self.permitir_real = permitir_real
        self.go = go
        self.ruta_go = ruta_go
        self.poses = AlmacenPoses(ruta_poses, cfg.get("robot", {}).get("limites_deg") or LIMITES_DEG)
        self._t_revision_go = 0.0
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
        self._hilo = None
        self.ultimo_resumen = None
        self.envio_pendiente = False
        self._cancelar_envio = CancelacionOrden()

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
        es_real = (self.backend.tipo in ("ros", "rws")) and (real is True or real is None)
        info["exige_go"] = es_real
        info["permitir_real"] = self.permitir_real
        info["vel_max_pct"] = self._tope_vel_pct(es_real)
        info["seguimiento_por_llegada"] = bool(getattr(self.backend, "seguimiento_por_llegada", False))
        permitidas = self._movimientos_guiados_permitidos()
        info["figuras_permitidas"] = permitidas
        info["figuras_motivo"] = None if permitidas else MOTIVO_SOLO_SIM
        info["figura_max_mm"] = 100.0 if self.backend.tipo == "sim" else self._figura_real_max_mm()
        ir = self._ir_a_pose_permitido()
        info["ir_a_pose_permitido"] = ir
        info["ir_a_pose_motivo"] = None if ir else MOTIVO_IR_A_POSE
        return info

    def _ir_a_pose_permitido(self) -> bool:
        """Mismas reglas que «Volver a cero»: SIM y ROS como siempre; RWS solo con C02 verificado."""
        return self.backend.tipo != "rws" or self._rws_c02()

    def _rws_c02(self) -> bool:
        """Robot real por RWS con MoveAbsJ y velocidad por junta acotada y verificada (C02)."""
        return bool(self.backend.tipo == "rws" and getattr(self.backend, "velocidad_articular_verificada", False)
                    and getattr(self.backend, "seguimiento_por_llegada", False))

    def _movimientos_guiados_permitidos(self) -> bool:
        """Preparar J5 y las figuras: SIM, o RWS con C02. ROS real y RWS sin C02 quedan fuera."""
        return self.backend.tipo == "sim" or self._rws_c02()

    def _figura_real_max_mm(self) -> float:
        v = self.cfg.get("seguridad", {}).get("figura_real_max_mm", FIGURA_REAL_MAX_MM)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            return FIGURA_REAL_MAX_MM
        return float(min(v, FIGURA_REAL_MAX_MM))

    def _tope_vel_pct(self, es_real: bool) -> float:
        """Tope de velocidad [% de la hoja de datos]. Con robot real manda el menor entre config.json y el GO."""
        s = self.cfg["seguridad"]
        if not es_real:
            return float(s["velocidad_sim_max_pct"])
        tope = float(s["velocidad_real_max_pct"])
        v_go = (self.go or {}).get("velocidad_max_pct")
        if isinstance(v_go, (int, float)) and not isinstance(v_go, bool) and v_go > 0:
            tope = min(tope, float(v_go))
        return tope

    def _validar_go_sesion(self):
        if self.ruta_go is None:
            return validar_go(self.go)
        try:
            with open(self.ruta_go, encoding="utf-8") as f:
                actual = json.load(f)
        except (OSError, ValueError) as ex:
            return False, f"No se puede leer el GO de la sesión: {ex}"
        ok, msg = validar_go(actual)
        if ok and actual != self.go:
            return False, "El GO cambió desde el inicio. Revisá la sesión y su entorno."
        return ok, msg

    def _comprobar_go_orden(self):
        ok, msg = self._validar_go_sesion()
        if not ok:
            raise ValueError(msg)

    def _q_medida(self):
        q, t = self.backend.leer()
        return q, t

    def pulso(self):
        self.pulso_t = time.monotonic()

    # Bucle principal (50 Hz)
    def iniciar(self):
        if self._hilo is not None and self._hilo.is_alive():
            return
        self.activo = True
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()

    def detener(self):
        self.activo = False
        if self._hilo is not None and self._hilo is not threading.current_thread():
            self._hilo.join(timeout=2.0)

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

            vigilado = moviendo or (self.envio_pendiente and not self._cancelar_envio.is_set())
            if vigilado and self.ruta_go and now - self._t_revision_go >= 0.2:
                self._t_revision_go = now
                ok_go, msg_go = self._validar_go_sesion()
                if not ok_go:
                    self._abortar(msg_go)
            if vigilado and edad > s["watchdog_estado_s"]:
                self._abortar(f"Sin datos del robot hace {edad:.2f} s (watchdog {s['watchdog_estado_s']} s).")
            elif vigilado and now - self.pulso_t > s["watchdog_interfaz_s"]:
                self._abortar("La interfaz dejó de responder (watchdog del navegador). Se detuvo por seguridad.")

            # Backends cuyo robot corre a su propia velocidad (RWS/MoveAbsJ): el tiempo del plan LSPB no se
            # cumple, así que no hay error de seguimiento temporal; se termina por llegada informada.
            por_llegada = bool(getattr(self.backend, "seguimiento_por_llegada", False))
            if self.ejec is not None and por_llegada:
                e = self.ejec
                e["t"] = now - e["t0"]
                self.q_ref = None
                exito, detalle = self.backend.resultado_trayectoria()
                if exito is False:
                    self._abortar(f"El robot no completó la trayectoria: {detalle}")
                elif exito is True:
                    err_fin = None if q_med is None else float(np.max(np.abs(np.degrees(np.asarray(q_med) - e["q_obj"]))))
                    if err_fin is None or err_fin > s["inicio_max_deg"]:
                        self._abortar(
                            "El backend informó llegada, pero la postura medida difiere del destino en "
                            + ("un valor desconocido." if err_fin is None else f"{err_fin:.2f}° (> {s['inicio_max_deg']}°).")
                        )
                    else:
                        self._terminar_ejecucion(q_med)
                elif e["t"] > e["limite_s"]:
                    self._abortar(f"Tiempo límite ({e['limite_s']:.0f} s) sin que el robot informe la llegada.")
            elif self.ejec is not None:
                e = self.ejec
                t = now - e["t0"]
                q_cmd, _, _ = e["tray"].evaluar(t)
                self.q_ref = q_cmd
                e["t"] = t
                resultado_ros = getattr(self.backend, "resultado_trayectoria", None)
                if resultado_ros is not None:
                    exito, detalle = resultado_ros()
                    if exito is False:
                        self._abortar(f"La acción del controlador no terminó con éxito: {detalle}")
                    elif t >= e["tray"].tf + 0.4:
                        if exito is True:
                            self._terminar_ejecucion(q_med)
                        elif t >= e["tray"].tf + 5.0:
                            self._abortar("No llegó resultado de la acción ROS en los 5 s posteriores al plan.")
                elif t >= e["tray"].tf + 0.4:
                    if self.backend.tipo != "sim" or self._llegada_sim_verificada(e, q_med):
                        self._terminar_ejecucion(q_med)
                    elif t >= e["tray"].tf + 5.0:
                        self._abortar("SIM agotó el tiempo de llegada sin confirmar el objetivo dentro de tolerancia.")
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
                v["sing"]["nivel"] = nivel_singularidad(sing["sigma_min"], float(self.cfg["ik"]["sigma_min"]))
                if self._ciclo_n % 10 == 0 or "J" not in (self.vivo.get("sing") or {}):
                    v["sing"]["J"] = [[round(x, 5) for x in fila] for fila in sing["J"]]
                else:
                    v["sing"]["J"] = self.vivo["sing"]["J"]
                v["lim_dist_deg"] = [d * GRAD for d in self.robot.distancia_a_limites(q_med)]
                if self._ciclo_n % 10 == 0:
                    v["fk_vivo"] = {"A": [_m16(A) for A in self.robot.matrices_eslabon(q_med)], "T06": _m16(M[6])}
                else:
                    v["fk_vivo"] = self.vivo.get("fk_vivo")
                if self.q_ref is not None and not por_llegada:
                    # Medida y referencia deben corresponder al mismo instante.
                    # Comparar un estado recibido antes con el objetivo de ahora
                    # convertía la latencia de muestreo en un error ficticio.
                    q_comparacion = self.q_ref
                    if self.ejec is not None:
                        q_comparacion = self.ejec["tray"].evaluar(t_est - self.ejec["t0"])[0]
                    T_ref = self.robot.fk_tcp(q_comparacion)
                    err = (np.asarray(q_med) - np.asarray(q_comparacion)) * GRAD
                    e_p = error_posicion(T_med, T_ref) * 1000.0
                    e_R = error_orientacion(T_med[:3, :3], T_ref[:3, :3]) * GRAD
                    v["q_ref_deg"] = [float(a) * GRAD for a in q_comparacion]
                    v["err_deg"] = [float(x) for x in err]
                    v["e_p_mm"] = e_p
                    v["e_R_deg"] = e_R
                    v["tcp_ref"] = describir_pose(T_ref)
                    v["marcos_ref"] = [_m16(T) for T in self.robot.marcos(q_comparacion)] + [_m16(T_ref)]
                    v["referencia_sincronizada_con_estado"] = self.ejec is not None
                    v["semaforo"] = semaforo(float(np.max(np.abs(err))), e_p, self.cfg)
                    if moviendo:
                        emax = float(np.max(np.abs(err)))
                        if self.ejec is not None:
                            self.ejec["err_max_deg"] = max(self.ejec["err_max_deg"], emax)
                            self.ejec["e_p_max_mm"] = max(self.ejec["e_p_max_mm"], e_p)
                            self.ejec["e_R_max_deg"] = max(self.ejec["e_R_max_deg"], e_R)
                        if not por_llegada and emax > s["error_seguimiento_max_deg"]:
                            j = int(np.argmax(np.abs(err))) + 1
                            self._abortar(
                                f"Error de seguimiento en J{j}: {emax:.2f}° > {s['error_seguimiento_max_deg']}° permitido."
                            )
                if vigilado and min(self.robot.distancia_a_limites(q_med)) < math.radians(1.0):
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
                ok, msg = self._validar_go_sesion()
                if not ok:
                    return False, msg
                faltan = [k for k in ("supervisor", "paro", "area", "velocidad") if not isinstance(checklist, dict) or checklist.get(k) is not True]
                if faltan:
                    return False, "Falta marcar en la lista: " + ", ".join(faltan)
                if (frase or "").strip().upper() != FRASE_REAL:
                    return False, f'Escribe exactamente la frase "{FRASE_REAL}" para armar el robot real.'
            q_med, t_est = self._q_medida()
            if q_med is None or time.monotonic() - t_est > self.cfg["seguridad"]["watchdog_estado_s"]:
                return False, "No se arma: no llega el estado del robot o está desactualizado."
            self.armado = True
            self.vel_pct = min(self.vel_pct, m["vel_max_pct"])
            self.avisar(f"Movimiento ARMADO ({m['texto']}). Velocidad {self.vel_pct:.0f} %.", "ok", checklist=checklist)
            return True, "Armado."

    def desarmar(self, motivo: str = "desarmado por el usuario"):
        with self.lock:
            if self.envio_pendiente or self.ejec is not None or self.jog is not None:
                self._abortar(motivo)
            self.armado = False
            self.motivo_desarmado = motivo
            self.avisar(f"Movimiento DESARMADO: {motivo}", "aviso")

    def fijar_velocidad(self, pct: float) -> float:
        with self.lock:
            m = self.modo()
            velocidad = float(max(1.0, min(pct, m["vel_max_pct"])))
            if self.backend.tipo == "sim" and velocidad != self.vel_pct:
                self.plan = None
            self.vel_pct = velocidad
            return self.vel_pct

    # Cálculo de planes
    def _origen_para_nuevo_plan(self, *, exigir_fresco=False):
        # Una revisión nueva invalida el envío aún pendiente del plan mostrado previamente.
        if self.envio_pendiente and not self._cancelar_envio.is_set():
            self._abortar("Se solicitó otro plan durante el envío.")
        q_med, t_est = self._q_medida()
        if q_med is None:
            raise RuntimeError("Todavía no llega la posición del robot (/joint_states).")
        if exigir_fresco and time.monotonic() - t_est > self.cfg["seguridad"]["watchdog_estado_s"]:
            raise RuntimeError("Esperá una postura actual " + ("de SIM." if self.backend.tipo == "sim" else "del robot."))
        return np.asarray(q_med, dtype=float)

    def _factor(self, vel_cap_deg_s: float | None) -> float:
        f = self.vel_pct / 100.0
        if vel_cap_deg_s is not None:
            f = min(f, vel_cap_deg_s / min(VEL_MAX_DEG))
        return f

    def fijar_perfil(self, perfil: str):
        if perfil not in ("suave", "lspb"):
            raise ValueError("Elegí el perfil suave o lspb.")
        with self.lock:
            if self.ejec is not None or self.jog is not None or self.envio_pendiente:
                raise RuntimeError("Esperá a que termine el movimiento para cambiar el perfil.")
            self.cfg["trayectoria"]["perfil"] = perfil
            self.plan = None
            self.reg.evento("perfil", perfil=perfil)
        return perfil

    def _planificar_punto(self, q0, q_obj, vel_cap_deg_s=None):
        tcfg = self.cfg["trayectoria"]
        creador = planificar if tcfg.get("perfil", "suave") == "lspb" else planificar_suave
        return creador(q0, q_obj, self.robot.vel_max, self._factor(vel_cap_deg_s),
                       tcfg["t_acel_s"], tcfg["kappa"], tcfg["t_min_s"])

    def _armar_plan(self, tipo, descripcion, q0, q_obj, extra: dict, vel_cap_deg_s=None, permitir_singular_destino=False, *, tray=None) -> dict:
        tray = tray if tray is not None else self._planificar_punto(q0, q_obj, vel_cap_deg_s)
        ts, Q, QD, _ = tray.muestrear(0.02)
        m = self.modo()
        revision = revisar_trayectoria(self.robot, ts, Q, QD, q0, self.cfg, m["vel_max_pct"], m["exige_go"],
                                       permitir_singular_destino=permitir_singular_destino,
                                       velocidad_articular_verificada=m.get("velocidad_articular_verificada", True))
        problemas = problemas_de(revision)
        T_obj = self.robot.fk_tcp(q_obj)
        sing = analisis_singularidad(self.robot, q_obj)
        # vista previa: marcos de ~40 muestras y el camino del TCP (siempre incluye la muestra de la falla)
        idx = np.linspace(0, len(ts) - 1, min(len(ts), 150)).astype(int)
        falla = revision["falla"]
        if falla is not None:
            idx = np.unique(np.append(idx, falla["k"]))
            k_f = int(falla["k"])
            falla["marcos"] = [_m16(A) for A in self.robot.marcos(Q[k_f])] + [_m16(self.robot.fk_tcp(Q[k_f]))]
            falla["tcp"] = [float(v) for v in self.robot.fk_tcp(Q[k_f])[:3, 3]]
            falla["k_preview"] = int(np.where(idx == k_f)[0][0])
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
            "preview": {"frames": frames, "camino": camino, "t": [float(ts[k]) for k in idx],
                        "puntos": [camino[0], camino[-1]]},
            "problemas": problemas,
            "revision": revision,
            "ejecutable": not revision["bloquea_envio"],
            "vel_pct": self.vel_pct,
            "vel_cap_deg_s": vel_cap_deg_s,
        }
        plan.update(extra)
        plan["_permitir_singular_destino"] = permitir_singular_destino
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
            q0 = self._origen_para_nuevo_plan()
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
            q0 = self._origen_para_nuevo_plan()
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

    def calcular_articular(self, q_deg, descripcion=None, vel_cap_deg_s=None, permitir_singular_destino=False,
                           *, preparar_j5=False, buscar_retorno=False, extra: dict | None = None) -> dict:
        with self.lock:
            q0 = self._origen_para_nuevo_plan(exigir_fresco=True) if preparar_j5 else self._origen_para_nuevo_plan()
            if preparar_j5:
                q_obj = q0.copy()
                q_obj[4] = np.radians(30.0)
            else:
                q_obj = np.radians(np.asarray(q_deg, dtype=float))
            fuera = [i + 1 for i, ok in enumerate(self.robot.dentro_de_limites(q_obj)) if not ok]
            desc = descripcion or "Juntas " + ", ".join(f"{v:.1f}" for v in q_deg) + " °"
            extra = dict(extra or {})
            if preparar_j5:
                extra["preparar_j5"] = True
            plan = self._armar_plan("articular", desc, q0, q_obj, extra, vel_cap_deg_s, permitir_singular_destino)
            if buscar_retorno and not plan["ejecutable"]:
                tcfg = self.cfg["trayectoria"]
                for punto_deg in (self.cfg.get("postura_inicial_deg", POSTURA_INICIAL_DEG),
                                  [0, -20, 30, 0, 30, 0], [0, -20, 30, 0, -30, 0]):
                    punto = np.radians(punto_deg)
                    if np.allclose(punto, q0) or np.allclose(punto, q_obj):
                        continue
                    tray = planificar_camino([q0, punto, q_obj], self.robot.vel_max,
                                              self._factor(None), tcfg["t_acel_s"], tcfg["t_min_s"])
                    alternativa = self._armar_plan(
                        "articular", desc + " · con punto intermedio", q0, q_obj,
                        {**extra, "punto_intermedio_deg": list(punto_deg)},
                        permitir_singular_destino=True, tray=tray)
                    if alternativa["ejecutable"]:
                        plan = alternativa
                        break
                if not plan["ejecutable"]:
                    plan["problemas"].append(
                        "Ninguna alternativa evaluada superó la revisión. Probá Postura inicial con su propia revisión.")
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

    def calcular_home(self, destino: str = "cero") -> dict:
        """Plan articular a q = 0 («cero») o a la postura inicial. Solo en simulación puede mostrarse la llegada singular a q = 0 como aviso."""
        if destino == "cero":
            simulacion = self.backend.tipo == "sim"
            rws_c02 = self._rws_c02()
            permitir = True if simulacion else ("articular_real" if rws_c02 else False)
            return self.calcular_articular([0.0] * 6, "Volver a cero",
                                           permitir_singular_destino=permitir, buscar_retorno=simulacion)
        if destino == "j5":
            # Con el robot real solo por RWS con C02: MoveAbsJ con velocidad por junta acotada. Salir de la
            # singularidad de muñeca (J5 ≈ 0) alejándose la revisión lo da como «aviso»; entrar sigue rechazado.
            if not self._movimientos_guiados_permitidos():
                raise ValueError("Preparar J5 a +30° está disponible en SIM y, con el robot real, solo por RWS "
                                 "con C02 verificado (velocidad por junta acotada). Con ROS real o sin C02 movelo "
                                 "desde el FlexPendant.")
            return self.calcular_articular(None, "Preparar J5 a +30°", preparar_j5=True)
        if destino == "inicial":
            return self.calcular_articular(list(self.cfg.get("postura_inicial_deg", POSTURA_INICIAL_DEG)),
                                           "Volver a la postura inicial")
        raise ValueError("destino debe ser 'cero', 'inicial' o 'j5'.")

    def calcular_figura(self, figura, tamano_mm=25, plano="XY") -> dict:
        """Recorrido completo de demostración; cada punto pasa por IK y revisión.

        Disponible en SIM y con el robot real por RWS con C02 verificado (mismas
        compuertas de siempre: revisión previa, GO, armado y confirmación). El
        origen es el TCP actual y la orientación se conserva. El saludo usa
        posturas articulares de muñeca y se identifica como tal, sin inventar IK.
        """
        from .figuras import generar_figura, paradas_figura, plano_de_vista
        if not self._movimientos_guiados_permitidos():
            raise RuntimeError("Figuras: " + MOTIVO_SOLO_SIM + " Con ROS real o sin C02 quedan deshabilitadas.")
        simulacion = self.backend.tipo == "sim"
        nombre = _nombre_figura(figura)
        if not simulacion and isinstance(tamano_mm, (int, float)) and not isinstance(tamano_mm, bool):
            tope_mm = self._figura_real_max_mm()
            if tamano_mm > tope_mm:
                raise ValueError(f"Con el robot real el tamaño máximo de una figura es {tope_mm:g} mm.")
        # El generador también valida tamaño, plano, finitud y tipos.
        offsets = generar_figura("circulo" if nombre == "saludo" else nombre, tamano_mm, plano)
        plano = plano.strip().upper()
        with self.lock:
            if self.ejec is not None or self.jog is not None:
                raise RuntimeError("Esperá a que termine el movimiento para calcular una figura.")
            q0 = self._origen_para_nuevo_plan(exigir_fresco=not simulacion)
            muneca_singular = analisis_singularidad(self.robot, q0)["muneca_sin_q5"] < 0.09
            sugerir_j5 = ("El robot está con J5 ≈ 0 (singularidad de muñeca): usá primero «Preparar J5 a +30°» "
                          "y después calculá la figura.")
            if muneca_singular and not simulacion:
                raise RuntimeError(sugerir_j5)
            origen = self.robot.fk_tcp(q0)
            objetivos = origen[:3, 3] + offsets
            juntas, ramas, error_ik = [q0.copy()], [], None
            ep_max, er_max = 0.0, 0.0
            if nombre == "saludo":
                Q = _juntas_saludo(q0, tamano_mm, len(offsets))
                objetivos = np.array([self.robot.fk_tcp(q)[:3, 3] for q in Q])
            else:
                ik = self.cfg["ik"]
                for k, punto in enumerate(objetivos[1:], 1):
                    T = origen.copy()
                    T[:3, 3] = punto
                    res = pp.resolver(self.robot, T, juntas[-1])
                    pp.elegir_rama(res["ramas"], ik["tol_e_p_m"], ik["tol_e_R_rad"],
                                   math.radians(ik["margen_limite_deg"]), ik["sigma_min"])
                    validas = [r for r in res["ramas"] if r.valida and r.q is not None]
                    if not validas:
                        error_ik = (k, "No hay una rama de Pieper válida en ese punto.")
                        break
                    mejor = min(validas, key=lambda r: float(np.linalg.norm(np.asarray(r.q) - juntas[-1])))
                    q = np.array(mejor.q, dtype=float)
                    if float(np.degrees(np.abs(q - juntas[-1])).max()) > 12.0:
                        error_ik = (k, "La rama requiere un salto articular; se rechaza la discontinuidad.")
                        break
                    if k == len(objetivos) - 1 and np.max(np.abs(q - q0)) < 1e-6:
                        q = q0.copy()
                    ver = self.robot.fk_tcp(q)
                    ep_max = max(ep_max, error_posicion(ver, T))
                    er_max = max(er_max, error_orientacion(ver[:3, :3], T[:3, :3]))
                    juntas.append(q)
                    ramas.append(mejor.indice)
                Q = np.asarray(juntas)
            tcfg = self.cfg["trayectoria"]
            # Corazón: se detiene en la punta (cúspide). Cuadrado y triángulo: en cada
            # esquina, para conservar la figura sin redondearla. La misma función
            # alimenta la vista previa de puntos.
            paradas = paradas_figura(nombre, objetivos, limite=len(Q))
            tray = (planificar_camino(Q, self.robot.vel_max, self._factor(None),
                                     tcfg["t_acel_s"], tcfg["t_min_s"], paradas=paradas)
                    if len(Q) > 1 else self._planificar_punto(q0, q0))
            datos_figura = {"nombre": nombre, "tamano_mm": float(tamano_mm), "plano": plano,
                            "n_puntos": len(objetivos), "puntos_objetivo_m": objetivos.tolist(),
                            "ramas": ramas, "paradas": paradas,
                            "plano_vista": plano_de_vista(objetivos - objetivos[0], plano),
                            "metodo": "articular" if nombre == "saludo" else "Pieper con orientación constante",
                            "falla_punto": error_ik[0] if error_ik else None}
            visible = {"circulo": "Círculo", "corazon": "Corazón", "triangulo": "Triángulo"}.get(nombre, nombre.capitalize())
            plan = self._armar_plan("figura", f"{visible} · {tamano_mm:g} mm · "
                                    + ("saludo articular" if nombre == "saludo" else plano),
                                    q0, Q[-1], {"figura": datos_figura}, tray=tray)
            plan["preview"]["puntos"] = objetivos.tolist()
            plan["verificacion"] = {"e_p_m": ep_max, "e_R_rad": er_max}
            if error_ik:
                k, motivo = error_ik
                mensaje = (f"Punto {k + 1}/{len(objetivos)}: {motivo} "
                           + (sugerir_j5 if muneca_singular else "Probá un tamaño menor o una postura inicial distinta."))
                plan["problemas"].insert(0, mensaje)
                rev = plan["revision"]
                rev["estado"], rev["bloquea_envio"] = "rechazado", True
                rev["chequeos"].append({"id": "ik_figura", "nombre": "Continuidad de Pieper en la figura",
                                        "estado": "rechazado", "detalle": mensaje, "obligatorio": True})
                plan["ejecutable"] = False
            lim = np.asarray(self.robot.limites, dtype=float)
            fuera = [i + 1 for i in range(6) if np.any(Q[:, i] < lim[i, 0]) or np.any(Q[:, i] > lim[i, 1])]
            if fuera:
                plan["problemas"].insert(0, "La figura sale de los límites en J" + ", J".join(map(str, fuera)) + ".")
                plan["ejecutable"] = False
            if muneca_singular and not plan["ejecutable"] and not error_ik:
                plan["problemas"].append(sugerir_j5)
            self.plan = plan
            self._guardar_plan(plan)
            self.avisar(f"Plan {plan['id']}: {plan['descripcion']} · "
                        + (("revisado en SIM." if simulacion else "revisado para el robot real (RWS · C02); "
                            "falta armar y confirmar.") if plan["ejecutable"] else "revisá los rechazos antes de enviar."),
                        "ok" if plan["ejecutable"] else "aviso")
            return publico(plan)

    def vista_figura(self, figura, tamano_mm=25, plano="XY") -> dict:
        """Vista previa de una figura: muestras del TCP, paradas e inicio/fin. No calcula plan ni mueve nada.

        Usa la misma geometría que calcular_figura (vevas/figuras.py). Si llega la postura
        actual, devuelve también los puntos absolutos en la base (origen = TCP actual).
        """
        from .figuras import PLANOS, generar_figura, paradas_figura, plano_de_vista
        nombre = _nombre_figura(figura)
        offsets = generar_figura("circulo" if nombre == "saludo" else nombre, tamano_mm, plano)
        plano = plano.strip().upper()
        q_med, _ = self._q_medida()
        q0 = None if q_med is None else np.asarray(q_med, dtype=float)
        if q0 is not None and not np.all(np.isfinite(q0)):
            q0 = None
        absolutos = None
        if nombre == "saludo":
            if q0 is None:
                raise RuntimeError("El saludo depende de la postura actual: esperá el estado del robot.")
            absolutos = np.array([self.robot.fk_tcp(q)[:3, 3] for q in _juntas_saludo(q0, tamano_mm, len(offsets))])
            offsets = absolutos - absolutos[0]
        elif q0 is not None:
            absolutos = self.robot.fk_tcp(q0)[:3, 3] + offsets
        vista = plano_de_vista(offsets, plano)
        i, j = PLANOS[vista]
        simulacion = self.backend.tipo == "sim"
        tope = 100.0 if simulacion else self._figura_real_max_mm()
        return {
            "nombre": nombre,
            "tamano_mm": float(tamano_mm),
            "plano": plano,
            "plano_vista": vista,
            "ejes": [vista[0], vista[1]],
            "n_puntos": len(offsets),
            "offsets_mm": (offsets * 1000.0).tolist(),
            "puntos_2d_mm": (offsets[:, [i, j]] * 1000.0).tolist(),
            "paradas": paradas_figura(nombre, offsets),
            "inicio": 0,
            "fin": len(offsets) - 1,
            "origen_m": None if absolutos is None else absolutos[0].tolist(),
            "puntos_m": None if absolutos is None else absolutos.tolist(),
            "metodo": "articular" if nombre == "saludo" else "Pieper con orientación constante",
            "permitida": self._movimientos_guiados_permitidos(),
            "tope_mm": tope,
            "excede_tope": float(tamano_mm) > tope,
        }

    # Poses guardadas
    def listar_poses(self) -> dict:
        return self.poses.listar()

    def guardar_pose_actual(self, nombre) -> dict:
        """Guarda las 6 juntas medidas [°] y la pose del TCP por FK, con fecha."""
        with self.lock:
            if self.ejec is not None or self.jog is not None or self.envio_pendiente:
                raise RuntimeError("Esperá a que el robot esté quieto para guardar la pose.")
            q_med, t_est = self._q_medida()
            if q_med is None:
                raise RuntimeError("Todavía no llega la posición del robot.")
            if time.monotonic() - t_est > self.cfg["seguridad"]["watchdog_estado_s"]:
                raise RuntimeError("El estado del robot está desactualizado: esperá una lectura reciente.")
            q = np.asarray(q_med, dtype=float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise RuntimeError("La postura recibida no es válida (6 juntas finitas).")
        tcp = describir_pose(self.robot.fk_tcp(q))
        tcp.pop("cuaternion", None)
        pose = self.poses.guardar(nombre, [float(v) for v in np.degrees(q)], tcp)
        self.reg.evento("pose_guardada", nombre=pose["nombre"], q_deg=pose["q_deg"], tcp=pose["tcp"])
        self.avisar(f"Pose «{pose['nombre']}» guardada.", "ok")
        return pose

    def renombrar_pose(self, nombre, nuevo) -> dict:
        pose = self.poses.renombrar(nombre, nuevo)
        self.reg.evento("pose_renombrada", anterior=nombre, nombre=pose["nombre"])
        return pose

    def borrar_pose(self, nombre, confirmar=False) -> dict:
        if confirmar is not True:
            raise ValueError("Confirmá el borrado de la pose.")
        pose = self.poses.borrar(nombre)
        self.reg.evento("pose_borrada", nombre=pose["nombre"], q_deg=pose["q_deg"])
        self.avisar(f"Pose «{pose['nombre']}» borrada.", "info")
        return pose

    def calcular_pose_guardada(self, nombre) -> dict:
        """«Ir a pose»: plan articular revisado hacia una pose guardada.

        Mismas reglas que «Volver a cero»: SIM siempre; robot real por RWS solo con C02
        verificado; ROS como cualquier plan articular. Pasa por la revisión previa
        (límites, singularidades, colisiones) y después por GO, armado, checklist y
        confirmación; si la postura cambió antes de enviar, exige recalcular.
        """
        if not self._ir_a_pose_permitido():
            raise RuntimeError(MOTIVO_IR_A_POSE + " Sin C02, movelo desde el FlexPendant.")
        pose = self.poses.obtener(nombre)
        simulacion = self.backend.tipo == "sim"
        permitir = True if simulacion else ("articular_real" if self._rws_c02() else False)
        return self.calcular_articular(
            pose["q_deg"], f"Ir a pose «{pose['nombre']}»", permitir_singular_destino=permitir,
            buscar_retorno=simulacion, extra={"pose_guardada": dict(pose)})

    def ik_vivo_rapido(self, pose: dict, q_ref_deg=None) -> dict:
        """IK de consulta (sin plan, sin registro, sin lock): para el panel en vivo de la interfaz."""
        ik = self.cfg["ik"]
        sigma_umbral = float(ik["sigma_min"])
        margen = math.radians(ik["margen_limite_deg"])
        T_d = pose_desde_xyz_euler(pose["x"], pose["y"], pose["z"], pose["rz"], pose["ry"], pose["rx"])
        if q_ref_deg is not None:
            q_ref = np.radians(np.asarray(q_ref_deg, dtype=float))
        else:
            q_ref, _ = self._q_medida()
            q_ref = np.zeros(6) if q_ref is None else np.asarray(q_ref, dtype=float)
        res = pp.resolver(self.robot, T_d, q_ref)
        mejor, _ = pp.elegir_rama(res["ramas"], ik["tol_e_p_m"], ik["tol_e_R_rad"], margen, sigma_umbral)
        motivos = []
        ramas = []
        for r in res["ramas"]:
            d = r.a_dict()
            ramas.append({"indice": r.indice, "nombre": d["nombre"], "existe": r.existe, "valida": r.valida,
                          "q_deg": d.get("q_deg"),
                          "motivo": r.motivo_no_existe if not r.existe else "; ".join(r.motivos)})
        alcanzable = any(r.existe for r in res["ramas"])
        if not alcanzable:
            motivos.append("La pose está fuera del alcance del robot.")
        # rama de referencia para singularidad: la elegida o, si no hay, la más cercana que exista
        ref = mejor
        if ref is None:
            ex = [r for r in res["ramas"] if r.existe and r.q is not None]
            ref = min(ex, key=lambda r: r.distancia) if ex else None
        sigma_min = None
        sing = {"tipo": None, "cerca": False, "detalle": "Sin postura para evaluar."}
        marcos = ver = q_deg = delta = None
        nivel = "ok"
        if ref is not None:
            q = np.asarray(ref.q, dtype=float)
            a = analisis_singularidad(self.robot, q)
            sigma_min = a["sigma_min"]
            nivel = nivel_singularidad(sigma_min, sigma_umbral)
            ratios = {"muñeca": a["muneca_sin_q5"] / 0.09, "codo": a["codo_sin_delta"] / 0.09, "hombro": a["hombro_r_w_m"] / 0.03}
            tipo = min(ratios, key=ratios.get) if nivel != "ok" else None
            if nivel == "ok":
                det = f"σ_min = {sigma_min:.3f}: lejos de singularidades."
            else:
                det = (f"σ_min = {sigma_min:.3f} ({'singular' if nivel == 'singular' else 'cerca de singularidad'}, "
                       f"umbral {sigma_umbral:g}); singularidad de {tipo}.")
            sing = {"tipo": tipo, "cerca": nivel != "ok", "detalle": det}
            delta = float(np.degrees(np.abs(q - q_ref)).max())
        if mejor is not None:
            q = np.asarray(mejor.q, dtype=float)
            q_deg = [float(v) * GRAD for v in q]
            marcos = [_m16(T) for T in self.robot.marcos(q)] + [_m16(self.robot.fk_tcp(q))]
            T_fin = self.robot.fk_tcp(q)
            ver = {"e_p_mm": error_posicion(T_fin, T_d) * 1000.0, "e_R_deg": error_orientacion(T_fin[:3, :3], T_d[:3, :3]) * GRAD}
        elif alcanzable:
            n_ex = sum(1 for r in res["ramas"] if r.existe)
            motivos.append(f"Ninguna rama de la IK es válida ({n_ex} de 8 existen, pero todas fallan por límites articulares, "
                           "singularidad o error de verificación).")
        if nivel != "ok":
            motivos.append(f"Cerca de una singularidad: {sing['detalle']}")
        if ver is not None and (ver["e_p_mm"] > 1e-3 or ver["e_R_deg"] > 1e-3):
            motivos.append(f"La verificación FK no cierra (e_p = {ver['e_p_mm']:.4f} mm, e_R = {ver['e_R_deg']:.4f}°).")
        motivos = list(dict.fromkeys(motivos))
        admisible = bool(alcanzable and mejor is not None and nivel == "ok" and not motivos)
        return {"alcanzable": alcanzable, "ramas": ramas, "elegida": mejor.indice if mejor else None, "q_deg": q_deg,
                "marcos": marcos, "verificacion": ver, "sigma_min": sigma_min, "singular": sing,
                "delta_max_deg": delta if mejor is not None else None, "admisible": admisible, "motivos": motivos}

    def calcular_junta(self, junta: int, valor_deg: float, relativo: bool) -> dict:
        with self.lock:
            q0 = self._origen_para_nuevo_plan()
            q = [float(a) * GRAD for a in q0]
            j = int(junta) - 1
            q[j] = q[j] + valor_deg if relativo else valor_deg
            txt = f"Solo J{junta} {'+' if relativo and valor_deg >= 0 else ''}{valor_deg:.1f}°" + (" (relativo)" if relativo else " (absoluto)")
            return self.calcular_articular(q, txt)

    # Ejecución
    def ejecutar(self, plan_id=None) -> tuple[bool, str]:
        with self.lock:
            if self.envio_pendiente:
                return False, "Ya hay un envío en curso."
            if not self.armado:
                return False, "Primero hay que ARMAR el movimiento (botón Armar)."
            if self.ejec is not None or self.jog is not None:
                return False, "El robot ya se está moviendo."
            plan = self.plan
            if plan is None or "_tray" not in plan:
                return False, "No hay un plan calculado."
            if plan_id is not None and (type(plan_id) is not int or plan_id != plan["id"]):
                return False, "El plan mostrado ya no coincide con el actual. Revisá el nuevo plan."
            if not plan.get("ejecutable", False):
                return False, "No se envía: el plan fue rechazado. Corregí el objetivo y volvé a calcular antes de enviar."
            m = self.modo()
            if m.get("solo_lectura"):
                return False, "La consola está en SOLO LECTURA."
            if m["exige_go"]:
                if not self.permitir_real:
                    return False, "Robot real sin --real: la consola solo lee."
                ok_go, msg_go = self._validar_go_sesion()  # el GO se vuelve a comprobar en cada envío
                if not ok_go:
                    return False, "No se envía: " + msg_go
            q_med, t_est = self._q_medida()
            if q_med is None:
                return False, "No llega la posición del robot."
            edad = time.monotonic() - t_est
            if edad > self.cfg["seguridad"]["watchdog_estado_s"]:
                return False, (
                    f"No se envía: el último estado del robot tiene {edad:.2f} s "
                    f"(máximo {self.cfg['seguridad']['watchdog_estado_s']} s)."
                )
            if plan.get("vel_pct", 0.0) > m["vel_max_pct"] + 1e-9:
                return False, f"No se envía: el plan se calculó al {plan['vel_pct']:.0f} % y el tope vigente es {m['vel_max_pct']:.0f} %. Recalculalo."
            if m.get("vlim_c02_deg_s") and m["vel_max_pct"] < 5.0 - 1e-9:
                return False, (f"No se envía: el GO autoriza {m['vel_max_pct']:g} % y el RAPID (C02) limita cada junta al 5 % "
                               "de la ficha. Para ir más lento hay que bajar vLim en rapid/VEVAS_RWS.mod y volver a verificar C02.")
            dif = np.degrees(np.abs(np.asarray(q_med) - plan["_tray"].q0)).max()
            if dif > self.cfg["seguridad"]["inicio_max_deg"]:
                if (m["exige_go"] or plan.get("tipo") == "figura" or plan.get("preparar_j5")
                        or plan.get("punto_intermedio_deg") or plan.get("pose_guardada")):
                    return False, f"La postura inicial cambió {dif:.2f}°. Recalculá y revisá el plan."
                self.avisar(f"El robot se movió {dif:.2f}° desde que se calculó el plan: se recalcula desde la postura actual.", "aviso")
                plan["_tray"] = self._planificar_punto(q_med, plan["q_obj_rad"], plan.get("vel_cap_deg_s"))
            tray = plan["_tray"]
            ts, Q, QD, QDD = tray.muestrear(self.cfg["trayectoria"]["dt_envio_s"])
            revision = revisar_trayectoria(self.robot, ts, Q, QD, q_med, self.cfg, m["vel_max_pct"], m["exige_go"],
                                           permitir_singular_destino=plan.get("_permitir_singular_destino", False),
                                           velocidad_articular_verificada=m.get("velocidad_articular_verificada", True))
            problemas = problemas_de(revision)
            if plan["tipo"] == "pose" and plan.get("ik", {}).get("elegida") is None:
                problemas.append("No hay rama válida.")
            if revision["bloquea_envio"] or problemas:
                self.reg.evento("envio_rechazado", plan_id=plan["id"], problemas=problemas)
                return False, "No se envía: " + " ".join(problemas)
            self.envio_pendiente = True
            self._cancelar_envio = CancelacionOrden()
            opciones = {}
            por_llegada = bool(getattr(self.backend, "seguimiento_por_llegada", False))
            if self.backend.tipo == "rws":
                opciones = {"origen_revisado": Q[0].copy(), "cancelar": self._cancelar_envio,
                            "comprobar_go": self._comprobar_go_orden}
        try:
            if self.backend.tipo == "sim":
                if tray.a_dict(1.0).get("perfil") != "lspb":
                    opciones["QDD"] = QDD
                ok, msg = self.backend.ejecutar_trayectoria(ts, Q, QD, **opciones)
            else:
                ok, msg = self.backend.ejecutar_trayectoria(ts[1:], Q[1:], QD[1:], **opciones)
        except Exception as ex:  # noqa: BLE001
            ok, msg = False, f"error al enviar: {ex}"
        with self.lock:
            self.envio_pendiente = False
            cancelar = self._cancelar_envio.is_set() or not self.armado
            if not ok:
                self.avisar(f"Plan {plan['id']}: el envío falló: {msg}", "error")
                return False, msg
            if cancelar:
                # Se desarmó o se pidió PARAR mientras la orden viajaba: se cancela lo que haya llegado.
                try:
                    self.backend.mantener(None)
                except Exception:  # noqa: BLE001
                    pass
                self.reg.evento("aborto", motivo="desarmado o PARAR durante el envío")
                self.avisar("Envío cancelado: se desarmó o se pidió PARAR mientras se enviaba.", "error")
                return False, "Se canceló el envío de puntos siguientes. Un tramo RWS ya emitido puede seguir en curso."
            limite_s = tray.tf * 6 + 30  # solo se usa con seguimiento por llegada
            est = getattr(self.backend, "duracion_estimada_s", None)
            if est is not None:
                try:
                    limite_s = max(limite_s, float(est(Q[1:])))
                except Exception:  # noqa: BLE001
                    pass
            self.ejec = {
                "limite_s": limite_s,
                "plan_id": plan["id"],
                "tray": tray,
                "t0": time.monotonic(),
                "t": 0.0,
                "err_max_deg": None if por_llegada else 0.0,
                "e_p_max_mm": None if por_llegada else 0.0,
                "e_R_max_deg": None if por_llegada else 0.0,
                "descripcion": plan["descripcion"],
                "q_obj": np.array(plan["q_obj_rad"]),
                "T_d": plan.get("_T_d"),
                "inicio": dt.datetime.now().isoformat(timespec="seconds"),
            }
            self.q_ref = None if por_llegada else tray.q0.copy()
            self.reg.evento("envio", plan_id=plan["id"], tf=tray.tf, n_puntos=len(ts) - 1, vel_pct=self.vel_pct, mensaje=msg,
                            revision=revision["estado"])
            self.avisar(f"Plan {plan['id']} ENVIADO al robot ({msg}). Duración planificada {tray.tf:.2f} s.", "ok")
            return True, msg

    def _llegada_sim_verificada(self, ejecucion, q_med):
        if q_med is None or not np.all(np.isfinite(q_med)):
            return False
        error_juntas = np.max(np.abs(np.degrees(np.asarray(q_med) - ejecucion["q_obj"])))
        if error_juntas > self.cfg["seguridad"].get("llegada_sim_juntas_deg", 0.25):
            return False
        objetivo = ejecucion.get("T_d")
        if objetivo is None:
            return True
        medida = self.robot.fk_tcp(q_med)
        seguridad = self.cfg["seguridad"]
        return (
            error_posicion(medida, objetivo) * 1000.0 <= seguridad.get("llegada_sim_posicion_mm", 2.0)
            and error_orientacion(medida[:3, :3], objetivo[:3, :3]) * GRAD
            <= seguridad.get("llegada_sim_orientacion_deg", 1.0)
        )

    def _terminar_ejecucion(self, q_med):
        e = self.ejec
        self.ejec = None
        self.q_ref = e["q_obj"].copy()
        duracion = e["tray"].tf
        if getattr(self.backend, "seguimiento_por_llegada", False):
            duracion = self.backend.duracion_observada_s
        r = {"plan_id": e["plan_id"], "descripcion": e["descripcion"], "inicio": e["inicio"],
             "duracion_s": duracion, "duracion_planificada_s": e["tray"].tf}
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
                r["e_R_vs_pose_pedida_deg"] = error_orientacion(T_fin[:3, :3], e["T_d"][:3, :3]) * GRAD
        r["err_seguimiento_max_deg"] = e["err_max_deg"]
        r["e_p_seguimiento_max_mm"] = e["e_p_max_mm"]
        r["e_R_seguimiento_max_deg"] = e["e_R_max_deg"]
        r["resultado_controlador"] = getattr(self.backend, "ultimo_resultado", None)
        r["estado"] = "completado"
        if self.backend.tipo == "sim":
            r["llegada_verificada"] = True
        self.ultimo_resumen = r
        self.reg.agregar_resumen(r)
        self.reg.evento("fin", **r)
        if e["err_max_deg"] is None:
            seguimiento = "Seguimiento temporal no medido."
        else:
            seguimiento = f"Error de seguimiento máx. {e['err_max_deg']:.3f}°, e_p máx. {e['e_p_max_mm']:.2f} mm."
        self.avisar(f"Plan {e['plan_id']} terminado. {seguimiento} "
                    f"Error final {r.get('error_final_max_deg', float('nan')):.3f}°.", "ok")

    def _abortar(self, motivo: str):
        """Detener todo, dejar el robot quieto y desarmar. Se registra el motivo."""
        if self.envio_pendiente:
            self._cancelar_envio.set()
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
            if self.envio_pendiente:
                self._cancelar_envio.set()
            if self.secuencia is not None and self.secuencia.get("estado") in ("corriendo", "esperando"):
                self._sec_cmd = "cancelar"
            if self.envio_pendiente or self.ejec is not None or self.jog is not None:
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
            if self.backend.tipo == "rws":
                return False, ("El jog continuo con flechas no existe con RWS (el robot recibe waypoints con MoveAbsJ). "
                               "Usá «Calcular y verificar» y luego Enviar.")
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
            q_med, t_est = self._q_medida()
            if q_med is None:
                return False, "No llega la posición del robot."
            if time.monotonic() - t_est > self.cfg["seguridad"]["watchdog_estado_s"]:
                return False, "El estado del robot está desactualizado: no se mueve."
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
            ok, msg = self.ejecutar(plan_id=plan["id"])
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
                    "progreso": None if getattr(self.backend, "seguimiento_por_llegada", False) else min(1.0, e["t"] / max(e["tray"].tf, 1e-6)),
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


def _nombre_figura(figura) -> str:
    nombre = figura.strip().lower() if isinstance(figura, str) else ""
    nombre = {"círculo": "circulo", "corazón": "corazon", "triángulo": "triangulo"}.get(nombre, nombre)
    if nombre not in FIGURAS:
        raise ValueError("Elegí círculo, corazón, cuadrado, triángulo o saludo.")
    return nombre


def _juntas_saludo(q0, tamano_mm, n) -> np.ndarray:
    """Saludo articular de la muñeca (J4, J5, J6) desde q0; empieza y termina en q0."""
    q0 = np.asarray(q0, dtype=float)
    u = np.linspace(0.0, 2.0 * math.pi, n)
    Q = np.tile(q0, (n, 1))
    amplitud = math.radians(float(tamano_mm) * 0.25)
    Q[:, 3] += amplitud * np.sin(2.0 * u)
    Q[:, 4] += (1 if q0[4] >= 0 else -1) * amplitud * (1.0 - np.cos(u)) * 0.5
    Q[:, 5] += amplitud * np.sin(u)
    Q[[0, -1]] = q0
    return Q


def pose_texto(p: dict) -> str:
    return f"({p['x']:.0f}, {p['y']:.0f}, {p['z']:.0f}) mm, Rz {p['rz']:.0f}° Ry {p['ry']:.0f}° Rx {p['rx']:.0f}°"


def publico(plan: dict) -> dict:
    return {k: v for k, v in plan.items() if not k.startswith("_")}


CHECKLIST_GO = (
    "1_identidad_controlador_manual",
    "2_autorizacion_acceso_robot",
    "3_operador_y_supervisor_presentes",
    "4_modo_operativo_permitido",
    "5_zona_despejada",
    "6_paro_emergencia_y_parada_normal_probados",
    "7_trayectoria_revisada_en_simulacion",
    "8_limites_velocidad_tool_base_revisados",
    "9_sin_pieza_ni_herramienta_peligrosa",
    "10_registro_y_aborto_funcionando",
    "11_condiciones_de_aborto_y_recuperacion_acordadas",
)
CAMPOS_GO = ("autoriza", "operador_pendant", "responsable_consola", "supervisor", "alcance")


def _texto_lleno(v) -> bool:
    if not isinstance(v, str):
        return False
    t = v.strip()
    return bool(t) and "COMPLETAR" not in t.upper()


def _numero_finito(v):
    return type(v) in (int, float) and abs(v) <= sys.float_info.max


def _validar_entorno_go(entorno):
    if not isinstance(entorno, dict) or not _numero_finito(entorno.get("mesa_z_m")):
        return False, "El entorno del GO necesita la altura finita de la mesa."
    for clave in ("sin_objetos_externos_confirmado", "sin_herramienta_confirmada"):
        if type(entorno.get(clave)) is not bool:
            return False, f"La declaración de entorno {clave} debe ser booleana."
    cajas = entorno.get("cajas")
    if not isinstance(cajas, list):
        return False, "Las cajas del entorno deben formar una lista."
    for caja in cajas:
        if not isinstance(caja, dict):
            return False, "Cada caja del entorno necesita min_mm y max_mm."
        minimo, maximo = caja.get("min_mm"), caja.get("max_mm")
        if not all(isinstance(v, list) and len(v) == 3 and all(_numero_finito(x) for x in v)
                   for v in (minimo, maximo)):
            return False, "Cada caja necesita seis coordenadas finitas."
        if any(a >= b for a, b in zip(minimo, maximo)):
            return False, "Cada eje de una caja necesita un mínimo menor que su máximo."
    herramienta = entorno.get("herramienta")
    if not isinstance(herramienta, dict) or type(herramienta.get("confirmada")) is not bool:
        return False, "La confirmación de herramienta debe ser booleana."
    radio = herramienta.get("radio_mm")
    if not _numero_finito(radio) or radio < 0 or (herramienta["confirmada"] and radio == 0):
        return False, "El radio de herramienta debe ser finito y positivo si hay herramienta."
    if herramienta["confirmada"] and entorno["sin_herramienta_confirmada"]:
        return False, "El GO declara a la vez herramienta confirmada y brida libre."
    return True, "Entorno válido."


def validar_go(go: dict | None) -> tuple[bool, str]:
    """El archivo GO lo llena el grupo. Se exige completo, de hoy y con todo el checklist en true.

    Un GO vacío, sin checklist, con campos sin completar o con algún ítem faltante NO vale.
    """
    if not isinstance(go, dict) or not go:
        return False, "No se cargó ningún archivo GO (usa --go GO_VEVAS.json) o está vacío."
    hoy = dt.date.today().isoformat()
    if go.get("fecha") != hoy:
        return False, f"El GO es de la fecha {go.get('fecha')} y hoy es {hoy}: llénalo de nuevo."
    for campo in CAMPOS_GO:
        if not _texto_lleno(go.get(campo)):
            return False, f"Falta completar el campo '{campo}' del GO."
    check = go.get("checklist")
    if not isinstance(check, dict) or not check:
        return False, "El GO no tiene 'checklist'."
    faltan = [k for k in CHECKLIST_GO if k not in check]
    if faltan:
        return False, "Al checklist del GO le faltan ítems: " + ", ".join(faltan)
    pend = [k for k in CHECKLIST_GO if check.get(k) is not True]
    if pend:
        return False, "Ítems del checklist GO sin confirmar: " + ", ".join(pend)
    v = go.get("velocidad_max_pct")
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not (0 < v <= 100):
        return False, "El GO debe indicar 'velocidad_max_pct' (número entre 0 y 100)."
    if "entorno" in go:
        ok, msg = _validar_entorno_go(go["entorno"])
        if not ok:
            return False, msg
    if go.get("decision") != "GO":
        return False, "La decisión del archivo GO no es 'GO'."
    return True, "GO válido."
