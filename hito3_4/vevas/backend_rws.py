"""Conexión de la consola con el IRC5 por RWS (sin ROS, sin EGM).

Adaptado de LabJointRws / rws_mover del repo robotica-IMT342, rama proyectoHito3y4: la PC escribe
objetivos articulares absolutos en las PERS del módulo RAPID VEVAS_RWS (rapid/VEVAS_RWS.mod) y el
robot los valida y ejecuta con MoveAbsJ. El protocolo vive en vevas/rws_cliente.py.

Cosas que este backend hace distinto de BackendROS/BackendSim (Consola lo sabe por los atributos):

  * seguimiento_por_llegada = True. El robot recorre cada MoveAbsJ a su propia velocidad (vPc fija en
    el RAPID: 20 mm/s de TCP, 5 °/s de orientación) y para en cada waypoint; el tiempo del plan LSPB no
    se cumple. Consola NO compara contra el LSPB: termina cuando el backend informa que llegó el último
    waypoint (resultado_trayectoria) o que falló, con un tiempo límite global.
  * ejecutar_trayectoria no bloquea: lanza un hilo que manda los waypoints uno tras otro. Vuelve en
    cuanto el robot tomó el primero (o lo rechazó).
  * mantener() NO frena el brazo: pide que no se envíe ningún waypoint más. El MoveAbsJ en curso
    termina (a lo sumo unos segundos a 20 mm/s). El freno físico es el PARO DE EMERGENCIA o soltar
    el habilitador. Después no se acepta otra trayectoria hasta que el robot conteste esa orden.
  * No hay jog continuo por RWS: Consola lo rechaza con tipo == "rws".
"""
from __future__ import annotations

import threading
import time

import numpy as np

from .rws_cliente import PASO_PC_MAX_DEG, Cliente, Falla, MODULO, subdividir


class BackendRWS:
    tipo = "rws"
    seguimiento_por_llegada = True

    def __init__(self, cfg: dict, avisar=None, solo_lectura: bool = False, cliente: Cliente | None = None):
        r = cfg["rws"]
        rc = cfg["robot"]
        self._cfg = cfg
        self._avisar = avisar or (lambda t: None)
        self.solo_lectura = solo_lectura
        self.cli = cliente or Cliente(r["ip"], r["usuario"], r["clave"], int(r["puerto"]), float(r["timeout_http_s"]))
        self.ip = r["ip"]
        self.signos = np.array(rc["signos"], dtype=float)
        self.offsets = np.radians(np.array(rc["offsets_deg"], dtype=float))
        self.paso_deg = min(float(r["paso_envio_deg"]), PASO_PC_MAX_DEG)
        self.periodo_lectura = 1.0 / max(1.0, float(r["hz_lectura"]))
        self.timeout_orden_s = float(r["timeout_orden_s"])
        self.vel_min_deg_s = float(r["vel_min_estimada_deg_s"])
        self._lock = threading.Lock()
        self._q = None
        self._t_estado = 0.0
        self.ultimo_error = None
        self._activo = False
        self._hilo = None
        self._worker = None
        self._cancelar = threading.Event()
        self._gen = 0
        self.ultimo_resultado = None
        self._resultado_exitoso = None
        self.progreso = ""
        self.cli.on_joints = self._poner_q

    # ---- conversión de convenciones: q_DH = signo · q_RWS + offset (igual que BackendROS)
    def _a_dh(self, q_deg) -> np.ndarray:
        return self.signos * np.radians(np.asarray(q_deg, dtype=float)) + self.offsets

    def _a_rws_deg(self, q_rad) -> np.ndarray:
        return np.degrees((np.asarray(q_rad, dtype=float) - self.offsets) / self.signos)

    def _poner_q(self, q_deg):
        q = self._a_dh(q_deg)
        with self._lock:
            self._q = q
            self._t_estado = time.monotonic()

    # ---- arranque
    def iniciar(self):
        self._activo = True
        self._hilo = threading.Thread(target=self._leer_en_bucle, daemon=True, name="rws-lectura")
        self._hilo.start()

    def detener(self):
        self._activo = False
        self._cancelar.set()
        self.cli.cerrar()

    def _leer_en_bucle(self):
        falla_previa = None
        while self._activo:
            t0 = time.monotonic()
            try:
                self.cli.joints_deg()  # llama a _poner_q
                self.cli.mantener_rmmp()
                if falla_previa is not None:
                    self._avisar("RWS: se recuperó la lectura de las juntas.")
                falla_previa = None
                self.ultimo_error = None
            except Falla as e:
                self.ultimo_error = str(e)
                if falla_previa != str(e):
                    self._avisar(f"RWS: no llega la postura del robot ({e}). Sin datos nuevos, Consola aborta cualquier movimiento.")
                falla_previa = str(e)
            time.sleep(max(0.0, self.periodo_lectura - (time.monotonic() - t0)))

    def preparar_rapid(self, ruta_mod: str, forzar: bool = False, eco=print, espera_servidor: float = 40.0):
        """Instala VEVAS_RWS si falta (o difiere), pone el PP en VevasServidor y arranca RAPID. Solo AUTO.
        Lanza Falla con el motivo."""
        if self.solo_lectura:
            raise Falla("El backend está en solo lectura.")
        accion = self.cli.instalar_modulo(ruta_mod, forzar=forzar, eco=eco)
        eco(f"  Módulo {MODULO}: {accion}.")
        self.cli.preparar_y_arrancar(eco=eco)
        eco(f"  RAPID arrancado en VevasServidor. Esperando al servidor (con el .mod por defecto hay que pulsar «Iniciar» en el FlexPendant antes de 30 s)...")
        if not self.cli.esperar_servidor(espera_servidor):
            raise Falla("VevasServidor no quedó activo (vvRbActivo=0): ¿falta pulsar «Iniciar» en el FlexPendant, o el modo no está permitido?")
        eco("  Servidor activo (vvRbActivo=1).")

    # ---- estado
    def leer(self):
        with self._lock:
            return (None if self._q is None else self._q.copy()), self._t_estado

    def info(self) -> dict:
        texto = "IRC5 vía RWS · ROBOT REAL" + (" · SOLO LECTURA" if self.solo_lectura else "")
        return {"tipo": "rws", "hardware": "IRC5 vía RWS", "real": True, "texto": texto,
                "solo_lectura": self.solo_lectura, "ip": self.ip, "paso_envio_deg": self.paso_deg,
                "ultimo_error": self.ultimo_error, "progreso": self.progreso}

    def en_movimiento(self) -> bool:
        return self._worker is not None and self._worker.is_alive()

    def resultado_trayectoria(self) -> tuple[bool | None, str | None]:
        """None: pendiente; True: llegó al último waypoint; False: falló o se canceló."""
        with self._lock:
            return self._resultado_exitoso, self.ultimo_resultado

    def duracion_estimada_s(self, Q_rad) -> float:
        """Cota GRUESA y conservadora del tiempo que tarda el robot (para el tiempo límite de Consola)."""
        q, _ = self.leer()
        q0 = self._a_rws_deg(q) if q is not None else self._a_rws_deg(Q_rad[0])
        wps = subdividir(q0, self._a_rws_deg(Q_rad), self.paso_deg)
        t, previo = 0.0, q0
        for w in wps:
            t += max(abs(a - b) for a, b in zip(w, previo)) / self.vel_min_deg_s + 1.5
            previo = w
        return t

    # ---- comandos
    def ejecutar_trayectoria(self, ts, Q, QD=None) -> tuple[bool, str]:
        if self.solo_lectura:
            return False, "Backend RWS en solo lectura."
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                return False, ("La orden anterior sigue ejecutándose en el robot (o se está cancelando): "
                               "esperá a que termine.")
        try:
            e = self.cli.leer_estado(con_juntas=False)
            problemas = self.cli.problemas_para_enviar(e)
            if problemas:
                return False, "RWS no está listo para mover: " + "; ".join(problemas)
            q0 = self.cli.joints_deg()
        except Falla as ex:
            return False, f"RWS: {ex}"
        try:
            wps = subdividir(q0, self._a_rws_deg(Q), self.paso_deg)
        except Falla as ex:
            return False, str(ex)
        with self._lock:
            self._gen += 1
            gen = self._gen
            self.ultimo_resultado = None
            self._resultado_exitoso = None
            self.progreso = f"0/{len(wps)} waypoints"
            self._cancelar = cancelar = threading.Event()
        caja = {"ok": False, "msg": "sin respuesta"}
        primera = threading.Event()

        def al_aceptar(codigo):
            if codigo in (1, 2):
                caja["ok"] = True
            primera.set()

        def terminar(exito, texto):
            with self._lock:
                if gen == self._gen and self._resultado_exitoso is None:
                    self._resultado_exitoso, self.ultimo_resultado = exito, texto
            if not primera.is_set():
                caja["msg"] = texto
                primera.set()

        def correr():
            try:
                with self.cli.escritor():
                    for k, w in enumerate(wps, 1):
                        if cancelar.is_set():
                            return terminar(False, f"cancelada antes del waypoint {k}/{len(wps)} (no se enviaron más)")
                        with self._lock:
                            if gen == self._gen:
                                self.progreso = f"{k}/{len(wps)} waypoints"
                        ok, cod, msg = self.cli.enviar_objetivo(
                            w, timeout=self.timeout_orden_s, cancelar=cancelar,
                            al_aceptar=al_aceptar if k == 1 else None)
                        if not ok:
                            return terminar(False, f"waypoint {k}/{len(wps)}: {msg}")
                    try:
                        qf = self.cli.joints_deg()
                        falta = max(abs(a - b) for a, b in zip(qf, wps[-1]))
                    except Falla:
                        falta = float("nan")
                    if cancelar.is_set():
                        return terminar(False, "cancelada: el último waypoint ya estaba en curso")
                    terminar(True, f"llegada confirmada: {len(wps)} waypoints, falta {falta:.3f}° al final")
            except Falla as ex:
                terminar(False, str(ex))
            except Exception as ex:  # noqa: BLE001
                terminar(False, f"error inesperado en el envío: {ex}")

        self._worker = threading.Thread(target=correr, daemon=True, name="rws-trayectoria")
        self._worker.start()
        if not primera.wait(15.0):
            cancelar.set()
            return False, "El robot no tomó el primer waypoint en 15 s."
        if not caja["ok"]:
            # el robot contestó con un rechazo: esperar el texto que deja el hilo (unos ms)
            t0 = time.monotonic()
            while self.ultimo_resultado is None and time.monotonic() - t0 < 2.0:
                time.sleep(0.01)
            return False, self.ultimo_resultado or caja["msg"]
        return True, f"trayectoria aceptada por el servidor RWS ({len(wps)} waypoints de ≤{self.paso_deg:g}°)"

    def mantener(self, q=None) -> None:
        """Pide que no se envíe ningún waypoint más. NO frena el MoveAbsJ en curso: el freno físico es el
        paro de emergencia (o soltar el habilitador)."""
        with self._lock:
            ev = self._cancelar
            vivo = self._worker is not None and self._worker.is_alive()
            if vivo and self._resultado_exitoso is None:
                self._resultado_exitoso = False
                self.ultimo_resultado = "cancelada por PARAR/aborto: no se envían más waypoints (el movimiento en curso termina solo)"
        ev.set()

    def enviar_punto(self, q, horizonte: float) -> None:
        """Jog: no existe por RWS (Consola ya lo rechaza)."""
        return
