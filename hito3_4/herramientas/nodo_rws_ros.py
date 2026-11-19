#!/usr/bin/env python3
"""Nodo ROS 2 Jazzy opcional: deja que RViz y MoveIt usen el robot real por RWS (sin EGM).

    publica   /joint_states                                        sensor_msgs/JointState (joint_1..joint_6, rad, ~10 Hz)
    atiende   /joint_trajectory_controller/follow_joint_trajectory control_msgs/FollowJointTrajectory

La acción manda la trayectoria al módulo RAPID VEVAS_RWS (rapid/VEVAS_RWS.mod) como waypoints
absolutos de a lo sumo `--paso` grados por eje (vevas/rws_cliente.py, el mismo cliente que usa
BackendRWS). El robot los ejecuta con MoveAbsJ a su velocidad fija (20 mm/s, 5 °/s): los tiempos y las
velocidades de la trayectoria de MoveIt NO se respetan. Cancelar una meta solo evita los waypoints
siguientes; el freno físico es el paro de emergencia.

UNA SOLA HERRAMIENTA A LA VEZ: este nodo y la consola VEVAS (--modo rws) escriben en las mismas variables
PERS. Los dos toman un candado de archivo mientras mueven el robot: si el otro lo tiene, la meta se
rechaza con un mensaje claro.

Adaptado de rws_mover del repo robotica-IMT342, rama proyectoHito3y4. Se puede importar sin ROS: rclpy
se importa dentro de main(), para poder probar los ayudantes.

Uso:  python3 herramientas/nodo_rws_ros.py --ip 192.168.125.1 [--paso 10] [--solo-lectura]
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(AQUI))

from vevas.rws_cliente import PASO_PC_MAX_DEG, Cliente, Falla, rechazo_local, subdividir  # noqa: E402

NOMBRES = [f"joint_{i}" for i in range(1, 7)]
# control_msgs/FollowJointTrajectory.Result
SUCCESSFUL, INVALID_GOAL, INVALID_JOINTS, PATH_TOLERANCE_VIOLATED, GOAL_TOLERANCE_VIOLATED = 0, -1, -2, -4, -5


# ---------------------------------------------------------------------------- ayudantes sin ROS
def ordenar_juntas(nombres, posiciones, esperados=NOMBRES) -> list[float]:
    """Reordena `posiciones` (en el orden de `nombres`) al orden J1..J6. ValueError si falta o sobra alguna."""
    nombres = list(nombres)
    if sorted(nombres) != sorted(esperados) or len(posiciones) != len(esperados):
        raise ValueError(f"Juntas {nombres}: se esperaban {list(esperados)}.")
    return [float(posiciones[nombres.index(n)]) for n in esperados]


def waypoints_desde_puntos(puntos_rad, q_actual_deg, paso_deg: float) -> list[list[float]]:
    """Puntos de la trayectoria (rad, orden J1..J6) → waypoints en grados con saltos ≤ paso."""
    return subdividir(q_actual_deg, [[math.degrees(v) for v in p] for p in puntos_rad], paso_deg)


def validar_waypoints(waypoints, q_actual_deg, paso_max_deg: float = PASO_PC_MAX_DEG):
    """None si todo está bien; si no, el motivo (límites con margen de 2° y paso máximo)."""
    if not waypoints:
        return "La trayectoria está vacía."
    previo = list(q_actual_deg)
    for k, w in enumerate(waypoints, 1):
        rech = rechazo_local(w, previo, paso_max_deg)
        if rech is not None:
            return f"waypoint {k}/{len(waypoints)}: {rech[1]}"
        previo = w
    return None


class Ejecutor:
    """Ejecuta una trayectoria (lista de puntos en rad) por RWS. Sin ROS: lo prueban los tests."""

    def __init__(self, cliente: Cliente, paso_deg: float = 10.0, timeout_orden_s: float = 120.0):
        self.cli = cliente
        self.paso = min(float(paso_deg), PASO_PC_MAX_DEG)
        self.timeout = timeout_orden_s
        self.ocupado = threading.Lock()

    def preparar(self, nombres, puntos_rad) -> tuple[list[list[float]] | None, int, str]:
        """(waypoints, código FJT, texto). waypoints es None si la meta no es válida."""
        try:
            pts = [ordenar_juntas(nombres, p) for p in puntos_rad]
        except ValueError as e:
            return None, INVALID_JOINTS, str(e)
        if not pts:
            return None, INVALID_GOAL, "La trayectoria no tiene puntos."
        try:
            q0 = self.cli.joints_deg()
            wps = waypoints_desde_puntos(pts, q0, self.paso)
        except Falla as e:
            return None, INVALID_GOAL, str(e)
        motivo = validar_waypoints(wps, q0, PASO_PC_MAX_DEG)
        if motivo:
            return None, INVALID_GOAL, motivo
        return wps, SUCCESSFUL, f"{len(wps)} waypoints de ≤{self.paso:g}°"

    def ejecutar(self, nombres, puntos_rad, cancelado=lambda: False, progreso=lambda k, n, q: None, eco=None):
        """Devuelve (estado, código FJT, texto); estado ∈ {'ok', 'cancelado', 'falla'}."""
        wps, cod, txt = self.preparar(nombres, puntos_rad)
        if wps is None:
            return "falla", cod, txt
        if not self.ocupado.acquire(blocking=False):
            return "falla", INVALID_GOAL, "Ya hay una trayectoria en ejecución."
        try:
            with self.cli.escritor():
                for k, w in enumerate(wps, 1):
                    if cancelado():
                        return "cancelado", SUCCESSFUL, f"cancelada antes del waypoint {k}/{len(wps)}"
                    ok, codigo, msg = self.cli.enviar_objetivo(w, timeout=self.timeout, eco=eco)
                    if not ok:
                        c = INVALID_GOAL if codigo in (-1, -2, -3) else GOAL_TOLERANCE_VIOLATED
                        return "falla", c, f"waypoint {k}/{len(wps)}: {msg}"
                    progreso(k, len(wps), w)
            return "ok", SUCCESSFUL, f"{len(wps)} waypoints ejecutados"
        except Falla as e:
            return "falla", INVALID_GOAL, str(e)
        finally:
            self.ocupado.release()


class Lector(threading.Thread):
    """Lee las juntas por RWS en un hilo para no bloquear a ROS."""

    def __init__(self, cliente: Cliente, hz: float = 10.0):
        super().__init__(daemon=True, name="rws-lector")
        self.cli, self.periodo = cliente, 1.0 / max(1.0, hz)
        self.q_deg, self.t, self.error = None, 0.0, None
        self.vivo = True
        cliente.on_joints = self._poner

    def _poner(self, q):
        self.q_deg, self.t = list(q), time.monotonic()

    def run(self):
        while self.vivo:
            t0 = time.monotonic()
            try:
                self.cli.joints_deg()
                self.error = None
            except Falla as e:
                self.error = str(e)
            time.sleep(max(0.0, self.periodo - (time.monotonic() - t0)))

    def reciente(self, max_edad_s: float = 1.0):
        return self.q_deg if self.q_deg is not None and time.monotonic() - self.t <= max_edad_s else None


# ---------------------------------------------------------------------------- nodo ROS
def main(argv=None):
    ap = argparse.ArgumentParser(description="Nodo ROS 2: /joint_states y FollowJointTrajectory sobre RWS")
    ap.add_argument("--ip", default="192.168.125.1")
    ap.add_argument("--rws-puerto", type=int, default=80)
    ap.add_argument("--usuario", default="Default User")
    ap.add_argument("--clave", default=os.environ.get("VEVAS_RWS_CLAVE", "robotics"))
    ap.add_argument("--paso", type=float, default=10.0, help="grados máximos por waypoint (el RAPID admite 45, la PC 40)")
    ap.add_argument("--hz", type=float, default=10.0, help="frecuencia de /joint_states")
    ap.add_argument("--accion", default="/joint_trajectory_controller/follow_joint_trajectory")
    ap.add_argument("--topico", default="/joint_states")
    ap.add_argument("--timeout-orden", type=float, default=120.0)
    ap.add_argument("--solo-lectura", action="store_true", help="solo publica /joint_states; rechaza toda meta")
    a = ap.parse_args(argv)
    if not 0 < a.paso <= PASO_PC_MAX_DEG:
        ap.error(f"--paso debe estar entre 0 y {PASO_PC_MAX_DEG}")

    import rclpy  # noqa: PLC0415  (aquí, no al importar: los tests no tienen ROS)
    from builtin_interfaces.msg import Duration  # noqa: F401,PLC0415
    from control_msgs.action import FollowJointTrajectory  # noqa: PLC0415
    from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: PLC0415
    from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: PLC0415
    from rclpy.executors import MultiThreadedExecutor  # noqa: PLC0415
    from rclpy.node import Node  # noqa: PLC0415
    from sensor_msgs.msg import JointState  # noqa: PLC0415

    cli = Cliente(a.ip, a.usuario, a.clave, a.rws_puerto, timeout=3.0)
    lector = Lector(cli, a.hz)
    ejecutor = Ejecutor(cli, a.paso, a.timeout_orden)

    class NodoRws(Node):
        def __init__(self):
            super().__init__("nodo_rws_ros")
            self.cg = ReentrantCallbackGroup()
            self.pub = self.create_publisher(JointState, a.topico, 10)
            self.create_timer(1.0 / a.hz, self.publicar, callback_group=self.cg)
            self.avisado_viejo = False
            self.meta_activa = False
            self.server = ActionServer(
                self, FollowJointTrajectory, a.accion,
                execute_callback=self.ejecutar, goal_callback=self.meta, cancel_callback=self.cancelar,
                callback_group=self.cg)
            self.get_logger().info(
                f"RWS {a.ip}:{a.rws_puerto} → {a.topico} y {a.accion} (waypoints ≤{ejecutor.paso:g}°"
                f"{', SOLO LECTURA' if a.solo_lectura else ''}). Una sola herramienta a la vez.")

        def publicar(self):
            q = lector.reciente()
            if q is None:
                if not self.avisado_viejo:
                    self.get_logger().warn(f"Sin postura reciente por RWS ({lector.error or 'esperando'}): no se publica /joint_states.")
                    self.avisado_viejo = True
                return
            self.avisado_viejo = False
            m = JointState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.name = list(NOMBRES)
            m.position = [math.radians(v) for v in q]
            self.pub.publish(m)

        def meta(self, goal):
            if a.solo_lectura:
                self.get_logger().warn("Meta rechazada: el nodo está en --solo-lectura.")
                return GoalResponse.REJECT
            if self.meta_activa:
                self.get_logger().warn("Meta rechazada: ya hay otra trayectoria en ejecución.")
                return GoalResponse.REJECT
            tr = goal.trajectory
            wps, _, txt = ejecutor.preparar(tr.joint_names, [list(p.positions) for p in tr.points])
            if wps is None:
                self.get_logger().warn(f"Meta rechazada: {txt}")
                return GoalResponse.REJECT
            problemas = cli.problemas_para_enviar()
            if problemas:
                self.get_logger().warn("Meta rechazada: RWS no está listo: " + "; ".join(problemas))
                return GoalResponse.REJECT
            self.meta_activa = True
            self.get_logger().info(f"Meta aceptada: {txt}")
            return GoalResponse.ACCEPT

        def cancelar(self, goal_handle):
            self.get_logger().warn("Cancelación pedida: no se envían más waypoints (el MoveAbsJ en curso termina solo).")
            return CancelResponse.ACCEPT

        def ejecutar(self, goal_handle):
            tr = goal_handle.request.trajectory
            res = FollowJointTrajectory.Result()
            try:
                def progreso(k, n, w):
                    fb = FollowJointTrajectory.Feedback()
                    fb.joint_names = list(NOMBRES)
                    fb.actual.positions = [math.radians(v) for v in w]
                    goal_handle.publish_feedback(fb)
                    self.get_logger().info(f"waypoint {k}/{n} alcanzado")

                estado, cod, txt = ejecutor.ejecutar(
                    tr.joint_names, [list(p.positions) for p in tr.points],
                    cancelado=lambda: goal_handle.is_cancel_requested, progreso=progreso)
            finally:
                self.meta_activa = False
            res.error_code, res.error_string = cod, txt
            if estado == "ok":
                goal_handle.succeed()
            elif estado == "cancelado":
                goal_handle.canceled()
            else:
                self.get_logger().error(txt)
                goal_handle.abort()
            return res

    rclpy.init()
    nodo = NodoRws()
    lector.start()
    ex = MultiThreadedExecutor()
    ex.add_node(nodo)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        lector.vivo = False
        cli.cerrar()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
