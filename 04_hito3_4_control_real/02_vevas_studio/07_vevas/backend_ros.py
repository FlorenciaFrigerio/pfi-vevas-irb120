"""Conexión de la consola con ROS 2 Jazzy (driver abb_ros2 o hardware simulado).

La consola NO habla directo con el controlador ABB. Habla con ROS 2:

    lee   /joint_states                                   (sensor_msgs/JointState)
    manda /joint_trajectory_controller/follow_joint_trajectory  (acción, trayectorias LSPB)
    manda /joint_trajectory_controller/joint_trajectory   (tópico, puntos de jog con flechas)

Con use_fake_hardware:=true eso lo atiende el hardware simulado de ros2_control.
Con use_fake_hardware:=false lo atiende abb_hardware_interface, que lo pasa por EGM
(UDP, puerto 6511) al IRC5. La consola es la misma en los dos casos; por eso
pregunta al controller_manager qué hardware está cargado y lo muestra en pantalla.

Convención de juntas: q_DH = signo · q_ROS + offset (se configura en config.json).
Con el IRB 120 y esta tabla D-H se espera signo = +1 y offset = 0 en las 6 juntas;
la herramienta 02_herramientas/verificar_convencion_rws.py lo comprueba con el robot real.
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np


class BackendROS:
    tipo = "ros"

    def __init__(self, cfg: dict, avisar, solo_lectura: bool = False):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data

        from sensor_msgs.msg import JointState

        self._rclpy = rclpy
        self.solo_lectura = solo_lectura
        self._avisar = avisar
        self._cfg = cfg
        rc = cfg["robot"]
        self.nombres = list(rc["nombres_juntas"])
        self.signos = np.array(rc["signos"], dtype=float)
        self.offsets = np.radians(np.array(rc["offsets_deg"], dtype=float))

        if not rclpy.ok():
            rclpy.init()
        self.node = Node("consola_vevas")
        rosc = cfg["ros"]
        self.node.create_subscription(JointState, rosc["topico_estado"], self._cb_estado, qos_profile_sensor_data)
        self._ac = None
        self._pub = None
        if not solo_lectura:
            from rclpy.action import ActionClient
            from control_msgs.action import FollowJointTrajectory
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

            self._FJT = FollowJointTrajectory
            self._JT = JointTrajectory
            self._JTP = JointTrajectoryPoint
            self._ac = ActionClient(self.node, FollowJointTrajectory, rosc["accion_trayectoria"])
            self._pub = self.node.create_publisher(JointTrajectory, rosc["topico_trayectoria"], 10)

        self._lock = threading.Lock()
        self._q = None
        self._t_estado = 0.0
        self._nombres_vistos = []
        self._goal_handle = None
        self.ultimo_resultado = None
        self._resultado_exitoso = None
        self._hardware = {"tipo": "ros", "hardware": "desconocido", "real": None, "texto": "ROS 2 · hardware sin identificar"}

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.node)
        self._hilo = threading.Thread(target=self._executor.spin, daemon=True)

    # Arranque y diagnóstico
    def iniciar(self):
        self._hilo.start()
        if not self.solo_lectura:
            threading.Thread(target=self._detectar_hardware, daemon=True).start()

    def detener(self):
        try:
            self._executor.shutdown()
            self.node.destroy_node()
            self._rclpy.shutdown()
        except Exception:
            pass

    def _detectar_hardware(self):
        """Pregunta al controller_manager qué plugin de hardware está activo."""
        try:
            from controller_manager_msgs.srv import ListHardwareComponents
        except Exception:
            self._avisar("No se pudo importar controller_manager_msgs: el tipo de hardware queda desconocido.")
            return
        cli = self.node.create_client(ListHardwareComponents, self._cfg["ros"]["servicio_hardware"])
        for _ in range(20):
            if cli.wait_for_service(timeout_sec=0.5):
                break
        else:
            self._avisar("El controller_manager no responde: ¿está lanzado abb_control.launch.py?")
            return
        fut = cli.call_async(ListHardwareComponents.Request())
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0:
            time.sleep(0.05)
        if not fut.done() or fut.result() is None:
            self._avisar("No llegó respuesta de list_hardware_components.")
            return
        plugins = [c.plugin_name for c in fut.result().component]
        texto = ", ".join(plugins) or "ninguno"
        if any("abb_hardware_interface" in p for p in plugins):
            hw = {"hardware": texto, "real": True, "texto": "ROS 2 · ROBOT REAL (abb_hardware_interface + EGM)"}
        elif any("mock_components" in p or "fake" in p.lower() for p in plugins):
            hw = {"hardware": texto, "real": False, "texto": "ROS 2 · hardware simulado (use_fake_hardware)"}
        else:
            hw = {"hardware": texto, "real": None, "texto": f"ROS 2 · hardware no reconocido ({texto})"}
        with self._lock:
            self._hardware.update(hw)
        self._avisar(f"Hardware detectado: {hw['texto']}")

    def info(self) -> dict:
        with self._lock:
            d = dict(self._hardware)
        d["solo_lectura"] = self.solo_lectura
        d["topico_estado"] = self._cfg["ros"]["topico_estado"]
        if self.solo_lectura:
            d["texto"] = "ROS 2 · SOLO LECTURA · " + d["topico_estado"]
        d["nombres_juntas_vistos"] = list(self._nombres_vistos)
        return d

    # Estado
    def _cb_estado(self, msg):
        idx = []
        for n in self.nombres:
            if n in msg.name:
                idx.append(msg.name.index(n))
            else:
                cand = [i for i, m in enumerate(msg.name) if m.endswith(n)]
                if not cand:
                    return
                idx.append(cand[0])
        if max(idx) >= len(msg.position):
            return
        q_ros = np.array([msg.position[i] for i in idx], dtype=float)
        if not np.all(np.isfinite(q_ros)):
            return
        q = self.signos * q_ros + self.offsets
        with self._lock:
            self._q = q
            self._t_estado = time.monotonic()
            self._nombres_vistos = list(msg.name)

    def leer(self):
        with self._lock:
            return (None if self._q is None else self._q.copy()), self._t_estado

    def _a_ros(self, q_dh) -> list[float]:
        return list(((np.asarray(q_dh) - self.offsets) / self.signos).astype(float))

    def _duracion(self, t: float):
        from builtin_interfaces.msg import Duration

        s = int(math.floor(t))
        return Duration(sec=s, nanosec=int(round((t - s) * 1e9)))

    # Comandos
    def ejecutar_trayectoria(self, ts, Q, QD=None) -> tuple[bool, str]:
        if self.solo_lectura:
            return False, "Backend ROS en solo lectura."
        if not self._ac.wait_for_server(timeout_sec=2.0):
            return False, "La acción follow_joint_trajectory no está disponible (¿controlador activo?)."
        goal = self._FJT.Goal()
        goal.trajectory.joint_names = list(self.nombres)
        for k, t in enumerate(ts):
            p = self._JTP()
            p.positions = self._a_ros(Q[k])
            if QD is not None:
                p.velocities = list((np.asarray(QD[k]) / self.signos).astype(float))
            p.time_from_start = self._duracion(float(t))
            goal.trajectory.points.append(p)
        evento = threading.Event()
        caja = {}
        respuesta_lock = threading.Lock()
        vencido = False

        def respuesta(fut):
            try:
                gh = fut.result()
                with respuesta_lock:
                    cancelar = vencido
                    if not cancelar:
                        caja["gh"] = gh
                if cancelar and gh is not None and gh.accepted:
                    gh.cancel_goal_async()
            except Exception as ex:  # noqa: BLE001
                with respuesta_lock:
                    caja["error"] = str(ex)
            finally:
                evento.set()

        with self._lock:
            self._goal_handle = None
            self.ultimo_resultado = None
            self._resultado_exitoso = None
        try:
            self._ac.send_goal_async(goal).add_done_callback(respuesta)
        except Exception as ex:  # noqa: BLE001
            return False, f"No se pudo enviar la trayectoria: {ex}"
        if not evento.wait(3.0):
            with respuesta_lock:
                vencido = True
                tardio = caja.get("gh")
            if tardio is not None and tardio.accepted:
                tardio.cancel_goal_async()
            return False, "El controlador no respondió a la trayectoria en 3 s."
        if "error" in caja:
            return False, "Falló la respuesta del controlador: " + caja["error"]
        gh = caja.get("gh")
        if gh is None or not gh.accepted:
            return False, "El joint_trajectory_controller RECHAZÓ la trayectoria."
        with self._lock:
            self._goal_handle = gh

        def fin(fut):
            try:
                from action_msgs.msg import GoalStatus

                respuesta_final = fut.result()
                r = respuesta_final.result
                exito = respuesta_final.status == GoalStatus.STATUS_SUCCEEDED and r.error_code == 0
                texto = f"estado {respuesta_final.status}, código {r.error_code} {r.error_string}".strip()
            except Exception as e:  # noqa: BLE001
                exito, texto = False, f"error al recibir resultado: {e}"
            with self._lock:
                # Un resultado anterior no puede finalizar una orden nueva.
                if self._goal_handle is gh:
                    self.ultimo_resultado = texto
                    self._resultado_exitoso = exito

        try:
            gh.get_result_async().add_done_callback(fin)
        except Exception as ex:  # noqa: BLE001
            self.mantener()
            return False, f"No se pudo vigilar el resultado de la trayectoria: {ex}"
        return True, "trayectoria aceptada por joint_trajectory_controller"

    def resultado_trayectoria(self) -> tuple[bool | None, str | None]:
        """None: pendiente; True: éxito ROS y FJT; False: fallo o cancelación."""
        with self._lock:
            return self._resultado_exitoso, self.ultimo_resultado

    def enviar_punto(self, q, horizonte: float) -> None:
        """Jog: un solo punto que reemplaza la trayectoria actual (interfaz por tópico)."""
        if self.solo_lectura:
            return
        msg = self._JT()
        msg.joint_names = list(self.nombres)
        p = self._JTP()
        p.positions = self._a_ros(q)
        p.time_from_start = self._duracion(horizonte)
        msg.points.append(p)
        self._pub.publish(msg)

    def mantener(self, q=None) -> None:
        """Parar y quedarse quieto.

        - Si hay una trayectoria por acción, se cancela: el joint_trajectory_controller
          mantiene la posición actual al cancelar.
        - Si se estaba moviendo con flechas (tópico), se manda un último punto en q
          (la última referencia) para que termine el pasito y se quede ahí.
        """
        if self.solo_lectura:
            return
        gh = self._goal_handle
        self._goal_handle = None
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
            return
        if q is None:
            q, _ = self.leer()
        if q is not None:
            self.enviar_punto(q, 0.25)

    def en_movimiento(self) -> bool:
        return self._goal_handle is not None and self.ultimo_resultado is None
