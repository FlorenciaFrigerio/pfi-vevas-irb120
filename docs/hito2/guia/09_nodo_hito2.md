# Script 09 — nodo_hito2.py: ruta de integración ROS 2 opcional

> **Nivel:** Complementario. Estúdialo a fondo solo si van a mostrar esta ruta ROS 2.

## Para qué sirve
Es una ruta opcional de integración: recibe una pose, calcula una rama ejecutable, produce LSPB y publica estado/diagnóstico en tópicos ROS. No es necesario para ejecutar demo_recorrido.py ni para la exposición numérica por consola.

**Entrada y salida:** Suscribe /pose_objetivo; publica /joint_states, /trayectoria_hito2 y /diagnostico_hito2.

## Recorrido por bloques

### A. ROS y límites de marcha
NodoHito2 crea suscripción, publicadores y timer de 100 Hz; define límites V_MAX/A_MAX.

**Código real** (`scripts/nodo_hito2.py`, líneas 87-106):

```python
    def __init__(self):
        super().__init__("nodo_hito2")

        self.q_actual = np.zeros(6)
        self.traj = None
        self.t0 = None

        self.sub = self.create_subscription(
            PoseStamped, "/pose_objetivo", self.cb_pose, 10)
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_traj = self.create_publisher(
            JointTrajectory, "/trayectoria_hito2", 10)
        self.pub_diag = self.create_publisher(
            String, "/diagnostico_hito2", 10)

        self.timer = self.create_timer(1.0/FRECUENCIA, self.cb_timer)

        self.get_logger().info(
            "Nodo Hito 2 activo. Esperando pose en /pose_objetivo")
        self.publicar_estado(self.q_actual)
```

**Cómo leerlo:** Los límites de marcha se fijan conservadores respecto al máximo permitido.

### B. Callback de pose
cb_pose convierte cuaternión, arma T, llama resolver, calcula error y crea perfil sincronizado.

**Código real** (`scripts/nodo_hito2.py`, líneas 109-156):

```python
    def cb_pose(self, msg):
        """Pose recibida -> Pieper -> filtrado -> LSPB."""
        p = msg.pose.position
        o = msg.pose.orientation
        try:
            R = quat_a_matriz(o.x, o.y, o.z, o.w)
        except ValueError as e:
            self.avisar(f"RECHAZADA: {e}")
            return

        T_d = np.eye(4)
        T_d[:3, :3] = R
        T_d[:3, 3] = [p.x, p.y, p.z]

        q_obj, admisibles, rechazadas = resolver(T_d, q_actual=self.q_actual)

        if q_obj is None:
            motivos = {}
            for _, m in rechazadas:
                clave = m.split(":")[0]
                motivos[clave] = motivos.get(clave, 0) + 1
            detalle = ", ".join(f"{k} x{v}" for k, v in motivos.items())
            self.avisar(
                f"POSE INALCANZABLE  p=[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]  "
                f"8 ramas calculadas, 0 admisibles ({detalle})")
            return

        ep, eR = error_pose(q_obj, T_d)

        # Advertencia de singularidad ANTES de mover: si la pose objetivo
        # esta cerca de q5 = 0, la cinematica diferencial inversa exigiria
        # velocidades articulares desmedidas.
        w = medida_manipulabilidad(q_obj)
        aviso_sing = ""
        if abs(q_obj[4]) < 0.05 or w < UMBRAL_W:
            aviso_sing = (f"  [!] CERCA DE SINGULARIDAD: q5 = {q_obj[4]:+.4f} "
                          f"rad, w = {w:.3e}")

        self.traj = LSPBSincronizado(self.q_actual, q_obj, V_MAX, A_MAX)
        self.t0 = self.get_clock().now()

        self.avisar(
            f"ACEPTADA  {len(admisibles)}/8 ramas admisibles  |  "
            f"elegida: {etiquetar(q_obj)}  |  "
            f"dE = {ep:.2e} m, dR = {eR:.2e}  |  "
            f"T = {self.traj.T:.3f} s{aviso_sing}")

        self.publicar_trayectoria(q_obj)
```

**Cómo leerlo:** Une geometría, inversa, selección, Jacobiano y trayectoria.

### C. Timer/publicación
cb_timer muestrea el perfil; publicar_estado y publicar_trayectoria generan mensajes ROS.

**Código real** (`scripts/nodo_hito2.py`, líneas 159-176):

```python
    def cb_timer(self):
        """Muestrea el perfil LSPB y publica el estado articular."""
        if self.traj is None:
            return
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if t >= self.traj.T:
            t = self.traj.T
            q, _, _ = self.traj(t)
            self.q_actual = q[0]
            self.publicar_estado(self.q_actual)
            self.avisar(f"TRAYECTORIA COMPLETA  q_final = "
                        f"{np.round(self.q_actual, 4).tolist()}")
            self.traj = None
            return

        q, qd, _ = self.traj(t)
        self.q_actual = q[0]
        self.publicar_estado(q[0], qd[0])
```

**Cómo leerlo:** Publica la trayectoria para visualización/inspección; no es por sí solo un controlador industrial.

### D. Ciclo de vida
main inicializa, mantiene spin y libera recursos al terminar.

**Código real** (`scripts/nodo_hito2.py`, líneas 209-221):

```python
def main(args=None):
    rclpy.init(args=args)
    nodo = NodoHito2()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
```

**Cómo leerlo:** Es el punto de entrada del proceso ROS.

## Cómo se relaciona
Integra pieper.py, lspb.py, jacobiano.py y fk_irb120.py.

## Pruebas y evidencia
Demostración propuesta: pose válida, pose rechazada y observación de topics y diagnósticos.

## Pregunta para practicar defensa
> ¿Qué recibe y publica el nodo? ¿Publicar JointState equivale a comandar el robot real?

## Límite o precaución
La integración es ROS/visualización; no declarar control de hardware que el código no implementa.

## Ficha de repaso
- Problema que resuelve:
- Parámetros de entrada:
- Salida principal:
- Función clave:
- Prueba que lo respalda:
- Lo que no demuestra:
