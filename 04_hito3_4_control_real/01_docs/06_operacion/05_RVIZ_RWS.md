# RViz con la postura del ABB IRB 120

La vista usa `03_modelo/irb120_rws.urdf`, que conserva la geometría de primitivas del gemelo del Hito 1 y gira 180° el marco fijo de la brida alrededor de su eje Z para que `tool0` coincida con la convención ABB usada en la cinemática de VEVAS. El nodo RWS publica `joint_1` a `joint_6` en radianes en `/joint_states`. A partir de esos datos y del URDF, `robot_state_publisher` publica `/tf` y `/tf_static`. RViz usa `base_link` como marco fijo y recibe el URDF desde `/robot_description` con durabilidad **Transient Local**, así que también puede recibirlo si arranca más tarde.

## Ensayo en Ubuntu 24.04 con ROS 2 Jazzy, sin conexión al IRC5

Desde `04_hito3_4_control_real/02_vevas_studio`, abrir dos terminales. Cargar en ambas el mismo entorno. Si faltan paquetes en Ubuntu, instalar `ros-jazzy-robot-state-publisher`, `ros-jazzy-rviz2` y `ros-jazzy-tf2-ros`.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

En la primera terminal, iniciar el modelo y RViz.

```bash
ros2 launch "$PWD/02_herramientas/rviz_rws.launch.py"
```

En la segunda, publicar una postura de ejemplo en radianes. Después de comprobarla, cortar con Ctrl+C y repetir con otro valor de `joint_2` para ver cómo cambia el modelo.

```bash
ros2 topic pub -r 10 /joint_states sensor_msgs/msg/JointState \
  '{header: auto, name: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], position: [0.0, -0.35, 0.52, 0.0, 0.70, 0.0]}'
```

## Postura leída por RWS en el laboratorio

Para probar solo la visualización, abrir nuevas terminales con el mismo entorno ROS. En una terminal ejecutar el mismo launch. En la otra, reemplazar la publicación de prueba por el nodo lector. La prueba se puede hacer con el robot detenido y el IRC5 accesible por RWS.

```bash
python3 02_herramientas/nodo_rws_ros.py --solo-lectura --ip 192.168.125.1
```

El nodo se inicia en modo de solo lectura y publica las posiciones medidas. El parámetro `--solo-lectura` también admite la invocación explícita del lanzador. La vista funciona sin MoveIt, sin el driver EGM y sin arrancar un programa RAPID.

## Si RViz abre vacío

En la sesión original, el lanzador comprobaba que existieran el tópico y la acción antes de abrir el launch externo de ABB. El tópico puede aparecer en ROS al crear el publicador, antes de que llegue la primera postura. Para identificar la causa de esa sesión hacen falta los logs `sesiones/nodo_rws_*.log` y `sesiones/moveit_*.log` del Ubuntu del laboratorio.

Ejecutar las comprobaciones desde una terminal con el mismo entorno ROS y guardar los resultados junto con la salida del launch.

```bash
ros2 node info /robot_state_publisher
timeout 8s ros2 topic echo --once /robot_description --qos-durability transient_local
ros2 topic info /joint_states --verbose
timeout 8s ros2 topic echo --once /joint_states
timeout 8s ros2 run tf2_ros tf2_echo base_link tool0
ros2 action list
```

- Si falta `/robot_description`, revisar el primer error del launch y confirmar en RViz que **RobotModel → Description Topic** apunte a `/robot_description` con durabilidad **Transient Local**.
- Si `/joint_states` figura pero el eco vence, revisar el log del lector. Cuando pasa más de un segundo sin una medición RWS válida, el nodo deja de publicar. Verificar que el publicador que figura en `ros2 topic info` corresponda a la fuente de esa prueba.
- Si hay muestras pero no hay TF de `base_link` a `tool0`, verificar los seis nombres `joint_1` a `joint_6` y que los mensajes traigan un sello de tiempo. En RViz, revisar **Global Options → Fixed Frame**, que debe ser `base_link`, y el estado de **RobotModel**.

El ensayo sintético permite verificar el URDF y la cadena ROS. La correspondencia de la postura mostrada con el brazo se comprueba en el laboratorio comparando los estados articulares con el FlexPendant.
