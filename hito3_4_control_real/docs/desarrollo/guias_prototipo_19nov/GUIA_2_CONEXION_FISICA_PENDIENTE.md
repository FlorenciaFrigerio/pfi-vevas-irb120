# Guía 2 — conexión física: orden de trabajo y puntos pendientes

Esta guía prepara la etapa siguiente. La interfaz de este ZIP solo admite simulación. No hay que quitar la restricción ROS/real para intentar mover el brazo. Antes de emitir órdenes físicas deben cerrarse los defectos de control de la auditoría y probarse la integración.

## 1. ¿Calibrar cada vez?

No. Encender, conectar el cable o abrir ROS no exige por sí mismo una calibración nueva.

En el FlexPendant, el operador comprueba Calibration → ROB_1 y el estado de los seis ejes. Los nombres exactos pueden variar por versión. El estado debe indicar calibración válida y contadores de revolución válidos, sin avisos pendientes que impidan confiar en la posición.

Ayer se reportó calibración/actualización de contadores hecha; es un antecedente, no una verificación de hoy. Tener q cerca de cero o ver datos por red no demuestra calibración correcta.

Actualizar contadores de revolución y realizar calibración fina son procedimientos distintos. Si los contadores se perdieron, el operador responsable sigue el procedimiento ABB para ese manipulador, con las marcas mecánicas y referencias correctas. No se actualizan contadores desde una postura arbitraria ni solo para borrar un aviso. La calibración fina no se improvisa desde la interfaz.

El evento 38200 Battery backup lost debe evaluarse con el responsable. Estar conectado a la alimentación principal no prueba que el respaldo de posición esté bien. Antes de ensayar se debe comprobar el estado actual de calibración/contadores y la causa del aviso. «Update» aquí no significa actualizar firmware ni RobotWare.

## 2. Preparar estación y leer por RWS

Operador responsable presente, zona despejada y procedimiento de seguridad del laboratorio vigente. La interfaz no reemplaza paro de emergencia, dispositivo de habilitación ni protecciones del IRC5. No se cambia a Auto ni se puentea Guard Stop para resolver un error de red.

Configuración usada previamente: IRC5 SERVICE 192.168.125.1, Ubuntu 192.168.125.10/24 por eno1. Verificar que corresponde a la estación actual.

En la Ubuntu del laboratorio:

```bash
cd ~/vevas-publicacion/hito3_4
mkdir -p docs/datos/conexion
ping -c 4 -W 2 192.168.125.1
ip route get 192.168.125.1
```

Esperado: respuesta al ping y ruta por la interfaz Ethernet con origen 192.168.125.10. No cambiar la red desde esta guía si la estación usa otra configuración.

Lectura de una postura, sin ordenar movimiento:

```bash
cd ~/vevas-publicacion/hito3_4
set -o pipefail
python3 -u herramientas/verificar_convencion_rws.py --posturas 1 2>&1 | tee "docs/datos/conexion/$(date +%F_%H%M%S)_fk_controlador.txt"
```

Comprobar juntas, herramienta/tool0 y residuos FK frente al controlador. Un error de jointtarget no se resuelve usando ignore=1 como certificación de calibración. Revisar el estado del IRC5. La comparación realizada anteriormente solo validó una postura y tool0; falta ampliar posturas y verificar cualquier herramienta montada.

## 3. Preparar y verificar ROS/EGM

Ayer se confirmó UDP bidireccional en 6511 y activación de ABBSystemHardware, joint_state_broadcaster y joint_trajectory_controller. Eso no garantiza que hoy sigan activos.

Cuando el driver esté iniciado dentro del procedimiento del laboratorio, usar una terminal de diagnóstico separada:

```bash
source /opt/ros/jazzy/setup.bash
source ~/abb_ws/install/setup.bash
ros2 node list
ros2 control list_controllers
ros2 action list -t
ros2 topic list -t
```

Esperado: controladores activos, acción FollowJointTrajectory y estados recientes con los seis nombres de juntas correctos. Verificar el tópico real antes de usarlo: el puente RWS publicó /irb120/real/joint_states y el driver normalmente usa /joint_states.

El módulo observado es EGM_ROS de T_ROB1. ros_egm_verificar solo transmite estados durante 20 segundos y no debe mover el brazo. ros_egm sí prepara seguimiento de órdenes; no se ejecuta como sustituto de una prueba de lectura. La IP destino y UDP6511 deben coincidir con la Ubuntu. Nunca cargar un módulo main del repo sobre los módulos existentes sin revisión y respaldo.

El arranque físico de EGM y la secuencia de operación del FlexPendant se documentarán contra la configuración actual, con el operador; este ZIP no cambia el controlador ni inicia movimiento.

## 4. Integrar primero solo lectura

Una vez corregido el backend, agregar el adaptador ROS a esta interfaz, mantener deshabilitado el envío y verificar: nombres/orden/signos/unidades en radianes, tiempos de la fuente, edad de muestra, TCP/herramienta/base, interpolación solo visual y desconexión. Actualizar gráficas con datos recibidos sin etiquetar LSPB como velocidad medida. No multiplicar muestras viejas fingiendo mayor frecuencia física.

## 5. Antes de habilitar envío

Cerrar C01–C04: validación completa de GO; tope de velocidad por cada junta; exclusión entre desarmado y envío pendiente; rechazo de estado viejo antes de enviar. Verificar cancelación, resultado de acción, watchdog y que el botón PARAR es solo una solicitud de software. El código actual no realiza detección completa de colisiones y no certifica seguridad física.

## 6. Ensayos físicos, de menor a mayor

1. Registrar estado inicial y postura supervisada adecuada, evitando comenzar ensayos cartesianos en la singularidad de muñeca reportada anteriormente.
2. Ensayo pequeño de una junta y comparación referencia/medida; documentar resultado real, límite de velocidad aplicado y parada.
3. Repetir por las seis juntas, de una en una.
4. Movimiento articular sincronizado de seis juntas.
5. Pose cartesiana: Pieper → selección de rama válida → verificación FK → LSPB → revisión → envío explícito.
6. Trayectoria corta y repetible del proyecto, gráficos y registro; luego tarea requerida por los hitos.
7. Video físico y exportación de evidencia con fechas, contexto, errores y resultado de cada acción. No presentar simulación como evidencia del robot real.

Los desplazamientos, velocidades y poses concretas se elegirán según herramienta, obstáculos y postura actuales. No usar los ejemplos del simulador como objetivos físicos automáticos.
