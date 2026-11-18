# Guía 0. Ruta completa desde la preparación hasta el video físico

Grupo VEVAS · Revisión del 17/11/2026 · Repositorio de entrega FlorenciaFrigerio/pfi-vevas-irb120

## Qué tiene que verse en la demostración

Los mensajes de Florencia que el grupo compartió piden cálculos y error en tiempo real. El recorrido debe verse en este orden.

1. Recibir una pose o un objetivo articular.
2. Calcular posiciones y ángulos con el modelo propio.
3. Mostrar las ramas de Pieper, reproducir cada candidata con FK y comparar sus errores.
4. Elegir una rama que cumpla los criterios, mostrar la justificación y construir la LSPB.
5. Revisar el plan antes del envío.
6. Enviar y observar el movimiento físico, mientras se muestran referencia, medición y error.
7. Guardar el plan, las muestras, los eventos y el resultado para analizarlos después.

La captura del chat aportada en la conversación es la fuente de este pedido. El equipo debe conservarla junto a su documentación. El alcance de la prueba de movimiento y las condiciones del laboratorio se registran en el GO del día.

## Orden de las guías

| Guía | Para qué sirve | Producto que debe quedar |
|---|---|---|
| [1. Lo que ya hicimos](GUIA_1_LO_QUE_YA_HICIMOS.md) | Historia desde el gemelo digital y estado de VEVAS | Lista de hechos con enlaces a evidencia |
| [2. Conectar al brazo](GUIA_2_CONECTAR_AL_BRAZO.md) | Cable, red, RWS, marcos, EGM y driver | Lectura del IRC5 y estado ROS trazable |
| [3. Cinemática en vivo](GUIA_3_CINEMATICA_EN_VIVO.md) | FK, Pieper, ramas, Jacobiano y LSPB | Explicación de cada cálculo que aparece en pantalla |
| [4. Cada junta](GUIA_4_PRUEBA_JUNTA_POR_JUNTA.md) | Ensayo individual, coordinado e inversa | Tabla J1–J6 y métricas por movimiento |
| [5. Consola](GUIA_5_CONSOLA.md) | Instalación, interfaz, teclado y modos | Consola operativa en simulación y ROS |
| [6. GO y video](GUIA_6_SEGURIDAD_GO_Y_VIDEO.md) | Roles, alcance del ensayo, grabación | Video físico, GO y registros asociados |
| [7. Análisis](GUIA_7_ANALISIS_DE_ERRORES.md) | Revisar sesiones e investigar fallas | Informe y corrección comprobada |
| [8. Extensiones](GUIA_8_EXTENSIONES.md) | Mejoras después del hito | Experimentos adicionales con criterio medible |
| [9. Complemento de Marcelo](GUIA_9_REFERENCIA_Y_DIAGNOSTICO.md) | Qué aporta el segundo repo y cómo conectarlo | Procedencia, diferencias entre rutas y recuperación |
| [Presentación](PRESENTACION.md) | Reparto y preguntas | Ensayo de defensa individual |

## Qué está hecho y hasta dónde llega

| Parte | Evidencia de VEVAS disponible al revisar main 9d62ae9 | Próximo cierre |
|---|---|---|
| Hito 1 | URDF, FK analítica, validación TF y documentos | Explicarlo como origen del modelo |
| Hito 2 | Pieper, Jacobiano, singularidad, LSPB, dinámica y resultados | Reutilizar el modelo dentro del pipeline físico |
| S0 de Hito 3/4 | Consola 3D, cálculo propio, selección de ramas, teclado, registro y validación offline | Repetir la secuencia con ros2_control simulado |
| S1 de VEVAS | Inventario descrito y herramienta RWS preparada | Captura propia con fuente, hora y tres posturas distintas |
| S2 de VEVAS | Backend ROS implementado | Conectar estados de origen IRC5 y guardar los tópicos y sus tiempos |
| S3 de VEVAS | Secuencia y RAPID preparados | Probar la ruta elegida y filmar comando, aceptación y movimiento |
| Análisis | Herramienta para sesiones implementada | Analizar una sesión física y registrar sus limitaciones |

El repositorio de Marcelo aporta antecedentes físicos del mismo robot, con serie 120-101516. Tiene lectura RWS, ROS/TF y movimiento por RWS/RAPID documentados. Para incorporarlos a la defensa hay que citar su autor, fecha, versión y alcance. La integración de la consola VEVAS con EGM necesita su propia prueba. [Detalle y fuentes](GUIA_9_REFERENCIA_Y_DIAGNOSTICO.md).

## Paso 1. Preparar la copia de trabajo

En Ubuntu, si el repo ya existe, empezar por comprobarlo.

```bash
cd ~/abb_ws/src/pfi-vevas-irb120
git status --short
git branch --show-current
git log -1 --oneline
```

Guardar o confirmar los cambios locales antes de actualizar la rama. Para la primera descarga, desde `~/abb_ws/src`, usar `git clone https://github.com/FlorenciaFrigerio/pfi-vevas-irb120.git`.

La carpeta de ejecución es `pfi-vevas-irb120/hito3_4`. Durante la revisión había otra copia anidada `hito3_4/hito3_4` con 117 archivos idénticos. Las actualizaciones de esta revisión se aplican a la primera carpeta. Comprobar siempre la ruta con `pwd`.

## Paso 2. Ensayar en simulación interna

Instalar las dependencias que falten con el procedimiento de la Guía 5. Después ejecutar un comando por línea.

```bash
cd ~/abb_ws/src/pfi-vevas-irb120/hito3_4
mkdir -p docs/datos/ensayo_local
set -o pipefail
python3 -m pytest -q tests/ | tee docs/datos/ensayo_local/pytest.txt
python3 consola_vevas.py --modo sim --abrir
```

Con esta revisión hay 21 pruebas. Dos usan capturas históricas de Marcelo como datos offline y una verifica el modo ROS de lectura mediante sustitutos de ROS. Las otras 18 son las pruebas originales. El resultado de pytest debe indicar que todas pasan.

1. Abrir `http://127.0.0.1:8000` si el navegador no apareció.
2. Confirmar la banda de simulación y seis valores articulares.
3. En Objetivo, elegir una junta y un incremento relativo pequeño. Calcular sin enviar todavía.
4. Confirmar que solo cambia la coordenada elegida en el objetivo articular.
5. Observar la FK y la LSPB. Armar y ejecutar en simulación.
6. Repetir con la Pose A, X=350, Y=150, Z=400 mm y Rz=0, Ry=180, Rx=0 grados.
7. Mirar las ocho candidatas y los motivos de rechazo. La cantidad de ramas válidas depende de la postura, los límites y el margen de singularidad.
8. Ejecutar, comprobar la gráfica de error y abrir el último resumen.
9. Cerrar y ejecutar `python3 herramientas/analizar_sesion.py --ultima`.
10. Registrar la carpeta de sesión y el resultado en la bitácora.

## Paso 3. Ensayar la misma interfaz con ROS simulado

Primero identificar el launch y el Xacro que realmente están en el Ubuntu del grupo.

```bash
source /opt/ros/jazzy/setup.bash
source ~/abb_ws/install/setup.bash
ros2 pkg prefix --share abb_irb120_support
ros2 launch abb_bringup abb_control.launch.py --show-args
rg --files ~/abb_ws/src/abb_irb120 -g '*.xacro' -g '*controllers*.yaml'
```

La Guía 5 conserva el comando del driver con un marcador para el Xacro. Reemplazarlo por el archivo observado. Guardar el comando final en la bitácora. Ese paquete local no estaba incluido en el repo de VEVAS revisado y su nombre exacto debe comprobarse en esa PC.

Arrancar el driver con `use_fake_hardware:=true`, verificar hardware simulado y los controladores activos. Abrir la consola con `python3 consola_vevas.py --modo ros --abrir`. Repetir las pruebas del paso 2 y conservar los resultados de los siguientes comandos.

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 action list -t
```

## Paso 4. Conectar al IRC5 y validar la lectura

Seguir la Guía 2 desde el cable y la dirección IP. Registrar la interfaz de red realmente utilizada, dirección local, estado del enlace, respuestas de ping y GET RWS. Comparar los seis ángulos con el FlexPendant y guardar la hora y una captura.

```bash
python3 herramientas/verificar_convencion_rws.py --posturas 3
```

Cada postura se mide con el robot quieto. El operador cambia de postura desde el FlexPendant entre adquisiciones. Usar variedad que ejercite también la muñeca. Anotar herramienta `tool0`, objeto `wobj0`, coordenadas `Base`, grados de RWS y metros/radianes del modelo. Conservar los intentos fallidos o inconclusos.

El script propone 0,5 mm y 0,05 grados como criterio exploratorio. Los umbrales físicos necesitan acuerdo y presupuesto de incertidumbre. Las lecturas secuenciales antes/después ayudan a detectar movimiento y tienen una ventana temporal que debe documentarse. [Protocolo W14](https://github.com/berquiroga/planificacion-UCB/blob/d982ad0da3cfe69fe9eac39d99f7fd5d2115e633/2-2026/IMT342/Semana_14/sesion_02/w14_s02_criterios_unificados_validacion_hito3.tex).

## Paso 5. Ver estados del robot en la consola

Con un puente RWS ya activo que publique el tópico de Marcelo, ejecutar en otra terminal del mismo dominio ROS.

```bash
source /opt/ros/jazzy/setup.bash
source ~/abb_ws/install/setup.bash
cd ~/abb_ws/src/pfi-vevas-irb120/hito3_4
ros2 topic info /irb120/real/joint_states -v
ros2 topic echo /irb120/real/joint_states --once
python3 consola_vevas.py --modo ros --solo-lectura --topico-estado /irb120/real/joint_states --abrir
```

La banda debe mostrar SOLO LECTURA y el tópico. El modelo, la FK y el Jacobiano se actualizan desde las seis posiciones recibidas. El origen físico y la vigencia se verifican con el puente y la comparación con el FlexPendant. Guardar captura y respuesta de `curl http://127.0.0.1:8000/api/estado`.

## Paso 6. Cerrar la ruta de comando

La consola VEVAS envía `FollowJointTrajectory` y puntos `JointTrajectory`. Para mover el ABB por esta ruta deben estar configurados y probados driver, ros2_control y EGM de la Guía 2. El puente de lectura por sí solo permite llegar al paso 5.

El control RWS/RAPID de Marcelo tiene otro contrato de órdenes y llegada. Reutilizarlo como backend de VEVAS requeriría adaptar envío, cancelación, velocidad, watchdog y resultado. Registrar esa decisión antes de cambiar el transporte.

Confirmar el GO del ensayo y su alcance con el operador y laboratorio. Con el driver físico listo, abrir `python3 consola_vevas.py --modo ros --real --go GO_VEVAS.json --abrir`. Aplicar la Guía 4, primero una junta y retorno, después las seis, después la pose por inversa. Revisar cada recorrido en simulación y en el espacio físico, incluyendo la herramienta montada.

## Paso 7. Video, análisis y entrega

Grabar pantalla y robot físico con la misma identificación verbal de plan. Mostrar entrada, ramas, error FK, selección, trayectoria, envío y medición. Guardar la versión de código y mantener la sesión original asociada. La Guía 6 contiene el guion y la Guía 7 explica el informe.

```bash
python3 herramientas/analizar_sesion.py --ultima
git status --short
```

`sesiones/` está ignorada por git. Para entregar una sesión seleccionada, copiarla a una carpeta identificada de `docs/datos/`, junto con el informe y un texto de procedencia. Revisar el contenido antes de agregarlo. Para videos grandes, agregar el enlace al archivo compartido y su identificación en la bitácora.

Cada entrada debe incluir hipótesis, comando, versiones, condiciones, resultado, interpretación, corrección y siguiente acción. La bitácora del ensayo se completa con números realmente medidos. La aprobación de software y la prueba física se registran con su alcance específico.

## Siguiente paso concreto

Correr la secuencia en el Ubuntu con ros2_control simulado y guardar SIM-ROS-001. Después conectar el puente de lectura, comparar con el FlexPendant y registrar S2 de VEVAS. La siguiente meta física es completar la ruta EGM, ensayar junta por junta y grabar la pose por inversa con el error en vivo.
