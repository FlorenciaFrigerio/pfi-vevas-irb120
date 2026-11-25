# Guía 1. Lo que ya hicimos, paso a paso, y cuál es el siguiente paso

Grupo VEVAS · Proyecto Final Integrador · ABB IRB 120-3/0.6

Esta guía cuenta en orden todo lo que el grupo ya hizo, desde el Hito 1 hasta hoy, qué evidencia hay de cada cosa y qué falló en el camino (y cómo se arregló). Al final está la lista de lo que falta y el siguiente paso concreto.

Cómo leer los estados:

| Estado | Significa |
|---|---|
| HECHO | Se hizo y hay evidencia guardada (archivo, commit, captura, log). |
| HECHO, FALTA EVIDENCIA | Se hizo, pero la prueba no quedó guardada en el repo ni en la bitácora. Hay que repetirla o guardarla. |
| PENDIENTE | Todavía no se hizo. |
| BLOQUEADO | No se puede hacer hasta que alguien de afuera (docente, encargado) lo permita. |

Y los niveles de evidencia que pide el docente (rúbrica ECI-3/4):

| Nivel | Qué es | Ejemplo nuestro |
|---|---|---|
| S0 | Modelo o simulación (solo software) | Notebook del Hito 2, consola en modo simulación |
| S1 | Dato leído del controlador, con fuente y hora | `curl` a RWS, `verificar_convencion_rws.py` |
| S2 | El dato del controlador llega a ROS y se puede rastrear | `/joint_states` real con el driver |
| S3 | Movimiento físico autorizado y observado | El video del robot moviéndose con la consola |

## Parte A. Hito 1: el gemelo digital (setiembre)

Qué se pedía: describir el IRB 120 con la convención Denavit-Hartenberg (D-H), calcular la cinemática directa, armar el modelo URDF para ROS y comprobar en RViz que el modelo y la matemática coinciden.

Lo que se hizo, en orden:

1. Se armó la tabla D-H estándar del IRB 120 (convención de Truc y Lam). La única rareza es el offset de la junta 2: θ2 = q2 − 90°, para que con q2 = 0 el brazo quede vertical como en el robot real.

   | i | a [m] | α | d [m] | θ |
   |---|---|---|---|---|
   | 1 | 0 | −90° | 0.290 | q1 |
   | 2 | 0.270 | 0 | 0 | q2 − 90° |
   | 3 | 0.070 | −90° | 0 | q3 |
   | 4 | 0 | +90° | 0.302 | q4 |
   | 5 | 0 | −90° | 0 | q5 |
   | 6 | 0 | 0 | 0.072 | q6 |

2. Se programó la cinemática directa (`fk_irb120.py`). Con q = 0 la brida queda en p = (0.374, 0, 0.630) m, el mismo valor del manual de ABB.
3. Se escribió el URDF de dos formas (a mano y generado por un script desde la tabla D-H), con la deducción algebraica que conecta las dos: Origen_i = B_(i−1) · Rz(offset_i).
4. Se armó el paquete ROS 2 `irb120_description` (Ubuntu 24.04, ROS 2 Jazzy) y se visualizó en RViz.
5. Se validó con TF: el script compara la pose que da tf2 con la FK propia. **Falló la primera vez** con un error de 1.11 m. La causa: había dos nodos publicando `/joint_states` al mismo tiempo y las marcas de tiempo no coincidían, así que se comparaba la FK de un instante con la pose de otro. Se corrigió dejando un solo publicador y sincronizando el tiempo. Resultado final: error de 2.459e-10 m en 501 muestras.
6. Se subió todo a GitHub (repo del grupo, después renombrado `pfi-vevas-irb120`) con las ramas `main`, `dev` y `feature/*` que pide la guía del PFI.
7. Se hizo el informe en formato IEEE, las diapositivas y el guion del video.

Estado: HECHO (evidencia en el repo y en la defensa del Hito 1).

## Parte B. Hito 2: cinemática inversa, Jacobiano, LSPB y dinámica (octubre)

Qué se pedía: resolver la cinemática inversa por Pieper (8 ramas), verificarla con FK, calcular el Jacobiano y estudiar singularidades, planificar trayectorias LSPB y calcular el torque con un modelo dinámico, todo en un notebook y un nodo de ROS.

Lo que se hizo:

1. Módulos de Python: `pieper.py` (8 ramas), `jacobiano.py`, `singularidad.py`, `lspb.py`, `dinamica_1r.py`, `resolver_en_vivo.py` (resolver una pose escrita a mano) y `demo_recorrido.py` (recorrido narrado de todo el pipeline).
2. Notebook con las 9 secciones del esqueleto oficial, ejecutado de punta a punta.
3. Informe IEEE, que se compactó de 7 a 5 páginas porque el docente lo pidió más corto.
4. Presentación web interactiva para la defensa.
5. Defensa: a Gor le tocó la parte de LSPB y dinámica (integrante 4).

Lo que falló y cómo se corrigió (esto sirve mucho para la defensa del Hito 3, porque muestra método):

| Problema encontrado | Causa | Corrección | Lección |
|---|---|---|---|
| El ángulo q5 salía con error 9.1e-15 m en algunas poses | Se calculaba con `arccos`, que pierde precisión cerca de 0° y 180° | Se cambió a `atan2(seno, coseno)`: el error bajó a 5e-16 m | Siempre `atan2`, nunca `arccos` o `arcsin` solos |
| 193 poses válidas rechazadas por "fuera de límites" | Ruido numérico de 1e-16 al comparar con el límite exacto | Tolerancia de 1e-9 rad en la comparación | Comparar números reales siempre con tolerancia |
| Algunas soluciones giraban J6 una vuelta completa (6.283 rad) sin necesidad | No se probaban los representantes ±2π | Se elige el representante dentro de límites más cercano a la postura actual | J4 y J6 pueden girar más de 360°: hay que elegir bien |
| La web de la defensa mostraba 1 rama de 8 en el 74 % de los casos | Error en la lógica que eliminaba ramas repetidas | Se reescribió el núcleo de la web y se comparó con Python: 99 de 99 verificaciones iguales | Toda reimplementación se compara contra la original |
| En el texto se decía que la dirección perdida en la singularidad era "angular" | Se mezclaron unidades (m/rad con rad/rad) | Se corrigió: sin declarar una escala no se puede decir | No mezclar unidades en una sola norma |
| El pico de torque citado variaba (4.8909 vs 4.8927 N·m) | El muestreo no caía justo en el instante del pico | Se citó "≈ 4.89 N·m al final del crucero (t ≈ 2 s)" | Citar con las cifras que el muestreo garantiza |

Estado: HECHO (defendido).

## Parte C. El robot real: identificación y conexión (noviembre)

Esta es la parte del Hito 3. Todo lo que sigue se hizo en el laboratorio o en el Ubuntu de Gor.

### C1. Identificación del equipo (HECHO)

De las fotos de las etiquetas se sacó:

| Equipo | Dato |
|---|---|
| Robot | ABB IRB 120-3/0.6, serie 120-101516, fabricado el 2018-01-10, inventario UCB T5565 |
| Controlador | IRC5 Compact |
| FlexPendant | DSQC 679 (3HAC028357-001) |
| Valores de calibración (resolver) | 5.7978 / 1.0574 / 1.9194 / 3.9979 / 5.2596 / 3.8435 |
| Límites | J1 ±165°, J2 ±110°, J3 +70°/−110°, J4 ±160°, J5 ±120°, J6 ±400° |
| Velocidades máximas | 250, 250, 250, 320, 320, 420 °/s |

### C2. Software del controlador (HECHO)

Desde el FlexPendant (Menú → Información del sistema) se confirmó:

- RobotWare **6.12.01.00**. El driver de ROS 2 (`abb_ros2`) necesita RobotWare 6 (RWS 1.0): cumple.
- Opción **689-1 Externally Guided Motion (EGM)**: es la que deja a ROS mandar posiciones en tiempo real. Está.
- Opción **616-1 PC Interface**: es la que deja a una PC hablar con el controlador por red (RWS). Está.
- **UDPUC Driver** (lo usa EGM) y el Add-In **StateMachine 1.1** (otra forma de arrancar EGM desde ROS).
- Puerto de servicio: **192.168.125.1**, máscara 255.255.255.0. La PC va en **192.168.125.10**.

Observaciones que quedaron anotadas: el FlexPendant marcaba 3196 días desde el último mantenimiento (hay que avisar al encargado) y el robot tenía solo 89 horas de uso.

Falta: confirmar que el backup del controlador se hizo de verdad (solo se vio la pantalla con la carpeta propuesta) y copiarlo a la PC. Estado: HECHO, FALTA EVIDENCIA.

### C3. El Ubuntu de Gor (HECHO)

1. Primero se intentó con Ubuntu 26.04, pero ROS 2 Jazzy y el driver están probados en 24.04. Se cambió a **Ubuntu 24.04 en un disco externo**.
2. Se instaló **ROS 2 Jazzy**.
3. Se instaló **Claude Code** en la terminal, para que haga el trabajo dentro de la PC (el chat de claude.ai no puede tocar esa PC).
4. Se descartó un paquete `claude-desktop` no oficial que se había instalado: no es de Anthropic.

### C4. Paquete del IRB 120 para ROS 2 (HECHO, FALTA EVIDENCIA)

El driver `abb_ros2` de PickNik solo trae la configuración del IRB 1200. Claude Code creó en `~/abb_ws/src/abb_irb120` el paquete **`abb_irb120_support`** para nuestro robot y lo compiló. También hizo un launch que **solo lee** las posiciones de las juntas por RWS y las muestra en RViz, sin mandar movimientos.

En esa lectura se vio que J4, J5 y J6 cambiaban entre 5° y 38°. Lo más probable es que alguien haya movido la muñeca con el joystick del FlexPendant. Hay que comparar esos valores con el FlexPendant la próxima vez.

Falta guardar en la bitácora: el log de esa lectura, la hora y una captura del FlexPendant con los mismos valores. Eso convierte la lectura en evidencia S1/S2.

### C5. Errores que pasaron en esta etapa

| Qué pasó | Por qué | Qué hacer |
|---|---|---|
| `d: no se encontró la orden` | Se escribió `d` en vez de `cd`, y se pegó texto del chat en la misma línea | Escribir un comando por línea; copiar solo lo que está dentro del bloque de código |
| `cp: no se puede efectuar stat ... Descargas/hito3-hito4-rubricas-y-tareas.md` | El archivo se había descargado en el celular, no en la PC | Bajar los archivos desde la PC, o usar el repo (por eso ahora todo va al repo) |
| Ventana de Ubuntu "robot_state_publisher se cerró inesperadamente" | Claude Code cortó un launch a propósito con `timeout` | No es un error: pulsar "No enviar" |
| Prueba de red "403" desde el chat | Esa prueba se hizo desde la nube de Claude, no desde la PC del laboratorio | Las pruebas de red siempre desde el Ubuntu conectado al cable |
| Scripts `saludo.py` y `driver_espera.sh` | Claude Code los preparó y los borró antes de usarlos | Confirmar en la bitácora que no llegaron a mover el robot |

### C6. Rúbrica y plan del Hito 3/4 (HECHO)

Se leyó el repositorio del docente (`berquiroga/planificacion-UCB`, semanas 12 y 14). Lo importante:

- El Hito 3 y el Hito 4 se juntaron en un solo elemento, el **ECI-3/4**, que vale el **50 % de la evaluación continua**.
- La cámara pasó a ser **opcional**.
- Rúbrica: ingeniería y compatibilidad 20 %, modelos propios y validación FK/IK/J 25 %, integración ROS–ABB 25 %, seguridad y reproducibilidad 20 %, defensa 10 %.
- El docente no acepta "funcionó en RViz": cada afirmación necesita su nivel de evidencia y una entrada de bitácora.

### C7. Lo que pidió el docente esta semana

1. Mostrar **en tiempo real** todos los cálculos vistos en clase y el **error en tiempo real** (mensaje de Florencia).
2. Primero calcular posiciones y ángulos, elegir la mejor rama comprobando que el error sea el mínimo, y **recién ahí** mandar al robot.
3. El GO lo emite el grupo ("simulen ustedes el GO"), la trayectoria la genera la IA, y hay que mandarle un **video del robot físico moviéndose** mientras se mandan los datos.

Eso es exactamente lo que hace la **consola VEVAS** (carpeta `hito3_4/`), que está en este repo.

## Parte D. Lo que se agregó ahora (esta entrega)

| Qué | Dónde | Estado |
|---|---|---|
| Núcleo de cálculo propio: FK, Pieper 8 ramas, selección de rama, Jacobiano, LSPB, seguridad | `hito3_4/vevas/` | HECHO, 18 pruebas automáticas pasan |
| Consola gráfica en tiempo real (3D, cálculos paso a paso, 8 ramas, LSPB, error en vivo, flechas del teclado) | `hito3_4/consola_vevas.py` y `hito3_4/web/` | HECHO en simulación; falta probar con ROS en tu Ubuntu |
| Validación offline S0 contra el URDF de ROS-Industrial | `hito3_4/docs/datos/validacion_offline/` | HECHO: error 6e-16 en 5000 posturas |
| Herramienta de solo lectura FK propia vs controlador | `hito3_4/herramientas/verificar_convencion_rws.py` | Lista; falta correrla con el robot |
| Análisis automático de sesiones | `hito3_4/herramientas/analizar_sesion.py` | HECHO |
| RAPID de EGM sin movimiento al arrancar | `hito3_4/rapid/TRob1Main_VEVAS.mod` | Listo; lo carga el encargado |
| Plantilla del GO | `hito3_4/plantillas/GO_VEVAS_plantilla.json` | Lista |

Un dato importante que salió de la validación offline: **nuestra tabla D-H y el URDF de ROS-Industrial dan exactamente la misma pose** (diferencia de 6e-16 en 5000 posturas al azar), siempre que el marco 6 de D-H se gire 180° en z para llegar al tool0 de ABB. Eso quiere decir que el ángulo de cada junta en ROS y en el FlexPendant es el mismo q de nuestra tabla: signo +1 y offset 0 en las seis juntas. Falta confirmarlo con el robot real (Guía 2, paso 5).

## Parte E. Qué falta, en orden

| # | Tarea | Dónde está explicada | Nivel que da | Bloqueo |
|---|---|---|---|---|
| 1 | Probar la consola en tu Ubuntu con `use_fake_hardware:=true` | Guía 5 | S0 | ninguno |
| 2 | Red PC ↔ IRC5: `ping` y `curl`, guardando la salida | Guía 2, pasos 1 a 4 | S1 | ninguno (sin movimiento) |
| 3 | FK propia vs controlador con `verificar_convencion_rws.py`, en 3 posturas | Guía 2, paso 5 | S1 (dominio 2) | ninguno (solo lectura) |
| 4 | Driver con el robot real y la consola en solo lectura | Guía 2, paso 9 | S2 | EGM configurado |
| 5 | Configurar EGM en el controlador (protocolo ROB_1 y RAPID) | Guía 2, pasos 6 a 8 | — | **encargado del laboratorio** |
| 6 | GO del día lleno y firmado | Guía 6 | dominio 4 | docente (dio la instrucción) |
| 7 | Prueba junta por junta en el robot real | Guía 4 | S3 | pasos 4 a 6 |
| 8 | Secuencia completa grabada en video | Guía 6 | S3 | paso 7 |
| 9 | Analizar la sesión y llenar la bitácora | Guía 7 | dominio 4 | ninguno |
| 10 | Ensayar la presentación | Guía de presentación | dominio 5 | ninguno |

## El siguiente paso, hoy mismo

1. Bajar este repo en el Ubuntu (`git pull`).
2. Correr la consola en modo simulación y jugar con ella 15 minutos (Guía 5, sección 2). Así todos entienden la interfaz antes de ir al robot.
3. Pegarle a Claude Code el mensaje de `MENSAJE_PARA_CLAUDE_CODE.md` para que la pruebe con ROS y el hardware simulado y documente el resultado.
4. Pedir al encargado del laboratorio un horario para los pasos 2 a 5 (los de solo lectura se pueden hacer el mismo día, antes de configurar EGM).
