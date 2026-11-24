# Guía completa del Hito 3/4: VEVAS Studio y el ABB IRB 120 real

Grupo VEVAS: Susann A. Baldiviezo Castellanos, Alejandro J. Bejarano Romero, Florencia Frigerio Arízaga, Benjamín Soruco Guzmán.
Ingeniería Mecatrónica, Depto. de Ciencias de la Tecnología e Innovación, UCB "San Pablo", Sede Tarija.
IMT-342 Robótica, docente M.Sc. Ing. Bernardo Quiroga Turdera. Repositorio: github.com/FlorenciaFrigerio/pfi-vevas-irb120.
Estado al 24/11/2026.

Esta guía junta en un solo lugar qué hace el sistema, cómo se usa, cómo está construido, por qué se decidió cada cosa, los cálculos, los resultados, los incidentes y lo que falta. Los números salen de la bitácora del 24/11 y del informe IEEE del hito (`docs/informe_ieee_hito3_4/informe_hito3_4_ieee.tex`); los ejemplos extra se verificaron con los módulos de `vevas/`.

## Índice

1. Qué hace el sistema, de punta a punta
2. Cómo se usa paso a paso
3. Cómo lo hicimos: arquitectura y archivos
4. Glosario de conceptos
5. Por qué cada decisión
6. Lenguajes, herramientas y protocolos
7. Cálculos con fórmulas y ejemplos numéricos
8. Resultados y evidencias
9. Incidentes y cómo se resolvieron
10. Limitaciones y próximas mejoras
11. Banco de 20 preguntas del docente

## 1. Qué hace el sistema, de punta a punta

### 1.1 La idea en una frase

VEVAS Studio es una consola que corre en nuestra PC con Ubuntu, muestra un gemelo 3D del IRB 120 en el navegador, calcula los movimientos con **nuestros propios modelos** de los Hitos 1 y 2 (FK, Pieper, Jacobiano, trayectorias) y, si todas las compuertas de seguridad están en verde, le manda al robot real objetivos articulares a través de Robot Web Services (RWS). En el controlador corre un módulo RAPID nuestro (`VEVAS_RWS`) que vuelve a validar cada objetivo y decide él mismo cuánto tiempo tarda el movimiento, de modo que ninguna junta puede ir más rápido que el 5 % de su velocidad de ficha.

### 1.2 Las piezas

1. **El robot**: ABB IRB 120 con controlador IRC5 Compact, RobotWare 6.12.01, en la IP 192.168.125.1 (puerto SERVICE).
2. **La PC**: Ubuntu 24.04 con ROS 2 Jazzy, interfaz `eno2` con 192.168.125.207/24, conectada por cable al puerto SERVICE.
3. **La consola Python** (`consola_vevas.py` + paquete `vevas/`): un servidor HTTP local (puerto 9012 con robot real, 9013 en simulación cuando se usan los dos a la vez) que concentra modelos, revisión y compuertas.
4. **La web** (`web/`): página estática con Three.js. Tiene el gemelo 3D, flechas y anillos para arrastrar el TCP, y pestañas Control, Gráficas, Datos en vivo, Cálculos, 8 ramas, Trayectoria, FK en vivo, Jacobiano, Demo e Historial.
5. **El backend de movimiento**: hay tres intercambiables. SIM (simulador interno), ROS 2 (opcional, vía `EGM_ROS` y RViz) y RWS (el que usamos con el robot real).
6. **El módulo RAPID** `VEVAS_RWS.mod`, con la rutina `VevasServidor`, que corre en el IRC5 y obedece solo a variables `PERS` que escribe la PC.
7. **El paro de emergencia físico**, que es la parada real y no depende de ningún software.

### 1.3 El recorrido de un movimiento, contado simple

1. La consola lee las seis juntas del robot por RWS diez veces por segundo y las pinta en el gemelo 3D. Con esas juntas calcula la FK propia y la muestra al lado de la del controlador.
2. El usuario pide un objetivo: arrastra el TCP con las flechas, escribe una pose, mueve «Una junta», elige «Volver a cero», «Preparar J5 a +30°» o una figura (cuadrado, círculo, saludo, corazón, triángulo).
3. **Pieper** resuelve la cinemática inversa y devuelve las 8 ramas. Cada rama se reinyecta en la FK (**verificación FK**): solo sirve si el error de posición es menor a 1e-6 m y el de orientación menor a 1e-5 rad.
4. Se elige la **rama** más cercana a la postura actual que respete límites y no sea singular.
5. Se arma la **trayectoria** articular con un perfil suave (LSPB o quíntico).
6. La **revisión previa** (`vevas/revision.py`) muestrea el plan completo (hasta 240 muestras) y revisa límites, velocidad, singularidad (σmin), altura del TCP, autocolisión, mesa/estructura y herramienta. Resultado: aprobado, aviso, rechazado o no_verificado.
7. Si el plan no está rechazado ni sin verificar, el usuario **arma** el sistema (checklist + frase «MOVER ROBOT REAL»), confirma y pulsa **Enviar**.
8. La trayectoria se parte en tramos colineales de hasta 40°. Por cada tramo, la PC escribe `vvPcTarget`, lo relee, comprueba que el robot no se movió y recién entonces incrementa `vvPcSeq`.
9. El RAPID ve el nuevo número de secuencia, valida otra vez (modo, límites con margen 2°, paso máximo 45°), calcula el tiempo `t` con la ley C02 y ejecuta `MoveAbsJ objetivo, vPc \T:=t, fine, tool0`.
10. El RAPID contesta con `vvRbCodigo` (1 moviendo, 2 llegó o un código negativo de rechazo). La PC da el tramo por cumplido solo si la secuencia coincide, el código es 2 y la postura **medida** está a menos de 0,05° del objetivo.
11. Todo queda en el **Historial** y en los archivos de sesión.

Los ocho pasos que ve el usuario en la interfaz son: Objetivo, Pieper, Verificación FK, Rama, Trayectoria, Revisión, Envío, Resultado.

### 1.4 Qué no hace (a propósito)

1. No mueve el robot si falta cualquiera de estas cosas: GO vigente, evidencia C02 válida para el módulo cargado y el modo actual, revisión no rechazada, sistema armado.
2. No interpola en cartesiano dentro del robot: siempre manda **objetivos articulares** (`MoveAbsJ`), así que el controlador nunca invierte el Jacobiano.
3. No toca los módulos de otros grupos (`LabJointRws`, `EGM_ROS`, `Imt342TrajectoryRapid`) ni arranca encima de un RAPID ajeno.
4. No cambia el modo AUTO/MANUAL ni enciende motores tras un reinicio: eso lo hace una persona.
5. No reemplaza el paro de emergencia. El botón PARAR (o Esc) detiene RAPID por RWS: corta los tramos siguientes, pero el tramo en curso termina. El freno real es el paro físico.

### 1.5 Qué se demostró el 24/11/2026

1. La FK propia coincide con la del controlador: 0,0000 mm de diferencia (0,0001 mm en la primera lectura) con `tool0`, `wobj0` y base.
2. C02 verificado en el robot real en AUTO: velocidades máximas de 5,2 a 8,9 °/s contra topes de 12,5 a 21 °/s; pico medido ≈ 1,06 veces el medio.
3. 265 pruebas automáticas en Python y 10 en JavaScript, todas pasan.
4. La postura HOME (q = 0) es singular de muñeca (σmin ≈ 0) y el sistema sabe entrar y salir de ella de forma controlada.

## 2. Cómo se usa paso a paso

Hay dos lugares desde donde se corren comandos:

1. La **raíz del proyecto** en la PC del laboratorio: `~/Descargas/ROBOTICA_01/proyecto_brazo`. Ahí están los scripts de seguridad `05_seguridad_y_control/verificar_c02.sh` y `05_seguridad_y_control/abrir_control_real.sh`.
2. La **carpeta del código**: `~/Descargas/ROBOTICA_01/proyecto_brazo/00_codigo/hito3_4` (en el repositorio es `hito3_4/`). Ahí están `consola_vevas.py`, `herramientas/`, `rapid/` y `web/`.

La clave de RWS no se escribe en ningún comando de esta guía. Si hace falta, se carga en la variable de entorno `VEVAS_RWS_CLAVE` en la terminal de la sesión, sin guardarla en archivos del repositorio.

### 2.1 Preparar el entorno (una sola vez)

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo/00_codigo/hito3_4
python3 -m venv .venv_interfaz
.venv_interfaz/bin/python -m pip install -r requirements.txt
```

Comprobar que todo pasa antes de ir al laboratorio:

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo/00_codigo/hito3_4
.venv_interfaz/bin/python -m pytest -q tests
node --test tests/test_calculos.mjs tests/test_orientacion.mjs
```

Esperado: 265 pruebas Python y 10 JS aprobadas.

### 2.2 Simulación (sin robot)

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo/00_codigo/hito3_4
.venv_interfaz/bin/python consola_vevas.py --modo sim --puerto 9013 --abrir
```

Se abre `http://127.0.0.1:9013`. La cabecera dice SIMULACIÓN. Ahí se ensaya todo (arrastre, figuras, Volver a cero, Preparar J5) sin riesgo. El script `bash iniciar_simulacion.sh` hace lo mismo en el puerto 9012 si se usa el entorno `.venv`.

### 2.3 Diagnóstico de conexión (solo lectura, no mueve nada)

Primero, la red. La PC debe tener 192.168.125.207/24 en `eno2`.

```bash
ip -4 addr show eno2
ping -c 3 192.168.125.1
```

Después, RWS en solo lectura:

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo/00_codigo/hito3_4
python3 herramientas/conectar_robot.py --ping
python3 herramientas/rws_irc5.py estado
bash iniciar_real.sh --solo-chequeo
```

`estado` imprime modo (AUTO/MANR), estado de motores, si RAPID corre, dónde está el PP, si el módulo `VEVAS_RWS` está cargado y las seis juntas.

Comparar nuestra FK con la del controlador (lo que mostramos como «FK propia vs FK del controlador»):

```bash
python3 herramientas/comparar_pendant.py
```

Debe decir que coincide (el 24/11 dio 0,0000 mm). Para ver los últimos eventos del controlador (elog):

```bash
python3 herramientas/rws_irc5.py diagnostico -n 8
```

Para solo mirar el robot en la web y en RViz, sin poder mandar nada:

```bash
bash iniciar_real.sh --solo-lectura            # web + RViz
bash iniciar_real.sh --solo-lectura --sin-rviz # solo web
```

### 2.4 Verificar C02 (una vez por versión del módulo y por modo)

Condiciones antes de correrlo:

1. Llave del IRC5 en **AUTO** (la gira una persona; ningún script cambia el modo).
2. Célula despejada, brida sin herramienta (o herramienta medida).
3. **Una persona con la mano en el paro de emergencia.**
4. Robot en una postura segura, lejos de límites y de la mesa (por ejemplo J1 a J3 cerca de 0 y J5 ≥ 30°).
5. Nadie más con RAPID corriendo.

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo
bash 05_seguridad_y_control/verificar_c02.sh
```

Qué pasa:

1. Muestra el estado y pide `s` para mover.
2. Si hay un `VevasServidor` viejo corriendo, lo detiene; carga el `VEVAS_RWS.mod` actual si falta o cambió, y arranca RAPID.
3. En el FlexPendant aparece «Aceptar objetivos enviados por la PC?»: pulsar **Iniciar** (hay 30 s).
4. Mueve J1 a J6 de a una, +5° y vuelta (o −5° si +5° se sale de límites). Por cada tramo imprime el tiempo que usó el RAPID y la velocidad máxima medida contra el tope.
5. Guarda `00_codigo/hito3_4/evidencia_c02.json`. Si dice **C02 VERIFICADO**, se puede pasar al control real.

Variantes:

```bash
bash 05_seguridad_y_control/verificar_c02.sh --delta 3          # pasos de 3°
bash 05_seguridad_y_control/verificar_c02.sh --juntas 4,5,6     # solo algunas juntas
```

Cuándo hay que repetirlo: si se cambia un solo carácter de `VEVAS_RWS.mod`, si se pasa de AUTO a manual reducido, o si se cambian vLim o kPico.

### 2.5 Abrir el control real

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo
bash 05_seguridad_y_control/abrir_control_real.sh
```

El script:

1. Exige que exista `evidencia_c02.json`; si no, pide correr primero `verificar_c02.sh`.
2. Valida el GO de la sesión. Si no hay GO válido de hoy, lanza `crear_go.py` en modo interactivo.
3. Arranca `consola_vevas.py --modo rws --real --go GO_VEVAS.json --preparar-rapid` en el puerto 9012 y abre el navegador.

En `http://127.0.0.1:9012` la cabecera debe decir **IRC5 vía RWS · ROBOT REAL** y no debe aparecer el aviso de C02. Si `--preparar-rapid` tiene que arrancar `VevasServidor`, el FlexPendant vuelve a pedir **Iniciar**.

### 2.6 Crear el GO de la sesión

Se puede crear a mano (o lo hace `abrir_control_real.sh` si falta):

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo/00_codigo/hito3_4
python3 herramientas/crear_go.py --salida GO_VEVAS.json
python3 herramientas/crear_go.py --salida GO_VEVAS.json --reemplazar   # si ya existe uno viejo
```

Pregunta, una por una: referencia de la autorización del responsable, operador del FlexPendant, responsable de consola, supervisor, alcance de los ensayos, velocidad máxima autorizada (en % y sin pasar el tope de `config.json`), cada punto del checklist (sí/no), si no hay objetos externos aparte de la mesa modelada y si la brida está sin herramienta. Solo queda **GO** si todo el checklist es «sí»; si no, guarda **NO-GO**. El GO vence (vale para la fecha del día) y nunca se genera automático. Responder solo lo verificado en persona. El GO no se sube al repositorio.

### 2.7 Armar, calcular, revisar y enviar (en la web)

1. Mirar la cabecera: modo ROBOT REAL, juntas actualizándose, estado **DESARMADO**.
2. Pulsar **ARMAR**: marcar el checklist y escribir exactamente `MOVER ROBOT REAL`. El estado pasa a **ARMADO**.
3. Elegir el objetivo. Para el primer movimiento, algo chico: «Una junta» de 5° o un arrastre corto del TCP.
4. Calcular. Revisar en orden las pestañas: Pieper (8 ramas), Verificación FK (errores ~1e-15 m), rama elegida, trayectoria.
5. Abrir la **Revisión**. Si sale **rechazado** o **no_verificado**, no se puede enviar; leer el motivo. Si sale **aviso**, leerlo y decidir.
6. Pulsar **Enviar**, leer el resumen en el diálogo «Confirmar envío al robot real» y confirmar.
7. Mirar el robot, no la pantalla, con la mano cerca del paro. El paso **Resultado** muestra la postura final medida y el error.

### 2.8 PARAR

1. Botón **■ PARAR** en la cabecera, o teclas **Esc** o **Espacio**.
2. Detiene RAPID por RWS: no se mandan más tramos. El tramo en curso puede terminar.
3. Ante cualquier duda real: **paro de emergencia físico**. Después hay que rearmar en el controlador (reset del paro y MOTORS ON por el operador).

### 2.9 Volver a cero

Botón **Volver a cero** (todas las juntas a 0°). No recalibra nada: es un plan articular revisado como cualquier otro. Como q = 0 es singular de muñeca (q5 = 0), la revisión lo da como **aviso** solo si el backend es RWS y C02 está verificado: el movimiento es articular y con velocidad por junta acotada. Desde la postura inicial [0, −20, 30, 0, 40, 0]° es un solo tramo de 6,25 s (domina J5 con 40°). Sin esas condiciones queda **rechazado**.

### 2.10 Preparar J5 a +30°

Botón **Preparar J5 a +30°**. Sirve para salir de HOME: deja q5 = 30° y σmin sube monótonamente de 0 a 0,231. La revisión lo da como **aviso** «sale de singularidad de muñeca» porque cumple el criterio de salida (zona singular solo al principio, solo muñeca, σ creciente desde el mínimo). Tarda 4,69 s. Es la postura recomendada antes de arrastrar el TCP o de hacer figuras.

### 2.11 Figuras

Se abren desde «Poses predeterminadas»: figura, tamaño en mm (5 a 100 en SIM) y plano base (XY, XZ o YZ). En el robot real el tamaño máximo es **80 mm** y exigen RWS con C02 verificado.

| Figura | Qué hace | Detalle |
|---|---|---|
| Cuadrado | Lado = tamaño, empieza y termina en el TCP actual | Paradas en las 4 esquinas |
| Círculo | Diámetro = tamaño | Recorrido continuo, frena entre puntos no colineales |
| Saludo | Movimiento **articular** de muñeca (no cartesiano) | No pasa por Pieper punto a punto |
| Corazón | Curva paramétrica clásica escalada al tamaño | Parada en la cúspide |
| Triángulo | Equilátero de lado = tamaño, un vértice en el TCP | Nuevo; paradas en los 3 vértices |

Cómo se calcula cada figura cartesiana: se generan entre 65 y 129 puntos relativos al TCP; Pieper resuelve cada punto con la orientación constante; se exige que el salto articular entre puntos sea como máximo 12°; se arma la trayectoria y pasa por la misma revisión. Recomendado: empezar con 25 mm en SIM, después en real desde la postura «Preparar J5 a +30°».

### 2.12 Qué hacer ante cada error típico

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| `ping` falla | Cable, IP de `eno2` o puerto equivocado | Conectar al puerto SERVICE; poner 192.168.125.207/24 en `eno2` |
| Ping OK pero HTTP 000 / RWS no responde | Servicio RWS colgado | STOP en el FlexPendant, reiniciar controlador (ver incidente 2) |
| HTTP 401 | Usuario o clave de RWS mal cargados | Revisar `VEVAS_RWS_CLAVE` en la terminal; no ponerla en archivos |
| HTTP 500 «Error registering callback» | Sesiones RWS agotadas | La consola hace `/logout` y reintenta; si sigue, cerrar otras herramientas y esperar |
| HTTP 403 al encender motores | Tras reinicio RWS no puede MOTORS ON | El operador pulsa el botón blanco MOTORS ON (la consola espera 60 s) |
| «RAPID está ejecutando otro programa» | Otro grupo tiene RAPID corriendo | No arrancar encima; pedirles que lo detengan |
| No aparece «Iniciar» o se canceló | Pasaron los 30 s o se pulsó Cancelar | Volver a correr el script y pulsar Iniciar a tiempo |
| Aviso de C02 en la cabecera / envío bloqueado | Falta evidencia, el módulo cambió o el modo no es el verificado | Correr `verificar_c02.sh` en el modo actual |
| C02 NO pasó | Alguna junta superó vLim o `vvRbTiempo` no coincidió | No mover; guardar la salida completa y revisarla |
| GO inválido o vencido | Fecha de otro día, NO-GO o tope de velocidad bajo | `crear_go.py --reemplazar` respondiendo solo lo verificado |
| GO autoriza menos del 5 % | El tope del GO es menor que vLim del módulo | No se envía; habría que bajar vLim en el `.mod` y verificar de nuevo |
| Revisión «rechazado» | Límite, singularidad, mesa, autocolisión o altura | Leer el motivo; cambiar objetivo o rama; salir de HOME con «Preparar J5» |
| Revisión «no_verificado» | Algo no se pudo evaluar (p. ej. herramienta sin medir) | En real bloquea; completar el dato o no mover |
| «Postura cambió» | El robot se movió desde que se calculó el plan | Recalcular |
| Watchdog: muestra vieja | La lectura de juntas dejó de llegar | Revisar RWS; no enviar hasta que se actualice |
| Código −1 | Objetivo vacío (9E9) | Recalcular y enviar de nuevo |
| Código −2 | Objetivo fuera de límites con margen 2° | Cambiar objetivo o rama |
| Código −3 | Paso mayor a 45° en alguna junta | No debería pasar (la PC parte en 40°); reportar |
| Código −4 | Modo no permitido | Poner AUTO o manual reducido según lo verificado |
| Evento 20054 al reiniciar | Programa en ejecución | STOP en el FlexPendant y luego reiniciar |
| Evento 20082 «Battery backup lost» | Batería SMB débil, contadores perdidos | Llevar ejes a marcas de sincronización y actualizar contadores (incidente 3) |
| Pausas en recorridos largos | Cada tramo termina en `fine` | Ya se usan tramos de 40°; las figuras siguen frenando entre puntos (ver sección 10) |

## 3. Cómo lo hicimos: arquitectura y archivos

### 3.1 Arquitectura en capas

```text
 Navegador (web/: Three.js, uPlot, KaTeX)
      |  HTTP local: GET estado (SSE), POST órdenes (armar, calcular, revisar, enviar, parar)
      v
 consola_vevas.py  ->  vevas/servidor.py  ->  vevas/consola.py (lógica central)
      |                                          |
      |        modelos propios (Hitos 1 y 2):    |  compuertas:
      |        robot_irb120.py (FK D-H)          |  seguridad.py (GO, config, semáforo)
      |        pieper.py (IK 8 ramas)            |  revision.py (revisión previa)
      |        jacobiano.py (J, sigma_min)       |  c02.py (evidencia de velocidad)
      |        trayectoria.py (LSPB/quíntico)    |  registro.py (sesión, historial)
      |        figuras.py, mesa.py               |
      v
 Backend intercambiable:  backend_sim.py | backend_ros.py | backend_rws.py
                                                      |
                                                      v
                                     vevas/rws_cliente.py (HTTP + digest + cookies)
                                                      |
                                    red SERVICE 192.168.125.0/24
                                                      v
                       IRC5 Compact, RobotWare 6.12.01, RWS 1.0
                       rapid/VEVAS_RWS.mod -> VevasServidor -> MoveAbsJ \T
                                                      |
                                                      v
                                     ABB IRB 120  (paro de emergencia físico)
```

La idea clave es que **los modelos son comunes a los tres backends**: lo que se prueba en simulación es exactamente el mismo código que mueve el robot. Solo cambia la última capa.

### 3.2 Flujo de datos

1. **Lectura** (siempre): `backend_rws.py` pide `/rw/motionsystem/mechunits/ROB_1/jointtarget` a 10 Hz, la consola calcula FK, σmin y estado, y lo empuja a la web por SSE.
2. **Plan**: la web manda el objetivo por POST; la consola ejecuta Pieper, verificación FK, elección de rama, trayectoria y revisión, y devuelve todo con números para las pestañas.
3. **Envío**: con todas las compuertas en verde, la consola parte la trayectoria en tramos de hasta 40° y `rws_cliente.py` hace el protocolo PERS tramo por tramo.
4. **Registro**: cada sesión queda en un archivo de sesión (`registro.py`), que después se puede analizar con `herramientas/analizar_sesion.py`.

### 3.3 Qué hace cada archivo importante

| Archivo | Qué hace |
|---|---|
| `consola_vevas.py` | Punto de entrada. Lee argumentos (`--modo sim/ros/rws`, `--real`, `--go`, `--preparar-rapid`, `--puerto`, `--abrir`) y levanta el servidor |
| `vevas/servidor.py` | Servidor HTTP local con la biblioteca estándar: sirve `web/`, emite el estado por SSE y recibe órdenes por POST |
| `vevas/consola.py` | Lógica central: estado, armado (frase «MOVER ROBOT REAL»), cálculo del plan de 8 pasos, envío, PARAR, validación del GO |
| `vevas/robot_irb120.py` | Modelo D-H del IRB 120 (tabla, límites, velocidades de ficha), FK a `tool0` y al TCP, utilidades de orientación |
| `vevas/pieper.py` | IK analítica por desacople de Pieper: 8 ramas, verificación FK de cada una, elección de rama |
| `vevas/jacobiano.py` | Jacobiano geométrico, verificación por diferencias finitas, σmin escalado y tipo de singularidad |
| `vevas/trayectoria.py` | Perfiles LSPB y quíntico para seis juntas, sincronizados, con cálculo de picos |
| `vevas/figuras.py` | Genera los puntos del círculo, corazón, cuadrado y triángulo relativos al TCP |
| `vevas/secuencias.py` | Secuencia de demostración (pestaña Demo) |
| `vevas/mesa.py` | Geometría de la mesa/estructura en el marco base, para la revisión de entorno |
| `vevas/revision.py` | Revisión previa del plan: límites, velocidad, singularidad, altura, autocolisión por cápsulas, entorno y herramienta; estados y excepciones de muñeca |
| `vevas/seguridad.py` | Configuración por defecto, carga de `config.json`, comprobaciones de plan y semáforo de error |
| `vevas/c02.py` | Calcula velocidades máximas (ventana 0,10 s) y evalúa si la evidencia C02 vale para el módulo y el modo actuales |
| `vevas/backend_sim.py` | Simulador interno (sin ROS) con reloj propio |
| `vevas/backend_ros.py` | Conexión opcional con ROS 2 Jazzy (driver `abb_ros2` / EGM) |
| `vevas/backend_rws.py` | Backend del robot real: lectura a 10 Hz, watchdog, compuerta C02, envío por tramos |
| `vevas/rws_cliente.py` | Cliente RWS: autenticación digest, cookies, reintento con `/logout`, instalar/cargar módulo, start/stop, escritura y lectura de PERS, candado de archivo |
| `vevas/verificacion_pendant.py` | Compara la pose de la brida del IRC5 con la FK propia (solo lectura) |
| `vevas/registro.py` | Guarda el registro de cada sesión (evidencia para la bitácora) |
| `rapid/VEVAS_RWS.mod` | Módulo RAPID propio (4123 caracteres): `VevasServidor`, validación, ley de tiempo C02, `MoveAbsJ \T` |
| `rapid/TRob1Main_VEVAS.mod` | Variante de `TRob1Main` de `abb_ros2` para la vía EGM/ROS (opcional) |
| `web/index.html`, `web/estilo.css` | Estructura y estilo de la interfaz (cabecera, pestañas, diálogos) |
| `web/app.js` | Lógica de la interfaz: recibe el estado por SSE, manda órdenes por POST, flujo de 8 pasos, PARAR con Esc |
| `web/escena.js`, `web/escena_worker.js` | Gemelo 3D con Three.js (las matrices vienen calculadas por Python) |
| `web/arrastre.js`, `web/orientacion.js` | Flechas y anillos para mover el TCP destino; orientación ZYX |
| `web/calculos.js` | Muestra los cálculos con sus números (KaTeX): FK, ramas, Jacobiano |
| `web/graficas.js` | Gráficas con uPlot: error en vivo y trayectorias |
| `herramientas/verificar_c02.py` | Verificación presencial de C02 y escritura de `evidencia_c02.json` |
| `herramientas/crear_go.py` | Crea el GO de la sesión de forma interactiva (nunca automática) |
| `herramientas/rws_irc5.py` | Ayudante de terminal: `estado`, `instalar`, `arrancar`, `servidor`, `parar`, `pp-main`, `texto`, `diagnostico` |
| `herramientas/conectar_robot.py` | Diagnóstico de red y RWS en solo lectura |
| `herramientas/comparar_pendant.py` | FK propia contra pose del controlador, sin mover |
| `herramientas/verificar_convencion_rws.py` | Comprueba la convención de marcos entre FK y RWS |
| `herramientas/validacion_offline.py` | Validación S0 de FK, IK y Jacobiano sin robot |
| `herramientas/analizar_sesion.py` | Analiza una sesión grabada y arma un informe |
| `herramientas/nodo_rws_ros.py`, `rviz_rws.launch.py` | Publican `/joint_states` desde RWS y abren RViz (solo visualización) |
| `iniciar_real.sh`, `iniciar_simulacion.sh` | Lanzadores (solo chequeo, solo lectura, simulación) |
| `05_seguridad_y_control/verificar_c02.sh` | Envoltorio de `verificar_c02.py` desde la raíz del proyecto |
| `05_seguridad_y_control/abrir_control_real.sh` | Exige evidencia C02, valida o crea el GO y abre la consola con envío real |
| `config.json` | Parámetros: IP, tope de velocidad real, entorno (mesa, herramienta), umbrales |
| `tests/` | 265 pruebas Python (incluye `mock_rws.py`, un IRC5 simulado por HTTP) y 10 JS |

## 4. Glosario de conceptos

**FK (cinemática directa).** Dadas las seis juntas q, calcula dónde está la brida (`tool0`) y con qué orientación. Se multiplica una matriz D-H por eslabón, A_i = Rz(θ_i)·Tz(d_i)·Tx(a_i)·Rx(α_i), y al final un giro Rz(π) para que el marco 6 coincida con el `tool0` de ABB. Con q = 0 la brida queda en (0,374; 0; 0,630) m.

**Pieper y las 8 ramas.** Método analítico de cinemática inversa que funciona porque los tres últimos ejes del IRB 120 se cortan en un punto (la muñeca). Se separa el problema en dos: la posición de la muñeca define q1, q2, q3 y la orientación define q4, q5, q6. Hay tres decisiones binarias: hombro adelante/atrás (q1 o q1 + π), codo arriba/abajo (signo de δ) y muñeca sin giro/girada (signo de q5). 2 × 2 × 2 = 8 ramas. Todas se calculan con `atan2` para no perder el cuadrante.

**Verificación FK.** Cada rama de Pieper se mete de vuelta en la FK y se compara con la pose pedida. Se acepta si el error de posición e_p < 1e-6 m y el de orientación e_R < 1e-5 rad. En la práctica da ~1e-15 m, que es error de redondeo de la máquina.

**Jacobiano.** Matriz 6×6 que relaciona velocidades articulares con la velocidad del TCP: v = J(q)·q̇. Usamos el geométrico: las tres primeras filas son lineales (m/s) y las tres últimas angulares (rad/s).

**σmin.** El valor singular más chico del Jacobiano. Mide qué tan cerca está el robot de perder una dirección de movimiento. Como las filas lineales y angulares tienen unidades distintas, se divide la parte lineal por una longitud característica L = 0,3 m antes de calcularlo. Umbrales: σmin < 0,02 es **singular**, σmin < 0,04 es **cerca**.

**Singularidades.** Posturas donde det J = 0 y el robot pierde un grado de libertad. Para el IRB 120 el determinante se factoriza como det J = −a2·L3·sin δ·r_W·sin q5, así que hay tres tipos:
1. **Muñeca**: sin q5 ≈ 0 (ejes 4 y 6 alineados). Umbral |sin q5| < 0,09.
2. **Codo**: sin δ ≈ 0 (brazo estirado o plegado). Umbral |sin δ| < 0,09.
3. **Hombro**: r_W ≈ 0 (muñeca sobre el eje de J1). Umbral r_W < 0,03 m.

**LSPB.** Linear Segment with Parabolic Blends: perfil trapezoidal de velocidad (acelera, crucero, frena). Es el del Hito 2.

**Quíntico.** Polinomio de grado 5 con posición, velocidad y aceleración nulas en los extremos (continuidad C²). Es el perfil «Suave» de la interfaz. Su pico de velocidad es 1,875 veces la velocidad media.

**RWS (Robot Web Services).** Interfaz HTTP del controlador ABB (versión 1.0 en RobotWare 6). Permite leer juntas, estado, modo, eventos y escribir variables RAPID. Requiere la opción PC Interface (616-1).

**RAPID.** Lenguaje de programación de los robots ABB. El nuestro es el módulo `VEVAS_RWS` con la rutina `VevasServidor`.

**PERS.** Variable persistente de RAPID: conserva su valor y se puede leer y escribir por RWS. Es nuestro «buzón» entre la PC y el robot (`vvPcTarget`, `vvPcSeq`, `vvRbCodigo`, etc.).

**MoveAbsJ.** Instrucción RAPID que lleva el robot a un objetivo **articular** (jointtarget) interpolando en el espacio de juntas. No usa la IK del controlador ni invierte el Jacobiano. Con el argumento `\T:=t` se le impone la duración del movimiento.

**fine / zona.** El último argumento de un movimiento. `fine` significa «llegar exactamente y detenerse». Una zona (por ejemplo `z1`, 1 mm) permite pasar cerca del punto sin parar y empalmar con el siguiente movimiento. Usamos `fine`, por eso hay pausas entre tramos.

**GO.** Archivo JSON de autorización de la sesión (`GO_VEVAS.json`): quién autoriza, operador, supervisor, alcance, tope de velocidad, checklist y fecha. Si no está vigente, no se mueve nada. Se crea solo con `crear_go.py` respondiendo en persona.

**C02.** El requisito de seguridad «ninguna junta supera su tope de velocidad». Lo cerramos con una ley de tiempo dentro del RAPID más una medición presencial guardada en `evidencia_c02.json`.

**Contadores de revoluciones.** El controlador sabe en qué vuelta está cada motor gracias a unos contadores que se mantienen con la batería de la placa SMB. Si la batería falla y se corta la energía, se pierden (evento 20082) y el robot puede creer que está en una postura distinta de la real. Se corrigen llevando cada eje a su marca de sincronización y actualizando contadores desde el FlexPendant.

**AUTO / MANUAL.** Modos de operación del IRC5, elegidos con la llave. AUTO ejecuta el programa sin pulsador de habilitación. Manual reducido (MANR) limita la velocidad a 250 mm/s y exige el pulsador del FlexPendant. Nuestro RAPID acepta AUTO y manual reducido; la evidencia C02 vale solo para el modo en que se midió.

**Mastership.** «Propiedad» de un dominio del controlador (por ejemplo RAPID). Para escribir módulos o variables por RWS hay que pedir mastership (`/rw/mastership/rapid`) y soltarla después; si otro cliente la tiene, se rechaza.

**Digest.** Esquema de autenticación HTTP en el que la clave nunca viaja en texto: el servidor manda un número aleatorio (nonce) y el cliente responde con un hash MD5 de usuario, clave, nonce y la petición. RWS 1.0 lo usa; después de autenticarse se trabaja con cookies de sesión.

**Watchdog de estado.** Vigila la edad de la última muestra de juntas. Si es demasiado vieja, se bloquea el envío.

**Huella SHA-256.** Resumen criptográfico del texto del módulo. Cambiar un carácter cambia la huella, y con eso la evidencia C02 deja de valer.

## 5. Por qué cada decisión

| Decisión | Alternativa descartada | Motivo |
|---|---|---|
| RWS como vía al robot real | EGM con ROS 2 como vía principal | RWS solo necesita PC Interface y un módulo RAPID propio; no interfiere con `EGM_ROS` ni con otros grupos. EGM queda como extensión |
| Objetivos articulares (`MoveAbsJ`) | `MoveL` cartesiano en el robot | `MoveAbsJ` no invierte el Jacobiano: cerca de singularidades no hay velocidades explosivas, y la IK la hace nuestro modelo |
| Pieper analítico con 8 ramas | IK numérica (Newton, pseudoinversa) | Es exacto (~1e-15 m), da todas las soluciones, no depende de un punto inicial y es el modelo propio del Hito 2 |
| Mismos modelos en SIM, ROS y RWS | Un código para simular y otro para el real | Lo validado en simulación es exactamente lo que mueve el robot |
| Tope de velocidad dentro del RAPID | Tope solo en la PC | Un tope en la PC depende de que la PC funcione bien; en el controlador, no. La PC solo replica la ley para estimar duraciones |
| `MoveAbsJ \T:=t` con t calculado | `speeddata` (vPc) | `speeddata` limita la velocidad del TCP y de reorientación, no la de cada eje |
| vLim = 5 % de la ficha | Un tope mayor | Primer contacto con el robot real; margen amplio para observar y frenar |
| kPico = 2,5 | kPico = 1 (tiempo = Δq/vLim) | El perfil interno de ABB tiene pico mayor que la media; con 2,5 el pico teórico queda ≤ 0,75·vLim aun con un quíntico (1,875) |
| Cerrar C02 por «tiempo acotado + medición» | Solo un booleano en el código, o SpeedLimAxis/LimitSpeed en EIO | Se pidió evidencia medida. SpeedLimAxis requiere tocar la configuración del controlador compartido; queda documentada como alternativa oficial |
| Evidencia atada a SHA-256 del módulo, modo y topes | Evidencia sin vínculo | Cambiar el módulo o el modo invalida la autorización automáticamente |
| Protocolo por variables PERS con número de secuencia | Escribir y arrancar una rutina por cada movimiento | Un solo `VevasServidor` corriendo; la escritura de `vvPcSeq` es el único acto que ordena movimiento; hay acuse y código de rechazo |
| Releer `vvPcTarget` antes de incrementar `vvPcSeq` | Escribir y disparar | Asegura que el robot va a leer exactamente lo que la PC quiso escribir |
| Doble validación (PC y RAPID) | Validar solo en la PC | El controlador es la última barrera, no la única. PC: margen 2°, paso 40°; RAPID: margen 2°, paso 45° |
| Acuse por postura medida (< 0,05°) | Confiar en el código 2 | Lo que importa es dónde quedó el robot, no lo que dice el programa |
| `confirmarEnPendant = TRUE` | Arrancar el servidor sin preguntar | Una persona frente al robot tiene que aceptar con «Iniciar» |
| Módulo < 4800 caracteres, todo `LOCAL` o con prefijo `vv` | Módulo largo con nombres libres | RW6 trunca el texto a 5000 al leerlo (y no se podría verificar la huella); los nombres globales chocan con módulos de otros grupos |
| Tramos colineales de hasta 40° | Tramos de 10° | Cada tramo termina en `fine`; con 10° el robot hacía pausas. El tiempo nominal total no cambia |
| `fine` en cada tramo (por ahora) | Zonas `z1` | Con `fine` la ley de tiempo y la medición C02 son simples y verificadas; las zonas cambian el módulo y exigen medir de nuevo |
| Revisión con 4 estados, «no_verificado» bloquea en real | Aprobado/rechazado | «No se pudo evaluar» no es «cumple» |
| σmin con escala L = 0,3 m | σmin del Jacobiano sin escalar | Sin escala se mezclan metros y radianes y el umbral depende de las unidades |
| Excepción estrecha para entrar/salir de q = 0 | Rechazar todo plan singular | Si no, el robot quedaría sin salida desde su propio reposo. Solo muñeca, monótono, contiguo, y solo con RWS + C02 |
| GO interactivo y con fecha | Un `-y` o GO automático | La autorización tiene que ser una declaración consciente de personas presentes |
| Frase «MOVER ROBOT REAL» + checklist | Un solo botón | Evita envíos por un clic accidental |
| Motores: solo apagar si hay que escribir el módulo | Apagar «por las dudas» | Apagarlos dejaba al sistema en un estado del que la PC no podía salir (403) |
| No arrancar encima de RAPID ajeno | Detener lo que haya | El controlador es compartido; se pide al otro grupo que lo detenga |
| Python con biblioteca estándar para HTTP | Frameworks web | Menos dependencias en la PC del laboratorio; fácil de probar con un IRC5 simulado |
| Figuras ≤ 80 mm en real, salto ≤ 12° | Figuras grandes | Mantener el espacio de trabajo chico y lejos de límites y de la mesa |

## 6. Lenguajes, herramientas y protocolos

### 6.1 Lenguajes y bibliotecas

| Capa | Lenguaje / herramienta | Uso |
|---|---|---|
| Cálculo y control | Python 3 con NumPy | Modelos, revisión, backends, cliente RWS, servidor HTTP (biblioteca estándar) |
| Pruebas | pytest, `node --test` | 265 pruebas Python (con `mock_rws.py`) y 10 JS |
| Interfaz | HTML, CSS, JavaScript (módulos ES) | Consola web |
| 3D | Three.js | Gemelo digital, flechas y anillos |
| Gráficas | uPlot | Error en vivo y trayectorias |
| Fórmulas | KaTeX | Pestaña Cálculos |
| Robot | RAPID | `VEVAS_RWS.mod` |
| Visualización opcional | ROS 2 Jazzy, RViz, URDF | `/joint_states` desde RWS |
| Scripts | Bash | Lanzadores y scripts de seguridad |
| Informe | LaTeX (IEEEtran) | Informe del hito |

### 6.2 HTTP y REST

HTTP es el protocolo de pedido/respuesta de la web: el cliente manda un método (GET para leer, POST para actuar, PUT para subir) a una dirección (recurso) y recibe un código (200 bien, 401 sin autenticar, 403 prohibido, 500 error del servidor). REST es el estilo de organizar una API como recursos con direcciones. RWS es REST: cada cosa del controlador es una dirección.

Lo usamos dos veces:
1. **Navegador ↔ consola** (127.0.0.1:9012): GET del estado por SSE (eventos que el servidor empuja) y POST de órdenes.
2. **Consola ↔ IRC5** (192.168.125.1:80): RWS.

### 6.3 Autenticación digest y sesiones

1. La primera petición recibe 401 con un `nonce`.
2. El cliente responde con un hash de usuario, clave, nonce, método y dirección. La clave no viaja.
3. El controlador entrega una cookie de sesión y las siguientes peticiones la usan.
4. El IRC5 admite pocas sesiones. Si se agotan responde 500 «Error registering callback»; el cliente hace `/logout` y reintenta.

El usuario es el de fábrica («Default User»); la clave no se publica.

### 6.4 Recursos RWS que usamos

| Recurso | Para qué |
|---|---|
| `/rw/panel/opmode` | Leer el modo (AUTO / MANR) |
| `/rw/panel/ctrlstate` | Estado de motores (motorson / motorsoff) |
| `/rw/rapid/execution` | start, stop, resetpp |
| `/rw/rapid/tasks/T_ROB1/pcp?action=set-pp-routine` | Poner el PP en `VEVAS_RWS/VevasServidor` |
| `/rw/rapid/modules` | Leer el texto del módulo, cargar y descargar (loadmod/unloadmod) |
| `PUT /fileservice/$HOME` | Subir el archivo `.mod` al controlador |
| `/rw/rapid/symbol/data/RAPID/T_ROB1/VEVAS_RWS/<pers>` | Leer y escribir las variables PERS |
| `/rw/mastership/rapid` | Pedir y soltar mastership |
| `/rw/motionsystem/mechunits/ROB_1/jointtarget` | Leer las juntas (10 Hz) |
| `/rw/elog/0` | Eventos del controlador |

Peculiaridades de RobotWare 6 que tuvimos que respetar: el texto del módulo se trunca a 5000 caracteres al leerlo; los nombres globales deben ser únicos entre módulos de `T_ROB1`; el campo de pcp tiene una errata («modulemame»); y tras un reinicio MOTORS ON por RWS devuelve 403.

### 6.5 Protocolo PERS

Variables (todas en `VEVAS_RWS`):

| Variable | Quién escribe | Significado |
|---|---|---|
| `vvPcTarget` | PC | Objetivo articular (jointtarget) |
| `vvPcSeq` | PC | Número de orden; incrementarlo es lo único que ordena movimiento |
| `vvPcFin` | PC | Distinto de 0 termina el servidor |
| `vvRbSeq` | Robot | Última orden atendida |
| `vvRbCodigo` | Robot | Resultado de la orden |
| `vvRbActivo` | Robot | 1 si `VevasServidor` está aceptando órdenes |
| `vvRbTiempo` | Robot | Duración t que el RAPID impuso al tramo |

Códigos de `vvRbCodigo`:

| Código | Significado |
|---|---|
| 2 | Llegó |
| 1 | Moviendo |
| 0 | Listo (validación pasó / servidor esperando) |
| −1 | Objetivo vacío (valores 9E9 sin cargar) |
| −2 | Fuera de límites con margen de 2° |
| −3 | Paso mayor a 45° en alguna junta |
| −4 | Modo no permitido |

Secuencia de un tramo:

```text
 PC (consola VEVAS)                         IRC5 (VevasServidor)
   |                                            | arranque: TPReadFK "Aceptar objetivos...?"
   |                                            | operador pulsa Iniciar -> vvRbActivo := 1
   |  lee vvRbActivo = 1, vvRbSeq = n          |
   |  [compuertas: GO, C02, revisión, ARMADO]   |
   |-- PUT vvPcTarget := q* ------------------->|
   |-- GET vvPcTarget (relectura) ------------->|
   |<-- q* -------------------------------------|
   |  compara q* y postura medida sin cambios   |
   |-- PUT vvPcSeq := n+1 --------------------->|
   |                                            | ve vvPcSeq <> vvRbSeq (sondeo 0,1 s)
   |                                            | Validar: modo, límites ±2°, paso ≤ 45°
   |                                            | si falla: vvRbCodigo := -1..-4, vvRbSeq := n+1
   |                                            | si pasa: vvRbCodigo := 1
   |                                            | t := max(0.5, max_i 2.5*|dq_i|/vLim_i)
   |                                            | vvRbTiempo := t
   |                                            | MoveAbsJ q*, vPc \T:=t, fine, tool0
   |                                            | vvRbCodigo := 2 ; vvRbSeq := n+1
   |-- GET vvRbSeq, vvRbCodigo, jointtarget --->|
   |<-- n+1, 2, q_medida -----------------------|
   |  acepta si |q_medida - q*| < 0,05°         |
   |  siguiente tramo ... o PARAR (stop RAPID)  |
```

Un candado de archivo en la PC impide que dos herramientas escriban a la vez.

## 7. Cálculos con fórmulas y ejemplos numéricos

Todos los ejemplos se calcularon con los módulos de `vevas/` o salen del informe IEEE.

### 7.1 Ley de tiempo C02

Fórmula (la calcula el RAPID, funciones `Tj` y `Tiempo`):

```text
t = max( 0,5 s ,  max_i  kPico · |Δq_i| / vLim_i )
kPico = 2,5
vLim  = [12,5  12,5  12,5  16  16  21] °/s   (5 % de 250/250/250/320/320/420 °/s)
```

Consecuencia directa: la velocidad **media** de la junta que domina es exactamente vLim/kPico = 0,4·vLim, y las demás van más lentas.

**Ejemplo 1 (el ensayo de verificación, δ = 5°):**

| Junta | vLim [°/s] | t = 2,5·5/vLim [s] | Media δ/t [°/s] |
|---|---|---|---|
| J1 a J3 | 12,5 | 1,00 | 5,0 |
| J4, J5 | 16 | 0,78 | 6,4 |
| J6 | 21 | 0,60 | 8,4 |

**Ejemplo 2 (tramo con varias juntas):** Δq = [10, 5, 0, 0, 20, 30]°.

```text
J1: 2,5·10/12,5 = 2,000 s
J2: 2,5·5/12,5  = 1,000 s
J5: 2,5·20/16   = 3,125 s
J6: 2,5·30/21   = 3,571 s   <- domina
t = 3,571 s
medias: J1 2,8  J2 1,4  J5 5,6  J6 8,4 °/s   (todas ≤ 0,4·vLim)
```

**Ejemplo 3 (movimientos de HOME):** «Volver a cero» desde [0, −20, 30, 0, 40, 0]°: domina J5 con 40°, t = 2,5·40/16 = 6,25 s. «Preparar J5 a +30°» desde cero: t = 2,5·30/16 = 4,69 s.

**Por qué partir en tramos no cambia el tiempo:** por encima del piso de 0,5 s la ley es lineal en |Δq|. Si un tramo de 40° se parte en 4 de 10°, cada uno tarda la cuarta parte y el total nominal es el mismo. Lo que sí cambia es que se agregan 3 paradas en `fine`. Por eso «Volver a cero» pasó de 4 paradas a 1 con el mismo 6,25 s.

### 7.2 Factor pico/medio y por qué kPico = 2,5

Si el controlador mueve cada junta con un perfil q(τ) = q0 + Δq·s(τ/t), con s monótono de 0 a 1, la velocidad pico es c·|Δq|/t, donde c = max s′ es el factor pico/medio del perfil. Combinando con la ley:

```text
q̇_pico ≤ (c / kPico) · vLim
```

| Perfil | c = pico/medio | Pico con kPico = 2,5 |
|---|---|---|
| Velocidad constante | 1,00 | 0,40·vLim |
| LSPB con tb = T/4 | T/(T−tb) = 1,33 | 0,53·vLim |
| Cúbico | 1,50 | 0,60·vLim |
| Quíntico | 1,875 | 0,75·vLim |
| Triangular (peor trapezoidal, tb = T/2) | 2,00 | 0,80·vLim |
| **Medido en el IRC5 (`MoveAbsJ \T`, tramos cortos)** | **≈ 1,06** | **≈ 0,42–0,43·vLim** |

Cálculo del quíntico: s(τ) = 10τ³ − 15τ⁴ + 6τ⁵, s′(τ) = 30τ² − 60τ³ + 30τ⁴; en τ = 0,5 da 7,5 − 7,5 + 1,875 = 1,875.
Cálculo del LSPB: con tiempo de mezcla tb, la velocidad de crucero es Δq/(T − tb), así que c = T/(T − tb).

Conclusión: aun con el perfil más agresivo imaginable entre los suaves, ninguna junta pasa de vLim. La medición dio c ≈ 1,06: el IRC5 ejecuta estos tramos casi a velocidad constante, y el pico real quedó en ≈ 2,2 % de la ficha.

### 7.3 σmin y singularidades

```text
J_L = diag(I3/L, I3) · J(q),   L = 0,3 m
σmin = σ6(J_L)
singular si σmin < 0,02 ; «cerca» si σmin < 0,04
det J = −a2 · L3 · sin δ · r_W · sin q5
```

Valores calculados con `vevas/jacobiano.py`:

| Postura q [°] | σmin | Estado |
|---|---|---|
| [0, 0, 0, 0, 0, 0] (HOME) | 1,2e-16 | singular (muñeca) |
| [0, 0, 0, 0, 2, 0] | 0,016 | singular |
| [0, 0, 0, 0, 5, 0] | 0,039 | cerca |
| [0, 0, 0, 0, 30, 0] | 0,231 | libre |
| [0, −20, 30, 0, 40, 0] (postura inicial) | 0,261 | libre |

Se ve que con q5 = 0 los ejes 4 y 6 quedan alineados y σmin cae a cero de máquina. Por eso «Preparar J5 a +30°» es la salida natural: σ sube monótono de 0 a 0,231.

Criterio de **salida** de singularidad (aviso y no rechazo), con σ_k las muestras del plan, S = {k : σ_k < 0,02} y k* el mínimo dentro de S:
1. S es un tramo inicial contiguo {0, …, m−1} con m < N.
2. σ no decrece desde k* hasta m.
3. En S la singularidad es solo de muñeca.
4. La última muestra tiene σ ≥ 0,04.

La condición desde k* (y no desde 0) admite que el robot arranque con q5 = −0,05° leído por RWS y cruce el mínimo. Criterio simétrico para **llegar** a q = 0: tramo singular contiguo al final con σ no creciente. Cualquier otra entrada a singularidad sigue rechazada.

### 7.4 FK y Pieper con un ejemplo completo

Tabla D-H (a [m], α, d [m], offset):

| i | a | α | d | offset θ |
|---|---|---|---|---|
| 1 | 0 | −π/2 | 0,290 | 0 |
| 2 | 0,270 | 0 | 0 | −π/2 |
| 3 | 0,070 | −π/2 | 0 | 0 |
| 4 | 0 | π/2 | 0,302 | 0 |
| 5 | 0 | −π/2 | 0 | 0 |
| 6 | 0 | 0 | 0,072 | 0 |

FK: T = A1·…·A6·Rz(π). Con q = 0: brida en (374; 0; 630) mm.

Ejemplo con la postura inicial q = [0, −20, 30, 0, 40, 0]°:

```text
FK  -> p_tool0 = (263,50 ; 0 ; 505,06) mm

Pieper (vuelta atrás):
p_W = p_6 − d6·z6 = (217,22 ; 0 ; 560,21) mm
L3  = sqrt(a3² + d4²) = sqrt(0,070² + 0,302²) = 0,3100 m
γ   = atan2(a3, d4) = 13,05°
r   = sqrt(x_W² + y_W²) = 0,2172 m ;  s = z_W − d1 = 0,2702 m
q1  = atan2(y_W, x_W) = 0°
cos δ = (r² + s² − a2² − L3²)/(2·a2·L3) = −0,2915  ->  δ = 106,95°
q3  = δ + γ − 90° = 30,00°
q2  = atan2(r, s) − atan2(L3·sin δ, a2 + L3·cos δ) = −20,00°
R_3^6 = (R_0^3)ᵀ·R_0^6  ->  q5 = atan2(±sqrt(r13² + r23²), r33) = 40°, q4 = 0°, q6 = 0°
```

Las 8 ramas para esa pose (todas con e_p < 2e-16 m):

| Rama | Hombro / codo / muñeca | q [°] | En límites |
|---|---|---|---|
| 0 | adelante / arriba / sin giro | [0, −20, 30, 0, 40, 0] | sí (elegida, margen 40°) |
| 1 | adelante / arriba / girada | [0, −20, 30, 180, −40, −180] | no (J4) |
| 2 | adelante / abajo / sin giro | [0, 97,59, 176,1, 0, 136,31, 0] | no (J3, J5) |
| 3 | adelante / abajo / girada | [0, 97,59, 176,1, 180, −136,31, −180] | no |
| 4 | atrás / arriba / sin giro | [180, −97,59, 30, −180, 162,41, 0] | no |
| 5 | atrás / arriba / girada | [180, −97,59, 30, 0, −162,41, −180] | no |
| 6 | atrás / abajo / sin giro | [180, 20, 176,1, 180, 66,1, 0] | no |
| 7 | atrás / abajo / girada | [180, 20, 176,1, 0, −66,1, −180] | no |

Las 8 existen matemáticamente y todas pasan la verificación FK; solo una cae dentro de los límites del IRB 120. Eso muestra por qué la elección de rama mira límites, cercanía a la postura actual y σmin.

Validación fuera de línea (bitácora S0-001): error FK 6e-16 m en 5000 posturas, Jacobiano contra diferencias finitas 9e-11 m/rad, amplificación ‖q̇‖ ∼ q5^(−1,00). Controles negativos (TCP desplazado y orden de factores alterado) fallan a propósito con 0,120 m y 0,469 rad.

### 7.5 LSPB y quíntico

LSPB para una junta con desplazamiento Δq, duración T y tiempo de mezcla tb:

```text
v_c = Δq / (T − tb)       velocidad de crucero
a   = v_c / tb            aceleración de mezcla
0 ≤ t < tb:        q = q0 + a·t²/2
tb ≤ t ≤ T − tb:   q = q0 + v_c·(t − tb/2)
T − tb < t ≤ T:    q = qf − a·(T − t)²/2
```

Ejemplo: Δq = 40° (J5 en «Volver a cero»), T = 6,25 s, tb = 1,0 s:
v_c = 40/5,25 = 7,62 °/s; a = 7,62 °/s²; media = 6,40 °/s; pico/medio = 1,19.

Quíntico con reposo en los extremos:

```text
q(t) = q0 + Δq·(10τ³ − 15τ⁴ + 6τ⁵),  τ = t/T
q̇_max = 1,875·Δq/T   (en τ = 0,5)
q̈_max = 5,77·Δq/T²   (en τ ≈ 0,21 y 0,79)
```

Mismo ejemplo: q̇_max = 1,875·40/6,25 = 12,0 °/s. Ese sería el pico **si** el robot siguiera un quíntico en 6,25 s; sigue bajo vLim = 16 °/s (0,75·vLim). Las seis juntas se sincronizan para que todas terminen juntas, con la duración que impone la más lenta.

## 8. Resultados y evidencias

### 8.1 Verificación C02 en el IRB 120 real (AUTO, 24/11/2026)

Herramienta: `herramientas/verificar_c02.py`, cada junta ±5° y vuelta, muestreo continuo de `jointtarget`, velocidad máxima en ventana de 0,10 s. Se exigió además que `vvRbTiempo` coincidiera con el t esperado (tolerancia 2 %).

| Juntas | vLim [°/s] | t [s] | Media nominal δ/t [°/s] | vmax medida [°/s] | vmax/vLim | Resultado |
|---|---|---|---|---|---|---|
| J1–J3 | 12,5 | 1,00 | 5,0 | 5,2–5,4 | 0,42–0,43 | OK |
| J4–J5 | 16 | 0,78 | 6,4 | 6,7–6,9 | 0,42–0,43 | OK |
| J6 | 21 | 0,60 | 8,4 | 8,8–8,9 | 0,42 | OK |

Pico medido ≈ 1,06 veces la media nominal. Todas las juntas pasaron con margen: el pico real es ≈ 2,2 % de la velocidad de ficha.

La evidencia `evidencia_c02.json` guarda la huella SHA-256 del texto del módulo, el modo, vLim, kPico y las seis juntas. El backend habilita el envío solo si:
1. la evidencia dice que pasó,
2. su huella coincide con la del `VEVAS_RWS.mod` local,
3. y con la del módulo cargado en el IRC5,
4. y el modo actual es el mismo en que se midió.

### 8.2 FK propia contra el controlador

| Prueba | Resultado |
|---|---|
| Juntas leídas por RWS contra FlexPendant | Coinciden las seis |
| FK propia vs pose del controlador (`tool0`, `wobj0`, base), primera lectura | 0,0001 mm y 0,0000° |
| Mismo diagnóstico tras actualizar contadores de revoluciones, juntas ≈ 0 | COINCIDE, 0,0000 mm |
| Validación fuera de línea, 5000 posturas | error FK 6e-16 m |
| Jacobiano contra diferencias finitas | 9e-11 m/rad |
| Pieper, verificación FK de cada rama | ~1e-15 m |

### 8.3 Pruebas automáticas

| Conjunto | Cantidad | Estado |
|---|---|---|
| Python (pytest), incluye `mock_rws.py`: un IRC5 simulado por HTTP | 265 | todas pasan |
| JavaScript (`test_calculos.mjs`, `test_orientacion.mjs`) | 10 | todas pasan |

El IRC5 simulado incluye un perfil que **no** respeta el tiempo, para comprobar que la evidencia C02 falla cuando debe. Hay pruebas de cinemática, revisión, C02, cliente RWS, GO, figuras, lanzadores, mesa y trayectorias suaves.

### 8.4 Otros resultados

1. HOME es singular: σmin(0) = 1,2e-16. «Volver a cero» con RWS + C02 da **aviso**; el mismo plan sin la excepción da **rechazado**. «Preparar J5 a +30°» cumple el criterio de salida (σ de 0 a 0,231, monótono).
2. Figuras (círculo, corazón, cuadrado, triángulo, saludo) resueltas con Pieper punto a punto, orientación constante, salto máximo 12°, ≤ 80 mm en real.
3. Tramos de 40° en lugar de 10°: se eliminaron las pausas visibles en recorridos largos.
4. Archivos de evidencia: `evidencia_c02.json`, registros de sesión (`08_datos_locales/sesiones`), `docs/bitacora.md`, informe IEEE con figuras generadas por `generar_figuras.py` usando los mismos módulos.

## 9. Incidentes y cómo se resolvieron

| # | Qué pasó | Causa | Qué hicimos | Qué aprendimos |
|---|---|---|---|---|
| 1 | El script no podía arrancar | Otro grupo tenía RAPID corriendo (`Imt342TrajectoryRapid/ServeTrajectoryOrders`) | El script se niega a arrancar encima; se pidió al otro grupo que lo detuviera | El controlador es compartido: nunca pisar programas ajenos |
| 2 | RWS dejó de responder (HTTP 000) con ping OK | Servicio RWS colgado | Reinicio del controlador. El FlexPendant rechazó reiniciar con programa en ejecución (evento 20054) hasta pulsar STOP; después paro de emergencia, reset del paro y MOTORS ON | Orden correcto de recuperación; la red puede estar bien y RWS no |
| 3 | Tras el reinicio, contadores de revoluciones perdidos (evento 20082, «Battery backup lost») | Batería SMB débil | Se llevaron los ejes a las marcas de sincronización y se actualizaron contadores; el diagnóstico FK volvió a 0,0000 mm | Ningún modelo detecta esto, porque compara contra las mismas lecturas; la única verificación válida es física. Hay que cambiar la batería y evitar reinicios |
| 4 | Instalar el módulo apagaba motores que el operador había encendido | Bug propio: se apagaban «por las dudas» | Ahora solo se apagan si hay que escribir el módulo; si RWS niega MOTORS ON (403) se espera 60 s al operador | Una medida de seguridad mal pensada puede dejar al sistema en un estado del que no sale solo |
| 5 | Pausas en el movimiento | Tramos de 10°, cada uno terminando en `fine` | Tramos colineales de hasta 40° | El tiempo nominal no cambia; las paradas sí |

El incidente 3 es el más importante para la defensa: después de perder contadores, el robot puede **creer** estar en una postura y estar en otra. Por eso el diagnóstico FK se repite después de cualquier reinicio, y por eso nunca se actualizan contadores solo porque las juntas «parezcan» estar cerca de cero: primero se verifican las marcas físicas.

## 10. Limitaciones y próximas mejoras (priorizadas)

| Prioridad | Tema | Situación actual | Próximo paso |
|---|---|---|---|
| 1 | Batería SMB | Débil; causó la pérdida de contadores | Cambiarla; verificar la calibración fina desde el FlexPendant; evitar reinicios mientras tanto |
| 2 | Paradas entre puntos en figuras | **Resuelto en código (24/11, tarde)**: streaming con zonas `z1` (variable `vvPcZona`, piso tZona = 0,2 s, Δq contra el objetivo previo, `fine` solo en el último punto y en las esquinas del plan). Pasa las pruebas con el IRC5 simulado | Repetir `verificar_c02.sh` (incluye una prueba de streaming con J6) y probar las figuras en el robot real vigilando avisos 50024 |
| 3 | Medición de velocidad | Diferencias de posiciones leídas por RWS con ventana de 0,1 s; puede subestimar un pico breve | El margen kPico lo cubre; mejorar con traza interna del controlador si se dispone |
| 4 | Modelo de entorno | La revisión ve solo lo descrito (cápsulas, mesa, cajas) y hasta 240 muestras | Medir herramienta, soportes y entorno; más muestras en planes largos |
| 5 | Visión | No integrada | Cámara cenital con calibración DLT propia (objetivo: error < 1,5 mm) |
| 6 | Pinza | No integrada | Pinza neumática por E/S del IRC5 |
| 7 | Control a alta frecuencia | RWS por tramos | EGM para control articular a 250 Hz (la opción 689-1 está en el controlador) |
| 8 | Tope oficial | Tope por ley de tiempo en RAPID | SpeedLimAxis/LimitSpeed (configuración EIO) si el docente lo exige |

Limitaciones que hay que decir con honestidad en la defensa:
1. «Aprobado» en la revisión significa «cumple dentro del modelo», no «es seguro».
2. PARAR por software deja terminar el tramo en curso; el freno real es el paro físico.
3. La evidencia C02 vale solo para el módulo, los topes y el modo medidos.
4. La FK en vivo se comparó en pocas posturas.

## 11. Banco de 20 preguntas del docente

Organizado según la rúbrica ECI-3/4 (50 % de la evaluación continua): ingeniería y compatibilidad 20 %, modelos propios y validación FK/IK/J 25 %, integración ROS–ABB 25 %, seguridad y reproducibilidad 20 %, defensa 10 %.

### Ingeniería y compatibilidad (20 %)

**1. ¿Por qué usaron RWS y no EGM o ROS 2 para mover el robot real?**
RWS solo necesita la opción PC Interface y un módulo RAPID propio, no interfiere con los módulos de otros grupos y deja el tope de velocidad dentro del controlador. ROS 2 lo dejamos para visualización en RViz y EGM como extensión a 250 Hz.

**2. ¿Qué restricciones de RobotWare 6 tuvieron que respetar?**
El texto del módulo se trunca a 5000 caracteres al leerlo (por eso el nuestro tiene 4123), los nombres globales deben ser únicos en `T_ROB1` (todo `LOCAL` o con prefijo `vv`), hay pocas sesiones HTTP (500 «Error registering callback» → `/logout` y reintento) y tras un reinicio MOTORS ON por RWS da 403.

**3. ¿Cómo conviven con otros grupos en el mismo controlador?**
No tocamos `LabJointRws`, `EGM_ROS` ni `Imt342TrajectoryRapid`; si hay RAPID ajeno corriendo, el script no arranca y pedimos que lo detengan. Pedimos y soltamos mastership en cada escritura.

**4. ¿Qué pasa si el programa se interrumpe a mitad de un tramo?**
El tramo en curso termina en el controlador (MoveAbsJ con `fine`), no se mandan más tramos y el robot queda quieto. El paro de emergencia físico es la parada real.

### Modelos propios y validación FK/IK/J (25 %)

**5. ¿Cómo saben que su FK es correcta?**
La comparamos con la pose del controlador en el robot real: 0,0001 mm y luego 0,0000 mm tras actualizar contadores. Fuera de línea, 6e-16 m en 5000 posturas, y los controles negativos fallan a propósito.

**6. ¿Por qué Pieper da 8 soluciones y cuál eligen?**
Por tres decisiones binarias: hombro, codo y muñeca. Elegimos la que verifica FK, respeta límites con margen, no es singular y está más cerca de la postura actual.

**7. ¿Qué es la verificación FK de una rama?**
Meter la solución de Pieper de vuelta en la FK y comparar: se acepta si e_p < 1e-6 m y e_R < 1e-5 rad. En la práctica da ~1e-15 m.

**8. ¿Qué mide σmin y por qué escalan el Jacobiano con L = 0,3 m?**
Mide la distancia a una singularidad. Escalamos la parte lineal para que metros y radianes sean comparables; sin escala, el umbral dependería de las unidades.

**9. ¿Por qué HOME es singular y cómo salen de ahí?**
Con q5 = 0 los ejes 4 y 6 se alinean y σmin = 1,2e-16. Salimos con «Preparar J5 a +30°», un movimiento articular en el que σ sube monótono hasta 0,231; la revisión lo acepta como aviso solo si es de muñeca, contiguo al inicio y con RWS + C02.

### Integración con el ABB (25 %)

**10. Expliquen el protocolo entre la PC y el robot.**
La PC escribe `vvPcTarget`, lo relee, comprueba que la postura no cambió e incrementa `vvPcSeq`. RAPID valida, calcula t, mueve con `MoveAbsJ \T` y responde `vvRbSeq` y `vvRbCodigo`. Aceptamos el tramo solo con código 2 y postura medida a menos de 0,05°.

**11. ¿Qué significan los códigos de respuesta?**
2 llegó, 1 moviendo, 0 listo, −1 objetivo vacío, −2 fuera de límites, −3 paso mayor a 45°, −4 modo no permitido.

**12. ¿Por qué `MoveAbsJ` y no `MoveL`?**
Porque interpola en juntas y nunca invierte el Jacobiano: cerca de singularidades no hay velocidades explosivas. La IK la hacemos nosotros con Pieper.

**13. ¿Por qué el robot hacía pausas y cómo lo resolvieron?**
Cada tramo termina en `fine`. Pasamos de tramos de 10° a tramos colineales de 40°; el tiempo nominal no cambia y las paradas bajan (Volver a cero: de 4 a 1). Las figuras siguen frenando entre puntos; el streaming con zonas `z1` está en desarrollo.

**14. ¿Qué rol cumple ROS 2 en este hito?**
Opcional: publicar `/joint_states` leídos por RWS y ver el robot en RViz. El mismo núcleo de modelos sirve a los tres backends (SIM, ROS 2, RWS).

### Seguridad y reproducibilidad (20 %)

**15. ¿Cómo garantizan que ninguna junta supera su tope (C02)?**
El RAPID impone t = max(0,5; máx 2,5·|Δq_i|/vLim_i) con vLim = 5 % de la ficha, y lo medimos en el robot: picos de 5,2 a 8,9 °/s contra topes de 12,5 a 21 °/s, es decir, 42–43 % del tope.

**16. ¿Por qué kPico = 2,5?**
Un perfil suave tiene pico de hasta 1,875 veces la media (quíntico), y hasta 2 el triangular; con 2,5 el pico teórico queda ≤ 0,8·vLim. Medimos 1,06.

**17. ¿Qué pasa si alguien edita el módulo RAPID?**
Cambia su huella SHA-256 y la evidencia C02 deja de valer: el backend bloquea el envío hasta repetir `verificar_c02`. Lo mismo si se cambia de modo.

**18. Nombren las compuertas antes de que el robot se mueva.**
GO vigente, evidencia C02, revisión previa no rechazada ni sin verificar, ARMAR con checklist y frase, confirmación de envío, validación en RAPID, watchdog de estado y «postura cambió». Siempre con una persona en el paro de emergencia.

### Defensa (10 %)

**19. ¿Cuál fue el incidente más importante y qué aprendieron?**
La pérdida de contadores de revoluciones por la batería SMB: el robot puede creer estar en otra postura y ningún modelo lo detecta. Se corrigió con las marcas físicas de sincronización y se verificó con FK = 0,0000 mm.

**20. ¿Qué harían a continuación?**
Cambiar la batería SMB, terminar y verificar el streaming con zonas `z1`, y después cámara cenital con DLT, pinza neumática por E/S y EGM a 250 Hz.
