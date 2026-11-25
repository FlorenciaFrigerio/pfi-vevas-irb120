# Guía 6. Seguridad, el GO del día y cómo grabar el video

Grupo VEVAS · Antes, durante y después de mover el robot real

## 1. Lo que dijo el docente y cómo lo cumplimos

El docente pidió que el grupo "simule el GO": que lo armemos nosotros (con ayuda de la IA), que la trayectoria la genere la IA y que le mandemos un video del robot físico moviéndose mientras mandamos los datos.

Cómo queda repartido, para que esté claro en la bitácora y en la defensa:

| Parte | Quién la hace |
|---|---|
| La plantilla del GO y el checklist | Generados con IA a partir del checklist oficial del docente (W12-S01) |
| La trayectoria de demostración | Generada con IA (`07_vevas/secuencias.py`), revisada en simulación por el grupo |
| Revisar que cada ítem del checklist se cumple, llenar el GO y firmarlo | **El grupo**, el mismo día, en el laboratorio |
| Autorizar que el robot se mueva | **El docente** (dio la instrucción). La IA no autoriza nada |
| Parar el robot si algo no está bien | **La persona del FlexPendant**, siempre, sin preguntar |

Recomendación: pedirle al docente que confirme la instrucción por escrito (un mensaje de WhatsApp alcanza) y guardar esa captura en `docs/01_datos/`. Así el campo "autoriza" del GO tiene respaldo y nadie del grupo queda expuesto si algo sale mal.

## 2. Roles el día de la prueba

| Rol | Qué hace | Quién |
|---|---|---|
| Operador del FlexPendant | Sostiene el FlexPendant, dedo cerca del paro de emergencia, vigila el robot y nada más | (completar) |
| Responsable de la consola | Calcula, explica en voz alta y envía. Tiene la barra espaciadora (PARAR) a mano | (completar) |
| Cámara | Graba el robot y el FlexPendant con el celular | (completar) |
| Bitácora | Anota hora, plan, resultado y cualquier cosa rara | (completar) |
| Supervisor | Docente o encargado del laboratorio, si está presente | (completar) |

Cada uno tiene que poder explicar su parte **y la de otro** (la rúbrica pregunta por el rol de respaldo).

## 3. El GO del día, paso a paso

1. Copiar la plantilla:
   ```bash
   cd ~/abb_ws/src/pfi-vevas-irb120/hito3_4
   cp 04_plantillas/GO_VEVAS_plantilla.json GO_VEVAS.json
   ```
2. Ir completando el checklist **en el laboratorio**, en orden. Cada ítem se pone en `true` solo si se verificó de verdad:

| # | Ítem del checklist oficial | Cómo se verifica (nuestro caso) |
|---|---|---|
| 1 | Identidad del controlador y manual correcto | IRB 120-3/0.6 serie 120-101516, IRC5 Compact, RobotWare 6.12.01.00 (Guía 2, paso 4) |
| 2 | Autorización de acceso al robot | Instrucción del docente + encargado del laboratorio enterado |
| 3 | Operador y supervisor presentes | Personas de la tabla de roles en la sala |
| 4 | Modo operativo permitido | Manual reducido con dispositivo de habilitación, o AUTO si el encargado lo decide |
| 5 | Zona despejada | Nadie a menos de 1 m del alcance; nada sobre la mesa en el camino del robot |
| 6 | Paro de emergencia y parada normal probados | Pulsar el paro del FlexPendant con motores ON: los motores se apagan. Rearmar. Probar también PARAR de la consola en simulación |
| 7 | Trayectoria revisada | La secuencia corrió completa en simulación ese día, sin abortos (guardar la sesión) |
| 8 | Límites, velocidad, tool y base revisados | Velocidad 5 % (máx. 10 %), tool0 sin herramienta, base del robot, z mínima 80 mm |
| 9 | Sin pieza ni herramienta peligrosa | Brida libre en la primera prueba |
| 10 | Registro y aborto funcionando | La consola registra en `sesiones/` y aborta sola (probado en simulación) |
| 11 | Condiciones de aborto y recuperación acordadas | Ver sección 5 |

3. Completar `fecha` (formato AAAA-MM-DD, la de hoy), `autoriza`, `operador_pendant`, `responsable_consola`, `supervisor` y `alcance`.
4. Cambiar `decision` a `"GO"` solo si los 11 ítems están en `true`. Si falta uno, es **NO-GO** y no se mueve el robot ese día.
5. Imprimir o fotografiar el archivo lleno y guardarlo en `docs/01_datos/`.

La consola revisa el archivo: si la fecha no es la de hoy, si falta un campo o si un ítem no está en `true`, no deja armar el robot real.

## 4. Límites que pone la consola (y por qué)

| Límite | Valor | Por qué |
|---|---|---|
| Velocidad máxima con robot real | 10 % de la hoja de datos (25 °/s en J1–J3) | Margen para reaccionar |
| Secuencia junta por junta | 3 °/s | Ver bien el sentido de cada junta |
| Jog con teclado (real) | 5 °/s por junta, 25 mm/s en cartesiano | Por debajo del límite de 250 mm/s del modo manual |
| Margen a los límites articulares | 5° | No llegar nunca a un tope mecánico |
| Altura mínima de codo, muñeca, brida y TCP | 80 mm sobre la base | No golpear la mesa |
| Error de seguimiento | 3° → aborto | Si el robot no sigue la orden, algo anda mal |
| Sin datos del robot | 0.5 s → aborto | Cable, driver o EGM caído |
| Interfaz colgada | 1 s → aborto | El navegador dejó de mandar el pulso |
| Jog | solo mientras la tecla está apretada | Hombre muerto |
| Singularidad | σ_min < 0.02 → plan rechazado; jog se detiene si una junta pasaría de 45 °/s | Cerca de una singularidad las juntas giran muy rápido |

Además, del lado del controlador: el RAPID usa `EGM_STOP_HOLD` (si ROS deja de mandar, el robot se detiene) y `MaxSpeedDeviation` de 20 °/s. Y por encima de todo, el paro de emergencia físico.

Una advertencia de la documentación de ABB que hay que conocer: **EGM reacciona rápido a cualquier referencia que reciba, también a las equivocadas, y pasa por encima de varias protecciones del robot.** Por eso todas las revisiones se hacen antes de enviar, en la consola.

## 5. Si algo sale mal

1. **Paro de emergencia** del FlexPendant (lo hace el operador, sin esperar a nadie). O PARAR en la consola.
2. No tocar nada hasta que el robot esté quieto y los motores apagados.
3. Anotar en la bitácora: hora, plan, qué se vio, mensaje del FlexPendant (foto).
4. Guardar la sesión de la consola y correr `analizar_sesion.py`.
5. Volver a mover solo cuando se entienda la causa. Primero en simulación.

Un aborto bien registrado y analizado suma en el dominio 4 de la rúbrica ("análisis técnico de fallos y decisiones operativas seguras"). No hay que esconderlo.

## 6. El video

### Qué tiene que mostrar

1. El robot físico moviéndose.
2. Al mismo tiempo, la consola mandando los datos: los cálculos, las 8 ramas, la rama elegida, la trayectoria y el error en tiempo real.
3. Que el flujo es el que pidió el docente: primero se calcula y se verifica, recién después se manda.

### Cómo grabarlo

Dos grabaciones a la vez:

- **Pantalla de la consola:** en Ubuntu, Ctrl + Alt + Shift + R empieza y termina la grabación de pantalla (o PrtSc → botón de grabar). El archivo queda en `~/Vídeos`. Poner la consola en pantalla completa (F11).
- **Celular** filmando el robot, con el FlexPendant y la pantalla de la PC en el cuadro si se puede. Celular horizontal, apoyado (no en la mano).

Para sincronizar las dos después: al empezar, con las dos grabando, el responsable de la consola dice "plan uno" y pulsa Enter; ese instante se ve en las dos.

Edición (opcional): la pantalla grande y el celular en una ventanita en una esquina (con cualquier editor; en Ubuntu, Kdenlive o Shotcut).

### Guion (4 a 6 minutos)

| Tiempo | En pantalla | Se dice |
|---|---|---|
| 0:00 | Banda roja ROBOT REAL, robot quieto | Grupo VEVAS, IRB 120. Conectados por ROS 2 Jazzy y EGM. GO del día firmado por el grupo según la instrucción del docente. |
| 0:20 | Pestaña FK en vivo; el operador mueve el robot con el joystick | La consola lee las juntas reales y recalcula la FK en vivo; el modelo 3D sigue al robot. |
| 0:50 | Secuencia demo, paso 1 | Postura inicial segura, no singular (q5 = 40°). |
| 1:10 | Pasos 2 a 13 (junta por junta, se puede acelerar en la edición) | Cada junta sola, +5° a 3 °/s: se comprueba sentido y error. |
| 2:00 | Las 6 a la vez; gráfica de error | LSPB sincronizado; e_p y e_R en vivo. |
| 2:40 | Pose A: Cálculo paso a paso | Centro de muñeca, q1, triángulo, muñeca, verificación FK 1e-16. |
| 3:20 | Las 8 ramas, pasando el mouse | 8 soluciones, solo una dentro de límites; por qué gana. |
| 3:50 | ENVIAR; robot moviéndose; error en vivo | Recién ahora se manda. Error de seguimiento menor a 1°. |
| 4:30 | Poses B y C | Lo mismo, más rápido. |
| 5:10 | Pestaña Bitácora; informe | Errores máximos y finales, sesión guardada, análisis automático. |
| 5:40 | Cierre | Qué falta y qué sigue (Guía 8). |

Al final, guardar el video, la sesión (`sesiones/...`), el informe y el GO juntos, y anotar los nombres de los archivos en la bitácora.
