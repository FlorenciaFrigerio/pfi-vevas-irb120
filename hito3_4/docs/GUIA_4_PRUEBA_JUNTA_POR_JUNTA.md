# Guía 4. Probar cada junta antes de mover todo el robot

Grupo VEVAS · Protocolo de prueba junta por junta (directa), luego todas juntas, luego inversa

La idea es la que pidió el grupo: **primero mover las juntas una por una, despacio, para ver que todo esté bien; después las seis a la vez calculando el error en tiempo real; y recién después la cinemática inversa.** Si algo está mal (un signo, un offset, una junta cambiada de lugar), se descubre con un movimiento de 5° y no con un movimiento grande.

## Por qué en este orden

| Prueba | Qué descubre | Qué tan grave sería descubrirlo tarde |
|---|---|---|
| Solo lectura (Guía 2, paso 5) | Que nuestra FK y la del controlador coinciden: tabla D-H, orden, signos y offsets | Sin esto, la IK mandaría el robot a otro lugar |
| Una junta, +5° | Que cada junta gira hacia el lado que creemos y la cantidad que creemos | Una junta con el signo al revés convierte "subir" en "bajar" |
| Seis a la vez, +5° | Que la trayectoria sincronizada se sigue bien y el error se mantiene bajo | Un retardo grande aparece recién con varias juntas |
| IK entre poses | Todo el pipeline: pose → Pieper → mejor rama → LSPB → robot | Es la prueba final; ya no debería haber sorpresas |

## Antes de empezar

- [ ] La consola corre en `--modo ros --real` con el GO del día (Guía 6).
- [ ] La lectura de solo lectura dio COINCIDE en 3 posturas (Guía 2, paso 5).
- [ ] La misma secuencia se corrió completa en **simulación** ese mismo día, sin abortos.
- [ ] Velocidad en la consola: 5 %. En la secuencia, cada junta va a 3 °/s como máximo.
- [ ] Una persona con el FlexPendant en la mano y el dedo cerca del paro de emergencia.
- [ ] Nadie dentro del alcance del robot (580 mm desde el eje 1, más la herramienta).
- [ ] Grabando: celular apuntando al robot y grabación de pantalla de la consola (Guía 6).

## Fase 1. Ir a la postura inicial segura

Postura: q = [0, −20, 30, 0, 40, 0]°.

Por qué esta: está lejos de todos los límites y **no es singular** (q5 = 40°). La postura de calibración q = 0 sí es singular (q5 = 0) y desde ahí la inversa se comporta mal.

En la consola: pestaña Objetivo → "Postura inicial segura" → Calcular → revisar el plan → ENVIAR. O usar la secuencia demo, que empieza con este paso.

Qué mirar: el robot real y el modelo 3D llegan a la misma postura; al final, el error de todas las juntas es menor a 0.1°.

## Fase 2. Una junta a la vez

Para cada junta j = 1 ... 6: mover **solo** la junta j, +5°, a 3 °/s, y volver.

Con la secuencia demo (pestaña Secuencia demo, con "pausar entre pasos" marcado) se hace sola: antes de cada paso se ve el cálculo y se pulsa CONTINUAR (tecla C).

A mano: pestaña Objetivo → "Una sola junta" → junta j, valor 5, relativo → Calcular → ENVIAR. Después lo mismo con −5.

Qué mirar en cada junta y qué significa:

| Lo que se ve | Está bien si... | Si no, probablemente... |
|---|---|---|
| Sentido de giro del robot real | Gira hacia donde gira el modelo 3D | Signo invertido de esa junta en `config.json` (`signos`) |
| Cuánto gira | 5° en el FlexPendant (Movimiento → ejes) | Unidad mal (grados vs radianes) o reductor mal configurado |
| Qué junta se mueve | La j y ninguna otra | Orden de juntas cambiado (`nombres_juntas`) |
| Error de la junta j en la gráfica | Menor a 0.5° durante el movimiento y menor a 0.05° al final | Retardo alto o velocidad demasiado alta |
| Error de las otras juntas | ~0 | Acople mecánico o lectura con ruido |

Sentidos esperados (convención ABB, que es la de nuestra tabla; regla de la mano derecha sobre el eje de cada junta). Parados **delante** del robot, es decir del lado +X, mirando hacia la base:

| Junta | +5° hace que... |
|---|---|
| J1 | el brazo gire hacia +Y: antihorario visto desde arriba; desde donde están parados, hacia su derecha |
| J2 | el brazo superior se incline hacia adelante (hacia ustedes) |
| J3 | el antebrazo baje |
| J4 | el antebrazo rote sobre su eje: horario visto desde atrás del robot mirando hacia la herramienta |
| J5 | la muñeca se doble hacia abajo |
| J6 | la brida gire sobre su eje: horario visto desde atrás del robot (con la muñeca recta) |

(J4 y J6 son difíciles de ver: pegar un pedazo de cinta en la brida ayuda.)

Si cualquier junta falla: PARAR, anotar en la bitácora, corregir `config.json`, volver a la Guía 2 paso 5 y a la simulación. **No seguir con la fase 3.**

Tabla para llenar en la bitácora (una fila por junta):

| Junta | Sentido OK | Grados medidos en el FlexPendant | Error máx. de seguimiento [°] | Error final [°] | Observación |
|---|---|---|---|---|---|
| J1 | | | | | |
| J2 | | | | | |
| J3 | | | | | |
| J4 | | | | | |
| J5 | | | | | |
| J6 | | | | | |

Los números de las columnas 4 y 5 los da la consola (pestaña Bitácora, "Último movimiento") y `analizar_sesion.py`.

## Fase 3. Las seis a la vez

Mismo movimiento chico (+5° en todas y vuelta) con LSPB sincronizado: todas empiezan y terminan juntas.

Qué mirar: en "Error en tiempo real", e_p (mm) y e_R (grados) de la punta. Es la primera vez que el error de varias juntas se suma en la punta.

Criterio propuesto por el grupo (anotarlo en la bitácora como propuesto, no como aprobado por el docente): error de seguimiento máximo < 1° por junta, error final < 0.05°, e_p de seguimiento < 5 mm a 3 °/s.

## Fase 4. Cinemática inversa

Las tres poses de la secuencia (herramienta hacia abajo):

| Pose | x [mm] | y [mm] | z [mm] | Rz, Ry, Rx [°] |
|---|---|---|---|---|
| A (izquierda) | 350 | 150 | 400 | 0, 180, 0 |
| B (derecha) | 350 | −150 | 400 | 0, 180, 0 |
| C (centro, más baja) | 400 | 0 | 320 | 0, 180, 0 |

Antes de cada una, explicar en voz alta para el video lo que muestra la consola: centro de muñeca, las 8 ramas, cuál gana y por qué, t_f de la LSPB. Después de cada una, leer "e_p vs pose pedida": es el error entre donde quedó la punta (según la FK con los ángulos medidos) y la pose que se pidió.

Prueba extra que suma puntos: con la herramienta del robot apuntando a una marca en una hoja sobre la mesa, pedir una pose sobre esa marca y medir con regla o vernier cuánto se separó. Eso es un error físico medido (además del calculado) y es lo que pedía el plan original del Hito 3.

## Si algo sale mal durante el movimiento

| Señal | La consola hace | Ustedes hacen |
|---|---|---|
| Error de una junta > 3° | Aborta sola, el robot se queda quieto, se desarma | Anotar, revisar velocidad y retardo con `analizar_sesion.py` |
| No llegan datos del robot por más de 0.5 s | Aborta sola | Revisar cable y driver |
| El navegador se cuelga por más de 1 s | Aborta sola | Recargar la página |
| Se suelta la flecha del teclado | El jog se detiene | Nada, es lo normal |
| Algo raro que la consola no detecta (un ruido, un movimiento inesperado) | — | **Paro de emergencia del FlexPendant.** Siempre manda la persona del FlexPendant |

Después de un paro de emergencia: no volver a arrancar hasta entender qué pasó. Anotarlo en la bitácora como "falla" con su análisis: eso también suma en el dominio de seguridad de la rúbrica.
