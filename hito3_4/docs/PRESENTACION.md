# Presentación del Hito 3/4 (ECI-3/4): guion y banco de preguntas

Grupo VEVAS · 4 integrantes · unos 15 minutos más preguntas

La presentación se hace **con la consola abierta**, no con diapositivas llenas de texto. Las diapositivas son pocas (ver el final) y sirven de mapa.

## Reparto

| Parte | Tiempo | Integrante | Domina además (respaldo) |
|---|---|---|---|
| 1. Sistema, compatibilidad y arquitectura (dominio 1) | 3 min | (completar) | Parte 4 |
| 2. Modelos propios y validación FK/IK/J (dominio 2) | 4 min | (completar) | Parte 3 |
| 3. Integración ROS–ABB y demostración en vivo (dominio 3) | 5 min | (completar) | Parte 2 |
| 4. Seguridad, bitácora, errores y cierre (dominio 4) | 3 min | (completar) | Parte 1 |

## Parte 1. Sistema (3 min)

1. Qué es el robot: IRB 120-3/0.6, 6 juntas, 3 kg, 580 mm; IRC5 Compact con RobotWare 6.12.01.00.
2. Por qué este camino: el controlador tiene las opciones 689-1 (EGM) y 616-1 (PC Interface), y el driver `abb_ros2` necesita justamente eso y RobotWare 6 (RWS 1.0). Se comprobó en el FlexPendant (mostrar foto).
3. Arquitectura (mostrar el diagrama de `docs/arquitectura.md`): ida (consola → trayectoria → `joint_trajectory_controller` → EGM → robot) y vuelta (robot → EGM → `/joint_states` → consola). Dos canales: RWS para leer, EGM para mover.
4. Alternativa descartada y por qué: `abb_driver` (RAPID con sockets) es ROS 1 y más lento; nuestro controlador permite la ruta moderna.

## Parte 2. Modelos propios (4 min)

En la consola:

1. Tecla F: marcos D-H sobre el robot. Explicar la tabla y el offset de q2.
2. Pestaña FK en vivo: las 6 matrices con los ángulos reales, actualizándose.
3. Pedir la Pose A. Pestaña Cálculo paso a paso: centro de muñeca, q1, triángulo con la ley de cosenos, R_3^6 y la muñeca. Verificación FK: 1e-16.
4. Pestaña Las 8 ramas: pasar el mouse por las 8. Por qué solo una es válida. Cómo se elige.
5. Validación (mostrar `docs/datos/validacion_offline/resumen.md`): contra el URDF de ROS-Industrial en 5000 posturas, 6e-16; contra el controlador real con `verificar_convencion_rws.py`, e_p y e_R medidos. Los dos casos "mal a propósito" (brida contra TCP, juntas cambiadas) muestran que la prueba sí detecta errores.
6. Jacobiano: curva error vs h (forma de V) y barrido hacia q5 = 0 (‖q̇‖ ~ q5^−1).

## Parte 3. Demostración (5 min)

1. Banda roja ROBOT REAL. Mover el robot con el joystick del FlexPendant: el 3D lo sigue (S2).
2. Secuencia demo con pausas: postura inicial, una junta, las seis, Pose A.
3. Mientras se mueve: gráfica de error en vivo, semáforo, barra de progreso.
4. Jog con flechas en cartesiano: la punta en línea recta, Pieper 50 veces por segundo. Llevarlo hacia la singularidad para que la consola lo detenga.
5. Si no se puede mover el robot ese día: el video, y la consola en simulación.

## Parte 4. Seguridad y cierre (3 min)

1. GO del día (mostrar el archivo lleno) y por qué la IA no autoriza.
2. Las protecciones de la consola (tabla de la Guía 6, sección 4) y las del controlador (STOP_HOLD, paro de emergencia).
3. Bitácora y análisis automático: un informe de sesión con retardo medido.
4. Errores corregidos (Guía 7, sección 4): tres ejemplos.
5. Qué sigue (Guía 8).

## Diapositivas (8 como máximo)

1. Título, grupo, robot.
2. Escalera de evidencia S0 → S1 → S2 → S3 con un dato en cada escalón.
3. Arquitectura bidireccional.
4. Pipeline 1–7 de la consola.
5. Validación: tabla FK y gráfica del Jacobiano.
6. Resultados del robot real: errores por junta y retardo.
7. Seguridad: GO, límites, abortos.
8. Errores corregidos y próximos pasos.

## Banco de preguntas (con respuesta corta)

**¿Por qué hay 8 soluciones de la inversa?**
Hombro adelante o atrás (q1 o q1 + 180°), codo arriba o abajo (signo de δ en la ley de cosenos) y muñeca sin giro o girada (signo de sen q5). 2 × 2 × 2.

**¿Por qué se puede separar el problema en brazo y muñeca?**
Porque los ejes 4, 5 y 6 se cruzan en un punto (muñeca esférica). La posición de ese punto depende solo de q1, q2, q3.

**¿Cómo eligen la rama?**
Válida = error FK bajo el umbral, dentro de límites con 5° de margen y σ_min ≥ 0.02. Entre las válidas, la que menos mueve el robot (distancia ponderada a la postura actual). El error FK no desempata porque en todas es ~1e-16.

**¿Por qué atan2 y no arctan?**
atan2 usa el signo de los dos argumentos y da el cuadrante correcto en 360°; arctan(y/x) confunde (1,1) con (−1,−1) y se rompe con x = 0.

**¿Qué es una singularidad y dónde están en el IRB 120?**
Postura donde J pierde rango: hay una dirección en que la punta no se puede mover. det J = −a2 L3 sin δ r_w sin q5: muñeca (q5 = 0), codo (brazo estirado o plegado) y hombro (muñeca sobre el eje 1).

**¿Por qué no arrancan desde q = 0?**
Porque q = 0 tiene q5 = 0: es singular (rango 5). La postura inicial segura tiene q5 = 40°.

**¿Qué pasa con las juntas cerca de una singularidad?**
Para mantener una velocidad de la punta, q̇ = J⁻¹ ẋ crece como 1/sen q5. En el barrido: 262 °/s con q5 = 1°. La consola corta el jog si una junta pasaría de 45 °/s.

**¿El torque se va a infinito en la singularidad?**
No. Pérdida de rango no es torque infinito. τ_ext = Jᵀ F es estática; el torque del motor es M q̈ + C q̇ + g. Lo que crece es la velocidad (y por eso la aceleración) que haría falta.

**¿Cómo saben que su FK es la del robot real?**
S0: contra el URDF de ROS-Industrial, 6e-16 en 5000 posturas. S1: contra la pose que reporta el controlador por RWS (tool0, base), con el robot quieto y las juntas leídas antes y después.

**¿Por qué el tool0 de ABB no es el marco 6 de su tabla?**
Están girados 180° en z6. Se ve en q = 0: el tool0 de ABB tiene x hacia abajo (cuaternión [0.7071, 0, 0.7071, 0]); nuestro marco 6, hacia arriba. T_tool0 = T_0^6 · Rz(180°).

**¿Por qué no se puede comparar la brida con el TCP?**
Son puntos distintos: con una herramienta de 120 mm el "error" sería 120 mm aunque los dos modelos estén bien (caso FK-D-TCP).

**¿Qué es el error de seguimiento y por qué no es cero?**
q medida − q comandada en cada instante. No es cero porque el robot copia la orden con retardo (driver, red, EGM, motores). A velocidad constante, error ≈ velocidad × retardo.

**¿Qué pasa si el error sube?**
Ámbar a la mitad del límite, rojo al pasarlo: la consola cancela la trayectoria, el controlador se queda quieto y se desarma. Después se analiza con `analizar_sesion.py`: retardo, signos, offsets o velocidad.

**¿Qué pasa si se corta el cable o se cuelga la PC?**
La consola aborta si no llegan datos en 0.5 s. Si la PC deja de mandar, EGM está en modo STOP_HOLD: el robot se detiene. Y el paro de emergencia físico siempre manda.

**¿Por qué LSPB sincronizado?**
Para que todas las juntas empiecen y terminen juntas; si no, la punta hace un camino raro. Manda la junta más exigida: t_f = max de los mínimos por velocidad y aceleración.

**¿Qué es EGM y por qué no usar RAPID con MoveJ?**
Externally Guided Motion: el controlador recibe posiciones de la PC cada 4 ms y las sigue. Con MoveJ el controlador planifica solo; con EGM la trayectoria es la nuestra (nuestro Pieper, nuestra LSPB), que es lo que se evalúa.

**¿Qué cambiaron del RAPID de ejemplo?**
El ejemplo movía el robot a una postura fija al arrancar. El nuestro crea el punto fino en la postura actual (CJointT): al pulsar PLAY no se mueve.

**¿Qué haría si la junta 3 se moviera al revés?**
Es un signo distinto. Se ve en la prueba junta por junta y en el informe ("se mueve al revés"). Se corrige en `config.json` y se confirma con `verificar_convencion_rws.py` antes de volver a mover.

**¿Qué parte hizo la IA?**
Ayudó a programar la consola, las herramientas y las guías, y generó la trayectoria de demostración y la plantilla del GO. Cada número se verificó con pruebas automáticas y contra referencias independientes, y el grupo revisó, probó y firmó. La autorización para mover el robot la da el docente, no la IA.
