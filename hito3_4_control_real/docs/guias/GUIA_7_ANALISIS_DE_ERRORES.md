# Guía 7. Analizar los datos, encontrar errores, corregirlos y mejorar

Grupo VEVAS · Qué mirar después de cada prueba y cómo convertirlo en evidencia

## 1. El análisis automático

Después de cada sesión con la consola (simulación o robot real):

```bash
cd ~/abb_ws/src/pfi-vevas-irb120/hito3_4
python3 herramientas/analizar_sesion.py --ultima          # la última sesión
python3 herramientas/analizar_sesion.py sesiones/<carpeta> # una en particular
```

Genera en `sesiones/<carpeta>/informe/`:

- `informe.md`: una fila por movimiento con duración, error de seguimiento máximo, e_p máximo, error final, e_p contra la pose pedida, retardo estimado y un diagnóstico en palabras. Al final, las entradas de bitácora ya armadas con los números de verdad, para copiar en `docs/bitacora.md`.
- `mov_NNN.png`: para cada movimiento, q comandada (punteada) contra q medida (llena), el error de cada junta con las líneas del límite, y e_p, e_R de la punta.
- `resumen_errores.png`: el error máximo de todos los movimientos lado a lado.

## 2. Qué significa cada número y cuándo preocuparse

| Número | Qué mide | Normal (a 3–12 °/s) | Si es alto, probablemente... |
|---|---|---|---|
| Error de seguimiento máximo [°] | Cuánto llegó a ir el robot detrás de la orden | 0.1 a 1° | Velocidad alta, retardo de red o de EGM |
| Error final [°] | Cuánto quedó lejos del destino, ya quieto | < 0.1° (EGM converge a ±0.1°) | No terminó de llegar, offset o problema de calibración |
| e_p de seguimiento [mm] | Lo mismo, pero en la punta | 1 a 5 mm | Igual que arriba; crece con el largo del brazo |
| e_p contra la pose pedida [mm] | Si la punta llegó a donde se pidió (según la FK con los ángulos medidos) | < 0.5 mm | Error final de juntas, o el plan se recalculó |
| Retardo estimado [ms] | Cuánto tarda el robot en "copiar" la orden | 20 a 100 ms | Red, carga de la PC, EGM |
| σ_min | Distancia a una singularidad | > 0.1 | Postura cerca de q5 = 0, brazo estirado o muñeca sobre el eje 1 |

Regla útil para explicar el error de seguimiento: a velocidad constante, **error ≈ velocidad × retardo**. Con 12.5 °/s y 40 ms: 0.5°. Si se duplica la velocidad, se duplica el error. Si el error medido es mucho mayor que velocidad × retardo, hay otra cosa (por ejemplo, el robot frenando por una supervisión).

## 3. Diagnósticos automáticos y cómo corregirlos

| El informe dice... | Qué pasa | Cómo se corrige | Cómo se confirma |
|---|---|---|---|
| "J? se mueve AL REVÉS de lo comandado" | Signo de esa junta distinto entre nuestro modelo y el robot | Cambiar `signos` en `config.json` | `verificar_convencion_rws.py` debe dar COINCIDE; repetir la prueba de esa junta |
| "J? termina con X° de diferencia: posible offset" | Diferencia constante entre lo que creemos y lo que mide | Si es la misma en todas las posturas: `offsets_deg` en `config.json`. Si cambia: no es offset (ver calibración) | Repetir en 3 posturas |
| "retardo alto (X ms)" | EGM o la red van lentos | Cable directo (sin switch), cerrar programas en la PC, bajar la velocidad | El retardo baja en la siguiente sesión |
| "error alto en J? (más de la mitad del límite)" | La junta va muy rápido para lo que el sistema puede seguir | Bajar la velocidad, o subir `t_acel_s` (aceleraciones más suaves) | El error máximo baja |
| "abortado: Sin datos del robot" | Se cortaron los `/joint_states` | Revisar cable, driver, EGM en el FlexPendant | — |
| "abortado: La interfaz dejó de responder" | El navegador quedó en segundo plano o se colgó | Dejar la consola visible; no abrir otras cosas durante la prueba | — |

## 4. Errores que ya tuvimos (y qué aprendimos)

Lista completa de los errores reales del proyecto. Sirve para la defensa: muestra que cada falla se diagnosticó con datos y se corrigió.

| # | Hito | Error | Causa | Corrección | Evidencia de la corrección |
|---|---|---|---|---|---|
| 1 | 1 | Validación TF con 1.11 m de error | Dos publicadores de `/joint_states` y tiempos desfasados | Un solo publicador, mismo instante | 2.459e-10 m en 501 muestras |
| 2 | 2 | q5 con 9.1e-15 m de error | `arccos` pierde precisión cerca de 0° y 180° | `atan2(seno, coseno)` | 5e-16 m |
| 3 | 2 | 193 ramas válidas rechazadas | Comparación exacta con el límite | Tolerancia de 1e-9 rad | 0 rechazos falsos |
| 4 | 2 | J6 daba una vuelta de más | Faltaba elegir el representante ±2π | Representante más cercano dentro de límites | Giro innecesario 6.283 → 0 rad |
| 5 | 2 | Web de la defensa con 1 rama de 8 | Error al quitar ramas repetidas | Núcleo reescrito y comparado con Python | 99/99 verificaciones |
| 6 | 2 | "La dirección perdida es angular" | Norma que mezclaba m/rad y rad/rad | Escalar con L antes de comparar | Texto corregido |
| 7 | 2 | Pico de torque citado con dos valores | Muestreo que no cae en el pico | Citar con la precisión del muestreo | ≈ 4.89 N·m |
| 8 | 3 | Ubuntu 26.04 sin soporte del driver | ROS 2 Jazzy y `abb_ros2` probados en 24.04 | Ubuntu 24.04 en disco externo | ROS y driver compilados |
| 9 | 3 | `d: no se encontró la orden` | Error de tipeo y texto del chat pegado | Un comando por línea | — |
| 10 | 3 | Archivo de la rúbrica "no existe" en la PC | Se había bajado en el celular | Todo va al repo | Este repo |
| 11 | 3 | Prueba de red que daba 403 | Se hizo desde la nube, no desde el laboratorio | Pruebas de red solo desde el Ubuntu conectado | Guía 2 |
| 12 | 3 | Orientación 180° distinta del FlexPendant (detectado al preparar esta entrega) | El marco 6 de D-H y el tool0 de ABB están girados 180° en z | `T_tool0 = T_0^6 · Rz(180°)` | 6e-16 contra el URDF de ROS-Industrial en 5000 posturas |
| 13 | 3 | e_R con resolución de 1e-8 rad (detectado al preparar esta entrega) | `arccos` en la fórmula del error de orientación | Mismo ángulo con `atan2` | `test_error_orientacion_atan2_es_mas_fino_que_arccos` |
| 14 | 3 | El RAPID de ejemplo movía el robot al arrancar (detectado al preparar esta entrega) | `MoveAbsJ home, v200` al inicio de `TRob1Main.mod` | `MoveAbsJ CJointT()`: punto fino sin movimiento | `rapid/TRob1Main_VEVAS.mod` |

Cuando aparezca un error nuevo, agregarlo a esta tabla y a la bitácora con el mismo formato.

## 5. Cómo escribir la entrada de bitácora

El formato oficial (W12) sigue el ciclo **hipótesis → prueba → resultado → interpretación → siguiente acción**. Ejemplo lleno con datos de una corrida en simulación:

```
ID: SIM-2026-11-17-001
fecha/hora: 2026-11-17 21:10
equipo y responsables: Grupo VEVAS (consola: ..., bitácora: ...)
hipotesis: la consola resuelve la Pose A con Pieper, elige la rama 0 y el robot la sigue con error < 1°
objetivo de la prueba: validar el flujo completo calcular → elegir → enviar → medir error
software/hardware y versiones: consola VEVAS 1.0 (commit ...), simulador interno, sin ROS
modo: simulacion
condiciones / frames / parametros: base del robot, tool0, velocidad 5 %, tau simulado 40 ms
prueba ejecutada: Pose (350, 150, 400) mm, Ry = 180°, desde [0,-20,30,0,40,0]°
resultado observable: 1 de 8 ramas válida (rama 0); e_p FK 1.1e-16 m; tf 3.98 s; error de
  seguimiento máx 0.56°; error final 0.004°; e_p contra la pose pedida 0.03 mm; retardo estimado 40 ms
evidencia: sesiones/2026-11-17_21-10-40_sim/informe/mov_001.png, planes/plan_001.json
estado funcional: functional (en simulación)
limitaciones / hipotesis descartadas: es S0, no dice nada del robot real; el retardo real será otro
proxima accion y responsable: repetir con ROS y use_fake_hardware (...)
revision docente/laboratorio requerida: no
```

## 6. Cómo mejorar los resultados

| Para mejorar... | Hacer | Por qué funciona |
|---|---|---|
| El error de seguimiento | Bajar la velocidad o alargar `t_acel_s` | El error es proporcional a la velocidad |
| El error de seguimiento, sin bajar la velocidad | Mandar las velocidades además de las posiciones (ya se hace en los planes) y comparar contra la q comandada de hace "un retardo" | Separa el retardo (que es normal) del error de verdad |
| El error físico real de la punta | Medir la herramienta real y poner su TCP en `config.json` | Si hay pinza, la punta no es tool0 |
| La confianza en el modelo | Más posturas en `verificar_convencion_rws.py`, incluyendo cerca de límites | Cubre más casos (FK-L del protocolo) |
| La repetibilidad | Repetir la secuencia 3 veces y comparar los informes | La rúbrica pide "pruebas repetibles" |
| La presentación | Guardar en `docs/datos/` los informes de simulación y de robot real y compararlos | Muestra el salto de S0 a S3 |
