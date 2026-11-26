# Bitácora del Hito 3/4 (formato oficial W12)

Grupo VEVAS · Susann A. Baldiviezo C., Alejandro J. Bejarano R., Florencia Frigerio A., Benjamín Soruco G.
Una entrada por prueba o decisión. Ciclo: hipótesis → prueba → resultado → interpretación → siguiente acción.

```
ID: G0-001
fecha/hora: 2026-11-16 (auditoría inicial de celda)
equipo y responsables: Susann Baldiviezo, Alejandro Bejarano, Florencia Frigerio, Benjamín Soruco
hipotesis: el IRC5 tiene las opciones necesarias para ROS 2 por RWS + EGM
objetivo de la prueba: identificar robot, controlador, RobotWare y opciones instaladas
software/hardware y versiones: IRB 120-3/0.6 s/n 120-101516, IRC5 Compact, FlexPendant DSQC 679
modo: auditoria
condiciones / frames / parametros: sin movimiento (MOTORS OFF)
prueba ejecutada: lectura de etiquetas físicas y de Información del sistema en el FlexPendant
resultado observable: RobotWare 6.12.01.00; opciones 689-1 EGM, 616-1 PC Interface, UDPUC; StateMachine 1.1; service port 192.168.125.1/24
evidencia: registro en informe IEEE sección IV-A (informe_hito3_4_ieee.tex) y capturas locales del FlexPendant
estado funcional: functional
limitaciones / hipotesis descartadas: el módulo StateMachine AddIn impedía movimiento externo directo sin una máquina RAPID dedicada
proxima accion y responsable: diseñar arquitectura RWS + RAPID dedicada (VEVAS_RWS.mod) — Grupo VEVAS
revision docente/laboratorio requerida: aviso de mantenimiento preventivo vencido (3196 días) notificado al encargado
```

```
ID: G0-002
fecha/hora: 2026-11-16
equipo y responsables: Grupo VEVAS
hipotesis: ROS 2 Jazzy en Ubuntu 24.04 compila abb_ros2 y el paquete propio del IRB 120
objetivo de la prueba: preparar el stack ROS 2 en disco externo dedicado
software/hardware y versiones: Ubuntu 24.04 LTS (disco externo), ROS 2 Jazzy, abb_ros2, irb120_description
modo: simulacion
condiciones / frames / parametros: use_fake_hardware:=true
prueba ejecutada: colcon build y lanzamiento de simulación sin hardware real
resultado observable: compilación limpia y publicación del árbol TF base_link -> tool0
evidencia: 05_ros2_irb120_description/ y flujo automatizado en .github/workflows/ci.yml
estado funcional: functional
limitaciones / hipotesis descartadas: Ubuntu 26.04 descartado por falta de soporte de binarios ROS 2 Jazzy
proxima accion y responsable: ejecutar validación Gate 1 de solo lectura frente al IRC5 — Grupo VEVAS
revision docente/laboratorio requerida: no
```

```
ID: G1-001
fecha/hora: 2026-11-19
equipo y responsables: Grupo VEVAS
hipotesis: la PC puede leer el estado de las juntas del IRC5 por RWS sin mover el robot
objetivo de la prueba: Gate 1, lectura remota sin movimiento
software/hardware y versiones: cliente RWS HTTP Digest (07_vevas/rws_cliente.py), RViz
modo: comunicacion sin movimiento
condiciones / frames / parametros: enlace Ethernet al puerto SERVICE (192.168.125.1), PC en 192.168.125.10/24
prueba ejecutada: lectura de juntas por RWS y visualización pasiva en RViz
resultado observable: lectura fluida de J1–J6; variación observada entre 5° y 38° durante jog manual desde el FlexPendant
evidencia: 01_CAMBIOS_19NOV.md y herramienta 02_herramientas/conectar_robot.py
estado funcional: functional
limitaciones / hipotesis descartadas: faltaba contrastar numéricamente la FK con el robtarget del FlexPendant (cerrado en S1-001)
proxima accion y responsable: contraste formal de convención y signos en laboratorio (S1-001) — Grupo VEVAS
revision docente/laboratorio requerida: no (solo lectura)
```

```
ID: S0-001
fecha/hora: 2026-11-17
equipo y responsables: Grupo VEVAS
hipotesis: la tabla D-H del grupo, con tool0 = T_0^6·Rz(180°), coincide con el URDF de ROS-Industrial
objetivo de la prueba: validación offline de FK, IK y Jacobiano (protocolo W12)
software/hardware y versiones: hito3_4/vevas, 02_herramientas/validacion_offline.py
modo: simulacion
condiciones / frames / parametros: base ABB, tool0, joint_1..joint_6, rad y m, signos +1, offsets 0
prueba ejecutada: casos FK-N1, FK-N2, FK-L, FK-D-TCP, FK-D-ORD, 5000 posturas al azar, IK-1/2/S, J vs diferencias finitas, barrido q5
resultado observable: FK 6e-16 m en 5000 posturas; FK-D-TCP y FK-D-ORD fallan a propósito (0.120 m y 0.469 rad); J 9e-11 m/rad; ||q̇|| ~ q5^-1.00
evidencia: docs/01_datos/01_validacion_offline/ (casos_fk.csv, casos_ik.csv, 01_resumen.md, gráficas)
estado funcional: functional
limitaciones / hipotesis descartadas: S0, no dice nada del robot real
proxima accion y responsable: S1 con verificar_convencion_rws.py
revision docente/laboratorio requerida: aprobar tolerancias 1e-6 m y 1e-5 rad
```

```
ID: SIM-001
fecha/hora: 2026-11-17
equipo y responsables: Grupo VEVAS
hipotesis: la consola ejecuta el flujo calcular → elegir rama → LSPB → enviar → medir error y aborta ante fallas
objetivo de la prueba: validar la consola en simulación interna
software/hardware y versiones: consola VEVAS 1.0, simulador interno (tau 40 ms)
modo: simulacion
condiciones / frames / parametros: velocidad 5 %, Pose A (350, 150, 400) mm Ry 180°
prueba ejecutada: 18 pruebas automáticas (06_tests/), Pose A, jog cartesiano y articular, secuencia demo, aborto por interfaz
resultado observable: 1 de 8 ramas válida; error de seguimiento máx 0.56°; error final 0.004°; e_p contra la pose pedida 0.03 mm; retardo estimado 40 ms (igual al simulado); aborto por interfaz a 1 s
evidencia: 06_tests/test_cinematica.py, sesiones de simulación (analizar_sesion.py)
estado funcional: functional
limitaciones / hipotesis descartadas: S0
proxima accion y responsable: repetir con ROS y use_fake_hardware (05_MENSAJE_PARA_CLAUDE_CODE.md)
revision docente/laboratorio requerida: no
```

## Plantilla para las próximas entradas

```
ID:
fecha/hora:
equipo y responsables:
hipotesis:
objetivo de la prueba:
software/hardware y versiones:
modo: simulacion / auditoria / comunicacion sin movimiento / movimiento autorizado
condiciones / frames / parametros:
prueba ejecutada:
resultado observable:
evidencia (log, commit, foto permitida, archivo):
estado funcional: functional / partially_verified / blocked / not verified
limitaciones / hipotesis descartadas:
proxima accion y responsable:
revision docente/laboratorio requerida:
```


## 20/11/2026 · Interfaz, marco medido y revisión del control

Hecho en desarrollo: controles 3D J1–J6 y anillos, cálculo/revisión explícitos con identificación del plan, comparación RWS/FlexPendant de solo lectura, perfil compartido de mesa abierta, RViz de lectura y correcciones de GO/cancelación/frescura/módulo. Desde los tornillos, marco a 19 cm y cilindro a 18 cm: diferencia 1 cm, sin sumar.

La primera suite tuvo 147/148 por un reloj inconsistente del simulador. Se corrigió el paso de integración sin cambiar el umbral; la suite posterior pasó 152/152 y JavaScript 5/5. La interfaz se revisó en navegador con datos SIM. Evidencia y detalle en [02_CAMBIOS_20NOV.md](02_CAMBIOS_20NOV.md) y pruebas de software en `docs/01_datos/pruebas_20nov/` (evidencia local, no versionada).

Pendiente: velocidad por junta y suavidad ejecutada en IRC5 (C02), validación presencial de comparación y RViz, herramienta/TCP, obstáculos, informe y video. El envío físico queda bloqueado. Siguiente paso: [ensayo SIM y solo lectura](../06_operacion/03_PRUEBA_INTERFAZ_20NOV.md). No se modificó el IRC5 ni la copia del Ubuntu del laboratorio; esta revisión todavía debe publicarse y descargarse.


## 20/11/2026 · Publicación pendiente

El complemento GitHub devolvió 403 Resource not accessible by integration y Git HTTPS autenticado devolvió 403 Permission denied. No se publicó rama ni se modificó main. Se entrega el parche y comandos para aplicar, probar y subir desde Ubuntu. Los cambios posteriores a las 152 pruebas Python y 5 JavaScript solo afectan documentación.


## 22/11/2026 · Entrega descargable alternativa

Los enlaces de archivo del chat no funcionaron en Ubuntu. Se subió el ZIP completo para SIM a file.io/LimeWire con la autorización previa del usuario. Enlace temporal: https://limewire.com/d/oLZi1#hAnGcIJXCt (el sitio indica 3 días). Se descargó y verificó igualdad SHA256 con el original de 2419517 bytes: `a904975c490d69b7861810aac1dc9cb9753a86da936057d77e285576d450b8af`. Software sin cambios; no se ejecutó hardware ni se publicó en GitHub.


## 23/11/2026 · Integración y preparación del laboratorio

Se integraron las mejoras de 01_figuras/suavidad/arrastre/confirmación y traza, junto con diagnóstico RWS de solo lectura y guía de conexión. La suite final pasó 232 pruebas Python y 6 JS. Las capturas de RWS son sintéticas. El envío físico sigue bloqueado por C02; se documentó la propuesta ABB SpeedLimAxis/LimitSpeed sin instalarla. Ver [04_CAMBIOS_23NOV.md](04_CAMBIOS_23NOV.md). Pendiente recibir la ruta/ping y lectura del Ubuntu real, herramienta/TCP y entorno, y validar el control en laboratorio.

## 23/11/2026 · Proyecto independiente ROBOTICA_01

Se agregó el exportador `02_herramientas/empaquetar_robotica01.py` y la guía
`docs/06_ORGANIZACION_ROBOTICA_01.md`. La entrega tiene código único en
`00_codigo/hito3_4` y apartados 01–09 para interfaz, conexión, cálculos, 03_modelo/mesa,
seguridad, pruebas, documentación, datos nuevos e historial. Cada apartado tiene
README; se incluyen contexto útil del chat, estado y próximos prompts para «brazo».
Destino previsto en Ubuntu: `/home/huesero/Descargas/ROBOTICA_01/proyecto_brazo`.
Los lanzadores resuelven rutas relativas y guardan sesiones fuera del código.
Se conservan ZIP original, fuentes del docente y bundle Git; se omiten venv,
cachés y copia anidada del código activo. El bundle conserva el historial antiguo.

Verificación de la copia reorganizada: 232 pruebas Python correctas en 186,70 s,
6 JavaScript correctas, seis scripts con sintaxis Bash válida, manifiesto SHA256
válido y clonación real del bundle sin red a un repositorio limpio. Se arrancó
SIM desde otro cwd y una ruta con espacios, cargando HTML/JS/Three y STL, mesa a
190 mm con cuatro cajas y sesiones en 08_datos_locales. No se cambió control de
movimiento ni se envió nada al robot; C02 continúa bloqueando el envío físico.

Pendiente de entrega: copiar/extraer la entrega en la Ubuntu del usuario y
vincular este chat con su proyecto ChatGPT «brazo» mediante el cliente del usuario.
Las herramientas disponibles solo acceden a la nube y no mueven conversaciones
entre proyectos. El archivo de contexto permite continuar allí sin perder decisiones.
No se afirma push: la publicación previa recibió 403 y no se reintentó.

## Interfaz SIM · revisión sobre la copia del ZIP

- Cambios en `07_vevas/consola.py`, `08_web/index.html`, `08_web/app.js` y `06_tests/test_ik_vivo.py`. El nuevo atajo parte de una muestra fresca, modifica solo J5 y conserva revisión y confirmación.
- El retorno en SIM intenta el plan directo y, si se rechaza, hasta tres puntos intermedios con revisión del trayecto completo.
- SIM comprueba llegada con tolerancias predeterminadas de 0,25° articulares y, para objetivo cartesiano, 2 mm y 1° respecto a la pose pedida. Si al cabo de 5 s sobre el plan sigue fuera de tolerancia, aborta. El watchdog sigue controlando la frescura del estado.
- Pruebas ejecutadas desde `00_codigo/hito3_4`: `python3 06_tests/test_ik_vivo.py` (9), `python3 06_tests/test_control_rws.py` (16), `python3 06_tests/test_revision.py` (16), `node --test 06_tests/test_orientacion.mjs` (6) y sintaxis de `08_web/app.js`. Quedó pendiente la suite completa porque el entorno no tenía pytest.
- El modelo de mesa devuelve cuatro cajas, centro abierto y aro superior a 190 mm. Quedan por comprobar el visor y las cotas pendientes del entorno. El atajo corto de J5 tarda 1,50 s a 5 %, 10 % y 20 % en el plan revisado debido al tiempo mínimo del perfil. Quedan pendientes las pruebas en navegador y el diagnóstico de fluidez.

## 24/11/2026 · Lectura presencial

El 24/11/2026, la lectura RWS de J1–J6 coincidió con el FlexPendant. FK con tool0, wobj0 y Base: 0,0001 mm y 0,0000° según el diagnóstico. La operadora confirmó que el robot está calibrado. C02 sigue pendiente.

## 24/11/2026 · C02 por tiempo acotado y medición

- `05_rapid/VEVAS_RWS.mod`: `MoveAbsJ \T` con tiempo calculado en el RAPID por eje (vLim 5 %, kPico 2.5) y `PERS vvRbTiempo`. Se quitaron los procedimientos de depuración `VevasMostrarObjetivo`/`MostrarEjes` y la sangría pasó a 2 espacios para quedar en 4123 caracteres (< 4800).
- `07_vevas/c02.py` (evaluación de evidencia y velocidad por ventana), `02_herramientas/verificar_c02.py` (medición presencial), `07_vevas/backend_rws.py` (habilita solo con evidencia; comprueba modo y módulo del IRC5 antes de cada envío; no reinstala un servidor verificado que ya corre), `07_vevas/consola.py` (no envía si el GO autoriza menos del 5 %), `08_web/app.js` (muestra el motivo de C02).
- Lanzadores `05_seguridad_y_control/verificar_c02.sh` y `abrir_control_real.sh`.
- Pruebas iniciales en SIM/IRC5 simulado: 242 Python (8 nuevas en `06_tests/test_c02.py`) y 6 JavaScript; posteriormente ampliadas con streaming `MoveAbsJ \T:=t, z1`, figuras en plano `YZ` («De frente») y pestaña de velocidades medidas hasta **329 pruebas Python y 20 JavaScript**.

```
ID: S1-001
fecha/hora: 2026-11-24
equipo y responsables: Susann Baldiviezo, Alejandro Bejarano, Florencia Frigerio, Benjamín Soruco
hipotesis: la cinemática directa de VEVAS Studio (tool0 = T_0^6 · Rz(180°)) coincide en posición y orientación con el controlador ABB IRC5
objetivo de la prueba: verificación presencial de convención articular, marcos y calibración (Gate 1 completo)
software/hardware y versiones: VEVAS Studio (02_herramientas/conectar_robot.py), IRB 120-3/0.6 + IRC5 RobotWare 6.12.01.00
modo: comunicacion sin movimiento
condiciones / frames / parametros: marco Base, wobj0, tool0 sin herramienta adicional, puerto SERVICE 192.168.125.1
prueba ejecutada: contraste simultáneo entre lecturas RWS de J1–J6 / robtarget y pantalla del FlexPendant
resultado observable: coincidencia exacta de J1–J6; diferencia FK frente al controlador de 0,0001 mm en XYZ y 0,0000° en orientación; robot con calibración verificada
evidencia: 05_CAMBIOS_24NOV.md, informe_hito3_4_ieee.tex (sección V-B)
estado funcional: functional
limitaciones / hipotesis descartadas: ninguna discrepancia de signos ni de offsets articulares
proxima accion y responsable: ejecutar verificación presencial de velocidad acotada C02 en modo AUTO — Grupo VEVAS
revision docente/laboratorio requerida: sí (supervisión de laboratorio cumplida)
```

```
ID: C02-001
fecha/hora: 2026-11-24
equipo y responsables: Susann Baldiviezo, Alejandro Bejarano, Florencia Frigerio, Benjamín Soruco
hipotesis: la ley MoveAbsJ \T:=t en VEVAS_RWS.mod mantiene cada eje por debajo del 5 % de la velocidad máxima de catálogo ABB (12,5 / 16,0 / 21,0 °/s)
objetivo de la prueba: cerrar la compuerta de seguridad C02 midiendo velocidades reales por RWS a ~30 Hz
software/hardware y versiones: VEVAS_RWS.mod (4707 caracteres), 02_herramientas/verificar_c02.py, 07_vevas/c02.py (VERSION_EVIDENCIA = 2)
modo: movimiento autorizado
condiciones / frames / parametros: modo AUTO, GO de sesión activo (tope 5 %), desplazamientos ±5° por junta y secuencia de streaming en J6
prueba ejecutada: ejecución presencial de verificar_c02.py con paro de emergencia a mano
resultado observable: picos medidos de 5,2–5,4 °/s en J1–J3 (límite 12,5 °/s), 6,7–6,9 °/s en J4–J5 (límite 16,0 °/s) y 8,8–8,9 °/s en J6 (límite 21,0 °/s), equivalentes al 42–43 % del tope C02 (~2,16 % de catálogo)
evidencia: 01_docs/06_figuras/05_velocidades_c02.png, 05_CAMBIOS_24NOV.md e informe_hito3_4_ieee.tex (Tabla III y Fig. 4)
estado funcional: functional
limitaciones / hipotesis descartadas: evidencia_c02.json y GO_VEVAS.json permanecen locales por política de seguridad (nunca versionados en Git)
proxima accion y responsable: emplear el registro de sesión y video sincronizado como figura central de defensa — Grupo VEVAS
revision docente/laboratorio requerida: cumplida en celda
```
