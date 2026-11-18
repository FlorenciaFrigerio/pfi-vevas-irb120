# Bitácora del Hito 3/4 (formato oficial W12)

Grupo VEVAS. Una entrada por prueba o decisión. Ciclo: hipótesis → prueba → resultado → interpretación → siguiente acción. Las entradas marcadas "completar" se llenan con los datos reales (fechas exactas, nombres, archivos).

```
ID: G0-001
fecha/hora: noviembre 2026 (completar)
equipo y responsables: Grupo VEVAS (completar)
hipotesis: el IRC5 tiene las opciones necesarias para ROS 2 por RWS + EGM
objetivo de la prueba: identificar robot, controlador, RobotWare y opciones
software/hardware y versiones: IRB 120-3/0.6 s/n 120-101516, IRC5 Compact, FlexPendant DSQC 679
modo: auditoria
condiciones / frames / parametros: sin movimiento
prueba ejecutada: lectura de etiquetas y de Información del sistema en el FlexPendant
resultado observable: RobotWare 6.12.01.00; opciones 689-1 EGM, 616-1 PC Interface, UDPUC; StateMachine 1.1; service port 192.168.125.1/24
evidencia: fotos del FlexPendant y de las etiquetas (completar nombres de archivo en docs/datos/gate0/)
estado funcional: functional
limitaciones / hipotesis descartadas: backup del controlador no confirmado
proxima accion y responsable: confirmar y copiar el backup (completar)
revision docente/laboratorio requerida: aviso de mantenimiento vencido (3196 días) al encargado
```

```
ID: G0-002
fecha/hora: noviembre 2026 (completar)
equipo y responsables: Grupo VEVAS
hipotesis: ROS 2 Jazzy en Ubuntu 24.04 compila abb_ros2 y un paquete propio del IRB 120
objetivo de la prueba: preparar el stack ROS
software/hardware y versiones: Ubuntu 24.04 (disco externo), ROS 2 Jazzy, abb_ros2 (rolling), abb_irb120_support en ~/abb_ws/src/abb_irb120
modo: simulacion
condiciones / frames / parametros: use_fake_hardware:=true
prueba ejecutada: colcon build y lanzamiento de abb_control.launch.py
resultado observable: compila y lanza (según Claude Code en la PC; completar con el log)
evidencia: completar (salida de colcon build y ros2 control list_controllers)
estado funcional: partially_verified
limitaciones / hipotesis descartadas: Ubuntu 26.04 descartado (sin soporte confirmado del driver)
proxima accion y responsable: guardar logs en docs/datos/
revision docente/laboratorio requerida: no
```

```
ID: G1-001
fecha/hora: noviembre 2026 (completar)
equipo y responsables: Grupo VEVAS
hipotesis: la PC puede leer el estado de las juntas del IRC5 por RWS sin mover el robot
objetivo de la prueba: Gate 1, lectura sin movimiento
software/hardware y versiones: launch de solo lectura por RWS creado por Claude Code, RViz
modo: comunicacion sin movimiento
condiciones / frames / parametros: cable al puerto SERVICE, PC 192.168.125.10
prueba ejecutada: lectura de juntas por RWS y visualización en RViz
resultado observable: lectura correcta; J4 a J6 cambiaron entre 5° y 38° (probable jog manual con el FlexPendant)
evidencia: completar (log con hora, captura del FlexPendant con los mismos valores)
estado funcional: partially_verified
limitaciones / hipotesis descartadas: sin la captura del FlexPendant no se puede afirmar que coinciden
proxima accion y responsable: repetir con herramientas/verificar_convencion_rws.py --posturas 3 (Guía 2, paso 5)
revision docente/laboratorio requerida: no (solo lectura)
```

```
ID: S0-001
fecha/hora: 2026-11-17
equipo y responsables: Grupo VEVAS
hipotesis: la tabla D-H del grupo, con tool0 = T_0^6·Rz(180°), coincide con el URDF de ROS-Industrial
objetivo de la prueba: validación offline de FK, IK y Jacobiano (protocolo W12)
software/hardware y versiones: hito3_4/vevas, herramientas/validacion_offline.py
modo: simulacion
condiciones / frames / parametros: base ABB, tool0, joint_1..joint_6, rad y m, signos +1, offsets 0
prueba ejecutada: casos FK-N1, FK-N2, FK-L, FK-D-TCP, FK-D-ORD, 5000 posturas al azar, IK-1/2/S, J vs diferencias finitas, barrido q5
resultado observable: FK 6e-16 m en 5000 posturas; FK-D-TCP y FK-D-ORD fallan a propósito (0.120 m y 0.469 rad); J 9e-11 m/rad; ||q̇|| ~ q5^-1.00
evidencia: docs/datos/validacion_offline/ (casos_fk.csv, casos_ik.csv, resumen.md, gráficas)
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
prueba ejecutada: 18 pruebas automáticas (tests/), Pose A, jog cartesiano y articular, secuencia demo, aborto por interfaz
resultado observable: 1 de 8 ramas válida; error de seguimiento máx 0.56°; error final 0.004°; e_p contra la pose pedida 0.03 mm; retardo estimado 40 ms (igual al simulado); aborto por interfaz a 1 s
evidencia: tests/test_cinematica.py, sesiones de simulación (analizar_sesion.py)
estado funcional: functional
limitaciones / hipotesis descartadas: S0
proxima accion y responsable: repetir con ROS y use_fake_hardware (MENSAJE_PARA_CLAUDE_CODE.md)
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
