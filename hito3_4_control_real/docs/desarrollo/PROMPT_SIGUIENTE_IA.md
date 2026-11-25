# Handoff VEVAS Studio · 20/11/2026

> Sos mi asistente para VEVAS Studio, PFI Hitos 3/4 de Robótica IMT-342 (UCB Tarija). Hablame en español informal, con comandos completos y entregables que pueda usar. Trabajá desde `hito3_4` del repo `FlorenciaFrigerio/pfi-vevas-irb120`. Revisá primero el estado de la rama y los cambios locales.

## Leé primero

1. [Cambios del 20/11](CAMBIOS_20NOV.md) y [prueba de interfaz](docs/PRUEBA_INTERFAZ_20NOV.md).
2. [Avances y contexto](AVANCES_CONTEXTO.md) y [registro del 19/11](CAMBIOS_19NOV.md).
3. [Prompts por tarea](docs/PROXIMOS_PROMPTS.md), [mejoras pendientes](docs/MEJORAS_EXTRAS.md) y [guía de RViz](docs/RVIZ_RWS.md).

## Estado actual

- La consola web ofrece simulación, arrastre del TCP con flechas y anillos, IK de Pieper, comprobación de FK, singularidades y revisión del plan. El envío en SIM es explícito. Verificá la versión local con la prueba de interfaz y registrá los resultados que obtengas.
- `bash iniciar_real.sh --solo-chequeo` consulta el estado. En la PC del laboratorio, `bash iniciar_real.sh --solo-lectura` abre una vista RWS con el lector ROS y RViz. Ese flujo publica postura sin acción de movimiento ni ejecución de MoveIt. La comparación con el FlexPendant es asistida y en lectura. Los contadores de revolución y los offsets del controlador quedan intactos.
- El grupo reportó movimiento del ABB IRB 120 por RWS el 19/11. Los registros físicos originales y la validación del arrastre y de los anillos con el brazo siguen pendientes. El chat registra EGM «Connected» el 18/11 con la PC en `.10`. El fallo de conexión EGM del 19/11 ocurrió con la PC en `.207`.
- C01–C04 tenían coberturas parciales en la base auditada. La rama incorpora correcciones de software. El backend y los puntos de entrada de movimiento mantienen bloqueados el envío real y `mover` hasta verificar C02. Los 5°/s de `vPc` describen reorientación del TCP. Mantener las compuertas, el GO manual por sesión y `confirmarEnPendant := TRUE`.
- Usá las pruebas de Python y de JavaScript disponibles para validar el cambio en local. Incluí el comando, la versión y lo que efectivamente pasó. El IRC5 simulado permite verificar el protocolo en local. La velocidad real por junta, el seguimiento temporal y la parada física se verifican en el laboratorio.

## Mesa confirmada

Marco exterior de 80 × 80 cm y perfil de 5 × 5 cm. Los claros son 29,5 cm en cada lateral, 44,5 cm del lado opuesto a los conectores y 14 cm del lado de los conectores. Desde el plano de los tornillos, el canto superior del marco está a 19 cm y el tope del cilindro a 18 cm. La diferencia es de 1 cm. Los valores anteriores de 37 y 39 cm quedan descartados.

## Continuación

Completá primero las pruebas en SIM y el diseño de la verificación de C02. Seguí con la coincidencia de las seis juntas entre el FlexPendant, RWS y el gemelo, y con la inspección de la mesa, herramienta y obstáculos. Para evaluar suavidad, retorno a cero y velocidad derivada, registrá el origen de las muestras, sus marcas temporales y la frecuencia efectiva. Incorporá al informe los registros originales y video disponibles con su procedencia.

Prepará por escrito el protocolo para cualquier ensayo supervisado. No conectes al controlador ni ejecutes hardware de forma automática. El equipo inicia cualquier ensayo físico con autorización del docente y las compuertas vigentes.
