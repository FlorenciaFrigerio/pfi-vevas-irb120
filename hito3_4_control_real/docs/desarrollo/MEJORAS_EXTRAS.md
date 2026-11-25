# Mejoras para VEVAS Studio

El grupo reportó movimiento físico por RWS el 19/11. La validación del arrastre web y de los anillos con el brazo sigue pendiente. La prioridad P0 habilita ensayos de movimiento después de verificar C02. Las prioridades P1 y P2 pueden avanzar con lectura, simulación y documentación. El esfuerzo es relativo al código y a las sesiones de laboratorio necesarias.

| Prioridad | Mejora | Beneficio | Esfuerzo | Evidencia necesaria |
|---|---|---|---|---|
| P0 | Validar el control temporal del movimiento físico y el límite de velocidad de cada junta (C02). | Permite evaluar el movimiento físico contra el límite del GO y cuantificar la diferencia entre el LSPB planificado y lo ejecutado. | Alto | Referencia del controlador para la semántica de velocidad y temporización, pruebas locales de bloqueo ante datos faltantes, registro autorizado de postura por junta con marcas temporales y condiciones del ensayo. Los campos de 20 mm/s y 5°/s de `vPc` corresponden al TCP. |
| P1 | Medir mesa, obstáculos y herramienta con respecto al marco base. | Alinea la escena 3D y la revisión previa con el laboratorio. | Medio | Croquis con origen, ejes, referencias, unidades y fotos. Usar las medidas confirmadas detalladas abajo y comprobar su correspondencia con la base, los obstáculos y la herramienta. |
| P1 | Ajustar la revisión de colisiones y evaluar la planificación en simulación. | Identifica restricciones de espacio durante el recorrido y ayuda a explicar los rechazos en la web. | Alto | Casos reproducibles con pose inicial, escena versionada y trayectoria. Comparar cápsulas y geometría medida. Registrar el instante, la pose y los objetos del contacto detectado. Incluir casos despejados y casos con obstáculos conocidos. |
| P1 | Verificar coincidencia entre FlexPendant, lectura RWS, gemelo web y RViz. | Hace visibles los errores de orden, signo, offset o unidades antes de usar las posturas en IK o FK. | Medio | Lecturas sincronizadas del FlexPendant y RWS, tabla de las seis juntas con unidades y sello temporal, capturas y trazas de `/joint_states`, `/robot_description` y TF. Probar primero con una postura sintética. |
| P1 | Entregar informe, bitácora y video del Hito 3/4. | Deja un rastro que el equipo y el docente pueden reproducir y defender. | Medio | Matriz por requisito con commit, escenario, resultado y evidencia. Diferenciar lo simulado, lo reportado por el grupo, lo observado físicamente con registro y lo pendiente. |
| P2 | Catalogar y reproducir las sesiones históricas. | Aclara los cambios entre EGM el 18/11 y RWS el 19/11 y permite comparar revisiones del proyecto. | Medio | Logs originales, CSV RWS, capturas, origen de cada archivo, fecha y versiones de RobotWare y del software cuando consten. Registrar los faltantes. Usar datos sintéticos identificados para probar la reproducción. |
| P2 | Registrar la telemetría disponible. | Permite estudiar latencia, seguimiento y detenciones a partir de medidas trazables. | Medio | Inventario de campos realmente disponibles por interfaz y versión, su unidad, frecuencia, latencia y frescura. Derivar velocidades de posiciones con sello temporal cuando la tasa lo permita e identificar esa estimación. Incluir torque solo si existe una fuente validada, con su unidad y procedencia. |

## Medidas confirmadas de la mesa

Marco exterior de 80 × 80 cm, perfil de 5 × 5 cm, claro de 29,5 cm en cada lateral, 44,5 cm del lado opuesto a los conectores y 14 cm del lado de los conectores. El canto superior del marco está a 19 cm del plano de los tornillos y el tope del cilindro a 18 cm del mismo plano. La diferencia es de 1 cm. Los valores anteriores de 37 y 39 cm quedan descartados.

## Asistencia de calibración

Un asistente de comparación puede alinear en el tiempo la postura del FlexPendant con la de RWS, convertir unidades, mostrar residuos por junta y registrar observaciones. El operador y el docente conservan el control de cualquier procedimiento de calibración del IRC5. El asistente trabaja en lectura y deja intactos los contadores de revolución, la calibración y los offsets del controlador. Las opciones de postura en la web son objetivos de movimiento sujetos a revisión y GO.

## Pendientes de control y análisis

- En SIM, comparar la suavidad de los planes, los puntos de unión entre tramos y el retorno a cero. Guardar posición, velocidad y aceleración planificadas junto con el resultado de revisión.
- Para analizar seguimiento físico, registrar muestras fechadas por junta e informar frecuencia de lectura configurada, frecuencia efectiva y pérdidas. Distinguir el ritmo de adquisición de la frecuencia de refresco del gráfico. Las velocidades derivadas requieren muestras con intervalos conocidos. El RWS por waypoints trabaja por llegada y el LSPB calculado en la web no garantiza la temporización ejecutada.
- Continuar la comparación guiada RWS/FlexPendant en lectura. El resultado físico depende de lecturas del equipo y postura comprobada en sitio. El backend mantiene bloqueado el envío real mientras su velocidad articular siga sin verificar.

## Orden sugerido para probar

1. En desarrollo, ejecutar las pruebas locales, la simulación web y RViz con posturas sintéticas.
2. En el laboratorio, completar el croquis y validar la lectura de postura del brazo detenido frente al FlexPendant.
3. Revisar con el docente la evidencia de C02 y los requisitos del GO manual. Mantener la confirmación en el FlexPendant.
4. Con esas condiciones satisfechas, documentar el ensayo presencial autorizado bajo supervisión y con el protocolo del laboratorio. El equipo inicia los ensayos físicos de forma deliberada.

Para tareas listas para retomar en otro chat, ver [Próximos prompts](PROXIMOS_PROMPTS.md).
