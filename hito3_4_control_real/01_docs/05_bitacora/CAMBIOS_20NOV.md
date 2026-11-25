# Continuación del 20 de noviembre de 2026

Base: `main` en `312cd96418a469f327e692029b776d8656cf7a17`, con el trabajo
del 19/11 ya incorporado. Rama de esta continuación: `codex/continuacion-19nov`.
Trabajo realizado en el entorno de desarrollo; no se modificó el controlador ni
la copia de Ubuntu del laboratorio. El movimiento por RWS del 19/11 es un
resultado reportado por el grupo; estas pruebas no lo reemplazan ni lo vuelven a
certificar.

## Hecho, con sus límites

| Pedido | Cambio de esta revisión | Qué queda por comprobar |
| --- | --- | --- |
| Mover el objetivo sobre el robot 3D | Selector J1–J6, botones ± junto a la junta, objetivo fantasma, pasos elegibles y cálculo explícito. Traslación y anillos de rotación del TCP, en WebGL y visor compatible. | Arrastre y lectura en la PC del laboratorio; el envío físico está bloqueado por C02. |
| Coincidir con el FlexPendant | Captura RWS de juntas–pose–juntas; comparación FK de `tool0 / wobj0 / Base`, XYZ, cuaternión y Euler ZYX/ZXY. Rechaza capturas inestables. | Lecturas presenciales del FlexPendant en la misma referencia. Coincidir con IRC5 no mide precisión absoluta externa. |
| Volver a cero y postura inicial | Botones generan planes con revisión. Postura inicial usa la configuración publicada. En SIM, cero permite mostrar la aproximación terminal a la singularidad con aviso. | En modos físicos se mantiene el rechazo por singularidad; cero no equivale a calibrar ni a una postura segura universal. |
| Velocidades y frecuencia | Selector de velocidad nominal del plan; refresco de pantalla a 5, 10 o 20 Hz. | El sondeo RWS conserva su tasa propia. Falta regular y verificar velocidad articular en el controlador. |
| Trayectoria más suave | Se conserva LSPB de planificación y se corrige el reloj del filtro del simulador. | RAPID sigue con `MoveAbsJ`, velocidad fija y `fine` entre puntos: no se afirma suavidad física mejorada. |
| Mesa real | Perfil externo 80 × 80 cm, barras 5 × 5, cuatro cajas de aro compartidas por revisión y visor; centro abierto. | Apoyos inferiores solo ilustrativos. Faltan herramienta/TCP, obstáculos y medidas de estructura inferior. |
| Botón de calibración | Asistente de comparación y procedimiento guiado de consulta del FlexPendant. | No resetea contadores, no cambia offsets ni calibra automáticamente el robot. |
| RViz vacío | URDF local de lectura con orientación `tool0` corregida, configuración RViz, TF y publicación `/joint_states` sin servidor de acción. | No hay ROS 2/RViz instalados en este entorno; la ejecución real de ese visor queda para Ubuntu. |

## Mesa: referencia final

Todas las cotas están en centímetros. Desde el plano de los cuatro tornillos:
la cara superior del marco está a **19 cm**, y el tope del cilindro a **18 cm**.
La diferencia es **1 cm**; no se suman. Las interpretaciones de 37/39 cm quedan
descartadas. Los claros son 29,5 a cada lateral, 44,5 del lado opuesto a los
conectores y 14 del lado de los conectores. La huella 21,5 × 21 cm es deducida.
Ver [referencia y límites del modelo](../03_guias/MESA_ENTORNO.md).

## Correcciones del control

- El envío exige el identificador del plan revisado. Un objetivo modificado no
  puede ejecutar silenciosamente el plan anterior.
- La cancelación se comprueba antes de comprometer una secuencia; se serializan
  los escritores por hilo y se conserva el recorrido revisado, incluidas esquinas.
- Se vuelve a comprobar la vigencia del GO, la postura inicial, la huella del
  módulo y el puntero de programa. Una lectura truncada del módulo se rechaza.
- El lanzador no genera un GO automático ni sobrescribe un NO-GO. `-y` no
  confirma condiciones físicas. Las declaraciones de entorno son de sesión y no
  cambian permanentemente `config.json`.
- El nodo RWS–ROS publica estados en solo lectura. La ruta anterior de acción
  sin las compuertas de la consola queda deshabilitada.
- PARAR por RWS informa una solicitud de cancelación: el tramo ya aceptado puede
  continuar. No informa falsamente que el brazo físico quedó quieto.
- Un plan por llegada RWS no tiene seguimiento temporal LSPB medido. Sus campos
  temporales no disponibles no se rellenan con ceros y se distingue duración
  observada de duración planificada.
- C02 sigue pendiente: `speeddata` del TCP no acredita un límite por junta.
  El backend de producción bloquea preparación/envío; los tests usan un backend
  acotado exclusivo para el IRC5 falso, sin opción de habilitarlo por IP o config.
  El módulo RAPID queda intacto, con confirmación del FlexPendant.

## Pruebas y evidencia de desarrollo

La primera suite integral dio 147 aprobadas y un fallo en la simulación. El
filtro usaba un paso fijo mientras la referencia avanzaba con el reloj real; un
retraso del scheduler producía seguimiento ficticiamente peor. Se corrigió el
filtro para usar tiempo monotónico transcurrido, conservando el límite de error.
La nueva regresión verifica la misma respuesta tras 200 ms con 1, 5 y 50 ciclos:
en la base fallaban tres casos y en la corrección pasan los cuatro.

La suite posterior completó **152 pruebas Python aprobadas**, y los tests de
orientación completaron **5 pruebas JavaScript aprobadas**. También pasaron la
sintaxis de cuatro scripts Bash, cinco módulos JavaScript y `git diff --check`.
Las pruebas Python usan simulación, HTTP local y un IRC5 falso; las de ROS
validan estructura/helpers con stubs, no un ROS 2 en ejecución.

```bash
python3 -m pytest -q tests/
node tests/test_orientacion.mjs
bash -n iniciar_real.sh iniciar_simulacion.sh iniciar_real_egm.sh subir_git.sh
git diff --check
```

La revisión de navegador en SIM comprobó el selector J2 y la edición de −20° a
−18° con clic, seguida de Calcular; el estado medido permaneció separado del
objetivo. Se corrigió además el movimiento subpíxel de los botones flotantes
por el ruido sintético. Las pruebas de visor cubrieron WebGL y Canvas, anillos,
invalidación del plan tras editar, respuestas IK atrasadas, rechazo y animación
del recorrido, frecuencia, comparación con fixture y envío explícito a SIM.
No se enviaron órdenes al IRC5 físico.

Los resultados antes y después y la captura de la interfaz están en
`docs/datos/pruebas_20nov/` (evidencia local del 20/11, no versionada en el repositorio).

## Siguiente paso

La revisión está destinada a prueba en **SIM y solo lectura**. Seguir
[PRUEBA_INTERFAZ_20NOV.md](../06_operacion/PRUEBA_INTERFAZ_20NOV.md). Antes de movimiento
físico, resolver C02, medir herramienta y entorno y completar la verificación
presencial. Las ideas adicionales están en [MEJORAS_EXTRAS.md](../08_desarrollo/MEJORAS_EXTRAS.md)
y las tareas para retomar en [PROXIMOS_PROMPTS.md](../08_desarrollo/PROXIMOS_PROMPTS.md).
