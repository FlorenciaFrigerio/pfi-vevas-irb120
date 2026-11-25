# VEVAS Studio — avances y contexto (para retomar en otro chat)

Última actualización: 2026-11-19. Grupo VEVAS, Robótica IMT-342 (UCB Tarija), PFI Hitos 3/4.
Carpeta de trabajo: `~/Descargas/ROBOTICA/vevas_interfaz_prototipo_2026-11-19`.

## Robot y red
- ABB IRB 120-3/0.6, IRC5 Compact, RobotWare 6.12.01, sistema `120-101516`.
- IRC5 en `192.168.125.1` (puerto SERVICE). RWS 1.0, digest, usuario `Default User` / `robotics`.
- La PC recibe la IP por DHCP (se vio `192.168.125.207` en `eno2`).
- ROS 2 Jazzy, con el workspace `~/abb_ws` (abb_ros2 compilado).

## RWS como vía principal
- El chat del 18/11 registra EGM «Connected», con la PC en `.10` y JSB/JTC activos. En la sesión del 19/11, el IRC5 seguía enviando a `.10` y la PC estaba en `.207` por DHCP. El driver quedó en «Not connected» durante esa sesión y el grupo continuó por RWS.
- El módulo RWS toma como referencia el proyecto del otro equipo (`Marcelo767n/robotica-IMT342`, rama `proyectoHito3y4`, carpeta `2-2026/IMT342/equipo/proyecto`). El grupo reportó movimiento del brazo por RWS desde la PC el 19/11.
  - Módulo RAPID `rapid/VEVAS_RWS.mod`, rutina `VevasServidor`, adaptado de su `LabJointRws` V3.
  - La PC escribe el objetivo articular en `vvPcTarget` y luego sube `vvPcSeq`.
  - RAPID valida (límites con margen de 2°, paso de 45° como máximo, modo) y ejecuta `MoveAbsJ`. `vPc` fija 20 mm/s de traslación del TCP y 5°/s de reorientación. La velocidad de cada junta requiere validación propia.
  - RAPID responde por `vvRbSeq`/`vvRbCodigo`: 2 llegó, 1 moviendo, 0 listo, −1 vacío, −2 límites, −3 paso grande, −4 modo.

## Archivos clave
| Archivo | Qué es |
|---|---|
| `iniciar_real.sh` | Lanzador RWS. `--solo-chequeo` consulta conectividad y estado. `--solo-lectura` abre la visualización VEVAS/RViz. El GO automático de la sesión del 19/11 se retiró. |
| `rapid/VEVAS_RWS.mod` | Servidor RAPID. `permitirAuto := TRUE`. `confirmarEnPendant := TRUE`: pide «Iniciar» en el FlexPendant (30 s) cada vez que arranca. |
| `vevas/rws_cliente.py` | Cliente RWS con solo la biblioteca estándar: estado, instalar módulo, PP, start/stop, `enviar_objetivo`, mastership, RMMP y candado entre escritores. |
| `vevas/backend_rws.py` | Backend de VEVAS (`tipo="rws"`). Lee las juntas a una frecuencia configurada. El backend bloquea el envío real mientras la velocidad articular siga sin verificar. |
| `vevas/consola.py` | Con `seguimiento_por_llegada` termina por llegada, no por error temporal. El jog continuo no funciona por RWS. |
| `consola_vevas.py` | `--modo sim\|ros\|rws`. Para mover el robot real exige `--real --go`. Sin eso, `--modo rws` es solo lectura. |
| `herramientas/nodo_rws_ros.py` | El nodo acompañante publica `/joint_states` en solo lectura. RViz visualiza la postura sin ofrecer `FollowJointTrajectory` ni arrancar MoveIt para ejecución. |
| `herramientas/rws_irc5.py` | CLI: `estado`, `instalar`, `arrancar`, `servidor`, `fin`, `parar`, `pp-main`, `comparar`. |
| `herramientas/crear_go.py` | Registra GO manual y confirmaciones del entorno en el archivo del ensayo. Las confirmaciones del `config.json` compartido quedan en falso. |
| `vevas/revision.py` | Revisión previa al envío: inicio, límites, velocidad por junta, singularidades, altura, autocolisión (cápsulas), mesa/cajas, herramienta. |
| `iniciar_real_egm.sh`, `herramientas/rws_irc5_egm.py`, `EGM_ROS.mod` | Ruta EGM anterior, solo como legado. |
| `tests/` | Pruebas de revisión, RWS e IK en vivo. `mock_rws.py` simula el IRC5. |

## Observaciones reportadas del laboratorio el 19/11
- Hay ping, y RWS responde lecturas y escrituras.
- El modo se lee bien, AUTO incluido.
- Postura inicial leída: `[0, 0, 0, −0.6, 41.6, 0]` grados.
- El módulo `EGM_ROS` estaba cargado.
- Se mueve el PP por RWS con `POST /rw/rapid/tasks/T_ROB1/pcp?action=set-pp-routine` (HTTP 204).
- Particularidades de RW6:
  - Los módulos se listan en `/rw/rapid/modules?task=T_ROB1`; `/tasks/T_ROB1/modules` da 404.
  - El campo del módulo en `pcp` viene escrito «modulemame».

## Estado tras la revisión de seguridad

La auditoría de `312cd96` encontró coberturas parciales en C01–C04. Se está corrigiendo la validación y vigencia del GO (C01), el límite de velocidad por junta (C02), la cancelación al desarmar o parar (C03) y la exigencia de una postura medida reciente (C04). Queda pendiente comprobar su comportamiento en el controlador. Hasta contar con una implementación válida del límite por junta, el envío real permanece bloqueado.

En AUTO, mantener supervisión en el paro de emergencia. La cancelación en la consola evita el envío de waypoints pendientes. La reacción del brazo durante un `MoveAbsJ` en curso requiere validación física.

## Mesa del laboratorio

Marco exterior de 80 × 80 cm, perfil de 5 × 5 cm, claro de 29,5 cm en cada lateral, 44,5 cm del lado opuesto a los conectores y 14 cm del lado de los conectores. El canto superior del marco está a 19 cm del plano de los tornillos y el tope del cilindro a 18 cm del mismo plano. La diferencia es de 1 cm. Los valores anteriores de 37 y 39 cm quedan descartados. Confirmar en sitio que el modelo coincide con la base y registrar los obstáculos y la herramienta en el GO.

## Cómo seguir

1. Validar en simulación el arrastre y los anillos de rotación incorporados en la revisión. El arrastre con envío y la rotación del TCP siguen pendientes de validación física.
2. Seguir la [guía de RViz](../06_operacion/RVIZ_RWS.md) con posturas sintéticas y luego con RWS en solo lectura. Recuperar los logs de la sesión original para diagnosticar la pantalla vacía.
3. Validar la escena medida, herramienta y obstáculos. El GO manual guarda las confirmaciones del ensayo. `confirmarEnPendant := TRUE` sigue vigente hasta que el docente decida si cambia.
4. Comparar las juntas del FlexPendant con RWS y el modelo mediante la verificación guiada en lectura. Revisar en SIM la transición a cero y la suavidad. Registrar muestras y frecuencia efectiva antes de evaluar velocidad medida o seguimiento con el brazo.
5. Completar el informe del Hito 3/4 con los logs originales, registros de movimientos y video que se puedan recuperar o producir en el laboratorio.

## Hallazgos del 19/11 (tarde) en el robot real
- 1er error (línea 26): nombres globales repetidos con `LabJointRws` (cargado en T_ROB1 por otro grupo). Solución: prefijo `vv`/`Vevas`, constantes `LOCAL`, rutina `VevasServidor`.
- 2do error (línea 47): **RWS corta el texto del módulo en 5000 caracteres** (`set-module-text`). El módulo quedó sin `Validar`. Solución: `VEVAS_RWS.mod` < 4800 caracteres (comentarios mínimos); el cliente rechaza módulos más largos.
- Herramientas de diagnóstico: `python3 herramientas/rws_irc5.py diagnostico` (registro de eventos) y `texto` (módulo en el controlador vs. archivo local).

## Integración Hito 2 en la web (19/11, noche)
- El grupo reportó movimiento del brazo desde la PC a través de RWS. La consola incorporó `--modo rws` y el cliente de terminal incorporó `mover`.
- Nuevo en la web (puerto 8000, la que abre `iniciar_real.sh`):
  - Flechas X/Y/Z sobre el TCP en el 3D (`web/arrastre.js`, `web/escena.js`): al arrastrar llama a `/api/ik_vivo` (Pieper 8 ramas, rama más cercana, verificación FK e_p/e_R, σmin del Jacobiano, tipo de singularidad, «admisible»). El fantasma sigue la solución; rojo si no es admisible. Al soltar: «Calcular y verificar» → plan LSPB + revisión → Enviar (nunca envía solo).
  - Panel «Cinemática inversa en vivo» en Control; badge de singularidad (ok/cerca/singular) bajo el 3D.
  - «Volver a cero» y «Postura inicial» (`/api/calcular_home`): plan revisado; en «cero» la singularidad del destino es aviso (movimiento articular). No recalibra el controlador.
  - Revisión rechaza trayectorias que pasan por o cerca de singularidades (σmin < 0.02; «cerca» < 0.04) y muestra el instante.
- Pruebas registradas el 19/11: `tests/test_revision.py` (16), `tests/test_rws.py` (29), `tests/test_ik_vivo.py` (7).
- El arrastre del 19/11 tenía flechas de traslación. La revisión posterior agregó los anillos de rotación. Su validación física sigue pendiente.
