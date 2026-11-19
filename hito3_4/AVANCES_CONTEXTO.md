# VEVAS Studio — avances y contexto (para retomar en otro chat)

Última actualización: 2026-11-19. Grupo VEVAS, Robótica IMT-342 (UCB Tarija), PFI Hitos 3/4.
Carpeta de trabajo: `~/Descargas/ROBOTICA/vevas_interfaz_prototipo_2026-11-19`.

## Robot y red
- ABB IRB 120-3/0.6, IRC5 Compact, RobotWare 6.12.01, sistema `120-101516`.
- IRC5 en `192.168.125.1` (puerto SERVICE). RWS 1.0, digest, usuario `Default User` / `robotics`.
- La PC recibe la IP por DHCP (se vio `192.168.125.207` en `eno2`).
- ROS 2 Jazzy, con el workspace `~/abb_ws` (abb_ros2 compilado).

## Decisión principal: mover por RWS, no por EGM
- El EGM (abb_ros2 + `EGM_ROS.mod`) no llegó a conectar: el driver quedó en «Not connected». El IRC5 manda el EGM a `192.168.125.10` y la PC tenía `.207`.
- Otro equipo del curso (repo `Marcelo767n/robotica-IMT342`, rama `proyectoHito3y4`, carpeta `2-2026/IMT342/equipo/proyecto`) movió el robot real por RWS, también en AUTO.
- Se portó su funcionalidad, no su interfaz:
  - Módulo RAPID `rapid/VEVAS_RWS.mod`, rutina `VevasServidor`, adaptado de su `LabJointRws` V3.
  - La PC escribe el objetivo articular en `vvPcTarget` y luego sube `vvPcSeq`.
  - RAPID valida (límites con margen de 2°, paso de 45° como máximo, modo) y ejecuta `MoveAbsJ` a `vPc` = 20 mm/s y 5°/s.
  - RAPID responde por `vvRbSeq`/`vvRbCodigo`: 2 llegó, 1 moviendo, 0 listo, −1 vacío, −2 límites, −3 paso grande, −4 modo.

## Archivos clave
| Archivo | Qué es |
|---|---|
| `iniciar_real.sh` | Lanzador de un comando. Pasos: ping, estado RWS (espera AUTO, cadena de seguridad, RAPID libre), GO automático (`-y`), instala `VEVAS_RWS`, PP a `VevasServidor`, Start, espera que el servidor esté listo, nodo ROS + RViz/MoveIt, VEVAS `--modo rws`. Si falta algo espera y explica qué hacer. Ctrl+C detiene RAPID, apaga motores y deja el PP en main. |
| `rapid/VEVAS_RWS.mod` | Servidor RAPID. `permitirAuto := TRUE`. `confirmarEnPendant := TRUE`: pide «Iniciar» en el FlexPendant (30 s) cada vez que arranca. |
| `vevas/rws_cliente.py` | Cliente RWS con solo la biblioteca estándar: estado, instalar módulo, PP, start/stop, `enviar_objetivo`, mastership, RMMP y candado entre escritores. |
| `vevas/backend_rws.py` | Backend de VEVAS (`tipo="rws"`). Lee las juntas a 10 Hz y manda la trayectoria en waypoints de 10° como máximo. |
| `vevas/consola.py` | Con `seguimiento_por_llegada` termina por llegada, no por error temporal. El jog continuo no funciona por RWS. |
| `consola_vevas.py` | `--modo sim\|ros\|rws`. Para mover el robot real exige `--real --go`. Sin eso, `--modo rws` es solo lectura. |
| `herramientas/nodo_rws_ros.py` | Nodo ROS 2: publica `/joint_states` y sirve la acción `FollowJointTrajectory` (la usa MoveIt). |
| `herramientas/rws_irc5.py` | CLI: `estado`, `instalar`, `arrancar`, `servidor`, `fin`, `parar`, `pp-main`, `comparar`. |
| `herramientas/crear_go.py` | Genera `GO_VEVAS.json`. `--automatico` lo usa el lanzador con `-y`. |
| `vevas/revision.py` | Revisión previa al envío: inicio, límites, velocidad por junta, singularidades, altura, autocolisión (cápsulas), mesa/cajas, herramienta. |
| `iniciar_real_egm.sh`, `herramientas/rws_irc5_egm.py`, `EGM_ROS.mod` | Ruta EGM anterior, solo como legado. |
| `tests/` | `test_revision.py` (16 pruebas), `test_rws.py` (27 pruebas), `mock_rws.py` (IRC5 falso). |

## Lo que ya se confirmó en el robot real (19/11)
- Hay ping, y RWS responde lecturas y escrituras.
- El modo se lee bien, AUTO incluido.
- Postura inicial leída: `[0, 0, 0, −0.6, 41.6, 0]` grados.
- El módulo `EGM_ROS` estaba cargado.
- Se mueve el PP por RWS con `POST /rw/rapid/tasks/T_ROB1/pcp?action=set-pp-routine` (HTTP 204).
- Particularidades de RW6:
  - Los módulos se listan en `/rw/rapid/modules?task=T_ROB1`; `/tasks/T_ROB1/modules` da 404.
  - El campo del módulo en `pcp` viene escrito «modulemame».

## Pendiente y sin verificar
- **La ruta RWS nunca se probó en el robot real.** Solo contra el IRC5 falso.
- Puntos dudosos en el IRC5 real:
  - Carga del módulo (`PUT /fileservice/$HOME/...` + `loadmod`).
  - `set-module-text`.
  - El nodo ROS y MoveIt con ROS real.
- **`confirmarEnPendant`**: hoy pide «Iniciar» en el FlexPendant cada vez que arranca `VevasServidor`. Para un arranque sin tocar el pendant hay que pasarlo a `FALSE` en `rapid/VEVAS_RWS.mod` y correr `./iniciar_real.sh --recargar-modulo`. Lo decide el grupo con el responsable.
- **`config.json` → `entorno`**: VEVAS bloquea el envío real si los objetos o la herramienta quedan sin confirmar. `crear_go.py --automatico` (que corre con `-y`) los marca al confirmar.
- **Velocidad**: fija en RAPID (20 mm/s, 5°/s). El robot frena en cada waypoint.
- **Seguridad**: VEVAS y MoveIt no deben mandar a la vez. Hay un candado de archivo, pero lo correcto es usar una sola herramienta por vez. MoveIt no pasa por la revisión de VEVAS.
- **Parar**: la parada física es el paro de emergencia. PARAR en VEVAS solo corta los waypoints siguientes, no el `MoveAbsJ` que esté en curso.

## Cómo seguir
```bash
cd ~/Descargas/ROBOTICA/vevas_interfaz_prototipo_2026-11-19
chmod +x iniciar_real.sh
./iniciar_real.sh --solo-chequeo   # lectura, no escribe nada
./iniciar_real.sh -y               # todo (llave en AUTO)
```
Si algo falla, pegar la salida desde el primer ⏳ o ✗.

## Hallazgos del 19/11 (tarde) en el robot real
- 1er error (línea 26): nombres globales repetidos con `LabJointRws` (cargado en T_ROB1 por otro grupo). Solución: prefijo `vv`/`Vevas`, constantes `LOCAL`, rutina `VevasServidor`.
- 2do error (línea 47): **RWS corta el texto del módulo en 5000 caracteres** (`set-module-text`). El módulo quedó sin `Validar`. Solución: `VEVAS_RWS.mod` < 4800 caracteres (comentarios mínimos); el cliente rechaza módulos más largos.
- Herramientas de diagnóstico: `python3 herramientas/rws_irc5.py diagnostico` (registro de eventos) y `texto` (módulo en el controlador vs. archivo local).

## Integración Hito 2 en la web (19/11, noche)
- Mover desde la PC ya funciona por RWS (`herramientas/rws_irc5.py mover`, y VEVAS `--modo rws`).
- Nuevo en la web (puerto 8000, la que abre `iniciar_real.sh`):
  - Flechas X/Y/Z sobre el TCP en el 3D (`web/arrastre.js`, `web/escena.js`): al arrastrar llama a `/api/ik_vivo` (Pieper 8 ramas, rama más cercana, verificación FK e_p/e_R, σmin del Jacobiano, tipo de singularidad, «admisible»). El fantasma sigue la solución; rojo si no es admisible. Al soltar: «Calcular y verificar» → plan LSPB + revisión → Enviar (nunca envía solo).
  - Panel «Cinemática inversa en vivo» en Control; badge de singularidad (ok/cerca/singular) bajo el 3D.
  - «Volver a cero» y «Postura inicial» (`/api/calcular_home`): plan revisado; en «cero» la singularidad del destino es aviso (movimiento articular). No recalibra el controlador.
  - Revisión rechaza trayectorias que pasan por o cerca de singularidades (σmin < 0.02; «cerca» < 0.04) y muestra el instante.
- Pruebas: `tests/test_revision.py` (16), `tests/test_rws.py` (29), `tests/test_ik_vivo.py` (7).
- Pendiente: anillos de rotación en el arrastre (solo traslación), probar el arrastre con el robot real.
