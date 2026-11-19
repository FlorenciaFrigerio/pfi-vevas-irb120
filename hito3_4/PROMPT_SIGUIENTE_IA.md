# Prompt para continuar con otra IA

Copiá todo lo que está debajo de la línea y pegalo como primer mensaje.

---

Sos mi asistente en el proyecto **VEVAS Studio** del grupo VEVAS, para Robótica IMT-342 (UCB Tarija), PFI Hitos 3/4. Hablame en español informal. Dame comandos completos y listos para copiar, y entregables directos.

## Contexto del robot y la PC
- **Robot:** ABB IRB 120-3/0.6 con IRC5 Compact y RobotWare 6.12.01 (sistema `120-101516`). El IRC5 está en `192.168.125.1`, puerto SERVICE.
- **RWS:** versión 1.0, autenticación digest, `Default User` / `robotics`.
- **PC:** Ubuntu con ROS 2 Jazzy. El workspace `~/abb_ws` tiene abb_ros2 compilado. La IP la da el DHCP del IRC5 (vista `192.168.125.207` en `eno2`).
- **Carpeta de trabajo:** `~/Descargas/ROBOTICA/vevas_interfaz_prototipo_2026-11-19`.
- **Repo:** https://github.com/FlorenciaFrigerio/pfi-vevas-irb120. Va en la carpeta `hito3_4/`, rama `19NOV`, que después se fusiona en `main`.

## Lo que ya funciona (19/11/2026)
- **Mover el robot real desde la PC por RWS.**
  - El módulo RAPID `rapid/VEVAS_RWS.mod` (rutina `VevasServidor`) recibe objetivos articulares en `vvPcTarget`/`vvPcSeq` y responde por `vvRbSeq`/`vvRbCodigo`. Ejecuta con `MoveAbsJ` a velocidad fija (20 mm/s, 5°/s).
  - Está adaptado de `LabJointRws` (repo `Marcelo767n/robotica-IMT342`, rama `proyectoHito3y4`).
- **Lanzador de un comando: `./iniciar_real.sh -y`** (con la llave en AUTO).
  - Pasos: ping → estado RWS → GO automático → instala el módulo → Start → nodo ROS y RViz → consola web en `http://127.0.0.1:8000`.
  - Al arrancar, el FlexPendant pide «Iniciar» (`confirmarEnPendant := TRUE`).
  - Ctrl+C detiene RAPID, apaga motores y pone el PP en `main`.
- **Línea de comandos `herramientas/rws_irc5.py`:**
  - `mover` (absoluto o `--rel`, en tramos de 10°; si el servidor está detenido lo arranca);
  - `estado`, `parar`, `diagnostico` (registro de eventos del IRC5), `texto` (diff del módulo en el controlador contra el archivo local).
- **Consola web (VEVAS)** con `--modo sim|ros|rws`:
  - arrastre con flechas X/Y/Z sobre la punta en el 3D, con IK de Pieper en vivo (8 ramas), verificación FK (e_p, e_R), σmin del Jacobiano, tipo de singularidad y si la pose es admisible;
  - «Calcular y verificar» genera la trayectoria LSPB y la revisión previa; después viene «Enviar»; nunca envía solo;
  - «Volver a cero» y «Postura inicial»;
  - indicador de singularidad en vivo;
  - gráficas de q, q̇ y q̈;
  - pestañas Jacobiano, FK y 8 ramas.
- **Revisión previa (`vevas/revision.py`):**
  - chequea inicio, límites, velocidad por junta, singularidades en todo el recorrido (rechaza si σmin < 0.02; «cerca» si σmin < 0.04), altura mínima, autocolisión por cápsulas, mesa/cajas y herramienta;
  - muestra el instante y la pose donde falla.
- **Seguridad:** C01–C04 cerrados (GO estricto, tope de velocidad por junta, cancelar al desarmar o parar, estado viejo), en `vevas/consola.py`.
- **Pruebas:** `python3 tests/test_revision.py` (16) y `python3 -m pytest -q tests/test_rws.py tests/test_ik_vivo.py` (29 + 7). El IRC5 simulado está en `tests/mock_rws.py`.

## Leé primero, en este orden
1. `CAMBIOS_19NOV.md`: qué se cambió, los problemas del robot real y lo que falta.
2. `AVANCES_CONTEXTO.md`: estado técnico y tabla de archivos.
3. `GUIA_3_PRIMER_MOVIMIENTO_REAL.md`.
4. Código, según la tarea:
   - backend: `vevas/rws_cliente.py`, `vevas/backend_rws.py`, `vevas/consola.py`, `vevas/revision.py`, `vevas/servidor.py`;
   - web: `web/app.js`, `web/arrastre.js`, `web/escena.js`;
   - RAPID: `rapid/VEVAS_RWS.mod`;
   - lanzador: `iniciar_real.sh`.

## Lecciones que NO hay que repetir
- **RWS corta el texto de un módulo RAPID en 5000 caracteres.** `VEVAS_RWS.mod` tiene que quedar por debajo de 4800, con comentarios mínimos. El cliente lo controla.
- **`LabJointRws` de otro grupo está cargado en T_ROB1.** No uses sus nombres globales (`pcTarget`, `ServidorPC`, `maxPasoDeg`…). Usá el prefijo `vv`/`Vevas` y constantes `LOCAL`.
- **Particularidades de RobotWare 6:**
  - los módulos se listan en `/rw/rapid/modules?task=T_ROB1`;
  - el campo del módulo en `pcp` viene con la errata `modulemame`;
  - el PP se mueve con `POST /rw/rapid/tasks/T_ROB1/pcp?action=set-pp-routine` (`module`, `routine`, `userlevel=false`).
- **EGM (abb_ros2) no conectó:** el IRC5 manda a `.10` y la PC tenía `.207`. Quedó como legado en `iniciar_real_egm.sh`; la ruta principal es RWS.
- **Dentro del controlador no se modifica nada que sea de otros grupos.**

## Lo que falta (prioridad)
1. Probar el arrastre 3D y «Enviar» desde la web con el robot real, con movimientos chicos.
2. RViz/MoveIt con el robot real aparece vacío. Revisar `sesiones/moveit_*.log` y `sesiones/nodo_rws_*.log`, y el nodo `herramientas/nodo_rws_ros.py` (`/joint_states` + `FollowJointTrajectory`).
3. Agregar anillos de rotación (Rz/Ry/Rx) al arrastre; hoy solo hay traslación.
4. Velocidad fija en RAPID (20 mm/s, 5°/s), con frenadas en cada tramo. Evaluar una velocidad parametrizable acotada o tramos más largos.
5. Decidir con el docente si `confirmarEnPendant` pasa a `FALSE`, para arrancar sin tocar el FlexPendant.
6. Medir el entorno real y cargar las cajas en `config.json → entorno`.
7. Documentación del Hito 3/4: informe, video y evidencias de los movimientos reales.

## Reglas de seguridad
- El freno real es el paro de emergencia. En AUTO no hay enabling device, así que siempre tiene que haber alguien con la mano en el paro.
- Nunca enviar movimientos sin revisión previa. No sacar las compuertas (GO, revisión, armado) sin que yo lo pida explícitamente y con autorización del docente.
- No mandar desde VEVAS y MoveIt al mismo tiempo.
