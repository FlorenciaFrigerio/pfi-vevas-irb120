# Hito 3/4 — Cambios de la sesión del 19 de noviembre de 2026

Grupo VEVAS, Robótica IMT-342 (UCB Tarija). Robot: ABB IRB 120-3/0.6, controlador IRC5 Compact, RobotWare 6.12.01 (sistema `120-101516`).

## Resumen
- Se movió el brazo real desde la PC.
- La ruta que funcionó fue **RWS**: la PC escribe el objetivo articular en un módulo RAPID propio (`VEVAS_RWS`), que lo ejecuta con `MoveAbsJ`. Es la misma idea que usa el equipo de `Marcelo767n/robotica-IMT342` (rama `proyectoHito3y4`, módulo `LabJointRws`).
- La ruta por **EGM** (abb_ros2) se dejó como legado: nunca conectó, el driver quedaba en «Not connected».
- Se integró todo el Hito 2 en la interfaz web, que ahora calcula en vivo:
  - cinemática inversa (Pieper, 8 ramas);
  - verificación con la cinemática directa;
  - Jacobiano y singularidades;
  - trayectoria LSPB;
  - revisión previa al envío.

## Cómo se usa
```bash
cd hito3_4            # o la carpeta del prototipo
chmod +x iniciar_real.sh
./iniciar_real.sh --solo-chequeo   # solo lectura: ping, RWS, modo
./iniciar_real.sh -y               # llave en AUTO: arranca todo
```
- La interfaz web queda en `http://127.0.0.1:8000`.
- También se puede mover desde la terminal:
  ```bash
  python3 herramientas/rws_irc5.py mover --rel 0 0 0 0 30 0   # relativo (grados)
  python3 herramientas/rws_irc5.py mover 0 -20 30 0 40 0      # absoluto
  python3 herramientas/rws_irc5.py parar                      # detiene RAPID y apaga motores
  ```

## Qué se cambió

### Comunicación con el robot (RWS)
- `rapid/VEVAS_RWS.mod`:
  - Servidor RAPID con la rutina `VevasServidor`, adaptado de `LabJointRws` V3.
  - Protocolo: la PC escribe `vvPcTarget`, lo lee de vuelta y recién después incrementa `vvPcSeq`.
  - RAPID valida límites (margen de 2°), paso máximo de 45° y modo, y responde por `vvRbSeq`/`vvRbCodigo`. Códigos: 2 llegada, 1 moviendo, 0 listo, −1 vacío, −2 límites, −3 paso, −4 modo.
  - Velocidad fija `vPc` = 20 mm/s y 5°/s.
  - `permitirAuto := TRUE`. Con `confirmarEnPendant := TRUE` pide «Iniciar» en el FlexPendant cada vez que arranca.
- `vevas/rws_cliente.py`: cliente RWS (urllib, digest). Hace:
  - lectura de estado y de juntas;
  - carga del módulo (PUT a fileservice + `loadmod`, o `set-module-text`);
  - movimiento del puntero de programa con `set-pp-routine`, y start/stop;
  - envío de objetivos con lectura de vuelta;
  - mastership, RMMP y un candado entre escritores.
- `vevas/backend_rws.py`: backend de la consola. Lee las juntas a 10 Hz y manda la trayectoria en tramos de 10° como máximo. La ejecución termina por llegada, no por tiempo.
- `consola_vevas.py`: agrega `--modo rws` (`--ip`, `--usuario`, `--clave`, `--rws-puerto`, `--preparar-rapid`). Para mover exige `--real --go`.
- `herramientas/rws_irc5.py`: línea de comandos con:
  - `estado`, `instalar`, `arrancar`, `servidor`, `fin`, `parar`, `pp-main`, `comparar`;
  - `mover`: arranca el servidor si hace falta;
  - `diagnostico`: muestra el registro de eventos del IRC5;
  - `texto`: compara el módulo que tiene el controlador con el archivo local.
- `herramientas/nodo_rws_ros.py`: nodo ROS 2 que publica `/joint_states` y sirve la acción `FollowJointTrajectory`, para RViz/MoveIt.
- `iniciar_real.sh`: lanzador de un comando. Si algo falta espera y explica qué hacer. Pasos:
  1. ping;
  2. estado RWS (espera AUTO y la cadena de seguridad);
  3. GO automático;
  4. instalación de `VEVAS_RWS`;
  5. Start;
  6. nodo ROS y RViz;
  7. consola VEVAS.

  Ctrl+C deja RAPID detenido, motores en OFF y el puntero en `main`.
- `herramientas/crear_go.py`: genera `GO_VEVAS.json`. Con `--automatico` lo usa el lanzador con `-y`.

### Revisión previa y seguridad (auditoría C01–C04)
- `vevas/revision.py`: revisa cada trayectoria antes de enviarla. Chequea:
  - inicio;
  - límites;
  - velocidad por junta;
  - singularidades a lo largo del recorrido;
  - altura mínima;
  - autocolisión (modelo de cápsulas, aproximado);
  - mesa y cajas del entorno;
  - herramienta.

  Estados posibles: aprobado / aviso / rechazado / no verificado. Muestra el instante y la pose de la falla.
- `vevas/consola.py`:
  - C01: GO validado de forma estricta y comprobado de nuevo en cada envío.
  - C02: tope de velocidad por junta igual al mínimo entre config y GO.
  - C03: desarmar o parar cancela un envío en curso.
  - C04: con estado viejo no se arma ni se envía.
  - Además, `ik_vivo_rapido`, `calcular_home` y el nivel de singularidad en vivo.
- `vevas/seguridad.py`, `config.json`: secciones nuevas `revision`, `entorno` y `rws`.

### Interfaz web (Hito 2 en vivo)
- **Arrastre en el 3D** (`web/arrastre.js`, `web/escena.js`): flechas X/Y/Z sobre la punta del robot.
  - Mientras se arrastra se llama a `/api/ik_vivo`, que devuelve:
    - Pieper con 8 ramas y la elegida;
    - la verificación FK (e_p, e_R);
    - σmin del Jacobiano;
    - el tipo de singularidad;
    - si la pose es admisible.
  - El fantasma sigue la solución y se pone rojo si no es admisible.
  - Al soltar: «Calcular y verificar» genera el plan LSPB con su revisión, y después se envía con «Enviar». Nunca envía solo.
- **Panel «Cinemática inversa en vivo»** e indicador de singularidad (lejos / cerca / singular).
- **«Volver a cero» y «Postura inicial»** (`/api/calcular_home`): plan revisado. No recalibra el controlador.
- **Flujo y estado:** pipeline de 8 pasos, con el paso «Revisión»; panel de revisión con cinta de severidad. La cabecera y los textos cambian según el modo (simulación, solo lectura, robot real).

### Pruebas
- `tests/test_revision.py`: 16 pruebas (revisión y C01–C04).
- `tests/test_rws.py`: 29 pruebas (IRC5 simulado en `tests/mock_rws.py`).
- `tests/test_ik_vivo.py`: 7 pruebas.

## Problemas encontrados en el robot real y su solución
1. **EGM «Not connected»:** el IRC5 manda el EGM a `192.168.125.10` y la PC tenía `.207` (DHCP). Se abandonó EGM.
2. **Particularidades de RobotWare 6:**
   - los módulos se listan en `/rw/rapid/modules?task=T_ROB1`;
   - el campo del módulo en `pcp` viene con la errata `modulemame`;
   - el puntero se mueve con `pcp?action=set-pp-routine`.
3. **Error en la línea 26:** nombres globales repetidos con `LabJointRws`, que ya está cargado en `T_ROB1`. Se renombró todo con el prefijo `vv`/`Vevas` y se pasaron las constantes a `LOCAL`.
4. **Error en la línea 47:** RWS corta el texto del módulo en **5000 caracteres**, así que `Validar` quedaba fuera. Ahora el `.mod` se mantiene por debajo de 4800 caracteres y el cliente lo controla.
5. **Bucle de «Módulo editado»:** el cliente ya no reescribe el módulo si la lectura de vuelta difiere; muestra el diff.

## Lo que falta
- Probar el **arrastre 3D con el robot real**. Hasta ahora se probó contra simulación y contra el IRC5 simulado.
- **RViz/MoveIt con el robot real:** la pantalla quedó vacía; falta revisar los logs `sesiones/moveit_*.log` y `nodo_rws_*.log`.
- **Rotación en el arrastre:** solo hay traslación (sin anillos Rz/Ry/Rx).
- **Velocidad fija en RAPID** (20 mm/s, 5°/s): el robot frena en cada tramo.
- **Confirmación en el FlexPendant:** decidir con el docente si `confirmarEnPendant` se cambia a `FALSE`.
- **Autocolisión por cápsulas:** es aproximada y no certifica seguridad.
- **Entorno real:** medir y cargar las cajas en `config.json → entorno`.

## Seguridad
- **Freno real:** el paro de emergencia. PARAR en la consola corta los tramos siguientes, pero el `MoveAbsJ` en curso termina igual.
- **En AUTO:** no hay enabling device; tiene que haber una persona con la mano en el paro.
- **Un solo emisor a la vez:** no mandar movimientos desde VEVAS y MoveIt al mismo tiempo.
