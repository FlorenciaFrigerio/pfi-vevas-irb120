# Hito 3/4 — Cambios de la sesión del 19 de noviembre de 2026

Grupo VEVAS, Robótica IMT-342 (UCB Tarija). Robot: ABB IRB 120-3/0.6, controlador IRC5 Compact, RobotWare 6.12.01 (sistema `120-101516`).

## Resumen
- El grupo reportó que movió el brazo real desde la PC el 19/11.
- La ruta reportada fue **RWS**: la PC escribe el objetivo articular en un módulo RAPID propio (`VEVAS_RWS`), que lo ejecuta con `MoveAbsJ`. Es la misma idea que usa el equipo de `Marcelo767n/robotica-IMT342` (rama `proyectoHito3y4`, módulo `LabJointRws`).
- El driver **EGM** (abb_ros2) quedó en «Not connected» en la sesión del 19/11. El chat del 18/11 registra «Connected» con la PC en `.10` y los controladores JSB/JTC activos. Para la sesión del 19/11 el grupo prefirió RWS.
- Se integró todo el Hito 2 en la interfaz web, que ahora calcula en vivo:
  - cinemática inversa (Pieper, 8 ramas);
  - verificación con la cinemática directa;
  - Jacobiano y singularidades;
  - trayectoria LSPB;
  - revisión previa al envío.

## Uso durante la sesión y estado actual

El lanzador del 19/11 abría la consola en `http://127.0.0.1:8000`. El cliente RWS de terminal también formaba parte de esa versión. El arranque automático de GO se retiró durante la revisión posterior. La verificación de conectividad y estado sigue disponible en lectura. El lanzador también ofrece `--solo-lectura` para visualizar postura y RViz. La simulación sigue disponible.

```bash
./iniciar_real.sh --solo-chequeo
```

Para preparar un ensayo, el flujo actual registra el GO manual y la confirmación del entorno en el archivo del ensayo. La configuración compartida conserva las confirmaciones en falso. El envío real permanece bloqueado hasta validar que el controlador respeta el límite de velocidad de cada junta (C02).

## Qué se cambió

### Comunicación con el robot (RWS)
- `05_rapid/VEVAS_RWS.mod`:
  - Servidor RAPID con la rutina `VevasServidor`, adaptado de `LabJointRws` V3.
  - Protocolo: la PC escribe `vvPcTarget`, lo lee de vuelta y recién después incrementa `vvPcSeq`.
  - RAPID valida límites (margen de 2°), paso máximo de 45° y modo, y responde por `vvRbSeq`/`vvRbCodigo`. Códigos: 2 llegada, 1 moviendo, 0 listo, −1 vacío, −2 límites, −3 paso, −4 modo.
  - `vPc` fija 20 mm/s de traslación del TCP y 5°/s de reorientación. El límite de velocidad de cada junta requiere validación propia.
  - `permitirAuto := TRUE`. Con `confirmarEnPendant := TRUE` pide «Iniciar» en el FlexPendant cada vez que arranca.
- `07_vevas/rws_cliente.py`: cliente RWS (urllib, digest). Hace:
  - lectura de estado y de juntas;
  - carga del módulo (PUT a fileservice + `loadmod`, o `set-module-text`);
  - movimiento del puntero de programa con `set-pp-routine`, y start/stop;
  - envío de objetivos con lectura de vuelta;
  - mastership, RMMP y un candado entre escritores.
- `07_vevas/backend_rws.py`: backend de la consola. Lee las juntas con una frecuencia configurable. El diseño del 19/11 segmentaba las trayectorias en tramos de hasta 10° y esperaba confirmación de llegada. El backend actual bloquea el envío real mientras la velocidad articular siga sin verificar.
- `consola_vevas.py`: agrega `--modo rws` (`--ip`, `--usuario`, `--clave`, `--rws-puerto`, `--preparar-rapid`). Para mover exige `--real --go`.
- `02_herramientas/rws_irc5.py`: línea de comandos con:
  - `estado`, `instalar`, `arrancar`, `servidor`, `fin`, `parar`, `pp-main`, `comparar`;
  - `mover`: arranca el servidor si hace falta;
  - `diagnostico`: muestra el registro de eventos del IRC5;
  - `texto`: compara el módulo que tiene el controlador con el archivo local.
- El nodo ROS que acompaña a la consola publica `/joint_states` en solo lectura. El flujo de visualización abre RViz sin exponer la acción `FollowJointTrajectory` ni arrancar ejecución de MoveIt. Ver [RViz con la postura del ABB IRB 120](../06_operacion/05_RVIZ_RWS.md).
- El lanzador del 19/11 preparaba RWS, iniciaba el módulo RAPID y abría el nodo ROS y la consola. El GO automático se retiró. `02_herramientas/crear_go.py` registra el GO manual y la confirmación del entorno en su archivo de ensayo.

### Revisión previa y seguridad (auditoría C01–C04)
- `07_vevas/revision.py`: revisa cada trayectoria antes de enviarla. Chequea:
  - inicio;
  - límites;
  - velocidad por junta;
  - singularidades a lo largo del recorrido;
  - altura mínima;
  - autocolisión (modelo de cápsulas, aproximado);
  - mesa y cajas del entorno;
  - herramienta.

  Estados posibles: aprobado / aviso / rechazado / no verificado. Muestra el instante y la pose de la falla.
- La auditoría sobre `312cd96` encontró coberturas parciales en C01–C04. La revisión de software aborda la vigencia y validación del GO (C01), la velocidad por junta (C02), la cancelación al desarmar o parar (C03) y la frescura de la postura medida (C04). Falta la validación en el controlador. El envío real se mantiene bloqueado hasta contar con una implementación válida de C02.
- `07_vevas/seguridad.py` y `config.json` incorporaron secciones para revisión, entorno y RWS.

### Interfaz web (Hito 2 en vivo)
- **Arrastre en el 3D** (`08_web/arrastre.js`, `08_web/escena.js`): flechas X/Y/Z sobre la punta del robot.
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

### Pruebas registradas al 19/11
- `06_tests/test_revision.py`: 16 pruebas (revisión y C01–C04).
- `06_tests/test_rws.py`: 29 pruebas (IRC5 simulado en `06_tests/mock_rws.py`).
- `06_tests/test_ik_vivo.py`: 7 pruebas.

## Problemas encontrados en el robot real y su solución
1. **EGM el 19/11.** El IRC5 enviaba a `192.168.125.10` y la PC estaba en `.207` por DHCP. El driver quedó en «Not connected» durante esa sesión y el grupo continuó por RWS.
2. **Particularidades de RobotWare 6:**
   - los módulos se listan en `/rw/rapid/modules?task=T_ROB1`;
   - el campo del módulo en `pcp` viene con la errata `modulemame`;
   - el puntero se mueve con `pcp?action=set-pp-routine`.
3. **Error en la línea 26:** nombres globales repetidos con `LabJointRws`, que ya está cargado en `T_ROB1`. Se renombró todo con el prefijo `vv`/`Vevas` y se pasaron las constantes a `LOCAL`.
4. **Error en la línea 47:** RWS corta el texto del módulo en **5000 caracteres**, así que `Validar` quedaba fuera. Ahora el `.mod` se mantiene por debajo de 4800 caracteres y el cliente lo controla.
5. **Bucle de «Módulo editado»:** el cliente ya no reescribe el módulo si la lectura de vuelta difiere; muestra el diff.

## Lo que falta

1. Validar en simulación el flujo de arrastre y los anillos de rotación incorporados en la revisión posterior. Su validación con el brazo queda pendiente hasta cerrar C02 y contar con GO manual y entorno medido.
2. Probar RViz con datos sintéticos y luego con lectura RWS en el laboratorio. Recuperar `sesiones/moveit_*.log` y `sesiones/nodo_rws_*.log` para determinar la causa de la pantalla vacía del 19/11.
3. Contrastar el modelo de la mesa con las medidas confirmadas, medir obstáculos y herramienta y registrar la verificación presencial en el GO de la sesión. La autocolisión por cápsulas sigue siendo una aproximación.
4. Conservar la confirmación del FlexPendant mientras el docente decide si cambia. Evaluar la velocidad y las frenadas por tramo después de validar el tope articular.
5. Incorporar al informe los registros originales, el video y las evidencias físicas disponibles.

## Medidas confirmadas de la mesa

Marco exterior de 80 × 80 cm, perfil de 5 × 5 cm, claro de 29,5 cm en cada lateral, 44,5 cm del lado opuesto a los conectores y 14 cm del lado de los conectores. El canto superior del marco está a 19 cm del plano de los tornillos y el tope del cilindro a 18 cm del mismo plano. La diferencia es de 1 cm. Los valores anteriores de 37 y 39 cm quedan descartados.

## Seguridad

- En AUTO, mantener supervisión en el paro de emergencia. La cancelación en la consola evita los tramos pendientes. La reacción del controlador ante un `MoveAbsJ` en curso requiere validación física.
- Durante la visualización, el nodo acompañante se limita a lectura. Mantener un solo emisor de movimiento en cualquier ensayo autorizado.
