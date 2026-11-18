# Mensaje para pegarle a Claude Code en el Ubuntu

Abrir Claude Code en `~/abb_ws` (`cd ~/abb_ws && claude`), pulsar Shift+Tab hasta salir del auto mode y pegar el bloque de abajo. Aprobar los permisos de crear archivos, git y Python; rechazar cualquier cosa que toque el robot o la red 192.168.125.x.

```
Contexto: curso IMT-342 (Grupo VEVAS). Mi workspace de ROS 2 Jazzy es ~/abb_ws (paquete abb_irb120_support en ~/abb_ws/src/abb_irb120, driver abb_ros2).
El repo del grupo es https://github.com/FlorenciaFrigerio/pfi-vevas-irb120 y la carpeta nueva del Hito 3/4 se llama hito3_4.

REGLAS: todo en simulación (use_fake_hardware:=true). No te conectes a 192.168.125.1, no llames servicios de escritura de rws_client, no arranques EGM ni muevas el robot. Pregúntame antes de cualquier sudo. Documenta cada prueba en hito3_4/docs/bitacora.md con el formato de ese archivo.

PASOS, mostrándome el resultado de cada uno:
1) Si no está, clona el repo en ~/abb_ws/src/pfi-vevas-irb120. Si ya está, git pull. Si te paso un zip hito3_4.zip (en ~/Descargas), descomprímelo en ~/abb_ws/src/pfi-vevas-irb120/hito3_4.
2) cd ~/abb_ws/src/pfi-vevas-irb120/hito3_4 y corre: python3 -m pytest -q tests/  (deben pasar 18) y python3 herramientas/validacion_offline.py.
3) Lee docs/GUIA_5_CONSOLA.md. Dime el nombre exacto del xacro de abb_irb120_support y si su ros2_control tiene rws_ip, rws_port, egm_port 6511 y juntas joint_1..joint_6 con interfaces position y velocity (comando) y position y velocity (estado).
4) Lanza el driver con use_fake_hardware:=true (comando de la Guía 5, sección 6, con el xacro correcto) y comprueba con ros2 control list_hardware_components y list_controllers que joint_state_broadcaster y joint_trajectory_controller estén active.
5) En otra terminal: python3 consola_vevas.py --modo ros. Comprueba con curl que http://127.0.0.1:8000/api/estado muestra modo "ROS 2 · hardware simulado" y q_med_deg con 6 valores.
6) Prueba por API (sin navegador): POST /api/calcular_articular {"q_deg":[0,-20,30,0,40,0]}, POST /api/armar {}, POST /api/ejecutar {}, y mientras se mueve manda POST /api/pulso {} cada 0.25 s hasta que termine. Después POST /api/calcular_pose {"x":350,"y":150,"z":400,"rz":0,"ry":180,"rx":0} y lo mismo. Al final GET /api/estado y muéstrame ultimo_resumen.
7) Corre python3 herramientas/analizar_sesion.py --ultima y muéstrame la tabla de movimientos.
8) Si algo falla en backend_ros.py (nombres de tópicos, acción, QoS, nombres de juntas con prefijo), corrígelo con un cambio mínimo, explícame por qué y agrega una entrada a la bitácora.
9) Agrega en docs/bitacora.md la entrada SIM-ROS-001 con los resultados reales.
10) Commit en una rama nueva feature/hito3_4-consola con mensajes tipo Conventional Commits (feat(consola): ..., docs(hito3_4): ...), push y dime el link para abrir el pull request a dev. No hagas push a main.
Al final, resumen: qué funcionó, qué falló, qué cambiaste y qué falta para el robot real.
```

## Después, con el robot (solo lectura, no mueve nada)

```
Estoy en el laboratorio con el cable en el puerto SERVICE del IRC5. Solo lectura: no muevas el robot ni llames servicios de escritura.
1) Configura la IP 192.168.125.10/24 como dice docs/GUIA_2_CONECTAR_AL_BRAZO.md, paso 2 (pídeme permiso para sudo).
2) Haz los pasos 3 y 4 de esa guía guardando las salidas en hito3_4/docs/datos/gate1/.
3) Corre python3 herramientas/verificar_convencion_rws.py --posturas 3 y espera mi Enter entre posturas (yo muevo el robot con el FlexPendant).
4) Agrega la entrada G1-002 a la bitácora con los resultados, haz commit y push en la rama feature/hito3_4-consola.
```
