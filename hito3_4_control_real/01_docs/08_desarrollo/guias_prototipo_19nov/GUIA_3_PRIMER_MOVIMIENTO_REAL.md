# Guía 3 · Primer movimiento real (J1 ±5°)

> **Guía histórica.** Empezá por la [prueba de interfaz del 20/11](../../06_operacion/PRUEBA_INTERFAZ_20NOV.md). Usá simulación y RWS en solo lectura. El envío real permanece bloqueado por C02 hasta verificar el límite de velocidad de cada junta. Las instrucciones históricas de esta guía quedan archivadas y el control sigue bloqueado.

Hacelo SIEMPRE con una persona al lado del FlexPendant con la mano en el paro de emergencia.

## 0. Antes de tocar nada
1. Describí el entorno en `config.json`, sección `entorno`:
   - `cajas`: cada objeto fijo como `{"nombre": "...", "min_mm": [x,y,z], "max_mm": [x,y,z]}` (ejes de la base del robot, en mm). Si no hay nada: `"sin_objetos_externos_confirmado": true`.
   - `mesa_z_m`: altura de la mesa respecto a la base (0.0 si la base está sobre la mesa).
   - Herramienta en la brida: `herramienta.radio_mm` y `confirmada: true`, o `"sin_herramienta_confirmada": true` si no hay.
   Si queda algo «NO VERIFICADO», la consola NO deja enviar al robot real.
2. Copiá `GO_VEVAS_plantilla.json` a `GO_VEVAS.json` y completalo (fecha de hoy, nombres, checklist en `true`, `velocidad_max_pct` chica, 5).

## 1. Ensayo en simulación
```bash
python3 consola_vevas.py --modo sim --abrir
```
Calculá el mismo movimiento (flechas ▶ de J1) y mirá la «Revisión previa».

## 2. Solo lectura con el robot real (lo que ya probaste)
Con el puente `puente_rws_ros.py` corriendo:
```bash
source /opt/ros/jazzy/setup.bash
python3 consola_vevas.py --modo ros --solo-lectura --abrir
```

## 3. Movimiento real
Solo cuando el driver de comando (EGM / ros2_control) esté conectado y probado por el docente:
```bash
python3 consola_vevas.py --modo ros --real --go GO_VEVAS.json --abrir
```
En la interfaz: marcar el checklist, escribir la frase `MOVER ROBOT REAL`, «Armar», calcular con las flechas, revisar que diga APROBADO/AVISO sin NO VERIFICADO, y «Enviar». El botón PARAR (Esc) cancela.

## Qué revisa la consola antes de enviar
Inicio coincide con la posición medida, límites articulares, velocidad por junta (tope = mínimo entre config y GO), singularidades, altura mínima, autocolisión (cápsulas, aproximada), mesa/cajas y herramienta. Si algo falla se rechaza y se puede ver la pose y el instante de la falla. Es una ayuda, no una certificación de seguridad.

## Arranque con un solo comando: `iniciar_real.sh`
```bash
cd ~/Descargas/ROBOTICA/vevas_interfaz_prototipo_2026-11-19
chmod +x iniciar_real.sh
./iniciar_real.sh --solo-chequeo      # ping + RWS + GO, sin escribir nada en el controlador
./iniciar_real.sh                     # espera AUTO y arranca todo solo; reintenta ante cada problema
./iniciar_real.sh --manual            # llave en MANR: vos pulsás Start en el FlexPendant
```
Hace, en orden: ROS/workspace → ping → estado RWS (modo, motores, juntas, eje 5) → validación del GO → confirmación escrita →
limpieza de procesos viejos y UDP 6511 → carga de `EGM_ROS` si falta y puntero en `ros_egm` → (AUTO: motores ON + Start) →
driver ABB → espera controlador activo y acción FollowJointTrajectory → compara `/joint_states` con el IRC5 (≤ 1°) →
abre RViz+MoveIt y la consola VEVAS. Ctrl+C cierra todo (AUTO: Stop de RAPID, motores OFF, puntero a main).

Opciones: `--ip`, `--go`, `--recargar-modulo` (reemplaza EGM_ROS por la v2 local), `--sin-rviz`, `--sin-vevas`, `--ws`.

Notas:
- El script NO gira la llave a AUTO ni toca vallas/paros. Si pedís `--auto` y el controlador está en MANR, se detiene.
- En AUTO no hay enabling device: solo el paro de emergencia. Pedí autorización del docente.
- Si el módulo EGM_ROS ya está en el controlador no se toca. La v1 cargada tiene el retraso B-017; la v2 local
  (`--recargar-modulo`) no se probó en un controlador y quita `ros_egm_verificar`: necesita autorización.
- Los endpoints de carga de módulo y de «PP to routine» por RWS se probaron solo contra un IRC5 simulado: el script prueba
  varias variantes y verifica leyendo de vuelta; si ninguna funciona, te dice qué hacer por FlexPendant.

## 3b. Movimiento real por RWS (sin EGM ni ROS): la ruta vigente
Adaptada de `LabJointRws` del repo robotica-IMT342 (rama proyectoHito3y4). Un módulo RAPID propio (`rapid/VEVAS_RWS.mod`, rutina `VevasServidor`) recibe objetivos articulares por RWS, los valida (límites con margen, paso máximo, modo) y los ejecuta con `MoveAbsJ`.
```bash
./iniciar_real.sh            # AUTO: instala el módulo, arranca VevasServidor y abre VEVAS (--modo rws)
python3 consola_vevas.py --modo rws --real --go GO_VEVAS.json --preparar-rapid   # lo mismo, sin el lanzador
```
- Con el `.mod` por defecto, el FlexPendant pide «Iniciar» (30 s) al arrancar VevasServidor. Con `confirmarEnPendant := FALSE` no hace falta (solo con autorización del responsable).
- La velocidad es fija en el RAPID (20 mm/s de TCP, 5 °/s): el % de VEVAS solo cambia la duración del plan, no la del robot. VEVAS termina cada movimiento cuando el robot informa la llegada.
- PARAR / Ctrl+C solo evitan los waypoints siguientes; el freno real es el paro de emergencia. No hay jog con flechas en este modo.
- Usá UNA herramienta a la vez (VEVAS o MoveIt): hay un candado de archivo, pero la revisión previa solo la hace VEVAS.
