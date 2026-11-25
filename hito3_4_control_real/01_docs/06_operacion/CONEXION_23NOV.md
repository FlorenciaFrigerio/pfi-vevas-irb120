# Conexión al ABB IRB 120 el 23/11

La prueba en el laboratorio consiste en leer la postura del IRC5 desde Ubuntu, verla en VEVAS y contrastar la cinemática del modelo con la brida del controlador. El envío físico de VEVAS sigue bloqueado por **C02**, la verificación del límite de velocidad por junta. Las figuras de esta entrega se ensayan en simulación.

## 1. Preparar una copia en Ubuntu

Guardá `VEVAS_SIM_23NOV.zip` en Descargas. Extraelo en una carpeta nueva y prepará ahí el entorno Python. Si Ubuntu usa otro nombre para Descargas, ajustá la ruta del ZIP en el bloque. La carpeta de destino debe estar libre.

```bash
(
set -e
mkdir -p "$HOME/Descargas/ROBOTICA"
cd "$HOME/Descargas/ROBOTICA"
test ! -e vevas_sim_23nov
unzip -n "$HOME/Descargas/VEVAS_SIM_23NOV.zip"
cd vevas_sim_23nov
python3 -m venv .venv_interfaz
.venv_interfaz/bin/python -m pip install -r requirements.txt
)
```

Prepará el entorno antes de ir al laboratorio si la conexión a Internet allí es limitada. Si Ubuntu indica que falta el módulo `venv`, instalá el paquete `python3-venv` de Ubuntu. Cerrá con Ctrl+C cualquier consola VEVAS que ya use el puerto 9012 en esa PC.

## 2. Revisar el cable y leer el controlador

Conectá el Ethernet de Ubuntu al puerto **SERVICE** del IRC5 encendido. Para esta prueba, podés mantener el selector en **Manual** con el brazo detenido. El IRC5 del laboratorio usa `192.168.125.1` en SERVICE. Si el responsable confirma otra dirección, pasala con `--ip` tanto al diagnóstico como a la consola.

```bash
cd "$HOME/Descargas/ROBOTICA/vevas_sim_23nov"
.venv_interfaz/bin/python herramientas/conectar_robot.py --ping --salida conexion-23nov.json
```

El diagnóstico usa `ip -j route get` e `ip -j address` para identificar la interfaz y la IPv4 de origen elegidas por Ubuntu. Muestra la interfaz y la dirección con su prefijo. La ruta del robot físico debe ser directa y la IPv4 de origen debe estar en la misma subred. Si la ruta falta, usa una puerta de enlace o selecciona otra interfaz, revisá el cable y el perfil de red de la conexión Ethernet que va al IRC5 con el responsable del laboratorio. Usá la IPv4 libre y el prefijo confirmados en el laboratorio. El diagnóstico deja la configuración de red intacta.

- El ping es opcional y tiene tiempo de espera. La consulta RWS sigue aunque ICMP no responda.
- El panel informa modo, estado del controlador y estado de RAPID. **MANR**, **guardstop** y motores apagados pueden ser estados válidos para leer. La lectura funciona con el brazo detenido y sin arrancar RAPID.
- RWS muestra seis juntas en grados, la duración de la consulta y el límite de frescura por lectura. En el JSON se guarda la hora UTC en que terminó la lectura, además de los resultados de la comparación.
- Si RWS devuelve 401, confirmá la cuenta y la contraseña con el responsable. Se usa la cuenta del archivo de configuración y se puede proporcionar la contraseña con la variable `VEVAS_RWS_CLAVE`. Si se usa otro archivo, pasalo con `--config` a ambos comandos.
- `--salida` es opcional y guarda un JSON en un archivo nuevo. Para repetir la captura, usá otro nombre. El diagnóstico omite las credenciales y los cuerpos de error del controlador.

La comparación usa **ROB_1 · tool0 · wobj0 · Base**. Lee juntas antes y después de la pose para detectar cambios durante la captura. Compará las juntas con la vista de ejes del FlexPendant. La FK usa los signos y offsets configurados y reporta error XYZ y error angular. Una captura válida tiene código de salida `0` si coincide y `1` si difiere. El código `2` indica que no se pudo completar la comparación o guardar el JSON pedido. Si las juntas cambian durante la captura, repetila con el brazo detenido. [Comparar el FlexPendant con VEVAS](COMPARAR_FLEXPENDANT.md)

## 3. Abrir VEVAS en solo lectura

En la misma Ubuntu, desde la copia nueva, ejecutá el servidor web local.

```bash
.venv_interfaz/bin/python consola_vevas.py --modo rws --solo-lectura --puerto 9012
```

Abrí `http://127.0.0.1:9012` en esa PC y comprobá la indicación **IRC5 vía RWS · ROBOT REAL · SOLO LECTURA**. Compará **Datos en vivo** y el modelo 3D con la postura que muestra el FlexPendant. El asistente de recalibración de la web toma la misma clase de captura para comparar la FK. El diagnóstico y la visualización usan consultas de lectura en el IRC5. La consola web funciona sin ROS, RViz ni MoveIt.

Si también necesitás RViz, seguí su guía después de confirmar la lectura web. [RViz con la postura del ABB IRB 120](RVIZ_RWS.md)
