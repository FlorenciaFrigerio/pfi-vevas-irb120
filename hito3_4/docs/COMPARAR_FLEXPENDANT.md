# Comparar el FlexPendant con VEVAS

Con el brazo detenido, compará las juntas y la pose que muestra el FlexPendant con una captura del IRC5. En la pantalla de movimiento manual elegí la unidad mecánica **ROB_1**. Para la pose cartesiana, seleccioná la brida **tool0**, el objeto **wobj0** y las coordenadas **Base**. Anotá los seis ejes en grados y, si la pantalla permite elegir el formato, la posición XYZ en milímetros y el cuaternión ABB en orden `q1, q2, q3, q4`, equivalente a `w, x, y, z`.

En VEVAS conectado por RWS, abrí el asistente de recalibración y tomá la captura con el brazo detenido. La misma comprobación está disponible desde la terminal de Ubuntu.

```bash
cd hito3_4
python3 herramientas/comparar_pendant.py --ip 192.168.125.1
python3 herramientas/comparar_pendant.py --ip 192.168.125.1 --json
```

El comando usa las credenciales y la convención articular de `config.json`. Para otra instalación, podés indicar `--config`, `--usuario` o `--rws-puerto`, y pasar la contraseña mediante `VEVAS_RWS_CLAVE`. El código de salida es `0` si la pose coincide dentro de la tolerancia, `1` si la captura es válida y difiere, y `2` si no se pudo comparar.

## Interpretar la captura

- VEVAS pide **juntas → pose de tool0 → juntas** por RWS. Si un eje cambia 0,01° o más entre las dos lecturas, repetí la prueba con el brazo detenido. Las juntas mostradas son el promedio de ambas lecturas.
- Compará **J1–J6 del IRC5** con la pantalla de juntas. La FK usa las **juntas del modelo**, calculadas con los signos y offsets de la configuración.
- Compará la pose del IRC5 y de VEVAS en el mismo marco. Los umbrales propuestos para esta revisión son 0,5 mm de error XYZ y 0,05° de error angular. La orientación se compara por el giro relativo entre ambas rotaciones. Los cuaterniones `q` y `−q` representan la misma orientación.
- Según la secuencia Euler, una orientación puede escribirse con ternas distintas. **ZYX** muestra los ángulos en orden Z, Y, X y **ZXY** en orden Z, X, Y. Cerca del bloqueo de cardán, dos de esos ángulos dejan de poder distinguirse por separado. Usá el cuaternión o el error angular para comparar la orientación de la brida.
