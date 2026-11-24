# Probar la interfaz y preparar la comparación física

Estado del 20 de noviembre de 2026. Esta continuación se prepara en la rama
`codex/continuacion-19nov`. No instala módulos en el IRC5 durante sus pruebas de software.

## 1. Abrir la copia que contiene estos cambios

Desde una terminal de Ubuntu, situarse en `hito3_4` de **esta rama**. Una copia
antigua en Descargas no recibe los cambios automáticamente.

```bash
python3 -m venv .venv_interfaz
.venv_interfaz/bin/python -m pip install -r requirements.txt
.venv_interfaz/bin/python consola_vevas.py --modo sim --puerto 9012 --abrir
```

Abrir `http://127.0.0.1:9012`. Dejar la terminal abierta; `Ctrl+C` cierra el
servidor. Si el puerto está ocupado, elegir otro con `--puerto`.

## 2. Ensayo en simulación

1. Comprobar el rótulo **SIM**. Seleccionar J1 en el visor y cambiar su objetivo
   con las flechas. El robot medido y el objetivo deben distinguirse.
2. Pulsar **Calcular**. Revisar los límites, la singularidad, la altura,
   autocolisión y colisión con las cuatro barras del marco. El modelo de
   colisiones usa cápsulas y cajas aproximadas; no certifica el espacio físico.
3. Probar una pose fuera de alcance o singular. Revisar el motivo, el punto del
   recorrido señalado y su reproducción. Una trayectoria rechazada no se envía.
4. Cambiar el objetivo después de calcular: debe exigir un cálculo nuevo antes
   de enviar. El envío identifica el plan revisado mediante `plan_id`.
5. Probar rotación y traslación del objetivo, las muestras, **Volver a cero** y
   **Postura inicial**. Los botones de retorno calculan un recorrido; no saltan
   las comprobaciones. Cero articular implica singularidad de muñeca: no es una
   postura universalmente segura ni una calibración.
6. Cambiar velocidad y frecuencia de pantalla. La frecuencia de pantalla no
   modifica el sondeo RWS ni la velocidad del controlador. Mirar las gráficas de
   juntas, TCP y errores; identificar siempre si el dato es medido o planificado.
7. Enviar únicamente en SIM para comprobar la animación y la cancelación.

## 3. Conectar para leer, sin ordenar movimiento

No hay que recalibrar cada vez que se abre la interfaz. Si el FlexPendant indica
calibración y contadores válidos en los seis ejes, se conserva esa calibración.
Si aparece un aviso, corresponde al operador aplicar el procedimiento ABB de
su controlador; no actualizar contadores desde una postura arbitraria.

Comprobar la interfaz Ethernet y la ruta actual; su nombre e IP pueden cambiar:

```bash
nmcli device status
ip -br address
ip route get 192.168.125.1
ping -c 4 -W 2 192.168.125.1
```

La ruta debe salir por el cable conectado al controlador. Una ruta por Wi-Fi
no demuestra conexión al brazo. En el último registro compartido funcionaba
`eno2` con `192.168.125.207/24`; esto no cambia la dirección remota configurada
para una sesión EGM antigua. Aquí se utiliza RWS de lectura.

En `hito3_4`, con la consola SIM cerrada:

```bash
.venv_interfaz/bin/python consola_vevas.py --modo rws --solo-lectura --puerto 9012 --abrir
```

Esta orden lee el estado y permite calcular planes, pero no los envía. Para
abrir además RViz con el ROS 2 de Ubuntu, ver [RVIZ_RWS.md](RVIZ_RWS.md).

## 4. Comparar con el FlexPendant

Usar el apartado de comparación con el robot detenido. La captura toma juntas,
pose y juntas otra vez para detectar movimiento durante la lectura. Compara
**ROB_1 / tool0 / wobj0 / Base** y muestra posición, cuaternión y convenciones
Euler. La diferencia entre dos representaciones Euler no demuestra un error:
la comparación de orientación se hace con matrices/cuaterniones.

Ver [COMPARAR_FLEXPENDANT.md](COMPARAR_FLEXPENDANT.md). Coincidencia con el IRC5
verifica la convención del modelo; no mide la precisión absoluta con un
instrumento externo ni certifica la calibración mecánica.

## 5. Qué falta antes del envío físico

El backend RWS de producción conserva bloqueado el envío: su `MoveAbsJ` usa
`speeddata` fija y no hay una cota por junta verificada que cumpla el porcentaje
del plan. Tampoco sigue el reloj de LSPB. Por eso los errores temporales de
seguimiento que no se han medido se muestran como no disponibles, no como cero.

Falta diseñar y validar en el controlador la regulación de velocidad y la
continuidad de la trayectoria. No basta con quitar `fine`, aumentar muestras o
marcar una variable como verificada. Conservar `confirmarEnPendant := TRUE`
hasta que el responsable acuerde el procedimiento de la sesión.

También faltan medir la herramienta negra montada en la brida y su TCP, los
obstáculos adicionales y las partes del soporte que no están acotadas. El marco
superior mide 80 × 80 cm y su cara superior está a **19 cm** sobre los tornillos;
la parte superior del cilindro está a **18 cm** desde el mismo plano. No se suman.

Una vez resueltos esos puntos: validar el recorrido en SIM, comparar una captura
real, revisar el plan vigente con el operador y recién preparar un ensayo físico
pequeño y documentado. Guardar las medidas y logs reales sin sustituirlos por
resultados del simulador.
