# Prueba de figuras en VEVAS Studio · Ubuntu

Esta guía prueba la simulación del 22/11. Para el control físico siguen vigentes las compuertas y el bloqueo por C02.

## Abrir el ZIP en una carpeta nueva

Guardá `VEVAS_SIM_23NOV.zip` en Descargas. Extraelo en `~/Descargas/ROBOTICA`; crea la carpeta nueva `vevas_sim_23nov`. Abrí la terminal dentro de esa carpeta, que contiene `consola_vevas.py`, `requirements.txt` y `08_web/`. Mantené juntos los subdirectorios del ZIP.

En el directorio que contiene `iniciar_simulacion.sh`, ejecutá:

```bash
python3 -m venv .venv_interfaz
.venv_interfaz/bin/python -m pip install -r requirements.txt
.venv_interfaz/bin/python consola_vevas.py --modo sim --puerto 9012 --abrir
```

En Ubuntu, si `python3 -m venv` indica que falta el módulo, instalá `python3-venv` y repetí. Abrí `http://127.0.0.1:9012` si el navegador no abre automáticamente. Confirmá que la interfaz diga SIM. Si ya hay otra consola en ese puerto, cerrá esa sesión antes de arrancar esta versión.

## Arrastre vertical del TCP

1. Activá el control cartesiano de traslación y localizá el asidero central en la punta. Arrastrá ese punto hacia arriba y hacia abajo con el mouse. Para restringir el desplazamiento a un eje de la base, usá su flecha. Observá el fantasma y los valores de pose. Girá la cámara y repetí para comprobar el mismo control desde otro ángulo.
2. Soltá el mouse y comprobá que el brazo simulado mantiene su última postura. Calculá y revisá el nuevo plan antes de pedir su ejecución.
3. Cambiá el objetivo después de calcular. La revisión anterior debe quedar invalidada y el envío debe requerir el plan nuevo.

## Figuras, saludo y retorno a cero

1. Abrí **Poses predeterminadas** y elegí **Círculo**. Usá una postura inicial admisible, un tamaño pequeño y elegí `XY`, `XZ` o `YZ` en **Plano base**. Pulsá **Calcular trayectoria** y revisá la dirección y el plano del recorrido.
2. Comprobá que se muestran la traza prevista del TCP y los puntos de paso. Podés usar **Ver en 3D** desde la revisión. Pulsá **Continuar a confirmación**, regresá con **Volver a revisión** y verificá que la postura no cambia. Cuando el plan esté listo, elegí **Confirmar ejecución** para probarlo en SIM.
3. Repetí con **Corazón** y **Cuadrado**. En el cuadrado, comprobá que aparecen los cuatro vértices y que el camino vuelve al punto de inicio. Si el plan se rechaza, registrá la causa y elegí otra ubicación, plano o tamaño que pase la revisión.
4. Elegí **Saludo** y revisá la secuencia de posturas antes de confirmar. Compará la vista previa con el rastro del TCP durante o al terminar la animación.
5. Desde una postura distinta de cero, elegí **Volver a cero**. Comprobá que el retorno se muestra y requiere confirmación. Revisá cualquier aviso de singularidad que presente SIM.
6. En **Perfil del plan**, compará **Suave** y **LSPB académico** para un mismo punto a punto. En las gráficas, observá la velocidad y la aceleración al comienzo, al final y en las uniones del recorrido. Guardá junto con la captura el perfil, frecuencia de pantalla, duración y cantidad de muestras mostradas.

Ante una diferencia, registrá la versión probada, navegador, modo de visor, pose de inicio, opción, tamaño y plano, mensaje de revisión y captura. Si el visor usa el modo compatible, indicá si repite el comportamiento en WebGL.
