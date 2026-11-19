# Guía 1 — instalar y ejecutar en Ubuntu

Versión de diseño temporal del 19/11/2026. No reemplaza ~/vevas-publicacion ni modifica abb_ws. No necesita estar conectado al robot, ROS, RobotStudio ni FlexPendant. El modelo se mueve en simulación; no es un gemelo dinámico validado del brazo real.

## Primera instalación

1. Descargá VEVAS_interfaz_prototipo_2026-11-19.zip desde el chat usando Guardar como… y guardalo en tu carpeta Descargas con ese nombre exacto. Si tu navegador cambia el nombre, renombralo antes de seguir.
2. Abrí una terminal nueva con Ctrl+Alt+T. Copiá un bloque por vez, sin agregar barras invertidas al final de cada línea.

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip unzip
```

La contraseña de sudo no se ve mientras la escribís. Presioná Enter al terminar.

```bash
mkdir -p "$HOME/Descargas/ROBOTICA"
unzip -n "$HOME/Descargas/VEVAS_interfaz_prototipo_2026-11-19.zip" -d "$HOME/Descargas/ROBOTICA"
cd "$HOME/Descargas/ROBOTICA/vevas_interfaz_prototipo_2026-11-19"
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
bash iniciar_simulacion.sh
```

Internet se necesita para instalar los paquetes. La interfaz, sus bibliotecas JavaScript y mallas 3D están incluidas. No uses sudo pip ni --break-system-packages. El script usa el Python de .venv sin cambiar la instalación de ROS.

3. Mantené esta terminal abierta. Se intentará abrir el navegador. Si no se abre, ingresá manualmente:

http://127.0.0.1:9012

Abrilo en esa misma Ubuntu, no en el celular ni en otra computadora. No abras index.html con doble clic: la interfaz necesita el servidor Python.

## Primera prueba visual

1. Comprobá arriba «PROTOTIPO EN REVISIÓN» y «Simulación · no conectado al brazo físico».
2. Girá la cámara arrastrando el robot 3D; la rueda acerca o aleja. «Vista inicial» restablece la cámara.
3. En Control → Una junta, seleccioná J6, ingresá 5 y dejá marcado Relativo.
4. Pulsá Calcular movimiento de J6. Revisá el resumen y el objetivo fantasma.
5. Pulsá Habilitar simulador y, cuando se habilite, Ejecutar en simulación.
6. Abrí Gráficas. Seleccioná J6 y «Posición · referencia y recibida». La recibida es del simulador; el retardo y ruido son sintéticos. No es evidencia física.
7. Probá Error articular, errores del TCP y las curvas LSPB. Las curvas de velocidad/aceleración son planificadas, no medidas. Singularidad muestra el sigma mínimo de un Jacobiano mixto, sin un umbral universal.
8. Elegí 20, 60 o 120 s. Podés pausar para inspeccionar. Exportar estados descarga un CSV de las muestras recientes de la ventana seleccionada; no exporta velocidad medida ni torque. Una ventana recién abierta puede contener menos segundos.
9. En Control → Pose del TCP, usá X=400, Y=0, Z=400 mm y Rz=0, Ry=180, Rx=0 grados. Pulsá Resolver inversa · Pieper. Revisá Cálculos y 8 ramas. No todas las ramas cumplen límites.
10. Datos en vivo muestra procedencia y valores no disponibles. FK en vivo y Jacobiano muestran las matrices calculadas.

Cambiar un objetivo invalida el plan anterior: hay que recalcular. Seleccionar una pieza 3D elige la junta, sin ejecutar por sí solo. Al soltar el deslizador se calcula la vista previa; enviar requiere el botón de ejecución. El movimiento manual por flechas exige habilitar antes el simulador. Evitá editar un campo mientras usás atajos.

## Cerrar y volver a abrir

Cerrar la pestaña no necesariamente detiene el servidor. Para cerrar el programa, en la terminal presioná Ctrl+C. Los registros de simulación quedan en sesiones/, dentro de esta copia.

La próxima vez no reinstales:

```bash
cd "$HOME/Descargas/ROBOTICA/vevas_interfaz_prototipo_2026-11-19"
bash iniciar_simulacion.sh
```

## Si algo falla

- «No such file»: comprobá dónde descargaste el ZIP. Ejecutá `ls "$HOME/Descargas"` y no cambies carpetas del repo original.
- «Address already in use»: primero abrí http://127.0.0.1:9012; puede existir una instancia previa. Cerrala con Ctrl+C en su terminal. Si necesitás otra, ejecutá `.venv/bin/python consola_vevas.py --modo sim --puerto 9013 --abrir` y usá 9013 en el navegador.
- «No module named numpy»: desde esta carpeta ejecutá `.venv/bin/python -m pip install -r requirements.txt`.
- Interfaz anterior: comprobá carpeta y puerto. Usá Ctrl+Shift+R para recargar sin caché.
- Sin WebGL: se intentará el renderizador compatible en un Worker. Su cadencia es menor; la telemetría y los cálculos no se convierten en mediciones físicas. Si aparece error, pasá el mensaje exacto y la versión del navegador.
- «Watchdog»: la interfaz dejó de entregar el pulso a tiempo. Revisá Historial y la carga del navegador; no aumentes ni desactives el límite para esconderlo. Se observaron abortos durante inspección automatizada, y también una prueba posterior completada. Falta verificar el comportamiento en tu Ubuntu.

Esta versión rechaza --modo ros y --real intencionalmente. No está lista para mandar órdenes físicas.
