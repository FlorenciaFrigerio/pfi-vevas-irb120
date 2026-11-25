# VEVAS Studio

Consola web y gemelo del ABB IRB 120 para el PFI Hitos 3/4 del grupo VEVAS. La versión actual permite ensayar en simulación y visualizar la postura del brazo mediante RWS en solo lectura. El envío físico permanece bloqueado hasta verificar C02, el límite de velocidad de cada junta.

Ejecutá los comandos desde `hito3_4`.

## Simulación

```bash
python3 -m venv .venv_interfaz
.venv_interfaz/bin/python -m pip install -r requirements.txt
.venv_interfaz/bin/python consola_vevas.py --modo sim --puerto 9012 --abrir
```

La consola abre en `http://127.0.0.1:9012`. Para probar el arrastre del TCP, usá las flechas y los anillos, revisá el plan y probá el envío simulado. Seguí la [prueba de interfaz del 20/11](../06_operacion/03_PRUEBA_INTERFAZ_20NOV.md).

## RWS en solo lectura

En la PC del laboratorio con acceso al IRC5, consultá el estado o abrí la visualización.

```bash
source .venv_interfaz/bin/activate
bash iniciar_real.sh --solo-chequeo
bash iniciar_real.sh --solo-lectura
```

Si ROS 2 o RViz no están disponibles, agregá `--sin-rviz` a la segunda línea para usar solo la consola web. La visualización usa el nodo RWS en lectura y RViz, sin ejecutar MoveIt. Ver [RViz con la postura del ABB IRB 120](../06_operacion/05_RVIZ_RWS.md).

## Documentación

- [Conexión al robot el 23/11](../06_operacion/02_CONEXION_23NOV.md), [prueba de figuras en SIM](../06_operacion/04_PRUEBA_SIM_FIGURAS.md) y [cambios/verificación final](../05_bitacora/04_CAMBIOS_23NOV.md).

- [Aplicar el parche en Ubuntu y subir la rama](10_SUBIR_20NOV.md).
- [Cambios del 20/11](../05_bitacora/02_CAMBIOS_20NOV.md) y [contexto del proyecto](01_AVANCES_CONTEXTO.md).
- [Cambios históricos del 19/11](../05_bitacora/01_CAMBIOS_19NOV.md).
- [Handoff para retomar el proyecto](07_PROMPT_SIGUIENTE_IA.md) y [prompts por tarea](08_PROXIMOS_PROMPTS.md).
- [Mejoras priorizadas y evidencia pendiente](04_MEJORAS_EXTRAS.md).
- [Créditos del modelo 3D](../../02_vevas_studio/08_web/03_modelo/CREDITOS.md).
