# Videos · Hitos 3 y 4 en simulación (VEVAS Studio SIM)

[Volver a 06 · Videos](../README.md) · [Ir a Hitos 3 y 4](../../04_hito3_4_control_real/README.md)

## Archivos previstos en esta carpeta

| Archivo esperado | Fecha / Plano / Unidades | Qué demuestra | CSV y figuras de la misma sesión |
| --- | --- | --- | --- |
| `2026-11-24_hito3_4_sim_corazon_yz_40mm_5pct.mp4` | `YZ` («De frente») · `40 mm` · `5 %` · `mm, °, °/s, s` | Cálculo, revisión aprobada, vista 2D del corazón con parada en la cúspide (`v = 0`) y superposición en vivo del TCP medido | [`muestras_corazon_yz_40mm.csv`](../../04_hito3_4_control_real/01_docs/04_resultados/01_sesion_evidencia/muestras_corazon_yz_40mm.csv) · [`13_evidencia_sesion_tcp.png`](../../01_docs/06_figuras/13_evidencia_sesion_tcp.png) |
| `2026-11-24_hito3_4_sim_saludo_y_aborto.mp4` | `YZ` («De frente») · `mm, °, s` | Vaivén en arco del saludo, lectura de la pestaña **Velocidades** y parada inmediata ante el botón **PARAR** | [`04_PRUEBA_SIM_FIGURAS.md`](../../04_hito3_4_control_real/01_docs/06_operacion/04_PRUEBA_SIM_FIGURAS.md) |

## Cómo reproducir exactamente la sesión grabada

```bash
cd 04_hito3_4_control_real/02_vevas_studio
python3 vevas.py --modo sim --puerto 9012 --abrir
```

1. Verificar postura inicial `[0°, −20°, +30°, 0°, +40°, 0°]` y slider al `5 %`.
2. Elegir figura **Corazón**, tamaño `40 mm`, plano **De frente (`YZ`)**.
3. Pulsar **Calcular y verificar**, abrir la vista 2D de puntos y **Confirmar ejecución**.
