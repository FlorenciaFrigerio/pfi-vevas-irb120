# Evidencia reproducible de sesión · Corazón 40 mm en plano YZ («De frente»)

[Volver a Resultados](../README.md) · [Ir a Hitos 3 y 4](../../../README.md) · [Videos asociados](../../../../06_videos/03_hito3_4_simulacion/README.md)

Esta carpeta conserva los artefactos tabulares y de planificación que alimentan la figura central [`13_evidencia_sesion_tcp.png`](../../../../01_docs/06_figuras/13_evidencia_sesion_tcp.png).

## Archivos incluidos

| Archivo | Formato y unidades | Contenido |
| --- | --- | --- |
| [`muestras_corazon_yz_40mm.csv`](muestras_corazon_yz_40mm.csv) | CSV (`s`, `°`, `mm`, `°/s`) | Serie temporal registrada a `50 Hz` (`q1..q6` comandadas y medidas, errores articulares, `e_p_mm`, `e_R_deg`, coordenadas `x_mm, y_mm, z_mm`, `sigma_min` y velocidades `v1..v6_med_deg_s`). |
| [`resumen_corazon_yz_40mm.json`](resumen_corazon_yz_40mm.json) | JSON | Duración ejecutada y planificada, error máximo de seguimiento, error final articular/cartesiano y velocidades pico por junta. |
| [`plan_001_corazon_yz_40mm.json`](plan_001_corazon_yz_40mm.json) | JSON (`m`, `rad`, `s`) | Los `65` puntos solicitados de la figura, parada en la cúspide (`#32`), ramas de Pieper seleccionadas y vista previa del camino quíntico $C^2$. |

## Cómo regenerar estos datos y la figura central

```bash
python3 01_docs/06_figuras/generar_evidencia_visual.py
```
