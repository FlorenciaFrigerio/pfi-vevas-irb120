# Videos · Hitos 3 y 4 en laboratorio (ABB IRB 120 + IRC5 por RWS)

[Volver a 06 · Videos](../README.md) · [Ir a Hitos 3 y 4](../../04_hito3_4_control_real/README.md)

## Archivos previstos en esta carpeta

| Archivo esperado | Condiciones y unidades | Qué demuestra | Registro de respaldo |
| --- | --- | --- | --- |
| `2026-11-24_hito3_4_gate1_lectura_rws_flexpendant.mp4` | Solo lectura · `Base / wobj0 / tool0` · `mm, °` | Coincidencia entre `J1`–`J6` leídas por RWS y la pantalla del FlexPendant (diferencia FK `0,0001 mm` y `0,0000°`) | [`06_bitacora.md`](../../04_hito3_4_control_real/01_docs/05_bitacora/06_bitacora.md) |
| `2026-11-24_hito3_4_c02_verificacion_5pct.mp4` | Modo `AUTO` · tope C02 `5 %` (`12,5 / 16 / 21 °/s`) | Corrida presencial de `verificar_c02.py`: pulsos individuales por eje y prueba de streaming en `J6` (`5,2–8,9 °/s` medidos) | [`05_velocidades_c02.png`](../../01_docs/06_figuras/05_velocidades_c02.png) |
| `2026-11-24_hito3_4_real_figuras_streaming_yz.mp4` | Modo `AUTO` · plano `YZ` · `≤ 80 mm` · `mm, °/s, s` | Ejecución continua con `MoveAbsJ \T:=t, z1` en puntos intermedios y `fine` en vértices/cúspide, con paro de emergencia a mano | [`05_CAMBIOS_24NOV.md`](../../04_hito3_4_control_real/01_docs/05_bitacora/05_CAMBIOS_24NOV.md) |

## Cómo actualizar la figura central con un CSV de laboratorio

Cuando copies el `muestras.csv` de la corrida presencial desde la PC Ubuntu, colócalo junto a su `.mp4` o ejecuta [`analizar_sesion.py`](../../04_hito3_4_control_real/02_vevas_studio/02_herramientas/analizar_sesion.py) sobre la carpeta de la sesión para generar automáticamente `mov_001.png` e `informe.md`.
