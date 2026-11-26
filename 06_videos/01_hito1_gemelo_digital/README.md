# Videos · Hito 1 (Gemelo digital URDF y validación TF)

[Volver a 06 · Videos](../README.md) · [Ir al Hito 1](../../02_hito1_gemelo_digital/README.md)

## Archivos previstos en esta carpeta

| Archivo esperado | Duración sugerida | Qué demuestra | Evidencia vinculada |
| --- | --- | --- | --- |
| `2026-10_hito1_visor_3d_y_rviz.mp4` | 45–60 s | Inspección de `base_link`, `J1`–`J6` y `tool0` en el visor web 3D y en RViz en la misma postura articular | [`10_comparacion_visor_rviz.png`](../../01_docs/06_figuras/10_comparacion_visor_rviz.png) |
| `2026-10_hito1_validacion_tf_501_muestras.mp4` | 60–90 s | Lanzamiento de `display.launch.py gui:=false`, cierre de publicadores en conflicto y corrida de `validate_tf.py` | [`01_validacion_hito1.txt`](../../02_hito1_gemelo_digital/01_docs/04_resultados/01_validacion_hito1.txt) |

## Lista de verificación visual

1. Mostrar el cierre en `q = [0, 0, 0, 0, 0, 0]` con `tool0 = [0,374, 0,000, 0,630] m`.
2. Mostrar la rotación de `J4` y `J6` confirmando que cambian la orientación de la brida sin desplazar el origen de `tool0`.
3. Mostrar el resumen final en consola (`501` lecturas válidas, `499` timeouts excluidos, error máximo `2,459 × 10⁻¹⁰ m`).
