# Videos · Hito 2 (Cinemática inversa, singularidades, LSPB y dinámica 1R)

[Volver a 06 · Videos](../README.md) · [Ir al Hito 2](../../03_hito2_cinematica_dinamica/README.md)

## Archivos previstos en esta carpeta

| Archivo esperado | Duración sugerida | Qué demuestra | Evidencia vinculada |
| --- | --- | --- | --- |
| `2026-10_hito2_demo_recorrido_pose.mp4` | 90–120 s | Ejecución en vivo de `resolver_en_vivo.py` y `demo_recorrido.py --pose`: entrada de `T_d`, 8 ramas de Pieper, selección `q*`, LSPB sincronizado y torque 1R | [`11_hito2_pose_y_ramas.png`](../../01_docs/06_figuras/11_hito2_pose_y_ramas.png) |
| `2026-10_hito2_singularidad_muneca_q5.mp4` | 45–60 s | Barrido hacia `q5 -> 0 rad`, caída del rango de `J` de `6` a `5` y ley de escalado `||qd|| ~ q5^-1` | [`salida_singularidad.txt`](../../03_hito2_cinematica_dinamica/01_docs/04_resultados/salida_singularidad.txt) |

## Comandos para grabar la demostración

```bash
cd 05_ros2_irb120_description/04_scripts
python3 demo_recorrido.py --seguido
python3 resolver_en_vivo.py --demo
```
