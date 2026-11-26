# 06 · Evidencias audiovisuales del proyecto

[Portada del proyecto](../README.md) · [Índice general](../01_docs/02_INDICE.md) · [Lista de entrega](../01_docs/04_LISTA_DE_ENTREGA.md)

Esta carpeta centraliza la organización de los videos de demostración, validación numérica, simulación y pruebas de laboratorio con el **ABB IRB 120 + IRC5**. Sigue la numeración por hitos del resto del repositorio para que cada registro visual quede vinculado con su código, su bitácora y sus datos tabulares (`CSV` / `JSON`).

> [!NOTE]
> **Política de almacenamiento en Git:** Los archivos de video pesados (`.mp4`, `.mov`, `.mkv`) no deben superar el límite de GitHub sin Git LFS. Cada subcarpeta conserva su ficha descriptiva (`README.md`) con el nombre canónico esperado, plano, unidades y vínculo al archivo de muestras para alojar los archivos localmente durante la defensa o enlazar su publicación externa.

## 1. Estructura de carpetas

| Carpeta | Hito | Contenido previsto | Evidencia numérica asociada |
| --- | --- | --- | --- |
| [`01_hito1_gemelo_digital/`](01_hito1_gemelo_digital/README.md) | Hito 1 | Comparación del visor 3D web con RViz y ejecución del validador TF (`501/1000` muestras) | [`01_validacion_hito1.txt`](../02_hito1_gemelo_digital/01_docs/04_resultados/01_validacion_hito1.txt) |
| [`02_hito2_cinematica_dinamica/`](02_hito2_cinematica_dinamica/README.md) | Hito 2 | Recorrido completo de una pose (`demo_recorrido.py`), ramas de Pieper, perfil LSPB y paso por singularidad | [`resultados.json`](../03_hito2_cinematica_dinamica/01_docs/04_resultados/resultados.json) |
| [`03_hito3_4_simulacion/`](03_hito3_4_simulacion/README.md) | Hitos 3/4 (SIM) | Trazado de figuras en plano `YZ` («De frente»), vista 2D del TCP, pestaña Velocidades y aborto seguro | [`muestras_corazon_yz_40mm.csv`](../04_hito3_4_control_real/01_docs/04_resultados/01_sesion_evidencia/muestras_corazon_yz_40mm.csv) |
| [`04_hito3_4_laboratorio_irc5/`](04_hito3_4_laboratorio_irc5/README.md) | Hitos 3/4 (IRC5) | Lectura RWS frente a FlexPendant (`0,0001 mm`), verificación C02 en `AUTO` al `5 %` y ejecución en streaming | [`05_CAMBIOS_24NOV.md`](../04_hito3_4_control_real/01_docs/05_bitacora/05_CAMBIOS_24NOV.md) |
| [`05_defensa_ejecutiva/`](05_defensa_ejecutiva/README.md) | Integrador | Video síntesis de defensa (3–5 min) conectando gemelo digital, cinemática y control real | [`02_Defensa_Grupo_VEVAS.pdf`](../04_hito3_4_control_real/01_docs/02_presentacion/02_Defensa_Grupo_VEVAS.pdf) |

## 2. Convención de nombres

Todos los archivos de video siguen el formato:

```text
AAAA-MM-DD_hitoX_<modo>_<descripcion_corta>.mp4
```

Ejemplos:
- `2026-10-12_hito1_ros2_validacion_tf_501_muestras.mp4`
- `2026-10-19_hito2_cli_recorrido_pose_pieper_lspb.mp4`
- `2026-11-24_hito3_4_sim_corazon_yz_40mm_5pct.mp4`
- `2026-11-24_hito3_4_irc5_verificacion_c02_auto_5pct.mp4`

## 3. Requisitos de grabación en laboratorio (según Guía 6)

De acuerdo con [`08_GUIA_6_SEGURIDAD_GO_Y_VIDEO.md`](../04_hito3_4_control_real/01_docs/03_guias/08_GUIA_6_SEGURIDAD_GO_Y_VIDEO.md), toda toma presencial con el ABB IRB 120 debe mostrar o documentar:

1. **Encuadre físico:** brazo completo, mesa despejada (`190 mm` de referencia) y operador con paro de emergencia accesible en el FlexPendant.
2. **Sincronización con pantalla:** captura simultánea o superpuesta de VEVAS Studio mostrando modo (`RWS · C02 verificado`), plano (`YZ`, `XY` o `XZ`), porcentaje del slider y pestaña **Velocidades**.
3. **Trazabilidad:** acompañar cada archivo `.mp4` de una copia de [`plantilla_metadatos_video.json`](plantilla_metadatos_video.json) apuntando al `muestras.csv` y `resumen.json` de la misma corrida.
