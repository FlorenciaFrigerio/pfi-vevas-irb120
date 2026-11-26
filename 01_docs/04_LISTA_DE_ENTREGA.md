# 04 · Lista de entrega y estado de verificación

[Portada del proyecto](../README.md) · [Índice general](02_INDICE.md) · [Flujo CI en GitHub Actions](../.github/workflows/ci.yml)

## 1. Material central de defensa

1. [Informe del Hito 1 (PDF)](../02_hito1_gemelo_digital/01_docs/01_informe/01_Informe_Hito1_VEVAS.pdf) · [Bitácora técnica Hito 1](../02_hito1_gemelo_digital/01_docs/05_bitacora/README.md)
2. [Informe del Hito 2 (PDF)](../03_hito2_cinematica_dinamica/01_docs/01_informe/01_Informe_Hito2.pdf) · [Bitácora técnica Hito 2](../03_hito2_cinematica_dinamica/01_docs/05_bitacora/README.md)
3. [Informe del Hito 3/4 (PDF)](../04_hito3_4_control_real/01_docs/01_informe/01_Informe_Hito3_4.pdf) · [Bitácora formal W12](../04_hito3_4_control_real/01_docs/05_bitacora/06_bitacora.md)
4. [Diapositivas de defensa (PDF)](../04_hito3_4_control_real/01_docs/02_presentacion/02_Defensa_Grupo_VEVAS.pdf) · [Guion y banco Q&A](../04_hito3_4_control_real/01_docs/02_presentacion/01_PRESENTACION.md)
5. [Evidencia visual de sesión TCP (`CSV` + `JSON` + figura central)](../04_hito3_4_control_real/01_docs/04_resultados/01_sesion_evidencia/README.md)
6. [Carpeta organizada de videos (`06_videos/`)](../06_videos/README.md)

## 2. Verificaciones técnicas y automatización en `main`

| N.º | Comprobación | Estado local | Ejecución continua en GitHub Actions (`ci.yml`) |
| --- | --- | --- | --- |
| 2.1 | Enlaces internos Markdown | `0` enlaces rotos en todo el repositorio | Job `enlaces` en cada push/PR |
| 2.2 | Pruebas de JavaScript (`node --test`) | `20 / 20` aprobadas | Job `javascript` (Node.js 22) |
| 2.3 | Módulos y demo del Hito 1 y Hito 2 | `compare_urdf.py` (`IDENTICOS`) y 6 etapas ejecutadas | Job `python-y-notebook` |
| 2.4 | Notebook del Hito 2 (`01_Hito2_VEVAS.ipynb`) | 36 celdas con salidas verificadas | Job `python-y-notebook` (`nbconvert --execute`) |
| 2.5 | Suite completa de Python (`pytest`) | Lógica verificada (26 módulos en `06_tests/`) | Job `python-y-notebook` (`pytest 06_tests -q`) |
| 2.6 | Paquete ROS 2 (`irb120_description`) | URDF manual = generado, launch y RViz verificados | Job `ros2-colcon` (`colcon build` en `ros:jazzy-ros-base-noble`) |
