# Descripción ROS 2 del ABB IRB 120

El paquete `irb120_description` contiene la geometría y los marcos que se comparan en el Hito 1 y los scripts de cálculo utilizados en el Hito 2.

| Área | Función |
| --- | --- |
| [01 · URDF](01_urdf/) | Descripciones manual y generada del manipulador |
| [02 · Lanzamiento](02_launch/) | Publicación del modelo y visualización |
| [03 · RViz](03_rviz/) | Configuración de visualización |
| [04 · Scripts](04_scripts/README.md) | Cinemática, trayectorias, dinámica y comparación |

## Verificar los dos modelos

Desde la raíz del repositorio ejecuta `python 05_ros2_irb120_description/04_scripts/compare_urdf.py`. El script compara los orígenes y las orientaciones articulares de ambos archivos. La reproducción del recorrido numérico del Hito 2 se encuentra en su [guía de ejecución](../03_hito2_cinematica_dinamica/01_COMO_EJECUTAR.md).
