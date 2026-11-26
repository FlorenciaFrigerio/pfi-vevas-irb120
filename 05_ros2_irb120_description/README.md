# Paquete ROS 2 `irb120_description` y Biblioteca Numérica Compartida

[Portada del proyecto](../README.md) · [Hito 1 · Gemelo digital](../02_hito1_gemelo_digital/README.md) · [Hito 2 · Cinemática y dinámica](../03_hito2_cinematica_dinamica/README.md)

Este paquete `ament_cmake` cumple una doble función en la arquitectura del repositorio:
1. **En ROS 2 Jazzy (Hito 1):** publica el modelo **URDF** del **ABB IRB 120**, la configuración de **RViz** y el archivo de lanzamiento `display.launch.py`.
2. **En Python científico (Hitos 1 y 2):** aloja en [`04_scripts/`](04_scripts/README.md) la fuente única de verdad de la tabla Denavit-Hartenberg, el generador URDF, el validador TF y los módulos de Pieper, Jacobiano, singularidades, LSPB y dinámica.

---

## Organización del paquete

| Subcarpeta | Archivos principales | Propósito |
| --- | --- | --- |
| [`01_urdf/`](01_urdf/) | `irb120_manual.urdf`, `irb120.urdf` | Descripción razonada a mano frente a descripción generada automáticamente desde `DH_TABLE`. |
| [`02_launch/`](02_launch/) | `display.launch.py` | Lanza `robot_state_publisher`, `joint_state_publisher_gui` (opcional con `gui:=true/false`) y `rviz2`. |
| [`03_rviz/`](03_rviz/) | `irb120.rviz` | Escena preconfigurada con `RobotModel`, grilla métrica en `base_link` y árbol `TF`. |
| [`04_scripts/`](04_scripts/README.md) | `fk_irb120.py`, `pieper.py`, `jacobiano.py`, `lspb.py`, `dinamica_1r.py`, `validate_tf.py`, `demo_recorrido.py` | 16 scripts documentados y verificables desde terminal o desde el notebook del Hito 2. |

---

## Verificación rápida

```bash
# Sin ROS (cualquier sistema con Python + NumPy)
python 05_ros2_irb120_description/04_scripts/compare_urdf.py
python 05_ros2_irb120_description/04_scripts/fk_irb120.py

# Con ROS 2 Jazzy (Ubuntu 24.04)
colcon build --packages-select irb120_description
source install/setup.bash
ros2 launch irb120_description display.launch.py
```
