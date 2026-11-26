# Guía de ejecución · Hito 1 (Visor 3D, URDF y validación en ROS 2)

[Volver al Hito 1](README.md) · [Portada del proyecto](../README.md) · [Paquete ROS 2](../05_ros2_irb120_description/README.md)

Esta guía reúne los tres modos de ejecutar y verificar el gemelo digital del **ABB IRB 120**: en cualquier navegador web (sin ROS), mediante scripts algebraicos en Python y dentro de **ROS 2 Jazzy** en Ubuntu 24.04.

---

## 1. Abrir el visor interactivo 3D en el navegador (Windows, macOS o Linux)

No requiere instalar ROS ni paquetes externos; utiliza únicamente el servidor HTTP integrado de Python 3:

```bash
# Desde la raíz del repositorio
python -m http.server 8000 --directory 02_hito1_gemelo_digital/02_visor_web
```

Luego abre **<http://localhost:8000/>** en Chrome, Edge o Firefox. Podrás:
- Inspeccionar las **29 mallas GLTF** agrupadas en `Base` y `J1`–`J6`, más efector y pieza ilustrativa.
- Alternar entre las pestañas **Estructura**, **Seis ejes** (límites y velocidades ABB) y **Celda de trabajo** (lista de materiales BOM).
- Usar el control deslizante de **vista explosionada (0–100 %)**, etiquetas flotantes y aislamiento de subconjuntos.

---

## 2. Verificar la cinemática directa y la identidad de ambos URDF (sin ROS)

```bash
# 1. Comprobar que irb120_manual.urdf e irb120.urdf sean exactamente el mismo robot
python 05_ros2_irb120_description/04_scripts/compare_urdf.py

# 2. Comprobar el cierre en q = 0 (x = 0,374 m, z = 0,630 m, error ~2e-18 m)
python 05_ros2_irb120_description/04_scripts/fk_irb120.py
```

Salida esperada de `compare_urdf.py`:
```text
joint_1 .. joint_6_tool0: diff pos 0.000e+00 m, diff rot 0.000e+00
RESULTADO: IDENTICOS
```

---

## 3. Visualizar en RViz y correr las 1.000 pruebas contra TF (ROS 2 Jazzy · Ubuntu 24.04)

### Paso A · Compilar el paquete `irb120_description`
```bash
mkdir -p ~/ros2_ws/src
ln -s "$(pwd)/05_ros2_irb120_description" ~/ros2_ws/src/irb120_description
cd ~/ros2_ws
colcon build --packages-select irb120_description
source install/setup.bash
```

### Paso B · Visualización interactiva con sliders en RViz
```bash
ros2 launch irb120_description display.launch.py
```

### Paso C · Ejecutar el validador sincronizado `validate_tf.py`
> [!WARNING]
> Cierra cualquier instancia de `joint_state_publisher` antes de lanzar el validador para que ningún otro nodo sobreescriba `/joint_states` con `q = 0`.

```bash
# Terminal 1: publicar el árbol TF sin interfaz de sliders
ros2 launch irb120_description display.launch.py gui:=false

# Terminal 2: asegurar único publicador y correr las 1 000 muestras
pkill -f joint_state_publisher || true
python3 05_ros2_irb120_description/04_scripts/validate_tf.py 1000
```

La salida reproducible de referencia se conserva en [`01_docs/04_resultados/01_validacion_hito1.txt`](01_docs/04_resultados/01_validacion_hito1.txt).
