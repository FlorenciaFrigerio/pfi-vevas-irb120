# Bitácora técnica · Hito 1 (Gemelo digital del ABB IRB 120)

[Volver al Hito 1](../../README.md) · [Cronología general](../../../01_docs/01_CRONOLOGIA.md) · [Informe del Hito 1](../01_informe/01_Informe_Hito1_VEVAS.pdf)

Registro de decisiones técnicas del Hito 1 siguiendo el formato trazable **Problema → Cambio → Resultado → Archivo de evidencia**.

---

## H1-01 · Reconciliación entre el cero mecánico de ABB y el cero Denavit-Hartenberg

- **Problema:** En la postura de calibración de fábrica de ABB ($q = 0$), el brazo inferior (`link_2`) apunta verticalmente hacia arriba, mientras que la convención Denavit-Hartenberg estándar exige paralelismo entre $x_1$ y $x_2$ (brazo horizontal). Inyectar un offset manual en `/joint_states` desplazaba las lecturas articulares respecto al controlador real.
- **Cambio:** Se factorizó $A_i = R_z(\text{off}_i)\,R_z(q_i)\,B_i$ con $B_i = T_z(d_i)\,T_x(a_i)\,R_x(\alpha_i)$ y se absorbió el desfase $\theta_2 = q_2 - \pi/2$ una sola vez en el origen constante de `joint_2` del URDF ($\text{Origin}_i = B_{i-1}\,R_z(\text{off}_i)$), dejando `/joint_states` en ángulos mecánicos puros de ABB.
- **Resultado:** En $q = [0,0,0,0,0,0]$, la cadena cierra analítica y geométricamente en $x = d_4 + d_6 = 0{,}374\text{ m}$, $y = 0\text{ m}$ y $z = d_1 + a_2 + a_3 = 0{,}630\text{ m}$, con error interno $2{,}082 \times 10^{-18}\text{ m}$ frente a las cotas del manual ABB.
- **Archivo de evidencia:** [`fk_irb120.py`](../../../05_ros2_irb120_description/04_scripts/fk_irb120.py) · [`generate_urdf.py`](../../../05_ros2_irb120_description/04_scripts/generate_urdf.py)

---

## H1-02 · Comprobación algebraica de los marcos del URDF sin ajuste visual en RViz

- **Problema:** Ajustar orígenes `xyz` y ángulos `rpy` a ojo en RViz puede producir un robot visualmente plausible cuyos ejes internos estén invertidos o desplazados respecto a la cinemática directa.
- **Cambio:** Se construyeron deliberadamente dos archivos independientes: `irb120_manual.urdf` (deducido junta por junta a mano) e `irb120.urdf` (generado por script desde `DH_TABLE` mediante `generate_urdf.py` y `add_visuals.py`). Se añadió `compare_urdf.py` para contrastar los orígenes y ejes de las 7 uniones (`joint_1`…`joint_6` y `joint_6_tool0`) y una prueba de perturbación individual de $0{,}5\text{ rad}$ por eje en `figs.py`.
- **Resultado:** Coincidencia exacta (`0.000e+00` en traslación y rotación en las 7 uniones, estado `IDENTICOS`). Además, mover solo `J4` o `J6` desde $q=0$ produjo desplazamiento $0{,}0000\text{ m}$ en `tool0`, confirmando que ambos ejes son colineales con el vector hacia la brida.
- **Archivo de evidencia:** [`compare_urdf.py`](../../../05_ros2_irb120_description/04_scripts/compare_urdf.py) · [`irb120_manual.urdf`](../../../05_ros2_irb120_description/01_urdf/irb120_manual.urdf) · [`juntas.pdf`](../01_informe/01_fuente/01_img/juntas.pdf)

---

## H1-03 · Sincronización temporal de `/joint_states` y tratamiento de las 499 lecturas por timeout

- **Problema:** Al publicar 1.000 configuraciones aleatorias hacia `robot_state_publisher` y consultar TF tras una pausa fija, ocurrían dos fallos: (1) si `joint_state_publisher` seguía abierto, sobrescribía las juntas con $q = 0$; (2) `lookup_transform` devolvía a veces la transformada de la iteración anterior, aparentando errores falsos de varios decímetros.
- **Cambio:** En `validate_tf.py` se incorporó `revisar_competencia()` (`count_publishers("/joint_states") > 1`) y `leer_tf_sincronizado(t_pub_ns, timeout=2.0)`, que sella cada `JointState` con la marca temporal en nanosegundos y descarta cualquier transformada TF con `stamp < t_pub_ns`. Si el árbol TF no actualiza dentro de `2,0 s`, la configuración se contabiliza como *timeout* y no entra como muestra falsa.
- **Resultado:** De **1.000 intentos**, **501 lecturas cumplieron sincronización estricta** con error posicional máximo de **$2{,}459 \times 10^{-10}\text{ m}$** ($4\,100\times$ por debajo del umbral de $10^{-6}\text{ m}$) y error angular máximo de $1{,}092 \times 10^{-9}$. Las **499 configuraciones restantes agotaron el tiempo de espera** del buffer TF por ráfaga de publicación y quedaron documentadas explícitamente fuera del promedio.
- **Archivo de evidencia:** [`validate_tf.py`](../../../05_ros2_irb120_description/04_scripts/validate_tf.py) · [`01_validacion_hito1.txt`](../04_resultados/01_validacion_hito1.txt) · [`06_cobertura_tf.png`](../../../01_docs/06_figuras/06_cobertura_tf.png)

---

## H1-04 · Diagnóstico de la brecha entre verificación interna (`10⁻¹⁶ m`) y medición TF (`10⁻¹⁰ m`)

- **Problema:** La verificación en Python (`float64`) arrojaba un error de $3{,}68 \times 10^{-16}\text{ m}$, mientras que la consulta a ROS 2 TF se estabilizaba alrededor de $2{,}46 \times 10^{-10}\text{ m}$ (seis órdenes de magnitud mayor).
- **Cambio:** Se auditó la cadena de serialización de `robot_state_publisher` y `tf2`, separando en el informe la exactitud algebraica del generador URDF frente a la resolución numérica del transporte de cuaterniones en ROS.
- **Resultado:** Se demostró que la diferencia no proviene de un error geométrico en los parámetros D-H ni en el URDF, sino del redondeo del transporte intermedio en TF, permaneciendo aun así más de tres órdenes de magnitud debajo de la tolerancia exigida ($10^{-6}\text{ m}$).
- **Archivo de evidencia:** [`informe.tex`](../01_informe/01_fuente/informe.tex) · [`errores.pdf`](../01_informe/01_fuente/01_img/errores.pdf)
