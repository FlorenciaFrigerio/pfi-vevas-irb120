# Script 08 - `resolver_en_vivo.py`: Resolvedor interactivo de poses

> **Nivel:** Núcleo de demostración - dominar uso

## Para qué sirve
Convierte entradas de usuario en una pose homogénea y muestra ramas IK, descartes y selección.

**Entrada y salida:** Entrada: matriz, cuaternión, RPY, eje z6 u otra representación. Salida: tabla de resultados en consola.

## Recorrido por bloques

### A. Constructores de pose
T_desde_cuaternion, T_desde_rpy, T_desde_eje_z y T_desde_R_p crean T.

**Código real** (`scripts/resolver_en_vivo.py`, líneas 43-54):

```python
def T_desde_cuaternion(x, y, z, qx, qy, qz, qw):
    """Cuaternion en convencion (qx, qy, qz, qw), parte real al final."""
    n = np.linalg.norm([qx, qy, qz, qw])
    if n < 1e-12:
        raise ValueError("cuaternion nulo")
    qx, qy, qz, qw = np.array([qx, qy, qz, qw]) / n
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])
    return T_desde_R_p(R, [x, y, z])
```

**Cómo leerlo:** Cada formato declara convención; RPY usa Rz(yaw)Ry(pitch)Rx(roll).

### B. Reporte
resolver_y_mostrar calcula alcance, ramas, errores FK, manipulabilidad y rango.

**Código real** (`scripts/resolver_en_vivo.py`, líneas 123-181):

```python
def resolver_y_mostrar(T_d, q_actual=None, titulo=""):
    """Tabla completa de las 8 ramas con verificacion FK."""
    if q_actual is None:
        q_actual = np.zeros(6)

    print()
    if titulo:
        print(f"  {titulo}")
    p = T_d[:3, 3]
    print(f"  p_d = [{p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f}] m "
          f"  ||p_d|| = {np.linalg.norm(p):.4f} m")

    # Chequeo de alcance antes de resolver: evita presentar como
    # "fallo del algoritmo" lo que es una pose fuera del espacio.
    d6 = 0.072
    p_W = p - d6 * T_d[:3, 2]
    r_W = np.linalg.norm(p_W - np.array([0.0, 0.0, 0.290]))
    print(f"  centro de muneca p_W = [{p_W[0]:+.4f}, {p_W[1]:+.4f}, "
          f"{p_W[2]:+.4f}]   r_W = {r_W:.4f} m  (max 0.5800 m)")
    # Tolerancia generosa a proposito: una pose exactamente sobre la
    # cascara esferica del alcance es resoluble, y anunciarla como
    # inalcanzable para despues resolverla seria contradecirse.
    holgura = 1e-4      # 0.1 mm: por debajo de eso es ruido numerico
    if r_W > 0.580 + holgura:
        exceso = r_W - 0.580
        print(f"  -> FUERA DE ALCANCE: r_W excede a2 + k = 0.5800 m "
              f"por {exceso*1000:.1f} mm.")
    elif r_W > 0.580 - 1e-3:
        print("  -> EN EL LIMITE del alcance: el brazo queda extendido, "
              "cerca de la singularidad de codo.")

    try:
        crudas = pieper(T_d)
    except Exception as exc:
        print(f"  -> Pieper no produjo ramas: {exc}")
        return None, [], []

    admisibles, rechazadas = filtrar(crudas, T_d, tol_pose=TOL_POSE)

    print(f"\n  ramas calculadas {len(crudas)}    "
          f"admisibles {len(admisibles)}    rechazadas {len(rechazadas)}\n")

    cab = (f"  {'#':>2}  {'q1':>8} {'q2':>8} {'q3':>8} {'q4':>8} "
           f"{'q5':>8} {'q6':>8}  {'e_p [m]':>10} {'e_R':>10}  "
           f"{'w':>9}  etiqueta")
    print(cab)
    print("  " + "-" * (len(cab) - 2))
    for i, q in enumerate(crudas, 1):
        ep, eR = error_pose(q, T_d)
        w = medida_manipulabilidad(q)
        gq = np.degrees(q)
        marca = "" if any(np.allclose(q, a) for a in admisibles) else "  (rechazada)"
        alerta = "  [!] cerca de singularidad" if w < UMBRAL_SINGULAR else ""
        print(f"  {i:>2}  {gq[0]:>8.2f} {gq[1]:>8.2f} {gq[2]:>8.2f} "
              f"{gq[3]:>8.2f} {gq[4]:>8.2f} {gq[5]:>8.2f}  "
              f"{ep:>10.2e} {eR:>10.2e}  {w:>9.2e}  "
              f"{etiquetar(q)}{marca}{alerta}")
    print("\n  Angulos en grados. Criterio de aceptacion: "
          f"e_p < {TOL_POSE:.0e} m y e_R < {TOL_POSE:.0e}.")
```

**Cómo leerlo:** Es una interfaz sobre pieper.py y jacobiano.py.

### C. Menú y casos
menu acepta datos; casos_borde ejecuta ejemplos regulares, singulares y fuera de alcance.

**Código real** (`scripts/resolver_en_vivo.py`, líneas 294-330):

```python
def casos_borde():
    """
    Poses dificiles a proposito.

    Si el docente dicta una pose incomoda, conviene haberla visto antes.
    """
    print("\n   CASOS BORDE\n")

    # 1. Interior del espacio, lejos de singularidades.
    q = np.radians([25.0, -35.0, 20.0, 40.0, 50.0, -30.0])
    resolver_y_mostrar(forward_kinematics(q),
                       titulo="1/6  Interior del espacio de trabajo")

    # 2. Singularidad de muneca: q5 = 0 exacto.
    q = np.radians([20.0, -30.0, 15.0, 35.0, 0.0, -25.0])
    resolver_y_mostrar(forward_kinematics(q),
                       titulo="2/6  Singularidad de muneca (q5 = 0)")

    # 3. Singularidad de codo: brazo extendido.
    q = np.array([0.3, 0.0, -1.3430, 0.2, 0.6, 0.1])
    resolver_y_mostrar(forward_kinematics(q),
                       titulo="3/6  Codo extendido (sin(q3 + psi) = 0)")

    # 4. Eje de la base: singularidad de hombro.
    T = np.eye(4)
    T[:3, 3] = [0.0, 0.0, 0.620]
    resolver_y_mostrar(T, titulo="4/6  Sobre el eje de la base (hombro)")

    # 5. Limite del alcance.
    T = np.eye(4)
    T[:3, 3] = [0.5795, 0.0, 0.362]
    resolver_y_mostrar(T, titulo="5/6  Al borde del alcance (r_W ~ 0.580 m)")

    # 6. Fuera de alcance.
    T = np.eye(4)
    T[:3, 3] = [1.5, 1.5, 1.5]
    resolver_y_mostrar(T, titulo="6/6  Fuera de alcance")
```

**Cómo leerlo:** Sirve para una demostración repetible sin escribir un nodo ROS.

## Cómo se relaciona
Depende de fk_irb120.py, pieper.py y jacobiano.py.

## Pruebas y evidencia
Los casos de demo ilustran la salida; las pruebas cuantitativas están en notebook y archivos de resultados.

## Pregunta para practicar defensa
> Si solo te dan posición y eje z6, ¿queda fijada toda la orientación?

## Límite o precaución
El eje z deja libre la rotación alrededor de sí mismo; el programa completa esa libertad con una elección.

## Ficha de repaso
- Problema que resuelve:
- Parámetros de entrada:
- Salida principal:
- Función clave:
- Prueba que lo respalda:
- Lo que no demuestra:
