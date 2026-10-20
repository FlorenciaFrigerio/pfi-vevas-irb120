# Script 12 - `generate_urdf.py`: Generador de URDF

> **Nivel:** Hito 1 - contexto geométrico

## Para qué sirve
Deriva origins URDF desde D-H y verifica offline que la cadena coincida con FK.

**Entrada y salida:** Entrada: DH_TABLE. Salida: urdf/irb120.urdf y discrepancia posicional.

## Recorrido por bloques

### A. Factorización
B_matrix y Rz separan la parte constante y la variable de cada junta.

**Código real** (`scripts/generate_urdf.py`, líneas 36-49):

```python
def B_matrix(d, a, alpha):
    """
    B_i = Tz(d) . Tx(a) . Rx(alpha)

    Las traslaciones puras SI conmutan entre si, por eso Tz(d).Tx(a)
    colapsa al vector (a, 0, d) expresado en el marco padre.
    """
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [1.0, 0.0, 0.0,   a],
        [0.0,  ca, -sa, 0.0],
        [0.0,  sa,  ca,   d],
        [0.0, 0.0, 0.0, 1.0],
    ])
```

**Cómo leerlo:** Convierte la forma D-H al requisito Origin*Rot(axis,q) de URDF.

### B. RPY
matrix_to_rpy convierte matrices de origen y trata gimbal lock.

**Código real** (`scripts/generate_urdf.py`, líneas 73-98):

```python
def matrix_to_rpy(R):
    """
    Extrae los angulos roll-pitch-yaw de una matriz de rotacion.

    URDF define rpy como composicion extrinseca en orden x-y-z, es decir
    R = Rz(yaw) . Ry(pitch) . Rx(roll).

    De R[2][0] = -sin(pitch) se despeja pitch. Cuando |R[2][0]| -> 1 el
    pitch vale +-pi/2 y la representacion pierde un grado de libertad
    (gimbal lock): roll y yaw dejan de ser independientes. En ese caso se
    fija yaw = 0 y se absorbe todo el giro en roll. Esta rama degenerada
    ocurre realmente en la junta 2 del IRB 120.
    """
    if abs(R[2, 0]) > 1.0 - 1e-10:
        pitch = -np.sign(R[2, 0]) * np.pi / 2
        yaw = 0.0
        if R[2, 0] > 0:
            roll = np.arctan2(-R[0, 1], -R[0, 2])
        else:
            roll = np.arctan2(R[0, 1], R[0, 2])
    else:
        pitch = -np.arcsin(R[2, 0])
        cp = np.cos(pitch)
        roll = np.arctan2(R[2, 1] / cp, R[2, 2] / cp)
        yaw = np.arctan2(R[1, 0] / cp, R[0, 0] / cp)
    return roll, pitch, yaw
```

**Cómo leerlo:** La rama degenerada aparece en el origen de joint_2.

### C. Escritura y validación
joint_origins, build_urdf y verify_chain calculan, escriben y comparan la cadena.

**Código real** (`scripts/generate_urdf.py`, líneas 189-214):

```python
def verify_chain(n=1000, seed=42):
    """
    Comprueba offline que la cadena URDF reproduce 0_T_6 analitica.

    Recorre el arbol tal como lo hara robot_state_publisher
    (Origin_i . Rz(q_i)) y compara con forward_kinematics().
    """
    origins = joint_origins()
    rng = np.random.default_rng(seed)
    peor = 0.0
    for _ in range(n):
        q = rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        T = np.eye(4)
        for i in range(6):
            xyz, rpy = origins[i]
            O = np.eye(4)
            O[:3, :3] = rpy_to_matrix(*rpy)
            O[:3, 3] = xyz
            T = T @ O @ Rz(q[i])
        xyz, rpy = origins[6]
        O = np.eye(4)
        O[:3, :3] = rpy_to_matrix(*rpy)
        O[:3, 3] = xyz
        T = T @ O
        peor = max(peor, np.linalg.norm(T[:3, 3] - forward_kinematics(q)[:3, 3]))
    return peor
```

**Cómo leerlo:** Establece el puente entre la tabla analítica y el árbol TF.

## Cómo se relaciona
Depende de fk_irb120.py; add_visuals.py reutiliza los orígenes.

## Pruebas y evidencia
La función verify_chain compara posiciones con configuraciones aleatorias.

## Pregunta para practicar defensa
> ¿Qué transformación exige URDF y cómo se acomoda la variable articular?

## Límite o precaución
El URDF generado no debe editarse manualmente; corregir el generador.

## Ficha de repaso
- Problema que resuelve:
- Parámetros de entrada:
- Salida principal:
- Función clave:
- Prueba que lo respalda:
- Lo que no demuestra:
