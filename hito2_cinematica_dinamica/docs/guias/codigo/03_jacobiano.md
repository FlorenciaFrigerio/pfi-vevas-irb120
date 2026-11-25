# Script 03 - `jacobiano.py`: Jacobiano geométrico

> **Nivel:** Núcleo Hito 2 - dominar

## Para qué sirve
Relaciona velocidades articulares con velocidad lineal y angular del efector.

**Entrada y salida:** Entrada: q. Salida: Jacobiano 6x6, rango, valores singulares y manipulabilidad.

## Recorrido por bloques

### A. Marcos
marcos guarda T0_i para obtener posición y eje de cada junta.

**Código real** (`scripts/jacobiano.py`, líneas 12-20):

```python
def marcos(q):
    """Devuelve la lista de T_0^i para i = 0..6."""
    Ts = [np.eye(4)]
    T = np.eye(4)
    for i in range(6):
        off, d, a, al = DH_TABLE[i]
        T = T @ dh_matrix(q[i] + off, d, a, al)
        Ts.append(T.copy())
    return Ts
```

**Cómo leerlo:** Los ejes deben expresarse en el mismo marco de referencia.

### B. Columnas
jacobiano usa Jv=z x (pe-p) y Jw=z para cada junta de revolución.

**Código real** (`scripts/jacobiano.py`, líneas 23-39):

```python
def jacobiano(q):
    """
    Jacobiano geometrico 6x6 respecto de la base.

    Para una junta de revolucion i:
        Jv_i = z_{i-1} x (p_e - p_{i-1})
        Jw_i = z_{i-1}
    """
    Ts = marcos(q)
    p_e = Ts[6][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z = Ts[i][:3, 2]
        p = Ts[i][:3, 3]
        J[:3, i] = np.cross(z, p_e - p)
        J[3:, i] = z
    return J
```

**Cómo leerlo:** Cada columna es el twist del efector causado por velocidad unitaria de una junta.

### C. Contrastes
jacobiano_numerico aproxima derivadas; validacion_manual_q0 compara una columna con la geometría conocida.

**Código real** (`scripts/jacobiano.py`, líneas 42-52):

```python
def jacobiano_numerico(q, h=1e-7):
    """Jacobiano por diferencias finitas, para validar el analitico."""
    J = np.zeros((6, 6))
    T0 = forward_kinematics(q)
    for i in range(6):
        dq = np.zeros(6); dq[i] = h
        Tp = forward_kinematics(np.asarray(q) + dq)
        J[:3, i] = (Tp[:3, 3] - T0[:3, 3]) / h
        dR = (Tp[:3, :3] - T0[:3, :3]) / h @ T0[:3, :3].T
        J[3:, i] = [dR[2, 1], dR[0, 2], dR[1, 0]]
    return J
```

**Cómo leerlo:** Son controles independientes del cálculo analítico.

### D. Diagnóstico
medida_manipulabilidad y diagnostico calculan indicadores de rango y condicionamiento.

**Código real** (`scripts/jacobiano.py`, líneas 102-111):

```python
def diagnostico(q, tol=1e-6):
    J = jacobiano(q)
    sv = np.linalg.svd(J, compute_uv=False)
    return {
        "det": float(np.linalg.det(J)),
        "rango": int(np.linalg.matrix_rank(J, tol=tol)),
        "sigma_min": float(sv[-1]),
        "cond": float(sv[0] / sv[-1]) if sv[-1] > 1e-15 else np.inf,
        "w": float(np.prod(sv)),
    }
```

**Cómo leerlo:** Los usa el análisis de singularidad y las alertas de la interfaz.

## Cómo se relaciona
singularidad.py, resolver_en_vivo.py, nodo_hito2.py y graficas.py.

## Pruebas y evidencia
La notebook incluye contraste por diferencias finitas y comprobación manual en q=0.

## Pregunta para practicar defensa
> ¿Qué unidades tienen las filas lineales y angulares de J? ¿Qué indica sigma_min cercano a cero?

## Límite o precaución
La manipulabilidad de la matriz completa mezcla magnitudes lineales y angulares; no la compares como medida universal sin definir métrica.

## Ficha de repaso
- Problema que resuelve:
- Parámetros de entrada:
- Salida principal:
- Función clave:
- Prueba que lo respalda:
- Lo que no demuestra:
