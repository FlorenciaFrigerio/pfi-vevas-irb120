# Script 04 - `singularidad.py`: Análisis de singularidad

> **Nivel:** Núcleo Hito 2 - dominar con límites

## Para qué sirve
Estudia con SVD la movilidad que se pierde al aproximarse a q5=0 y la velocidad articular que exige una tarea particular.

**Entrada y salida:** Entrada: postura base, twist deseado y valores de q5. Salida: rango, sigma_min, condición, qdot y residuos.

## Recorrido por bloques

### A. Direcciones
direccion_perdida y direcciones_conservadas usan SVD para identificar modos perdidos y conservados.

**Código real** (`scripts/singularidad.py`, líneas 39-49):

```python
def direccion_perdida(q, tol=1e-9):
    """
    Direccion del espacio de tareas que el robot no puede recorrer.

    En una singularidad J pierde rango, asi que su imagen deja de cubrir
    R^6. El vector singular izquierdo asociado al menor valor singular
    es, justamente, la direccion de tarea que queda fuera de alcance
    instantaneo.
    """
    U, S, _ = np.linalg.svd(jacobiano(q))
    return U[:, -1], S[-1]
```

**Cómo leerlo:** Singularidad significa rango(J)<6; no significa que todo movimiento se detenga.

### B. Pseudoinversa y barrido
velocidad_requerida usa mínimos cuadrados; barrido cambia q5 y guarda las métricas.

**Código real** (`scripts/singularidad.py`, líneas 85-104):

```python
def barrido(q_base, xd, q5_vals):
    """Recorre q5 -> 0 registrando sigma_min, ||qd|| y el numero de condicion."""
    filas = []
    for q5 in q5_vals:
        q = np.array(q_base, float)
        q[4] = q5
        J = jacobiano(q)
        sv = np.linalg.svd(J, compute_uv=False)
        qd = velocidad_requerida(q, xd)
        filas.append({
            "q5": q5,
            "sigma_min": sv[-1],
            "cond": sv[0]/sv[-1] if sv[-1] > 1e-300 else np.inf,
            "rango": int(np.linalg.matrix_rank(J, tol=1e-8)),
            "det": float(np.linalg.det(J)),
            "norma_qd": float(np.linalg.norm(qd)),
            "qd4": float(qd[3]),
            "qd6": float(qd[5]),
        })
    return filas
```

**Cómo leerlo:** Una tarea fuera de la imagen de J queda con residuo.

### C. Ajuste numérico
ley_de_escalado ajusta pendiente en escala log-log para el rango estudiado.

**Código real** (`scripts/singularidad.py`, líneas 107-119):

```python
def ley_de_escalado(q5_vals, valores):
    """
    Ajusta  log|valor| = m log|q5| + b  y devuelve la pendiente m.

    Una pendiente -1 confirma divergencia como 1/q5; una pendiente -2
    confirma 1/q5^2. Es la evidencia cuantitativa de que la divergencia
    es analitica y no un artefacto numerico.
    """
    x = np.log10(np.abs(q5_vals))
    y = np.log10(np.abs(valores))
    ok = np.isfinite(x) & np.isfinite(y)
    m, b = np.polyfit(x[ok], y[ok], 1)
    return float(m), float(b)
```

**Cómo leerlo:** Describe una tendencia del experimento, no una ley universal.

### D. Torque adicional
torque_muneca estima aceleración derivando qdot respecto de q5 y aplica inercias supuestas.

**Código real** (`scripts/singularidad.py`, líneas 122-150):

```python
def torque_muneca(q_base, xd, q5_vals, v5=1.0, paso_rel=1e-3):
    """
    Torque inercial en las juntas de muneca al atravesar la singularidad.

    Escenario: el robot se aproxima a la singularidad con velocidad
    articular constante qd5 = v5 [rad/s] mientras el efector mantiene la
    velocidad de tarea xd. Entonces, por regla de la cadena,

        qdd_i = d(qd_i)/dt = [d(qd_i)/dq5] . v5

    Como qd_i ~ C/q5, su derivada va como -C/q5^2, de modo que la
    aceleracion -y con ella el torque- diverge con el CUADRADO.

        tau_i ~ I_i qdd_i          (modelo de rotor desacoplado)

    El paso de la diferencia finita es RELATIVO a q5. Con un paso fijo,
    al acercarse a la singularidad el intervalo termina cruzandola y la
    derivada se satura en un valor espurio.
    """
    taus = []
    for q5 in q5_vals:
        h = max(abs(q5) * paso_rel, 1e-18)
        qa = np.array(q_base, float); qa[4] = q5 + h
        qb = np.array(q_base, float); qb[4] = q5 - h
        qd_a = velocidad_requerida(qa, xd)
        qd_b = velocidad_requerida(qb, xd)
        qdd = (qd_a - qd_b) / (2*h) * v5
        taus.append(np.abs(I_ROTOR * qdd[[3, 4, 5]]).max())
    return np.array(taus)
```

**Cómo leerlo:** Es una ilustración condicionada, no el torque real certificado del ABB.

## Cómo se relaciona
Usa jacobiano.py; graficas.py visualiza resultados; el alcance está explicado en anexo_tecnico.md.

## Pruebas y evidencia
El anexo técnico reporta caída de rango de 6 a 5 en q5=0 y un barrido particular con velocidad creciente al acercarse a cero.

## Pregunta para practicar defensa
> ¿Por qué mínimos cuadrados puede devolver qdot finita y aun así no completar el twist pedido?

## Límite o precaución
La dirección perdida mezcla m/s y rad/s. La afirmación robusta es la caída de rango, no que el modo sea puramente angular.

## Ficha de repaso
- Problema que resuelve:
- Parámetros de entrada:
- Salida principal:
- Función clave:
- Prueba que lo respalda:
- Lo que no demuestra:
