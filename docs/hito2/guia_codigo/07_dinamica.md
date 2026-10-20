# Script 07 - `dinamica.py`: Modelo dinámico planar 2R

> **Nivel:** Complementario - comprender límites

## Para qué sirve
Implementa un modelo planar con matriz de inercia, Coriolis y gravedad para experimentación secundaria.

**Entrada y salida:** Entrada: vectores q, qdot, qddot de dos articulaciones. Salida: componentes de torque y energía.

## Recorrido por bloques

### A. Parámetros
Longitudes, masas, centros de masa e inercias se declaran al inicio.

**Código real** (`scripts/dinamica.py`, líneas 25-33):

```python

G = 9.81

# Eslabon 1 = brazo (a2), eslabon 2 = antebrazo (hasta la muneca)
L1, L2 = 0.270, 0.302
M1, M2 = 6.0, 4.0
LC1, LC2 = L1/2, L2/2
I1, I2 = M1*L1**2/12, M2*L2**2/12

```

**Cómo leerlo:** Las masas son estimaciones declaradas; no son ficha dinámica oficial del robot.

### B. Ecuación
M_matriz, C_matriz, g_vector y torque organizan tau=M*qddot+C*qdot+g.

**Código real** (`scripts/dinamica.py`, líneas 35-42):

```python
def M_matriz(q):
    """Matriz de inercia 2x2. Simetrica y definida positiva."""
    q2 = q[1]
    c2 = np.cos(q2)
    m11 = M1*LC1**2 + I1 + M2*(L1**2 + LC2**2 + 2*L1*LC2*c2) + I2
    m12 = M2*(LC2**2 + L1*LC2*c2) + I2
    m22 = M2*LC2**2 + I2
    return np.array([[m11, m12], [m12, m22]])
```

**Cómo leerlo:** Cada término corresponde a un efecto físico distinto.

### C. Trayectoria y energía
torque_trayectoria adapta una trayectoria y energia calcula energía total.

**Código real** (`scripts/dinamica.py`, líneas 74-93):

```python
def torque_trayectoria(traj, t, idx=(1, 2)):
    """
    Evalua tau(t) a lo largo de una trayectoria LSPB completa.

    idx selecciona que juntas del robot alimentan el modelo 2R;
    por defecto las juntas 2 y 3 (indices 1 y 2).
    """
    q, qd, qdd = traj(t)
    n = len(np.atleast_1d(t))
    out = {k: np.zeros((n, 2)) for k in ("tau", "inercial", "coriolis", "gravedad")}
    for j in range(n):
        qq = q[j, list(idx)]
        vv = qd[j, list(idx)]
        aa = qdd[j, list(idx)]
        tau, ti, tc, tg = torque(qq, vv, aa)
        out["tau"][j] = tau
        out["inercial"][j] = ti
        out["coriolis"][j] = tc
        out["gravedad"][j] = tg
    return out
```

**Cómo leerlo:** Las propiedades estructurales prueban consistencia interna del modelo.

## Cómo se relaciona
Lo usa graficas.py; el anexo advierte que no sustituye al modelo 1R principal.

## Pruebas y evidencia
La salida revisa simetría/positividad de M, estructura Mdot-2C y deriva de energía.

## Pregunta para practicar defensa
> ¿Qué prueba la conservación de energía simulada y qué no demuestra del ABB?

## Límite o precaución
Parámetros estimados y falta de conversión de coordenadas limitan la interpretación física.

## Ficha de repaso
- Problema que resuelve:
- Parámetros de entrada:
- Salida principal:
- Función clave:
- Prueba que lo respalda:
- Lo que no demuestra:
