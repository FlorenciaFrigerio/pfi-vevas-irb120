# Script 06 - `dinamica_1r.py`: Modelo dinámico reducido 1R

> **Nivel:** Núcleo Hito 2 - dominar modelo y alcance

## Para qué sirve
Calcula el torque pedagógico 1R prescrito para la trayectoria LSPB y separa sus contribuciones.

**Entrada y salida:** Entrada: q y qddot. Salida: torque total, inercial y gravitatorio.

## Recorrido por bloques

### A. Ecuación y parámetros
El encabezado deriva tau=J*qddot+m*g*lc*sin(q), con q medido desde vertical inferior.

**Código real** (`scripts/dinamica_1r.py`, líneas 23-52):

```python
        lc = 0.50 m           distancia al centro de masa
        g  = 9.81 m/s^2

    El angulo q se mide desde la VERTICAL INFERIOR, tambien segun esa
    hoja: por eso la energia potencial es V = m g lc (1 - cos q) y el
    termino gravitatorio resulta m g lc sin(q), nulo en q = 0.

Derivacion (Euler-Lagrange), para poder reconstruirla en la defensa

    T = (1/2) J qd^2
    V = m g lc (1 - cos q)
    L = T - V = (1/2) J qd^2 - m g lc (1 - cos q)

    dL/dqd          = J qd
    d/dt (dL/dqd)   = J qdd
    dL/dq           = -m g lc sin(q)

    tau = d/dt(dL/dqd) - dL/dq = J qdd + m g lc sin(q)

Grupo VEVAS
"""

import numpy as np

# Valores pedagogicos declarados por el docente (W10-S01, Tarea 4D).
J_INERCIA = 0.25      # kg m^2
M_MASA    = 1.00      # kg
LC_CENTRO = 0.50      # m
G_GRAV    = 9.81      # m/s^2

```

**Cómo leerlo:** J, m y lc provienen de la hoja docente, no de una identificación del IRB 120.

### B. Funciones de torque
torque_1r calcula las contribuciones; torque_sobre_lspb las evalúa sobre la trayectoria.

**Código real** (`scripts/dinamica_1r.py`, líneas 54-65):

```python
def torque_1r(q, qdd, J=J_INERCIA, m=M_MASA, lc=LC_CENTRO, g=G_GRAV):
    """
    tau = J qdd + m g lc sin(q)

    Acepta escalares o arrays; devuelve el torque total junto con sus
    dos contribuciones por separado, porque la rubrica exige "separa e
    interpreta contribuciones".
    """
    q, qdd = np.asarray(q, float), np.asarray(qdd, float)
    tau_inercial = J * qdd
    tau_gravedad = m * g * lc * np.sin(q)
    return tau_inercial + tau_gravedad, tau_inercial, tau_gravedad
```

**Cómo leerlo:** Separar términos facilita interpretar cuándo domina gravedad o aceleración.

### C. Caso de referencia
El bloque principal evalúa la trayectoria 0 a 2 rad, tf=3 s, y un punto a t=2.5 s.

**Código real** (`scripts/dinamica_1r.py`, líneas 100-141):

```python
if __name__ == "__main__":
    from lspb import LSPB

    np.set_printoptions(precision=4, suppress=True)
    print("\n   MODELO DINAMICO REDUCIDO 1R   tau = J qdd + m g lc sin(q)\n")

    print("1. Parametros declarados (W10-S01, Tarea 4D)")
    print(f"   J  = {J_INERCIA:.4f} kg m^2")
    print(f"   m  = {M_MASA:.4f} kg")
    print(f"   lc = {LC_CENTRO:.4f} m")
    print(f"   g  = {G_GRAV:.4f} m/s^2")
    print("   Ninguno es estimado: todos provienen de la hoja del docente.\n")

    perfil = LSPB.desde_tiempo_final(0.0, 2.0, tf=3.0, acc=1.0)
    print("2. Trayectoria LSPB de referencia (W10-S01, Tarea 3)")
    print(f"   q0 = 0 rad   qf = 2 rad   tf = 3 s   qddc = 1 rad/s^2")
    print(f"   tiempo de mezcla  tc = {perfil.tc:.4f} s")
    print(f"   velocidad crucero V  = {perfil.V:.4f} rad/s\n")

    t_c = 2.5
    q_c, qd_c, qdd_c = (float(v[0]) for v in perfil(t_c))
    chequeo = verificacion_analitica(q_c, qdd_c)
    print(f"3. Punto de control t = {t_c} s  (Tarea 4D de la hoja)")
    print(f"   q   = {q_c:.4f} rad")
    print(f"   qd  = {qd_c:.4f} rad/s")
    print(f"   qdd = {qdd_c:.4f} rad/s^2")
    print(f"   tau inercial  = {chequeo['inercial']:>8.4f} N.m")
    print(f"   tau gravedad  = {chequeo['gravedad']:>8.4f} N.m")
    print(f"   tau total     = {chequeo['total']:>8.4f} N.m")
    signo = "negativa (frena)" if chequeo["inercial"] < 0 else "positiva (acelera)"
    print(f"   La contribucion inercial es {signo}: en t = 2.5 s el perfil")
    print("   esta en la fase de desaceleracion, asi que qdd < 0 y el motor")
    print("   debe oponerse al movimiento que el ya habia impuesto.\n")

    t = np.linspace(0.0, perfil.T, 601)
    res = torque_sobre_lspb(perfil, t)
    i_max = int(np.argmax(np.abs(res["tau"])))
    i_qdd = int(np.argmax(np.abs(res["qdd"])))

    print("4. Interpretacion de tau(t) sobre toda la trayectoria\n")
    print(f"   |tau| maximo      = {abs(res['tau'][i_max]):.4f} N.m  en t = {t[i_max]:.3f} s")
    print(f"   |qdd| maximo      = {abs(res['qdd'][i_qdd]):.4f} rad/s^2 en t = {t[i_qdd]:.3f} s")
```

**Cómo leerlo:** En crucero qddot=0, pero puede quedar torque gravitatorio.

## Cómo se relaciona
Consume lspb.py; graficas.py dibuja las series.

## Pruebas y evidencia
La salida guardada muestra tc, velocidad de crucero y contribuciones de torque en t=2.5 s.

## Pregunta para practicar defensa
> ¿Por qué puede haber torque aunque la aceleración sea cero?

## Límite o precaución
No afirmar que este 1R representa las seis juntas ni la inercia real ABB.

## Ficha de repaso
- Problema que resuelve:
- Parámetros de entrada:
- Salida principal:
- Función clave:
- Prueba que lo respalda:
- Lo que no demuestra:
