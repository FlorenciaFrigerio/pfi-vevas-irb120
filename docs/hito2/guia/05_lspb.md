# Script 05 - `lspb.py`: Perfiles de trayectoria LSPB

> **Nivel:** Núcleo Hito 2 - dominar

## Para qué sirve
Genera q(t), qdot(t) y qddot(t) con fases de aceleración, crucero y desaceleración; sincroniza las seis juntas. En `demo_recorrido.py`, esta etapa muestra una trayectoria articular de seis juntas. La etapa de torque posterior usa otro perfil LSPB escalar, de referencia, para el modelo 1R.

**Entrada y salida:** Entrada: q inicial/final y límites de velocidad/aceleración, o tiempo final impuesto. Salida: perfil temporal.

## Recorrido por bloques

### A. Perfil por junta
LSPB.__init__ determina perfil trapezoidal o triangular y tiempos.

**Código real** (`scripts/lspb.py`, líneas 25-47):

```python
    def __init__(self, q0, qf, V, A):
        self.q0, self.qf = float(q0), float(qf)
        self.h = self.qf - self.q0
        self.sgn = np.sign(self.h) if self.h != 0 else 1.0
        V, A = abs(V), abs(A)

        if abs(self.h) < 1e-12:                 # sin movimiento
            self.V = self.A = 0.0
            self.tc = self.T = 0.0
            self.triangular = False
            return

        if abs(self.h) * A >= V * V:            # hay tramo de crucero
            self.V, self.A = V, A
            self.tc = V / A
            self.T = abs(self.h) / V + self.tc
            self.triangular = False
        else:                                   # perfil triangular
            self.A = A
            self.tc = np.sqrt(abs(self.h) / A)
            self.T = 2 * self.tc
            self.V = A * self.tc
            self.triangular = True
```

**Cómo leerlo:** Distancias cortas pueden impedir llegar a la velocidad de crucero.

### B. Tiempo impuesto
desde_tiempo_final resuelve tc y rechaza parámetros físicamente incompatibles.

**Código real** (`scripts/lspb.py`, líneas 50-86):

```python
    def desde_tiempo_final(cls, q0, qf, tf, acc):
        """
        Parametrizacion (tf, qdd_c) de la hoja de trabajo W09-S01.

        En vez de fijar velocidad de crucero y aceleracion, se fija el
        TIEMPO TOTAL y la aceleracion. El tiempo de mezcla sale de
        resolver la cuadratica que impone q(tf) = qf:

            tc = tf/2 - (1/2) sqrt( tf^2 - 4(qf - q0)/qdd_c )

        Condicion de factibilidad: el radicando debe ser no negativo,

            |qdd_c| >= 4 |qf - q0| / tf^2

        Si no se cumple, no existe perfil trapezoidal que recorra esa
        distancia en ese tiempo con esa aceleracion: el robot no alcanza
        a llegar. Se lanza ValueError en lugar de devolver NaN.
        """
        h = float(qf) - float(q0)
        acc = abs(float(acc))
        if abs(h) < 1e-12:
            return cls(q0, qf, 1.0, acc)

        acc_min = 4.0 * abs(h) / (tf * tf)
        if acc < acc_min - 1e-12:
            raise ValueError(
                f"Perfil no factible: con tf = {tf} s y |h| = {abs(h):.4f} rad "
                f"se requiere |qdd_c| >= {acc_min:.4f} rad/s^2, "
                f"pero se dio {acc:.4f} rad/s^2.")

        rad = tf*tf - 4.0*abs(h)/acc
        tc = 0.5*tf - 0.5*np.sqrt(max(rad, 0.0))
        V = acc * tc                      # velocidad de crucero resultante

        obj = cls(q0, qf, V, acc)
        obj.acc_min = acc_min
        return obj
```

**Cómo leerlo:** Un error explícito evita resultados NaN.

### C. Evaluación por fases
__call__ devuelve posición, velocidad y aceleración en cada intervalo.

**Código real** (`scripts/lspb.py`, líneas 88-116):

```python
    def __call__(self, t):
        """Devuelve (q, qd, qdd) evaluado en t (escalar o arreglo)."""
        t = np.atleast_1d(np.asarray(t, float))
        q = np.full_like(t, self.q0)
        qd = np.zeros_like(t)
        qdd = np.zeros_like(t)
        if self.T == 0:
            return q, qd, qdd

        s, A, V, tc, T = self.sgn, self.A, self.V, self.tc, self.T
        f1 = t < tc
        f2 = (t >= tc) & (t < T - tc)
        f3 = (t >= T - tc) & (t <= T)
        f4 = t > T

        q[f1] = self.q0 + s * 0.5 * A * t[f1]**2
        qd[f1] = s * A * t[f1]
        qdd[f1] = s * A

        q[f2] = self.q0 + s * (V * t[f2] - 0.5 * V * V / A)
        qd[f2] = s * V

        tr = T - t[f3]
        q[f3] = self.qf - s * 0.5 * A * tr**2
        qd[f3] = s * A * tr
        qdd[f3] = -s * A

        q[f4] = self.qf
        return q, qd, qdd
```

**Cómo leerlo:** q y qdot son continuas en empalmes; qddot es constante por tramos.

### D. Sincronización
LSPBSincronizado iguala el tiempo de llegada; comprobar_continuidad revisa extremos y saltos muestreados.

**Código real** (`scripts/lspb.py`, líneas 119-151):

```python
class LSPBSincronizado:
    """
    Seis perfiles LSPB con tiempo final comun.

    Se calcula el T individual de cada junta, se toma el mayor, y se
    reescalan las demas reduciendo su velocidad de crucero para que
    todas arranquen y lleguen a la vez. Asi el movimiento del efector es
    coordinado y no una sucesion de tramos descoordinados.
    """

    def __init__(self, q0, qf, V, A):
        q0, qf = np.asarray(q0, float), np.asarray(qf, float)
        V, A = np.asarray(V, float), np.asarray(A, float)
        perfiles = [LSPB(q0[i], qf[i], V[i], A[i]) for i in range(len(q0))]
        self.T = max(p.T for p in perfiles)

        self.perfiles = []
        for i, p in enumerate(perfiles):
            if p.T == 0 or abs(self.T - p.T) < 1e-12:
                self.perfiles.append(p)
                continue
            # Reescalado: se busca V' tal que el perfil dure exactamente T.
            h, a = abs(p.h), A[i]
            disc = a*a*self.T*self.T - 4*a*h
            V_new = 0.5 * (a*self.T - np.sqrt(max(disc, 0.0)))
            self.perfiles.append(LSPB(q0[i], qf[i], V_new, a))

    def __call__(self, t):
        res = [p(t) for p in self.perfiles]
        q = np.stack([r[0] for r in res], axis=-1)
        qd = np.stack([r[1] for r in res], axis=-1)
        qdd = np.stack([r[2] for r in res], axis=-1)
        return q, qd, qdd
```

**Cómo leerlo:** Es la trayectoria usada por el nodo ROS y los modelos de torque.

## Cómo se relaciona
nodo_hito2.py, dinamica_1r.py, dinamica.py y graficas.py.

## Pruebas y evidencia
El script revisa q(0), q(T), velocidades en extremos y continuidad numérica en una malla.

## Pregunta para practicar defensa
> ¿Cuál es la condición de factibilidad del tiempo final? ¿Qué se modifica al sincronizar juntas?

## Límite o precaución
LSPB no limita jerk: la aceleración cambia bruscamente entre fases.

## Ficha de repaso
- Problema que resuelve:
- Parámetros de entrada:
- Salida principal:
- Función clave:
- Prueba que lo respalda:
- Lo que no demuestra:
