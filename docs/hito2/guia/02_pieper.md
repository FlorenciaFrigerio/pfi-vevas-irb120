# Script 02 - `pieper.py`: Cinemática inversa analítica de Pieper

> **Nivel:** Núcleo Hito 2 - dominar

## Para qué sirve
Calcula hasta ocho configuraciones, descarta las no ejecutables y selecciona una cercana a la postura actual.

**Entrada y salida:** Entrada: T deseada y q actual opcional. Salida: solución seleccionada, candidatas admisibles y rechazadas.

## Recorrido por bloques

### A. Centro de muñeca
wrist_center retrocede d6 a lo largo de z6. _R03 obtiene la rotación producida por las tres primeras juntas.

**Código real** (`scripts/pieper.py`, líneas 21-23):

```python
def wrist_center(T):
    """p_w = p - d6 * R[:,2]  (la junta 6 solo traslada d6 sobre z6)."""
    return T[:3, 3] - d6 * T[:3, 2]
```

**Cómo leerlo:** La posición del centro de muñeca permite resolver q1-q3 sin mezclar todavía la orientación de la muñeca.

### B. Enumerar ramas
pieper agrupa a3 y d4 en k, psi y gamma; recorre las alternativas de hombro, codo y muñeca.

**Código real** (`scripts/pieper.py`, líneas 35-86):

```python
def pieper(T_d, tol=1e-9, tol_s5=1e-12):
    """Devuelve la lista de ramas q (hasta 8) que alcanzan la pose T_d."""
    pw = wrist_center(T_d)
    xw, yw, zw = pw
    soluciones = []

    for signo_hombro in (+1, -1):
        if signo_hombro > 0:
            q1 = np.arctan2(yw, xw)
            r = np.hypot(xw, yw)
        else:
            q1 = np.arctan2(yw, xw) + np.pi
            r = -np.hypot(xw, yw)
        q1 = np.arctan2(np.sin(q1), np.cos(q1))
        s = zw - d1

        D = (r*r + s*s - a2*a2 - k*k) / (2.0*a2*k)
        if abs(D) > 1.0 + 1e-9:
            continue                      # fuera del espacio de trabajo
        D = np.clip(D, -1.0, 1.0)

        for signo_codo in (+1, -1):
            gamma = signo_codo * np.arccos(D)     # gamma = q3 + psi
            q3 = gamma - psi
            U = a2 + k*np.cos(gamma)
            V = k*np.sin(gamma)
            q2 = np.arctan2(U*r - V*s, V*r + U*s)

            R36 = _R03(q1, q2, q3).T @ T_d[:3, :3]
            c5 = np.clip(R36[2, 2], -1.0, 1.0)

            # s5 se mide directamente de la matriz, no via arccos(c5).
            # arccos pierde precision cerca de |c5| = 1: su derivada
            # diverge ahi, y una tolerancia sobre c5 declara singular una
            # banda de |q5| < 4.5e-5 rad que en realidad es regular.
            s5_abs = np.hypot(R36[0, 2], R36[1, 2])

            if s5_abs < tol_s5:                   # SINGULARIDAD q5 = 0
                q5 = 0.0 if c5 > 0 else np.pi
                q4 = 0.0
                q6 = np.arctan2(R36[1, 0], R36[0, 0]) * (1 if c5 > 0 else -1)
                soluciones.append(np.array([q1, q2, q3, q4, q5, q6]))
                continue

            for signo_muneca in (+1, -1):
                q5 = signo_muneca * np.arctan2(s5_abs, c5)
                s5 = np.sin(q5)
                q4 = np.arctan2(-R36[1, 2]/s5, -R36[0, 2]/s5)
                q6 = np.arctan2(-R36[2, 1]/s5,  R36[2, 0]/s5)
                soluciones.append(np.array([q1, q2, q3, q4, q5, q6]))

    return soluciones
```

**Cómo leerlo:** Tres elecciones binarias producen hasta 2^3=8 ramas. En singularidad algunas coinciden.

### C. Límites y vueltas
dentro_de_limites, recortar_a_limites y representantes_admisibles atienden tolerancias y vueltas 2*pi dentro de los topes.

**Código real** (`scripts/pieper.py`, líneas 115-140):

```python
def representantes_admisibles(q):
    """
    Todas las vueltas q_i + 2*pi*n que caen dentro de los limites.

    J6 admite +-400 grados, pero atan2 devuelve siempre el
    representante principal en (-pi, pi]. Sin expandir las vueltas, el
    selector puede planificar un giro completo innecesario para una
    pose que el robot YA esta alcanzando.
    """
    import itertools
    opciones = []
    for i in range(6):
        vals = []
        n = 0
        while True:
            crecio = False
            for cand in ({q[i] + 2*np.pi*n, q[i] - 2*np.pi*n} if n else {q[i]}):
                if JOINT_LIMITS[i, 0] - TOL_LIMITE <= cand <= JOINT_LIMITS[i, 1] + TOL_LIMITE:
                    vals.append(cand); crecio = True
            if n and not crecio:
                break
            n += 1
            if n > 4:
                break
        opciones.append(vals if vals else [q[i]])
    return [np.array(c) for c in itertools.product(*opciones)]
```

**Cómo leerlo:** Importa especialmente para J6, cuyos límites permiten más de una vuelta.

### D. Verificación y selección
filtrar valida números finitos, reconstrucción FK y límites. elegir minimiza el movimiento ponderado desde q_actual.

**Código real** (`scripts/pieper.py`, líneas 173-210):

```python
def filtrar(soluciones, T_d, tol_pose=1e-6):
    """
    Descarta las ramas inservibles. Tres filtros, en este orden:

      1. Numerico   -- elimina NaN, que aparecen cuando una rama cae
                       fuera del espacio de trabajo alcanzable.
      2. Cinematico -- verifica T_0^6(q) ~ T_d. Una rama que no
                       reproduce la pose es un error algebraico, no una
                       solucion alternativa.
      3. Fisico     -- descarta las que violan los limites articulares
                       del datasheet de ABB. Son matematicamente validas
                       pero mecanicamente imposibles.

    Devuelve (admisibles, rechazadas) para poder documentar el descarte
    en vez de ocultarlo.
    """
    admisibles, rechazadas = [], []
    for q in soluciones:
        if not np.all(np.isfinite(q)):
            rechazadas.append((q, "no finita"))
            continue
        ep, eR = error_pose(q, T_d)
        if max(ep, eR) > tol_pose:
            rechazadas.append((q, f"FK no coincide (dE = {ep:.2e} m)"))
            continue
        if not dentro_de_limites(q):
            fuera = [f"J{i+1}" for i in range(6)
                     if not (JOINT_LIMITS[i, 0] - TOL_LIMITE <= q[i]
                             <= JOINT_LIMITS[i, 1] + TOL_LIMITE)]
            rechazadas.append((q, "fuera de limites: " + ", ".join(fuera)))
            continue
        # Se acepta: se recorta el exceso de redondeo y se vuelve a
        # verificar FK, para no introducir error por el recorte.
        qc = recortar_a_limites(q)
        if max(error_pose(qc, T_d)) > tol_pose:
            qc = q
        admisibles.append(qc)
    return admisibles, rechazadas
```

**Cómo leerlo:** La FK es el control de calidad de cada solución inversa.

## Cómo se relaciona
Lo llaman resolver_en_vivo.py y nodo_hito2.py. Depende de fk_irb120.py.

## Pruebas y evidencia
El cuaderno y docs/hito2/resultados registran barrido aleatorio y pruebas dirigidas de límites, muñeca casi singular y vueltas de J6.

## Pregunta para practicar defensa
> ¿En qué orden se filtran las ramas? ¿Por qué el signo de q3 no basta para nombrar la rama del codo?

## Límite o precaución
La implementación actual minimiza diferencias al cuadrado. No repitas descripciones anteriores que digan valor absoluto.

## Ficha de repaso
- Problema que resuelve:
- Parámetros de entrada:
- Salida principal:
- Función clave:
- Prueba que lo respalda:
- Lo que no demuestra:
