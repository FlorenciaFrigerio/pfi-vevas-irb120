# Anexo técnico — Hito 2

Grupo VEVAS · IMT-342 Robótica · UCB Sede Tarija

Este anexo contiene el material que no entra en el informe IEEE por
razones de extensión: derivaciones completas, tablas de barrido, casos
borde y el registro de correcciones. El informe remite aquí.

Todo lo que figura en este documento es reproducible con el código
versionado en `scripts/` y el cuaderno `notebooks/Hito2_VEVAS.ipynb`.

---

## 1. Parámetros Denavit–Hartenberg

Tomados de la especificación del fabricante, documento 3HAC035960-001.

| Junta | θ | d [m] | a [m] | α [rad] | Límites [°] |
| --- | --- | --- | --- | --- | --- |
| 1 | q₁ | 0.290 | 0.000 | −π/2 | −165 a +165 |
| 2 | q₂ − π/2 | 0.000 | 0.270 | 0 | −110 a +110 |
| 3 | q₃ | 0.000 | 0.070 | −π/2 | −110 a +70 |
| 4 | q₄ | 0.302 | 0.000 | +π/2 | −160 a +160 |
| 5 | q₅ | 0.000 | 0.000 | −π/2 | −120 a +120 |
| 6 | q₆ | 0.072 | 0.000 | 0 | −400 a +400 |

Tres rasgos que importan:

- El desfase de −π/2 en la junta 2 alinea el cero de la convención con
  el cero mecánico del fabricante.
- Los tres últimos valores de `a` son nulos: eso es lo que hace esférica
  a la muñeca, y lo que habilita el método de Pieper.
- `d₆ = 0.072 m` es la distancia del punto de corte de los ejes de
  muñeca a la brida. Es el valor que se resta en la fórmula del centro
  de muñeca.

La transformación elemental de cada fila:

```
         [ cosθ   −sinθ·cosα    sinθ·sinα   a·cosθ ]
i-1T_i = [ sinθ    cosθ·cosα   −cosθ·sinα   a·sinθ ]
         [  0        sinα          cosα        d   ]
         [  0         0             0          1   ]
```

---

## 2. Las ocho ramas de Pieper

Tres signos binarios independientes:

| Signo | Origen geométrico | Etiqueta |
| --- | --- | --- |
| q₁ o q₁ + π | hacia dónde mira la base | hombro frontal / posterior |
| ± arccos en el subproblema del brazo | codo plegado arriba o abajo | codo arriba / abajo |
| ± en la extracción de muñeca | dos formas de lograr la misma orientación | muñeca invertida / no |

La relación entre las dos ramas de muñeca es
q₄ → q₄+π, q₅ → −q₅, q₆ → q₆+π.

### Etiquetado del codo

La etiqueta **no** se lee del signo de q₃. La geometría del proyecto usa
el eslabón equivalente, cuyo ángulo es γ = q₃ + ψ con ψ = 1.3430 rad. La
rama cambia con el signo de sin(γ), no al cruzar q₃ = 0.

Ejemplo concreto: para una misma pose aparecen candidatos con
q₃ = −1.0861 y q₃ = −1.6000, que corresponden a γ = +0.2570 y
γ = −0.2570. Usando el signo de q₃ ambos salían etiquetados igual.

---

## 3. Criterio de selección de rama

```
q* = argmin_k  Σ_i  w_i · |q_i^(k) − q_i^actual|

w = [4, 3, 2, 1, 1, 0.5]
```

Los pesos decrecen de la base a la brida porque mover la base arrastra
la inercia de todo el brazo, mientras que girar la brida mueve solo el
efector. No es arbitrario: es un criterio de mínimo esfuerzo.

---

## 4. Las tres correcciones de robustez

Encontradas por auditoría externa el 18 de octubre de 2026. Las tres
sobrevivieron a 2000 ensayos aleatorios y solo aparecieron bajo pruebas
dirigidas a fronteras y singularidades.

### 4.1 Extracción de muñeca: atan2, no arccos

La versión original calculaba q₅ = ±arccos(c₅) y declaraba singular la
rama cuando |c₅| estaba a menos de 1e-9 de la unidad.

El problema es geométrico. Cerca de c₅ = 1 vale

```
1 − cos q₅ ≈ q₅² / 2
```

de modo que una tolerancia de 1e-9 sobre c₅ equivale a una banda de

```
|q₅| < √(2 × 10⁻⁹) ≈ 4.5 × 10⁻⁵ rad
```

Todo ese intervalo se declaraba singular siendo regular, y las poses que
caían ahí se perdían. Además arccos pierde precisión ahí, porque su
derivada diverge en el extremo del dominio.

**Corrección.** Medir el seno directamente de la matriz:

```
s₅ = √( (R³₆)₁₃² + (R³₆)₂₃² )
q₅ = ±atan2(s₅, c₅)
```

**Efecto medido:**

| q₅ [rad] | Antes | Después |
| --- | --- | --- |
| 1e-7 | resuelve | resuelve |
| 1e-6 | **falla** | resuelve |
| 1e-5 | **falla** | resuelve |
| 3e-5 | **falla** | resuelve |
| 5e-5 | resuelve | resuelve |

Efecto colateral: el error posicional general bajó de 9.10e-15 a
4.88e-16 m, porque atan2 es más preciso también lejos de la
singularidad.

### 4.2 Tolerancia de redondeo en los límites

La comparación contra los topes era exacta. Una solución que debería
caer justo sobre un límite sale de atan2 unas unidades de precisión por
fuera, y se descartaba una configuración físicamente válida.

**Corrección.** Admitir 1e-9 rad (5.7e-8 grados), recortar ese exceso a
la frontera y **volver a verificar FK** para no introducir error por el
recorte. Esa tolerancia es de otro orden que cualquier violación
mecánica real.

**Efecto medido**, 150 configuraciones por extremo de cada junta:

| Junta | Rechazos antes (min / max) | Después |
| --- | --- | --- |
| J1 | 6 / 6 | 0 / 0 |
| J2 | 23 / 64 | 0 / 0 |
| J3 | 1 / 67 | 0 / 0 |
| J4 | 0 / 0 | 0 / 0 |
| J5 | 11 / 15 | 0 / 0 |
| J6 | 0 / 0 | 0 / 0 |
| **Total** | **193 de 1800** | **0 de 1800** |

### 4.3 Vueltas admisibles de J6

J6 admite ±400°, pero atan2 devuelve siempre el representante principal
en (−π, π]. Con el robot en q₆ = 6.0 rad, pidiéndole la pose que ya
tiene, el selector elegía −0.283 rad: un giro completo innecesario.

**Corrección.** Expandir los representantes q_i + 2πn que caen dentro de
límites **antes** de minimizar.

**Efecto medido:** movimiento innecesario de 6.283 rad → 0.000 rad.

La función de coste siempre estuvo bien; lo que estaba incompleto era el
conjunto sobre el que minimizaba.

---

## 5. Barrido de singularidad, tabla completa

| q₅ [rad] | rango | σ_min | cond(J) | ‖q̇‖ [rad/s] |
| --- | --- | --- | --- | --- |
| 1e-1 | 6 | 2.680e-2 | 6.80e+1 | 1.648e+1 |
| 1e-2 | 6 | 2.679e-3 | 6.81e+2 | 1.652e+2 |
| 1e-3 | 6 | 2.678e-4 | 6.81e+3 | 1.652e+3 |
| 1e-4 | 6 | 2.678e-5 | 6.81e+4 | 1.652e+4 |
| 1e-5 | 6 | 2.678e-6 | 6.81e+5 | 1.652e+5 |
| 1e-6 | 6 | 2.678e-7 | 6.81e+6 | 1.652e+6 |
| 0 | **5** | 2.83e-18 | — | — |

Ajuste: `‖q̇‖ ~ q₅^(−1.000)`.

**Alcance de este resultado.** Es un ajuste numérico sobre seis décadas,
para la dirección de tarea solicitada. No es una demostración analítica
ni una ley universal. La cota correcta es

```
‖ẋ‖ / σ_max  ≤  ‖q̇‖  ≤  ‖ẋ‖ / σ_min
```

y la descomposición que explica de qué depende realmente la divergencia:

```
‖q̇‖² = Σ_i ( uᵢᵀ ẋ )² / σᵢ²
```

Una tarea sin componente en el modo que pierde capacidad permanece
acotada. Contraejemplo: en q = [0.3, 0.5, −0.4, 0.2, 0.001, 0.1], con
ẋ = J[:,0] resulta ‖q̇‖ = 1 exactamente.

### Direcciones conservadas en q₅ = 0

| Base | σ | ‖v‖ | ‖ω‖ |
| --- | --- | --- | --- |
| 1 | 1.8244 | 0.329 | 0.944 |
| 2 | 1.4235 | 0.066 | 0.998 |
| 3 | 1.1195 | 0.447 | 0.894 |
| 4 | 0.3458 | 0.949 | 0.316 |
| 5 | 0.1100 | 0.992 | 0.130 |

**Advertencia de unidades.** Las normas parciales del vector singular
izquierdo perdido son 0.897 (lineal) y 0.442 (angular), pero compararlas
mezcla m/s con rad/s. Para hacerlo haría falta declarar una longitud
característica o una métrica de tarea. Lo incondicional, y lo único que
afirmamos, es la caída de rango de 6 a 5.

### Residuo de la pseudoinversa

| Tarea | ‖J q̇ − ẋ‖ |
| --- | --- |
| incompatible con la imagen de J | 0.442357 |
| compatible con la imagen de J | 0.000e+00 |

`lstsq` no resuelve la tarea incompatible: devuelve el mejor ajuste por
mínimos cuadrados. Ese 0.442 es la parte de la tarea que se abandona.
Una velocidad finita en una singularidad no significa que la tarea se
haya cumplido.

### Las tres familias singulares del IRB 120

| Familia | Condición | Qué ocurre |
| --- | --- | --- |
| muñeca | sin q₅ = 0 | los ejes 4 y 6 se alinean; se conserva q₄ + q₆ |
| codo | sin(q₃ + ψ) = 0 | brazo extendido; las ramas de codo colapsan |
| hombro | x_W = y_W = 0 | centro de muñeca sobre el eje de la base; q₁ indeterminado |

**Limitación declarada.** La de hombro se verificó solo por tendencia:
la manipulabilidad cae de 1e-2 a 1.73e-4 al aproximarse, pero el punto
exacto no se evaluó por quedar fuera del espacio alcanzable con
orientación válida. En las familias exactas el conjunto de soluciones
deja de ser discreto y pasa a contener familias continuas; su
tratamiento completo queda fuera del alcance de este hito.

---

## 6. Dinámica: derivación completa

### Euler–Lagrange del modelo 1R

Con q medido desde la vertical inferior y J = I_c + m·ℓc²:

```
T = ½ J q̇²
V = m g ℓc (1 − cos q)
L = T − V = ½ J q̇² − m g ℓc (1 − cos q)
```

Las tres derivadas:

```
∂L/∂q̇        = J q̇
d/dt(∂L/∂q̇)  = J q̈
∂L/∂q         = −m g ℓc sin q
```

Ecuación de movimiento:

```
τ = d/dt(∂L/∂q̇) − ∂L/∂q = J q̈ + m g ℓc sin q
```

El signo del término gravitatorio sale de restar una derivada negativa.
Ese doble cambio de signo es el error más común.

### Correspondencia angular

En la tabla D–H, q₂ = 0 sitúa el brazo vertical hacia **arriba**,
mientras el modelo mide desde la vertical **inferior**. Inyectar q₂
directamente daría torque gravitatorio máximo donde debe anularse.

Midiendo la dirección del eslabón hombro→codo:

| q₂ [rad] | φ medido | π − q₂ | sin φ | sin q₂ |
| --- | --- | --- | --- | --- |
| −1.5 | −1.6416 | 4.6416 | −0.9975 | −0.9975 |
| −0.6 | −2.5416 | 3.7416 | −0.5646 | −0.5646 |
| 0.0 | 3.1416 | 3.1416 | 0.0000 | 0.0000 |
| 0.6 | 2.5416 | 2.5416 | 0.5646 | 0.5646 |
| 1.5 | 1.6416 | 1.6416 | 0.9975 | 0.9975 |

φ y π − q₂ difieren para q₂ < 0 porque atan2 los devuelve en ramas
distintas, pero **sin φ = sin q₂ en todos los casos**, que es lo único
que entra en la ecuación. Por tanto:

```
τ(t) = −J q̈₂(t) + m g ℓc sin q₂(t)
```

### Procedencia de los parámetros

| Parámetro | Valor | Origen |
| --- | --- | --- |
| J | 0.25 kg·m² | hoja W10-S01, Tarea 4D |
| m | 1.00 kg | hoja W10-S01, Tarea 4D |
| ℓc | 0.50 m | hoja W10-S01, Tarea 4D |
| g | 9.81 m/s² | constante |

Ninguno estimado. ABB no publica masas ni inercias del IRB 120 y la guía
prohíbe inventarlas.

### El modelo 2R, como experimento secundario

`scripts/dinamica.py` implementa un 2R planar con M(q), C(q,q̇) y g(q),
y verifica tres propiedades estructurales:

| Propiedad | Valor medido | Debe ser |
| --- | --- | --- |
| asimetría de M | 0.000e+00 | 0 |
| autovalor mínimo de M | 0.0271 kg·m² | > 0 |
| ‖(Ṁ − 2C) + (Ṁ − 2C)ᵀ‖ | 3.911e-11 | ~0 |
| deriva de energía en 2 s | 4.303e-04 | ~0 |

**Alcance declarado.** Estas validaciones confirman la consistencia
interna del 2R consigo mismo. **No** prueban que represente las juntas 2
y 3 del IRB 120: el adaptador inyecta q₂ y q₃ sin convertir
coordenadas, y el primer ángulo del 2R estándar se mide desde la
horizontal. Por eso el modelo evaluado es el 1R.

---

## 7. Casos borde del resolvedor

`scripts/resolver_en_vivo.py --demo` recorre seis casos:

| Caso | Qué debe pasar |
| --- | --- |
| interior del espacio | 8 ramas, errores en 1e-16 |
| singularidad de muñeca | **7** ramas, no 8; alerta de singularidad |
| codo extendido | 8 ramas con pares coincidentes, con aviso |
| eje de la base | q₁ = 0 y 180; manipulabilidad 0, rango 5 |
| borde del alcance | aviso "EN EL LÍMITE", resuelve igual |
| fuera de alcance | 0 admisibles, informa cuántos mm se pasa |

Los casos 2 y 3 parecen errores si no se los espera. No lo son: son la
firma de la singularidad. Cuando el signo que distinguía dos ramas deja
de distinguir nada, las ramas colapsan.

---

## 8. Poses objetivo

La guía oficial indica procesar poses dadas por el docente. Consultado
el 16 de octubre de 2026, respondió:

> *Las poses serán dadas el día de la evaluación, es decir, deberán
> validar todas las posibles de manera que cualquier Transformada
> Homogénea que les pida sea resuelta.*

En consecuencia, la evidencia de cobertura de este hito es la validación
sobre 2000 poses aleatorias del espacio alcanzable más las pruebas
dirigidas de la sección 4, y no una tabla de poses asignadas.

Para la evaluación, `scripts/resolver_en_vivo.py` acepta T_d en cinco
formatos: matriz homogénea, posición + cuaternión, posición + matriz R,
posición + RPY en grados, y posición + eje z₆.

---

## 9. Reproducción

```bash
git clone https://github.com/FlorenciaFrigerio/pfi-vevas-irb120.git
cd pfi-vevas-irb120
git checkout feature/hito2-pieper-lspb

# Modulos individuales
cd scripts
python3 pieper.py          # verificacion FK sobre 2000 poses
python3 jacobiano.py       # validacion manual y por diferencias finitas
python3 lspb.py            # perfiles y continuidad
python3 singularidad.py    # experimento de singularidad
python3 dinamica_1r.py     # modelo 1R sobre la trayectoria

# Cuaderno completo
cd ../notebooks
python3 -m nbconvert --to notebook --execute --inplace Hito2_VEVAS.ipynb
```

Requiere Python 3 con NumPy y Matplotlib. El cuaderno localiza los
módulos de `scripts/` por la ubicación real de los archivos, de modo que
funciona tanto si se abre desde `notebooks/` como desde la raíz del
repositorio.

### Informe IEEE

El fuente está en `docs/hito2/informe_ieee/`. Se compila en Overleaf
(New Project → Upload Project → pdfLaTeX, dos pasadas) o localmente con
`texlive-publishers` instalado.

---

## 10. Registro de correcciones

| Fecha | Qué se corrigió | Evidencia |
| --- | --- | --- |
| 2026-10-18 | extracción de muñeca por atan2 | sección 4.1 |
| 2026-10-18 | tolerancia en límites articulares | sección 4.2 |
| 2026-10-18 | vueltas admisibles de J6 | sección 4.3 |
| 2026-10-18 | etiquetas de codo por sin(q₃+ψ) | sección 2 |
| 2026-10-18 | se retira la afirmación de dirección angular | sección 5 |
| 2026-10-18 | se publica el residuo de la pseudoinversa | sección 5 |
| 2026-10-19 | el 1R pasa a bloque principal de dinámica | sección 6 |
| 2026-10-19 | cota SVD corregida | sección 5 |
| 2026-10-19 | comparación torque/aceleración por valores | informe, §IV.A |
| 2026-10-19 | bootstrap de rutas del cuaderno | sección 9 |

Las afirmaciones que se retiraron, y por qué:

- *"la dirección perdida es angular"* — sus propias normas dan 0.897
  lineal contra 0.442 angular, y compararlas mezcla unidades.
- *"‖q̇‖ ≥ ‖ẋ‖/σ_min"* — σ_min acota por arriba; hay contraejemplo.
- *"el pico de torque no cae en aceleración máxima"* — depende de la
  trayectoria; se comparaban índices de argmax en lugar de valores.
- *"no se puede arrancar bajo control cartesiano desde HOME"* — se
  pueden ejecutar tareas compatibles con la imagen de J.
- *"divergencia analítica de torque"* — la derivada empleada es parcial
  respecto de q₅ sobre una familia de posturas, no la derivada total de
  una trayectoria física, y las inercias de rotor son supuestas.
