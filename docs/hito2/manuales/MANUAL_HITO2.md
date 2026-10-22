# Manual completo del Hito 2

**Grupo VEVAS** · IMT-342 Robótica
Carrera de Ingeniería Mecatrónica · Departamento de Ciencias de la
Tecnología e Innovación
Universidad Católica Boliviana "San Pablo", Sede Tarija

Susann A. Baldiviezo Castellanos · Alejandro J. Bejarano Romero ·
Florencia Frigerio Arízaga · Benjamín Soruco Guzmán

Docente: M.Sc. Ing. Bernardo Quiroga Turdera

---

Este documento está escrito para que lo entienda alguien que no sabe
nada de robótica, y al mismo tiempo sirva para defender el trabajo ante
un tribunal técnico. Tiene todo: las ecuaciones, de dónde sale cada
una, cómo se demuestran, un glosario, la guía de presentación y un
ejemplo resuelto a mano de punta a punta.

## Índice

1. [El problema, en palabras simples](#1-el-problema-en-palabras-simples)
2. [Glosario](#2-glosario)
3. [El robot y sus números](#3-el-robot-y-sus-números)
4. [Bloque 1 — De los ángulos a la pose](#4-bloque-1--de-los-ángulos-a-la-pose-cinemática-directa)
5. [Bloque 2 — De la pose a los ángulos](#5-bloque-2--de-la-pose-a-los-ángulos-método-de-pieper)
6. [Bloque 3 — Verificación](#6-bloque-3--verificación-por-cinemática-directa)
7. [Bloque 4 — Trayectoria](#7-bloque-4--trayectoria-lspb)
8. [Bloque 5 — Jacobiano](#8-bloque-5--jacobiano-geométrico)
9. [Bloque 6 — Singularidades](#9-bloque-6--singularidades)
10. [Bloque 7 — Dinámica](#10-bloque-7--dinámica-y-torque)
11. [Resultados medidos](#11-resultados-medidos)
12. [Guía de presentación: formatos, casos borde y glosario](#12-guía-de-presentación-formatos-casos-borde-y-glosario)
13. [Ejemplo resuelto y defendido](#13-ejemplo-resuelto-y-defendido)
14. [Banco de preguntas](#14-banco-de-preguntas)
15. [Errores que bajan la nota](#15-errores-que-bajan-la-nota)

---

# 1. El problema, en palabras simples

Un brazo robótico industrial no se comanda diciéndole "andá a este
punto". Se comanda moviendo sus motores, uno por uno.

Imaginá tu propio brazo. Si alguien te dice "tocá ese vaso", tu cerebro
resuelve solo cuánto girar el hombro, cuánto doblar el codo y cuánto
rotar la muñeca. El robot no tiene ese cerebro: hay que calcularle los
seis ángulos. Eso es lo que hace este trabajo.

Y aparecen tres preguntas, una por cada parte del proyecto:

**Primera: ¿qué combinación de seis ángulos pone la pinza exactamente
ahí, con esa inclinación?**

La respuesta sorprende: normalmente hay **ocho** combinaciones
distintas que llegan al mismo lugar. Igual que vos podés tocar el mismo
vaso con el codo hacia arriba o hacia abajo. Pero no todas sirven: las
articulaciones reales tienen topes mecánicos, igual que tu codo no gira
360 grados. Hay que encontrar las ocho, verificarlas, descartar las
imposibles y elegir una.

**Segunda: ¿cómo llego desde donde estoy hasta esos ángulos, sin
sacudidas?**

No basta con decir "el motor 2 tiene que terminar en 40 grados". Hay
que decir cómo varía ese ángulo instante a instante, respetando qué tan
rápido puede moverse y qué tan rápido puede acelerar.

**Tercera: ¿cuánto esfuerzo le cuesta eso al motor?**

Esta pregunta decide si el movimiento es realizable o si el motor se
queda corto.

Y en el camino aparece algo llamativo: en ciertas posturas el brazo
**pierde la capacidad de moverse en alguna dirección**, aunque todos
sus motores funcionen perfectamente. Se llaman *singularidades*, y
detectarlas vale el 20% de la nota.

### Una analogía para las singularidades

Estirá el brazo completamente hacia adelante, con el codo trabado. Ahora
intentá, sin mover el hombro, alejar la mano un centímetro más. **No
podés.** No porque te falte fuerza: porque en esa postura el codo ya no
sirve para alargar el brazo. Perdiste una dirección de movimiento.

Eso es una singularidad. Y fijate en dos cosas que la gente confunde:

- **No estás paralizado.** Podés mover la mano hacia arriba, abajo, a
  los costados. Solo perdiste *una* dirección de las posibles.
- **No tiene nada que ver con la fuerza.** Podés tener el brazo
  descansado, sin peso, y la singularidad sigue ahí. Es geometría, no
  esfuerzo.

---

# 2. Glosario

Ordenado por aparición, no alfabéticamente, para que se pueda leer de
corrido.

| Término | Qué significa |
| --- | --- |
| **Grado de libertad** | Cada movimiento independiente que puede hacer el robot. El IRB 120 tiene seis: seis motores, seis ángulos |
| **Articulación / junta** | Cada punto donde el robot gira. Se numeran J1 a J6, desde la base |
| **Eslabón** | Cada tramo rígido entre dos articulaciones. El "hueso" del robot |
| **Efector / brida** | La punta del robot, donde se monta la herramienta |
| **Pose** | Posición **y** orientación juntas. No basta con dónde está la pinza: importa cómo está inclinada |
| **Configuración** | El conjunto de los seis ángulos, escrito **q** = (q₁,…,q₆) |
| **Espacio de tareas** | El mundo visto en coordenadas cartesianas: x, y, z y orientación |
| **Espacio articular** | El mundo visto en ángulos de motor |
| **Cinemática directa (FK)** | De los ángulos a la pose. Fácil: una multiplicación de matrices |
| **Cinemática inversa (IK)** | De la pose a los ángulos. Difícil: ecuaciones no lineales con varias soluciones |
| **Matriz homogénea** | Una matriz 4×4 que guarda posición y orientación en un solo objeto. Se llama T |
| **Denavit–Hartenberg (D-H)** | Convención para describir un robot con solo cuatro números por eslabón |
| **Muñeca esférica** | Cuando los tres últimos ejes se cortan en un punto. El IRB 120 la tiene, y eso simplifica todo |
| **Centro de muñeca** | Ese punto donde se cortan los tres ejes. Se llama **p**_W |
| **Rama** | Cada una de las ocho soluciones distintas que alcanzan la misma pose |
| **Candidato** | Una rama matemáticamente correcta |
| **Admisible** | Una rama que además respeta los topes mecánicos y pasa la verificación |
| **atan2** | Función que calcula un ángulo a partir de dos números, mirando el signo de cada uno. Distingue los cuatro cuadrantes |
| **SO(3)** | El conjunto de todas las rotaciones posibles en 3D. No es un espacio vectorial: no se pueden restar rotaciones |
| **Norma de Frobenius** | Una forma de medir el "tamaño" de una matriz. Se escribe ‖·‖_F |
| **LSPB** | *Linear Segment with Parabolic Blends*. Un perfil de movimiento: acelera, va a velocidad constante, frena |
| **Tiempo de mezcla (t_c)** | Cuánto dura cada tramo de aceleración |
| **Crucero** | El tramo del medio, a velocidad constante, con aceleración cero |
| **Jacobiano** | Matriz que relaciona la velocidad de los motores con la velocidad de la punta |
| **Rango de una matriz** | Cuántas direcciones independientes puede generar. Si J tiene rango 6, el robot puede moverse en las seis direcciones |
| **Singularidad** | Configuración donde el Jacobiano pierde rango: el robot pierde al menos una dirección de movimiento |
| **Valor singular (σ)** | Números que miden cuánto "estira" una matriz en cada dirección. Si uno vale cero, esa dirección se perdió |
| **SVD** | Descomposición en valores singulares. Parte una matriz en tres y revela qué direcciones conserva y cuáles no |
| **Manipulabilidad (w)** | Un número que mide qué tan lejos está el robot de una singularidad. Cero significa que está en ella |
| **Número de condición (κ)** | Cuánto se amplifican los errores numéricos. Crece sin límite al acercarse a una singularidad |
| **Pseudoinversa / mínimos cuadrados** | Qué hacer cuando un sistema no tiene solución exacta: dar la mejor aproximación |
| **Residuo** | Cuánto falta para que la aproximación sea exacta. Si es cero, era exacta |
| **Euler–Lagrange** | Método para obtener las ecuaciones de movimiento a partir de las energías |
| **Lagrangiano (L)** | Energía cinética menos energía potencial |
| **Torque / par** | El esfuerzo de giro que debe hacer un motor. Se mide en newton-metro |
| **Épsilon de máquina** | El error más chico que una computadora puede representar, alrededor de 10⁻¹⁶. Es el piso de precisión |

---

# 3. El robot y sus números

El **ABB IRB 120** es un brazo industrial de seis grados de libertad,
25 kg de peso, 580 mm de alcance y 3 kg de carga útil.

Su descripción completa cabe en una tabla de 24 números, gracias a la
convención de Denavit–Hartenberg: cuatro parámetros por articulación.

| Junta | θ (variable) | d [m] | a [m] | α [rad] | Límites [°] |
| --- | --- | --- | --- | --- | --- |
| J1 | q₁ | 0.290 | 0.000 | −π/2 | −165 a +165 |
| J2 | q₂ − π/2 | 0.000 | 0.270 | 0 | −110 a +110 |
| J3 | q₃ | 0.000 | 0.070 | −π/2 | −110 a +70 |
| J4 | q₄ | 0.302 | 0.000 | +π/2 | −160 a +160 |
| J5 | q₅ | 0.000 | 0.000 | −π/2 | −120 a +120 |
| J6 | q₆ | 0.072 | 0.000 | 0 | −400 a +400 |

**Qué significa cada parámetro:**

- **θ** es el ángulo que gira esa articulación. Es la variable.
- **d** es cuánto se desplaza a lo largo del eje de giro.
- **a** es cuánto se desplaza perpendicular al eje.
- **α** es cuánto se tuerce el eje siguiente respecto del actual.

**Tres cosas de esta tabla que se preguntan:**

**El −π/2 de la junta 2.** No es un capricho. Alinea el cero de la
convención matemática con el cero mecánico que define ABB. Sin él,
q = 0 no correspondería a la postura de referencia del fabricante.

**Los tres últimos valores de `a` son cero.** Eso es exactamente lo que
hace *esférica* a la muñeca: los ejes 4, 5 y 6 se cortan en un punto.
Sin esa propiedad, el método de Pieper no se podría aplicar.

**d₆ = 0.072 m** es la distancia desde ese punto de corte hasta la
brida. Es el número que aparece restado en la fórmula del centro de
muñeca.

---

# 4. Bloque 1 — De los ángulos a la pose (cinemática directa)

## La idea

Cada fila de la tabla D-H se convierte en una matriz 4×4 que describe
"cómo se llega del eslabón anterior al siguiente". Multiplicando las
seis, se obtiene la pose del extremo.

## La fórmula

Cada transformación elemental:

```
             ⎡ cos θ   −sin θ cos α    sin θ sin α   a cos θ ⎤
  i-1T_i  =  ⎢ sin θ    cos θ cos α   −cos θ sin α   a sin θ ⎥
             ⎢   0         sin α          cos α         d    ⎥
             ⎣   0           0              0           1    ⎦
```

Y la pose completa:

```
  ⁰T₆(q) = ⁰T₁(q₁) · ¹T₂(q₂) · ²T₃(q₃) · ³T₄(q₄) · ⁴T₅(q₅) · ⁵T₆(q₆)
```

## Cómo leer el resultado

La matriz T que sale tiene esta estructura:

```
      ⎡           │     ⎤
      ⎢    R      │  p  ⎥      R = orientación (3×3)
  T = ⎢   3×3     │ 3×1 ⎥      p = posición (3×1)
      ⎢───────────┼─────⎥
      ⎣  0  0  0  │  1  ⎦
```

La tercera columna de R, que se llama **z₆**, es la dirección hacia
donde "mira" la herramienta. La vamos a usar en el bloque siguiente.

## Validación cruzada que hicimos

Con a₂ = 0.270 m y el eslabón equivalente k = 0.310 m (sección
siguiente), el alcance máximo resulta:

```
  d_max = a₂ + k = 0.270 + 0.310 = 0.580 m
```

ABB publica **580 mm**. Dos caminos independientes, el mismo número.
Eso valida que la tabla está bien transcrita.

---

# 5. Bloque 2 — De la pose a los ángulos (método de Pieper)

Este bloque vale el **15%** de la nota.

## Por qué es difícil

Invertir la cinemática directa significa resolver seis ecuaciones no
lineales acopladas. En general no hay fórmula cerrada: hay que iterar
numéricamente, y los resultados dependen del punto de partida.

## La idea de Pieper (1968)

Donald Pieper demostró en su tesis doctoral en Stanford que **si tres
ejes consecutivos se cortan en un punto**, el problema se parte en dos
mitades independientes de tres incógnitas cada una.

El IRB 120 cumple esa condición. Por eso tiene **solución cerrada**: se
calcula directamente, sin iterar.

## Paso 1 — El centro de muñeca

```
  p_W = p_d − d₆ · z₆
```

donde z₆ es la tercera columna de la orientación deseada R_d.

**Por qué funciona.** Girar las juntas 4, 5 o 6 mueve la brida, pero
**no mueve el punto donde se cortan sus tres ejes**. Entonces p_W no
depende de q₄, q₅ ni q₆, y se puede calcular q₁, q₂ y q₃ mirando solo
p_W.

Esa es toda la magia del método: desacopla posición de orientación.

**Pregunta típica: ¿por qué la orientación deseada cambia el centro de
muñeca?** Porque z₆ sale de R_d. Si cambiás la orientación sin mover
p_d, el centro de muñeca se desplaza sobre una esfera de radio d₆.

## Paso 2 — El ángulo de la base

```
  q₁ = atan2(p_Wy , p_Wx)        o        q₁ = atan2(p_Wy , p_Wx) + π
```

Las dos soluciones son el **hombro frontal** y el **posterior**: el
robot puede alcanzar el mismo punto girándose media vuelta y plegando
el brazo hacia atrás.

## Paso 3 — El brazo como mecanismo plano

Acá hay un detalle que se pregunta. Los parámetros a₃ = 0.070 y
d₄ = 0.302 **no son colineales**: uno va perpendicular al eje y el otro
a lo largo. No se pueden sumar directamente.

Se agrupan en un eslabón equivalente:

```
  k = √(a₃² + d₄²) = √(0.070² + 0.302²) = 0.3100 m

  ψ = atan2(d₄ , a₃) = atan2(0.302 , 0.070) = 1.3430 rad = 76.95°
```

**ψ es el ángulo que ese eslabón equivalente forma con la dirección de
a₃.** Aparece sumado a q₃ en todas las fórmulas siguientes, y es la
razón de que la singularidad de codo caiga en sin(q₃ + ψ) = 0 y no en
q₃ = 0.

Con eso, el brazo se reduce a un mecanismo plano de dos barras: a₂ y k.
Y un mecanismo de dos barras se resuelve por **ley de cosenos**:

```
            r² + s² − a₂² − k²
  cos γ = ──────────────────────          γ = ± arccos(·)
                 2 · a₂ · k
```

donde r es la distancia horizontal al centro de muñeca y s la vertical,
medida desde el hombro.

Los dos signos de γ son el **codo arriba** y el **codo abajo**.

Y de ahí:

```
  q₃ = γ − ψ
```

**El alcance máximo sale solo.** Para que arccos exista, su argumento
tiene que estar entre −1 y 1. Despejando:

```
  r_W ≤ a₂ + k = 0.580 m
```

## Paso 4 — La muñeca

Con q₁, q₂, q₃ ya fijos se calcula la orientación que lograron, y se
despeja lo que falta:

```
  ³R₆ = (⁰R₃)ᵀ · R_d
```

**Por qué la inversa es la traspuesta.** Porque R pertenece a SO(3) y
sus columnas son ortonormales: R⁻¹ = Rᵀ. Es una propiedad del grupo de
rotaciones, no una aproximación.

De ³R₆ se extraen q₄, q₅ y q₆ como ángulos de Euler.

## Las ocho ramas

Tres signos binarios independientes dan 2³ = 8 configuraciones
distintas que alcanzan **exactamente** la misma pose:

| Signo | De dónde sale | Etiqueta |
| --- | --- | --- |
| q₁ o q₁+π | hacia dónde mira la base | hombro frontal / posterior |
| ±arccos del paso 3 | codo plegado arriba o abajo | codo arriba / abajo |
| ± en la extracción de muñeca | dos formas de lograr la misma orientación | muñeca invertida / no |

Las dos ramas de muñeca se relacionan así:
q₄ → q₄+π, q₅ → −q₅, q₆ → q₆+π.

**Dato para la defensa.** La etiqueta de codo **no** se lee del signo
de q₃, sino del signo de sin(q₃ + ψ). Con ψ = 76.95°, usar q₃ etiqueta
igual a ramas opuestas. Este fue uno de los errores que corregimos.

## Por qué atan2 y nunca arctan

`arctan` devuelve un ángulo entre −90° y +90°. Eso **colapsa cuadrantes
opuestos**.

Ejemplo concreto: el punto (1, 1) y el punto (−1, −1) tienen el mismo
cociente y/x = 1. `arctan` les da 45° a los dos, pero uno está en el
primer cuadrante y el otro en el tercero: apuntan en direcciones
**opuestas**.

`atan2` recibe numerador y denominador por separado, mira el signo de
cada uno, y devuelve el ángulo correcto en todo el círculo.

**Si no usáramos atan2**, las ramas de hombro frontal y posterior serían
indistinguibles, y el robot podría girar 180° en la dirección
equivocada.

## Extracción estable de la muñeca

Esta es una corrección nuestra que conviene saber defender.

La versión original calculaba q₅ = ±arccos(c₅) y declaraba singular la
rama cuando |c₅| estaba a menos de 10⁻⁹ de 1.

**El problema.** Cerca de c₅ = 1 vale la aproximación:

```
  1 − cos q₅ ≈ q₅² / 2
```

Entonces una tolerancia de 10⁻⁹ sobre c₅ equivale a una banda de:

```
  |q₅| < √(2 × 10⁻⁹) ≈ 4.5 × 10⁻⁵ rad
```

Todo ese intervalo se declaraba singular **siendo perfectamente
regular**, y las poses que caían ahí se perdían. Además arccos pierde
precisión ahí, porque su derivada se vuelve infinita en el extremo.

**La corrección.** Medir el seno directamente de la matriz:

```
  s₅ = √( (³R₆)₁₃² + (³R₆)₂₃² )

  q₅ = ± atan2(s₅ , c₅)
```

**Efecto medido:** poses con q₅ entre 10⁻⁶ y 3×10⁻⁵ pasaron de **fallar**
a resolverse. Y como efecto colateral, el error general bajó de
9.10×10⁻¹⁵ a **4.88×10⁻¹⁶ m**.

## Candidato frente a candidato admisible

Una rama es **candidata** si es matemáticamente correcta. Es
**admisible** si además:

1. Pasa la verificación por cinemática directa (bloque 3)
2. Respeta los topes mecánicos del fabricante

En nuestras pruebas, de las 8 ramas solo **3.94 en promedio** resultan
admisibles. En muchas poses sobreviven 1 o 2.

**Esta distinción es el punto de mayor contenido ingenieril del
bloque.** Separa una solución geométrica de una orden ejecutable. Un
resolvedor que devolviera una sola solución sin enumerar las ramas no
podría detectar cuándo esa única solución es inalcanzable.

## Tolerancia en los límites

Otra corrección nuestra. La comparación contra los topes era exacta,
pero una solución que debería caer **justo** sobre un límite sale de
atan2 unas unidades de precisión por fuera, y se descartaba una
configuración físicamente válida.

La corrección admite 10⁻⁹ rad (que son 5.7×10⁻⁸ grados) de holgura,
recorta ese exceso a la frontera, y **vuelve a verificar** la pose para
no introducir error por el recorte.

**Efecto medido:** sobre 1800 poses generadas exactamente sobre los
topes, los rechazos pasaron de **193 a 0**.

**Si preguntan si eso afloja el criterio:** no. 10⁻⁹ rad es ruido de
coma flotante, de un orden completamente distinto a cualquier violación
mecánica real.

## Vueltas admisibles de J6

J6 admite ±400°, pero atan2 devuelve siempre el representante principal
entre −180° y +180°. Si el robot está en q₆ = 6.0 rad (344°) y se le
pide la pose que **ya tiene**, el selector elegía −0.283 rad: un giro
completo innecesario.

La corrección expande los representantes q_i + 2πn que caen dentro de
límites **antes** de minimizar.

**Efecto medido:** movimiento innecesario de 6.283 rad → **0.000 rad**.

**Cómo defenderlo:** la función de coste siempre estuvo bien; lo que
estaba incompleto era el conjunto sobre el que minimizaba.

## Criterio de selección

Entre las admisibles se elige por mínimo desplazamiento articular
ponderado:

```
  q* = argmin_k  Σᵢ  wᵢ · |qᵢ⁽ᵏ⁾ − qᵢ_actual|

  w = [4, 3, 2, 1, 1, 0.5]
```

**Por qué pesos decrecientes.** Mover la base arrastra la inercia de
todo el brazo; girar la brida mueve solo el efector. Es un criterio
físico de mínimo esfuerzo, no un número arbitrario.

Sin pesos, el criterio podría elegir girar media vuelta la base para
ahorrar unos grados en la muñeca.

---

# 6. Bloque 3 — Verificación por cinemática directa

Este bloque vale el **10%**.

## La idea

Ninguna rama se acepta porque sí. Cada candidato se **reinyecta** en la
cinemática directa y se compara la pose que produce con la pedida.

```
  e_p = ‖ p(q⁽ᵏ⁾) − p_d ‖₂              <  10⁻⁶ m

  e_R = ‖ Rᵀ(q⁽ᵏ⁾) · R_d − I ‖_F        <  10⁻⁶
```

## Por qué RᵀR_d y no una resta — el punto fino

La tentación es escribir ‖R − R_d‖ y listo. **Está mal
conceptualmente**, y es una pregunta típica de tribunal.

**SO(3) no es un espacio vectorial.** La resta de dos matrices de
rotación **no es una rotación**: no cumple RᵀR = I ni det = 1. Medir
con una resta es medir en un espacio al que el objeto no pertenece.

En cambio **RᵀR_d sí es una rotación**: es exactamente "la rotación que
falta" para pasar de la orientación alcanzada a la deseada. Si
coinciden, esa rotación es la identidad, y la norma de Frobenius de la
diferencia contra I mide cuánto falta.

**La frase para decir en voz alta:**

> *Medimos el error como la rotación residual, porque la diferencia de
> dos rotaciones no es una rotación, pero su composición sí lo es.*

## Qué dio

Sobre 2000 poses aleatorias del espacio alcanzable:

- 8 ramas por pose, en todos los casos
- Error de posición máximo: **4.88 × 10⁻¹⁶ m**
- Error de orientación máximo: **4.55 × 10⁻¹⁴**
- Cero fallas

**Por qué el error es 10⁻¹⁶ y no 10⁻⁸.** Porque la solución es
**analítica**, no iterativa. El único residuo posible es el redondeo en
coma flotante. Un solucionador numérico daría 10⁻⁸ y dependiendo del
punto inicial.

---

# 7. Bloque 4 — Trayectoria LSPB

Este bloque vale el **15%**.

## La idea

Saber los ángulos finales no alcanza. Hay que decir **cómo** se llega.

LSPB significa *Linear Segment with Parabolic Blends*: un tramo lineal
en velocidad, con mezclas parabólicas en los extremos.

## Las tres fases

| Fase | Intervalo | q̈ | q̇ | q |
| --- | --- | --- | --- | --- |
| aceleración | 0 ≤ t < t_c | +A constante | crece lineal | parábola |
| crucero | t_c ≤ t < t_f − t_c | 0 | V constante | recta |
| desaceleración | t_f − t_c ≤ t ≤ t_f | −A constante | decrece lineal | parábola |

**Cuidado con el nombre.** El perfil **trapezoidal es el de
velocidad**. El de posición es parábola-recta-parábola. Confundirlos
está en la lista de errores del docente.

## Las condiciones de frontera

q̇(0) = q̇(t_f) = 0 se cumplen **por construcción**, no por ajuste. El
robot arranca y termina quieto.

## Dos parametrizaciones equivalentes

**Por velocidad y aceleración máximas (V, A).** Existe tramo de crucero
si:

```
  |q_f − q_i| ≥ V² / A
```

Si no se cumple, el perfil **degenera a triangular**: el robot alcanza
la velocidad máxima justo cuando ya tiene que frenar, o ni siquiera la
alcanza.

**Por tiempo total y aceleración de mezcla (t_f, q̈_c).** La
factibilidad exige:

```
            4 · |q_f − q_i|
  |q̈_c| ≥ ─────────────────
                  t_f²
```

Y el tiempo de mezcla resulta:

```
        t_f     1        ⎛      4(q_f − q_i) ⎞
  t_c = ─── − ───── · √  ⎜ t_f² − ─────────── ⎟
         2      2        ⎝            q̈_c    ⎠
```

**Cómo defender la condición de factibilidad.** Mirá el discriminante:
si la aceleración es menor que ese umbral, la raíz se vuelve negativa y
t_c **no existe en los reales**. Físicamente significa que con esa
aceleración el perfil no alcanza a cubrir la distancia en el tiempo
pedido, por más que se estire el crucero.

El caso límite, con igualdad, da t_c = t_f/2: el perfil triangular, sin
crucero.

## El caso de referencia de la cátedra

q_i = 0, q_f = 2 rad, t_f = 3 s, q̈_c = 1 rad/s²

| Magnitud | Cálculo | Valor |
| --- | --- | --- |
| aceleración mínima factible | 4(2)/9 | 0.8889 rad/s² |
| t_c | 1.5 − 0.5√(9−8) | **1.0000 s** |
| velocidad de crucero | q̈_c · t_c | **1.0000 rad/s** |
| q(2.5 s) | 2 − 0.5(0.5)² | **1.8750 rad** |
| q̇(2.5 s) | 3 − 2.5 | **0.5000 rad/s** |
| q̈(2.5 s) | — | **−1.0000 rad/s²** |

Como 1 > 0.8889, el perfil admite tramo de crucero.

## Sincronización multi-junta

Cuando varias juntas se mueven a la vez, cada una tendría su propio
tiempo. Se toma el de la **más lenta** y se reescalan las demás, de modo
que todas arrancan y llegan juntas.

Sin eso, el efector describiría una curva impredecible en vez de una
transición coordinada.

## Limitación honesta

La aceleración tiene discontinuidades de salto en los límites de fase.
Eso implica **jerk infinito** (jerk = derivada de la aceleración) en
esos instantes. Es inherente al LSPB. Un perfil de jerk acotado (curva
S) lo corrige, al costo de mayor tiempo total.

Decirlo antes de que te lo pregunten suma.

---

# 8. Bloque 5 — Jacobiano geométrico

## La idea

El Jacobiano relaciona **velocidades**: cuánto se mueve la punta del
robot cuando giran los motores.

```
  ẋ = J(q) · q̇
```

donde ẋ tiene seis componentes: tres de velocidad lineal (m/s) y tres
de velocidad angular (rad/s).

## Construcción columna por columna

Para una junta rotacional i:

```
         ⎡ z_{i-1} × (p_e − p_{i-1}) ⎤
  J_i =  ⎢                           ⎥
         ⎣        z_{i-1}            ⎦
```

donde z_{i−1} es el eje de giro de esa junta y p_{i−1} el origen de su
marco.

## La interpretación física — lo que preguntan

**El producto vectorial es el brazo de palanca.**

Si la junta i gira a velocidad unitaria, el efector se mueve con
velocidad z × r, donde r es el vector que va del eje al efector. Es
exactamente la fórmula de la velocidad de un punto en un cuerpo rígido
que rota.

La parte inferior es z_{i−1} a secas: la velocidad angular que esa
junta aporta **no depende de la distancia**.

**Dos consecuencias inmediatas:**

- Una junta lejana del efector aporta más velocidad lineal que una
  cercana.
- Las juntas de muñeca casi no mueven linealmente al efector, porque r
  es pequeño.

## Validación manual

En q = 0, la primera columna derivada a mano:

```
         ⎡0⎤   ⎡0.374⎤   ⎡  0  ⎤
  J₁ᵛ =  ⎢0⎥ × ⎢  0  ⎥ = ⎢0.374⎥
         ⎣1⎦   ⎣0.630⎦   ⎣  0  ⎦
```

El 0.374 es la distancia horizontal de la base al efector en esa
postura. El resultado apunta en **y** porque girar sobre z mueve en
dirección perpendicular al radio.

**La implementación coincide con esto en 2.08 × 10⁻¹⁸.**

## Segunda validación, independiente

Toda la matriz se validó por **diferencias finitas**: se mueve cada
ángulo un poquito y se mide cuánto se movió la punta.

Discrepancia: **3.25 × 10⁻⁸**.

**Por qué los dos números difieren en seis órdenes.** Las diferencias
finitas son un esquema de primer orden; ese 10⁻⁸ es su error de
truncamiento, no un error del Jacobiano.

**Por qué dos métodos.** Un método que no falla no prueba nada; dos
métodos independientes que coinciden sí. El manual verifica la fórmula;
las diferencias finitas verifican que el código implementa esa fórmula.

---

# 9. Bloque 6 — Singularidades

Este bloque vale el **20%**: es el de mayor peso.

## Definición

Una singularidad es una configuración donde el Jacobiano **pierde
rango**:

```
  rank J(q) < 6
```

La imagen de J deja de cubrir todo ℝ⁶: existen direcciones de velocidad
del efector que **ninguna** combinación de velocidades articulares
puede producir instantáneamente.

## Rango, no determinante — error típico

El docente lo marca explícitamente como error: *"determinante como
única singularidad — significado físico ausente"*.

**Tres razones concretas:**

1. **El determinante solo existe para J cuadrada.** En un robot
   redundante (7 grados de libertad) no hay determinante, y sigue
   habiendo singularidades.
2. **Las filas de J mezclan unidades:** las tres primeras son m/s, las
   tres últimas rad/s. El determinante hereda esa mezcla y **cambia de
   valor si se reescalan las unidades**. El rango no.
3. **El determinante dice *que* se perdió algo. El rango dice
   *cuánto*.** Rango 5 es un grado de libertad perdido; rango 4 son dos.

## Los indicadores que usamos

| Indicador | Qué es | Qué aporta |
| --- | --- | --- |
| rango | rank J | cuántas direcciones se pierden |
| σ_min | menor valor singular | qué tan cerca se está, de forma **continua** |
| w | √det(JJᵀ) | manipulabilidad de Yoshikawa: volumen del elipsoide |
| κ | σ_max / σ_min | cuánto se amplifica el error numérico |

El rango es **binario** y salta de golpe. σ_min y w son **continuos** y
avisan *antes* de llegar. Por eso el sistema usa w con umbral 10⁻³ como
alerta temprana.

## Las tres familias del IRB 120

| Familia | Condición | Qué pasa |
| --- | --- | --- |
| **muñeca** | sin q₅ = 0 | los ejes 4 y 6 se alinean; se conserva q₄+q₆ |
| **codo** | sin(q₃+ψ) = 0 | brazo extendido; las ramas de codo colapsan |
| **hombro** | x_W = y_W = 0 | centro de muñeca sobre el eje de la base; q₁ indeterminado |

## Qué se pierde y qué sobrevive

Acá está el otro error que el docente marca: *"singularidad = no se
mueve"*. **Es falso.**

La respuesta correcta usa la descomposición en valores singulares:

```
  J = U · Σ · Vᵀ
```

- La **última columna de U** (la del σ nulo) es la dirección de tarea
  inalcanzable.
- Las **columnas restantes** generan el subespacio que *sí* permanece
  accesible.

En nuestra singularidad de muñeca medimos:

| Base | σ |
| --- | --- |
| 1 | 1.8244 |
| 2 | 1.4235 |
| 3 | 1.1195 |
| 4 | 0.3458 |
| 5 | 0.1100 |
| **perdida** | **2.83 × 10⁻¹⁸** |

**La frase para la defensa:**

> *El robot pierde una dirección de seis y conserva cinco, tres de
> ellas con valor singular superior a la unidad. No se detiene: se
> degrada de 6 a 5 dimensiones.*

## Advertencia de unidades — importante

El vector singular perdido tiene norma parcial **0.897** en sus tres
primeras componentes (lineales) y **0.442** en las tres últimas
(angulares).

Es tentador concluir de ahí qué clase de movimiento se pierde. **Pero
esa comparación mezcla m/s con rad/s.** Para hacerla haría falta
declarar una longitud característica o una métrica de tarea.

Lo que **sí** es incondicional, y no depende de unidades, es la caída de
rango de 6 a 5: se pierde **una dirección del twist combinado**.

(Este fue uno de nuestros errores: afirmábamos que lo perdido era "una
rotación". Lo corregimos.)

## La amplificación de velocidad

Si se exige mantener una velocidad de tarea fija al acercarse a la
singularidad, la cinemática diferencial inversa amplifica la velocidad
articular:

```
  q̇ = J⁻¹ ẋ        ⟹        ‖q̇‖ ~ q₅^(−1.000)
```

| q₅ [rad] | σ_min | κ(J) | ‖q̇‖ [rad/s] |
| --- | --- | --- | --- |
| 10⁻¹ | 2.680e-2 | 6.80e+1 | 1.65e+1 |
| 10⁻² | 2.679e-3 | 6.81e+2 | 1.65e+2 |
| 10⁻³ | 2.678e-4 | 6.81e+3 | 1.65e+3 |
| 10⁻⁴ | 2.678e-5 | 6.81e+4 | 1.65e+4 |
| 10⁻⁵ | 2.678e-6 | 6.81e+5 | 1.65e+5 |
| 10⁻⁶ | 2.678e-7 | 6.81e+6 | 1.65e+6 |

**Alcance de este resultado — sé preciso acá.** Es un **ajuste
numérico** sobre seis décadas, para **la dirección de tarea
solicitada**. No es una demostración analítica ni una ley universal.

## La cota correcta — esto se pregunta

Una versión anterior de nuestro trabajo afirmaba ‖q̇‖ ≥ ‖ẋ‖/σ_min. **Es
falso.** Para un Jacobiano cuadrado regular las cotas correctas son:

```
     ‖ẋ‖                      ‖ẋ‖
   ───────  ≤  ‖q̇‖  ≤      ───────
    σ_max                    σ_min
```

Es decir, **σ_min acota por arriba**, no por abajo.

La expresión que explica de qué depende realmente la divergencia:

```
            (uᵢᵀ ẋ)²
  ‖q̇‖² = Σ ─────────
      i        σᵢ²
```

La divergencia depende de la **proyección de la tarea sobre el modo que
pierde capacidad**. Si la tarea no tiene componente en ese modo, el
término con σ → 0 no aparece en la suma y la velocidad permanece
acotada.

**Contraejemplo numérico:** en q = [0.3, 0.5, −0.4, 0.2, 0.001, 0.1],
eligiendo ẋ = J[:,0] resulta q̇ = [1,0,0,0,0,0], de norma **uno**. La
supuesta cota inferior daba 4189.39.

## El residuo de la pseudoinversa

El código usa `lstsq` (mínimos cuadrados) en vez de `inv`. ¿Por qué?
Porque en la singularidad exacta la inversa **no existe**, e `inv`
abortaría; `lstsq` devuelve la solución de norma mínima y permite
graficar la curva completa.

**Pero hay que declarar qué devuelve:**

| Tarea | ‖J q̇ − ẋ‖ |
| --- | --- |
| **incompatible** con la imagen de J | **0.442357** |
| **compatible** con la imagen de J | 0.000e+00 |

Ese 0.442 es **la parte de la tarea que se abandona**. Una velocidad
finita en una singularidad no significa que la tarea se haya cumplido.

## Hallazgo: la postura de reposo es singular

La manipulabilidad vale **w = 0 exactamente en t = 0** de la trayectoria
del proyecto.

No es ruido numérico: la postura de reposo del robot es q = 0, que
incluye **q₅ = 0** — es decir, la configuración de origen del IRB 120
*es* la singularidad de muñeca.

**La consecuencia hay que enunciarla con cuidado.** NO es cierto que el
robot no pueda moverse bajo control cartesiano desde el reposo: puede
ejecutar cualquier tarea **compatible** con la imagen de J, que en
q₅ = 0 sigue teniendo dimensión 5. Lo que no puede es seguir una
velocidad con componente en la dirección perdida.

Que el perfil LSPB atraviese esa configuración sin inconveniente **no es
una contradicción**: LSPB interpola en el espacio **articular** y nunca
invierte J.

---

# 10. Bloque 7 — Dinámica y torque

Este bloque vale el **15%**.

## El modelo que pide la guía

La guía oficial es explícita: *"Se utilizará el modelo reducido 1R
estudiado en clase"*.

## Derivación completa por Euler–Lagrange

Con q medido desde la **vertical inferior** y J = I_c + m·ℓc²:

**Paso 1 — Las energías**

```
  T = ½ · J · q̇²                    (energía cinética)

  V = m · g · ℓc · (1 − cos q)       (energía potencial)
```

**Paso 2 — El lagrangiano**

```
  L = T − V = ½ J q̇² − m g ℓc (1 − cos q)
```

**Paso 3 — Las tres derivadas**

```
  ∂L/∂q̇         = J q̇

  d/dt (∂L/∂q̇)  = J q̈

  ∂L/∂q          = − m g ℓc sin q
```

**Paso 4 — La ecuación de movimiento**

```
  τ = d/dt(∂L/∂q̇) − ∂L/∂q = J q̈ + m g ℓc sin q
      └──────┬──────┘   └─┬─┘
          inercial    gravitatorio
```

**Fijate en el signo.** ∂L/∂q es **negativo**, y al **restarlo** queda
positivo. Ese doble cambio de signo es donde más se equivoca la gente.

## Correspondencia angular — el punto fino

Inyectar q₂ del IRB 120 directamente en esa ecuación **sería un error**.

El modelo del docente mide desde la **vertical inferior**. En la tabla
D-H, q₂ = 0 sitúa el brazo vertical **hacia arriba**. Si se inyecta q₂
directo, el término gravitatorio sale máximo justo donde debe anularse.

**La correspondencia no se supone: se mide.** Calculando la dirección
del eslabón hombro→codo en función de q₂:

```
  φ = π − q₂        ⟹        φ̈ = − q̈₂
```

Y como sin(π − q₂) = sin(q₂):

```
  τ(t) = − J · q̈₂(t)  +  m g ℓc · sin q₂(t)
```

La tabla de verificación que imprime el cuaderno:

| q₂ [rad] | φ medido | π − q₂ | sin φ | sin q₂ |
| --- | --- | --- | --- | --- |
| −1.5 | −1.6416 | 4.6416 | −0.9975 | −0.9975 |
| −0.6 | −2.5416 | 3.7416 | −0.5646 | −0.5646 |
| 0.0 | 3.1416 | 3.1416 | 0.0000 | 0.0000 |
| 0.6 | 2.5416 | 2.5416 | 0.5646 | 0.5646 |
| 1.5 | 1.6416 | 1.6416 | 0.9975 | 0.9975 |

φ y π−q₂ difieren para q₂ < 0 porque atan2 los devuelve en ramas
distintas, pero **sin φ = sin q₂ en todos los casos**, que es lo único
que entra en la ecuación.

**Si preguntan por qué no usaron q₂ directamente:** porque el cero de
D-H y el cero del péndulo no coinciden, y renombrar una variable no
cambia dónde está su origen.

## Procedencia de los parámetros

Esto importa: la rúbrica pone en "Insuficiente" los *"parámetros sin
procedencia"*.

ABB **no publica** masas ni inercias del IRB 120, y la guía **prohíbe**
inventarlas. Se usan los valores pedagógicos que la cátedra declara:

| Parámetro | Valor | Origen |
| --- | --- | --- |
| J | 0.25 kg·m² | hoja W10-S01, Tarea 4D |
| m | 1.00 kg | hoja W10-S01, Tarea 4D |
| ℓc | 0.50 m | hoja W10-S01, Tarea 4D |
| g | 9.81 m/s² | constante |

Ninguno estimado.

## Validación contra el solucionario oficial

Evaluamos la ecuación en t = 2.5 s y la contrastamos con el desarrollo
que publicó el docente:

| Magnitud | Solucionario | Nuestro código |
| --- | --- | --- |
| q | 1.875 rad | 1.8750 rad |
| q̇ | 0.5 rad/s | 0.5000 rad/s |
| q̈ | −1 rad/s² | −1.0000 rad/s² |
| τ inercial | −0.250 N·m | −0.2500 N·m |
| τ gravedad | 4.679 N·m | 4.6798 N·m |
| **τ total** | **4.429 N·m** | **4.4298 N·m** |

**El acuerdo es dentro del redondeo.** El residuo de 8×10⁻⁴ N·m viene
de que el docente redondeó sin(1.875) ≈ 0.954 en su desarrollo. **No
digas "exacto"**: decí "coincide dentro del redondeo, y el residuo se
explica por el redondeo del seno en el desarrollo publicado".

Esta tabla es la mejor respuesta a *"¿cómo saben que está bien?"*. No es
que nos dio bien: **reproducimos el resultado que él mismo publicó**.

## El signo de la contribución inercial

En t = 2.5 s el perfil está en fase de desaceleración, así que q̈ < 0 y
el término inercial sale **negativo**: el motor **frena**, se opone al
movimiento que él mismo había impuesto.

El torque total sigue siendo positivo porque la gravedad domina.

## Velocidad constante no implica torque nulo

Durante el crucero (de t = 1.002 a t = 1.998 s) la aceleración es
**exactamente cero**, así que el término inercial se anula.

Y sin embargo |τ| va de **2.36 a 4.89 N·m**.

**La razón:** el término gravitatorio m·g·ℓc·sin q no depende de
ninguna derivada de q, **sino de la postura**. Mientras el eslabón esté
fuera de la vertical, el motor debe sostener su peso aunque no lo esté
acelerando.

De hecho el pico de torque, 4.89 N·m, ocurre en t = 1.998 s, con
**aceleración exactamente nula**.

## Cómo comparar el pico con la aceleración — precaución

En un perfil LSPB la aceleración alcanza su magnitud máxima a lo largo
de un **intervalo completo**, no en un instante aislado. Comparar el
índice de argmax|τ| contra el de argmax|q̈| produce un falso "no
coinciden", porque argmax devuelve solo el primero de muchos empatados.

**La comparación correcta se hace sobre valores, no sobre índices.**

| Trayectoria | ¿pico en aceleración máxima? | ¿tiene crucero? |
| --- | --- | --- |
| del proyecto, J2 | **sí** | no, perfil triangular |
| ejemplo de clase | **no** | sí |

La afirmación defendible es la del **ejemplo de clase**.

## Singularidad cinemática no es torque alto

Las dos nociones se confunden y conviene separarlas.

**Una singularidad** es una propiedad de la **cinemática diferencial**:
el Jacobiano pierde rango. Es geometría, y existe con independencia de
las masas: un robot sin peso, en gravedad cero, tiene exactamente las
mismas singularidades.

**Un torque alto** es una propiedad de la **dinámica**: expresa cuánto
esfuerzo exige una trayectoria dadas las inercias y la gravedad.

**Que sean independientes se comprueba en ambos sentidos:**

- El modelo 1R **no tiene singularidad alguna** —su Jacobiano es un
  escalar que nunca se anula— y aun así demanda 4.89 N·m.
- La singularidad de muñeca del IRB 120 se alcanza en posturas que no
  exigen torque gravitatorio particularmente elevado.

**Dónde sí hay vínculo, y es indirecto.** Si se exige mantener una
velocidad de tarea con componente en la dirección perdida, la
cinemática diferencial inversa amplifica q̇; la aceleración, siendo su
derivada, hereda esa amplificación; y recién entonces el término
inercial crece.

> *La singularidad no produce torque: produce velocidad articular
> ilimitada, y es esa velocidad la que se traduce en torque.*

La distinción importa porque **la mitigación es distinta**:
replanificar la trayectoria frente a redimensionar el actuador.

---

# 11. Resultados medidos

| Magnitud | Valor | Contra qué se contrastó |
| --- | --- | --- |
| Ramas de Pieper por pose | 8.00 | enumeración analítica 2³ |
| Error de posición FK, máximo | 4.88 × 10⁻¹⁶ m | épsilon de máquina |
| Error de orientación FK, máximo | 4.55 × 10⁻¹⁴ | épsilon de máquina |
| Poses sin solución, de 2000 | 0 | — |
| Ramas admisibles, media | 3.94 de 8 | límites articulares ABB |
| Alcance a₂ + k | 0.580 m | 580 mm publicados por ABB |
| Jacobiano, validación manual | 2.08 × 10⁻¹⁸ | derivación a mano en q = 0 |
| Jacobiano, diferencias finitas | 3.25 × 10⁻⁸ | truncamiento de primer orden |
| Pendiente de ‖q̇‖ contra q₅ | −1.000 | ley teórica 1/q₅ |
| σ_min en la singularidad | 2.83 × 10⁻¹⁸ | cero numérico |
| Rango de J en la singularidad | 5 de 6 | pierde una dirección |
| Residuo tarea incompatible | 0.442357 | parte abandonada |
| t_c del LSPB de clase | 1.0000 s | solucionario oficial: 1 s |
| τ(2.5 s) del modelo 1R | 4.4298 N·m | solucionario oficial: 4.429 N·m |

## Las tres correcciones de robustez

| Prueba dirigida | Antes | Después |
| --- | --- | --- |
| Poses con q₅ entre 1e-6 y 3e-5 | falla | resuelve |
| 1800 poses sobre los topes, rechazos | 193 | 0 |
| Giro de J6 ante pose ya alcanzada | 6.283 rad | 0.000 rad |
| Error posicional máximo | 9.10e-15 m | 4.88e-16 m |

**El hallazgo metodológico:** los tres fallos sobrevivieron a 2000
ensayos aleatorios. Solo aparecieron bajo pruebas **dirigidas** a
fronteras y singularidades. Una muestra uniforme del espacio de trabajo
visita las fronteras con probabilidad despreciable.

## Los tres hallazgos propios

**La configuración de origen del robot es singular.** Con q = 0 resulta
q₅ = 0, que es exactamente la condición de singularidad de muñeca. Lo
descubrimos en la gráfica de manipulabilidad.

**El alcance calculado coincide con el publicado.** a₂ + k = 0.580 m, y
ABB publica 580 mm. Dos caminos independientes, mismo número.

**En la singularidad, Pieper devuelve 7 ramas y no 8.** En la de codo
devuelve 8 pero con pares coincidentes. No es un fallo: es **la firma
de la singularidad** — el signo que distinguía dos ramas deja de
distinguir nada.

---

# 12. Guía de presentación: formatos, casos borde y glosario

El resolvedor en vivo se abre así:

```bash
cd pfi-vevas-irb120/scripts
python3 resolver_en_vivo.py
```

Y muestra:

```
  1) Matriz homogenea T_d (4x4)
  2) Posicion + cuaternion (x y z qx qy qz qw)
  3) Posicion + matriz R (3x3)
  4) Posicion + RPY en grados (x y z roll pitch yaw)
  5) Posicion + eje z6 (x y z zx zy zz)
  6) Casos borde de demostracion
  7) Glosario: que significa cada numero del informe
  0) Salir
```

El menú tiene cinco formatos de pose (1–5), los casos borde (6), el glosario de salida (7) y salir (0). Las cifras se pueden pegar en una línea o por filas: el lector pide las que falten.

**IMPORTANTE:** primero se aprieta el número de la opción y **Enter**.
Recién después se pegan las coordenadas. Si pegás las coordenadas en el
prompt `opcion>`, responde "opción no reconocida".

---

## Opción 1 — Matriz homogénea T_d

**Cuándo usarla.** Si el docente dicta la matriz 4×4 completa. Es el
formato más probable si viene de un ejercicio escrito.

**Qué se pega.** Las 16 cifras, por filas, separadas por espacios:

```
0.4085 0.8880 0.2113 0.1019
0.2993 -0.3490 0.8881 0.1140
0.8623 -0.2995 -0.4083 0.6132
0 0 0 1
```

Se puede pegar todo en una sola línea.

**Qué decir mientras tanto:**

> La matriz tiene la rotación en el bloque 3×3 de arriba a la izquierda
> y la posición en la última columna. Lo que el programa necesita es la
> tercera columna de la rotación, que es el eje de aproximación z₆, y
> la posición: con eso calcula el centro de muñeca.

**Qué revisar en la salida.** Si la matriz que dictan tiene redondeos,
el programa avisa que R no es exactamente ortonormal y la corrige por
SVD. Decilo vos:

> La matriz que dictaron viene redondeada, así que no es exactamente
> una rotación. El programa lo detecta, mide cuánto se desvía y la
> reortonormaliza por SVD. Si el error fuera grande, habría que revisar
> el dato en vez de corregirlo.

---

## Opción 2 — Posición + cuaternión

**Cuándo usarla.** Es el formato más compacto y el más usado en
robótica moderna. Siete números.

**Qué se pega:**

```
0.1019 0.1140 0.6132 0.7358 0.4034 0.3648 -0.4035
```

En ese orden: `x y z qx qy qz qw`. **La parte real va al final.**

**Qué decir:**

> Un cuaternión representa una rotación con cuatro números en vez de
> nueve. Es más compacto y no sufre el bloqueo de cardán. Nuestra
> convención pone la parte real al final; si la dictan con la parte real
> adelante, hay que reordenar.

**Qué revisar.** Si el cuaternión no está normalizado, el programa lo
normaliza solo. Mencionalo si ves que los números no suman 1 al
cuadrado.

---

## Opción 3 — Posición + matriz R

**Cuándo usarla.** Si dictan posición y orientación por separado, con la
rotación como matriz 3×3.

**Qué se pega.** Primero tres números (la posición), Enter, después
nueve (la matriz por filas).

**Qué decir:**

> Es el mismo contenido que la opción 1, pero separado. La verificación
> de ortonormalidad es igual de importante acá.

---

## Opción 4 — Posición + RPY en grados

**Cuándo usarla.** Si dictan ángulos de Roll, Pitch y Yaw. Es muy común
en ambientes industriales.

**Qué se pega:**

```
0.35 0.10 0.55 30 45 -20
```

**Qué decir — y acá hay que ser preciso:**

> RPY sin más es ambiguo: hay doce convenciones de Euler y dos
> interpretaciones, ejes fijos o ejes móviles, que dan resultados
> distintos. Nosotros usamos ángulos fijos XYZ, es decir
> R = Rz(yaw)·Ry(pitch)·Rx(roll), y el programa lo declara en pantalla.
> Si la convención del enunciado es otra, el resultado cambia.

**Eso suma puntos**: demuestra que sabés que la convención importa.

---

## Opción 5 — Posición + eje z₆

**Cuándo usarla.** Es el formato de la práctica W09-S01 del docente.
Dan la posición y hacia dónde apunta la herramienta.

**Qué se pega:**

```
0.5 0.2 0.6 1 0 0
```

**Qué decir — importante:**

> Este formato especifica solo **dos** grados de libertad de
> orientación. El giro alrededor del propio eje z₆ queda libre, así que
> la pose está parcialmente definida. El programa completa esa libertad
> con una elección declarada y lo avisa en pantalla. Si el enunciado
> fija ese giro, hay que usar otro formato.

---

## Opción 6 — Casos borde de demostración

**Cuándo usarla.** Si te dan tiempo libre, o si preguntan qué pasa en
casos difíciles. Recorre seis situaciones preparadas.

**Qué se ve y qué decir en cada una:**

| Caso | Qué sale | Qué decir |
| --- | --- | --- |
| 1. interior del espacio | 8 ramas, errores en e-16, 2 admisibles | *"Caso normal. Ocho ramas, pero solo dos ejecutables: las otras violan los topes."* |
| 2. singularidad de muñeca | **7** ramas, alerta `[!]` | *"Salen siete y no ocho. No es un fallo: en la singularidad las dos ramas de muñeca colapsan en una."* |
| 3. codo extendido | 8 ramas con pares coincidentes, con aviso | *"Las ramas de codo arriba y codo abajo se vuelven la misma configuración: el brazo está estirado y el signo del codo ya no distingue nada."* |
| 4. eje de la base | q₁ = 0 y 180, w = 0, rango 5 | *"El centro de muñeca cae sobre el eje de la base, así que q₁ queda indeterminado. El solver devuelve dos representantes de una familia continua."* |
| 5. borde del alcance | aviso "EN EL LÍMITE", resuelve igual | *"Está sobre la cáscara esférica del alcance. Es resoluble, pero el brazo queda extendido, cerca de la singularidad de codo."* |
| 6. fuera de alcance | 0 admisibles, dice cuántos mm se pasa | *"Las ocho ramas se calculan, ninguna sobrevive el filtro, y el sistema no se mueve. Falla de forma segura: sabe cuándo NO debe actuar."* |

**Este último punto es el más valioso de toda la demo.** Mostrar que el
sistema sabe cuándo no actuar vale más que tres poses exitosas.

---

# 13. Ejemplo resuelto y defendido

Este es un caso completo, con la resolución a mano y la narración de
defensa. Practicalo hasta poder hacerlo sin leer.

## El dato

El docente dicta, en formato cuaternión:

```
0.1019 0.1140 0.6132 0.7358 0.4034 0.3648 -0.4035
```

## Lo que hacés

```bash
python3 resolver_en_vivo.py
```

Apretás `2`, Enter. Pegás la línea, Enter.

## Lo que sale

```
  Pose por cuaternion
  p_d = [+0.1019, +0.1140, +0.6132] m   ||p_d|| = 0.6320 m
  centro de muneca p_W = [+0.0867, +0.0501, +0.6426]   r_W = 0.3665 m

  ramas calculadas 8    admisibles 2    rechazadas 6

   #     q1      q2      q3      q4      q5      q6    e_p [m]   e_R        w       etiqueta
   1   30.00  -39.99   25.00   50.01   59.99  -35.01  1.49e-16  1.00e-15  7.10e-03  codo arriba / muneca no invertida / hombro frontal
   2   30.00  -39.99   25.00 -129.99  -59.99  144.99  1.49e-16  1.05e-15  7.10e-03  codo arriba / muneca invertida / hombro frontal
   3   30.00   71.69 -178.90   54.22  125.13   34.39  1.22e-16  1.07e-15  6.70e-03  codo abajo ... (rechazada)
   ...

  SELECCIONADA: codo arriba / muneca no invertida / hombro frontal
  q* = [30.00, -39.99, 25.00, 50.01, 59.99, -35.01] grados
  verificacion FK: e_p = 1.495e-16 m, e_R = 1.002e-15
  manipulabilidad w = 7.0982e-03, rango(J) = 6
```

## La resolución a mano, para el pizarrón

Si te piden reconstruir el cálculo, estos son los pasos con los números
reales de este caso.

**Paso 1 — La matriz desde el cuaternión**

```
         ⎡  0.4085   0.8880   0.2113 │ 0.1019 ⎤
  T_d =  ⎢  0.2993  -0.3490   0.8881 │ 0.1140 ⎥
         ⎢  0.8623  -0.2995  -0.4083 │ 0.6132 ⎥
         ⎣    0        0        0    │   1    ⎦
```

**Paso 2 — El eje de aproximación**

z₆ es la tercera columna de R:

```
  z₆ = [0.2113 , 0.8881 , −0.4083]
```

**Paso 3 — El centro de muñeca**

```
  p_W = p_d − d₆ · z₆
      = [0.1019, 0.1140, 0.6132] − 0.072 · [0.2113, 0.8881, −0.4083]
      = [0.0867 , 0.0500 , 0.6426]  m
```

**Paso 4 — El ángulo de la base**

```
  q₁ = atan2(0.0500 , 0.0867) = 0.5236 rad = 30.00°
```

La otra rama sería 30° − 180° = −150°, que es el hombro posterior.

**Paso 5 — Coordenadas en el plano del brazo**

```
  r = √(p_Wx² + p_Wy²) = √(0.0867² + 0.0500²) = 0.1000 m

  s = p_Wz − d₁ = 0.6426 − 0.290 = 0.3526 m

  r_W = √(r² + s²) = 0.3665 m      (menor que 0.580: alcanzable)
```

**Paso 6 — Ley de cosenos**

```
        r² + s² − a₂² − k²     0.1000² + 0.3526² − 0.270² − 0.310²
  cos γ = ───────────────── = ────────────────────────────────────
             2 · a₂ · k                 2 · 0.270 · 0.310

        = −0.207058
```

```
  γ = ± arccos(−0.207058) = ± 1.7794 rad
```

**Paso 7 — El ángulo del codo**

```
  q₃ = γ − ψ = 1.7794 − 1.3430 =  0.4363 rad =   25.00°   (codo arriba)
  q₃ = −γ − ψ = −1.7794 − 1.3430 = −3.1224 rad = −178.90°  (codo abajo)
```

Y ahí está: **25.00° es exactamente lo que devolvió el programa**.

La rama de codo abajo da −178.90°, fuera del límite de J3 (−110° a
+70°), y por eso el filtro la rechaza.

## La narración de defensa

Esto es lo que decís en voz alta, en orden, mientras señalás la
pantalla. Tiene que durar entre 90 segundos y dos minutos.

> **[Señalando la línea de p_W]**
>
> Lo primero que hace el programa es calcular el centro de muñeca:
> p_W = p_d menos d₆ por z₆. Ese punto es donde se cortan los tres
> últimos ejes, y la clave es que **no depende** de las tres juntas de
> muñeca. Por eso podemos resolver las tres primeras mirando solo este
> punto. Ese es el desacoplamiento de Pieper.
>
> **[Señalando r_W = 0.3665]**
>
> Verificamos que esté dentro del alcance: 0.3665 contra el máximo de
> 0.580, que sale de sumar a₂ más el eslabón equivalente k. Ese 0.580
> coincide con los 580 milímetros que publica ABB: es una validación
> independiente de nuestra geometría.
>
> **[Señalando "ramas calculadas 8"]**
>
> Salen ocho ramas. Son tres signos binarios independientes: hombro
> frontal o posterior, codo arriba o abajo, muñeca invertida o no. Las
> ocho alcanzan exactamente la misma pose, con posturas distintas.
>
> **[Señalando "admisibles 2"]**
>
> Pero solo **dos** son ejecutables. Las otras seis violan los límites
> articulares del datasheet de ABB. Acá está el punto de más contenido
> ingenieril del bloque: distinguir una solución matemática de una
> orden ejecutable. Un resolvedor que devolviera una sola solución sin
> enumerar las ramas no podría detectar cuándo esa única solución es
> inalcanzable.
>
> **[Señalando la columna e_p]**
>
> El error de la pose alcanzada contra la pedida está en diez a la
> menos dieciséis. Eso es épsilon de máquina: la solución es analítica,
> no iterativa, así que el único residuo posible es el redondeo en coma
> flotante. Un solver numérico daría diez a la menos ocho, y dependiendo
> del punto inicial.
>
> **[Señalando e_R]**
>
> El error de orientación lo medimos como la norma de Frobenius de
> R traspuesta por R deseada menos la identidad. No como una resta,
> porque SO(3) no es un espacio vectorial: la diferencia de dos
> rotaciones no es una rotación. R traspuesta por R deseada sí lo es, y
> mide cuánto falta para la identidad.
>
> **[Señalando la rama seleccionada]**
>
> Entre las admisibles elegimos por mínimo desplazamiento articular
> ponderado, con pesos que decrecen de la base a la brida. La razón es
> física: mover la base arrastra la inercia de todo el brazo, mientras
> que girar la brida mueve solo el efector.
>
> **[Señalando w y rango]**
>
> Y verificamos la manipulabilidad: siete por diez a la menos tres, con
> rango seis. Está lejos de una singularidad, así que esta configuración
> es segura para control cartesiano.

## Si la pose cae en una singularidad

La narración cambia en el tramo final:

> **[Señalando "ramas calculadas 7"]**
>
> Acá salen **siete** ramas, no ocho. No es un fallo del algoritmo: es
> la firma de la singularidad. Cuando q₅ vale cero, las dos ramas de
> muñeca colapsan en la misma configuración, porque el signo que las
> distinguía deja de distinguir nada.
>
> **[Señalando la alerta y w = 0]**
>
> La manipulabilidad vale cero y el rango cae de seis a cinco. Eso
> significa que el robot pierde **una** dirección de movimiento
> instantáneo, no las seis. Conserva cinco, tres de ellas con valor
> singular mayor que uno.
>
> Si quisiéramos seguir una velocidad con componente en la dirección
> perdida, la cinemática diferencial inversa exigiría velocidad
> articular infinita. Pero cualquier tarea compatible con la imagen del
> Jacobiano se sigue pudiendo ejecutar.

## Si la pose está fuera de alcance

> **[Señalando el aviso de fuera de alcance]**
>
> El programa detecta **antes** de resolver que el centro de muñeca
> queda a más de 580 milímetros del hombro, y dice por cuántos
> milímetros se pasa. Las ocho ramas se calculan igual, ninguna
> sobrevive el filtro, y el sistema no se mueve.
>
> Eso es un resultado correcto, no un fallo: el sistema sabe cuándo
> **no** debe actuar.

---

# 14. Banco de preguntas

La defensa individual vale el **20%** y la guía es explícita: *"No se
permite dividir el proyecto ('yo solo hice Pieper')"*. A cualquiera le
pueden preguntar por cualquier bloque.

## G1 — Pipeline (5%)

**¿Por qué Pieper va antes que LSPB y no al revés?**
Porque LSPB interpola en el espacio articular, y para eso necesita
conocer la configuración final en ángulos. La pose cartesiana no sirve
como punto final de un perfil articular.

**¿Qué entra y qué sale de cada bloque?**
Pieper: entra T_d, salen hasta 8 configuraciones q. FK: entra q, sale T
para comparar. LSPB: entran q inicial y final, sale q(t) con sus
derivadas. Jacobiano: entra q, sale J y de ahí la velocidad cartesiana.
Singularidad: entra J, sale el rango y qué se pierde. Dinámica: entran
q y q̈, sale τ.

**¿Qué pasa si una etapa falla?**
El pipeline se detiene ahí. Sin q admisible no hay trayectoria que
generar ni torque que calcular. El sistema no se mueve, y ese es el
comportamiento correcto.

## G2 — Matemática (5%)

Tenés que poder reconstruir **al menos una** ecuación en el pizarrón.
Las tres más probables:

**Centro de muñeca:** p_W = p_d − d₆·z₆. Explicar que z₆ es la tercera
columna de R_d y que p_W no depende de las tres últimas juntas.

**Columna del Jacobiano:** J_i = [z_{i−1} × (p_e − p_{i−1}) ; z_{i−1}].
Explicar que el producto vectorial es el brazo de palanca.

**Euler–Lagrange del 1R:** la derivación completa está en la sección 10.
Practicala escrita: T, V, L, las tres derivadas, el resultado.

**¿Por qué atan2 y no arctan?**
Porque arctan colapsa cuadrantes opuestos: (1,1) y (−1,−1) dan el mismo
cociente. atan2 recibe los dos argumentos por separado, mira sus signos
y cubre todo el círculo. Sin eso, hombro frontal y posterior serían
indistinguibles.

**¿De dónde sale k = 0.310?**
De agrupar a₃ = 0.070 y d₄ = 0.302, que no son colineales, en un eslabón
equivalente: k = √(a₃² + d₄²). Y ψ = atan2(d₄, a₃) es el ángulo que
forma con a₃.

## G3 — Validación e interpretación (5%)

**¿Cómo saben que un resultado numérico es correcto?**
Tres niveles. Uno, criterio explícito de aceptación: e_p < 10⁻⁶ m.
Dos, métodos independientes que coinciden: Jacobiano manual contra
código, 2.08×10⁻¹⁸. Tres, reproducción de un resultado publicado:
τ = 4.4298 contra el 4.429 del solucionario.

**¿Por qué el error es 10⁻¹⁶ y no 10⁻⁸?**
Porque la solución es analítica, no iterativa.

**¿Por qué miden la orientación con RᵀR_d y no restando?**
Porque SO(3) no es un espacio vectorial.

**¿Qué significa físicamente que w = 0?**
Que el elipsoide de manipulabilidad se aplastó contra un plano: su
volumen es cero porque perdió una dimensión. El robot conserva
movilidad en las otras cinco.

**¿Toda velocidad diverge cerca de una singularidad?**
No. Depende de la proyección de la tarea sobre el modo cuyo valor
singular tiende a cero. Una tarea compatible permanece acotada. Y la
cota correcta es ‖ẋ‖/σ_max ≤ ‖q̇‖ ≤ ‖ẋ‖/σ_min: σ_min acota por
**arriba**.

**¿Qué devuelve una pseudoinversa ante una tarea incompatible?**
Una solución de mínimos cuadrados, no exacta. Hay que informar el
residuo: el nuestro es 0.442357.

## G4 — Transferencia (5%)

Acá preguntan sobre un bloque distinto al que programaste. Las cuatro
trampas más probables:

**"Si hay singularidad, el torque se dispara. ¿Cierto?"**
No directamente. Son fenómenos independientes. El vínculo es indirecto:
q̇ diverge, q̈ hereda la divergencia, y recién ahí el término inercial.
Contraejemplo: el modelo 1R no tiene singularidades y exige 4.89 N·m.

**"En singularidad el robot no se mueve."**
Falso. Pierde **una** dirección de seis. Rango 5, cinco direcciones
conservadas, tres con σ > 1.

**"Si la velocidad es constante, el torque es cero."**
Falso. Durante el crucero q̈ = 0 y τ va de 2.36 a 4.89 N·m, porque la
gravedad depende de la postura, no de la derivada.

**"¿El perfil LSPB es trapezoidal?"**
El de **velocidad** sí. El de posición es parábola, recta, parábola.

**¿Siempre hay ocho soluciones?**
Hay hasta ocho familias geométricas regulares. Los límites y las
singularidades cambian el conjunto admisible, y en una singularidad
pueden aparecer **familias continuas**: en q₅ = 0 se conserva q₄+q₆, y
con el centro de muñeca sobre el eje de la base q₁ queda indeterminado.

**¿Los torques son los de los motores ABB?**
No. Son los de un modelo reducido 1R con parámetros pedagógicos
declarados por la cátedra.

**¿Qué aportan las pruebas de frontera?**
Detectan fallos de tolerancia y cobertura que una muestra uniforme
visita con probabilidad despreciable. Nuestros 2000 ensayos aleatorios
pasaban sin observaciones y no detectaron ninguno de los tres fallos.

## Sobre el uso de IA

La guía lo permite explícitamente, pero advierte: *"la respuesta pasiva
'lo generó la IA' no constituye en ningún caso una justificación
técnica válida"*.

Lo que se evalúa es si podés **verificar, razonar y diagnosticar**. Si
preguntan cómo verificaron algo, la respuesta correcta nunca es
"funcionó". Es el criterio numérico, el método independiente, o el
resultado publicado contra el que contrastaron.

## Cómo repartir el estudio

La trampa de este hito es estudiar "mi parte". Sugerencia: que cada una
explique en voz alta a las otras tres el bloque que **menos** programó.
Si las cuatro pueden explicar los seis bloques, el 20% está cubierto.

---

# 15. Errores que bajan la nota

De la matriz diagnóstica W10-S01, que es documento de uso docente: son
literalmente los errores que va a buscar.

| Error | Brecha | Nuestra respuesta |
| --- | --- | --- |
| Centro de muñeca incorrecto | geometría de IK | p_W = p_d − d₆z₆, con z₆ la tercera columna de R_d |
| Se descarta una rama analítica | razonamiento de ramas | enumeramos las 8 y etiquetamos cada una |
| Cuadrante incorrecto | atan2 | atan2 en todos los ángulos, nunca arctan |
| Omite verificación de FK | disciplina de validación | criterio explícito sobre 2000 poses |
| Marcos del Jacobiano mezclados | geometría del Jacobiano | todo en el marco 0, superíndices declarados |
| Determinante como única singularidad | significado físico ausente | rango como criterio central, más σ_min, w y κ |
| "Singularidad = no se mueve" | movilidad local | mostramos las 5 direcciones que persisten |
| Límites de fase LSPB erróneos | procedimiento LSPB | t_c resuelto primero, con su condición de factibilidad |
| Llamar "trapezoidal" a la posición | confusión de perfiles | el trapezoidal es el de velocidad |
| Saltarse la derivación T, V, L | procedimiento dinámico | derivación completa documentada |
| Gravedad constante | interpretación dinámica | τ_g = mgℓc·sin q depende de la postura |
| "Singularidad = torque alto" | confusión de dominios | contraejemplo del 1R: 4.89 N·m sin singularidad |

## Errores que tuvimos nosotros y corregimos

Conviene conocerlos: poder explicar qué estaba mal y cómo se arregló
vale más que no haberlos tenido.

| Lo que afirmábamos | Por qué estaba mal | Lo que decimos ahora |
| --- | --- | --- |
| la dirección perdida es angular | 0.897 lineal contra 0.442 angular, y compararlos mezcla m/s con rad/s | se pierde una dirección del twist combinado; rango 6 → 5 |
| ‖q̇‖ ≥ ‖ẋ‖/σ_min | σ_min acota por arriba; hay contraejemplo con ‖q̇‖ = 1 | ‖ẋ‖/σ_max ≤ ‖q̇‖ ≤ ‖ẋ‖/σ_min |
| el pico de torque no cae en aceleración máxima | se comparaban índices de argmax | depende de la trayectoria; se comparan valores |
| no se puede arrancar desde HOME | se pueden ejecutar tareas compatibles con la imagen de J | solo las incompatibles quedan fuera |
| el 2R representa las juntas 2 y 3 | inyecta q₂ y q₃ sin convertir coordenadas | experimento secundario declarado; el principal es el 1R |
| divergencia analítica del torque | la derivada es parcial respecto de q₅, no total | experimento adicional con hipótesis declaradas |

**Cómo presentarlo si surge.** No como una lista de errores, sino como
lo que es: las pruebas aleatorias pasaban y aun así había fallos, de
modo que agregamos pruebas dirigidas a fronteras y singularidades. Esa
es la lección metodológica del trabajo.

## La regla que resume todo

Si la respuesta a **"¿cómo lo sabés?"** es *"porque funcionó"* o
*"porque lo dice el código"*, es insuficiente.

Tiene que ser una de estas tres:

1. **Un criterio numérico explícito** — e_p < 10⁻⁶ m
2. **Dos métodos independientes que coinciden** — Jacobiano manual
   contra código, 2.08×10⁻¹⁸
3. **Un resultado publicado que reprodujimos** — τ = 4.4298 contra el
   4.429 del solucionario
