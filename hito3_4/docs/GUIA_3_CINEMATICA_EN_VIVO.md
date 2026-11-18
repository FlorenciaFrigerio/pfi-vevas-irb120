# Guía 3. Los cálculos de clase, cómo los hace la consola y cómo explicarlos

Grupo VEVAS · Directa, inversa (Pieper), selección de rama, Jacobiano, singularidades y LSPB

Esta guía explica cada cálculo desde cero, con un ejemplo con números de verdad (los mismos que aparecen en la consola) y dice en qué pestaña se ve. Si alguien del grupo tiene que explicar una parte en la presentación, esta es la guía para estudiarla.

El orden es el mismo de la barra de pasos de la consola:

```
1 Pose → 2 Pieper: 8 ramas → 3 Verificación FK → 4 Mejor rama → 5 LSPB → 6 Envío → 7 Error en vivo
```

## 1. Cinemática directa (FK): de ángulos a pose

**Pregunta que responde:** si las juntas están en q = (q1, ..., q6), ¿dónde está la punta y hacia dónde apunta?

**Idea.** Cada eslabón se describe con 4 números de la tabla D-H (a, α, d, θ). Pasar del marco i−1 al marco i es hacer 4 movimientos simples, en este orden: girar θ en z, avanzar d en z, avanzar a en x, girar α en x.

$$A_i = R_z(\theta_i)\,T_z(d_i)\,T_x(a_i)\,R_x(\alpha_i) =
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i\\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i\\
0 & \sin\alpha_i & \cos\alpha_i & d_i\\
0&0&0&1\end{bmatrix}$$

De dónde sale la matriz: multiplicando las cuatro matrices básicas. Por ejemplo, la primera fila, cuarta columna: R_z(θ) lleva el avance a·x̂ a a·(cos θ, sin θ, 0), por eso aparece a cos θ.

La pose de la brida es el producto en orden:

$$T_0^6(q) = A_1(q_1)\,A_2(q_2)\,A_3(q_3)\,A_4(q_4)\,A_5(q_5)\,A_6(q_6)$$

Tabla del IRB 120 (la del Hito 1):

| i | a [m] | α | d [m] | θ |
|---|---|---|---|---|
| 1 | 0 | −90° | 0.290 | q1 |
| 2 | 0.270 | 0 | 0 | q2 − 90° |
| 3 | 0.070 | −90° | 0 | q3 |
| 4 | 0 | +90° | 0.302 | q4 |
| 5 | 0 | −90° | 0 | q5 |
| 6 | 0 | 0 | 0.072 | q6 |

**El detalle del tool0.** El marco 6 de la tabla D-H no es igual al tool0 de ABB: están girados 180° alrededor de z6. Se comprobó con q = 0: nuestro marco 6 tiene x hacia arriba; el tool0 de ABB tiene x hacia abajo (su cuaternión es [0.7071, 0, 0.7071, 0]). Por eso:

$$T_0^{tool0} = T_0^6\,R_z(180^\circ)$$

Sin esa corrección, la posición coincide pero la orientación sale 180° distinta de la del FlexPendant.

**Ejemplo con números feos.** q = [35.5, 12.3, −27.8, 61.2, −44.4, 123.9]°.

Primera matriz (θ1 = 35.5°, α1 = −90°, d1 = 0.290):

$$A_1 = \begin{bmatrix}0.8141 & 0 & -0.5807 & 0\\0.5807 & 0 & 0.8141 & 0\\0 & -1 & 0 & 0.290\\0&0&0&1\end{bmatrix}$$

Segunda (θ2 = 12.3° − 90° = −77.7°, a2 = 0.270): cos(−77.7°) = 0.2130, sin(−77.7°) = −0.9770, entonces la traslación es (0.270·0.2130, 0.270·(−0.9770), 0) = (0.0575, −0.2638, 0) m.

Haciendo las seis y multiplicando:

$$T_0^{tool0} = \begin{bmatrix}-0.5356 & 0.0464 & 0.8432 & 0.3292\\-0.3198 & -0.9353 & -0.1517 & 0.1806\\0.7816 & -0.3509 & 0.5157 & 0.7391\\0&0&0&1\end{bmatrix}$$

La punta está en (329.2, 180.6, 739.1) mm.

**Dónde se ve en la consola:** pestaña **FK en vivo**. Ahí las seis matrices A_i y su producto se recalculan solas con los ángulos que llegan del robot (`/joint_states`), cinco veces por segundo. También en **Cálculo paso a paso** cuando se pide un movimiento por juntas.

**Cómo se validó:** contra una referencia independiente (la cadena del URDF de ROS-Industrial, que no usa D-H) en 5000 posturas al azar: error máximo 6e-16 m. Ver `docs/datos/validacion_offline/resumen.md`. Contra el controlador real: `herramientas/verificar_convencion_rws.py` (Guía 2, paso 5).

## 2. Cinemática inversa por Pieper: de pose a ángulos

**Pregunta que responde:** quiero la punta en esta pose, ¿con qué ángulos?

Es más difícil que la directa: hay que resolver ecuaciones con senos y cosenos, y puede haber varias soluciones, una o ninguna.

**La idea de Pieper.** En el IRB 120 los ejes 4, 5 y 6 se cruzan en un mismo punto: el **centro de muñeca**. Entonces el problema se parte en dos:

1. La **posición** del centro de muñeca depende solo de q1, q2 y q3 (el brazo).
2. La **orientación** que falta depende solo de q4, q5 y q6 (la muñeca).

Ejemplo de toda esta sección: la **Pose A** de la secuencia, punta en (350, 150, 400) mm con la herramienta hacia abajo (Rz = 0°, Ry = 180°, Rx = 0°), saliendo de la postura inicial q = [0, −20, 30, 0, 40, 0]°.

### Paso P1. Pose pedida a matriz

$$R = R_z(0^\circ)R_y(180^\circ)R_x(0^\circ) = \begin{bmatrix}-1&0&0\\0&1&0\\0&0&-1\end{bmatrix},\qquad
T_d = \begin{bmatrix}-1&0&0&0.350\\0&1&0&0.150\\0&0&-1&0.400\\0&0&0&1\end{bmatrix}$$

### Paso P2. Quitar la herramienta y el giro de tool0

$$T_0^6 = T_d\,(T_{tool0}^{TCP})^{-1}\,R_z(180^\circ)^{-1} = \begin{bmatrix}1&0&0&0.350\\0&-1&0&0.150\\0&0&-1&0.400\\0&0&0&1\end{bmatrix}$$

(Sin herramienta, T_tool0^TCP es la identidad.)

### Paso P3. Centro de muñeca

El centro de muñeca está d6 = 0.072 m "detrás" de la brida, sobre el eje z6:

$$p_w = p_6 - d_6\,z_6 = \begin{bmatrix}0.350\\0.150\\0.400\end{bmatrix} - 0.072\begin{bmatrix}0\\0\\-1\end{bmatrix} = \begin{bmatrix}0.350\\0.150\\0.472\end{bmatrix}\text{ m}$$

### Paso P4. q1: girar la base hacia la muñeca

Visto desde arriba, el brazo tiene que apuntar al centro de muñeca:

$$q_1 = \operatorname{atan2}(y_w, x_w) = \operatorname{atan2}(0.150, 0.350) = 23.199^\circ$$

o de espaldas: q1 = 23.199° − 180° = −156.801° (hombro atrás).

Se usa `atan2` y no `arctan(y/x)` porque `atan2` mira los signos de los dos y da el cuadrante correcto (como una brújula de 360°).

### Paso P5. q2 y q3: el triángulo hombro–codo–muñeca

Se mira el brazo de costado, en su propio plano vertical. Dos medidas:

$$r = \pm\sqrt{x_w^2 + y_w^2} = 0.38079\text{ m},\qquad s = z_w - d_1 = 0.472 - 0.290 = 0.182\text{ m}$$

r es la distancia horizontal del eje 1 a la muñeca (con signo menos para el hombro atrás) y s es la altura de la muñeca sobre el hombro.

El brazo (de largo a2 = 0.270) y el "antebrazo efectivo" (del codo a la muñeca) forman un triángulo. El antebrazo efectivo no es recto: sube a3 = 0.070 y avanza d4 = 0.302, así que mide y está inclinado:

$$L_3 = \sqrt{a_3^2 + d_4^2} = \sqrt{0.070^2 + 0.302^2} = 0.31001\text{ m},\qquad \gamma = \operatorname{atan2}(a_3, d_4) = 13.050^\circ$$

**Ley de cosenos.** Si δ es el ángulo entre el brazo y el antebrazo efectivo, la distancia hombro–muñeca al cuadrado es r² + s² = a2² + L3² + 2 a2 L3 cos δ. Despejando:

$$\cos\delta = \frac{r^2 + s^2 - a_2^2 - L_3^2}{2\,a_2\,L_3} = \frac{0.14500 + 0.03312 - 0.07290 - 0.09610}{2(0.270)(0.31001)} = 0.05448$$

Si |cos δ| > 1 la pose está **fuera de alcance**: no hay triángulo posible.

$$\delta = \pm\arccos(0.05448) = \pm 86.877^\circ$$

El signo + es **codo arriba** y el − es **codo abajo**. (Acá sí se usa arccos porque el signo se elige a mano y cos δ está lejos de ±1; cerca de ±1 el brazo está estirado o plegado, que es una singularidad.)

Con δ se sacan las juntas. q3 es δ corregido por la inclinación γ del antebrazo y por el offset de la tabla:

$$q_3 = \delta + \gamma - 90^\circ = 86.877^\circ + 13.050^\circ - 90^\circ = 9.927^\circ$$

y q2 es el ángulo hacia la muñeca menos el ángulo que agrega el codo:

$$q_2 = \operatorname{atan2}(r, s) - \operatorname{atan2}(L_3\sin\delta,\; a_2 + L_3\cos\delta) = 64.454^\circ - 47.175^\circ = 17.279^\circ$$

Comprobación rápida: con q = 0 sale r = 0.302, s = 0.340, cos δ = 0.2258, δ = 76.95° y entonces q3 = 76.95 + 13.05 − 90 = 0 y q2 = 0. Correcto.

### Paso P6. Muñeca: q4, q5, q6

Con q1, q2, q3 ya conocidos se calcula la rotación del brazo R_0^3 (FK de las tres primeras juntas). Lo que la muñeca todavía tiene que girar es:

$$R_3^6 = (R_0^3)^T R_0^6 = \begin{bmatrix}0.4202 & -0.1801 & -0.8894\\0.3939 & 0.9191 & 0\\0.8175 & -0.3503 & 0.4572\end{bmatrix}$$

Multiplicando las tres matrices de la muñeca de la tabla D-H (α4 = +90°, α5 = −90°, α6 = 0) se obtiene la forma general:

$$R_3^6 = \begin{bmatrix}\cdot & \cdot & -c_4 s_5\\ \cdot & \cdot & -s_4 s_5\\ s_5 c_6 & -s_5 s_6 & c_5\end{bmatrix}$$

Comparando término a término:

$$q_5 = \operatorname{atan2}\left(\pm\sqrt{r_{13}^2 + r_{23}^2},\; r_{33}\right) = \operatorname{atan2}(0.8894,\; 0.4572) = 62.794^\circ$$

$$q_4 = \operatorname{atan2}\left(\frac{-r_{23}}{s_5}, \frac{-r_{13}}{s_5}\right) = 0^\circ,\qquad q_6 = \operatorname{atan2}\left(\frac{-r_{32}}{s_5}, \frac{r_{31}}{s_5}\right) = 23.199^\circ$$

El ± de q5 da las dos soluciones de muñeca: **sin giro** (s5 > 0) y **girada** (s5 < 0, que equivale a q4 + 180°, −q5, q6 + 180°).

Si s5 ≈ 0 (q5 ≈ 0) los ejes 4 y 6 quedan alineados: solo se conoce q4 + q6. La consola fija q4 en su valor actual y calcula q6. Es la **singularidad de muñeca**.

**Por qué q6 tiene ±400° de rango:** la junta 6 puede dar más de una vuelta, así que un mismo ángulo tiene varios representantes (por ejemplo 23° y 23° + 360°). La consola elige el que esté dentro de límites y más cerca de la postura actual, para que no dé una vuelta completa sin necesidad (error real que tuvimos en el Hito 2).

### Las 8 ramas

2 opciones de hombro × 2 de codo × 2 de muñeca = 8. Para la Pose A:

| # | Rama | q1 | q2 | q3 | q4 | q5 | q6 | ¿Válida? |
|---|---|---|---|---|---|---|---|---|
| 0 | adelante · codo arriba · sin giro | 23.2 | 17.3 | 9.9 | 0.0 | 62.8 | 23.2 | **sí (elegida)** |
| 1 | adelante · codo arriba · girada | 23.2 | 17.3 | 9.9 | 180.0 | −62.8 | −156.8 | no: J4 fuera (±160°) |
| 2 | adelante · codo abajo · sin giro | 23.2 | 111.6 | −163.8 | 0.0 | 142.2 | 23.2 | no: J2, J3, J5 |
| 3 | adelante · codo abajo · girada | 23.2 | 111.6 | −163.8 | 180.0 | −142.2 | −156.8 | no: J2, J3, J4, J5 |
| 4 | atrás · codo arriba · sin giro | −156.8 | −111.6 | 9.9 | 180.0 | 168.3 | 23.2 | no: J2, J4, J5 |
| 5 | atrás · codo arriba · girada | −156.8 | −111.6 | 9.9 | 0.0 | −168.3 | −156.8 | no: J2, J5 |
| 6 | atrás · codo abajo · sin giro | −156.8 | −17.3 | −163.8 | 180.0 | 88.9 | 23.2 | no: J3, J4 |
| 7 | atrás · codo abajo · girada | −156.8 | −17.3 | −163.8 | 0.0 | −88.9 | −156.8 | no: J3 |

Las 8 son soluciones matemáticas correctas (todas tienen error FK de 1e-16), pero el robot real tiene topes. Por eso se filtran.

**Dónde se ve:** pestaña **Las 8 ramas**. Pasando el mouse por una fila, el robot fantasma (celeste) muestra esa postura en 3D: así se ve en la presentación por qué "codo abajo" choca o por qué "atrás" es absurdo. Haciendo clic en otra rama válida, se usa esa.

## 3. Verificación con FK

Cada rama se vuelve a meter en la cinemática directa y se compara con la pose pedida:

$$e_p = \lVert p(q^*) - p_d\rVert_2\;[\text{m}],\qquad e_R = \angle\left(R_d^T R(q^*)\right)\;[\text{rad}]$$

Para la rama elegida: e_p = 1.1e-16 m y e_R = 1.2e-16 rad. Es el error de redondeo de la computadora: la solución es exacta.

Un detalle de precisión: el protocolo escribe e_R = arccos((tr(R_e) − 1)/2). La consola calcula el mismo ángulo con `atan2(seno, coseno)` porque arccos cerca de 1 solo distingue errores mayores a ~1e-8 rad (su pendiente es infinita en 1). Está probado en `tests/test_cinematica.py`.

## 4. Elegir la mejor rama

Una rama es **válida** si cumple todo esto (los umbrales están en `config.json`):

1. existe (la pose está al alcance);
2. e_p ≤ 1e-6 m y e_R ≤ 1e-5 rad;
3. todas las juntas dentro de sus límites con 5° de margen;
4. lejos de singularidades: σ_min ≥ 0.02 (ver la sección 5).

Entre las válidas gana la que **menos mueve el robot** desde donde está:

$$\text{puntaje} = \sum_i w_i\,|q_i - q_{i,\text{actual}}|,\qquad w = [1, 1, 1, 0.6, 0.6, 0.4]$$

Los pesos hacen que mover las juntas grandes (1, 2, 3, que cargan todo el brazo) "cueste" más que mover la muñeca. El error FK no decide entre ramas válidas porque en todas es ~1e-16; lo que decide es la cercanía.

El flujo verifica los residuos de cada rama antes del envío. La selección actual filtra por error, límites y singularidad, y luego minimiza distancia ponderada con una penalización por cercanía a límites. Los residuos de todas las candidatas se muestran para explicar la elección en la demostración. La consola no deja enviar nada si no hay rama válida o si el plan no pasa la revisión.

## 5. Jacobiano y singularidades

**Pregunta que responde:** si las juntas giran con velocidades q̇, ¿a qué velocidad se mueve la punta?

$$\dot x = \begin{bmatrix}v\\\omega\end{bmatrix} = J(q)\,\dot q$$

Cada columna sale de la geometría de la junta i (eje z_(i−1) en la posición o_(i−1)):

$$J_i = \begin{bmatrix} z_{i-1} \times (p_{TCP} - o_{i-1})\\ z_{i-1}\end{bmatrix}$$

Se entiende con la mano: si solo gira la junta i, la punta da vueltas alrededor de ese eje, y su velocidad es "eje por radio" (producto vectorial).

**Verificación:** contra derivadas numéricas (diferencias finitas centrales) en `herramientas/validacion_offline.py`: error 9e-11 m/rad. La curva de error contra el paso h tiene forma de V: a la derecha manda el error de truncamiento O(h²) y a la izquierda el redondeo. El mejor paso es h ≈ 1e-5.

**Singularidad** = postura donde el Jacobiano pierde rango: hay una dirección en la que la punta no se puede mover, y para acercarse a ella las juntas tendrían que girar muy rápido. Para el IRB 120 el determinante sale en forma cerrada (verificado contra el numérico en 200 posturas):

$$\det J = -a_2\,L_3\,\sin\delta\;r_w\;\sin q_5$$

Se anula en tres casos:

| Singularidad | Condición | Qué pasa físicamente |
|---|---|---|
| Muñeca | sin q5 = 0 | Ejes 4 y 6 alineados: dos motores hacen lo mismo |
| Codo | sin δ = 0 | Brazo estirado o plegado del todo: borde del espacio de trabajo |
| Hombro | r_w = 0 | Muñeca justo sobre el eje 1: q1 queda indefinido |

**Ojo con q = 0:** la postura de calibración del IRB 120 tiene q5 = 0, así que **es singular** (rango 5). Por eso la postura inicial segura de la consola es [0, −20, 30, 0, 40, 0]°.

Resultado del barrido hacia q5 = 0 (validación offline): para girar la herramienta a 0.1 rad/s, la norma de q̇ crece como q5^(−1.00): con q5 = 1° harían falta 262 °/s, y con q5 = 0.1°, 2633 °/s. La consola detiene el movimiento con flechas cuando una junta tendría que pasar de 45 °/s.

Para comparar filas de unidades distintas (m/rad arriba, rad/rad abajo) la consola divide la parte lineal por L = 0.3 m antes de calcular los valores singulares; el rango usa la tolerancia max(m, n)·ε·σ_max.

Concepto que suelen preguntar: τ_ext = Jᵀ F (cuánto torque hace falta para aguantar una fuerza F en la punta, en estática) **no es** el torque del motor durante el movimiento, que es M(q) q̈ + C(q, q̇) q̇ + g(q). Perder rango no significa torque infinito.

**Dónde se ve:** pestaña **Jacobiano**, en vivo: los 6 valores singulares como barras, rango, número de condición, det J, los tres indicadores de singularidad y la matriz J. Además, en el 3D aparece un aviso amarillo si el robot se acerca a una singularidad.

## 6. Trayectoria LSPB: cómo se mueve en el tiempo

**Pregunta que responde:** ya sé a qué ángulos ir; ¿cómo paso de los actuales a esos, de forma suave?

LSPB (tramo lineal con mezclas parabólicas) = perfil de velocidad trapezoidal: acelera, va a velocidad constante, frena.

Para una junta que tiene que recorrer D en un tiempo t_f, con un tiempo de aceleración t_c:

| Tramo | Tiempo | Posición |
|---|---|---|
| acelera | 0 ≤ t < t_c | q = q0 + (A/2) t² |
| crucero | t_c ≤ t < t_f − t_c | q = q0 + V (t − t_c/2) |
| frena | t_f − t_c ≤ t ≤ t_f | q = q_f − (A/2)(t_f − t)² |

De dónde salen V y A: el área bajo la velocidad es el recorrido. El trapecio de altura V y bases t_f y t_f − 2t_c tiene área V (t_f − t_c) = D, entonces V = D / (t_f − t_c), y como llega a V en t_c, A = V / t_c.

**Sincronización.** Si cada junta usara su propio t_f, llegarían en momentos distintos y la punta haría "eses". Por eso las seis usan el mismo t_f y el mismo t_c = κ t_f (κ = 0.25). Cada junta tiene su V_i y A_i, y la junta más exigida decide t_f:

$$t_f = \max_i \left\{ \frac{|D_i|}{(1-\kappa)\,V_{\max,i}},\; \sqrt{\frac{|D_i|}{\kappa(1-\kappa)\,A_{\max,i}}} \right\}$$

(Se obtiene pidiendo V_i ≤ V_max,i y A_i ≤ A_max,i y despejando t_f.)

**Ejemplo:** de [0, −20, 30, 0, 40, 0]° a la rama 0 de la Pose A, al 5 % de la velocidad de hoja de datos (12.5 °/s en J1–J3) y A_max = V_max / 0.5 s:

| Junta | D [°] | V [°/s] | A [°/s²] |
|---|---|---|---|
| J1 | 23.20 | 7.78 | 7.82 |
| **J2** | **37.28** | **12.50** | 12.57 |
| J3 | −20.07 | −6.73 | −6.77 |
| J4 | 0.00 | 0.00 | 0.00 |
| J5 | 22.79 | 7.64 | 7.69 |
| J6 | 23.20 | 7.78 | 7.82 |

Manda J2 (la que más recorre): t_f = 37.28 / (0.75 · 12.5) = 3.976 s, t_c = 0.994 s. J2 va justo a su velocidad máxima permitida y las demás más despacio, para llegar todas a la vez.

Posición y velocidad son continuas; la aceleración salta en t_c y t_f − t_c (es propio de LSPB). La consola lo comprueba con muestras de 1 ms (pestaña **Trayectoria LSPB**).

**Antes de enviar**, la consola revisa la trayectoria muestra por muestra: límites con margen y que ninguna parte del brazo (codo, muñeca, brida, TCP) baje de la altura mínima. Es necesario porque aunque el inicio y el final sean seguros, en el camino el brazo puede pasar por una zona prohibida.

## 7. Error en tiempo real

Mientras el robot se mueve, la consola calcula 50 veces por segundo:

$$e_{q,i}(t) = q_{i,\text{medida}}(t) - q_{i,\text{comandada}}(t)\quad[^\circ]$$

$$e_p(t) = \lVert p(q_{\text{medida}}) - p(q_{\text{comandada}})\rVert\;[\text{mm}],\qquad e_R(t) = \angle\left(R(q_{\text{com}})^T R(q_{\text{med}})\right)\;[^\circ]$$

q_comandada es el punto de la trayectoria LSPB en ese instante; q_medida es lo que llega del robot por `/joint_states`. e_p y e_R pasan esa diferencia a la punta usando la FK: es "cuánto se separa la punta real de la que se mandó".

El error no es cero porque el robot real va un poco detrás de la orden (el driver, la red y los motores tardan). A velocidad constante, el error es aproximadamente velocidad × retardo: con 12.5 °/s y 40 ms de retardo, 0.5°. Es lo que se ve en la simulación. `analizar_sesion.py` estima ese retardo con los datos.

Semáforo: verde si el error es menor que la mitad del límite, ámbar si pasa la mitad, rojo si pasa el límite (3° por defecto). **En rojo la consola aborta sola**: cancela la trayectoria, el controlador se queda quieto y se desarma el movimiento.

## Comprobar los números de esta guía con Python

```python
import sys, math, numpy as np
sys.path.insert(0, "hito3_4")                    # desde la raíz del repo
from vevas.robot_irb120 import Robot, pose_desde_xyz_euler
from vevas.pieper import resolver, elegir_rama
from vevas.trayectoria import planificar

r = Robot()
q0 = np.radians([0, -20, 30, 0, 40, 0])
T = pose_desde_xyz_euler(350, 150, 400, 0, 180, 0)
out = resolver(r, T, q0)
print("centro de muñeca:", out["pasos"]["p_w"])
mejor, razon = elegir_rama(out["ramas"], 1e-6, 1e-5, math.radians(5), 0.02)
print(razon)
print("q* [°]:", np.round(np.degrees(mejor.q), 3))
t = planificar(q0, mejor.q, r.vel_max, 0.05, 0.5, 0.25, 1.5)
print("tf =", round(t.tf, 3), "s ·", t.motivo_tf)
```
