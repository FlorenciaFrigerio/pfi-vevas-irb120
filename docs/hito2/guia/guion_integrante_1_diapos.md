# Guion de exposición — Integrante 1
## Portada y cinemática del ABB IRB 120

> **Alcance:** portada (diapositiva 1) y bloque de cinemática (diapositivas 3–7), según el reparto registrado en el proyecto. La diapositiva 2 es el mapa común; incluyo un guion breve por si te toca abrir con ella.

## La historia de tu parte

~~~mermaid
flowchart LR
  A["3. Geometría D-H"] --> B["4. Cinemática directa"]
  B --> C["5. Centro de muñeca y Pieper"]
  C --> D["6. Ramas y singularidad"]
  D --> E["7. Selección y verificación"]
~~~

**Idea central:** primero describimos la geometría; después calculamos la pose que producen unos ángulos; finalmente invertimos el problema para buscar ángulos que alcancen una pose, verificarlos y elegir una solución.

| Diapositiva | Pregunta que responde |
|---|---|
| 1 | ¿Qué proyecto presentamos? |
| 3 | ¿Cómo expresamos la geometría del robot? |
| 4 | ¿Cómo calculamos la pose desde los ángulos? |
| 5 | ¿Cómo separamos posición y orientación para resolver la inversa? |
| 6 | ¿Por qué aparecen varias soluciones y qué significa una singularidad? |
| 7 | ¿Cómo validamos y seleccionamos una solución? |

---

# Diapositiva 1 — Portada

## Guion oral

“Buenos días. Somos el equipo ____. En este hito trabajamos la cinemática del robot ABB IRB 120. Vamos a mostrar cómo modelamos su geometría, cómo calculamos la pose de la herramienta y cómo encontramos y verificamos configuraciones articulares para alcanzar una pose deseada.”

Completa el nombre del equipo y di el título tal como aparece en la portada.

**Transición:** “Para entender esos cálculos, primero necesitamos describir matemáticamente la geometría del brazo.”

---

# Diapositiva 2 — Mapa general, solo si te toca abrir

“Seguiremos un recorrido: primero la cinemática y la búsqueda de soluciones; después, la movilidad y las singularidades; finalmente, las pruebas, la trayectoria y la dinámica. Mi parte empieza con el modelo geométrico y la relación entre las juntas y la pose de la herramienta.”

El reparto actual deja la diapositiva 2 como mapa general compartido; coordina quién la presenta para evitar repetir la introducción.

---

# Diapositiva 3 — Geometría y parámetros Denavit–Hartenberg

## Explícalo de forma sencilla

El robot es una cadena de eslabones conectados por juntas. Asignamos un sistema de coordenadas a cada eslabón y usamos cuatro parámetros D-H para describir el paso de un marco al siguiente.

| Parámetro | Significado | Unidad |
|---|---|---|
| $\theta_i$ | Giro alrededor del eje $z$ de la junta | rad |
| $d_i$ | Desplazamiento a lo largo de $z$ | m |
| $a_i$ | Desplazamiento/longitud a lo largo de $x$ | m |
| $\alpha_i$ | Giro entre ejes $z$ consecutivos alrededor de $x$ | rad |

El script usa la convención D-H estándar: $R_z(\theta_i)$, luego $T_z(d_i)$, $T_x(a_i)$ y $R_x(\alpha_i)$. El orden importa porque las transformaciones espaciales no conmutan.

## Tabla del modelo que conviene poder leer

El orden guardado por el programa es **offset, d, a, alpha**. El ángulo efectivo es $\theta_i=q_i+\text{offset}_i$.

| Junta | Offset | $d$ (m) | $a$ (m) | $\alpha$ |
|---|---:|---:|---:|---:|
| J1 | 0 | 0.290 | 0 | $-\pi/2$ |
| J2 | $-\pi/2$ | 0 | 0.270 | 0 |
| J3 | 0 | 0 | 0.070 | $-\pi/2$ |
| J4 | 0 | 0.302 | 0 | $+\pi/2$ |
| J5 | 0 | 0 | 0 | $-\pi/2$ |
| J6 | 0 | 0.072 | 0 | 0 |

Las longitudes están en metros; los ángulos que usa Python están en radianes. Los límites del manual pueden estar en grados y el código los convierte.

## El offset de J2

Para J2 el programa calcula $q_2-\pi/2$. Es una corrección fija de la referencia geométrica de la tabla D-H. Se aplica dentro del modelo: no vuelvas a restar 90° a la lectura de la junta.

## Guion oral sugerido

“Modelamos el IRB 120 con parámetros Denavit–Hartenberg. Cada fila describe una junta y su eslabón: theta y d se relacionan con z; a y alpha, con x. Las distancias están en metros y los ángulos en radianes. J2 tiene un offset de menos 90 grados por la referencia de los marcos; el programa lo aplica una sola vez.”

## Muñeca esférica

Los ejes de las tres juntas de muñeca se intersectan en un punto común. Esto permite dividir el cálculo inverso: J1–J3 colocan el centro de muñeca y J4–J6 orientan la herramienta. La concurrencia es una propiedad geométrica de las tres juntas en conjunto, no de una sola fila de la tabla.

**Pregunta posible: ¿por qué importa el orden D-H?**

“Porque girar antes de trasladar puede dar una posición distinta que trasladar antes de girar.”

**Transición:** “Con las seis transformaciones ya definidas, las multiplicamos para obtener la pose completa.”

---

# Diapositiva 4 — Cinemática directa

## Qué resuelve

La cinemática directa responde: **si conozco los seis ángulos, ¿dónde está la herramienta y cómo está orientada?**

El script multiplica las transformaciones D-H:

$$
{}^0T_6(q)={}^0T_1\,{}^1T_2\,{}^2T_3\,{}^3T_4\,{}^4T_5\,{}^5T_6
$$

La matriz homogénea reúne orientación y posición:

$$
T=\begin{bmatrix}R&p\\0\;0\;0&1\end{bmatrix}
$$

$R$ es la orientación, una matriz $3\times3$; $p=[x,y,z]^T$ es la posición de la herramienta respecto a la base. La matriz completa es $4\times4$.

## Comprobación en postura cero

Para $q=0$, el modelo espera $p=[0.374,0,0.630]\text{ m}$:

- $x=d_4+d_6=0.302+0.072=0.374\text{ m}$.
- $z=d_1+a_2+a_3=0.290+0.270+0.070=0.630\text{ m}$.

También se comprueba que $R^TR\approx I$ y $\det(R)\approx1$. Esto indica que las columnas son ortogonales y que $R$ representa una rotación válida.

## Guion oral sugerido

“La cinemática directa convierte seis ángulos en la pose de la herramienta. El programa comienza con la identidad y multiplica las seis matrices desde la base hasta la brida. La matriz final contiene la orientación en R y la posición en p. Revisamos la postura cero comparando la posición con las cotas del robot y verificando que R sea una rotación válida.”

**Pregunta: ¿posición y pose son lo mismo?**

“No. Posición es dónde está el punto; pose incluye su posición y orientación.”

**Transición:** “La directa va de ángulos a pose. Ahora resolvemos el problema inverso: de una pose a posibles ángulos.”

---

# Diapositiva 5 — Centro de muñeca y método de Pieper

## Centro de muñeca

La herramienta está separada del centro de muñeca por $d_6=0.072\text{ m}$ sobre su eje $z_6$. Para ubicar el centro, retrocedemos desde la herramienta:

$$
p_W=p-d_6z_6
$$

En la matriz de orientación, $z_6$ es la tercera columna de $R$; el código la obtiene con **T[:3, 2]**. Se resta porque la posición objetivo es la de la herramienta y necesitamos volver al punto donde se intersectan los ejes de muñeca.

## Separar posición y orientación

La muñeca esférica permite resolver la inversa en dos pasos:

1. Con el centro de muñeca se calculan $q_1,q_2,q_3$.
2. Con la orientación que queda se calculan $q_4,q_5,q_6$.

## Eslabón equivalente

En el plano del brazo, $a_3=0.070\text{ m}$ y $d_4=0.302\text{ m}$ son dos componentes geométricas. Se combinan para resolver el triángulo del codo:

$$
k=\sqrt{a_3^2+d_4^2}\approx0.310\text{ m},\qquad
\psi=\operatorname{atan2}(d_4,a_3)\approx76.95^\circ
$$

$k$ y $\psi$ son una representación geométrica auxiliar, no una pieza física añadida. El ángulo del triángulo es $\gamma=q_3+\psi$. Con $a_2=0.270\text{ m}$, el alcance geométrico máximo aproximado al centro de muñeca es $a_2+k\approx0.580\text{ m}$. Estar dentro de esa distancia no garantiza una pose completa válida: también cuentan orientación y límites articulares.

## Guion oral sugerido

“Para resolver la inversa, primero quitamos el desplazamiento final de la herramienta y encontramos el centro de muñeca. Como los ejes de muñeca se cruzan, separamos posición y orientación: las tres primeras juntas ubican el centro y las tres últimas orientan la herramienta. Para el triángulo del codo combinamos a3 y d4 en un eslabón equivalente de 31 centímetros; es una simplificación matemática, no una pieza nueva.”

**Pregunta: ¿el centro de muñeca es siempre el TCP?**

“No necesariamente. El TCP puede estar desplazado. En este modelo retrocedemos d6 para recuperar el centro de muñeca.”

**Pregunta: ¿0.580 m garantiza solución?**

“No. Es un límite geométrico aproximado para el centro de muñeca; hay que verificar orientación y topes.”

**Transición:** “El cálculo puede dar varias configuraciones para llegar a la misma pose; veamos esas ramas.”

---

# Diapositiva 6 — Ramas y singularidad de muñeca

## Hasta ocho ramas

Hay hasta tres decisiones binarias:

$$
2\text{ hombros}\times2\text{ codos}\times2\text{ muñecas}=8
$$

Ocho es el máximo, no una promesa de que siempre aparecerán ocho soluciones distintas. Algunas pueden quedar fuera del alcance o de los límites; otras pueden coincidir en una singularidad.

## Etiqueta del codo

El código usa $\gamma=q_3+\psi$. Por eso “codo arriba/abajo” se determina con el signo de $\sin(\gamma)$, no mirando solo si $q_3$ es positivo o negativo. El cero geométrico del triángulo está desplazado respecto a la referencia de J3.

## Orientación residual y cálculo de q5

Tras calcular las tres primeras juntas, la orientación restante es:

$$
R_{36}=R_{03}^{T}R_{06}
$$

El código obtiene:

$$
s_5=\sqrt{R_{36}[0,2]^2+R_{36}[1,2]^2},\qquad
c_5=R_{36}[2,2],\qquad
q_5=\pm\operatorname{atan2}(s_5,c_5)
$$

**atan2** usa las componentes tipo seno y coseno y conserva el cuadrante. En esta extracción es más estable que calcular $\arccos(c_5)$ cerca de $|c_5|=1$. El programa trata aparte el caso singular; atan2 no elimina la singularidad geométrica.

## Qué pasa en la singularidad

Cuando $\sin(q_5)=0$ (por ejemplo, $q_5=0$), los ejes de J4 y J6 se alinean. Ya no se puede repartir de forma única el giro entre ambos: ramas que antes eran distintas pueden coincidir y el código escoge una representación conveniente.

La singularidad es una propiedad de la postura del robot, no automáticamente un error del programa.

## Guion oral sugerido

“Hay hasta ocho ramas: dos hombros, dos codos y dos muñecas. Después validamos cada una. Para la muñeca calculamos q5 usando seno y coseno de la matriz residual. Cuando q5 llega a cero o pi, J4 y J6 quedan alineadas y la orientación deja de tener una descomposición única; por eso pueden coincidir ramas.”

**Preguntas rápidas**

- **¿Siempre hay ocho?** No; es el máximo teórico y el filtrado o las singularidades pueden reducir las distintas soluciones.
- **¿El codo se clasifica con el signo de q3?** No; aquí depende de $\sin(q_3+\psi)$.
- **¿atan2 evita la singularidad?** No; mejora la extracción regular y el código maneja aparte el caso singular, pero no cambia la geometría.

**Transición:** “Tener ramas no basta: las comprobamos y escogemos una con un criterio explícito.”

---

# Diapositiva 7 — Selección, verificación y límites

## Costo de selección

El selector compara cada rama con una postura de referencia:

$$
C(q)=\sum_{i=1}^{6}w_i(q_i-q_{i,\mathrm{ref}})^2,\qquad
w=[4,3,2,1,1,0.5]
$$

Escoge la alternativa de menor costo. Los pesos expresan una preferencia del programa; **no** representan energía, torque ni una optimización industrial calibrada. En **demo_recorrido.py** se usa como referencia $q=0$; en la integración ROS de **nodo_hito2.py** se usa la postura actual recibida.

## Cómo se valida la solución

1. Se pasan los ángulos candidatos de nuevo por cinemática directa.
2. Se compara la posición reconstruida con la deseada.
3. Se compara la orientación reconstruida con la deseada.
4. Se rechazan resultados no finitos, con error excesivo o fuera de límites articulares.

El error de posición se mide en metros; el residuo de orientación es una norma matricial y no tiene unidades físicas. Ambos deben explicarse por separado.

## Prueba de 2000 casos

La presentación reporta un barrido de casos generados: se parte de configuraciones, se obtiene su pose por cinemática directa y se comprueba si la inversa recupera soluciones. El resumen indica cero casos sin resolver en esa muestra y un promedio cercano al máximo de ramas antes de filtros/singularidades.

Esto respalda el comportamiento en los casos muestreados; no prueba exhaustivamente todas las poses, ni valida movimiento en el ABB físico.

## Cifra de residuo: precisión al defenderla

La diapositiva y el archivo de resultados muestran valores de error del orden de $10^{-16}\text{ m}$, con una diferencia en el último dígito. Puedes decir: **“El residuo numérico es del orden de $10^{-16}$ metros, esencialmente cero a precisión de máquina para este cálculo.”** Si editas la diapositiva, unifica la cifra con el resultado vigente.

Ese residuo demuestra cierre numérico del modelo, no precisión física del robot real.

## Guion oral sugerido

“Como una pose puede tener varias soluciones, asignamos un costo a cada rama según cuánto cambian las juntas respecto a una referencia y elegimos la menor. Luego verificamos la solución volviendo a calcular la pose con cinemática directa, revisamos errores y límites. En los 2000 casos muestreados no quedaron poses sin resolver. Eso respalda esa prueba, pero no significa que hayamos cubierto todo el espacio ni validado el robot físico.”

**Preguntas rápidas**

- **¿El costo es energía?** No, es una regla ponderada para preferir ciertos cambios articulares.
- **¿Error pequeño significa que ya se puede ejecutar?** No. No sustituye límites, colisiones, velocidad, aceleración ni validación física segura.
- **¿Qué prueban los 2000 casos?** El resultado en esa muestra, no en todas las poses posibles.
- **¿Por qué repetir la directa al final?** Para confirmar que los ángulos hallados reconstruyen la pose solicitada.

---

# Cómo se conectan los cuatro scripts que preparas

| Script | Qué debes explicar | Relación con tu exposición |
|---|---|---|
| **fk_irb120.py** | Tabla D-H, matriz elemental, cadena y prueba en postura cero | Diapositivas 3–4; verifica también candidatos de Pieper |
| **pieper.py** | Centro de muñeca, eslabón equivalente, ramas, singularidad, filtros y selector | Diapositivas 5–7 |
| **resolver_en_vivo.py** | Recibe una pose y presenta alcance, ramas, descartes, errores y selección | Interfaz demostrativa del solucionador |
| **compare_urdf.py** | Compara los orígenes de juntas del URDF manual y el derivado | Verificación complementaria del modelo; no calcula la IK |

**Flujo principal:** pose deseada → centro de muñeca → ramas de Pieper → filtros de pose y límites → selección por costo → verificación con cinemática directa. La comparación de URDF es una comprobación aparte del modelo.

---

# Respuestas para memorizar

- **Cinemática directa:** convierte ángulos de juntas en la pose de la herramienta.
- **Cinemática inversa:** busca ángulos que produzcan una pose objetivo.
- **Centro de muñeca:** punto de intersección de los ejes de muñeca; se obtiene retrocediendo $d_6$ sobre $z_6$.
- **Eslabón equivalente:** construcción geométrica que combina $a_3$ y $d_4$; no es una pieza adicional.
- **Ocho ramas:** máximo por las alternativas de hombro, codo y muñeca.
- **Singularidad:** alineación que vuelve no única una parte de la solución articular.
- **Verificación FK:** comprueba que los ángulos propuestos reconstruyan la pose objetivo.

# Lista de preparación

- [ ] Puedo definir $\theta,d,a,\alpha$ y sus unidades.
- [ ] Puedo explicar el offset $-\pi/2$ de J2 sin aplicarlo dos veces.
- [ ] Puedo señalar posición $p$ y orientación $R$ en una matriz homogénea.
- [ ] Puedo explicar por qué se resta $d_6z_6$ para hallar el centro de muñeca.
- [ ] Puedo justificar $k$, $\psi$ y la etiqueta del codo.
- [ ] Puedo explicar el máximo de ocho ramas y la singularidad sin llamarla un fallo.
- [ ] Puedo decir qué mide y qué no mide el costo ponderado.
- [ ] Puedo distinguir evidencia numérica de validación en el robot físico.
- [ ] Coordiné quién presenta la diapositiva 2 y quién controla la demostración.

## Tres ideas para cerrar

1. **D-H convierte la geometría del robot en transformaciones calculables.**
2. **La muñeca esférica separa posición y orientación en la cinemática inversa.**
3. **Las ramas se verifican; una prueba numérica no reemplaza la validación física.**

Para estudiar cada script por bloques, revisa [fk_irb120.py](01_fk_irb120.md), [pieper.py](02_pieper.md), [resolver_en_vivo.py](08_resolver_en_vivo.md) y [compare_urdf.py](14_compare_urdf.md).
