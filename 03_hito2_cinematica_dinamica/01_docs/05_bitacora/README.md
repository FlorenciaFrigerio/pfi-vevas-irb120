# Bitácora técnica · Hito 2 (Cinemática inversa, singularidades, LSPB y dinámica)

[Volver al Hito 2](../../README.md) · [Cronología general](../../../01_docs/01_CRONOLOGIA.md) · [Anexo técnico](../01_informe/02_anexo_tecnico.md)

Registro de decisiones de ingeniería y correcciones de robustez del Hito 2 en formato **Problema → Cambio → Resultado → Archivo de evidencia**.

---

## H2-01 · Eliminación de los 193 rechazos falsos sobre los límites articulares

- **Problema:** Aunque un barrido aleatorio de 2.000 poses dentro del espacio de trabajo dio 0 fallos, una prueba dirigida de **1.800 poses generadas exactamente sobre las fronteras articulares** produjo **193 rechazos falsos**: `atan2` devolvía ángulos que excedían el tope del catálogo ABB por ruido de punto flotante ($\sim 10^{-16}\text{ rad}$), haciendo que la comparación estricta descartara posturas físicamente válidas.
- **Cambio:** En `pieper.py` se introdujo una tolerancia puramente numérica `TOL_LIMITE = 1e-9 rad` ($5{,}7 \times 10^{-8}\text{ grados}$) en `dentro_de_limites()` combinada con `recortar_a_limites()` (`np.clip`), que lleva al borde exacto únicamente los excesos atribuibles a redondeo sin admitir violaciones mecánicas reales.
- **Resultado:** Los rechazos en el banco dirigido de **1.800 poses en frontera bajaron de 193 a 0**, conservando un error máximo de reconstrucción FK de $4{,}98 \times 10^{-16}\text{ m}$.
- **Archivo de evidencia:** [`pieper.py`](../../../05_ros2_irb120_description/04_scripts/pieper.py) · [`resultados.json`](../04_resultados/resultados.json) · [`02_anexo_tecnico.md`](../01_informe/02_anexo_tecnico.md)

---

## H2-02 · Resolución cerca de la singularidad de muñeca (`|q5|` entre `10⁻⁷` y `5×10⁻⁵ rad`)

- **Problema:** Al calcular $q_5$ mediante $\arccos(R_{36}[2,2])$, la derivada infinita de $\arccos(c_5)$ en $|c_5| \to 1$ degradaba la precisión y declaraba falsamente singulares todas las posturas regulares con $|q_5| < 4{,}5 \times 10^{-5}\text{ rad}$.
- **Cambio:** Se reemplazó $\arccos(c_5)$ por la lectura directa del seno $|s_5| = \operatorname{hypot}(R_{36}[0,2], R_{36}[1,2])$ y $q_5 = \pm\operatorname{atan2}(|s_5|, c_5)$, reservando la rama degenerada únicamente para $|s_5| < 10^{-12}$.
- **Resultado:** El resolvedor recupera ramas admisibles exactas en todo el barrido cercano a la singularidad ($q_5 \in \{10^{-7}, 10^{-6}, 10^{-5}, 3\times 10^{-5}, 5\times 10^{-5}\}\text{ rad}$).
- **Archivo de evidencia:** [`pieper.py`](../../../05_ros2_irb120_description/04_scripts/pieper.py) · [`resultados.json`](../04_resultados/resultados.json)

---

## H2-03 · Expansión de vueltas equivalentes en `J6` (`±400°`) para evitar giros de `360°`

- **Problema:** La brida `J6` del ABB IRB 120 admite $\pm 400^\circ$ ($\pm 6{,}9813\text{ rad}$), pero `atan2` devuelve siempre el representante principal en $(-\pi, \pi]$. Cuando el robot partía de $q_6 = 6{,}0\text{ rad}$ y se pedía mantener la misma pose, el selector elegía $q_6 - 2\pi = -0{,}2832\text{ rad}$, ordenando una vuelta completa innecesaria de $6{,}2832\text{ rad}$.
- **Cambio:** Se implementó `representantes_admisibles(q)` en `pieper.py` para expandir todas las congruencias $q_i \pm 2\pi n$ dentro de `JOINT_LIMITS` antes de evaluar la distancia ponderada $w = [4, 3, 2, 1, 1, 0{,}5]$ respecto a `q_actual`.
- **Resultado:** Ante una pose ya alcanzada en $q_6 = 6{,}0\text{ rad}$, el movimiento planificado pasó de **$6{,}2832\text{ rad}$ a $8{,}88 \times 10^{-16}\text{ rad}$**.
- **Archivo de evidencia:** [`pieper.py`](../../../05_ros2_irb120_description/04_scripts/pieper.py) · [`resultados.json`](../04_resultados/resultados.json)

---

## H2-04 · Etiquetado geométrico consistente de las 8 ramas de Pieper

- **Problema:** Clasificar «codo arriba/abajo» con el signo crudo de $q_3$ confundía ramas opuestas debido al offset estructural del antebrazo ($a_3 = 0{,}070\text{ m}$, $d_4 = 0{,}302\text{ m}$, ángulo $\psi = \operatorname{atan2}(d_4, a_3) = 1{,}343\text{ rad}$).
- **Cambio:** Se definió la etiqueta de codo a partir del signo de $\sin(q_3 + \psi)$ del eslabón equivalente $k = \sqrt{a_3^2 + d_4^2} = 0{,}310\text{ m}$, y la etiqueta de hombro comparando $q_1$ contra el acimut real del centro de muñeca $p_W$.
- **Resultado:** Las 8 soluciones quedan identificadas de forma biunívoca (`2 hombro × 2 codo × 2 muñeca`) tanto en el notebook como en `resolver_en_vivo.py`.
- **Archivo de evidencia:** [`pieper.py`](../../../05_ros2_irb120_description/04_scripts/pieper.py) · [`01_Hito2_VEVAS.ipynb`](../../02_notebooks/01_Hito2_VEVAS.ipynb)

---

## H2-05 · Sincronización multi-junta en LSPB y separación declarada del modelo dinámico 1R

- **Problema:** (1) Planificar cada eje con su propio tiempo mínimo hacía que las juntas rápidas terminaran antes, deformando la trayectoria espacial. (2) ABB no publica masas ni tensores de inercia de los 6 eslabones del IRB 120, y en la trayectoria de 6 juntas el eje `J2` resultaba triangular ($t_{\text{crucero}} = 0\text{ s}$), impidiendo observar el efecto gravitatorio a velocidad constante.
- **Cambio:** En `lspb.py` se fijó $T = \max_i T_i$ reescalando hacia abajo únicamente la velocidad de crucero $V_i$ (sin incrementar aceleraciones). En `dinamica_1r.py` se derivó explícitamente la correspondencia angular $\phi = \pi - q_2$ y se mantuvo separado el perfil de referencia de cátedra ($0 \to 2\text{ rad}$ en $3\text{ s}$) para verificar el punto de control en $t = 2{,}5\text{ s}$ y el torque no nulo durante el crucero.
- **Resultado:** Las 6 juntas convergen simultáneamente en $T = 0{,}7746\text{ s}$ sin saltos en posición ni velocidad, y el modelo 1R reproduce $\tau(2{,}5\text{ s}) = -0{,}2500 + 4{,}6798 = \mathbf{4{,}4298\text{ N}\cdot\text{m}}$ frente al valor publicado de $4{,}429\text{ N}\cdot\text{m}$ (diferencia explicada por el redondeo de $\sin(1{,}875) \approx 0{,}954$ en la guía).
- **Archivo de evidencia:** [`lspb.py`](../../../05_ros2_irb120_description/04_scripts/lspb.py) · [`dinamica_1r.py`](../../../05_ros2_irb120_description/04_scripts/dinamica_1r.py) · [`salida_dinamica_1r.txt`](../04_resultados/salida_dinamica_1r.txt)
