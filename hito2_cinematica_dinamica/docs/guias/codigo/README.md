# Guía navegable del código — Hito 2 ABB IRB 120

> Un archivo Markdown por cada uno de los 16 scripts. Los bloques se contrastan con el código vigente en scripts/; la secuencia y el reparto siguen la exposición de 20 diapositivas.

## Guion por integrante

- [Guion de diapositivas — Integrante 1](guion_integrante_1_diapos.md): portada y diapositivas 3–7.

## Empieza por el recorrido

El archivo que narra las seis etapas es **demo_recorrido.py**. Se ejecuta desde scripts/, usa Python, NumPy y Matplotlib y **no necesita ROS**:

~~~bash
cd scripts
python3 demo_recorrido.py --pose --figuras
~~~

Por defecto pausa entre etapas. Usa --seguido para recorrerlas sin pausas. Para exponer el recorrido con transiciones y justificación del orden, consulta el [guion de exposición](guion_recorrido_exposicion.md), la [secuencia técnica](00_secuencia_completa.md) y la guía del [demo](16_demo_recorrido.md).

~~~mermaid
flowchart LR
  D["demo_recorrido.py: orquesta seis etapas, sin ROS"] --> A["1. FK: q a pose T"]
  A --> B["2. Pieper: pose T a ramas q"]
  B --> C["3. Jacobiano: movilidad local"]
  C --> E["4. Singularidad: pérdida de rango"]
  B --> F["5. LSPB multiarticular: transición a q objetivo"]
  D --> G["6. Dinámica 1R: perfil escalar de referencia"]
  H["resolver_en_vivo.py: entrada interactiva de pose"] -.-> B
  I["nodo_hito2.py: integración ROS 2 opcional"] -.-> B
  I -.-> F
~~~

> **Precisión para la defensa:** en demo_recorrido.py, la etapa 6 calcula el torque con un perfil LSPB 1R de referencia de 0 a 2 rad en 3 s. No calcula el torque del movimiento multiarticular de la etapa 5. La dinámica demuestra el modelo reducido y su interpretación.

## Los 16 scripts, orden de estudio

| # | Archivo | Enfoque |
|---:|---|---|
| 1 | [fk_irb120.py](01_fk_irb120.md) | D-H, cinemática directa y límites |
| 2 | [pieper.py](02_pieper.md) | IK analítica, ramas, filtros y selección |
| 3 | [jacobiano.py](03_jacobiano.md) | Jacobiano geométrico y validaciones |
| 4 | [singularidad.py](04_singularidad.md) | SVD, rango y tarea compatible/incompatible |
| 5 | [lspb.py](05_lspb.md) | Perfil de posición, velocidad y aceleración; sincronización |
| 6 | [dinamica_1r.py](06_dinamica_1r.md) | Modelo reducido 1R y procedencia de parámetros |
| 7 | [dinamica.py](07_dinamica.md) | Modelo 2R complementario, con supuestos |
| 8 | [resolver_en_vivo.py](08_resolver_en_vivo.md) | Menú de poses, validación e interpretación |
| 9 | [nodo_hito2.py](09_nodo_hito2.md) | Ruta ROS 2 opcional de pose a trayectoria |
| 10 | [graficas.py](10_graficas.md) | Gráficas y lectura de unidades |
| 11 | [capturar_resultados.py](11_capturar_resultados.md) | Evidencia reproducible |
| 12 | [generate_urdf.py](12_generate_urdf.md) | Generación de URDF desde D-H |
| 13 | [add_visuals.py](13_add_visuals.md) | Geometría visual del URDF |
| 14 | [compare_urdf.py](14_compare_urdf.md) | Comparación de URDF manual y derivado |
| 15 | [validate_tf.py](15_validate_tf.md) | Validación TF de ROS 2 para Hito 1 |
| 16 | [demo_recorrido.py](16_demo_recorrido.md) | Orquesta y explica las seis etapas |

## Qué dominar y qué ubicar

**Estudio a fondo:** fk_irb120.py, pieper.py, jacobiano.py, singularidad.py, lspb.py, dinamica_1r.py, resolver_en_vivo.py y demo_recorrido.py.

**Conocer propósito, conexión y límites:** dinamica.py, nodo_hito2.py, graficas.py, capturar_resultados.py, generate_urdf.py, add_visuals.py, compare_urdf.py y validate_tf.py.

La ruta principal de la exposición va de las páginas 3–7 (cinemática), 8–11 (movilidad), 12–15 (pruebas y reproducibilidad) y 16–20 (trayectoria y dinámica). El demo es el hilo común: cada integrante debe conocer la etapa que le toca y saber señalar dónde se ejecuta.

## Reparto para cuatro, alineado con las diapositivas

| Persona / diapositivas | Scripts que prepara | Defensa principal |
|---|---|---|
| **1 · págs. 3–7 — Cinemática** | fk_irb120.py, pieper.py, resolver_en_vivo.py, compare_urdf.py | D-H, centro de muñeca, ocho ramas, filtrado y selección |
| **2 · págs. 8–11 — Movilidad** | jacobiano.py, singularidad.py, graficas.py, validate_tf.py | Jacobiano, SVD, rangos, residuo y lectura de evidencia |
| **3 · págs. 12–15 — Pruebas y repositorio** | demo_recorrido.py, capturar_resultados.py, generate_urdf.py, add_visuals.py | Cómo reproducir cifras, correr el recorrido y encontrar módulos |
| **4 · págs. 16–20 — Trayectoria y dinámica** | lspb.py, dinamica_1r.py, dinamica.py, nodo_hito2.py | Sincronización, torque 1R, límites del 2R e integración ROS opcional |

Son cuatro scripts por persona. La pieza compartida es demo_recorrido.py: la persona 3 sabe operarla y cada integrante explica su etapa. Para una pregunta cruzada, todos deben saber qué entra y qué sale de la etapa anterior y de la siguiente.

## Ficha común por archivo

- **Propósito:** ¿qué problema resuelve?
- **Entrada/salida:** ¿qué recibe y qué devuelve o publica?
- **Bloques:** ¿qué hacen las funciones en orden?
- **Relación:** ¿quién lo llama y qué usa después?
- **Evidencia:** ¿qué cifra, prueba o gráfica respalda la explicación?
- **Límite:** ¿qué no demuestra?
- **Defensa:** preparar una respuesta oral de 30–60 segundos.

## Límites que no deben ocultarse

- Pieper obtiene hasta ocho ramas; en singularidades algunas coinciden.
- La FK vuelve a comprobar las soluciones inversas; pasar la FK no sustituye revisar límites articulares.
- Una singularidad reduce direcciones instantáneas; la pseudoinversa puede dejar residuo ante una tarea incompatible.
- El perfil de posición LSPB es parabólico-lineal-parabólico; el trapezoidal es el de velocidad. La aceleración salta en los empalmes.
- dinamica_1r.py usa parámetros indicados por cátedra, no una identificación de los seis ejes del ABB.
- demo_recorrido.py calcula el torque 1R sobre un perfil escalar de referencia separado del perfil sincronizado de seis juntas.
- dinamica.py es un experimento 2R con parámetros estimados.
- nodo_hito2.py requiere ROS 2 y publica mensajes; no es control industrial de hardware.
- capturar_resultados.py escribe archivos de resultados; revisar su destino antes de correrlo.

## Documentación relacionada

- [Secuencia completa](00_secuencia_completa.md)
- [Manual de estudio y defensa](../manuales/MANUAL_HITO2.md)
- [Módulos y comandos de scripts](../../../scripts/README.md)
- [Anexo técnico](../anexo_tecnico.md)
- [Resultados reproducibles](../resultados/)
