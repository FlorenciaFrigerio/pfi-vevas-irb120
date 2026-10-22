# Guion de exposición: recorrido del código hasta el resultado

Este guion explica **qué código mostrar, en qué orden, por qué va en ese orden y qué resultado conduce al siguiente paso**. Está pensado para narrar `scripts/demo_recorrido.py` durante la exposición.

> **Idea central:** el demo enlaza seis bloques de análisis. La trayectoria de seis juntas termina en una configuración objetivo; el torque final ilustra aparte un modelo escalar 1R con su propio perfil de referencia.

## Mapa rápido

~~~mermaid
flowchart TD
  A["Modelo D-H y postura inicial"] --> B["Cinemática directa q a T"]
  B --> C["Pose objetivo T_d"]
  C --> D["Pieper: centro de muñeca y ramas"]
  D --> E["Filtrar y escoger q estrella"]
  E --> F["Jacobiano en q estrella"]
  F --> G["Estudio de singularidad"]
  E --> H["LSPB de seis juntas: q inicial a q estrella"]
  I["Perfil LSPB 1R de referencia"] --> J["Dinámica reducida y torque"]
  H --> K["Cierre: alcance y límites"]
  J --> K
~~~

**Cómo leer el mapa:** la pose calculada en la etapa 1 puede guiar la inversa, o puede sustituirse por una pose dictada. Las etapas de Jacobiano y LSPB usan la configuración seleccionada. El barrido de singularidad es un experimento de diagnóstico. El perfil escalar de torque es independiente del LSPB de seis juntas.

## Preparación de la demostración

Desde la carpeta `scripts/`:

~~~bash
python3 demo_recorrido.py
~~~

El programa pausa entre las etapas para que puedas explicar cada salida. Opciones:

- `--seguido`: ejecuta sin pausas.
- `--pose`: permite introducir una pose objetivo en alguno de los formatos disponibles.
- `--figuras`: guarda las figuras de las etapas.

Para enseñar el recorrido completo sin depender de una pose dictada, usa primero el ejemplo predeterminado. Una pose fuera del alcance o incompatible con los límites detiene correctamente el flujo después de la cinemática inversa: sin una configuración admisible no se debe inventar una trayectoria.

## Orden justificado

| Paso | Archivo y función | Qué entra y qué sale | Por qué va aquí |
|---:|---|---|---|
| 1 | `fk_irb120.py` — `forward_kinematics(q)`; llamada desde `demo_recorrido.py` — `etapa_1()` | Entran seis ángulos `q`; sale la pose `T` de la brida | Primero presentamos el modelo geométrico. La directa demuestra cómo la tabla D-H transforma ángulos en posición y orientación. También crea una pose conocida para comprobar el recorrido de ida y vuelta. |
| 2 | `resolver_en_vivo.py` — lectura y construcción de pose, si se usa `--pose` | Entran posición y orientación dictadas; sale una matriz objetivo `T_d` | La inversa necesita una pose de destino. Con el modo predeterminado, se reutiliza la pose de la etapa 1; con `--pose`, la entrada dictada la reemplaza. |
| 3 | `pieper.py` — `wrist_center`, `pieper`, `filtrar`, `elegir`; presentado por `etapa_2()` | Entra `T_d`; salen ramas candidatas, descartes y una configuración seleccionada `q*` | Solo después de definir la pose podemos resolver qué ángulos la alcanzan. Pieper calcula hasta ocho ramas; la FK verifica que reconstruyan la pose y los límites ABB descartan las no ejecutables. |
| 4 | `jacobiano.py` — `jacobiano(q*)`, SVD y manipulabilidad; `etapa_3()` | Entra `q*`; salen `J`, rango, valores singulares y medida de movilidad | Ya conocemos una configuración. Ahora preguntamos cómo se mueve localmente el extremo desde esa postura y si conserva seis direcciones independientes. |
| 5 | `etapa_4()` de `demo_recorrido.py` — barrido de `q5` | Cambia `q5` en un experimento controlado; salen rango, `σ_min` y manipulabilidad | El Jacobiano explica movilidad; este barrido muestra qué ocurre cuando la muñeca se acerca a una singularidad. Es un diagnóstico aislado y no modifica `q*`. El módulo `singularidad.py` contiene el estudio ampliado. |
| 6 | `lspb.py` — `LSPBSincronizado`; `etapa_5(q*)` | Entran la postura inicial y `q*`; sale una trayectoria sincronizada `q(t)`, `q̇(t)` y `q̈(t)` | Una configuración final solo dice dónde terminar. LSPB añade cómo llegar allí, con aceleración, crucero y frenado, y sincroniza las seis juntas para terminar juntas. |
| 7 | `dinamica_1r.py` — `LSPB.desde_tiempo_final` y `torque_1r`; `etapa_6()` | Entra un perfil escalar 1R de referencia; salen torque inercial, gravitatorio y total | Después de hablar de movimiento, se ilustra el esfuerzo con el modelo reducido. Este ejemplo usa un perfil propio de 0 a 2 rad en 3 s y no calcula el torque de la trayectoria multiarticular de seis juntas de la etapa 5. |
| 8 | `cierre()` de `demo_recorrido.py` | Resume qué demuestra el recorrido y sus límites | Cerramos separando movilidad cinemática, trayectoria y torque dinámico, y declaramos qué no se ha validado en hardware. |

### Qué scripts ejecuta realmente el demo

`demo_recorrido.py` importa y usa directamente `fk_irb120.py`, `pieper.py`, `jacobiano.py`, `lspb.py` y `dinamica_1r.py`. Si se introduce una pose, reutiliza constructores de `resolver_en_vivo.py`. La etapa de singularidad está programada dentro de `demo_recorrido.py`; `singularidad.py` es el módulo independiente para estudiar el fenómeno con más detalle.

`nodo_hito2.py` es otra ruta de integración: recibe una pose mediante ROS 2, llama al resolvedor y publica una trayectoria. No es necesario para ejecutar ni explicar el demo de consola y no comanda un ABB industrial.

## Guion oral por transiciones

### Inicio → cinemática directa

> “Primero necesitamos un modelo de la geometría. Damos seis ángulos al modelo D-H y la cinemática directa nos devuelve una pose: posición y orientación de la brida.”

### Directa → pose objetivo → Pieper

> “Con una pose conocida podemos comprobar el recorrido de ida y vuelta. También podemos reemplazarla por una pose dictada. Pieper recibe esa pose y calcula las configuraciones que podrían alcanzarla.”

### Pieper → verificación y selección

> “Las ramas son alternativas geométricas, no todas necesariamente ejecutables. Recalculamos cada candidata con cinemática directa, comprobamos el error de pose y aplicamos los límites articulares. Entre las admisibles elegimos una con un costo ponderado.”

> **Detalle para defender con precisión:** en `demo_recorrido.py`, `etapa_2()` llama a `elegir(admisibles, np.zeros(6))`, así que la demostración selecciona respecto a la postura cero. En el nodo ROS, el selector recibe `q_actual`, que representa la postura vigente.

### Configuración seleccionada → movilidad

> “Con `q*` ya definida evaluamos el Jacobiano. Este relaciona la velocidad de las juntas con la velocidad lineal y angular del extremo. Sus valores singulares muestran si alguna dirección se vuelve difícil o imposible.”

### Movilidad → singularidad

> “Para mostrar el caso límite, acercamos `q5` a cero. La menor singularidad cae y, en el punto exacto, el rango baja. Eso significa que se pierde una dirección instantánea de movimiento.”

### Configuración → trayectoria

> “Con una solución válida podemos describir cómo moverse desde la postura inicial hasta `q*`. LSPB genera posición, velocidad y aceleración en el tiempo. La sincronización hace que las seis juntas terminen a la vez.”

### Trayectoria → ejemplo dinámico → cierre

> “Al final mostramos un ejemplo de torque con un modelo 1R y parámetros dados por cátedra. Es una demostración pedagógica independiente: no es el torque real de las seis juntas ni el torque de la trayectoria recién generada.”

## Qué significa cada símbolo que aparecerá

- `q`: ángulos de las seis juntas, en radianes.
- `T` o `T_d`: matriz homogénea; contiene posición y orientación.
- `q*`: configuración articular que se seleccionó para la pose objetivo.
- `J`: Jacobiano, que relaciona velocidades articulares y velocidad del extremo.
- `σ_min`: menor valor singular; si llega a cero, el Jacobiano pierde rango.
- `q(t)`, `q̇(t)`, `q̈(t)`: posición, velocidad y aceleración articulares a lo largo del tiempo.
- `τ`: torque requerido por el modelo dinámico reducido.

## Módulos que apoyan, pero no son una etapa directa del demo

| Archivo | Función en la defensa | Relación con el recorrido principal |
|---|---|---|
| `resolver_en_vivo.py` | Construir una pose dictada, mostrar ramas, errores y descartes | Se usa como ayuda de entrada cuando se activa `--pose`; también puede demostrarse por separado en consola. |
| `singularidad.py` | Ejecutar el análisis ampliado de rango y SVD | Complementa la etapa 4; `demo_recorrido.py` hace su propio barrido para narrar el fenómeno. |
| `graficas.py` | Dibujar curvas y figuras | Se importa cuando se usa `--figuras`; no calcula la solución cinemática. |
| `capturar_resultados.py` | Guardar salidas y cifras reproducibles | Sirve para presentar la evidencia después del demo; no orquesta el movimiento. |
| `dinamica.py` | Probar un modelo planar 2R | Es un análisis complementario; el resultado principal de la presentación usa el modelo docente reducido 1R. |
| `nodo_hito2.py` | Recibir una pose con ROS 2 y publicar estados/trayectoria | Es una ruta de integración alternativa. Requiere ROS 2 y no es necesaria para el recorrido de consola. |
| `generate_urdf.py`, `add_visuals.py` | Crear y completar el modelo visual URDF | Corresponden al gemelo digital y contextualizan la geometría; no son llamadas por el demo numérico. |
| `compare_urdf.py`, `validate_tf.py` | Comparar el URDF con la referencia y validar TF de ROS 2 | Son verificaciones del gemelo digital/integración, no pasos de la solución Pieper → LSPB. |

## El orden real dentro de `main()`

Si se activa `--pose`, `pedir_pose()` pregunta el formato antes de comenzar las etapas. Luego `main()` siempre muestra la cinemática directa de ejemplo. Si se había dictado una pose, sustituye con ella la pose que usará Pieper:

~~~python
T_dictada = pedir_pose() if POSE_DICTADA else None
q_orig, T = etapa_1()
if T_dictada is not None:
    T = T_dictada

q_sel = etapa_2(q_orig, T, dictada=T_dictada is not None)
if q_sel is None:
    return

etapa_3(q_sel)
etapa_4()
etapa_5(q_sel)
etapa_6()
cierre()
~~~

Esto da una explicación útil al docente: la etapa 1 ilustra `q → T`; si hay pose dictada, se reemplaza el destino para la inversa. Si la inversa no encuentra una rama admisible, el demo se detiene antes de Jacobiano y trayectoria porque no tiene una configuración final válida.
## Límites que se deben decir con claridad

1. Las 2000 poses verifican el solver en un conjunto muestreado; no prueban toda pose matemática ni todas las condiciones físicas del robot.
2. La trayectoria LSPB muestra el movimiento articular calculado; el demo no acciona el hardware.
3. El torque final pertenece al modelo pedagógico de una junta y a su perfil de referencia separado. No es una identificación dinámica de los seis ejes ABB.
4. El nodo ROS 2 es integración de software adicional; la demostración por consola corre sin ROS.

## Resumen de 30 segundos

> “El recorrido empieza con D-H y cinemática directa para convertir una configuración en una pose. Pieper resuelve la pose objetivo, verifica sus ramas con FK y límites, y selecciona una configuración. Con esa configuración analizamos movilidad mediante el Jacobiano y mostramos una singularidad. Luego LSPB genera una trayectoria sincronizada de seis juntas. El torque final es un ejemplo 1R independiente que sirve para explicar las contribuciones inercial y gravitatoria, no para afirmar que modelamos toda la dinámica del ABB.”

## Enlaces para seguir el código

- [Secuencia técnica y diagrama](00_secuencia_completa.md)
- [Índice de los 16 scripts explicados](README.md)
- [Demo narrado, bloque por bloque](16_demo_recorrido.md)
- [FK y tabla D-H](01_fk_irb120.md)
- [Pieper, filtros y selección](02_pieper.md)
