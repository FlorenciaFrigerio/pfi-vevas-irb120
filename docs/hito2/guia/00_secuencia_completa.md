# Secuencia completa: cómo se conectan los scripts del Hito 2

La demostración por consola está en **scripts/demo_recorrido.py**. Orquesta seis etapas con funciones importadas de los módulos: directa, inversa por Pieper, Jacobiano, singularidad, trayectoria y dinámica. Es la ruta recomendada para exponer el flujo y no requiere ROS.

## Mapa de ejecución

~~~mermaid
flowchart TD
  D["demo_recorrido.py: demo narrada, sin ROS"] --> A["1. FK: q inicial a T"]
  A --> B["2. Pieper: T a ramas, filtrado y selección"]
  B --> C["3. Jacobiano en la postura elegida"]
  C --> E["4. Tendencia de singularidad de muñeca"]
  B --> F["5. LSPB sincronizado hasta q objetivo"]
  D --> G["6. Torque 1R en perfil escalar de referencia"]
  R["resolver_en_vivo.py: cinco formatos de pose"] -.pose alternativa.-> B
  N["nodo_hito2.py: extensión con ROS 2"] -.-> B
  N -.publica trayectoria.-> F
~~~

Las ramas desde demo_recorrido.py indican qué etapas coordina. **No** significan que el perfil de seis juntas de la etapa 5 se reutilice en el torque de la etapa 6: el código crea un perfil escalar 1R independiente, como se explica más abajo.

## Ejecutar la demostración

Desde scripts/:

~~~bash
python3 demo_recorrido.py
python3 demo_recorrido.py --seguido
python3 demo_recorrido.py --pose --figuras
~~~

- Por defecto pausa con Enter entre etapas.
- --seguido quita las pausas.
- --pose permite dictar una pose en cinco formatos; Enter vacío conserva el ejemplo.
- --figuras guarda las figuras generadas en figuras_recorrido/.
- Es una demostración numérica de consola; no mueve el ABB ni requiere ROS.

## Etapas que ejecuta

| Etapa | Función | Código que usa | Resultado que debes explicar |
|---:|---|---|---|
| 1 | etapa_1() | forward_kinematics de fk_irb120.py | Una configuración q determina una pose T; comprueba ortogonalidad y determinante de R |
| 2 | etapa_2() | pieper, filtrar, elegir, error_pose | Centro de muñeca, hasta ocho ramas, errores FK y selección por costo |
| 3 | etapa_3() | jacobiano.py | Jacobiano 6×6, rango, valores singulares y manipulabilidad |
| 4 | etapa_4() | jacobiano.py | Al acercarse q5 a cero, cae sigma_min y el rango llega a 5 |
| 5 | etapa_5(q_destino) | LSPBSincronizado y comprobar_continuidad de lspb.py | Trayectoria desde reposo hasta q objetivo; junta más lenta fija el tiempo total |
| 6 | etapa_6() | LSPB y funciones de dinamica_1r.py | Torque inercial más gravitatorio para un perfil 1R de referencia |

## Cómo la orquesta main()

El programa mantiene una postura de ejemplo para la directa, permite sustituir la pose de entrada y detiene el flujo si IK no devuelve una solución admisible:

~~~python
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

La pose dictada sustituye la T de ejemplo para la IK. La selección q_sel alimenta el Jacobiano y el perfil sincronizado de la etapa 5. Si todas las ramas se rechazan, no se calculan las etapas posteriores.

## El detalle importante del torque

La etapa 5 arma un movimiento de seis juntas de q0=[0,…,0] a q_sel, con límites V y A por junta. La etapa 6, en cambio, crea explícitamente este perfil:

~~~python
perfil = LSPB.desde_tiempo_final(0.0, 2.0, tf=3.0, acc=1.0)
~~~

Luego evalúa el torque de la junta 2 con parámetros de la hoja W10-S01 de cátedra. El control en t=2.5 s corresponde a ese ejemplo escalar 0→2 rad, no a q_sel[1] ni a la trayectoria sincronizada de la etapa 5. Por eso describe la demo como **seis bloques de análisis conectados por tema**, no como un cálculo dinámico completo del movimiento IK seleccionado.

## Dos interfaces y una ruta ROS opcional

- resolver_en_vivo.py recibe la pose del docente, muestra alcance, ramas, descartes, errores y etiquetas. La opción 6 corre casos borde y la 7 muestra un glosario.
- nodo_hito2.py es una segunda ruta de integración que requiere ROS 2: recibe PoseStamped, usa el resolvedor, crea LSPB desde la postura actual y publica /joint_states, /trayectoria_hito2 y diagnóstico.
- El demo narrado es la opción portátil para la defensa de cálculo. El nodo ROS es complementario y no convierte el proyecto en controlador de hardware.

## Guion oral breve

“demo_recorrido.py enlaza la cinemática directa con la inversa: construye una pose y después calcula y filtra ramas de Pieper. Sobre una rama válida analiza el Jacobiano y la singularidad, y genera un perfil LSPB multiarticular. La etapa final ilustra por separado el modelo 1R con el caso de referencia de cátedra; no es el torque de la trayectoria de seis juntas. El código no comanda el robot físico.”

## Lectura siguiente

[01_fk_irb120.md](01_fk_irb120.md) → [02_pieper.md](02_pieper.md) → [03_jacobiano.md](03_jacobiano.md) → [04_singularidad.md](04_singularidad.md) → [05_lspb.md](05_lspb.md) → [06_dinamica_1r.md](06_dinamica_1r.md) → [16_demo_recorrido.md](16_demo_recorrido.md).
