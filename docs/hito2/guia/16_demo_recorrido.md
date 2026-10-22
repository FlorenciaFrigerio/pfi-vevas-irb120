# Script 16 — demo_recorrido.py: recorrido narrado del Hito 2

> **Nivel:** conocer el flujo completo. Cada integrante debe dominar la etapa que le corresponde y poder ubicar las otras cinco.

## Para qué sirve

Ejecuta y explica en consola seis bloques con los módulos del proyecto: cinemática directa, inversa por Pieper, Jacobiano, singularidad, trayectoria sincronizada y dinámica reducida. Es el hilo más útil para recorrer el código durante la exposición; corre sin ROS.

**Entrada:** por defecto usa una configuración/pose de ejemplo; con `--pose` permite introducir una pose en cinco formatos.
**Salida:** explicaciones y resultados numéricos; opcionalmente guarda las figuras que se generan.

## Bloque A — controles de ejecución

El script combina los argumentos leyendo sys.argv: pausa entre etapas por defecto, --seguido elimina las pausas, --pose pide una pose y --figuras prepara una salida sin ventanas gráficas.

~~~python
PAUSAR = "--seguido" not in sys.argv
FIGURAS = "--figuras" in sys.argv
POSE_DICTADA = "--pose" in sys.argv
DIR_FIGURAS = "figuras_recorrido"
~~~

La función pedir_pose() reutiliza _leer() y los constructores del resolvedor, en vez de mantener otro parser distinto. Así el formato aceptado por el demo y el menú de resolver_en_vivo.py se mantienen coherentes.

## Bloque B — etapa 1: cinemática directa

etapa_1() crea una configuración de ejemplo de seis ángulos y llama a forward_kinematics(q). Explica la tabla D-H, imprime T y verifica que R sea ortogonal y tenga determinante +1.

~~~python
q = np.radians([10.0, 20.0, -30.0, 40.0, 50.0, 60.0])
T = forward_kinematics(q)
~~~

**Idea para defender:** una configuración articular determina una pose; la solución directa es única para ese q.

## Bloque C — etapa 2: pose inversa y selección

etapa_2() calcula el centro de muñeca, revisa el alcance, llama a pieper(T), filtra las candidatas y elige una admisible con costo ponderado desde q actual. Si no queda una rama admisible devuelve None y el recorrido no sigue con cálculos que requieren q*.

~~~python
crudas = pieper(T)
admisibles, rechazadas = filtrar(crudas, T)
q_sel, costo = elegir(admisibles, np.zeros(6))
~~~

La pose dictada sustituye la T del ejemplo de FK. Con una pose generada en etapa 1, el programa también compara la rama elegida con la configuración inicial y hace una comprobación de ida y vuelta.

## Bloque D — etapas 3 y 4: movilidad

etapa_3(q_sel) forma el Jacobiano y calcula rango, valores singulares y manipulabilidad. etapa_4() varía q5 hacia cero para observar la caída de sigma_min y el colapso del rango.

**No mezcles las conclusiones:** el rango y los valores singulares describen movilidad local; no son una medida de torque.

## Bloque E — etapa 5: LSPB sincronizado

La trayectoria multiarticular parte de q0=0 y termina en la rama seleccionada. LSPBSincronizado determina un tiempo común; comprobar_continuidad() revisa extremos y muestras.

~~~python
q0 = np.zeros(6)
tr = LSPBSincronizado(q0, q_destino, V, A)
c = comprobar_continuidad(tr)
~~~

La posición tiene forma parabólica-lineal-parabólica. El perfil de velocidad es trapezoidal; la aceleración salta en los empalmes, por lo que el jerk ideal es infinito allí.

## Bloque F — etapa 6: torque de referencia 1R

El código usa un perfil escalar de referencia distinto del perfil sincronizado multiarticular de la etapa 5:

~~~python
perfil = LSPB.desde_tiempo_final(0.0, 2.0, tf=3.0, acc=1.0)
res = torque_sobre_lspb(perfil, t)
~~~

Este es el caso de cátedra para la junta 2. Calcula suma inercial y gravitatoria y compara el punto t=2.5 s con el solucionario. Por eso **no afirmes que el valor de torque final corresponde a la trayectoria de seis juntas que termina en q_sel**.

## Bloque G — orquestación y salida segura

La función main() corre las etapas en orden. Si la pose no produce una rama admisible, se detiene antes del Jacobiano, LSPB y torque. Las figuras solo se inicializan cuando se activa --figuras.

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

La secuencia que ves es de análisis y enseñanza. El programa no publica ROS ni comanda un robot real.

## Cómo correrlo

Desde la carpeta scripts/:

~~~bash
python3 demo_recorrido.py
python3 demo_recorrido.py --seguido
python3 demo_recorrido.py --pose --figuras
~~~

No pases --seguido en el primer ensayo si quieres practicar la explicación por etapa. Prueba la pose dictada con el ejemplo de la diapositiva y ensaya también el rechazo fuera de alcance con resolver_en_vivo.py --demo.

## Relación con otros módulos

- Llama a fk_irb120.py, pieper.py, jacobiano.py, lspb.py y dinamica_1r.py.
- Reutiliza el parser de resolver_en_vivo.py si se activa --pose.
- Usa gráficas de graficas.py cuando --figuras lo pide.
- nodo_hito2.py es otra ruta, opcional y dependiente de ROS 2.

## Preguntas para practicar

- ¿Qué ocurre si la pose dictada no es alcanzable? Se informa y las etapas dependientes de q* se detienen.
- ¿Por qué aparecen varias soluciones IK? Las ramas son configuraciones distintas que alcanzan la misma pose.
- ¿La etapa 6 calcula el torque del perfil multiarticular de la etapa 5? **No:** usa un perfil 1R separado de referencia.
- ¿Qué parte requiere ROS 2? El nodo opcional nodo_hito2.py, no este demo.
