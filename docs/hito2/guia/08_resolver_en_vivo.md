# Script 08 — resolver_en_vivo.py: resolvedor interactivo de poses

> **Nivel:** herramienta de defensa. Domina el flujo de entrada y la lectura de la tabla; para las derivaciones de IK estudia también [02_pieper.md](02_pieper.md).

## Para qué sirve

Acepta una pose objetivo en uno de cinco formatos, la convierte a la matriz homogénea T y presenta el alcance, las ramas calculadas, las ramas admisibles y rechazadas, el error de reconstrucción y la manipulabilidad. Si hay ramas admisibles, selecciona una según el costo desde q_actual, imprime q* y devuelve la selección; si no, explica por qué no se debe mover.

**Entrada:** matriz homogénea, posición+cuaternión, posición+matriz R, posición+RPY en grados o posición+eje z6. Las cifras se pueden ingresar en una línea o en varias.
**Salida:** explicación y tabla en consola; no publica ROS ni mueve el robot.

## Bloque A — construir la orientación

Los constructores T_desde_cuaternion, T_desde_rpy, T_desde_eje_z y T_desde_R_p forman T 4×4. El cuaternión sigue el orden x, y, z, w y se normaliza antes de construir R.

**Código real** (scripts/resolver_en_vivo.py, función T_desde_cuaternion):

~~~python
def T_desde_cuaternion(x, y, z, qx, qy, qz, qw):
    n = np.linalg.norm([qx, qy, qz, qw])
    if n < 1e-12:
        raise ValueError("cuaternion nulo")
    qx, qy, qz, qw = np.array([qx, qy, qz, qw]) / n
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])
    return T_desde_R_p(R, [x, y, z])
~~~

**Cómo explicarlo:** los cuatro componentes definen orientación; se normalizan para que representen una rotación válida. El orden del programa es (qx, qy, qz, qw), con la parte real al final.

## Bloque B — aceptar datos repartidos

_limpiar_numeros() tolera separadores comunes. _leer() acumula los números de una o varias líneas, indica cuántos faltan, rechaza valores extra y permite cancelar escribiendo q, cancelar o salir.

**Código real** (scripts/resolver_en_vivo.py, líneas 227–272):

~~~python
def _leer(mensaje, n, ayuda=None):
    if ayuda:
        print(ayuda)
    vals = []
    while len(vals) < n:
        faltan = n - len(vals)
        if vals:
            prompt = f"  ... faltan {faltan}> "
        else:
            prompt = mensaje
        crudo = input(prompt).strip()
        if not crudo:
            continue
        if crudo.lower() in ("q", "cancelar", "salir"):
            raise ValueError("entrada cancelada por el operador")
        for token in _limpiar_numeros(crudo):
            try:
                vals.append(float(token))
            except ValueError:
                raise ValueError(
                    f"'{token}' no es un numero. Escriba 'cancelar' "
                    f"para volver al menu.")
        if len(vals) > n:
            raise ValueError(
                f"se esperaban {n} numeros y llegaron {len(vals)}. "
                f"Vuelva a intentarlo.")
    return vals
~~~

El código real también captura números mal formados e informa qué token no pudo convertir. Para una matriz, se pueden pegar las 16 cifras juntas o por filas.

## Bloque C — evaluar y mostrar las ramas

resolver_y_mostrar() estima el centro de muñeca y el alcance antes de llamar a pieper(). Después filtra candidatas con FK y límites, imprime cada rama y marca las rechazadas. También muestra w de manipulabilidad y una alerta si queda bajo el umbral.

**Código real** (scripts/resolver_en_vivo.py, líneas 123–181):

~~~python
p_W = p - d6 * T_d[:3, 2]
r_W = np.linalg.norm(p_W - np.array([0.0, 0.0, 0.290]))
holgura = 1e-4
if r_W > 0.580 + holgura:
    print("-> FUERA DE ALCANCE")
elif r_W > 0.580 - 1e-3:
    print("-> EN EL LIMITE del alcance")

crudas = pieper(T_d)
admisibles, rechazadas = filtrar(crudas, T_d, tol_pose=TOL_POSE)
~~~

**Importante:** el primer chequeo es diagnóstico de alcance. La decisión final depende también de las ramas, el error de pose y los topes articulares. No confundas una pose geométricamente alcanzable con una configuración admisible.

**Selección y salida:** con candidatas admisibles, el resolvedor elige una desde q_actual (cero si no se le entrega postura actual), imprime la solución elegida y la vuelve a verificar. Sin admisibles, no elige una rama y reporta los motivos del rechazo.

~~~python
if admisibles:
    q_sel, costo = elegir(admisibles, q_actual)
    ep, eR = error_pose(q_sel, T_d)
    print(f"SELECCIONADA: {etiquetar(q_sel)} costo {costo:.4f}")
    print(f"q* = {np.degrees(q_sel).round(2)} grados")
    return q_sel, admisibles, rechazadas
~~~


## Bloque D — menú actualizado

El menú tiene **cinco formatos de entrada**, casos borde en la opción 6, glosario de la salida en la opción 7 y salir en 0. Los datos se piden después de elegir el formato; se pueden repartir entre líneas.

~~~python
print("  1) Matriz homogenea T_d, 4x4        16 cifras")
print("  2) Posicion + cuaternion            x y z qx qy qz qw")
print("  3) Posicion + matriz R, 3x3         3 cifras + 9 cifras")
print("  4) Posicion + RPY en grados         x y z roll pitch yaw")
print("  5) Posicion + eje z6                x y z zx zy zz")
print("  6) Casos borde de demostracion")
print("  7) Glosario: que significa cada numero del informe")
~~~

## Bloque E — casos preparados para preguntas difíciles

casos_borde() construye seis situaciones: interior normal, singularidad de muñeca, codo extendido, eje de la base, borde del alcance y fuera de alcance.

**Código real** (scripts/resolver_en_vivo.py, líneas 437–473):

~~~python
# Singularidad de muñeca: q5 = 0 exacto
q = np.radians([20.0, -30.0, 15.0, 35.0, 0.0, -25.0])
resolver_y_mostrar(forward_kinematics(q),
                   titulo="2/6  Singularidad de muneca (q5 = 0)")

# Fuera de alcance: debe informar el rechazo
T = np.eye(4)
T[:3, 3] = [1.5, 1.5, 1.5]
resolver_y_mostrar(T, titulo="6/6  Fuera de alcance")
~~~

Úsalo para ensayar, pero no digas que los seis casos son una prueba exhaustiva de todo el espacio de trabajo.

## Cómo se relaciona

- Llama a pieper.py para obtener ramas y a su filtro/selector para validarlas.
- Usa fk_irb120.py para validar las poses y calcular ejemplos de casos borde.
- Usa medidas de jacobiano.py para mostrar movilidad.
- Es la interfaz interactiva que complementa [demo_recorrido.py](16_demo_recorrido.md); no es el algoritmo analítico de Pieper.

## Preparación para la defensa

- Si solo me dan posición y eje z6, ¿queda definida toda la orientación? **No:** queda libre el giro alrededor de z6.
- ¿Qué significa que salgan menos de ocho ramas? Puede haber ramas que coinciden por singularidad.
- ¿Una pose fuera de alcance es un fallo del programa? No: si se detecta y se rechaza correctamente, es comportamiento esperado.
- ¿El programa controla el brazo real? No; presenta el cálculo por consola.

## Límite

La opción eje z6 no fija por sí sola el giro de la herramienta alrededor de ese eje; el programa completa esa libertad con una elección declarada. El resolvedor tampoco convierte una pose válida en una orden de movimiento físico.
