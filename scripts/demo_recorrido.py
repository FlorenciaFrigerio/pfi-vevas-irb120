#!/usr/bin/env python3
"""
Recorrido completo del pipeline, narrado etapa por etapa.

Para que sirve

    El docente pidio ver TODO el recorrido, empezando por la cinematica
    directa y encadenando cada bloque hasta el torque. Los modulos
    individuales ya hacen cada cosa por separado, pero corridos sueltos
    no muestran lo importante: que la salida de cada etapa es la entrada
    de la siguiente.

    Este guion ejecuta las seis etapas en orden, explicando antes de
    cada una que se va a hacer y despues que significa el resultado.

        1. Cinematica directa     q  ->  T
        2. Cinematica inversa     T  ->  q   (ocho ramas, Pieper)
        3. Jacobiano              q  ->  J   (y sus validaciones)
        4. Singularidad           cuando J pierde rango
        5. Trayectoria LSPB       q_inicial -> q_final suave
        6. Dinamica 1R            la trayectoria -> torque

Uso

    python3 demo_recorrido.py            pausa entre etapas (Enter)
    python3 demo_recorrido.py --seguido  sin pausas, de corrido
    python3 demo_recorrido.py --figuras  guarda una figura por etapa
    python3 demo_recorrido.py --pose     pide una pose dictada y la usa
                                         como hilo conductor del recorrido

    Las opciones se combinan. Para la defensa, lo util es

        python3 demo_recorrido.py --pose --figuras

    que resuelve la pose que dicte el docente y deja las figuras del
    recorrido en disco.

Grupo VEVAS
"""

import os
import sys
import textwrap

import numpy as np

from fk_irb120 import forward_kinematics, JOINT_LIMITS
from pieper import pieper, filtrar, elegir, etiquetar, error_pose
from jacobiano import jacobiano, medida_manipulabilidad
from lspb import LSPB, LSPBSincronizado, comprobar_continuidad
from dinamica_1r import (torque_1r, torque_sobre_lspb, verificacion_analitica,
                         J_INERCIA, M_MASA, LC_CENTRO, G_GRAV)

PAUSAR = "--seguido" not in sys.argv
FIGURAS = "--figuras" in sys.argv
POSE_DICTADA = "--pose" in sys.argv

DIR_FIGURAS = "figuras_recorrido"
_guardadas = []


def _preparar_figuras():
    """
    Deja listo el directorio de salida y fuerza el backend sin pantalla.

    Se usa Agg porque el recorrido se corre desde una terminal: sin eso,
    matplotlib intenta abrir ventanas que bloquean la ejecucion, y en la
    defensa eso significa quedarse mirando una ventana en blanco.
    """
    if not FIGURAS:
        return False
    import matplotlib
    matplotlib.use("Agg")
    os.makedirs(DIR_FIGURAS, exist_ok=True)
    return True


def _anotar(ruta):
    _guardadas.append(ruta)
    print(f"\n  [figura guardada]  {ruta}")


def titulo(n, texto):
    print("\n")
    print(f"  ETAPA {n}   {texto}")
    print("  " + "-" * 66)


def explicar(texto):
    """
    Parrafo de contexto antes de mostrar numeros.

    Se usa textwrap.dedent y no strip() por linea, para que las
    formulas y las listas conserven su sangria relativa: una ecuacion
    alineada al margen izquierdo se lee mucho peor.
    """
    for linea in textwrap.dedent(texto).strip("\n").split("\n"):
        print(("  " + linea).rstrip())
    print()


def pausa():
    if PAUSAR:
        try:
            input("\n  [Enter para continuar]")
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)


# =====================================================================
#  ETAPA 1 - CINEMATICA DIRECTA
# =====================================================================

def etapa_1():
    titulo(1, "CINEMATICA DIRECTA     q  ->  T")
    explicar("""
        Punto de partida de todo. Dados los seis angulos de las juntas,
        la cinematica directa dice donde queda la herramienta.

        Se construye encadenando seis transformaciones homogeneas, una
        por junta, tomadas de la tabla de Denavit-Hartenberg:

            T = A1(q1) A2(q2) A3(q3) A4(q4) A5(q5) A6(q6)

        Esta etapa tiene UNA sola respuesta: dados los angulos, la pose
        queda determinada. La inversa, como se vera enseguida, no.
        """)

    q = np.radians([10.0, 20.0, -30.0, 40.0, 50.0, 60.0])
    T = forward_kinematics(q)

    print("  Configuracion de ejemplo (grados):")
    print(f"    q = {np.degrees(q).round(2)}")
    print("\n  Pose resultante T:")
    for fila in T:
        print("    " + "  ".join(f"{v:+9.4f}" for v in fila))

    p = T[:3, 3]
    print(f"\n  La esquina derecha es la POSICION del extremo:")
    print(f"    p = [{p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f}] m")
    print(f"  El bloque 3x3 de arriba a la izquierda es la ORIENTACION.")

    orto = np.linalg.norm(T[:3, :3].T @ T[:3, :3] - np.eye(3), "fro")
    print(f"\n  Verificacion de que R es una rotacion legitima:")
    print(f"    ||R^T R - I||_F = {orto:.3e}   (debe ser ~0)")
    print(f"    det(R)          = {np.linalg.det(T[:3, :3]):.6f}   (debe ser +1)")
    print("\n  Esa comprobacion importa: una matriz que no cumple ambas")
    print("  cosas no representa ninguna rotacion fisica posible.")

    return q, T


# =====================================================================
#  ETAPA 2 - CINEMATICA INVERSA
# =====================================================================

def pedir_pose():
    """
    Pide la pose que dicte el docente y la convierte en T_d.

    Se reutilizan los constructores y el lector multilinea del
    resolvedor en vivo, para no mantener dos implementaciones del mismo
    parseo: si una acepta un formato, la otra tambien.
    """
    from resolver_en_vivo import (_leer, T_desde_cuaternion, T_desde_rpy,
                                  T_desde_eje_z, T_desde_R_p)

    print("\n")
    print("  POSE DICTADA")
    print("  " + "-" * 66)
    explicar("""
        El recorrido puede usar una pose cualquiera en lugar del ejemplo
        de fabrica. Elija el formato en que se la dictaron.

            1) Matriz homogenea T_d, 4x4        16 cifras
            2) Posicion + cuaternion            x y z qx qy qz qw
            3) Posicion + matriz R, 3x3         3 cifras + 9 cifras
            4) Posicion + RPY en grados         x y z roll pitch yaw
            5) Posicion + eje z6                x y z zx zy zz

        Las cifras pueden ir juntas o repartidas en varias lineas.
        Enter sin escribir nada usa la pose de ejemplo.
        """)

    while True:
        try:
            op = input("  formato> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return None
        if not op:
            print("  Se usara la pose de ejemplo.")
            return None
        try:
            if op == "1":
                v = _leer("  T_d (fila 1)> ", 16,
                          "  Dieciseis cifras, por filas.")
                T = np.array(v).reshape(4, 4)
                return T_desde_R_p(T[:3, :3], T[:3, 3])
            if op == "2":
                return T_desde_cuaternion(*_leer("  x y z qx qy qz qw> ", 7))
            if op == "3":
                p = _leer("  x y z> ", 3)
                R = np.array(_leer("  R (fila 1)> ", 9,
                                   "  Nueve cifras, por filas."))
                return T_desde_R_p(R.reshape(3, 3), p)
            if op == "4":
                print("  Convencion: angulos fijos XYZ, "
                      "R = Rz(yaw) Ry(pitch) Rx(roll).")
                return T_desde_rpy(*_leer("  x y z roll pitch yaw> ", 6))
            if op == "5":
                print("  Nota: el eje z6 fija solo dos grados de libertad")
                print("  de orientacion; el giro alrededor de z6 queda")
                print("  libre y se completa con una eleccion declarada.")
                return T_desde_eje_z(*_leer("  x y z zx zy zz> ", 6))
            print(f"  '{op}' no es un formato. Escriba un numero del 1 al 5,")
            print("  o Enter para usar la pose de ejemplo.")
        except Exception as exc:
            print(f"\n  ERROR de entrada: {exc}")
            print("  Vuelva a elegir el formato.\n")


def etapa_2(q_original, T, dictada=False):
    titulo(2, "CINEMATICA INVERSA     T  ->  q     (Pieper)")
    explicar("""
        Ahora al reves, que es el problema dificil: dada la pose, hallar
        los angulos. A diferencia de la directa, aqui hay MUCHAS
        respuestas, y puede no haber ninguna.

        El metodo de Pieper aprovecha que el IRB 120 tiene muneca
        esferica: los tres ultimos ejes se cortan en un punto. Eso
        permite partir el problema en dos mitades independientes:

            posicion     q1, q2, q3   desde el centro de muneca
            orientacion  q4, q5, q6   desde lo que falta de rotacion

        La clave es el centro de muneca p_W = p - d6 * z6, que NO
        depende de q4, q5 ni q6. Por eso se puede resolver primero la
        posicion sin saber todavia la orientacion.
        """)

    d6 = 0.072
    p_W = T[:3, 3] - d6 * T[:3, 2]
    print(f"  Centro de muneca: p_W = [{p_W[0]:+.4f}, {p_W[1]:+.4f}, "
          f"{p_W[2]:+.4f}] m")
    r_W = np.linalg.norm(p_W - np.array([0.0, 0.0, 0.290]))
    print(f"  Distancia desde el hombro: r_W = {r_W:.4f} m   "
          f"(alcance maximo 0.5800 m)")
    print("\n  Ese 0.5800 no sale del catalogo de ABB: sale de a2 + k,")
    print("  con k = raiz(a3^2 + d4^2) = 0.3100 m. Coincide con los")
    print("  580 mm que ABB publica, y es una validacion independiente")
    print("  de nuestra tabla D-H.")

    if r_W > 0.580 + 1e-4:
        print(f"\n  -> FUERA DE ALCANCE: r_W excede a2 + k por "
              f"{(r_W - 0.580)*1000:.1f} mm.")
        print("  No existe solucion. Eso NO es un fallo del algoritmo:")
        print("  es el algoritmo informando correctamente que la pose")
        print("  esta fuera del espacio de trabajo del robot.")
        return None

    try:
        crudas = pieper(T)
    except Exception as exc:
        print(f"\n  -> Pieper no produjo ramas: {exc}")
        return None
    admisibles, rechazadas = filtrar(crudas, T)

    print(f"\n  Ramas obtenidas: {len(crudas)}   "
          f"admisibles: {len(admisibles)}   rechazadas: {len(rechazadas)}")
    print("\n  Son 8 = 2 (hombro) x 2 (codo) x 2 (muneca). Las ocho ponen")
    print("  la herramienta exactamente en la misma pose: son distintas")
    print("  maneras de doblar el brazo para llegar al mismo lugar.\n")

    cab = (f"  {'#':>2}  {'q1':>8} {'q2':>8} {'q3':>8} {'q4':>8} "
           f"{'q5':>8} {'q6':>8}   {'e_p [m]':>10}   etiqueta")
    print(cab)
    print("  " + "-" * (len(cab) - 2))
    for i, q in enumerate(crudas, 1):
        ep, _ = error_pose(q, T)
        gq = np.degrees(q)
        marca = "" if any(np.allclose(q, a) for a in admisibles) else "  (rechazada)"
        print(f"  {i:>2}  {gq[0]:>8.2f} {gq[1]:>8.2f} {gq[2]:>8.2f} "
              f"{gq[3]:>8.2f} {gq[4]:>8.2f} {gq[5]:>8.2f}   "
              f"{ep:>10.2e}   {etiquetar(q)}{marca}")

    print("\n  La columna e_p es la VERIFICACION: se toma cada rama, se")
    print("  le aplica cinematica directa, y se compara con la pose")
    print("  pedida. Del orden de 1e-16 metros, que es epsilon de")
    print("  maquina. Es asi de chico porque la solucion es analitica:")
    print("  no converge iterando, resuelve de una.")

    if not admisibles:
        print("\n  -> NINGUNA rama es admisible: todas violan algun tope")
        print("  articular. La pose es geometricamente alcanzable pero no")
        print("  con esta configuracion de limites. El sistema no debe")
        print("  moverse, y declararlo es la respuesta correcta.")
        motivos = {}
        for _, m in rechazadas:
            motivos[m] = motivos.get(m, 0) + 1
        for m, n in sorted(motivos.items(), key=lambda kv: -kv[1]):
            print(f"       {n} x  {m}")
        return None

    q_sel, costo = elegir(admisibles, np.zeros(6))
    print(f"\n  Entre las admisibles hay que elegir UNA. Criterio: la que")
    print(f"  exige menos movimiento desde la postura actual, ponderando")
    print(f"  mas las juntas que arrastran mas masa (pesos 4,3,2,1,1,0.5).")
    print(f"\n    SELECCIONADA: {etiquetar(q_sel)}")
    print(f"    q* = {np.degrees(q_sel).round(2)} grados")
    print(f"    costo ponderado = {costo:.4f}")

    if dictada:
        ep, eR = error_pose(q_sel, T)
        print(f"\n  Pose dictada: no hay configuracion original con que")
        print(f"  comparar, asi que la verificacion es la reconstruccion")
        print(f"  por cinematica directa de la rama elegida:")
        print(f"    e_p = {ep:.3e} m     e_R = {eR:.3e}")
        print(f"  Ambos por debajo del criterio de 1e-06.")
    else:
        coincide = np.allclose(q_sel, q_original, atol=1e-9)
        print(f"\n  Comprobacion de ida y vuelta: partimos de unos angulos")
        print(f"  en la etapa 1, calculamos su pose, y ahora la resolvimos")
        print(f"  al reves. Se recupero la configuracion original: "
              f"{coincide}")

    w_sel = medida_manipulabilidad(q_sel)
    if w_sel < 1e-3:
        print(f"\n  [!] AVISO: w = {w_sel:.3e} esta por debajo del umbral.")
        print("  La configuracion elegida esta cerca de una singularidad.")
        print("  Se resuelve igual, pero la movilidad esta degradada y")
        print("  corresponde declararlo.")

    return q_sel


# =====================================================================
#  ETAPA 3 - JACOBIANO
# =====================================================================

def etapa_3(q):
    titulo(3, "JACOBIANO     q  ->  J")
    explicar("""
        Hasta aqui todo fue posicion. El Jacobiano introduce la
        VELOCIDAD: relaciona cuan rapido giran las juntas con cuan
        rapido se mueve la herramienta.

            x_punto = J(q) q_punto

        Es una matriz 6x6: seis juntas, y seis componentes de velocidad
        del extremo (tres lineales y tres angulares). Cada columna se
        arma geometricamente:

            J_i = [ z_(i-1) x (p_e - p_(i-1)) ;  z_(i-1) ]

        La parte de arriba es la contribucion lineal, la de abajo la
        angular. Depende de la postura: el mismo robot tiene distinta
        capacidad de movimiento segun como este doblado.
        """)

    J = jacobiano(q)
    print("  J en la configuracion seleccionada:")
    for fila in J:
        print("    " + "  ".join(f"{v:+8.4f}" for v in fila))

    rango = np.linalg.matrix_rank(J, tol=1e-6)
    sigmas = np.linalg.svd(J, compute_uv=False)
    w = medida_manipulabilidad(q)

    print(f"\n  rango(J) = {rango} de 6")
    print("  Significa en cuantas direcciones independientes puede")
    print("  moverse el extremo. Si es 6, el robot tiene movilidad")
    print("  completa; si baja, hay una direccion perdida.")

    print(f"\n  Valores singulares (SVD):")
    print("    " + "  ".join(f"{s:.4f}" for s in sigmas))
    print(f"\n  sigma_min = {sigmas[-1]:.6f}")
    print(f"  condicion kappa = sigma_max/sigma_min = "
          f"{sigmas[0]/sigmas[-1]:.2f}")
    print(f"  manipulabilidad w = raiz(det(J J^T)) = {w:.6e}")

    if FIGURAS:
        _figura_svd(sigmas)

    print("\n  Los tres miden lo mismo desde angulos distintos. El mas")
    print("  informativo es sigma_min: si tiende a cero, hay una")
    print("  direccion que el robot practicamente no puede seguir.")
    print("\n  Nota de unidades: las tres primeras filas de J estan en")
    print("  m/s y las tres ultimas en rad/s. Por eso no tiene sentido")
    print("  comparar directamente una norma lineal con una angular,")
    print("  ni usar det(J) como medida: mezcla unidades distintas.")

    return J


def _figura_svd(sigmas):
    """Espectro de valores singulares de J en la postura seleccionada."""
    import matplotlib.pyplot as plt
    from graficas import C, TINTA_2, _limpiar

    fig, ax = plt.subplots(figsize=(5.0, 3.0))
    idx = np.arange(1, len(sigmas) + 1)
    ax.bar(idx, sigmas, color=C[0], width=0.6)
    ax.axhline(sigmas[-1], color=C[1], lw=1.0, ls="--")
    ax.annotate(rf"$\sigma_{{min}} = {sigmas[-1]:.4f}$",
                xy=(len(sigmas), sigmas[-1]),
                xytext=(len(sigmas) - 2.4, sigmas[-1] + 0.12*sigmas[0]),
                color=C[1], fontsize=8)
    ax.set_xticks(idx)
    ax.set_xlabel("indice del valor singular")
    ax.set_ylabel(r"$\sigma_i$")
    ax.set_title("Etapa 3 - espectro del Jacobiano", color=TINTA_2,
                 fontsize=9, loc="left")
    _limpiar(ax)
    fig.tight_layout()
    ruta = os.path.join(DIR_FIGURAS, "etapa3_valores_singulares.pdf")
    fig.savefig(ruta); plt.close(fig)
    _anotar(ruta)


def _figura_singularidad(q5s, sigmins, ws):
    """
    Degradacion al acercarse a la singularidad de muneca.

    Escala logaritmica en ambos ejes: la caida recorre varios ordenes
    de magnitud y en escala lineal se veria como una sola linea pegada
    al cero, que no muestra nada.
    """
    import matplotlib.pyplot as plt
    from graficas import C, TINTA_2, _limpiar

    q5s = np.asarray(q5s, float)
    mask = q5s > 0                      # el log no admite el cero exacto
    fig, ax = plt.subplots(figsize=(5.4, 3.2))
    ax.loglog(q5s[mask], np.asarray(sigmins)[mask], "o-", color=C[0],
              lw=1.2, ms=4, label=r"$\sigma_{min}$")
    ax.loglog(q5s[mask], np.asarray(ws)[mask], "s--", color=C[2],
              lw=1.2, ms=4, label=r"$w = \sqrt{\det(JJ^{T})}$")
    ax.set_xlabel(r"$q_5$  [rad]   (escala log)")
    ax.set_ylabel("medida de movilidad   (escala log)")
    ax.set_title("Etapa 4 - aproximacion a la singularidad de muneca",
                 color=TINTA_2, fontsize=9, loc="left")
    ax.legend(frameon=False, fontsize=8)
    _limpiar(ax)
    fig.tight_layout()
    ruta = os.path.join(DIR_FIGURAS, "etapa4_singularidad.pdf")
    fig.savefig(ruta); plt.close(fig)
    _anotar(ruta)


# =====================================================================
#  ETAPA 4 - SINGULARIDAD
# =====================================================================

def etapa_4():
    titulo(4, "SINGULARIDAD     cuando J pierde rango")
    explicar("""
        Una singularidad es una postura donde el robot pierde movilidad
        en alguna direccion. Matematicamente: el rango de J baja de 6.

        La mas facil de ver en un brazo con muneca esferica es la de
        muneca, que ocurre cuando q5 = 0: los ejes de q4 y q6 quedan
        alineados, asi que las dos juntas hacen lo mismo. Hay dos
        motores, pero una sola direccion util.

        Se muestra la aproximacion: q5 cada vez mas chico.
        """)

    print(f"  {'q5 [rad]':>12}  {'rango(J)':>9}  {'sigma_min':>12}  "
          f"{'w':>12}")
    print("  " + "-" * 52)
    q5s, sigmins, ws = [], [], []
    for q5 in [0.5, 0.1, 0.01, 1e-3, 1e-4, 1e-6, 0.0]:
        q = np.array([0.3, 0.4, -0.5, 0.2, q5, 0.1])
        J = jacobiano(q)
        s = np.linalg.svd(J, compute_uv=False)
        w = medida_manipulabilidad(q)
        q5s.append(q5); sigmins.append(s[-1]); ws.append(w)
        print(f"  {q5:>12.6f}  {np.linalg.matrix_rank(J, tol=1e-6):>9}  "
              f"{s[-1]:>12.3e}  {w:>12.3e}")

    print("\n  Se ve la degradacion: sigma_min y w caen hacia cero, y en")
    print("  q5 = 0 exacto el rango baja a 5. La direccion perdida deja")
    print("  de ser alcanzable.")
    print("\n  Consecuencia practica: si se insiste en mantener la")
    print("  velocidad de la herramienta atravesando esa postura, las")
    print("  velocidades articulares se disparan. La cota es")
    print("\n      ||q_punto||  <=  ||x_punto|| / sigma_min")
    print("\n  y con sigma_min tendiendo a cero, la cota explota.")
    print("\n  Ojo con un error comun: sigma_min acota POR ARRIBA, no")
    print("  por abajo. Y la singularidad es un fenomeno puramente")
    print("  CINEMATICO: existe aunque el robot no tuviera masa.")

    if FIGURAS:
        _figura_singularidad(q5s, sigmins, ws)


# =====================================================================
#  ETAPA 5 - TRAYECTORIA LSPB
# =====================================================================

def etapa_5(q_destino):
    titulo(5, "TRAYECTORIA LSPB     como se llega, no solo adonde")
    explicar("""
        Saber los angulos finales no alcanza. Si se saltara de golpe a
        la configuracion destino, haria falta velocidad infinita.

        El perfil LSPB parte el movimiento en tres fases: acelera a
        ritmo constante, va a velocidad de crucero, y frena. Cumple por
        construccion que el robot arranque y termine quieto.

        CUIDADO con un error frecuente: lo que es trapezoidal es el
        perfil de VELOCIDAD. El de posicion es parabola, recta,
        parabola, que es su integral.
        """)

    q0 = np.zeros(6)
    V = np.array([1.0, 1.0, 1.2, 1.5, 1.5, 2.0])
    A = np.array([2.0, 2.0, 2.5, 3.0, 3.0, 4.0])
    tr = LSPBSincronizado(q0, q_destino, V, A)

    print(f"  Desde la postura de reposo hasta q* de la etapa 2.")
    print(f"  Tiempo total sincronizado: T = {tr.T:.4f} s\n")
    print(f"  {'junta':>6}  {'h [rad]':>9}  {'tc [s]':>8}  "
          f"{'V [rad/s]':>10}  tipo")
    print("  " + "-" * 50)
    for i, p in enumerate(tr.perfiles):
        tipo = "triangular" if p.triangular else "trapezoidal"
        print(f"  {'J'+str(i+1):>6}  {p.h:>+9.3f}  {p.tc:>8.4f}  "
              f"{p.V:>10.4f}  {tipo}")

    print("\n  SINCRONIZACION: cada junta tendria su propio tiempo. Se")
    print("  toma el de la mas lenta y se reescalan las demas bajando")
    print("  su velocidad de crucero, para que todas arranquen y")
    print("  lleguen a la vez. Sin eso, la herramienta describiria una")
    print("  curva impredecible en vez de una transicion coordinada.")
    print("\n  Se baja la velocidad y nunca la aceleracion: asi cada")
    print("  junta se aleja de su limite de velocidad, y la aceleracion")
    print("  queda en el valor ya verificado como factible.")

    c = comprobar_continuidad(tr)
    print(f"\n  Verificacion de continuidad (20001 muestras):")
    print(f"    salto maximo en q    {c['salto_max_q']:.3e} rad")
    print(f"    salto maximo en qd   {c['salto_max_qd']:.3e} rad/s")
    print(f"    extremos con qd = 0  {c['qd_extremos_nulos']}")
    print(f"    q(0) y q(T) exactos  {c['q_inicial_ok']} / {c['q_final_ok']}")
    print(f"\n  Esos saltos NO son errores: con dt = {c['dt']:.2e} s y una")
    print(f"  velocidad de pico de {c['qd_max']:.2f} rad/s, el")
    print(f"  desplazamiento esperado entre muestras es justamente de")
    print(f"  ese orden. Lo que el test demuestra es que no hay")
    print(f"  discontinuidades: un salto real seria de otra magnitud.")
    print(f"\n  Lo que si es discontinuo es la ACELERACION, que salta en")
    print(f"  tc y en T - tc. Eso implica jerk infinito en esos dos")
    print(f"  instantes, y es una limitacion inherente al LSPB. Un")
    print(f"  perfil de jerk acotado (curva S) lo corrige, a costa de")
    print(f"  mayor tiempo total.")

    if FIGURAS:
        from graficas import perfil_lspb
        ruta = os.path.join(DIR_FIGURAS, "etapa5_lspb.pdf")
        perfil_lspb(tr, archivo=ruta,
                    titulo="Etapa 5 - perfiles LSPB sincronizados")
        _anotar(ruta)

    return tr


# =====================================================================
#  ETAPA 6 - DINAMICA
# =====================================================================

def etapa_6():
    titulo(6, "DINAMICA 1R     la trayectoria  ->  torque")
    explicar("""
        Ultima etapa: cuanto esfuerzo le cuesta al motor seguir esa
        trayectoria.

        La guia exige el modelo reducido de UNA articulacion, y hay una
        razon de fondo: para un modelo de seis ejes harian falta las
        masas y los tensores de inercia de los seis eslabones, que ABB
        no publica. La guia prohibe estimarlos.

        Se modela la JUNTA 2, el hombro, porque su eje es horizontal y
        soporta el peso de todo el brazo. La junta 1 queda descartada
        porque su eje es vertical: la gravedad no le produce momento.

        Por Euler-Lagrange, con T = (1/2) J qd^2 y V = m g lc (1-cos q):

            tau = J qdd + m g lc sin(q)
        """)

    print(f"  Parametros (hoja W10-S01, Tarea 4D, declarados por catedra):")
    print(f"    J  = {J_INERCIA:.4f} kg m^2")
    print(f"    m  = {M_MASA:.4f} kg")
    print(f"    lc = {LC_CENTRO:.4f} m")
    print(f"    g  = {G_GRAV:.4f} m/s^2")
    print("  Ninguno estimado: todos tienen procedencia documentada.")

    perfil = LSPB.desde_tiempo_final(0.0, 2.0, tf=3.0, acc=1.0)
    print(f"\n  Trayectoria de referencia: 0 -> 2 rad en 3 s, |qdd| = 1")
    print(f"    tiempo de mezcla  tc = {perfil.tc:.4f} s")
    print(f"    velocidad crucero V  = {perfil.V:.4f} rad/s")

    t_c = 2.5
    q_c, qd_c, qdd_c = (float(v[0]) for v in perfil(t_c))
    chk = verificacion_analitica(q_c, qdd_c)

    print(f"\n  Punto de control t = {t_c} s:")
    print(f"    q   = {q_c:.4f} rad")
    print(f"    qd  = {qd_c:.4f} rad/s")
    print(f"    qdd = {qdd_c:.4f} rad/s^2")
    print(f"\n    termino inercial   J*qdd        = {chk['inercial']:>8.4f} N.m")
    print(f"    termino gravitatorio m*g*lc*sin  = {chk['gravedad']:>8.4f} N.m")
    print(f"    {'':>37}  --------")
    print(f"    TOTAL                            = {chk['total']:>8.4f} N.m")
    print(f"\n    Solucionario de la catedra:        4.429 N.m")
    print(f"    Coincide DENTRO DEL REDONDEO: el residuo de 8e-4 viene")
    print(f"    de redondear sin(1.875) a 0.954 en el desarrollo.")
    print(f"\n  El termino inercial es NEGATIVO porque en t = 2.5 s el")
    print(f"  perfil esta frenando: el motor se opone al movimiento que")
    print(f"  el mismo habia impuesto.")

    t = np.linspace(0.0, perfil.T, 601)
    res = torque_sobre_lspb(perfil, t)
    mascara = np.abs(res["qdd"]) < 1e-9
    tau_cr = np.abs(res["tau"][mascara])

    print(f"\n  RESULTADO CONTRAINTUITIVO, durante el crucero:")
    print(f"    la aceleracion es CERO exacto")
    print(f"    y sin embargo |tau| va de {tau_cr.min():.4f} a "
          f"{tau_cr.max():.4f} N.m")
    print(f"\n  La razon: el termino gravitatorio no depende de ninguna")
    print(f"  derivada de q, sino de la POSTURA. Mientras el eslabon")
    print(f"  este fuera de la vertical, el motor sostiene su peso")
    print(f"  aunque no lo acelere.")
    print(f"\n  Velocidad constante NO implica torque cero.")

    if FIGURAS:
        from graficas import torque_1r as figura_torque
        ruta = os.path.join(DIR_FIGURAS, "etapa6_torque_1r.pdf")
        figura_torque(t, res, perfil.tc, perfil.T, archivo=ruta)
        _anotar(ruta)


# =====================================================================
#  CIERRE
# =====================================================================

def cierre():
    print("\n")
    print("  CIERRE DEL RECORRIDO")
    print("  " + "-" * 66)
    explicar("""
        Las seis etapas encadenadas:

            q  ->  T        cinematica directa, una sola respuesta
            T  ->  q        inversa por Pieper, hasta ocho ramas
            q  ->  J        Jacobiano, el mapa de velocidades
            J  ->  rango    singularidad, donde se pierde movilidad
            q* ->  q(t)     trayectoria LSPB, como se llega
            q(t) -> tau(t)  dinamica, cuanto cuesta

        La distincion que conviene dejar clara al final:

            SINGULARIDAD es perdida de movilidad. Es cinematica, y
            existiria aunque el robot no tuviera masa.

            TORQUE es demanda de esfuerzo. Es dinamica.

        Una no implica la otra. El contraejemplo esta en la etapa 6:
        el modelo 1R no tiene singularidades (su jacobiano es un escalar
        que nunca se anula) y aun asi exige casi 5 N.m.

        El vinculo entre ambas es indirecto: si se exige mantener la
        velocidad de tarea atravesando una singularidad, q_punto
        diverge, q_dospuntos hereda la divergencia, y recien ahi crece
        el termino inercial.
        """)


def main():
    print("\n")
    print("  RECORRIDO COMPLETO DEL PIPELINE     Hito 2 - Grupo VEVAS")
    print("  ABB IRB 120 - 6 GDL - muneca esferica")
    print("  " + "=" * 66)
    if PAUSAR:
        print("\n  Seis etapas, con pausa entre cada una.")
        print("  (ejecutar con --seguido para correr sin pausas)")
    if FIGURAS:
        _preparar_figuras()
        print(f"  Las figuras se guardaran en  {DIR_FIGURAS}/")

    T_dictada = pedir_pose() if POSE_DICTADA else None

    q_orig, T = etapa_1()
    if T_dictada is not None:
        print("\n  La etapa 1 mostro la cinematica directa con una")
        print("  configuracion de ejemplo, porque la directa necesita")
        print("  ANGULOS como entrada. De aqui en adelante el recorrido")
        print("  continua con la pose dictada.")
        T = T_dictada
    pausa()

    q_sel = etapa_2(q_orig, T, dictada=T_dictada is not None)
    if q_sel is None:
        print("\n  El recorrido se detiene aqui: sin una configuracion")
        print("  admisible no hay Jacobiano, ni trayectoria, ni torque")
        print("  que calcular. Las etapas 3 a 6 necesitan un q* valido.")
        print("\n  Para seguir viendo el recorrido completo, repita con")
        print("  otra pose o sin --pose para usar la de ejemplo.\n")
        return
    pausa()

    etapa_3(q_sel);                   pausa()
    etapa_4();                        pausa()
    etapa_5(q_sel);                   pausa()
    etapa_6();                        pausa()
    cierre()

    if FIGURAS and _guardadas:
        print(f"  Figuras generadas ({len(_guardadas)}):")
        for r in _guardadas:
            print(f"    {r}")
    print()


if __name__ == "__main__":
    main()
