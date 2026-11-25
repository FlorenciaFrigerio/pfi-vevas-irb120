![Proyecto final de Robótica · Grupo VEVAS](01_docs/06_figuras/00_proyecto.png)

[Catálogo numerado de todos los archivos](01_docs/07_CATALOGO_DE_ARCHIVOS.md)

# Proyecto final de Robótica · Grupo VEVAS

El grupo construyó una representación del ABB IRB 120, resolvió cómo llevar su herramienta a una pose y llevó esos cálculos a una consola que permite contrastar el plan con lo observado en el controlador. Cada hito conserva el informe, la explicación del método y sus resultados.

## Informes y defensa

| Material | Para qué sirve |
| --- | --- |
| [Informe del Hito 1](02_hito1_gemelo_digital/01_docs/01_informe/01_Informe_Hito1_VEVAS.pdf) | Geometría, URDF y marcos de referencia. |
| [Informe del Hito 2](03_hito2_cinematica_dinamica/01_docs/01_informe/01_Informe_Hito2.pdf) | Cinemática inversa, singularidades, trayectorias y torque. |
| [Informe del Hito 3/4](04_hito3_4_control_real/01_docs/01_informe/01_Informe_Hito3_4.pdf) | Integración de VEVAS Studio con el ABB IRC5. |

[Diapositivas de defensa](04_hito3_4_control_real/01_docs/02_presentacion/02_Defensa_Grupo_VEVAS.pdf) · [Guía de entrega del 24/11](04_hito3_4_control_real/01_docs/03_guias/02_Guia_aportada_24NOV.md)

## Recorrido del proyecto

| Material | Para qué sirve |
| --- | --- |
| [01 · Gemelo digital](02_hito1_gemelo_digital/README.md) | Explorar el robot articulado y contrastar TF con la cinemática directa. |
| [02 · Cinemática y dinámica](03_hito2_cinematica_dinamica/README.md) | Elegir una rama de Pieper y calcular una trayectoria sincronizada. |
| [03–04 · Control del robot](04_hito3_4_control_real/README.md) | Examinar el plan, la operación de la consola y la evidencia de laboratorio. |
| [VEVAS Studio](04_hito3_4_control_real/02_vevas_studio/README.md) | Abrir la documentación del software, las pruebas y el simulador. |

## Cómo interpretar las evidencias

| Hito | Qué se compara | Dónde está el resultado |
| --- | --- | --- |
| 1 | Pose analítica frente a transformación TF | [Registro de las 501 muestras válidas](02_hito1_gemelo_digital/01_docs/04_resultados/01_validacion_hito1.txt) |
| 2 | Pose objetivo frente a la pose recuperada por cinemática directa | [Registro numérico del Hito 2](03_hito2_cinematica_dinamica/01_docs/04_resultados/resultados.json) |
| 3/4 | Velocidades por junta y observaciones del controlador | [Figura de la entrega C02](01_docs/06_figuras/05_velocidades_c02.png) |

El orden de los hitos también sirve para investigar un resultado inesperado. Primero se verifica la geometría y la convención de los ejes. Luego se comprueba si la postura y la trayectoria son admisibles. Finalmente se contrasta lo esperado con la evidencia fechada del controlador.

## Equipo y materia

**Grupo VEVAS** · Robótica IMT-342 · Gestión II-2026
Carrera de Ingeniería Mecatrónica
Departamento de Ciencias de la Tecnología e Innovación
Universidad Católica Boliviana "San Pablo", Sede Tarija

Susann A. Baldiviezo Castellanos · Alejandro J. Bejarano Romero ·
Florencia Frigerio Arízaga · Benjamín Soruco Guzmán

Docente: M.Sc. Ing. Bernardo Quiroga Turdera

<details>
<summary>Resultados y notas técnicas</summary>

# HITO 2 — Cinemática inversa, trayectorias, singularidades y dinámica

```
T_d → Pieper → FK + límites → q*
                     ├── Jacobiano → singularidad
                     └── LSPB sincronizado de seis juntas

Perfil LSPB 1R de referencia → dinámica reducida → τ(t)
```

## Resultados

| Indicador | Valor |
| --- | --- |
| Ramas analíticas por pose, 2000 poses | 8.00 |
| Error de posición máximo | ~5 × 10⁻¹⁶ m |
| Poses sin solución | 0 de 2000 |
| Jacobiano, validación manual | 2.08 × 10⁻¹⁸ |
| Jacobiano, diferencias finitas | 3.25 × 10⁻⁸ |
| Rango de J en la singularidad de muñeca | 5 de 6 |
| Residuo ante tarea incompatible | 0.442357 |
| τ(2.5 s) del modelo 1R | 4.4298 N·m (referencia: 4.429) |
| Alcance a₂ + k vs. fabricante | 0.580 m / 0.580 m |

El registro completo, con semillas y entorno, está en
[resultados.json](03_hito2_cinematica_dinamica/01_docs/04_resultados/resultados.json).

## Pruebas dirigidas

Tres fallos de robustez sobrevivieron a 2000 ensayos aleatorios y solo
aparecieron bajo pruebas dirigidas a fronteras y singularidades:

| Prueba | Antes | Después |
| --- | --- | --- |
| Poses con q₅ entre 1e-6 y 3e-5 | falla | resuelve |
| 1800 poses sobre los topes, rechazos | 193 | 0 |
| Giro de J6 ante pose ya alcanzada | 6.283 rad | 0.000 rad |

Una muestra uniforme del espacio de trabajo visita las fronteras con
probabilidad despreciable: si el fallo vive sobre un tope, hay que ir a
buscarlo ahí. El detalle está en
[02_anexo_tecnico.md](03_hito2_cinematica_dinamica/01_docs/01_informe/02_anexo_tecnico.md), sección 4.

## Módulos

La ruta numérica por consola usa Python 3 y NumPy; Matplotlib se necesita para generar gráficas. El nodo opcional nodo_hito2.py requiere ROS 2.

| Archivo | Qué hace |
| --- | --- |
| `fk_irb120.py` | tabla D-H, cinemática directa, límites articulares |
| `pieper.py` | cinemática inversa analítica, 8 ramas, filtrado, selección |
| `jacobiano.py` | Jacobiano geométrico y sus dos validaciones |
| `lspb.py` | perfiles de trayectoria y sincronización multi-junta |
| `singularidad.py` | experimento de aproximación a la singularidad |
| `dinamica_1r.py` | modelo reducido 1R sobre la trayectoria LSPB |
| `graficas.py` | figuras, con paleta validada para daltonismo |
| `demo_recorrido.py` | **seis etapas narradas; torque 1R con perfil de referencia separado** |
| `resolver_en_vivo.py` | resolvedor interactivo para la defensa |
| `capturar_resultados.py` | genera el registro reproducible de resultados |

Extras, no exigidos por la guía del Hito 2:

| Archivo | Qué hace |
| --- | --- |
| `dinamica.py` | modelo 2R planar, experimento secundario declarado |
| `nodo_hito2.py` | nodo ROS 2 que ejecuta el pipeline en vivo |

## Cómo correrlo

```bash
git clone https://github.com/FlorenciaFrigerio/pfi-vevas-irb120.git
cd pfi-vevas-irb120/05_ros2_irb120_description/04_scripts

# El recorrido completo, narrado
python3 demo_recorrido.py                    # pausa entre etapas
python3 demo_recorrido.py --seguido          # de corrido
python3 demo_recorrido.py --pose --figuras   # con pose dictada y figuras

# Modulos individuales
python3 pieper.py          # verificacion FK sobre 2000 poses
python3 jacobiano.py       # validacion manual y por diferencias finitas
python3 lspb.py            # perfiles y continuidad
python3 singularidad.py    # experimento de singularidad
python3 dinamica_1r.py     # modelo 1R sobre la trayectoria

# Cuaderno completo
cd ../../03_hito2_cinematica_dinamica/02_notebooks
python3 -m nbconvert --to notebook --execute --inplace 01_Hito2_VEVAS.ipynb

# Registro completo de resultados
cd ../../05_ros2_irb120_description/04_scripts
python3 capturar_resultados.py
```

### Resolver una pose en vivo

```bash
python3 resolver_en_vivo.py          # menu interactivo
python3 resolver_en_vivo.py --demo   # seis casos borde
```

Acepta la pose en cinco formatos: matriz homogénea, posición +
cuaternión, posición + matriz R, posición + RPY en grados, y posición +
eje z₆. Las cifras pueden ir en una línea o repartidas en varias. La
**opción 7** del menú explica qué significa cada número del informe.

### Compilar el informe

El fuente usa la clase `IEEEtran`. En Overleaf: New Project → Upload
Project con la carpeta [01_informe_ieee](03_hito2_cinematica_dinamica/01_docs/01_informe/01_informe_ieee/),
compilador pdfLaTeX, dos pasadas. Localmente requiere `texlive-publishers`.

## Documentación del Hito 2

| Dónde | Qué |
| --- | --- |
| [01_codigo](03_hito2_cinematica_dinamica/01_docs/03_guias/01_codigo/) | un archivo por módulo, explicado línea por línea |
| [03_guias](03_hito2_cinematica_dinamica/01_docs/03_guias/) | manuales de estudio, defensa y demostración |
| [02_anexo_tecnico.md](03_hito2_cinematica_dinamica/01_docs/01_informe/02_anexo_tecnico.md) | material de respaldo que no entró al informe |
| [04_resultados](03_hito2_cinematica_dinamica/01_docs/04_resultados/) | salida literal de los cinco módulos |

## Alcance y limitaciones

Declaradas explícitamente, porque un trabajo que no dice qué no probó
no es verificable:

- Los parámetros dinámicos son **pedagógicos**, declarados por la
  cátedra. ABB no publica masas ni inercias del IRB 120. Las
  conclusiones sobre la forma de τ(t) son válidas; las magnitudes
  absolutas no son especificación del robot real.
- La singularidad de hombro se verificó **solo por tendencia**.
- En las familias singulares exactas el conjunto de soluciones deja de
  ser discreto y contiene familias continuas; su tratamiento completo
  queda fuera de este hito.
- El modelo 2R valida su consistencia interna, **no** que represente
  las juntas 2 y 3 del IRB 120.
- El modelo dinámico ignora fricción y flexibilidad de transmisión.

---

# HITO 1 — Gemelo digital URDF

Descripción URDF generada programáticamente a partir de la tabla
Denavit-Hartenberg y validada contra la cinemática directa analítica.

## Web interactiva del brazo ABB

La carpeta [`08_web/`](02_hito1_gemelo_digital/02_visor_web/) contiene el explorador técnico del **ABB IRB
120**: modelo 3D, selección de conjuntos, vista de articulaciones y
celda de trabajo.

## Resultado

| Métrica | Valor | Umbral del hito |
|---|---|---|
| Error posicional máximo (TF vs. analítica) | 2,459 × 10⁻¹⁰ m | 1 × 10⁻⁶ m |
| Error posicional medio | 1,498 × 10⁻¹⁰ m | — |
| Error angular máximo | 1,092 × 10⁻⁹ | — |
| Muestras válidas | 501 de 1000 | — |
| Margen respecto al umbral | 4,1 × 10³ veces por debajo | — |

Verificación interna del generador (sin pasar por ROS):
**3,68 × 10⁻¹⁶ m**.

La diferencia entre ambas cifras no es error del modelo: TF transporta
las rotaciones como cuaterniones en `float32`, cuyo épsilon es
1,2 × 10⁻⁷. El modelo es exacto hasta el épsilon de `float64`; lo que se
mide vía TF es la precisión del transporte de ROS.

## Tabla Denavit-Hartenberg

Parametrización de Truc & Lam (2020), contrastada con las cotas del
manual técnico oficial de ABB.

| i | θᵢ | dᵢ [m] | aᵢ [m] | αᵢ | Interpretación |
|---|---|---|---|---|---|
| 1 | q₁ | 0,290 | 0 | −π/2 | Altura de la columna base |
| 2 | q₂ − π/2 | 0 | 0,270 | 0 | Brazo, ejes paralelos |
| 3 | q₃ | 0 | 0,070 | −π/2 | Codo, offset lateral corto |
| 4 | q₄ | 0,302 | 0 | +π/2 | Antebrazo hacia la muñeca |
| 5 | q₅ | 0 | 0 | −π/2 | Ejes concurrentes (a = d = 0) |
| 6 | q₆ | 0,072 | 0 | 0 | Brida de salida (tool0) |

### Verificación de cierre en q = 0

Sin multiplicar una sola matriz, las cotas del manual predicen la pose:

```
x = d₄ + d₆      = 0,302 + 0,072         = 0,374 m
z = d₁ + a₂ + a₃ = 0,290 + 0,270 + 0,070 = 0,630 m
```

### El offset θ₂ = q₂ − π/2

Hay dos ceros distintos y no coinciden. El cero mecánico de ABB
corresponde al brazo vertical (postura de calibración del fabricante).
El cero de D-H exige que x₁ y x₂ sean paralelos, lo que ocurre con el
brazo horizontal. El offset reconcilia ambos y se codifica una sola vez
dentro de la transformación, nunca en el `joint_state`.

## Conversión D-H → URDF

URDF impone `T_padre→hijo = Origin · Rot(eje, q)` con `Origin`
constante, pero en D-H la variable de junta va al principio. Se resuelve
factorizando:

```
Aᵢ = Rz(qᵢ + offᵢ) · Tz(dᵢ) · Tx(aᵢ) · Rx(αᵢ)
   = Rz(offᵢ) · Rz(qᵢ) · Bᵢ        con  Bᵢ = Tz(dᵢ)·Tx(aᵢ)·Rx(αᵢ)
```

porque `Rz(a)·Rz(b) = Rz(a+b)`: las rotaciones en torno al mismo eje
conmutan. Reordenando la cadena, cada junta URDF recibe

```
Originᵢ = Bᵢ₋₁ · Rz(offᵢ)        con  B₀ = I
```

y una junta fija final con `B₆` cierra en la brida. Como `Bᵢ` es
traslación pura seguida de un giro en x, se reduce a `xyz="aᵢ 0 dᵢ"` y
`rpy="αᵢ 0 0"`.

**Ningún origen fue ajustado visualmente en RViz.**

## Los dos URDF

Se mantienen **dos** de forma deliberada:

| Archivo | Para qué |
| --- | --- |
| `01_urdf/irb120_manual.urdf` | escrito a mano, documenta el razonamiento de cada número |
| `01_urdf/irb120.urdf` | generado por script, lo demuestra (no editar a mano) |

`04_scripts/compare_urdf.py` verifica que sean el mismo robot.

## Visualización en RViz

Requiere Ubuntu 24.04 con ROS 2 Jazzy.

```bash
cd ~/ros2_ws && colcon build --packages-select irb120_description
source install/setup.bash
ros2 launch irb120_description display.launch.py
```

---

## Estructura del repositorio

[Mapa de las carpetas](01_docs/05_MAPA_DEL_REPOSITORIO.md) · [Catálogo completo](01_docs/07_CATALOGO_DE_ARCHIVOS.md)

## Convenciones

- Ramas `main`, `dev` y `feature/*`, según exige la guía del PFI.
- Conventional Commits.
- Las entregas cerradas se marcan con tag: `v1.0-hito1`, `v2.0-hito2`.

## Licencia

MIT. Ver [`LICENSE`](LICENSE).

</details>
