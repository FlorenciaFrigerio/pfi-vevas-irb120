# Módulos

Los módulos numéricos y el demo corren con Python 3 y NumPy. Matplotlib se usa al generar figuras. **La ruta por consola no requiere ROS.** El nodo opcional nodo_hito2.py sí requiere ROS 2. Los módulos se importan entre sí como hermanos, así que se ejecutan desde esta carpeta.

```bash
cd ros2_irb120_description/scripts
python3 demo_recorrido.py
```

---

## Empezar por acá

| Archivo | Qué hace |
| --- | --- |
| **`demo_recorrido.py`** | **El recorrido narrado.** Explica seis etapas: directa, inversa, Jacobiano, singularidad, trayectoria y dinámica. La etapa de torque usa su propio perfil LSPB 1R de referencia. Explica qué hace cada bloque y qué significa el resultado. |

```bash
python3 demo_recorrido.py              # pausa entre etapas
python3 demo_recorrido.py --seguido    # de corrido
python3 demo_recorrido.py --pose       # con una pose dictada
python3 demo_recorrido.py --figuras    # guarda una figura por etapa
```

---

## Hito 2 — Cinemática inversa, trayectorias y dinámica

### El núcleo

| Archivo | Qué hace | Se verifica con |
| --- | --- | --- |
| `fk_irb120.py` | Tabla D-H, cinemática directa, límites articulares. **Base de todo lo demás.** | Cierre analítico en q = 0 |
| `pieper.py` | Cinemática inversa analítica: hasta 8 ramas, filtrado por topes y selección por costo ponderado | Reconstrucción FK sobre 2000 poses |
| `jacobiano.py` | Jacobiano geométrico 6×6 y medidas de movilidad | Derivación manual y diferencias finitas |
| `lspb.py` | Perfiles de trayectoria y sincronización multi-junta | Continuidad sobre 20 001 muestras |
| `singularidad.py` | Experimento de aproximación a la singularidad de muñeca | Caída de σ_min y colapso del rango |
| `dinamica_1r.py` | Modelo reducido 1R de la junta 2 sobre la trayectoria LSPB | Solucionario de la cátedra |

**El orden de dependencia:** `fk_irb120` ← `pieper` ← `jacobiano` ←
`singularidad`; `lspb` ← `dinamica_1r`. Ningún módulo depende de los
que están más abajo en la tabla.

### Herramientas

| Archivo | Qué hace |
| --- | --- |
| `resolver_en_vivo.py` | Resolvedor interactivo para la defensa. Acepta la pose en cinco formatos; la opción 7 explica qué significa cada número |
| `capturar_resultados.py` | Ejecuta los cinco módulos y genera el registro reproducible en `hito2_cinematica_dinamica/docs/resultados/` |
| `graficas.py` | Figuras del informe, con paleta validada para daltonismo |

### Extras, no exigidos por la guía

| Archivo | Qué hace |
| --- | --- |
| `dinamica.py` | Modelo 2R planar. **Experimento secundario declarado:** sus parámetros inerciales son una estimación, no especificación de ABB |
| `nodo_hito2.py` | Nodo ROS 2 que ejecuta el pipeline en vivo |

---

## Hito 1 — Gemelo digital URDF

| Archivo | Qué hace |
| --- | --- |
| `generate_urdf.py` | Deriva el URDF desde la tabla D-H, con verificación sin ROS |
| `compare_urdf.py` | Comprueba que el URDF manual y el derivado son el mismo robot |
| `validate_tf.py` | Valida contra el árbol TF de ROS 2 |
| `add_visuals.py` | Agrega la geometría visual al URDF |

`fk_irb120.py` es compartido: el Hito 1 lo usa como referencia analítica
contra la que validar el URDF.

---

## Correrlos de a uno

```bash
python3 fk_irb120.py       # cinematica directa y cierre
python3 pieper.py          # 2000 poses, 8 ramas, 0 fallas
python3 jacobiano.py       # las dos validaciones
python3 lspb.py            # perfiles y continuidad
python3 singularidad.py    # experimento de singularidad
python3 dinamica_1r.py     # modelo 1R sobre la trayectoria
python3 dinamica.py        # validaciones estructurales del 2R
```

Cada uno imprime sus propios resultados con el criterio de aceptación
declarado.

---

## Dónde está lo demás

| Qué | Dónde |
| --- | --- |
| Qué es este paquete y qué script es de cada hito | [`../README.md`](../README.md) |
| Cada módulo explicado línea por línea | [`hito2_cinematica_dinamica/docs/guias/codigo/`](../../hito2_cinematica_dinamica/docs/guias/codigo/) |
| Manual de estudio y defensa | [`hito2_cinematica_dinamica/docs/guias/MANUAL_HITO2.md`](../../hito2_cinematica_dinamica/docs/guias/MANUAL_HITO2.md) |
| Salida literal de los cinco módulos | [`hito2_cinematica_dinamica/docs/resultados/`](../../hito2_cinematica_dinamica/docs/resultados/) |
| Informe IEEE del Hito 2 | [`hito2_cinematica_dinamica/docs/informe/Informe_Hito2.pdf`](../../hito2_cinematica_dinamica/docs/informe/Informe_Hito2.pdf) |
| Cuaderno reproducible | [`hito2_cinematica_dinamica/notebooks/Hito2_VEVAS.ipynb`](../../hito2_cinematica_dinamica/notebooks/Hito2_VEVAS.ipynb) |
| Material del Hito 1 (informe, validación) | [`hito1_gemelo_digital/`](../../hito1_gemelo_digital/README.md) |
