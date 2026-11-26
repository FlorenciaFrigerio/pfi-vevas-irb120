# Guía de ejecución · Hito 2 (Pieper, Singularidades, LSPB, Dinámica y Notebook)

[Volver al Hito 2](README.md) · [Manual completo](01_docs/03_guias/01_MANUAL_HITO2.md) · [Portada del proyecto](../README.md)

Todos los cálculos cinemáticos y dinámicos del Hito 2 funcionan en **Windows, macOS y Linux** usando únicamente Python 3, NumPy y Matplotlib.

---

## 1. Preparación del entorno

```bash
python -m pip install numpy matplotlib jupyter nbconvert ipykernel
```

---

## 2. Recorrido completo narrado para exposición o estudio

```bash
cd 05_ros2_irb120_description/04_scripts

# Opción A: de corrido (imprime las 6 etapas encadenadas de principio a fin)
python demo_recorrido.py --seguido

# Opción B: interactivo con pausa entre etapas (ideal para explicar en vivo)
python demo_recorrido.py

# Opción C: ingresando una pose dictada por el docente y guardando figuras PDF
python demo_recorrido.py --pose --figuras
```

---

## 3. Resolvedor interactivo en vivo (`5` formatos de pose)

```bash
cd 05_ros2_irb120_description/04_scripts

python resolver_en_vivo.py          # Menú interactivo (T 4x4, cuaternión, R+p, RPY en grados o eje z6)
python resolver_en_vivo.py --demo   # Ejecuta automáticamente 6 casos borde comentados
```

---

## 4. Ejecutar cada módulo de validación por separado

| Comando (dentro de `05_ros2_irb120_description/04_scripts`) | Qué verifica | Salida de referencia |
| --- | --- | --- |
| `python pieper.py` | 2.000 poses aleatorias + fronteras (`8.00` ramas/pose, error `~5e-16 m`) | [`salida_pieper.txt`](01_docs/04_resultados/salida_pieper.txt) |
| `python jacobiano.py` | Contraste contra derivación manual (`2e-18`) y diferencias finitas (`3e-8`) | [`salida_jacobiano.txt`](01_docs/04_resultados/salida_jacobiano.txt) |
| `python singularidad.py` | Rango `5` en $q_5=0$ y leyes de potencia $\|\dot{q}\|\sim q_5^{-1}$, $\|\tau\|\sim q_5^{-2}$ | [`salida_singularidad.txt`](01_docs/04_resultados/salida_singularidad.txt) |
| `python lspb.py` | Sincronización multi-junta y continuidad de posición/velocidad | [`salida_lspb.txt`](01_docs/04_resultados/salida_lspb.txt) |
| `python dinamica_1r.py` | Torque en $t = 2{,}5\text{ s}$ (`4,4298 N·m`) y esfuerzo no nulo en crucero | [`salida_dinamica_1r.txt`](01_docs/04_resultados/salida_dinamica_1r.txt) |
| `python capturar_resultados.py` | Regenera `resultados.json` y todos los `.txt` de respaldo | [`resultados.json`](01_docs/04_resultados/resultados.json) |

---

## 5. Abrir o re-ejecutar el Jupyter Notebook oficial

```bash
cd 03_hito2_cinematica_dinamica/02_notebooks

# Abrir en interfaz interactiva
jupyter notebook 01_Hito2_VEVAS.ipynb

# O ejecutar todas las celdas de forma automática por consola
python -m nbconvert --to notebook --execute --inplace 01_Hito2_VEVAS.ipynb
```
