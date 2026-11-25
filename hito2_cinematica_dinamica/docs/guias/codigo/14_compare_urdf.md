# Script 14 - `compare_urdf.py`: Comparación de URDF manual y derivado

> **Nivel:** Hito 1 - apoyo de verificación

## Para qué sirve
Compara los orígenes del modelo escrito a mano con el generado.

**Entrada y salida:** Entrada: dos URDF. Salida: diferencias de posición/ángulo y estado.

## Recorrido por bloques

### A. Parseo
leer_origenes extrae xyz/rpy por nombre de junta.

**Código real** (`scripts/compare_urdf.py`, líneas 19-30):

```python
def leer_origenes(path):
    root = ET.parse(path).getroot()
    datos = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        if o is None:
            datos[j.get("name")] = (np.zeros(3), np.zeros(3))
            continue
        xyz = np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
        rpy = np.array([float(v) for v in o.get("rpy", "0 0 0").split()])
        datos[j.get("name")] = (xyz, rpy)
    return datos
```

**Cómo leerlo:** Permite comparar automáticamente el contenido XML.

### B. Diferencia angular
ang_dif envuelve diferencias al intervalo angular principal.

**Código real** (`scripts/compare_urdf.py`, líneas 33-35):

```python
def ang_dif(a, b):
    """Diferencia angular envuelta a (-pi, pi]."""
    return abs((a - b + np.pi) % (2 * np.pi) - np.pi)
```

**Cómo leerlo:** Evita tratar dos representaciones equivalentes como una vuelta completa de diferencia.

### C. Resumen
El bloque principal revisa juntas faltantes y tolerancias.

**Código real** (`scripts/compare_urdf.py`, líneas 38-63):

```python
if __name__ == "__main__":
    man = leer_origenes(URDF_DIR / "irb120_manual.urdf")
    gen = leer_origenes(URDF_DIR / "irb120.urdf")

    print("\n   Manual vs derivado   comparacion de <origin>\n")

    peor_p, peor_a = 0.0, 0.0
    for nombre in man:
        if nombre not in gen:
            print(f"  {nombre:16s} ausente en el generado")
            continue
        xm, rm = man[nombre]
        xg, rg = gen[nombre]
        ep = float(np.linalg.norm(xm - xg))
        ea = max(ang_dif(rm[k], rg[k]) for k in range(3))
        peor_p, peor_a = max(peor_p, ep), max(peor_a, ea)
        estado = "OK" if ep < 1e-6 and ea < 1e-6 else "DIFIERE"
        print(f"  {nombre:16s} d_pos = {ep:.2e} m   "
              f"d_ang = {ea:.2e} rad   {estado}")

    print(f"\n  Peor discrepancia posicional: {peor_p:.3e} m")
    print(f"  Peor discrepancia angular:    {peor_a:.3e} rad")
    print("  RESULTADO:",
          "IDENTICOS" if peor_p < 1e-6 and peor_a < 1e-6
          else "REVISAR TRANSCRIPCION")
    print()
```

**Cómo leerlo:** Comprueba transcripción de orígenes, no la ejecución ROS.

## Cómo se relaciona
Compara urdf/irb120_manual.urdf y urdf/irb120.urdf.

## Pruebas y evidencia
El umbral escrito en el script es 1e-6 para posición y ángulo.

## Pregunta para practicar defensa
> ¿Por qué la diferencia angular debe envolverse?

## Límite o precaución
Coincidencia de origins no garantiza mallas, apariencia ni TF en vivo.

## Ficha de repaso
- Problema que resuelve:
- Parámetros de entrada:
- Salida principal:
- Función clave:
- Prueba que lo respalda:
- Lo que no demuestra:
