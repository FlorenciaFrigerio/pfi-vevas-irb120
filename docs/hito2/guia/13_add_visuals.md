# Script 13 - `add_visuals.py`: Geometría visual del URDF

> **Nivel:** Hito 1 - contexto visual

## Para qué sirve
Añade primitivas visuales y de colisión simplificadas al modelo URDF.

**Entrada y salida:** Entrada: tabla D-H y funciones del generador. Salida: URDF con geometría.

## Recorrido por bloques

### A. Primitivas
cilindro, esfera y colision producen fragmentos XML.

**Código real** (`scripts/add_visuals.py`, líneas 50-71):

```python
def cilindro(largo, radio, eje, color, nombre_mat):
    """
    Cilindro que arranca en el origen del marco y se extiende 'largo'
    metros a lo largo de 'eje' ('x' o 'z').

    En URDF el cilindro se dibuja centrado en su origen y alineado con
    z. Para que recorra el segmento hay que desplazarlo medio largo, y
    si el eje es x, rotarlo +pi/2 en y.
    """
    if largo <= 1e-9:
        return ""
    if eje == "z":
        origen = f'<origin xyz="0 0 {largo/2:.6f}" rpy="0 0 0"/>'
    else:
        origen = f'<origin xyz="{largo/2:.6f} 0 0" rpy="0 1.5707963 0"/>'
    return (f'    <visual>\n'
            f'      {origen}\n'
            f'      <geometry>\n'
            f'        <cylinder length="{largo:.6f}" radius="{radio:.4f}"/>\n'
            f'      </geometry>\n'
            + material(nombre_mat, color) +
            f'    </visual>\n')
```

**Cómo leerlo:** Son formas aproximadas para visualizar.

### B. Geometría por eslabón
geometria_eslabon selecciona a_i o d_i y coloca cilindro/articulación.

**Código real** (`scripts/add_visuals.py`, líneas 100-138):

```python
def geometria_eslabon(i):
    """
    Devuelve el bloque de <visual>/<collision> del eslabon i.

    i = 0  base_link, dibuja el pedestal (d_1 de la fila 1)
    i>= 1  link_i, dibuja el segmento correspondiente a la fila i+1
    """
    r = RADIOS[min(i, len(RADIOS) - 1)]
    bloque = ""

    if i == 0:
        # Pedestal: disco ancho y bajo. No sale de la tabla, es soporte.
        bloque += (f'    <visual>\n'
                   f'      <origin xyz="0 0 0.02" rpy="0 0 0"/>\n'
                   f'      <geometry>\n'
                   f'        <cylinder length="0.04" radius="0.11"/>\n'
                   f'      </geometry>\n'
                   + material("base_gris", COLOR_JUNTA) +
                   f'    </visual>\n')
        return bloque

    if i > 6:
        return bloque + esfera(r * 1.15, COLOR_JUNTA, "junta")

    _, d, a, _ = DH_TABLE[i - 1]

    # El eslabon recorre a_i en x local, o d_i en z local.
    if abs(a) > 1e-9:
        bloque += cilindro(abs(a), r, "x", COLOR_CUERPO, "cuerpo")
        bloque += colision(abs(a), r, "x")
    elif abs(d) > 1e-9:
        bloque += cilindro(abs(d), r, "z", COLOR_CUERPO, "cuerpo")
        bloque += colision(abs(d), r, "z")

    # La esfera va DESPUES del cilindro: RViz aplica a todo el link el
    # primer <material> que encuentra, asi que el cuerpo debe ir primero
    # para que el eslabon salga naranja y no gris.
    bloque += esfera(r * 1.15, COLOR_JUNTA, "junta")
    return bloque
```

**Cómo leerlo:** La muñeca sin traslación usa solo la representación de junta.

### C. Ensamble
construir conserva orígenes y agrega visual/collision; --plain omite geometría.

**Código real** (`scripts/add_visuals.py`, líneas 141-195):

```python
def construir(con_geometria=True):
    origins = gen.joint_origins()
    names = ["base_link"] + [f"link_{i}" for i in range(1, 7)] + ["tool0"]

    out = ['<?xml version="1.0"?>\n',
           '<!-- Generado por scripts/add_visuals.py  NO EDITAR A MANO -->\n',
           '<!-- Geometria por primitivas dimensionadas desde la tabla D-H -->\n',
           '<!-- Grupo VEVAS -->\n',
           '<robot name="irb120">\n\n']

    for i, n in enumerate(names):
        if not con_geometria:
            out.append(f'  <link name="{n}"/>\n')
            continue
        cuerpo = geometria_eslabon(i)
        if n == "tool0":
            # La brida se marca con un disco fino, sin cilindro largo.
            cuerpo = (f'    <visual>\n'
                      f'      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
                      f'      <geometry>\n'
                      f'        <cylinder length="0.012" radius="0.031"/>\n'
                      f'      </geometry>\n'
                      + material("brida", COLOR_JUNTA) +
                      f'    </visual>\n')
        out.append(f'  <link name="{n}">\n{cuerpo}  </link>\n')
    out.append("\n")

    for i in range(6):
        xyz, rpy = origins[i]
        lo, hi = gen.JOINT_LIMITS[i]
        out.append(
            f'  <joint name="joint_{i+1}" type="revolute">\n'
            f'    <parent link="{names[i]}"/>\n'
            f'    <child  link="{names[i+1]}"/>\n'
            f'    <origin xyz="{gen.fmt(xyz[0])} {gen.fmt(xyz[1])} '
            f'{gen.fmt(xyz[2])}" rpy="{gen.fmt(rpy[0])} {gen.fmt(rpy[1])} '
            f'{gen.fmt(rpy[2])}"/>\n'
            f'    <axis xyz="0 0 1"/>\n'
            f'    <limit lower="{gen.fmt(lo, 6)}" upper="{gen.fmt(hi, 6)}"'
            f' effort="{gen.EFFORT[i]}" velocity="{gen.VELOCITY[i]}"/>\n'
            f'  </joint>\n\n'
        )

    xyz, rpy = origins[6]
    out.append(
        f'  <joint name="joint_6_tool0" type="fixed">\n'
        f'    <parent link="link_6"/>\n'
        f'    <child  link="tool0"/>\n'
        f'    <origin xyz="{gen.fmt(xyz[0])} {gen.fmt(xyz[1])} '
        f'{gen.fmt(xyz[2])}" rpy="{gen.fmt(rpy[0])} {gen.fmt(rpy[1])} '
        f'{gen.fmt(rpy[2])}"/>\n'
        f'  </joint>\n\n'
        '</robot>\n'
    )
    return "".join(out)
```

**Cómo leerlo:** La cadena cinemática se verifica después de generar visuales.

## Cómo se relaciona
Depende de fk_irb120.py y generate_urdf.py.

## Pruebas y evidencia
El script llama a verify_chain tras escribir el URDF.

## Pregunta para practicar defensa
> ¿Qué dimensiones vienen de D-H y cuáles son decisiones de apariencia?

## Límite o precaución
No presentes radios ni primitivas como dimensiones oficiales del ABB.

## Ficha de repaso
- Problema que resuelve:
- Parámetros de entrada:
- Salida principal:
- Función clave:
- Prueba que lo respalda:
- Lo que no demuestra:
