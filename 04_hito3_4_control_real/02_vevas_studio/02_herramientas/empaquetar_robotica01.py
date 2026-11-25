#!/usr/bin/env python3
"""Crea una entrega independiente por áreas, sin cambiar el robot ni la PC del usuario.

Solo copia archivos versionados de VEVAS Studio (04_hito3_4_control_real/02_vevas_studio en el repo,
00_codigo/hito3_4 en el laboratorio), la documentación del hito y materiales explícitos opcionales.
No exporta el directorio .git, credenciales del sistema, venv ni sesiones locales.
El bundle opcional contiene el historial versionado de HEAD, tal como estaba.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

CODIGO_REPO = Path(__file__).resolve().parents[1]      # 04_hito3_4_control_real/02_vevas_studio
DOCS_REPO = CODIGO_REPO.parent / "01_docs"                 # 04_hito3_4_control_real/01_docs
REPO = Path(subprocess.check_output(["git", "-C", str(CODIGO_REPO), "rev-parse", "--show-toplevel"],
                                    text=True).strip())
CARPETA_CODIGO = CODIGO_REPO.relative_to(REPO).as_posix()
CARPETA_DOCS = DOCS_REPO.relative_to(REPO).as_posix()


def git(*args: str) -> bytes:
    return subprocess.check_output(["git", "-C", str(REPO), *args])


def escribir(raiz: Path, ruta: str, texto: str) -> None:
    destino = raiz / ruta
    destino.parent.mkdir(parents=True, exist_ok=True)
    destino.write_text(texto.strip() + "\n", encoding="utf-8")
    if destino.suffix == ".sh":
        destino.chmod(0o755)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--destino", type=Path, required=True, help="Directorio NUEVO de preparación")
    ap.add_argument("--zip", type=Path, required=True, help="ZIP NUEVO de salida")
    ap.add_argument("--original", type=Path, help="ZIP original recibido; se conserva sin extraer")
    ap.add_argument("--fuentes-docente", type=Path, help="Directorio de fuentes/PDF y procedencia")
    ap.add_argument("--historial", action="store_true", help="Incluye un git bundle de HEAD")
    args = ap.parse_args()
    if args.destino.exists() or args.zip.exists():
        ap.error("Destino o ZIP ya existe. Elegí rutas nuevas para conservar las entregas anteriores.")
    if args.original and not args.original.is_file():
        ap.error("El ZIP original no existe.")
    if args.fuentes_docente and not args.fuentes_docente.is_dir():
        ap.error("El directorio de fuentes del docente no existe.")
    raiz = args.destino / "ROBOTICA_01" / "proyecto_brazo"
    codigo = raiz / "00_codigo" / "hito3_4"
    codigo.mkdir(parents=True)
    commit = git("rev-parse", "HEAD").decode().strip()
    rama = git("branch", "--show-current").decode().strip()
    copiados = 0
    omitidos: list[str] = []
    for nombre in git("ls-files", "-z", "--", CARPETA_CODIGO).decode().split("\0"):
        if not nombre:
            continue
        relativo = Path(nombre).relative_to(CARPETA_CODIGO)
        partes = relativo.parts
        if (partes[0] == "hito3_4" or any(p in {".venv", ".venv_interfaz", "__pycache__", ".pytest_cache", "sesiones", "node_modules"} for p in partes)
                or relativo.suffix == ".pyc" or relativo.name == "GO_VEVAS.json"):
            omitidos.append(nombre)
            continue
        origen = REPO / nombre
        if origen.is_symlink():
            raise ValueError(f"No se exportan enlaces simbólicos: {nombre}")
        destino = codigo / relativo
        destino.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(origen, destino)
        copiados += 1
    shutil.copy2(REPO / "LICENSE", raiz / "00_codigo" / "LICENSE")
    # Documentación del hito (en el repo vive junto al código, en 04_hito3_4_control_real/01_docs):
    # se copia a 00_codigo/docs y sus links a ../../02_vevas_studio/ pasan a ../../hito3_4/.
    for nombre in git("ls-files", "-z", "--", CARPETA_DOCS).decode().split("\0"):
        if not nombre:
            continue
        relativo = Path(nombre).relative_to(CARPETA_DOCS)
        destino = raiz / "00_codigo" / "docs" / relativo
        destino.parent.mkdir(parents=True, exist_ok=True)
        if destino.suffix == ".md":
            texto = (REPO / nombre).read_text(encoding="utf-8")
            destino.write_text(texto.replace("02_vevas_studio/", "hito3_4/"), encoding="utf-8")
        else:
            shutil.copy2(REPO / nombre, destino)
        copiados += 1

    escribir(raiz, "00_codigo/README.md", """
# Código único del proyecto

`hito3_4/` es la única copia editable de la aplicación. Conserva sus rutas internas
para que Python, las mallas STL, la mesa y las pruebas sigan encontrándose.
No hay que copiar módulos a los apartados numerados: sus guías apuntan aquí.
Los lanzadores nuevos están fuera del código y guardan datos en `08_datos_locales`.
Los lanzadores antiguos se conservan por trazabilidad; usá los nuevos.
""")
    areas = {
        "01_interfaz": ("Interfaz gráfica", """
Abrí `bash 01_interfaz/abrir_simulacion.sh` desde la raíz del proyecto.
La web muestra el modelo 3D, la mesa, juntas, pose, cálculo Pieper, ramas, Jacobiano,
trayectoria, gráficas, errores y traza del TCP. Las flechas y el arrastre editan un
objetivo; calculá, revisá y confirmá antes de ejecutarlo en SIM.
Las figuras círculo, corazón, cuadrado y saludo están habilitadas en SIM.
Código: [web](../00_codigo/hito3_4/08_web/README.md),
[servidor](../00_codigo/hito3_4/07_vevas/servidor.py),
[consola](../00_codigo/hito3_4/07_vevas/consola.py).
Las animaciones interpoladas no sustituyen los datos recibidos ni certifican movimiento físico.
"""),
        "02_conexion": ("Conexión al controlador", """
1. Conectá Ethernet al SERVICE del IRC5 encendido. Para leer, Manual y detenido.
2. Revisá Calibración → ROB_1: si los seis ejes son válidos, no recalibres ni actualices contadores.
3. Ejecutá `bash 02_conexion/diagnosticar.sh` y revisá que FK y controlador coincidan.
4. Ejecutá `bash 02_conexion/abrir_solo_lectura.sh`.

El diagnóstico detecta interfaz y ruta; no impone eno1, eno2 ni una IP de PC.
IP del IRC5 por defecto: 192.168.125.1. Para otra IP confirmada, usá
`VEVAS_ROBOT_IP=... bash 02_conexion/diagnosticar.sh` y la misma variable al abrir.
Los JSON nuevos quedan en `08_datos_locales/conexion` sin sobrescribir capturas.
No se arranca RAPID ni se envía movimiento. No se necesita ROS para esta web.
[Procedimiento técnico](../00_codigo/docs/06_operacion/02_CONEXION_23NOV.md),
[comparar FlexPendant](../00_codigo/docs/06_operacion/01_COMPARAR_FLEXPENDANT.md),
[ROS/RViz opcional](../00_codigo/docs/06_operacion/05_RVIZ_RWS.md).
Las rutas de entrega antiguas de esas guías son históricas; aquí usá estos lanzadores.
"""),
        "03_calculos": ("Cinemática y trayectorias", """
Los módulos [robot_irb120.py](../00_codigo/hito3_4/07_vevas/robot_irb120.py),
[pieper.py](../00_codigo/hito3_4/07_vevas/pieper.py) y
[jacobiano.py](../00_codigo/hito3_4/07_vevas/jacobiano.py) calculan FK, IK y singularidades.
[trayectoria.py](../00_codigo/hito3_4/07_vevas/trayectoria.py) conserva LSPB y añade
perfiles quinticos C²; [figuras.py](../00_codigo/hito3_4/07_vevas/figuras.py) genera figuras.
El plan se calcula antes de enviar y se revisa el camino, no solo el destino.
Hay verificaciones de límites, FK de la IK, singularidad y colisiones aproximadas.
Los resultados de validación offline y sus casos deliberadamente incorrectos están
en `00_codigo/hito3_4/01_docs/01_datos/01_validacion_offline`.
La suavidad verificada es de simulación; no es una certificación de jerk ni del IRC5.
"""),
        "04_modelo_y_mesa": ("Robot, mesa y entorno", """
Fuente única de medidas: [mesa_medidas.json](../00_codigo/hito3_4/03_modelo/mesa_medidas.json).
Mesa abierta de 80×80 cm exteriores; perfil de 5×5 cm. Cara superior del marco
a 19 cm desde el plano de los tornillos; cilindro de la base a 18 cm desde ese MISMO plano.
La diferencia es 1 cm: no sumar 18+19 y no usar 37 ni 39 cm.
Laterales 29,5 cm; longitudinales 44,5 cm al lado opuesto a conectores y 14 cm al lado de conectores.
Las cuatro barras superiores se modelan para colisión. Soportes inferiores ilustrativos:
medidas no confirmadas. Herramienta/bracket y obstáculos externos todavía por medir.
Modelo: `00_codigo/hito3_4/03_modelo/irb120_rws.urdf` y `08_web/03_modelo/*.stl`.
[Explicación geométrica](../00_codigo/docs/03_guias/11_MESA_ENTORNO.md).
"""),
        "05_seguridad_y_control": ("Revisión y control físico pendiente", """
El envío físico continúa bloqueado por C02: falta verificar el límite de velocidad
de cada junta en el controlador. No habilitar el booleano por configuración ni
confundir TCP, frecuencia de lectura o porcentaje visual con límites articulares.
[Trabajo físico pendiente](../00_codigo/docs/08_desarrollo/02_CONTROL_FISICO_PENDIENTE.md)
contiene la propuesta SpeedLimAxis/LimitSpeed y las verificaciones presenciales.
Las plantillas GO son plantillas vacías, no autorizaciones completadas.
Recalibrar en la interfaz compara lecturas; no calibra encoders ni actualiza contadores.
PARAR desde el software solicita detener/cancelar; no reemplaza el paro físico.
RAPID conservado: `00_codigo/hito3_4/05_rapid/VEVAS_RWS.mod`; no se instala al extraer.
Este paquete no inicia movimiento, AUTO, motores ni RAPID por sí solo.
"""),
        "06_pruebas": ("Pruebas y evidencias", """
Después de preparar Ubuntu, ejecutá `bash 06_pruebas/ejecutar_pruebas.sh`.
Usa pytest sobre la copia de código, guarda el log en `08_datos_locales/pruebas`
y ejecuta 6 pruebas JavaScript si está instalado Node. Si Node falta, lo indica.
La suite de origen del 23/11 terminó con 232 pruebas Python y 6 JavaScript correctas.
La verificación de esta reorganización se documenta en `VERIFICACION_ORGANIZACION.md`.
[Evidencias de origen](../00_codigo/hito3_4/01_docs/01_datos/pruebas_23nov/README.md).
Las pruebas con controladores HTTP simulados no son pruebas físicas del robot.
"""),
        "07_documentacion": ("Guías y continuación", """
Empezá por [CONTEXTO_PARA_PROYECTO_BRAZO.md](CONTEXTO_PARA_PROYECTO_BRAZO.md),
[ESTADO_Y_PENDIENTES.md](ESTADO_Y_PENDIENTES.md) y
[08_PROXIMOS_PROMPTS.md](08_PROXIMOS_PROMPTS.md).
El código conserva bitácora, CAMBIOS_19/20/22/23NOV y guías previas.
Son documentos fechados: las notas nuevas de esta carpeta y los bloqueos de seguridad
prevalecen sobre instrucciones antiguas de carpetas y arranque físico.
[Extras posibles](../00_codigo/docs/08_desarrollo/04_MEJORAS_EXTRAS.md).
Los requisitos públicos del docente, si se aportaron al exportar, quedan en
`09_historial/fuentes_docente` con su procedencia original.
"""),
        "08_datos_locales": ("Datos nuevos de esta instalación", """
Aquí se guardan las sesiones, diagnósticos y logs de pruebas de esta copia.
`sesiones/`: muestras y eventos de la interfaz.
`conexion/`: diagnósticos RWS nuevos y fechados.
`pruebas/`: salida de pruebas nuevas.
Al entregar, estas carpetas están vacías: no hay GO completados ni pruebas físicas nuevas.
No subir sesiones, IP de red institucional ni capturas con información personal sin revisar.
"""),
        "09_historial": ("Originales e historial", """
Este apartado es de consulta y recuperación, no de ejecución.
`hito3_4_original_recibido.zip`, cuando está incluido, es el archivo recibido sin modificar.
`fuentes_docente/`, cuando está incluido, preserva los PDF/TEX y su procedencia.
`repositorio.bundle`, cuando está incluido, conserva el historial versionado alcanzable de HEAD.
El bundle puede conservar carpetas duplicadas y venv antiguos tal como fueron versionados;
no están en el código operativo limpio. No contiene el keyring ni configuración Git local.
Para recuperar Git sin red en una carpeta NUEVA fuera de este proyecto:
`git clone 09_historial/repositorio.bundle ../repositorio_recuperado`
La rama recuperada puede estar separada; consultá ORIGEN.json para el commit exacto.
No copies archivos viejos encima del código activo sin comparar primero.
Los ZIP de entregas intermedias se omiten por redundantes; sus cambios están en Git y bitácoras.
"""),
    }
    for area, (titulo, contenido) in areas.items():
        escribir(raiz, f"{area}/README.md", f"# {titulo}\n\n{contenido.strip()}")
    for carpeta, explicacion in {
        "08_web": "Interfaz HTML/CSS/JavaScript, bibliotecas locales y mallas del robot. La sirve 07_vevas/servidor.py.",
        "07_vevas": "Núcleo Python: cinemática, planes, seguridad, backends SIM/ROS/RWS, registro y servidor web.",
        "02_herramientas": "Diagnósticos, comparación del FlexPendant, ROS/RViz y utilidades. Preferir los lanzadores de 02_conexion; las utilidades de control físico están bloqueadas por C02.",
        "03_modelo": "URDF del IRB120 y medidas de la mesa, consumidos por la configuración y el modelo 3D.",
        "05_rapid": "Módulos RAPID conservados para revisión. Extraer o abrir esta carpeta no los carga ni ejecuta en el IRC5.",
        "06_tests": "Pruebas automatizadas offline con mocks; usar 06_pruebas/ejecutar_pruebas.sh. No certifican el montaje físico.",
        "01_docs": "Bitácoras, guías fechadas y evidencia. Consultar primero 07_documentacion y las notas del 23/11.",
        "04_plantillas": "Plantillas para documentar una sesión; no son GO completados ni autorización permanente.",
    }.items():
        escribir(codigo, f"{carpeta}/README.md", f"# {carpeta}\n\n{explicacion}")

    for sub in ("sesiones", "conexion", "pruebas"):
        (raiz / "08_datos_locales" / sub).mkdir(parents=True, exist_ok=True)
        escribir(raiz, f"08_datos_locales/{sub}/.gitkeep", "")

    escribir(raiz, "comun.sh", r'''
#!/usr/bin/env bash
# Se carga con source desde los lanzadores; no contacta al robot.
set -euo pipefail
BRAZO_RAIZ="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BRAZO_CODIGO="$BRAZO_RAIZ/00_codigo/hito3_4"
BRAZO_PYTHON="$BRAZO_RAIZ/.venv/bin/python"
export PYTHONDONTWRITEBYTECODE=1
if [[ ! -x "$BRAZO_PYTHON" ]]; then
  echo "Primero ejecutá: bash $BRAZO_RAIZ/preparar_ubuntu.sh" >&2
  exit 1
fi
mkdir -p "$BRAZO_RAIZ/08_datos_locales/"{sesiones,conexion,pruebas}
''')
    escribir(raiz, "preparar_ubuntu.sh", r'''
#!/usr/bin/env bash
set -euo pipefail
BRAZO_RAIZ="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
command -v python3 >/dev/null || { echo "Instalá Python 3 de Ubuntu." >&2; exit 1; }
if [[ ! -x "$BRAZO_RAIZ/.venv/bin/python" ]]; then
  python3 -m venv "$BRAZO_RAIZ/.venv" || {
    echo "Si falta venv: sudo apt install python3-venv" >&2
    exit 1
  }
fi
"$BRAZO_RAIZ/.venv/bin/python" -m pip install -r "$BRAZO_RAIZ/00_codigo/hito3_4/requirements.txt" -r "$BRAZO_RAIZ/06_pruebas/requirements_pruebas.txt"
echo "Preparado en $BRAZO_RAIZ"
echo "Simular: bash $BRAZO_RAIZ/01_interfaz/abrir_simulacion.sh"
''')
    comun = 'source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)/comun.sh"\n'
    escribir(raiz, "01_interfaz/abrir_simulacion.sh", '#!/usr/bin/env bash\n' + comun + r'''
cd "$BRAZO_CODIGO"
exec "$BRAZO_PYTHON" consola_vevas.py --modo sim --puerto "${VEVAS_PUERTO:-9012}" --sesiones "$BRAZO_RAIZ/08_datos_locales/sesiones" --abrir
''')
    escribir(raiz, "02_conexion/abrir_solo_lectura.sh", '#!/usr/bin/env bash\n' + comun + r'''
cd "$BRAZO_CODIGO"
exec "$BRAZO_PYTHON" consola_vevas.py --modo rws --solo-lectura --ip "${VEVAS_ROBOT_IP:-192.168.125.1}" --puerto "${VEVAS_PUERTO:-9012}" --sesiones "$BRAZO_RAIZ/08_datos_locales/sesiones" --abrir
''')
    escribir(raiz, "02_conexion/diagnosticar.sh", '#!/usr/bin/env bash\n' + comun + r'''
BRAZO_JSON="$("$BRAZO_PYTHON" -c 'import datetime,uuid; print(datetime.datetime.now().strftime("%Y%m%d_%H%M%S_")+uuid.uuid4().hex[:8]+"_diagnostico.json")')"
cd "$BRAZO_CODIGO"
exec "$BRAZO_PYTHON" 02_herramientas/conectar_robot.py --ip "${VEVAS_ROBOT_IP:-192.168.125.1}" --ping --salida "$BRAZO_RAIZ/08_datos_locales/conexion/$BRAZO_JSON"
''')
    escribir(raiz, "06_pruebas/requirements_pruebas.txt", "pytest>=8,<10")
    escribir(raiz, "06_pruebas/ejecutar_pruebas.sh", '#!/usr/bin/env bash\n' + comun + r'''
cd "$BRAZO_CODIGO"
BRAZO_LOG="$(mktemp "$BRAZO_RAIZ/08_datos_locales/pruebas/pytest_$(date +%Y%m%d_%H%M%S)_XXXXXX.log")"
"$BRAZO_PYTHON" -m pytest -q -p no:cacheprovider 06_tests/ 2>&1 | tee "$BRAZO_LOG"
if command -v node >/dev/null; then
  node 06_tests/test_orientacion.mjs
else
  echo "Node no está instalado: no se ejecutaron las 6 pruebas JavaScript."
fi
''')
    escribir(raiz, "README.md", """
# Proyecto brazo · VEVAS IRB 120

Copia independiente del proyecto de Florencia, organizada el 23/11/2026.
Destino previsto: `/home/huesero/Descargas/ROBOTICA_01/proyecto_brazo`.
Todos los lanzadores resuelven rutas desde su ubicación; también funcionan en otra carpeta.
El ZIP no modifica Ubuntu ni el robot al descargarlo o extraerlo.

## Abrir en Ubuntu

Guardá ROBOTICA_01_BRAZO.zip en Descargas. En una terminal:

```bash
(
set -e
test ! -e "$HOME/Descargas/ROBOTICA_01" || {
  echo "ROBOTICA_01 ya existe: conservá esa carpeta y extraé el ZIP en otro directorio nuevo."
  exit 1
}
unzip -n "$HOME/Descargas/ROBOTICA_01_BRAZO.zip" -d "$HOME/Descargas"
cd "$HOME/Descargas/ROBOTICA_01/proyecto_brazo"
bash preparar_ubuntu.sh
bash 01_interfaz/abrir_simulacion.sh
)
```

Abrí http://127.0.0.1:9012 en la MISMA Ubuntu y dejá la terminal abierta.
Ctrl+C cierra el servidor. Si el puerto está ocupado, cerrá tu VEVAS anterior o
ejecutá `VEVAS_PUERTO=9013 bash 01_interfaz/abrir_simulacion.sh` y abrí ese puerto.
No hay que conectarse al robot para ensayar en SIM.

## Índice por partes

- [00 Código único](00_codigo/README.md).
- [01 Interfaz y simulación](01_interfaz/README.md).
- [02 Conexión al IRC5 en lectura](02_conexion/README.md).
- [03 Cálculos](03_calculos/README.md).
- [04 Modelo del brazo y mesa](04_modelo_y_mesa/README.md).
- [05 Seguridad y control pendiente](05_seguridad_y_control/README.md).
- [06 Pruebas](06_pruebas/README.md).
- [07 Documentación y contexto para ChatGPT](07_documentacion/README.md).
- [08 Datos nuevos de esta instalación](08_datos_locales/README.md).
- [09 Historial y materiales originales](09_historial/README.md).

## Estado real

Software SIM verificado. Conexión RWS preparada para solo lectura.
Envío físico bloqueado por C02: límite de velocidad por junta pendiente en IRC5.
No hay movimiento ni calibración física nuevos confirmados desde esta sesión.
La publicación anterior en GitHub falló con 403; el bundle local conserva el historial.
Esta entrega no vincula automáticamente el chat ni escribe en la Ubuntu del usuario.
Para ChatGPT, adjuntá el contexto de 07_documentacion al proyecto «brazo».
""")
    escribir(raiz, "07_documentacion/ESTADO_Y_PENDIENTES.md", """
# Hecho, pendiente y siguiente paso

## Hecho en software

Interfaz clara con modelo STL, mesa abierta, FK/IK Pieper, ocho ramas, Jacobiano,
gráficas y traza del TCP; revisión de trayectoria, confirmación de ejecución y
perfiles quinticos C² en SIM. Figuras círculo/corazón/cuadrado/saludo en SIM.
Diagnóstico RWS de lectura y comparación estable joints-pose-joints con el IRC5.
Se conservan LSPB académico y la vía ROS/RViz opcional. Pruebas de origen: 232 Python, 6 JS.

## Hecho antes en laboratorio, según el chat

El grupo reportó calibración válida, lectura RWS y coincidencia FK con tool0 en una
postura: aproximadamente [381,43; 7,15; 613,04] mm, errores impresos redondeados 0.
Se publicaron juntas en ROS tras corregir serialización array→list.
Hubo captura EGM bidireccional y logs de controladores activados; eso NO prueba una
trayectoria física completa desde esta interfaz. El contexto de otra IA reportaba
movimiento RWS: falta separar esa afirmación de evidencia y del bloqueo C02 actual.

## Pendiente antes de envío físico

1. Obtener lectura de HOY y comparación con FlexPendant, con robot detenido.
2. Verificar C02 en controlador: SpeedLimAxis/LimitSpeed, estados, pérdida de señal,
   reinicio, compilación y trazas medidas; no basta un booleano ni velocidad TCP.
3. Medir y verificar herramienta/bracket, entorno y soportes sin cotas.
4. Probar movimiento pequeño presencial con operador, GO de sesión y revisión de trayectoria.
5. Validar retorno a cero/postura inicial y arrastre físico con la MISMA revisión previa.
6. Verificar suavidad física, informe, video y errores con datos medidos.

No ejecutar instrucciones antiguas que salten estas condiciones. No recalibrar ni
actualizar contadores solo por iniciar sesión o porque las juntas estén cerca de cero.
""")
    escribir(raiz, "07_documentacion/CONTEXTO_PARA_PROYECTO_BRAZO.md", """
# Contexto para continuar en el proyecto ChatGPT «brazo»

Este documento resume la información técnica útil disponible del chat, sin repetir
errores de escritura ni todas las conversaciones. No es una transcripción completa.
Fecha de corte 23/11/2026. Adjuntarlo al proyecto no mueve automáticamente el chat.
La raíz nueva en Ubuntu es /home/huesero/Descargas/ROBOTICA_01/proyecto_brazo.
Código único en 00_codigo/hito3_4; guías y lanzadores por área a su lado.

## Objetivo y preferencias

Florencia quiere una presentación completa de hitos 3 y 4 con ABB IRB120: ingresar
pose o una junta, calcular FK/IK Pieper, elegir rama, revisar el camino, mostrar
errores y cálculos en vivo, y solo después confirmar envío. Interfaz clara, limpia,
títulos destacados; modelo similar a RViz, flechas y arrastre, posiciones ejemplo,
gráficas necesarias y trazas de cada trayectoria. Figuras círculo/corazón/cuadrado
y saludo. Trayectorias suaves, selector de velocidad y frecuencia bien explicados.
Documentar hechos, pendientes, fallos y correcciones; generar extras y próximos prompts.
Usar subagentes por tarea; modelos económicos para inventarios repetitivos, revisión
profunda para control físico. Preguntar antes de asumir medidas ambiguas.

## Fuentes y propiedad

Repo principal: https://github.com/FlorenciaFrigerio/pfi-vevas-irb120, 04_hito3_4_control_real/02_vevas_studio.
Referencia de Marcelo: robotica-IMT342, rama proyectoHito3y4,
2-2026/IMT342/equipo/proyecto. Referencia, no destino de escritura.
La rama local es la que consta en ORIGEN.json. No afirmar push exitoso: los intentos
previos devolvieron 403. La autenticación gh de la laptop no se traslada a la nube.

## Robot y conexión

ABB IRB 120-3/0.6, IRC5 Compact, RobotWare 6.12.01 reportado.
SERVICE 192.168.125.1. Ubuntu con ROS2 Jazzy y ~/abb_ws, pero la web RWS no necesita ROS.
La interfaz Ethernet cambió entre eno1 y eno2; PC .10 y luego .207. Detectar ruta,
no imponer IP/interfaz. La IP antigua del destino EGM .10 no demuestra la IP actual.
Credenciales: consultar configuración local/responsable; no incluir tokens en el chat.
Manual/guardstop o motores apagados pueden permitir lectura. No poner AUTO ni arrancar
RAPID solo para diagnosticar. Comparar ROB_1, tool0, wobj0 y Base en ambos lados.
Aviso battery backup lost se relacionó con revisión de contadores; no equivale a
«falta alimentación externa». No actualizar contadores desde cualquier postura.

## Lecciones verificadas

Una lectura jointtarget dio 400 y ?ignore=1 devolvió datos durante el diagnóstico;
no usar ese parámetro para ocultar calibración pendiente. Luego hubo HTTP200 normal.
Timeout puede ser red/ruta; mirar ip route antes de tocar calibración.
ROS JointState.position es array: list(...) al serializar JSON.
EGMStreamStart de la rutina ros_egm_verificar transmite estados durante 20 s;
no mueve el brazo. Hubo paquetes a UDP6511 y driver Connected; no confundirlo con
movimiento físico probado. EGM se conserva como vía histórica; no es el arranque recomendado.
RWS exige módulo <5000 caracteres, nombres vv únicos para evitar LabJointRws y
peculiaridades de RW6. RAPID actual usa MoveAbsJ por llegada con fine: no reproduce
automáticamente los tiempos ni la suavidad quintica de SIM.
El error de seguimiento SIM se alinea al instante de la muestra; el umbral y watchdog
no se relajaron. La traza RWS procede de FK de juntas, no de un sensor externo TCP.

## Medidas definitivas

Todo en cm. Marco exterior80×80, perfil5×5; laterales29,5; longitudinales44,5 y14.
Desde el MISMO plano de tornillos: marco19, base18; diferencia1. No sumar18+19.
No usar las cotas37/39 ni las primeras24/9 sustituidas por fotos corregidas.
Datos activos: 03_modelo/mesa_medidas.json; cuatro barras superiores para colisión.
Herramienta/bracket, soportes inferiores y otros obstáculos no están completamente medidos.

## Seguridad y estado

Leer ESTADO_Y_PENDIENTES.md y docs/02_CONTROL_FISICO_PENDIENTE.md antes de control.
C02 sigue cerrado al envío: velocidad articular física no verificada. Las figuras
nuevas solo SIM. No activar backend.velocidad_articular_verificada por preferencia.
Propuesta: SpeedLimAxis para6ejes y sistema LimitSpeed con estado fresco verificado;
documento cita manualABB. Compilación RW6.12, EIO, pérdida/reinicio y trazas pendientes.
GO explícito de sesión; no completar casillas ni autorizar por tiempo transcurrido.
No afirmar que un botón web PARAR confirmó parada física. No cambiar calibración
con el botón de comparación de la interfaz.

## Cómo retomar

1. Leer README de raíz y 07_documentacion.
2. Probar SIM con 01_interfaz/abrir_simulacion.sh.
3. Diagnosticar conexión con 02_conexion/diagnosticar.sh en Ubuntu del laboratorio.
4. Abrir solo lectura y contrastar FlexPendant.
5. Resolver C02 con responsable presente antes de habilitar movimiento.
El entorno cloud /workspace no es /home/huesero: no decir que se guardó en la PC
sin comprobarlo allí. No confundir Space/Page con proyecto de ChatGPT.
""")
    escribir(raiz, "07_documentacion/08_PROXIMOS_PROMPTS.md", """
# Prompts para continuar

## Conectar y comparar, sin mover

«Continuamos VEVAS desde /home/huesero/Descargas/ROBOTICA_01/proyecto_brazo.
Leé CONTEXTO_PARA_PROYECTO_BRAZO.md, ESTADO_Y_PENDIENTES y ORIGEN.json. Te paso la
salida de bash 02_conexion/diagnosticar.sh. Revisá ruta, calibración informada y
comparación FK/tool0/Base; dame el siguiente paso de solo lectura sin arrancar RAPID.»

## Revisar el pendiente físico

«Leé docs/02_CONTROL_FISICO_PENDIENTE.md del código único. Prepará una propuesta concreta
para verificar C02 en RW6.12, con SpeedLimAxis y LimitSpeed. Separá lo verificado por
manual de lo pendiente de compilación y medición. No habilites movimiento ni cambies
flags hasta tener evidencia del controlador. Documentá cada resultado.»

## Mejorar interfaz

«Usá subagentes por tareas independientes. Trabajá sobre 00_codigo/hito3_4; conservá
revisión y confirmación antes de ejecutar. Mejorá [detalle]. Validá en SIM, documentá
cambios y pendientes y no presentes resultados simulados como movimiento real.»

## Informe y video

«Con la evidencia disponible, armá guion de demo e informe de hitos3y4. Incluí FK,
Pieper/8ramas, Jacobiano, singularidades, trayectorias, errores y límites. Separá
datos reales, simulados y pruebas aún no realizadas. No inventes resultados.»

## Publicar

«Revisá cambios, estado de Git y evidencias antes de subir. Usá el repositorio de
Florencia, no el de Marcelo. Conservá sesiones/GO/credenciales fuera del commit.
La publicación anterior falló403; verificá acceso actual sin pedir tokens por chat.»
""")
    escribir(raiz, ".gitignore", ".venv/\n08_datos_locales/**\n!08_datos_locales/README.md\n__pycache__/\n.pytest_cache/\nGO_VEVAS.json\n")
    if args.original:
        shutil.copy2(args.original, raiz / "09_historial" / "hito3_4_original_recibido.zip")
    if args.fuentes_docente:
        shutil.copytree(args.fuentes_docente, raiz / "09_historial" / "fuentes_docente", symlinks=False)
    if args.historial:
        git("bundle", "create", str(raiz / "09_historial" / "repositorio.bundle"), "HEAD")
    escribir(raiz, "ORIGEN.json", json.dumps({
        "fecha_utc": dt.datetime.now(dt.timezone.utc).isoformat(), "commit": commit, "rama": rama,
        "repo": "https://github.com/FlorenciaFrigerio/pfi-vevas-irb120",
        "ruta_objetivo_ubuntu": "/home/huesero/Descargas/ROBOTICA_01/proyecto_brazo",
        "codigo_fuente": "00_codigo/hito3_4", "archivos_versionados_copiados": copiados,
        "exclusiones": "venv/cachés/sesiones/GO completado/copia anidada (solo en código activo)",
        "omitidos_codigo_activo": omitidos,
        "historial_bundle": args.historial, "original_incluido": bool(args.original),
        "fuentes_docente_incluidas": bool(args.fuentes_docente),
        "escritura_pc_usuario": False, "chat_movido_a_proyecto": False,
        "movimiento_fisico_habilitado": False, "bloqueo": "C02",
    }, ensure_ascii=False, indent=2))
    escribir(raiz, "MANIFIESTO_SHA256.txt", "\n".join(
        hashlib.sha256(p.read_bytes()).hexdigest() + "  " + p.relative_to(raiz).as_posix()
        for p in sorted(raiz.rglob("*")) if p.is_file()))
    args.zip.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(args.zip, "w", compression=zipfile.ZIP_DEFLATED) as z:
        for p in sorted(args.destino.rglob("*")):
            if p.is_file():
                z.write(p, p.relative_to(args.destino))
    with zipfile.ZipFile(args.zip) as z:
        assert z.testzip() is None
    print(json.dumps({"carpeta": str(raiz), "zip": str(args.zip), "bytes": args.zip.stat().st_size,
                      "sha256": hashlib.sha256(args.zip.read_bytes()).hexdigest(), "commit": commit}, indent=2))


if __name__ == "__main__":
    main()
