# Guía completa del Hito 3/4: VEVAS Studio y el ABB IRB 120 real

Grupo VEVAS: Susann A. Baldiviezo Castellanos, Alejandro J. Bejarano Romero, Florencia Frigerio Arízaga, Benjamín Soruco Guzmán.
Ingeniería Mecatrónica, Depto. de Ciencias de la Tecnología e Innovación, UCB "San Pablo", Sede Tarija.
IMT-342 Robótica, docente M.Sc. Ing. Bernardo Quiroga Turdera. Repositorio: github.com/FlorenciaFrigerio/pfi-vevas-irb120.
Guía final, estado al 24/11/2026 (noche).

Esta guía junta en un solo lugar qué hace el sistema, cómo se usa, cómo está construido, por qué se decidió cada cosa, los cálculos, los resultados, los incidentes y lo que falta. Las mediciones salen de la bitácora del 24/11 ([`docs/bitacora/bitacora.md`](../05_bitacora/bitacora.md)), de [`CAMBIOS_24NOV.md`](../05_bitacora/CAMBIOS_24NOV.md) y del [informe IEEE del hito](../01_informe/). Los demás números se calcularon con los módulos de `vevas_studio/vevas/` (se indica cuando es así). La guía no contiene claves, GO ni datos de sesiones.

## Índice rápido

| Si necesitás... | Andá a |
|---|---|
| Entender el proyecto en un minuto | Resumen en una página |
| La secuencia de comandos del día de la defensa | 2.15 |
| Instalar o actualizar el código en la PC del laboratorio | 2.2 |
| Verificar C02 (obligatorio tras cambiar el módulo RAPID) | 2.5 |
| Crear el GO (dos preguntas) | 2.7 |
| Armar, revisar y enviar un movimiento | 2.8 |
| Figuras (de frente, saludo, aproximación J5) | 2.12 |
| Guardar poses e ir a ellas | 2.13 |
| Slider de velocidad y pestaña Velocidades | 2.14 |
| El FlexPendant no muestra «Iniciar», RWS colgado, contadores, MOTORS ON, cable | 2.16 |
| Tabla de errores típicos | 2.17 |
| Protocolo PC ↔ RAPID (con zonas y código 3) | 6.5 |
| Fórmulas y ejemplos numéricos | 7 |
| Resultados medidos | 8 |
| Preguntas del docente con respuesta | 11 |
| Dónde está cada archivo | 12 |

## Resumen en una página

**Qué es VEVAS.** VEVAS Studio es una consola que corre en nuestra PC con Ubuntu 24.04, muestra un gemelo 3D del ABB IRB 120 en el navegador, calcula los movimientos con nuestros propios modelos de los Hitos 1 y 2 (FK por D-H, Pieper con 8 ramas, Jacobiano y σmin, trayectorias LSPB y quíntica) y, si todas las compuertas de seguridad están en verde, manda objetivos articulares al robot real por Robot Web Services (RWS). En el IRC5 corre un módulo RAPID nuestro (`VEVAS_RWS`) que vuelve a validar cada objetivo y fija él mismo la duración de cada movimiento, de modo que ninguna junta puede pasar del 5 % de su velocidad de ficha.

**Qué demuestra.**

1. Que los modelos propios son correctos en el robot real: la FK propia coincide con la del controlador.
2. Que se puede mover un robot industrial compartido desde una PC con una cadena de compuertas verificable (GO, C02, revisión previa, armado, confirmación, validación en RAPID) sin tocar la configuración del controlador ni los programas de otros grupos.
3. Que el requisito C02 («ninguna junta supera su tope») está cerrado por diseño (ley de tiempo dentro del RAPID) y por medición en el robot.
4. Que el sistema mueve en continuo (streaming con zonas `z1`), dibuja figuras de frente, guarda poses y mide la velocidad real de cada junta.

**Cifras clave.**

| Dato | Valor | Fuente |
|---|---|---|
| Pieper, error de la verificación FK de cada rama | ~1e-15 m | Informe IEEE, pruebas |
| FK propia contra FK del controlador (`tool0`, `wobj0`, base) | 0,0001 mm en la primera lectura; 0,0000 mm tras actualizar contadores | Bitácora 24/11 |
| C02 medido en el robot (AUTO, 24/11) | pico de 42–43 % de vLim en las seis juntas | `verificar_c02.py` |
| Factor pico/medio medido en el IRC5 | ≈ 1,06 | `verificar_c02.py` |
| Tope del RAPID (vLim) | 12,5 / 12,5 / 12,5 / 16 / 16 / 21 °/s (5 % de la ficha) | `rapid/VEVAS_RWS.mod` |
| Pruebas automáticas | 346 Python + 20 JavaScript, todas pasan | pytest y `node --test`, 24/11 |
| Módulo RAPID | 4707 caracteres (< 4800, RobotWare 6 trunca en 5000) | `rapid/VEVAS_RWS.mod` |

**Qué cambió la tarde del 24/11.** Movimiento continuo con zonas (código 3 al encolar), C02 versión 2 con prueba de streaming, poses guardadas, vista de puntos de las figuras, figuras de frente por defecto, saludo en arco ∩, aproximación automática J5 → +30°, GO en dos preguntas, slider hasta 50 % con tope físico C02, pestaña Velocidades, `verificar_c02` a ~30 Hz y script `subir_24nov.sh`.

**Lo que hay que decir con honestidad.** «Aprobado» significa «cumple dentro del modelo». PARAR por software no frena el tramo en curso: el freno real es el paro de emergencia físico. Como el módulo cambió (zonas), hay que correr `verificar_c02.sh` otra vez (evidencia versión 2) antes de mover el robot real.

## Índice

1. Qué hace el sistema, de punta a punta
2. Cómo se usa paso a paso
3. Cómo lo hicimos: arquitectura y archivos
4. Glosario de conceptos
5. Por qué cada decisión
6. Lenguajes, herramientas y protocolos
7. Cálculos con fórmulas y ejemplos numéricos
8. Resultados y evidencias
9. Incidentes y cómo se resolvieron
10. Limitaciones y próximas mejoras
11. Banco de 28 preguntas del docente
12. Mapa de archivos del repo

## 1. Qué hace el sistema, de punta a punta

### 1.1 La idea en una frase

VEVAS Studio calcula cada movimiento con nuestros modelos, lo revisa completo antes de mandarlo y se lo pasa al IRC5 como una serie de objetivos articulares; el RAPID del controlador decide cuánto tarda cada uno para que ninguna junta pase del 5 % de su velocidad de ficha.

### 1.2 Las piezas

1. **El robot**: ABB IRB 120 con controlador IRC5 Compact, RobotWare 6.12.01, IP 192.168.125.1 (puerto SERVICE).
2. **La PC**: Ubuntu 24.04 con ROS 2 Jazzy, interfaz `eno2` con 192.168.125.207/24, conectada por cable al puerto SERVICE.
3. **La consola Python** (`consola_vevas.py` + paquete `vevas/`): servidor HTTP local (puerto 9012 con robot real, 9013 en simulación cuando se usan los dos a la vez) que concentra modelos, revisión, compuertas, poses guardadas y medición de velocidad.
4. **La web** (`web/`): página estática con Three.js. Tiene el gemelo 3D, flechas y anillos para arrastrar el TCP, botón «De frente» en la cámara, puntos de la figura en 3D y las pestañas Control, Gráficas, Datos en vivo, **Velocidades**, Cálculos, 8 ramas, Trayectoria, FK en vivo, Jacobiano, Demo e Historial.
5. **El backend de movimiento**: tres intercambiables. SIM (simulador interno), ROS 2 (opcional, vía `EGM_ROS` y RViz) y RWS (el que usamos con el robot real).
6. **El módulo RAPID** `VEVAS_RWS.mod`, con la rutina `VevasServidor`, que corre en el IRC5 y obedece solo a variables `PERS` que escribe la PC.
7. **El paro de emergencia físico**, que es la parada real y no depende de ningún software.

### 1.3 El recorrido de un movimiento, contado simple

1. La consola lee las seis juntas del robot por RWS diez veces por segundo y las pinta en el gemelo 3D. Con esas juntas calcula la FK propia, σmin y la **velocidad medida** de cada junta.
2. El usuario pide un objetivo: arrastra el TCP, escribe una pose, mueve «Una junta», elige «Volver a cero», «Preparar J5 a +30°», «Ir» a una pose guardada o una figura (círculo, corazón, cuadrado, triángulo, saludo).
3. **Pieper** resuelve la cinemática inversa y devuelve las 8 ramas. Cada rama se reinyecta en la FK (**verificación FK**): solo sirve si el error de posición es menor a 1e-6 m y el de orientación menor a 1e-5 rad.
4. Se elige la **rama** más cercana a la postura actual que respete límites y no sea singular.
5. Se arma la **trayectoria** articular con un perfil suave (quíntico o LSPB). Las figuras tienen paradas (v = 0) en esquinas, cúspide o extremos del saludo.
6. La **revisión previa** (`vevas/revision.py`) muestrea el plan completo (hasta 240 muestras) y revisa límites, velocidad del plan, tope físico C02, singularidad (σmin), altura del TCP, autocolisión, mesa/estructura y herramienta. Resultado: aprobado, aviso, rechazado o no_verificado.
7. Si el plan no está rechazado ni sin verificar, el usuario **arma** el sistema (checklist + frase «MOVER ROBOT REAL»), confirma y pulsa **Enviar**. Al enviar, la revisión se repite con la postura medida en ese instante.
8. **Movimiento continuo**: la trayectoria se convierte en puntos (tramos rectos de hasta 10° y curvas con un punto cada ~0,5–2°). Los intermedios van con zona `z1`; el último punto y las paradas del plan van con `fine`. Si el módulo cargado no soportara zonas, se usa el envío viejo por tramos colineales de hasta 40° con `fine`.
9. Por cada punto la PC escribe `vvPcZona` (1 zona, 0 fine) y `vvPcTarget`, lo relee, comprueba que el robot no se movió de lo previsto y recién entonces incrementa `vvPcSeq`.
10. El RAPID ve la secuencia nueva, valida (modo, límites con margen 2°, paso máximo 45° contra el objetivo anterior), calcula el tiempo `t` con la ley C02 y ejecuta `MoveAbsJ objetivo, vPc \T:=t, z1 | fine, tool0`.
11. El RAPID contesta con `vvRbCodigo`: **3** apenas encola un punto con zona (la PC manda el siguiente mientras el robot se mueve), 1 moviendo y 2 llegó en los puntos con `fine`, o un código negativo de rechazo. La PC da el recorrido por cumplido solo si el último punto contestó 2 y la postura **medida** está a menos de 0,05° del objetivo.
12. Todo queda en el **Historial** y en los archivos de sesión (`muestras.csv` incluye la velocidad medida `v1..v6_med_deg_s`).

Los ocho pasos que ve el usuario en la interfaz son: Objetivo, Pieper, Verificación FK, Rama, Trayectoria, Revisión, Envío, Resultado.

### 1.4 Qué no hace (a propósito)

1. No mueve el robot si falta cualquiera de estas cosas: GO vigente, evidencia C02 válida (versión 2) para el módulo cargado y el modo actual, revisión no rechazada, sistema armado.
2. No interpola en cartesiano dentro del robot: siempre manda **objetivos articulares** (`MoveAbsJ`), así que el controlador nunca invierte el Jacobiano.
3. No toca los módulos de otros grupos (`LabJointRws`, `EGM_ROS`, `Imt342TrajectoryRapid`) ni arranca encima de un RAPID ajeno.
4. No cambia el modo AUTO/MANUAL, no actualiza contadores de revoluciones ni enciende motores tras un reinicio: eso lo hace una persona.
5. No reemplaza el paro de emergencia. PARAR (o Esc) deja de mandar puntos y detiene RAPID por RWS, pero lo ya encolado termina. El freno real es el paro físico.
6. No hace que el robot real vaya más rápido aunque el slider marque más del 5 %: el RAPID limita cada junta a vLim y solo estira el tiempo.

### 1.5 Qué se demostró el 24/11/2026

1. La FK propia coincide con la del controlador: 0,0001 mm en la primera lectura y 0,0000 mm tras actualizar contadores, con `tool0`, `wobj0` y base.
2. C02 verificado en el robot real en AUTO (versión 1, envío con `fine`): velocidades máximas de 5,2 a 8,9 °/s contra topes de 12,5 a 21 °/s (42–43 % de vLim); pico medido ≈ 1,06 veces el medio.
3. 346 pruebas automáticas en Python y 20 en JavaScript, todas pasan (incluyen un IRC5 simulado por HTTP con zonas y código 3).
4. La postura HOME (q = 0) es singular de muñeca (σmin ≈ 0) y el sistema sabe entrar y salir de ella de forma controlada, incluso agregando sola la salida J5 → +30° antes de una figura.

## 2. Cómo se usa paso a paso

Hay dos lugares desde donde se corren comandos:

1. La **raíz del proyecto** en la PC del laboratorio: `~/Descargas/ROBOTICA_01/proyecto_brazo`. Ahí están los lanzadores: `02_conexion/diagnosticar.sh`, `02_conexion/abrir_solo_lectura.sh`, `05_seguridad_y_control/verificar_c02.sh` y `05_seguridad_y_control/abrir_control_real.sh`. Usan el entorno `.venv` de la raíz (lo crea `preparar_ubuntu.sh`) y guardan datos en `08_datos_locales/`.
2. La **carpeta del código**: `~/Descargas/ROBOTICA_01/proyecto_brazo/00_codigo/hito3_4` (en el repositorio es `hito3_4_control_real/vevas_studio/`). Ahí están `consola_vevas.py`, `herramientas/`, `rapid/`, `web/` y `tests/`.

La clave de RWS no se escribe en ningún comando de esta guía. Si hace falta, se carga en la variable de entorno `VEVAS_RWS_CLAVE` en la terminal de la sesión, sin guardarla en archivos del repositorio.

### 2.1 Preparar el entorno (una sola vez)

Entorno de la raíz (lo usan los lanzadores de `02_conexion` y `05_seguridad_y_control`):

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo
bash preparar_ubuntu.sh
```

Entorno de la carpeta del código (para pruebas y para correr la consola a mano):

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo/00_codigo/hito3_4
python3 -m venv .venv_interfaz
.venv_interfaz/bin/python -m pip install -r requirements.txt
```

Comprobar que todo pasa antes de ir al laboratorio (la parte Python tarda unos 5 minutos):

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo/00_codigo/hito3_4
.venv_interfaz/bin/python -m pytest -q tests
node --test tests/test_calculos.mjs tests/test_orientacion.mjs tests/test_velocidad.mjs
```

Esperado: 346 pruebas Python y 20 JS aprobadas.

### 2.2 Instalar o actualizar con `subir_24nov.sh`

El script viene junto a `vevas_24nov.bundle` (un paquete Git con la rama `24NOV`). Los dos tienen que estar en la misma carpeta.

```bash
cd <carpeta donde están subir_24nov.sh y vevas_24nov.bundle>
bash subir_24nov.sh
```

Qué hace:

1. Usa el clon del repositorio en `~/pfi-vevas-irb120` (si no existe, lo clona; eso sí necesita internet). Si el clon tiene cambios sin guardar, se detiene.
2. Prueba `git fetch`. **Sin internet sigue igual**: actualiza el código local y deja la subida para después.
3. Actualiza el código en `00_codigo/hito3_4` desde la rama `24NOV`. **No toca** `GO_VEVAS.json`, `evidencia_c02.json`, sesiones ni `.venv`. De `config.json` cambia solo los topes del slider (`velocidad_real_max_pct` y `velocidad_sim_max_pct` a 50).
4. Copia los scripts nuevos a `05_seguridad_y_control` (con `--datos 08_datos_locales` para las poses).
5. Copia la guía y el informe PDF a `07_documentacion/`.
6. Con internet: sube la rama `24NOV` y pregunta `¿Fusionar 24NOV en main y subir main? (s/n)`. Con `s` fusiona en `main` y la sube; con `n` deja `main` como estaba.
7. Al final recuerda: Ctrl+C en la consola si estaba abierta, `verificar_c02.sh` (cambió el módulo RAPID) y recién después `abrir_control_real.sh`.

Rutas distintas a las de siempre: `PROYECTO=... REPO=... bash subir_24nov.sh`.

### 2.3 Simulación (sin robot)

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo/00_codigo/hito3_4
.venv_interfaz/bin/python consola_vevas.py --modo sim --puerto 9013 --abrir
```

Se abre `http://127.0.0.1:9013`. La cabecera dice SIMULACIÓN. Ahí se ensaya todo (arrastre, figuras, poses guardadas, Volver a cero, Preparar J5) sin riesgo. En SIM las figuras admiten de 5 a 100 mm y el slider llega a 50 %. Sin `--datos`, las poses se guardan en `vevas_studio/datos/poses_guardadas.json` (carpeta ignorada por git).

### 2.4 Diagnóstico de conexión (solo lectura, no mueve nada)

Primero, la red. La PC debe tener 192.168.125.207/24 en `eno2`:

```bash
ip -br addr show eno2
ping -c 3 192.168.125.1
```

Si el cable se desconectó o `eno2` quedó sin IP:

```bash
nmcli connection up "Conexión cableada 1"
# si no existe esa conexión, a mano (se pierde al reiniciar la PC):
sudo ip addr add 192.168.125.207/24 dev eno2
```

Diagnóstico completo desde la raíz (red, RWS, juntas y FK propia contra la del controlador; guarda un JSON en `08_datos_locales/conexion`):

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo
bash 02_conexion/diagnosticar.sh
```

Debe decir **COINCIDE** (el 24/11 dio 0,0000 mm). Ojo: COINCIDE compara contra las mismas lecturas del controlador, así que **no** detecta contadores de revoluciones corridos (ver 2.16).

Herramientas sueltas desde la carpeta del código:

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo/00_codigo/hito3_4
python3 herramientas/rws_irc5.py estado          # modo, motores, RAPID, PP, módulo cargado, juntas
python3 herramientas/comparar_pendant.py         # FK propia vs pose del controlador
python3 herramientas/rws_irc5.py diagnostico -n 8   # últimos 8 eventos del controlador (elog)
```

Para solo mirar el robot, sin poder mandar nada:

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo
bash 02_conexion/abrir_solo_lectura.sh
```

o, desde la carpeta del código, `bash iniciar_real.sh --solo-lectura` (web + RViz) o `bash iniciar_real.sh --solo-lectura --sin-rviz` (solo web).

### 2.5 Verificar C02 (versión 2, con prueba de streaming)

Se repite cada vez que cambia **un solo carácter** de `VEVAS_RWS.mod`, cada vez que se cambia de modo (AUTO o manual reducido) y si se cambian vLim o kPico. **El módulo cambió la tarde del 24/11 (zonas), así que hay que correrlo de nuevo**: la evidencia vieja (versión 1) ya no habilita el envío.

Condiciones antes de correrlo:

1. Llave del IRC5 en **AUTO** (la gira una persona; ningún script cambia el modo).
2. Célula despejada, brida sin herramienta (o herramienta medida).
3. **Una persona con la mano en el paro de emergencia.**
4. Robot en una postura segura, lejos de límites y de la mesa (por ejemplo J1 a J3 cerca de 0 y J5 ≥ 30°).
5. Nadie más con RAPID corriendo y ningún otro programa nuestro conectado por RWS (cerrar la consola con Ctrl+C).

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo
bash 05_seguridad_y_control/verificar_c02.sh
```

Qué pasa:

1. Muestra modo, motores, RAPID y juntas, y pide `s` para mover.
2. Si hay un `VevasServidor` de **otra versión** corriendo, lo detiene (Stop + motores OFF) para cargar la nueva. Si es la misma versión pero quedó **esperando «Iniciar»** o se canceló, también lo detiene y lo arranca de nuevo.
3. Carga `VEVAS_RWS.mod` si falta o cambió, y arranca RAPID.
4. En el FlexPendant, en la **ventana del operador** (Production Window / Operator Window), aparece «Aceptar objetivos enviados por la PC?» con los botones **Iniciar** y **Cancelar** abajo. Pulsar **Iniciar** (hay 30 s).
5. Mueve J1 a J6 de a una, +5° y vuelta (o −5° si +5° se sale de límites). Lee las juntas a ~30 Hz con una conexión aparte. Por cada tramo imprime el tiempo esperado, el que usó el RAPID y la velocidad máxima medida contra el tope.
6. **Prueba de streaming**: J6 recorre +5°, −5° y vuelve en 8 puntos (los 7 primeros con zona, el último con `fine`). Pasa si los códigos son `[3, 3, 3, 3, 3, 3, 3, 2]`, ninguna junta supera vLim y la duración observada es al menos el 90 % de la suma de los tiempos de cada tramo (≈ 2,56 s con δ = 5°).
7. Guarda `00_codigo/hito3_4/evidencia_c02.json` (la anterior queda como `.anterior`). Si dice **C02 VERIFICADO**, se puede pasar al control real.

Variantes:

```bash
bash 05_seguridad_y_control/verificar_c02.sh --delta 3          # pasos de 3° (entre 0,5 y 10)
bash 05_seguridad_y_control/verificar_c02.sh --juntas 4,5,6     # solo algunas juntas (no habilita: hacen falta las seis)
```

### 2.6 Abrir el control real

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo
bash 05_seguridad_y_control/abrir_control_real.sh
```

El script:

1. Exige que exista `evidencia_c02.json`; si no, pide correr primero `verificar_c02.sh`.
2. Valida el GO de la sesión. Si no hay GO válido de hoy, lanza `crear_go.py` (dos preguntas, ver 2.7).
3. Arranca `consola_vevas.py --modo rws --real --go GO_VEVAS.json --preparar-rapid --puerto 9012 --sesiones 08_datos_locales/sesiones --datos 08_datos_locales --abrir`.

En `http://127.0.0.1:9012` la cabecera debe decir **IRC5 vía RWS · ROBOT REAL** y no debe aparecer el aviso de C02. Si `--preparar-rapid` tiene que arrancar `VevasServidor`, el FlexPendant vuelve a pedir **Iniciar**. Con un servidor de la versión verificada ya activo, no se reinstala nada.

### 2.7 Crear el GO de la sesión (dos preguntas)

Lo hace `abrir_control_real.sh` si falta. A mano:

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo/00_codigo/hito3_4
python3 herramientas/crear_go.py --salida GO_VEVAS.json
python3 herramientas/crear_go.py --salida GO_VEVAS.json --reemplazar   # si ya existe uno de otro día o NO-GO
```

Las dos preguntas:

1. «¿Zona despejada, brida sin herramienta, nada fuera de la mesa y paro de emergencia a mano? [s/n]». La respuesta marca todo el checklist; con `n` queda **NO-GO**.
2. «Tope de velocidad en %». **Enter = 5 %**; máximo 50 (o menos si `config.json` lo baja).

Los nombres de operador, responsable de consola y supervisor se completan solos como «Grupo VEVAS» y el alcance como ensayos del Hito 3/4. El GO lleva la fecha y **vale todo el día**. Nunca se genera automático (no hay `-y`), no cambia `config.json` y no se sube al repositorio. Responder solo lo verificado en persona.

### 2.8 Armar, calcular, revisar y enviar (en la web)

1. Mirar la cabecera: modo ROBOT REAL, juntas actualizándose, estado **DESARMADO**.
2. Pulsar **ARMAR**: marcar el checklist (supervisor, paro, área, velocidad) y escribir exactamente `MOVER ROBOT REAL`. El estado pasa a **ARMADO**.
3. Elegir el objetivo. Para el primer movimiento, algo chico: «Una junta» de 5° o un arrastre corto del TCP.
4. Calcular. Revisar en orden: Pieper (8 ramas), Verificación FK (errores ~1e-15 m), rama elegida, trayectoria.
5. Abrir la **Revisión**. Si sale **rechazado** o **no_verificado**, no se puede enviar; leer el motivo. Si sale **aviso**, leerlo (por ejemplo «Tope físico del RAPID (C02)» con el tiempo real estimado) y decidir.
6. Pulsar **Enviar**, leer el resumen en el diálogo «Confirmar envío al robot real» y confirmar.
7. Mirar el robot, no la pantalla, con la mano cerca del paro. El paso **Resultado** muestra la postura final medida y el error.

### 2.9 PARAR

1. Botón **PARAR** (■) en la cabecera, o teclas **Esc** o **Espacio**.
2. Deja de mandar puntos y detiene RAPID por RWS. En movimiento continuo el robot termina lo ya encolado: a lo sumo el tramo en curso y el siguiente (≤ 2 × 10° por junta). Con `fine`, termina el `MoveAbsJ` en curso.
3. Ante cualquier duda real: **paro de emergencia físico**. Después hay que rearmar en el controlador (soltar la seta, reconocer y MOTORS ON por el operador, ver 2.16).

### 2.10 Volver a cero

Botón **Volver a cero** (todas las juntas a 0°). No recalibra nada: es un plan articular revisado como cualquier otro. Como q = 0 es singular de muñeca (q5 = 0), la revisión lo da como **aviso** solo si el backend es RWS y C02 está verificado (movimiento articular con velocidad por junta acotada). Sin esas condiciones queda **rechazado**.

Desde la postura inicial [0, −20, 30, 0, 40, 0]° el recorrido es una recta en el espacio de juntas: con zonas sale en **4 puntos** (J5 en 30°, 20°, 10° y 0°; los tres primeros con `z1`, el último con `fine`), **sin paradas intermedias**, y tarda lo mismo que en un solo tramo: 6,25 s (domina J5 con 40°). Calculado con `muestrear_con_paradas` y `tiempo_movimiento`.

### 2.11 Preparar J5 a +30°

Botón **Preparar J5 a +30°**. Habilitado en SIM y con el robot real **solo por RWS con C02 verificado** (con ROS real o sin C02 se mueve desde el FlexPendant). Toma una lectura fresca de las juntas y cambia solo J5 a 30°. Sirve para salir de HOME: σmin sube monótonamente de 0 a 0,231. La revisión lo da como **aviso** «sale de una singularidad de muñeca» porque cumple el criterio de salida (zona singular solo al principio, solo muñeca, σ creciente desde el mínimo). Desde q = 0 tarda 4,69 s (3 puntos de 10°). Es la postura recomendada antes de arrastrar el TCP o hacer figuras.

### 2.12 Figuras

Se abren desde **Poses predeterminadas** (habilitado en SIM o con RWS + C02). El diálogo pide:

1. **Figura**: Círculo, Corazón, Cuadrado, Triángulo o Saludo.
2. **Tamaño** en mm: de 5 a 100 en SIM; **máximo 80 mm** con el robot real.
3. **Vista** (plano de dibujo): **De frente (plano YZ, vertical)**, que es el valor por defecto; **Desde arriba (plano XY, horizontal)**; **De costado (plano XZ, vertical)**.

Debajo se ve la **vista de puntos** antes de calcular (2.12.3). Al pulsar «Calcular trayectoria» con una figura de frente, la cámara 3D se ubica delante del robot.

| Figura | Qué hace | Paradas (índices con 65 puntos) |
|---|---|---|
| Círculo | Diámetro = tamaño; empieza y termina en el TCP actual | Ninguna: recorrido continuo |
| Corazón | Curva paramétrica clásica escalada al ancho | La cúspide (32) |
| Cuadrado | Lado = tamaño | Las 3 esquinas intermedias (16, 32, 48); la cuarta es el inicio/fin |
| Triángulo | Equilátero de lado = tamaño, un vértice en el TCP | Los 2 vértices intermedios (22, 43) |
| Saludo | Vaivén en arco ∩ de lado a lado, amplitud 40°, 2 vaivenes | Cada extremo (8, 24, 40, 56) |

Índices calculados con `vevas/figuras.py` (`paradas_figura`).

Cómo se calcula cada figura: se generan 65 puntos relativos al TCP en el plano elegido; Pieper resuelve cada punto con la orientación constante; se exige que el salto articular entre puntos sea como máximo 12°; se arma una trayectoria por tramos que se detiene en las paradas; y todo pasa por la misma revisión. En el robot real las paradas van con `fine` (el robot no redondea la esquina) y el resto en continuo con `z1`.

#### 2.12.1 De frente: cómo se ve

El robot mira hacia +X. «De frente» es el plano YZ visto por alguien parado delante del brazo: **Y a la derecha y Z hacia arriba**. Así un corazón o un saludo de frente se ven derechos, como los vería el público. El botón **De frente** de la cámara 3D pone la cámara sobre +X mirando a la base (posición (1,75; 0; 0,5) m, mirando a (0,3; 0; 0,45) m).

#### 2.12.2 El saludo

Antes era un movimiento de muñeca; ahora es un **vaivén en arco ∩** de la mano, como un antebrazo que pivota en el codo: 0 → +40° → −40° → +40° → −40° → 0, con los puntos repartidos parejos a lo largo del arco. El ancho total es el tamaño elegido, así que el radio es R = tamaño / (2·sen 40°). Con 25 mm: R = 19,45 mm y el arco baja 4,55 mm en los extremos. La mano se detiene un instante en cada extremo (índices 8, 24, 40 y 56). Se resuelve con Pieper punto a punto, como las demás figuras.

#### 2.12.3 Vista de puntos

1. **Antes de calcular** (en el diálogo): dibujo 2D en mm, con la misma escala en los dos ejes, de los 65 puntos de la figura. El color va de claro a oscuro según el orden, con flechas de sentido y numeración. Se marcan **inicio**, **fin** y **paradas**. No calcula plan ni mueve nada; la geometría sale del mismo `vevas/figuras.py`.
2. **Durante y después de la ejecución** (panel «Recorrido de la figura»): sobre los puntos planificados se superponen en violeta los puntos **medidos** del TCP (FK de las juntas recibidas). Abajo dice «Medido vs planificado (distancia 3D al recorrido): máx X mm · medio Y mm · N puntos». El botón **Limpiar** borra los medidos.
3. **En el 3D**: la casilla **Puntos** muestra los puntos de la figura sobre el gemelo.

#### 2.12.4 Aproximación automática J5 → +30°

Si al calcular una figura J5 está cerca de 0 (|sen q5| < 0,09, es decir |q5| < 5,16°), en vez de rechazar el plan agrega primero una **aproximación articular J5 → +30°** (la misma de «Preparar J5») con parada en el índice 1, y dibuja la figura desde esa postura. Es **un solo plan** revisado, enviado y confirmado una vez. En la revisión aparece una nota informativa en verde azulado: «J5 estaba en ≈ 0° (singularidad de muñeca): el plan primero lleva J5 a +30° con parada y después dibuja la figura desde ahí». La descripción del plan termina en «antes J5 → +30°». Vale para todas las figuras, saludo incluido.

Por qué hace falta: con q5 = 0 los ejes 4 y 6 quedan alineados y Pieper solo determina q4 + q6, no cada uno; además las ramas con σmin bajo el umbral se descartan. Desde el primer punto no hay una rama válida y continua (salto ≤ 12°), y antes el plan salía rechazado con «no hay rama de Pieper».

### 2.13 Poses guardadas

Panel **POSES GUARDADAS** en la pestaña Control.

| Acción | Cómo | Qué pasa |
|---|---|---|
| Guardar | Escribir un nombre y pulsar «Guardar pose actual» | Guarda las 6 juntas **medidas** y la pose del TCP por FK, con fecha. Exige robot quieto y lectura fresca |
| Listar | Se ve sola | Nombre, juntas, TCP y fecha; cuenta de poses |
| Renombrar | «Renombrar», escribir, Enter o «Aceptar» | Cambia el nombre (sin repetir otro existente) |
| Borrar | «Borrar» y luego «Sí, borrar» | Pide confirmación; queda en el Historial |
| Ir | «Ir» | Calcula un plan articular revisado hasta esa pose |

Reglas de nombre: hasta 40 caracteres; letras, números, espacios y `. , - + ( ) # °`; empieza con letra o número; no se repite (sin distinguir mayúsculas). Máximo 200 poses.

Dónde se guardan: en `<--datos>/poses_guardadas.json`. Con `abrir_control_real.sh` es `~/Descargas/ROBOTICA_01/proyecto_brazo/08_datos_locales/poses_guardadas.json`; sin `--datos`, `vevas_studio/datos/` (ignorada por git). La escritura es atómica (archivo temporal + reemplazo). Si el archivo está dañado, la consola avisa, lo ignora y guarda una copia `.danado-<fecha>` antes de volver a escribir.

Compuertas de «Ir»: las mismas que «Volver a cero». En SIM siempre; con el robot real por RWS **solo con C02 verificado**; después pasa por revisión previa, GO, armado, checklist y confirmación. Si la postura cambió desde que se calculó, hay que recalcular. Un destino singular (por ejemplo una pose guardada en q = 0) se acepta como aviso solo con RWS + C02.

### 2.14 Velocidad: slider y pestaña Velocidades

#### 2.14.1 Slider «Velocidad del plan»

Está en la cabecera: de 1 a 50 % de la ficha (250/250/250/320/320/420 °/s); arranca en 5 %.

| Valor | Color | Mensaje |
|---|---|---|
| ≤ 10 % | normal | (ninguno) |
| > 10 % y ≤ 25 % | naranja | «Más del 10 %: solo simulación / fuera de lo verificado» |
| > 25 % | rojo | el mismo mensaje |

Qué pasa con el robot real:

1. **RWS con C02 verificado**: aparece la nota «Robot real: tope físico C02 = 5 % de la ficha por junta». El RAPID limita cada junta a vLim aunque el plan pida más: el robot no va más rápido, **solo tarda más**. La revisión no rechaza por eso; agrega el chequeo **«Tope físico del RAPID (C02)»**: aviso si el plan supera vLim («el RAPID limita J1 a 12.5°/s…; el robot real tardará ≈ T s (C02) en vez de Tplan s») o aprobado si no. T se calcula con los mismos puntos y la misma ley del módulo.
2. **Sin C02** (ROS real, o RWS sin evidencia): el slider queda en **10 % como máximo**, porque no hay tope físico verificado.
3. **Siempre**: el slider nunca supera el tope del GO ni el de `config.json`. Si el GO autoriza menos del 5 % con RWS, el envío se rechaza (el RAPID no puede ir más lento que vLim sin cambiar el módulo).

#### 2.14.2 Pestaña «Velocidades»

Por junta muestra:

| Columna | Qué es |
|---|---|
| MEDIDA | Velocidad calculada de las posiciones recibidas: pendiente por mínimos cuadrados sobre los últimos ~0,3 s, con la marca de tiempo de cada muestra (descarta muestras repetidas) |
| PLAN | Velocidad comandada en ese instante. Con RWS es el plan escalado al tiempo C02 y se marca «estimada» |
| TOPE | Con RWS + C02, vLim del RAPID; en SIM, ficha × % del slider |
| % del tope | Barra verde (< 80 %), naranja (80–100 %) o roja (> 100 %) |
| MÁXIMO | Máximo medido del último movimiento; se reinicia al empezar cada movimiento y con «Reiniciar máximos» |

Debajo hay un gráfico de los últimos 10 s (una línea por junta). Los mismos números se guardan en la sesión: `muestras.csv` agrega las columnas `v1_med_deg_s` a `v6_med_deg_s` y `resumen.json` el máximo medido y el tiempo C02.

### 2.15 Secuencia completa del día de la defensa

En la terminal:

```bash
cd ~/Descargas/ROBOTICA_01/proyecto_brazo
bash 02_conexion/diagnosticar.sh                    # red, RWS, juntas y FK COINCIDE
bash 05_seguridad_y_control/verificar_c02.sh        # solo si cambió el módulo o el modo, o falta evidencia v2
bash 05_seguridad_y_control/abrir_control_real.sh   # GO (dos preguntas si falta) y consola en :9012
```

Antes de cada script que mueve: llave en AUTO, célula despejada, una persona en el paro de emergencia, «Iniciar» en la ventana del operador del FlexPendant cuando aparezca.

En la web (`http://127.0.0.1:9012`):

1. Confirmar cabecera **IRC5 vía RWS · ROBOT REAL**, sin aviso de C02, juntas moviéndose.
2. **ARMAR**: checklist y frase `MOVER ROBOT REAL`.
3. Si J5 está en ≈ 0: **Preparar J5 a +30°** (o dejar que la figura agregue la aproximación).
4. **Poses predeterminadas** → figura (por ejemplo Corazón, 25 mm, De frente) → mirar la vista de puntos → Calcular trayectoria.
5. **Revisión**: leer avisos (tope físico C02 con el tiempo real, salida de singularidad).
6. **Enviar** → leer el diálogo de confirmación → confirmar.
7. Mirar el robot con la mano cerca del paro; después, el panel de la figura con el error medido y la pestaña Velocidades.
8. Para cortar: **PARAR** o **Esc**. Ante cualquier duda: **paro de emergencia físico**.

### 2.16 Procedimientos de recuperación

#### 2.16.1 «Aceptar objetivos enviados por la PC?» e «Iniciar»

1. La pregunta aparece en la **ventana del operador** del FlexPendant, con los botones **Iniciar** y **Cancelar** abajo. Hay 30 s. Si no se ve, abrir la ventana del operador (Production Window / Operator Window).
2. Si el script se cortó con **Ctrl+C** mientras esperaba, `VevasServidor` **sigue corriendo** en el IRC5 (esperando «Iniciar» o ya activo). Antes de reintentar, pulsar **STOP** en el FlexPendant (el botón cuadrado de detener programa, **no** la seta de emergencia).
3. Si se intenta instalar o mover el puntero con el programa corriendo, el cliente responde «RAPID debe estar confirmado en stopped». Solución: STOP y volver a correr el script. (`verificar_c02.sh` ya detiene solo un servidor de la misma versión que quedó esperando «Iniciar».)

#### 2.16.2 RWS colgado

Síntoma: el ping responde y la petición sin autenticar sigue contestando 401, pero las peticiones autenticadas dan «timed out».

```bash
curl -s -o /dev/null -w "%{http_code}\n" --max-time 5 http://192.168.125.1/rw/panel/opmode   # 401 = el servidor web vive
pgrep -af "consola_vevas|verificar_c02|nodo_rws_ros|rws_irc5"                                # nuestros procesos conectados
```

Qué hacer, en este orden:

1. Esperar uno o dos minutos: el IRC5 tiene pocas sesiones y se liberan solas.
2. Cerrar nuestros procesos conectados por RWS (Ctrl+C en su terminal o `kill <PID>` con el número que muestra `pgrep`) y pedir a los otros grupos que cierren los suyos.
3. Último recurso: reiniciar el controlador desde el FlexPendant con **Reiniciar normal** (antes STOP si hay programa corriendo; si no, sale el evento 20054). **No apagar con el interruptor principal.**
4. Después de un reinicio, seguir 2.16.3 y 2.16.4.

#### 2.16.3 Contadores de revoluciones tras un reinicio

Síntomas: eventos **10036** (contador de revoluciones no actualizado) y **20082** (contadores perdidos por la batería SMB), y las lecturas de juntas por RWS devuelven HTTP 400.

1. Llave en **MANUAL**.
2. Llevar **cada eje a su marca de sincronización** con el joystick (la ranura o flecha de la carcasa alineada con la marca del eslabón).
3. En el FlexPendant: **ABB → Calibración → ROB_1 → Cuentarrevoluciones → Actualizar**, marcar los ejes y confirmar.
4. Comprobar a ojo, con el robot en q = 0, que **todas las marcas quedan alineadas**. Si un eje quedó corrido unos grados (nos pasó con J3), repetir **solo ese eje**: alinearlo con un incremento pequeño del joystick y volver a actualizar su contador.
5. Si queda menos de 0,5° residual, es la calibración fina del eje: **no tocar**.
6. Correr `bash 02_conexion/diagnosticar.sh`. Importante: «FK COINCIDE» **no detecta** este error, porque compara nuestra FK contra las mismas lecturas del controlador; la única verificación válida es ver las marcas en q = 0.

El evento **10106** (tiempo de servicio) es solo un recordatorio de mantenimiento; no bloquea nada.

#### 2.16.4 Después de pasar a AUTO

1. Confirmar en el FlexPendant el cuadro de cambio de modo.
2. Reconocer los avisos pendientes.
3. Soltar las setas de emergencia (girar para liberar) y reconocer el paro.
4. Pulsar **MOTORS ON** (botón blanco del controlador). La luz indica:

| Luz de MOTORS ON | Significado |
|---|---|
| Fija | Motores encendidos, listo |
| Parpadeo lento | Motores apagados por un paro de seguridad activo (seta, puerta): revisar el paso 3 |
| Parpadeo rápido | Robot sin sincronizar o contadores no actualizados: volver a 2.16.3 |

Tras un reinicio, MOTORS ON por RWS devuelve 403: lo pulsa siempre una persona (la consola la espera 60 s).

#### 2.16.5 Reconectar el cable

```bash
ip -br addr show eno2
nmcli connection up "Conexión cableada 1"
sudo ip addr add 192.168.125.207/24 dev eno2    # solo si la conexión de nmcli no existe
ping -c 3 192.168.125.1
```

### 2.17 Qué hacer ante cada error típico

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| `ping` falla | Cable, IP de `eno2` o puerto equivocado | Puerto SERVICE; 2.16.5 |
| Ping OK, `curl` da 401, peticiones autenticadas «timed out» | RWS colgado o sesiones agotadas | 2.16.2: esperar, cerrar procesos, último recurso Reiniciar normal |
| HTTP 401 en la consola | Usuario o clave de RWS mal cargados | Revisar `VEVAS_RWS_CLAVE` en la terminal; no ponerla en archivos |
| HTTP 500 «Error registering callback» | Sesiones RWS agotadas | La consola hace `/logout` y reintenta; si sigue, cerrar otras herramientas y esperar |
| HTTP 400 al leer juntas tras reiniciar | Contadores de revoluciones sin actualizar (10036/20082) | 2.16.3 en MANUAL |
| HTTP 403 al encender motores | Tras reinicio RWS no puede MOTORS ON | El operador pulsa MOTORS ON (la consola espera 60 s) |
| «RAPID está ejecutando otro programa» | Otro grupo tiene RAPID corriendo | No arrancar encima; pedirles que lo detengan |
| «RAPID debe estar confirmado en stopped» | Quedó corriendo `VevasServidor` (por ejemplo tras Ctrl+C) | STOP (botón cuadrado, no la seta) y reintentar |
| No aparece «Iniciar» o se canceló | Pasaron los 30 s, se pulsó Cancelar o no se abrió la ventana del operador | 2.16.1; volver a correr el script |
| Aviso de C02 en la cabecera / envío bloqueado | Falta evidencia, es versión 1, el módulo cambió o el modo no es el verificado | `verificar_c02.sh` en el modo actual |
| «falta la prueba de streaming aprobada» | Evidencia sin la prueba de zonas | Repetir `verificar_c02.sh` completo |
| C02 NO pasó | Una junta superó vLim, `vvRbTiempo` no coincidió o los códigos de streaming no fueron `3…3, 2` | No mover; guardar la salida completa y revisarla |
| GO inválido o vencido | Fecha de otro día o NO-GO | `crear_go.py --reemplazar` respondiendo solo lo verificado |
| GO autoriza menos del 5 % | El RAPID no va más lento que vLim | Crear el GO con Enter (5 %) o bajar vLim en el `.mod` y verificar de nuevo |
| Slider no pasa de 10 % con robot real | Sin C02 verificado | Es lo esperado; verificar C02 |
| Revisión «rechazado» | Límite, singularidad, mesa, autocolisión o altura | Leer el motivo; cambiar objetivo o rama; salir de HOME con «Preparar J5» |
| Revisión «no_verificado» | Algo no se pudo evaluar (p. ej. herramienta sin medir) | En real bloquea; completar el dato o no mover |
| Figura rechazada «Punto k/65» | Sin rama continua o salto > 12° | Tamaño menor, otro plano u otra postura inicial |
| «Postura cambió» | El robot se movió desde que se calculó el plan | Recalcular |
| Watchdog: muestra vieja | La lectura de juntas dejó de llegar | Revisar RWS; no enviar hasta que se actualice |
| Código −1 | Objetivo vacío (9E9) | Recalcular y enviar de nuevo |
| Código −2 | Objetivo fuera de límites con margen 2° | Cambiar objetivo o rama |
| Código −3 | Paso mayor a 45° contra el objetivo anterior | No debería pasar (la PC parte en ≤ 10° con zonas, ≤ 40° sin zonas); reportar |
| Código −4 | Modo no permitido | AUTO o manual reducido según lo verificado |
| Aviso 50024 «corner path failure» | El punto siguiente no llegó a tiempo y el robot paró en un punto con zona | No es un error; si se repite mucho, revisar carga de la PC y de RWS |
| Evento 20054 al reiniciar | Programa en ejecución | STOP en el FlexPendant y luego reiniciar |
| Evento 10106 | Recordatorio de tiempo de servicio | Ninguna acción para la sesión |

## 3. Cómo lo hicimos: arquitectura y archivos

### 3.1 Arquitectura en capas

```text
 Navegador (web/: Three.js, uPlot, KaTeX)
      |  HTTP local: GET estado (SSE), POST órdenes (armar, calcular, revisar, enviar, parar, poses, vista_figura)
      v
 consola_vevas.py  ->  vevas/servidor.py  ->  vevas/consola.py (lógica central)
      |                                          |
      |        modelos propios (Hitos 1 y 2):    |  compuertas y datos:
      |        robot_irb120.py (FK D-H)          |  seguridad.py (GO, config, semáforo)
      |        pieper.py (IK 8 ramas)            |  revision.py (revisión previa, tope C02)
      |        jacobiano.py (J, sigma_min)       |  c02.py (evidencia de velocidad, v2)
      |        trayectoria.py (LSPB/quíntico)    |  velocidad.py (slider, velocidad medida)
      |        figuras.py, mesa.py               |  poses_guardadas.py, registro.py
      v
 Backend intercambiable:  backend_sim.py | backend_ros.py | backend_rws.py
                                                      |
                                                      v
                                     vevas/rws_cliente.py (HTTP + digest + cookies, streaming)
                                                      |
                                    red SERVICE 192.168.125.0/24
                                                      v
                       IRC5 Compact, RobotWare 6.12.01, RWS 1.0
                       rapid/VEVAS_RWS.mod -> VevasServidor -> MoveAbsJ \T (z1 o fine)
                                                      |
                                                      v
                                     ABB IRB 120  (paro de emergencia físico)
```

La idea clave es que **los modelos son comunes a los tres backends**: lo que se prueba en simulación es exactamente el mismo código que mueve el robot. Solo cambia la última capa.

### 3.2 Flujo de datos

1. **Lectura** (siempre): `backend_rws.py` pide `/rw/motionsystem/mechunits/ROB_1/jointtarget` a 10 Hz; la consola calcula FK, σmin, velocidad medida y estado, y lo empuja a la web por SSE.
2. **Plan**: la web manda el objetivo por POST; la consola ejecuta Pieper, verificación FK, elección de rama, trayectoria y revisión, y devuelve todo con números para las pestañas.
3. **Envío**: con todas las compuertas en verde, la consola repite la revisión con la postura medida y el backend arma los puntos con `muestrear_con_paradas`: tramos rectos de hasta 10°, curvas con desvío de cuerda ≤ 0,1°, paradas del plan como puntos con `fine`. `rws_cliente.py` hace el protocolo PERS punto por punto (`enviar_recorrido`).
4. **Registro**: cada sesión queda en una carpeta con `eventos.jsonl`, `muestras.csv`, `resumen.json` y `planes/` (`registro.py`), que se puede analizar con `herramientas/analizar_sesion.py`.

### 3.3 Qué hace cada archivo importante

| Archivo | Qué hace |
|---|---|
| `consola_vevas.py` | Punto de entrada. Argumentos: `--modo sim/ros/rws`, `--real`, `--go`, `--preparar-rapid`, `--puerto`, `--sesiones`, `--datos`, `--abrir` |
| `vevas/servidor.py` | Servidor HTTP local (biblioteca estándar): sirve `web/`, emite el estado por SSE y recibe órdenes por POST, incluidas `poses_*`, `vista_figura` y `reiniciar_vel_max` |
| `vevas/consola.py` | Lógica central: estado, armado (frase «MOVER ROBOT REAL»), plan de 8 pasos, figuras con aproximación J5, poses guardadas, tope del slider, envío, PARAR, validación del GO |
| `vevas/robot_irb120.py` | Modelo D-H del IRB 120 (tabla, límites, velocidades de ficha), FK a `tool0` y al TCP |
| `vevas/pieper.py` | IK analítica por desacople de Pieper: 8 ramas, verificación FK de cada una, elección de rama |
| `vevas/jacobiano.py` | Jacobiano geométrico, σmin escalado y tipo de singularidad |
| `vevas/trayectoria.py` | Perfiles LSPB y quíntico para seis juntas, sincronizados, con paradas por tramos |
| `vevas/figuras.py` | Puntos del círculo, corazón, cuadrado, triángulo y saludo en arco; paradas; plano de vista |
| `vevas/poses_guardadas.py` | Almacén de poses: validación de nombre y juntas, escritura atómica, tolerancia a archivo dañado |
| `vevas/velocidad.py` | Tope del slider (≤ 50 %, 10 % verificado sin C02) y velocidad medida por mínimos cuadrados (~0,3 s) |
| `vevas/secuencias.py` | Secuencia de demostración (pestaña Demo) |
| `vevas/mesa.py` | Geometría de la mesa/estructura en el marco base |
| `vevas/revision.py` | Revisión previa: límites, velocidad, tope físico C02, singularidad, altura, autocolisión, entorno, herramienta |
| `vevas/seguridad.py` | Configuración por defecto, carga de `config.json`, semáforo de error |
| `vevas/c02.py` | Velocidades máximas (ventana 0,10 s) y evaluación de la evidencia (versión 2, exige streaming) |
| `vevas/backend_sim.py` | Simulador interno con reloj propio |
| `vevas/backend_ros.py` | Conexión opcional con ROS 2 Jazzy (`abb_ros2` / EGM) |
| `vevas/backend_rws.py` | Robot real: lectura a 10 Hz, watchdog, compuerta C02, plan de puntos con zonas, tiempo C02 estimado |
| `vevas/rws_cliente.py` | Cliente RWS: digest, cookies, `/logout`, instalar/cargar módulo, start/stop, PERS, envío por tramos y streaming, candado de archivo |
| `vevas/verificacion_pendant.py` | Compara la pose de la brida del IRC5 con la FK propia (solo lectura) |
| `vevas/registro.py` | Registro de cada sesión (con `v1..v6_med_deg_s`) |
| `rapid/VEVAS_RWS.mod` | Módulo RAPID (4707 caracteres): `VevasServidor`, validación, ley C02, zonas con `vvPcZona` y `tZona` |
| `rapid/TRob1Main_VEVAS.mod` | Variante de `TRob1Main` para la vía EGM/ROS (opcional) |
| `web/index.html`, `web/estilo.css` | Estructura y estilo (cabecera con slider, pestañas, diálogos, poses, panel de figura) |
| `web/app.js` | Lógica de la interfaz: SSE, órdenes, flujo de 8 pasos, poses, figura, Velocidades, PARAR con Esc |
| `web/calculos.js` | Cálculos mostrados (KaTeX), estado del slider, filas de Velocidades, error de la figura |
| `web/vista_figura.js` | Dibujo 2D de los puntos de la figura (planificados y medidos) |
| `web/escena.js`, `web/escena_worker.js` | Gemelo 3D con Three.js, vista inicial y «De frente», puntos de la figura |
| `web/arrastre.js`, `web/orientacion.js` | Flechas y anillos para mover el TCP destino; orientación ZYX |
| `web/graficas.js` | Gráficas con uPlot: error en vivo, trayectorias y velocidad de 10 s |
| `herramientas/verificar_c02.py` | Verificación presencial de C02 (seis juntas + streaming J6), lectura a ~30 Hz |
| `herramientas/crear_go.py` | GO de la sesión en dos preguntas (nunca automático) |
| `herramientas/rws_irc5.py` | Ayudante de terminal: `estado`, `instalar`, `arrancar`, `servidor`, `parar`, `pp-main`, `texto`, `diagnostico` |
| `herramientas/conectar_robot.py` | Diagnóstico de red, RWS y FK en solo lectura (lo usa `diagnosticar.sh`) |
| `herramientas/comparar_pendant.py` | FK propia contra pose del controlador, sin mover |
| `herramientas/verificar_convencion_rws.py` | Comprueba la convención de marcos entre FK y RWS |
| `herramientas/validacion_offline.py` | Validación S0 de FK, IK y Jacobiano sin robot |
| `herramientas/analizar_sesion.py` | Analiza una sesión grabada |
| `herramientas/nodo_rws_ros.py`, `rviz_rws.launch.py` | Publican `/joint_states` desde RWS y abren RViz (solo visualización) |
| `iniciar_real.sh`, `iniciar_simulacion.sh` | Lanzadores de la carpeta del código (solo chequeo, solo lectura, simulación) |
| `config.json` | IP, topes del slider (50), entorno (mesa, herramienta), umbrales, paso de envío |
| `tests/` | 346 pruebas Python (con `mock_rws.py`, un IRC5 simulado por HTTP con zonas) y 20 JS |

## 4. Glosario de conceptos

**FK (cinemática directa).** Dadas las seis juntas q, calcula dónde está la brida (`tool0`) y con qué orientación. Se multiplica una matriz D-H por eslabón, A_i = Rz(θ_i)·Tz(d_i)·Tx(a_i)·Rx(α_i), y al final un giro Rz(π) para que el marco 6 coincida con el `tool0` de ABB. Con q = 0 la brida queda en (0,374; 0; 0,630) m.

**Pieper y las 8 ramas.** Método analítico de cinemática inversa que funciona porque los tres últimos ejes del IRB 120 se cortan en un punto (la muñeca). La posición de la muñeca define q1, q2, q3 y la orientación define q4, q5, q6. Hay tres decisiones binarias: hombro adelante/atrás, codo arriba/abajo y muñeca sin giro/girada. 2 × 2 × 2 = 8 ramas, todas con `atan2`.

**Verificación FK.** Cada rama de Pieper se mete de vuelta en la FK y se compara con la pose pedida. Se acepta si e_p < 1e-6 m y e_R < 1e-5 rad. En la práctica da ~1e-15 m.

**Jacobiano.** Matriz 6×6 que relaciona velocidades articulares con la velocidad del TCP: v = J(q)·q̇. Usamos el geométrico.

**σmin.** El valor singular más chico del Jacobiano, con la parte lineal dividida por L = 0,3 m. Umbrales: σmin < 0,02 es **singular**, σmin < 0,04 es **cerca**.

**Singularidades.** Posturas donde det J = 0. Para el IRB 120, det J = −a2·L3·sin δ·r_W·sin q5: **muñeca** (|sin q5| < 0,09), **codo** (|sin δ| < 0,09) y **hombro** (r_W < 0,03 m).

**LSPB.** Perfil trapezoidal de velocidad (acelera, crucero, frena). Es el del Hito 2.

**Quíntico.** Polinomio de grado 5 con posición, velocidad y aceleración nulas en los extremos. Es el perfil «Suave». Pico de velocidad = 1,875 veces la media.

**RWS (Robot Web Services).** Interfaz HTTP del controlador ABB (1.0 en RobotWare 6). Lee juntas, estado, modo, eventos y escribe variables RAPID. Requiere la opción PC Interface (616-1).

**RAPID.** Lenguaje de los robots ABB. El nuestro es `VEVAS_RWS` con la rutina `VevasServidor`.

**PERS.** Variable persistente de RAPID, legible y escribible por RWS. Es el «buzón» entre la PC y el robot.

**MoveAbsJ.** Lleva el robot a un objetivo **articular** interpolando en juntas. No usa la IK del controlador. Con `\T:=t` se impone la duración.

**fine y zona `z1`.** El último argumento de un movimiento. `fine`: llegar exactamente y detenerse. `z1`: pasar a 1 mm del punto sin parar y empalmar con el siguiente. Usamos `z1` en los puntos intermedios de un recorrido y `fine` en el último y en las paradas.

**Movimiento continuo (streaming).** Mandar los puntos de un recorrido uno detrás de otro mientras el robot se mueve: el RAPID encola el punto con zona y contesta **código 3**; la PC escribe el siguiente sin esperar a que el robot llegue.

**vvPcZona.** PERS que la PC pone en 1 antes de un punto intermedio (zona) y en 0 antes del último o de una orden suelta (`fine`).

**tZona.** Piso de tiempo de un punto con zona: 0,2 s (con `fine` el piso es 0,5 s). La ley C02 no cambia.

**Aviso 50024 («corner path failure»).** El IRC5 avisa que no pudo empalmar porque el punto siguiente no llegó a tiempo; el robot para en ese punto. No es un error.

**Parada de figura.** Muestra donde el plan tiene velocidad nula (esquina, cúspide, extremo del saludo). En el robot real va con `fine`.

**Plano de vista.** Plano de dibujo de la figura: YZ «De frente» (por defecto), XY «Desde arriba», XZ «De costado».

**Aproximación J5.** Tramo articular J5 → +30° con parada que el plan de una figura agrega solo cuando parte con J5 ≈ 0.

**Pose guardada.** Seis juntas medidas, pose del TCP por FK, nombre y fecha, en `poses_guardadas.json`.

**Velocidad medida.** Pendiente de q(t) por mínimos cuadrados sobre ~0,3 s de posiciones recibidas.

**GO.** Archivo JSON de autorización de la sesión (`GO_VEVAS.json`): tope de velocidad, checklist, entorno confirmado y fecha. Se crea con `crear_go.py` en dos preguntas y vale todo el día.

**C02.** El requisito «ninguna junta supera su tope de velocidad». Cerrado con una ley de tiempo dentro del RAPID más una medición presencial (`evidencia_c02.json`, versión 2 con prueba de streaming).

**Contadores de revoluciones.** Cuentan en qué vuelta está cada motor; se mantienen con la batería SMB. Si se pierden (10036, 20082) el robot puede creer que está en otra postura. Se actualizan en MANUAL con cada eje en su marca.

**AUTO / MANUAL.** Modos del IRC5, con la llave. AUTO ejecuta sin habilitador. Manual reducido limita a 250 mm/s y exige el habilitador. La evidencia C02 vale solo para el modo en que se midió.

**Mastership.** «Propiedad» del dominio RAPID para escribir por RWS; se pide y se suelta en cada escritura.

**Digest.** Autenticación HTTP en la que la clave nunca viaja: el cliente responde con un hash MD5 de usuario, clave, nonce y petición. Después se trabaja con cookies.

**Watchdog de estado.** Vigila la edad de la última muestra de juntas; si es vieja, bloquea el envío.

**Huella SHA-256.** Resumen del texto del módulo. Cambiar un carácter cambia la huella e invalida la evidencia C02.

## 5. Por qué cada decisión

| Decisión | Alternativa descartada | Motivo |
|---|---|---|
| RWS como vía al robot real | EGM con ROS 2 como vía principal | RWS solo necesita PC Interface y un módulo RAPID propio; no interfiere con otros grupos. EGM queda como extensión |
| Objetivos articulares (`MoveAbsJ`) | `MoveL` cartesiano en el robot | No invierte el Jacobiano: cerca de singularidades no hay velocidades explosivas |
| Pieper analítico con 8 ramas | IK numérica | Exacto (~1e-15 m), da todas las soluciones, no depende de un punto inicial, es el modelo del Hito 2 |
| Mismos modelos en SIM, ROS y RWS | Código distinto para simular | Lo validado en simulación es lo que mueve el robot |
| Tope de velocidad dentro del RAPID | Tope solo en la PC | En el controlador no depende de que la PC funcione bien |
| `MoveAbsJ \T:=t` con t calculado | `speeddata` | `speeddata` limita el TCP, no cada eje |
| vLim = 5 % de la ficha, kPico = 2,5 | Tope mayor, kPico = 1 | Primer contacto; el pico de un perfil suave es hasta 1,875–2 veces la media |
| Evidencia atada a SHA-256, modo y topes | Evidencia sin vínculo | Cambiar módulo o modo invalida la autorización solo |
| Protocolo PERS con número de secuencia | Arrancar una rutina por movimiento | Un solo servidor; escribir `vvPcSeq` es el único acto que ordena movimiento |
| Releer `vvPcTarget` antes de `vvPcSeq` | Escribir y disparar | El robot lee exactamente lo que la PC quiso escribir |
| Doble validación (PC y RAPID) | Validar solo en la PC | El controlador es la última barrera, no la única |
| Acuse por postura medida (< 0,05°) | Confiar en el código 2 | Importa dónde quedó el robot |
| `confirmarEnPendant = TRUE` | Arrancar sin preguntar | Una persona frente al robot acepta con «Iniciar» |
| Módulo < 4800 caracteres, todo `LOCAL` o `vv` | Módulo largo con nombres libres | RW6 trunca a 5000 al leerlo; nombres globales chocan con otros grupos |
| **Streaming con zonas `z1`** | Tramos con `fine` (pausas) | Con `fine` el robot frena en cada punto; con `z1` empalma y la figura sale fluida |
| **Código 3 apenas encola** | Esperar al código 2 en cada punto | Sin el 3 la PC no podría mandar el siguiente mientras el robot se mueve, y el robot pararía igual |
| **`fine` en el último punto y en las paradas** | Zona en todo | El último punto tiene que confirmarse por postura medida; en las esquinas la zona redondearía la figura |
| **Piso tZona = 0,2 s** | Piso de 0,5 s también con zona | Con 0,5 s un recorrido de 60 puntos chicos tardaría 30 s; 0,2 s deja tiempo a la PC para escribir el siguiente. El tope kPico·|Δq|/vLim no cambia |
| **Δq contra el objetivo anterior** | Δq contra la postura actual | Con zonas el robot todavía no llegó al punto anterior; medir contra el objetivo encolado es lo que el robot va a recorrer |
| **Tramos rectos de hasta 10° en streaming** | 40° como sin zonas | Acota lo que queda encolado tras PARAR (≤ 2 tramos, ≤ 20°) |
| **C02 versión 2 con prueba de streaming J6** | Reusar la evidencia de `fine` | Las zonas cambian el módulo y la forma de moverse: se mide de nuevo, incluido el empalme |
| **Aproximación J5 → +30° dentro del plan** | Rechazar o pedir «Preparar J5» antes | Un solo plan revisado y confirmado; la salida de singularidad es la misma ya aceptada |
| **Saludo en arco ∩ con Pieper** | Saludo articular de muñeca | Se ve como un saludo desde el frente y pasa por la misma tubería que las demás figuras |
| **Figuras de frente (YZ) por defecto** | XY | El robot mira a +X: el público ve el plano YZ; la figura se ve derecha |
| **Poses en `08_datos_locales`, fuera del código** | Guardarlas en el repo | Son datos del laboratorio; actualizar el código no las pisa |
| **«Ir a pose» con las reglas de «Volver a cero»** | Reglas propias | Misma compuerta C02 y misma excepción de destino singular |
| **Slider hasta 50 % con aviso, no rechazo, por el tope C02** | Rechazar si supera vLim | El RAPID nunca deja ir más rápido; el plan solo se estira. Se avisa el tiempo real |
| **Sin C02, slider ≤ 10 %** | 50 % igual | Sin tope físico verificado no se pasa de lo ensayado |
| **Velocidad medida por mínimos cuadrados (~0,3 s)** | Diferencia de dos muestras | RWS entrega ~10 Hz y el bucle 50 Hz: hay muestras repetidas y tiempos irregulares |
| **GO en dos preguntas** | Diez preguntas con nombres | Las preguntas largas demoraban la sesión; lo que importa (seguridad y tope) sigue siendo una declaración explícita |
| **`verificar_c02` a ~30 Hz** | Leer sin pausa | Sin pausa saturaba RWS del IRC5 |
| **`subir_24nov.sh` sin tocar GO, evidencia ni sesiones** | Copiar todo | Una actualización no puede borrar autorizaciones ni evidencia |
| Revisión con 4 estados | Aprobado/rechazado | «No se pudo evaluar» no es «cumple» |
| Excepción estrecha para entrar/salir de q = 0 | Rechazar todo plan singular | Si no, el robot quedaría sin salida desde su propio reposo |
| Frase «MOVER ROBOT REAL» + checklist | Un solo botón | Evita envíos por un clic accidental |
| Motores: solo apagar si hay que escribir el módulo | Apagar «por las dudas» | Dejaba al sistema en un estado del que la PC no salía (403) |
| No arrancar encima de RAPID ajeno | Detener lo que haya | El controlador es compartido |
| Figuras ≤ 80 mm en real, salto ≤ 12° | Figuras grandes | Espacio de trabajo chico y lejos de límites y de la mesa |

## 6. Lenguajes, herramientas y protocolos

### 6.1 Lenguajes y bibliotecas

| Capa | Lenguaje / herramienta | Uso |
|---|---|---|
| Cálculo y control | Python 3 con NumPy | Modelos, revisión, backends, cliente RWS, servidor HTTP (biblioteca estándar) |
| Pruebas | pytest, `node --test` | 346 pruebas Python (con `mock_rws.py`) y 20 JS |
| Interfaz | HTML, CSS, JavaScript (módulos ES) | Consola web |
| 3D | Three.js | Gemelo, flechas, anillos, puntos de la figura |
| 2D | Canvas | Vista de puntos de la figura |
| Gráficas | uPlot | Error en vivo, trayectorias, velocidad 10 s |
| Fórmulas | KaTeX | Pestaña Cálculos |
| Robot | RAPID | `VEVAS_RWS.mod` |
| Visualización opcional | ROS 2 Jazzy, RViz, URDF | `/joint_states` desde RWS |
| Scripts | Bash, Git | Lanzadores, seguridad, `subir_24nov.sh` |
| Informe | LaTeX (IEEEtran) | Informe del hito |

### 6.2 HTTP y REST

HTTP es el protocolo de pedido/respuesta: método (GET leer, POST actuar, PUT subir), dirección y código de respuesta (200 bien, 400 pedido inválido, 401 sin autenticar, 403 prohibido, 500 error del servidor). RWS es REST: cada cosa del controlador es una dirección.

Lo usamos dos veces:
1. **Navegador ↔ consola** (127.0.0.1:9012): estado por SSE y órdenes por POST.
2. **Consola ↔ IRC5** (192.168.125.1:80): RWS.

### 6.3 Autenticación digest y sesiones

1. La primera petición recibe 401 con un `nonce`.
2. El cliente responde con un hash de usuario, clave, nonce, método y dirección. La clave no viaja.
3. El controlador entrega una cookie de sesión.
4. El IRC5 admite pocas sesiones: si se agotan responde 500 «Error registering callback» y el cliente hace `/logout` y reintenta. Si el servicio se cuelga, la petición sin autenticar sigue dando 401 pero las autenticadas dan «timed out» (2.16.2).

El usuario es el de fábrica («Default User»); la clave no se publica.

### 6.4 Recursos RWS que usamos

| Recurso | Para qué |
|---|---|
| `/rw/panel/opmode` | Leer el modo (AUTO / MANR) |
| `/rw/panel/ctrlstate` | Estado de motores |
| `/rw/rapid/execution` | start, stop, resetpp |
| `/rw/rapid/tasks/T_ROB1/pcp?action=set-pp-routine` | PP en `VEVAS_RWS/VevasServidor` |
| `/rw/rapid/modules` | Leer el texto del módulo, cargar y descargar |
| `PUT /fileservice/$HOME` | Subir el `.mod` |
| `/rw/rapid/symbol/data/RAPID/T_ROB1/VEVAS_RWS/<pers>` | Leer y escribir las PERS |
| `/rw/mastership/rapid` | Pedir y soltar mastership |
| `/rw/motionsystem/mechunits/ROB_1/jointtarget` | Leer las juntas (10 Hz en la consola, ~30 Hz en `verificar_c02`) |
| `/rw/elog/0` | Eventos del controlador |

Peculiaridades de RobotWare 6: el texto del módulo se trunca a 5000 caracteres al leerlo; los nombres globales deben ser únicos en `T_ROB1`; el campo de pcp tiene una errata («modulemame»); tras un reinicio MOTORS ON por RWS devuelve 403.

### 6.5 Protocolo PERS

Variables (todas en `VEVAS_RWS`):

| Variable | Quién escribe | Significado |
|---|---|---|
| `vvPcTarget` | PC | Objetivo articular (jointtarget) |
| `vvPcZona` | PC | 1: vienen más puntos (zona `z1`); 0: `fine` (último punto, parada u orden suelta) |
| `vvPcSeq` | PC | Número de orden; incrementarlo es lo único que ordena movimiento |
| `vvPcFin` | PC | Distinto de 0 termina el servidor |
| `vvRbSeq` | Robot | Última orden atendida |
| `vvRbCodigo` | Robot | Resultado de la orden |
| `vvRbActivo` | Robot | 1 si `VevasServidor` acepta órdenes |
| `vvRbTiempo` | Robot | Duración t que el RAPID impuso al último movimiento |

Códigos de `vvRbCodigo`:

| Código | Significado |
|---|---|
| 3 | Tomado en zona: encolado, vienen más puntos |
| 2 | Llegó (punto con `fine`) |
| 1 | Moviendo (punto con `fine`) |
| 0 | Listo (validación pasó / servidor esperando) |
| −1 | Objetivo vacío (9E9 sin cargar) |
| −2 | Fuera de límites con margen de 2° |
| −3 | Paso mayor a 45° contra el objetivo anterior |
| −4 | Modo no permitido |

Arranque del servidor (una vez):

```text
IRC5: VevasServidor comprueba modo (AUTO o manual reducido)
IRC5: TPReadFK "Aceptar objetivos enviados por la PC?"  [Iniciar | Cancelar], 30 s
Operador: pulsa Iniciar
IRC5: vvPcZona := 0 ; vvRbSeq := vvPcSeq ; vvRbActivo := 1 ; sondea cada 0,02 s
```

Secuencia de un punto (zona o fine):

```text
PC:   lee vvRbActivo = 1 y vvRbSeq = n ; compuertas: GO, C02, revisión, ARMADO
PC:   PUT vvPcZona := 1 (punto intermedio) o 0 (último o parada)
PC:   PUT vvPcTarget := q*
PC:   GET vvPcTarget (relectura) y compara con q* ; comprueba la postura medida
PC:   PUT vvPcSeq := n+1
IRC5: ve vvPcSeq <> vvRbSeq
IRC5: previo := objetivo anterior (si venía en zona) o CJointT() (si no)
IRC5: Validar(q*, previo): modo, límites ±2°, paso ≤ 45°   -> si falla: vvRbCodigo := -1..-4
IRC5: t := max(piso, max_i 2.5·|q*_i − previo_i| / vLim_i)   piso = 0,2 s (zona) o 0,5 s (fine)
IRC5: vvRbTiempo := t
IRC5 (zona):  MoveAbsJ q*, vPc \T:=t, z1, tool0 ; vvRbCodigo := 3     (en cuanto lo encola)
IRC5 (fine):  vvRbCodigo := 1 ; MoveAbsJ q*, vPc \T:=t, fine, tool0 ; vvRbCodigo := 2
IRC5: vvRbSeq := n+1
PC:   espera vvRbSeq = n+1 ; con 3 escribe el punto siguiente mientras el robot se mueve
PC:   en el último punto exige código 2 y |q_medida − q*| < 0,05°
PC:   al terminar deja vvPcZona := 0 para que una orden futura no herede la zona
```

Si un punto es rechazado, la PC no manda ninguno más y espera a que el robot termine lo encolado. Un candado de archivo en la PC impide que dos herramientas escriban a la vez.

## 7. Cálculos con fórmulas y ejemplos numéricos

Los ejemplos se calcularon con los módulos de `vevas/` o salen del informe IEEE.

### 7.1 Ley de tiempo C02

Fórmula (la calcula el RAPID, funciones `Tj` y `Tiempo`):

```text
t = max( piso ,  max_i  kPico · |Δq_i| / vLim_i )
piso  = 0,5 s con fine ; 0,2 s (tZona) con zona
Δq    = objetivo − objetivo anterior (en zona) o − postura actual (si no venía en zona)
kPico = 2,5
vLim  = [12,5  12,5  12,5  16  16  21] °/s   (5 % de 250/250/250/320/320/420 °/s)
```

Consecuencia: la velocidad **media** de la junta que domina es como mucho vLim/kPico = 0,4·vLim, con zona o con fine.

**Ejemplo 1 (verificación, δ = 5°):**

| Junta | vLim [°/s] | t = 2,5·5/vLim [s] | Media δ/t [°/s] |
|---|---|---|---|
| J1 a J3 | 12,5 | 1,00 | 5,0 |
| J4, J5 | 16 | 0,78 | 6,4 |
| J6 | 21 | 0,60 | 8,4 |

**Ejemplo 2 (varias juntas):** Δq = [10, 5, 0, 0, 20, 30]°.

```text
J1: 2,5·10/12,5 = 2,000 s
J2: 2,5·5/12,5  = 1,000 s
J5: 2,5·20/16   = 3,125 s
J6: 2,5·30/21   = 3,571 s   <- domina
t = 3,571 s
medias: J1 2,8  J2 1,4  J5 5,6  J6 8,4 °/s   (todas ≤ 0,4·vLim)
```

**Ejemplo 3 (movimientos de HOME con zonas):** «Volver a cero» desde [0, −20, 30, 0, 40, 0]°: 4 puntos de 10° en J5, cada uno 2,5·10/16 = 1,5625 s, total 6,25 s (igual que un solo tramo de 40°). «Preparar J5 a +30°» desde cero: 3 puntos, total 2,5·30/16 = 4,69 s.

**Ejemplo 4 (prueba de streaming de `verificar_c02`, δ = 5°):** J6 = 5·sen(2πm/8), m = 1…8.

| Punto | J6 [°] | |Δq6| [°] | t [s] |
|---|---|---|---|
| 1 | 3,536 | 3,536 | 0,421 (domina kPico) |
| 2 | 5,000 | 1,464 | 0,200 (piso tZona) |
| 3 | 3,536 | 1,464 | 0,200 |
| 4 | 0,000 | 3,536 | 0,421 |
| 5 | −3,536 | 3,536 | 0,421 |
| 6 | −5,000 | 1,464 | 0,200 |
| 7 | −3,536 | 1,464 | 0,200 |
| 8 (fine) | 0,000 | 3,536 | 0,500 (piso fine) |

Total ≈ 2,56 s. La prueba exige duración observada ≥ 0,9 × 2,56 s. Calculado con `puntos_streaming` y `tiempo_movimiento`.

**Por qué partir en puntos no cambia el tiempo:** por encima del piso, la ley es lineal en |Δq|. Si 40° se parten en 4 de 10°, cada uno tarda la cuarta parte y el total es el mismo. Con `fine` eso agregaba 3 paradas; con zonas, ninguna.

### 7.2 Factor pico/medio y por qué kPico = 2,5

Si una junta sigue q(τ) = q0 + Δq·s(τ/t), la velocidad pico es c·|Δq|/t con c = max s′. Con la ley:

```text
q̇_pico ≤ (c / kPico) · vLim
```

| Perfil | c = pico/medio | Pico con kPico = 2,5 |
|---|---|---|
| Velocidad constante | 1,00 | 0,40·vLim |
| LSPB con tb = T/4 | 1,33 | 0,53·vLim |
| Cúbico | 1,50 | 0,60·vLim |
| Quíntico | 1,875 | 0,75·vLim |
| Triangular (tb = T/2) | 2,00 | 0,80·vLim |
| **Medido en el IRC5 (`MoveAbsJ \T`, fine)** | **≈ 1,06** | **≈ 0,42–0,43·vLim** |

Quíntico: s′(τ) = 30τ² − 60τ³ + 30τ⁴; en τ = 0,5 da 1,875. LSPB: c = T/(T − tb).

Con zonas, el empalme entre dos puntos mezcla el final de uno con el principio del siguiente; la prueba de streaming de C02 versión 2 existe justamente para medir que eso tampoco supera vLim.

### 7.3 σmin y singularidades

```text
J_L = diag(I3/L, I3) · J(q),   L = 0,3 m
σmin = σ6(J_L)
singular si σmin < 0,02 ; «cerca» si σmin < 0,04
det J = −a2 · L3 · sin δ · r_W · sin q5
```

| Postura q [°] | σmin | Estado |
|---|---|---|
| [0, 0, 0, 0, 0, 0] (HOME) | 1,2e-16 | singular (muñeca) |
| [0, 0, 0, 0, 2, 0] | 0,016 | singular |
| [0, 0, 0, 0, 5, 0] | 0,039 | cerca |
| [0, 0, 0, 0, 30, 0] | 0,231 | libre |
| [0, −20, 30, 0, 40, 0] (postura inicial) | 0,261 | libre |

Criterio de **salida** de singularidad (aviso), con S = {k : σ_k < 0,02} y k* el mínimo dentro de S:
1. S es un tramo inicial contiguo {0, …, m−1} con m < N.
2. σ no decrece desde k* hasta m.
3. En S la singularidad es solo de muñeca.
4. La última muestra tiene σ ≥ 0,04.

Criterio simétrico para **llegar** a q = 0. Cualquier otra entrada a singularidad sigue rechazada.

**Umbral de la aproximación J5:** |sen q5| < 0,09 ⇔ |q5| < arcsen(0,09) = 5,16°.

### 7.4 FK y Pieper con un ejemplo completo

Tabla D-H (a [m], α, d [m], offset):

| i | a | α | d | offset θ |
|---|---|---|---|---|
| 1 | 0 | −π/2 | 0,290 | 0 |
| 2 | 0,270 | 0 | 0 | −π/2 |
| 3 | 0,070 | −π/2 | 0 | 0 |
| 4 | 0 | π/2 | 0,302 | 0 |
| 5 | 0 | −π/2 | 0 | 0 |
| 6 | 0 | 0 | 0,072 | 0 |

FK: T = A1·…·A6·Rz(π). Con q = 0: brida en (374; 0; 630) mm.

Ejemplo con la postura inicial q = [0, −20, 30, 0, 40, 0]°:

```text
FK  -> p_tool0 = (263,50 ; 0 ; 505,06) mm

Pieper (vuelta atrás):
p_W = p_6 − d6·z6 = (217,22 ; 0 ; 560,21) mm
L3  = sqrt(a3² + d4²) = sqrt(0,070² + 0,302²) = 0,3100 m
γ   = atan2(a3, d4) = 13,05°
r   = sqrt(x_W² + y_W²) = 0,2172 m ;  s = z_W − d1 = 0,2702 m
q1  = atan2(y_W, x_W) = 0°
cos δ = (r² + s² − a2² − L3²)/(2·a2·L3) = −0,2915  ->  δ = 106,95°
q3  = δ + γ − 90° = 30,00°
q2  = atan2(r, s) − atan2(L3·sin δ, a2 + L3·cos δ) = −20,00°
R_3^6 = (R_0^3)ᵀ·R_0^6  ->  q5 = atan2(±sqrt(r13² + r23²), r33) = 40°, q4 = 0°, q6 = 0°
```

Las 8 ramas para esa pose (todas con e_p < 2e-16 m):

| Rama | Hombro / codo / muñeca | q [°] | En límites |
|---|---|---|---|
| 0 | adelante / arriba / sin giro | [0, −20, 30, 0, 40, 0] | sí (elegida, margen 40°) |
| 1 | adelante / arriba / girada | [0, −20, 30, 180, −40, −180] | no (J4) |
| 2 | adelante / abajo / sin giro | [0, 97,59, 176,1, 0, 136,31, 0] | no (J3, J5) |
| 3 | adelante / abajo / girada | [0, 97,59, 176,1, 180, −136,31, −180] | no |
| 4 | atrás / arriba / sin giro | [180, −97,59, 30, −180, 162,41, 0] | no |
| 5 | atrás / arriba / girada | [180, −97,59, 30, 0, −162,41, −180] | no |
| 6 | atrás / abajo / sin giro | [180, 20, 176,1, 180, 66,1, 0] | no |
| 7 | atrás / abajo / girada | [180, 20, 176,1, 0, −66,1, −180] | no |

Las 8 existen matemáticamente y pasan la verificación FK; solo una cae dentro de los límites. Por eso la elección de rama mira límites, cercanía y σmin.

Validación fuera de línea (bitácora S0-001): error FK 6e-16 m en 5000 posturas, Jacobiano contra diferencias finitas 9e-11 m/rad, amplificación ‖q̇‖ ∼ q5^(−1,00). Los controles negativos fallan a propósito con 0,120 m y 0,469 rad.

### 7.5 LSPB y quíntico

```text
v_c = Δq / (T − tb)       velocidad de crucero
a   = v_c / tb            aceleración de mezcla
0 ≤ t < tb:        q = q0 + a·t²/2
tb ≤ t ≤ T − tb:   q = q0 + v_c·(t − tb/2)
T − tb < t ≤ T:    q = qf − a·(T − t)²/2
```

Ejemplo: Δq = 40°, T = 6,25 s, tb = 1,0 s: v_c = 7,62 °/s; a = 7,62 °/s²; media 6,40 °/s; pico/medio 1,19.

```text
q(t) = q0 + Δq·(10τ³ − 15τ⁴ + 6τ⁵),  τ = t/T
q̇_max = 1,875·Δq/T   (en τ = 0,5)
q̈_max = 5,77·Δq/T²   (en τ ≈ 0,21 y 0,79)
```

Mismo ejemplo: q̇_max = 1,875·40/6,25 = 12,0 °/s < vLim = 16 °/s.

### 7.6 Geometría del saludo

```text
θ(u) recorre 0 → +A → −A → +A → −A → 0,  A = 40°, repartido parejo a lo largo del arco (65 puntos)
R = tamaño / (2·sen A)
a = R·sen θ          (primer eje del plano: Y si es de frente)
b = −R·(1 − cos θ)   (segundo eje: Z si es de frente; ∩: el centro queda arriba)
```

Con 25 mm: R = 25/(2·0,6428) = 19,45 mm; altura del arco R·(1 − cos 40°) = 4,55 mm. Los extremos caen en los índices 8, 24, 40 y 56 (el recorrido acumulado mide 8 unidades de A y hay 64 intervalos: un extremo cada 16 muestras, empezando en la 8).

### 7.7 Velocidad medida

Con las muestras (t_k, q_k) de los últimos ~0,3 s (sin repetidas):

```text
v = Σ (t_k − t̄)·(q_k − q̄) / Σ (t_k − t̄)²
```

Es la pendiente de la recta de mínimos cuadrados. Si las muestras abarcan menos de 0,04 s o la última tiene más de 1 s, la velocidad se da como desconocida. El máximo del movimiento es el mayor |v| visto desde que empezó.

### 7.8 Tiempo real con el slider alto

Con RWS + C02 el robot tarda lo que fija la ley 7.1, no lo que dice el plan. Ejemplo: un plan al 20 % puede pedir hasta 0,20·250 = 50 °/s en J1, que supera vLim = 12,5 °/s; la revisión da aviso «Tope físico del RAPID (C02)» y muestra el tiempo T que calcula `duracion_c02_s` con los mismos puntos y la misma ley del módulo. La vigilancia de llegada usa el mayor entre 6·tf + 30 s, 2·T + 30 s y la duración estimada con holguras.

## 8. Resultados y evidencias

### 8.1 Verificación C02 en el IRB 120 real (AUTO, 24/11/2026, versión 1)

Herramienta: `herramientas/verificar_c02.py`, cada junta ±5° y vuelta, muestreo continuo de `jointtarget`, velocidad máxima en ventana de 0,10 s, `vvRbTiempo` igual al t esperado (tolerancia 2 %).

| Juntas | vLim [°/s] | t [s] | Media nominal δ/t [°/s] | vmax medida [°/s] | vmax/vLim | Resultado |
|---|---|---|---|---|---|---|
| J1–J3 | 12,5 | 1,00 | 5,0 | 5,2–5,4 | 0,42–0,43 | OK |
| J4–J5 | 16 | 0,78 | 6,4 | 6,7–6,9 | 0,42–0,43 | OK |
| J6 | 21 | 0,60 | 8,4 | 8,8–8,9 | 0,42 | OK |

Pico medido ≈ 1,06 veces la media nominal; el pico real es ≈ 2,2 % de la velocidad de ficha.

La evidencia guarda la huella SHA-256 del módulo, el modo, vLim, kPico, las seis juntas y, desde la versión 2, la prueba de streaming. El backend habilita el envío solo si:
1. la evidencia es versión 2 y dice que pasó,
2. su huella coincide con el `VEVAS_RWS.mod` local y con el cargado en el IRC5,
3. los topes coinciden y están las seis juntas,
4. la prueba de streaming pasó (si el módulo soporta zonas),
5. el modo actual es el medido.

**Estado de la versión 2:** el módulo con zonas (4707 caracteres) cambia la huella, así que la evidencia versión 1 ya no habilita. La medición con streaming se obtiene corriendo `verificar_c02.sh` en el robot (2.5); hasta tener «C02 VERIFICADO», la consola bloquea el envío.

### 8.2 FK propia contra el controlador

| Prueba | Resultado |
|---|---|
| Juntas leídas por RWS contra FlexPendant | Coinciden las seis |
| FK propia vs pose del controlador, primera lectura | 0,0001 mm y 0,0000° |
| Mismo diagnóstico tras actualizar contadores, juntas ≈ 0 | COINCIDE, 0,0000 mm |
| Validación fuera de línea, 5000 posturas | error FK 6e-16 m |
| Jacobiano contra diferencias finitas | 9e-11 m/rad |
| Pieper, verificación FK de cada rama | ~1e-15 m |

### 8.3 Pruebas automáticas

| Conjunto | Cantidad | Estado |
|---|---|---|
| Python (pytest), con `mock_rws.py`: IRC5 simulado por HTTP con zonas y código 3 | 346 | todas pasan (≈ 5,5 min) |
| JavaScript (`test_calculos.mjs`, `test_orientacion.mjs`, `test_velocidad.mjs`) | 20 | todas pasan |

El IRC5 simulado incluye un perfil que **no** respeta el tiempo, para comprobar que la evidencia falla cuando debe. Hay pruebas de cinemática, revisión, C02 (versión 2 y streaming), cliente RWS, GO de dos preguntas, figuras de frente y saludo, aproximación J5, poses guardadas, velocidad medida, slider, lanzadores, mesa y trayectorias suaves.

### 8.4 Otros resultados

1. HOME es singular: σmin(0) = 1,2e-16. «Volver a cero» con RWS + C02 da **aviso**; sin la excepción, **rechazado**. «Preparar J5 a +30°» cumple el criterio de salida.
2. Figuras resueltas con Pieper punto a punto, orientación constante, 65 puntos, salto máximo 12°, ≤ 80 mm en real, de frente por defecto. En SIM, desde q = 0, las cinco figuras de 25 mm de frente salen ejecutables con la aproximación J5 y la nota en el plan (calculado con `Consola` + `BackendSim`).
3. Movimiento continuo: «Volver a cero» desde la postura inicial sale en 4 puntos (3 con zona y el último con `fine`), sin paradas intermedias y con el mismo 6,25 s; los recorridos largos y las figuras ya no frenan entre puntos, salvo en sus paradas (calculado con el IRC5 simulado y `muestrear_con_paradas`).
4. Archivos de evidencia: `evidencia_c02.json`, sesiones en `08_datos_locales/sesiones` (con velocidad medida), `docs/bitacora/bitacora.md`, informe IEEE con figuras de `docs/informe/informe_ieee/generar_figuras.py`.

## 9. Incidentes y cómo se resolvieron

| # | Qué pasó | Causa | Qué hicimos | Qué aprendimos |
|---|---|---|---|---|
| 1 | El script no podía arrancar | Otro grupo tenía RAPID corriendo (`Imt342TrajectoryRapid/ServeTrajectoryOrders`) | El script se niega a arrancar encima; se pidió detenerlo | El controlador es compartido: nunca pisar programas ajenos |
| 2 | RWS dejó de responder con ping OK | Servicio RWS colgado | Reinicio del controlador; el FlexPendant lo rechazó con programa en ejecución (20054) hasta pulsar STOP; después paro, reset y MOTORS ON | La red puede estar bien y RWS no; hay un orden de recuperación |
| 3 | Contadores de revoluciones perdidos tras el reinicio (20082) | Batería SMB débil | Ejes a sus marcas y actualización de contadores; FK volvió a 0,0000 mm | Ningún modelo lo detecta; la verificación válida es física |
| 4 | Instalar el módulo apagaba motores encendidos | Bug propio: se apagaban «por las dudas» | Solo se apagan si hay que escribir el módulo; si RWS da 403 se espera al operador 60 s | Una medida de seguridad mal pensada puede dejar al sistema trabado |
| 5 | Pausas en el movimiento | Tramos de 10° con `fine` | Primero tramos de 40°; después streaming con zonas | El tiempo nominal no cambia; las paradas sí |
| 6 | `verificar_c02` saturaba RWS | Leía juntas sin pausa | Lectura a ~30 Hz (pausa de 0,03 s) | Un cliente demasiado ansioso puede colgar el servicio compartido |
| 7 | Un `VevasServidor` quedaba esperando «Iniciar» | Script cortado con Ctrl+C o «Iniciar» no pulsado | `verificar_c02` lo detecta y lo reinicia; si no, STOP (botón cuadrado) antes de reintentar | Ctrl+C en la PC no detiene el programa del controlador |
| 8 | RWS colgado otra vez: 401 sin autenticar, «timed out» autenticado | Sesiones ocupadas o servicio trabado | Esperar, cerrar procesos propios y de otros grupos; último recurso Reiniciar normal | El 401 prueba que el servidor web vive; el problema es la sesión |
| 9 | Tras reiniciar, J3 quedó corrido unos grados al actualizar contadores | Eje no alineado exactamente con su marca | Se repitió solo J3, alineándolo con incremento pequeño | «FK COINCIDE» no lo ve; hay que mirar las marcas en q = 0 |

El incidente 3 (y su repetición en el 9) es el más importante para la defensa: después de perder contadores el robot puede **creer** estar en una postura y estar en otra. Por eso tras cualquier reinicio se miran las marcas físicas en q = 0, y nunca se actualizan contadores solo porque las juntas «parezcan» cerca de cero.

## 10. Limitaciones y próximas mejoras (priorizadas)

| Prioridad | Tema | Situación actual | Próximo paso |
|---|---|---|---|
| 1 | Batería SMB | Débil; causó la pérdida de contadores | Cambiarla; evitar reinicios mientras tanto |
| 2 | C02 versión 2 en el robot | Código terminado y probado con el IRC5 simulado | Correr `verificar_c02.sh` (incluye streaming J6) y probar figuras en real vigilando avisos 50024 |
| 3 | Medición de velocidad | Posiciones RWS (~10 Hz en consola, ~30 Hz en `verificar_c02`); puede subestimar un pico breve | El margen kPico lo cubre; traza interna del controlador si se dispone |
| 4 | Modelo de entorno | La revisión ve solo lo descrito y hasta 240 muestras | Medir herramienta, soportes y entorno |
| 5 | Error de la figura | Medido con la FK de las juntas leídas, no con un sensor externo | Cámara cenital con calibración DLT propia (objetivo < 1,5 mm) |
| 6 | Pinza | No integrada | Pinza neumática por E/S del IRC5 |
| 7 | Control a alta frecuencia | RWS punto a punto | EGM a 250 Hz (opción 689-1 presente) |
| 8 | Tope oficial | Ley de tiempo en RAPID | SpeedLimAxis/LimitSpeed (EIO) si el docente lo exige |

Limitaciones que hay que decir con honestidad en la defensa:
1. «Aprobado» significa «cumple dentro del modelo», no «es seguro».
2. PARAR por software deja terminar lo encolado (≤ 2 tramos de ≤ 10° con zonas); el freno real es el paro físico.
3. La evidencia C02 vale solo para el módulo, los topes y el modo medidos.
4. Por encima del 5 % el slider no acelera el robot real: solo cambia el plan nominal.
5. La FK en vivo se comparó en pocas posturas, y no detecta contadores corridos.

## 11. Banco de 28 preguntas del docente

Organizado según la rúbrica ECI-3/4 (50 % de la evaluación continua): ingeniería y compatibilidad 20 %, modelos propios y validación FK/IK/J 25 %, integración ROS–ABB 25 %, seguridad y reproducibilidad 20 %, defensa 10 %. Las preguntas 21 a 28 cubren lo agregado la tarde del 24/11.

### Ingeniería y compatibilidad (20 %)

**1. ¿Por qué usaron RWS y no EGM o ROS 2 para mover el robot real?**
RWS solo necesita PC Interface y un módulo RAPID propio, no interfiere con otros grupos y deja el tope de velocidad dentro del controlador. ROS 2 queda para RViz y EGM como extensión a 250 Hz.

**2. ¿Qué restricciones de RobotWare 6 tuvieron que respetar?**
El texto del módulo se trunca a 5000 caracteres al leerlo (el nuestro tiene 4707), los nombres globales deben ser únicos en `T_ROB1` (todo `LOCAL` o `vv`), hay pocas sesiones HTTP y tras un reinicio MOTORS ON por RWS da 403.

**3. ¿Cómo conviven con otros grupos en el mismo controlador?**
No tocamos sus módulos; si hay RAPID ajeno corriendo, no arrancamos. Pedimos y soltamos mastership en cada escritura y leemos a frecuencia moderada para no saturar RWS.

**4. ¿Qué pasa si el programa se interrumpe a mitad de un recorrido?**
No se mandan más puntos. El robot termina lo ya encolado (a lo sumo el tramo en curso y el siguiente, ≤ 20° por junta) y queda quieto. El paro de emergencia físico es la parada real.

### Modelos propios y validación FK/IK/J (25 %)

**5. ¿Cómo saben que su FK es correcta?**
Contra la pose del controlador: 0,0001 mm y luego 0,0000 mm. Fuera de línea, 6e-16 m en 5000 posturas; los controles negativos fallan a propósito.

**6. ¿Por qué Pieper da 8 soluciones y cuál eligen?**
Tres decisiones binarias: hombro, codo y muñeca. Elegimos la que verifica FK, respeta límites, no es singular y está más cerca de la postura actual.

**7. ¿Qué es la verificación FK de una rama?**
Meter la solución en la FK y comparar: e_p < 1e-6 m y e_R < 1e-5 rad. Da ~1e-15 m.

**8. ¿Qué mide σmin y por qué escalan con L = 0,3 m?**
La distancia a una singularidad. Escalar la parte lineal hace comparables metros y radianes.

**9. ¿Por qué HOME es singular y cómo salen de ahí?**
Con q5 = 0 los ejes 4 y 6 se alinean (σmin = 1,2e-16). Salimos con «Preparar J5 a +30°»: σ sube monótono hasta 0,231 y la revisión lo acepta como aviso solo con RWS + C02.

### Integración con el ABB (25 %)

**10. Expliquen el protocolo entre la PC y el robot.**
La PC escribe `vvPcZona` y `vvPcTarget`, relee, comprueba la postura e incrementa `vvPcSeq`. RAPID valida, calcula t y mueve con `MoveAbsJ \T` en zona o fine; responde 3 al encolar un punto con zona y 1/2 en los fine. El último punto se acepta con código 2 y postura medida a menos de 0,05°.

**11. ¿Qué significan los códigos de respuesta?**
3 tomado en zona, 2 llegó, 1 moviendo, 0 listo, −1 vacío, −2 fuera de límites, −3 paso mayor a 45°, −4 modo no permitido.

**12. ¿Por qué `MoveAbsJ` y no `MoveL`?**
Interpola en juntas y nunca invierte el Jacobiano. La IK la hacemos nosotros con Pieper.

**13. ¿Por qué el robot hacía pausas y cómo lo resolvieron?**
Cada tramo terminaba en `fine`. Primero tramos de 40°; después streaming: zonas `z1` en los intermedios con código 3 al encolar, `fine` solo al final y en las paradas del plan. El tiempo total no cambia.

**14. ¿Qué rol cumple ROS 2 en este hito?**
Opcional: `/joint_states` desde RWS y RViz. El mismo núcleo sirve a SIM, ROS 2 y RWS.

### Seguridad y reproducibilidad (20 %)

**15. ¿Cómo garantizan que ninguna junta supera su tope (C02)?**
El RAPID impone t = max(piso; máx 2,5·|Δq_i|/vLim_i) con vLim = 5 % de la ficha, y lo medimos: picos de 5,2 a 8,9 °/s contra topes de 12,5 a 21 °/s (42–43 %).

**16. ¿Por qué kPico = 2,5?**
Un perfil suave tiene pico de hasta 1,875 (quíntico) o 2 (triangular) veces la media; con 2,5 queda ≤ 0,8·vLim. Medimos 1,06.

**17. ¿Qué pasa si alguien edita el módulo RAPID?**
Cambia su huella SHA-256 y la evidencia deja de valer hasta repetir `verificar_c02`. Pasó de verdad el 24/11 al agregar zonas.

**18. Nombren las compuertas antes de que el robot se mueva.**
GO vigente, evidencia C02 versión 2, revisión previa no rechazada ni sin verificar (repetida al enviar), ARMAR con checklist y frase, tope del slider ≤ GO, confirmación de envío, «Iniciar» en el FlexPendant, validación en RAPID, watchdog y «postura cambió». Siempre con una persona en el paro.

### Defensa (10 %)

**19. ¿Cuál fue el incidente más importante y qué aprendieron?**
La pérdida de contadores por la batería SMB, repetida tras otro reinicio con J3 corrido unos grados. El robot puede creer estar en otra postura y «FK COINCIDE» no lo ve: se verifica mirando las marcas en q = 0.

**20. ¿Qué harían a continuación?**
Cambiar la batería SMB, verificar C02 versión 2 y probar las figuras en continuo en el robot, y después cámara cenital con DLT, pinza por E/S y EGM a 250 Hz.

### Lo agregado la tarde del 24/11

**21. ¿Cómo funciona el movimiento continuo y qué significa el código 3?**
Con `vvPcZona = 1` el RAPID ejecuta `MoveAbsJ \T:=t, z1` y, en cuanto lo encola, contesta 3 y actualiza `vvRbSeq`; la PC escribe el punto siguiente mientras el robot recorre el actual. Si el punto no llega a tiempo, el robot para en ese punto (aviso 50024), sin error.

**22. ¿Por qué el último punto y las esquinas van con `fine`?**
El último tiene que confirmarse con código 2 y la postura medida a menos de 0,05°. En las esquinas del cuadrado y del triángulo, la cúspide del corazón y los extremos del saludo, una zona redondearía la figura.

**23. Si el piso baja de 0,5 s a 0,2 s con zona, ¿no se viola C02?**
No: el término 2,5·|Δq|/vLim sigue igual, así que la media de cualquier junta sigue ≤ 0,4·vLim; el piso solo evita que tramos de décimas de grado tarden 0,5 s cada uno. Igual lo medimos: C02 versión 2 agrega una prueba de streaming con J6 en 8 puntos (≈ 2,56 s con δ = 5°) y exige códigos 3…3, 2.

**24. ¿Qué pasa si piden una figura con J5 en cero?**
Pieper no tiene rama válida y continua con q5 ≈ 0 (los ejes 4 y 6 alineados, solo se determina q4 + q6). El plan agrega solo una aproximación J5 → +30° con parada en el índice 1 y dibuja desde ahí; es un único plan revisado, con una nota en verde azulado.

**25. ¿Por qué las figuras son «de frente» y cómo se ven?**
El robot mira a +X, así que el público ve el plano YZ: Y a la derecha y Z arriba. Es el plano por defecto; también hay «Desde arriba» (XY) y «De costado» (XZ). El botón «De frente» pone la cámara 3D delante del robot.

**26. ¿Cómo es el saludo nuevo?**
Un vaivén en arco ∩ de la mano, ±40°, dos idas y vueltas, ancho = tamaño (R = tamaño/(2·sen 40°)); se detiene en los extremos (índices 8, 24, 40, 56) y se resuelve con Pieper como las demás figuras.

**27. Si el slider llega a 50 %, ¿el robot real va al 50 %?**
No. Con RWS + C02 el RAPID limita cada junta a vLim (5 % de la ficha); la revisión da aviso «Tope físico del RAPID (C02)» con el tiempo real. Sin C02 el slider no pasa de 10 %, y nunca pasa del tope del GO.

**28. ¿Cómo miden la velocidad de las juntas y dónde queda?**
Pendiente por mínimos cuadrados sobre ~0,3 s de posiciones recibidas, descartando repetidas. Se ve en la pestaña Velocidades (medida, plan, tope, % con colores, máximo y gráfico de 10 s) y queda en `muestras.csv` como `v1_med_deg_s` a `v6_med_deg_s`.

## 12. Mapa de archivos del repo

### 12.1 Repositorio `pfi-vevas-irb120` (estructura por hitos)

> [!NOTE]
> Desde la reorganización del 24/11 el repositorio tiene una carpeta por hito. Lo que antes era `hito3_4/` ahora es `hito3_4_control_real/vevas_studio/` (mismo contenido, sin reordenar por dentro) y `docs/hito3_4/` se unificó en `hito3_4_control_real/01_docs/`. En la PC del laboratorio la carpeta del código sigue llamándose `00_codigo/hito3_4`.

| Ruta | Para qué sirve |
|---|---|
| `README.md`, `LICENSE` | Portada del PFI (los tres hitos) y licencia |
| `docs/` | Índice general, cronología del proyecto e imágenes comunes (banners) |
| `ros2_irb120_description/` | Paquete ROS 2 `irb120_description`: `CMakeLists.txt`, `package.xml`, `launch/display.launch.py`, `rviz/`, `urdf/` (gemelo del Hito 1) y `scripts/` (Hito 1 y Hito 2: `fk_irb120.py`, `pieper.py`, `jacobiano.py`, `singularidad.py`, `lspb.py`, `dinamica.py`, `demo_recorrido.py` y otros) |
| `hito1_gemelo_digital/` | Hito 1: web 3D (`visor_web/`), informe, presentación y validación |
| `hito2_cinematica_dinamica/` | Hito 2: `notebooks/Hito2_VEVAS.ipynb`, informe IEEE, anexo técnico, guías y resultados |
| `hito3_4_control_real/01_docs/guias/GUIA_COMPLETA_HITO3_4.md` | Esta guía |
| `hito3_4_control_real/01_docs/informe/Informe_Hito3_4.pdf` | Informe del hito en PDF |
| `hito3_4_control_real/01_docs/informe/informe_ieee/` | Fuente del informe IEEE: `informe_hito3_4_ieee.tex`, `informe_hito3_4_ieee.pdf`, `generar_figuras.py` (figuras con los mismos módulos), `img/` (arquitectura, secuencia, C02, σ, tubería), `IEEEtran.cls`, `spanish.ldf` |
| `hito3_4_control_real/01_docs/{operacion,solucion_de_problemas,bitacora,desarrollo}` | Procedimientos del laboratorio, recuperación, bitácora con `CAMBIOS_*.md` y notas internas |
| `hito3_4_control_real/vevas_studio/` | Código de VEVAS Studio (tabla 12.2) |

Presentación de la defensa (Slides): https://claude.ai/artifact/2cXikzrdtavuHUr3bijWGg

### 12.2 Carpeta `hito3_4_control_real/vevas_studio/` (en el laboratorio: `00_codigo/hito3_4`)

| Ruta | Para qué sirve |
|---|---|
| `consola_vevas.py` | Punto de entrada de la consola |
| `config.json` | Parámetros (IP, topes del slider, entorno, umbrales); comentarios con `//` |
| `requirements.txt` | Dependencias Python |
| `iniciar_real.sh`, `iniciar_simulacion.sh`, `iniciar_real_egm.sh` | Lanzadores locales (chequeo, solo lectura, simulación, EGM) |
| `subir_git.sh` | Subida de la rama 19NOV (histórico; para el 24/11 se usa `subir_24nov.sh`) |
| `GO_VEVAS_plantilla.json`, `plantillas/` | Plantilla vacía del GO (no es una autorización) |
| `../docs/bitacora/CAMBIOS_19NOV.md` … `CAMBIOS_24NOV.md` | Lista de cambios por día; `CAMBIOS_24NOV.md` es la de esta guía (en el repo están en `hito3_4_control_real/01_docs/bitacora/`) |
| `vevas/` | Paquete Python: modelos, revisión, C02, velocidad, poses, backends, cliente RWS, servidor (tabla 3.3) |
| `web/` | Interfaz: `index.html`, `estilo.css`, `app.js`, `calculos.js`, `vista_figura.js`, `escena.js`, `graficas.js`, `arrastre.js`, `orientacion.js`, `modelo/` (STL, alineación y créditos), `vendor/` (Three.js, OrbitControls, uPlot, KaTeX) |
| `rapid/VEVAS_RWS.mod` | Módulo RAPID del servidor (4707 caracteres) |
| `rapid/TRob1Main_VEVAS.mod`, `EGM_ROS.mod` (en la raíz de `vevas_studio/`) | Vía EGM/ROS opcional |
| `herramientas/` | `verificar_c02.py`, `crear_go.py`, `conectar_robot.py`, `rws_irc5.py`, `comparar_pendant.py`, `analizar_sesion.py`, `validacion_offline.py`, `verificar_convencion_rws.py`, `nodo_rws_ros.py`, `rviz_rws.launch.py`, `empaquetar_robotica01.py` |
| `modelo/` | URDF para RViz con RWS y medidas de la mesa |
| `tests/` | 346 pruebas Python (incluye `mock_rws.py`) y 20 JS |
| `docs/` | Solo `datos/validacion_offline/` (la escribe `herramientas/validacion_offline.py`), `model_manifest.yaml` y `README_AREA.md`. Las guías parciales 1 a 8, `arquitectura.md`, la bitácora, `CONEXION_23NOV.md`, `RVIZ_RWS.md` y el resto están en `hito3_4_control_real/01_docs/` (guias/, operacion/, bitacora/, desarrollo/) |

No se versionan (están en `.gitignore`): `GO_VEVAS.json`, `evidencia_c02*.json`, `sesiones/`, `datos/`, `conexion-rws-*.json`, `.venv/`.

### 12.3 Proyecto en la PC del laboratorio (`~/Descargas/ROBOTICA_01/proyecto_brazo`)

| Ruta | Para qué sirve |
|---|---|
| `preparar_ubuntu.sh`, `comun.sh` | Crea `.venv` de la raíz; variables comunes de los lanzadores |
| `00_codigo/hito3_4/` | Código (12.2), más `GO_VEVAS.json` y `evidencia_c02.json` locales |
| `02_conexion/diagnosticar.sh` | Diagnóstico de red, RWS y FK (solo lectura) |
| `02_conexion/abrir_solo_lectura.sh` | Consola sin envío |
| `05_seguridad_y_control/verificar_c02.sh` | Verificación presencial de C02 (mueve el robot) |
| `05_seguridad_y_control/abrir_control_real.sh` | Exige evidencia, valida o crea el GO y abre la consola con envío real |
| `07_documentacion/` | Copia de esta guía y del informe PDF (la deja `subir_24nov.sh`) |
| `08_datos_locales/sesiones/` | Registros de sesión (`eventos.jsonl`, `muestras.csv`, `resumen.json`, `planes/`) |
| `08_datos_locales/conexion/` | JSON de cada diagnóstico |
| `08_datos_locales/poses_guardadas.json` | Poses guardadas |

Fuera del repo, en la carpeta de entrega: `subir_24nov.sh`, `vevas_24nov.bundle` y `operacion/` (los dos scripts nuevos de `05_seguridad_y_control`).
