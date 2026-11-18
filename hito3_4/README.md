# Hito 3/4 (ECI-3/4): integración ROS 2 – ABB IRB 120 con cálculos y error en tiempo real

Grupo VEVAS · Robótica IMT-342 · UCB Tarija · Gestión II-2026

Esta carpeta tiene todo lo del Hito 3/4: la **consola VEVAS** (interfaz gráfica que calcula en vivo FK, Pieper con sus 8 ramas, Jacobiano y LSPB, muestra el error en tiempo real y mueve el robot), las herramientas de validación, el RAPID de EGM y las guías.

## Empezar en 1 minuto (simulación, sin robot)

```bash
cd ~/abb_ws/src/pfi-vevas-irb120/hito3_4
python3 -m pytest -q tests/                 # 21 pruebas, deben pasar todas
python3 consola_vevas.py --modo sim --abrir # abre http://127.0.0.1:8000
```

## Recorrido completo actualizado

Empezar por [Guía 0, desde la preparación hasta el video físico](docs/GUIA_0_RUTA_COMPLETA.md). La revisión incorpora [la referencia de Marcelo y su diagnóstico](docs/GUIA_9_REFERENCIA_Y_DIAGNOSTICO.md), capturas históricas con procedencia y un modo ROS explícito de lectura.

```bash
python3 consola_vevas.py --modo ros --solo-lectura --topico-estado /irb120/real/joint_states --abrir
```

Este comando presupone un puente RWS que publique ese tópico. El envío físico desde VEVAS sigue pendiente de comprobación en laboratorio por la ruta EGM. Los resultados locales de esta revisión están en [docs/datos/revision_2026-11-17](docs/datos/revision_2026-11-17/).

## Guías (leer en este orden)

| # | Guía | Para qué |
|---|---|---|
| 1 | [Lo que ya hicimos](docs/GUIA_1_LO_QUE_YA_HICIMOS.md) | Todo lo hecho desde el Hito 1, qué falló, qué falta y el siguiente paso |
| 2 | [Conectar al brazo](docs/GUIA_2_CONECTAR_AL_BRAZO.md) | Del cable al movimiento: red, RWS, validación con el controlador, EGM, driver |
| 3 | [Cinemática en vivo](docs/GUIA_3_CINEMATICA_EN_VIVO.md) | FK, Pieper, 8 ramas, selección, Jacobiano, singularidades, LSPB, con números |
| 4 | [Prueba junta por junta](docs/GUIA_4_PRUEBA_JUNTA_POR_JUNTA.md) | Protocolo: una junta, las seis, después la inversa |
| 5 | [La consola](docs/GUIA_5_CONSOLA.md) | Instalar, abrir, cada pantalla, teclado, modos |
| 6 | [Seguridad, GO y video](docs/GUIA_6_SEGURIDAD_GO_Y_VIDEO.md) | GO del día, roles, límites, guion del video |
| 7 | [Análisis de errores](docs/GUIA_7_ANALISIS_DE_ERRORES.md) | Informe automático, diagnóstico, errores ya corregidos, mejoras |
| 8 | [Extensiones](docs/GUIA_8_EXTENSIONES.md) | Más allá del hito y lo que pedía el plan original |
| — | [Presentación](docs/PRESENTACION.md) | Guion por integrante y banco de preguntas |
| — | [Arquitectura](docs/arquitectura.md) · [Manifiesto del modelo](docs/model_manifest.yaml) · [Bitácora](docs/bitacora.md) | Evidencia de Gate 0 y registro |

## Qué hay en cada carpeta

| Carpeta / archivo | Contenido |
|---|---|
| `consola_vevas.py` | Programa principal (modos `sim` y `ros`; `--real --go` para el robot real) |
| `config.json` | Umbrales, velocidades, nombres de juntas, convención, tópicos |
| `vevas/` | Núcleo propio: `robot_irb120` (D-H, FK), `pieper` (8 ramas), `jacobiano`, `trayectoria` (LSPB), `seguridad`, `consola`, `backend_sim`, `backend_ros`, `servidor`, `registro`, `secuencias` |
| `web/` | Interfaz (three.js, KaTeX y uPlot incluidos, no necesita internet) |
| `herramientas/validacion_offline.py` | Validación S0 contra el URDF de ROS-Industrial (FK, IK, Jacobiano, singularidad) |
| `herramientas/verificar_convencion_rws.py` | FK propia contra el controlador real, solo lectura (S1) |
| `herramientas/analizar_sesion.py` | Informe con gráficas y diagnóstico de cada sesión |
| `rapid/TRob1Main_VEVAS.mod` | RAPID de EGM que no mueve el robot al arrancar |
| `plantillas/GO_VEVAS_plantilla.json` | GO del día (checklist oficial de 11 ítems) |
| `docs/datos/` | Evidencia: validación offline, validación con el controlador, Gate 1 |
| `sesiones/` | Registros de cada uso de la consola (no se suben a git) |

## Estado (17/11/2026)

| Dominio de la rúbrica | Evidencia lista | Falta |
|---|---|---|
| 1. Sistema y compatibilidad (20 %) | Inventario, opciones del IRC5, arquitectura y matriz A/B, manifiesto | Guardar fotos y salidas de `curl` en `docs/datos/gate1/` |
| 2. Modelos y validación (25 %) | FK/IK/J propios, 21 pruebas, validación S0 contra URDF ROS-I (6e-16) | FK propia vs controlador (S1) con `verificar_convencion_rws.py` |
| 3. Integración ROS–ABB (25 %) | Consola probada en simulación; driver y paquete del IRB 120 en el Ubuntu | Probar con `use_fake_hardware`, después robot real (S2, S3) |
| 4. Seguridad y reproducibilidad (20 %) | GO, límites, abortos automáticos, registro de sesiones, análisis automático | Bitácora del día de prueba, sesiones reales analizadas |
| 5. Defensa (10 %) | Guion y banco de preguntas | Ensayar |
