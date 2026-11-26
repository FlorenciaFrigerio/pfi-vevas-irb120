<div align="center">

<img src="../../01_docs/06_figuras/hero_hito3_4.svg" alt="VEVAS Studio · Consola del IRB 120" width="100%">

<br>

<p>
  <a href="../01_COMO_EJECUTAR.md"><kbd>Arranque rápido</kbd></a>&nbsp;
  <a href="../README.md"><kbd>← Capítulo Hitos 3 y 4</kbd></a>&nbsp;
  <a href="../01_docs/03_guias/01_GUIA_COMPLETA_HITO3_4.md"><kbd>Manual completo</kbd></a>&nbsp;
  <a href="06_tests/README.md"><kbd>329 tests Python + 20 JS</kbd></a>&nbsp;
  <a href="../../README.md"><kbd>Portada principal</kbd></a>
</p>

</div>

![Divisor](../../01_docs/06_figuras/divisor_seccion.svg)

## Qué hace VEVAS Studio

**VEVAS Studio** es una aplicación modular en Python + Three.js/Canvas sin dependencias pesadas (`numpy` únicamente en el servidor) diseñada para operar el **ABB IRB 120** bajo el principio *calcular → revisar muestra por muestra → autorizar → medir en tiempo real*:

- **Visor 3D interactivo + vista 2D de figuras:** arrastre cartesiano/articular del fantasma, selección explícita de rama de Pieper, vista milimetrada previa y superposición en vivo del TCP medido.
- **Cinco figuras parametrizadas (`07_vevas/figuras.py`):** `círculo`, `corazón`, `cuadrado`, `triángulo` y `saludo` en arco, sobre planos **`YZ` («De frente»)**, `XY` («Desde arriba») o `XZ` («De costado»), con paradas exactas $v = a = 0$ en esquinas y cúspides.
- **Tres backends intercambiables:** `BackendSim` (dinámica $\tau = 40\text{ ms}$ a `250 Hz`), `BackendROS` (`/joint_states` + acción ROS 2) y `BackendRWS` (HTTP Digest hacia el **ABB IRC5** con servidor RAPID `VEVAS_RWS.mod` en streaming `z1`/`fine`).
- **Centro de velocidades y telemetría a `50 Hz`:** cálculo de velocidad medida por regresión móvil (`~0,3 s`), comparación frente al plan y al tope físico **C02 (`5 %` de catálogo)** y grabación automática en `muestras.csv`, `eventos.jsonl` y `resumen.json`.

---

## Mapa de las 8 áreas numeradas del paquete

| Área | Carpeta | Responsabilidad técnica |
| :---: | --- | --- |
| **01** | [`01_docs/`](01_docs/README.md) | Manifiesto cinemático y datos de validación offline S0 (`5 000` posturas, `casos_fk.csv`, `casos_ik.csv`). |
| **02** | [`02_herramientas/`](02_herramientas/README.md) | Diagnóstico RWS (`conectar_robot.py`), creación de `GO`, verificador presencial `verificar_c02.py` y `analizar_sesion.py`. |
| **03** | [`03_modelo/`](03_modelo/README.md) | URDF alineado con `tool0` ABB (`irb120_rws.urdf`) y geometría medida de la mesa (`190 mm`, 4 cajas). |
| **04** | [`04_plantillas/`](04_plantillas/README.md) | Plantilla canónica `GO_VEVAS_plantilla.json` con los 11 puntos del protocolo de seguridad. |
| **05** | [`05_rapid/`](05_rapid/README.md) | Módulo [`VEVAS_RWS.mod`](05_rapid/VEVAS_RWS.mod) (`4 707` caracteres `< 4 800`) con `MoveAbsJ \T:=t` y tope C02 por eje. |
| **06** | [`06_tests/`](06_tests/README.md) | `23` suites Python (`329` casos con `MockIRC5`) y `3` suites JavaScript (`20` casos con `node --test`). |
| **07** | [`07_vevas/`](07_vevas/README.md) | Núcleo: `robot_irb120.py`, `pieper.py`, `jacobiano.py`, `trayectoria.py`, `revision.py`, `c02.py`, `consola.py`, `servidor.py`. |
| **08** | [`08_web/`](08_web/README.md) | Interfaz web autónoma (HTML/CSS/JS, Three.js local, KaTeX local, mallas STL y lienzo 2D `vista_figura.js`). |

---

## Comandos esenciales

```bash
# Lanzar el simulador interactivo en http://127.0.0.1:9012
python vevas.py --modo sim --puerto 9012 --abrir

# Ejecutar las pruebas unitarias e integración
python -m pytest 06_tests -q
node --test 06_tests/*.mjs
```
