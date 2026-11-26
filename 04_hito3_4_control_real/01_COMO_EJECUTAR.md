# Guía de ejecución · Hitos 3 y 4 (VEVAS Studio en Simulación, Pruebas y Laboratorio IRC5)

[Volver a Hitos 3 y 4](README.md) · [Guía técnica completa](01_docs/03_guias/01_GUIA_COMPLETA_HITO3_4.md) · [VEVAS Studio](02_vevas_studio/README.md)

**VEVAS Studio** puede ejecutarse en **modo Simulación (`SIM`)** en cualquier computadora con Python 3 y NumPy (Windows, Linux o macOS), o conectarse en **Ubuntu 24.04** al controlador **ABB IRC5** mediante Robot Web Services (**RWS**).

---

## 1. Arranque rápido en modo Simulación (`SIM`, sin robot físico ni ROS)

### En Windows (PowerShell o CMD)
```powershell
cd 04_hito3_4_control_real\02_vevas_studio
python -m pip install -r requirements.txt
python vevas.py --modo sim --puerto 9012 --abrir
```

### En Ubuntu / Linux
```bash
cd 04_hito3_4_control_real/02_vevas_studio
python3 -m venv .venv_interfaz
.venv_interfaz/bin/python -m pip install -r requirements.txt
.venv_interfaz/bin/python vevas.py --modo sim --puerto 9012 --abrir
```

Abre **<http://127.0.0.1:9012/>** si el navegador no se lanza automáticamente. Desde allí puedes:
1. Pulsar **ARMAR** en el panel de seguridad.
2. Elegir una figura (**Corazón**, **Cuadrado**, **Círculo**, **Triángulo** o **Saludo**) en el plano **De frente (`YZ`)**, tamaño `40 mm` y slider al `5 %`.
3. Pulsar **Calcular y verificar**, inspeccionar la vista 2D milimetrada y **Confirmar ejecución**.
4. Observar en tiempo real la superposición del TCP medido y la pestaña **Velocidades**.

---

## 2. Ejecutar las suites de pruebas automatizadas (`329` Python + `20` JavaScript)

```bash
cd 04_hito3_4_control_real/02_vevas_studio

# Suite Python completa (incluye cinemática, colisiones, suavidad C², MockIRC5 y C02)
python -m pytest 06_tests -q

# Suite JavaScript de interfaz web (proyección 2D, cuaterniones/Euler y velocímetro)
node --test 06_tests/*.mjs
```

---

## 3. Flujo seguro de operación presencial con el ABB IRC5 (`192.168.125.1`)

> [!CAUTION]
> Nunca omitas una compuerta: antes de enviar movimiento físico deben estar confirmados el diagnóstico de solo lectura, el archivo `GO_VEVAS.json` del día y la evidencia presencial `evidencia_c02.json` con paro de emergencia a mano.

```bash
cd 04_hito3_4_control_real/02_vevas_studio

# 1. Gate 1 (solo lectura): comprobar red, juntas y coincidencia FK con el FlexPendant
python3 herramientas.py conectar_robot --host 192.168.125.1

# 2. Emitir autorización GO diaria (zona despejada, sin herramienta peligrosa, tope 5 %)
python3 herramientas.py crear_go

# 3. Compuerta C02 (en modo AUTO): cargar VEVAS_RWS.mod y verificar velocidades <= 5 % ficha
python3 herramientas.py verificar_c02 --host 192.168.125.1

# 4. Abrir VEVAS Studio habilitado con RWS + C02 verificado
python3 vevas.py --modo rws --rws-host 192.168.125.1 --real --go GO_VEVAS.json --puerto 9012 --abrir
```

Al finalizar una sesión, genera el reporte automático con gráficas ejecutando:
```bash
python3 herramientas.py analizar_sesion --ultima
```
