# Guía 8. Más allá del hito: cómo extenderlo y hacerlo mucho mejor

Grupo VEVAS · Ideas ordenadas por valor para la nota y esfuerzo

Lo que pide la rúbrica termina en: modelos validados, lectura de estado, movimiento autorizado y bitácora. Todo lo de abajo es extra. Está ordenado de lo que más suma con menos esfuerzo a lo más ambicioso. Cada idea dice qué parte del código tocar.

## Nivel 1. Rápidas (una tarde cada una)

### 1.1 Movimiento en línea recta de punto a punto (interpolación cartesiana)

Hoy los planes van en el espacio de juntas (LSPB por junta): la punta hace una curva. Agregar un modo "lineal": dividir el segmento entre la pose actual y la pedida en N puntos (posición lineal, orientación con SLERP de cuaterniones, tema de la Unidad I), resolver Pieper en cada punto eligiendo la rama más cercana a la anterior, y aplicar LSPB al parámetro del camino.

Dónde: nueva función en `vevas/trayectoria.py` y un botón en la pestaña Objetivo. Se ve muy bien en el 3D (camino recto) y une las Unidades I y II.

### 1.2 Torque estimado en vivo

Con el modelo dinámico del Hito 2 (`dinamica_1r.py`), calcular τ(t) = M q̈ + C q̇ + g para la junta 2 durante cada trayectoria y graficarlo junto al error. Muestra que la dinámica también se puede calcular en tiempo real y qué parte es gravedad y qué parte es inercia.

Dónde: `vevas/consola.py` (`_armar_plan` agrega la curva de torque) y una pestaña más.

### 1.3 Grabar y repetir (teach and repeat)

Botón "guardar postura" mientras se mueve con las flechas; después "repetir secuencia" pasa por todas con LSPB. Es lo que hace un operador con el FlexPendant, pero con nuestros cálculos.

Dónde: lista de posturas en `Consola`, guardada en JSON en la sesión.

### 1.4 Repetibilidad medida

Ir 10 veces a la misma pose desde posturas distintas y reportar la dispersión del error final. ABB declara 0.01 mm de repetibilidad: comparar.

## Nivel 2. Medianas (dos o tres días)

### 2.1 Control con gamepad

Un control de PlayStation o Xbox conectado a la PC: la API Gamepad del navegador lee los joysticks; se mandan como jog cartesiano proporcional (más inclinación = más velocidad, con tope). Mantener un gatillo apretado como hombre muerto.

Dónde: `web/app.js` (leer `navigator.getGamepads()`), mismo `/api/jog`.

### 2.2 Zonas prohibidas (cajas de colisión)

Definir cajas en `config.json` (la mesa, una pared, el controlador) y revisar en `revisar_plan` que ningún punto del brazo entre. Dibujarlas en el 3D.

### 2.3 Verificar el modelo con más datos del controlador

RWS también da la pose calculada por el controlador para cualquier ángulo (`CalcPoseFromJoints`) sin mover el robot. Con eso se puede repetir la validación FK en cientos de posturas contra el propio controlador, no solo en las que se pueden poner con el FlexPendant. Ojo: es un POST de cálculo; confirmar con el docente que está permitido.

### 2.4 Comparación con RobotStudio

Si el laboratorio tiene RobotStudio, el mismo driver funciona contra un controlador virtual (la documentación de `abb_ros2` lo explica). Sirve para probar EGM sin tocar el robot y para mostrar las mismas trayectorias en dos simuladores distintos.

## Nivel 3. Ambiciosas (lo que pedía el plan original de los hitos 3 y 4)

### 3.1 Visión: tomar una pieza que ve una cámara

El plan original del Hito 3 era una cámara cenital: calibrarla (DLT con SVD propio, sin `findHomography`), detectar una pieza, pasar su posición a la base del robot con la cadena de transformaciones (ejercicio de la semana 3: T_0^C · T_C^O) y mandar la pose a la consola. La consola ya hace todo lo demás. Criterio del plan original: error de reproyección < 0.5 px y error físico < 1.5 mm medido con vernier.

### 3.2 Celda de clasificación

Pinza neumática (salida digital del IRC5), 5 piezas, ciclo < 8 s y parada segura si se corta la red (lo que pedía el Hito 4 original). Requiere autorización para usar E/S del controlador.

### 3.3 Artículo IEEE

Si el docente lo sigue pidiendo: 4 a 6 páginas a dos columnas con el método (modelo, validación S0–S3, consola), resultados (tablas de error, retardo) y discusión. Los números ya los generan `validacion_offline.py` y `analizar_sesion.py`.

## Lo que hace que la presentación valga más que el 100

1. **Mostrar la cadena de evidencia completa:** S0 (validación offline) → S1 (FK propia vs controlador) → S2 (robot real en la consola, solo lectura) → S3 (video con movimiento). Una diapositiva con esa escalera y un dato por escalón.
2. **Mostrar los errores corregidos** (Guía 7, sección 4): el docente valora el diagnóstico más que el "funcionó".
3. **Explicar con la consola, no con diapositivas:** pasar el mouse por las 8 ramas, apretar F para los marcos D-H, llevar el robot con las flechas cerca de q5 = 0 para que salte el aviso de singularidad.
4. **Datos propios:** retardo medido, error típico a cada velocidad, repetibilidad. Nadie más los va a tener.
5. **Cada integrante domina su parte y la de otro** (dominio 5 de la rúbrica): usar el banco de preguntas de `PRESENTACION.md`.


## Orden para una presentación más completa

1. Cerrar la evidencia física del pipeline principal y conservar el video y los registros.
2. Repetir tres movimientos con el mismo objetivo y comparar error final, tiempo y dispersión entre ensayos.
3. Mostrar un fallo reproducible en simulación, su log, una corrección comprobada y la repetición del caso.
4. Armar una tarea de tablero con transformación de celda a base, posiciones identificadas y TCP de herramienta medido.
5. Implementar y validar el camino cartesiano de la sección 1.1, midiendo desviación máxima respecto a la recta.
6. Agregar torque estimado con parámetros y alcance declarados. La dinámica del Hito 2 está disponible para análisis offline. Su visualización en vivo requiere implementación adicional.
7. Añadir visión con cámara cuando la tarea y los marcos estén validados. Medir error de calibración y tasa de aciertos sobre un conjunto de pruebas.

Para cada mejora declarar entrada, salida, criterio, datos y resultado. La elección de las siguientes extensiones depende de qué se logre demostrar de forma repetible con el tiempo disponible.
