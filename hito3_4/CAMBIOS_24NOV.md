# Cambios del 24/11/2026 · cierre de C02 por tiempo acotado + medición

| Archivo | Cambio |
|---|---|
| `rapid/VEVAS_RWS.mod` | `MoveAbsJ objetivo,vPc\T:=t,fine,tool0`; `t = max(0.5, 2.5·|Δq_i|/vLim_i)`, vLim = 12.5/12.5/12.5/16/16/21 °/s (5 % de la ficha); `PERS vvRbTiempo`. 4123 caracteres. |
| `vevas/c02.py` | Nuevo: evalúa `evidencia_c02.json` (versión, ok, huella del módulo, topes, seis juntas, modo, módulo en el IRC5) y calcula velocidades por ventana. |
| `herramientas/verificar_c02.py` | Nuevo: carga el módulo, arranca el servidor, mueve cada junta ±δ, mide con lecturas RWS y guarda la evidencia. |
| `vevas/rws_cliente.py` | `limites_c02()` lee vLim/kPico del `.mod`; `tiempo_movimiento()` replica el cálculo del RAPID. |
| `vevas/backend_rws.py` | `velocidad_articular_verificada` sale de la evidencia; comprobación de modo y módulo del IRC5 antes de cada envío; estimación de duración con el tiempo del RAPID; `preparar_rapid` no reinstala un servidor verificado activo. |
| `vevas/consola.py` | Rechaza el envío si el GO autoriza menos del 5 % (el RAPID limita al 5 %). |
| `web/app.js` | El aviso de C02 muestra el motivo concreto. |
| `tests/mock_rws.py`, `tests/test_c02.py` | IRC5 simulado con perfil por tiempo (y uno que no respeta el tiempo); 8 pruebas. |

Resultado inicial: 242 pruebas Python y 6 JS. C02 se verificó después en el robot real (AUTO, 24/11; ver docs/bitacora.md).
Siguiente paso presencial: `bash 05_seguridad_y_control/verificar_c02.sh` (AUTO, «Iniciar» en el FlexPendant, paro a mano).

## Más tarde el 24/11

- Envío por RWS en tramos colineales de hasta 40° (antes 10°): un movimiento de una junta o «Volver a cero» sale en un solo movimiento, sin pausas intermedias.
- «Volver a cero» con RWS y C02 verificado: la llegada a q = 0 (singular, q5 = 0) es aviso y no rechazo.
- Revisión: salir de una singularidad de muñeca con σ creciente es aviso (C02).
- RWS: reintento ante HTTP 500 «Error registering callback»; los motores solo se apagan si hay que escribir el módulo; si RWS niega MOTORS ON (403) se espera al operador 60 s.
- «Preparar J5 a +30°» y las figuras predeterminadas (cuadrado, círculo, saludo, corazón y el nuevo triángulo) habilitadas con el robot real por RWS con C02 verificado. Tamaño máximo 80 mm en real; si J5 ≈ 0 piden preparar J5 primero.
- Resultado: 265 pruebas Python y 10 JS pasan.

## Tarde del 24/11

- Movimiento continuo: los recorridos de muchos puntos se mandan en streaming. RAPID usa `MoveAbsJ \T:=t, z1` en los intermedios y acusa (código 3) apenas encola, así la PC manda el siguiente mientras el robot se mueve. `fine` solo en el último punto y en las paradas del plan (esquinas del cuadrado y del triángulo, punta del corazón). Piso de tiempo 0,2 s con zona y 0,5 s con fine; la ley C02 no cambia. Módulo: 4707 caracteres.
- C02 versión 2: verificar_c02 agrega una prueba de streaming (J6 ±δ en 8 puntos) y la evidencia la exige. **Hay que repetir verificar_c02.sh** antes de mover.
- Poses guardadas: guardar la postura actual con nombre, listar, renombrar, borrar (con confirmación) e «Ir a pose» con revisión y todas las compuertas. Se guardan en `<--datos>/poses_guardadas.json` (escritura atómica).
- Vista de puntos: dibujo 2D en mm de la figura antes de calcular (inicio, fin, paradas, orden) y, durante la ejecución, los puntos medidos del TCP encima con el error máximo y medio.
- Resultado: 321 pruebas Python y 16 JS pasan.
- GO más corto: `crear_go.py` hace solo dos preguntas (confirmación de seguridad en una línea y tope de velocidad, Enter = 5 %). Los nombres de operador, supervisor y alcance se completan como Grupo VEVAS. La validación del GO no cambió y vale todo el día. 322 pruebas Python.
- Figuras con J5 ≈ 0: en vez de rechazar («no hay rama de Pieper»), el plan agrega una aproximación articular J5 → +30° con parada y dibuja desde ahí, en un solo plan revisado. El saludo no la necesita. 329 pruebas Python y 16 JS.
- Figuras de frente por defecto: el plano base pasa a YZ (vertical, mirando al frente del brazo), con opciones «De frente», «Desde arriba» y «De costado». Botón «De frente» en el 3D; al calcular una figura de frente la cámara se ubica delante del robot.
- Saludo de frente: deja de ser un movimiento de muñeca y pasa a ser un vaivén en arco (∩) de lado a lado en el plano elegido (de frente por defecto), resuelto con Pieper; para un instante en cada extremo.
