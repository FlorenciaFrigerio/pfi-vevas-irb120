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
- En desarrollo, no incluido: movimiento continuo con zonas para recorridos de muchos puntos (cambia el módulo RAPID y exige repetir verificar_c02).
