# Integración y conexión · 23/11/2026

Se integra la revisión de SIM del [22/11](CAMBIOS_22NOV.md), el diagnóstico de conexión y la guía de laboratorio. El paquete `VEVAS_SIM_23NOV.zip` se extrae en `vevas_sim_23nov`. No se modificó el IRC5 ni la copia de Ubuntu desde el entorno de desarrollo. La publicación GitHub sigue pendiente por los 403 registrados el 20/11.

## Implementado

- Perfil suave quíntico C²: posición, velocidad y aceleración continuas. LSPB académico sigue disponible. No se afirma límite de jerk certificado.
- Círculo, corazón y cuadrado con Pieper continuo; saludo articular, sin atribuirle una IK que no utiliza. Las figuras nuevas son de SIM. Cuadrado se detiene en sus esquinas y corazón en su cúspide. La revisión usa la trayectoria completa y rechaza errores, colisiones y singularidades.
- Arrastre con mouse en Y, Z y plano vertical-lateral, confirmación visible en ventanas pequeñas, recorrido nominal punteado, puntos y traza del estado. La interpolación visual usa un buffer de aproximadamente 85 ms; las tablas y errores usan datos recibidos.
- La traza también funciona en lectura RWS: es FK de las juntas recibidas, no metrología externa del TCP. Se descartan muestras vencidas y se limpia al cambiar la fuente.
- `herramientas/conectar_robot.py` detecta ruta/IP de Ubuntu, consulta RWS y compara FK. No modifica la red, instala RAPID ni solicita movimiento. Puede guardar un JSON de diagnóstico sin credenciales.
- La comparación temporal de seguimiento usa el instante local del estado del backend. Antes se comparaban muestras antiguas con la referencia de ahora, generando falsos excesos con retardo de muestreo. El umbral de 3° y los watchdogs quedan intactos. En ROS el sello local de recepción no resuelve una latencia interna desconocida del sensor. RWS por llegada sigue sin inventar un seguimiento temporal LSPB.
- Los planes ya rechazados no pueden ejecutar un prefijo válido de una figura fallida. El cierre de la consola espera al hilo de registro para evitar escrituras después de cerrarlo.

## Verificado

La primera corrida de la integración obtuvo 217 aprobadas y 2 fallos: seguimiento sintético con referencias de instantes distintos y una diferencia en el texto del rechazo. Se corrigieron ambos y se añadieron regresiones: una muestra retrasada válida no crea error ficticio, un error real de 5° sigue abortando, y una muestra vencida sigue abortando por watchdog.

La suite final obtuvo **232 pruebas Python aprobadas**. Pasaron **6 pruebas JavaScript**, sintaxis de seis módulos JS, cuatro scripts Bash y revisión de formato Git. La conexión se ensayó con red y controlador falsos; no hubo un IRC5 físico ni ROS/RViz ejecutándose aquí.

Chromium con visor compatible: las cuatro figuras se calcularon; cuadrado y saludo completaron en SIM. Cero permitió revisión y cancelación sin enviar. Confirmación accesible en cinco resoluciones. El arrastre cambió Y/Z y el plano libre. RWS de lectura con juntas sintéticas acumuló 11 puntos de traza, mostró C02 y bloqueó envío/figuras. Las capturas están identificadas como datos sintéticos en la evidencia.

## Laboratorio

Seguir [CONEXION_23NOV.md](../06_operacion/CONEXION_23NOV.md). No recalibrar si el FlexPendant confirma calibración y contadores válidos. Si hay avisos, corresponde revisar el procedimiento ABB de esa unidad antes del movimiento.

El envío físico sigue bloqueado por C02. Se documentó una solución candidata con `SpeedLimAxis` y señales de sistema `LimitSpeed`, que aún necesita compilación y verificación en el IRC5 RW6.12. Un porcentaje TCP o una variable booleana no sustituyen esa comprobación. La propuesta está en [CONTROL_FISICO_PENDIENTE.md](../08_desarrollo/CONTROL_FISICO_PENDIENTE.md). Siguen pendientes herramienta/TCP, obstáculos y la validación presencial de FK/RViz, informe y video.

Evidencia de software en `docs/datos/pruebas_23nov/` (local, no versionada) · [Guía SIM](../06_operacion/PRUEBA_SIM_FIGURAS.md)
