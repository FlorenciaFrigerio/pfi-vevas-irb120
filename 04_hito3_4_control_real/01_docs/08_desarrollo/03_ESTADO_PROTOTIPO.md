# VEVAS Studio — prototipo temporal, 19/11/2026

Se trabaja en una copia separada. No se modifica el repo de Florencia, no se publica, no se copia al Ubuntu del laboratorio. El diseño está sujeto a cambios.

Hecho:
- Tema claro, títulos destacados, formularios separados para una junta, pose y seis juntas.
- Geometría articulada ROS-Industrial IRB120, selección de eslabón y vista previa de referencia. Se conservan créditos y licencia en 08_web/modelo.
- Visualizador WebGL y alternativa Canvas para navegadores sin WebGL. Canvas reduce la cadencia de renderizado; no modifica el cálculo cinemático. No se verificó WebGL en este navegador porque está deshabilitado.
- Pestaña Gráficas: posición recibida vs referencia, error articular, error TCP en mm y grados separados, sigma mínimo, velocidad y aceleración LSPB planificadas. Filtros por junta, ventanas 20/60/120 s, pausa y CSV.
- Datos en vivo con procedencia y telemetría ausente identificada. La pose FK no es medición externa.
- Resultado de la tubería basado en el resumen del backend, sin marcar los pasos Pieper como realizados para un plan articular.
- Esta copia rechaza modo ROS/real por línea de comandos: únicamente simulación durante diseño.

Verificado:
- Sintaxis de los tres módulos JS.
- Alineación DH → marcos ROS en 100 posturas aleatorias: diferencia máxima 3.33e-16 en matrices.
- Navegador: carga, datos por SSE, cálculo de J1 +5°, habilitación y ejecución simulada. Cambio de q de ~0° a ~5°, referencia recibida y curvas de error activas.
- Pose [400,0,400] mm, Euler [0,180,0]°: ocho ramas existentes, una válida, residuos FK del orden de 1e-16; tabla de ramas visible.
- Pestaña Datos y filtros de posición por junta revisados en navegador.

Pendiente:
- Evaluar distribución con Florencia y resolución real de su pantalla; validar WebGL en su Ubuntu.
- Se verificaron los siete modos de gráfica en navegador y una descarga CSV válida con 23 columnas, fuente simulación y tiempos crecientes. El exportador se ajustó después a la ventana seleccionada; comprobar su duración final.
- Corregir C01–C04 de la auditoría antes de integrar movimiento físico: GO vacío, tope de velocidad por junta, carrera desarmado/envío, estado viejo antes del envío.
- Integrar y comprobar ROS/EGM con feedback fresco, resultado de acción y operador; no hay movimiento físico confirmado desde VEVAS.
- Ampliar validación FK a varias posturas, ensayos por junta y cartesianos, evidencia de hitos y registros físicos. No fabricar torques/corrientes/temperatura ni datos externos.
- Importación y análisis de sesiones históricas completas; hoy se exporta únicamente la ventana en memoria de hasta 120 s.

Inicio en este entorno: /workspace/work/venv/bin/python consola_vevas.py --modo sim --puerto 9012 --sesiones /tmp/vevas-ui-sesiones
La URL local pertenece al entorno de desarrollo de la nube; no supone instalación en la computadora del usuario.

Verificación final de la sesión:
- Selección de pieza 3D comprobada: clic en antebrazo seleccionó J4, sin enviar movimiento.
- Cambiar un valor invalida el plan anterior y deshabilita ejecución hasta recalcular; el plan previo tampoco se recupera automáticamente al recargar.
- El renderizado compatible se trasladó a un Web Worker con OffscreenCanvas para separar el dibujo de controles y pulso.
- Se observaron abortos por watchdog durante pruebas con inspección del navegador. No se modificó ni deshabilitó el watchdog.
- Prueba posterior sin inspección durante el movimiento: plan 6 completado, J1 +5°, error final máximo 0.00458°, error cartesiano por FK 0.01551 mm. Datos exclusivamente simulados. En ocho lecturas posteriores, edad del pulso entre 0.015 y 0.307 s. Pendiente medir latencia bajo interacción intensa y en Ubuntu.
- La pestaña de gráficas quedó abierta con una captura pausada de referencia vs estado simulado, ventana de 120 s.
