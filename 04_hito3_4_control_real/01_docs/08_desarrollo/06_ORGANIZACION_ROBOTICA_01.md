# Entrega independiente ROBOTICA_01 · 23/11/2026

Se prepara `ROBOTICA_01/proyecto_brazo`, destinado a
`/home/huesero/Descargas/ROBOTICA_01/proyecto_brazo`. Es una entrega portable:
los lanzadores resuelven su ubicación sin depender de `vevas-publicacion`, ROS ni
carpetas de prototipos anteriores. No se ha escrito en la computadora del usuario.

El código activo se conserva una sola vez en `00_codigo/hito3_4`, manteniendo las
rutas de importación, configuración, mesa y mallas. Cada área numerada contiene su
README y, cuando corresponde, su lanzador. Cada módulo de código también recibe un
README de área. Los datos nuevos van a `08_datos_locales`, fuera del código.

Se incluyen contexto de este chat, estado, pendientes y prompts para el proyecto
ChatGPT «brazo». La herramienta disponible no permite mover/vincular conversaciones
a proyectos de ChatGPT. El contexto es un archivo para adjuntar; no se crea un Space
o Page sustituto ni se afirma que el chat se haya movido.

Los originales aportados y las fuentes públicas del docente pueden conservarse en
`09_historial` junto a un bundle Git de HEAD. Se omiten ZIP intermedios redundantes.
El código activo excluye venv, cachés, sesiones, GO completados y copia anidada. El
bundle mantiene el historial tal como fue versionado; puede contener venv y duplicados
históricos. No incluye configuración de autenticación Git ni keyring de la PC.

`02_herramientas/empaquetar_robotica01.py --help` documenta la generación. Se exigen
destinos nuevos para no sobrescribir entregas previas. El paquete contiene hashes
SHA256 y ORIGEN.json con commit, exclusiones y estado real. Los archivos se copian de
la revisión de trabajo versionada; no ejecutar la exportación con cambios pendientes
sin registrarlos y revisar la procedencia.

Los lanzadores nuevos ofrecen SIM y RWS solo lectura. El diagnóstico no cambia red,
calibración, motores ni RAPID. Se conserva el bloqueo físico C02 y las diferencias
entre evidencia simulada y física. Pruebas y entrega de esta reorganización se
registran en la bitácora y en `06_pruebas/VERIFICACION_ORGANIZACION.md` del paquete.
