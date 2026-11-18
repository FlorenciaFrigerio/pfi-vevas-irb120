# Guía 9. Referencia de Marcelo, conexión a VEVAS y diagnóstico

Revisión del 17/11/2026. El destino de los cambios es el repositorio de Florencia. La referencia consultada es Marcelo767n/robotica-IMT342, rama proyectoHito3y4, commit `194c0778d51754fc20f492d1a7d67c30d7f5c9be`.

## Qué aporta la referencia

| Evidencia | Qué demuestra en su contexto | Cómo aprovecharla |
|---|---|---|
| GET RWS y contraste con FlexPendant | Lectura de los ejes del IRC5 | Repetir el contraste y conservar fuente, hora y foto |
| RWS → ROS → TF del 03/11 | Estado físico publicado y recibido en ROS | Suscribir VEVAS al tópico real y verificar nombres/unidades |
| FK física del 04/11 | Comparación con tool0 en Base/wobj0 | Usar las muestras históricas para comprobar nuestra matemática |
| Sesión del 16/11 | 24 órdenes con llegada confirmada por RWS/RAPID | Tomar los fallos y su recuperación como antecedente |
| Sesión del 17/11 | Modo AUTO, puntero RAPID, llegada a cero y solución del enlace USB | Identificar condiciones equivalentes antes de repetir |

Las fuentes están fijadas al commit revisado para que un cambio posterior de la rama conserve la trazabilidad de esta guía. [Proyecto de referencia](https://github.com/Marcelo767n/robotica-IMT342/tree/194c0778d51754fc20f492d1a7d67c30d7f5c9be/2-2026/IMT342/equipo/proyecto).

## Estado observado y límites que deben ir a la defensa

El ensayo ROS/TF del 03/11 publicó 30 mensajes y conservó 28 filas sincronizadas, con las seis juntas estacionarias. Dejó pendiente la confirmación simultánea en el FlexPendant. [Registro ROS/TF](https://github.com/Marcelo767n/robotica-IMT342/blob/194c0778d51754fc20f492d1a7d67c30d7f5c9be/2-2026/IMT342/equipo/proyecto/docs/conexion_abb/evidencias/verificacion_rws_ros_tf_2026-11-03.json).

La sesión FK del 04/11 tiene 1.365 muestras, 1.202 comparables, 1.177 dentro del criterio, 25 fuera y 163 no evaluables. Las últimas 100 son una postura estacionaria repetida durante 19,8 segundos. J4–J6 tuvieron variación insuficiente para una campaña de seis ejes. Las discrepancias del tramo inicial necesitan investigación con sus datos originales. [Informe FK](https://github.com/Marcelo767n/robotica-IMT342/blob/194c0778d51754fc20f492d1a7d67c30d7f5c9be/2-2026/IMT342/equipo/proyecto/docs/conexion_abb/evidencias/fk_real_2026-11-04_224412Z_informe.md).

La bitácora del 16/11 registra 24 órdenes con llegada confirmada, ejes J6, J1, J4, J3 y J2, error máximo por orden de hasta 0,016 grados. J5 quedó pendiente tras un 403 de RWS. La continuación del 17/11 registra control desde RViz y llegada a cero en AUTO con error final de 0,0046 grados. El resumen de hitos más antiguo todavía decía que G3 estaba pendiente. Para describir ese avance corresponde citar la entrada más reciente y su ruta RWS/RAPID. [Bitácora física](https://github.com/Marcelo767n/robotica-IMT342/blob/194c0778d51754fc20f492d1a7d67c30d7f5c9be/2-2026/IMT342/equipo/proyecto/docs/bitacora/2026-11-16_primer_movimiento_real.md).

## Comparación offline con VEVAS

Se guardó una selección de dos posturas con sus URL de origen en [posturas_historicas.json](datos/referencia_marcelo/posturas_historicas.json). Las pruebas calculan `Robot().fk_tool0(q)` y comparan con posición y cuaternión de las capturas.

| Captura | Error de posición recalculado | Error de orientación recalculado | Contexto |
|---|---|---|---|
| RWS del 04/11 | aproximadamente 0,000075 mm | aproximadamente 0,000012 grados | Última muestra del registro |
| FlexPendant del 29/10 | aproximadamente 0,0051 mm | aproximadamente 0,00040 grados | Pantalla con redondeo |

Los criterios de las pruebas son exploratorios para estas capturas históricas. Se declaran por caso en el JSON. Los resultados corresponden al reprocesamiento offline. La siguiente validación física de VEVAS tiene que guardar su propia hora, versión y datos de adquisición.

## Cómo conectar los estados de la referencia

El paquete `irb120_conexion` del proyecto de Marcelo publica `/irb120/real/joint_states`, con nombres `joint_1` a `joint_6` y posiciones en radianes. Desde la raíz `proyecto/` de aquella copia, su arranque documentado es el siguiente.

```bash
bash ros2_ws/iniciar.sh validar_fk --host 192.168.125.1 --user "Default User" --ignore-joint-state
```

La opción `--ignore-joint-state` queda registrada por esa herramienta. Revisar con el operador la vigencia y validez física del estado que permite consultar. El usuario y la contraseña se manejan con el mecanismo del cliente. [Paquete y contrato de tópicos](https://github.com/Marcelo767n/robotica-IMT342/blob/194c0778d51754fc20f492d1a7d67c30d7f5c9be/2-2026/IMT342/equipo/proyecto/ros2_ws/src/irb120_conexion/README.md).

En la copia de Florencia, en otra terminal con ROS cargado, abrir la consola.

```bash
cd ~/abb_ws/src/pfi-vevas-irb120/hito3_4
python3 consola_vevas.py --modo ros --solo-lectura --topico-estado /irb120/real/joint_states --abrir
```

Este modo crea la suscripción de estados y deshabilita el armado, la acción de trayectorias y la publicación de jog. PARAR y el cierre conservan el carácter de lectura del backend. Se puede explorar FK, Jacobiano y planes, y registrar los estados recibidos.

La opción de lectura es explícita. `--modo ros` por sí solo conserva el modo de control usado para el ensayo con hardware simulado. Para lectura física usar el comando completo anterior. La suscripción tampoco demuestra por sí sola el origen IRC5 del publicador. Verificarlo con `ros2 topic info -v`, el proceso upstream y el contraste en el FlexPendant.

## Por qué el movimiento de la referencia necesita adaptación para VEVAS

| Contrato | VEVAS | Referencia física documentada |
|---|---|---|
| Objetivo | Trayectoria muestreada LSPB | Objetivo y protocolo propio de RWS/RAPID |
| Transporte de comando | Acción FollowJointTrajectory o tópico JointTrajectory → ros2_control → EGM | Escritura RWS → LabJointRws → MoveAbsJ |
| Resultado | Respuesta de acción y seguimiento medido | Código de llegada, lectura y comprobación de objetivo |
| Parada | Cancelación/hold según driver y controlador | Procedimiento y estado de RAPID del ensayo |

Cambiar el transporte requiere un backend que preserve estos contratos, incluidos tiempos, cancelación y confirmación de llegada. El primer complemento implementado es la recepción de estados reales en la interfaz de VEVAS. El cierre del movimiento de VEVAS sigue la ruta EGM de las guías 2 y 4.

## Fallos registrados y recuperación

| Síntoma | Comprobación | Corrección y repetición |
|---|---|---|
| `d` no encontrado | Revisar la línea escrita | Usar `cd` y pegar un comando por línea |
| Archivo de Descargas ausente | Ejecutar `pwd` y comprobar la ruta | Descargar en esa PC o usar la copia del repo |
| Red inaccesible | Ver `ip -br addr` y `ip route get 192.168.125.1` | Corregir la interfaz/IP de la Guía 2 y repetir ping/RWS |
| El enlace desaparece al tocar el hub | Revisar enlace USB y tarjeta Ethernet | La referencia recuperó conectando el adaptador directo a la laptop |
| VEVAS recibe seis ceros o no actualiza | Revisar tópico, publicador, `ROS_DOMAIN_ID`, nombres y timestamps | Apuntar al tópico real con `--topico-estado` y comparar con el FlexPendant |
| Controlador de trayectorias ausente | `ros2 control list_controllers` y `ros2 action list -t` | Revisar launch/configuración y activar el controlador en simulación |
| El puntero cambia a main en AUTO | Leer rutina y módulo activos | La referencia verificó por RWS el puntero a LabJointRws/ServidorPC antes de arrancar |
| 403 RWS después de parar RAPID | Consultar permiso y estado de ejecución | La referencia invalida su permiso RMMP y lo solicita otra vez antes del siguiente envío |
| Objetivo se queda sin enviar con ejes mayores a 100 grados | Revisar objetivo escrito y leído | La referencia redondea a tres decimales y comprueba la lectura con tolerancia declarada |
| Pausas entre tramos | Revisar versiones PC/RAPID y fragmentación | La referencia modificó el protocolo v3. En VEVAS medir primero el seguimiento de su LSPB |
| Error FK de orientación grande con posición correcta | Comprobar marcos 6 DH y tool0, además del TCP | Aplicar la transformación fija de tool0 y repetir comparación quieta |
| Una rama IK sale rechazada | Leer motivo de límites, residuo y sigma mínimo | Probar otra pose o rama válida y revisar el plan |
| pytest falla porque falta el módulo | Ver intérprete con `python3 -m pip --version` | Instalar las dependencias en el entorno usado para ejecutar |
| `git push` devuelve 403 | Revisar remoto y acceso de la conexión GitHub | Reconectar la app, incluir repo de Florencia y aceptar permisos de escritura solicitados |

Los fallos de RMMP, puntero RAPID y redondeo pertenecen a la ruta RWS/RAPID de la referencia. Conservar sus condiciones al investigar una reproducción. [Relato de fallos físicos](https://github.com/Marcelo767n/robotica-IMT342/blob/194c0778d51754fc20f492d1a7d67c30d7f5c9be/2-2026/IMT342/equipo/proyecto/docs/bitacora/2026-11-16_primer_movimiento_real.md).

## Cómo retroalimentar una falla

Registrar el comando exacto, hora, versión, estado previo, esperado y observado. Agregar log y carpeta de sesión. Formular una causa que pueda comprobarse. Aplicar una corrección pequeña, repetir primero la prueba que falló y después su entrada normal. Anotar qué resultado cambió y qué queda pendiente. Mantener la entrada fallida y agregar la repetición con otro identificador en la bitácora.

## Cómo subir al repo correcto

Desde la raíz del repo de Florencia, revisar qué archivos se van a publicar.

```bash
git remote -v
git status --short
git diff --check
git add hito3_4
git diff --cached --stat
git commit -m "feat(hito3_4): integra lectura ROS y guias de validacion"
git push -u origin HEAD
```

El remoto de destino debe ser `https://github.com/FlorenciaFrigerio/pfi-vevas-irb120.git`. Si el push recibe 403, el commit permanece local. Después de corregir la autorización, repetir el push. Verificar la rama y su commit desde GitHub antes de declarar publicada la entrega.
