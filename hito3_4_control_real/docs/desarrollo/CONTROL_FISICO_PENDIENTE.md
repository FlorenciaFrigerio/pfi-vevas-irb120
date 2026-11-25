# IRC5 y límite de velocidad por junta (C02)

> **Estado 24/11/2026 · vía implementada: tiempo acotado en el RAPID + medición presencial.**
> `rapid/VEVAS_RWS.mod` ejecuta `MoveAbsJ objetivo,vPc\T:=t,fine,tool0` con `t = max(0.5, kPico·|Δq_i|/vLim_i)`,
> vLim = 5 % de la ficha (12.5, 12.5, 12.5, 16, 16, 21 °/s) y kPico = 2.5, calculado por el propio controlador.
> `herramientas/verificar_c02.py` mueve cada junta ±δ, mide la velocidad con las posiciones RWS (ventana 0.1 s) y guarda
> `evidencia_c02.json` ligada a la huella del módulo, el modo y los topes. `BackendRWS` habilita el envío solo si esa
> evidencia pasó y coincide con el módulo local, el módulo cargado en el IRC5 y el modo actual (`vevas/c02.py`).
> Pruebas: `tests/test_c02.py` (IRC5 simulado que respeta el tiempo y otro que va 2.5× más rápido: este último no pasa).
> Límites de esta vía: la velocidad sale de posiciones RWS, no de una traza interna; una verificación acredita solo su modo;
> la suavidad física (paradas en cada tramo `fine`) sigue pendiente. SpeedLimAxis/LimitSpeed queda como alternativa oficial.


El módulo actual acepta un destino por RWS y ejecuta `MoveAbsJ objetivo,vPc,fine,tool0`. El IRC5 genera el movimiento y se detiene en cada destino. La PC revisa las velocidades de su plan y espera la llegada antes de enviar el siguiente objetivo. Los tiempos del plan y sus picos articulares describen la referencia. La velocidad ejecutada requiere evidencia del controlador. En esta versión `BackendRWS.velocidad_articular_verificada` vale `False` y el envío físico está bloqueado. [Módulo RAPID](../rapid/VEVAS_RWS.mod) · [Backend RWS](../vevas/backend_rws.py) · [Revisión de trayectoria](../vevas/revision.py)

## Qué puede limitar RobotWare 6

| Mecanismo | Magnitud que controla | Uso para C02 |
| --- | --- | --- |
| `speeddata [20,5,20,5]` | Traslación del TCP, reorientación del TCP, eje externo lineal y eje externo rotativo. | Los cuatro valores dejan sin especificar los seis máximos articulares. |
| `VelSet Override, Max` | `Override` escala la velocidad programada de las instrucciones posteriores, incluidos los movimientos temporizados. `Max` limita el TCP en mm/s. ABB excluye este máximo al especificar tiempo en la instrucción. | Un ajuste del 5 % por sí solo carece de equivalencia documentada con el 5 % de ficha de cada junta. |
| `MoveAbsJ` con `\T` | Solicita el tiempo total del tramo en segundos, en sustitución del dato de velocidad. | El desplazamiento dividido entre el tiempo da una media. El pico durante aceleración y frenado requiere una cota propia. |
| `SpeedLimAxis` | Define un límite en °/s para un eje rotativo de una unidad mecánica. La entrada de sistema `LimitSpeed` activa los valores. La salida de sistema `LimitSpeed` indica que el controlador alcanzó la reducción. | Permite aplicar un límite explícito por eje. Al retirar la entrada, el IRC5 vuelve a acelerar hacia la velocidad programada. |

Véanse *MoveAbsJ*, págs. 378–381, *SpeedLimAxis*, págs. 734–737, y *VelSet*, págs. 970–971, en el manual RAPID de RW6, revisión F. ABB identifica la revisión N como la publicada con RW6.12. [Referencia RAPID con estos números de página](https://library.e.abb.com/public/b227fcd260204c4dbeb8a58f8002fe64/Rapid_instructions.pdf) · [Historial de revisiones de ABB](https://library.e.abb.com/public/8a747513121e473992d2c43b690ee72e/3HAC050917%20TRM%20RAPID%20RW%206-en.pdf)

## Propuesta para evaluar en el controlador virtual RW6.12

Para el IRB 120-3/0.6, ABB publica 250, 250, 250, 320, 320 y 420 °/s. Al 5 % corresponden 12,5, 12,5, 12,5, 16, 16 y 21 °/s. El 5 % es el valor de la plantilla del GO. Para una sesión se debe usar el porcentaje autorizado y respetar cualquier límite inferior aplicable. [Ficha ABB, sección 1.8.3](https://library.e.abb.com/public/b11cfa9db47643579c66187c2c755952/3HAC035960%20PS%20IRB%20120-en.pdf) · [Plantilla GO](../GO_VEVAS_plantilla.json)

Este fragmento ilustra únicamente la configuración del caso de 5 % en `T_ROB1`. Requiere comprobar el nombre de la unidad mecánica. Está sin compilar y sin instalar.

```rapid
LOCAL PROC ConfigurarLimitesCincoPorCiento()
    SpeedLimAxis ROB_1, 1, 12.5;
    SpeedLimAxis ROB_1, 2, 12.5;
    SpeedLimAxis ROB_1, 3, 12.5;
    SpeedLimAxis ROB_1, 4, 16;
    SpeedLimAxis ROB_1, 5, 16;
    SpeedLimAxis ROB_1, 6, 21;
ENDPROC
```

La integración tiene que ejecutar las seis instrucciones con el robot en reposo, antes de declarar el servidor activo. Debe solicitar el límite por la entrada configurada para esa unidad y esperar una confirmación de activación nueva de la salida de sistema. Una salida que ya estaba en alto puede corresponder a límites anteriores. El código deberá gestionar ausencia de señal, error de configuración o confirmación vencida con el servidor inactivo. La señal debe permanecer aplicada durante todo el movimiento. Su pérdida requiere una respuesta del controlador probada también con un tramo en curso. Omitir el siguiente destino desde la PC deja continuar el tramo actual. La supervisión funcional de velocidad y los elementos de seguridad de la celda desempeñan funciones distintas. [ABB, parámetros del sistema, acción Limit Speed](https://library.abb.com/d/3HAC050948-001)

El IRC5 restablece los valores predeterminados con Reset RAPID, carga de programa o módulo, arranque desde el principio y ciertos cambios del puntero a main u otra rutina. El diseño debe invalidar la sesión anterior y reconfigurar los seis límites tras esos eventos. ABB excluye `SpeedLimAxis` de la rutina de evento POWER ON. La integración debe cubrir específicamente `OP_MAN_PROG` y/o `OP_AUTO` según los modos que el laboratorio vaya a utilizar. Una prueba de un modo acredita solo ese modo. [ABB, SpeedLimAxis, págs. 734–737](https://library.e.abb.com/public/b227fcd260204c4dbeb8a58f8002fe64/Rapid_instructions.pdf)

## Inventario y criterio de comprobación

1. **Lectura en el laboratorio.** Registrar variante del manipulador, versión exacta de RobotWare, tarea y unidad mecánica, modo operativo y versión del módulo cargado. En la configuración EIO identificar el nombre, unidad y origen de la entrada de sistema `LimitSpeed` y el nombre y destino de su salida de estado. Registrar ambos valores por separado. Si falta el mapeo, documentarlo como pendiente. Ni `vvRbActivo` ni una variable `PERS` confirman los seis valores vigentes.
2. **Prueba virtual.** Compilar con la versión del laboratorio. Comprobar activación en reposo, entrada ausente, retorno antiguo, pérdida de señal en movimiento, recarga de módulo, reinicio y reposición del puntero. Registrar la respuesta y comprobar que ninguna orden anterior se ejecuta tras recuperar la sesión. Conservar la confirmación del FlexPendant ya presente.
3. **Medición presencial.** En el alcance del GO vigente, recoger una traza del controlador de velocidades de ejes o de posiciones con reloj y periodo conocidos. Vincular configuración, identificación de la sesión, entrada y salida, modo, inicio y fin de la orden. Para cada eje informar límite, máximo absoluto observado y margen de medición. Si se derivan posiciones, registrar intervalos, resolución y pérdidas. Una diferencia finita estima la media durante el intervalo y puede ocultar picos entre muestras. Documentar como inconclusas las trazas que no permitan resolver las rampas. Una trayectoria con solo J1 activo caracteriza únicamente ese ensayo.
4. **Suavidad.** Para un tramo aislado, comparar las rampas y el frenado finales ejecutados con la referencia revisada. Para un trazo continuo de varios puntos, el IRC5 necesita conocer el siguiente tramo antes de llegar al anterior. Una futura versión puede precargar la secuencia, usar zonas intermedias y terminar con parada fina. Hay que comprobar la geometría redondeada, las distancias a obstáculos y el comportamiento al cancelar. Otra vía es EGM con referencias articulares temporizadas si la opción está disponible. Su ciclo nominal es de alrededor de 4 ms y `MaxSpeedDeviation` especifica el cambio de velocidad admitido. Esa opción requiere validar el seguimiento, la pérdida de paquetes y los máximos medidos. [ABB, manual de EGM para RW6](https://library.abb.com/d/3HAC073319-001)

Una futura habilitación tendrá que asociar la revisión presencial a la versión del módulo, configuración, modo y límite ensayados y comprobar el estado activo del controlador al ejecutar. El protocolo de producción actual no suministra esa evidencia.
