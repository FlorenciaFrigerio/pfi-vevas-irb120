![Hitos 3 y 4 · Del modelo al controlador ABB](../01_docs/06_figuras/03_control.png)

# Hitos 3 y 4 · Del modelo al controlador ABB

[Cómo ejecutar](01_COMO_EJECUTAR.md) · [Índice del hito](01_docs/01_INDICE.md) · [Portada del proyecto](../README.md)

VEVAS Studio conecta el cálculo de trayectorias con una interfaz de supervisión. Durante la simulación se puede seguir el plan. Los registros de las sesiones con el IRC5 permiten comparar lo previsto con las lecturas del controlador.

## 1. Material de entrega

| Material | Para qué sirve |
| --- | --- |
| [Informe IEEE · PDF](01_docs/01_informe/01_Informe_Hito3_4.pdf) | Método, arquitectura, resultados y conclusiones. |
| [Defensa del grupo · PDF](01_docs/02_presentacion/02_Defensa_Grupo_VEVAS.pdf) | Diapositivas de la exposición. |
| [Guía aportada el 24/11](01_docs/03_guias/02_Guia_aportada_24NOV.md) | Versión compartida para la entrega. |
| [Guía técnica completa](01_docs/03_guias/01_GUIA_COMPLETA_HITO3_4.md) | Procedimientos y explicación ampliada del sistema. |

## 2. Del plan a la revisión

~~~mermaid
flowchart TD
 A["Pose o figura"] --> B["Cálculo de trayectoria"]
 B --> C["Revisión de límites y condiciones"]
 C --> D["Simulación"]
 C --> E["Sesión documentada por RWS"]
 D --> F["Estado y trayectoria"]
 E --> G["Muestras del IRC5"]
 F --> H["Comparación con el plan"]
 G --> H
~~~

La pose, el marco de referencia y la unidad tienen que coincidir antes de comparar dos puntos. En las figuras, el plano elegido cambia la interpretación del dibujo proyectado. Para rastrear una discrepancia se consulta el plan, el registro de muestras y la configuración de la sesión.

## 3. Evidencia C02

![Figura C02 del informe IEEE del Hito 3/4](../01_docs/06_figuras/05_velocidades_c02.png)

La figura proviene del informe de la entrega. Los rangos y topes por grupo de juntas permiten interpretar por qué el tiempo observado puede diferir del tiempo planificado. La bitácora identifica la versión de la prueba y el contexto de las mediciones.

[Leer la bitácora del 24/11](01_docs/05_bitacora/05_CAMBIOS_24NOV.md) · [Ver la figura original](01_docs/01_informe/01_informe_ieee/01_img/fig_c02.pdf)

## 4. Entrar al software

[Abrir VEVAS Studio](02_vevas_studio/README.md) · [Consultar resultados offline](02_vevas_studio/01_docs/01_datos/01_validacion_offline/01_resumen.md)

## 5. Arquitectura de la aplicación

![Arquitectura reproducida del informe IEEE](../01_docs/06_figuras/09_arquitectura.png)

La interfaz recibe los objetivos y dibuja los resultados. La consola organiza la revisión y el estado de la sesión. El backend conecta esa lógica con el simulador o con el controlador. Esta separación ayuda a localizar una discrepancia: una figura mal orientada puede venir de la selección del plano, mientras que una diferencia de duración requiere comparar el plan temporal con las muestras recibidas.

## 6. Cómo leer una sesión del 24/11

| Pregunta | Evidencia que corresponde |
| --- | --- |
| ¿Coinciden los ángulos de las seis juntas? | Lecturas RWS y comparación con el FlexPendant |
| ¿Se comparó el mismo punto y la misma base? | Identificación de herramienta, sistema de referencia y convención articular |
| ¿Qué velocidad se pidió y cuál se observó? | Perfil del plan y muestras fechadas de posición articular |
| ¿Qué versión se verificó? | Fecha de la bitácora, revisión del módulo y alcance de C02 |

La entrada de lectura presencial registra la comparación con **tool0**, **wobj0** y el marco **Base**. El diagnóstico anotó una diferencia de posición de **0,0001 mm** para esa comparación. Ese valor corresponde a las lecturas comparadas en esa sesión. Para estudiar el seguimiento durante una trayectoria se necesitan las muestras de la ejecución.

## 7. Orden de lectura para la defensa

1. Abre las diapositivas y ubica la arquitectura.
2. Usa el informe para justificar cómo se obtiene el límite por junta y cómo se interpreta la figura C02.
3. Consulta la bitácora para distinguir las pruebas offline, la lectura presencial y las revisiones posteriores.
4. Abre la guía de entrega para relacionar cada resultado con la pantalla o el procedimiento que lo produjo.
