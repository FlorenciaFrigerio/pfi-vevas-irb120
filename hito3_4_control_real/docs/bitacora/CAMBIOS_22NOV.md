# Continuación del 22 de noviembre de 2026

Esta revisión se concentra en la simulación de VEVAS Studio. El [registro del 20/11](CAMBIOS_20NOV.md) conserva el contexto de RWS, el modelo de mesa y las correcciones de las compuertas.

## Cambios de simulación

| Pedido | Cambio | Comprobación |
|---|---|---|
| Movimientos más suaves | Perfil quíntico C² para SIM. Se conserva LSPB como perfil de la comparación académica. | Comparar curvas de posición, velocidad y aceleración y revisar sus uniones. |
| Arrastrar hacia arriba y abajo con el mouse | Asidero central del TCP para arrastrar desde el visor y flechas para trasladar sobre un eje. | Verificar la dirección y continuidad del arrastre con distintas posiciones de cámara. |
| Acceso a trayectorias frecuentes | «Poses predeterminadas» reúne círculo, corazón, cuadrado y saludo. | Probar cada opción en SIM con una postura inicial admisible. |
| Confirmación antes de ejecutar o volver a cero | Vista del recorrido y confirmación explícita. | Comprobar que cancelar conserva la postura y que editar el objetivo invalida el plan previo. |
| Ver el recorrido | Traza del TCP y puntos de paso. El suavizado visual interpola cuadros de presentación. | Distinguir la vista previa del rastro del simulador y verificar los vértices del cuadrado. |

La [prueba en Ubuntu](../operacion/PRUEBA_SIM_FIGURAS.md) detalla lo que se espera ver y cómo registrar una diferencia. Los recuentos y resultados de la suite deben corresponder a la copia final que se pruebe.

## Alcance físico y entrega

El perfil temporal y las figuras de esta revisión se prueban en SIM. El envío RWS real permanece bloqueado por C02, porque el controlador aún no acredita un límite de velocidad para cada junta. Siguen pendientes la verificación de herramienta/TCP y entorno, la comparación presencial con el FlexPendant y la ejecución de RViz en Ubuntu con lectura RWS. Se mantienen el GO manual y la confirmación del FlexPendant.

El push al remoto sigue pendiente por HTTP 403. El ZIP completo se puede probar desde una carpeta nueva, sin reemplazar la copia anterior. Los [prompts de continuación](../desarrollo/PROXIMOS_PROMPTS.md) cubren la validación local y la integración.
