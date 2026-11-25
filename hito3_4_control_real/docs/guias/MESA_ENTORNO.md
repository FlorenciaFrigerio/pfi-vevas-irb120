# Bastidor abierto del IRB 120

El perfil en [mesa_medidas.json](../../vevas_studio/modelo/mesa_medidas.json) recoge las cotas confirmadas del marco azul. Las distancias horizontales se toman entre las caras exteriores del bastidor y los bordes de la huella de montaje del brazo.

| Cota | Medida |
| --- | ---: |
| Largo × ancho exterior del marco | 80 × 80 cm |
| Sección del perfil del marco superior | 5 × 5 cm |
| Del lado opuesto a los conectores al borde de la huella | 44,5 cm |
| Del lado de los conectores al borde de la huella | 14 cm |
| De cada lateral al borde correspondiente de la huella | 29,5 cm |
| Del plano de los cuatro tornillos a la cara superior del marco | 19 cm |
| Del plano de los cuatro tornillos a la parte superior del cilindro de base | 18 cm |

Las dos alturas se miden desde el mismo plano. La parte superior del marco queda 1 cm por encima de la parte superior del cilindro. La huella deducida de las distancias horizontales mide 21,5 cm en X y 21 cm en Y.

El modelo toma Z = 0 en el plano de los cuatro tornillos y centra XY en la huella de montaje. +X apunta al lado opuesto a los conectores, hacia la cota de 44,5 cm. Los límites exteriores del aro son X de −247,5 a 552,5 mm, Y de −400 a 400 mm y Z de 140 a 190 mm.

## Representación y revisión

`vevas.mesa.generar_mesa()` publica las cuatro cajas del aro en `cajas` con sus esquinas `min_mm` y `max_mm` expresadas en el marco de la base. El centro queda abierto a la altura del aro. La visualización puede usar esas mismas cajas. Los dos apoyos inferiores de `geometria_visual` ilustran el montaje y tienen la etiqueta «no medido». Sus secciones y posiciones quedan pendientes de medición. La cota inferior configurada en la revisión es Z = 0 en el plano de montaje.

Quedan por medir la estructura por debajo del plano de montaje, los objetos y obstáculos del entorno, y el TCP y volumen de la herramienta negra visible en la brida. La confirmación del perfil cubre las cotas del marco superior. La comprobación con cajas y cápsulas aproxima los volúmenes y no certifica la seguridad física.

## Historial de las cotas

Las cotas preliminares de 39 cm y 37 cm para la altura del marco quedaron sustituidas al aclarar que los 18 y 19 cm del esquema parten del mismo plano. Las últimas cotas horizontales reemplazaron las lecturas preliminares de 44 cm al frente, 9 cm atrás y 24 cm por lado.
