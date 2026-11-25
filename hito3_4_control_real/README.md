# 03 · Hitos 3 y 4 · Control del robot

![VEVAS](../docs/banner-vevas.svg)

[![Abrir documentos del hito](https://img.shields.io/badge/03-Documentos-be185d)](docs/INDICE.md) [![Consultar ejecución](https://img.shields.io/badge/Guía-Ejecución-831843)](COMO_EJECUTAR.md)

VEVAS Studio integra la simulación, la cinemática y la supervisión del ABB IRB 120 mediante RWS.

[1. Cómo ejecutar](COMO_EJECUTAR.md) · [2. Informe](docs/informe/01_Informe_Hito3_4.pdf) · [3. Índice de documentos](docs/INDICE.md)

La bitácora del 24 de noviembre registra la ejecución y los cambios de VEVAS Studio.

[Bitácora del 24/11](docs/bitacora/CAMBIOS_24NOV.md) · [Figura C02](docs/informe/informe_ieee/img/fig_c02.pdf)

```mermaid
flowchart TD
    A["Interfaz VEVAS Studio"] --> B["Plan y revisión"]
    B --> C["Simulador"]
    B --> D["Robot real · RWS"]
    C --> E["Estado y mediciones"]
    D --> E
```

<details>
<summary>Evidencia y documentación del 24/11</summary>

| Cambio | Documento |
| --- | --- |
| C02, streaming y comprobaciones registradas | [Bitácora del 24/11](docs/bitacora/CAMBIOS_24NOV.md) |
| Poses guardadas, figuras y medición de velocidad | [Guía completa](docs/guias/01_GUIA_COMPLETA_HITO3_4.md) |
| Diapositivas de defensa | [Presentación oficial](docs/presentacion/02_Defensa_Grupo_VEVAS.pdf) |
| Guía aportada del 24/11 | [Guía en Markdown](docs/guias/02_Guia_aportada_24NOV.md) |
| Arquitectura y figuras de validación | [Informe IEEE](docs/informe/informe_ieee/informe_hito3_4_ieee.pdf) |
| Validación reproducible y gráficas offline | [Resultados](docs/resultados/README.md) |
| Problemas de conexión y ejecución | [Solución de problemas](docs/solucion_de_problemas/README.md) |

</details>
