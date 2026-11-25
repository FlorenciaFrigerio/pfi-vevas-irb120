# 02 · Hito 2 · Cinemática y dinámica

![VEVAS](../docs/banner-vevas.svg)

[![Abrir documentos del hito](https://img.shields.io/badge/02-Documentos-be185d)](docs/INDICE.md) [![Consultar ejecución](https://img.shields.io/badge/Guía-Ejecución-831843)](COMO_EJECUTAR.md)

Cinemática inversa de Pieper, Jacobiano, singularidades, perfiles LSPB y dinámica reducida.

[1. Cómo ejecutar](COMO_EJECUTAR.md) · [2. Informe](docs/informe/01_Informe_Hito2.pdf) · [3. Índice de documentos](docs/INDICE.md)

El método analítico calcula candidatos articulares y comprueba la pose con cinemática directa.

```mermaid
flowchart TD
 A[Pose objetivo] --> B[Pieper]
 B --> C[Validación y selección]
 C --> D[Trayectoria LSPB]
```

[Figura de trayectoria LSPB](docs/resultados/figuras/fig_lspb.pdf)

<details>
<summary>Resultados de cinemática y dinámica</summary>

La trayectoria LSPB sincroniza el movimiento de las seis juntas. El modelo dinámico reducido usa un perfil de referencia para calcular el torque de una junta.

$$\tau = I\ddot{q} + \tau_g(q)$$

- [Notebook del Hito 2](notebooks/Hito2_VEVAS.ipynb)
- [Resultados reproducibles](docs/resultados/resultados.json)
- [Figura de manipulabilidad](docs/resultados/figuras/fig_manipulabilidad.pdf)
- [Figura del torque 1R](docs/resultados/figuras/fig_torque_1r.pdf)

</details>
