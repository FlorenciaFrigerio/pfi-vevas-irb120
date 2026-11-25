# Hito 2 · Cinemática y dinámica

![VEVAS](../01_docs/banner-vevas.svg)

**Sigue el cálculo desde la pose deseada hasta la trayectoria y el torque.**

[Abrir notebook](notebooks/Hito2_VEVAS.ipynb) · [Documentación](01_docs/)

## Material del hito

Cinemática inversa de Pieper, Jacobiano, singularidades, perfiles LSPB y dinámica reducida.

[1. Cómo ejecutar](COMO_EJECUTAR.md) · [2. Informe](01_docs/01_informe/01_Informe_Hito2.pdf) · [3. Índice de documentos](01_docs/INDICE.md)

El método analítico calcula candidatos articulares y comprueba la pose con cinemática directa.

```mermaid
flowchart TD
 A[Pose objetivo] --> B[Pieper]
 B --> C[Validación y selección]
 C --> D[Trayectoria LSPB]
```

[Figura de trayectoria LSPB](01_docs/04_resultados/figuras/fig_lspb.pdf)

<details>
<summary>Resultados de cinemática y dinámica</summary>

La trayectoria LSPB sincroniza el movimiento de las seis juntas. El modelo dinámico reducido usa un perfil de referencia para calcular el torque de una junta.

$$\tau = I\ddot{q} + \tau_g(q)$$

- [Notebook del Hito 2](notebooks/Hito2_VEVAS.ipynb)
- [Resultados reproducibles](01_docs/04_resultados/resultados.json)
- [Figura de manipulabilidad](01_docs/04_resultados/figuras/fig_manipulabilidad.pdf)
- [Figura del torque 1R](01_docs/04_resultados/figuras/fig_torque_1r.pdf)

</details>
