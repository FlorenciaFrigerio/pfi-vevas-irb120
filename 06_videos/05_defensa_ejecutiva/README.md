# Video integrador de defensa · Grupo VEVAS

[Volver a 06 · Videos](../README.md) · [Abrir diapositivas de defensa](../../04_hito3_4_control_real/01_docs/02_presentacion/02_Defensa_Grupo_VEVAS.pdf)

## Archivo previsto

- `2026-11_defensa_ejecutiva_grupo_vevas.mp4` (duración objetivo: **3 a 5 minutos**, `1920×1080`, `30 fps`).

## Escaleta recomendada minuto a minuto

| Tramo | Tiempo | Contenido visual | Mensaje clave |
| --- | --- | --- | --- |
| **1. Gemelo digital (Hito 1)** | `0:00 – 0:45` | Visor 3D y RViz lado a lado + resultado de `validate_tf.py` | El URDF generado desde D-H coincide con la cinemática directa en `2,459 × 10⁻¹⁰ m` sobre `501` lecturas sincronizadas. |
| **2. Cinemática y dinámica (Hito 2)** | `0:45 – 1:45` | Una pose seguida de principio a fin: 8 ramas de Pieper, postura elegida `q*`, LSPB (`0,7746 s`) y recuadro 1R (`4,4298 N·m`) | La cinemática inversa analítica resuelve sin iterar, filtra fronteras sin falsos rechazos y sincroniza las seis juntas. |
| **3. Arquitectura y seguridad C02 (Hito 3/4)** | `1:45 – 2:45` | Consola VEVAS Studio, compuerta GO y gráfica de topes `MoveAbsJ \T:=t` en RAPID | El controlador IRC5 impone físicamente el tope del `5 %` por junta (`5,2–8,9 °/s` medidos frente a `12,5–21 °/s`). |
| **4. Evidencia de sesión y cierre** | `2:45 – 4:00` | Figura solicitada + trayectoria planificada + muestras medidas del TCP y toma del ABB IRB 120 | Toda ejecución conserva su plan, su CSV fechado y la comparación directa entre lo pedido y lo medido. |
