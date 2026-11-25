# Pruebas · VEVAS Studio

Pruebas offline de la aplicación y la interfaz.

[Consola y entregables](../README.md) · [Índice del hito](../../01_docs/01_INDICE.md)

## Archivos de esta área

1. [mock_rws.py](mock_rws.py)
2. [test_api_revision.py](test_api_revision.py)
3. [test_backend_suave.py](test_backend_suave.py)
4. [test_c02.py](test_c02.py)
5. [test_calculos.mjs](test_calculos.mjs)
6. [test_cinematica.py](test_cinematica.py)
7. [test_conectar_robot.py](test_conectar_robot.py)
8. [test_config_mesa.py](test_config_mesa.py)
9. [test_consola_figuras.py](test_consola_figuras.py)
10. [test_consola_poses.py](test_consola_poses.py)
11. [test_control_rws.py](test_control_rws.py)
12. [test_crear_go.py](test_crear_go.py)
13. [test_figuras.py](test_figuras.py)
14. [test_ik_vivo.py](test_ik_vivo.py)
15. [test_lanzador.py](test_lanzador.py)
16. [test_mesa.py](test_mesa.py)
17. [test_orientacion.mjs](test_orientacion.mjs)
18. [test_referencia_sincronizada.py](test_referencia_sincronizada.py)
19. [test_reloj_simulador.py](test_reloj_simulador.py)
20. [test_revision.py](test_revision.py)
21. [test_rviz_rws.py](test_rviz_rws.py)
22. [test_rws.py](test_rws.py)
23. [test_trayectorias_suaves.py](test_trayectorias_suaves.py)
24. [test_velocidad.mjs](test_velocidad.mjs)
25. [test_velocidad.py](test_velocidad.py)
26. [test_verificacion_pendant.py](test_verificacion_pendant.py)

## Referencia del área

Desde `02_vevas_studio/`, instala las dependencias y ejecuta:

```bash
python -m pip install -r requirements.txt pytest
python -m pytest -q
node --test 06_tests/*.mjs
```

Las pruebas se ejecutan offline con simulación y mocks. La [bitácora del 24/11](../../01_docs/05_bitacora/05_CAMBIOS_24NOV.md) registra las verificaciones con el equipo físico.
