# Pruebas de VEVAS Studio

Desde `vevas_studio/`, instala las dependencias y ejecuta:

```bash
python -m pip install -r requirements.txt pytest
python -m pytest -q
node --test tests/*.mjs
```

Las pruebas se ejecutan offline con simulación y mocks. La [bitácora del 24/11](../../docs/bitacora/CAMBIOS_24NOV.md) registra las verificaciones con el equipo físico.
