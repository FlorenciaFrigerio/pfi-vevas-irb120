# Script 11 - `capturar_resultados.py`: Captura reproducible de resultados

> **Nivel:** Complementario - entender escritura y reproducibilidad

## Para qué sirve
Ejecuta módulos, guarda salidas y recompone métricas en JSON con semilla y entorno.

**Entrada y salida:** Entrada: scripts del análisis. Salida: TXT y resultados.json en docs/hito2/resultados.

## Recorrido por bloques

### A. Ejecución
correr_modulos ejecuta módulos y captura stdout/stderr/código de salida.

**Código real** (`scripts/capturar_resultados.py`, líneas 39-54):

```python
def correr_modulos():
    """Ejecuta cada modulo y guarda su salida literal."""
    estado = {}
    for mod in MODULOS:
        print(f"  ejecutando {mod}.py ...", flush=True)
        try:
            r = subprocess.run([sys.executable, f"{mod}.py"], cwd=AQUI,
                               capture_output=True, text=True, timeout=600)
            texto = r.stdout + (("\n--- STDERR ---\n" + r.stderr)
                                if r.stderr.strip() else "")
            (SALIDA / f"salida_{mod}.txt").write_text(texto, encoding="utf-8")
            estado[mod] = {"codigo_salida": r.returncode,
                           "lineas": len(texto.splitlines())}
        except Exception as exc:
            estado[mod] = {"error": repr(exc)}
    return estado
```

**Cómo leerlo:** Permite rastrear fallos además de cifras.

### B. Medición
medir calcula coberturas, fronteras, Jacobiano, singularidad y caso dinámico.

**Código real** (`scripts/capturar_resultados.py`, líneas 57-93):

```python
def medir():
    """Recalcula las cifras principales del informe, en este proceso."""
    from fk_irb120 import forward_kinematics, JOINT_LIMITS
    from pieper import pieper, resolver, error_pose
    from jacobiano import jacobiano, validacion_manual_q0, jacobiano_numerico
    from lspb import LSPB
    from dinamica_1r import verificacion_analitica
    from singularidad import (direccion_perdida, direcciones_conservadas,
                              velocidad_requerida)

    m = {}

    # --- Cobertura regular
    rng = np.random.default_rng(SEMILLA)
    ep_max = eR_max = 0.0
    sin_sol = 0
    ramas = []
    for _ in range(2000):
        q = rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        T = forward_kinematics(q)
        cand = pieper(T)
        ramas.append(len(cand))
        for c in cand:
            ep, eR = error_pose(c, T)
            ep_max, eR_max = max(ep_max, ep), max(eR_max, eR)
        sel, _, _ = resolver(T, q)
        sin_sol += sel is None
    m["cobertura_regular"] = {
        "semilla": SEMILLA, "poses": 2000,
        "ramas_por_pose_media": float(np.mean(ramas)),
        "error_posicion_maximo_m": ep_max,
        "error_orientacion_maximo": eR_max,
        "poses_sin_solucion": int(sin_sol)}

    # --- Fronteras: la prueba dirigida que expuso el fallo de tolerancia
    rngf = np.random.default_rng(18026)
    rechazos = 0
```

**Cómo leerlo:** Semillas fijas hacen comparables los ensayos.

### C. Registro
El bloque principal agrega versiones, fecha, plataforma y métricas al JSON.

**Código real** (`scripts/capturar_resultados.py`, líneas 173-205):

```python
if __name__ == "__main__":
    SALIDA.mkdir(parents=True, exist_ok=True)
    print("\n   CAPTURA DE RESULTADOS DEL HITO 2\n")

    estado = correr_modulos()
    print("\n  midiendo cifras principales ...", flush=True)
    medidas = medir()

    doc = {
        "proyecto": "PFI Hito 2 - ABB IRB 120 - Grupo VEVAS",
        "fecha_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "entorno": {"python": platform.python_version(),
                    "numpy": np.__version__,
                    "plataforma": platform.platform()},
        "modulos_ejecutados": estado,
        "resultados": medidas,
    }
    (SALIDA / "resultados.json").write_text(
        json.dumps(doc, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"\n  escrito en {SALIDA}")
    r = medidas
    print(f"\n  ramas por pose           {r['cobertura_regular']['ramas_por_pose_media']:.2f}")
    print(f"  error posicional max     {r['cobertura_regular']['error_posicion_maximo_m']:.3e} m")
    print(f"  poses sin solucion       {r['cobertura_regular']['poses_sin_solucion']} de 2000")
    print(f"  rechazos en fronteras    {r['fronteras_dirigidas']['rechazos']} de 1800  (antes 193)")
    print(f"  giro innecesario de J6   {r['vueltas_j6']['movimiento_innecesario_rad']:.4f} rad  (antes 6.2832)")
    print(f"  jacobiano manual         {r['jacobiano']['validacion_manual_q0']:.3e}")
    print(f"  rango en la singularidad {r['singularidad']['rango']} de 6")
    print(f"  residuo incompatible     {r['singularidad']['residuo_tarea_incompatible']:.6f}")
    print(f"  tau(2.5 s)               {r['lspb_y_dinamica_1r']['tau_total_Nm']:.4f} N.m"
          f"   (referencia 4.4290, residuo {r['lspb_y_dinamica_1r']['residuo_Nm']:.1e})")
    print()
```

**Cómo leerlo:** Da contexto de procedencia a resultados.

## Cómo se relaciona
Ejecuta pieper.py, jacobiano.py, lspb.py, singularidad.py y dinamica_1r.py.

## Pruebas y evidencia
Los TXT y JSON versionados están en docs/hito2/resultados.

## Pregunta para practicar defensa
> ¿Qué aporta la semilla? ¿Qué datos de entorno se guardan?

## Límite o precaución
El script escribe/sobrescribe archivos de resultados; revisa el destino antes de ejecutarlo.

## Ficha de repaso
- Problema que resuelve:
- Parámetros de entrada:
- Salida principal:
- Función clave:
- Prueba que lo respalda:
- Lo que no demuestra:
