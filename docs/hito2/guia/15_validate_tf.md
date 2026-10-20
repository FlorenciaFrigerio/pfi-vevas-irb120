# Script 15 - `validate_tf.py`: Validación de TF contra cinemática analítica

> **Nivel:** Hito 1 - prueba de integración

## Para qué sirve
Publica configuraciones, espera TF sincronizado y compara la pose recibida con FK.

**Entrada y salida:** Entrada: ROS activo y número de muestras. Salida: errores máximos y aprobado/no aprobado.

## Recorrido por bloques

### A. Conversión y marcas
quat_to_matrix convierte TF; stamp_a_ns permite comparar tiempos.

**Código real** (`scripts/validate_tf.py`, líneas 58-72):

```python
def quat_to_matrix(x, y, z, w):
    """
    Cuaternion unitario -> matriz de rotacion.

    Formula de Euler-Rodrigues expandida. Se implementa a mano en vez de
    usar una libreria porque forma parte de lo que hay que poder
    defender. Se normaliza antes por seguridad numerica.
    """
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])
```

**Cómo leerlo:** TF transporta orientación como cuaternión.

### B. Publicación y sincronía
Validador.publicar envía JointState; leer_tf_sincronizado espera una transformación con marca suficiente.

**Código real** (`scripts/validate_tf.py`, líneas 115-140):

```python
    def leer_tf_sincronizado(self, t_pub_ns, timeout=2.0):
        """
        Devuelve (p, R) de base_link -> tool0 correspondiente a la
        configuracion publicada en t_pub_ns.

        Relee hasta que la marca de tiempo de la transformada alcanza a
        la del JointState enviado. Asi se garantiza que la pose leida no
        es la de la configuracion anterior.
        """
        fin = time.time() + timeout
        while time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.005)
            try:
                t = self.buffer.lookup_transform(
                    "base_link", "tool0", rclpy.time.Time())
            except Exception:
                continue

            if stamp_a_ns(t.header.stamp) < t_pub_ns:
                continue          # todavia es la pose vieja

            tr = t.transform.translation
            ro = t.transform.rotation
            return (np.array([tr.x, tr.y, tr.z]),
                    quat_to_matrix(ro.x, ro.y, ro.z, ro.w))
        return None
```

**Cómo leerlo:** Evita leer el TF de la configuración anterior.

### C. Barrido y métricas
correr genera configuraciones y calcula error de posición/orientación.

**Código real** (`scripts/validate_tf.py`, líneas 132-203):

```python

            if stamp_a_ns(t.header.stamp) < t_pub_ns:
                continue          # todavia es la pose vieja

            tr = t.transform.translation
            ro = t.transform.rotation
            return (np.array([tr.x, tr.y, tr.z]),
                    quat_to_matrix(ro.x, ro.y, ro.z, ro.w))
        return None

    def correr(self):
        rng = np.random.default_rng(7)
        err_p, err_r = [], []
        fallos = 0

        print(f"\n   Validacion TF vs analitica   {self.n} configuraciones\n")

        self.revisar_competencia()

        # Calentamiento: el listener necesita acumular el arbol antes de
        # que cualquier lookup pueda tener exito.
        for _ in range(20):
            self.publicar(np.zeros(6))
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.5)

        for k in range(self.n):
            q = rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
            t_pub = self.publicar(q)

            lectura = self.leer_tf_sincronizado(t_pub)
            if lectura is None:
                fallos += 1
                continue
            p_tf, R_tf = lectura

            T = forward_kinematics(q)
            p_ana, R_ana = T[:3, 3], T[:3, :3]

            err_p.append(np.linalg.norm(p_tf - p_ana))
            err_r.append(np.linalg.norm(R_ana.T @ R_tf - np.eye(3), "fro"))

            if (k + 1) % 100 == 0:
                print(f"  {k + 1:5d} / {self.n}   "
                      f"peor pos {max(err_p):.3e} m")

        if not err_p:
            print("\n  No se obtuvo ninguna lectura de TF.")
            print("  Verifique que display.launch.py este corriendo.\n")
            return False

        ep = np.array(err_p)
        er = np.array(err_r)

        print(f"\nResultados sobre {len(ep)} muestras validas")
        print(f"  Error posicional   medio  {ep.mean():.3e} m")
        print(f"  Error posicional   maximo {ep.max():.3e} m")
        print(f"  Error angular      maximo {er.max():.3e}")
        if fallos:
            print(f"  Lecturas perdidas por timeout: {fallos}")

        ok = ep.max() < UMBRAL
        print(f"\n  Umbral exigido por el hito: {UMBRAL:.0e} m")
        if ok:
            print(f"  Margen: {UMBRAL / max(ep.max(), 1e-300):.1e}x "
                  f"por debajo del umbral")
        else:
            print(f"  Excede el umbral por un factor de "
                  f"{ep.max() / UMBRAL:.1e}x")
        print("  RESULTADO:", "APROBADO" if ok else "NO APROBADO")
        print()
        return ok
```

**Cómo leerlo:** Valida integración ROS del modelo Hito 1.

## Cómo se relaciona
Depende de FK y del lanzamiento de robot_state_publisher.

## Pruebas y evidencia
El README guarda errores TF y explica precisión de transporte. Esto es independiente del error FK de Pieper.

## Pregunta para practicar defensa
> ¿Por qué una espera fija podía comparar una postura nueva con TF anterior?

## Límite o precaución
No mezcles el error del transporte ROS con precisión matemática interna en float64.

## Ficha de repaso
- Problema que resuelve:
- Parámetros de entrada:
- Salida principal:
- Función clave:
- Prueba que lo respalda:
- Lo que no demuestra:
