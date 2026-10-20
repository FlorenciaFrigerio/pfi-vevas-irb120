# Script 10 - `graficas.py`: Generación de figuras

> **Nivel:** Complementario - saber leer

## Para qué sirve
Representa trayectorias, contribuciones de torque, barridos de singularidad y manipulabilidad.

**Entrada y salida:** Entrada: datos calculados por otros módulos. Salida: archivos PDF de gráficas.

## Recorrido por bloques

### A. Estilo y fases
_limpiar y _fases preparan ejes y sombreado LSPB.

**Código real** (`scripts/graficas.py`, líneas 55-60):

```python
def _fases(ax, tc, T):
    """Sombrea las tres fases del LSPB y las nombra una sola vez."""
    ax.axvspan(0, tc, color="#000000", alpha=0.04, lw=0)
    ax.axvspan(T - tc, T, color="#000000", alpha=0.04, lw=0)
    ax.axvline(tc, color=TINTA_2, lw=0.8, ls=":")
    ax.axvline(T - tc, color=TINTA_2, lw=0.8, ls=":")
```

**Cómo leerlo:** La apariencia no cambia el cálculo numérico.

### B. Trayectoria/torque
perfil_lspb y descomposicion_torque separan paneles y unidades.

**Código real** (`scripts/graficas.py`, líneas 63-108):

```python
def perfil_lspb(traj, archivo="fig_lspb.pdf", titulo=None):
    """
    Tres paneles apilados: q(t), qd(t), qdd(t), con eje de tiempo comun.

    Se apilan en vez de superponerse porque tienen unidades distintas
    (rad, rad/s, rad/s^2): un solo eje con dos escalas seria enganoso.
    """
    t = np.linspace(0, traj.T, 1200)
    q, qd, qdd = traj(t)

    # Cada junta tiene su propio tiempo de mezcla. Se sombrean las fases
    # de la junta que FIJA el tiempo total (la de mayor tc), y se dice
    # explicitamente: sombrear un tc comun que no existe seria enganoso.
    j_ref = int(np.argmax([p.tc for p in traj.perfiles]))
    tc = traj.perfiles[j_ref].tc

    fig, axs = plt.subplots(3, 1, figsize=(7.0, 6.2), sharex=True)
    etiquetas = [f"J{i+1}" for i in range(q.shape[1])]
    datos = [(q, "$q(t)$  [rad]"), (qd, r"$\dot{q}(t)$  [rad/s]"),
             (qdd, r"$\ddot{q}(t)$  [rad/s$^2$]")]

    for ax, (Y, ylab) in zip(axs, datos):
        _fases(ax, tc, traj.T)
        for j in range(Y.shape[1]):
            ax.plot(t, Y[:, j], color=C[j], lw=2.0, label=etiquetas[j])
        ax.set_ylabel(ylab)
        _limpiar(ax)

    for x, txt in [(tc/2, "acel."), (traj.T/2, "crucero"),
                   (traj.T - tc/2, "desacel.")]:
        axs[0].annotate(txt, xy=(x/traj.T, 1.005), xycoords="axes fraction",
                        ha="center", va="bottom", fontsize=7.5, color=TINTA_2)

    axs[2].set_xlabel(f"$t$  [s]      $T = {traj.T:.3f}$ s")
    fig.text(0.5, -0.012,
             f"Zonas sombreadas: fases de J{j_ref+1}, la junta que fija $T$. "
             f"Cada junta tiene su propio tiempo de mezcla $t_c$.",
             ha="center", fontsize=7.5, color=TINTA_2, style="italic")
    axs[0].legend(ncol=6, frameon=False, fontsize=8,
                  loc="lower center", bbox_to_anchor=(0.5, 1.09))
    if titulo:
        fig.suptitle(titulo, y=0.99, fontsize=10)
    fig.tight_layout()
    fig.savefig(archivo, bbox_inches="tight")
    plt.close(fig)
    return archivo
```

**Cómo leerlo:** Lee ejes, leyendas y contribuciones por separado.

### C. Singularidad
divergencia y manipulabilidad grafican tendencias y rango.

**Código real** (`scripts/graficas.py`, líneas 168-199):

```python
def divergencia(q5_vals, norma_qd, taus, m_v, m_t,
                archivo="fig_divergencia.pdf"):
    """
    El pasaporte, en escala log-log.

    En log-log una ley de potencia es una recta cuya pendiente es el
    exponente. Que los puntos caigan sobre una recta de pendiente -1 y
    -2 es la prueba grafica de que la divergencia es analitica y no un
    accidente numerico.
    """
    fig, axs = plt.subplots(1, 2, figsize=(7.4, 3.2))

    for ax, y, m, lab, col, uni in [
            (axs[0], norma_qd, m_v, r"$\|\dot{q}\|$", C[0], "rad/s"),
            (axs[1], taus,     m_t, r"$|\tau|_{\max}$", C[3], "N·m")]:
        ax.loglog(q5_vals, y, "o-", color=col, lw=2.0, ms=6,
                  mec="white", mew=1.2)
        ref = y[0] * (q5_vals/q5_vals[0])**round(m)
        ax.loglog(q5_vals, ref, "--", color=TINTA_2, lw=1.0,
                  label=f"pendiente {round(m):+d}")
        ax.set_xlabel("$q_5$  [rad]")
        ax.set_ylabel(f"{lab}  [{uni}]")
        ax.legend(frameon=False, fontsize=8, loc="upper right")
        ax.grid(which="both", color=REJILLA, lw=0.5)
        _limpiar(ax)

    axs[0].set_title("Velocidad articular exigida", fontsize=9, pad=8)
    axs[1].set_title("Torque inercial de muñeca", fontsize=9, pad=8)
    fig.tight_layout()
    fig.savefig(archivo, bbox_inches="tight")
    plt.close(fig)
    return archivo
```

**Cómo leerlo:** La gráfica visualiza un experimento; no convierte una tendencia en ley universal.

## Cómo se relaciona
Importa lspb.py, jacobiano.py, singularidad.py y dinamica.py.

## Pruebas y evidencia
Ver docs/hito2/figuras y docs/hito2/informe_ieee/img.

## Pregunta para practicar defensa
> ¿Por qué se separan magnitudes con unidades diferentes en paneles?

## Límite o precaución
Una gráfica no prueba por sí sola la fórmula; debe acompañarse con métrica, condiciones y muestra.

## Ficha de repaso
- Problema que resuelve:
- Parámetros de entrada:
- Salida principal:
- Función clave:
- Prueba que lo respalda:
- Lo que no demuestra:
