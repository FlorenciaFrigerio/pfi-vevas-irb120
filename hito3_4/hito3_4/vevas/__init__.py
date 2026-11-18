"""Paquete de cálculo y control del Grupo VEVAS para el ABB IRB 120-3/0.6.

Módulos:
    robot_irb120  tabla D-H, límites, cinemática directa (FK) y marcos
    pieper        cinemática inversa analítica (8 ramas) y selección de rama
    jacobiano     Jacobiano geométrico, diferencias finitas y singularidades
    trayectoria   trayectorias LSPB sincronizadas (perfil trapezoidal)
    seguridad     comprobaciones antes y durante el movimiento
    consola       lógica central de la consola (estado, planes, jog, abortos)

Unidades en todo el paquete: metros, radianes, segundos. Los grados y
milímetros solo aparecen en la interfaz y en los informes.
"""

__version__ = "1.0.0"
