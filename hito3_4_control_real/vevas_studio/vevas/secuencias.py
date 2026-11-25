"""Secuencia de demostración (la "trayectoria generada por IA" para el video).

Orden pedido por el grupo y por el docente:
    1. ir a una postura inicial segura (NO q = 0, porque q5 = 0 es singular)
    2. mover las juntas UNA POR UNA (+5° y vuelta) a velocidad mínima
    3. mover las 6 juntas A LA VEZ (+5° y vuelta)
    4. trayectoria por cinemática inversa: 3 poses con la herramienta hacia abajo,
       cada una resuelta con Pieper (8 ramas -> mejor rama -> LSPB)
    5. volver a la postura inicial

Antes de cada paso la consola recalcula todo desde la postura MEDIDA y revisa
límites, altura mínima y singularidades. Si un paso no pasa la revisión, la
secuencia se detiene y lo explica.
"""

from __future__ import annotations

POSTURA_INICIAL_DEG = [0.0, -20.0, 30.0, 0.0, 40.0, 0.0]

# Poses del TCP (tool0) en mm y grados (EulerZYX como el FlexPendant).
# Ry = 180: herramienta apuntando hacia abajo.
POSES_DEMO = [
    {"nombre": "Pose A (izquierda)", "x": 350.0, "y": 150.0, "z": 400.0, "rz": 0.0, "ry": 180.0, "rx": 0.0},
    {"nombre": "Pose B (derecha)", "x": 350.0, "y": -150.0, "z": 400.0, "rz": 0.0, "ry": 180.0, "rx": 0.0},
    {"nombre": "Pose C (centro, más baja)", "x": 400.0, "y": 0.0, "z": 320.0, "rz": 0.0, "ry": 180.0, "rx": 0.0},
]

PRESETS_POSE = [
    {"nombre": "Frente, herramienta abajo", "x": 400.0, "y": 0.0, "z": 400.0, "rz": 0.0, "ry": 180.0, "rx": 0.0},
    {"nombre": "Izquierda, herramienta abajo", "x": 350.0, "y": 150.0, "z": 400.0, "rz": 0.0, "ry": 180.0, "rx": 0.0},
    {"nombre": "Derecha, herramienta abajo", "x": 350.0, "y": -150.0, "z": 400.0, "rz": 0.0, "ry": 180.0, "rx": 0.0},
    {"nombre": "Frente, herramienta horizontal", "x": 450.0, "y": 0.0, "z": 500.0, "rz": 0.0, "ry": 90.0, "rx": 0.0},
]


def pasos_demo(paso_deg: float = 5.0, vel_junta_deg_s: float = 3.0, vel_ik_pct: float | None = None) -> list[dict]:
    """Lista de pasos. 'vel_cap_deg_s' limita la velocidad de la junta más rápida del paso."""
    p = []
    p.append(
        {
            "titulo": "Ir a la postura inicial segura",
            "explicacion": "Se parte de q = [0, -20, 30, 0, 40, 0]°: lejos de límites y de la singularidad q5 = 0.",
            "tipo": "articular",
            "q_deg": POSTURA_INICIAL_DEG,
            "vel_cap_deg_s": 10.0,
        }
    )
    for j in range(6):
        ida = list(POSTURA_INICIAL_DEG)
        ida[j] += paso_deg
        p.append(
            {
                "titulo": f"Junta {j + 1} sola: +{paso_deg:.0f}°",
                "explicacion": f"Solo se mueve J{j + 1}. Se comprueba el sentido de giro y que el error de J{j + 1} se mantenga chico.",
                "tipo": "articular",
                "q_deg": ida,
                "vel_cap_deg_s": vel_junta_deg_s,
                "grupo": "una_por_una",
            }
        )
        p.append(
            {
                "titulo": f"Junta {j + 1} sola: vuelta",
                "explicacion": f"J{j + 1} regresa a la postura inicial.",
                "tipo": "articular",
                "q_deg": list(POSTURA_INICIAL_DEG),
                "vel_cap_deg_s": vel_junta_deg_s,
                "grupo": "una_por_una",
            }
        )
    todas = [v + paso_deg for v in POSTURA_INICIAL_DEG]
    p.append(
        {
            "titulo": f"Las 6 juntas a la vez: +{paso_deg:.0f}°",
            "explicacion": "LSPB sincronizado: las 6 juntas empiezan y terminan juntas. Se mide e_p(t) y e_R(t) con FK en vivo.",
            "tipo": "articular",
            "q_deg": todas,
            "vel_cap_deg_s": vel_junta_deg_s,
            "grupo": "todas",
        }
    )
    p.append(
        {
            "titulo": "Las 6 juntas a la vez: vuelta",
            "explicacion": "Regreso sincronizado a la postura inicial.",
            "tipo": "articular",
            "q_deg": list(POSTURA_INICIAL_DEG),
            "vel_cap_deg_s": vel_junta_deg_s,
            "grupo": "todas",
        }
    )
    for pose in POSES_DEMO:
        p.append(
            {
                "titulo": f"Cinemática inversa: {pose['nombre']}",
                "explicacion": "Pose -> T_d -> centro de muñeca -> 8 ramas de Pieper -> verificación FK -> mejor rama -> LSPB -> envío.",
                "tipo": "pose",
                "pose": pose,
                "vel_cap_deg_s": 12.0,
                "grupo": "ik",
            }
        )
    p.append(
        {
            "titulo": "Volver a la postura inicial",
            "explicacion": "Cierre de la secuencia.",
            "tipo": "articular",
            "q_deg": POSTURA_INICIAL_DEG,
            "vel_cap_deg_s": 10.0,
        }
    )
    return p
