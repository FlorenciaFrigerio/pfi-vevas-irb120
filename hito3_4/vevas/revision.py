"""Revisión previa de un plan antes de enviarlo.

Cada comprobación termina en uno de cuatro estados:

    aprobado       se evaluó y cumple
    aviso          se evaluó, cumple, pero hay algo que el operador debe saber
    rechazado      se evaluó y NO cumple: el plan no se envía
    no_verificado  no se pudo evaluar con los datos que hay (por ejemplo, objetos del
                   entorno que nadie describió). No es lo mismo que «aprobado».

Se revisan, muestra por muestra, las posiciones intermedias de la trayectoria completa:

    inicio        el plan empieza donde está el robot
    limites       juntas dentro de límites con margen
    velocidad     velocidad de cada junta bajo su tope (hoja de datos × % permitido)
    singularidad  valor singular mínimo del Jacobiano y tipo de singularidad
    altura        partes del brazo sobre la altura mínima (mesa)
    autocolision  brazo contra sí mismo y contra la herramienta (cápsulas)
    entorno       cajas del entorno descritas en config.json (mesa, tablero, objetos)

Lo que NO hace: no ve objetos que no estén descritos, no conoce la herramienta real si no
se configuró, y las cápsulas son una aproximación de los eslabones. Un «aprobado» significa
«cumple dentro de este modelo», no «es seguro». La seguridad física sigue siendo del
operador, del paro de emergencia y del controlador IRC5.
"""

from __future__ import annotations

import math

import numpy as np

from .jacobiano import analisis_singularidad
from .robot_irb120 import VEL_MAX_DEG

GRAD = 180.0 / math.pi

ESTADOS = ("aprobado", "aviso", "rechazado", "no_verificado")
GRAVEDAD = {"aprobado": 0, "aviso": 1, "no_verificado": 2, "rechazado": 3}

# Radios de las cápsulas [m] (aproximación de los eslabones del IRB 120)
RADIOS_POR_DEFECTO = {"hombro": 0.055, "codo": 0.050, "antebrazo": 0.045, "muneca": 0.040, "base": 0.085}


# ---------------------------------------------------------------- geometría
def _dist_segmentos(p1, q1, p2, q2) -> float:
    """Distancia mínima entre los segmentos p1-q1 y p2-q2 (Ericson, Real-Time Collision Detection)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    eps = 1e-12
    if a <= eps and e <= eps:
        return float(np.linalg.norm(r))
    if a <= eps:
        s, t = 0.0, min(1.0, max(0.0, f / e))
    else:
        c = float(d1 @ r)
        if e <= eps:
            t, s = 0.0, min(1.0, max(0.0, -c / a))
        else:
            b = float(d1 @ d2)
            den = a * e - b * b
            s = min(1.0, max(0.0, (b * f - c * e) / den)) if den > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, min(1.0, max(0.0, -c / a))
            elif t > 1.0:
                t, s = 1.0, min(1.0, max(0.0, (b - c) / a))
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


def _dist_segmento_caja(p, q, lo, hi, n: int = 24) -> float:
    """Distancia (aproximada por muestreo) de un segmento a una caja alineada con los ejes."""
    mejor = float("inf")
    for s in np.linspace(0.0, 1.0, n):
        x = p + (q - p) * s
        d = np.maximum(np.maximum(lo - x, x - hi), 0.0)
        mejor = min(mejor, float(np.linalg.norm(d)))
    return mejor


def capsulas(robot, q, cfg: dict) -> list[tuple[str, np.ndarray, np.ndarray, float]]:
    """Cápsulas del brazo en el marco base: (nombre, A, B, radio)."""
    M = robot.marcos(q)
    tcp = robot.fk_tcp(q)[:3, 3]
    rev = cfg.get("revision", {})
    r = {**RADIOS_POR_DEFECTO, **rev.get("radios_m", {})}
    tool = cfg.get("entorno", {}).get("herramienta", {})
    r_tool = float(tool.get("radio_mm", 30.0)) / 1000.0
    P = lambda i: M[i][:3, 3]  # noqa: E731
    caps = [
        ("base", np.zeros(3), P(1), r["base"]),
        ("hombro", P(1), P(2), r["hombro"]),
        ("codo", P(2), P(3), r["codo"]),
        ("antebrazo", P(3), P(4), r["antebrazo"]),
        ("muneca", P(4), P(6), r["muneca"]),
    ]
    if np.linalg.norm(tcp - P(6)) > 1e-4:
        caps.append(("herramienta", P(6), tcp, r_tool))
    return caps


# Pares que se comparan. Se omiten los eslabones contiguos (comparten una junta) y los que
# el propio rango articular del IRB 120 ya impide que se toquen.
PARES = (
    ("base", "antebrazo"), ("base", "muneca"), ("base", "herramienta"), ("base", "codo"),
    ("hombro", "muneca"), ("hombro", "herramienta"),
    ("codo", "muneca"), ("codo", "herramienta"),
)


def margen_autocolision(robot, q, cfg: dict) -> tuple[float, tuple[str, str] | None]:
    """Menor holgura [m] entre pares de cápsulas (negativa = se tocan) y el par que la da."""
    caps = {c[0]: c for c in capsulas(robot, q, cfg)}
    peor, par = float("inf"), None
    for a, b in PARES:
        if a not in caps or b not in caps:
            continue
        _, pa, qa, ra = caps[a]
        _, pb, qb, rb = caps[b]
        h = _dist_segmentos(pa, qa, pb, qb) - (ra + rb)
        if h < peor:
            peor, par = h, (a, b)
    return peor, par


def margen_entorno(robot, q, cfg: dict) -> tuple[float, str | None]:
    """Menor holgura [m] contra las cajas del entorno y la mesa (negativa = se tocan)."""
    ent = cfg.get("entorno", {})
    peor, quien = float("inf"), None
    mesa = float(ent.get("mesa_z_m", 0.0))
    for nombre, a, b, rad in capsulas(robot, q, cfg):
        if nombre != "base":
            h = min(a[2], b[2]) - rad - mesa
            if h < peor:
                peor, quien = h, f"mesa ({nombre})"
        for caja in ent.get("cajas", []):
            lo = np.asarray(caja["min_mm"], dtype=float) / 1000.0
            hi = np.asarray(caja["max_mm"], dtype=float) / 1000.0
            h = _dist_segmento_caja(a, b, lo, hi) - rad
            if h < peor:
                peor, quien = h, f"{caja.get('nombre', 'objeto')} ({nombre})"
    return peor, quien


def tipo_singularidad(s: dict) -> str:
    tipos = []
    if s["muneca_sin_q5"] < 0.09:
        tipos.append("muñeca (q5 ≈ 0)")
    if s["codo_sin_delta"] < 0.09:
        tipos.append("codo (brazo estirado)")
    if s["hombro_r_w_m"] < 0.03:
        tipos.append("hombro (muñeca sobre el eje 1)")
    return ", ".join(tipos) or "mezcla de ejes"


# ---------------------------------------------------------------- revisión
def _chequeo(id_, nombre, estado, detalle, obligatorio=True, k=None, t=None, juntas=None) -> dict:
    return {
        "id": id_, "nombre": nombre, "estado": estado, "detalle": detalle,
        "obligatorio": obligatorio, "k": k, "t": t, "juntas": juntas or [],
    }


FACTOR_CERCA = 2.0  # «cerca» de una singularidad: σ_min < FACTOR_CERCA · ik.sigma_min


def nivel_singularidad(sigma_min: float, sigma_umbral: float) -> str:
    """«singular» (σ_min < umbral: la revisión rechaza), «cerca» (< 2·umbral) u «ok»."""
    if sigma_min < sigma_umbral:
        return "singular"
    if sigma_min < FACTOR_CERCA * sigma_umbral:
        return "cerca"
    return "ok"


def revisar_trayectoria(robot, ts, Q, QD, q_medida, cfg: dict, vel_pct_max: float, exige_go: bool = False,
                        permitir_singular_destino: bool = False) -> dict:
    """Revisa la trayectoria completa y devuelve el informe.

    ts, Q, QD   muestras de la trayectoria (s, rad, rad/s)
    q_medida    postura medida por el robot (o None)
    vel_pct_max tope de velocidad vigente en este modo, en % de la hoja de datos
    exige_go    True con robot real o hardware sin identificar: ahí «no verificado» bloquea.
    permitir_singular_destino  solo para movimientos articulares a una postura home (q = 0): si la
                singularidad aparece únicamente al final del recorrido, es «aviso» y no «rechazado».
    """
    ts, Q, QD = np.asarray(ts), np.asarray(Q), np.asarray(QD)
    n = len(ts)
    s_cfg, ik = cfg["seguridad"], cfg["ik"]
    rev = cfg.get("revision", {})
    margen_lim = math.radians(ik["margen_limite_deg"])
    z_min = s_cfg["z_min_m"]
    holgura = float(rev.get("holgura_colision_m", 0.005))
    sigma_min = float(ik["sigma_min"])
    paso = max(1, n // int(rev.get("muestras_max", 240)))
    idx = list(range(0, n, paso))
    if idx[-1] != n - 1:
        idx.append(n - 1)

    chequeos: list[dict] = []
    sev = np.zeros(n, dtype=int)  # gravedad por muestra: 0 ok, 1 aviso, 2 falla
    etiqueta_k: dict[int, str] = {}

    def marcar(k, nivel, texto):
        if nivel > sev[k]:
            sev[k] = nivel
            etiqueta_k[k] = texto

    # 1. inicio
    if q_medida is None:
        chequeos.append(_chequeo("inicio", "Empieza donde está el robot", "rechazado", "No llega la posición del robot."))
    else:
        dif = np.degrees(np.abs(Q[0] - np.asarray(q_medida)))
        if dif.max() > s_cfg["inicio_max_deg"]:
            j = int(np.argmax(dif)) + 1
            chequeos.append(_chequeo("inicio", "Empieza donde está el robot", "rechazado",
                                     f"El plan no empieza donde está el robot (J{j} difiere {dif.max():.2f}°). Recalculá desde la postura actual.",
                                     k=0, t=0.0, juntas=[j]))
        else:
            chequeos.append(_chequeo("inicio", "Empieza donde está el robot", "aprobado", f"Diferencia máxima {dif.max():.2f}°."))

    # 2. límites articulares
    lim_k, lim_j, peor_marg = None, None, float("inf")
    for k in idx:
        dentro = robot.dentro_de_limites(Q[k], margen_lim)
        m = min(robot.distancia_a_limites(Q[k]))
        peor_marg = min(peor_marg, m)
        if not all(dentro):
            marcar(k, 2, "límite articular")
            if lim_k is None:
                lim_k, lim_j = k, dentro.index(False) + 1
    if lim_k is not None:
        chequeos.append(_chequeo("limites", "Límites articulares", "rechazado",
                                 f"La trayectoria pasa a menos de {math.degrees(margen_lim):.0f}° del límite de J{lim_j} (en t = {ts[lim_k]:.2f} s).",
                                 k=lim_k, t=float(ts[lim_k]), juntas=[lim_j]))
    else:
        chequeos.append(_chequeo("limites", "Límites articulares", "aprobado",
                                 f"Margen mínimo a un límite: {math.degrees(peor_marg):.1f}° (se pide ≥ {math.degrees(margen_lim):.0f}°)."))

    # 3. velocidad por junta (C02)
    tope_pct = min(float(vel_pct_max), 100.0)
    tope = np.array(VEL_MAX_DEG) * tope_pct / 100.0
    vmax = np.degrees(np.abs(QD)).max(axis=0)
    exceso = vmax / np.maximum(tope, 1e-9)
    j_ex = int(np.argmax(exceso))
    if exceso[j_ex] > 1.001:
        k_ex = int(np.argmax(np.degrees(np.abs(QD[:, j_ex]))))
        marcar(k_ex, 2, "velocidad")
        chequeos.append(_chequeo("velocidad", "Velocidad por junta", "rechazado",
                                 f"J{j_ex + 1} llegaría a {vmax[j_ex]:.1f}°/s y su tope es {tope[j_ex]:.1f}°/s ({tope_pct:.0f} % de la hoja de datos).",
                                 k=k_ex, t=float(ts[k_ex]), juntas=[j_ex + 1]))
    else:
        chequeos.append(_chequeo("velocidad", "Velocidad por junta", "aprobado",
                                 f"Mayor uso de tope: J{j_ex + 1} a {vmax[j_ex]:.1f}°/s de {tope[j_ex]:.1f}°/s permitidos."))

    # 4. singularidad a lo largo de la trayectoria
    sig = []
    for k in idx:
        s = analisis_singularidad(robot, Q[k])
        sig.append((k, s["sigma_min"], s))
    k_ini, sig_ini, s_ini = sig[0]
    k_min, sig_mn, s_mn = min(sig, key=lambda x: x[1])
    texto_tipo = tipo_singularidad(s_mn)
    # Rechazo: el camino se mete en (o más hondo en) una zona singular que no estaba al principio.
    # «empeora» exige una caída real (no ruido numérico cuando ya parte en una singularidad exacta)
    cae = sig_mn < 0.9 * sig_ini and (sig_ini - sig_mn) > 0.002
    entra = sig_mn < sigma_min and cae
    solo_destino = False
    if entra and permitir_singular_destino:
        # la zona singular debe ser un tramo continuo que termina en la última muestra
        mal = [i for i, (_, sg, _) in enumerate(sig) if sg < sigma_min and sg < 0.9 * sig_ini and (sig_ini - sg) > 0.002]
        solo_destino = bool(mal) and mal[-1] == len(sig) - 1 and mal == list(range(mal[0], len(sig)))
    if entra and solo_destino:
        for k, sg, _ in sig:
            if sg < sigma_min:
                marcar(k, 1, "singularidad")
        chequeos.append(_chequeo("singularidad", "Singularidades en el recorrido", "aviso",
                                 f"El destino es singular (σ = {sig[-1][1]:.4f}, {tipo_singularidad(sig[-1][2])}). "
                                 "q = 0 es singular de muñeca; en un movimiento articular no es un problema. "
                                 "No hagas movimientos cartesianos desde ahí.",
                                 k=sig[-1][0], t=float(ts[sig[-1][0]])))
    elif entra:
        for k, sg, _ in sig:
            if sg < sigma_min and sg < 0.9 * sig_ini and (sig_ini - sg) > 0.002:
                marcar(k, 2, "singularidad")
        chequeos.append(_chequeo("singularidad", "Singularidades en el recorrido", "rechazado",
                                 f"En t = {ts[k_min]:.2f} s el valor singular mínimo cae a σ = {sig_mn:.4f} (umbral {sigma_min}); zona: {texto_tipo}.",
                                 k=k_min, t=float(ts[k_min])))
    elif sig_ini < sigma_min:
        for k, sg, _ in sig:
            if sg < sigma_min:
                marcar(k, 1, "singularidad")
        chequeos.append(_chequeo("singularidad", "Singularidades en el recorrido", "aviso",
                                 f"El robot ya parte cerca de una singularidad (σ = {sig_ini:.4f}, {tipo_singularidad(s_ini)}). "
                                 f"El camino no la empeora (σ mín. = {sig_mn:.4f}). Evitá movimientos cartesianos desde ahí."))
    else:
        chequeos.append(_chequeo("singularidad", "Singularidades en el recorrido", "aprobado",
                                 f"σ mínimo en el recorrido: {sig_mn:.3f} (umbral {sigma_min}). Se evaluaron {len(idx)} posturas; entre muestras no se garantiza."))

    # 5. altura mínima
    z_k, z_val = None, float("inf")
    for k in idx:
        z = float(robot.puntos_cuerpo(Q[k])[1:, 2].min())
        if z < z_min and z_k is None:
            z_k, z_val = k, z
        if z < z_min:
            marcar(k, 2, "altura mínima")
    if z_k is not None:
        chequeos.append(_chequeo("altura", "Altura mínima sobre la mesa", "rechazado",
                                 f"Una parte del brazo baja a z = {z_val * 1000:.0f} mm (< {z_min * 1000:.0f} mm) en t = {ts[z_k]:.2f} s.",
                                 k=z_k, t=float(ts[z_k])))
    else:
        chequeos.append(_chequeo("altura", "Altura mínima sobre la mesa", "aprobado", f"Se mantiene sobre {z_min * 1000:.0f} mm."))

    # 6. autocolisión (modelo de cápsulas)
    ac_k, ac_par, ac_h, mejor_h = None, None, float("inf"), float("inf")
    for k in idx:
        h, par = margen_autocolision(robot, Q[k], cfg)
        mejor_h = min(mejor_h, h)
        if h < holgura:
            marcar(k, 2, "autocolisión")
            if ac_k is None:
                ac_k, ac_par, ac_h = k, par, h
    if ac_k is not None:
        chequeos.append(_chequeo("autocolision", "Brazo contra sí mismo (modelo aproximado)", "rechazado",
                                 f"En t = {ts[ac_k]:.2f} s «{ac_par[0]}» y «{ac_par[1]}» quedan a {ac_h * 1000:.0f} mm (se exige ≥ {holgura * 1000:.0f} mm).",
                                 k=ac_k, t=float(ts[ac_k])))
    else:
        chequeos.append(_chequeo("autocolision", "Brazo contra sí mismo (modelo aproximado)", "aprobado",
                                 f"Holgura mínima entre eslabones: {mejor_h * 1000:.0f} mm (cápsulas)."))

    # 7. entorno (mesa y objetos descritos)
    ent = cfg.get("entorno", {})
    cajas = ent.get("cajas", [])
    en_k, en_q, mejor_e = None, None, float("inf")
    for k in idx:
        h, quien = margen_entorno(robot, Q[k], cfg)
        mejor_e = min(mejor_e, h)
        if h < holgura:
            marcar(k, 2, "entorno")
            if en_k is None:
                en_k, en_q = k, quien
    sin_objetos = bool(ent.get("sin_objetos_externos_confirmado", False))
    if en_k is not None:
        chequeos.append(_chequeo("entorno", "Colisión con mesa y objetos descritos", "rechazado",
                                 f"En t = {ts[en_k]:.2f} s el brazo toca {en_q} (holgura {mejor_e * 1000:.0f} mm; se exige ≥ {holgura * 1000:.0f} mm).",
                                 k=en_k, t=float(ts[en_k])))
    elif cajas or sin_objetos:
        det = f"Holgura mínima a mesa y {len(cajas)} objeto(s) descritos: {mejor_e * 1000:.0f} mm."
        if sin_objetos and not cajas:
            det = f"Mesa revisada (holgura {mejor_e * 1000:.0f} mm). El operador confirmó en config.json que no hay otros objetos."
        chequeos.append(_chequeo("entorno", "Colisión con mesa y objetos descritos", "aprobado", det))
    else:
        chequeos.append(_chequeo("entorno", "Colisión con mesa y objetos descritos", "no_verificado",
                                 "No hay objetos descritos. Solo se revisó la mesa. Agregá las cajas del entorno en config.json "
                                 "(sección «entorno») o confirmá que no hay nada más con sin_objetos_externos_confirmado."))

    # 8. herramienta
    tool = ent.get("herramienta", {})
    tcp_cero = all(abs(v) < 1e-9 for v in cfg["robot"]["tcp_xyz_m"])
    if tool.get("confirmada") or (tcp_cero and ent.get("sin_herramienta_confirmada")):
        chequeos.append(_chequeo("herramienta", "Herramienta montada", "aprobado", "El operador confirmó la herramienta en config.json."))
    else:
        chequeos.append(_chequeo("herramienta", "Herramienta montada", "no_verificado",
                                 "No se confirmó qué hay en la brida. Configurá robot.tcp_xyz_m y entorno.herramienta "
                                 "(radio y largo), o declará entorno.sin_herramienta_confirmada."))

    # Resumen
    peor = max((GRAVEDAD[c["estado"]] for c in chequeos), default=0)
    rechazados = [c for c in chequeos if c["estado"] == "rechazado"]
    pendientes = [c for c in chequeos if c["estado"] == "no_verificado" and c["obligatorio"]]
    if rechazados:
        estado = "rechazado"
    elif pendientes:
        estado = "no_verificado"
    elif any(c["estado"] == "aviso" for c in chequeos):
        estado = "aviso"
    else:
        estado = "aprobado"
    bloquea_envio = bool(rechazados) or (bool(pendientes) and exige_go)

    falla = None
    if rechazados:
        primero = min((c for c in rechazados if c["k"] is not None), key=lambda c: c["k"], default=None)
        if primero is not None:
            k = int(primero["k"])
            falla = {"check": primero["id"], "nombre": primero["nombre"], "k": k, "t": float(ts[k]),
                     "detalle": primero["detalle"], "q_deg": [float(a) * GRAD for a in Q[k]], "juntas": primero["juntas"]}
    # Cinta: gravedad por tramo de tiempo para dibujarla (máx. 80 casillas)
    casillas = min(80, n)
    bordes = np.linspace(0, n, casillas + 1).astype(int)
    cinta = [int(sev[a:max(b, a + 1)].max()) for a, b in zip(bordes[:-1], bordes[1:])]
    return {
        "estado": estado,
        "bloquea_envio": bool(bloquea_envio),
        "chequeos": chequeos,
        "falla": falla,
        "cinta": cinta,
        "t_total": float(ts[-1]),
        "muestras_revisadas": len(idx),
        "alcance": "Modelo del brazo (límites, Jacobiano, cápsulas) y objetos descritos en config.json. "
                   "No detecta objetos no descritos ni sustituye el paro de emergencia ni la revisión del operador.",
    }


def problemas_de(informe: dict) -> list[str]:
    """Textos que impiden enviar (para mostrarlos y para el registro)."""
    out = [c["detalle"] for c in informe["chequeos"] if c["estado"] == "rechazado"]
    if not out and informe["bloquea_envio"]:
        out = [f"Falta verificar: {c['nombre']}. {c['detalle']}" for c in informe["chequeos"]
               if c["estado"] == "no_verificado" and c["obligatorio"]]
    return out
