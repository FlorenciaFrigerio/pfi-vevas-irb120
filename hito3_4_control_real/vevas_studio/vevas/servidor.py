"""Servidor web local de la consola (solo biblioteca estándar de Python).

    GET  /                 la interfaz (carpeta web/)
    GET  /api/stream       estado en vivo (Server-Sent Events, ~20 por segundo)
    GET  /api/config       configuración y datos fijos del robot
    POST /api/<orden>      órdenes de la interfaz (ver ORDENES)
                           poses guardadas: poses_listar, pose_guardar, pose_renombrar,
                           pose_borrar (exige "confirmar": true), calcular_pose_guardada
                           vista previa de figura (no mueve): vista_figura

Escucha solo en 127.0.0.1 por defecto: nadie de la red puede mandar órdenes al robot.
"""

from __future__ import annotations

import json
import math
import mimetypes
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from .robot_irb120 import DH, LIMITES_DEG, VEL_MAX_DEG
from .secuencias import POSES_DEMO, POSTURA_INICIAL_DEG, PRESETS_POSE, pasos_demo

mimetypes.add_type("text/javascript", ".js")
mimetypes.add_type("text/javascript", ".mjs")
mimetypes.add_type("text/css", ".css")
mimetypes.add_type("font/woff2", ".woff2")
mimetypes.add_type("font/woff", ".woff")
mimetypes.add_type("font/ttf", ".ttf")


def _limpiar(o):
    if hasattr(o, "item") and not hasattr(o, "__len__"):
        o = o.item()
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {k: _limpiar(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_limpiar(v) for v in o]
    if hasattr(o, "tolist"):
        return _limpiar(o.tolist())
    return o


def crear_servidor(consola, carpeta_web: str, host: str, puerto: int, hz: float):
    consola.cfg["servidor"]["hz_estado"] = hz
    def config_publica():
        from copy import deepcopy
        cfg_publica = deepcopy(consola.cfg)
        if "rws" in cfg_publica:
            cfg_publica["rws"].pop("clave", None)
        return {
            "dh": [list(f) for f in DH],
            "limites_deg": [list(l) for l in LIMITES_DEG],
            "vel_max_deg_s": list(VEL_MAX_DEG),
            "postura_inicial_deg": list(consola.cfg.get("postura_inicial_deg", POSTURA_INICIAL_DEG)),
            "presets": PRESETS_POSE,
            "poses_demo": POSES_DEMO,
            "cfg": cfg_publica,
            "frase_real": "MOVER ROBOT REAL",
            "pasos_demo": [{"titulo": p["titulo"], "explicacion": p["explicacion"], "grupo": p.get("grupo", "")} for p in pasos_demo()],
        }

    class Manejador(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def _json(self, datos, codigo=200):
            cuerpo = json.dumps(_limpiar(datos), ensure_ascii=False).encode("utf-8")
            self.send_response(codigo)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(cuerpo)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(cuerpo)

        def do_GET(self):
            ruta = urlparse(self.path).path
            if ruta == "/api/stream":
                return self._stream()
            if ruta == "/api/config":
                return self._json(config_publica())
            if ruta == "/api/estado":
                return self._json(consola.estado())
            if ruta == "/api/verificacion_pendant":
                if consola.backend.tipo != "rws":
                    return self._json({"ok": False, "mensaje": "La comparación con el controlador requiere modo RWS; en SIM no se inventan medidas del FlexPendant."})
                with consola.lock:
                    ocupado = consola.ejec is not None or consola.jog is not None or getattr(consola, "envio_pendiente", False)
                if ocupado:
                    return self._json({"ok": False, "mensaje": "Esperá a que termine el movimiento antes de tomar una captura de comparación."})
                try:
                    from .verificacion_pendant import comparar_con_controlador
                    rc = consola.cfg["robot"]
                    comparacion = comparar_con_controlador(consola.backend.cli, consola.robot,
                                                          signos=rc["signos"], offsets_deg=rc["offsets_deg"])
                    consola.reg.evento("verificacion_pendant", datos=comparacion)
                    return self._json(comparacion)
                except (ValueError, RuntimeError) as error:
                    return self._json({"ok": False, "mensaje": str(error)})
            if ruta == "/api/plan":
                from .consola import publico

                return self._json(publico(consola.plan) if consola.plan else None)
            if ruta == "/":
                ruta = "/index.html"
            archivo = os.path.normpath(os.path.join(carpeta_web, ruta.lstrip("/")))
            if not archivo.startswith(os.path.abspath(carpeta_web)) or not os.path.isfile(archivo):
                self.send_error(404)
                return
            with open(archivo, "rb") as f:
                datos = f.read()
            tipo = mimetypes.guess_type(archivo)[0] or "application/octet-stream"
            self.send_response(200)
            self.send_header("Content-Type", tipo)
            self.send_header("Content-Length", str(len(datos)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(datos)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            try:
                while True:
                    datos = json.dumps(_limpiar(consola.estado()), ensure_ascii=False)
                    self.wfile.write(f"data: {datos}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    # Frecuencia de pantalla: no cambia el sondeo RWS ni el mando RAPID.
                    time.sleep(1.0 / float(consola.cfg["servidor"]["hz_estado"]))
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

        def do_POST(self):
            ruta = urlparse(self.path).path
            n = int(self.headers.get("Content-Length") or 0)
            try:
                cuerpo = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                return self._json({"ok": False, "mensaje": "JSON inválido"}, 400)
            orden = ruta.replace("/api/", "")
            consola.pulso()  # cualquier orden demuestra que la interfaz sigue viva
            try:
                r = ORDENES[orden](consola, cuerpo)
            except KeyError:
                return self._json({"ok": False, "mensaje": f"Orden desconocida: {orden}"}, 404)
            except Exception as e:  # noqa: BLE001
                consola.avisar(f"Orden '{orden}' rechazada: {e}", "error")
                return self._json({"ok": False, "mensaje": str(e)}, 200)
            return self._json(r)

    servidor = ThreadingHTTPServer((host, puerto), Manejador)
    servidor.daemon_threads = True
    return servidor


def _ok(mensaje="ok", **extra):
    return {"ok": True, "mensaje": mensaje, **extra}


def _res(t):
    ok, msg = t
    return {"ok": ok, "mensaje": msg}


def ejecutar_revisado(consola, cuerpo):
    plan_id = cuerpo.get("plan_id")
    if type(plan_id) is not int or plan_id < 1:
        return {"ok": False, "mensaje": "Falta un plan_id válido del plan que revisaste. Volvé a calcular antes de enviar."}
    return _res(consola.ejecutar(plan_id=plan_id))


def fijar_frecuencia(consola, cuerpo):
    hz = cuerpo.get("hz")
    if isinstance(hz, bool) or not isinstance(hz, (int, float)) or hz not in (5, 10, 20):
        return {"ok": False, "mensaje": "La frecuencia de pantalla debe ser 5, 10 o 20 Hz."}
    with consola.lock:
        consola.cfg["servidor"]["hz_estado"] = hz
    fuente = consola.cfg.get("rws", {}).get("hz_lectura") if consola.backend.tipo == "rws" else None
    return {"ok": True, "hz_vista": hz, "hz_fuente": fuente}


def vista_figura(consola, cuerpo):
    """Vista previa de puntos de una figura. No mueve nada ni cambia el plan; los errores de
    validación (tamaño fuera de rango mientras se escribe) no se registran como avisos."""
    try:
        vista = consola.vista_figura(cuerpo.get("figura"), cuerpo.get("tamano_mm", 25), cuerpo.get("plano", "YZ"))
    except (ValueError, RuntimeError) as error:
        return {"ok": False, "mensaje": str(error)}
    return _ok(vista=vista)


def _con_lista(consola, mensaje, **extra):
    return _ok(mensaje, **extra, **consola.listar_poses())


ORDENES = {
    "pulso": lambda c, b: (c.pulso(), _ok())[1],
    "calcular_pose": lambda c, b: _ok(plan=c.calcular_pose({k: float(b[k]) for k in ("x", "y", "z", "rz", "ry", "rx")})),
    "calcular_articular": lambda c, b: _ok(plan=c.calcular_articular([float(v) for v in b["q_deg"]])),
    "calcular_junta": lambda c, b: _ok(plan=c.calcular_junta(int(b["junta"]), float(b["valor_deg"]), bool(b.get("relativo")))),
    "ik_vivo": lambda c, b: _ok(ik=c.ik_vivo_rapido({k: float(b[k]) for k in ("x", "y", "z", "rz", "ry", "rx")}, b.get("q_ref_deg"))),
    "calcular_home": lambda c, b: _ok(plan=c.calcular_home(str(b.get("destino", "cero")))),
    "calcular_figura": lambda c, b: _ok(plan=c.calcular_figura(b.get("figura"), b.get("tamano_mm", 25), b.get("plano", "YZ"))),
    "vista_figura": vista_figura,
    "poses_listar": lambda c, b: _con_lista(c, "ok"),
    "pose_guardar": lambda c, b: _con_lista(c, "Pose guardada.", pose=c.guardar_pose_actual(b.get("nombre"))),
    "pose_renombrar": lambda c, b: _con_lista(c, "Pose renombrada.", pose=c.renombrar_pose(b.get("nombre"), b.get("nuevo"))),
    "pose_borrar": lambda c, b: _con_lista(c, "Pose borrada.", pose=c.borrar_pose(b.get("nombre"), b.get("confirmar"))),
    "calcular_pose_guardada": lambda c, b: _ok(plan=c.calcular_pose_guardada(b.get("nombre"))),
    "perfil": lambda c, b: _ok("Perfil actualizado. Volvé a calcular el recorrido.", perfil=c.fijar_perfil(b.get("perfil"))),
    "elegir_rama": lambda c, b: _ok(plan=c.elegir_rama_manual(int(b["indice"]))),
    "ejecutar": ejecutar_revisado,
    "frecuencia": fijar_frecuencia,
    "parar": lambda c, b: (c.parar(b.get("motivo", "botón PARAR")), _ok("Cancelación solicitada; el tramo físico puede seguir en curso." if c.backend.tipo == "rws" else "Parado."))[1],
    "armar": lambda c, b: _res(c.armar(b.get("checklist", {}), b.get("frase", ""))),
    "desarmar": lambda c, b: (c.desarmar(), _ok("Desarmado."))[1],
    "velocidad": lambda c, b: _ok(vel_pct=c.fijar_velocidad(float(b["pct"]))),
    "reiniciar_vel_max": lambda c, b: (c.reiniciar_maximos_velocidad(), _ok("Máximos de velocidad reiniciados."))[1],
    "jog": lambda c, b: _res(c.orden_jog(b.get("accion", "mantener"), b.get("modo", "junta"), int(b.get("eje", 1)), int(b.get("dir", 1)))),
    "secuencia": lambda c, b: _res(c.iniciar_secuencia(bool(b.get("pausar", True)))),
    "continuar": lambda c, b: (c.continuar_secuencia(), _ok())[1],
}
