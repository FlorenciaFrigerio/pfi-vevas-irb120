"""Servidor web local de la consola (solo biblioteca estándar de Python).

    GET  /                 la interfaz (carpeta web/)
    GET  /api/stream       estado en vivo (Server-Sent Events, ~20 por segundo)
    GET  /api/config       configuración y datos fijos del robot
    POST /api/<orden>      órdenes de la interfaz (ver ORDENES)

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
    def config_publica():
        return {
            "dh": [list(f) for f in DH],
            "limites_deg": [list(l) for l in LIMITES_DEG],
            "vel_max_deg_s": list(VEL_MAX_DEG),
            "postura_inicial_deg": POSTURA_INICIAL_DEG,
            "presets": PRESETS_POSE,
            "poses_demo": POSES_DEMO,
            "cfg": consola.cfg,
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
            periodo = 1.0 / hz
            try:
                while True:
                    datos = json.dumps(_limpiar(consola.estado()), ensure_ascii=False)
                    self.wfile.write(f"data: {datos}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(periodo)
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


ORDENES = {
    "pulso": lambda c, b: (c.pulso(), _ok())[1],
    "calcular_pose": lambda c, b: _ok(plan=c.calcular_pose({k: float(b[k]) for k in ("x", "y", "z", "rz", "ry", "rx")})),
    "calcular_articular": lambda c, b: _ok(plan=c.calcular_articular([float(v) for v in b["q_deg"]])),
    "calcular_junta": lambda c, b: _ok(plan=c.calcular_junta(int(b["junta"]), float(b["valor_deg"]), bool(b.get("relativo")))),
    "elegir_rama": lambda c, b: _ok(plan=c.elegir_rama_manual(int(b["indice"]))),
    "ejecutar": lambda c, b: _res(c.ejecutar()),
    "parar": lambda c, b: (c.parar(b.get("motivo", "botón PARAR")), _ok("Parado."))[1],
    "armar": lambda c, b: _res(c.armar(b.get("checklist", {}), b.get("frase", ""))),
    "desarmar": lambda c, b: (c.desarmar(), _ok("Desarmado."))[1],
    "velocidad": lambda c, b: _ok(vel_pct=c.fijar_velocidad(float(b["pct"]))),
    "jog": lambda c, b: _res(c.orden_jog(b.get("accion", "mantener"), b.get("modo", "junta"), int(b.get("eje", 1)), int(b.get("dir", 1)))),
    "secuencia": lambda c, b: _res(c.iniciar_secuencia(bool(b.get("pausar", True)))),
    "continuar": lambda c, b: (c.continuar_secuencia(), _ok())[1],
}
