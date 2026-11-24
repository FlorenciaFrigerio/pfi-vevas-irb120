"""Poses guardadas por el usuario: las 6 juntas medidas y la pose del TCP por FK.

Se guardan en un JSON local (por defecto ``datos/poses_guardadas.json``, carpeta
ignorada por git; se cambia con ``--datos``). Formato::

    {"version": 1,
     "poses": [{"nombre": "Sobre la mesa", "q_deg": [6 números],
                "tcp": {"x_mm", "y_mm", "z_mm", "rz_deg", "ry_deg", "rx_deg"},
                "fecha": "2026-11-24T10:31:05"}]}

La escritura es atómica (archivo temporal en la misma carpeta + ``os.replace``).
Un archivo dañado no rompe la consola: se avisa, se ignora y, antes de volver a
escribir, se conserva una copia con el sufijo ``.danado-<fecha>``.
"""

from __future__ import annotations

import datetime as dt
import json
import math
import os
import re
import tempfile
import threading

VERSION = 1
NOMBRE_MAX = 40
MAX_POSES = 200
# Letras (con tildes), dígitos, espacio y algunos signos. Sin barras, comillas ni < >.
_NOMBRE_OK = re.compile(r"^[^\W_]([\w .,()+#°-]*[\w.)°#])?$")
_CLAVES_TCP = ("x_mm", "y_mm", "z_mm", "rz_deg", "ry_deg", "rx_deg")


def _finito(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def validar_nombre(nombre) -> str:
    """Devuelve el nombre limpio o lanza ValueError con un motivo legible."""
    if not isinstance(nombre, str):
        raise ValueError("El nombre de la pose debe ser texto.")
    limpio = " ".join(nombre.split())
    if not limpio:
        raise ValueError("Escribí un nombre para la pose.")
    if len(limpio) > NOMBRE_MAX:
        raise ValueError(f"El nombre puede tener como máximo {NOMBRE_MAX} caracteres.")
    if not _NOMBRE_OK.match(limpio):
        raise ValueError("El nombre solo admite letras, números, espacios y los signos . , - + ( ) # °; "
                         "debe empezar con letra o número.")
    return limpio


def validar_juntas(q_deg, limites_deg) -> list[float]:
    """6 números finitos dentro de los límites articulares [°]."""
    if not isinstance(q_deg, (list, tuple)) or len(q_deg) != 6:
        raise ValueError("La pose debe tener exactamente 6 juntas.")
    if not all(_finito(v) for v in q_deg):
        raise ValueError("Las 6 juntas deben ser números finitos.")
    fuera = [i + 1 for i, (v, (lo, hi)) in enumerate(zip(q_deg, limites_deg)) if not lo <= v <= hi]
    if fuera:
        raise ValueError("Fuera de los límites articulares en J" + ", J".join(map(str, fuera)) + ".")
    return [float(v) for v in q_deg]


class AlmacenPoses:
    def __init__(self, ruta: str | None, limites_deg):
        self.ruta = ruta
        self.limites_deg = [tuple(map(float, l)) for l in limites_deg]
        self._lock = threading.Lock()

    @property
    def disponible(self) -> bool:
        return bool(self.ruta)

    # Lectura tolerante
    def _leer(self) -> tuple[list[dict], str | None, bool]:
        """(poses válidas, aviso, archivo_dañado)."""
        if not self.ruta:
            return [], "No hay carpeta de datos configurada para guardar poses (--datos).", False
        try:
            with open(self.ruta, encoding="utf-8") as f:
                datos = json.load(f)
        except FileNotFoundError:
            return [], None, False
        except (OSError, ValueError) as ex:
            return [], f"El archivo de poses guardadas está dañado y se ignora ({type(ex).__name__}).", True
        lista = datos.get("poses") if isinstance(datos, dict) else None
        if not isinstance(lista, list):
            return [], "El archivo de poses guardadas no tiene el formato esperado y se ignora.", True
        poses, descartadas, vistos = [], 0, set()
        for p in lista:
            try:
                poses.append(self._normalizar(p, vistos))
            except (ValueError, TypeError, AttributeError, KeyError):
                descartadas += 1
        aviso = (f"Se ignoraron {descartadas} poses inválidas del archivo (nombre, juntas o límites)."
                 if descartadas else None)
        return poses, aviso, False

    def _normalizar(self, p: dict, vistos: set) -> dict:
        nombre = validar_nombre(p["nombre"])
        if nombre.casefold() in vistos:
            raise ValueError("nombre repetido")
        q = validar_juntas(p["q_deg"], self.limites_deg)
        tcp = p.get("tcp") or {}
        tcp = {k: float(tcp[k]) for k in _CLAVES_TCP if _finito(tcp.get(k))}
        fecha = p.get("fecha") if isinstance(p.get("fecha"), str) else ""
        vistos.add(nombre.casefold())
        return {"nombre": nombre, "q_deg": q, "tcp": tcp, "fecha": fecha[:32]}

    def _escribir(self, poses: list[dict], danado: bool):
        carpeta = os.path.dirname(os.path.abspath(self.ruta))
        os.makedirs(carpeta, exist_ok=True)
        if danado and os.path.exists(self.ruta):
            sello = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
            os.replace(self.ruta, f"{self.ruta}.danado-{sello}")
        fd, tmp = tempfile.mkstemp(prefix=".poses-", suffix=".tmp", dir=carpeta)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump({"version": VERSION, "poses": poses}, f, ensure_ascii=False, indent=2)
                f.write("\n")
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.ruta)
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    def _exigir_disponible(self):
        if not self.ruta:
            raise RuntimeError("No hay carpeta de datos configurada para guardar poses (--datos).")

    @staticmethod
    def _buscar(poses, nombre):
        clave = nombre.casefold()
        return next((i for i, p in enumerate(poses) if p["nombre"].casefold() == clave), None)

    # API
    def listar(self) -> dict:
        with self._lock:
            poses, aviso, _ = self._leer()
        return {"poses": poses, "aviso": aviso, "ruta": self.ruta, "disponible": self.disponible}

    def obtener(self, nombre) -> dict:
        nombre = validar_nombre(nombre)
        with self._lock:
            poses, _, _ = self._leer()
        i = self._buscar(poses, nombre)
        if i is None:
            raise ValueError(f"No existe la pose «{nombre}».")
        return poses[i]

    def guardar(self, nombre, q_deg, tcp: dict, *, reemplazar=False) -> dict:
        self._exigir_disponible()
        nombre = validar_nombre(nombre)
        q = validar_juntas(list(q_deg), self.limites_deg)
        tcp = {k: round(float(tcp[k]), 4) for k in _CLAVES_TCP if _finito(tcp.get(k))}
        pose = {"nombre": nombre, "q_deg": [round(v, 6) for v in q], "tcp": tcp,
                "fecha": dt.datetime.now().isoformat(timespec="seconds")}
        with self._lock:
            poses, _, danado = self._leer()
            i = self._buscar(poses, nombre)
            if i is not None and not reemplazar:
                raise ValueError(f"Ya existe una pose llamada «{poses[i]['nombre']}». Elegí otro nombre.")
            if i is not None:
                poses[i] = pose
            else:
                if len(poses) >= MAX_POSES:
                    raise ValueError(f"Se alcanzó el máximo de {MAX_POSES} poses guardadas. Borrá alguna.")
                poses.append(pose)
            self._escribir(poses, danado)
        return pose

    def renombrar(self, nombre, nuevo) -> dict:
        self._exigir_disponible()
        nombre, nuevo = validar_nombre(nombre), validar_nombre(nuevo)
        with self._lock:
            poses, _, danado = self._leer()
            i = self._buscar(poses, nombre)
            if i is None:
                raise ValueError(f"No existe la pose «{nombre}».")
            j = self._buscar(poses, nuevo)
            if j is not None and j != i:
                raise ValueError(f"Ya existe una pose llamada «{poses[j]['nombre']}».")
            poses[i] = {**poses[i], "nombre": nuevo}
            self._escribir(poses, danado)
            return poses[i]

    def borrar(self, nombre) -> dict:
        self._exigir_disponible()
        nombre = validar_nombre(nombre)
        with self._lock:
            poses, _, danado = self._leer()
            i = self._buscar(poses, nombre)
            if i is None:
                raise ValueError(f"No existe la pose «{nombre}».")
            borrada = poses.pop(i)
            self._escribir(poses, danado)
            return borrada
