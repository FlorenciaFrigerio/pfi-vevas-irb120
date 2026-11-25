# Llevar esta revisión a Ubuntu y subirla a una rama

La revisión se preparó en el entorno de desarrollo, no en la PC del laboratorio.
El archivo entregado `VEVAS_20NOV.patch` contiene un commit sobre la base
`312cd96418a469f327e692029b776d8656cf7a17` del repo de Florencia. Incluye código,
documentación, pruebas y captura de SIM. No incluye un GO completado.

## Publicación pendiente

El complemento GitHub y el Git HTTPS autenticado devolvieron HTTP 403 al intentar publicar el 20/11. No se creó la rama remota. Aplicar el parche y subir desde Ubuntu con los comandos de esta guía.

## 1. Descargar

Guardar el parche adjunto del chat con el nombre exacto `VEVAS_20NOV.patch` en
`/home/huesero/Descargas`. Antes de ejecutar, comprobar:

```bash
ls -lh "$HOME/Descargas/VEVAS_20NOV.patch"
```

Si el archivo no está, resolver la descarga antes de continuar. La carpeta
`/workspace` de la sesión de desarrollo no es el disco del Ubuntu del laboratorio.

## 2. Aplicar en una copia nueva

Este bloque se detiene si falla algún paso. Si la carpeta destino ya existe,
se conserva y hay que elegir otro nombre; no borrarla para repetir a ciegas.

```bash
(
set -e
mkdir -p "$HOME/Descargas/ROBOTICA"
git clone https://github.com/FlorenciaFrigerio/pfi-vevas-irb120.git "$HOME/Descargas/ROBOTICA/vevas-revision-20nov"
cd "$HOME/Descargas/ROBOTICA/vevas-revision-20nov"
git switch -c revision-20nov 312cd96418a469f327e692029b776d8656cf7a17
git am "$HOME/Descargas/VEVAS_20NOV.patch"
git status --short
git log -1 --oneline
)
```

Si `git am` informa conflicto, detenerse y conservar la salida; no usar `--skip`
ni sobrescribir archivos. La copia original permanece disponible para comparar.

## 3. Publicar la rama

Después de comprobar que la aplicación terminó bien:

```bash
cd "$HOME/Descargas/ROBOTICA/vevas-revision-20nov"
git push -u origin revision-20nov
```

La rama se sube al repo de Florencia. Revisar el diff en GitHub antes de fusionar
a `main`. Para esta entrega, usar este flujo en lugar del `subir_git.sh` histórico,
que copia una carpeta del prototipo a la rama `19NOV`.

## 4. Probar en SIM

```bash
cd "$HOME/Descargas/ROBOTICA/vevas-revision-20nov/hito3_4"
python3 -m venv .venv_interfaz
.venv_interfaz/bin/python -m pip install -r requirements.txt
.venv_interfaz/bin/python consola_vevas.py --modo sim --puerto 9012 --abrir
```

Seguir [03_PRUEBA_INTERFAZ_20NOV.md](../06_operacion/03_PRUEBA_INTERFAZ_20NOV.md). La siguiente etapa
con el brazo es solo lectura y comparación; el envío físico sigue bloqueado por
la verificación pendiente de velocidad por junta.
