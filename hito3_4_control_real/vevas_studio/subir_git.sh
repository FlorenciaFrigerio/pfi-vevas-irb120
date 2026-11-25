#!/usr/bin/env bash
# subir_git.sh — sube esta carpeta a hito3_4_control_real/vevas_studio/ del repo del grupo en la rama 19NOV
# y después la fusiona en main. Script histórico del 19/11 (la carpeta se llamaba hito3_4/ hasta la
# reorganización por hitos); DESTINO=otra/ruta ./subir_git.sh cambia la carpeta de destino.
#   ./subir_git.sh               (pregunta antes de tocar main)
#   REPO=~/otra/ruta ./subir_git.sh   si ya tenés el repo clonado en otro lado
set -euo pipefail

ORIGEN="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URL="https://github.com/FlorenciaFrigerio/pfi-vevas-irb120.git"
REPO="${REPO:-$HOME/pfi-vevas-irb120}"
RAMA="19NOV"
DESTINO="${DESTINO:-hito3_4_control_real/vevas_studio}"

if [ ! -d "$REPO/.git" ]; then
  echo "== Clonando el repo en $REPO"
  git clone "$URL" "$REPO"
fi
cd "$REPO"

if [ -n "$(git status --porcelain)" ]; then
  echo "El repo en $REPO tiene cambios sin guardar. Guardalos o descartalos y volvé a correr el script."
  git status --short
  exit 1
fi

echo "== Actualizando main"
git fetch origin
git checkout main
git pull --ff-only origin main

echo "== Rama $RAMA"
if git show-ref --verify --quiet "refs/remotes/origin/$RAMA"; then
  git checkout -B "$RAMA" "origin/$RAMA"
else
  git checkout -B "$RAMA"
fi

echo "== Copiando la carpeta del prototipo a $DESTINO/ (sin GO_VEVAS.json, sesiones ni cachés)"
mkdir -p "$DESTINO"
tar -C "$ORIGEN" \
  --exclude=./.git --exclude=./sesiones --exclude=./GO_VEVAS.json \
  --exclude='__pycache__' --exclude='.pytest_cache' --exclude='*.pyc' --exclude='*.log' \
  -cf - . | tar -C "$DESTINO" -xf -

git add "$DESTINO"
if git diff --cached --quiet; then
  echo "No hay cambios para subir."
else
  git commit -m "feat(hito3_4): movimiento real por RWS e interfaz con Hito 2 en vivo (19/11)

- VEVAS_RWS.mod + cliente/backend RWS (--modo rws), herramientas/rws_irc5.py (mover, diagnostico, texto)
- iniciar_real.sh de un comando con esperas y cierre seguro; crear_go.py
- revisión previa (singularidades, límites, velocidad, colisiones aproximadas) y defectos C01-C04
- web: arrastre en 3D con IK de Pieper en vivo, verificación FK, sigma min, volver a cero
- documentación: CAMBIOS_19NOV.md y AVANCES_CONTEXTO.md"
fi

echo "== Subiendo la rama $RAMA"
git push -u origin "$RAMA"

read -r -p "¿Fusionar $RAMA en main y subir main? (s/n): " R
if [ "$R" = "s" ] || [ "$R" = "S" ]; then
  git checkout main
  git pull --ff-only origin main
  git merge --no-ff "$RAMA" -m "Merge rama $RAMA: Hito 3/4 del 19/11"
  git push origin main
  echo "Listo: $RAMA subida y fusionada en main."
else
  echo "Listo: $RAMA subida. main no se tocó."
fi
echo "Ver: https://github.com/FlorenciaFrigerio/pfi-vevas-irb120/tree/$RAMA/$DESTINO"
