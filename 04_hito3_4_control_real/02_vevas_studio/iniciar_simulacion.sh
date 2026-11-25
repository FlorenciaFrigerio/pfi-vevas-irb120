#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ ! -x .venv/bin/python ]]; then
  echo "Primero crea el entorno: python3 -m venv .venv"
  echo "Después instala: .venv/bin/python -m pip install -r requirements.txt"
  exit 1
fi
printf '\nVEVAS Studio: prototipo en SIMULACIÓN, sin conexión física.\n'
exec .venv/bin/python consola_vevas.py --modo sim --puerto 9012 --abrir
