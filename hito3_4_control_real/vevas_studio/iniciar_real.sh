#!/usr/bin/env bash
# Consulta el ABB por RWS y abre la consola VEVAS y RViz.
set -uo pipefail

AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$AQUI" || exit 1
IP="192.168.125.1"
RWS_PUERTO=80
WS="$HOME/abb_ws"
GO="$AQUI/GO_VEVAS.json"
MODO=control
MANUAL=0
SI=0
RECARGAR=0
RVIZ=1
VEVAS=1

ayuda() {
  cat <<'TXT'
Uso: ./iniciar_real.sh [opciones]
  --solo-chequeo      Consulta el estado una vez y sale.
  --solo-lectura      Abre VEVAS y RViz en modo de visualización.
  --ip IP             Dirección del IRC5.
  --rws-puerto N       Puerto RWS.
  --sin-rviz          Omite el nodo ROS y RViz.
  --sin-vevas         Omite la consola web.
  --ws RUTA           Workspace ROS 2 opcional.
  --go ARCHIVO        GO de la sesión para iniciar control.
  -y, --si           Omite Enter si ya existe un GO válido.
  --auto, --manual   Selecciona el modo de control esperado.
  --recargar-modulo  Solicita recarga al iniciar control.
TXT
}

fallo() { printf '%s\n' "$*" >&2; exit 1; }

while [ "$#" -gt 0 ]; do
  case "$1" in
    --solo-chequeo|--solo-lectura)
      [ "$MODO" = control ] || fallo "Elegí un solo modo de lectura."
      MODO="${1#--solo-}"
      ;;
    --auto) MANUAL=0;;
    --manual) MANUAL=1;;
    -y|--si) SI=1;;
    --recargar-modulo) RECARGAR=1;;
    --sin-rviz) RVIZ=0;;
    --sin-vevas) VEVAS=0;;
    --ip|--rws-puerto|--ws|--go)
      [ "$#" -ge 2 ] && [[ "$2" != --* ]] || fallo "Falta un valor para $1."
      case "$1" in
        --ip) IP="$2";;
        --rws-puerto) RWS_PUERTO="$2";;
        --ws) WS="$2";;
        --go) GO="$2";;
      esac
      shift
      ;;
    -h|--help) ayuda; exit 0;;
    *) printf 'Opción desconocida: %s\n' "$1" >&2; exit 2;;
  esac
  shift
done

RWS() {
  python3 "$AQUI/herramientas/rws_irc5.py" --ip "$IP" --puerto "$RWS_PUERTO" "$@"
}

RWS_control() { RWS --go "$GO" "$@"; }

PIDS=()
RAPID_PROPIO=0
limpiar() {
  local pid
  for pid in "${PIDS[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  if [ "$RAPID_PROPIO" = 1 ]; then
    if RWS parar; then
      RWS_control pp-main || true
    else
      echo "No se pudo confirmar la parada de RAPID. Verificá el controlador en el FlexPendant." >&2
    fi
  fi
}
trap limpiar EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

validar_go() {
  python3 - "$GO" <<'PY'
import json
import sys

from vevas.consola import validar_go

try:
    with open(sys.argv[1], encoding="utf-8") as archivo:
        go = json.load(archivo)
except (OSError, ValueError) as exc:
    sys.exit(f"No se pudo leer el archivo GO: {exc}")

ok, mensaje = validar_go(go)
if not ok:
    sys.exit(mensaje)
PY
}

if [ "$MODO" = control ]; then
  validar_go || fallo "Control bloqueado. Usá --solo-lectura para visualizar el robot."
  python3 -c 'import sys; from vevas.backend_rws import BackendRWS; sys.exit(0 if BackendRWS.velocidad_articular_verificada is True else 1)' ||
    fallo "Control bloqueado: el backend RWS no verifica la velocidad articular real. Usá --solo-lectura para visualizar el robot."
  if [ "$SI" = 0 ]; then
    read -r -p "GO válido. Enter para continuar (Ctrl+C cancela): " _ ||
      fallo "Entrada cerrada. Se canceló el arranque."
  fi
fi

if [ "$MODO" = chequeo ]; then
  ping -c 1 -W 2 "$IP" >/dev/null 2>&1 || fallo "El IRC5 no responde al ping en $IP."
else
  until ping -c 1 -W 2 "$IP" >/dev/null 2>&1; do
    echo "Esperando ping del IRC5 en $IP (revisá la conexión al puerto SERVICE)."
    sleep 3
  done
fi

refrescar() {
  local salida clave valor
  OPMODE="" CTRLSTATE="" EXEC="" PP_MODULO="" PP_RUTINA="" MODULO_VEVAS="" Q_DEG=""
  if ! salida="$(RWS estado --shell)"; then
    return 1
  fi
  while IFS='=' read -r clave valor; do
    case "$clave" in
      OPMODE) OPMODE="$valor";;
      CTRLSTATE) CTRLSTATE="$valor";;
      EXEC) EXEC="$valor";;
      PP_MODULO) PP_MODULO="$valor";;
      PP_RUTINA) PP_RUTINA="$valor";;
      MODULO_VEVAS) MODULO_VEVAS="$valor";;
      Q_DEG) Q_DEG="$valor";;
    esac
  done <<< "$salida"
  [ -n "$OPMODE" ] && [ -n "$CTRLSTATE" ] && [ -n "$Q_DEG" ]
}

if [ "$MODO" = chequeo ]; then
  refrescar || fallo "No se pudo consultar el estado por RWS en $IP:$RWS_PUERTO."
else
  until refrescar; do
    echo "Esperando respuesta de RWS en $IP:$RWS_PUERTO."
    sleep 3
  done
fi

printf 'Modo=%s  estado=%s  RAPID=%s  módulo VEVAS_RWS=%s  PP=%s/%s\n' \
  "$OPMODE" "$CTRLSTATE" "$EXEC" "$MODULO_VEVAS" "$PP_MODULO" "$PP_RUTINA"
printf 'Juntas [°] = %s\n' "$Q_DEG"
if [ "$MODO" = chequeo ]; then
  echo "Chequeo de lectura terminado."
  exit 0
fi

if [ "$MODO" = control ]; then
  case "$CTRLSTATE" in motorson|motorsoff) ;; *) fallo "El estado $CTRLSTATE no permite iniciar el control.";; esac
  if [ "$EXEC" = running ] && [ "$PP_MODULO/$PP_RUTINA" != VEVAS_RWS/VevasServidor ]; then
    fallo "RAPID está ejecutando otro programa ($PP_MODULO/$PP_RUTINA)."
  fi
  validar_go || fallo "El GO ya no es válido. Se canceló el arranque."
  if [ "$MANUAL" = 0 ]; then
    [ "$OPMODE" = AUTO ] || fallo "El control automático requiere modo AUTO."
    if [ "$EXEC" != running ]; then
      if [ "$RECARGAR" = 1 ]; then
        RWS_control instalar --recargar || exit 1
      else
        RWS_control instalar || exit 1
      fi
      echo "Confirmá «Iniciar» en el FlexPendant cuando lo solicite VevasServidor."
      RWS_control arrancar || exit 1
      RAPID_PROPIO=1
    fi
  else
    [ "$OPMODE" = MANR ] || fallo "El control manual requiere manual reducido."
    [ "$MODULO_VEVAS" = si ] || fallo "Falta el módulo VEVAS_RWS en el controlador."
    echo "Poné PP en VEVAS_RWS/VevasServidor, pulsá Start y confirmá «Iniciar» en el FlexPendant."
  fi
  RWS servidor --espera 35 || exit 1
fi

iniciar_rviz() {
  local setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
  if [ -f "$setup" ]; then
    set +u
    source "$setup"
    if [ -f "$WS/install/setup.bash" ]; then
      source "$WS/install/setup.bash"
    fi
    set -u
  fi
  if ! command -v ros2 >/dev/null ||
     ! ros2 pkg prefix robot_state_publisher >/dev/null 2>&1 ||
     ! ros2 pkg prefix rviz2 >/dev/null 2>&1; then
    echo "RViz no disponible. Se necesitan ROS 2, robot_state_publisher y rviz2." >&2
    return 1
  fi

  local log_nodo log_rviz nodo
  mkdir -p "$AQUI/sesiones" || return 1
  log_nodo="$(mktemp "$AQUI/sesiones/nodo_rws_XXXXXXXX.log")" || return 1
  log_rviz="$(mktemp "$AQUI/sesiones/rviz_rws_XXXXXXXX.log")" || return 1
  python3 "$AQUI/herramientas/nodo_rws_ros.py" --ip "$IP" --rws-puerto "$RWS_PUERTO" --solo-lectura >"$log_nodo" 2>&1 &
  nodo=$!
  PIDS+=("$nodo")
  if ! timeout 15s ros2 topic echo --once /joint_states sensor_msgs/msg/JointState >/dev/null 2>&1 ||
     ! kill -0 "$nodo" 2>/dev/null; then
    echo "No llegó un mensaje de /joint_states. Revisá $log_nodo." >&2
    return 1
  fi
  ros2 launch "$AQUI/herramientas/rviz_rws.launch.py" >"$log_rviz" 2>&1 &
  PIDS+=("$!")
  echo "RViz iniciado en modo visualización. Registro: $log_rviz"
}

RVIZ_LISTO=0
if [ "$RVIZ" = 1 ] && iniciar_rviz; then
  RVIZ_LISTO=1
fi

if [ "$VEVAS" = 1 ]; then
  OPCIONES=(--solo-lectura)
  if [ "$MODO" = control ]; then
    OPCIONES=(--real --go "$GO")
  fi
  python3 "$AQUI/consola_vevas.py" --modo rws --ip "$IP" --rws-puerto "$RWS_PUERTO" --abrir "${OPCIONES[@]}" &
  PIDS+=("$!")
  wait "${PIDS[-1]}"
elif [ "$RVIZ_LISTO" = 1 ]; then
  wait "${PIDS[-1]}"
elif [ "$RVIZ" = 1 ]; then
  fallo "RViz no pudo iniciarse."
fi
