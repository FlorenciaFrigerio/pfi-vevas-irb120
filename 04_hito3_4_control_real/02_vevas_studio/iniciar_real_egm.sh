#!/usr/bin/env bash
# LEGADO (ruta EGM + driver abb_ros2, que nunca conectó). La ruta vigente es iniciar_real.sh (RWS + VEVAS_RWS). Usa 02_herramientas/rws_irc5_egm.py.
# iniciar_real.sh — levanta TODO con un comando y ESPERA a que lo que falte se resuelva (no se cierra por un problema).
#
#   ./iniciar_real.sh                 # espera AUTO si hace falta y arranca todo solo (sin tocar el FlexPendant)
#   ./iniciar_real.sh --manual        # llave en MANR: prepara todo y espera tu Start en el FlexPendant
#   ./iniciar_real.sh -y              # igual, sin pedir ni Enter
#   ./iniciar_real.sh --solo-chequeo  # una pasada de chequeos, sin esperar y sin escribir en el controlador
#
# Opciones: --ip IP  --go archivo.json  --recargar-modulo  --sin-rviz  --sin-vevas  --ws ~/abb_ws
# Cada problema se explica con qué hacer; el script reintenta solo. Ctrl+C cierra todo en orden.
# NO cambia el modo de operación (llave física) ni puentea paros o vallas.
set -uo pipefail

AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$AQUI" || exit 1
IP="192.168.125.1"; PC_IP_ESPERADA="192.168.125.10"; WS="$HOME/abb_ws"; GO="$AQUI/GO_VEVAS.json"
MANUAL=0; SI=0; SOLO_CHEQUEO=0; RECARGAR=0; RVIZ=1; VEVAS=1; RWS_PUERTO=80
while [ $# -gt 0 ]; do
  case "$1" in
    --auto) MANUAL=0;; -y|--si) SI=1;; --manual) MANUAL=1;; --solo-chequeo) SOLO_CHEQUEO=1;; --recargar-modulo) RECARGAR=1;;
    --sin-rviz) RVIZ=0;; --sin-vevas) VEVAS=0;;
    --ip) IP="$2"; shift;; --go) GO="$2"; shift;; --ws) WS="$2"; shift;; --rws-puerto) RWS_PUERTO="$2"; shift;;
    -h|--help) sed -n '2,9p' "$0"; exit 0;;
    *) echo "Opción desconocida: $1"; exit 2;;
  esac; shift
done
AUTO=$((1 - MANUAL))

R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; B=$'\e[1m'; N=$'\e[0m'
paso()  { echo; echo "${B}== $* ==${N}"; }
ok()    { echo "  ${G}✓${N} $*"; }
aviso() { echo "  ${Y}!${N} $*"; }
fallo() { echo "  ${R}✗ $*${N}"; salir_con 1; }
RWS() { python3 "$AQUI/02_herramientas/rws_irc5_egm.py" --ip "$IP" --puerto "$RWS_PUERTO" "$@"; }

PIDS=(); RAPID_ARRANCADO=0; LIMPIO=0; DRV=""; LOG=""; MOTIVO=""; AYUDA=""; INTERVALO=3
limpiar() {
  [ "$LIMPIO" = 1 ] && return; LIMPIO=1
  echo; paso "Cerrando"
  for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill -INT "$p" 2>/dev/null; done
  sleep 2
  for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill -TERM "$p" 2>/dev/null; done
  sleep 1
  for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill -KILL "$p" 2>/dev/null; done
  if [ "$RAPID_ARRANCADO" = 1 ]; then
    RWS parar && ok "RAPID detenido y motores OFF" || aviso "No se pudo confirmar el stop de RAPID: pulsá Stop en el FlexPendant."
    RWS pp-main >/dev/null 2>&1 && ok "Puntero de programa de vuelta en main" || true
  elif [ "$MANUAL" = 1 ] && [ "$SOLO_CHEQUEO" = 0 ]; then
    aviso "En el FlexPendant: Stop, y Debug → PP to Main para volver al programa original."
  fi
}
salir_con() { limpiar; exit "${1:-0}"; }
trap 'echo; echo "Interrumpido."; salir_con 130' INT TERM

# espera "titulo" "qué hacer" condición...   → reintenta hasta que la condición dé 0.
# La condición deja el motivo del fallo en $MOTIVO. En --solo-chequeo no espera: informa y sale.
espera() {
  local titulo="$1" ayuda="$2"; shift 2
  local n=0 t0=$SECONDS
  until "$@"; do
    if [ "$SOLO_CHEQUEO" = 1 ]; then fallo "$titulo: $MOTIVO"; fi
    if [ $((n % 5)) -eq 0 ]; then
      echo "  ${Y}⏳ Esperando — $titulo${N} ($((SECONDS - t0)) s)"
      [ -n "$MOTIVO" ] && echo "     Motivo: $MOTIVO" | head -4
      [ -n "$ayuda" ] && printf '%s\n' "$ayuda" | sed 's/^/     → /'
    fi
    n=$((n + 1)); sleep "$INTERVALO"
  done
  [ "$n" -gt 0 ] && ok "$titulo: resuelto tras $((SECONDS - t0)) s"
  return 0
}
refrescar() { local o; if o="$(RWS estado --shell 2>&1)"; then eval "$o"; return 0; fi; MOTIVO="$o"; return 1; }

# ------------------------------------------------------------------ 1. entorno (no se arregla esperando)
paso "1. Entorno"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
[ -f "$ROS_SETUP" ] || fallo "No encuentro $ROS_SETUP (¿está instalado ROS 2 Jazzy?)"
# shellcheck disable=SC1090
set +u; source "$ROS_SETUP"; [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"; set -u
command -v ros2 >/dev/null || fallo "ros2 no está en el PATH"
ros2 pkg prefix abb_bringup >/dev/null 2>&1 || fallo "abb_bringup no está en $WS. Compilá: cd $WS && colcon build (o usá --ws)."
[ -f "$AQUI/EGM_ROS.mod" ] || fallo "Falta $AQUI/EGM_ROS.mod"
export LC_NUMERIC=C
python3 -c "import vevas.consola" 2>/dev/null || fallo "No se pudo importar vevas.consola (corré el script desde la carpeta del prototipo)"
ok "ROS 2, abb_bringup, EGM_ROS.mod y código de VEVAS disponibles"
SUDO_OK=0
if [ "$SOLO_CHEQUEO" = 0 ]; then
  echo "  Se pide la clave de sudo UNA vez: abrir el puerto UDP 6511 en el firewall y diagnosticar la red EGM."
  if sudo -v; then SUDO_OK=1; ( while kill -0 $$ 2>/dev/null; do sudo -n true 2>/dev/null; sleep 50; done ) & PIDS+=("$!"); ok "sudo disponible"
  else aviso "Sin sudo: no podré abrir el firewall ni capturar paquetes si el driver no conecta."; fi
fi
SU() { [ "$SUDO_OK" = 1 ] && sudo -n "$@"; }

# ------------------------------------------------------------------ 2. red
paso "2. Comunicación con el IRC5 ($IP)"
c_ping() { ping -c 1 -W 2 "$IP" >/dev/null 2>&1 && return 0; MOTIVO="El IRC5 no responde al ping."; return 1; }
espera "ping a $IP" "Revisá: IRC5 encendido, cable en el puerto SERVICE, y la PC con IP $PC_IP_ESPERADA/24 (nmcli connection up abb-service)." c_ping
ok "ping a $IP responde"
IFAZ="$(ip route get "$IP" 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p' | head -1)"
c_ip() {
  ip -4 addr 2>/dev/null | grep -q "inet $PC_IP_ESPERADA/" && return 0
  if [ "$SUDO_OK" = 1 ] && [ -n "$IFAZ" ]; then
    SU ip addr add "$PC_IP_ESPERADA/24" dev "$IFAZ" 2>/dev/null
    ip -4 addr 2>/dev/null | grep -q "inet $PC_IP_ESPERADA/" && { echo "    Agregué $PC_IP_ESPERADA a $IFAZ (además de la IP que ya tenía; se borra al reiniciar)."; return 0; }
  fi
  MOTIVO="La PC no tiene $PC_IP_ESPERADA (interfaz $IFAZ). El IRC5 envía el EGM a esa IP."; return 1
}
espera "IP $PC_IP_ESPERADA en la PC" "Ejecutá en otra terminal: sudo ip addr add $PC_IP_ESPERADA/24 dev ${IFAZ:-eno2}" c_ip
ok "La PC tiene la IP $PC_IP_ESPERADA (destino del EGM) en $IFAZ"

# ------------------------------------------------------------------ 3. RWS y estado
paso "3. Estado del controlador por RWS (solo lectura)"
c_rws() { refrescar; }
espera "RWS responde" "Revisá que el controlador haya terminado de arrancar, que la opción PC Interface esté activa y las credenciales (Default User / robotics)." c_rws
ok "RWS responde"
c_modo() {
  refrescar || return 1
  if [ "$MANUAL" = 0 ] && [ "$OPMODE" != "AUTO" ]; then MOTIVO="El controlador está en $OPMODE y se necesita AUTO."; return 1; fi
  return 0
}
espera "controlador en AUTO" "Una persona debe girar la llave del IRC5 a AUTO y confirmar en el FlexPendant si lo pide. (Para operar con el FlexPendant: ./iniciar_real.sh --manual)" c_modo
c_seguridad() {
  refrescar || return 1
  case "$CTRLSTATE" in
    guardstop) MOTIVO="Guard stop: valla/puerta abierta o auto stop activo."; return 1;;
    emergencystop|emergencystopreset) MOTIVO="Paro de emergencia activo o sin rearmar."; return 1;;
    sysfail) MOTIVO="El controlador está en falla de sistema (SYS FAIL)."; return 1;;
  esac
  return 0
}
espera "cadena de seguridad cerrada" "Cerrá la valla/puerta, soltá y rearmá el paro de emergencia y pulsá Motors On/reset en el controlador. Este script no puentea ninguna protección." c_seguridad
c_rapid_libre() {
  refrescar || return 1
  if [ "$EXEC" = "running" ] && [ "$PP_RUTINA" != "ros_egm" ]; then MOTIVO="RAPID está ejecutando otro programa ($PP_RUTINA)."; return 1; fi
  return 0
}
espera "RAPID libre" "Detené el programa que está corriendo (FlexPendant → Stop)." c_rapid_libre
echo "    modo=$OPMODE  motores=$CTRLSTATE  RAPID=$EXEC  módulo EGM_ROS=$MODULO_EGM  PP=$PP_RUTINA"
echo "    juntas [°] = $Q_DEG"
IFS=',' read -r _ _ _ _ Q5 _ <<< "$Q_DEG"
python3 -c "import sys; sys.exit(0 if abs(float('$Q5')) > 10 else 1)" || aviso "El eje 5 está cerca de 0°: la muñeca está en singularidad. Llevalo a ~30–40° antes de operar."
ok "Controlador listo en modo $OPMODE"

# ------------------------------------------------------------------ 4. GO (se genera solo al confirmar)
c_go() {
  [ -f "$GO" ] || { MOTIVO="No existe $GO."; return 1; }
  local r
  if r="$(python3 - "$GO" 2>&1 <<'PY'
import json, sys
from vevas.consola import validar_go
try:
    d = json.load(open(sys.argv[1], encoding="utf-8"))
except Exception as e:
    print("JSON ilegible: %s" % e); sys.exit(1)
ok, msg = validar_go(d)
print(msg); sys.exit(0 if ok else 1)
PY
)"; then return 0; fi
  MOTIVO="$r"; return 1
}

if [ "$SOLO_CHEQUEO" = 1 ]; then paso "Chequeo terminado: todo en orden (no se escribió nada en el controlador)"; salir_con 0; fi

# ------------------------------------------------------------------ 5. confirmación
paso "5. Confirmación"
if [ "$AUTO" = 1 ]; then
  cat <<TXT
  ${R}${B}MODO AUTO${N}: nadie sostiene el enabling device y el IRC5 no limita el TCP a 250 mm/s.
  La limitación viene de VEVAS (tope del GO) y de MaxSpeedDeviation=10 °/s en EGM_ROS.
  Hace falta: célula despejada, una persona con la mano en el paro de emergencia y autorización del docente.
TXT
  echo "  Al escribir la frase confirmás: autorización del docente, zona despejada, paro de emergencia a mano,"
  echo "  sin herramienta ni objetos dentro del alcance. Velocidad máxima ${VEVAS_VEL_PCT:-5} % (VEVAS_VEL_PCT=… para cambiarla)."
  FRASE="MOVER ROBOT REAL EN AUTO"
else
  echo "  Modo $OPMODE: vos pulsás Start en el FlexPendant (enabling device apretado). Célula despejada y paro a mano."
  FRASE="EMPEZAR"
fi
if [ "$SI" = 1 ]; then ok "Confirmado con -y"
else read -r -p "  Enter para continuar (Ctrl+C cancela): " _ || true; fi
if c_go; then ok "Se usa el GO de hoy que ya existía ($GO)"
else python3 "$AQUI/02_herramientas/crear_go.py" --automatico --salida "$GO" >/dev/null && c_go && ok "GO y entorno registrados automáticamente (velocidad ${VEVAS_VEL_PCT:-5} %)" || fallo "No se pudo generar el GO: $MOTIVO"; fi

# ------------------------------------------------------------------ 6. limpieza previa
paso "6. Limpieza previa"
if pgrep -f ros2_control_node >/dev/null; then aviso "Había un ros2_control_node anterior: se cierra."; pkill -INT -f ros2_control_node; sleep 3; pkill -KILL -f ros2_control_node 2>/dev/null; fi
if pgrep -f "abb_moveit.launch|move_group" >/dev/null; then aviso "Había MoveIt anterior: se cierra."; pkill -INT -f "abb_moveit.launch|move_group"; sleep 2; fi
c_puerto() { ss -ulpn 2>/dev/null | grep -q ':6511 ' && { MOTIVO="Otro proceso usa UDP 6511: $(ss -ulpn 2>/dev/null | grep ':6511 ' | head -1)"; return 1; }; return 0; }
espera "puerto UDP 6511 libre" "Cerrá ese proceso (ss -ulpn | grep 6511; kill <pid>)." c_puerto
ok "Puerto UDP 6511 libre y sin procesos viejos"
if command -v ufw >/dev/null && [ "$SUDO_OK" = 1 ]; then
  if SU ufw status 2>/dev/null | grep -qi "Status: active"; then
    if SU ufw status 2>/dev/null | grep -q "6511"; then ok "Firewall activo con el puerto UDP 6511 abierto"
    else SU ufw allow in proto udp from "$IP" to any port 6511 >/dev/null && ok "Firewall: abrí UDP 6511 desde $IP" || aviso "No pude abrir UDP 6511 en ufw"; fi
  else ok "Firewall (ufw) inactivo: no bloquea EGM"; fi
fi

# ------------------------------------------------------------------ 7. programa en el controlador
paso "7. Módulo EGM_ROS y puntero de programa"
c_preparado() {
  refrescar || return 1
  if [ "$MODULO_EGM" = "si" ] && [ "$PP_RUTINA" = "ros_egm" ] && [ "$RECARGAR" = 0 ]; then return 0; fi
  local o rc
  if [ "$RECARGAR" = 1 ]; then o="$(RWS preparar --recargar 2>&1)"; rc=$?; else o="$(RWS preparar 2>&1)"; rc=$?; fi
  if [ "$rc" = 0 ]; then echo "$o" | sed 's/^/    /'; RECARGAR=0; return 0; fi
  MOTIVO="$o"; return 1
}
INTERVALO=15; espera "EGM_ROS cargado y puntero en ros_egm" "Si el script no logra hacerlo por RWS: FlexPendant → Program Editor → Modules → Load Module → EGM_ROS.mod, y luego Debug → PP to Routine… → ros_egm (sin pulsar Start). El script lo detecta solo." c_preparado; INTERVALO=3
ok "El controlador tiene EGM_ROS y el puntero en ros_egm"

if [ "$AUTO" = 1 ]; then
  c_arrancar() {
    local o; if o="$(RWS arrancar 2>&1)"; then echo "$o" | sed 's/^/    /'; return 0; fi
    MOTIVO="$o"; return 1
  }
  INTERVALO=5; espera "RAPID arrancado en AUTO" "Revisá el motivo: la llave debe seguir en AUTO, la cadena de seguridad cerrada y el puntero en ros_egm." c_arrancar; INTERVALO=3
  RAPID_ARRANCADO=1
  ok "RAPID corriendo en AUTO (esperando a ROS; el robot no se mueve)"
else
  echo; echo "  ${B}En el FlexPendant:${N} motores ON, enabling device apretado, y pulsá ${B}Start${N} (el puntero ya está en ros_egm)."
  c_running() { refrescar || return 1; [ "$EXEC" = "running" ] && return 0; MOTIVO="RAPID todavía no está en ejecución."; return 1; }
  espera "Start en el FlexPendant" "Motores ON, enabling device apretado y botón Start." c_running
  ok "RAPID en ejecución: esperando a ROS"
fi

# ------------------------------------------------------------------ 8. driver
paso "8. Driver ABB (ros2_control + EGM)"
mkdir -p "$AQUI/sesiones"
lanzar_driver() {
  LOG="$AQUI/sesiones/driver_$(date +%F_%H%M%S).log"
  ros2 launch abb_bringup abb_control.launch.py \
    description_package:=abb_irb120_support description_file:=irb120_3_58.xacro \
    moveit_config_package:=abb_irb120_3_58_moveit_config \
    use_fake_hardware:=false rws_ip:="$IP" launch_rviz:=false >"$LOG" 2>&1 &
  DRV=$!; PIDS+=("$DRV"); echo "    driver lanzado (log: $LOG)"
}
lanzar_driver
DIAG_HECHO=0; T_DRV0=$SECONDS
diagnostico_egm() {
  echo "  ${B}Diagnóstico automático del enlace EGM${N}"
  local sock=0 pkts="?"
  grep -qi ":196F " /proc/net/udp /proc/net/udp6 2>/dev/null && sock=1
  [ "$sock" = 1 ] && echo "     · Socket UDP 6511 abierto en la PC: sí" || echo "     · Socket UDP 6511 abierto en la PC: NO"
  if [ "$SUDO_OK" = 1 ] && command -v tcpdump >/dev/null; then
    pkts="$(SU timeout 4 tcpdump -ni any "udp port 6511" -c 5 2>/dev/null | grep -c 'UDP' || true)"
    echo "     · Paquetes EGM del IRC5 en 4 s: $pkts"
  fi
  refrescar >/dev/null 2>&1 && echo "     · RAPID: $EXEC en $PP_RUTINA, motores $CTRLSTATE, modo $OPMODE"
  echo "  ${B}Conclusión:${N}"
  if [ "$sock" = 0 ]; then echo "     → El driver no abrió el puerto: mirá el log $LOG (se relanza solo)."
  elif [ "${EXEC:-}" != "running" ]; then echo "     → RAPID no está corriendo ros_egm: sin eso el IRC5 no envía EGM."
  elif [ "$pkts" = "0" ]; then echo "     → No llega nada: en el FlexPendant, Configuration → Communication → Transmission Protocol → ROB_1 debe tener Remote Address 192.168.125.10 y puerto 6511 (y reinicio). Revisá también que la PC tenga 192.168.125.10."
  elif [ "$pkts" != "?" ]; then echo "     → Llegan paquetes y el socket está abierto pero no conecta: reiniciá ros_egm (Stop/Start) — el script lo hace solo en AUTO."
  else echo "     → Sin sudo/tcpdump no puedo ver si llegan paquetes: sudo apt install tcpdump."; fi
}
c_driver() {
  if [ "$DIAG_HECHO" = 0 ] && [ $((SECONDS - T_DRV0)) -gt 30 ]; then DIAG_HECHO=1; diagnostico_egm
    if [ "$AUTO" = 1 ] && [ "$RAPID_ARRANCADO" = 1 ]; then echo "     Reinicio ros_egm (Stop → Start) para forzar la reconexión."; RWS parar >/dev/null 2>&1; sleep 2; RWS preparar >/dev/null 2>&1; RWS arrancar >/dev/null 2>&1; fi
  fi
  if [ $((SECONDS - T_DRV0)) -gt 120 ]; then DIAG_HECHO=0; T_DRV0=$SECONDS; fi
  if ! kill -0 "$DRV" 2>/dev/null; then MOTIVO="El driver terminó. Último log: $(tail -3 "$LOG" | tr '\n' ' ')"; pkill -KILL -f ros2_control_node 2>/dev/null; sleep 1; lanzar_driver; return 1; fi
  if ros2 control list_controllers 2>/dev/null | grep -q "joint_trajectory_controller.*active"; then return 0; fi
  MOTIVO="El driver sigue en 'Not connected' ($(grep -c 'Not connected' "$LOG") intentos)."
  if [ "$AUTO" = 1 ]; then refrescar && [ "$EXEC" != "running" ] && { MOTIVO="$MOTIVO RAPID se detuvo: reintentando arrancarlo."; RWS arrancar >/dev/null 2>&1; }; fi
  return 1
}
espera "driver conectado y controlador activo" "Debe estar corriendo ros_egm en el controlador y llegar UDP al puerto 6511. Diagnóstico en otra terminal: sudo tcpdump -ni any udp port 6511 -c 5 ; ss -ulpn | grep 6511 ; sudo ufw status" c_driver
ok "Driver conectado y joint_trajectory_controller ACTIVO"
ros2 control list_controllers 2>/dev/null | sed 's/^/    /'
c_accion() { ros2 action list -t 2>/dev/null | grep -q follow_joint_trajectory && return 0; MOTIVO="No aparece follow_joint_trajectory."; return 1; }
espera "acción FollowJointTrajectory" "El controlador debería exponerla al activarse; revisá el log del driver." c_accion
ok "Acción FollowJointTrajectory disponible"

# ------------------------------------------------------------------ 9. coherencia ROS vs controlador
paso "9. ROS coincide con el controlador"
c_comparar() { local o; if o="$(RWS comparar --topico /joint_states --tol 1.0 2>&1)"; then echo "$o" | sed 's/^/    /'; return 0; fi; MOTIVO="$o"; return 1; }
espera "ROS y el IRC5 con la misma postura" "Si persiste: revisá signos/offsets en config.json (robot.*) y que /joint_states sea del driver (no del puente RWS)." c_comparar
ok "ROS y el controlador reportan la misma postura"

# ------------------------------------------------------------------ 10. RViz + VEVAS
paso "10. Todo OK — abriendo las herramientas"
if [ "$RVIZ" = 1 ]; then
  ros2 launch abb_bringup abb_moveit.launch.py \
    robot_xacro_file:=irb120_3_58.xacro support_package:=abb_irb120_support \
    moveit_config_package:=abb_irb120_3_58_moveit_config \
    moveit_config_file:=abb_irb120_3_58.srdf.xacro >"$AQUI/sesiones/moveit_$(date +%F_%H%M%S).log" 2>&1 &
  PIDS+=("$!")
  ok "RViz + MoveIt abriéndose. En MotionPlanning: Velocity/Accel scaling 0.05, arrastrá el marcador unos cm → Plan → Execute"
  aviso "MoveIt NO pasa por la revisión de VEVAS. Mandá movimientos desde UNA sola herramienta a la vez."
fi
echo; echo "  PARO: el paro de emergencia (o soltar el enabling device en MANR). Ctrl+C aquí cierra todo."
if [ "$VEVAS" = 1 ]; then
  while true; do
    ok "Consola VEVAS en http://127.0.0.1:8000 (revisión previa activa)"
    python3 consola_vevas.py --modo ros --real --go "$GO" --abrir; rc=$?
    [ "$rc" = 0 ] && break
    aviso "VEVAS terminó con error (código $rc). Corregilo y presioná Enter para reintentar (Ctrl+C cierra todo)."
    read -r _ || break
  done
else
  echo "  (--sin-vevas) Driver y RViz corriendo. Ctrl+C para cerrar."
  wait "$DRV"
fi
salir_con 0
