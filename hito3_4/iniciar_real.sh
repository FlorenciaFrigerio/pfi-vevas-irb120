#!/usr/bin/env bash
# iniciar_real.sh — levanta TODO con un comando por la ruta RWS (sin EGM, sin driver abb_ros2) y ESPERA a que
# lo que falte se resuelva (no se cierra por un problema).
#
#   ./iniciar_real.sh                 # espera AUTO si hace falta, instala VEVAS_RWS, arranca VevasServidor y abre VEVAS
#   ./iniciar_real.sh --manual        # llave en MANR: vos ponés PP a VevasServidor y Start en el FlexPendant
#   ./iniciar_real.sh -y              # igual, sin pedir ni Enter
#   ./iniciar_real.sh --solo-chequeo  # una pasada de chequeos, sin esperar y sin escribir en el controlador
#
# Opciones: --ip IP  --rws-puerto N  --go archivo.json  --recargar-modulo  --sin-rviz  --sin-vevas  --ws ~/abb_ws
# Cada problema se explica con qué hacer; el script reintenta solo. Ctrl+C cierra todo en orden.
# NO cambia el modo de operación (llave física) ni puentea paros o vallas.
#
# UNA SOLA HERRAMIENTA A LA VEZ: VEVAS (--modo rws) y el nodo de MoveIt escriben en las mismas variables PERS del
# robot. VEVAS usa su propio cliente RWS (conserva GO, revisión previa, armado y PARAR); el nodo de MoveIt solo
# se usa para ver el robot en RViz y planear. Ambos toman un candado de archivo mientras mueven: si uno está
# moviendo, el otro rechaza su orden con un mensaje claro (no se pisan).
set -uo pipefail

AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$AQUI" || exit 1
IP="192.168.125.1"; WS="$HOME/abb_ws"; GO="$AQUI/GO_VEVAS.json"
MANUAL=0; SI=0; SOLO_CHEQUEO=0; RECARGAR=0; RVIZ=1; VEVAS=1; RWS_PUERTO=80
while [ $# -gt 0 ]; do
  case "$1" in
    --auto) MANUAL=0;; -y|--si) SI=1;; --manual) MANUAL=1;; --solo-chequeo) SOLO_CHEQUEO=1;; --recargar-modulo) RECARGAR=1;;
    --sin-rviz) RVIZ=0;; --sin-vevas) VEVAS=0;;
    --ip) IP="$2"; shift;; --go) GO="$2"; shift;; --ws) WS="$2"; shift;; --rws-puerto) RWS_PUERTO="$2"; shift;;
    -h|--help) sed -n '2,16p' "$0"; exit 0;;
    *) echo "Opción desconocida: $1"; exit 2;;
  esac; shift
done
AUTO=$((1 - MANUAL))

R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; B=$'\e[1m'; N=$'\e[0m'
paso()  { echo; echo "${B}== $* ==${N}"; }
ok()    { echo "  ${G}✓${N} $*"; }
aviso() { echo "  ${Y}!${N} $*"; }
fallo() { echo "  ${R}✗ $*${N}"; salir_con 1; }
RWS() { python3 "$AQUI/herramientas/rws_irc5.py" --ip "$IP" --puerto "$RWS_PUERTO" "$@"; }

PIDS=(); RAPID_ARRANCADO=0; LIMPIO=0; NODO=""; MOTIVO=""; AYUDA=""; INTERVALO=3; ROS=0
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
refrescar() {
  local o
  RB_ACTIVO=""; RB_CODIGO=""; PP_MODULO=""
  if o="$(RWS estado --shell 2>&1)"; then eval "$o"; return 0; fi
  MOTIVO="$o"; return 1
}

# ------------------------------------------------------------------ 1. entorno (no se arregla esperando)
paso "1. Entorno"
export LC_NUMERIC=C
python3 -c "import vevas.consola, vevas.rws_cliente" 2>/dev/null || fallo "No se pudo importar vevas (corré el script desde la carpeta del prototipo)"
[ -f "$AQUI/rapid/VEVAS_RWS.mod" ] || fallo "Falta $AQUI/rapid/VEVAS_RWS.mod"
ok "Código de VEVAS y rapid/VEVAS_RWS.mod disponibles (RWS no necesita ROS ni cambiar la IP de la PC)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [ "$RVIZ" = 1 ]; then
  if [ -f "$ROS_SETUP" ]; then
    # shellcheck disable=SC1090
    set +u; source "$ROS_SETUP"; [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"; set -u
    if command -v ros2 >/dev/null && ros2 pkg prefix abb_bringup >/dev/null 2>&1; then
      ROS=1; ok "ROS 2 y abb_bringup disponibles: se abrirán el nodo RWS y RViz + MoveIt"
    else aviso "No encuentro ros2/abb_bringup (¿$WS?): se sigue sin RViz/MoveIt (VEVAS no los necesita)."; fi
  else aviso "No existe $ROS_SETUP: se sigue sin RViz/MoveIt (VEVAS no los necesita)."; fi
fi

# ------------------------------------------------------------------ 2. red
paso "2. Comunicación con el IRC5 ($IP)"
c_ping() { ping -c 1 -W 2 "$IP" >/dev/null 2>&1 && return 0; MOTIVO="El IRC5 no responde al ping."; return 1; }
espera "ping a $IP" "Revisá: IRC5 encendido, cable en el puerto SERVICE (X2) y que la PC tenga IP en 192.168.125.x (DHCP del controlador)." c_ping
ok "ping a $IP responde"

# ------------------------------------------------------------------ 3. RWS y estado
paso "3. Estado del controlador por RWS (solo lectura)"
c_rws() { refrescar; }
espera "RWS responde" "Revisá que el controlador haya terminado de arrancar, que la opción PC Interface esté activa y las credenciales (Default User / robotics; VEVAS_RWS_CLAVE=… para cambiarlas)." c_rws
ok "RWS responde"
c_modo() {
  refrescar || return 1
  if [ "$MANUAL" = 0 ] && [ "$OPMODE" != "AUTO" ]; then MOTIVO="El controlador está en $OPMODE y se necesita AUTO."; return 1; fi
  if [ "$MANUAL" = 1 ] && [ "$OPMODE" = "MANF" ]; then MOTIVO="Manual 100 %: el módulo solo acepta AUTO o manual reducido."; return 1; fi
  return 0
}
espera "controlador en el modo correcto" "Una persona debe girar la llave del IRC5 a AUTO y confirmar en el FlexPendant si lo pide. (Con el FlexPendant: ./iniciar_real.sh --manual, llave en manual reducido)" c_modo
c_seguridad() {
  refrescar || return 1
  case "$CTRLSTATE" in
    guardstop) if [ "$MANUAL" = 1 ]; then return 0; fi; MOTIVO="Guard stop: valla/puerta abierta o auto stop activo."; return 1;;
    emergencystop|emergencystopreset) MOTIVO="Paro de emergencia activo o sin rearmar."; return 1;;
    sysfail) MOTIVO="El controlador está en falla de sistema (SYS FAIL)."; return 1;;
  esac
  return 0
}
espera "cadena de seguridad cerrada" "Cerrá la valla/puerta, soltá y rearmá el paro de emergencia y pulsá Motors On/reset en el controlador. Este script no puentea ninguna protección." c_seguridad
c_rapid_libre() {
  refrescar || return 1
  if [ "$EXEC" = "running" ] && [ "$PP_MODULO" != "VEVAS_RWS" ]; then MOTIVO="RAPID está ejecutando otro programa ($PP_MODULO/$PP_RUTINA)."; return 1; fi
  return 0
}
espera "RAPID libre" "Detené el programa que está corriendo (FlexPendant → Stop)." c_rapid_libre
echo "    modo=$OPMODE  motores=$CTRLSTATE  RAPID=$EXEC  módulo VEVAS_RWS=$MODULO_VEVAS  PP=$PP_MODULO/$PP_RUTINA"
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
  ${R}${B}MODO AUTO${N}: nadie sostiene el enabling device. El robot se mueve con MoveAbsJ a velocidad FIJA del RAPID
  (vPc = 20 mm/s de TCP, 5 °/s de orientación); la consola además limita el plan con el tope del GO.
  Hace falta: célula despejada, una persona con la mano en el paro de emergencia y autorización del docente.
  Ctrl+C o PARAR NO frenan el brazo al instante (termina el waypoint en curso): el freno es el paro de emergencia.
TXT
  echo "  Al continuar confirmás: autorización del docente, zona despejada, paro de emergencia a mano,"
  echo "  sin herramienta ni objetos dentro del alcance. Velocidad máxima ${VEVAS_VEL_PCT:-5} % (VEVAS_VEL_PCT=… para cambiarla)."
else
  echo "  Modo $OPMODE: vos ponés el PP en VEVAS_RWS/VevasServidor y pulsás Start en el FlexPendant (enabling device apretado)."
  echo "  Con VEVAS, la primera orden pide RMMP: aceptalo en el FlexPendant. Célula despejada y paro a mano."
fi
if [ "$SI" = 1 ]; then ok "Confirmado con -y"
else read -r -p "  Enter para continuar (Ctrl+C cancela): " _ || true; fi
if c_go; then ok "Se usa el GO de hoy que ya existía ($GO)"
else python3 "$AQUI/herramientas/crear_go.py" --automatico --salida "$GO" >/dev/null && c_go && ok "GO y entorno registrados automáticamente (velocidad ${VEVAS_VEL_PCT:-5} %)" || fallo "No se pudo generar el GO: $MOTIVO"; fi

# ------------------------------------------------------------------ 6. limpieza previa
paso "6. Limpieza previa"
if pgrep -f "nodo_rws_ros.py" >/dev/null; then aviso "Había un nodo RWS anterior: se cierra."; pkill -INT -f "nodo_rws_ros.py"; sleep 2; pkill -KILL -f "nodo_rws_ros.py" 2>/dev/null; fi
if pgrep -f "abb_moveit.launch|move_group" >/dev/null; then aviso "Había MoveIt anterior: se cierra."; pkill -INT -f "abb_moveit.launch|move_group"; sleep 2; fi
ok "Sin procesos viejos"

# ------------------------------------------------------------------ 7. módulo VEVAS_RWS
paso "7. Módulo VEVAS_RWS en el controlador"
c_modulo() {
  refrescar || return 1
  if [ "$EXEC" = "running" ] && [ "$PP_MODULO" = "VEVAS_RWS" ]; then return 0; fi   # ya corre VevasServidor: no se toca
  if [ "$AUTO" = 0 ]; then
    [ "$MODULO_VEVAS" = "si" ] && return 0
    MOTIVO="VEVAS_RWS no está cargado y instalarlo por RWS exige AUTO."; return 1
  fi
  local o rc
  if [ "$RECARGAR" = 1 ]; then o="$(RWS instalar --recargar 2>&1)"; rc=$?; else o="$(RWS instalar 2>&1)"; rc=$?; fi
  if [ "$rc" = 0 ]; then echo "$o" | sed 's/^/    /'; RECARGAR=0; return 0; fi
  MOTIVO="$o"; return 1
}
INTERVALO=10; espera "VEVAS_RWS instalado" "La instalación por RWS necesita AUTO, motores OFF y RAPID detenido. Si no se puede: FlexPendant → Editor de programas → Módulos → Cargar módulo → rapid/VEVAS_RWS.mod (copiado a una USB); en manual el módulo se carga una vez y después se usa --manual." c_modulo; INTERVALO=3
ok "El controlador tiene VEVAS_RWS"

# ------------------------------------------------------------------ 8. servidor VevasServidor
paso "8. VevasServidor (vvRbActivo)"
if [ "$AUTO" = 1 ]; then
  RAPID_ARRANCADO=1
  c_servidor() {
    refrescar || return 1
    if [ "$EXEC" != "running" ] || [ "$PP_MODULO" != "VEVAS_RWS" ]; then
      local o; if ! o="$(RWS arrancar 2>&1)"; then
        MOTIVO="$o"
        if [ "$o" != "${ULT_DIAG:-}" ]; then ULT_DIAG="$o"
          echo "  ${B}Mensajes del IRC5 (registro de eventos):${N}"; RWS diagnostico -n 6 --solo-errores 2>&1 | sed 's/^/  /'; fi
        if [ "${RECARGA_AUTO:-0}" = 0 ] && echo "$o" | grep -q "set-pp-routine"; then
          RECARGA_AUTO=1
          echo "  El programa del controlador no acepta el puntero: recargo VEVAS_RWS desde cero (descargar + cargar)."
          RWS parar >/dev/null 2>&1
          RWS instalar --recargar 2>&1 | sed 's/^/    /'
        fi
        return 1; fi
      echo "$o" | sed 's/^/    /'
    fi
    local s; if s="$(RWS servidor --espera 35 2>&1)"; then echo "$s" | sed 's/^/    /'; return 0; fi
    MOTIVO="$s"; return 1
  }
  INTERVALO=2; espera "VevasServidor activo" "En el FlexPendant aparece «Aceptar objetivos enviados por la PC?»: pulsá «Iniciar» antes de 30 s (si vence, el script reinicia VevasServidor y vuelve a pedirlo). Sin nadie en el pendant: poné confirmarEnPendant := FALSE en rapid/VEVAS_RWS.mod (con autorización del responsable) y ./iniciar_real.sh --recargar-modulo." c_servidor; INTERVALO=3
else
  echo; echo "  ${B}En el FlexPendant:${N} Depurar → PP a rutina → VEVAS_RWS / VevasServidor, habilitador apretado, motores ON, ${B}Start${N}, y «Iniciar» (30 s)."
  c_servidor() {
    local s; if s="$(RWS servidor --espera 5 --pausa 0 2>&1)"; then echo "$s" | sed 's/^/    /'; return 0; fi
    MOTIVO="$s"; return 1
  }
  espera "VevasServidor activo (PP a rutina + Start + Iniciar)" "Motores ON, enabling device apretado, PP en VevasServidor, Start y luego «Iniciar»." c_servidor
fi
ok "VevasServidor activo: el robot espera objetivos y NO se mueve solo"

# ------------------------------------------------------------------ 9. nodo RWS + RViz/MoveIt (opcional)
mkdir -p "$AQUI/sesiones"
if [ "$ROS" = 1 ]; then
  paso "9. Nodo ROS (/joint_states + FollowJointTrajectory) y RViz + MoveIt"
  LOGN="$AQUI/sesiones/nodo_rws_$(date +%F_%H%M%S).log"
  # NODO_RWS_CMD permite sustituir el nodo (pruebas); por defecto es el de herramientas/
  # shellcheck disable=SC2086
  ${NODO_RWS_CMD:-python3 "$AQUI/herramientas/nodo_rws_ros.py"} --ip "$IP" --rws-puerto "$RWS_PUERTO" >"$LOGN" 2>&1 &
  NODO=$!; PIDS+=("$NODO"); echo "    nodo lanzado (log: $LOGN)"
  c_nodo() {
    kill -0 "$NODO" 2>/dev/null || { MOTIVO="El nodo terminó: $(tail -2 "$LOGN" | tr '\n' ' ')"; return 2; }
    ros2 topic list 2>/dev/null | grep -qx "/joint_states" || { MOTIVO="Todavía no aparece /joint_states."; return 1; }
    ros2 action list 2>/dev/null | grep -q follow_joint_trajectory || { MOTIVO="Todavía no aparece la acción follow_joint_trajectory."; return 1; }
    return 0
  }
  for _ in 1 2 3 4 5 6 7 8 9 10; do c_nodo; rc=$?; [ "$rc" = 0 ] && break; [ "$rc" = 2 ] && break; sleep 2; done
  if [ "$rc" = 0 ]; then
    ok "Nodo RWS publicando /joint_states y sirviendo la acción"
    ros2 launch abb_bringup abb_moveit.launch.py \
      robot_xacro_file:=irb120_3_58.xacro support_package:=abb_irb120_support \
      moveit_config_package:=abb_irb120_3_58_moveit_config \
      moveit_config_file:=abb_irb120_3_58.srdf.xacro >"$AQUI/sesiones/moveit_$(date +%F_%H%M%S).log" 2>&1 &
    PIDS+=("$!")
    ok "RViz + MoveIt abriéndose. En MotionPlanning: Plan → Execute. El robot va por waypoints a velocidad FIJA (MoveIt no la controla)."
    aviso "MoveIt NO pasa por la revisión de VEVAS ni por el GO. Mandá movimientos desde UNA sola herramienta a la vez (hay candado, pero la revisión solo la hace VEVAS)."
  else
    aviso "El nodo RWS/ROS no quedó listo ($MOTIVO). Sigo sin RViz/MoveIt: VEVAS funciona igual."
  fi
fi

# ------------------------------------------------------------------ 10. VEVAS
paso "10. Todo OK — abriendo las herramientas"
echo "  PARO: el paro de emergencia (o soltar el enabling device en MANR). Ctrl+C aquí cierra todo."
if [ "$VEVAS" = 1 ]; then
  while true; do
    ok "Consola VEVAS en http://127.0.0.1:8000 (revisión previa activa, modo RWS)"
    python3 consola_vevas.py --modo rws --real --go "$GO" --ip "$IP" --rws-puerto "$RWS_PUERTO" --abrir; rc=$?
    [ "$rc" = 0 ] && break
    aviso "VEVAS terminó con error (código $rc). Corregilo y presioná Enter para reintentar (Ctrl+C cierra todo)."
    read -r _ || break
  done
else
  echo "  (--sin-vevas) Servidor${NODO:+, nodo ROS} y RViz corriendo. Ctrl+C para cerrar."
  while true; do sleep 1; done
fi
salir_con 0
