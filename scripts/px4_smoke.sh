#!/usr/bin/env bash
# Дымовая проверка Фазы 5: два дрона PX4 SITL летят в режиме offboard из ROS 2 (всё в Docker, без окон):
#   scripts/px4_smoke.sh            (make px4-smoke)
# Для каждого мира из WORLDS: свой сервер Gazebo, один агент XRCE-DDS, два PX4 (экземпляры 1 и 2,
# пространства имён px4_1 и px4_2, модель gz_x500), узел scripts/px4_smoke.py летит сценарий.
# Результаты: results/phase-5/px4_smoke/<мир>/ (result.json, samples_drone_N.csv, логи) и summary.md.
#
# Переменные окружения:
#   ROS_DOMAIN_ID  (по умолчанию 47)          другой, чем 42, чтобы не мешать другим симуляциям
#   GZ_PARTITION   (по умолчанию px4_smoke)   свой раздел gz-transport
#   IMAGE          (по умолчанию rescueswarm-dev:jazzy)  образ, из которого берётся контейнер
#   WORLDS         (по умолчанию "default step2ms")      default: мир PX4, step2ms: тот же с шагом 0.002 с,
#                  slow: тот же с целевым RTF 0.3 (проверка поведения при RTF < 1)
#   AGENT_PORT     (по умолчанию 8888)        UDP-порт агента XRCE-DDS
#   SERVER_CONFIG  (по умолчанию 1)           1: сервер читает server.config PX4 (плагины мира), 0: не читает
# Код возврата 0 только если все миры прошли.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-47}" GZ_PARTITION="${GZ_PARTITION:-px4_smoke}"
IMAGE="${IMAGE:-rescueswarm-dev:jazzy}"
WORLDS="${WORLDS:-default step2ms}"
AGENT_PORT="${AGENT_PORT:-8888}"
SERVER_CONFIG="${SERVER_CONFIG:-1}"
PX4=/opt/px4
BIN="$PX4/build/px4_sitl_default"

# ======================================================================= внутри контейнера
if [ "${1:-}" = inside ]; then
  LABEL="$2"
  OUT="/workspace/results/phase-5/px4_smoke/$LABEL"
  RUNDIR="/tmp/px4_smoke_run/$LABEL"      # рабочие каталоги PX4: вне дерева /opt/px4
  rm -rf "$OUT" "$RUNDIR"; mkdir -p "$OUT" "$RUNDIR"
  set +u   # setup.bash ROS обращается к неустановленным переменным
  source /opt/ros/jazzy/setup.bash
  source /workspace/install_px4/setup.bash
  set -u
  export GZ_IP="${GZ_IP:-127.0.0.1}"
  # 1) мир: копия мира PX4, при необходимости с другим шагом физики
  WORLD_SRC="$PX4/Tools/simulation/gz/worlds/default.sdf"
  cp "$WORLD_SRC" "$OUT/world.sdf"
  case "$LABEL" in
    step2ms) sed -i 's|<max_step_size>[^<]*</max_step_size>|<max_step_size>0.002</max_step_size>|' "$OUT/world.sdf" ;;
    slow)    sed -i 's|<real_time_factor>[^<]*</real_time_factor>|<real_time_factor>0.3</real_time_factor>|' "$OUT/world.sdf" ;;
  esac
  grep -E "max_step_size|real_time_factor" "$OUT/world.sdf" > "$OUT/physics.txt"
  WORLD_NAME="$(sed -n 's|.*<world name="\([^"]*\)".*|\1|p;s|.*<world name='"'"'\([^'"'"']*\)'"'"'.*|\1|p' "$OUT/world.sdf" | head -1)"

  # 2) сервер gz сам (PX4 в режиме standalone). Модели и плагины PX4 сервер находит по этим переменным.
  export GZ_SIM_RESOURCE_PATH="$PX4/Tools/simulation/gz/models:$PX4/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$BIN/src/modules/simulation/gz_plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
  [ "$SERVER_CONFIG" = 1 ] && export GZ_SIM_SERVER_CONFIG_PATH="$PX4/src/modules/simulation/gz_bridge/server.config"
  export PX4_GZ_MODELS="$PX4/Tools/simulation/gz/models" PX4_GZ_WORLDS="$PX4/Tools/simulation/gz/worlds"

  stop_all() {
    # корректная остановка: SIGTERM, ждём; остальное - SIGKILL. Имена процессов для pkill: px4, MicroXRCEAgent,
    # parameter_bridge, а сервер gz это ruby (gz sim) и gz-sim-server.
    for p in $(pgrep -x px4); do kill -TERM "$p" 2>/dev/null; done
    for _ in $(seq 1 20); do pgrep -x px4 >/dev/null || break; sleep 0.5; done
    pkill -TERM -x MicroXRCEAgent; pkill -TERM -f "[p]arameter_bridge"; pkill -TERM -f "[g]z sim"; pkill -TERM -f "[g]z-sim-server"
    sleep 2
    pkill -9 -x px4; pkill -9 -x MicroXRCEAgent; pkill -9 -f "[p]arameter_bridge"; pkill -9 -f "[g]z sim"; pkill -9 -f "[g]z-sim-server"; pkill -9 -f "[r]uby"
    true
  }
  trap stop_all EXIT
  stop_all   # остатки прошлых прогонов (в контейнере только наши процессы)

  gz sim -s -r --headless-rendering "$OUT/world.sdf" > "$OUT/gz.log" 2>&1 &
  for _ in $(seq 1 60); do gz topic -l 2>/dev/null | grep -q "^/world/$WORLD_NAME/clock" && break; sleep 1; done
  gz topic -l 2>/dev/null | grep -q "^/world/$WORLD_NAME/clock" || { echo "$LABEL: сервер gz не запустился"; exit 3; }
  gz topic -l > "$OUT/gz_topics_before_px4.txt" 2>&1

  # 3) мост /clock (истинные позы узел читает прямо из gz-transport: мост Pose_V теряет имена)
  ros2 run ros_gz_bridge parameter_bridge \
      "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" > "$OUT/bridge.log" 2>&1 &

  # 4) один агент, он стартует первым и не перезапускается
  MicroXRCEAgent udp4 -p "$AGENT_PORT" > "$OUT/agent.log" 2>&1 &
  sleep 2

  # 5) два PX4. Параметры отказов без GCS и без пульта через PX4_PARAM_*.
  T0S=""
  start_px4() {   # $1 = экземпляр, $2 = поза спауна "x,y,z,roll,pitch,yaw"
    local I="$1" POSE="$2" WD="$RUNDIR/px4_$1"
    mkdir -p "$WD"
    T0S="$T0S${T0S:+,}$(date +%s.%N)"
    ( cd "$WD" && \
      PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$WORLD_NAME" PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 \
      PX4_GZ_MODEL_POSE="$POSE" PX4_UXRCE_DDS_NS="px4_$I" PX4_UXRCE_DDS_PORT="$AGENT_PORT" \
      PX4_PARAM_NAV_DLL_ACT=0 PX4_PARAM_COM_RCL_EXCEPT=4 PX4_PARAM_NAV_RCL_ACT=1 PX4_PARAM_COM_DISARM_PRFLT=60 \
      exec "$BIN/bin/px4" -i "$I" -d "$BIN/etc" < /dev/null > "$OUT/px4_$I.log" 2>&1 ) &
  }
  start_px4 1 "0,0,0,0,0,0"
  sleep 4          # первый экземпляр успевает создать модель в мире
  start_px4 2 "3,0,0,0,0,0"
  # NAV_DLL_ACT через PX4_PARAM_* не держится: env применяется до airframe, значение 0 совпадает
  # с умолчанием и не сохраняется, а airframe 4001 потом ставит set-default 2 ("No connection to the GCS").
  # Поэтому ставим после старта через клиент px4-param.
  sleep 6
  for I in 1 2; do
    for _ in $(seq 1 20); do "$BIN/bin/px4-param" --instance "$I" set NAV_DLL_ACT 0 >/dev/null 2>&1 && break; sleep 1; done
    "$BIN/bin/px4-param" --instance "$I" show NAV_DLL_ACT 2>&1 | tail -1 >> "$OUT/params.txt"
  done

  # 6) узел сценария (по времени симуляции), результаты в $OUT
  python3 /workspace/scripts/px4_smoke.py run --out "$OUT" --world "$WORLD_NAME" --world-label "$LABEL" \
      --px4-t0 "$T0S" 2>&1 | tee "$OUT/node.log"
  RC=${PIPESTATUS[0]}
  echo "$RC" > "$OUT/node_rc.txt"
  # 7) остановка с записью, как процессы завершились
  { echo "до остановки:"; ps -eo pid,comm,args | grep -E "[p]x4|[M]icroXRCE|[g]z|[r]uby" | cut -c1-150; } > "$OUT/shutdown.log"
  stop_all
  { echo "после остановки:"; ps -eo pid,comm,args | grep -E "[p]x4|[M]icroXRCE|[g]z|[r]uby" | cut -c1-150; } >> "$OUT/shutdown.log"
  trap - EXIT
  exit "$RC"
fi

# ======================================================================= на хосте
RES="$ROOT/results/phase-5/px4_smoke"; mkdir -p "$RES"
NAME=rescueswarm-px4-smoke
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
OVR="$(mktemp /tmp/px4_smoke_compose.XXXXXX.yaml)"
printf 'services:\n  dev:\n    image: %s\n' "$IMAGE" > "$OVR"
COMPOSE=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml" -f "$OVR")
cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; rm -f "$OVR"; }
trap cleanup EXIT
test -d "$ROOT/install_px4" || { echo "нет install_px4: соберите px4_msgs (make deps; см. README этапа)" >&2; exit 2; }

RC=0; DIRS=()
for W in $WORLDS; do
  echo "== PX4 smoke: мир $W (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, GZ_PARTITION=$GZ_PARTITION)"
  docker rm -f "$NAME" >/dev/null 2>&1
  "${COMPOSE[@]}" run -d --rm -T --name "$NAME" dev sleep infinity >/dev/null || { echo "контейнер не запустился"; exit 2; }
  sleep 2
  docker exec -e ROS_DOMAIN_ID -e GZ_PARTITION -e AGENT_PORT -e SERVER_CONFIG "$NAME" \
      bash /workspace/scripts/px4_smoke.sh inside "$W" | grep -E "ready to arm|->|результат|ERROR|Traceback|не "
  R=${PIPESTATUS[0]}
  echo "== $W: код $R"
  [ "$R" = 0 ] || RC=1
  DIRS+=("$RES/$W")
  docker rm -f "$NAME" >/dev/null 2>&1
done

python3 "$ROOT/scripts/px4_smoke.py" summarize --output "$RES/summary.md" "${DIRS[@]}" | tail -n +1 >/dev/null
[ "$RC" = 0 ] && echo "ИТОГ: OK (summary: results/phase-5/px4_smoke/summary.md)" || echo "ИТОГ: есть ошибки (см. results/phase-5/px4_smoke/)"
exit "$RC"
