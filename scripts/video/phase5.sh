#!/usr/bin/env bash
# Видео Фазы 5 -> media/phase-5.mp4. Запуск с хоста из любого каталога (или: make video PHASE=5).
#   scripts/video/phase5.sh [--rerecord]   --rerecord: заново записать прогон
# Всё (Gazebo, PX4, ROS 2, python, ffmpeg) работает в Docker. Сырьё (кэш): media/raw/phase-5/.
# 1) запись: один headless-прогон explore.launch (seed SEED, по умолчанию 1; 3 наземных робота и 2 дрона,
#    бэкенд DBACK, по умолчанию px4; истинная поза наземных роботов) + phase5_rec.py пишет общую карту,
#    наземный слой, позы, цели, отметки людей и тепловые кадры дронов по времени симуляции; после прогона
#    phase3_eval.py считает итог (check.json) по эталону этого же прогона;
# 2) сборка: phase5_render.py рисует кадры из записи; окно и экран не нужны.
# Внутри контейнера скрипт вызывается с аргументом record или compose.
set -euo pipefail

SEED=${SEED:-1}
DBACK=${DBACK:-px4}
N=3
M=2
RAW=/workspace/media/raw/phase-5

stop_all() {
  pkill -INT -f "[p]hase5_rec.py" 2>/dev/null || true; sleep 3
  kill -INT "$LPID" 2>/dev/null || true; sleep 4
  pkill -9 -f "[/]workspace/install" 2>/dev/null || true
  pkill -9 -f "[g]z sim" 2>/dev/null || true
  pkill -9 -f "[r]uby" 2>/dev/null || true
  pkill -9 -x px4 2>/dev/null || true
  pkill -9 -f "[M]icroXRCEAgent" 2>/dev/null || true
  pkill -9 -f "[/]opt/ros" 2>/dev/null || true
  wait "$LPID" 2>/dev/null || true; sleep 2
}

record() {
  set +u; source /workspace/install/setup.bash; set -u
  rm -rf "$RAW/run" "$RAW/rec.npz"; mkdir -p "$RAW/run"
  # launch сам завершается после EXPLORATION COMPLETE; предел 600 сим. с
  ros2 launch rs_bringup explore.launch.py seed:=$SEED n:=$N headless:=true rviz:=false localization:=gt \
    drones:=$M drone_backend:=$DBACK max_time:=600 out_dir:=$RAW/run >"$RAW/run/launch.log" 2>&1 &
  LPID=$!
  until [ -f "$RAW/run/world/gt_map.yaml" ]; do sleep 1; done
  python3 /workspace/scripts/video/phase5_rec.py "$RAW/rec.npz" $N $M >"$RAW/rec.log" 2>&1 &
  for _ in $(seq 1 1500); do                # до ~50 мин реального времени
    grep -q "EXPLORATION COMPLETE" "$RAW/run/launch.log" 2>/dev/null && { sleep 6; break; }
    kill -0 "$LPID" 2>/dev/null || break
    sleep 2
  done
  stop_all
  tail -n 3 "$RAW/run/coverage.csv"
  echo "$SEED" > "$RAW/seed.txt"
  echo "$DBACK" > "$RAW/backend.txt"
  python3 -c "import yaml,json;g=yaml.safe_load(open('$RAW/run/world/ground_truth.yaml'));json.dump({'people':g['people'],'decoys':g.get('decoys',[])},open('$RAW/run/gt.json','w'),indent=1)"
  python3 /workspace/scripts/phase3_eval.py "$RAW/run" "$SEED" 600 || true
  python3 /workspace/scripts/layers_eval.py "$RAW/run" >"$RAW/run/layers_eval.txt" 2>&1 || true
}

compose() {
  python3 /workspace/scripts/video/phase5_render.py "$RAW" /workspace/assets/brand/bekai-logo-dark.png \
    /workspace/media/phase-5.mp4
}

case "${1:-}" in
  record) record; exit ;;
  compose) compose; exit ;;
esac

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
RUN=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml" run --rm -T -e SEED="$SEED" -e DBACK="$DBACK"
     ${ROS_DOMAIN_ID_VIDEO:+-e ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VIDEO} ${GZ_PARTITION_VIDEO:+-e GZ_PARTITION=$GZ_PARTITION_VIDEO})
mkdir -p "$ROOT/media/raw/phase-5"
if [ "${1:-}" = --rerecord ] || [ ! -f "$ROOT/media/raw/phase-5/rec.npz" ] \
   || [ ! -f "$ROOT/media/raw/phase-5/run/check.json" ]; then
  echo "Запись прогона смешанного роя (seed $SEED, $DBACK, headless, несколько минут)..."
  docker rm -f rescueswarm-video-rec >/dev/null 2>&1 || true
  "${RUN[@]}" --name rescueswarm-video-rec dev bash scripts/video/phase5.sh record
fi
echo "Сборка media/phase-5.mp4..."
"${RUN[@]}" dev bash scripts/video/phase5.sh compose
echo "Готово: $ROOT/media/phase-5.mp4"
