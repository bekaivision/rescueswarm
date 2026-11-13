#!/usr/bin/env bash
# Видео Фазы 6 -> media/phase-6.mp4. Запуск с хоста из любого каталога (или: make video PHASE=6).
#   scripts/video/phase6.sh [--rerecord]   --rerecord: заново записать прогон
# Всё (Gazebo, ROS 2, python, ffmpeg) работает в Docker. Сырьё (кэш): media/raw/phase-6/.
# 1) запись: один headless-прогон explore.launch (seed SEED, по умолчанию 1; 3 наземных робота, без дронов,
#    радиосвязь comms:=true link:=good, обрыв связи ground_1 с 60-й по 200-ю секунду; истинная поза) +
#    phase6_rec.py пишет карту базы, собственную карту ground_1, позы и отметки людей по времени симуляции;
#    после прогона phase3_eval.py считает итог (check.json), phase6_eval.py - comms_eval.json (время догона);
# 2) сборка: phase6_render.py рисует кадры из записи; окно и экран не нужны.
# Внутри контейнера скрипт вызывается с аргументом record или compose.
set -euo pipefail

SEED=${SEED:-1}
N=3
RAW=/workspace/media/raw/phase-6

stop_all() {
  pkill -INT -f "[p]hase6_rec.py" 2>/dev/null || true; sleep 3
  kill -INT "$LPID" 2>/dev/null || true; sleep 4
  pkill -9 -f "[/]workspace/install" 2>/dev/null || true
  pkill -9 -f "[g]z sim" 2>/dev/null || true
  pkill -9 -f "[r]uby" 2>/dev/null || true
  pkill -9 -f "[/]opt/ros" 2>/dev/null || true
  wait "$LPID" 2>/dev/null || true; sleep 2
}

record() {
  set +u; source /workspace/install/setup.bash; set -u
  rm -rf "$RAW/run" "$RAW/rec.npz"; mkdir -p "$RAW/run"
  # launch сам завершается после EXPLORATION COMPLETE; предел 700 сим. с
  ros2 launch rs_bringup explore.launch.py seed:=$SEED n:=$N drones:=0 headless:=true rviz:=false localization:=gt \
    comms:=true link:=good outages:='ground_1:60:200' max_time:=700 out_dir:=$RAW/run >"$RAW/run/launch.log" 2>&1 &
  LPID=$!
  until [ -f "$RAW/run/world/gt_map.yaml" ]; do sleep 1; done
  python3 /workspace/scripts/video/phase6_rec.py "$RAW/rec.npz" $N >"$RAW/rec.log" 2>&1 &
  for _ in $(seq 1 1500); do                # до ~50 мин реального времени
    grep -q "EXPLORATION COMPLETE" "$RAW/run/launch.log" 2>/dev/null && { sleep 6; break; }
    kill -0 "$LPID" 2>/dev/null || break
    sleep 2
  done
  stop_all
  tail -n 3 "$RAW/run/coverage.csv"
  echo "$SEED" > "$RAW/seed.txt"
  python3 -c "import yaml,json;g=yaml.safe_load(open('$RAW/run/world/ground_truth.yaml'));json.dump({'people':g['people'],'decoys':g.get('decoys',[])},open('$RAW/run/gt.json','w'),indent=1)"
  python3 /workspace/scripts/phase3_eval.py "$RAW/run" "$SEED" 700 || true
  python3 /workspace/scripts/phase6_eval.py "$RAW/run" >"$RAW/run/comms_eval.txt" 2>&1 || true
}

compose() {
  python3 /workspace/scripts/video/phase6_render.py "$RAW" /workspace/assets/brand/bekai-logo-dark.png \
    /workspace/media/phase-6.mp4
}

case "${1:-}" in
  record) record; exit ;;
  compose) compose; exit ;;
esac

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
RUN=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml" run --rm -T -e SEED="$SEED"
     ${ROS_DOMAIN_ID_VIDEO:+-e ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VIDEO} ${GZ_PARTITION_VIDEO:+-e GZ_PARTITION=$GZ_PARTITION_VIDEO})
mkdir -p "$ROOT/media/raw/phase-6"
if [ "${1:-}" = --rerecord ] || [ ! -f "$ROOT/media/raw/phase-6/rec.npz" ] \
   || [ ! -f "$ROOT/media/raw/phase-6/run/check.json" ] || [ ! -f "$ROOT/media/raw/phase-6/run/comms_eval.json" ]; then
  echo "Запись прогона с обрывом связи (seed $SEED, headless, несколько минут)..."
  docker rm -f rescueswarm-video-rec >/dev/null 2>&1 || true
  "${RUN[@]}" --name rescueswarm-video-rec dev bash scripts/video/phase6.sh record
fi
echo "Сборка media/phase-6.mp4..."
"${RUN[@]}" dev bash scripts/video/phase6.sh compose
echo "Готово: $ROOT/media/phase-6.mp4"
