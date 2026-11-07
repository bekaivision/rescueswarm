#!/usr/bin/env bash
# Видео Фазы 2 -> media/phase-2.mp4. Запуск с хоста из любого каталога (или: make video PHASE=2).
#   scripts/video/phase2.sh [--rerecord]   --rerecord: заново записать прогон исследования
# Всё (Gazebo, ROS 2, python, ffmpeg) работает в Docker. Сырьё (кэш): media/raw/phase-2/.
# 1) запись: один headless-прогон explore.launch (seed SEED, по умолчанию 1) + phase2_rec.py пишет
#    карту, позы, цели и фронтиры по времени симуляции; рядом лежит coverage.csv того же прогона;
# 2) сборка: phase2_render.py рисует кадры из записи (числа берутся из coverage.csv по времени
#    симуляции) и отдаёт их ffmpeg; окно и экран не нужны.
# Внутри контейнера скрипт вызывается с аргументом record или compose.
set -euo pipefail

SEED=${SEED:-1}
N=3
RAW=/workspace/media/raw/phase-2

stop_all() {
  pkill -INT -f "[p]hase2_rec.py" 2>/dev/null || true; sleep 3
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
  # сами launch не останавливаем по покрытию (target > 1): иначе последние секунды записи теряются при
  # остановке Gazebo. Останавливаем по строке coverage.csv >= 0.97 и ещё пишем несколько секунд. Предел 420 сим. с.
  ros2 launch rs_bringup explore.launch.py seed:=$SEED n:=$N headless:=true rviz:=false \
    target_coverage:=1.1 max_time:=420 out_dir:=$RAW/run >"$RAW/run/launch.log" 2>&1 &
  LPID=$!
  until [ -f "$RAW/run/world/gt_map.yaml" ]; do sleep 1; done
  python3 /workspace/scripts/video/phase2_rec.py "$RAW/rec.npz" $N >"$RAW/rec.log" 2>&1 &
  for _ in $(seq 1 1500); do                # до ~25 мин реального времени
    grep -q "EXPLORATION COMPLETE" "$RAW/run/launch.log" 2>/dev/null && break
    tail -n 1 "$RAW/run/coverage.csv" 2>/dev/null | awk -F, '$2 >= 0.97 {f=1} END {exit !f}' && { sleep 4; break; }
    kill -0 "$LPID" 2>/dev/null || break
    sleep 2
  done
  sleep 2
  stop_all
  tail -n 3 "$RAW/run/coverage.csv"
  echo "$SEED" > "$RAW/seed.txt"
}

compose() {
  python3 /workspace/scripts/video/phase2_render.py "$RAW" /workspace/assets/brand/bekai-logo-dark.png \
    /workspace/media/phase-2.mp4
}

case "${1:-}" in
  record) record; exit ;;
  compose) compose; exit ;;
esac

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
RUN=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml" run --rm -T -e SEED="$SEED"
     ${ROS_DOMAIN_ID_VIDEO:+-e ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VIDEO} ${GZ_PARTITION_VIDEO:+-e GZ_PARTITION=$GZ_PARTITION_VIDEO})
mkdir -p "$ROOT/media/raw/phase-2"
if [ "${1:-}" = --rerecord ] || [ ! -f "$ROOT/media/raw/phase-2/rec.npz" ] \
   || [ ! -f "$ROOT/media/raw/phase-2/run/coverage.csv" ]; then
  echo "Запись прогона исследования (seed $SEED, headless, несколько минут)..."
  docker rm -f rescueswarm-video-rec >/dev/null 2>&1 || true
  "${RUN[@]}" --name rescueswarm-video-rec dev bash scripts/video/phase2.sh record
fi
echo "Сборка media/phase-2.mp4..."
"${RUN[@]}" dev bash scripts/video/phase2.sh compose
echo "Готово: $ROOT/media/phase-2.mp4"
