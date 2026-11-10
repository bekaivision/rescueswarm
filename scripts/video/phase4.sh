#!/usr/bin/env bash
# Видео Фазы 4 -> media/phase-4.mp4. Запуск с хоста из любого каталога (или: make video PHASE=4).
#   scripts/video/phase4.sh [--rerecord]   --rerecord: заново записать прогон поиска
# Всё (Gazebo, ROS 2, python, ffmpeg) работает в Docker. Сырьё (кэш): media/raw/phase-4/.
# 1) запись: один headless-прогон explore.launch (seed SEED, по умолчанию 1, localization:=slam; тепловизор + слияние отметок
#    включены) + phase4_rec.py пишет карту, позы, цели, отметки людей и тепловые кадры по времени
#    симуляции; после прогона phase3_eval.py считает итог (check.json) по эталону этого же прогона;
# 2) сборка: phase4_render.py рисует кадры из записи, итоговые числа берёт из check.json; окно и экран не нужны.
# Внутри контейнера скрипт вызывается с аргументом record или compose.
set -euo pipefail

SEED=${SEED:-1}
N=3
RAW=/workspace/media/raw/phase-4

stop_all() {
  pkill -INT -f "[p]hase3_rec.py" 2>/dev/null || true; sleep 3
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
  # launch сам завершается после EXPLORATION COMPLETE (покрытие 95 % и нет открытых кандидатов); предел 600 сим. с
  ros2 launch rs_bringup explore.launch.py seed:=$SEED n:=$N headless:=true rviz:=false localization:=slam \
    max_time:=600 out_dir:=$RAW/run >"$RAW/run/launch.log" 2>&1 &
  LPID=$!
  until [ -f "$RAW/run/world/gt_map.yaml" ]; do sleep 1; done
  python3 /workspace/scripts/video/phase4_rec.py "$RAW/rec.npz" $N >"$RAW/rec.log" 2>&1 &
  for _ in $(seq 1 1200); do                # до ~40 мин реального времени
    grep -q "EXPLORATION COMPLETE" "$RAW/run/launch.log" 2>/dev/null && { sleep 6; break; }
    kill -0 "$LPID" 2>/dev/null || break
    sleep 2
  done
  stop_all
  tail -n 3 "$RAW/run/coverage.csv"
  echo "$SEED" > "$RAW/seed.txt"
  # итог по эталону этого прогона (числа для финального кадра)
  python3 -c "import yaml,json;g=yaml.safe_load(open('$RAW/run/world/ground_truth.yaml'));json.dump({'people':g['people'],'decoys':g.get('decoys',[])},open('$RAW/run/gt.json','w'),indent=1)"
  python3 /workspace/scripts/phase3_eval.py "$RAW/run" "$SEED" 600 || true
  python3 /workspace/scripts/ate_eval.py "$RAW/run" >"$RAW/run/ate.txt" 2>&1 || true
}

compose() {
  python3 /workspace/scripts/video/phase4_render.py "$RAW" /workspace/assets/brand/bekai-logo-dark.png \
    /workspace/media/phase-4.mp4
}

case "${1:-}" in
  record) record; exit ;;
  compose) compose; exit ;;
esac

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
RUN=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml" run --rm -T -e SEED="$SEED"
     ${ROS_DOMAIN_ID_VIDEO:+-e ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VIDEO} ${GZ_PARTITION_VIDEO:+-e GZ_PARTITION=$GZ_PARTITION_VIDEO})
mkdir -p "$ROOT/media/raw/phase-4"
if [ "${1:-}" = --rerecord ] || [ ! -f "$ROOT/media/raw/phase-4/rec.npz" ] \
   || [ ! -f "$ROOT/media/raw/phase-4/run/check.json" ]; then
  echo "Запись прогона поиска (seed $SEED, headless, несколько минут)..."
  docker rm -f rescueswarm-video-rec >/dev/null 2>&1 || true
  "${RUN[@]}" --name rescueswarm-video-rec dev bash scripts/video/phase4.sh record
fi
echo "Сборка media/phase-4.mp4..."
"${RUN[@]}" dev bash scripts/video/phase4.sh compose
echo "Готово: $ROOT/media/phase-4.mp4"
