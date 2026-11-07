#!/usr/bin/env bash
# Только для разработки: один прогон исследования в уже запущенном контейнере (по умолчанию rs-wpc-dev).
#   run_seed.sh SEED TARGET MAXTIME OUT_NAME [N]
# Результаты: results/phase-2/OUT_NAME/ (CSV, карта, график) + rtf.txt, cpu.txt, launch.log рядом.
set -uo pipefail
SEED=$1; TARGET=$2; MAXT=$3; NAME=$4; N=${5:-3}
C=${CONTAINER:-rs-wpc-dev}
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$ROOT/results/phase-2/$NAME"
mkdir -p "$OUT"; rm -f "$OUT"/rtf.txt "$OUT"/cpu.txt
docker exec "$C" bash -c "pkill -9 -f '[/]workspace/install'; pkill -9 -f '[g]z sim'; pkill -9 -f '[r]uby'; pkill -9 -f '[/]opt/ros'; pkill -9 -f '[c]onst_map'; pkill -9 -f '[p]hase2_monitor'; true"; sleep 2
docker exec -d "$C" bash -c "cd /workspace && source install/setup.bash && ros2 launch rs_bringup explore.launch.py seed:=$SEED n:=$N headless:=true rviz:=false target_coverage:=$TARGET max_time:=$MAXT out_dir:=/workspace/results/phase-2/$NAME > /workspace/results/phase-2/$NAME/launch.log 2>&1"
# монитор безопасности (мин. расстояние между роботами, журнал поз): safety.json, safety_poses.csv
docker exec -d "$C" bash -c "cd /workspace && source install/setup.bash && until [ -f results/phase-2/$NAME/world/gt_map.yaml ]; do sleep 1; done; python3 scripts/phase2_monitor.py results/phase-2/$NAME/safety.json results/phase-2/$NAME/world $N > results/phase-2/$NAME/monitor.log 2>&1"
sleep 45
while ! grep -q "EXPLORATION COMPLETE" "$OUT/launch.log" 2>/dev/null; do
  docker exec "$C" bash -c "source /workspace/install/setup.bash; gz topic -e -t /world/rescue/stats -n 1 2>/dev/null | grep -E 'real_time_factor|sim_time' | tr '\n' ' '" >> "$OUT/rtf.txt"; echo >> "$OUT/rtf.txt"
  docker exec "$C" bash -c "top -b -n 2 -d 5 -w 160 | awk '/^top/{n++} n==2' | sed -n 8,26p" >> "$OUT/cpu.txt"; echo ---- >> "$OUT/cpu.txt"
  sleep 15
  pgrep -f x >/dev/null
  if ! docker exec "$C" pgrep -f "ros2 launch" >/dev/null; then break; fi
done
sleep 8
docker exec "$C" bash -c "pkill -9 -f '[/]workspace/install'; pkill -9 -f '[g]z sim'; pkill -9 -f '[r]uby'; pkill -9 -f '[/]opt/ros'; pkill -9 -f '[c]onst_map'; true"
tail -1 "$OUT/coverage.csv"; cat "$OUT/summary.txt"; grep -E 'min_robot|min_pair' "$OUT/safety.json"
