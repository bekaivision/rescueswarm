#!/usr/bin/env bash
# Проверка Фазы 1 (всё внутри Docker, без окон):
#   scripts/check_phase1.sh [N=3]
# Результаты: results/phase-1/ (топики, частоты, движение, TF, кадры).
set -uo pipefail
N="${1:-3}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/results/phase-1"; mkdir -p "$OUT"
NAME=rescueswarm-check-phase1
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
COMPOSE=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml")
docker rm -f "$NAME" >/dev/null 2>&1 || true
"${COMPOSE[@]}" run -d --rm -T --name "$NAME" dev bash -c \
  "source /workspace/install/setup.bash && ros2 launch rs_bringup robots.launch.py n:=$N headless:=true rviz:=false" >/dev/null
trap 'docker rm -f "$NAME" >/dev/null 2>&1 || true' EXIT
dx() { docker exec "$NAME" bash -c "source /workspace/install/setup.bash; cd /workspace/results/phase-1; $*"; }
FAIL=0
echo "== прогрев 30 с"; sleep 30

echo "== 1. топики"
dx 'ros2 topic list' | tee "$OUT/topics.txt" > /dev/null
for i in $(seq 1 "$N"); do
  c=$(grep -c "^/ground_$i/" "$OUT/topics.txt"); echo "ground_$i: $c топиков"
  [ "$c" -ge 10 ] || { echo "FAIL: мало топиков у ground_$i"; FAIL=1; }
done

echo "== 2. частоты ground_1 (ros2 topic hz, 15 с)"
: > "$OUT/hz.txt"
SPECS="lidar/points:10 rgb/image_raw:5 thermal/image_raw:5 imu:50 odom:30"
for t in $SPECS; do
  key="${t%%:*}"; key="${key//\//_}"
  ( dx "timeout 15 ros2 topic hz /ground_1/${t%%:*} 2>&1 | grep 'average rate' | tail -1" > "$OUT/hz_tmp_$key.txt" ) &
done
wait
for t in $SPECS; do
  key="${t%%:*}"; key="${key//\//_}"
  echo "/ground_1/${t%%:*} (цель ${t##*:} Гц): $(cat "$OUT/hz_tmp_$key.txt")" | tee -a "$OUT/hz.txt"
  rm -f "$OUT/hz_tmp_$key.txt"
done

echo "== 3. TF"
dx 'timeout 8 ros2 run tf2_ros tf2_echo world ground_2/base_link 2>&1 | head -12' | tee "$OUT/tf2_echo.txt"
grep -q Translation "$OUT/tf2_echo.txt" || { echo "FAIL: tf2_echo"; FAIL=1; }
dx 'ros2 run tf2_tools view_frames >/dev/null 2>&1; ls frames_*.pdf frames_*.gv 2>/dev/null' | tee "$OUT/view_frames_files.txt" >/dev/null
f=$(ls -t "$OUT"/frames_*.pdf 2>/dev/null | head -1); [ -n "$f" ] && mv "$f" "$OUT/tf_frames.pdf"
f=$(ls -t "$OUT"/frames_*.gv 2>/dev/null | head -1); [ -n "$f" ] && mv "$f" "$OUT/tf_frames.gv"

echo "== 4. кадры RGB и тепловизора"
dx 'python3 /workspace/scripts/phase1_probe.py frames /workspace/results/phase-1' | tee "$OUT/frames.txt" || FAIL=1

echo "== 5. независимое управление (до/после odom)"
dx 'python3 /workspace/scripts/phase1_probe.py move /workspace/results/phase-1' | tee "$OUT/independent_motion.log" || FAIL=1

docker logs "$NAME" > "$OUT/launch_check.log" 2>&1
[ "$FAIL" = 0 ] && echo "ИТОГ: OK" || echo "ИТОГ: есть ошибки"
exit "$FAIL"
