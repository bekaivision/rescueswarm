#!/usr/bin/env bash
# Замер Фазы 1 (N роботов). Запускать на хосте:
#   scripts/bench_phase1.sh headless|gui [DURATION_S=60] [N=3]
# Результаты: results/phase-1/bench_<mode>/ (сырые данные) и summary.txt.
# gui: gz с окном + RViz, снимки окон через xwd -id.
set -euo pipefail
MODE="${1:-headless}"; DUR="${2:-60}"; N="${3:-3}"; WARM=20
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/results/phase-1/bench_$MODE"; rm -rf "$OUT"; mkdir -p "$OUT"
NAME="rescueswarm-bench1-$MODE"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
COMPOSE=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml")
docker rm -f "$NAME" >/dev/null 2>&1 || true
EXTRA="${BENCH_EXTRA:-}"   # доп. аргументы launch (например thermal_rate:=2)

{ echo "# host before start"; uptime; top -b -n 1 -o %CPU | head -12; nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv; } > "$OUT/idle.txt"
sleep 5
nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader > "$OUT/vram_idle.txt"

if [ "$MODE" = headless ]; then H=true; R=false; else H=false; R=true; fi
"${COMPOSE[@]}" run -d --rm -T --name "$NAME" dev bash -c \
  "source /workspace/install/setup.bash && ros2 launch rs_bringup robots.launch.py n:=$N headless:=$H rviz:=$R $EXTRA" > "$OUT/container_id.txt"
trap 'docker rm -f "$NAME" >/dev/null 2>&1 || true' EXIT
echo "warm-up ${WARM}s"; sleep "$WARM"
dx() { docker exec "$NAME" bash -c "source /workspace/install/setup.bash; $*"; }
dx 'ros2 topic list' > "$OUT/topics.txt"

# Частоты ground_1 за окно замера (ros2 topic hz)
for t in lidar/points rgb/image_raw thermal/image_raw imu odom; do
  ( dx "timeout $((DUR-5)) ros2 topic hz /ground_1/$t 2>&1 | grep 'average rate'" > "$OUT/hz_${t//\//_}.txt" || true ) &
done
# RTF из /world/phase1/stats
( end=$((SECONDS+DUR)); while [ $SECONDS -lt $end ]; do
    dx 'gz topic -e -t /world/phase1/stats -n 1' 2>/dev/null | grep -E 'real_time_factor' | awk '{print $2}' >> "$OUT/rtf.txt" || true
    sleep 1; done ) &
( end=$((SECONDS+DUR)); echo "cpu_pct mem_mib" > "$OUT/docker_stats.txt"
  while [ $SECONDS -lt $end ]; do
    docker stats --no-stream --format '{{.CPUPerc}} {{.MemUsage}}' "$NAME" >> "$OUT/docker_stats.txt"; done ) &
( end=$((SECONDS+DUR)); echo "mem_mib util_pct" > "$OUT/gpu_samples.txt"
  while [ $SECONDS -lt $end ]; do
    nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader,nounits >> "$OUT/gpu_samples.txt"; sleep 1; done ) &
sleep $((DUR/2)); nvidia-smi > "$OUT/nvidia-smi_midrun.txt"; nvidia-smi pmon -c 1 > "$OUT/pmon_midrun.txt" 2>&1 || true
if [ "$MODE" = gui ]; then
  dx 'xwininfo -root -tree' > "$OUT/xwininfo_tree.txt" 2>&1 || true
  for w in "RViz" "Gazebo Sim"; do
    id=$(grep -i "\"$w" "$OUT/xwininfo_tree.txt" | grep -v "has no name" | head -1 | awk '{print $1}')
    f="${w// /_}"
    if [ -n "$id" ]; then
      dx "xwd -id $id -silent > /tmp/$f.xwd && ffmpeg -loglevel error -y -i /tmp/$f.xwd /workspace/results/phase-1/bench_gui/$f.png" || echo "снимок $w не удался"
    else echo "окно $w не найдено" | tee -a "$OUT/notes.txt"; fi
  done
fi
wait
docker logs "$NAME" > "$OUT/launch.log" 2>&1 || true

python3 - "$OUT" "$MODE" <<'PY'
import sys, re, statistics as s, pathlib
o = pathlib.Path(sys.argv[1]); mode = sys.argv[2]
rtf = [float(x) for x in (o/'rtf.txt').read_text().split()][1:] if (o/'rtf.txt').exists() else []
cpu=[]; mem=[]
for l in (o/'docker_stats.txt').read_text().splitlines()[1:]:
    m = re.match(r'([\d.]+)% ([\d.]+)(\w+)', l)
    if m:
        cpu.append(float(m[1])/100); v=float(m[2]); mem.append(v*1024 if m[3].startswith('G') else v)
gpu=[tuple(float(x) for x in l.split(',')) for l in (o/'gpu_samples.txt').read_text().splitlines()[1:] if l.strip()]
idle = (o/'vram_idle.txt').read_text().strip()
def hz(p):
    t=(o/p).read_text() if (o/p).exists() else ''
    a=[float(x) for x in re.findall(r'average rate: ([\d.]+)', t)]; return (f'mean {s.mean(a):.2f} min {min(a):.2f} max {max(a):.2f} (n={len(a)})' if a else 'n/a')
with open(o/'summary.txt','w') as f:
    def w(x): print(x); f.write(x+'\n')
    w(f'mode={mode}')
    if rtf: w(f'RTF samples={len(rtf)} mean={s.mean(rtf):.3f} min={min(rtf):.3f} max={max(rtf):.3f}')
    if cpu: w(f'CPU cores mean={s.mean(cpu):.2f} max={max(cpu):.2f}')
    if mem: w(f'RAM MiB mean={s.mean(mem):.0f} max={max(mem):.0f}')
    if gpu: w(f'VRAM idle(before)="{idle}" MiB, run mean={s.mean(g[0] for g in gpu):.0f} max={max(g[0] for g in gpu):.0f}; GPU util % mean={s.mean(g[1] for g in gpu):.0f} max={max(g[1] for g in gpu):.0f}')
    for t in ('lidar_points','rgb_image_raw','thermal_image_raw','imu','odom'):
        w(f'ground_1 {t} Hz: {hz("hz_"+t+".txt")}')
PY
