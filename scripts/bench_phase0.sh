#!/usr/bin/env bash
# Замер тестовой сцены Фазы 0. Запускать на хосте из любого каталога:
#   scripts/bench_phase0.sh headless|gui [DURATION_S=60]
# Результаты: results/phase-0/bench_<mode>/ (сырые данные) и summary.txt.
set -euo pipefail
MODE="${1:-headless}"; DUR="${2:-60}"; WARM=15
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/results/phase-0/bench_$MODE"; mkdir -p "$OUT"
NAME="rescueswarm-bench-$MODE"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
COMPOSE=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml")
docker rm -f "$NAME" >/dev/null 2>&1 || true

{ echo "# host before start"; uptime; top -b -n 1 -o %CPU | head -12; nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv; } > "$OUT/idle.txt"
sleep 5
nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader > "$OUT/vram_idle.txt"

if [ "$MODE" = headless ]; then
  CMD=(gz sim -s --headless-rendering -r /workspace/configs/worlds/phase0_bench.sdf)
else
  CMD=(gz sim -r /workspace/configs/worlds/phase0_bench.sdf)
fi
"${COMPOSE[@]}" run -d --rm -T --name "$NAME" dev "${CMD[@]}" > "$OUT/container_id.txt"
trap 'docker rm -f "$NAME" >/dev/null 2>&1 || true' EXIT
echo "warm-up ${WARM}s"; sleep "$WARM"
dx() { docker exec "$NAME" bash -c "source /opt/ros/jazzy/setup.bash; $*"; }
dx 'gz topic -l' > "$OUT/topics.txt"

# Частоты сенсоров (параллельно со всем остальным)
( dx "timeout $((DUR)) gz topic -f -t /camera -d $((DUR-5))" > "$OUT/hz_camera.txt" 2>&1 || true ) &
( dx "timeout $((DUR)) gz topic -f -t /lidar -d $((DUR-5))" > "$OUT/hz_lidar.txt" 2>&1 || true ) &
# RTF из /world/phase0_bench/stats (1 сообщение за раз, раз в 2 с)
( end=$((SECONDS+DUR)); while [ $SECONDS -lt $end ]; do
    dx 'gz topic -e -t /world/phase0_bench/stats -n 1' 2>/dev/null | grep -E 'real_time_factor' | awk '{print $2}' >> "$OUT/rtf.txt" || true
    sleep 1; done ) &
# docker stats и nvidia-smi раз в секунду
( end=$((SECONDS+DUR)); echo "cpu_pct mem_mib" > "$OUT/docker_stats.txt"
  while [ $SECONDS -lt $end ]; do
    docker stats --no-stream --format '{{.CPUPerc}} {{.MemUsage}}' "$NAME" >> "$OUT/docker_stats.txt"; done ) &
( end=$((SECONDS+DUR)); echo "mem_mib util_pct" > "$OUT/gpu_samples.txt"
  while [ $SECONDS -lt $end ]; do
    nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader,nounits >> "$OUT/gpu_samples.txt"; sleep 1; done ) &
sleep $((DUR/2)); nvidia-smi > "$OUT/nvidia-smi_midrun.txt"; nvidia-smi pmon -c 1 > "$OUT/pmon_midrun.txt" 2>&1 || true
wait

python3 - "$OUT" "$MODE" <<'PY'
import sys, re, statistics as s, pathlib
o = pathlib.Path(sys.argv[1]); mode = sys.argv[2]
def nums(p): return [float(x) for x in re.findall(r'[-+]?\d*\.?\d+', p.read_text())] if p.exists() else []
rtf = [float(x) for x in (o/'rtf.txt').read_text().split()][1:] if (o/'rtf.txt').exists() else []  # первый отсчёт отбрасываем: это устаревшее сообщение, не стационарный режим
cpu=[]; mem=[]
for l in (o/'docker_stats.txt').read_text().splitlines()[1:]:
    m = re.match(r'([\d.]+)% ([\d.]+)(\w+)', l)
    if m:
        cpu.append(float(m[1])/100); v=float(m[2]); mem.append(v*1024 if m[3].startswith('G') else v)
gpu=[tuple(float(x) for x in l.split(',')) for l in (o/'gpu_samples.txt').read_text().splitlines()[1:] if l.strip()]
idle = (o/'vram_idle.txt').read_text().strip()
def hz(p):
    t=(o/p).read_text() if (o/p).exists() else ''
    a=[float(x) for x in re.findall(r'average rate: ([\d.]+)', t)]; return (f'mean {s.mean(a):.2f} min {min(a):.2f} max {max(a):.2f} (windows={len(a)})' if a else 'n/a')
with open(o/'summary.txt','w') as f:
    def w(x): print(x); f.write(x+'\n')
    w(f'mode={mode}')
    if rtf: w(f'RTF samples={len(rtf)} mean={s.mean(rtf):.3f} min={min(rtf):.3f} max={max(rtf):.3f}')
    if cpu: w(f'CPU cores mean={s.mean(cpu):.2f} max={max(cpu):.2f}')
    if mem: w(f'RAM MiB mean={s.mean(mem):.0f} max={max(mem):.0f}')
    if gpu:
        w(f'VRAM idle(before)="{idle}" MiB, run mean={s.mean(g[0] for g in gpu):.0f} max={max(g[0] for g in gpu):.0f}; GPU util % mean={s.mean(g[1] for g in gpu):.0f} max={max(g[1] for g in gpu):.0f}')
    w(f'camera Hz={hz("hz_camera.txt")}  lidar Hz={hz("hz_lidar.txt")}')
PY
