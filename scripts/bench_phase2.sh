#!/usr/bin/env bash
# Замер Фазы 2 во время активного исследования (seed 1). Запускать на хосте:
#   scripts/bench_phase2.sh headless|gui [DURATION_S=120] [N=3]
# Результаты: results/phase-2/bench_<mode>/ (сырые данные) и summary.txt.
# gui: Gazebo с окном + RViz (explore.rviz), снимки окон через xwd -id.
set -euo pipefail
MODE="${1:-headless}"; DUR="${2:-120}"; N="${3:-3}"; WARM=30; SEED=1
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REL="results/phase-2/bench_$MODE"; OUT="$ROOT/$REL"; rm -rf "$OUT"; mkdir -p "$OUT"
NAME="rescueswarm-bench2-$MODE"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
COMPOSE=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml")
docker rm -f "$NAME" >/dev/null 2>&1 || true
trap 'docker rm -f "$NAME" >/dev/null 2>&1 || true' EXIT
dx() { docker exec "$NAME" bash -c "source /workspace/install/setup.bash; $*"; }

{ echo "# host before start"; uptime; top -b -n 1 -o %CPU | head -12; } > "$OUT/idle.txt"
nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader,nounits > "$OUT/vram_idle.txt"

if [ "$MODE" = headless ]; then H=true; R=false; else H=false; R=true; fi
"${COMPOSE[@]}" run -d --rm -T --name "$NAME" dev sleep infinity > "$OUT/container_id.txt"
docker exec "$NAME" bash -c 'pkill -9 -f /workspace/install; pkill -9 -f "gz sim"; pkill -9 -f ruby; true' >/dev/null 2>&1 || true
# цель 1.0 недостижима: исследование не завершится само в окне замера
docker exec -d "$NAME" bash -c "cd /workspace && source install/setup.bash && ros2 launch rs_bringup explore.launch.py seed:=$SEED n:=$N headless:=$H rviz:=$R target_coverage:=1.0 out_dir:=/workspace/$REL/run > $REL/launch.log 2>&1"
echo "прогрев ${WARM} с"; sleep "$WARM"
docker exec -d "$NAME" bash -c "cd /workspace && python3 scripts/phase2_procstat.py $REL/procstat.jsonl $DUR 5"
START=$SECONDS

( while [ $((SECONDS - START)) -lt "$DUR" ]; do
    dx "timeout 8 gz topic -e -t /world/rescue/stats -n 1 2>/dev/null | grep -E 'real_time_factor' | awk '{print \$2}'" >> "$OUT/rtf.txt" 2>/dev/null || true
    sleep 2; done ) &
( echo "cpu_pct mem_mib" > "$OUT/docker_stats.txt"
  while [ $((SECONDS - START)) -lt "$DUR" ]; do
    docker stats --no-stream --format '{{.CPUPerc}} {{.MemUsage}}' "$NAME" >> "$OUT/docker_stats.txt"; done ) &
( echo "mem_mib util_pct" > "$OUT/gpu_samples.txt"
  while [ $((SECONDS - START)) -lt "$DUR" ]; do
    nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader,nounits >> "$OUT/gpu_samples.txt"; sleep 1; done ) &

if [ "$MODE" = gui ]; then
  snap() {  # $1 = суффикс
    dx 'xwininfo -root -tree' > "$OUT/xwininfo_tree.txt" 2>&1 || true
    for w in "RViz" "Gazebo Sim"; do
      id=$(grep -i "\"$w" "$OUT/xwininfo_tree.txt" | grep -v "has no name" | head -1 | awk '{print $1}')
      f="${w// /_}_$1"
      if [ -n "$id" ]; then
        dx "xwd -id $id -silent > /tmp/$f.xwd && ffmpeg -loglevel error -y -i /tmp/$f.xwd /workspace/$REL/$f.png" || echo "снимок $w не удался" | tee -a "$OUT/notes.txt"
      else echo "окно $w не найдено" | tee -a "$OUT/notes.txt"; fi
    done
  }
  sleep $((DUR / 3)); snap early
  sleep $((DUR / 3)); snap late
  # кадр всего экрана на случай, если окно RViz закрыто другим окном
  dx "ffmpeg -loglevel error -y -f x11grab -i \$DISPLAY -frames:v 1 /workspace/$REL/screen.png" || true
fi
wait
dx 'ros2 topic list' > "$OUT/topics.txt" 2>&1 || true
dx 'nvidia-smi pmon -c 1' > "$OUT/pmon_end.txt" 2>&1 || true
tail -1 "$OUT/run/coverage.csv" > "$OUT/coverage_last.txt" 2>/dev/null || true
cp "$OUT/launch.log" "$OUT/launch_tmp.log"
docker rm -f "$NAME" >/dev/null 2>&1 || true
mv "$OUT/launch_tmp.log" "$OUT/launch.log" 2>/dev/null || true

python3 - "$OUT" "$MODE" <<'PY'
import json, re, sys, statistics as s, pathlib
o = pathlib.Path(sys.argv[1]); mode = sys.argv[2]
rtf = [float(x) for x in (o/'rtf.txt').read_text().split()] if (o/'rtf.txt').exists() else []
ps = [json.loads(l) for l in (o/'procstat.jsonl').read_text().splitlines() if l.strip()]
cpu=[]; mem=[]
for l in (o/'docker_stats.txt').read_text().splitlines()[1:]:
    m = re.match(r'([\d.]+)% ([\d.]+)(\w+)', l)
    if m:
        cpu.append(float(m[1])/100); v=float(m[2]); mem.append(v*1024 if m[3].startswith('G') else v)
gpu=[tuple(float(x) for x in l.split(',')) for l in (o/'gpu_samples.txt').read_text().splitlines()[1:] if l.strip()]
idle=(o/'vram_idle.txt').read_text().strip().split(',')[0]
with open(o/'summary.txt','w') as f:
    def w(x): print(x); f.write(x+'\n')
    w(f'mode={mode} seed=1 N=3')
    if rtf: w(f'RTF samples={len(rtf)} mean={s.mean(rtf):.3f} min={min(rtf):.3f} max={max(rtf):.3f}')
    if ps:
        w(f'CPU cores (по /proc, сумма процессов контейнера) mean={s.mean(p["total_cores"] for p in ps):.2f} max={max(p["total_cores"] for p in ps):.2f} (n={len(ps)})')
        gs = sorted({g for p in ps for g in p['groups']})
        for g in gs: w(f'  группа {g}: {s.mean(p["groups"].get(g,0.0) for p in ps):.2f} ядра')
        w(f'RSS MiB (сумма процессов контейнера) mean={s.mean(p["rss_mib"] for p in ps):.0f} max={max(p["rss_mib"] for p in ps)}')
    if cpu: w(f'docker stats: CPU cores mean={s.mean(cpu):.2f} max={max(cpu):.2f}; RAM MiB mean={s.mean(mem):.0f} max={max(mem):.0f}')
    if gpu: w(f'VRAM idle(before)={idle} MiB, run mean={s.mean(g[0] for g in gpu):.0f} max={max(g[0] for g in gpu):.0f}; GPU util % mean={s.mean(g[1] for g in gpu):.0f} max={max(g[1] for g in gpu):.0f}')
    cl = o/'coverage_last.txt'
    if cl.exists(): w('покрытие в конце окна (sim_time, coverage, ...): ' + cl.read_text().strip())
PY
