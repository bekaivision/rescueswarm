#!/usr/bin/env bash
# Замер VRAM и RTF: симуляция поиска (3 робота, headless) отдельно и вместе с YOLO.
#   scripts/yolo_vram.sh [SEED=1] [N=3] [SECONDS_PER_PHASE=45] [IMGSZ=640]
# Результаты: results/phase-3b/vram/ (samples.csv, events.csv, yolo_*.json, summary.md).
# Фазы: sim -> sim+yolo11n -> sim+yolo11s -> sim+обе -> sim (контроль). target_coverage:=2.0, чтобы сим не завершился раньше. Одна симуляция за раз; контейнер удаляется в конце.
set -euo pipefail
SEED="${1:-1}"; N="${2:-3}"; PH="${3:-45}"; SZ="${4:-640}"; WARM=70
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/results/phase-3b/vram"; rm -rf "$OUT"; mkdir -p "$OUT"
NAME=rescueswarm-search-headless
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
if docker ps --format '{{.Names}}' | grep -q '^rescueswarm-'; then
  echo "уже запущен контейнер rescueswarm-*: остановите его" >&2; exit 1; fi
trap 'docker rm -f "$NAME" >/dev/null 2>&1 || true; kill $(jobs -p) 2>/dev/null || true' EXIT
echo "idle_mib=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits)" > "$OUT/idle.txt"
MAXT=$((WARM + 6 * PH + 200))
docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml" run -d --rm -T --name "$NAME" dev bash -c \
  "source /workspace/install/setup.bash && ros2 launch rs_bringup explore.launch.py seed:=$SEED n:=$N headless:=true rviz:=false max_time:=$MAXT target_coverage:=2.0 out_dir:=/workspace/results/phase-3b/vram/sim_run" > /dev/null
dx() { docker exec "$NAME" bash -c "source /workspace/install/setup.bash; $*"; }
echo "t,phase,mem_mib,util_pct" > "$OUT/samples.csv"; 
PHASE=warmup; echo "$PHASE" > "$OUT/.phase"
( while true; do
    echo "$(date +%s),$(cat "$OUT/.phase"),$(nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader,nounits | tr -d ' ')" >> "$OUT/samples.csv"; sleep 1; done ) &
docker exec -d "$NAME" bash -c "source /workspace/install/setup.bash; python3 /workspace/scripts/yolo_vram_rtf.py /workspace/results/phase-3b/vram/rtf_raw.csv $((WARM + 6 * PH + 100))"
echo "прогрев ${WARM} с"; sleep "$WARM"
dx 'ros2 topic list | grep -c rgb/image_raw' > "$OUT/rgb_topics.txt" || true
phase() { echo "$1" > "$OUT/.phase"; echo "$(date +%s),$1" >> "$OUT/events.csv"; echo "фаза $1"; }
phase sim_only; sleep "$PH"
phase sim_yolo11n; dx "python3 /workspace/scripts/yolo_vram_load.py --models yolo11n --seconds $PH --imgsz $SZ" > "$OUT/yolo_n.json"
phase sim_yolo11s; dx "python3 /workspace/scripts/yolo_vram_load.py --models yolo11s --seconds $PH --imgsz $SZ" > "$OUT/yolo_s.json"
phase sim_yolo11n_s; dx "python3 /workspace/scripts/yolo_vram_load.py --models yolo11n,yolo11s --seconds $PH --imgsz $SZ" > "$OUT/yolo_ns.json"
phase sim_only_end; sleep "$PH"
phase end; sleep 3
docker logs "$NAME" > "$OUT/launch.log" 2>&1 || true
python3 - "$OUT" <<'PY'
import sys, csv, statistics as st, collections
o = sys.argv[1]
mem = collections.defaultdict(list); util = collections.defaultdict(list); rtf = collections.defaultdict(list)
for r in csv.DictReader(open(o + '/samples.csv')):
    mem[r['phase']].append(float(r['mem_mib'])); util[r['phase']].append(float(r['util_pct']))
ev = [tuple(l.strip().split(',')) for l in open(o + '/events.csv')]
def phase_of(t):
    cur = 'warmup'
    for te, name in ev:
        if t >= int(te):
            cur = name
    return cur
for r in csv.DictReader(open(o + '/rtf_raw.csv')):
    rtf[phase_of(int(r['t']))].append(float(r['rtf']))
idle = open(o + '/idle.txt').read().strip()
L = ['# VRAM и RTF', '', idle + ' (до запуска)', '',
     '| фаза | VRAM пик, МиБ | VRAM среднее, МиБ | GPU util среднее, % | RTF среднее | RTF мин | замеров RTF |', '|---|---|---|---|---|---|---|']
for p in ('warmup', 'sim_only', 'sim_yolo11n', 'sim_yolo11s', 'sim_yolo11n_s', 'sim_only_end'):
    if p in mem:
        r = rtf.get(p, [])
        L.append(f'| {p} | {max(mem[p]):.0f} | {st.mean(mem[p]):.0f} | {st.mean(util[p]):.0f} | '
                 f'{st.mean(r) if r else float("nan"):.3f} | {min(r) if r else float("nan"):.3f} | {len(r)} |')
open(o + '/summary.md', 'w').write('\n'.join(L) + '\n'); print('\n'.join(L))
PY
