#!/usr/bin/env bash
# Один прогон Фазы 7a: симуляция поиска людей в Docker (без окон) и оценка на хосте.
#   scripts/p7a_run.sh <каталог_результатов_от_корня_репозитория> <seed> [доп. аргументы launch...]
# Переменные окружения:
#   N=3  MAX_SIM_S=900  LOC=slam|gt (slam)  DRONES=0  DBACK=light|px4 (light)  COMMS= (пусто = связь выключена | ideal|good|medium|poor)
#   RGB=1  SETUP=/workspace/install/setup.bash (оверлей, например /workspace/install_p7/patched/setup.bash)
#   NAME=rescueswarm-p7a-run (имя контейнера)
# Оценки (хост): phase3_eval, map_eval всегда; ate_eval при LOC=slam; layers_eval при DRONES>0; phase6_eval при COMMS.
# Рядом с результатами пишется run_meta.json (параметры, время стены, повторы старта).
# Код возврата: 0 - прогон дошёл до конца и оценка записана, 1 - прогон не дал данных.
set -uo pipefail
[ $# -ge 2 ] || { echo "Использование: $0 <out_dir> <seed> [доп. аргументы launch...]" >&2; exit 2; }
REL="${1%/}"; S="$2"; shift 2
EXTRA=("$@")
N="${N:-3}"; MAX_SIM_S="${MAX_SIM_S:-900}"; LOC="${LOC:-slam}"; DRONES="${DRONES:-0}"; DBACK="${DBACK:-light}"
COMMS="${COMMS:-}"; RGB="${RGB:-1}"
SETUP="${SETUP:-/workspace/install/setup.bash}"; NAME="${NAME:-rescueswarm-p7a-run}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/$REL"; PARENT="$(dirname "$OUT")"
mkdir -p "$PARENT"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
COMPOSE=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml")
cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT
dx() { docker exec "$NAME" bash -c "source $SETUP; $*"; }
# процессы PX4 и агента тоже (DBACK=px4)
killall_in() { docker exec "$NAME" bash -c 'pkill -9 -f "[/]workspace/install"; pkill -9 -f "[g]z sim"; pkill -9 -f "[r]uby"; pkill -9 -x px4; pkill -9 -f "[M]icroXRCEAgent"; pkill -9 -f "[/]opt/ros"; true' >/dev/null 2>&1; }

# аргументы launch как в check_phase4/5/6
ARGS="seed:=$S n:=$N headless:=true rviz:=false localization:=$LOC rgb:=$([ "$RGB" = 0 ] && echo false || echo true)"
ARGS="$ARGS target_coverage:=0.95 max_time:=$MAX_SIM_S out_dir:=/workspace/$REL drones:=$DRONES"
[ "$DRONES" -gt 0 ] && ARGS="$ARGS drone_backend:=$DBACK"
[ -n "$COMMS" ] && ARGS="$ARGS comms:=true link:=$COMMS"
EXTRA_Q=""; [ ${#EXTRA[@]} -gt 0 ] && EXTRA_Q="$(printf '%q ' "${EXTRA[@]}")"

WALL_LIMIT=$((MAX_SIM_S * 3 + 600))   # медленный RTF (SLAM + PX4 + связь): до ~0.4
W0=$SECONDS; try=0
echo "== p7a: seed $S, N=$N LOC=$LOC DRONES=$DRONES DBACK=$DBACK COMMS=${COMMS:-off} RGB=$RGB, лимит $MAX_SIM_S сим. с, $REL"
while : ; do
  rm -rf "$OUT"; mkdir -p "$OUT"
  cleanup
  "${COMPOSE[@]}" run -d --rm -T --name "$NAME" dev sleep infinity >/dev/null
  sleep 3
  docker exec -d "$NAME" bash -c "cd /workspace && source $SETUP && ros2 launch rs_bringup explore.launch.py $ARGS $EXTRA_Q > $REL/launch.log 2>&1"
  t0=$SECONDS; ok=0; started=0
  while [ $((SECONDS - t0)) -lt "$WALL_LIMIT" ]; do
    sleep 10
    cut -d' ' -f1 /proc/loadavg >> "$OUT/load.txt"
    dx "timeout 8 gz topic -e -t /world/rescue/stats -n 1 2>/dev/null | grep -E 'real_time_factor|sim_time' | tr '\n' ' '" >> "$OUT/rtf.txt" 2>/dev/null; echo >> "$OUT/rtf.txt"
    if [ $(( (SECONDS - t0) % 120 )) -lt 10 ]; then
      { echo "--- $((SECONDS - t0)) s"; docker exec "$NAME" ps -eo pcpu,rss,args --sort=-pcpu | head -30 | cut -c1-150; } >> "$OUT/ps.txt" 2>/dev/null
      nvidia-smi --query-gpu=memory.used --format=csv,noheader >> "$OUT/vram.txt" 2>/dev/null
    fi
    grep -q "EXPLORATION COMPLETE" "$OUT/launch.log" 2>/dev/null && { ok=1; break; }
    docker exec "$NAME" pgrep -f "ros2 launch" >/dev/null 2>&1 || break
    # сбой старта: время симуляции не дошло до узлов (coverage.csv почти пустой)
    if [ $((SECONDS - t0)) -ge 240 ] && [ "$started" = 0 ]; then
      rows=$(wc -l < "$OUT/coverage.csv" 2>/dev/null || echo 0)
      if [ "$rows" -lt 20 ]; then break; else started=1; fi
    fi
  done
  sleep 8   # дать запуску погаснуть самому и записать people.json, final_map*, pose_log
  if [ "$ok" = 1 ] || [ "$try" -ge 1 ]; then break; fi
  rows=$(wc -l < "$OUT/coverage.csv" 2>/dev/null || echo 0)
  if [ "$rows" -ge 20 ]; then break; fi
  echo "$REL: сбой старта (строк coverage.csv: $rows), повтор"; try=$((try + 1))
  mv "$OUT/launch.log" "$PARENT/failed_start_$(basename "$OUT").log" 2>/dev/null
done
echo "$try" > "$OUT/retries.txt"
dx "python3 -c \"import yaml,json;g=yaml.safe_load(open('/workspace/$REL/world/ground_truth.yaml'));json.dump({'people':g['people'],'decoys':g.get('decoys',[])},open('/workspace/$REL/gt.json','w'),indent=1)\"" > "$OUT/gt_dump.log" 2>&1
killall_in; sleep 2
cleanup
grep -E "аукцион #|цель #|чёрный список|осмотр|кандидат|застряли|снятие с мели|exploration done" "$OUT/launch.log" > "$OUT/explorer_decisions.log" 2>/dev/null
grep -E "air_explorer|drone_adapter" "$OUT/launch.log" > "$OUT/air_decisions.log" 2>/dev/null
grep -E "people_fusion|SEARCH COMPLETE|EXPLORATION COMPLETE|process has died" "$OUT/launch.log" > "$OUT/search_events.log" 2>/dev/null
[ -n "$COMMS" ] && grep -E "comms_sim|comms_agent|SYNC|возврат на базу|на базе" "$OUT/launch.log" > "$OUT/comms_events.log" 2>/dev/null

python3 "$ROOT/scripts/phase3_eval.py" "$OUT" "$S" "$MAX_SIM_S" > "$OUT/eval.txt" 2>&1
python3 "$ROOT/scripts/map_eval.py" "$OUT" > "$OUT/map_eval.txt" 2>&1
[ "$LOC" = slam ] && python3 "$ROOT/scripts/ate_eval.py" "$OUT" > "$OUT/ate.txt" 2>&1
[ "$DRONES" -gt 0 ] && python3 "$ROOT/scripts/layers_eval.py" "$OUT" > "$OUT/layers_eval.txt" 2>&1
[ -n "$COMMS" ] && python3 "$ROOT/scripts/phase6_eval.py" "$OUT" > "$OUT/comms_eval.txt" 2>&1

WALL=$((SECONDS - W0))
P7A_SEED="$S" P7A_N="$N" P7A_LOC="$LOC" P7A_DRONES="$DRONES" P7A_DBACK="$DBACK" P7A_COMMS="$COMMS" P7A_RGB="$RGB" \
P7A_SETUP="$SETUP" P7A_WALL="$WALL" P7A_TRY="$try" P7A_MAX="$MAX_SIM_S" P7A_OUT="$OUT" \
python3 - "${EXTRA[@]+"${EXTRA[@]}"}" <<'PY'
import json, os, sys
e = os.environ
json.dump({'seed': int(e['P7A_SEED']), 'N': int(e['P7A_N']), 'LOC': e['P7A_LOC'], 'DRONES': int(e['P7A_DRONES']),
           'DBACK': e['P7A_DBACK'], 'COMMS': e['P7A_COMMS'], 'RGB': e['P7A_RGB'], 'SETUP': e['P7A_SETUP'],
           'MAX_SIM_S': int(e['P7A_MAX']), 'extra_args': sys.argv[1:], 'wall_s': int(e['P7A_WALL']),
           'retries': int(e['P7A_TRY'])}, open(e['P7A_OUT'] + '/run_meta.json', 'w'), indent=1)
PY
echo "--- $REL (wall $WALL с, повторов $try)"
grep -E "^(OK|FAIL)" "$OUT/eval.txt"
if [ -s "$OUT/check.json" ]; then echo "ПРОГОН: OK (check.json записан)"; exit 0; fi
echo "ПРОГОН: FAIL (нет check.json, см. $REL/launch.log и eval.txt)"; exit 1
