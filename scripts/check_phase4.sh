#!/usr/bin/env bash
# Проверка Фазы 4 (всё внутри Docker, без окон, без участия человека):
#   scripts/check_phase4.sh [SEEDS="1 2 3 7"] [N=3] [MAX_SIM_S=900]
#   (make check-phase4 CHECK_SEEDS="1 2" ...)
# Для каждого seed два прогона поиска людей (как Фаза 3): localization:=gt (истинная поза) и
# localization:=slam (колёса + гироскоп + поправка mrg_slam). Затем ate_eval, map_eval, phase3_eval
# и сводка (phase4_summary.py). Результаты:
#   results/phase-4/check/seed_<S>_gt/, seed_<S>_slam/, summary.md, summary.json
# RGB=0: без YOLO по RGB (по умолчанию RGB=1).
set -uo pipefail
SEEDS="${1:-${SEEDS:-1 2 3 7}}"; N="${2:-${N:-3}}"; MAX_SIM_S="${3:-${MAX_SIM_S:-900}}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RGB="${RGB:-1}"
CHK="$ROOT/results/phase-4/check"; mkdir -p "$CHK"
NAME=rescueswarm-check-phase4
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
COMPOSE=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml")
cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT
dx() { docker exec "$NAME" bash -c "source /workspace/install/setup.bash; $*"; }
killall_in() { docker exec "$NAME" bash -c 'pkill -9 -f "[/]workspace/install"; pkill -9 -f "[g]z sim"; pkill -9 -f "[r]uby"; pkill -9 -f "[/]opt/ros"; true' >/dev/null 2>&1; }

run_one() {   # $1 = seed, $2 = gt|slam
  local S="$1" M="$2" D="seed_$1_$2" try=0
  local OUT="$CHK/$D" REL="results/phase-4/check/$D"
  while : ; do
    rm -rf "$OUT"; mkdir -p "$OUT"
    cleanup
    "${COMPOSE[@]}" run -d --rm -T --name "$NAME" dev sleep infinity >/dev/null
    sleep 3
    docker exec -d "$NAME" bash -c "cd /workspace && source install/setup.bash && ros2 launch rs_bringup explore.launch.py seed:=$S n:=$N headless:=true rviz:=false localization:=$M rgb:=$([ "$RGB" = 0 ] && echo false || echo true) target_coverage:=0.95 max_time:=$MAX_SIM_S out_dir:=/workspace/$REL > $REL/launch.log 2>&1"
    local t0=$SECONDS ok=0 started=0
    while [ $((SECONDS - t0)) -lt $((MAX_SIM_S * 13 / 10 + 240)) ]; do
      sleep 10
      cut -d' ' -f1 /proc/loadavg >> "$OUT/load.txt"
      dx "timeout 8 gz topic -e -t /world/rescue/stats -n 1 2>/dev/null | grep -E 'real_time_factor|sim_time' | tr '\n' ' '" >> "$OUT/rtf.txt" 2>/dev/null; echo >> "$OUT/rtf.txt"
      grep -q "EXPLORATION COMPLETE" "$OUT/launch.log" 2>/dev/null && { ok=1; break; }
      docker exec "$NAME" pgrep -f "ros2 launch" >/dev/null 2>&1 || break
      if [ $((SECONDS - t0)) -ge 150 ] && [ "$started" = 0 ]; then
        rows=$(wc -l < "$OUT/coverage.csv" 2>/dev/null || echo 0)
        if [ "$rows" -lt 20 ]; then break; else started=1; fi
      fi
    done
    sleep 8   # дать запуску погаснуть самому и записать people.json, final_map, pose_log
    if [ "$ok" = 1 ] || [ "$try" -ge 1 ]; then break; fi
    rows=$(wc -l < "$OUT/coverage.csv" 2>/dev/null || echo 0)
    if [ "$rows" -ge 20 ]; then break; fi
    echo "$D: сбой старта (строк coverage.csv: $rows), повтор"; try=$((try + 1))
    mv "$OUT/launch.log" "$CHK/failed_start_$D.log" 2>/dev/null
  done
  echo "$try" > "$OUT/retries.txt"
  dx "python3 -c \"import yaml,json;g=yaml.safe_load(open('/workspace/$REL/world/ground_truth.yaml'));json.dump({'people':g['people'],'decoys':g.get('decoys',[])},open('/workspace/$REL/gt.json','w'),indent=1)\"" > "$OUT/gt_dump.log" 2>&1
  killall_in; sleep 2
  grep -E "аукцион #|цель #|чёрный список|осмотр|кандидат|застряли|снятие с мели|exploration done" "$OUT/launch.log" > "$OUT/explorer_decisions.log" 2>/dev/null
  grep -E "people_fusion|SEARCH COMPLETE|EXPLORATION COMPLETE|process has died" "$OUT/launch.log" > "$OUT/search_events.log" 2>/dev/null
  python3 "$ROOT/scripts/phase3_eval.py" "$OUT" "$S" "$MAX_SIM_S" > "$OUT/eval.txt" 2>&1
  python3 "$ROOT/scripts/ate_eval.py" "$OUT" > "$OUT/ate.txt" 2>&1
  python3 "$ROOT/scripts/map_eval.py" "$OUT" > "$OUT/map_eval.txt" 2>&1
  echo "--- $D"; cat "$OUT/eval.txt" "$OUT/ate.txt" 2>/dev/null | grep -E "^(OK|FAIL)|^ground_|^все"
}

echo "== Фаза 4: seeds [$SEEDS], режимы gt и slam, N=$N, лимит $MAX_SIM_S сим. с, RGB=$RGB"
for S in $SEEDS; do
  for M in gt slam; do
    echo "== seed $S, $M"
    run_one "$S" "$M"
  done
done
cleanup
python3 "$ROOT/scripts/phase4_summary.py" "$CHK" "$SEEDS"
RC=$?
[ "$RC" = 0 ] && echo "ИТОГ: OK" || echo "ИТОГ: есть ошибки"
exit "$RC"
