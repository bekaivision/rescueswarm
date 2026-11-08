#!/usr/bin/env bash
# Проверка Фазы 3 (всё внутри Docker, без окон, без участия человека):
#   scripts/check_phase3.sh [SEEDS="1 2 3 7"] [N=3] [MAX_SIM_S=900]
#   (make check-phase3 CHECK_SEEDS="1 2" ...)
# Для каждого seed: мир с ловушками, 3 робота ищут людей сами (исследование + осмотр кандидатов);
# потом сопоставление отметок с эталоном (phase3_eval.py). Результаты:
#   results/phase-3/check/seed_<S>/ и results/phase-3/check/summary.md
set -uo pipefail
SEEDS="${1:-${SEEDS:-1 2 3 7}}"; N="${2:-${N:-3}}"; MAX_SIM_S="${3:-${MAX_SIM_S:-900}}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CHK="$ROOT/results/phase-3/check"; mkdir -p "$CHK"
NAME=rescueswarm-check-phase3
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
COMPOSE=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml")
cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT
FAIL=0
dx() { docker exec "$NAME" bash -c "source /workspace/install/setup.bash; $*"; }
killall_in() { docker exec "$NAME" bash -c 'pkill -9 -f "[/]workspace/install"; pkill -9 -f "[g]z sim"; pkill -9 -f "[r]uby"; pkill -9 -f "[/]opt/ros"; true' >/dev/null 2>&1; }

run_seed() {   # $1 = seed
  local S="$1" OUT="$CHK/seed_$1" REL="results/phase-3/check/seed_$1" try=0
  while : ; do
    rm -rf "$OUT"; mkdir -p "$OUT"
    cleanup
    "${COMPOSE[@]}" run -d --rm -T --name "$NAME" dev sleep infinity >/dev/null
    sleep 3
    docker exec -d "$NAME" bash -c "cd /workspace && source install/setup.bash && ros2 launch rs_bringup explore.launch.py seed:=$S n:=$N headless:=true rviz:=false target_coverage:=0.95 max_time:=$MAX_SIM_S out_dir:=/workspace/$REL > $REL/launch.log 2>&1"
    local t0=$SECONDS ok=0 started=0
    while [ $((SECONDS - t0)) -lt $((MAX_SIM_S * 13 / 10 + 240)) ]; do
      sleep 10
      dx "timeout 8 gz topic -e -t /world/rescue/stats -n 1 2>/dev/null | grep -E 'real_time_factor|sim_time' | tr '\n' ' '" >> "$OUT/rtf.txt" 2>/dev/null; echo >> "$OUT/rtf.txt"
      grep -q "EXPLORATION COMPLETE" "$OUT/launch.log" 2>/dev/null && { ok=1; break; }
      docker exec "$NAME" pgrep -f "ros2 launch" >/dev/null 2>&1 || break
      if [ $((SECONDS - t0)) -ge 150 ] && [ "$started" = 0 ]; then
        rows=$(wc -l < "$OUT/coverage.csv" 2>/dev/null || echo 0)
        if [ "$rows" -lt 20 ]; then break; else started=1; fi
      fi
    done
    sleep 8   # дать запуску погаснуть самому (launch завершает всё после COMPLETE) и записать people.json
    if [ "$ok" = 1 ] || [ "$try" -ge 1 ]; then break; fi
    rows=$(wc -l < "$OUT/coverage.csv" 2>/dev/null || echo 0)
    if [ "$rows" -ge 20 ]; then break; fi
    echo "seed $S: сбой старта (строк coverage.csv: $rows), повтор"; try=$((try + 1))
    mv "$OUT/launch.log" "$CHK/failed_start_seed_${S}.log" 2>/dev/null
  done
  echo "$try" > "$OUT/retries.txt"
  # эталон -> json (yaml есть только в контейнере)
  dx "python3 -c \"import yaml,json;g=yaml.safe_load(open('/workspace/$REL/world/ground_truth.yaml'));json.dump({'people':g['people'],'decoys':g.get('decoys',[])},open('/workspace/$REL/gt.json','w'),indent=1)\"" > "$OUT/gt_dump.log" 2>&1
  killall_in; sleep 2
  cp "$OUT/world/preview.png" "$OUT/world_preview.png" 2>/dev/null
  grep -E "аукцион #|цель #|чёрный список|осмотр|кандидат|застряли|снятие с мели|exploration done" "$OUT/launch.log" > "$OUT/explorer_decisions.log" 2>/dev/null
  grep -E "people_fusion|SEARCH COMPLETE|EXPLORATION COMPLETE|process has died" "$OUT/launch.log" > "$OUT/search_events.log" 2>/dev/null
  python3 "$ROOT/scripts/phase3_eval.py" "$OUT" "$S" "$MAX_SIM_S" | tee "$OUT/eval.txt"
  return "${PIPESTATUS[0]}"
}

echo "== Фаза 3: seeds [$SEEDS], N=$N, лимит $MAX_SIM_S сим. с"
for S in $SEEDS; do
  echo "== seed $S"
  run_seed "$S" || FAIL=1
done
cleanup
python3 "$ROOT/scripts/phase3_eval.py" --summary "$CHK" "$SEEDS" || FAIL=1
[ "$FAIL" = 0 ] && echo "ИТОГ: OK" || echo "ИТОГ: есть ошибки"
exit "$FAIL"
