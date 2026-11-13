#!/usr/bin/env bash
# Проверка Фазы 6 (всё внутри Docker, без окон, без участия человека):
#   scripts/check_phase6.sh [SEEDS="1 2 3 7"] [N=3] [MAX_SIM_S=900]
#   (make check-phase6 CHECK_SEEDS="1 2" DRONES=2 LEVELS="ideal good medium poor" ...)
# Для каждого seed и каждого уровня связи (LEVELS: ideal good medium poor) один прогон поиска людей смешанным
# роем (N наземных роботов, DRONES лёгких дронов) с радиосвязью через comms_sim (comms:=true link:=<уровень>).
# Порядок уровней на seed сдвигается по кругу, чтобы нагрузка машины не давала систематического перекоса.
# Тест обрыва связи (OUTAGE=1): только наземные роботы, link=OUTAGE_LINK, outages=OUTAGE_SPEC, seed OUTAGE_SEED.
# Затем phase3_eval, map_eval, phase6_eval (трафик, связность, задержки) и сводка (phase6_summary.py).
# Результаты: results/phase-6/check/seed_<S>_<уровень>/, outage_seed_<S>/, summary.md, summary.json.
set -uo pipefail
SEEDS="${1:-${SEEDS:-1 2 3 7}}"; N="${2:-${N:-3}}"; MAX_SIM_S="${3:-${MAX_SIM_S:-900}}"
DRONES="${DRONES:-2}"
LEVELS="${LEVELS:-ideal good medium poor}"
OUTAGE="${OUTAGE:-1}"; OUTAGE_SEED="${OUTAGE_SEED:-1}"; OUTAGE_SPEC="${OUTAGE_SPEC:-ground_1:60:200}"; OUTAGE_LINK="${OUTAGE_LINK:-ideal}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CHK="$ROOT/results/phase-6/check"; mkdir -p "$CHK"
NAME=rescueswarm-check-phase6
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
COMPOSE=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml")
cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT
dx() { docker exec "$NAME" bash -c "source /workspace/install/setup.bash; $*"; }
# процессы PX4 и агента тоже
killall_in() { docker exec "$NAME" bash -c 'pkill -9 -f "[/]workspace/install"; pkill -9 -f "[g]z sim"; pkill -9 -f "[r]uby"; pkill -9 -x px4; pkill -9 -f "[M]icroXRCEAgent"; pkill -9 -f "[/]opt/ros"; true' >/dev/null 2>&1; }

run_one() {   # $1 = seed, $2 = уровень связи, $3 = каталог прогона, $4 = дроны (число), $5 = outages (может быть пустым)
  local S="$1" L="$2" D="$3" try=0
  local OUT="$CHK/$D" REL="results/phase-6/check/$D"
  local ARGS="drones:=$4 drone_backend:=light comms:=true link:=$L"
  [ -n "${5:-}" ] && ARGS="$ARGS outages:='$5'"
  while : ; do
    rm -rf "$OUT"; mkdir -p "$OUT"
    cleanup
    "${COMPOSE[@]}" run -d --rm -T --name "$NAME" dev sleep infinity >/dev/null
    sleep 3
    docker exec -d "$NAME" bash -c "cd /workspace && source install/setup.bash && ros2 launch rs_bringup explore.launch.py seed:=$S n:=$N headless:=true rviz:=false localization:=gt rgb:=true target_coverage:=0.95 max_time:=$MAX_SIM_S out_dir:=/workspace/$REL $ARGS > $REL/launch.log 2>&1"
    local t0=$SECONDS ok=0 started=0
    while [ $((SECONDS - t0)) -lt $((MAX_SIM_S * 13 / 10 + 240 + 150)) ]; do
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
      if [ $((SECONDS - t0)) -ge 150 ] && [ "$started" = 0 ]; then
        rows=$(wc -l < "$OUT/coverage.csv" 2>/dev/null || echo 0)
        if [ "$rows" -lt 20 ]; then break; else started=1; fi
      fi
    done
    sleep 8   # дать запуску погаснуть самому и записать people.json, final_map*, pose_log
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
  grep -E "comms_sim|comms_agent|SYNC|возврат на базу|на базе" "$OUT/launch.log" > "$OUT/comms_events.log" 2>/dev/null
  python3 "$ROOT/scripts/phase3_eval.py" "$OUT" "$S" "$MAX_SIM_S" > "$OUT/eval.txt" 2>&1
  python3 "$ROOT/scripts/map_eval.py" "$OUT" > "$OUT/map_eval.txt" 2>&1
  python3 "$ROOT/scripts/phase6_eval.py" "$OUT" > "$OUT/comms_eval.txt" 2>&1
  echo "--- $D"; grep -E "^(OK|FAIL)" "$OUT/eval.txt"; head -8 "$OUT/comms_eval.txt" 2>/dev/null
}

echo "== Фаза 6: seeds [$SEEDS], N=$N, дронов $DRONES, уровни [$LEVELS], лимит $MAX_SIM_S сим. с"
read -r -a LV <<< "$LEVELS"
nl=${#LV[@]}
k=0
for S in $SEEDS; do
  for ((j = 0; j < nl; j++)); do
    L="${LV[$(( (j + k) % nl ))]}"
    echo "== seed $S, связь $L"
    run_one "$S" "$L" "seed_${S}_$L" "$DRONES" ""
  done
  k=$((k + 1))
done
if [ "$OUTAGE" = 1 ]; then
  echo "== тест обрыва связи: seed $OUTAGE_SEED, link $OUTAGE_LINK, outages $OUTAGE_SPEC, только наземные роботы"
  run_one "$OUTAGE_SEED" "$OUTAGE_LINK" "outage_seed_$OUTAGE_SEED" 0 "$OUTAGE_SPEC"
fi
cleanup
python3 "$ROOT/scripts/phase6_summary.py" "$CHK" "$SEEDS" "$LEVELS"
RC=$?
[ "$RC" = 0 ] && echo "ИТОГ: OK" || echo "ИТОГ: есть ошибки"
exit "$RC"
