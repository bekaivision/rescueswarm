#!/usr/bin/env bash
# Проверка Фазы 5 (всё внутри Docker, без окон, без участия человека):
#   scripts/check_phase5.sh [SEEDS="1 2 3 7"] [N=3] [MAX_SIM_S=900]
#   (make check-phase5 CHECK_SEEDS="1 2" DRONES=2 ...)
# Для каждого seed два прогона поиска людей с истинной позой наземных роботов:
#   ground - только N наземных роботов (как Фаза 3/4), mixed - те же роботы и DRONES дронов (DBACK: light|px4).
# Затем phase3_eval, map_eval, layers_eval (слои воздух/земля), ate_eval (позы дронов) и сводка
# (phase5_summary.py). Результаты: results/phase-5/check/seed_<S>_{ground,mixed}/, summary.md, summary.json;
# при DBACK=px4 каталог results/phase-5/check_px4/. MODES="mixed" пропускает прогоны только наземных роботов.
# Порядок прогонов на seed чередуется (ground/mixed, затем mixed/ground), чтобы нагрузка машины
# не давала систематического перекоса в сравнении.
set -uo pipefail
SEEDS="${1:-${SEEDS:-1 2 3 7}}"; N="${2:-${N:-3}}"; MAX_SIM_S="${3:-${MAX_SIM_S:-900}}"
DRONES="${DRONES:-2}"; DBACK="${DBACK:-light}"
# MODES="mixed" - только смешанный рой (например, для DBACK=px4, когда наземные прогоны уже есть)
MODES="${MODES:-ground mixed}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SUF=""; [ "$DBACK" = px4 ] && SUF="_px4"
CHK="$ROOT/results/phase-5/check$SUF"; mkdir -p "$CHK"
NAME=rescueswarm-check-phase5
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
COMPOSE=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml")
cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT
dx() { docker exec "$NAME" bash -c "source /workspace/install/setup.bash; $*"; }
# процессы PX4 и агента тоже (DBACK=px4)
killall_in() { docker exec "$NAME" bash -c 'pkill -9 -f "[/]workspace/install"; pkill -9 -f "[g]z sim"; pkill -9 -f "[r]uby"; pkill -9 -x px4; pkill -9 -f "[M]icroXRCEAgent"; pkill -9 -f "[/]opt/ros"; true' >/dev/null 2>&1; }

run_one() {   # $1 = seed, $2 = ground|mixed
  local S="$1" M="$2" D="seed_$1_$2" try=0
  local OUT="$CHK/$D" REL="results/phase-5/check$SUF/$D"
  local ARGS="drones:=0"
  [ "$M" = mixed ] && ARGS="drones:=$DRONES drone_backend:=$DBACK"
  while : ; do
    rm -rf "$OUT"; mkdir -p "$OUT"
    cleanup
    "${COMPOSE[@]}" run -d --rm -T --name "$NAME" dev sleep infinity >/dev/null
    sleep 3
    docker exec -d "$NAME" bash -c "cd /workspace && source install/setup.bash && ros2 launch rs_bringup explore.launch.py seed:=$S n:=$N headless:=true rviz:=false localization:=gt rgb:=true target_coverage:=0.95 max_time:=$MAX_SIM_S out_dir:=/workspace/$REL $ARGS > $REL/launch.log 2>&1"
    local t0=$SECONDS ok=0 started=0
    while [ $((SECONDS - t0)) -lt $((MAX_SIM_S * 13 / 10 + 240)) ]; do
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
  grep -E "air_explorer|drone_adapter" "$OUT/launch.log" > "$OUT/air_decisions.log" 2>/dev/null
  grep -E "people_fusion|SEARCH COMPLETE|EXPLORATION COMPLETE|process has died" "$OUT/launch.log" > "$OUT/search_events.log" 2>/dev/null
  python3 "$ROOT/scripts/phase3_eval.py" "$OUT" "$S" "$MAX_SIM_S" > "$OUT/eval.txt" 2>&1
  python3 "$ROOT/scripts/map_eval.py" "$OUT" > "$OUT/map_eval.txt" 2>&1
  if [ "$M" = mixed ]; then
    python3 "$ROOT/scripts/layers_eval.py" "$OUT" > "$OUT/layers_eval.txt" 2>&1
    python3 "$ROOT/scripts/ate_eval.py" "$OUT" > "$OUT/ate.txt" 2>&1
  fi
  echo "--- $D"; grep -E "^(OK|FAIL)" "$OUT/eval.txt"; grep -E "best_shift_m|agree_air" "$OUT/layers_eval.txt" 2>/dev/null
}

echo "== Фаза 5: seeds [$SEEDS], N=$N, дронов $DRONES ($DBACK), лимит $MAX_SIM_S сим. с"
k=0
for S in $SEEDS; do
  if [ $((k % 2)) = 0 ]; then ORDER="ground mixed"; else ORDER="mixed ground"; fi
  for M in $ORDER; do
    case " $MODES " in *" $M "*) ;; *) continue ;; esac
    echo "== seed $S, $M"
    run_one "$S" "$M"
  done
  k=$((k + 1))
done
cleanup
# без своих наземных прогонов (MODES="mixed") сравниваем с наземными прогонами основной проверки
for S in $SEEDS; do
  G="$ROOT/results/phase-5/check/seed_${S}_ground"
  [ -e "$CHK/seed_${S}_ground" ] || { [ -d "$G" ] && [ "$CHK" != "$ROOT/results/phase-5/check" ] && ln -s "$G" "$CHK/seed_${S}_ground"; }
done
python3 "$ROOT/scripts/phase5_summary.py" "$CHK" "$SEEDS"
RC=$?
[ "$RC" = 0 ] && echo "ИТОГ: OK" || echo "ИТОГ: есть ошибки"
exit "$RC"
