#!/usr/bin/env bash
# Проверка Фазы 2 (всё внутри Docker, без окон, без участия человека):
#   scripts/check_phase2.sh [SEEDS="1 2 3 7"] [N=3] [MAX_SIM_S=900]
#   (переменные окружения SEEDS, N, MAX_SIM_S тоже работают; make check-phase2 CHECK_SEEDS="1 2" ...)
# Для каждого seed: мир генерируется, 3 робота исследуют его сами; потом проверки (см. phase2_eval.py).
# Результаты: results/phase-2/check/seed_<S>/ и results/phase-2/check/summary.md
set -uo pipefail
SEEDS="${1:-${SEEDS:-1 2 3 7}}"; N="${2:-${N:-3}}"; MAX_SIM_S="${3:-${MAX_SIM_S:-900}}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CHK="$ROOT/results/phase-2/check"; mkdir -p "$CHK"
NAME=rescueswarm-check-phase2
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
COMPOSE=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml")
cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT
FAIL=0
dx() { docker exec "$NAME" bash -c "source /workspace/install/setup.bash; $*"; }
killall_in() { docker exec "$NAME" bash -c 'pkill -9 -f "[/]workspace/install"; pkill -9 -f "[g]z sim"; pkill -9 -f "[r]uby"; pkill -9 -f "[/]opt/ros"; pkill -9 -f "[p]hase2_"; true' >/dev/null 2>&1; }

run_seed() {   # $1 = seed; печатает число повторов в $OUT/retries.txt
  local S="$1" OUT="$CHK/seed_$1" REL="results/phase-2/check/seed_$1" try=0
  while : ; do
    rm -rf "$OUT"; mkdir -p "$OUT"
    cleanup
    "${COMPOSE[@]}" run -d --rm -T --name "$NAME" dev sleep infinity >/dev/null
    sleep 3   # контейнер новый: чистить нечего (pkill в первые секунды убил бы его точку входа)
    docker exec -d "$NAME" bash -c "cd /workspace && source install/setup.bash && ros2 launch rs_bringup explore.launch.py seed:=$S n:=$N headless:=true rviz:=false target_coverage:=0.95 max_time:=$MAX_SIM_S out_dir:=/workspace/$REL > $REL/launch.log 2>&1"
    docker exec -d "$NAME" bash -c "cd /workspace && source install/setup.bash && until [ -f $REL/world/gt_map.yaml ]; do sleep 1; done; python3 scripts/phase2_monitor.py $REL/safety.json $REL/world $N > $REL/monitor.log 2>&1"
    docker exec -d "$NAME" bash -c "cd /workspace && python3 scripts/phase2_procstat.py $REL/procstat.jsonl $((MAX_SIM_S * 2 + 400)) 5"
    local t0=$SECONDS ok=0 started=0
    while [ $((SECONDS - t0)) -lt $((MAX_SIM_S * 13 / 10 + 240)) ]; do
      sleep 10
      dx "timeout 8 gz topic -e -t /world/rescue/stats -n 1 2>/dev/null | grep -E 'real_time_factor|sim_time' | tr '\n' ' '" >> "$OUT/rtf.txt" 2>/dev/null; echo >> "$OUT/rtf.txt"
      grep -q "EXPLORATION COMPLETE" "$OUT/launch.log" 2>/dev/null && { ok=1; break; }
      docker exec "$NAME" pgrep -f "ros2 launch" >/dev/null 2>&1 || break
      # сбой старта: через 150 с реального времени покрытие не идёт
      if [ $((SECONDS - t0)) -ge 150 ] && [ "$started" = 0 ]; then
        rows=$(wc -l < "$OUT/coverage.csv" 2>/dev/null || echo 0)
        if [ "$rows" -lt 20 ]; then break; else started=1; fi
      fi
    done
    docker exec "$NAME" pkill -f phase2_procstat >/dev/null 2>&1
    sleep 8   # дать запуску погаснуть самому (launch завершает всё после COMPLETE)
    docker logs "$NAME" >/dev/null 2>&1
    if [ "$ok" = 1 ] || [ "$try" -ge 1 ]; then break; fi
    # покрытие не успело/старт не удался: если это сбой старта, повторяем один раз
    rows=$(wc -l < "$OUT/coverage.csv" 2>/dev/null || echo 0)
    if [ "$rows" -ge 20 ]; then break; fi
    echo "seed $S: сбой старта (строк coverage.csv: $rows), повтор"; try=$((try + 1))
    mv "$OUT/launch.log" "$CHK/failed_start_seed_${S}.log" 2>/dev/null
  done
  # данные для проверок, пока контейнер жив
  echo "$try" > "$OUT/retries.txt"
  dx "cd /tmp && rm -rf gen_a && python3 -m rs_worlds.generate --seed $S --out /tmp/gen_a --size medium >/dev/null && sha256sum /tmp/gen_a/world.sdf /workspace/$REL/world/world.sdf" > "$OUT/determinism.txt" 2>&1
  local h1 h2; h1=$(sed -n 1p "$OUT/determinism.txt" | cut -d' ' -f1); h2=$(sed -n 2p "$OUT/determinism.txt" | cut -d' ' -f1)
  [ -n "$h1" ] && [ "$h1" = "$h2" ] && echo 1 > "$OUT/determinism_ok.txt" || echo 0 > "$OUT/determinism_ok.txt"
  # публикации в cmd_vel (разовый снимок в конце, если узлы ещё живы)
  dx "for i in \$(seq 1 $N); do ros2 topic info -v /ground_\$i/cmd_vel; done" > "$OUT/cmd_vel_publishers_end.txt" 2>&1
  killall_in; sleep 2
  cp "$OUT/world/preview.png" "$OUT/world_preview.png" 2>/dev/null
  grep -E "аукцион #|цель #|чёрный список|застряли|снятие с мели|exploration done" "$OUT/launch.log" > "$OUT/explorer_decisions.log" 2>/dev/null
  grep -E "process has died|EXPLORATION COMPLETE" "$OUT/launch.log" > "$OUT/exit_events.log" 2>/dev/null
  python3 "$ROOT/scripts/phase2_eval.py" "$OUT" "$S" "$MAX_SIM_S" "$N" "$(cat "$OUT/determinism_ok.txt")" "$try" | tee "$OUT/eval.txt"
  return "${PIPESTATUS[0]}"
}

echo "== Фаза 2: seeds [$SEEDS], N=$N, лимит $MAX_SIM_S сим. с"
for S in $SEEDS; do
  echo "== seed $S"
  run_seed "$S" || FAIL=1
done
cleanup

python3 - "$CHK" "$SEEDS" "$MAX_SIM_S" <<'PY'
import json, sys
chk, seeds, mx = sys.argv[1], sys.argv[2].split(), sys.argv[3]
L = [f'# Проверка Фазы 2 (seeds {" ".join(seeds)}, лимит {mx} сим. с)\n',
     '| seed | итог % | t50 | t80 | t90 | t95 | путь g1/g2/g3, м | мин. расст. между роботами, м | касания стен | снятий с мели | чёрный список | уступок / ожиданий | повторы старта | RTF ср./мин | детерм. | ошибки |',
     '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
f = lambda v: '-' if v is None else f'{v:.0f}'
for s in seeds:
    try: c = json.load(open(f'{chk}/seed_{s}/check.json'))
    except Exception: L.append(f'| {s} | нет данных |' + ' |' * 14); continue
    sf = c.get('safety', {})
    L.append(f"| {s} | {c['final_coverage']:.1%} | {f(c['t50'])} | {f(c['t80'])} | {f(c['t90'])} | {f(c['t95'])} | "
             + ' / '.join(f'{v:.0f}' for v in c['dist'].values()) + f" | {sf.get('min_robot_distance_m', '-')} | "
             + ' / '.join(str(v) for v in sf.get('wall_touches', {}).values()) + ' | '
             + ' / '.join(str(v) for v in c['unstick'].values()) + ' | ' + ' / '.join(str(v) for v in c['blacklist'].values())
             + f" | {sum(c.get('yields', {}).values())} / {sum(c.get('waits', {}).values())} | {c['retries']} | {c['rtf_mean']} / {c['rtf_min']} | {'да' if c['deterministic'] else 'НЕТ'} | {len(c['fails'])} |")
fl = []
for s in seeds:
    try: fl += json.load(open(f'{chk}/seed_{s}/check.json')).get('fails', [])
    except Exception: fl.append(f'seed {s}: нет check.json')
L.append('\nОшибки:' if fl else '\nОшибок нет.')
L += [f'- {x}' for x in fl]
open(f'{chk}/summary.md', 'w').write('\n'.join(L) + '\n')
print('\n'.join(L))
PY
[ "$FAIL" = 0 ] && echo "ИТОГ: OK" || echo "ИТОГ: есть ошибки"
exit "$FAIL"
