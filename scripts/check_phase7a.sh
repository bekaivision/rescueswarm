#!/usr/bin/env bash
# Проверка Фазы 7a (всё в Docker, без окон, без участия человека):
#   scripts/check_phase7a.sh   (make check-phase7a CHECK_SEEDS="1 2 3 7" PARTS="light px4 n5")
# Переменные: SEEDS="1 2 3 7", MAX_SIM_S=900, PARTS="light px4 n5", SETUP (оверлей, передаётся в p7a_run.sh).
# Части:
#   light - на каждом seed LOC=slam, 2 лёгких дрона, связь medium, N=3 -> seed_<S>_light/
#   px4   - то же с PX4 SITL -> seed_<S>_px4/
#   n5    - seed 1, N=5: смешанный рой со связью medium -> n5_seed_1/, и только наземные без связи -> n5_seed_1_ground/
# Результаты: results/phase-7a/check/, сводка summary.md и summary.json (phase7a_summary.py).
set -uo pipefail
SEEDS="${SEEDS:-1 2 3 7}"; MAX_SIM_S="${MAX_SIM_S:-900}"; PARTS="${PARTS:-light px4 n5}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REL="results/phase-7a/check"; mkdir -p "$ROOT/$REL"
export MAX_SIM_S
[ -n "${SETUP:-}" ] && export SETUP
has() { case " $PARTS " in *" $1 "*) return 0 ;; *) return 1 ;; esac; }

echo "== Фаза 7a: seeds [$SEEDS], части [$PARTS], лимит $MAX_SIM_S сим. с"
if has light; then
  for S in $SEEDS; do
    N=3 LOC=slam DRONES=2 DBACK=light COMMS=medium "$ROOT/scripts/p7a_run.sh" "$REL/seed_${S}_light" "$S"
  done
fi
if has px4; then
  for S in $SEEDS; do
    N=3 LOC=slam DRONES=2 DBACK=px4 COMMS=medium "$ROOT/scripts/p7a_run.sh" "$REL/seed_${S}_px4" "$S"
  done
fi
if has n5; then
  N=5 LOC=slam DRONES=2 DBACK=light COMMS=medium "$ROOT/scripts/p7a_run.sh" "$REL/n5_seed_1" 1
  N=5 LOC=slam DRONES=0 COMMS= "$ROOT/scripts/p7a_run.sh" "$REL/n5_seed_1_ground" 1
fi
python3 "$ROOT/scripts/phase7a_summary.py" "$ROOT/$REL" "$SEEDS"
RC=$?
[ "$RC" = 0 ] && echo "ИТОГ: OK" || echo "ИТОГ: есть ошибки"
exit "$RC"
