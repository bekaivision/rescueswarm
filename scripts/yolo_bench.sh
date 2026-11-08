#!/usr/bin/env bash
# Замер YOLO на кадрах симулятора. Запускать на хосте (всё считается в Docker):
#   scripts/yolo_bench.sh sanity|capture|eval|latency|all
# Результаты: results/phase-3b/{sanity,yolo_bench}/. Одновременно только одна симуляция.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAGE="${1:-all}"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
if docker ps --format '{{.Names}}' | grep -q '^rescueswarm-'; then
  echo "уже запущен контейнер rescueswarm-*: остановите его (одна симуляция за раз)" >&2; exit 1
fi
run() {  # run <подкоманда yolo_bench.py ...>
  docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml" run --rm -T -e RS_SANITY_DIR --name rescueswarm-yolobench dev \
    bash -c "source /workspace/install/setup.bash && cd /workspace/scripts && python3 yolo_bench.py $*"
}
case "$STAGE" in
  sanity) run sanity ;;
  capture) run capture-bench; run capture-scene ;;
  capture-scene) run capture-scene ;;
  eval) run eval ;;
  latency) run latency ;;
  all) run sanity; run capture-bench; run capture-scene; run eval; run latency ;;
  *) echo "usage: $0 sanity|capture|eval|latency|all" >&2; exit 2 ;;
esac
