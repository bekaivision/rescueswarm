#!/bin/bash
# Воспроизведение записи (лидар + колёсная одометрия) через slam.launch.py без симулятора, для опытов с mrg_slam.
# Запуск (на хосте): scripts/slam_replay.sh <bag_в_контейнере> <out_dir_в_контейнере> "<x,y,yaw;...>" [аргументы slam.launch.py]
# Пример: scripts/slam_replay.sh /workspace/results/phase-4/diag/bag_explore_p0 /workspace/results/phase-4/diag/rp_base "..." n:=3
# Скорость проигрывания: RATE (по умолчанию 1.0). OVERLAY=/workspace/путь/setup.bash - подключить другой install поверх основного.
# Другой ROS_DOMAIN_ID (DOM, по умолчанию 77), чтобы не пересекаться с симуляцией и можно было гнать несколько воспроизведений параллельно.
BAG=$1; OUT=$2; POSES=$3; shift 3
RATE=${RATE:-1.0}
cd "$(dirname "$0")/.."
docker compose -p rescueswarm -f docker/compose.yaml run --rm -T dev bash -c "
source /workspace/install/setup.bash
if [ -n '$OVERLAY' ]; then source $OVERLAY; fi
export GZ_PARTITION=replay${DOM} ROS_DOMAIN_ID=${DOM:-77}
rm -rf $OUT; mkdir -p $OUT
python3 /workspace/scripts/slam_tfrelay.py > $OUT/relay.log 2>&1 &
ros2 launch rs_bringup slam.launch.py poses:='$POSES' out_dir:=$OUT $* > $OUT/launch.log 2>&1 &
LP=\$!
sleep 12
ros2 bag play $BAG --clock --rate $RATE --remap /tf:=/tf_bag --exclude-topics /clock > $OUT/play.log 2>&1
sleep 3
kill -INT \$LP; sleep 4
" 2>&1 | grep -v Container
