#!/usr/bin/env bash
# Один экземпляр PX4 SITL для дрона, который уже создан в мире (режим attach: PX4_GZ_MODEL_NAME).
#   px4_instance.sh <экземпляр> <имя модели = имя дрона> <имя мира>
# Запускает rs_bringup/robots.launch.py при drone_backend:=px4. Агент MicroXRCEAgent (UDP 8888)
# запускается там же один раз на все экземпляры, до PX4.
# Рабочий каталог PX4 (параметры, журналы) вне /opt/px4: /tmp/rs_px4/<имя>.
# Пространство имён ROS топиков PX4 = имя дрона: /<имя>/fmu/in/..., /<имя>/fmu/out/...
# MAV_SYS_ID = экземпляр + 1 (так задаёт PX4), его ждёт drone_adapter в параметре target_system.
set -u
I="$1"; NAME="$2"; WORLD="$3"
[ -n "$NAME" ] || { echo "px4_instance: пустое имя модели" >&2; exit 2; }
BIN="${PX4_ROOT:-/opt/px4}/build/px4_sitl_default"
WD="/tmp/rs_px4/$NAME"
rm -rf -- "$WD"; mkdir -p "$WD"; cd "$WD" || exit 2

# ждём, пока модель появится в мире (её IMU нужен PX4 сразу при подключении)
IMU="/world/$WORLD/model/$NAME/link/base_link/sensor/imu_sensor/imu"
for _ in $(seq 1 180); do gz topic -l 2>/dev/null | grep -qx "$IMU" && break; sleep 1; done
gz topic -l 2>/dev/null | grep -qx "$IMU" || { echo "px4_instance: нет $IMU" >&2; exit 3; }

# Без пульта и без наземной станции: offboard не считается потерей RC (COM_RCL_EXCEPT=4).
# EKF2_GPS_DELAY=0: GPS в Gazebo без задержки (как в SIH-airframe самого PX4). При умолчании 110 мс EKF
# сдвигает позицию вперёд на 0.11 с * скорость, и облака глубины ложатся в мир со сдвигом (0.15 м на 1.5 м/с).
# Значение 0 отличается от умолчания, поэтому через env сохраняется (airframe 4001 его не трогает).
export PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$WORLD" PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_NAME="$NAME" \
       PX4_UXRCE_DDS_NS="$NAME" PX4_PARAM_COM_RCL_EXCEPT=4 PX4_PARAM_NAV_RCL_ACT=1 PX4_PARAM_COM_DISARM_PRFLT=60 \
       PX4_PARAM_EKF2_GPS_DELAY=0
"$BIN/bin/px4" -i "$I" -d "$BIN/etc" < /dev/null &
PID=$!
trap 'kill -TERM "$PID" 2>/dev/null; wait "$PID"; exit 0' INT TERM

# NAV_DLL_ACT=0 (нет наземной станции) нельзя задать через PX4_PARAM_*: env применяется до airframe,
# значение 0 совпадает с умолчанием и не сохраняется, а airframe 4001 потом ставит умолчание 2,
# и арминг блокируется ("No connection to the GCS"). Ставим через клиент px4-param, но только после
# того, как airframe отработал (значение стало 2): если поставить раньше, airframe его перезапишет.
dll() { "$BIN/bin/px4-param" --instance "$I" show NAV_DLL_ACT 2>/dev/null | sed -n 's/.*NAV_DLL_ACT.*: *\([0-9-]*\).*/\1/p'; }
for _ in $(seq 1 120); do
  [ "$(dll)" = 2 ] && break
  sleep 1
done
for _ in $(seq 1 10); do
  "$BIN/bin/px4-param" --instance "$I" set NAV_DLL_ACT 0 >/dev/null 2>&1
  [ "$(dll)" = 0 ] && break
  sleep 1
done
echo "px4_instance $NAME: $("$BIN/bin/px4-param" --instance "$I" show NAV_DLL_ACT 2>/dev/null | grep NAV_DLL_ACT)"
echo "px4_instance $NAME: $("$BIN/bin/px4-param" --instance "$I" show EKF2_GPS_DELAY 2>/dev/null | grep EKF2_GPS_DELAY)"
wait "$PID"
