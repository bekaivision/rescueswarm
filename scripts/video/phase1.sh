#!/usr/bin/env bash
# Видео Фазы 1 -> media/phase-1.mp4. Запуск с хоста из любого каталога (или: make video PHASE=1).
#   scripts/video/phase1.sh [--rerecord]   --rerecord: заново снять Gazebo и RViz
# Всё (Gazebo, ROS 2, RViz, ffmpeg) работает в Docker. Сырьё: media/raw/phase-1/ (кэш).
# Съёмка идёт в два прохода одного сценария движения (scripts/video/phase1_drive.py, время
# симуляции): 1) обзорная камера Gazebo (кадры PNG по времени симуляции, headless);
# 2) окно RViz, захват экрана x11grab (нужен DISPLAY; окно RViz появится на экране ~15 с).
# Внутри контейнера скрипт вызывается с аргументом record или compose.
set -euo pipefail

FPS=30
FRAMES=360                      # 12 с симуляции при 30 Гц (Gazebo)
RVIZ_SEC=12.5                   # длительность захвата RViz, с
WORLD=/workspace/src/rs_bringup/worlds/phase1_video.sdf
RAW=/workspace/media/raw/phase-1
FONT=/usr/share/fonts/truetype/dejavu
BG=0x0b0f14                     # цвет фона логотипа

ros() { set +u; source /workspace/install/setup.bash; set -u; }

stop_all() {                    # остановить launch и всё, что он запустил
  kill -INT "$LPID" 2>/dev/null || true; sleep 4
  pkill -9 -f "gz sim|ros_gz|rviz2|robot_state_publisher|static_transform_publisher|parameter_bridge" 2>/dev/null || true
  pkill -f "gz-transport-topic" 2>/dev/null || true
  wait "$LPID" 2>/dev/null || true; sleep 2
}

launch() {                      # launch робота: headless, без RViz (RViz запускаем сами)
  ros2 launch rs_bringup robots.launch.py n:=3 headless:=true rviz:=false world:=$WORLD \
    >"$RAW/launch_$1.log" 2>&1 &
  LPID=$!
  sleep 28                      # прогрев: Gazebo, мосты, сенсоры
}

record_cam() {                  # проход 1: обзорная камера Gazebo
  rm -rf "$RAW/frames" "$RAW/seq"; mkdir -p "$RAW"
  launch cam
  gz topic -e -t /video_cam >/dev/null 2>&1 &        # сенсор рендерит только при подписчике
  sleep 3
  n0=$(ls "$RAW/frames" | wc -l)                      # кадры до старта движения не берём
  echo "кадров до старта движения: $n0"
  python3 /workspace/scripts/video/phase1_drive.py 13.5 | tee "$RAW/drive_cam.log"
  for _ in $(seq 1 30); do
    [ "$(ls "$RAW/frames" | wc -l)" -ge $((n0 + FRAMES)) ] && break; sleep 1
  done
  stop_all
  mkdir -p "$RAW/seq"
  for i in $(seq 0 $((FRAMES-1))); do
    mv "$RAW/frames/video_cam::link::camera_$((n0 + i)).png" "$RAW/seq/frame_$(printf %05d "$i").png"
  done
  rm -rf "$RAW/frames"
}

record_rviz() {                 # проход 2: окно RViz, захват экрана
  rm -f "$RAW/rviz.mkv" "$RAW/geom.txt"
  launch rviz
  rviz2 -d /workspace/src/rs_bringup/rviz/video.rviz --ros-args -p use_sim_time:=true \
    >"$RAW/rviz.log" 2>&1 &
  sleep 14                      # окно открывается и разворачивается
  # геометрия: 3D-вид (самое большое окно rviz2) и две панели картинок 320 px шириной
  xwininfo -root -tree | python3 -c '
import re, sys
w = []
for l in sys.stdin:
    m = re.search(r"\"rviz2\": .*?\s(\d+)x(\d+)\+(-?\d+)\+(-?\d+)\s+\+(-?\d+)\+(-?\d+)", l)
    if m: w.append(tuple(map(int, m.groups())))
view = max(w, key=lambda t: t[0] * t[1])
panels = sorted([t for t in w if t[0] == 320 and t[1] > 300], key=lambda t: t[5])
assert len(panels) >= 2, w
print("view", view[4], view[5], view[0], view[1])
for k, p in zip(("panel1", "panel2"), panels):   # сверху вниз; что есть что, решит compose по цвету
    print(k, p[4], p[5], p[0], p[1])
' | tee "$RAW/geom.txt"
  sleep 4
  python3 /workspace/scripts/video/phase1_drive.py 13 >"$RAW/drive_rviz.log" &
  DPID=$!
  ffmpeg -loglevel error -y -f x11grab -framerate $FPS -draw_mouse 0 -video_size 1920x1080 -i "$DISPLAY" \
    -t $RVIZ_SEC -c:v libx264 -preset ultrafast -crf 12 -pix_fmt yuv444p "$RAW/rviz.mkv"
  wait "$DPID" 2>/dev/null || true
  stop_all
}

record() { ros; record_cam; record_rviz; }

# drawtext из файла: так не нужно экранировать двоеточия и кавычки
T() { printf '%s' "$2" > "$TXT/$1.txt"; }
dt() {  # имя файла, размер, цвет, x, y, шрифт, доп. параметры
  echo "drawtext=fontfile=$FONT/$6:textfile=$TXT/$1.txt:fontsize=$2:fontcolor=$3:x=$4:y=$5${7:+:$7}"
}
BOLD=DejaVuSans-Bold.ttf

badge() { dt sim 36 white 30 "$1" $BOLD "box=1:boxcolor=0x000000@0.6:boxborderw=14"; }

compose() {                     # внутри контейнера: сборка роликов и склейка
  OUT=$RAW/parts; TXT=$RAW/txt; mkdir -p "$OUT" "$TXT"
  ENC=(-c:v libx264 -preset slow -crf 14 -pix_fmt yuv420p -r $FPS -an)
  T sim "Simulation"
  T a1 "3 ground robots"; T a2 "ROS 2 Jazzy + Gazebo Harmonic"
  T a3 "Independent control, separate namespaces"
  T a4 "Each robot: 3D LiDAR | RGB | thermal | IMU"
  T b1 "Live in RViz: LiDAR of all 3 robots"
  T l1 "ground_1 thermal (160x120)"; T l2 "ground_1 RGB (320x240)"
  T end1 "Bekai Vision"; T end2 "RescueSwarm  |  Phase 1"

  # --- шот 1 (12 с): обзорная камера Gazebo, результат виден с первой секунды
  ffmpeg -loglevel error -y -framerate $FPS -start_number 0 -i "$RAW/seq/frame_%05d.png" -frames:v $FRAMES \
    -vf "crop=850:637:130:130,scale=1080:810:flags=lanczos,pad=1080:1080:0:135:$BG,$(dt a1 56 white '(w-text_w)/2' 22 $BOLD "enable='lt(t,6)'"),$(dt a2 38 0x2dd4bf '(w-text_w)/2' 86 $BOLD "enable='lt(t,6)'"),$(dt a3 40 white '(w-text_w)/2' 55 $BOLD "enable='gte(t,6)'"),$(dt a4 38 0x2dd4bf '(w-text_w)/2' 990 $BOLD),$(badge 160)" \
    "${ENC[@]}" "$OUT/1.mp4"

  # --- шот 2 (10 с): окно RViz: 3D-вид сверху, под ним картинки RGB и тепловизора ground_1
  read -r _ vx vy vw vh < <(grep '^view' "$RAW/geom.txt")
  # порядок панелей RViz от запуска к запуску разный: цветная (RGB) отличается от серого тепловизора насыщенностью
  read -r _ px1 py1 pw1 ph1 < <(grep '^panel1' "$RAW/geom.txt")
  read -r _ px2 py2 pw2 ph2 < <(grep '^panel2' "$RAW/geom.txt")
  sat=$(ffmpeg -loglevel error -ss 3 -i "$RAW/rviz.mkv" -frames:v 1 \
    -vf "crop=316:236:$((px1 + 2)):$((py1 + (ph1 - 240) / 2 + 2)),signalstats,metadata=print:key=lavfi.signalstats.SATAVG:file=-" -f null - \
    | sed -n 's/.*SATAVG=\([0-9]*\).*/\1/p' | head -1)
  echo "насыщенность панели 1: ${sat:-?}"
  if [ "${sat:-0}" -gt 4 ]; then          # панель 1 цветная -> RGB
    rx=$px1; ry=$py1; rh=$ph1; tx=$px2; ty=$py2; th=$ph2
  else
    tx=$px1; ty=$py1; th=$ph1; rx=$px2; ry=$py2; rh=$ph2
  fi
  ch=$((vw * 570 / 1080)); [ "$ch" -gt "$vh" ] && ch=$vh   # 3D-вид режем под полосу 1080x570
  ffmpeg -loglevel error -y -ss 0.4 -t 10 -i "$RAW/rviz.mkv" -filter_complex "
    color=c=$BG:s=1080x1080:r=$FPS[bg];
    [0:v]split=3[a][b][c];
    [a]crop=$vw:$ch:$vx:$vy,scale=1080:570:flags=lanczos[v];
    [b]crop=316:236:$((tx + 2)):$((ty + (th - 240) / 2 + 2)),scale=470:352:flags=lanczos[t];
    [c]crop=316:236:$((rx + 2)):$((ry + (rh - 240) / 2 + 2)),scale=470:352:flags=lanczos[r];
    [bg][v]overlay=0:105:shortest=1[o1];[o1][t]overlay=40:690[o2];[o2][r]overlay=570:690,
    $(dt b1 50 white '(w-text_w)/2' 30 $BOLD),$(dt l1 26 white 52 702 $BOLD 'box=1:boxcolor=0x000000@0.6:boxborderw=6'),$(dt l2 26 white 582 702 $BOLD 'box=1:boxcolor=0x000000@0.6:boxborderw=6'),$(badge 120)" \
    "${ENC[@]}" "$OUT/2.mp4"

  # --- шот 3 (3.5 с): логотип
  ffmpeg -loglevel error -y -f lavfi -i "color=c=$BG:s=1080x1080:r=$FPS:d=3.5" -i /workspace/assets/brand/bekai-logo-dark.png \
    -filter_complex "[1:v]scale=640:640:flags=lanczos[l];[0:v][l]overlay=220:90,$(dt end1 70 white '(w-text_w)/2' 790 $BOLD),$(dt end2 44 0x2dd4bf '(w-text_w)/2' 890 $BOLD)" \
    "${ENC[@]}" "$OUT/3.mp4"

  printf "file '%s'\n" "$OUT/1.mp4" "$OUT/2.mp4" "$OUT/3.mp4" > "$OUT/list.txt"
  ffmpeg -loglevel error -y -f concat -safe 0 -i "$OUT/list.txt" -c:v libx264 -preset slow -crf 18 \
    -pix_fmt yuv420p -r $FPS -movflags +faststart -an /workspace/media/phase-1.mp4
}

case "${1:-}" in
  record) record; exit ;;
  compose) compose; exit ;;
esac

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
RUN=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml" run --rm -T)
mkdir -p "$ROOT/media/raw/phase-1"
if [ "${1:-}" = --rerecord ] || [ ! -f "$ROOT/media/raw/phase-1/seq/frame_00359.png" ] \
   || [ ! -f "$ROOT/media/raw/phase-1/rviz.mkv" ] || [ ! -f "$ROOT/media/raw/phase-1/geom.txt" ]; then
  echo "Съёмка Gazebo (headless) и RViz (окно появится на экране, не закрывайте его ~1 мин)..."
  "${RUN[@]}" --name rescueswarm-video-rec dev bash scripts/video/phase1.sh record
fi
echo "Сборка media/phase-1.mp4..."
"${RUN[@]}" dev bash scripts/video/phase1.sh compose
echo "Готово: $ROOT/media/phase-1.mp4"
