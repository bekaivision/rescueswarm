#!/usr/bin/env bash
# Видео Фазы 0 -> media/phase-0.mp4. Запуск с хоста из любого каталога (или: make video PHASE=0).
#   scripts/video/phase0.sh [--rerecord]   --rerecord: заново снять кадры Gazebo
# Всё (Gazebo, ffmpeg) работает в Docker. Сырые кадры: media/raw/phase-0/ (кэш).
# Внутри контейнера скрипт вызывается с аргументом record или compose.
set -euo pipefail

FPS=30
FRAMES=330                      # 11 с симуляции при 30 Гц
WORLD=/workspace/configs/worlds/phase0_video.sdf
RAW=/workspace/media/raw/phase-0
FONT=/usr/share/fonts/truetype/dejavu
BG=0x0b0f14                     # цвет фона логотипа

record() {                      # внутри контейнера: Gazebo без окна, камера пишет PNG (тег <save> в мире)
  rm -rf "$RAW/frames" "$RAW/seq"; mkdir -p "$RAW"
  gz sim -s --headless-rendering "$WORLD" >"$RAW/gz.log" 2>&1 &
  sleep 12
  gz topic -e -t /video_cam >/dev/null 2>&1 &     # сенсор рендерит только при подписчике
  sleep 3
  # симуляция стартует на паузе: снимаем с момента t=0
  gz service -s /world/phase0_video/control --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false' >/dev/null
  for _ in $(seq 1 180); do
    [ "$(ls "$RAW/frames" 2>/dev/null | wc -l)" -ge "$FRAMES" ] && break; sleep 1
  done
  kill %1 %2 2>/dev/null || true; wait 2>/dev/null || true
  # camera_N.png -> seq/frame_00000.png (ffmpeg не любит двоеточия и нет нулей в именах)
  mkdir -p "$RAW/seq"
  for i in $(seq 0 $((FRAMES-1))); do
    mv "$RAW/frames/video_cam::link::camera_$i.png" "$RAW/seq/frame_$(printf %05d "$i").png"
  done
  rm -rf "$RAW/frames"
}

# drawtext из файла: так не нужно экранировать двоеточия и кавычки
T() { printf '%s' "$2" > "$TXT/$1.txt"; }
dt() {  # имя файла, размер, цвет, x, y, шрифт, доп. параметры
  echo "drawtext=fontfile=$FONT/$6:textfile=$TXT/$1.txt:fontsize=$2:fontcolor=$3:x=$4:y=$5${7:+:$7}"
}
BOLD=DejaVuSans-Bold.ttf; MONO=DejaVuSansMono.ttf; MONOB=DejaVuSansMono-Bold.ttf

badge() { dt sim 36 white 30 "$1" $BOLD "box=1:boxcolor=0x000000@0.6:boxborderw=14"; }

compose() {                     # внутри контейнера: сборка роликов и склейка
  OUT=$RAW/parts; TXT=$RAW/txt; mkdir -p "$OUT" "$TXT"
  ENC=(-c:v libx264 -preset slow -crf 14 -pix_fmt yuv420p -r $FPS -an)
  T sim "Simulation"
  T c1t "Gazebo Harmonic in Docker"; T c1b "Rendered on the NVIDIA GPU"
  T c3t "Headless GPU camera, 30 Hz"; T c3b "Measured: RTF 0.99 | 153 MiB VRAM | camera 30.2 Hz"
  T c2t "One command"; T c2b "Gazebo runs on the NVIDIA GPU in Docker"
  T end1 "Bekai Vision"; T end2 "RescueSwarm  |  Phase 0"

  # --- шот 1 (0-4 с): кадры 0-119, сразу результат
  ffmpeg -loglevel error -y -framerate $FPS -start_number 0 -i "$RAW/seq/frame_%05d.png" -frames:v 120 \
    -vf "pad=1080:1080:0:135:$BG,$(dt c1t 60 white '(w-text_w)/2' 38 $BOLD),$(dt c1b 46 0x2dd4bf '(w-text_w)/2' 990 $BOLD),$(badge 160)" \
    "${ENC[@]}" "$OUT/1.mp4"

  # --- шот 2 (6 с): терминал, команда печатается по буквам, дальше строки реального вывода
  cmd="make sim"; f="color=c=$BG:s=1080x1080:r=$FPS:d=6,drawbox=x=40:y=250:w=1000:h=500:color=0x161c24:t=fill,drawbox=x=40:y=250:w=1000:h=500:color=0x2dd4bf:t=3"
  f="$f,$(dt c2t 72 white '(w-text_w)/2' 50 $BOLD),$(dt c2b 40 0x2dd4bf '(w-text_w)/2' 960 $BOLD),$(badge 160)"
  y=290
  for n in $(seq 1 ${#cmd}); do                      # печать "$ make sim"
    T "p$n" "\$ ${cmd:0:n}"; s=$(awk "BEGIN{print 0.4+$n*0.1}"); e=$(awk "BEGIN{print $s+0.1}")
    [ "$n" -eq ${#cmd} ] && en="gte(t,$s)" || en="between(t,$s,$e)"
    f="$f,$(dt "p$n" 34 white 80 $y $MONOB "enable='$en'")"
  done
  T l2 "\$ glxinfo -B | grep renderer";                  f="$f,$(dt l2 30 white 80 400 $MONOB "enable='gte(t,2)'")"
  T l3 "OpenGL renderer: NVIDIA GeForce RTX 3060 Laptop GPU"; f="$f,$(dt l3 25 0x2dd4bf 80 450 $MONO "enable='gte(t,2.6)'")"
  T l4 "\$ nvidia-smi";                                   f="$f,$(dt l4 30 white 80 560 $MONOB "enable='gte(t,3.4)'")"
  T l5 "G  gz sim gui    272MiB";                         f="$f,$(dt l5 30 0x2dd4bf 80 610 $MONO "enable='gte(t,4)'")"
  T l6 "(GPU process list; G = graphics)";                f="$f,$(dt l6 24 0x8b97a5 80 660 $MONO "enable='gte(t,4)'")"
  ffmpeg -loglevel error -y -f lavfi -i "$f" "${ENC[@]}" "$OUT/2.mp4"

  # --- шот 3 (7 с): кадры 120-329 и измеренные числа
  ffmpeg -loglevel error -y -framerate $FPS -start_number 120 -i "$RAW/seq/frame_%05d.png" -frames:v 210 \
    -vf "pad=1080:1080:0:135:$BG,$(dt c3t 56 white '(w-text_w)/2' 40 $BOLD),$(dt c3b 31 0x2dd4bf '(w-text_w)/2' 1005 $BOLD),$(badge 160)" \
    "${ENC[@]}" "$OUT/3.mp4"

  # --- шот 4 (3.5 с): логотип
  ffmpeg -loglevel error -y -f lavfi -i "color=c=$BG:s=1080x1080:r=$FPS:d=3.5" -i /workspace/assets/brand/bekai-logo-dark.png \
    -filter_complex "[1:v]scale=640:640:flags=lanczos[l];[0:v][l]overlay=220:90,$(dt end1 70 white '(w-text_w)/2' 790 $BOLD),$(dt end2 44 0x2dd4bf '(w-text_w)/2' 890 $BOLD)" \
    "${ENC[@]}" "$OUT/4.mp4"

  printf "file '%s'\n" "$OUT/1.mp4" "$OUT/2.mp4" "$OUT/3.mp4" "$OUT/4.mp4" > "$OUT/list.txt"
  ffmpeg -loglevel error -y -f concat -safe 0 -i "$OUT/list.txt" -c:v libx264 -preset slow -crf 18 \
    -pix_fmt yuv420p -r $FPS -movflags +faststart -an /workspace/media/phase-0.mp4
}

case "${1:-}" in
  record) record; exit ;;
  compose) compose; exit ;;
esac

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
RUN=(docker compose -p rescueswarm -f "$ROOT/docker/compose.yaml" run --rm -T)
mkdir -p "$ROOT/media/raw/phase-0"
if [ "${1:-}" = --rerecord ] || [ ! -f "$ROOT/media/raw/phase-0/seq/frame_00329.png" ]; then
  echo "Съёмка кадров Gazebo (headless, GPU)..."
  "${RUN[@]}" --name rescueswarm-video-rec dev bash scripts/video/phase0.sh record
fi
echo "Сборка media/phase-0.mp4..."
"${RUN[@]}" dev bash scripts/video/phase0.sh compose
echo "Готово: $ROOT/media/phase-0.mp4"
