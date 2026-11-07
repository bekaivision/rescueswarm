# Фаза 1: отчёт

Дата проверки: 2026-11-07. Сырые логи, кадры и замеры лежат в `results/phase-1/`
(не в git). Всё выполнялось в Docker, на хост ничего не ставилось.

## Что сделано

- `src/rs_description/`: робот с дифференциальным приводом (`urdf/ground_robot.urdf.xacro`).
  Сенсоры: 3D LiDAR (`gpu_lidar`, 16 лучей x 360 точек, 10 Гц), RGB 320x240 при 5 Гц,
  тепловизор 160x120 при 5 Гц (формат L16), IMU 50 Гц. Плагины Gazebo: DiffDrive,
  JointStatePublisher. Частоты и разрешения задаются аргументами xacro.
- `src/rs_bringup/`: `launch/robots.launch.py`, мир `worlds/phase1.sdf`, `rviz/robots.rviz`.
  Launch поднимает Gazebo, `/clock` (один мост на всех) и для каждого робота `ground_i`:
  свой `robot_state_publisher` (namespace и `frame_prefix`), свой `ros_gz_bridge`,
  статический TF `world -> ground_i/odom` по позе старта.
- Мир: земля, красный куб, синий цилиндр, две стены и два «нагретых» тела
  (310 K и 330 K, плагин Thermal), которые видит тепловизор.
- Makefile: `sim-robots`, `sim-robots-headless`, `teleop`, `check-phase1`; `make build`
  собирает `src/`. В образ добавлен `ros-jazzy-teleop-twist-keyboard`.
- Скрипты: `scripts/check_phase1.sh` (проверка), `scripts/phase1_probe.py` (помощник),
  `scripts/bench_phase1.sh` (замер).

Кадры TF: `world -> ground_i/odom -> ground_i/base_link -> ...`. Имена звеньев в URDF
простые, префикс `ground_i/` добавляют `frame_prefix` и Gazebo (`frame_id` сенсоров, odom).

## Как запустить

```bash
make build
make sim-robots                   # окно Gazebo + RViz, 3 робота (N=5 для пяти)
make sim-robots-headless          # без окон
make teleop ROBOT=ground_2        # второй терминал: клавиатура -> /ground_2/cmd_vel
make check-phase1                 # скриптовая проверка
scripts/bench_phase1.sh headless 60 3
scripts/bench_phase1.sh gui 45 3
```

Параметры launch: `n`, `headless`, `rviz`, `world`, `spacing`, а также `lidar_rate`,
`lidar_samples`, `lidar_beams`, `rgb_rate`, `rgb_width`, `rgb_height`, `thermal_rate`,
`thermal_width`, `thermal_height`, `imu_rate`.

Вручную: `ros2 launch rs_bringup robots.launch.py n:=3 headless:=true rviz:=false`.

## Критерии готовности

| Критерий | Статус | Доказательство |
| --- | --- | --- |
| Модель с приводом, LiDAR, RGB, тепловизором, IMU | выполнено | `topics.txt`, `frames.txt` |
| Топики роботов в отдельных пространствах имён | выполнено | `topics.txt`: по 10 топиков у `/ground_1`, `/ground_2`, `/ground_3` |
| 3 робота управляются независимо | выполнено | `independent_motion.json` (таблица ниже) |
| TF без конфликтов, дерево валидно | выполнено | `tf2_echo.txt` (`world -> ground_2/base_link`), `tf_frames.pdf`, `tf_frames.gv` |
| RGB и тепловизор дают не пустые кадры | выполнено | `ground_1_rgb.png`, `ground_1_thermal.png`, `ground_1_thermal_raw16.png`, `frames.txt` |
| RViz показывает всех роботов | выполнено | `bench_gui/RViz.png` |
| Окно Gazebo с 3 роботами | выполнено | `gazebo_gui.png` |
| Управление с клавиатуры | частично | `make teleop` запускает `teleop_twist_keyboard` с remap на `/ROBOT/cmd_vel`; нажатие клавиш в автоматической проверке не имитировалось (нужен TTY). Тот же топик `cmd_vel` проверен скриптом |
| `make test` проходит | выполнено | 0 тестов, 0 ошибок (lint-тесты не добавлялись) |

### Частоты (ground_1, `ros2 topic hz`, `hz.txt`, `bench_headless/summary.txt`)

| Топик | Цель, Гц | Измерено, Гц (среднее, окно 55 с) |
| --- | --- | --- |
| `/ground_1/lidar/points` | 10 | 10.00 |
| `/ground_1/rgb/image_raw` | 5 | 5.00 |
| `/ground_1/thermal/image_raw` | 5 | 5.00 |
| `/ground_1/imu` | 50 | 50.00 |
| `/ground_1/odom` | 30 | 29.41 |

Odom чуть ниже 30 Гц: шаг физики 2 мс, публикация идёт на ближайшем шаге (17 шагов = 34 мс).

### Независимое управление (`independent_motion.json`, 6 с команды)

| Робот | Команда | Ожидание | Результат odom |
| --- | --- | --- | --- |
| ground_1 | v = 0.3 м/с | около 1.8 м вперёд | 1.783 м вперёд, поворот 0.0 град |
| ground_2 | w = 0.5 рад/с | около 172 град на месте | 170.6 град, смещение 0.0 м |
| ground_3 | 0 | стоит | смещение 0.0 м, поворот 0.0 град |

(Небольшой недобор из-за разгона: заданы пределы ускорения.)

## Замеры (3 робота, 20 с прогрева)

Скрипт `scripts/bench_phase1.sh`. Headless: 60 с. GUI (окно Gazebo + RViz с 2 картинками
и тремя облаками точек): 45 с. Мост подписан на все сенсоры, поэтому они рендерят всегда.

| Показатель | Headless | GUI (gz + RViz) |
| --- | --- | --- |
| RTF, среднее | 1.000 (39 отсчётов) | 1.003 (28 отсчётов) |
| RTF, минимум | 0.998 | 0.995 |
| CPU, ядра (среднее / максимум) | 1.93 / 6.65 | 3.78 / 8.82 |
| RAM, MiB (среднее / максимум) | 572 / 609 | 975 / 1020 |
| VRAM до запуска | 16 MiB | 16 MiB |
| VRAM во время | 215 MiB | 547 MiB |
| Загрузка GPU, % (среднее / максимум) | 3 / 4 | 37 / 40 |

Пики CPU (6.65 и 8.82) короткие: `docker stats` и `docker exec` измерительных скриптов
добавляют нагрузку; среднее ближе к реальному. Измерения на обычной рабочей машине (идёт
другая работа), не на полностью простаивающей. Подробности: `bench_*/summary.txt`, `idle.txt`.

RTF около 1.0, поэтому стартовые значения сенсоров не менялись.

## Отклонения от PLAN.md

- Топик `/tf` общий для всех роботов (кадры разделены префиксом `ground_i/`), а не
  `/ground_i/tf`. Так проще для RViz и Nav2. Остальные топики лежат в пространствах имён.
  Для строгой изоляции в Фазе 6 (`comms_sim`) это придётся пересмотреть.
- Общий кадр `world` создан статическими TF по позам старта (odom каждого робота начинается
  в нуле его позы старта). Это не оценка положения, а ground truth, годится до Фазы 4.
- Датчик тепловизора в URDF имеет тип `thermal` (не `thermal_camera`): иначе плагин Thermal
  не влияет на картинку. Температура задаётся плагином в `<visual>` тел.
- RViz-конфиг статический на 3 робота; картинки RGB и тепла включены только у ground_1
  (остальные выключены, включаются галочкой). Для другого N облака и модели надо добавить в RViz.
- Поза старта: `y = (i - (n+1)/2) * spacing`, `x = 0`, без поворота.

## Известные проблемы

- `robot_state_publisher` пишет предупреждение kdl_parser: у корневого звена `base_link` есть
  инерция. На работу не влияет.
- sdformat пишет предупреждение про `gz_frame_id` («not defined in SDF. Copying»); frame_id
  сенсоров при этом правильные (`ground_1/lidar_link` и т. д.), проверено `ros2 topic echo`.
- В Gazebo звенья без подвижных суставов склеиваются с `base_link` (сенсоры лежат в нём);
  в TF от `robot_state_publisher` они остаются отдельными кадрами.
- Скриншот окна Gazebo в `bench_gui/Gazebo_Sim.png` получился закрыт окном RViz (окна лежат
  в одном месте), поэтому отдельный снимок сделан без RViz: `gazebo_gui.png`.
- Тепловизор: значения L16 в сотых долях кельвина (28815 = 288.15 K); для просмотра
  картинка нормируется по min и max кадра.
- Окно клавиатурного управления нужно запускать во втором терминале, TTY обязателен.

## Как перепроверить

```bash
make image && make build
make check-phase1
scripts/bench_phase1.sh headless 60 3
scripts/bench_phase1.sh gui 45 3
make sim-robots                  # смотреть вживую
make teleop ROBOT=ground_2       # во втором терминале
```

## Видео

Файл `media/phase-1.mp4` (не в git), 25.6 с, H.264 yuv420p, 1080x1080, 30 к/с, без звука,
7.4 МБ, подписи вшиты (DejaVu Sans), на кадрах метка «Simulation». Сборка: `make video PHASE=1`
(если сырьё есть в `media/raw/phase-1/`, только склейка; пересъёмка: `scripts/video/phase1.sh --rerecord`,
около 2 минут). Всё идёт в Docker.

Съёмка в два прохода одного сценария движения (`scripts/video/phase1_drive.py`, время берётся
из `/clock`, поэтому скорость не зависит от ноутбука; у роботов разные программы:
ground_1 прямо и замедляется у нагретого тела, ground_2 едет и крутится на месте, ground_3
змейка):

1. Обзорная камера Gazebo (мир `src/rs_bringup/worlds/phase1_video.sdf`, камера 1080x810 с тегом
   `save`, headless): кадры идут по времени симуляции.
2. Окно RViz (конфиг `src/rs_bringup/rviz/video.rviz`: вид ближе, без осей TF): захват экрана
   `x11grab` внутри контейнера. Нужен экран (`DISPLAY`): окно RViz появится на рабочем столе
   примерно на минуту, его не нужно закрывать и перекрывать другими окнами. Положение 3D-вида
   и панелей картинок скрипт берёт из `xwininfo`, RGB и тепловизор различает по цвету.

| Время | Шот |
| --- | --- |
| 0-12 с | Gazebo, 3 робота едут; подписи «3 ground robots», «Independent control, separate namespaces», «Each robot: 3D LiDAR / RGB / thermal / IMU» |
| 12-22 с | RViz: облака точек всех трёх роботов, под ними RGB и тепловизор ground_1 |
| 22-25.6 с | Логотип Bekai Vision, «RescueSwarm / Phase 1» |

Ограничение: на видео картинки RGB и тепловизора только у ground_1 (так настроен RViz-конфиг);
IMU показан только в подписи, в RViz не отображается.
