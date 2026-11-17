# CLAUDE.md

Руководство для Claude Code в этом репозитории. План и контекст: [docs/PLAN.md](docs/PLAN.md).

## Главное правило

**Собираем и запускаем только в Docker.** На хост ничего не ставим, ROS Humble,
`.bashrc` и `.zshrc` не трогаем. Не запускайте `colcon`, `gz`, `ros2` на хосте.
Каталог проекта содержит пробел (`bekai vision`): все пути в кавычках.

## Команды (из корня репозитория)

| Команда | Действие |
| --- | --- |
| `make image [NOCACHE=1]` | Собрать образ `rescueswarm-dev:jazzy` |
| `make shell` | Терминал в контейнере (`/workspace`) |
| `make deps` | `vcs import` из `deps.repos` в `external/` |
| `make build` / `make test` | `colcon build --symlink-install` / `colcon test` (только `src/`) |
| `make sim-robots [N=3]` | Фаза 1: Gazebo с окном + RViz, роботы `ground_1..N` |
| `make sim-robots-headless [N=3]` | то же без окон |
| `make teleop [ROBOT=ground_1]` | клавиатурное управление выбранным роботом (нужен TTY) |
| `make check-phase1` | скриптовая проверка Фазы 1, результаты в `results/phase-1/` |
| `make explore [SEED=1] [N=3]` | Фаза 2: мир по `seed`, роботы сами исследуют его (Gazebo с окном + RViz) |
| `make explore-headless [SEED=1] [N=3] [MAXTIME=900]` | то же без окон, результаты в `results/phase-2/run_seed_<SEED>/` |
| `make check-phase2 [CHECK_SEEDS="1 2 3 7"] [MAX_SIM_S=900]` | скриптовая проверка Фазы 2, результаты в `results/phase-2/check/` |
| `make search [SEED=1] [N=3] [MAXTIME=900] [RGB=1\|0]` | Фаза 3: поиск людей по тепловизору и YOLO по RGB (RGB=0: только тепловизор; исследование + осмотр кандидатов; Gazebo с окном + RViz) |
| `make search-headless [SEED=1] [N=3] [MAXTIME=900] [RGB=1\|0]` | то же без окон, результаты в `results/phase-3/run_seed_<SEED>/` (`people.json`, `detections.jsonl`, `rgb_detections.jsonl`); RGB=0: в `results/phase-3/run_seed_<SEED>_thermal_only/` |
| `make check-phase3 [CHECK_SEEDS="1 2 3 7"] [MAX_SIM_S=900] [RGB=1\|0]` | скриптовая проверка Фазы 3 (`scripts/phase3_eval.py`), результаты в `results/phase-3/check/`; RGB=0 (только тепловизор): `results/phase-3/check_thermal_only/` |
| `make explore-headless\|search-headless LOC=slam` | Фаза 4: поза от `mrg_slam` + колёса + гироскоп (`slam_adapter`) вместо истинной (`LOC=gt` по умолчанию); `OUT=/workspace/путь` другой каталог результатов, `EXTRA='perception:=false'` доп. аргументы launch |
| `make slam-smoke [SLAMDUR=120]` | Фаза 4: `mrg_slam` на одном роботе в открытом мире, результаты в `results/phase-4/smoke/` |
| `make check-phase4 [CHECK_SEEDS="1 2 3 7"] [MAX_SIM_S=900] [RGB=1\|0]` | скриптовая проверка Фазы 4: поиск людей на каждом seed с истинной позой (gt) и со SLAM, ATE, качество карты, сводка в `results/phase-4/check/summary.md` |
| `make explore\|search[-headless] DRONES=2 [DBACK=light\|px4]` | Фаза 5: те же прогоны с дронами `drone_1..M` (смешанный рой, одна карта: `/map`, слои `/map_ground` и `/map_air`); `light` - простая модель полёта, `px4` - PX4 SITL v1.17 (offboard, поза от EKF) |
| `make check-phase5 [CHECK_SEEDS="1 2 3 7"] [MAX_SIM_S=900] [DRONES=2] [DBACK=light\|px4] [MODES="ground mixed"]` | скриптовая проверка Фазы 5: на каждом seed только наземные роботы и смешанный рой, сводка в `results/phase-5/check/summary.md` (`DBACK=px4`: `results/phase-5/check_px4/`) |
| `make px4-smoke` | Фаза 5: два дрона PX4 SITL в мире PX4 летят квадрат в offboard из ROS 2, результаты в `results/phase-5/px4_smoke/` (нужен `install_px4` с `px4_msgs`) |
| `make video PHASE=5 [SEED=1] [RERECORD=1] [VIDEO_DBACK=px4\|light]` | Видео Фазы 5 в `media/phase-5.mp4` (`scripts/video/phase5.sh`; сырьё в `media/raw/phase-5/`; запись прогона смешанного роя с PX4, ~5-10 мин) |
| `make explore\|search[-headless] COMMS=ideal\|good\|medium\|poor [OUTAGES='ground_1:60:200']` | Фаза 6: обмен между роботами только через `comms_sim` (дальность, стены, полоса, потери, задержка; `OUTAGES` - полные обрывы узлов по сим. времени); у каждого робота своя карта и своё слияние отметок, база `base` у входа = то, что видит спасатель (`/map`, `/people/*`) |
| `make check-phase6 [CHECK_SEEDS="1 2 3 7"] [LEVELS="ideal good medium poor"] [MAX_SIM_S=900] [OUTAGE=1]` | скриптовая проверка Фазы 6: смешанный рой (2 лёгких дрона) на каждом уровне связи + тест обрыва связи ground_1 (только наземные), сводка в `results/phase-6/check/summary.md` |
| `make video PHASE=6 [SEED=1] [RERECORD=1]` | Видео Фазы 6 в `media/phase-6.mp4` (`scripts/video/phase6.sh`: обрыв связи ground_1 60-200 с, карта базы против карты робота) |
| `make check-phase7a [CHECK_SEEDS="1 2 3 7"] [MAX_SIM_S=900] [PARTS="light px4 n5"]` | Фаза 7a: полная связка `LOC=slam DRONES=2 COMMS=medium` (light и px4) на каждом seed и N=5 (с дронами и без), сводка в `results/phase-7a/check/summary.md` |
| `make sim [WORLD=file.sdf]` | Gazebo с окном, по умолчанию `shapes.sdf` |
| `make sim-headless` | Gazebo без окна на `configs/worlds/phase0_bench.sdf` |
| `make video PHASE=0` | Видео Фазы 0 в `media/phase-0.mp4` (`scripts/video/phase0.sh`; кадры в `media/raw/phase-0/`, `--rerecord` снимает заново) |
| `make video PHASE=1` | Видео Фазы 1 в `media/phase-1.mp4` (`scripts/video/phase1.sh`; сырьё в `media/raw/phase-1/`; снимается окно RViz на экране, пересъёмка: `scripts/video/phase1.sh --rerecord`) |
| `make video PHASE=2 [SEED=1] [RERECORD=1]` | Видео Фазы 2 в `media/phase-2.mp4` (`scripts/video/phase2.sh`; сырьё в `media/raw/phase-2/`: запись headless-прогона + кадры рисуются из неё, экран не нужен; из кэша собирается за ~1 мин, `RERECORD=1` пишет прогон заново, ~5 мин) |
| `make video PHASE=3 [SEED=1] [RERECORD=1]` | Видео Фазы 3 в `media/phase-3.mp4` (`scripts/video/phase3.sh`; сырьё в `media/raw/phase-3/`: запись headless-прогона поиска (карта, отметки, тепловые кадры) + кадры рисуются из неё, экран не нужен; из кэша ~1-2 мин, `RERECORD=1` пишет прогон заново, ~5-8 мин) |
| `make video PHASE=4 [SEED=1] [RERECORD=1]` | Видео Фазы 4 в `media/phase-4.mp4` (gt против slam; `RERECORD=1` пишет заново) |
| `make eval`, `make video PHASE=<n>` (n>6) | Заглушки (код 1), реализация позже |

Фаза 7a: один прогон с оценками `scripts/p7a_run.sh <каталог> <seed> [аргументы launch]` (env `N LOC DRONES DBACK COMMS SETUP`), повтор слияния отметок по записанным детекциям `python3 scripts/replay_fusion.py --root results -j 6` (хост, проверка правил `marks.py` без симуляции). Список роботов задают только launch-файлы (узел без списка падает с ошибкой). Обмен графами `mrg_slam` между роботами по умолчанию выключен (иначе он шёл бы мимо `comms_sim`; `slam_graph_exchange:=true` включает). Патч `patches/mrg_slam-init-guess-sign.patch` не применяется: с ним хуже. Отчёт: [docs/PHASE_7A_REPORT.md](docs/PHASE_7A_REPORT.md).

Фаза 6: пакет `rs_comms` (`comms_sim`, агент связи `onboard` на каждом узле), оценка `scripts/phase6_eval.py <каталог прогона>`, сводка `scripts/phase6_summary.py <каталог> "1 2 3 7" "ideal good medium poor"` (хост, stdlib + numpy). Отчёт: [docs/PHASE_6_REPORT.md](docs/PHASE_6_REPORT.md). С `comms:=false` (по умолчанию) всё как в Фазе 5.

Фаза 5: оценка слоёв карты `scripts/layers_eval.py <каталог прогона>` (RQ3: сдвиг воздушного слоя
относительно наземного), сводка `scripts/phase5_summary.py <каталог> "1 2 3 7"` (хост, stdlib + numpy).
Отчёт: [docs/PHASE_5_REPORT.md](docs/PHASE_5_REPORT.md). Образ `rescueswarm-dev:jazzy` содержит PX4
(`/opt/px4`, этап `px4-build` в Dockerfile) и `MicroXRCEAgent`; `px4_msgs` в `deps.repos`, версия
меняется только вместе с `PX4_VERSION` в Dockerfile. В `docker/compose.yaml` `GZ_IP=127.0.0.1`:
без этого gz-transport объявлял адрес tailscale0, и `/clock` иногда не доходил до узлов.
Параметры PX4 задаёт `src/rs_drone/scripts/px4_instance.sh`: `EKF2_GPS_DELAY=0` обязателен (при умолчании
110 мс поза EKF уходит вперёд на 0.11 с x скорость, и воздушный слой карты «размазывается»), `NAV_DLL_ACT`
ставится через `px4-param` после старта (через env его перезаписывает airframe).

Фаза 4: оценка `scripts/ate_eval.py`, `scripts/map_eval.py`, сводка `scripts/phase4_summary.py <каталог> "1 2 3 7"` (хост, stdlib). Отчёт: [docs/PHASE_4_REPORT.md](docs/PHASE_4_REPORT.md).

Замер Фазы 2: `scripts/bench_phase2.sh headless|gui [секунды] [N]`. Отчёт: [docs/PHASE_2_REPORT.md](docs/PHASE_2_REPORT.md).

Замер Фазы 1: `scripts/bench_phase1.sh headless|gui [секунды] [N]`. Отчёт: [docs/PHASE_1_REPORT.md](docs/PHASE_1_REPORT.md).

Замер Фазы 0: `scripts/bench_phase0.sh headless|gui [секунды]`. Результаты лежат в
`results/` (не в git). Отчёт: [docs/PHASE_0_REPORT.md](docs/PHASE_0_REPORT.md).

## Правила

- Каждая фаза ведётся в отдельной сессии. В конце фазы, после проверки, делаем
  коммит в `main`. В остальное время не коммитить без запроса.
- Документы на русском, простым языком.
- `docs/PLAN.md` ведёт руководитель проекта.
