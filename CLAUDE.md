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
| `make sim [WORLD=file.sdf]` | Gazebo с окном, по умолчанию `shapes.sdf` |
| `make sim-headless` | Gazebo без окна на `configs/worlds/phase0_bench.sdf` |
| `make video PHASE=0` | Видео Фазы 0 в `media/phase-0.mp4` (`scripts/video/phase0.sh`; кадры в `media/raw/phase-0/`, `--rerecord` снимает заново) |
| `make video PHASE=1` | Видео Фазы 1 в `media/phase-1.mp4` (`scripts/video/phase1.sh`; сырьё в `media/raw/phase-1/`; снимается окно RViz на экране, пересъёмка: `scripts/video/phase1.sh --rerecord`) |
| `make video PHASE=2 [SEED=1] [RERECORD=1]` | Видео Фазы 2 в `media/phase-2.mp4` (`scripts/video/phase2.sh`; сырьё в `media/raw/phase-2/`: запись headless-прогона + кадры рисуются из неё, экран не нужен; из кэша собирается за ~1 мин, `RERECORD=1` пишет прогон заново, ~5 мин) |
| `make eval`, `make video PHASE=<n>` (n>2) | Заглушки (код 1), реализация позже |

Замер Фазы 2: `scripts/bench_phase2.sh headless|gui [секунды] [N]`. Отчёт: [docs/PHASE_2_REPORT.md](docs/PHASE_2_REPORT.md).

Замер Фазы 1: `scripts/bench_phase1.sh headless|gui [секунды] [N]`. Отчёт: [docs/PHASE_1_REPORT.md](docs/PHASE_1_REPORT.md).

Замер Фазы 0: `scripts/bench_phase0.sh headless|gui [секунды]`. Результаты лежат в
`results/` (не в git). Отчёт: [docs/PHASE_0_REPORT.md](docs/PHASE_0_REPORT.md).

## Правила

- Каждая фаза ведётся в отдельной сессии. В конце фазы, после проверки, делаем
  коммит в `main`. В остальное время не коммитить без запроса.
- Без атрибуции ИИ в коммитах и документах.
- Документы на русском, простым языком.
- `docs/PLAN.md` ведёт руководитель проекта.
