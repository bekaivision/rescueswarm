# Единая точка входа. Всё запускается в Docker; на хосте ничего не ставится.
# Пути берём только из $(CURDIR) и всегда в кавычках: в пути есть пробел.

SHELL := /bin/bash
export HOST_UID := $(shell id -u)
export HOST_GID := $(shell id -g)
export XAUTHORITY ?= $(HOME)/.Xauthority

COMPOSE   := docker compose -p rescueswarm -f "$(CURDIR)/docker/compose.yaml"
# RUNFLAGS=-T отключает TTY (для CI и скриптов)
RUN       := $(COMPOSE) run --rm $(RUNFLAGS)
WORLD     ?= shapes.sdf
HWORLD    ?= /workspace/configs/worlds/phase0_bench.sdf
NOCACHE   ?= 0
PHASE     ?=
N         ?= 3
ROBOT     ?= ground_1
SEED      ?= 1
MAXTIME   ?= 900
CHECK_SEEDS ?= 1 2 3 7
MAX_SIM_S ?= 900
# RGB=0 отключает YOLO по RGB (только тепловизор); результаты тогда в каталогах с суффиксом _thermal_only
RGB       ?= 1
RGBARG    := rgb:=$(if $(filter 0,$(RGB)),false,true)
RGBSUF    := $(if $(filter 0,$(RGB)),_thermal_only,)
# LOC=gt|slam: gt - истинная поза (по умолчанию), slam - колёсная одометрия + поправка mrg_slam (slam_adapter)
LOC       ?= gt
# OUT=путь в контейнере: другой out_dir для explore/search; EXTRA='perception:=false ...': дополнительные аргументы launch
OUT       ?=
EXTRA     ?=
# Фаза 5: DRONES=M дронов drone_1..M (0 = как в Фазе 4), DBACK=light|px4 бэкенд полёта
DRONES    ?= 0
DBACK     ?= light
DRONEARG  := drones:=$(DRONES) drone_backend:=$(DBACK)
# Фаза 6: COMMS=ideal|good|medium|poor включает радиосвязь через comms_sim (comms:=true link:=...),
# OUTAGES='ground_1:60:180;drone_1:100:200' - обрывы связи узлов (узел:начало:конец, сим. секунды)
COMMS     ?=
OUTAGES   ?=
COMMSARG  := $(if $(COMMS),comms:=true link:=$(COMMS) $(if $(OUTAGES),outages:='"'"'$(OUTAGES)'"'"',),)
# check-phase6: уровни связи, тест обрыва (OUTAGE=0 пропускает его)
LEVELS    ?= ideal good medium poor
OUTAGE    ?= 1
# видео Фазы 5 снимается с PX4 (VIDEO_DBACK=light - с лёгкими дронами)
VIDEO_DBACK ?= px4
# все ROS-команды запускаются после source install/setup.bash
WS        := source /workspace/install/setup.bash

.PHONY: help image shell deps build test sim sim-headless sim-robots sim-robots-headless explore explore-headless search search-headless teleop check-phase1 check-phase2 check-phase3 check-phase4 check-phase5 check-phase6 px4-smoke eval video slam-smoke

help: ## Показать команды
	@echo "make image [NOCACHE=1]      собрать образ rescueswarm-dev:jazzy"
	@echo "make shell                  терминал в контейнере"
	@echo "make deps                   vcs import deps.repos в external/"
	@echo "make build                  colcon build (src/ и external/, Release)"
	@echo "make test                   colcon test (только наши пакеты из src/)"
	@echo "make slam-smoke [SLAMDUR=120] [SLAM_DOMAIN=] [SLAMINSTALL=install]  Фаза 4: проверка mrg_slam на одном роботе, результаты в results/phase-4/smoke/"
	@echo "make sim [WORLD=file.sdf]   Gazebo с окном (по умолчанию shapes.sdf)"
	@echo "make sim-headless [HWORLD=] gz sim -s --headless-rendering -r (по умолчанию phase0_bench.sdf)"
	@echo "make sim-robots [N=3]      Фаза 1: Gazebo с окном + RViz, N роботов ground_1..N"
	@echo "make sim-robots-headless [N=3]  то же без окон (gz -s --headless-rendering)"
	@echo "  LOC=slam для explore*/search*: позиция от mrg_slam + колёса (slam_adapter) вместо истинной; OUT=/workspace/путь - другой out_dir; EXTRA='perception:=false' - доп. аргументы launch"
	@echo "make explore [SEED=1] [N=3]  Фаза 2: автономное исследование (Gazebo с окном + RViz)"
	@echo "make explore-headless [SEED=1] [N=3] [MAXTIME=900]  то же без окон, результаты в results/phase-2/run_seed_<SEED>/"
	@echo "make search [SEED=1] [N=3] [MAXTIME=900] [RGB=1]  Фаза 3: поиск людей (исследование + осмотр кандидатов, Gazebo с окном + RViz)"
	@echo "make search-headless [SEED=1] [N=3] [MAXTIME=900] [RGB=1]  то же без окон, результаты в results/phase-3/run_seed_<SEED>/ (RGB=0: без YOLO, в run_seed_<SEED>_thermal_only/)"
	@echo "make teleop [ROBOT=ground_1]    клавиатурное управление роботом (нужен TTY, второй терминал)"
	@echo "make check-phase1           скриптовая проверка Фазы 1 (топики, частоты, движение, кадры)"
	@echo "make check-phase2 [CHECK_SEEDS=\"1 2 3 7\"] [N=3] [MAX_SIM_S=900]  проверка Фазы 2: исследование нескольких seed без человека, результаты в results/phase-2/check/"
	@echo "make check-phase4 [CHECK_SEEDS=\"1 2 3 7\"] [N=3] [MAX_SIM_S=900] [RGB=1]  проверка Фазы 4: поиск людей на каждом seed с истинной позой (gt) и с mrg_slam (slam), ATE/карта/люди, сводка в results/phase-4/check/summary.md"
	@echo "  DRONES=2 для explore*/search*: Фаза 5, дроны drone_1..2 (DBACK=light|px4, по умолчанию light)"
	@echo "make check-phase5 [CHECK_SEEDS=\"1 2 3 7\"] [N=3] [MAX_SIM_S=900] [DRONES=2]  проверка Фазы 5: на каждом seed только наземные роботы и смешанный рой, сводка в results/phase-5/check/summary.md"
	@echo "  COMMS=ideal|good|medium|poor для explore*/search*: Фаза 6, радиосвязь через comms_sim (OUTAGES='ground_1:60:180;drone_1:100:200' - обрывы связи узлов)"
	@echo "make check-phase6 [CHECK_SEEDS=\"1 2 3 7\"] [N=3] [MAX_SIM_S=900] [DRONES=2] [LEVELS=\"ideal good medium poor\"] [OUTAGE=1]  проверка Фазы 6: поиск людей смешанным роем при разных уровнях связи и тест обрыва, сводка в results/phase-6/check/summary.md"
	@echo "make px4-smoke              Фаза 5: два дрона PX4 SITL летят в offboard из ROS 2, результаты в results/phase-5/px4_smoke/"
	@echo "make eval                   заглушка (появится вместе с rs_eval), завершается с кодом 1"
	@echo "make check-phase3 [CHECK_SEEDS=\"1 2 3 7\"] [N=3] [MAX_SIM_S=900] [RGB=1]  проверка Фазы 3: поиск людей на нескольких seed, результаты в results/phase-3/check/ (RGB=0: только тепловизор, results/phase-3/check_thermal_only/)"
	@echo "make video PHASE=<0..6>  собрать media/phase-N.mp4 (Docker; для фазы 1 нужен экран, для фаз 2 и 3 нет; пересъёмка: RERECORD=1); другие фазы: код 1"

image:
	$(COMPOSE) build $(if $(filter 1,$(NOCACHE)),--no-cache,) dev

shell:
	$(RUN) dev bash

deps:
	$(RUN) dev bash -c 'mkdir -p external && vcs import external < deps.repos'

# external/ (make deps) собирается вместе с src/; Release нужен: в Debug scan matching mrg_slam в разы медленнее.
# Полная сборка external ~4 мин на 16 потоках, ограничивать параллельность не пришлось.
# Для слабой машины: make build BUILDARGS="--parallel-workers 2" (и MAKEFLAGS=-j2 внутри контейнера).
build:
	$(RUN) dev bash -c 'colcon build --symlink-install --base-paths src external --cmake-args -DCMAKE_BUILD_TYPE=Release $(BUILDARGS)'

# тестируем только наши пакеты: base-paths src, поэтому внешние пакеты из external/ не затрагиваются
test:
	$(RUN) dev bash -c 'colcon test --base-paths src && colcon test-result --verbose'

sim:
	$(RUN) --name rescueswarm-sim dev gz sim -r "$(WORLD)"

sim-headless:
	$(RUN) --name rescueswarm-sim-headless dev gz sim -s --headless-rendering -r "$(HWORLD)"

sim-robots:
	$(RUN) --name rescueswarm-robots dev bash -c '$(WS) && ros2 launch rs_bringup robots.launch.py n:=$(N) headless:=false rviz:=true'

sim-robots-headless:
	$(RUN) --name rescueswarm-robots-headless dev bash -c '$(WS) && ros2 launch rs_bringup robots.launch.py n:=$(N) headless:=true rviz:=false'

explore:
	$(RUN) --name rescueswarm-explore dev bash -c '$(WS) && ros2 launch rs_bringup explore.launch.py seed:=$(SEED) n:=$(N) headless:=false rviz:=true max_time:=$(MAXTIME) localization:=$(LOC) $(if $(OUT),out_dir:=$(OUT),) $(DRONEARG) $(COMMSARG) $(EXTRA)'

explore-headless:
	$(RUN) --name rescueswarm-explore-headless dev bash -c '$(WS) && ros2 launch rs_bringup explore.launch.py seed:=$(SEED) n:=$(N) headless:=true rviz:=false max_time:=$(MAXTIME) localization:=$(LOC) $(if $(OUT),out_dir:=$(OUT),) $(DRONEARG) $(COMMSARG) $(EXTRA)'

search:
	$(RUN) --name rescueswarm-search dev bash -c '$(WS) && ros2 launch rs_bringup explore.launch.py seed:=$(SEED) n:=$(N) headless:=false rviz:=true max_time:=$(MAXTIME) localization:=$(LOC) $(RGBARG) out_dir:=$(if $(OUT),$(OUT),/workspace/results/phase-3/run_seed_$(SEED)$(RGBSUF)) $(DRONEARG) $(COMMSARG) $(EXTRA)'

search-headless:
	$(RUN) --name rescueswarm-search-headless dev bash -c '$(WS) && ros2 launch rs_bringup explore.launch.py seed:=$(SEED) n:=$(N) headless:=true rviz:=false max_time:=$(MAXTIME) localization:=$(LOC) $(RGBARG) out_dir:=$(if $(OUT),$(OUT),/workspace/results/phase-3/run_seed_$(SEED)$(RGBSUF)) $(DRONEARG) $(COMMSARG) $(EXTRA)'

teleop:
	$(RUN) dev bash -c '$(WS) && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/$(ROBOT)/cmd_vel'

check-phase1:
	"$(CURDIR)/scripts/check_phase1.sh"

check-phase2:
	SEEDS="$(CHECK_SEEDS)" N="$(N)" MAX_SIM_S="$(MAX_SIM_S)" "$(CURDIR)/scripts/check_phase2.sh"

check-phase3:
	SEEDS="$(CHECK_SEEDS)" N="$(N)" MAX_SIM_S="$(MAX_SIM_S)" RGB="$(RGB)" "$(CURDIR)/scripts/check_phase3.sh"

check-phase4:
	SEEDS="$(CHECK_SEEDS)" N="$(N)" MAX_SIM_S="$(MAX_SIM_S)" RGB="$(RGB)" "$(CURDIR)/scripts/check_phase4.sh"

check-phase5:
	SEEDS="$(CHECK_SEEDS)" N="$(N)" MAX_SIM_S="$(MAX_SIM_S)" DRONES="$(if $(filter 0,$(DRONES)),2,$(DRONES))" DBACK="$(DBACK)" "$(CURDIR)/scripts/check_phase5.sh"

check-phase6:
	SEEDS="$(CHECK_SEEDS)" N="$(N)" MAX_SIM_S="$(MAX_SIM_S)" DRONES="$(if $(filter 0,$(DRONES)),2,$(DRONES))" LEVELS="$(LEVELS)" OUTAGE="$(OUTAGE)" "$(CURDIR)/scripts/check_phase6.sh"

# Фаза 5: PX4 SITL (два дрона, offboard) без нашего мира; нужен install_px4 (px4_msgs, см. deps.repos)
px4-smoke:
	"$(CURDIR)/scripts/px4_smoke.sh"

eval:
	@echo "make eval: пока не реализовано (серии экспериментов появятся в Фазе 7 вместе с rs_eval)." >&2
	@exit 1

video:
	@test -n "$(PHASE)" || { echo "Укажите фазу: make video PHASE=<n>" >&2; exit 2; }
	@if [ "$(PHASE)" = 0 ]; then "$(CURDIR)/scripts/video/phase0.sh"; \
	elif [ "$(PHASE)" = 1 ]; then "$(CURDIR)/scripts/video/phase1.sh"; \
	elif [ "$(PHASE)" = 2 ]; then SEED="$(SEED)" "$(CURDIR)/scripts/video/phase2.sh" $(if $(filter 1,$(RERECORD)),--rerecord,); \
	elif [ "$(PHASE)" = 3 ]; then SEED="$(SEED)" "$(CURDIR)/scripts/video/phase3.sh" $(if $(filter 1,$(RERECORD)),--rerecord,); \
	elif [ "$(PHASE)" = 4 ]; then SEED="$(SEED)" "$(CURDIR)/scripts/video/phase4.sh" $(if $(filter 1,$(RERECORD)),--rerecord,); \
	elif [ "$(PHASE)" = 5 ]; then SEED="$(SEED)" DBACK="$(VIDEO_DBACK)" "$(CURDIR)/scripts/video/phase5.sh" $(if $(filter 1,$(RERECORD)),--rerecord,); \
		elif [ "$(PHASE)" = 6 ]; then SEED="$(SEED)" "$(CURDIR)/scripts/video/phase6.sh" $(if $(filter 1,$(RERECORD)),--rerecord,); \
	else echo "make video PHASE=$(PHASE): пока не реализовано (media/phase-$(PHASE).mp4)." >&2; exit 1; fi

# Фаза 4: mrg_slam на одном роботе (ground_1, Gazebo без окон, истинная поза только для сравнения).
# Нужны make deps и make build. Свой GZ_PARTITION, чтобы не пересекаться с другими симуляциями;
# SLAM_DOMAIN=43 задаёт другой ROS_DOMAIN_ID, SLAMINSTALL=install_slam берёт другой каталог установки.
SLAMDUR     ?= 120
SLAM_DOMAIN ?=
SLAMINSTALL ?= install
slam-smoke:
	mkdir -p "$(CURDIR)/results/phase-4/smoke"
	$(RUN) --name rescueswarm-slam-smoke dev bash -c 'export GZ_PARTITION=slam_smoke; $(if $(SLAM_DOMAIN),export ROS_DOMAIN_ID=$(SLAM_DOMAIN);) source /workspace/$(SLAMINSTALL)/setup.bash && rm -f /workspace/results/phase-4/smoke/summary.json && ros2 launch rs_bringup slam_smoke.launch.py duration:=$(SLAMDUR) out_dir:=/workspace/results/phase-4/smoke 2>&1 | tee /workspace/results/phase-4/smoke/launch.log | grep --line-buffered -E "slam_smoke_runner|ERROR|Traceback"; test -f /workspace/results/phase-4/smoke/summary.json && cat /workspace/results/phase-4/smoke/summary.json'
