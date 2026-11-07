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
# все ROS-команды запускаются после source install/setup.bash
WS        := source /workspace/install/setup.bash

.PHONY: help image shell deps build test sim sim-headless sim-robots sim-robots-headless explore explore-headless teleop check-phase1 check-phase2 eval video

help: ## Показать команды
	@echo "make image [NOCACHE=1]      собрать образ rescueswarm-dev:jazzy"
	@echo "make shell                  терминал в контейнере"
	@echo "make deps                   vcs import deps.repos в external/"
	@echo "make build                  colcon build"
	@echo "make test                   colcon test"
	@echo "make sim [WORLD=file.sdf]   Gazebo с окном (по умолчанию shapes.sdf)"
	@echo "make sim-headless [HWORLD=] gz sim -s --headless-rendering -r (по умолчанию phase0_bench.sdf)"
	@echo "make sim-robots [N=3]      Фаза 1: Gazebo с окном + RViz, N роботов ground_1..N"
	@echo "make sim-robots-headless [N=3]  то же без окон (gz -s --headless-rendering)"
	@echo "make explore [SEED=1] [N=3]  Фаза 2: автономное исследование (Gazebo с окном + RViz)"
	@echo "make explore-headless [SEED=1] [N=3] [MAXTIME=900]  то же без окон, результаты в results/phase-2/run_seed_<SEED>/"
	@echo "make teleop [ROBOT=ground_1]    клавиатурное управление роботом (нужен TTY, второй терминал)"
	@echo "make check-phase1           скриптовая проверка Фазы 1 (топики, частоты, движение, кадры)"
	@echo "make check-phase2 [CHECK_SEEDS=\"1 2 3 7\"] [N=3] [MAX_SIM_S=900]  проверка Фазы 2: исследование нескольких seed без человека, результаты в results/phase-2/check/"
	@echo "make eval                   заглушка (появится вместе с rs_eval), завершается с кодом 1"
	@echo "make video PHASE=<0|1|2>    собрать media/phase-N.mp4 (Docker; для фазы 1 нужен экран, для фазы 2 нет; ПЕРЕСЪЁМКА фазы 2: scripts/video/phase2.sh --rerecord); другие фазы: код 1"

image:
	$(COMPOSE) build $(if $(filter 1,$(NOCACHE)),--no-cache,) dev

shell:
	$(RUN) dev bash

deps:
	$(RUN) dev bash -c 'mkdir -p external && vcs import external < deps.repos'

build:
	$(RUN) dev bash -c 'colcon build --symlink-install --base-paths src'

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
	$(RUN) --name rescueswarm-explore dev bash -c '$(WS) && ros2 launch rs_bringup explore.launch.py seed:=$(SEED) n:=$(N) headless:=false rviz:=true max_time:=$(MAXTIME)'

explore-headless:
	$(RUN) --name rescueswarm-explore-headless dev bash -c '$(WS) && ros2 launch rs_bringup explore.launch.py seed:=$(SEED) n:=$(N) headless:=true rviz:=false max_time:=$(MAXTIME)'

teleop:
	$(RUN) dev bash -c '$(WS) && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/$(ROBOT)/cmd_vel'

check-phase1:
	"$(CURDIR)/scripts/check_phase1.sh"

check-phase2:
	SEEDS="$(CHECK_SEEDS)" N="$(N)" MAX_SIM_S="$(MAX_SIM_S)" "$(CURDIR)/scripts/check_phase2.sh"

eval:
	@echo "make eval: пока не реализовано (серии экспериментов появятся в Фазе 7 вместе с rs_eval)." >&2
	@exit 1

video:
	@test -n "$(PHASE)" || { echo "Укажите фазу: make video PHASE=<n>" >&2; exit 2; }
	@if [ "$(PHASE)" = 0 ]; then "$(CURDIR)/scripts/video/phase0.sh"; \
	elif [ "$(PHASE)" = 1 ]; then "$(CURDIR)/scripts/video/phase1.sh"; \
	elif [ "$(PHASE)" = 2 ]; then SEED="$(SEED)" "$(CURDIR)/scripts/video/phase2.sh" $(if $(filter 1,$(RERECORD)),--rerecord,); \
	else echo "make video PHASE=$(PHASE): пока не реализовано (media/phase-$(PHASE).mp4)." >&2; exit 1; fi
