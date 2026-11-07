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
# все ROS-команды запускаются после source install/setup.bash
WS        := source /workspace/install/setup.bash

.PHONY: help image shell deps build test sim sim-headless sim-robots sim-robots-headless teleop check-phase1 eval video

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
	@echo "make teleop [ROBOT=ground_1]    клавиатурное управление роботом (нужен TTY, второй терминал)"
	@echo "make check-phase1           скриптовая проверка Фазы 1 (топики, частоты, движение, кадры)"
	@echo "make eval                   заглушка (появится вместе с rs_eval), завершается с кодом 1"
	@echo "make video PHASE=<0|1>      собрать media/phase-N.mp4 (Docker, GPU; для фазы 1 нужен экран); другие фазы: код 1"

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

teleop:
	$(RUN) dev bash -c '$(WS) && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/$(ROBOT)/cmd_vel'

check-phase1:
	"$(CURDIR)/scripts/check_phase1.sh"

eval:
	@echo "make eval: пока не реализовано (серии экспериментов появятся в Фазе 7 вместе с rs_eval)." >&2
	@exit 1

video:
	@test -n "$(PHASE)" || { echo "Укажите фазу: make video PHASE=<n>" >&2; exit 2; }
	@if [ "$(PHASE)" = 0 ]; then "$(CURDIR)/scripts/video/phase0.sh"; \
	elif [ "$(PHASE)" = 1 ]; then "$(CURDIR)/scripts/video/phase1.sh"; \
	else echo "make video PHASE=$(PHASE): пока не реализовано (media/phase-$(PHASE).mp4)." >&2; exit 1; fi
