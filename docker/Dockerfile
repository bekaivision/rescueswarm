# Базовый образ закреплён по digest индекса (тег: ros:jazzy-ros-base, Ubuntu 24.04).
FROM ros:jazzy-ros-base@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617 AS aptbase
ARG HOST_UID=1000
ARG HOST_GID=1000
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-dev-tools \
      ros-jazzy-ros-gz \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
      ros-jazzy-rviz2 ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-teleop-twist-keyboard \
      python3-vcstool \
      libglvnd0 libegl1 libgl1 libglx0 libgles2 libgl1-mesa-dri mesa-utils \
      x11-utils x11-apps \
      ffmpeg fonts-dejavu-core procps git curl nano sudo \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------------------------------
# Фаза 5: PX4 SITL и Micro XRCE-DDS Agent собираются в отдельном этапе px4-build.
# Он растёт из того же слоя apt (aptbase), что и рабочий образ: те же ros-jazzy-gz-*-vendor, то же
# закреплённое основание, но pip-пакеты PX4 и его сборочные инструменты в рабочий образ не попадают.
# Версию PX4 и версию px4_msgs в deps.repos менять только вместе.
FROM aptbase AS px4-build
ARG PX4_VERSION=v1.17.0
ARG PX4_COMMIT=d6f12ad1c4f70ad3230afd7d86e971421e02fef4
ARG XRCE_VERSION=v2.4.3
ARG XRCE_COMMIT=73622810d984349b80bbac0ef55fc0b694d62222
# Не больше 8 потоков компиляции: на машине параллельно идут симуляции.
ARG BUILD_JOBS=8
SHELL ["/bin/bash", "-c"]
# Список пакетов из Tools/setup/ubuntu.sh v1.17.0 (без NuttX, без gz-harmonic из репозитория OSRF).
# Gazebo Harmonic здесь только vendor-пакеты ROS из aptbase: другой Gazebo в образ не попадает.
RUN apt-get update && apt-get install -y --no-install-recommends \
      astyle ccache cmake cppcheck file g++ gcc gdb lcov libssl-dev libxml2-dev libxml2-utils \
      make ninja-build python3-dev python3-pip python3-setuptools python3-wheel rsync unzip zip \
      bc cppzmq-dev libunwind-dev libeigen3-dev libopencv-dev pkg-config protobuf-compiler \
      gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
      gstreamer1.0-plugins-ugly gstreamer1.0-libav libgstreamer-plugins-base1.0-dev \
      libimage-exiftool-perl ca-certificates \
    && rm -rf /var/lib/apt/lists/*
# Исходники PX4 по тегу, с проверкой хэша коммита. Путь /opt/px4 зашит в бинарник: не менять.
RUN git clone --depth 1 --branch ${PX4_VERSION} --recurse-submodules --shallow-submodules -j${BUILD_JOBS} \
      https://github.com/PX4/PX4-Autopilot.git /opt/px4 \
 && test "$(git -C /opt/px4 rev-parse HEAD)" = "${PX4_COMMIT}"
# --ignore-installed: numpy в aptbase стоит из deb, pip не может его удалить. Этап сборочный,
# в рабочий образ эти пакеты не попадают.
RUN pip install --break-system-packages --no-cache-dir --ignore-installed -r /opt/px4/Tools/setup/requirements.txt
# Сборка px4_sitl_default против Gazebo из vendor-пакетов ROS (source даёт CMAKE_PREFIX_PATH).
RUN source /opt/ros/jazzy/setup.bash \
 && cd /opt/px4 \
 && GZ_DISTRO=harmonic make px4_sitl_default -j${BUILD_JOBS}
# Агент XRCE-DDS (v2.x: клиент PX4 несовместим с агентом v3). Ставим в свой префикс, свои Fast DDS внутри.
RUN git clone --depth 1 --branch ${XRCE_VERSION} https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce \
 && test "$(git -C /tmp/xrce rev-parse HEAD)" = "${XRCE_COMMIT}" \
 && cmake -S /tmp/xrce -B /tmp/xrce/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/microxrce_agent \
 && cmake --build /tmp/xrce/build -j${BUILD_JOBS} \
 && cmake --install /tmp/xrce/build \
 && rm -rf /tmp/xrce
# Для запуска нужны только build/px4_sitl_default (bin, etc, плагины gz), Tools/simulation (модели и миры)
# и server.config из src. Остальное (git, исходники, объектные файлы) убираем: ~2.3 ГБ -> см. отчёт.
RUN cd /opt/px4 \
 && cp src/modules/simulation/gz_bridge/server.config /tmp/server.config \
 && rm -rf .git src platforms boards \
 && mkdir -p src/modules/simulation/gz_bridge && mv /tmp/server.config src/modules/simulation/gz_bridge/ \
 && find Tools -mindepth 1 -maxdepth 1 ! -name simulation -exec rm -rf {} + \
 && find build -name '*.o' -delete \
 && find build -type d -name CMakeFiles -prune -exec rm -rf {} +
# ---------------------------------------------------------------------------------------------------
FROM aptbase AS base
ARG HOST_UID=1000
ARG HOST_GID=1000
# Модель Rescue Randy (OpenRobotics, Gazebo Fuel, лицензия CC-BY 4.0), версия 3.
RUN apt-get update && apt-get install -y --no-install-recommends unzip python3-pip \
    && rm -rf /var/lib/apt/lists/* \
 && curl -fsSL -o /tmp/randy.zip "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rescue%20Randy/3/Rescue%20Randy.zip" \
 && echo "abf964e3c417318156ad8fe6d34bc815f6be0b0843cea4ce73fa76883d40f4d2  /tmp/randy.zip" | sha256sum -c - \
 && mkdir -p /opt/rs_models/rescue_randy && unzip -q /tmp/randy.zip -d /opt/rs_models/rescue_randy \
 && rm /tmp/randy.zip
ENV GZ_SIM_RESOURCE_PATH=/opt/rs_models

# Веса YOLO11 (ultralytics assets v8.3.0), проверка sha256, без загрузок во время работы.
RUN mkdir -p /opt/yolo && cd /opt/yolo \
 && curl -fsSL -O https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt \
 && curl -fsSL -O https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11s.pt \
 && printf '%s\n' \
      "0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1  yolo11n.pt" \
      "85a76fe86dd8afe384648546b56a7a78580c7cb7b404fc595f97969322d502d5  yolo11s.pt" | sha256sum -c -
# Офлайн-режим ultralytics: без скачивания весов и без отправки статистики.
ENV YOLO_OFFLINE=1 YOLO_AUTOINSTALL=False

# Python-пакеты. numpy<2 обязателен: cv_bridge в ROS собран под numpy 1.x.
# opencv ставим headless (без Qt/GUI), чтобы не ломать RViz/Gazebo.
# setuptools<80: colcon-core требует это ограничение.
RUN printf '%s\n' "numpy<2" "opencv-python-headless==4.11.0.86" "setuptools<80" > /tmp/constraints.txt \
 && pip install --break-system-packages --no-cache-dir -c /tmp/constraints.txt \
      --extra-index-url https://download.pytorch.org/whl/cu126 \
      torch==2.14.1+cu126 torchvision==0.29.1+cu126 ultralytics==8.4.208 opencv-python-headless==4.11.0.86 \
 && (pip uninstall -y --break-system-packages opencv-python || true) \
 && pip install --break-system-packages --no-cache-dir --force-reinstall --no-deps \
      -c /tmp/constraints.txt opencv-python-headless \
 && rm /tmp/constraints.txt
# Фаза 4: зависимости mrg_slam (мульти-робот графовый SLAM, исходники в external/ через `make deps`).
# Список взят из docker/jazzy_local/Dockerfile репозитория mrg_slam. g2o приходит как ros-jazzy-libg2o.
# rmw_zenoh ставим про запас (автор советует его для нескольких роботов), RMW не переключаем.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-nmea-msgs \
      ros-jazzy-tf2-sensor-msgs ros-jazzy-tf2-geometry-msgs \
      ros-jazzy-geodesy ros-jazzy-libg2o ros-jazzy-interactive-markers \
      libceres-dev libgeographiclib-dev libsuitesparse-dev libomp-dev \
      python3-scipy python3-progressbar \
      ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*
# Фаза 5: PX4 SITL и агент XRCE-DDS из этапа px4-build, по тем же абсолютным путям (пути зашиты в px4).
# Библиотеки Gazebo для px4 и его плагинов берутся из vendor-пакетов ROS: запускать после source /opt/ros/jazzy.
# Рабочие каталоги px4 (параметры, логи) создаются на каждый запуск вне /opt/px4.
RUN apt-get update && apt-get install -y --no-install-recommends bc \
    && rm -rf /var/lib/apt/lists/*
COPY --from=px4-build /opt/microxrce_agent /opt/microxrce_agent
COPY --from=px4-build /opt/px4 /opt/px4
# Агент несёт свои Fast DDS / Fast CDR: свои библиотеки только для него, не через общий LD_LIBRARY_PATH
# (иначе после source ROS подхватятся библиотеки Fast DDS из ROS другой версии).
RUN printf '#!/bin/sh\nLD_LIBRARY_PATH=/opt/microxrce_agent/lib exec /opt/microxrce_agent/bin/MicroXRCEAgent "$@"\n' \
      > /usr/local/bin/MicroXRCEAgent && chmod +x /usr/local/bin/MicroXRCEAgent
ENV PX4_ROOT=/opt/px4 \
    PX4_GZ_MODELS=/opt/px4/Tools/simulation/gz/models \
    PX4_GZ_WORLDS=/opt/px4/Tools/simulation/gz/worlds \
    PX4_GZ_PLUGINS=/opt/px4/build/px4_sitl_default/src/modules/simulation/gz_plugins
# В ros:jazzy в образе уже есть пользователь ubuntu с UID 1000: заменяем его на dev.
RUN (userdel -r ubuntu 2>/dev/null || true) \
 && groupadd -g ${HOST_GID} dev \
 && useradd -m -u ${HOST_UID} -g dev -s /bin/bash dev \
 && echo 'dev ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/dev \
 && mkdir -p /home/dev/.gz /workspace \
 && chown -R dev:dev /home/dev /workspace \
 && echo 'source /opt/ros/jazzy/setup.bash' >> /home/dev/.bashrc \
 && echo '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' >> /home/dev/.bashrc

FROM base AS dev
USER dev
WORKDIR /workspace
# /ros_entrypoint.sh (из базового образа) подключает /opt/ros/jazzy, в том числе для `make shell`.
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
