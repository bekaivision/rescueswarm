# Базовый образ закреплён по digest индекса (тег: ros:jazzy-ros-base, Ubuntu 24.04).
FROM ros:jazzy-ros-base@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617 AS base
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
