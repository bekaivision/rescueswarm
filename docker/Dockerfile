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
