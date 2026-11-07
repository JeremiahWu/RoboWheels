# syntax=docker/dockerfile:1
# =============================================================================
# RoboWheels - RDK X5 runtime image (arm64)
#
# Pins the whole runtime: Ubuntu 22.04 + ROS2 Humble (apt) + TROS
# (mipi_cam for the GS130WI) + uv-managed python env (Hobot.GPIO,
# pyserial, colcon) + the robowheels ROS workspace built with colcon.
#
# Build (on the RDK X5 or any arm64 docker host):
#   docker build -t robowheels:rdkx5 .
#
# Run:
#   docker run -it --rm --privileged --network host \
#     -v /dev:/dev -v /sys:/sys:rw \
#     robowheels:rdkx5 \
#     ros2 launch robowheels bringup.launch.py
# (--privileged + device mounts are required for GPIO/UART/MIPI/IMU/USB)
#
# Mirrors default to TUNA (this board is on a CN network). Override with
#   --build-arg UBUNTU_MIRROR=archive.ubuntu.com \
#   --build-arg ROS_MIRROR=packages.ros.org/ros2/ubuntu \
#   --build-arg UV_INDEX=https://pypi.org/simple
# =============================================================================
FROM ros:humble-ros-base AS base

ARG UBUNTU_MIRROR=mirrors.tuna.tsinghua.edu.cn/ubuntu
ARG UBUNTU_PORTS_MIRROR=mirrors.tuna.tsinghua.edu.cn/ubuntu-ports
ARG UBUNTU_SEC_MIRROR=mirrors.tuna.tsinghua.edu.cn/ubuntu-security
ARG ROS_MIRROR=mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
ENV DEBIAN_FRONTEND=noninteractive

# ---- Ubuntu apt -> mirror (arm64 images use ports.ubuntu.com) ----
RUN sed -i -E 's#https?://ports\.ubuntu\.com/ubuntu-ports#http://'"$UBUNTU_PORTS_MIRROR"'#g' /etc/apt/sources.list \
    && sed -i -E 's#https?://archive\.ubuntu\.com/ubuntu#http://'"$UBUNTU_MIRROR"'#g' /etc/apt/sources.list \
    && sed -i -E 's#https?://security\.ubuntu\.com/ubuntu#http://'"$UBUNTU_SEC_MIRROR"'#g' /etc/apt/sources.list

# ---- ROS2 apt -> mirror (key already shipped by the base image) ----
RUN find /etc/apt/sources.list.d -name 'ros2*.list' -exec \
    sed -i -E 's#https?://packages\.ros\.org/ros2/ubuntu#https://'"$ROS_MIRROR"'#g' {} +

# ---- D-Robotics RDK X5 repo (tros: mipi_cam / hobot_shm) ----
COPY docker/keys/sunrise.gpg /usr/share/keyrings/sunrise.gpg
RUN echo "deb [arch=arm64 signed-by=/usr/share/keyrings/sunrise.gpg] http://archive.d-robotics.cc/ubuntu-rdk-x5 jammy main" \
    > /etc/apt/sources.list.d/sunrise.list

# ---- system packages ----
# python3-opencv / python3-numpy: cv2+numpy for cv_bridge (keep the apt
# build, do NOT pip-install opencv - two cv2 builds break cv_bridge)
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-opencv \
        python3-numpy \
        ros-humble-cv-bridge \
        ros-humble-tf2-ros \
        ros-humble-joy-linux \
        ros-humble-teleop-twist-joy \
        tros-humble-mipi-cam \
        tros-humble-hobot-shm \
    && rm -rf /var/lib/apt/lists/*

# ============================== build stage ==============================
FROM base AS build

ARG PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
ARG UV_INDEX=https://pypi.tuna.tsinghua.edu.cn/simple
ENV UV_DEFAULT_INDEX=$UV_INDEX

# uv (the python package manager for everything apt does not provide)
RUN python3 -m pip install --no-cache-dir -i "$PIP_INDEX_URL" uv

# uv-managed venv. --system-site-packages keeps the ROS python stack
# (rclpy, cv_bridge, cv2, numpy from apt) importable inside the venv.
RUN uv venv --system-site-packages /opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH

WORKDIR /opt/robowheels

# uv deps first (layer cache): pyproject + vendored Hobot.GPIO
COPY pyproject.toml uv.lock ./
COPY third_party/ third_party/
RUN uv sync --frozen

# build the ROS workspace (ament_python via colcon from the venv)
COPY ros2_ws/ ros2_ws/
RUN . /opt/ros/humble/setup.sh \
    && . /opt/tros/humble/setup.sh \
    && cd ros2_ws \
    && colcon build --symlink-install --packages-select robowheels

# ============================== runtime stage =============================
FROM base AS runtime

COPY --from=build /opt/venv /opt/venv
COPY --from=build /opt/robowheels/ros2_ws /opt/robowheels/ros2_ws
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /opt/robowheels
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
