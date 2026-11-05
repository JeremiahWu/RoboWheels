ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ARG UBUNTU_PORTS_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports
RUN sed -i "s|http://ports.ubuntu.com/ubuntu-ports|${UBUNTU_PORTS_MIRROR}|g" /etc/apt/sources.list
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-foxglove-bridge python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY ros2_ws/src/robowheels_description src/robowheels_description
COPY ros2_ws/src/robowheels_sim src/robowheels_sim
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install
COPY docker/sim/entrypoint.sh /sim-entrypoint.sh
ENTRYPOINT ["/bin/bash", "/sim-entrypoint.sh"]
CMD ["ros2", "launch", "robowheels_sim", "sim.launch.py"]
