# go2-sim:local: Gazebo Harmonic + ROS 2 Humble bridge for the Go2 + D1 simulation.
# Built by gazebo/setup_gazebo.sh. Kept separate from the upstream go2-lidar3d
# image so recreating either never affects the other.
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Gazebo Harmonic from the official OSRF repository (pairs with Humble via
# ros-humble-ros-gzharmonic; see gazebosim.org "ROS 2 Gazebo vendor packages").
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg lsb-release \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-humble-ros-gzharmonic \
        python3-gz-transport13 python3-gz-msgs10 \
        ros-humble-rmw-cyclonedds-cpp \
        python3-numpy python3-pip \
        iproute2 iputils-ping \
        libgl1-mesa-dri mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# The arm emulator uses the same pure-Python DDS bindings as the arm scripts.
RUN python3 -m pip install --no-cache-dir 'cyclonedds==0.10.2'

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    LIBGL_ALWAYS_SOFTWARE=1 \
    HOME=/tmp/simhome
WORKDIR /sim
CMD ["bash", "/sim/sim/start_inside.sh"]
