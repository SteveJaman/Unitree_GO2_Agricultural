# go2-realsense-host:local: the D435i driver on the WORKSTATION (amd64), for testing the
# camera -> detector -> arm logic with the camera plugged into this computer instead of the
# Go2 (scripts/runtime/arm_servo_test.sh camera). ROS 2 Humble's realsense2_camera and
# librealsense2 debs, CycloneDDS. The robot's own image is realsense/Dockerfile (Foxy, arm64).
#   docker build -t go2-realsense-host:local realsense/host
FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
# The ROS apt key baked into older images expired on 2025-06-01: re-add it with the renewed one.
RUN rm -f /etc/apt/sources.list.d/ros2*.list /etc/apt/sources.list.d/ros2*.sources \
 && apt-get update -qq && apt-get install -y -qq --no-install-recommends curl ca-certificates \
 && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list \
 && apt-get update -qq \
 && apt-get install -y -qq --no-install-recommends ros-humble-realsense2-camera ros-humble-rmw-cyclonedds-cpp \
      ros-humble-image-transport-plugins usbutils \
 && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CMD ["bash"]
