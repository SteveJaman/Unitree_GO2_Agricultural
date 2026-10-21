# go2-3d:local: RTAB-Map + Foxglove/rosbridge for the 3D digital-twin session.
# Built by scripts/runtime/wifi_3d_nav.sh --build. Kept separate from the
# upstream go2-lidar3d image so wifi_nav.sh recreating that container never
# kills RTAB-Map before it has saved its database.
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rtabmap-ros \
        ros-humble-foxglove-bridge \
        ros-humble-rosbridge-server \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-tf2-geometry-msgs \
        ros-humble-visualization-msgs \
        ros-humble-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    HOME=/tmp/go2-3d-home
WORKDIR /twin
CMD ["bash", "/twin/start_inside.sh"]
