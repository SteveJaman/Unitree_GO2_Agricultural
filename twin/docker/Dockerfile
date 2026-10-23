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

# Camera obstacles (wifi_3d_nav.sh --camera-obstacles): ONNX Runtime (CPU) and
# Depth Anything V2-Small (Apache-2.0), pinned by revision and checksum.
# --no-deps keeps the system numpy that the ROS Python packages are built on.
ARG DEPTH_MODEL_URL=https://huggingface.co/onnx-community/depth-anything-v2-small/resolve/4472b7362082ad9968fee890ca0f1e5aca36b93d/onnx/model.onnx
ARG DEPTH_MODEL_SHA256=afb6a5c28f3b6bf1618c6e43f02073ef9dfdc70e937502d51603e57b0a1df10c
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip curl ca-certificates \
    && pip3 install --no-cache-dir --no-deps onnxruntime==1.23.2 \
    && mkdir -p /opt/depth \
    && curl -fsSL -o /opt/depth/depth_anything_v2_small.onnx "${DEPTH_MODEL_URL}" \
    && echo "${DEPTH_MODEL_SHA256}  /opt/depth/depth_anything_v2_small.onnx" | sha256sum -c - \
    && python3 -c "import onnxruntime" \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    HOME=/tmp/go2-3d-home
WORKDIR /twin
CMD ["bash", "/twin/start_inside.sh"]
