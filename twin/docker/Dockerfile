# go2-3d:local: RTAB-Map + rosbridge (+ foxglove_bridge) + YOLO11 + the web
# dashboard for the 3D digital-twin session. Built by
# scripts/runtime/wifi_3d_nav.sh --build with twin/ as the build context:
#   docker build -t go2-3d:local -f twin/docker/Dockerfile twin
# Kept separate from the upstream go2-lidar3d image so wifi_nav.sh recreating
# that container never kills RTAB-Map before it has saved its database.

# --- Web dashboard (twin/webapp): built here, so the VM needs no Node.js ----
# Build stage only: nothing from it but dist/ reaches the final image.
FROM node:22-bookworm-slim AS webapp
WORKDIR /webapp
COPY webapp/package.json webapp/package-lock.json ./
RUN npm ci --no-audit --no-fund
COPY webapp/ ./
RUN npm run build && test -s dist/index.html

# --- ROS side -------------------------------------------------------------------
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rtabmap-ros \
        ros-humble-foxglove-bridge \
        ros-humble-rosbridge-server \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs \
        ros-humble-visualization-msgs \
        ros-humble-vision-msgs \
        ros-humble-sensor-msgs-py \
        ros-humble-nav2-msgs \
        python3-pip libgl1 libglib2.0-0 libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# YOLO11 on the CPU. The CPU wheel index avoids the multi-GB CUDA build of
# torch; numpy stays < 2 because ROS Humble's Python packages are built
# against numpy 1.x (opencv-python >= 4.12 requires numpy 2).
RUN python3 -m pip install --no-cache-dir \
        --index-url https://download.pytorch.org/whl/cpu \
        torch==2.5.1 torchvision==0.20.1 \
    && python3 -m pip install --no-cache-dir \
        'numpy<2' 'opencv-python<4.12' 'ultralytics==8.3.40' \
    && python3 -c 'import numpy, cv2, torch, ultralytics; print("numpy", numpy.__version__, "cv2", cv2.__version__, "torch", torch.__version__, "ultralytics", ultralytics.__version__)'

# Open3D: Poisson meshing for wifi_3d_nav.sh --export --mesh (scripts/tools/export_tour.py).
RUN python3 -m pip install --no-cache-dir 'numpy<2' 'open3d==0.18.0' \
    && python3 -c 'import open3d, numpy; print("open3d", open3d.__version__, "numpy", numpy.__version__)'

# Bake the weights in, so the robot network needs no internet at run time,
# and run one inference so a broken install fails the build, not the robot run.
WORKDIR /opt/yolo
RUN YOLO_CONFIG_DIR=/tmp/yolo-build python3 -c 'import numpy as np; from ultralytics import YOLO; m = YOLO("yolo11s.pt"); m.predict(np.zeros((64, 64, 3), np.uint8), verbose=False)[0].plot(); print("yolo11s OK,", len(m.names), "classes")' \
    && test -s /opt/yolo/yolo11s.pt && chmod a+r /opt/yolo/yolo11s.pt && rm -rf /tmp/yolo-build

# Served on port 3000 by start_inside.sh (python3 -m http.server).
COPY --from=webapp /webapp/dist /opt/webapp

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    HOME=/tmp/go2-3d-home
WORKDIR /twin
CMD ["bash", "/twin/start_inside.sh"]
