# go2-3d:local: RTAB-Map + Foxglove/rosbridge + YOLO11 for the 3D digital-twin
# session. Built by scripts/runtime/wifi_3d_nav.sh --build. Kept separate from
# the upstream go2-lidar3d image so wifi_nav.sh recreating that container never
# kills RTAB-Map before it has saved its database.
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
        python3-pip libgl1 libglib2.0-0 \
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

# Bake the weights in, so the robot network needs no internet at run time,
# and run one inference so a broken install fails the build, not the robot run.
WORKDIR /opt/yolo
RUN YOLO_CONFIG_DIR=/tmp/yolo-build python3 -c 'import numpy as np; from ultralytics import YOLO; m = YOLO("yolo11s.pt"); m.predict(np.zeros((64, 64, 3), np.uint8), verbose=False)[0].plot(); print("yolo11s OK,", len(m.names), "classes")' \
    && test -s /opt/yolo/yolo11s.pt && chmod a+r /opt/yolo/yolo11s.pt && rm -rf /tmp/yolo-build

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    HOME=/tmp/go2-3d-home
WORKDIR /twin
CMD ["bash", "/twin/start_inside.sh"]
