# go2-realsense:foxy: Intel RealSense D435i driver for the Go2 Jetson, in a
# container, so the robot's own ROS 2 Foxy and Unitree packages stay untouched.
#
#   librealsense  v2.51.1  (RSUSB / libusb backend: no kernel patches, no udev rules on the host)
#   realsense-ros 4.51.1   (matched to 2.51.1; later wrapper releases dropped Foxy)
#   RMW           rmw_cyclonedds_cpp (stock Foxy, Cyclone 0.7.x), isolated from Unitree's Cyclone
#
# Build ON THE ROBOT (native arm64, ~25-40 min on the Orin NX the first time):
#   cd ~/go2-realsense && ./go2_realsense.sh build
# or by hand:
#   docker build -t go2-realsense:foxy .
#
# Nothing from the host is mounted except /dev/bus/usb (see go2_realsense.sh).
ARG BASE_IMAGE=ros:foxy-ros-base-focal
FROM ${BASE_IMAGE}

ARG LIBREALSENSE_VERSION=2.51.1
ARG REALSENSE_ROS_VERSION=4.51.1
# Keep this low on the Jetson: each C++ job can take ~1.5 GB and the robot's
# own processes share the same RAM. "c++: fatal error: Killed" = lower it.
ARG BUILD_JOBS=4

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=foxy

# --- 1. ROS apt key ----------------------------------------------------------
# The key baked into old ros:foxy images expired on 2025-06-01, so a plain
# `apt-get update` fails with EXPKEYSIG F42ED6FBAB17C654. Drop the stale ROS
# source, then re-add it with the renewed key (valid to 2030). Foxy is EOL; its
# last (frozen) debs stay on packages.ros.org.
RUN rm -f /etc/apt/sources.list.d/ros2*.list \
          /usr/share/keyrings/ros2-latest-archive-keyring.gpg \
          /usr/share/keyrings/ros-archive-keyring.gpg \
    && apt-get update \
    && apt-get install -y --no-install-recommends curl ca-certificates gnupg \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" \
            > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && rm -rf /var/lib/apt/lists/*

# --- 2. Build tools + the wrapper's ROS dependencies ---------------------------
# Listed by hand instead of rosdep: rosdep needs --include-eol-distros for Foxy
# and would pull ros-foxy-librealsense2 (a second, different librealsense =
# the classic bad_alloc / segfault at start-up). image-transport-plugins adds
# the /compressed (JPEG) and /compressedDepth (PNG) topics used over Wi-Fi.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git pkg-config \
        libusb-1.0-0-dev libudev-dev libssl-dev libeigen3-dev \
        usbutils gdb iproute2 \
        python3-colcon-common-extensions \
        ros-foxy-rmw-cyclonedds-cpp \
        ros-foxy-cv-bridge \
        ros-foxy-image-transport \
        ros-foxy-image-transport-plugins \
        ros-foxy-diagnostic-updater \
        ros-foxy-tf2-ros \
        ros-foxy-launch-ros \
        ros-foxy-xacro \
    && if dpkg -l | grep -q 'ros-foxy-librealsense2'; then \
           echo "ros-foxy-librealsense2 got installed: it would shadow the source build" >&2; exit 1; fi \
    && rm -rf /var/lib/apt/lists/*

# --- 3. librealsense from source, RSUSB backend --------------------------------
# FORCE_RSUSB_BACKEND talks to the camera through libusb from user space, so the
# Jetson kernel needs no RealSense patches and the host needs no udev rules.
# IMPORT_DEPTH_CAM_FW=OFF and CHECK_FOR_UPDATES=OFF: no downloads at cmake time;
# BUILD_WITH_TM2=OFF (T265 only) is required once the firmware import is off.
RUN git clone --depth 1 --branch "v${LIBREALSENSE_VERSION}" \
        https://github.com/IntelRealSense/librealsense.git /tmp/librealsense \
    && cmake -S /tmp/librealsense -B /tmp/librealsense/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_WITH_CUDA=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_GLSL_EXTENSIONS=OFF \
        -DBUILD_TOOLS=ON \
        -DBUILD_PYTHON_BINDINGS=OFF \
        -DBUILD_UNIT_TESTS=OFF \
        -DIMPORT_DEPTH_CAM_FW=OFF \
        -DBUILD_WITH_TM2=OFF \
        -DCHECK_FOR_UPDATES=OFF \
    && cmake --build /tmp/librealsense/build -j"${BUILD_JOBS}" \
    && cmake --install /tmp/librealsense/build \
    && ldconfig \
    && rm -rf /tmp/librealsense \
    && command -v rs-enumerate-devices

# --- 4. realsense-ros 4.51.1 (Foxy-compatible tag) ------------------------------
# --packages-up-to realsense2_camera builds realsense2_camera_msgs too.
WORKDIR /opt/rs_ws
RUN git clone --depth 1 --branch "${REALSENSE_ROS_VERSION}" \
        https://github.com/IntelRealSense/realsense-ros.git src/realsense-ros \
    && source /opt/ros/foxy/setup.bash \
    && MAKEFLAGS="-j${BUILD_JOBS}" colcon build \
        --merge-install \
        --packages-up-to realsense2_camera \
        --parallel-workers 1 \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log \
    && test -f install/lib/realsense2_camera/realsense2_camera_node \
    && ldd install/lib/librealsense2_camera.so | grep -q '/usr/local/lib/librealsense2.so' \
    && echo "realsense2_camera links /usr/local librealsense ${LIBREALSENSE_VERSION}: OK"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENTRYPOINT ["/entrypoint.sh"]
CMD ["camera"]
