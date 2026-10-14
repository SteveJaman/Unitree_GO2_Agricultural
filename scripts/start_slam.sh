#!/usr/bin/env bash
# start_slam.sh — bring up pointcloud_to_scan + slam_toolbox inside the
# running go2-lidar3d container. Idempotent: kills any previous instance first.
set -eo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"

# Verify container is up
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "ERROR: container $CONTAINER_NAME is not running." >&2
    echo "Start it with: cd ~/files/go2-nav2-wifi && ./setup-lidar3d.sh --laptop" >&2
    exit 1
fi

echo "[slam] Killing any previous SLAM instances..."
docker exec -u root "$CONTAINER_NAME" bash -c '
    pkill -9 -f "slam_toolbox|pointcloud_to_laserscan|async_slam" 2>/dev/null || true
    sleep 1
' 2>/dev/null || true

echo "[slam] Checking packages are installed..."
docker exec -u root "$CONTAINER_NAME" bash -lc '
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash 2>/dev/null
    for pkg in slam_toolbox pointcloud_to_laserscan; do
        if ros2 pkg list 2>/dev/null | grep -qx "$pkg"; then
            echo "  $pkg: OK"
        else
            echo "  $pkg: MISSING"
            exit 1
        fi
    done
' || { echo "ERROR: missing SLAM packages. Install with:"; echo "  apt-get install ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan"; exit 1; }

echo "[slam] Starting pointcloud_to_scan (cloud_in=/lidar3d/registered → scan=/scan)..."
docker exec -u root -d "$CONTAINER_NAME" bash -lc '
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash 2>/dev/null
    exec ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
        --ros-args \
        -r cloud_in:=/lidar3d/registered \
        -r scan:=/scan \
        -p target_frame:=base_link \
        -p min_height:=-0.10 \
        -p max_height:=1.50 \
        -p angle_min:=-3.14159 \
        -p angle_max:=3.14159 \
        -p angle_increment:=0.0087 \
        -p range_min:=0.20 \
        -p range_max:=20.0
' > /tmp/scan.log 2>&1 &
SCAN_PID=$!

echo "[slam] Starting slam_toolbox (online_async)..."
docker exec -u root -d "$CONTAINER_NAME" bash -lc '
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash 2>/dev/null
    exec ros2 launch slam_toolbox online_async_launch.py \
        use_sim_time:=false
' > /tmp/slam.log 2>&1 &
SLAM_PID=$!

echo "[slam] Launched. Scan PID=$SCAN_PID  SLAM PID=$SLAM_PID"
echo "[slam] Wait ~10 s, then verify with: ./preflight.sh"
