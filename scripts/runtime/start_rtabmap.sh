#!/usr/bin/env bash
# start_rtabmap.sh — launch RTAB-Map for dense 3D reconstruction.
#
# Subscribes to the live camera + LiDAR topics, builds a dense colored
# map, and stores it in an on-disk database. Runs alongside the Nav2
# session; does NOT interfere with navigation.
#
# Requires:
#   - A session running (wifi_nav.sh --no-explore or similar)
#   - RealSense D435i, or the built-in camera as fallback
#   - ros-humble-rtabmap-ros installed in the container
#
# Usage:
#   ./scripts/runtime/start_rtabmap.sh              # default topics
#   ./scripts/runtime/start_rtabmap.sh --lidar-only # skip RGB-D
#   RTAB_DB=/tmp/tour.db ./scripts/runtime/start_rtabmap.sh
set -eo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
RTAB_DB="${RTAB_DB:-$HOME/.ros/tour.db}"
LIDAR_ONLY="false"

while [ $# -gt 0 ]; do
    case "$1" in
        --lidar-only) LIDAR_ONLY="true" ;;
        -h|--help)    sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "unknown: $1" >&2; exit 1 ;;
    esac
    shift
done

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "ERROR: container $CONTAINER_NAME not running." >&2
    exit 1
fi

# Verify packages
docker exec -u root "$CONTAINER_NAME" bash -lc '
    source /opt/ros/humble/setup.bash
    ros2 pkg list 2>/dev/null | grep -q rtabmap_ros
' || {
    echo "ERROR: rtabmap_ros not installed. Run inside the container:"
    echo "  apt-get install -y ros-humble-rtabmap-ros"
    exit 1
}

echo "[rtabmap] Killing any previous instance..."
docker exec -u root "$CONTAINER_NAME" pkill -9 -f rtabmap 2>/dev/null || true
sleep 1

if [ "$LIDAR_ONLY" = "true" ]; then
    echo "[rtabmap] LiDAR-only mode (no RGB-D)."
    LAUNCH_ARGS="scan_cloud_topic:=/lidar3d/registered \
        subscribe_scan_cloud:=true \
        subscribe_depth:=false \
        subscribe_rgb:=false"
else
    echo "[rtabmap] RGB-D + LiDAR fusion mode."
    LAUNCH_ARGS="rgb_topic:=/camera/color/image_raw \
        depth_topic:=/camera/depth/image_rect_raw \
        camera_info_topic:=/camera/color/camera_info \
        scan_cloud_topic:=/lidar3d/registered \
        subscribe_scan_cloud:=true \
        subscribe_depth:=true \
        subscribe_rgb:=true"
fi

echo "[rtabmap] Database: $RTAB_DB"
echo "[rtabmap] Starting..."

docker exec -u root -d "$CONTAINER_NAME" bash -lc "
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash 2>/dev/null
    exec ros2 launch rtabmap_ros rtabmap.launch.py \
        $LAUNCH_ARGS \
        frame_id:=base_link \
        approx_sync:=true \
        rtabmapviz:=false \
        rviz:=false \
        database_path:=$RTAB_DB \
        rtabmap_args:='--delete_db_on_start --RGBD/CreateOccupancyGrid true --Grid/Sensor 2'
" > /tmp/rtabmap.log 2>&1 &

sleep 3

if docker exec "$CONTAINER_NAME" pgrep -f rtabmap >/dev/null; then
    echo "[rtabmap] Running. Log: /tmp/rtabmap.log (inside container)"
    echo "[rtabmap] To export the dense map after the run:"
    echo "  ./scripts/tools/export_tour.py $RTAB_DB -o tour.ply"
else
    echo "[rtabmap] ERROR: process not running. Last 20 lines:"
    docker exec "$CONTAINER_NAME" tail -20 /tmp/rtabmap.log 2>/dev/null || true
    exit 1
fi