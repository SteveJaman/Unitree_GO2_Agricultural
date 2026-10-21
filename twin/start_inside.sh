#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# start_inside.sh: entry point of the go2-3d container (run by wifi_3d_nav.sh).
#
# Starts, on the session's DDS domain:
#   * RTAB-Map            dense 3D map -> /rtabmap/cloud_map, database TWIN_DB
#   * web bridge          foxglove_bridge (default) or rosbridge, for Foxglove
#   * go2_click_to_goal   /clicked_point -> /goal_pose
#
# Odometry comes from TF (lidar3d_map -> base_link, Point-LIO + go2_odom_tf),
# so RTAB-Map's map frame "rtabmap_map" becomes the parent of lidar3d_map and
# Nav2 keeps working in lidar3d_map. On SIGTERM (docker stop) every child is
# stopped and RTAB-Map writes its database before exiting. If RTAB-Map dies
# the container exits; a bridge or click-node crash is logged only.
#
# Environment (set by wifi_3d_nav.sh; ROS_DOMAIN_ID / RMW / CYCLONEDDS_URI
# are copied from the running wifi_nav.sh session):
#   TWIN_DB=/data/rtabmap.db   TWIN_MODE=lidar|rgbd
#   TWIN_BRIDGE=foxglove|rosbridge|none   TWIN_BRIDGE_PORT=8765
#   TWIN_CLOUD_TOPIC=/lidar3d/registered
#   TWIN_RGB_TOPIC / TWIN_DEPTH_TOPIC / TWIN_CAMINFO_TOPIC   (rgbd mode)
#   TWIN_MAP_FRAME=lidar3d_map  TWIN_BASE_FRAME=base_link
#   TWIN_RTAB_ARGS="..."        extra RTAB-Map parameters, e.g. "--Grid/CellSize 0.1"
# ---------------------------------------------------------------------------
set -eo pipefail

TWIN_DB="${TWIN_DB:-/data/rtabmap.db}"
TWIN_MODE="${TWIN_MODE:-lidar}"
TWIN_BRIDGE="${TWIN_BRIDGE:-foxglove}"
TWIN_CLOUD_TOPIC="${TWIN_CLOUD_TOPIC:-/lidar3d/registered}"
TWIN_RGB_TOPIC="${TWIN_RGB_TOPIC:-/camera/color/image_raw}"
TWIN_DEPTH_TOPIC="${TWIN_DEPTH_TOPIC:-/camera/depth/image_rect_raw}"
TWIN_CAMINFO_TOPIC="${TWIN_CAMINFO_TOPIC:-/camera/color/camera_info}"
TWIN_MAP_FRAME="${TWIN_MAP_FRAME:-lidar3d_map}"
TWIN_BASE_FRAME="${TWIN_BASE_FRAME:-base_link}"
LOGS=/logs

mkdir -p "${HOME:-/tmp/go2-3d-home}" "$LOGS"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

# LiDAR-only RTAB-Map: ICP registration and proximity loop closures, 3D grid
# from the scans (after the rtabmap_examples lidar3d settings).
RTAB_PARAMS=(
    --Rtabmap/DetectionRate 1
    --RGBD/LinearUpdate 0.05 --RGBD/AngularUpdate 0.05
    --RGBD/ProximityMaxGraphDepth 0 --RGBD/ProximityPathMaxNeighbors 1
    --Mem/NotLinkedNodesKept false --Mem/STMSize 30 --Mem/LaserScanNormalK 20
    --Reg/Strategy 1
    --Icp/VoxelSize 0.1 --Icp/PointToPlane true --Icp/PointToPlaneK 20 --Icp/PointToPlaneRadius 0
    --Icp/Iterations 10 --Icp/Epsilon 0.001 --Icp/MaxTranslation 3
    --Icp/MaxCorrespondenceDistance 1 --Icp/CorrespondenceRatio 0.2
    --RGBD/CreateOccupancyGrid true --Grid/3D true --Grid/CellSize 0.05 --Grid/RangeMax 15
)
ROS_ARGS=(
    -r __node:=rtabmap -r __ns:=/rtabmap
    -p frame_id:="$TWIN_BASE_FRAME"
    -p odom_frame_id:="$TWIN_MAP_FRAME"
    -p map_frame_id:=rtabmap_map
    -p publish_tf:=true
    -p database_path:="$TWIN_DB"
    -p wait_for_transform:=0.5
    -p subscribe_scan_cloud:=true -p qos_scan:=2
    -r scan_cloud:="$TWIN_CLOUD_TOPIC"
)
if [ "$TWIN_MODE" = rgbd ]; then
    RTAB_PARAMS+=(--Grid/Sensor 2)
    ROS_ARGS+=(
        -p subscribe_rgb:=true -p subscribe_depth:=true -p approx_sync:=true
        -p qos_image:=2 -p qos_camera_info:=2
        -r rgb/image:="$TWIN_RGB_TOPIC" -r depth/image:="$TWIN_DEPTH_TOPIC"
        -r rgb/camera_info:="$TWIN_CAMINFO_TOPIC"
    )
else
    RTAB_PARAMS+=(--Grid/Sensor 0)
    ROS_ARGS+=(-p subscribe_rgb:=false -p subscribe_depth:=false)
fi
# shellcheck disable=SC2206
RTAB_PARAMS+=(${TWIN_RTAB_ARGS:-})

# Executables are run directly, not through `ros2 run`: its wrapper does not
# forward a signal sent to it alone, and RTAB-Map must see SIGTERM to save.
PIDS=(); NAMES=()
start() {   # $1 = name (log file /logs/<name>.log), rest = command
    local name="$1"; shift
    "$@" > "$LOGS/${name}.log" 2>&1 &
    PIDS+=("$!"); NAMES+=("$name")
    echo "[go2-3d] ${name} started (pid $!, log ${LOGS}/${name}.log)"
}

STOPPING=0
stop_all() {
    [ "$STOPPING" = 1 ] && return 0
    STOPPING=1
    echo "[go2-3d] stopping (RTAB-Map saves ${TWIN_DB})..."
    for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
    wait || true
    echo "[go2-3d] stopped"
}
trap 'stop_all; exit 0' TERM INT

echo "[go2-3d] DDS domain ${ROS_DOMAIN_ID:-0}, ${RMW_IMPLEMENTATION:-default RMW}, ${CYCLONEDDS_URI:-no CYCLONEDDS_URI}"
echo "[go2-3d] RTAB-Map ${TWIN_MODE}: cloud ${TWIN_CLOUD_TOPIC}, odometry TF ${TWIN_MAP_FRAME} -> ${TWIN_BASE_FRAME}"
start rtabmap "$(ros2 pkg prefix rtabmap_slam)/lib/rtabmap_slam/rtabmap" \
    "${RTAB_PARAMS[@]}" --ros-args "${ROS_ARGS[@]}"
RTAB_PID="${PIDS[0]}"

case "$TWIN_BRIDGE" in
    foxglove)
        start bridge "$(ros2 pkg prefix foxglove_bridge)/lib/foxglove_bridge/foxglove_bridge" \
            --ros-args -p port:="${TWIN_BRIDGE_PORT:-8765}" -p address:=0.0.0.0 ;;
    rosbridge)
        start bridge ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
            port:="${TWIN_BRIDGE_PORT:-9090}" ;;
    none) ;;
    *) echo "[go2-3d] ERROR: unknown TWIN_BRIDGE=${TWIN_BRIDGE}" >&2; stop_all; exit 1 ;;
esac

start click_to_goal python3 -u /nodes/go2_click_to_goal.py --ros-args \
    -p map_frame:="$TWIN_MAP_FRAME" -p robot_base_frame:="$TWIN_BASE_FRAME"

sleep 3
kill -0 "$RTAB_PID" 2>/dev/null || {
    echo "[go2-3d] ERROR: RTAB-Map exited at startup. Last log lines:" >&2
    tail -20 "$LOGS/rtabmap.log" >&2
    stop_all; exit 1
}
echo "[go2-3d] READY"

while :; do
    set +e
    wait -n
    set -e
    [ "$STOPPING" = 1 ] && break
    if ! kill -0 "$RTAB_PID" 2>/dev/null; then
        echo "[go2-3d] ERROR: RTAB-Map exited; see ${LOGS}/rtabmap.log" >&2
        stop_all; exit 1
    fi
    for i in "${!PIDS[@]}"; do
        kill -0 "${PIDS[$i]}" 2>/dev/null || echo "[go2-3d] WARN: ${NAMES[$i]} exited; see ${LOGS}/${NAMES[$i]}.log" >&2
    done
done
