#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# system_real_robot_ethernet.sh
#
# Full autonomy pipeline using the CMU autonomy_stack_go2, with our
# integration-layer additions layered on top.
#
# Default behavior mirrors the CMU autonomy repo:
#   - Launches vehicle_simulator system_real_robot.launch
#   - The CMU RViz opens with waypoint tools (this is the interactive UI)
#   - You drive the robot by clicking waypoints in the CMU RViz
#
# On top of that, this script adds:
#   - TF bridges so RViz can render the point cloud without "No transform"
#     errors (the CMU stack publishes in 'vehicle'/'body'/'sensor' frames
#     but its RViz config uses 'camera_init' as fixed frame)
#   - map_node.py running in the background, saving a 2D occupancy grid
#     and 3D point cloud to ~/go2_maps/<timestamp>/ on Ctrl+C
#   - pointcloud_to_scan.py producing /scan for SLAM / Nav2 (optional)
#   - camera_relay.py for RViz-friendly camera QoS (optional)
#
# On Ctrl+C, the script:
#   1. Sends SIGINT to map_node.py so it saves the map
#   2. Waits for the map save to complete
#   3. Tears down the CMU stack and other background processes
#
# Network:
#   VM (enp0s8) : 192.168.123.100/24
#   Go2 Jetson  : 192.168.123.18/24
# ---------------------------------------------------------------------------
set -eo pipefail

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
LOG_DIR="${WS_ROOT}/log"
CYCLONEDDS_XML="${WS_ROOT}/config/cyclonedds_ethernet.xml"
AUTONOMY_WS="${AUTONOMY_WS:-$HOME/files/autonomy_stack_go2}"

# Network
VM_IFACE="${VM_IFACE:-enp0s8}"
VM_IP="${VM_IP:-192.168.123.100}"
JETSON_IP="${JETSON_IP:-192.168.123.18}"

# Feature flags
ENABLE_MAPPING="${ENABLE_MAPPING:-true}"     # save map via map_node.py
ENABLE_SCAN_CONV="${ENABLE_SCAN_CONV:-false}" # /registered_scan -> /scan
ENABLE_CAMERA="${ENABLE_CAMERA:-auto}"       # auto | true | false
ENABLE_TF_BRIDGES="${ENABLE_TF_BRIDGES:-true}" # static frames for RViz
USE_CMU_RVIZ="${USE_CMU_RVIZ:-true}"         # use CMU's RViz (waypoint tools)

# Fixed frame expected by the CMU RViz config
TOP_FRAME="${TOP_FRAME_OVERRIDE:-camera_init}"

mkdir -p "${LOG_DIR}"

# ---------------------------------------------------------------------------
# Sanity checks
# ---------------------------------------------------------------------------
[ -f "${CYCLONEDDS_XML}" ] || { echo "ERROR: ${CYCLONEDDS_XML} not found"; exit 1; }
[ -d "${AUTONOMY_WS}/install" ] || { echo "ERROR: ${AUTONOMY_WS} not built"; exit 1; }
[ -f "${WS_ROOT}/install/setup.bash" ] || {
  echo "ERROR: this workspace not built. Run: colcon build --symlink-install"
  exit 1
}

# ---------------------------------------------------------------------------
# Source ROS 2 and workspaces
# ---------------------------------------------------------------------------
set +u
source /opt/ros/humble/setup.bash
source "${AUTONOMY_WS}/install/setup.bash"
source "${WS_ROOT}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONEDDS_XML}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ---------------------------------------------------------------------------
# Banner
# ---------------------------------------------------------------------------
echo "================================================================"
echo " Go2 autonomy + mapping (Ethernet)"
echo "================================================================"
echo "  RMW             = ${RMW_IMPLEMENTATION}"
echo "  CYCLONEDDS_URI  = ${CYCLONEDDS_URI}"
echo "  ROS_DOMAIN_ID   = ${ROS_DOMAIN_ID}"
echo "  VM interface    = ${VM_IFACE}  (${VM_IP})"
echo "  Jetson          = ${JETSON_IP}"
echo "  Fixed frame     = ${TOP_FRAME}"
echo "  Mapping         = ${ENABLE_MAPPING}"
echo "  Scan conversion = ${ENABLE_SCAN_CONV}"
echo "  TF bridges      = ${ENABLE_TF_BRIDGES}"
echo "  CMU RViz        = ${USE_CMU_RVIZ}"
echo "  Logs            = ${LOG_DIR}"
echo

if ! ip addr show "${VM_IFACE}" 2>/dev/null | grep -q "${VM_IP}"; then
  echo "[ethernet] WARN: ${VM_IFACE} missing ${VM_IP}/24"
  echo "[ethernet]       run: sudo ip addr add ${VM_IP}/24 dev ${VM_IFACE}"
fi

if ! ping -c 1 -W 1 "${JETSON_IP}" > /dev/null 2>&1; then
  echo "[ethernet] WARN: cannot ping ${JETSON_IP} (ICMP often blocked — continuing)"
fi

# ---------------------------------------------------------------------------
# Cleanup
# ---------------------------------------------------------------------------
PIDS=()
MAP_NODE_PID=""

cleanup() {
  echo
  echo "[ethernet] Shutting down..."

  # 1. Ask map_node.py to save and exit (it handles SIGINT specially)
  if [ -n "${MAP_NODE_PID}" ] && kill -0 "${MAP_NODE_PID}" 2>/dev/null; then
    echo "[ethernet]   saving map (this may take a few seconds)..."
    kill -INT "${MAP_NODE_PID}" 2>/dev/null || true
    # Wait up to 60s for the map save to finish
    for i in $(seq 1 60); do
      kill -0 "${MAP_NODE_PID}" 2>/dev/null || break
      sleep 1
    done
    kill -TERM "${MAP_NODE_PID}" 2>/dev/null || true
  fi

  # 2. Kill everything else
  for pid in "${PIDS[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true

  echo "[ethernet] Done."
}
trap cleanup EXIT INT TERM

wait_for_topic() {
  local topic="$1"; local timeout="${2:-30}"; local i=0
  while [ "${i}" -lt "${timeout}" ]; do
    ros2 topic list 2>/dev/null | grep -qxF "${topic}" && return 0
    sleep 1; i=$((i + 1))
  done
  return 1
}

# ---------------------------------------------------------------------------
# 0. Kill any stale processes from previous runs
# ---------------------------------------------------------------------------
pkill -f "rviz2" 2>/dev/null || true
pkill -f "static_transform_publisher" 2>/dev/null || true
sleep 1

# ---------------------------------------------------------------------------
# 1. Launch the CMU autonomy stack
# ---------------------------------------------------------------------------
# Use USE_CMU_RVIZ to control whether the CMU RViz opens (recommended — it
# has the waypoint tools and is the intended interaction surface).
if [ "${USE_CMU_RVIZ}" = "true" ]; then
  RVIZ_ARG="rviz:=true"
else
  RVIZ_ARG="rviz:=false"
fi

echo "[1/5] Launching CMU autonomy stack (${RVIZ_ARG})..."
ros2 launch vehicle_simulator system_real_robot.launch ${RVIZ_ARG} \
    > "${LOG_DIR}/autonomy.log" 2>&1 &
CMU_PID=$!
PIDS+=("${CMU_PID}")
echo "      CMU PID = ${CMU_PID}  (log: ${LOG_DIR}/autonomy.log)"

if [ "${USE_CMU_RVIZ}" = "false" ]; then
  # CMU still opens its own RViz regardless — clean it up
  sleep 5
  pkill -f "rviz2" 2>/dev/null || true
fi

echo "[1/5] Waiting for Point-LIO to publish /state_estimation (up to 90s)..."
if wait_for_topic /state_estimation 90; then
  echo "      /state_estimation is up."
else
  echo "      WARN: /state_estimation did not appear within 90s."
  echo "      Check ${LOG_DIR}/autonomy.log"
  echo "      If stuck at 'IMU Initializing: 100.0%', fix use_sim_time:"
  echo "        grep use_sim_time ${AUTONOMY_WS}/src/slam/point_lio_unilidar/config/utlidar.yaml"
fi

# ---------------------------------------------------------------------------
# 2. Publish static TF bridges
# ---------------------------------------------------------------------------
if [ "${ENABLE_TF_BRIDGES}" = "true" ]; then
  echo
  echo "[2/5] Publishing static TF bridges under ${TOP_FRAME}..."

  CMU_FRAMES=(
    body vehicle sensor lidar3d_map camera aft_mapped base_link base_footprint
  )

  for child in "${CMU_FRAMES[@]}"; do
    [ "${child}" = "${TOP_FRAME}" ] && continue
    ros2 run tf2_ros static_transform_publisher \
        --frame-id "${TOP_FRAME}" --child-frame-id "${child}" \
        > /dev/null 2>&1 &
    PIDS+=("$!")
  done

  # Also bridge `map` -> `camera_init` so any RViz config using `map`
  # as Fixed Frame resolves.
  ros2 run tf2_ros static_transform_publisher \
      --frame-id map --child-frame-id "${TOP_FRAME}" \
      > /dev/null 2>&1 &
  PIDS+=("$!")

  sleep 1
  echo "      bridges: ${CMU_FRAMES[*]} + map -> ${TOP_FRAME}"
else
  echo
  echo "[2/5] TF bridges: disabled"
fi

# ---------------------------------------------------------------------------
# 3. Pick cloud topic + optional scan conversion / camera relay
# ---------------------------------------------------------------------------
echo
echo "[3/5] Detecting topics..."

CLOUD_TOPIC=""
for candidate in /registered_scan /cloud_registered /cloud_registered_body \
                 /utlidar/cloud_deskewed /utlidar/cloud; do
  if ros2 topic list 2>/dev/null | grep -qxF "${candidate}"; then
    CLOUD_TOPIC="${candidate}"
    break
  fi
done

if [ -z "${CLOUD_TOPIC}" ]; then
  echo "      ERROR: no cloud topic available."
  exit 1
fi
echo "      cloud topic = ${CLOUD_TOPIC}"

CAMERA_IMAGE_TOPIC=""
CAMERA_INFO_TOPIC=""
for candidate in /camera/image_raw /camera/image/raw \
                 /frontvideostream /videohub/inner; do
  ros2 topic list 2>/dev/null | grep -qxF "${candidate}" && {
    CAMERA_IMAGE_TOPIC="${candidate}"; break; }
done
for candidate in /camera/camera_info /camera/image/camera_info; do
  ros2 topic list 2>/dev/null | grep -qxF "${candidate}" && {
    CAMERA_INFO_TOPIC="${candidate}"; break; }
done

if [ "${ENABLE_SCAN_CONV}" = "true" ]; then
  echo "      launching pointcloud_to_scan.py (${CLOUD_TOPIC} -> /scan)"
  ros2 run go2_integration_pkg pointcloud_to_scan.py \
      --ros-args -p input_topic:="${CLOUD_TOPIC}" \
      > "${LOG_DIR}/pointcloud_to_scan.log" 2>&1 &
  PIDS+=("$!")
  sleep 1
fi

RUN_CAMERA="false"
case "${ENABLE_CAMERA}" in
  auto)  [ -n "${CAMERA_IMAGE_TOPIC}" ] && RUN_CAMERA="true" ;;
  true)  RUN_CAMERA="true" ;;
  false) RUN_CAMERA="false" ;;
esac

if [ "${RUN_CAMERA}" = "true" ] && [ -n "${CAMERA_IMAGE_TOPIC}" ]; then
  echo "      launching camera_relay.py (${CAMERA_IMAGE_TOPIC} -> /camera/image_raw_relayed)"
  ros2 run go2_integration_pkg camera_relay.py \
      --ros-args \
      -p input_image_topic:="${CAMERA_IMAGE_TOPIC}" \
      -p output_image_topic:="/camera/image_raw_relayed" \
      ${CAMERA_INFO_TOPIC:+-p input_info_topic:="${CAMERA_INFO_TOPIC}"} \
      > "${LOG_DIR}/camera_relay.log" 2>&1 &
  PIDS+=("$!")
  sleep 1
fi

# ---------------------------------------------------------------------------
# 4. Launch map_node.py in the background (saves map on Ctrl+C)
# ---------------------------------------------------------------------------
if [ "${ENABLE_MAPPING}" = "true" ]; then
  echo
  echo "[4/5] Launching map_node.py (cloud=${CLOUD_TOPIC}, odom=/state_estimation)..."
  ros2 run go2_integration_pkg map_node.py --ros-args \
      -p cloud_topic:="${CLOUD_TOPIC}" \
      -p odom_topic:="/state_estimation" \
      > "${LOG_DIR}/map_node.log" 2>&1 &
  MAP_NODE_PID=$!
  PIDS+=("${MAP_NODE_PID}")
  sleep 2

  if ros2 topic list 2>/dev/null | grep -qxF "/map/occupancy"; then
    echo "      /map/occupancy is up."
    echo "      /map/points     is up."
  else
    echo "      WARN: /map/occupancy did not appear. Check ${LOG_DIR}/map_node.log"
  fi
else
  echo
  echo "[4/5] map_node.py: disabled"
fi

# ---------------------------------------------------------------------------
# 5. Summary + wait
# ---------------------------------------------------------------------------
echo
echo "================================================================"
echo " Full autonomy + mapping pipeline running"
echo "================================================================"
echo "  CMU stack        : Point-LIO, localPlanner, terrainAnalysis,"
echo "                     pathFollower"
echo "  Fixed frame      : ${TOP_FRAME}"
echo "  Cloud used       : ${CLOUD_TOPIC}"
echo "  Odom used        : /state_estimation"
[ "${ENABLE_SCAN_CONV}" = "true" ] && echo "  Scan conversion  : /registered_scan -> /scan"
[ -n "${CAMERA_IMAGE_TOPIC}" ] && echo "  Camera relay     : ${CAMERA_IMAGE_TOPIC} -> /camera/image_raw_relayed"
[ "${ENABLE_MAPPING}" = "true" ] && echo "  Mapping outputs  : /map/occupancy, /map/points (saved on Ctrl+C)"
echo
if [ "${USE_CMU_RVIZ}" = "true" ]; then
  echo "  >>> Use the CMU RViz window that opened to control the robot:"
  echo "      1. Stand the robot with the physical remote first."
  echo "      2. In RViz, use the 'Waypoint' button to set a goal."
  echo "      3. Click a point 1-2 meters ahead on the ground."
  echo "      4. The robot will walk to that waypoint."
  echo
  echo "  >>> Optional: also add in RViz:"
  echo "        Map         -> /map/occupancy   (2D grid)"
  echo "        PointCloud2 -> /map/points      (accumulated 3D cloud)"
else
  echo "  RViz was suppressed. To view:  rviz2"
fi
echo
[ "${ENABLE_MAPPING}" = "true" ] && \
  echo "  On Ctrl+C: map saves to ~/go2_maps/<timestamp>/"
echo "  Press Ctrl+C to stop everything."
echo "================================================================"
echo

# Block. Cleanup trap handles everything on Ctrl+C.
if [ "${USE_CMU_RVIZ}" = "false" ]; then
  # Nothing is in the foreground, so tail the log to keep the script alive
  tail -f "${LOG_DIR}/autonomy.log"
else
  # CMU RViz is already open in the background; just wait for Ctrl+C
  wait "${CMU_PID}" 2>/dev/null || true
fi
