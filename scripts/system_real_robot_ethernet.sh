#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# system_real_robot_ethernet.sh
#
# Full real-robot pipeline over Ethernet to the Go2 Jetson.
#
# What it does:
#   1. Sources ROS 2 Humble + the CMU autonomy stack + this workspace
#   2. Sets CycloneDDS to use the Ethernet interface + Jetson peer
#   3. Sanity-checks the network path to the Jetson
#   4. Launches the CMU autonomy stack in the background
#   5. Waits for Point-LIO to publish /registered_scan + /state_estimation
#   6. Launches pointcloud_to_scan.py to produce /scan for SLAM
#   7. Launches map_node.py for 2D occupancy + accumulated 3D cloud
#   8. Launches camera_relay.py if a camera topic is present
#   9. Optionally launches RViz with sensible displays
#  10. On Ctrl+C, tears down everything cleanly
#
# Network:
#   VM (enp0s8) : 192.168.123.100/24
#   Go2 Jetson  : 192.168.123.18/24
#
# No WebRTC, no SDK, no relay. The Jetson publishes /utlidar/* natively
# over DDS, and the autonomy stack consumes them.
# ---------------------------------------------------------------------------
set -eo pipefail

# ---------------------------------------------------------------------------
# Paths and configuration
# ---------------------------------------------------------------------------
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
LOG_DIR="${WS_ROOT}/log"
CYCLONEDDS_XML="${WS_ROOT}/config/cyclonedds_ethernet.xml"
AUTONOMY_WS="${AUTONOMY_WS:-$HOME/files/autonomy_stack_go2}"

# Network config
VM_IFACE="${VM_IFACE:-enp0s8}"
VM_IP="${VM_IP:-192.168.123.100}"
JETSON_IP="${JETSON_IP:-192.168.123.18}"

# Feature flags (override with environment variables)
ENABLE_MAPPING="${ENABLE_MAPPING:-true}"     # run map_node.py
ENABLE_SCAN_CONV="${ENABLE_SCAN_CONV:-true}" # run pointcloud_to_scan.py
ENABLE_CAMERA="${ENABLE_CAMERA:-auto}"       # auto | true | false
ENABLE_RVIZ="${ENABLE_RVIZ:-true}"           # open RViz at the end
PREFER_DESKEWED="${PREFER_DESKEWED:-true}"   # prefer /utlidar/cloud_deskewed for map_node

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
# Network checks
# ---------------------------------------------------------------------------
echo "================================================================"
echo " Go2 real-robot pipeline (Ethernet)"
echo "================================================================"
echo "  RMW            = ${RMW_IMPLEMENTATION}"
echo "  CYCLONEDDS_URI = ${CYCLONEDDS_URI}"
echo "  ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID}"
echo "  VM interface   = ${VM_IFACE}  (${VM_IP})"
echo "  Jetson         = ${JETSON_IP}"
echo "  Logs           = ${LOG_DIR}"
echo

if ! ip addr show "${VM_IFACE}" 2>/dev/null | grep -q "${VM_IP}"; then
  echo "[ethernet] WARN: ${VM_IFACE} missing ${VM_IP}/24"
  echo "[ethernet]       run: sudo ip addr add ${VM_IP}/24 dev ${VM_IFACE}"
fi

if ! ping -c 1 -W 1 "${JETSON_IP}" > /dev/null 2>&1; then
  echo "[ethernet] WARN: cannot reach Jetson at ${JETSON_IP}"
  echo "[ethernet]       continuing anyway — DDS may still work"
fi

# ---------------------------------------------------------------------------
# Cleanup handler
# ---------------------------------------------------------------------------
PIDS=()

cleanup() {
  echo
  echo "[ethernet] Shutting down background nodes..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
  echo "[ethernet] Done."
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------------
# Helper: wait for a topic to appear
# ---------------------------------------------------------------------------
wait_for_topic() {
  local topic="$1"
  local timeout="${2:-30}"
  local elapsed=0
  while [ "${elapsed}" -lt "${timeout}" ]; do
    if ros2 topic list 2>/dev/null | grep -qxF "${topic}"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

# ---------------------------------------------------------------------------
# 1. Launch the CMU autonomy stack in the background
# ---------------------------------------------------------------------------
echo "[1/5] Launching CMU autonomy stack..."
ros2 launch vehicle_simulator system_real_robot.launch \
    > "${LOG_DIR}/autonomy.log" 2>&1 &
AUTONOMY_PID=$!
PIDS+=("${AUTONOMY_PID}")
echo "      autonomy PID = ${AUTONOMY_PID}  (log: ${LOG_DIR}/autonomy.log)"

echo "[1/5] Waiting for Point-LIO to publish /state_estimation..."
if wait_for_topic /state_estimation 60; then
  echo "      /state_estimation is up."
else
  echo "      WARN: /state_estimation did not appear within 60s."
  echo "      Check ${LOG_DIR}/autonomy.log for errors."
  echo "      Common fix: set use_sim_time: false in"
  echo "      ${AUTONOMY_WS}/src/slam/point_lio_unilidar/config/utlidar.yaml"
fi

# ---------------------------------------------------------------------------
# 2. Determine which cloud topic to use
# ---------------------------------------------------------------------------
echo
echo "[2/5] Detecting cloud and camera topics..."

CLOUD_TOPIC=""
if [ "${PREFER_DESKEWED}" = "true" ] && \
   ros2 topic list 2>/dev/null | grep -qxF "/utlidar/cloud_deskewed"; then
  CLOUD_TOPIC="/utlidar/cloud_deskewed"
else
  for candidate in /registered_scan /cloud_registered \
                   /utlidar/cloud_deskewed /utlidar/cloud; do
    if ros2 topic list 2>/dev/null | grep -qxF "${candidate}"; then
      CLOUD_TOPIC="${candidate}"
      break
    fi
  done
fi

if [ -z "${CLOUD_TOPIC}" ]; then
  echo "      ERROR: no cloud topic available."
  echo "      Is the robot powered on and publishing /utlidar/cloud?"
  exit 1
fi
echo "      cloud topic  = ${CLOUD_TOPIC}"

# Camera detection — handle both /camera/image_raw and /camera/image/raw
CAMERA_IMAGE_TOPIC=""
CAMERA_INFO_TOPIC=""
for candidate in /camera/image_raw /camera/image/raw \
                 /frontvideostream /videohub/inner; do
  if ros2 topic list 2>/dev/null | grep -qxF "${candidate}"; then
    CAMERA_IMAGE_TOPIC="${candidate}"
    break
  fi
done
for candidate in /camera/camera_info /camera/image/camera_info; do
  if ros2 topic list 2>/dev/null | grep -qxF "${candidate}"; then
    CAMERA_INFO_TOPIC="${candidate}"
    break
  fi
done

if [ -n "${CAMERA_IMAGE_TOPIC}" ]; then
  echo "      camera image = ${CAMERA_IMAGE_TOPIC}"
  [ -n "${CAMERA_INFO_TOPIC}" ] && \
    echo "      camera info  = ${CAMERA_INFO_TOPIC}"
else
  echo "      camera image = (not found)"
fi

# ---------------------------------------------------------------------------
# 3. Launch pointcloud_to_scan.py
# ---------------------------------------------------------------------------
if [ "${ENABLE_SCAN_CONV}" = "true" ]; then
  echo
  echo "[3/5] Launching pointcloud_to_scan.py (${CLOUD_TOPIC} -> /scan)..."
  ros2 run go2_integration_pkg pointcloud_to_scan.py \
      --ros-args -p input_topic:="${CLOUD_TOPIC}" \
      > "${LOG_DIR}/pointcloud_to_scan.log" 2>&1 &
  PIDS+=("$!")
  sleep 2

  if ros2 topic list 2>/dev/null | grep -qxF "/scan"; then
    echo "      /scan is up."
  else
    echo "      WARN: /scan did not appear. Check ${LOG_DIR}/pointcloud_to_scan.log"
  fi
else
  echo
  echo "[3/5] pointcloud_to_scan.py: disabled"
fi

# ---------------------------------------------------------------------------
# 4. Launch map_node.py (2D occupancy + 3D cloud)
# ---------------------------------------------------------------------------
if [ "${ENABLE_MAPPING}" = "true" ]; then
  echo
  echo "[4/5] Launching map_node.py..."
  ros2 run go2_integration_pkg map_node.py \
      --ros-args \
      -p cloud_topic:="${CLOUD_TOPIC}" \
      > "${LOG_DIR}/map_node.log" 2>&1 &
  PIDS+=("$!")
  sleep 2

  if ros2 topic list 2>/dev/null | grep -qxF "/map/occupancy"; then
    echo "      /map/occupancy is up."
    echo "      /map/points     is up."
  else
    echo "      WARN: /map/occupancy did not appear."
    echo "      Check ${LOG_DIR}/map_node.log"
  fi
else
  echo
  echo "[4/5] map_node.py: disabled"
fi

# ---------------------------------------------------------------------------
# 5. Launch camera_relay.py if a camera is present
# ---------------------------------------------------------------------------
RUN_CAMERA="false"
case "${ENABLE_CAMERA}" in
  auto)  [ -n "${CAMERA_IMAGE_TOPIC}" ] && RUN_CAMERA="true" ;;
  true)  RUN_CAMERA="true" ;;
  false) RUN_CAMERA="false" ;;
esac

if [ "${RUN_CAMERA}" = "true" ] && [ -n "${CAMERA_IMAGE_TOPIC}" ]; then
  echo
  echo "[5/5] Launching camera_relay.py (${CAMERA_IMAGE_TOPIC} -> /camera/image_raw_relayed)..."
  ros2 run go2_integration_pkg camera_relay.py \
      --ros-args \
      -p input_image_topic:="${CAMERA_IMAGE_TOPIC}" \
      -p output_image_topic:="/camera/image_raw_relayed" \
      ${CAMERA_INFO_TOPIC:+-p input_info_topic:="${CAMERA_INFO_TOPIC}"} \
      > "${LOG_DIR}/camera_relay.log" 2>&1 &
  PIDS+=("$!")
  sleep 1

  if ros2 topic list 2>/dev/null | grep -qxF "/camera/image_raw_relayed"; then
    echo "      /camera/image_raw_relayed is up."
  else
    echo "      WARN: relay topic did not appear. Check ${LOG_DIR}/camera_relay.log"
  fi
else
  echo
  echo "[5/5] camera_relay.py: disabled (no camera topic or ENABLE_CAMERA=false)"
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo
echo "================================================================"
echo " Pipeline running. Active topics:"
echo "================================================================"
echo "  Sensors from robot      : /utlidar/cloud, /utlidar/imu,"
echo "                            /utlidar/cloud_deskewed"
echo "  Point-LIO output        : /registered_scan, /state_estimation"
echo "  Scan conversion         : /scan"
echo "  Mapping outputs         : /map/occupancy, /map/points"
[ -n "${CAMERA_IMAGE_TOPIC}" ] && echo "  Camera (relayed)        : /camera/image_raw_relayed"
echo "  Control                 : /cmd_vel"
echo
echo " Logs in ${LOG_DIR}/"
echo "  autonomy.log            — CMU stack"
echo "  pointcloud_to_scan.log  — scan conversion"
echo "  map_node.log            — mapping"
[ -n "${CAMERA_IMAGE_TOPIC}" ] && echo "  camera_relay.log        — camera relay"
echo
echo " In RViz, add these displays:"
echo "   Map         -> /map/occupancy   (TRANSIENT_LOCAL)"
echo "   Map         -> /map             (from slam_toolbox if running)"
echo "   LaserScan   -> /scan"
echo "   PointCloud2 -> /registered_scan"
echo "   PointCloud2 -> /map/points"
echo "   Odometry    -> /state_estimation"
[ -n "${CAMERA_IMAGE_TOPIC}" ] && echo "   Image       -> /camera/image_raw_relayed"
echo "   TF          -> /tf, /tf_static"
echo
echo " Send a goal with '2D Goal Pose' in RViz — /cmd_vel will appear."
echo " Press Ctrl+C to shut down cleanly."
echo "================================================================"
echo

# ---------------------------------------------------------------------------
# Launch RViz in the foreground (blocks until Ctrl+C)
# ---------------------------------------------------------------------------
if [ "${ENABLE_RVIZ}" = "true" ]; then
  # Prefer the CMU stack's default RViz config if present
  RViz_CFG="${AUTONOMY_WS}/install/far_planner/share/far_planner/rviz/default.rviz"
  if [ -f "${RViz_CFG}" ]; then
    rviz2 -d "${RViz_CFG}"
  else
    rviz2
  fi
else
  # No RViz — block on the autonomy log instead
  echo "[ethernet] RViz disabled. Tailing autonomy log (Ctrl+C to stop)..."
  tail -f "${LOG_DIR}/autonomy.log"
fi