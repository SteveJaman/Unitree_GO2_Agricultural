#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# verify_topics.sh
#
# Checks that every expected ROS 2 topic exists with the correct type.
# Run after launching the robot or simulation.
#
# Usage:
#   ./scripts/verify_topics.sh              # real robot (Ethernet)
#   ./scripts/verify_topics.sh --sim        # Unity simulation
#   ./scripts/verify_topics.sh --webrtc     # WebRTC fallback
#   ./scripts/verify_topics.sh --dog        # just the dog, no autonomy stack
# ---------------------------------------------------------------------------
set -eo pipefail

MODE="ethernet"
case "${1:-}" in
    --sim)     MODE="simulation" ;;
    --webrtc)  MODE="webrtc" ;;
    --dog)     MODE="dog" ;;
    --ethernet|"") MODE="ethernet" ;;
    *) echo "Unknown mode: $1"; exit 1 ;;
esac

WS_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." &> /dev/null && pwd )"

set +u
source /opt/ros/humble/setup.bash
[ -f "${WS_ROOT}/install/setup.bash" ] && source "${WS_ROOT}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

case "$MODE" in
    ethernet|simulation|dog)
        export CYCLONEDDS_URI="file://${WS_ROOT}/config/cyclonedds_ethernet.xml" ;;
    webrtc)
        export CYCLONEDDS_URI="file://${WS_ROOT}/config/cyclonedds_wireless.xml" ;;
esac

echo "================================================================"
echo " Verifying topics in mode: ${MODE}"
echo " CYCLONEDDS_URI: ${CYCLONEDDS_URI}"
echo "================================================================"
echo

# --- Define expected topics per mode ---
case "$MODE" in
    ethernet|simulation)
        # CMU autonomy stack remaps /cloud_registered to /registered_scan
        EXPECTED=(
            "/utlidar/cloud:sensor_msgs/msg/PointCloud2"
            "/utlidar/imu:sensor_msgs/msg/Imu"
            "/registered_scan:sensor_msgs/msg/PointCloud2"
            "/state_estimation:nav_msgs/msg/Odometry"
            "/cmd_vel:geometry_msgs/msg/TwistStamped"
        )
        ;;
    dog)
        # Just the dog powered on, no autonomy stack
        EXPECTED=(
            "/utlidar/cloud:sensor_msgs/msg/PointCloud2"
            "/utlidar/cloud_deskewed:sensor_msgs/msg/PointCloud2"
            "/utlidar/imu:sensor_msgs/msg/Imu"
            "/utlidar/robot_odom:nav_msgs/msg/Odometry"
        )
        ;;
    webrtc)
        EXPECTED=(
            "/point_cloud2:sensor_msgs/msg/PointCloud2"
            "/imu/data:sensor_msgs/msg/Imu"
            "/cloud_registered:sensor_msgs/msg/PointCloud2"
            "/state_estimation:nav_msgs/msg/Odometry"
            "/cmd_vel:geometry_msgs/msg/Twist"
        )
        ;;
esac

# --- Check each expected topic ---
PASS=0
FAIL=0

for entry in "${EXPECTED[@]}"; do
    topic="${entry%%:*}"
    expected_type="${entry##*:}"

    if ! ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
        echo "  MISSING  ${topic}"
        FAIL=$((FAIL + 1))
        continue
    fi

    actual_type=$(ros2 topic info "${topic}" 2>/dev/null | grep "^Type:" | awk '{print $2}')

    if [ "${actual_type}" = "${expected_type}" ]; then
        echo "  OK       ${topic}  [${actual_type}]"
        PASS=$((PASS + 1))
    else
        echo "  TYPE ERR ${topic}"
        echo "           expected: ${expected_type}"
        echo "           actual:   ${actual_type}"
        FAIL=$((FAIL + 1))
    fi
done

echo
echo "---------------------------------------------------------------"
echo "  ${PASS} OK,  ${FAIL} failed"
echo "---------------------------------------------------------------"

# --- Check /map/occupancy QoS if the mapping node is running ---
if ros2 topic list 2>/dev/null | grep -qx "/map/occupancy"; then
    echo
    echo "Checking /map/occupancy QoS (must be TRANSIENT_LOCAL):"
    ros2 topic info /map/occupancy -v 2>/dev/null | grep -A 1 "Durability" || true
fi

exit ${FAIL}