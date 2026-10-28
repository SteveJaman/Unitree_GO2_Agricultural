#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# arm_moveit.sh
#
# MoveIt 2 for the D1-550 arm on the Go2, on mock hardware (no real arm), plus
# the plant-inspection demo. Runs on the host, no Docker.
#
# Usage:
#   ./scripts/runtime/arm_moveit.sh              # build if needed, RViz + MoveIt
#   ./scripts/runtime/arm_moveit.sh --inspect    # same, then run the plant inspection
#   ./scripts/runtime/arm_moveit.sh --build      # force a rebuild first
#   ./scripts/runtime/arm_moveit.sh --no-rviz    # headless
#
# Needs: sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
# The D1 description is built from D1_ARM_DIR (default ~/files/unitree-D1-550-Robot-ARM).
#
# Safety: ROS_DOMAIN_ID 78 + localhost-only DDS, so nothing reaches the robot.
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/../.." &> /dev/null && pwd )"
D1_ARM_DIR="${D1_ARM_DIR:-$HOME/files/unitree-D1-550-Robot-ARM}"
ARM_ROS_DOMAIN_ID="${ARM_ROS_DOMAIN_ID:-78}"

BUILD="auto"; INSPECT="false"; RVIZ="true"
while [ $# -gt 0 ]; do
    case "$1" in
        --build)   BUILD="always" ;;
        --inspect) INSPECT="true" ;;
        --no-rviz) RVIZ="false" ;;
        -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
        *) echo "Unknown option: $1 (see --help)" >&2; exit 2 ;;
    esac
    shift
done

fail() { echo "ERROR: $*" >&2; exit 1; }

[ -f /opt/ros/humble/setup.bash ] || fail "ROS 2 Humble not found"
for pkg in moveit_ros_move_group moveit_configs_utils controller_manager joint_trajectory_controller; do
    [ -d "/opt/ros/humble/share/${pkg}" ] || fail "${pkg} missing. Install with:
  sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers"
done
[ -f "${D1_ARM_DIR}/d1_550_description/urdf/d1_550_description.urdf" ] || \
    fail "D1 description not found in ${D1_ARM_DIR} (set D1_ARM_DIR)"

unset CYCLONEDDS_URI
export ROS_DOMAIN_ID="${ARM_ROS_DOMAIN_ID}" ROS_LOCALHOST_ONLY=1
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd "${REPO_ROOT}"
if [ "${BUILD}" = "always" ] || [ ! -d install/d1_moveit_config ] || [ ! -d install/d1_550_description ]; then
    echo "[build] d1_550_description (from ${D1_ARM_DIR}) + d1_moveit_config"
    colcon build --symlink-install \
        --base-paths src/d1_moveit_config "${D1_ARM_DIR}/d1_550_description" \
        --packages-select d1_550_description d1_moveit_config
fi
# shellcheck disable=SC1091
source install/setup.bash

if [ "${INSPECT}" = "true" ]; then
    ros2 launch d1_moveit_config demo.launch.py use_rviz:="${RVIZ}" &
    LAUNCH_PID=$!
    trap 'kill -INT ${LAUNCH_PID} 2>/dev/null; wait ${LAUNCH_PID} 2>/dev/null' EXIT INT TERM
    echo "[inspect] waiting for MoveIt..."
    sleep 8
    ros2 run d1_moveit_config inspect_plant.py
    echo "[inspect] done. Ctrl+C to close RViz."
    wait ${LAUNCH_PID}
else
    exec ros2 launch d1_moveit_config demo.launch.py use_rviz:="${RVIZ}"
fi
