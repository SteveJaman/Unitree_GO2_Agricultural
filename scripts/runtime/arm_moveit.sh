#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# arm_moveit.sh
#
# MoveIt 2 for the D1-550 arm on the Go2, on mock hardware, plus the
# plant-inspection demo. --real drives the physical arm. Runs on the host, no Docker.
#
# Usage:
#   ./scripts/runtime/arm_moveit.sh              # build if needed, RViz + MoveIt
#   ./scripts/runtime/arm_moveit.sh --inspect    # same, then run the plant inspection
#   ARM_JOINT_SIGNS="1 1 1 1 1 1" ./scripts/runtime/arm_moveit.sh --real
#                                                # the inspection on the REAL arm: MOVES IT
#   ... --real --robot 192.168.1.7               # arm plugged into the Go2 (or ARM_ROBOT)
#   ... --real --camera                          # camera on the wrist flange, not the gripper
#   ./scripts/runtime/arm_moveit.sh --build      # force a rebuild first
#   ./scripts/runtime/arm_moveit.sh --no-rviz    # headless
#
# Needs: sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
# The D1 description is built from D1_ARM_DIR (default ~/files/unitree-D1-550-Robot-ARM).
#
# Safety: ROS_DOMAIN_ID 78 + localhost-only DDS, so ROS traffic never reaches the
# robot. Without --real nothing touches the arm. --real needs ARM_JOINT_SIGNS,
# from the joint sign test in docs/hardware/d1arm.md, and stops if the bridge
# to the arm dies: there is no mock arm behind it.
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/../.." &> /dev/null && pwd )"
D1_ARM_DIR="${D1_ARM_DIR:-$HOME/files/unitree-D1-550-Robot-ARM}"
ARM_ROS_DOMAIN_ID="${ARM_ROS_DOMAIN_ID:-78}"

BUILD="auto"; INSPECT="false"; REAL="false"; RVIZ="true"; END_EFFECTOR="gripper"
while [ $# -gt 0 ]; do
    case "$1" in
        --build)   BUILD="always" ;;
        --inspect) INSPECT="true" ;;
        --real)    REAL="true"; INSPECT="true" ;;
        --no-rviz) RVIZ="false" ;;
        --camera)  END_EFFECTOR="camera" ;;
        --robot)   export ARM_ROBOT="${2:?--robot needs the Go2 IP}"; shift ;;
        -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
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
if [ "${REAL}" = "true" ] && [ -z "${ARM_JOINT_SIGNS:-}" ]; then
    fail "--real needs ARM_JOINT_SIGNS, six values of 1 or -1 for Joint1..Joint6.
  The D1-550 model and Unitree's disagree on Joint1 and Joint6, so run the joint
  sign test first: docs/hardware/d1arm.md, \"Joint signs\". Example:
  ARM_JOINT_SIGNS=\"1 1 1 1 1 1\" $0 --real"
fi

unset CYCLONEDDS_URI
export ROS_DOMAIN_ID="${ARM_ROS_DOMAIN_ID}" ROS_LOCALHOST_ONLY=1
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd "${REPO_ROOT}"
if [ "${BUILD}" = "always" ] || [ ! -d install/d1_550_description ] ||
   [ ! -e install/d1_moveit_config/share/d1_moveit_config/config/moveit_controllers_real.yaml ]; then
    echo "[build] d1_550_description (from ${D1_ARM_DIR}) + d1_moveit_config"
    colcon build --symlink-install \
        --base-paths src/d1_moveit_config "${D1_ARM_DIR}/d1_550_description" \
        --packages-select d1_550_description d1_moveit_config
fi
# shellcheck disable=SC1091
source install/setup.bash

LAUNCH_ARGS=(use_rviz:="${RVIZ}" end_effector:="${END_EFFECTOR}")
if [ "${INSPECT}" = "true" ]; then
    # --real: velocity scaled to 20 % and the MoveIt→D1 bridge replaces mock hardware
    VELOCITY_SCALE="0.3"
    LAUNCH_PID=""; BRIDGE_PID=""
    trap '[ -n "${LAUNCH_PID}" ] && kill -INT ${LAUNCH_PID} 2>/dev/null; [ -n "${BRIDGE_PID}" ] && kill -INT ${BRIDGE_PID} 2>/dev/null; wait 2>/dev/null' EXIT INT TERM
    if [ "${REAL}" = "true" ]; then
        VELOCITY_SCALE="0.2"
        LAUNCH_ARGS+=(hardware:=real)
        echo "[real] velocity_scaling=${VELOCITY_SCALE}, signs ${ARM_JOINT_SIGNS}, starting moveit_d1_bridge..."
        python3 "${REPO_ROOT}/scripts/nodes/moveit_d1_bridge.py" &
        BRIDGE_PID=$!
        # MoveIt starts only once the bridge reports the real arm's joints.
        for _ in $(seq 1 30); do
            kill -0 "${BRIDGE_PID}" 2>/dev/null || fail "moveit_d1_bridge stopped: no real arm, nothing was moved"
            timeout 3 ros2 topic echo --once /joint_states >/dev/null 2>&1 && break
        done
        timeout 3 ros2 topic echo --once /joint_states >/dev/null 2>&1 || fail "no joint states from moveit_d1_bridge"
    fi

    ros2 launch d1_moveit_config demo.launch.py "${LAUNCH_ARGS[@]}" &
    LAUNCH_PID=$!
    echo "[inspect] waiting for MoveIt..."
    sleep 8

    INSPECT_ARGS=(--ros-args -p velocity_scaling:="${VELOCITY_SCALE}")
    [ "${END_EFFECTOR}" = "camera" ] && INSPECT_ARGS+=(-p tool_link:=camera_link)
    ros2 run d1_moveit_config inspect_plant.py "${INSPECT_ARGS[@]}"

    [ "${REAL}" = "true" ] && echo "[real] the arm stays enabled and holding; fold it with start_arm.sh fold"
    echo "[inspect] done. Ctrl+C to close RViz."
    wait ${LAUNCH_PID}
else
    exec ros2 launch d1_moveit_config demo.launch.py "${LAUNCH_ARGS[@]}"
fi
