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
#                                                # (a folded arm first goes to stow)
#   ... --real --robot 192.168.1.7               # arm plugged into the Go2 (or ARM_ROBOT)
#   ... --real --camera                          # camera on the wrist flange, not the gripper
#   ./scripts/runtime/arm_moveit.sh --orbit --plan-only   # 180 degree sweep, planned on the mock arm
#   ./scripts/runtime/arm_moveit.sh --orbit      # the same, moving the mock arm
#   ... --real --orbit-test                      # REAL arm: 3 stops around the centre, a first test
#   ... --real --orbit                           # REAL arm: the whole 13-stop sweep
#   ORBIT_ARGS="-p target:=[0.0,0.45,0.20]" ...  # more orbit_scanner parameters
#   ./scripts/runtime/arm_moveit.sh --build      # force a rebuild first
#   ./scripts/runtime/arm_moveit.sh --no-rviz    # headless
#
# Needs: sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
# The D1 description is built from D1_ARM_DIR (default ~/files/unitree-D1-550-Robot-ARM).
#
# Safety: ROS_DOMAIN_ID 78 + localhost-only DDS, so ROS traffic never reaches the
# robot. Without --real nothing touches the arm. --real needs ARM_JOINT_SIGNS,
# from the joint sign test in docs/hardware/d1arm.md, and stops if the bridge
# to the arm dies: there is no mock arm behind it. --plan-only never uses the real arm.
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/../.." &> /dev/null && pwd )"
D1_ARM_DIR="${D1_ARM_DIR:-$HOME/files/unitree-D1-550-Robot-ARM}"
ARM_ROS_DOMAIN_ID="${ARM_ROS_DOMAIN_ID:-78}"

BUILD="auto"; INSPECT="false"; REAL="false"; RVIZ="true"; END_EFFECTOR="gripper"
ORBIT="false"; ORBIT_TEST="false"; PLAN_ONLY="false"
while [ $# -gt 0 ]; do
    case "$1" in
        --build)   BUILD="always" ;;
        --inspect) INSPECT="true" ;;
        --real)    REAL="true"; INSPECT="true" ;;
        --no-rviz) RVIZ="false" ;;
        --camera)  END_EFFECTOR="camera" ;;
        --orbit)   ORBIT="true"; INSPECT="true"; END_EFFECTOR="camera" ;;
        --orbit-test) ORBIT="true"; ORBIT_TEST="true"; INSPECT="true"; END_EFFECTOR="camera" ;;
        --plan-only) PLAN_ONLY="true" ;;
        --robot)   export ARM_ROBOT="${2:?--robot needs the Go2 IP}"; shift ;;
        -h|--help) sed -n '2,31p' "$0"; exit 0 ;;
        *) echo "Unknown option: $1 (see --help)" >&2; exit 2 ;;
    esac
    shift
done

fail() { echo "ERROR: $*" >&2; exit 1; }

[ "${PLAN_ONLY}" = "false" ] || [ "${ORBIT}" = "true" ] || fail "--plan-only goes with --orbit"
[ "${PLAN_ONLY}" = "false" ] || [ "${REAL}" = "false" ] || \
    fail "--plan-only runs on the mock arm: leave out --real"

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
# Every time: inspect_plant.py and orbit_scanner.py are installed as copies, so a
# run would otherwise use old code. Incremental, a few seconds.
[ "${BUILD}" = "always" ] && CLEAN=(--cmake-clean-cache) || CLEAN=()
echo "[build] d1_550_description (from ${D1_ARM_DIR}) + d1_moveit_config + d1_active_perception"
colcon build --symlink-install "${CLEAN[@]}" \
    --base-paths src/d1_moveit_config src/d1_active_perception "${D1_ARM_DIR}/d1_550_description" \
    --packages-select d1_550_description d1_moveit_config d1_active_perception
# shellcheck disable=SC1091
source install/setup.bash

LAUNCH_ARGS=(use_rviz:="${RVIZ}" end_effector:="${END_EFFECTOR}")
if [ "${INSPECT}" = "true" ]; then
    # --real: velocity scaled to 20 % and the MoveIt→D1 bridge replaces mock hardware
    VELOCITY_SCALE="0.3"
    LAUNCH_PID=""; BRIDGE_PID=""
    # The launch has its own process group, so all of it stops: it ignores the
    # SIGINT of a background job, and its nodes outlive it otherwise.
    stop_all() {
        trap - EXIT INT TERM
        [ -z "${BRIDGE_PID}" ] || kill -INT "${BRIDGE_PID}" 2>/dev/null
        if [ -n "${LAUNCH_PID}" ]; then
            kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null
            for _ in 1 2 3 4 5 6 7 8; do
                kill -0 -- "-${LAUNCH_PID}" 2>/dev/null || break
                sleep 0.5
            done
            kill -KILL -- "-${LAUNCH_PID}" 2>/dev/null
        fi
        wait 2>/dev/null
    }
    trap 'stop_all; exit' EXIT INT TERM
    if [ "${REAL}" = "true" ]; then
        VELOCITY_SCALE="0.2"
        LAUNCH_ARGS+=(hardware:=real)
        echo "[real] velocity_scaling=${VELOCITY_SCALE}, signs ${ARM_JOINT_SIGNS}, starting moveit_d1_bridge..."
        [ "${ORBIT}" = "false" ] || echo "[real] the camera model has no gripper: fingers still on the arm are not collision-checked"
        # A folded arm first moves to stow: MoveIt plans nothing from the fold.
        # ARM_BRIDGE_CMD: another bridge, tests/test_moveit_bridge.py --serve in a test.
        read -ra BRIDGE_CMD <<< "${ARM_BRIDGE_CMD:-python3 ${REPO_ROOT}/scripts/nodes/moveit_d1_bridge.py}"
        READY_FILE="$(mktemp -u /tmp/arm_bridge_ready.XXXXXX)"
        ARM_START_STOW=1 ARM_BRIDGE_READY="${READY_FILE}" "${BRIDGE_CMD[@]}" &
        BRIDGE_PID=$!
        # MoveIt starts only once the bridge is up, which is after any move to
        # stow. A file, not ros2 topic echo: that hangs on a broken ros2 daemon.
        DEADLINE=$((SECONDS + 90))
        until [ -e "${READY_FILE}" ]; do
            kill -0 "${BRIDGE_PID}" 2>/dev/null || fail "moveit_d1_bridge stopped: see its message above"
            [ "${SECONDS}" -lt "${DEADLINE}" ] || fail "moveit_d1_bridge not ready in 90 s"
            sleep 0.5
        done
        rm -f "${READY_FILE}"
    fi

    setsid ros2 launch d1_moveit_config demo.launch.py "${LAUNCH_ARGS[@]}" &
    LAUNCH_PID=$!
    echo "[inspect] waiting for MoveIt..."
    sleep 8

    if [ "${ORBIT}" = "true" ]; then
        read -ra EXTRA <<< "${ORBIT_ARGS:-}"
        ORBIT_PARAMS=(--ros-args)
        [ "${PLAN_ONLY}" = "true" ] && ORBIT_PARAMS+=(-p plan_only:=true)
        [ "${ORBIT_TEST}" = "true" ] && ORBIT_PARAMS+=(-p start_deg:=-15.0 -p end_deg:=15.0 -p waypoints:=3)
        ros2 run d1_active_perception orbit_scanner "${ORBIT_PARAMS[@]}" "${EXTRA[@]}"
    else
        INSPECT_ARGS=(--ros-args -p velocity_scaling:="${VELOCITY_SCALE}")
        [ "${END_EFFECTOR}" = "camera" ] && INSPECT_ARGS+=(-p tool_link:=camera_link)
        ros2 run d1_moveit_config inspect_plant.py "${INSPECT_ARGS[@]}"
    fi

    [ "${REAL}" = "true" ] && echo "[real] the arm stays enabled and holding; fold it with start_arm.sh fold"
    echo "[inspect] done. Ctrl+C to close RViz."
    wait ${LAUNCH_PID}
else
    exec ros2 launch d1_moveit_config demo.launch.py "${LAUNCH_ARGS[@]}"
fi
