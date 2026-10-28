#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# sim_explore.sh
#
# Test the frontier explorer against the real go2-nav2-wifi Nav2 config in
# the CMU autonomy_stack_go2 Unity simulator. Runs on the host (no Docker,
# no robot).
#
#   CMU Unity sim ─► go2_sim_bridge ─► go2_nav2 planner/controller ─► frontier_explorer.py
#
# Usage:
#   ./scripts/runtime/sim_explore.sh                 # build if needed, launch, click START in RViz
#   ./scripts/runtime/sim_explore.sh --autostart     # start exploring without the button
#   ./scripts/runtime/sim_explore.sh --build         # force a rebuild first
#   ./scripts/runtime/sim_explore.sh --no-rviz       # headless (Unity window still opens)
#   ./scripts/runtime/sim_explore.sh --no-unity      # Unity already running elsewhere
#   ./scripts/runtime/sim_explore.sh --check         # verify prerequisites only
#
# Safety: the sim runs on its own ROS domain with localhost-only DDS, so a
# robot on the same network can never receive its /cmd_vel.
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/../.." &> /dev/null && pwd )"

NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"
CMU_STACK_DIR="${CMU_STACK_DIR:-$HOME/files/autonomy_stack_go2}"
UNITY_BIN="${CMU_STACK_DIR}/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64"
SIM_ROS_DOMAIN_ID="${SIM_ROS_DOMAIN_ID:-77}"
SIM_PACKAGES=(go2_description go2_nav2 go2_explore_panel go2_sim_bridge)

BUILD="auto"
ACTION="run"
LAUNCH_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --build)     BUILD="always" ;;
        --autostart) LAUNCH_ARGS+=("autostart:=true") ;;
        --no-rviz)   LAUNCH_ARGS+=("rviz:=false") ;;
        --no-unity)  LAUNCH_ARGS+=("launch_unity:=false") ;;
        --check)     ACTION="check" ;;
        -h|--help)   sed -n '2,22p' "$0"; exit 0 ;;
        *) echo "Unknown option: $1 (see --help)" >&2; exit 2 ;;
    esac
    shift
done

fail() { echo "ERROR: $*" >&2; exit 1; }

# --- Prerequisites ---------------------------------------------------------
[ -f /opt/ros/humble/setup.bash ] || fail "ROS 2 Humble not found in /opt/ros/humble"
[ -d "${NAV2_WIFI_DIR}/go2_nav2" ] || fail "go2-nav2-wifi not found at ${NAV2_WIFI_DIR} (set NAV2_WIFI_DIR)"
[ -f "${CMU_STACK_DIR}/install/setup.bash" ] || \
    fail "autonomy_stack_go2 not built at ${CMU_STACK_DIR} (cd there; colcon build --symlink-install)"
[ -x "${UNITY_BIN}" ] || \
    fail "Unity environment missing: ${UNITY_BIN} (see autonomy_stack_go2/README.md 'Simulation Setup')"
[ -d /opt/ros/humble/share/nav2_smac_planner ] || \
    fail "Nav2 not installed on host: sudo apt install ros-humble-navigation2"
echo "[checks] ROS, go2-nav2-wifi, autonomy_stack_go2, Unity env, Nav2 ..... OK"

# Keep the sim off the robot's DDS network. Unset the Wi-Fi CycloneDDS
# profile in case a go2 bashrc snippet exported it.
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID="${SIM_ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY=1
echo "[checks] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, ROS_LOCALHOST_ONLY=1"

# setup.bash scripts reference unset variables, so do not run them under -u.
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${CMU_STACK_DIR}/install/setup.bash"

# --- Build -----------------------------------------------------------------
needs_build() {
    [ "${BUILD}" = "always" ] && return 0
    for pkg in "${SIM_PACKAGES[@]}"; do
        [ -d "${REPO_ROOT}/install/${pkg}" ] || return 0
    done
    return 1
}

if needs_build; then
    echo "[build] colcon build ${SIM_PACKAGES[*]}"
    (
        cd "${REPO_ROOT}"
        # go2_nav2 and go2_description are built from go2-nav2-wifi into this
        # repo's install/ so that repo stays untouched (its ws/install is
        # Docker-built with a /ws prefix and is not usable on the host).
        colcon build --symlink-install \
            --base-paths src "${NAV2_WIFI_DIR}/go2_nav2" "${NAV2_WIFI_DIR}/go2_description" \
            --packages-select "${SIM_PACKAGES[@]}"
    )
fi
# shellcheck disable=SC1091
source "${REPO_ROOT}/install/setup.bash"
# go2_nav2's package.xml has an email catkin_pkg rejects ("danya@local"), so
# colcon builds it as a plain Python package and never adds it to
# AMENT_PREFIX_PATH. Register it by hand so ros2 launch can find it.
case ":${AMENT_PREFIX_PATH}:" in
    *":${REPO_ROOT}/install/go2_nav2:"*) ;;
    *) export AMENT_PREFIX_PATH="${REPO_ROOT}/install/go2_nav2:${AMENT_PREFIX_PATH}" ;;
esac
ros2 pkg prefix go2_nav2 >/dev/null 2>&1 || fail "go2_nav2 not found after build"
echo "[checks] workspace built ($(ros2 pkg prefix go2_sim_bridge)) ..... OK"

[ "${ACTION}" = "check" ] && { echo "All checks passed."; exit 0; }

# --- Launch ----------------------------------------------------------------
echo
echo "Launching sim. In RViz: wait for the costmap, then click 'Auto Explore: START'."
echo "Watch:  ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=1 ros2 topic echo /explore/status"
echo
exec ros2 launch go2_sim_bridge sim_explore.launch.py "${LAUNCH_ARGS[@]}"
