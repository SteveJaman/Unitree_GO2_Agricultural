#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# setup_env.sh
#
# Source this to prepare a shell for the Unitree_GO2_Agricultural workspace.
#
# Usage:
#   source ./scripts/setup_env.sh                # default: ethernet
#   source ./scripts/setup_env.sh wireless       # use cyclonedds_wireless.xml
#   source ./scripts/setup_env.sh --no-dds       # skip CYCLONEDDS_URI
#
# After sourcing, the current shell has:
#   - ROS 2 Humble environment
#   - CMU autonomy_stack_go2 environment (if present)
#   - This workspace's install/setup.bash
#   - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   - CYCLONEDDS_URI pointing at the requested config
#   - ROS_DOMAIN_ID=0 (override with ROS_DOMAIN_ID=... before sourcing)
# ---------------------------------------------------------------------------

# Must be sourced, not executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "ERROR: this script must be sourced, not executed." >&2
    echo "       Run:  source ${BASH_SOURCE[0]}" >&2
    exit 1
fi

# ---- Locate the workspace root -------------------------------------------
_SETUP_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${_SETUP_SCRIPT_DIR}/.." &> /dev/null && pwd )"
AUTONOMY_WS="${AUTONOMY_WS:-$HOME/files/autonomy_stack_go2}"

# ---- Parse argument: ethernet | wireless | --no-dds ----------------------
NET_MODE="ethernet"
case "${1:-}" in
    ""|ethernet)    NET_MODE="ethernet" ;;
    wireless)       NET_MODE="wireless" ;;
    --no-dds)       NET_MODE="none" ;;
    *)
        echo "setup_env.sh: unknown mode '$1' (use ethernet|wireless|--no-dds)" >&2
        return 1
        ;;
esac

# ---- Source ROS 2 (set -u safe) ------------------------------------------
set +u
source /opt/ros/humble/setup.bash

# ---- Source the CMU autonomy stack, if installed -------------------------
if [ -f "${AUTONOMY_WS}/install/setup.bash" ]; then
    source "${AUTONOMY_WS}/install/setup.bash"
else
    echo "setup_env.sh: note — ${AUTONOMY_WS}/install/setup.bash not found"
    echo "              CMU autonomy stack will not be available."
fi

# ---- Source this workspace ----------------------------------------------
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
    source "${WS_ROOT}/install/setup.bash"
else
    echo "setup_env.sh: WARN — ${WS_ROOT}/install/setup.bash not found."
    echo "              Run: cd ${WS_ROOT} && colcon build --symlink-install"
fi

# ---- DDS configuration ---------------------------------------------------
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

case "${NET_MODE}" in
    ethernet)   export CYCLONEDDS_URI="file://${WS_ROOT}/config/cyclonedds_ethernet.xml" ;;
    wireless)   export CYCLONEDDS_URI="file://${WS_ROOT}/config/cyclonedds_wireless.xml" ;;
    none)       unset CYCLONEDDS_URI ;;
esac
set -u

# ---- Report --------------------------------------------------------------
echo "setup_env.sh: workspace ready"
echo "  WS_ROOT        = ${WS_ROOT}"
echo "  RMW            = ${RMW_IMPLEMENTATION}"
echo "  ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID}"
echo "  NET_MODE       = ${NET_MODE}"
[ -n "${CYCLONEDDS_URI:-}" ] && echo "  CYCLONEDDS_URI = ${CYCLONEDDS_URI}"

unset _SETUP_SCRIPT_DIR
