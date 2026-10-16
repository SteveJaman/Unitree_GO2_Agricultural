#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# wifi_mapping.sh
#
# Wrapper for the go2-nav2-wifi 3D mapping stack.
#
# The go2-nav2-wifi repo has two distinct workflows:
#
#   CANONICAL (recommended):
#     Point-LIO runs on the robot's Jetson. Raw LiDAR stays on the robot.
#     Only the map + trajectory are transferred to the laptop after stop.
#     Requires: ./setup-lidar3d.sh --jetson (one-time, on the robot).
#     Launch:   ./mapping.sh --3d
#
#   DIAGNOSTIC (Wi-Fi debugging only):
#     Point-LIO runs on the laptop, consuming topics over Wi-Fi.
#     Useful if the Jetson-side compute is misbehaving.
#     Launch:   ./mapping.sh --3d --laptop
#
# Usage:
#   ./scripts/wifi_mapping.sh                    # canonical 3D mapping
#   ./scripts/wifi_mapping.sh --record           # canonical + input recording
#   ./scripts/wifi_mapping.sh --headless         # canonical, no RViz
#   ./scripts/wifi_mapping.sh --diag             # laptop-side compute
#   ./scripts/wifi_mapping.sh --jetson-setup     # one-time robot-side setup
#   ./scripts/wifi_mapping.sh --check            # verify prerequisites only
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/../.." &> /dev/null && pwd )"

# --- Configuration ---------------------------------------------------------
NAV2_WIFI_DIR="${NAV2_WIFI_DIR:-$HOME/files/go2-nav2-wifi}"
GO2_HOST_IP="${GO2_HOST_IP:-192.168.1.4}"
GO2_ROBOT_IP="${GO2_ROBOT_IP:-192.168.1.9}"
GO2_RELAY_DOMAIN_ID="${GO2_RELAY_DOMAIN_ID:-64}"

# --- Parse args ------------------------------------------------------------
ACTION="map"          # map | jetson-setup | check
LAUNCH_ARGS="--3d"
export GO2_RECORD="${GO2_RECORD:-0}"
export GO2_RVIZ="${GO2_RVIZ:-1}"

while [ $# -gt 0 ]; do
    case "$1" in
        --record)       export GO2_RECORD=1 ;;
        --headless)     export GO2_RVIZ=0 ;;
        --diag)         LAUNCH_ARGS="--3d --laptop" ;;
        --jetson-setup) ACTION="jetson-setup" ;;
        --check)        ACTION="check" ;;
        -h|--help)      sed -n '2,35p' "$0"; exit 0 ;;
        *)              echo "Unknown option: $1" >&2; exit 1 ;;
    esac
    shift
done

# --- Banner ----------------------------------------------------------------
echo "================================================================"
echo " Go2 Nav2 Wi-Fi 3D mapping"
echo "================================================================"
echo "  Action      : ${ACTION}"
echo "  Launch args : ${LAUNCH_ARGS}"
echo "  GO2_RECORD  : ${GO2_RECORD}"
echo "  GO2_RVIZ    : ${GO2_RVIZ}"
echo "  Host IP     : ${GO2_HOST_IP}"
echo "  Robot IP    : ${GO2_ROBOT_IP}"
echo "  Repo        : ${NAV2_WIFI_DIR}"
echo

# --- Prerequisite checks ---------------------------------------------------
if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker not found." >&2
    echo "  Install:  curl -fsSL https://get.docker.com | sudo sh" >&2
    exit 1
fi
if ! docker ps >/dev/null 2>&1; then
    echo "ERROR: cannot talk to Docker daemon." >&2
    echo "  Try:  newgrp docker" >&2
    exit 1
fi
[ -d "${NAV2_WIFI_DIR}" ] || {
    echo "ERROR: ${NAV2_WIFI_DIR} not found." >&2
    echo "  Clone:  git clone https://github.com/dancher00/go2-nav2-wifi.git ${NAV2_WIFI_DIR}" >&2
    exit 1
}
[ -f "${NAV2_WIFI_DIR}/mapping.sh" ] || {
    echo "ERROR: ${NAV2_WIFI_DIR}/mapping.sh not found." >&2
    echo "  Run setup first (see below)." >&2
    exit 1
}
echo "[checks] docker + repo present .................... OK"

# --- Export env ------------------------------------------------------------
export GO2_HOST_IP
export GO2_ROBOT_IP
export GO2_RELAY_DOMAIN_ID

# --- Dispatch --------------------------------------------------------------
cd "${NAV2_WIFI_DIR}"

case "${ACTION}" in
    jetson-setup)
        echo
        echo "[setup] Building robot-side container..."
        echo "        SSH will prompt for the 'unitree' user's password."
        echo
        exec ./setup-lidar3d.sh --jetson
        ;;

    check)
        echo
        echo "All checks passed. Run without --check to launch mapping."
        echo
        echo "Canonical workflow:"
        echo "  1. One-time:  ./scripts/wifi_mapping.sh --jetson-setup"
        echo "  2. Each run:  ./scripts/wifi_mapping.sh"
        echo
        echo "Diagnostic (Wi-Fi debugging):"
        echo "      ./scripts/wifi_mapping.sh --diag"
        ;;
    map)
        # Reminder about robot-side readiness
        echo "================================================================"
        echo " Canonical mode requires the Jetson-side container to be running."
        echo "================================================================"
        echo " If you have not set it up yet:"
        echo "     ./scripts/wifi_mapping.sh --jetson-setup"
        echo
        echo " Start it on the robot (in a separate terminal):"
        echo "     ssh unitree@${GO2_ROBOT_IP}"
        echo "     cd ~/go2-nav2-wifi && ./mapping.sh --3d   # or whatever the repo uses"
        echo
        echo " Then launch here."
        echo "================================================================"
        echo
        echo "Launching: ./mapping.sh ${LAUNCH_ARGS}"
        echo

        cleanup() {
            echo
            echo "[wifi_mapping] Ctrl+C — cleaning up container..."
            if [ -f "${NAV2_WIFI_DIR}/docker/docker-compose.yml" ]; then
                (cd "${NAV2_WIFI_DIR}/docker" && docker compose down 2>/dev/null) || true
            fi
            echo "[wifi_mapping] Done."
        }
        trap cleanup EXIT INT TERM

        exec ./mapping.sh ${LAUNCH_ARGS}
        ;;
esac
