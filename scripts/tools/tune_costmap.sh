#!/usr/bin/env bash
# tune_costmap.sh — change inflation and padding at runtime, no restart.
#
# Usage:
#   ./tune_costmap.sh 0.35           # set inflation radius to 0.35
#   ./tune_costmap.sh 0.35 0.02      # inflation + footprint padding
#   ./tune_costmap.sh --show         # print current values
#   ./tune_costmap.sh --safe         # restore the safe defaults
#   ./tune_costmap.sh --tight        # aggressive, small spaces
#
# Requires: a running mapping.sh --3d --nav session.
set -eo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"

# Costmap node names (both local and global)
GLOBAL_NS="/global_costmap/global_costmap"
LOCAL_NS="/local_costmap/local_costmap"

run_ros2() {
    docker exec -u root "$CONTAINER_NAME" bash -lc "
        source /opt/ros/humble/setup.bash
        source /ws/install/setup.bash 2>/dev/null
        $1
    "
}

set_param() {
    local node="$1"; local name="$2"; local value="$3"
    run_ros2 "ros2 param set $node $name $value" >/dev/null 2>&1 && \
        echo "  $node $name = $value" || \
        echo "  FAILED: $node $name = $value"
}

show_params() {
    echo "=== Current costmap parameters ==="
    for node in "$GLOBAL_NS" "$LOCAL_NS"; do
        for param in inflation_radius cost_scaling_factor footprint_padding; do
            val=$(run_ros2 "ros2 param get $node $param 2>/dev/null" | grep -oP 'value is.*' | head -1)
            printf "  %-35s %-20s %s\n" "$node" "$param" "$val"
        done
    done
}

# --- Parse arguments ---
case "${1:-}" in
    --show)
        show_params
        exit 0
        ;;
    --safe)
        INFLATION=0.35
        PADDING=0.02
        echo "[tune] Applying SAFE defaults"
        ;;
    --tight)
        INFLATION=0.28
        PADDING=0.01
        echo "[tune] Applying TIGHT values (small spaces, tighter corridors)"
        ;;
    "")
        echo "Usage: $0 INFLATION [PADDING]"
        echo "       $0 --show | --safe | --tight"
        exit 1
        ;;
    *)
        INFLATION="$1"
        PADDING="${2:-}"
        ;;
esac

echo "[tune] Setting inflation_radius=$INFLATION"

set_param "$GLOBAL_NS" inflation_radius "$INFLATION"
set_param "$LOCAL_NS"  inflation_radius "$INFLATION"

if [ -n "$PADDING" ]; then
    echo "[tune] Setting footprint_padding=$PADDING"
    set_param "$GLOBAL_NS" footprint_padding "$PADDING"
    set_param "$LOCAL_NS"  footprint_padding "$PADDING"
fi

echo
echo "[tune] Done. Changes take effect on next costmap update."
echo "[tune] Verify in RViz: obstacle zones should resize immediately."
echo
echo "[tune] These values are runtime-only. To persist them across sessions,"
echo "[tune] edit go2_nav2/config/lidar3d_planner.yaml and the launch override."