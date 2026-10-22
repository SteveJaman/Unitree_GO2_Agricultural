#!/usr/bin/env bash
# tune_costmap.sh — change inflation and padding at runtime, no restart.
#
# Usage:
#   ./tune_costmap.sh 0.65           # set inflation radius to 0.65 (recommended)
#   ./tune_costmap.sh 0.65 0.05      # inflation + footprint padding
#   ./tune_costmap.sh --show         # print current values
#   ./tune_costmap.sh --safe         # restore the safe defaults (0.70)
#   ./tune_costmap.sh --wide         # large margin for crowded spaces (0.85)
#   ./tune_costmap.sh --tight        # aggressive, small spaces (0.45)
#
# Requires: a running mapping.sh --3d --nav session.
set -eo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"

# Costmap node names (both local and global)
GLOBAL_NS="/global_costmap/global_costmap"
LOCAL_NS="/local_costmap/local_costmap"

run_ros2() {
    docker exec -u root "$CONTAINER_NAME" bash -lc "
        source /opt/ros/humble/setup.bash 2>/dev/null
        source /ws/install/setup.bash 2>/dev/null
        $1
    "
}

# Check if a node is actually running before trying to set params
check_node() {
    if ! run_ros2 "ros2 node list" 2>/dev/null | grep -q "$1"; then
        echo "  [!] WARNING: Node $1 is not running. Is Nav2 active?"
        return 1
    fi
    return 0
}

set_param() {
    local node="$1"; local name="$2"; local value="$3"
    # Use -w 2 (wait 2 seconds) to ensure the parameter is set
    if run_ros2 "ros2 param set $node $name $value" >/dev/null 2>&1; then
        echo "  ✅ $node $name = $value"
    else
        echo "  ❌ FAILED: $node $name = $value (node might be down)"
    fi
}

show_params() {
    echo "=== Current costmap parameters ==="
    if ! check_node "$GLOBAL_NS"; then return; fi
    for node in "$GLOBAL_NS" "$LOCAL_NS"; do
        for param in inflation_radius cost_scaling_factor footprint_padding; do
            val=$(run_ros2 "ros2 param get $node $param 2>/dev/null" | grep -oP 'value is.*' | head -1 || true)
            if [ -z "$val" ]; then
                printf "  %-35s %-20s %s\n" "$node" "$param" "NOT FOUND"
            else
                printf "  %-35s %-20s %s\n" "$node" "$param" "$val"
            fi
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
        INFLATION=0.70
        PADDING=0.05
        echo "[tune] Applying SAFE defaults (prevents collisions)"
        ;;
    --wide)
        INFLATION=0.85
        PADDING=0.05
        echo "[tune] Applying WIDE values (maximum clearance)"
        ;;
    --tight)
        INFLATION=0.45
        PADDING=0.02
        echo "[tune] Applying TIGHT values (small spaces, watch out!)"
        ;;
    "")
        echo "Usage: $0 INFLATION [PADDING]"
        echo "       $0 --show | --safe | --wide | --tight"
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

# Optional: also increase the cost scaling factor so the gradient is smoother
echo "[tune] Setting cost_scaling_factor=3.0"
set_param "$GLOBAL_NS" cost_scaling_factor 3.0
set_param "$LOCAL_NS"  cost_scaling_factor 3.0

echo
echo "[tune] Done. Changes take effect immediately on the next costmap update."
echo "[tune] Verify in RViz: the red/blue inflation zones should expand."
echo
echo "[tune] These values are runtime-only. To persist them, edit"
echo "[tune] go2_nav2/config/lidar3d_planner.yaml."
