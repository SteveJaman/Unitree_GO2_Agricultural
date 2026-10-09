#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# verify_topics.sh
#
# Checks that every expected ROS 2 topic exists with the correct type,
# lists ALL visible topics with a short description of each, and reports
# what is missing and what to do about it.
#
# Usage:
#   ./scripts/verify_topics.sh              # real robot (Ethernet)
#   ./scripts/verify_topics.sh --dog        # just the dog, no autonomy stack
#   ./scripts/verify_topics.sh --cmu        # after CMU stack launches
#   ./scripts/verify_topics.sh --sim        # mock sim running
#   ./scripts/verify_topics.sh --webrtc     # WebRTC fallback
#   ./scripts/verify_topics.sh --ref        # print the full topic reference
# ---------------------------------------------------------------------------
set -eo pipefail

MODE="ethernet"
case "${1:-}" in
    --sim)     MODE="simulation" ;;
    --webrtc)  MODE="webrtc" ;;
    --dog)     MODE="dog" ;;
    --cmu)     MODE="cmu" ;;
    --ref)     MODE="reference" ;;
    --ethernet|"") MODE="ethernet" ;;
    *) echo "Unknown mode: $1"; exit 1 ;;
esac

WS_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." &> /dev/null && pwd )"

# ===========================================================================
# Topic descriptions
# ===========================================================================
describe_topic() {
    case "$1" in
        # --- Go2 native LiDAR / IMU / odometry ---
        /utlidar/cloud)               echo "Raw LiDAR point cloud (sensor_msgs/PointCloud2)" ;;
        /utlidar/cloud_deskewed)      echo "Motion-compensated LiDAR cloud (preferred for mapping)" ;;
        /utlidar/cloud_base)          echo "LiDAR cloud in the robot's base frame" ;;
        /utlidar/imu)                 echo "IMU data from the Go2 (~250 Hz)" ;;
        /utlidar/lidar_state)         echo "LiDAR status / health information" ;;
        /utlidar/range_info)          echo "LiDAR range information" ;;
        /utlidar/robot_odom)          echo "Robot odometry in the lidar/body frame (~50 Hz)" ;;
        /utlidar/robot_pose)          echo "Robot pose in the map frame" ;;
        /utlidar/switch)              echo "LiDAR on/off control" ;;
        /utlidar/grid_map)            echo "Onboard 2D grid map" ;;
        /utlidar/height_map)          echo "Onboard 2D height map" ;;
        /utlidar/voxel_map)           echo "Onboard 3D voxel map" ;;

        # --- Point-LIO output (CMU autonomy stack) ---
        /registered_scan)             echo "Point-LIO: aligned cloud in map frame (~10 Hz) — preferred" ;;
        /cloud_registered)            echo "Point-LIO: alternate name for aligned cloud" ;;
        /cloud_registered_body)       echo "Point-LIO: aligned cloud in body frame" ;;
        /cloud_effected)              echo "Point-LIO: affected region marker" ;;
        /state_estimation)            echo "Point-LIO: 6-DOF odometry (~10 Hz)" ;;
        /lio_sam_ros2/mapping/odometry) echo "Point-LIO: LIO-SAM compat odometry" ;;
        /Laser_map)                   echo "CMU terrain: laser map" ;;
        /terrain_map)                 echo "CMU terrain: local terrain" ;;
        /terrain_map_ext)             echo "CMU terrain: extended terrain" ;;
        /overall_map)                 echo "CMU terrain: accumulated map" ;;
        /path)                        echo "CMU planner: current path" ;;
        /free_paths)                  echo "CMU planner: free paths" ;;
        /way_point)                   echo "CMU planner: waypoint" ;;
        /speed)                       echo "CMU planner: speed command" ;;
        /stop)                        echo "CMU planner: stop command" ;;
        /cmd_vel)                     echo "Velocity command (TwistStamped on Ethernet)" ;;
        /cmd_vel_stamped)             echo "Velocity command (alternate name)" ;;

        # --- Integration package outputs ---
        /scan)                        echo "2D LaserScan from pointcloud_to_scan.py" ;;
        /map/occupancy)               echo "2D log-odds grid from map_node.py (TRANSIENT_LOCAL)" ;;
        /map/points)                  echo "Accumulated 3D cloud from map_node.py" ;;
        /map)                         echo "2D grid from slam_toolbox or map_server" ;;

        # --- Robot state ---
        /lowstate)                    echo "Low-level state: motors, IMU, battery (~500 Hz)" ;;
        /lowcmd)                      echo "Low-level motor command channel" ;;
        /sportmodestate)              echo "Sport mode state: pos, vel, gait, body height" ;;
        /lf/lowstate)                 echo "Low-frequency lowstate" ;;
        /lf/sportmodestate)           echo "Low-frequency sport mode state" ;;
        /lf/battery_alarm)            echo "Battery alarm / warning" ;;
        /wirelesscontroller)          echo "Wireless controller (joystick) state" ;;
        /wirelesscontroller_unprocessed) echo "Raw wireless controller state" ;;

        # --- Sport API ---
        /api/sport/request)           echo "Sport API command channel (Move, StandUp, etc.)" ;;
        /api/sport/response)          echo "Sport API response from robot" ;;
        /api/sport_lease/request)     echo "Sport mode lease request" ;;
        /api/sport_lease/response)    echo "Sport mode lease response" ;;
        /api/motion_switcher/request) echo "Motion switcher: select/release motion service" ;;
        /api/obstacles_avoid/request) echo "Obstacle-avoid service request" ;;

        # --- Camera / video ---
        /camera/image_raw)            echo "Camera image (WebRTC SDK required)" ;;
        /camera/image/raw)            echo "Camera image (real-robot naming, WebRTC required)" ;;
        /camera/image_raw_relayed)    echo "Camera image republished by camera_relay.py" ;;
        /camera/camera_info)          echo "Camera intrinsics" ;;
        /frontvideostream)            echo "Front camera video stream (H.264 over WebRTC)" ;;
        /videohub/inner)              echo "Internal video hub stream" ;;

        # --- ROS 2 standard ---
        /rosout)                      echo "ROS 2 log output" ;;
        /parameter_events)            echo "ROS 2 parameter change events" ;;
        /tf)                          echo "Transform tree (dynamic)" ;;
        /tf_static)                   echo "Transform tree (static)" ;;

        # --- WebRTC mode ---
        /point_cloud2)                echo "WebRTC SDK cloud topic (only in WebRTC mode)" ;;
        /imu/data)                    echo "WebRTC SDK IMU topic (only in WebRTC mode)" ;;

        *) echo "(no description on file)" ;;
    esac
}

# ===========================================================================
# Reference mode
# ===========================================================================
if [ "$MODE" = "reference" ]; then
    echo "================================================================"
    echo " Unitree Go2 EDU — Topic Reference"
    echo "================================================================"
    echo
    echo "--- Go2 native sensors ---"
    for t in /utlidar/cloud /utlidar/cloud_deskewed /utlidar/cloud_base \
             /utlidar/imu /utlidar/robot_odom /utlidar/robot_pose \
             /utlidar/grid_map /utlidar/height_map /utlidar/voxel_map; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    echo "--- Point-LIO output (CMU stack) ---"
    for t in /registered_scan /cloud_registered /cloud_registered_body \
             /state_estimation /Laser_map /terrain_map /path /cmd_vel; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    echo "--- Integration package ---"
    for t in /scan /map /map/occupancy /map/points; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    echo "--- Robot state ---"
    for t in /lowstate /lowcmd /sportmodestate /wirelesscontroller; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    echo "--- Sport API ---"
    for t in /api/sport/request /api/sport/response \
             /api/sport_lease/request /api/sport_lease/response; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    exit 0
fi

# ===========================================================================
# Normal verify mode
# ===========================================================================
set +u
source /opt/ros/humble/setup.bash
[ -f "${WS_ROOT}/install/setup.bash" ] && source "${WS_ROOT}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

case "$MODE" in
    ethernet|simulation|dog|cmu)
        export CYCLONEDDS_URI="file://${WS_ROOT}/config/cyclonedds_ethernet.xml" ;;
    webrtc)
        export CYCLONEDDS_URI="file://${WS_ROOT}/config/cyclonedds_wireless.xml" ;;
esac

echo "================================================================"
echo " Verifying topics in mode: ${MODE}"
echo " CYCLONEDDS_URI: ${CYCLONEDDS_URI}"
echo "================================================================"
echo

# --- Expected topics per mode ---
case "$MODE" in
    ethernet)
        # Full pipeline — robot + CMU stack + integration package
        EXPECTED=(
            "/utlidar/cloud_deskewed:sensor_msgs/msg/PointCloud2"
            "/utlidar/imu:sensor_msgs/msg/Imu"
            "/utlidar/robot_pose:geometry_msgs/msg/PoseStamped"
            "/registered_scan:sensor_msgs/msg/PointCloud2"
            "/state_estimation:nav_msgs/msg/Odometry"
            "/map/occupancy:nav_msgs/msg/OccupancyGrid"
            "/map/points:sensor_msgs/msg/PointCloud2"
        )
        ;;
    cmu)
        # Just the CMU autonomy stack running (before integration package)
        EXPECTED=(
            "/registered_scan:sensor_msgs/msg/PointCloud2"
            "/state_estimation:nav_msgs/msg/Odometry"
            "/cloud_registered_body:sensor_msgs/msg/PointCloud2"
        )
        ;;
    dog)
        # Robot powered on, no autonomy stack
        EXPECTED=(
            "/utlidar/cloud_deskewed:sensor_msgs/msg/PointCloud2"
            "/utlidar/imu:sensor_msgs/msg/Imu"
            "/utlidar/robot_pose:geometry_msgs/msg/PoseStamped"
            "/lowstate:unitree_go/msg/LowState"
            "/sportmodestate:unitree_go/msg/SportModeState"
        )
        ;;
    simulation)
        # Mock sim running
        EXPECTED=(
            "/registered_scan:sensor_msgs/msg/PointCloud2"
            "/state_estimation:nav_msgs/msg/Odometry"
            "/map/occupancy:nav_msgs/msg/OccupancyGrid"
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

ALL_TOPICS="$(ros2 topic list 2>/dev/null || true)"

# --- Check each expected topic ---
PASS=0
FAIL=0
MISSING_LIST=()

echo "--- Expected topics ---"
echo

for entry in "${EXPECTED[@]}"; do
    topic="${entry%%:*}"
    expected_type="${entry##*:}"

    if ! grep -qxF "${topic}" <<< "${ALL_TOPICS}"; then
        echo "  MISSING  ${topic}"
        echo "           $(describe_topic "${topic}")"
        MISSING_LIST+=("${topic}")
        FAIL=$((FAIL + 1))
        continue
    fi

    actual_type="$(ros2 topic info "${topic}" 2>/dev/null \
        | grep '^Type:' | awk '{print $2}' || true)"

    if [ "${actual_type}" = "${expected_type}" ]; then
        echo "  OK       ${topic}  [${actual_type}]"
        echo "           $(describe_topic "${topic}")"
        PASS=$((PASS + 1))
    else
        echo "  TYPE ERR ${topic}"
        echo "           $(describe_topic "${topic}")"
        echo "           expected: ${expected_type}"
        echo "           actual:   ${actual_type:-<unknown>}"
        FAIL=$((FAIL + 1))
    fi
done

echo
echo "---------------------------------------------------------------"
echo "  Expected result: ${PASS} OK,  ${FAIL} failed"
echo "---------------------------------------------------------------"
echo

# --- List every visible topic with description ---
echo "--- All topics visible on this machine ---"
echo

if [ -z "${ALL_TOPICS}" ]; then
    echo "  (none — is the robot powered on and connected?)"
else
    EXPECTED_NAMES="$(for entry in "${EXPECTED[@]}"; do echo "${entry%%:*}"; done)"

    echo "  [expected + present]"
    found_expected=0
    while IFS= read -r topic; do
        [ -z "${topic}" ] && continue
        if grep -qxF "${topic}" <<< "${EXPECTED_NAMES}"; then
            printf "    %-38s %s\n" "${topic}" "$(describe_topic "${topic}")"
            found_expected=$((found_expected + 1))
        fi
    done <<< "${ALL_TOPICS}"
    [ "${found_expected}" -eq 0 ] && echo "    (none)"
    echo

    echo "  [expected but missing]"
    found_missing=0
    for entry in "${EXPECTED[@]}"; do
        topic="${entry%%:*}"
        if ! grep -qxF "${topic}" <<< "${ALL_TOPICS}"; then
            printf "    %-38s %s\n" "${topic}" "$(describe_topic "${topic}")"
            found_missing=$((found_missing + 1))
        fi
    done
    [ "${found_missing}" -eq 0 ] && echo "    (none)"
    echo

    echo "  [present but not expected for this mode]"
    extra=0
    while IFS= read -r topic; do
        [ -z "${topic}" ] && continue
        if ! grep -qxF "${topic}" <<< "${EXPECTED_NAMES}"; then
            printf "    %-38s %s\n" "${topic}" "$(describe_topic "${topic}")"
            extra=$((extra + 1))
        fi
    done <<< "${ALL_TOPICS}"
    [ "${extra}" -eq 0 ] && echo "    (none)"
fi

echo

# --- QoS check on /map/occupancy ---
if grep -qxF "/map/occupancy" <<< "${ALL_TOPICS}"; then
    echo "--- /map/occupancy QoS (should be TRANSIENT_LOCAL) ---"
    ros2 topic info /map/occupancy -v 2>/dev/null \
        | grep -A 1 "Durability" || true
    echo
fi

# --- TF frame check ---
if grep -qxF "/tf" <<< "${ALL_TOPICS}"; then
    echo "--- TF publishers ---"
    ros2 topic info /tf 2>/dev/null | grep -E "Publisher count|Subscription count" || true
    echo "  (Point-LIO publishes TF with BEST_EFFORT — RViz needs Best Effort to see it)"
    echo
fi

# --- Guidance on missing topics ---
if [ "${FAIL}" -gt 0 ] && [ "${#MISSING_LIST[@]}" -gt 0 ]; then
    echo "--- What to do about missing topics ---"
    for topic in "${MISSING_LIST[@]}"; do
        case "${topic}" in
            /utlidar/cloud|/utlidar/cloud_deskewed|/utlidar/imu|/utlidar/robot_pose)
                echo "  ${topic}:"
                echo "      Robot not publishing. Check robot power, DDS, and CYCLONEDDS_URI."
                ;;
            /registered_scan|/state_estimation|/cloud_registered_body)
                echo "  ${topic}:"
                echo "      Point-LIO not running or stuck."
                echo "      Launch:  ./scripts/live_mapping.sh"
                ;;
            /map/occupancy|/map/points)
                echo "  ${topic}:"
                echo "      map_node.py not running."
                echo "      Launch:  ./scripts/live_mapping.sh"
                echo "      Or:      ros2 run go2_integration_pkg map_node.py"
                ;;
            /cmd_vel)
                echo "  ${topic}:"
                echo "      No active goal. Appears only when a path is being followed."
                ;;
            /scan)
                echo "  ${topic}:"
                echo "      pointcloud_to_scan.py not running."
                ;;
            *)
                echo "  ${topic}: not found. Check whether the relevant node is running."
                ;;
        esac
    done
    echo
fi

echo "Tip: run './scripts/verify_topics.sh --ref' for the full topic reference."
echo "Tip: './scripts/verify_topics.sh --dog' checks just the robot (no autonomy stack)."
echo
