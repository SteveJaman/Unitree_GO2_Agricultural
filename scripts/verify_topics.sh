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
#   ./scripts/verify_topics.sh --sim        # Unity simulation
#   ./scripts/verify_topics.sh --webrtc     # WebRTC fallback
#   ./scripts/verify_topics.sh --dog        # just the dog, no autonomy stack
#   ./scripts/verify_topics.sh --ref        # print the full topic reference
# ---------------------------------------------------------------------------
set -eo pipefail

MODE="ethernet"
case "${1:-}" in
    --sim)     MODE="simulation" ;;
    --webrtc)  MODE="webrtc" ;;
    --dog)     MODE="dog" ;;
    --ref)     MODE="reference" ;;
    --ethernet|"") MODE="ethernet" ;;
    *) echo "Unknown mode: $1"; exit 1 ;;
esac

WS_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." &> /dev/null && pwd )"

# ===========================================================================
# Topic descriptions
# ---------------------------------------------------------------------------
# describe_topic <topic>
#
# Prints a short human-readable description of what the topic carries.
# Used both in the "all topics" listing and in the reference output.
# ===========================================================================
describe_topic() {
    case "$1" in
        # --- Go2 native LiDAR / IMU / odometry ---
        /utlidar/cloud)               echo "Raw LiDAR point cloud from the Go2's L1 lidar (sensor_msgs/PointCloud2)" ;;
        /utlidar/cloud_deskewed)      echo "Motion-compensated LiDAR cloud (preferred for mapping)" ;;
        /utlidar/cloud_base)          echo "LiDAR cloud transformed into the robot's base frame" ;;
        /utlidar/imu)                 echo "IMU data from the Go2 (~250 Hz)" ;;
        /utlidar/lidar_state)         echo "LiDAR status / health information" ;;
        /utlidar/range_info)          echo "LiDAR range information (min/max ranges, status)" ;;
        /utlidar/robot_odom)          echo "Robot odometry in the lidar/body frame (~50 Hz)" ;;
        /utlidar/robot_pose)          echo "Robot pose (position + orientation) in the map frame" ;;
        /utlidar/switch)              echo "LiDAR on/off control switch" ;;
        /utlidar/mapping_cmd)         echo "Onboard mapping command interface" ;;
        /utlidar/client_command)      echo "Onboard client command interface" ;;
        /utlidar/server_log)          echo "Onboard lidar server log" ;;
        /utlidar/grid_map)            echo "Onboard 2D grid map" ;;
        /utlidar/height_map)          echo "Onboard 2D height map" ;;
        /utlidar/height_map_array)    echo "Onboard height map array" ;;
        /utlidar/range_map)           echo "Onboard range map" ;;
        /utlidar/voxel_map)           echo "Onboard 3D voxel map" ;;
        /utlidar/voxel_map_compressed) echo "Onboard 3D voxel map (compressed)" ;;
        /utlidar/imu_calibration)     echo "IMU calibration data" ;;

        # --- Onboard SLAM (uslam) ---
        /uslam/frontend/cloud_world_ds) echo "Onboard SLAM: downsampled cloud in world frame" ;;
        /uslam/frontend/odom)         echo "Onboard SLAM: frontend odometry" ;;
        /uslam/localization/cloud_world) echo "Onboard SLAM: localized cloud in world frame" ;;
        /uslam/localization/odom)     echo "Onboard SLAM: localization odometry" ;;
        /uslam/client_command)        echo "Onboard SLAM: client command" ;;
        /uslam/map_file_pub)          echo "Onboard SLAM: map file publisher" ;;
        /uslam/map_file_sub)          echo "Onboard SLAM: map file subscriber" ;;
        /uslam/navigation/global_path) echo "Onboard SLAM: global navigation path" ;;
        /uslam/server_log)            echo "Onboard SLAM: server log" ;;

        # --- Camera / video ---
        /frontvideostream)            echo "Front camera video stream (H.264 over WebRTC)" ;;
        /videohub/inner)              echo "Internal video hub stream" ;;
        /pctoimage_local)             echo "Point cloud projected to image (local)" ;;
        /webrtcreq)                   echo "WebRTC request (signaling / control)" ;;
        /webrtcres)                   echo "WebRTC response (signaling / control)" ;;
        /xfk_webrtcreq)               echo "XFK WebRTC request (alternate signaling)" ;;
        /xfk_webrtcres)               echo "XFK WebRTC response (alternate signaling)" ;;
        /camera/image_raw)            echo "Camera image from WebRTC SDK (only in WebRTC mode)" ;;
        /camera/image_raw_relayed)    echo "Camera image republished by camera_relay.py for RViz" ;;
        /camera/camera_info)          echo "Camera intrinsics (fx, fy, cx, cy, distortion)" ;;

        # --- Sport API (motion control) ---
        /api/sport/request)           echo "Sport API command channel (Move, StandUp, Sit, etc.)" ;;
        /api/sport/response)          echo "Sport API response from robot (status codes)" ;;
        /api/sport_lease/request)     echo "Sport mode lease request (exclusive control)" ;;
        /api/sport_lease/response)    echo "Sport mode lease response" ;;
        /api/motion_switcher/request) echo "Motion switcher: select/release motion service" ;;
        /api/motion_switcher/response) echo "Motion switcher response" ;;
        /api/obstacles_avoid/request) echo "Obstacle-avoid service request" ;;
        /api/obstacles_avoid/response) echo "Obstacle-avoid service response" ;;

        # --- Robot state ---
        /lowstate)                    echo "Low-level state: motor angles, IMU, battery (~500 Hz)" ;;
        /lf/lowstate)                 echo "Low-frequency lowstate" ;;
        /sportmodestate)              echo "Sport mode state: position, velocity, gait, body height" ;;
        /lf/sportmodestate)           echo "Low-frequency sport mode state" ;;
        /lf/battery_alarm)            echo "Battery alarm / warning" ;;
        /multiplestate)               echo "Aggregated robot state" ;;
        /servicestate)                echo "Service state (motion service / developer mode)" ;;
        /servicestateactivate)        echo "Service state activation" ;;
        /selftest)                    echo "Robot self-test status" ;;
        /gnss)                        echo "GNSS / GPS data" ;;

        # --- Remote control ---
        /wirelesscontroller)          echo "Wireless controller (joystick) state" ;;
        /wirelesscontroller_unprocessed) echo "Raw wireless controller state" ;;
        /api/rm_con/request)          echo "Remote control API request" ;;
        /api/rm_con/response)         echo "Remote control API response" ;;
        /uwbstate)                    echo "UWB positioning state" ;;
        /uwbswitch)                   echo "UWB on/off switch" ;;
        /api/uwbswitch/request)       echo "UWB switch API request" ;;
        /api/uwbswitch/response)      echo "UWB switch API response" ;;

        # --- Arm (D1) ---
        /api/arm/request)             echo "D1 arm control API request" ;;
        /api/arm/response)            echo "D1 arm control API response" ;;
        /arm/action/state)            echo "Arm action state" ;;
        /arm_command)                 echo "Arm command channel" ;;
        /arm_Feedback)                echo "Arm feedback" ;;
        /api/programming_actuator/request)  echo "Programming actuator API request" ;;
        /api/programming_actuator/response) echo "Programming actuator API response" ;;

        # --- Audio / voice ---
        /api/audiohub/request)        echo "Audio hub API request" ;;
        /api/audiohub/response)       echo "Audio hub API response" ;;
        /audio_msg)                   echo "Audio message" ;;
        /audiohub/player/state)       echo "Audio player state" ;;
        /audioreceiver)               echo "Audio receiver (incoming audio)" ;;
        /audiosender)                 echo "Audio sender (outgoing audio)" ;;
        /api/voice/request)           echo "Voice API request" ;;
        /api/voice/response)          echo "Voice API response" ;;
        /api/vui/request)             echo "Voice UI API request" ;;
        /api/vui/response)            echo "Voice UI API response" ;;

        # --- Sensors ---
        /api/gas_sensor/request)      echo "Gas sensor API request" ;;
        /api/gas_sensor/response)     echo "Gas sensor API response" ;;
        /gas_sensor)                  echo "Gas sensor data" ;;
        /api/gesture/request)         echo "Gesture recognition API request" ;;
        /api/gesture/response)        echo "Gesture recognition API response" ;;
        /gesture/result)              echo "Gesture recognition result" ;;
        /api/pet/request)             echo "Pet mode API request" ;;
        /api/pet/response)            echo "Pet mode API response" ;;
        /pet/flowfeedback)            echo "Pet mode flow feedback" ;;

        # --- AI / GPT ---
        /api/gpt/request)             echo "GPT/AI API request (cloud AI integration)" ;;
        /api/gpt/response)            echo "GPT/AI API response" ;;
        /gpt_cmd)                     echo "GPT command" ;;
        /gpt_state)                   echo "GPT state" ;;
        /gptflowfeedback)             echo "GPT flow feedback" ;;

        # --- Config / system ---
        /api/config/request)          echo "Config API request" ;;
        /api/config/response)         echo "Config API response" ;;
        /config_change_status)        echo "Config change status" ;;
        /api/robot_state/request)     echo "Robot state API request" ;;
        /api/robot_state/response)    echo "Robot state API response" ;;
        /api/slam_operate/request)    echo "SLAM operate API request" ;;
        /api/slam_operate/response)   echo "SLAM operate API response" ;;
        /slam_info)                   echo "SLAM info" ;;
        /slam_key_info)               echo "SLAM key info" ;;
        /api/bashrunner/request)      echo "Bash runner API request" ;;
        /api/bashrunner/response)     echo "Bash runner API response" ;;
        /api/fourg_agent/request)     echo "4G agent API request" ;;
        /api/fourg_agent/response)    echo "4G agent API response" ;;
        /api/videohub/request)        echo "Video hub API request" ;;
        /api/videohub/response)       echo "Video hub API response" ;;
        /public_network_status)       echo "Public network status" ;;
        /rtc/state)                   echo "RTC (real-time clock) state" ;;
        /rtc_status)                  echo "RTC status" ;;

        # --- SLAM graph (Qt-based visualization) ---
        /qt_add_edge)                 echo "SLAM graph: add edge" ;;
        /qt_add_node)                 echo "SLAM graph: add node" ;;
        /qt_command)                  echo "SLAM graph: command" ;;
        /qt_notice)                   echo "SLAM graph: notice" ;;
        /query_result_edge)           echo "SLAM graph: query result edge" ;;
        /query_result_node)           echo "SLAM graph: query result node" ;;

        # --- CMU autonomy stack (internal to VM) ---
        /registered_scan)             echo "Point-LIO output: registered/aligned LiDAR scan (~10 Hz)" ;;
        /state_estimation)            echo "Point-LIO output: 6-DOF state estimation (~10 Hz)" ;;
        /cloud_registered)            echo "Point-LIO alternate name for registered cloud" ;;
        /cmd_vel)                     echo "Velocity command sent to the robot (TwistStamped on Ethernet)" ;;
        /cmd_vel_stamped)             echo "Velocity command (TwistStamped, alternate name)" ;;

        # --- Integration package (this repo) ---
        /scan)                        echo "2D LaserScan produced by pointcloud_to_scan.py" ;;
        /map)                         echo "2D occupancy grid from slam_toolbox or map_server" ;;
        /map/occupancy)               echo "2D log-odds occupancy grid from map_node.py (TRANSIENT_LOCAL)" ;;
        /map/points)                  echo "Accumulated 3D point cloud from map_node.py" ;;
        /point_cloud2)                echo "WebRTC SDK cloud topic (only in WebRTC mode)" ;;
        /imu/data)                    echo "WebRTC SDK IMU topic (only in WebRTC mode)" ;;

        # --- ROS 2 standard ---
        /rosout)                      echo "ROS 2 log output" ;;
        /parameter_events)            echo "ROS 2 parameter change events" ;;
        /tf)                          echo "Transform tree (dynamic)" ;;
        /tf_static)                   echo "Transform tree (static)" ;;

        # --- Fallback ---
        *) echo "(no description on file)" ;;
    esac
}

# ===========================================================================
# Reference mode: print the full categorized topic reference and exit
# ===========================================================================
if [ "$MODE" = "reference" ]; then
    echo "================================================================"
    echo " Unitree Go2 EDU — Topic Reference"
    echo "================================================================"
    echo
    echo "--- Go2 native sensors (published by the robot's Jetson) ---"
    for t in /utlidar/cloud /utlidar/cloud_deskewed /utlidar/cloud_base \
             /utlidar/imu /utlidar/lidar_state /utlidar/range_info \
             /utlidar/robot_odom /utlidar/robot_pose /utlidar/switch \
             /utlidar/grid_map /utlidar/height_map /utlidar/voxel_map; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    echo "--- Onboard SLAM (uslam) ---"
    for t in /uslam/frontend/cloud_world_ds /uslam/frontend/odom \
             /uslam/localization/cloud_world /uslam/localization/odom \
             /uslam/navigation/global_path; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    echo "--- Camera / video ---"
    for t in /frontvideostream /videohub/inner /pctoimage_local \
             /webrtcreq /webrtcres /camera/image_raw /camera/camera_info; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    echo "--- Sport API (motion control) ---"
    for t in /api/sport/request /api/sport/response \
             /api/sport_lease/request /api/sport_lease/response \
             /api/motion_switcher/request /api/obstacles_avoid/request; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    echo "--- Robot state ---"
    for t in /lowstate /sportmodestate /lf/battery_alarm \
             /multiplestate /servicestate /selftest /gnss; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    echo "--- Remote control / UWB ---"
    for t in /wirelesscontroller /wirelesscontroller_unprocessed \
             /uwbstate /uwbswitch; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    echo "--- Arm (D1) ---"
    for t in /api/arm/request /arm_command /arm/action/state /arm_Feedback; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    echo "--- Audio / voice / gesture / sensors ---"
    for t in /audio_msg /audioreceiver /audiosender /gesture/result \
             /gas_sensor /pet/flowfeedback; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    echo "--- AI / GPT ---"
    for t in /gpt_cmd /gpt_state /gptflowfeedback; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    echo "--- CMU autonomy stack (runs on the VM) ---"
    for t in /registered_scan /state_estimation /cloud_registered \
             /cmd_vel /cmd_vel_stamped; do
        printf "  %-38s %s\n" "$t" "$(describe_topic "$t")"
    done
    echo
    echo "--- Integration package (this repo) ---"
    for t in /scan /map /map/occupancy /map/points; do
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
        EXPECTED=(
            "/utlidar/cloud:sensor_msgs/msg/PointCloud2"
            "/utlidar/imu:sensor_msgs/msg/Imu"
            "/registered_scan:sensor_msgs/msg/PointCloud2"
            "/state_estimation:nav_msgs/msg/Odometry"
            "/cmd_vel:geometry_msgs/msg/TwistStamped"
        )
        ;;
    dog)
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

# --- List every visible topic, categorised, with descriptions ---
echo "--- All topics visible on this machine ---"
echo

if [ -z "${ALL_TOPICS}" ]; then
    echo "  (none — is the robot powered on and connected?)"
else
    EXPECTED_NAMES="$(for entry in "${EXPECTED[@]}"; do echo "${entry%%:*}"; done)"

    # --- Expected & present ---
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

    # --- Expected but missing ---
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

    # --- Present but not expected ---
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

# --- /map/occupancy QoS check ---
if grep -qxF "/map/occupancy" <<< "${ALL_TOPICS}"; then
    echo "--- /map/occupancy QoS (should be TRANSIENT_LOCAL) ---"
    ros2 topic info /map/occupancy -v 2>/dev/null \
        | grep -A 1 "Durability" || true
    echo
fi

# --- Guidance on missing topics ---
if [ "${FAIL}" -gt 0 ] && [ "${#MISSING_LIST[@]}" -gt 0 ]; then
    echo "--- What to do about missing topics ---"
    for topic in "${MISSING_LIST[@]}"; do
        case "${topic}" in
            /utlidar/cloud|/utlidar/imu|/utlidar/cloud_deskewed|/utlidar/robot_odom)
                echo "  ${topic}:"
                echo "      Robot not publishing. Check robot power, DDS connection,"
                echo "      and that CYCLONEDDS_URI points at the right interface."
                ;;
            /registered_scan|/state_estimation)
                echo "  ${topic}:"
                echo "      Point-LIO not running or stuck."
                echo "      Launch:  ./scripts/system_real_robot_ethernet.sh"
                echo "      Check:   grep use_sim_time ~/files/autonomy_stack_go2/src/slam/point_lio_unilidar/config/utlidar.yaml"
                ;;
            /cmd_vel)
                echo "  ${topic}:"
                echo "      No active goal. This topic appears lazily when a path is being followed."
                echo "      Set a waypoint in RViz (Waypoint / 2D Goal Pose) and re-run this script."
                ;;
            /scan)
                echo "  ${topic}:"
                echo "      pointcloud_to_scan.py not running."
                echo "      Launch:  ros2 run go2_integration_pkg pointcloud_to_scan.py"
                ;;
            *)
                echo "  ${topic}:"
                echo "      not found. Check whether the relevant node is running."
                ;;
        esac
    done
    echo
fi

echo "Tip: run './scripts/verify_topics.sh --ref' to print the full topic reference."
echo

exit ${FAIL}