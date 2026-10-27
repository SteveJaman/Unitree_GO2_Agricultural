#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# start_inside.sh: entry point of the go2-3d container (run by wifi_3d_nav.sh).
#
# Starts, on the session's DDS domain:
#   * rosbridge :9090     for the web dashboard (skipped if the port is taken)
#   * web dashboard :3000 twin/webapp, static files served by python http.server
#                         (/twin/webapp/dist if newer than the image's copy)
#   * foxglove_bridge     :8765, only with TWIN_FOXGLOVE=1 (alternative viewer)
#   * go2_click_to_goal   /clicked_point -> nearest safe floor -> /goal_pose
#   * camera support      (if the camera publishes) base_link -> camera TF,
#                         go2_camera_info.py (intrinsics), image_compressor.py
#                         (JPEG copies for the dashboard)
#   * RTAB-Map            dense 3D map -> /rtabmap/cloud_map, database TWIN_DB
#   * yolo_node.py        (TWIN_YOLO=1 and camera) /yolo/detections, /yolo/obstacles
#   * vlm_reasoning_node  (TWIN_VLM=1 and camera) Ollama llava -> /vlm_decision;
#                         confirmed BLOCKED pauses the explorer and docks (TWIN_VLM_ACT)
#   * auto_localize       (TWIN_AUTOLOCALIZE=1) one-shot /initialpose at the dock
#   * room_segmentation   (TWIN_ROOMS=1) costmap -> rooms (/semantic_rooms), names
#                         and merges from the dashboard -> config/semantic_rooms.json
#   * survey_manager      (TWIN_SURVEY=1) autonomous survey: explore, rooms, visit and
#                         categorize each, report, dock; TWIN_SURVEY_AUTO=1 starts it
#
# A TWIN_DB that already exists is continued: RTAB-Map adds a new session to
# it and links it to the old map once it recognises a place (--resume).
#
# RTAB-Map modes (TWIN_MODE):
#   auto    camera if TWIN_RGB_TOPIC publishes within TWIN_CAMERA_WAIT s, else lidar
#   lidar   LiDAR only (ICP)
#   camera  Go2 camera + LiDAR: the cloud is projected into the image as a depth
#           image (rtabmap_util pointcloud_to_depthimage), so RTAB-Map gets
#           RGB-D and the map is coloured
#   fusion  like camera, but from the lens calibration (TWIN_CAMERA_INFO):
#           go2_fusion_camera.py undistorts the image and projects the LiDAR
#           into it; go2_lidar_overlay.py draws the check image
#   rgbd    RealSense colour + depth + LiDAR
#
# Odometry comes from TF (lidar3d_map -> base_link, Point-LIO + go2_odom_tf),
# so RTAB-Map's map frame "rtabmap_map" becomes the parent of lidar3d_map and
# Nav2 keeps working in lidar3d_map. On SIGTERM (docker stop) every child is
# stopped and RTAB-Map writes its database before exiting. If RTAB-Map dies
# the container exits; a crash of any other node is logged only.
#
# Environment (set by wifi_3d_nav.sh; ROS_DOMAIN_ID / RMW / CYCLONEDDS_URI
# are copied from the running wifi_nav.sh session):
#   TWIN_DB=/data/rtabmap.db   TWIN_MODE=auto|lidar|camera|fusion|rgbd
#   TWIN_CAMERA_INFO=/camera_cfg/camera.yaml   (fusion / camera obstacles: lens calibration)
#   TWIN_CAMERA_OBSTACLES=1    camera obstacles for Nav2 (go2_camera_obstacles.py)
#   TWIN_ROSBRIDGE=1  TWIN_ROSBRIDGE_PORT=9090   TWIN_WEB=1  TWIN_WEB_PORT=3000
#   TWIN_FOXGLOVE=0   TWIN_FOXGLOVE_PORT=8765
#   TWIN_CLOUD_TOPIC=/lidar3d/registered
#   TWIN_RGB_TOPIC=/camera/image_raw (rgbd: /camera/color/image_raw)
#   TWIN_DEPTH_TOPIC / TWIN_CAMINFO_TOPIC   (rgbd mode, RealSense defaults)
#   TWIN_CAMERA_WAIT=60         s to wait for the first camera frame (auto)
#   TWIN_CAMERA_TF="0.327 0 0.043 -1.5708 0 -1.5708"   x y z yaw pitch roll of
#                               the optical frame in base_link (Go2 URDF
#                               front_camera_joint); "none" = do not publish
#   TWIN_CAMERA_HFOV_DEG=120    used when the camera has no camera_info
#   TWIN_YOLO=0|1  TWIN_YOLO_MODEL=/opt/yolo/yolo11s.pt  TWIN_YOLO_RATE=2
#   TWIN_VLM=0|1   TWIN_VLM_URL=http://localhost:11434/api/generate
#   TWIN_VLM_MODEL=llava:7b  TWIN_VLM_PERIOD=5  TWIN_VLM_ACT=1  TWIN_VLM_CONFIRM=2
#   TWIN_AUTOLOCALIZE=0|1    TWIN_DOCK="0 0 0"  (x y yaw in TWIN_MAP_FRAME)
#   TWIN_ROOMS=1  TWIN_ROOM_DOOR_WIDTH=0.9 (m)  TWIN_ROOM_MIN_AREA=1.5 (m^2)
#   TWIN_SURVEY=1  TWIN_SURVEY_AUTO=0|1
#   TWIN_YOLO_CLASSES="chair,person,..."|all
#   TWIN_MAP_FRAME=lidar3d_map  TWIN_BASE_FRAME=base_link
#   TWIN_RTAB_ARGS="..."        extra RTAB-Map parameters, e.g. "--Grid/CellSize 0.1"
# ---------------------------------------------------------------------------
set -eo pipefail

TWIN_DB="${TWIN_DB:-/data/rtabmap.db}"
TWIN_MODE="${TWIN_MODE:-auto}"
TWIN_ROSBRIDGE="${TWIN_ROSBRIDGE:-1}"
TWIN_ROSBRIDGE_PORT="${TWIN_ROSBRIDGE_PORT:-9090}"
TWIN_WEB="${TWIN_WEB:-1}"
TWIN_WEB_PORT="${TWIN_WEB_PORT:-3000}"
TWIN_FOXGLOVE="${TWIN_FOXGLOVE:-0}"
TWIN_FOXGLOVE_PORT="${TWIN_FOXGLOVE_PORT:-8765}"
TWIN_CLOUD_TOPIC="${TWIN_CLOUD_TOPIC:-/lidar3d/registered}"
if [ "$TWIN_MODE" = rgbd ]; then
    TWIN_RGB_TOPIC="${TWIN_RGB_TOPIC:-/camera/color/image_raw}"
else
    TWIN_RGB_TOPIC="${TWIN_RGB_TOPIC:-/camera/image_raw}"
fi
TWIN_DEPTH_TOPIC="${TWIN_DEPTH_TOPIC:-/camera/depth/image_rect_raw}"
TWIN_CAMINFO_TOPIC="${TWIN_CAMINFO_TOPIC:-/camera/color/camera_info}"
TWIN_CAMERA_WAIT="${TWIN_CAMERA_WAIT:-60}"
TWIN_CAMERA_TF="${TWIN_CAMERA_TF:-0.327 0 0.043 -1.5708 0 -1.5708}"
TWIN_CAMERA_HFOV_DEG="${TWIN_CAMERA_HFOV_DEG:-120}"
TWIN_YOLO="${TWIN_YOLO:-0}"
TWIN_YOLO_MODEL="${TWIN_YOLO_MODEL:-/opt/yolo/yolo11s.pt}"
TWIN_YOLO_RATE="${TWIN_YOLO_RATE:-2}"
TWIN_VLM="${TWIN_VLM:-0}"
TWIN_VLM_URL="${TWIN_VLM_URL:-http://localhost:11434/api/generate}"
TWIN_VLM_MODEL="${TWIN_VLM_MODEL:-llava:7b}"
TWIN_VLM_PERIOD="${TWIN_VLM_PERIOD:-5}"
TWIN_VLM_ACT="${TWIN_VLM_ACT:-1}"
TWIN_VLM_CONFIRM="${TWIN_VLM_CONFIRM:-2}"
TWIN_AUTOLOCALIZE="${TWIN_AUTOLOCALIZE:-0}"
TWIN_ROOMS="${TWIN_ROOMS:-1}"
TWIN_SURVEY="${TWIN_SURVEY:-1}"
TWIN_SURVEY_AUTO="${TWIN_SURVEY_AUTO:-0}"
TWIN_ROOM_DOOR_WIDTH="${TWIN_ROOM_DOOR_WIDTH:-0.9}"
TWIN_ROOM_MIN_AREA="${TWIN_ROOM_MIN_AREA:-1.5}"
read -r DOCK_X DOCK_Y DOCK_YAW <<< "${TWIN_DOCK:-0 0 0}"
TWIN_MAP_FRAME="${TWIN_MAP_FRAME:-lidar3d_map}"
TWIN_BASE_FRAME="${TWIN_BASE_FRAME:-base_link}"
TWIN_CAMERA_INFO="${TWIN_CAMERA_INFO:-}"
TWIN_CAMERA_OBSTACLES="${TWIN_CAMERA_OBSTACLES:-0}"
GO2_CAMINFO_TOPIC=/camera/camera_info      # published by go2_camera_info.py
LIDAR_DEPTH_TOPIC=/camera/lidar_depth      # published by pointcloud_to_depthimage
LOGS=/logs

mkdir -p "${HOME:-/tmp/go2-3d-home}" "$LOGS"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

# Executables are run directly, not through `ros2 run`: its wrapper does not
# forward a signal sent to it alone, and RTAB-Map must see SIGTERM to save.
PIDS=(); NAMES=()
start() {   # $1 = name (log file /logs/<name>.log), rest = command
    local name="$1"; shift
    "$@" > "$LOGS/${name}.log" 2>&1 &
    PIDS+=("$!"); NAMES+=("$name")
    echo "[go2-3d] ${name} started (pid $!, log ${LOGS}/${name}.log)"
}
exe() { echo "$(ros2 pkg prefix "$1")/lib/$1/$2"; }
# Float ROS parameters reject "2"; make it "2.0".
float() { case "$1" in *.*|*[eE]*) echo "$1" ;; *) echo "$1.0" ;; esac; }

STOPPING=0
stop_all() {
    [ "$STOPPING" = 1 ] && return 0
    STOPPING=1
    echo "[go2-3d] stopping (RTAB-Map saves ${TWIN_DB})..."
    for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
    wait || true
    echo "[go2-3d] stopped"
}
trap 'stop_all; exit 0' TERM INT

echo "[go2-3d] DDS domain ${ROS_DOMAIN_ID:-0}, ${RMW_IMPLEMENTATION:-default RMW}, ${CYCLONEDDS_URI:-no CYCLONEDDS_URI}"

# True if something already listens on a local TCP port.
port_busy() { (exec 3<>"/dev/tcp/127.0.0.1/$1") 2>/dev/null; }

# --- Bridges, dashboard, click-to-goal first: viewers connect while we wait --
if [ "$TWIN_ROSBRIDGE" = 1 ]; then
    if port_busy "$TWIN_ROSBRIDGE_PORT"; then
        echo "[go2-3d] rosbridge: port ${TWIN_ROSBRIDGE_PORT} already in use, using the running server"
    else
        start rosbridge ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
            port:="$TWIN_ROSBRIDGE_PORT"
    fi
fi
if [ "$TWIN_WEB" = 1 ]; then
    WEBROOT=/opt/webapp
    # A dashboard built on the host (cd twin/webapp && npm run build) wins if newer.
    [ /twin/webapp/dist/index.html -nt /opt/webapp/index.html ] && WEBROOT=/twin/webapp/dist
    if [ ! -f "$WEBROOT/index.html" ]; then
        echo "[go2-3d] WARN: no dashboard build in the image (rebuild: wifi_3d_nav.sh --build)"
    elif port_busy "$TWIN_WEB_PORT"; then
        echo "[go2-3d] WARN: port ${TWIN_WEB_PORT} already in use; dashboard not served"
    else
        # One served root: the app, maps/ (databases and *_mesh.ply from
        # --export --mesh) and config/ (semantic_locations.json).
        SERVE=/tmp/go2-3d-web
        rm -rf "$SERVE" && mkdir -p "$SERVE"
        for f in "$WEBROOT"/*; do ln -s "$f" "$SERVE/"; done
        ln -s /data "$SERVE/maps"
        ln -s /config "$SERVE/config"
        start web python3 -m http.server "$TWIN_WEB_PORT" --bind 0.0.0.0 --directory "$SERVE"
        echo "[go2-3d] dashboard: ${WEBROOT} on port ${TWIN_WEB_PORT} (+ maps/, config/)"
    fi
fi
if [ "$TWIN_FOXGLOVE" = 1 ]; then
    start foxglove "$(exe foxglove_bridge foxglove_bridge)" \
        --ros-args -p port:="$TWIN_FOXGLOVE_PORT" -p address:=0.0.0.0
fi
start click_to_goal python3 -u /nodes/go2_click_to_goal.py --ros-args \
    -p map_frame:="$TWIN_MAP_FRAME" -p robot_base_frame:="$TWIN_BASE_FRAME"

# Rooms: costmap -> room polygons (/semantic_rooms), names/merges from the
# dashboard, saved to config/semantic_rooms.json.
if [ "$TWIN_ROOMS" = 1 ]; then
    start rooms python3 -u /twin/room_segmentation_node.py --ros-args \
        -p door_width:="$(float "$TWIN_ROOM_DOOR_WIDTH")" -p min_room_area:="$(float "$TWIN_ROOM_MIN_AREA")"
fi

# Autonomous survey: explore -> rooms -> visit + categorize each -> report -> dock.
# Always running (dashboard "Auto Survey"); TWIN_SURVEY_AUTO=1 (--survey) starts it unattended.
if [ "$TWIN_SURVEY" = 1 ]; then
    start survey python3 -u /twin/survey_manager.py --ros-args \
        -p auto_start:="$( [ "$TWIN_SURVEY_AUTO" = 1 ] && echo true || echo false )" \
        -p dock_x:="$(float "$DOCK_X")" -p dock_y:="$(float "$DOCK_Y")" \
        -p map_frame:="$TWIN_MAP_FRAME" -p base_frame:="$TWIN_BASE_FRAME" \
        -p database:="$TWIN_DB"
fi

# One-shot: /initialpose at the session's start pose (the dock), then exits.
# Not in PIDS, so its normal exit is not reported as a crash.
if [ "$TWIN_AUTOLOCALIZE" = 1 ]; then
    python3 -u /nodes/auto_localize.py --ros-args \
        -p map_frame:="$TWIN_MAP_FRAME" -p robot_base_frame:="$TWIN_BASE_FRAME" \
        > "$LOGS/auto_localize.log" 2>&1 &
    echo "[go2-3d] auto_localize started (log ${LOGS}/auto_localize.log)"
fi

# --- Camera: wait for the first frame, learn its TF frame -------------------
CAMERA_FRAME=""
if [ "$TWIN_MODE" = auto ] || [ "$TWIN_MODE" = camera ]; then
    echo "[go2-3d] waiting up to ${TWIN_CAMERA_WAIT} s for a frame on ${TWIN_RGB_TOPIC}..."
    CAMERA_FRAME="$(timeout "$TWIN_CAMERA_WAIT" ros2 topic echo --once --no-arr \
            --qos-reliability best_effort "$TWIN_RGB_TOPIC" sensor_msgs/msg/Image 2>/dev/null \
        | awk '/frame_id:/ {gsub(/["\047]/, "", $2); print $2; exit}' || true)"
    if [ -n "$CAMERA_FRAME" ]; then
        echo "[go2-3d] camera frames arrive (frame ${CAMERA_FRAME})"
        [ "$TWIN_MODE" = auto ] && TWIN_MODE=camera
    elif [ "$TWIN_MODE" = camera ]; then
        echo "[go2-3d] WARN: no frame on ${TWIN_RGB_TOPIC} yet; RTAB-Map waits for it (no map until then)"
        CAMERA_FRAME="${TWIN_CAMERA_FRAME:-camera_link}"
    else
        echo "[go2-3d] no camera frames on ${TWIN_RGB_TOPIC}: LiDAR-only map, YOLO off"
        TWIN_MODE=lidar
    fi
    CAMERA_FRAME="${TWIN_CAMERA_FRAME:-$CAMERA_FRAME}"
fi

if [ "$TWIN_MODE" = camera ]; then
    if [ "$TWIN_CAMERA_TF" = none ]; then
        echo "[go2-3d] camera TF: not published (TWIN_CAMERA_TF=none)"
    elif timeout 5 "$(exe tf2_ros tf2_echo)" "$TWIN_BASE_FRAME" "$CAMERA_FRAME" 2>/dev/null | grep -q Translation; then
        echo "[go2-3d] camera TF: ${TWIN_BASE_FRAME} -> ${CAMERA_FRAME} already published by the session"
    else
        read -r cx cy cz cyaw cpitch croll <<< "$TWIN_CAMERA_TF"
        echo "[go2-3d] camera TF: ${TWIN_BASE_FRAME} -> ${CAMERA_FRAME} = ${TWIN_CAMERA_TF} (x y z yaw pitch roll)"
        start camera_tf "$(exe tf2_ros static_transform_publisher)" \
            --x "$cx" --y "$cy" --z "$cz" --yaw "$cyaw" --pitch "$cpitch" --roll "$croll" \
            --frame-id "$TWIN_BASE_FRAME" --child-frame-id "$CAMERA_FRAME"
    fi
    # A calibrated relay (go2_camera_relay.py camera_info_file) already
    # publishes the real intrinsics there; approximate ones only without it.
    if timeout 5 ros2 topic echo --once --no-arr --qos-reliability best_effort \
            "$GO2_CAMINFO_TOPIC" sensor_msgs/msg/CameraInfo >/dev/null 2>&1; then
        echo "[go2-3d] camera info: ${GO2_CAMINFO_TOPIC} already published (calibrated relay)"
    else
        start camera_info python3 -u /nodes/go2_camera_info.py --ros-args \
            -p image_topic:="$TWIN_RGB_TOPIC" -p camera_info_topic:="$GO2_CAMINFO_TOPIC" \
            -p hfov_deg:="$(float "$TWIN_CAMERA_HFOV_DEG")"
    fi
    if [ "$TWIN_ROSBRIDGE" = 1 ]; then
        start image_compressor python3 -u /twin/image_compressor.py --ros-args \
            -p "pairs:=[\"${TWIN_RGB_TOPIC}:/twin/camera/compressed\", \"/yolo/image:/twin/yolo/compressed\"]"
    fi
    start lidar_depth "$(exe rtabmap_util pointcloud_to_depthimage)" --ros-args \
        -r __node:=lidar_depth \
        -p fixed_frame_id:="$TWIN_MAP_FRAME" -p decimation:=4 \
        -p fill_holes_size:=3 -p fill_iterations:=1 \
        -p approx:=true -p wait_for_transform:=0.2 -p qos:=2 \
        -r cloud:="$TWIN_CLOUD_TOPIC" -r camera_info:="$GO2_CAMINFO_TOPIC" \
        -r image:="$LIDAR_DEPTH_TOPIC" -r image_raw:="${LIDAR_DEPTH_TOPIC}_raw"
fi

if [ "$TWIN_MODE" = fusion ]; then
    # Calibrated front camera + LiDAR: go2_fusion_camera.py publishes the
    # undistorted image, its camera_info and a LiDAR depth image projected into
    # that camera, all with one stamp (/twin/rgb/*, /twin/depth/image); the
    # camera TF comes from the session's URDF (front_camera).
    [ -f "$TWIN_CAMERA_INFO" ] || { echo "[go2-3d] ERROR: fusion needs TWIN_CAMERA_INFO (got '${TWIN_CAMERA_INFO}')" >&2; stop_all; exit 1; }
    start fusion_camera python3 -u /nodes/go2_fusion_camera.py --ros-args \
        -p image_topic:="$TWIN_RGB_TOPIC" -p camera_info_file:="$TWIN_CAMERA_INFO" \
        -p cloud_topic:="$TWIN_CLOUD_TOPIC"
    # Live fusion check: LiDAR points drawn on the camera image.
    start overlay python3 -u /nodes/go2_lidar_overlay.py --ros-args \
        -p image_topic:="$TWIN_RGB_TOPIC" -p camera_info_file:="$TWIN_CAMERA_INFO" \
        -p cloud_topic:="$TWIN_CLOUD_TOPIC"
    if [ "$TWIN_ROSBRIDGE" = 1 ]; then
        start image_compressor python3 -u /twin/image_compressor.py --ros-args \
            -p "pairs:=[\"${TWIN_RGB_TOPIC}:/twin/camera/compressed\", \"/yolo/image:/twin/yolo/compressed\"]"
    fi
fi

# --- RTAB-Map ---------------------------------------------------------------
# LiDAR registration (ICP) and proximity loop closures, 3D grid (after the
# rtabmap_examples lidar3d settings). Camera modes add visual loop closures
# and colour.
RTAB_PARAMS=(
    --Rtabmap/DetectionRate 1
    --RGBD/LinearUpdate 0.05 --RGBD/AngularUpdate 0.05
    --RGBD/ProximityMaxGraphDepth 0 --RGBD/ProximityPathMaxNeighbors 1
    --Mem/NotLinkedNodesKept false --Mem/STMSize 30 --Mem/LaserScanNormalK 20
    --Reg/Strategy 1
    --Icp/VoxelSize 0.1 --Icp/PointToPlane true --Icp/PointToPlaneK 20 --Icp/PointToPlaneRadius 0
    --Icp/Iterations 10 --Icp/Epsilon 0.001 --Icp/MaxTranslation 3
    --Icp/MaxCorrespondenceDistance 1 --Icp/CorrespondenceRatio 0.2
    --RGBD/CreateOccupancyGrid true --Grid/3D true --Grid/CellSize 0.05 --Grid/RangeMax 15
    # Denser surfaces for the dashboard: ray tracing clears space behind
    # moving objects; without normals segmentation every surface (floor too)
    # stays in /rtabmap/cloud_map. RTAB-Map publishes no mesh topic: the
    # dashboard renders this cloud as solid voxels, and a real triangle mesh
    # comes from wifi_3d_nav.sh --export --mesh.
    --Grid/RayTracing true --Grid/NormalsSegmentation false
)
ROS_ARGS=(
    -r __node:=rtabmap -r __ns:=/rtabmap
    -p frame_id:="$TWIN_BASE_FRAME"
    -p odom_frame_id:="$TWIN_MAP_FRAME"
    -p map_frame_id:=rtabmap_map
    -p publish_tf:=true
    -p database_path:="$TWIN_DB"
    -p wait_for_transform:=0.5
    -p subscribe_scan_cloud:=true -p qos_scan:=2
    -r scan_cloud:="$TWIN_CLOUD_TOPIC"
)
case "$TWIN_MODE" in
    camera) RGB="$TWIN_RGB_TOPIC"; DEPTH="$LIDAR_DEPTH_TOPIC"; INFO="$GO2_CAMINFO_TOPIC" ;;
    fusion) RGB=/twin/rgb/image;   DEPTH=/twin/depth/image;   INFO=/twin/rgb/camera_info ;;
    rgbd)   RGB="$TWIN_RGB_TOPIC"; DEPTH="$TWIN_DEPTH_TOPIC"; INFO="$TWIN_CAMINFO_TOPIC" ;;
    lidar)  ;;
    *) echo "[go2-3d] ERROR: unknown TWIN_MODE=${TWIN_MODE}" >&2; stop_all; exit 1 ;;
esac
if [ "$TWIN_MODE" = lidar ]; then
    RTAB_PARAMS+=(--Grid/Sensor 0)
    ROS_ARGS+=(-p subscribe_rgb:=false -p subscribe_depth:=false)
elif [ "$TWIN_MODE" = fusion ]; then
    # go2_fusion_camera.py publishes image, camera_info and LiDAR depth with
    # one stamp on a reliable QoS. LiDAR depth is sparse: keep every depth
    # pixel (default decimation is 4).
    RTAB_PARAMS+=(--Grid/Sensor 2 --Grid/DepthDecimation 1)
    ROS_ARGS+=(
        -p subscribe_rgb:=true -p subscribe_depth:=true -p approx_sync:=true
        -p qos_image:=0 -p qos_camera_info:=0
        -r rgb/image:="$RGB" -r depth/image:="$DEPTH" -r rgb/camera_info:="$INFO"
    )
else
    RTAB_PARAMS+=(--Grid/Sensor 2)
    ROS_ARGS+=(
        -p subscribe_rgb:=true -p subscribe_depth:=true -p approx_sync:=true
        -p qos_image:=2 -p qos_camera_info:=2
        -r rgb/image:="$RGB" -r depth/image:="$DEPTH" -r rgb/camera_info:="$INFO"
    )
fi
# shellcheck disable=SC2206
RTAB_PARAMS+=(${TWIN_RTAB_ARGS:-})

echo "[go2-3d] RTAB-Map ${TWIN_MODE}: cloud ${TWIN_CLOUD_TOPIC}$( [ "$TWIN_MODE" != lidar ] && echo ", image ${RGB}, depth ${DEPTH}" ), odometry TF ${TWIN_MAP_FRAME} -> ${TWIN_BASE_FRAME}"
start rtabmap "$(exe rtabmap_slam rtabmap)" "${RTAB_PARAMS[@]}" --ros-args "${ROS_ARGS[@]}"
RTAB_PID="${PIDS[-1]}"

# --- YOLO -------------------------------------------------------------------
if [ "$TWIN_YOLO" = 1 ] && { [ "$TWIN_MODE" = camera ] || [ "$TWIN_MODE" = fusion ]; }; then
    if python3 -c 'import ultralytics' 2>/dev/null; then
        YOLO_ARGS=(
            -p image_topic:="$RGB" -p camera_info_topic:="$INFO"
            -p cloud_topic:="$TWIN_CLOUD_TOPIC" -p model:="$TWIN_YOLO_MODEL"
            -p rate_hz:="$(float "$TWIN_YOLO_RATE")"
            -p map_frame:="$TWIN_MAP_FRAME" -p robot_base_frame:="$TWIN_BASE_FRAME"
        )
        [ -n "${TWIN_YOLO_CLASSES:-}" ] && YOLO_ARGS+=(-p "obstacle_classes:=${TWIN_YOLO_CLASSES}")
        start yolo python3 -u /twin/yolo_node.py --ros-args "${YOLO_ARGS[@]}"
    else
        echo "[go2-3d] WARN: ultralytics not in the image (rebuild: wifi_3d_nav.sh --build); YOLO off"
    fi
elif [ "$TWIN_YOLO" = 1 ]; then
    echo "[go2-3d] YOLO off (needs the Go2 camera: TWIN_MODE=camera or fusion)"
fi

# --- VLM (Ollama) -----------------------------------------------------------
if [ "$TWIN_VLM" = 1 ] && { [ "$TWIN_MODE" = camera ] || [ "$TWIN_MODE" = fusion ]; }; then
    start vlm python3 -u /nodes/vlm_reasoning_node.py --ros-args \
        -p image_topic:="$TWIN_RGB_TOPIC" -p url:="$TWIN_VLM_URL" \
        -p model:="$TWIN_VLM_MODEL" -p period:="$(float "$TWIN_VLM_PERIOD")" \
        -p act_on_blocked:="$( [ "$TWIN_VLM_ACT" = 1 ] && echo true || echo false )" \
        -p blocked_confirmations:="$TWIN_VLM_CONFIRM" \
        -p dock_x:="$(float "$DOCK_X")" -p dock_y:="$(float "$DOCK_Y")" -p dock_yaw:="$(float "$DOCK_YAW")" \
        -p map_frame:="$TWIN_MAP_FRAME"
elif [ "$TWIN_VLM" = 1 ]; then
    echo "[go2-3d] VLM off (needs the Go2 camera: TWIN_MODE=camera or fusion)"
fi

if [ "$TWIN_CAMERA_OBSTACLES" = 1 ]; then
    # Depth Anything V2-Small scaled by the LiDAR -> /camera/obstacle_points,
    # a marking source in the Nav2 costmaps (scripts/tools/nav2_camera_source.py).
    [ -f "$TWIN_CAMERA_INFO" ] || { echo "[go2-3d] ERROR: camera obstacles need TWIN_CAMERA_INFO" >&2; stop_all; exit 1; }
    start camera_obstacles python3 -u /nodes/go2_camera_obstacles.py --ros-args \
        -p image_topic:="$TWIN_RGB_TOPIC" -p camera_info_file:="$TWIN_CAMERA_INFO" \
        -p cloud_topic:="$TWIN_CLOUD_TOPIC" -p map_frame:="$TWIN_MAP_FRAME"
fi

sleep 3
kill -0 "$RTAB_PID" 2>/dev/null || {
    echo "[go2-3d] ERROR: RTAB-Map exited at startup. Last log lines:" >&2
    tail -20 "$LOGS/rtabmap.log" >&2
    stop_all; exit 1
}
echo "[go2-3d] MODE ${TWIN_MODE}"
echo "[go2-3d] READY"

while :; do
    set +e
    wait -n
    set -e
    [ "$STOPPING" = 1 ] && break
    if ! kill -0 "$RTAB_PID" 2>/dev/null; then
        echo "[go2-3d] ERROR: RTAB-Map exited; see ${LOGS}/rtabmap.log" >&2
        stop_all; exit 1
    fi
    for i in "${!PIDS[@]}"; do
        kill -0 "${PIDS[$i]}" 2>/dev/null || echo "[go2-3d] WARN: ${NAMES[$i]} exited; see ${LOGS}/${NAMES[$i]}.log" >&2
    done
done
