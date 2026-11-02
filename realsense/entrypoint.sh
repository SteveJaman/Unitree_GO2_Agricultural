#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# entrypoint.sh: inside the go2-realsense container.
#
# Writes a CycloneDDS config bound to ONE robot interface with the workstation
# as a unicast peer (the same pattern as scripts/robot/go2_camera_wifi.sh),
# then starts realsense2_camera_node as /camera/camera, so the topics are
#   /camera/color/image_raw                  /camera/color/camera_info
#   /camera/depth/image_rect_raw             /camera/depth/camera_info
#   /camera/aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info
# plus /compressed and /compressedDepth copies (image_transport_plugins).
#
# Commands:  camera (default) | check | shell | <any command>
#
# Environment (set by go2_realsense.sh):
#   ROS_DOMAIN_ID=64          DDS domain (64 = the Wi-Fi domain of this repo)
#   RS_DDS_IFACE=wlan0        robot interface DDS binds to (wlan0 / eth0)
#   RS_DDS_PEERS="ip ..."     workstation IP(s), unicast discovery
#   RS_DDS_MULTICAST=spdp     Cyclone AllowMulticast (spdp | false | true)
#   RS_COLOR_PROFILE=640,480,15   RS_DEPTH_PROFILE=640,480,15   (w,h,fps; 4.51.1 uses commas)
#   RS_ALIGN=true  RS_IMU=false  RS_POINTCLOUD=false  RS_INITIAL_RESET=true
#   RS_QOS=SYSTEM_DEFAULT     image QoS (SYSTEM_DEFAULT = reliable; SENSOR_DATA = best effort)
#   RS_LOG_LEVEL=info         RS_DDS_TRACE=0 (1 = Cyclone trace in /tmp/cdds.log)
# ---------------------------------------------------------------------------
set -eo pipefail

if [ "${1:-}" = check ]; then
    # SDK only, no ROS / DDS: separates USB / librealsense problems from ROS / DDS ones.
    lsusb | grep -i '8086:' || echo "[rs] no Intel (8086:) USB device visible in the container" >&2
    exec rs-enumerate-devices -s
fi

DOMAIN="${ROS_DOMAIN_ID:-64}"
IFACE="${RS_DDS_IFACE:-wlan0}"
PEERS="${RS_DDS_PEERS:-}"
MCAST="${RS_DDS_MULTICAST:-spdp}"
XML=/tmp/rs-cyclonedds.xml

for v in "${IFACE}" ${PEERS//,/ }; do
    [[ "${v}" =~ ^[a-zA-Z0-9_.:-]+$ ]] || { echo "[rs] invalid interface or peer: '${v}'" >&2; exit 2; }
done
[[ "${MCAST}" =~ ^(spdp|false|true|asm)$ ]] || { echo "[rs] RS_DDS_MULTICAST must be spdp|false|true|asm" >&2; exit 2; }
if [ "${DOMAIN}" = 0 ]; then
    echo "[rs] refusing ROS_DOMAIN_ID=0: that is Unitree's onboard DDS domain." >&2
    echo "     Use a separate domain (default 64) so the camera never mixes with sport/state topics." >&2
    exit 2
fi
if ! ip link show "${IFACE}" >/dev/null 2>&1; then
    echo "[rs] interface ${IFACE} not found (container must run with --network host). Have:" >&2
    ip -br link >&2 || true
    exit 2
fi

peer_xml=""
for p in ${PEERS//,/ }; do peer_xml+="        <Peer address=\"${p}\"/>"$'\n'; done
[ -n "${peer_xml}" ] || echo "[rs] WARNING: no RS_DDS_PEERS: discovery relies on multicast (often blocked on Wi-Fi)." >&2

trace_xml=""
if [ "${RS_DDS_TRACE:-0}" = 1 ]; then
    trace_xml="    <Tracing><Verbosity>config</Verbosity><OutputFile>/tmp/cdds.log</OutputFile></Tracing>"
fi

# Foxy's stock Cyclone is 0.7.x: it rejects the newer <Interfaces> element
# (participant creation fails), so this uses NetworkInterfaceAddress, which
# 0.7 understands and 0.10 (Humble) still accepts.
cat > "${XML}" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <NetworkInterfaceAddress>${IFACE}</NetworkInterfaceAddress>
      <AllowMulticast>${MCAST}</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers>
${peer_xml}      </Peers>
    </Discovery>
${trace_xml}
  </Domain>
</CycloneDDS>
EOF

# Never inherit library paths from the robot: Unitree's Cyclone 0.10 next to
# Foxy's 0.7 is what makes stock rclpy/rclcpp crash on this Jetson.
unset LD_LIBRARY_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
export ROS_DOMAIN_ID="${DOMAIN}" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://${XML}"
export ROS_LOCALHOST_ONLY=0 RCUTILS_COLORIZED_OUTPUT=1
echo "${ROS_DOMAIN_ID}" > /tmp/rs-domain   # read by `go2_realsense.sh topics|hz` (docker exec)
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
# shellcheck disable=SC1091
source /opt/rs_ws/install/setup.bash

echo "[rs] domain ${ROS_DOMAIN_ID}, ${RMW_IMPLEMENTATION}, bound to ${IFACE}, peers: ${PEERS:-none}, multicast: ${MCAST}"

write_params() {
    local imu="${RS_IMU:-false}" qos="${RS_QOS:-SYSTEM_DEFAULT}"
    cat > /tmp/rs_params.yaml <<EOF
/camera/camera:
  ros__parameters:
    enable_color: true
    enable_depth: true
    enable_infra1: false
    enable_infra2: false
    rgb_camera.profile: "${RS_COLOR_PROFILE:-640,480,15}"
    depth_module.profile: "${RS_DEPTH_PROFILE:-640,480,15}"
    align_depth.enable: ${RS_ALIGN:-true}
    enable_sync: ${RS_ALIGN:-true}
    pointcloud.enable: ${RS_POINTCLOUD:-false}
    enable_gyro: ${imu}
    enable_accel: ${imu}
    unite_imu_method: $([ "${imu}" = true ] && echo 2 || echo 0)
    initial_reset: ${RS_INITIAL_RESET:-true}
    color_qos: "${qos}"
    depth_qos: "${qos}"
    publish_tf: true
    tf_publish_rate: 0.0
    wait_for_device_timeout: 30.0
    reconnect_timeout: 6.0
EOF
}

case "${1:-camera}" in
    camera)
        echo "[rs] USB devices seen by the container:"
        lsusb | grep -i '8086:' || { echo "[rs] no Intel (8086:) device visible: check /dev/bus/usb passthrough and the rear USB-C port" >&2; }
        write_params
        echo "[rs] parameters:"; sed 's/^/    /' /tmp/rs_params.yaml
        exec ros2 run realsense2_camera realsense2_camera_node \
            --ros-args -r __ns:=/camera -r __node:=camera \
            --params-file /tmp/rs_params.yaml \
            --log-level "${RS_LOG_LEVEL:-info}"
        ;;
    shell)
        exec bash
        ;;
    *)
        exec "$@"
        ;;
esac
