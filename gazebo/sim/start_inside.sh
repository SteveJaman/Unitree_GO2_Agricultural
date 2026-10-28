#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# start_inside.sh: entry point of the go2-sim container (run by run_sim.sh).
#
# Builds the Go2 + D1 model, starts Gazebo Harmonic, spawns the robot, then
# runs ros_gz_bridge, sim_robot_bridge.py (the "Jetson") and the D1 arm
# emulator. If any child exits, everything stops (no half-working sim).
#
# Environment (set by run_sim.sh):
#   GO2_HOST_IP          address the laptop-side DDS binds to (required)
#   GO2_ROBOT_IP         DDS peer; in simulation the same host (default GO2_HOST_IP)
#   GO2_RELAY_DOMAIN_ID  Wi-Fi DDS domain (default 64)
#   SIM_GUI=1            open the Gazebo window (0 = headless rendering)
#   SIM_RENDER_ENGINE    ogre2 (default) or ogre (older/software GL)
#   SIM_ARM=1            run the D1 emulator on SIM_ARM_IFACE (default simarm0)
#   SIM_SPAWN="x y yaw"  robot start pose (default "-4.5 0 0")
#   SIM_WORLD            worlds/<name>.sdf: agri_rooms (default, field + building)
#                        or agri_field (field only)
#   SIM_D1_MOUNT         D1 pose on the trunk, see build_model.py --mount
#   SIM_SENSOR_YAML      lidar mount file (sensor_from_base), see build_model.py --sensor-yaml
# ---------------------------------------------------------------------------
set -eo pipefail

SIM=/sim
GEN=/tmp/sim-gen
WORLD="${SIM_WORLD:-agri_rooms}"
[ -f "$SIM/worlds/${WORLD}.sdf" ] || { echo "[sim] ERROR: no world $SIM/worlds/${WORLD}.sdf" >&2; exit 1; }
: "${GO2_HOST_IP:?GO2_HOST_IP must be set}"
GO2_ROBOT_IP="${GO2_ROBOT_IP:-$GO2_HOST_IP}"
GO2_RELAY_DOMAIN_ID="${GO2_RELAY_DOMAIN_ID:-64}"
SIM_GUI="${SIM_GUI:-1}"
SIM_RENDER_ENGINE="${SIM_RENDER_ENGINE:-ogre2}"
SIM_ARM="${SIM_ARM:-1}"
SIM_ARM_IFACE="${SIM_ARM_IFACE:-simarm0}"
read -r SPAWN_X SPAWN_Y SPAWN_YAW <<< "${SIM_SPAWN:--4.5 0 0}"

mkdir -p "$GEN" "${HOME:-/tmp/simhome}"
export HOME="${HOME:-/tmp/simhome}"
# One gz-transport partition and interface for every gz process here,
# including the arm emulator (which runs with a cleaned environment).
export GZ_PARTITION=go2sim GZ_IP=127.0.0.1

echo "[sim] building model..."
python3 "$SIM/sim/build_model.py" --go2 "$SIM/models/go2_description" --d1 "$SIM/models/d1_description" \
    --mesh-root "$SIM/models" --out "$GEN"
sed "s/@RENDER_ENGINE@/${SIM_RENDER_ENGINE}/" "$SIM/worlds/${WORLD}.sdf" > "$GEN/world.sdf"
SPAWN_Z="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["spawn_z"])' "$GEN/sim_params.json")"

# Same DDS settings the laptop session writes (go2-env.sh -> setup-robot-wifi.sh):
# bind to the host address, explicit peer, SPDP multicast only.
cat > "$GEN/cyclonedds.xml" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces><NetworkInterface address="${GO2_HOST_IP}" priority="default" multicast="default" /></Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers><Peer address="${GO2_ROBOT_IP}"/></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

PIDS=()
cleanup() {
    trap '' INT TERM
    echo "[sim] stopping..."
    for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
    sleep 1
    for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

GZ_SERVER_ARGS=(-r -s --render-engine "$SIM_RENDER_ENGINE")
# The lidar and camera are rendered by the server. With a desktop it renders
# through $DISPLAY (GLX) even without the window. --headless-rendering (EGL)
# is only for machines with no display: on the VM's software GL it returns
# lidar scans that are all inf (empty /lidar3d/registered, no map).
[ -n "${DISPLAY:-}" ] || GZ_SERVER_ARGS+=(--headless-rendering)
echo "[sim] starting Gazebo server (${SIM_RENDER_ENGINE}, GUI=${SIM_GUI})..."
gz sim "${GZ_SERVER_ARGS[@]}" "$GEN/world.sdf" &
PIDS+=("$!")

for _ in $(seq 1 120); do
    gz service -l 2>/dev/null | grep -q "^/world/${WORLD}/create$" && break
    kill -0 "${PIDS[0]}" 2>/dev/null || { echo "[sim] ERROR: Gazebo server exited during startup" >&2; exit 1; }
    sleep 0.5
done
gz service -l 2>/dev/null | grep -q "^/world/${WORLD}/create$" || { echo "[sim] ERROR: world never came up" >&2; exit 1; }

# ROS 2 side: Wi-Fi DDS domain, same RMW/config as the laptop container.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="$GO2_RELAY_DOMAIN_ID" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$GEN/cyclonedds.xml"

echo "[sim] spawning go2 at (${SPAWN_X}, ${SPAWN_Y}, ${SPAWN_Z}) yaw ${SPAWN_YAW}"
# flag=value form: a separate negative value ("-x -4.5") can be read as a flag.
ros2 run ros_gz_sim create -world="$WORLD" -file="$GEN/go2_d1.urdf" -name=go2 \
    -x="$SPAWN_X" -y="$SPAWN_Y" -z="$SPAWN_Z" -Y="$SPAWN_YAW"

if [ "$SIM_GUI" = 1 ]; then
    gz sim -g --render-engine "$SIM_RENDER_ENGINE" &
    PIDS+=("$!")
fi

ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SIM/sim/bridge.yaml" &
PIDS+=("$!")
python3 -u "$SIM/sim/sim_robot_bridge.py" --params "$GEN/sim_params.json" &
PIDS+=("$!")

if [ "$SIM_ARM" = 1 ]; then
    if [ -e "/sys/class/net/${SIM_ARM_IFACE}" ]; then
        # No ROS environment: pip cyclonedds must not load ROS's libddsc.
        env -i HOME="$HOME" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
            GZ_PARTITION="$GZ_PARTITION" GZ_IP="$GZ_IP" \
            python3 -u "$SIM/sim/d1_arm_emulator.py" --iface "$SIM_ARM_IFACE" --params "$GEN/sim_params.json" &
        PIDS+=("$!")
    else
        echo "[sim] WARN: ${SIM_ARM_IFACE} not found; D1 emulator not started (run_sim.sh creates it)"
    fi
fi

echo "[sim] all processes started; waiting for the first lidar frame..."
set +e
wait -n "${PIDS[@]}"
status=$?
set -e
echo "[sim] a simulation process exited (status ${status}); shutting down" >&2
exit "$status"
