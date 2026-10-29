#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# test_sim.sh: automated smoke test of the Gazebo simulation (on the host).
#
# Uses the real go2-lidar3d container and the real arm scripts:
#   1. starts the simulation (run_sim.sh up)
#   2. robot topics reach the container on the Wi-Fi DDS domain
#   3. motion: drives 0.3 m/s for 3 s through TCP 17999 and checks /lidar3d/odom moved
#   4. arm: start_arm.sh read, start_arm.sh fold, and a joint move with feedback
#
#   ./gazebo/test_sim.sh            # leaves the simulation running
#   ./gazebo/test_sim.sh --down     # stops it at the end
# ---------------------------------------------------------------------------
set -uo pipefail

GZ_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${GZ_DIR}/.." && pwd)"
CONTAINER="${CONTAINER_NAME:-go2-lidar3d}"
SIM_ARM_IFACE="${SIM_ARM_IFACE:-simarm0}"
SIM_ARM_SUBNET="${SIM_ARM_SUBNET:-192.168.123}"
DOWN=0; [ "${1:-}" = --down ] && DOWN=1
PASS=0; FAIL=0
ok()  { echo "  PASS  $*"; PASS=$((PASS + 1)); }
bad() { echo "  FAIL  $*"; FAIL=$((FAIL + 1)); }

echo "=== 1. simulation up ==="
SIM_GUI="${SIM_GUI:-0}" "$GZ_DIR/run_sim.sh" up || { echo "simulation did not start"; exit 1; }
HOST_IP="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' go2-sim | sed -n 's/^GO2_HOST_IP=//p')"
echo "  sim DDS address: ${HOST_IP}"

docker ps -a --format '{{.Names}}' | grep -x "$CONTAINER" >/dev/null || { echo "container ${CONTAINER} missing (run wifi_nav.sh --sim once, or setup-lidar3d.sh --laptop)"; exit 1; }
docker start "$CONTAINER" >/dev/null

# Laptop-side ROS environment exactly as a session gets it (go2-env.sh), pointed at the sim.
ros_exec() {
    docker exec -i -e GO2_HOST_IP="$HOST_IP" -e GO2_ROBOT_IP="$HOST_IP" -e GO2_RELAY_DOMAIN_ID=64 \
        "$CONTAINER" bash -c "source /ws/scripts/go2-env.sh >/dev/null 2>&1; $1"
}

echo "=== 2. robot topics reach ${CONTAINER} (domain 64) ==="
# The first check used to fail on /lidar3d/odom while the CLI daemon was still
# discovering the sim (the motion test below reads odom fine). Wait for
# discovery first, and read the sensor-QoS topics as best effort.
ros_exec "for i in \$(seq 1 30); do ros2 topic list 2>/dev/null | grep -qx /lidar3d/odom && exit 0; sleep 1; done; exit 1" \
    || echo "  (warn) /lidar3d/odom not discovered after 30 s"
for t in /lidar3d/odom /lidar3d/registered /lidar3d/path /joint_states /camera/image_raw; do
    qos=""
    case "$t" in /lidar3d/odom|/lidar3d/registered) qos="--qos-reliability best_effort" ;; esac
    if ros_exec "timeout 20 ros2 topic echo --once $qos $t >/dev/null 2>&1"; then ok "$t"; else bad "$t (nothing within 20 s)"; fi
done

echo "=== 3. motion through TCP 17999 ==="
moved="$(ros_exec "python3 - <<'PY'
import socket, struct, time, rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
rclpy.init(); n = rclpy.create_node('sim_motion_check'); last = {}
n.create_subscription(Odometry, '/lidar3d/odom', lambda m: last.update(p=m.pose.pose.position), qos_profile_sensor_data)
def pos(t=10):
    last.clear(); end = time.time() + t
    while 'p' not in last and time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
    return last.get('p')
p0 = pos()
s = socket.create_connection(('${HOST_IP}', 17999), timeout=5)
for _ in range(60):
    s.sendall(struct.pack('!fff', 0.3, 0.0, 0.0)); time.sleep(0.05)
s.close(); time.sleep(0.8)
p1 = pos()
print('NA' if p0 is None or p1 is None else f'{((p1.x-p0.x)**2 + (p1.y-p0.y)**2) ** 0.5:.2f}')
PY" 2>/dev/null | tail -1)"
if [ "$moved" = NA ] || [ -z "$moved" ]; then bad "no /lidar3d/odom to measure motion"
elif python3 -c "import sys; sys.exit(0 if float('$moved') > 0.5 else 1)"; then ok "robot moved ${moved} m for a 0.9 m command (0.3 m/s x 3 s)"
else bad "robot moved only ${moved} m (expected ~0.9)"; fi

echo "=== 4. D1 arm (real scripts, ARM_IFACE=${SIM_ARM_IFACE}) ==="
if ! ip link show dev "$SIM_ARM_IFACE" >/dev/null 2>&1; then
    bad "${SIM_ARM_IFACE} missing (SIM_ARM=0?)"
else
    export ARM_IFACE="$SIM_ARM_IFACE" ARM_IP="${SIM_ARM_SUBNET}.100" ARM_HOST_IP="${SIM_ARM_SUBNET}.51"
    out="$(bash "$REPO_DIR/scripts/runtime/start_arm.sh" read 2>&1)"
    if grep -q "^FOLDED_ANGLES = \[" <<< "$out"; then ok "start_arm.sh read: $(grep '^FOLDED_ANGLES' <<< "$out")"
    else bad "start_arm.sh read"; echo "$out" | tail -5 | sed 's/^/        /'; fi
    out="$(bash "$REPO_DIR/scripts/runtime/start_arm.sh" fold 2>&1)"
    if grep -q "^\[+\] Done" <<< "$out"; then ok "start_arm.sh fold"
    else bad "start_arm.sh fold"; echo "$out" | tail -5 | sed 's/^/        /'; fi
    out="$(bash "$REPO_DIR/scripts/runtime/start_arm.sh" enable sleep 1 joint 0 20 sleep 3 feedback 2>&1)"
    if grep -q "angles: {" <<< "$out"; then ok "d1_arm_control.py joint + feedback: $(grep 'angles' <<< "$out" | tail -1 | sed 's/^\[d1\] //')"
    else bad "d1_arm_control.py joint + feedback"; echo "$out" | tail -5 | sed 's/^/        /'; fi
fi

echo
echo "TOTAL: PASS=${PASS} FAIL=${FAIL}"
[ "$DOWN" = 1 ] && "$GZ_DIR/run_sim.sh" down
[ "$FAIL" = 0 ]
