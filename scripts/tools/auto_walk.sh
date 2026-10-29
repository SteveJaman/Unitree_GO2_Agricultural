#!/usr/bin/env bash
# auto_walk.sh — walk the Go2 in a square loop via Nav2.
#
# PREREQUISITES (must be running before you launch this):
#   1. Dog standing, powered on
#   2. Jetson: no leftover relay  → ssh unitree@192.168.1.9 'pkill -9 -f relay; pkill -9 -f go2_cmd_vel'
#   3. Host: ./mapping.sh --3d --nav  is running and RViz shows a live map
#   4. Verify: docker exec -u root go2-lidar3d bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash 2>/dev/null; ros2 run tf2_ros tf2_echo lidar3d_map base_link' — must show pose updates
set -eo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-go2-lidar3d}"
SIDE="${SIDE:-1.0}"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "ERROR: container $CONTAINER_NAME not running. Run ./mapping.sh --3d --nav first."
    exit 1
fi

cat > /tmp/auto_walk.py <<PYEOF
#!/usr/bin/env python3
import math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

MAP_FRAME  = 'lidar3d_map'
BASE_FRAME = 'base_link'
GOAL_TOPIC = '/goal_pose'
SIDE       = float("${SIDE}")
TOL        = 0.40
GOAL_TIMEOUT  = 60.0
STALL_TIMEOUT = 20.0
PAUSE         = 2.0

class Walk(Node):
    def __init__(self):
        super().__init__('auto_walk')
        self.pub = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)

    def pose(self):
        try:
            t = self.tf.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def send(self, x, y):
        m = PoseStamped()
        m.header.frame_id = MAP_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.w = 1.0
        self.pub.publish(m)
        self.get_logger().info(f'goal -> ({x:.2f}, {y:.2f})')

    def wait(self, x, y):
        t0 = time.time(); last = None; last_t = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            p = self.pose()
            if p is None:
                continue
            if math.hypot(p[0]-x, p[1]-y) < TOL:
                self.get_logger().info(f'reached ({x:.2f}, {y:.2f})')
                return True
            if last is None:
                last, last_t = p, time.time()
            elif math.hypot(p[0]-last[0], p[1]-last[1]) > 0.15:
                last, last_t = p, time.time()
            if time.time() - last_t > STALL_TIMEOUT:
                self.get_logger().warn('stalled, next waypoint')
                return False
            if time.time() - t0 > GOAL_TIMEOUT:
                self.get_logger().warn('timeout, next waypoint')
                return False
        return False

def main():
    rclpy.init()
    n = Walk()
    n.get_logger().info('waiting for TF lidar3d_map -> base_link (up to 60s)...')
    origin = None; t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.2)
        origin = n.pose()
        if origin:
            break
    if not origin:
        n.get_logger().error('no TF. Is mapping.sh --3d --nav running? Is the dog standing?')
        return 1
    ox, oy = origin
    n.get_logger().info(f'origin = ({ox:.2f}, {oy:.2f})')
    wp = [(ox+SIDE, oy), (ox+SIDE, oy+SIDE), (ox, oy+SIDE), (ox, oy)]
    lap = 0
    try:
        while rclpy.ok():
            lap += 1
            n.get_logger().info(f'=== lap {lap} ===')
            for x, y in wp:
                if not rclpy.ok():
                    break
                n.send(x, y)
                n.wait(x, y)
                time.sleep(PAUSE)
    except KeyboardInterrupt:
        pass
    finally:
        n.get_logger().info('stopped')
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main())
PYEOF

docker cp /tmp/auto_walk.py "$CONTAINER_NAME:/tmp/auto_walk.py" >/dev/null

trap 'echo; echo "[auto_walk] Ctrl+C — stopping node."; docker exec -u root "$CONTAINER_NAME" pkill -9 -f auto_walk.py 2>/dev/null || true' EXIT INT TERM

echo "[auto_walk] starting. Remote in hand as e-stop. Ctrl+C to stop."
echo
docker exec -u root -it "$CONTAINER_NAME" bash -lc '
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash 2>/dev/null
    exec python3 /tmp/auto_walk.py
'
