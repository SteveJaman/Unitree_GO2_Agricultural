#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# run_phase1_test.sh: Phase 1 acceptance test for HeightBandLayer.
#
# Requires: go2-lidar3d + go2-sim containers running with Nav2 + HeightBandLayer.
# Injects synthetic elevated PointCloud2 on /lidar3d/obstacle_points,
# then checks the global_costmap for the expected costs.
#
# Usage (from WSL, with containers up):
#   ./tests/run_phase1_test.sh
# ---------------------------------------------------------------------------
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONTAINER="${CONTAINER_NAME:-go2-lidar3d}"

DDS_ENV='source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file:///ws/log/3d/cyclonedds.xml && export ROS_DOMAIN_ID=64'

echo "=== Phase 1 HeightBandLayer Acceptance Test ==="

# Verify container is running
if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
    echo "ERROR: ${CONTAINER} is not running."
    exit 1
fi

# Copy source and rebuild plugin
echo "[1/3] Building semantic_traversability_layer..."
cp -r "${REPO_DIR}/src/semantic_traversability_layer" /tmp/semantic_traversability_layer
docker cp /tmp/semantic_traversability_layer "${CONTAINER}:/ws/src/semantic_traversability_layer"
rm -rf /tmp/semantic_traversability_layer

docker exec "${CONTAINER}" bash -c "${DDS_ENV} && cd /ws && colcon build --packages-select semantic_traversability_layer --symlink-install --paths src/semantic_traversability_layer 2>&1" | tail -5
echo "[1/3] Build OK."

# Lifecycle restart the global costmap to load the new plugin
echo "[2/3] Restarting global_costmap..."
docker exec "${CONTAINER}" bash -c "${DDS_ENV} && \
    ros2 lifecycle set /global_costmap/global_costmap deactivate --no-daemon 2>/dev/null && sleep 1 && \
    ros2 lifecycle set /global_costmap/global_costmap cleanup --no-daemon 2>/dev/null && sleep 1 && \
    ros2 lifecycle set /global_costmap/global_costmap configure --no-daemon 2>/dev/null && sleep 1 && \
    ros2 lifecycle set /global_costmap/global_costmap activate --no-daemon 2>/dev/null"
echo "[2/3] Global costmap restarted."

# Inject elevated points and check costmap
echo "[3/3] Running acceptance test..."

# Copy the injector script
cat > /tmp/inject_test.py << 'PYEOF'
#!/usr/bin/env python3
import struct, time, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry

SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
INJECT = [
    ("step_over", 1.5, 0.0, 0.10),
    ("caution",   1.5, 2.0, 0.20),
    ("lethal",    1.5, -2.0, 0.30),
]

def make_cloud(stamp, frame_id, points):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    buf = bytearray()
    for x, y, z in points:
        buf.extend(struct.pack("<fff", x, y, z))
    msg.data = bytes(buf)
    return msg

class Injector(Node):
    def __init__(self):
        super().__init__("hbl_test_injector")
        self.odom = None
        self.pub = self.create_publisher(PointCloud2, "/lidar3d/obstacle_points", SENSOR_QOS)
        self.create_subscription(Odometry, "/lidar3d/odom", self._cb, SENSOR_QOS)
    def _cb(self, msg):
        self.odom = msg

def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    n = Injector()
    t0 = time.time()
    while n.odom is None and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=1.0)
    if n.odom is None:
        print("ERROR: no odom"); n.destroy_node(); rclpy.shutdown(); sys.exit(1)
    rx, ry = n.odom.pose.pose.position.x, n.odom.pose.pose.position.y
    pts = [(rx+dx, ry+dy, h) for _, dx, dy, h in INJECT]
    t0 = time.time()
    while time.time() - t0 < dur:
        n.pub.publish(make_cloud(n.get_clock().now().to_msg(), "lidar3d_map", pts))
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
PYEOF

cat > /tmp/check_test.py << 'PYEOF'
#!/usr/bin/env python3
import time, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry

COSTMAP_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
CHECKS = [
    ("step_over", 1.5, 0.0, 0.10, 10, 60),
    ("caution",   1.5, 2.0, 0.20, 60, 100),
    ("lethal",    1.5, -2.0, 0.30, 99, 100),
]

class C(Node):
    def __init__(self):
        super().__init__("hbl_test_checker")
        self.cm = None; self.odom = None
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._cm, COSTMAP_QOS)
        self.create_subscription(Odometry, "/lidar3d/odom", self._od, SENSOR_QOS)
    def _cm(self, m): self.cm = m
    def _od(self, m): self.odom = m
    def cost(self, wx, wy, r=3):
        if not self.cm: return -1
        i = self.cm.info
        cx, cy = int((wx-i.origin.position.x)/i.resolution), int((wy-i.origin.position.y)/i.resolution)
        best = 0
        for dx in range(-r, r+1):
            for dy in range(-r, r+1):
                ix, iy = cx+dx, cy+dy
                if 0<=ix<i.width and 0<=iy<i.height:
                    c = self.cm.data[iy*i.width+ix]
                    if c > best: best = c
        return best

def main():
    rclpy.init(); n = C()
    t0 = time.time()
    while (not n.odom or not n.cm) and time.time()-t0 < 15:
        rclpy.spin_once(n, timeout_sec=1.0)
    if not n.odom or not n.cm:
        print("ERROR: no data"); n.destroy_node(); rclpy.shutdown(); sys.exit(1)
    for _ in range(10): rclpy.spin_once(n, timeout_sec=0.5)
    rx, ry = n.odom.pose.pose.position.x, n.odom.pose.pose.position.y
    i = n.cm.info
    print("=" * 60)
    print("HeightBandLayer Phase 1 Acceptance Test")
    print("=" * 60)
    print("Costmap: %dx%d @ %.3fm  Robot: (%.2f, %.2f)" % (i.width, i.height, i.resolution, rx, ry))
    ok_all = True
    for name, dx, dy, h, lo, hi in CHECKS:
        c = n.cost(rx+dx, ry+dy)
        ok = lo <= c <= hi
        if not ok: ok_all = False
        print("  [%s] %-12s h=%.2f expect=[%d,%d] actual=%d" % ("PASS" if ok else "FAIL", name, h, lo, hi, c))
    nz = sum(1 for c in n.cm.data if c > 0)
    print("\nNon-zero cells: %d / %d" % (nz, len(n.cm.data)))
    print("\nRESULT: %s" % ("ALL CHECKS PASSED" if ok_all else "SOME CHECKS FAILED"))
    print("=" * 60)
    n.destroy_node(); rclpy.shutdown(); sys.exit(0 if ok_all else 1)

if __name__ == "__main__":
    main()
PYEOF

docker cp /tmp/inject_test.py "${CONTAINER}:/tmp/inject_test.py"
docker cp /tmp/check_test.py "${CONTAINER}:/tmp/check_test.py"

# Run injector in background (60s)
docker exec -d "${CONTAINER}" bash -c "${DDS_ENV} && python3 /tmp/inject_test.py 60"
sleep 10

# Run checker
docker exec "${CONTAINER}" bash -c "${DDS_ENV} && python3 /tmp/check_test.py"
EXIT_CODE=$?

echo ""
if [ $EXIT_CODE -eq 0 ]; then
    echo "=== Phase 1 Test: PASSED ==="
else
    echo "=== Phase 1 Test: FAILED ==="
fi
exit $EXIT_CODE
