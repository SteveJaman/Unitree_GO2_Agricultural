#!/usr/bin/env python3
"""Live VoxelLayer check: synthetic 3D obstacles -> raw costmap cost, with ROS 2.

  voxel_costmap_check.py LOCAL_COSTMAP_PARAMS_YAML OUT_JSON

LOCAL_COSTMAP_PARAMS_YAML holds {'local_costmap': {'local_costmap': {'ros__parameters':
...}}}, e.g. what lidar3d_controller.launch.py writes after nav2_voxel_layer.py
--enable. Only the obstacle layer's lidar source is kept (pointed at
/voxeltest/points, sensor_frame base_link); a standalone nav2_costmap_2d node runs
it, a static lidar3d_map -> base_link TF puts the robot at the origin, and a cloud
of objects 1.5 m ahead is published. Reports the highest raw cost on each object's
cells and the occupied voxels in /costmap/voxel_grid. Run on its own
ROS_DOMAIN_ID.
"""
import json
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav2_msgs.msg import Costmap, VoxelGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

TOPIC = "/voxeltest/points"
X = 1.5
# name, y, list of z values for a 0.2 x 0.2 m column of points (only these heights)
OBJECTS = [
    ("table top at 0.75 m (no legs)", -1.2, [0.73, 0.75, 0.77]),
    ("1.4 m tall object", -0.6, [0.2 + 0.1 * k for k in range(13)]),
    ("single point at 0.5 m", 0.0, None),
    ("0.10 m item (below 0.15 m floor cut)", 0.6, [0.05, 0.10]),
    ("ceiling at 2.0 m (above the grid)", 1.2, [2.0]),
]


def cloud(stamp):
    pts = []
    for _, y0, zs in OBJECTS:
        if zs is None:
            pts.append((X, y0, 0.5))
            continue
        for i in range(5):
            for j in range(5):
                for z in zs:
                    pts.append((X - 0.1 + 0.05 * i, y0 - 0.1 + 0.05 * j, z))
    m = PointCloud2()
    m.header.frame_id, m.header.stamp = "lidar3d_map", stamp
    m.height, m.width = 1, len(pts)
    m.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1) for k, n in enumerate("xyz")]
    m.point_step, m.row_step, m.is_dense = 12, 12 * len(pts), True
    m.data = b"".join(struct.pack("<fff", *p) for p in pts)
    return m


def main():
    src, out = sys.argv[1:3]
    cfg = yaml.safe_load(open(src))["local_costmap"]["local_costmap"]["ros__parameters"]
    cfg["plugins"] = [p for p in cfg["plugins"] if p in ("obstacle_layer",)]
    ob = cfg["obstacle_layer"]
    for s in ob["observation_sources"].split():
        if s != "lidar":
            ob.pop(s, None)
    ob["observation_sources"] = "lidar"
    ob["lidar"].update(topic=TOPIC, sensor_frame="base_link", expected_update_rate=0.0)
    cfg["update_frequency"] = 5.0
    f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    yaml.safe_dump({"costmap": {"costmap": {"ros__parameters": cfg}}}, f)
    f.close()
    proc = subprocess.Popen(["ros2", "run", "nav2_costmap_2d", "nav2_costmap_2d", "--ros-args", "--params-file", f.name],
                            stdout=open(out + ".log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node("voxel_check_pub")
    tfb = StaticTransformBroadcaster(node)
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = "lidar3d_map", "base_link"
    t.header.stamp = node.get_clock().now().to_msg()
    t.transform.rotation.w = 1.0
    tfb.sendTransform(t)
    pub = node.create_publisher(PointCloud2, TOPIC, qos_profile_sensor_data)
    got = {"voxels": 0}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Costmap, "/costmap/costmap_raw", lambda m: got.update(m=m), qos)
    # publish_voxel_map publishes nav2_msgs/VoxelGrid on <costmap>/voxel_grid (RViz needs
    # nav2_costmap_2d_cloud to turn it into a point cloud).
    node.create_subscription(VoxelGrid, "/costmap/voxel_grid",
                             lambda m: got.update(voxels=max(got["voxels"], sum(bin(v >> 16).count("1") for v in m.data))), 10)
    try:
        time.sleep(3)
        for state in ("configure", "activate"):
            subprocess.run(["ros2", "lifecycle", "set", "/costmap/costmap", state], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        end = time.time() + 15
        while time.time() < end:
            pub.publish(cloud(node.get_clock().now().to_msg()))
            rclpy.spin_once(node, timeout_sec=0.2)
        m = got.get("m")
        if m is None:
            raise SystemExit("no /costmap/costmap_raw; see " + out + ".log")
        res, w = m.metadata.resolution, m.metadata.size_x
        ox, oy = m.metadata.origin.position.x, m.metadata.origin.position.y
        rows = []
        for name, y0, _ in OBJECTS:
            best = 0
            for i in range(int((X - 0.1 - ox) / res), int((X + 0.1 - ox) / res) + 1):
                for j in range(int((y0 - 0.1 - oy) / res), int((y0 + 0.1 - oy) / res) + 1):
                    best = max(best, m.data[j * w + i])
            rows.append({"object": name, "max_raw_cost": best})
        result = {"plugin": ob["plugin"], "z_resolution": ob.get("z_resolution"), "z_voxels": ob.get("z_voxels"),
                  "mark_threshold": ob.get("mark_threshold"), "objects": rows,
                  "voxel_grid_marked_voxels": got["voxels"]}
        json.dump(result, open(out, "w"), indent=1)
        print(json.dumps(result, indent=1))
    finally:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        os.unlink(f.name)


if __name__ == "__main__":
    main()
