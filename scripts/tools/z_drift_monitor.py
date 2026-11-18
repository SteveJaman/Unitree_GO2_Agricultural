#!/usr/bin/env python3
"""Watch the dog's height live: Point-LIO's raw z against the levelled z RViz uses.

Run inside go2-lidar3d while wifi_3d_nav.sh is up (docs/hardware/z_drift.md has the walk test):

  docker cp scripts/tools/z_drift_monitor.py go2-lidar3d:/tmp/
  docker exec -it go2-lidar3d bash -c "source /opt/ros/humble/setup.bash && python3 /tmp/z_drift_monitor.py"

One line a second:
  raw z       /lidar3d/raw/odom: Point-LIO as it comes off the dog (drifts)
  level z     /lidar3d/odom: after go2_lidar3d_level.py (what Nav2 and the costmap use)
  base z      TF lidar3d_map -> base_link (what RViz and the web app draw), with its tilt
Each with its change since the first reading. "raw: no data" means the levelling is off
(GO2_LIDAR3D_RAW not reaching the dog): the session then shows Point-LIO's drift.
"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

TOPICS = {"raw": "/lidar3d/raw/odom", "level": "/lidar3d/odom"}


def roll_pitch_deg(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return math.degrees(roll), math.degrees(pitch)


class Monitor(Node):
    def __init__(self, period):
        super().__init__("z_drift_monitor")
        self.last = {}
        self.first = {}
        for key, topic in TOPICS.items():
            self.create_subscription(
                Odometry, topic, lambda m, k=key: self.last.__setitem__(k, m.pose.pose.position.z), qos_profile_sensor_data
            )
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.t0 = time.time()
        self.create_timer(period, self.report)

    def field(self, key, value):
        if value is None:
            return f"{key} z: no data      "
        self.first.setdefault(key, value)
        return f"{key} z {value:+.3f} ({value - self.first[key]:+.3f})"

    def report(self):
        base = None
        tilt = ""
        where = ""
        try:
            t = self.tf.lookup_transform("lidar3d_map", "base_link", rclpy.time.Time())
            base = t.transform.translation.z
            where = f" at ({t.transform.translation.x:+.2f}, {t.transform.translation.y:+.2f})"
            r, p = roll_pitch_deg(t.transform.rotation)
            tilt = f" tilt {r:+5.1f}/{p:+5.1f} deg"
        except Exception:  # noqa: BLE001  (no TF yet)
            pass
        print(
            f"t {time.time() - self.t0:6.1f}s | "
            + " | ".join(self.field(k, self.last.get(k)) for k in TOPICS)
            + " | "
            + self.field("base", base)
            + tilt
            + where,
            flush=True,
        )


def main():
    period = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    rclpy.init()
    node = Monitor(period)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
