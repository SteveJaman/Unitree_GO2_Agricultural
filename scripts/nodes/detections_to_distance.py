#!/usr/bin/env python3
"""Bridge the RealSense 3D detector to the arm servo: /detections_3d -> /object_distance.

The detector from the test branch (src/object_detection_rgbd, scripts/runtime/plant_detection.sh)
publishes vision_msgs/Detection3DArray on /detections_3d, in the camera's optical frame
(z = straight out of the lens), on the camera's domain (64). distance_servo.py wants a
std_msgs/Float32 distance in metres on /object_distance, on MoveIt's domain (78).

Each array: the wanted detection (target_class, '' = any; at least min_score), the nearest
one if several, its box centre's z (depth along the camera axis, what yolo_depth_tracker.py
measured too) -> /object_distance. Nothing is published when nothing wanted is seen, which
distance_servo.py treats as "stop" after stale_s.

  python3 detections_to_distance.py --ros-args -p target_class:="potted plant" -p out_domain_id:=78

Run it with the camera's DDS setup (domain 64); with out_domain_id >= 0 a second rclpy
context publishes on that domain (arm_servo_test.sh detector does both, with one Cyclone
config for 64 on the Wi-Fi and 78 on loopback). Parameters: target_class (''), min_score
(0.4), measure ("z" along the camera axis, or "range": the straight-line distance),
out_domain_id (78), report_s (10).
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from vision_msgs.msg import Detection3DArray


def pick(detections, target, min_score):
    """(class, score, x, y, z) of the nearest wanted detection, or None."""
    best = None
    for det in detections:
        if not det.results:
            continue
        hyp = det.results[0].hypothesis
        if hyp.score < min_score or (target and hyp.class_id != target):
            continue
        p = det.bbox.center.position
        if not (math.isfinite(p.z) and p.z > 0.0):
            continue
        if best is None or p.z < best[4]:
            best = (hyp.class_id, hyp.score, p.x, p.y, p.z)
    return best


class DetectionsToDistance(Node):
    def __init__(self):
        super().__init__("detections_to_distance")
        d = self.declare_parameter
        d("target_class", "")
        d("min_score", 0.4)
        d("measure", "z")
        d("out_domain_id", 78)
        d("report_s", 10.0)
        self.p = lambda n: self.get_parameter(n).value
        if self.p("measure") not in ("z", "range"):
            raise ValueError('measure must be "z" or "range"')

        out_domain = int(self.p("out_domain_id"))
        self.out_ctx = self.out_node = None
        if out_domain >= 0:
            self.out_ctx = rclpy.Context()
            rclpy.init(context=self.out_ctx, domain_id=out_domain)
            self.out_node = Node("detections_to_distance_out", context=self.out_ctx)
            self.pub = self.out_node.create_publisher(Float32, "/object_distance", 10)
        else:
            self.pub = self.create_publisher(Float32, "/object_distance", 10)
        self.create_subscription(
            Detection3DArray, "/detections_3d", self.on_detections, 10
        )
        self.arrays = self.sent = 0
        self.last = None
        self.create_timer(float(self.p("report_s")), self.report)
        target = self.p("target_class").strip()
        self.get_logger().info(
            f'/detections_3d -> /object_distance ({self.p("measure")}, target "{target or "any"}", '
            f'score >= {self.p("min_score")}) on domain {out_domain if out_domain >= 0 else "of the camera"}'
        )

    def on_detections(self, msg):
        self.arrays += 1
        best = pick(
            msg.detections, self.p("target_class").strip(), float(self.p("min_score"))
        )
        if best is None:
            return
        cls, score, x, y, z = best
        dist = z if self.p("measure") == "z" else math.sqrt(x * x + y * y + z * z)
        self.pub.publish(Float32(data=float(dist)))
        self.sent += 1
        self.last = f"{cls} {score:.2f} at {dist:.3f} m"

    def report(self):
        self.get_logger().info(
            f"{self.arrays} detection arrays, {self.sent} distances sent"
            + (f"; last {self.last}" if self.last else "")
        )
        self.arrays = self.sent = 0

    def shutdown(self):
        if self.out_node is not None:
            self.out_node.destroy_node()
            rclpy.try_shutdown(context=self.out_ctx)


def main():
    rclpy.init()
    node = DetectionsToDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
