#!/usr/bin/env python3
"""Simulation stand-in for a trained detector. NOT a vision model.

Gazebo's obstacles are plain boxes and cylinders, which a YOLO model cannot
recognise as cables or debris. This node takes each object's true class and
shape from the world SDF, projects it into the camera using TF, and publishes
the 2D box a perfect detector would, with pixel noise. That tests everything
after the detector (LiDAR fusion, tracking, gait choice) without a custom model.

Objects outside max_range, hidden behind a taller object, or too small in the
image are not reported.

Publishes /objects/detections_json: {"detections": [{class, score, bbox}]}.
With dry_run, detections are logged and nothing is published.
"""
import json
import math
import random
import re
import xml.etree.ElementTree as ET

import numpy as np

CLASS_PREFIXES = ("cable_tray", "cable", "hose", "pipe", "debris", "toolbox", "barrel",
                  "crate", "post", "planter", "barrier", "cone")


def load_objects(sdf_path):
    """Objects from a world's JSON truth file, or parsed from a hand-written SDF."""
    if sdf_path.endswith(".json"):
        return json.load(open(sdf_path))["objects"]
    text = re.sub(r"@\w+@", "ogre2", open(sdf_path).read())
    objects = []
    for model in ET.fromstring(text).iter("model"):
        name = model.get("name")
        cls = next((c for c in CLASS_PREFIXES if name.startswith(c)), None)
        if cls is None:
            continue
        x, y, z, roll, _, yaw = (float(v) for v in model.find("pose").text.split())
        geo = model.find("link/collision/geometry")
        box, cyl = geo.find("box"), geo.find("cylinder")
        if box is not None:
            sx, sy, sz = (float(v) for v in box.find("size").text.split())
        else:
            radius = float(cyl.find("radius").text)
            length = float(cyl.find("length").text)
            lying = abs(abs(roll) - math.pi / 2) < 0.01
            sx, sy, sz = (2 * radius, length, 2 * radius) if lying else (2 * radius, 2 * radius, length)
        objects.append({"name": name, "class": cls, "x": x, "y": y, "z": z, "yaw": yaw,
                        "sx": sx, "sy": sy, "sz": sz, "height": z + sz / 2})
    return objects


def corners(o):
    c, s = math.cos(o["yaw"]), math.sin(o["yaw"])
    out = []
    for dx in (-o["sx"] / 2, o["sx"] / 2):
        for dy in (-o["sy"] / 2, o["sy"] / 2):
            for dz in (-o["sz"] / 2, o["sz"] / 2):
                out.append((o["x"] + c * dx - s * dy, o["y"] + s * dx + c * dy, o["z"] + dz))
    return np.array(out)


def occluded(cam_pos, o, objects):
    """True if a taller object blocks the line from the camera to o's top centre."""
    target = np.array([o["x"], o["y"], o["height"]])
    for other in objects:
        if other is o or other["height"] <= o["height"]:
            continue
        pts = corners(other)
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        d = target - cam_pos
        tmin, tmax = 0.0, 1.0
        for i in range(3):
            if abs(d[i]) < 1e-9:
                if cam_pos[i] < lo[i] or cam_pos[i] > hi[i]:
                    break
                continue
            a, b = sorted(((lo[i] - cam_pos[i]) / d[i], (hi[i] - cam_pos[i]) / d[i]))
            tmin, tmax = max(tmin, a), min(tmax, b)
            if tmin > tmax:
                break
        else:
            return True
    return False


def detect(objects, rot, trans, cam_pos, intrinsics, size=(640, 360), max_range=6.0,
           min_pixels=4, noise_px=3.0, rng=random):
    fx, fy, cx, cy = intrinsics
    out = []
    for o in objects:
        cam = corners(o) @ rot.T + trans
        if cam[:, 0].min() < 0.1 or cam[:, 0].min() > max_range:
            continue
        u = cx - fx * cam[:, 1] / cam[:, 0]
        v = cy - fy * cam[:, 2] / cam[:, 0]
        x1, x2 = max(0.0, u.min()), min(size[0] - 1.0, u.max())
        y1, y2 = max(0.0, v.min()), min(size[1] - 1.0, v.max())
        if x2 - x1 < min_pixels or y2 - y1 < min_pixels or occluded(cam_pos, o, objects):
            continue
        bbox = [v + rng.gauss(0.0, noise_px) for v in (x1, y1, x2, y2)]
        out.append({"class": o["class"], "score": 0.8, "bbox": [round(b, 1) for b in bbox],
                    "truth": o["name"]})
    return out


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    class SimBoxDetector(Node):
        def __init__(self):
            super().__init__("sim_box_detector")
            p = self.declare_parameter
            p("world", "/tmp/trav/cluttered_nav_test.sdf")
            p("rate_hz", 2.0)
            p("map_frame", "lidar3d_map")
            p("camera_frame", "front_camera")
            p("max_range", 6.0)
            p("noise_px", 3.0)
            p("only_classes", "")   # comma list, empty = all: with real YOLO, the classes it lacks
            p("dry_run", False)
            gp = lambda k: self.get_parameter(k).value  # noqa: E731
            only = {c for c in str(gp("only_classes")).split(",") if c}
            self.objects = load_objects(gp("world"))
            self.report = {o["name"] for o in self.objects if not only or o["class"] in only}
            self.map_frame, self.cam_frame = gp("map_frame"), gp("camera_frame")
            self.max_range, self.noise = float(gp("max_range")), float(gp("noise_px"))
            self.dry_run = bool(gp("dry_run"))
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.pub = self.create_publisher(String, "/objects/detections_json", 10)
            self.create_timer(1.0 / float(gp("rate_hz")), self.tick)
            self.get_logger().warn(f"SIM ground-truth detector, not YOLO: {len(self.objects)} objects")

        def tick(self):
            from object_box_fusion import rotation_matrix
            try:
                tf = self.tf.lookup_transform(self.cam_frame, self.map_frame, Time())
            except TransformException:
                return
            rot = rotation_matrix(tf.transform.rotation)
            tr = tf.transform.translation
            trans = np.array([tr.x, tr.y, tr.z])
            cam_pos = -rot.T @ trans
            dets = detect(self.objects, rot, trans, cam_pos, (468.0, 468.0, 320.0, 180.0),
                          max_range=self.max_range, noise_px=self.noise)
            dets = [d for d in dets if d["truth"] in self.report]
            if self.dry_run:
                self.get_logger().info(f"dry_run detections: {len(dets)}", throttle_duration_sec=5.0)
            else:
                self.pub.publish(String(data=json.dumps({"detections": dets})))

    rclpy.init()
    node = SimBoxDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
