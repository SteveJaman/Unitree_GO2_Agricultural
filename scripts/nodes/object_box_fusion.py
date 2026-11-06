#!/usr/bin/env python3
"""Camera + LiDAR fusion: a 2D detection becomes a 3D box with a height.

For each detection, the last cloud_history self-filtered scans are projected
into the image. The points inside the box and above the floor are clustered in
the floor plane, and the cluster spanning most of the box gives the object's
size and position. Height above the floor decides how the dog can handle it:

  step_over  height <= step_over_height_max
  caution    height <  lethal_height_min
  blocked    height >= lethal_height_min
  unknown    seen, but the LiDAR points do not explain the detection

Detections of one object are merged into a world-fixed track, so the dog still
knows about an object once it has left the camera's view under the nose.

Reads the YOLO node's output, and does not modify it:
  /yolo/detections            vision_msgs/Detection2DArray, when vision_msgs exists
  /objects/detections_json    std_msgs/String, same content (sim_box_detector.py)
  /lidar3d/obstacle_points    sensor_msgs/PointCloud2, map frame
  /camera/camera_info         sensor_msgs/CameraInfo, else the fx.. parameters

Publishes /objects/boxes_json (std_msgs/String): {"stamp", "boxes": [...]}.
With dry_run, boxes are logged and nothing is published.
"""
import json
import math

import numpy as np
from scipy.spatial import cKDTree


def classify_height(height, step_over_max=0.15, lethal_min=0.25):
    if height <= step_over_max:
        return "step_over"
    return "caution" if height < lethal_min else "blocked"


def project(points_cam, fx, fy, cx, cy):
    """Camera frame is x forward, y left, z up. Returns pixels u, v."""
    depth = points_cam[:, 0]
    return cx - fx * points_cam[:, 1] / depth, cy - fy * points_cam[:, 2] / depth


def clusters(points_map, radius):
    """Connected groups of points in the floor plane, as index arrays."""
    tree = cKDTree(points_map[:, :2])
    seen = np.zeros(len(points_map), bool)
    out = []
    for start in range(len(points_map)):
        if seen[start]:
            continue
        seen[start] = True
        group, todo = [], [start]
        while todo:
            i = todo.pop()
            group.append(i)
            for j in tree.query_ball_point(points_map[i, :2], radius):
                if not seen[j]:
                    seen[j] = True
                    todo.append(j)
        out.append(np.array(group))
    return out


def best_cluster(points_map, u, bbox, radius):
    """The cluster spanning most of the box's width: a smaller object in front
    of the detected one spans less of it."""
    width = max(bbox[2] - bbox[0], 1.0)
    fill = [(float(np.ptp(u[g])) / width, g) for g in clusters(points_map, radius)]
    return max(fill, key=lambda f: f[0])


def ground_point(bbox, intrinsics, rotation, translation, floor_z):
    """Where the ray through the box's bottom centre meets the floor, or None."""
    fx, fy, cx, cy = intrinsics
    u, v = (bbox[0] + bbox[2]) / 2, bbox[3]
    ray_map = rotation.T @ np.array([1.0, -(u - cx) / fx, -(v - cy) / fy])
    origin = -rotation.T @ translation
    if ray_map[2] > -1e-3:
        return None
    return origin + (floor_z - origin[2]) / ray_map[2] * ray_map


def oriented_box(xy):
    """(center_x, center_y, size_x, size_y, yaw): principal axis in the floor plane."""
    mean = xy.mean(axis=0)
    if len(xy) < 3:
        yaw = 0.0
    else:
        vals, vecs = np.linalg.eigh(np.cov((xy - mean).T))
        axis = vecs[:, int(np.argmax(vals))]
        yaw = math.atan2(axis[1], axis[0])
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, s], [-s, c]])
    local = (xy - mean) @ rot.T
    lo, hi = local.min(axis=0), local.max(axis=0)
    center = mean + rot.T @ ((lo + hi) / 2)
    return center[0], center[1], hi[0] - lo[0], hi[1] - lo[1], yaw


def fuse_detection(points_map, points_cam, intrinsics, bbox, floor_z, min_points=4,
                   ground_tolerance=0.02, max_range=6.0, cluster_radius=0.25, pad_px=2.0,
                   min_fill=0.4, max_top_gap=0.3, floor_range=None):
    """One detection -> a 3D box dict, or {"traverse": "unknown", ...}.

    A measurement must explain the detection: the cluster must span min_fill of
    the box's width, reach within max_top_gap of its top edge, and sit at about
    floor_range, the distance of the box's bottom edge on the floor. Otherwise the
    height is unknown, never low: the points may be another object entirely, or
    only the bottom of this one.
    """
    x1, y1, x2, y2 = bbox
    ahead = (points_cam[:, 0] > 0.1) & (points_cam[:, 0] < max_range)
    pts, cam = points_map[ahead], points_cam[ahead]
    if len(pts) == 0:
        return {"traverse": "unknown", "n_points": 0, "reason": "no_points"}
    u, v = project(cam, *intrinsics)
    inside = (u >= x1 - pad_px) & (u <= x2 + pad_px) & (v >= y1 - pad_px) & (v <= y2 + pad_px)
    inside &= pts[:, 2] > floor_z + ground_tolerance
    pts, u, v, depth = pts[inside], u[inside], v[inside], cam[:, 0][inside]
    if len(pts) < min_points:
        return {"traverse": "unknown", "n_points": int(len(pts)), "reason": "too_few_points"}
    fill, keep = best_cluster(pts, u, bbox, cluster_radius)
    pts, v, depth = pts[keep], v[keep], depth[keep]
    if len(pts) < min_points:
        return {"traverse": "unknown", "n_points": int(len(pts)), "reason": "too_few_points"}
    if fill < min_fill:
        return {"traverse": "unknown", "n_points": int(len(pts)), "reason": "small_in_box"}
    if (float(v.min()) - y1) / max(y2 - y1, 1.0) > max_top_gap:
        return {"traverse": "unknown", "n_points": int(len(pts)), "reason": "top_not_seen"}
    if floor_range is not None and abs(float(np.median(depth)) - floor_range) > max(0.4, 0.2 * floor_range):
        return {"traverse": "unknown", "n_points": int(len(pts)), "reason": "wrong_range"}
    x, y, sx, sy, yaw = oriented_box(pts[:, :2])
    top = float(pts[:, 2].max())
    return {"x": float(x), "y": float(y), "z": (top + floor_z) / 2, "sx": float(sx), "sy": float(sy),
            "sz": top - floor_z, "yaw": yaw, "height": top - floor_z, "n_points": int(len(pts))}


class Tracks:
    """World-fixed objects. Same class and close together is the same object;
    a track is published once it has been seen min_obs times."""

    def __init__(self, merge_dist=0.7, step_over_max=0.15, lethal_min=0.25, history=9, min_obs=2):
        self.merge, self.step, self.lethal, self.history, self.min_obs = (
            merge_dist, step_over_max, lethal_min, history, min_obs)
        self.items = []
        self.next_id = 0

    def radius(self, t):
        return max(self.merge, max(t["sx"], t["sy"]) / 2 + 0.3)

    def find(self, cls, x, y):
        best, dist = None, 1e9
        for t in self.items:
            d = math.hypot(t["x"] - x, t["y"] - y)
            if t["class"] == cls and d < self.radius(t) and d < dist:
                best, dist = t, d
        return best

    def new(self, cls, x, y):
        t = {"id": self.next_id, "class": cls, "x": x, "y": y, "z": 0.0, "sx": 0.0, "sy": 0.0, "sz": 0.0,
             "yaw": 0.0, "heights": [], "n_obs": 0, "n_points": 0, "height": None, "traverse": "unknown"}
        self.next_id += 1
        self.items.append(t)
        return t

    def update(self, cls, score, box, now):
        t = self.find(cls, box["x"], box["y"]) or self.new(cls, box["x"], box["y"])
        t["heights"] = (t["heights"] + [box["height"]])[-self.history:]
        t["n_obs"] += 1
        t.update({k: box[k] for k in ("x", "y", "z", "sx", "sy", "sz", "yaw", "n_points")})
        t.update(score=float(score), last_seen=now)
        self.merge_duplicates(t)
        self.rate(t)

    def update_unknown(self, cls, score, x, y, now):
        """Seen but not measured: placed on the floor, height stays unknown."""
        t = self.find(cls, x, y) or self.new(cls, x, y)
        t["n_obs"] += 1
        t.update(score=float(score), last_seen=now)

    def rate(self, t):
        # High percentile: a partial view reads low, and low is the dangerous side.
        t["height"] = float(np.percentile(t["heights"], 80))
        t["traverse"] = classify_height(t["height"], self.step, self.lethal)

    def merge_duplicates(self, keep):
        for t in list(self.items):
            if t is not keep and t["class"] == keep["class"] and \
                    math.hypot(t["x"] - keep["x"], t["y"] - keep["y"]) < self.radius(keep):
                keep["heights"] = (t["heights"] + keep["heights"])[-self.history:]
                keep["n_obs"] += t["n_obs"]
                self.items.remove(t)

    def publishable(self):
        return [{k: (round(v, 3) if isinstance(v, float) else v)
                 for k, v in t.items() if k != "heights"}
                for t in self.items if t["n_obs"] >= self.min_obs]


def cloud_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
    raw = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
    pts = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float64)
    return pts[np.isfinite(pts).all(axis=1)]


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    from collections import deque

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, PointCloud2
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    class ObjectBoxFusion(Node):
        def __init__(self):
            super().__init__("object_box_fusion")
            p = self.declare_parameter
            p("cloud_topic", "/lidar3d/obstacle_points")
            p("map_frame", "lidar3d_map")
            p("camera_frame", "front_camera")
            p("cloud_history", 15)
            p("floor_z", 0.0)
            p("step_over_height_max", 0.15)
            p("lethal_height_min", 0.25)
            p("min_points", 4)
            p("max_range", 6.0)
            p("unknown_range", 3.0)   # a floor-ray position is poor beyond this
            p("fx", 468.0)
            p("fy", 468.0)
            p("cx", 320.0)
            p("cy", 180.0)
            p("dry_run", False)
            gp = lambda k: self.get_parameter(k).value  # noqa: E731
            self.map_frame, self.cam_frame = gp("map_frame"), gp("camera_frame")
            self.floor_z, self.min_points = float(gp("floor_z")), int(gp("min_points"))
            self.max_range, self.dry_run = float(gp("max_range")), bool(gp("dry_run"))
            self.unknown_range = float(gp("unknown_range"))
            self.K = (float(gp("fx")), float(gp("fy")), float(gp("cx")), float(gp("cy")))
            self.scans = deque(maxlen=int(gp("cloud_history")))
            self.tracks = Tracks(step_over_max=float(gp("step_over_height_max")),
                                 lethal_min=float(gp("lethal_height_min")))
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.pub = self.create_publisher(String, "/objects/boxes_json", 10)
            self.create_subscription(PointCloud2, gp("cloud_topic"), self.on_cloud,
                                     qos_profile_sensor_data)
            self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, 1)
            self.create_subscription(String, "/objects/detections_json", self.on_json, 10)
            try:
                from vision_msgs.msg import Detection2DArray
                self.create_subscription(Detection2DArray, "/yolo/detections", self.on_yolo, 10)
            except ImportError:
                self.get_logger().info("vision_msgs missing: JSON detections only")
            self.get_logger().info(f"object_box_fusion dry_run={self.dry_run}")

        def on_info(self, msg):
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

        def on_cloud(self, msg):
            self.scans.append(cloud_xyz(msg))

        def on_yolo(self, msg):
            dets = []
            for d in msg.detections:
                if d.results:
                    b = d.bbox
                    hw, hh = b.size_x / 2, b.size_y / 2
                    cx, cy = b.center.position.x, b.center.position.y
                    dets.append({"class": d.results[0].hypothesis.class_id,
                                 "score": d.results[0].hypothesis.score,
                                 "bbox": [cx - hw, cy - hh, cx + hw, cy + hh]})
            self.process(dets)

        def on_json(self, msg):
            self.process(json.loads(msg.data)["detections"])

        def process(self, detections):
            if not self.scans:
                return
            try:
                tf = self.tf.lookup_transform(self.cam_frame, self.map_frame, Time())
            except TransformException as ex:
                self.get_logger().warn(str(ex), throttle_duration_sec=5.0)
                return
            pts = np.concatenate(list(self.scans))
            tr = tf.transform.translation
            cam = pts @ rotation_matrix(tf.transform.rotation).T + np.array([tr.x, tr.y, tr.z])
            now = self.get_clock().now().nanoseconds * 1e-9
            rot = rotation_matrix(tf.transform.rotation)
            trans = np.array([tr.x, tr.y, tr.z])
            for det in detections:
                p = ground_point(det["bbox"], self.K, rot, trans, self.floor_z)
                floor_range = None if p is None else float((rot @ p + trans)[0])
                box = fuse_detection(pts, cam, self.K, det["bbox"], self.floor_z, self.min_points,
                                     max_range=self.max_range, floor_range=floor_range)
                if "height" in box:
                    self.tracks.update(det["class"], det["score"], box, now)
                else:
                    if p is not None and np.hypot(*(p + rot.T @ trans)[:2]) < self.unknown_range:
                        self.tracks.update_unknown(det["class"], det["score"], float(p[0]), float(p[1]), now)
            out = {"stamp": now, "boxes": self.tracks.publishable()}
            if self.dry_run:
                self.get_logger().info(f"dry_run boxes: {len(out['boxes'])}",
                                       throttle_duration_sec=5.0)
            else:
                self.pub.publish(String(data=json.dumps(out)))

    rclpy.init()
    node = ObjectBoxFusion()
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
