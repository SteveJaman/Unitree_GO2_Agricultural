#!/usr/bin/env python3
"""Semantic traversability labels for fused 3D boxes, from a VLM (Ollama) and LiDAR height.

For each world-fixed track on /objects/boxes_json (object_box_fusion.py), the
node finds the track's 2D detection in the current camera image, crops it, and
asks the VLM to verify the detector's class and classify it independently:
STEP_OVER | CAUTION | AVOID. One request is in flight at a time; a track is asked
again after reclassify_s. Height comes from LiDAR only: the published label is the
more severe of the LiDAR band and the VLM label (VLM AVOID is lethal only for a
LiDAR-measured object); with no LiDAR height any VLM label is CAUTION, flagged
low_confidence
(scripts/nodes/semantic_labeler.py has the rule and the prompt).

Subscribes (all topic names are parameters):
  /objects/boxes_json        fused tracks (JSON)
  /objects/detections_json   2D detections, YOLO and the sim stand-in (JSON)
  /camera/image_raw, /camera/camera_info
  TF map_frame -> camera_frame
Publishes:
  /semantic/labels_json      every track's labels, VLM answer and latency (always)
  /semantic/obstacles        go2_semantic_msgs/SemanticObstacleArray for
                             SemanticCostmapLayer (not in dry_run)

Parameters: ollama_url, model, timeout_s, confidence_min, reclassify_s,
fallback_on_timeout ("geometry": a failed request leaves the LiDAR label;
"unknown": the track is published UNKNOWN), dry_run (default true), backend
(only "gazebo"; the sim and the robot differ only in where the camera comes from,
and the robot path is Phase 2 hardware work).
"""
import json
import os
import queue
import sys
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import semantic_labeler as sl  # noqa: E402

COSTS = {"AVOID": 254, "CAUTION": 200, "STEP_OVER": 40, "UNKNOWN": 0}


class VlmHeuristicNode(Node):
    def __init__(self):
        super().__init__("vlm_heuristic_node")
        p = self.declare_parameter
        p("ollama_url", "http://172.30.160.1:11434")
        p("model", "qwen2.5vl")
        p("timeout_s", 10.0)
        p("confidence_min", 0.7)
        p("reclassify_s", 5.0)
        p("fallback_on_timeout", "geometry")
        p("dry_run", True)
        p("backend", "gazebo")
        p("map_frame", "lidar3d_map")
        p("camera_frame", "front_camera")
        p("boxes_topic", "/objects/boxes_json")
        p("detections_topic", "/objects/detections_json")
        p("image_topic", "/camera/image_raw")
        p("camera_info_topic", "/camera/camera_info")
        p("obstacles_topic", "/semantic/obstacles")
        p("labels_topic", "/semantic/labels_json")
        p("step_over_height_max", 0.12)
        p("lethal_height_min", 0.22)
        p("detection_max_age_s", 0.6)
        p("radius_margin", 0.05)
        p("publish_hz", 2.0)
        p("fx", 468.0)
        p("fy", 468.0)
        p("cx", 320.0)
        p("cy", 180.0)
        g = lambda k: self.get_parameter(k).value  # noqa: E731

        self.backend = g("backend")
        if self.backend != "gazebo":
            raise NotImplementedError(
                f"vlm_heuristic_node backend '{self.backend}': only 'gazebo' is implemented in sim.")
        self.url, self.model = g("ollama_url"), g("model")
        self.timeout_s, self.conf_min = float(g("timeout_s")), float(g("confidence_min"))
        self.reclassify_s, self.fallback = float(g("reclassify_s")), g("fallback_on_timeout")
        if self.fallback not in ("geometry", "unknown"):
            raise ValueError(f"fallback_on_timeout must be geometry or unknown, not {self.fallback}")
        self.dry_run = bool(g("dry_run"))
        self.map_frame, self.cam_frame = g("map_frame"), g("camera_frame")
        self.step, self.lethal = float(g("step_over_height_max")), float(g("lethal_height_min"))
        self.det_age, self.r_margin = float(g("detection_max_age_s")), float(g("radius_margin"))
        self.K = (float(g("fx")), float(g("fy")), float(g("cx")), float(g("cy")))

        self.tracks = {}            # id -> fused box
        self.answers = {}           # id -> latest VLM answer
        self.dets = deque(maxlen=50)  # (receipt time, detection)
        self.image = None
        self.lock = threading.Lock()
        self.jobs = queue.Queue(maxsize=1)
        self.busy = False
        self.n_asked = self.n_failed = 0

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(String, g("boxes_topic"), self.on_boxes, 10)
        self.create_subscription(String, g("detections_topic"), self.on_dets, 10)
        self.create_subscription(Image, g("image_topic"), self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g("camera_info_topic"), self.on_info, 1)
        self.labels_pub = self.create_publisher(String, g("labels_topic"), 10)
        self.obst_pub = None
        if not self.dry_run:
            from go2_semantic_msgs.msg import SemanticObstacle, SemanticObstacleArray
            self.Obstacle, self.ObstacleArray = SemanticObstacle, SemanticObstacleArray
            self.obst_pub = self.create_publisher(SemanticObstacleArray, g("obstacles_topic"), 10)
        threading.Thread(target=self.worker, daemon=True).start()
        self.create_timer(0.25, self.schedule)
        self.create_timer(1.0 / float(g("publish_hz")), self.publish)
        ready, why = self.ollama_ready()
        self.get_logger().info(
            f"vlm_heuristic_node dry_run={self.dry_run} backend={self.backend} model={self.model} "
            f"url={self.url} ollama={'ok' if ready else 'NOT READY: ' + why} "
            f"confidence_min={self.conf_min} reclassify_s={self.reclassify_s} fallback={self.fallback}")

    def ollama_ready(self):
        import urllib.request
        try:
            with urllib.request.urlopen(self.url.rstrip("/") + "/api/tags", timeout=3) as r:
                names = [m["name"] for m in json.loads(r.read())["models"]]
            ok = any(n.split(":")[0] == self.model.split(":")[0] for n in names)
            return ok, "" if ok else f"model {self.model} not in {names}"
        except Exception as ex:  # noqa: BLE001 - reported in the startup line
            return False, str(ex)

    # --- inputs -------------------------------------------------------------
    def on_info(self, msg):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_image(self, msg):
        self.image = msg

    def on_dets(self, msg):
        now = time.time()
        for d in json.loads(msg.data).get("detections", []):
            self.dets.append((now, d))

    def on_boxes(self, msg):
        boxes = json.loads(msg.data).get("boxes", [])
        with self.lock:
            self.tracks = {b["id"]: b for b in boxes}
            for k in list(self.answers):
                if k not in self.tracks:
                    del self.answers[k]

    # --- VLM requests -------------------------------------------------------
    def schedule(self):
        if self.busy or self.image is None or not self.tracks:
            return
        try:
            tf = self.tf.lookup_transform(self.cam_frame, self.map_frame, Time())
        except TransformException:
            return
        q, t = tf.transform.rotation, tf.transform.translation
        rot, trans = sl.rotation_matrix((q.x, q.y, q.z, q.w)), np.array([t.x, t.y, t.z])
        cam_xy = (-rot.T @ trans)[:2]
        now = time.time()
        dets = [d for (ts, d) in self.dets if now - ts < self.det_age]
        with self.lock:
            due = [b for b in self.tracks.values()
                   if b["id"] not in self.answers or now - self.answers[b["id"]]["t"] > self.reclassify_s]
        due.sort(key=lambda b: (b["id"] in self.answers, np.hypot(b["x"] - cam_xy[0], b["y"] - cam_xy[1])))
        img = None
        for b in due:
            det = sl.associate(b, dets, rot, trans, self.K)
            if det is None:
                continue
            if img is None:
                m = self.image
                img = sl.to_rgb(np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1), m.encoding)
            got = sl.crop_bbox(img, det["bbox"])
            if got is None:
                continue
            self.busy = True
            self.jobs.put((b["id"], det.get("class", b["class"]), float(det.get("score", b.get("score", 0.0))),
                           got[0].copy()))
            return

    def worker(self):
        while True:
            tid, cls, score, crop = self.jobs.get()
            ans = {"t": time.time(), "error": None}
            try:
                # The detector's class and score go in for verification, not as an answer.
                label, conf, reason, lat = sl.ask_ollama(self.url, self.model, crop, cls, score, self.timeout_s)
                ans.update(label=label, conf=conf, reason=reason, latency_s=round(lat, 2))
                self.get_logger().info(
                    f"track {tid} {cls}: {label} {conf:.2f} ({reason}) {lat:.2f}s")
            except Exception as ex:  # noqa: BLE001 - timeout/connection: fallback, logged
                self.n_failed += 1
                ans.update(label="UNKNOWN", conf=0.0, reason="", latency_s=None, error=str(ex))
                self.get_logger().warn(f"track {tid} {cls}: VLM failed ({ex}); fallback={self.fallback}",
                                       throttle_duration_sec=5.0)
            self.n_asked += 1
            with self.lock:
                if tid in self.tracks:
                    self.answers[tid] = ans
            self.busy = False

    # --- output ---------------------------------------------------------------
    def label_track(self, b, now):
        geo = sl.geometry_label(b, self.step, self.lethal)
        a = self.answers.get(b["id"])
        out = {"track_id": b["id"], "class": b["class"], "x": b["x"], "y": b["y"],
               "height": b.get("height"), "geometry": geo, "vlm": None, "vlm_conf": None,
               "vlm_reason": None, "latency_s": None, "label_age_s": None, "error": None}
        low = geo == "UNKNOWN"   # no LiDAR height: whatever is published is flagged
        if a is None:
            final, source, conf = geo, ("geometry_only" if geo != "UNKNOWN" else "none"), 1.0
        else:
            out.update(vlm=a["label"], vlm_conf=round(a["conf"], 2), vlm_reason=a["reason"],
                       latency_s=a["latency_s"], label_age_s=round(now - a["t"], 1), error=a["error"])
            if a["error"] and self.fallback == "unknown":
                final, source, conf = "UNKNOWN", "fallback_unknown", 0.0
            else:
                final, source, low = sl.combine(a["label"], a["conf"], geo, self.conf_min)
                conf = a["conf"] if source.startswith("vlm") else 1.0
        out.update(label=final, source=source, confidence=round(conf, 2), low_confidence=low,
                   cost=COSTS[final], radius=round(sl.radius(b, self.r_margin), 3),
                   size_x=b.get("sx") or 0.0, size_y=b.get("sy") or 0.0, yaw=b.get("yaw") or 0.0)
        return out

    def publish(self):
        now = time.time()
        with self.lock:
            objs = [self.label_track(b, now) for b in self.tracks.values()]
        self.labels_pub.publish(String(data=json.dumps(
            {"stamp": now, "dry_run": self.dry_run, "model": self.model, "asked": self.n_asked,
             "failed": self.n_failed, "objects": objs})))
        if self.obst_pub is None:
            return
        arr = self.ObstacleArray()
        arr.header.frame_id = self.map_frame
        arr.header.stamp = self.get_clock().now().to_msg()
        for o in objs:
            if o["label"] == "UNKNOWN":
                continue
            s = self.Obstacle()
            s.header = arr.header
            s.class_name = o["class"]
            s.label = o["label"]
            s.track_id = int(o["track_id"])
            s.confidence = float(o["confidence"])
            s.position = Point(x=float(o["x"]), y=float(o["y"]), z=float(o["height"] or 0.0))
            s.cost = o["cost"]
            s.radius = float(o["radius"])
            s.size_x, s.size_y, s.yaw = float(o["size_x"]), float(o["size_y"]), float(o["yaw"])
            arr.obstacles.append(s)
        self.obst_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = VlmHeuristicNode()
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
