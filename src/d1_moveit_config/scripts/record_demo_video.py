#!/usr/bin/env python3
"""
record_demo_video.py: the showcase video of run_sim_demo.sh (inside go2-sim, --video).

  python3 record_demo_video.py --out /tmp/servo_demo/demo.mp4      # until Ctrl-C / SIGINT / SIGTERM

1280x720 at 10 fps, composed from:
  left, large   Gazebo observer camera: the overview of the bay while the dog walks, the
                side view of the dog and arm at the rack once it is parked;
  right, top    the wrist RealSense with the tracker's YOLO box and distance
                (yolo_depth_tracker.py annotated_topic, ROS domain 64);
  right, below  the live numbers: phase, measured distance against the 0.50 m target, the
                servo's state and step count (newest /tmp/servo_demo/trial_*/servo_log.csv),
                and a strip chart of the distance.
The overview/side switch follows the dog: parked = within 0.3 m of the world's waypoint.
"""
import argparse
import csv
import glob
import json
import math
import os
import signal
import threading
import time

import cv2
import numpy as np

W, H, FPS = 1280, 720, 10
LW, RW = 853, 427


class Sources:
    def __init__(self, a):
        from gz.msgs10.image_pb2 import Image as GzImage
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.transport13 import Node
        self.lock = threading.Lock()
        self.frames, self.dog = {}, None
        self.node = Node()
        for name in ("overview", "side"):
            self.node.subscribe(GzImage, f"/observer/{name}", lambda m, n=name: self._gz(n, m))
        self.node.subscribe(Odometry, "/model/go2/odometry", self._odom)
        import rclpy
        from sensor_msgs.msg import Image
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=a.camera_domain)
        self.ros = rclpy.create_node("demo_video", context=self.ctx)
        self.ros.create_subscription(Image, a.annotated_topic, self._annotated, 2)
        self.ros.create_subscription(Image, "/camera/color/image_raw", self._raw, 2)
        from rclpy.executors import SingleThreadedExecutor
        # an executor of this node's own context (rclpy.spin_once would use the default one)
        self.executor = SingleThreadedExecutor(context=self.ctx)
        self.executor.add_node(self.ros)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _gz(self, name, m):
        a = np.frombuffer(m.data, np.uint8)
        if a.size == m.width * m.height * 3:
            with self.lock:
                self.frames[name] = cv2.cvtColor(a.reshape(m.height, m.width, 3), cv2.COLOR_RGB2BGR)

    def _odom(self, m):
        p, q = m.pose.position, m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.dog = (p.x, p.y, yaw)

    def _image(self, m):
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        return a.copy() if m.encoding == "bgr8" else cv2.cvtColor(a, cv2.COLOR_RGB2BGR)

    def _annotated(self, m):
        with self.lock:
            self.frames["wrist"] = self._image(m)
            self.frames["wrist_t"] = time.time()

    def _raw(self, m):  # the plain wrist view while the tracker publishes nothing yet
        with self.lock:
            if time.time() - self.frames.get("wrist_t", 0) > 1.0:
                self.frames["wrist_raw"] = self._image(m)

    def get(self, k):
        with self.lock:
            v = self.frames.get(k)
            return None if v is None else v.copy()


def latest_log(root):
    logs = sorted(glob.glob(os.path.join(root, "trial_*", "servo_log.csv")), key=os.path.getmtime)
    if not logs:
        return None, []
    rows = []
    try:
        with open(logs[-1]) as f:
            for r in csv.DictReader(f):
                try:                                 # the servo may be writing the last row
                    float(r["time_s"])
                    if r.get("current_distance_m"):
                        float(r["current_distance_m"])
                except (TypeError, ValueError, KeyError):
                    continue
                if r.get("state") is not None:
                    rows.append(r)
    except OSError:
        pass
    return os.path.basename(os.path.dirname(logs[-1])), rows


def fit(img, w, h):
    if img is None:
        out = np.full((h, w, 3), 30, np.uint8)
        cv2.putText(out, "waiting for video...", (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
        return out
    ih, iw = img.shape[:2]
    s = min(w / iw, h / ih)
    r = cv2.resize(img, (int(iw * s), int(ih * s)), interpolation=cv2.INTER_AREA)
    out = np.zeros((h, w, 3), np.uint8)
    y0, x0 = (h - r.shape[0]) // 2, (w - r.shape[1]) // 2
    out[y0:y0 + r.shape[0], x0:x0 + r.shape[1]] = r
    return out


def label(img, text, org, scale=0.6, color=(255, 255, 255), thick=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def panel(trial, rows, phase, target=0.50, band=0.02, w=RW, h=H - 320):
    p = np.full((h, w, 3), 24, np.uint8)
    label(p, phase, (14, 30), 0.65, (120, 220, 255), 2)
    d = [float(r["current_distance_m"]) if r.get("current_distance_m") else math.nan for r in rows]
    last = next((x for x in reversed(d) if math.isfinite(x)), math.nan)
    state = rows[-1]["state"] if rows else "-"
    steps = sum(1 for r in rows if r.get("state") == "step")
    col = (120, 230, 120) if math.isfinite(last) and abs(last - target) <= band else (90, 170, 255)
    label(p, f"distance  {last:.3f} m" if math.isfinite(last) else "distance  -", (14, 70), 0.75, col, 2)
    label(p, f"target    {target:.2f} m +/- {band * 100:.0f} cm", (14, 102), 0.55)
    label(p, f"servo     {state}, {steps} steps", (14, 130), 0.55)
    if trial:
        label(p, trial.replace("_", " "), (14, 158), 0.5, (170, 170, 170))
    # strip chart of the last 30 s
    x0, y0, cw, ch = 14, 180, w - 28, h - 200
    cv2.rectangle(p, (x0, y0), (x0 + cw, y0 + ch), (70, 70, 70), 1)
    lo, hi = 0.35, 0.70
    yv = lambda v: int(y0 + ch - (v - lo) / (hi - lo) * ch)  # noqa: E731
    cv2.rectangle(p, (x0 + 1, yv(target + band)), (x0 + cw - 1, yv(target - band)), (40, 70, 55), -1)
    cv2.line(p, (x0, yv(target)), (x0 + cw, yv(target)), (90, 160, 120), 1)
    t = [float(r["time_s"]) for r in rows]
    if t:
        t1 = t[-1]
        pts = [(int(x0 + cw - (t1 - ti) / 30.0 * cw), yv(min(hi, max(lo, di))))
               for ti, di in zip(t, d) if math.isfinite(di) and t1 - ti <= 30]
        for a_, b_ in zip(pts, pts[1:]):
            cv2.line(p, a_, b_, (90, 200, 255), 2, cv2.LINE_AA)
    label(p, f"{hi:.2f}", (x0 + 4, y0 + 14), 0.4, (150, 150, 150))
    label(p, f"{lo:.2f} m", (x0 + 4, y0 + ch - 6), 0.4, (150, 150, 150))
    label(p, "last 30 s", (x0 + cw - 70, y0 + ch - 6), 0.4, (150, 150, 150))
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="/tmp/servo_demo/demo.mp4")
    ap.add_argument("--trials-dir", default="/tmp/servo_demo")
    ap.add_argument("--world-json", default="/sim/worlds/cropking_nft46.json")
    ap.add_argument("--annotated-topic", default="/tracker/annotated")
    ap.add_argument("--camera-domain", type=int, default=64)
    a = ap.parse_args()
    wp = json.load(open(a.world_json))["waypoint"]
    src = Sources(a)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    n, t_next = 0, time.time()
    print(f"[video] recording {a.out}", flush=True)
    while not stop.is_set():
        dog = src.dog
        parked = dog is not None and math.hypot(dog[0] - wp[0], dog[1] - wp[1]) < 0.3
        trial, rows = latest_log(a.trials_dir) if parked else (None, [])
        main_view = src.get("side" if parked else "overview")
        frame = np.zeros((H, W, 3), np.uint8)
        frame[:, :LW] = fit(main_view, LW, H)
        wrist = src.get("wrist")
        frame[:320, LW:] = fit(wrist if wrist is not None else src.get("wrist_raw"), RW, 320)
        phase = ("servo: holding the camera 0.50 m from the lettuce" if parked and rows else
                 "parked at the rack" if parked else "Nav2: walking to the NFT 4-6")
        frame[320:, LW:] = panel(trial, rows, phase)
        label(frame, "Go2 + D1 arm, RealSense D435i on the arm  |  Gazebo", (14, 28), 0.65, (255, 255, 255), 2)
        label(frame, "overview" if not parked else "side view", (14, H - 14), 0.5, (200, 200, 200))
        label(frame, "wrist camera + YOLO (lettuce)", (LW + 8, 20), 0.5, (200, 255, 200))
        cv2.line(frame, (LW, 0), (LW, H), (60, 60, 60), 2)
        vw.write(frame)
        n += 1
        t_next += 1.0 / FPS
        time.sleep(max(0.0, t_next - time.time()))
    vw.release()
    print(f"[video] {n} frames ({n / FPS:.0f} s) -> {a.out}", flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
