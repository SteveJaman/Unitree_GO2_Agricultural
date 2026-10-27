#!/usr/bin/env python3
"""
calibrate_camera.py: headless intrinsic calibration of the Go2 front camera.

Hold a printed checkerboard in front of the dog and move it around. Frames in
which the board is found, and whose board pose differs enough from the views
already taken, are captured automatically. With enough views the camera
matrix and distortion are solved with OpenCV and written as a ROS camera
calibration YAML (the format camera_calibration writes and CameraInfo uses).

Live (inside go2-lidar3d, via scripts/tools/calibrate_camera.sh):
    python3 calibrate_camera.py --cols 9 --rows 6 --square 0.025 --out /tmp/go2-camera-calib
Offline, from images saved by an earlier run (no ROS needed):
    python3 calibrate_camera.py --cols 9 --rows 6 --square 0.025 --from-dir DIR --out DIR

--cols/--rows count INNER corners (a board of 10x7 squares has 9x6).
Output in --out: front_camera.yaml, views/*.png, undistorted_preview.png.
"""
import argparse
import glob
import math
import os
import sys
import time

import cv2
import numpy as np


# --------------------------------------------------------------- detection
def find_board(gray, pattern, fast=True):
    """Sub-pixel checkerboard corners (N x 1 x 2 float32), or None."""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    # Detect on a downscaled copy for speed, then refine at full resolution.
    scale = min(1.0, 640.0 / gray.shape[1])
    small = cv2.resize(gray, None, fx=scale, fy=scale) if scale < 1.0 else gray
    ok, corners = cv2.findChessboardCorners(
        small, pattern, flags | (cv2.CALIB_CB_FAST_CHECK if fast else 0))
    if not ok:
        return None
    corners = (corners / scale).astype(np.float32)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    # The refinement window must stay inside one square: a wide-angle lens at
    # 960x540 can show the board with squares of ~10 px.
    c = corners.reshape(pattern[1], pattern[0], 2)
    sq = min(np.linalg.norm(np.diff(c, axis=1), axis=2).min(), np.linalg.norm(np.diff(c, axis=0), axis=2).min())
    win = int(max(2, min(11, sq * 0.4)))
    cv2.cornerSubPix(gray, corners, (win, win), (-1, -1), crit)
    return corners


def board_params(corners, pattern, size):
    """(x, y, size, skew) of a view in [0, 1], as ROS camera_calibration uses
    them to judge coverage: board centre, apparent size and tilt."""
    cols, rows = pattern
    c = corners.reshape(-1, 2)
    ul, ur, dl, dr = c[0], c[cols - 1], c[-cols], c[-1]
    w, h = size
    area = 0.5 * abs(np.cross(ur - ul, dr - ul) + np.cross(dr - ul, dl - ul))
    a, b = ur - ul, dl - ul
    angle = math.acos(float(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1, 1)))
    skew = min(1.0, 2.0 * abs(math.pi / 2 - angle))
    mx, my = c.mean(axis=0)
    return np.array([mx / w, my / h, math.sqrt(area / (w * h)), skew])


def coverage(params):
    """Range covered by the views in each of x, y, size, skew (0..1 each)."""
    p = np.array(params)
    lo = np.array([0.0, 0.0, 0.1, 0.0]) if len(p) == 0 else p.min(axis=0)
    hi = np.array([0.0, 0.0, 0.1, 0.0]) if len(p) == 0 else p.max(axis=0)
    span = hi - lo
    # Same nominal full ranges as ROS camera_calibration.
    return np.clip(span / np.array([0.7, 0.7, 0.4, 0.5]), 0, 1)


def bar(v):
    n = int(round(v * 10))
    return "#" * n + "." * (10 - n)


# --------------------------------------------------------------- solving
def calibrate(views, pattern, square, size):
    cols, rows = pattern
    obj = np.zeros((cols * rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    objs = [obj] * len(views)
    rms, k, d, rvecs, tvecs = cv2.calibrateCamera(objs, views, size, None, None)
    per_view = []
    for o, img_pts, r, t in zip(objs, views, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, r, t, k, d)
        per_view.append(float(np.sqrt(np.mean(np.sum((proj - img_pts) ** 2, axis=2)))))
    return rms, k, d.reshape(-1), per_view


def write_yaml(path, name, size, k, d):
    w, h = size
    p = [k[0, 0], 0.0, k[0, 2], 0.0, 0.0, k[1, 1], k[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
    fmt = lambda vals: "[" + ", ".join(f"{float(v):.8g}" for v in vals) + "]"
    with open(path, "w") as f:
        f.write(f"""image_width: {w}
image_height: {h}
camera_name: {name}
camera_matrix:
  rows: 3
  cols: 3
  data: {fmt(k.reshape(-1))}
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: {len(d)}
  data: {fmt(d)}
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: {fmt(p)}
""")


def finish(views, images, pattern, args, size):
    if len(views) < 8:
        sys.exit(f"only {len(views)} usable views; need at least 8")
    rms, k, d, per_view = calibrate(views, pattern, args.square, size)
    os.makedirs(args.out, exist_ok=True)
    yaml_path = os.path.join(args.out, "front_camera.yaml")
    write_yaml(yaml_path, "go2_front_camera", size, k, d)
    if images:
        und = cv2.undistort(images[-1], k, d)
        cv2.imwrite(os.path.join(args.out, "undistorted_preview.png"), np.hstack([images[-1], und]))
    hfov = math.degrees(2 * math.atan(size[0] / (2 * k[0, 0])))
    worst = sorted(range(len(per_view)), key=lambda i: -per_view[i])[:3]
    print()
    print(f"views            : {len(views)}  image {size[0]}x{size[1]}")
    print(f"RMS reprojection : {rms:.3f} px   (< 0.5 good, < 1.0 usable, > 1.0 redo)")
    print(f"fx fy            : {k[0, 0]:.1f} {k[1, 1]:.1f}   (horizontal FOV {hfov:.1f} deg)")
    print(f"cx cy            : {k[0, 2]:.1f} {k[1, 2]:.1f}")
    print(f"distortion       : {' '.join(f'{v:.4f}' for v in d)}")
    print(f"worst views      : " + ", ".join(f"#{i} {per_view[i]:.2f}px" for i in worst))
    print(f"wrote {yaml_path}")
    return rms


# --------------------------------------------------------------- sources
def run_offline(args, pattern):
    files = sorted(glob.glob(os.path.join(args.from_dir, "*.png")) +
                   glob.glob(os.path.join(args.from_dir, "*.jpg")))
    views, images, size = [], [], None
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        c = find_board(gray, pattern, fast=False)
        print(f"{os.path.basename(f)}: {'board' if c is not None else 'no board'}")
        if c is not None:
            views.append(c)
            images.append(img)
    if size is None:
        sys.exit(f"no images in {args.from_dir}")
    return finish(views, images, pattern, args, size)


def run_live(args, pattern):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    views_dir = os.path.join(args.out, "views")
    os.makedirs(views_dir, exist_ok=True)

    class Calib(Node):
        def __init__(self):
            super().__init__("go2_camera_calibrator")
            self.views, self.params, self.images = [], [], []
            self.size = None
            self.last = 0.0
            self.frames = 0
            self.searched = 0
            self.seen = 0
            self.done = False
            # The relay and Gazebo publish reliable; a best-effort reader would
            # lose most large raw frames.
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
            self.create_subscription(Image, args.topic, self.on_image, qos)
            self.create_timer(5.0, self.status)
            print(f"waiting for {args.topic} ... board {pattern[0]}x{pattern[1]} inner corners, "
                  f"{args.square * 1000:.0f} mm squares, {args.samples} views")

        def status(self):
            if self.frames == 0:
                print(f"  no images on {args.topic} yet (is a wifi_nav.sh / wifi_3d_nav.sh session running?)")
            elif not self.done:
                print(
                    f"  {self.frames} frames, {self.searched} searched, board seen in {self.seen}, "
                    f"views {len(self.views)}/{args.samples}"
                    + ("  <- no board found: hold it fully in view, check --cols/--rows" if self.seen == 0 else "")
                )
                self.frames = self.searched = self.seen = 0

        def on_image(self, msg):
            self.frames += 1
            now = time.monotonic()
            if self.done or now - self.last < 1.0 / args.rate:
                return
            self.last = now
            if msg.encoding not in ("bgr8", "rgb8", "mono8"):
                print(f"unsupported encoding {msg.encoding}")
                return
            ch = 1 if msg.encoding == "mono8" else 3
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // ch, ch)[:, :msg.width]
            if msg.encoding == "rgb8":
                img = img[:, :, ::-1]
            img = np.ascontiguousarray(img if ch == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            size = (msg.width, msg.height)
            if self.size and size != self.size:
                print(f"image size changed {self.size} -> {size}; restart the calibration")
                self.done = True
                return
            self.size = size
            self.searched += 1
            c = find_board(gray, pattern)
            if c is None:
                return
            self.seen += 1
            p = board_params(c, pattern, size)
            if self.params and min(np.abs(p - q).sum() for q in self.params) < args.min_change:
                return
            self.views.append(c)
            self.params.append(p)
            self.images.append(img)
            cv2.imwrite(os.path.join(views_dir, f"view_{len(self.views):02d}.png"), img)
            cov = coverage(self.params)
            print(f"  view {len(self.views):2d}/{args.samples}   X {bar(cov[0])}  Y {bar(cov[1])}  "
                  f"size {bar(cov[2])}  skew {bar(cov[3])}")
            if len(self.views) >= args.samples:
                self.done = True

    rclpy.init()
    node = Calib()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        print("\ninterrupted: calibrating with the views taken so far")
    finally:
        views, images, size = node.views, node.images, node.size
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if size is None:
        sys.exit("no images received")
    cov = coverage(node.params)
    if cov.min() < 0.5:
        print("WARN: poor coverage (move the board to the edges/corners, nearer/farther, and tilt it)")
    return finish(views, images, pattern, args, size)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cols", type=int, default=9, help="inner corners per row")
    ap.add_argument("--rows", type=int, default=6, help="inner corners per column")
    ap.add_argument("--square", type=float, default=0.025, help="square size in metres")
    ap.add_argument("--topic", default="/camera/image_raw")
    ap.add_argument("--samples", type=int, default=25, help="views to capture")
    ap.add_argument("--rate", type=float, default=3.0, help="max frames/s searched for the board")
    ap.add_argument("--min-change", type=float, default=0.2,
                    help="how different a view must be from all earlier ones (sum of x, y, size, skew)")
    ap.add_argument("--out", default="/tmp/go2-camera-calib")
    ap.add_argument("--from-dir", help="calibrate from saved images instead of the live topic")
    args = ap.parse_args()
    pattern = (args.cols, args.rows)
    rms = run_offline(args, pattern) if args.from_dir else run_live(args, pattern)
    sys.exit(0 if rms < 1.0 else 2)


if __name__ == "__main__":
    main()
