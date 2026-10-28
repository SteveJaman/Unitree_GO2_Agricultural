#!/usr/bin/env python3
"""
D435i sender: runs on the machine the camera is plugged into (Jetson / Go2 PC).

Captures color + depth (depth aligned to color), compresses them, and publishes
over ZeroMQ so a receiver on another machine can republish as ROS2 topics.

Wire format (one multipart ZMQ message per frame):
    part 0: JSON header (seq, stamp, intrinsics, depth_scale, sizes)
    part 1: color frame, JPEG (8-bit BGR)
    part 2: depth frame, PNG (16-bit, lossless, in sensor depth units)

Install (on the sender):
    pip install pyrealsense2 pyzmq opencv-python numpy
    (if there is no aarch64 wheel for pyrealsense2, build librealsense from
     source with -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS=ON)

Run:
    python3 d435i_sender.py --width 640 --height 480 --fps 30 --port 5555
"""

import argparse
import json
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import zmq


def parse_args():
    ap = argparse.ArgumentParser(description="Stream D435i color+depth over ZeroMQ")
    ap.add_argument(
        "--bind", default="0.0.0.0", help="interface to bind (default: all)"
    )
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument(
        "--jpeg-quality", type=int, default=80, help="color JPEG quality 1-100"
    )
    ap.add_argument(
        "--serial",
        default="",
        help="D435i serial number (only needed with several cameras)",
    )
    return ap.parse_args()


def main():
    args = parse_args()

    # --- RealSense pipeline -------------------------------------------------
    pipe = rs.pipeline()
    cfg = rs.config()
    if args.serial:
        cfg.enable_device(args.serial)
    cfg.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
    )
    cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)

    try:
        profile = pipe.start(cfg)
    except RuntimeError as e:
        raise SystemExit(
            f"Could not start camera: {e}\n"
            "Check: camera on a USB 3 port/cable, no other process using it, "
            "and that the width/height/fps combo is supported by the D435i."
        )

    # Align depth into the color camera frame so one set of intrinsics fits both.
    align = rs.align(rs.stream.color)
    depth_scale = (
        profile.get_device().first_depth_sensor().get_depth_scale()
    )  # metres per unit
    intr = (
        profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    )
    intrinsics = {
        "width": intr.width,
        "height": intr.height,
        "fx": intr.fx,
        "fy": intr.fy,
        "cx": intr.ppx,
        "cy": intr.ppy,
        "coeffs": list(intr.coeffs),
    }
    print(f"Streaming {args.width}x{args.height}@{args.fps}, depth_scale={depth_scale}")

    # --- ZeroMQ publisher ---------------------------------------------------
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 2)  # small queue: drop frames rather than build latency
    sock.bind(f"tcp://{args.bind}:{args.port}")
    print(f"Publishing on tcp://{args.bind}:{args.port}")

    jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
    png_params = [cv2.IMWRITE_PNG_COMPRESSION, 1]  # fast, still lossless

    seq = 0
    sent_bytes = 0
    t_report = time.time()
    try:
        while True:
            frames = pipe.wait_for_frames(5000)
            stamp = (
                time.time()
            )  # sender wall clock; keep sender and receiver clocks synced
            frames = align.process(frames)
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if not color or not depth:
                continue

            color_img = np.asanyarray(color.get_data())
            depth_img = np.asanyarray(depth.get_data())  # uint16

            ok_c, color_buf = cv2.imencode(".jpg", color_img, jpeg_params)
            ok_d, depth_buf = cv2.imencode(".png", depth_img, png_params)
            if not (ok_c and ok_d):
                continue

            header = {
                "seq": seq,
                "stamp": stamp,
                "depth_scale": depth_scale,
                "intrinsics": intrinsics,
            }
            parts = [
                json.dumps(header).encode(),
                color_buf.tobytes(),
                depth_buf.tobytes(),
            ]
            sock.send_multipart(parts, copy=False)

            seq += 1
            sent_bytes += sum(len(p) for p in parts)
            now = time.time()
            if now - t_report >= 5.0:
                mbps = sent_bytes * 8 / (now - t_report) / 1e6
                print(f"frame {seq}  ~{mbps:.1f} Mbit/s")
                sent_bytes = 0
                t_report = now
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()
        sock.close(0)
        ctx.term()


if __name__ == "__main__":
    main()
