#!/usr/bin/env python3
"""Standalone test: YOLO on the Go2's RealSense D435i, distance to the detected object.

Subscribes to the compressed colour and aligned-depth topics that the robot's camera
service publishes over Wi-Fi, runs YOLO on the colour image, takes the centre pixel
(u, v) of the chosen box, reads the aligned depth there and publishes it in metres on
/object_distance (std_msgs/Float32). Nothing is published while no object is seen;
distance_servo.py treats silence as "stop".

Depth: the aligned depth image has the colour image's size, so (u, v) indexes it
directly. A single pixel is often 0 (no stereo match) on edges and shiny leaves, so the
value is the median of the valid pixels in a small window around (u, v)
(depth_window px, 1 = exactly the centre pixel).

Domains: the camera is on ROS_DOMAIN_ID 64 (realsense_host.sh), MoveIt on 78
(arm_moveit.sh). Run this node with the camera's DDS setup and give out_domain_id:=78
to publish /object_distance on MoveIt's domain from a second rclpy context. With
out_domain_id < 0 it publishes on the camera's own domain.

Usage:
    python3 scripts/nodes/yolo_depth_tracker.py --ros-args -p target_class:=bottle \
        -p out_domain_id:=78 -p show:=true

Parameters: model (yolo11s.pt), target_class ('' = any class), min_confidence (0.5),
depth_mode (center: the median of a depth_window square at the box centre, 5 px; box: the
box_percentile-th (25) of the depths in the central box_fraction (0.5) of the box, the
front of a leafy plant), color_topic, depth_topic (a topic not ending in /compressed or
/compressedDepth is read as a raw Image: rgb8/bgr8 colour, 32FC1 m or 16UC1 mm depth, as
the Gazebo wrist camera publishes them), out_domain_id (-1), device ('' = auto),
show (false: an OpenCV window with the box and distance), annotated_topic ('': the same
picture published as a bgr8 Image, e.g. for run_sim_demo.sh --video).
"""

import struct
import time

import cv2
import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def decode_color(msg):
    """sensor_msgs/CompressedImage (JPEG/PNG) or sensor_msgs/Image (rgb8/bgr8) -> BGR uint8 image."""
    if isinstance(msg, Image):
        enc = msg.encoding.lower()
        if enc not in ("rgb8", "bgr8"):
            raise ValueError(f'unhandled colour encoding "{msg.encoding}"')
        img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[
            :, : msg.width * 3
        ]
        img = img.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img.copy()
    img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"cannot decode colour image ({msg.format})")
    return img


def decode_compressed_depth(msg):
    """compressed_depth_image_transport message -> depth in metres (float32, 0 = no data).

    The data is a 12-byte ConfigHeader followed by a PNG:
        int32 compressionFormat; float32 depthQuantA; float32 depthQuantB
    format "16UC1; compressedDepth png": PNG of the raw uint16 depth, millimetres for the
        RealSense (depth_scale 0.001).
    format "32FC1; compressedDepth png": PNG of uint16 inverse-depth codes,
        depth = depthQuantA / (code - depthQuantB), code 0 = no data.
    """
    data = bytes(msg.data)
    start = data.find(PNG_MAGIC)
    if start < 0:
        raise ValueError(
            f'no PNG in compressedDepth message ("{msg.format}"); rvl is not handled'
        )
    raw = cv2.imdecode(np.frombuffer(data[start:], np.uint8), cv2.IMREAD_UNCHANGED)
    if raw is None:
        raise ValueError("cannot decode compressedDepth PNG")
    encoding = msg.format.split(";")[0].strip().lower()
    if encoding == "16uc1" or encoding == "mono16":
        return raw.astype(np.float32) / 1000.0
    if encoding == "32fc1":
        if start < 12:
            raise ValueError("32FC1 compressedDepth without its 12-byte header")
        _, quant_a, quant_b = struct.unpack("<iff", data[:12])
        code = raw.astype(np.float32)
        depth = np.zeros_like(code)
        valid = code > quant_b
        depth[valid] = quant_a / (code[valid] - quant_b)
        return depth
    raise ValueError(f'unhandled depth encoding "{encoding}"')


def decode_depth(msg):
    """Depth in metres (float32, 0 = no data) from a raw sensor_msgs/Image (32FC1 metres, as
    the Gazebo wrist camera; 16UC1 millimetres, as the RealSense driver) or a
    compressedDepth CompressedImage."""
    if not isinstance(msg, Image):
        return decode_compressed_depth(msg)
    enc = msg.encoding.lower()
    if enc == "32fc1":
        depth = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, -1)[
            :, : msg.width
        ]
        depth = np.nan_to_num(depth.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        return depth
    if enc in ("16uc1", "mono16"):
        raw = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, -1)[
            :, : msg.width
        ]
        return raw.astype(np.float32) / 1000.0
    raise ValueError(f'unhandled raw depth encoding "{msg.encoding}"')


def depth_in_box(depth_m, box, fraction=0.5, percentile=25.0):
    """Distance to the front of the object: the percentile-th of the valid depths in the
    central fraction of the box (x1, y1, x2, y2), or None.

    For a plant the box centre often falls between leaves onto whatever is behind it (in the
    simulation, with the camera on the claw: 0.83 m read for a lettuce 0.69 m away); a low
    percentile over the box's middle measures the leaves nearest the camera instead."""
    h, w = depth_m.shape[:2]
    x1, y1, x2, y2 = box
    cx, cy, hw, hh = (
        (x1 + x2) / 2,
        (y1 + y2) / 2,
        (x2 - x1) * fraction / 2,
        (y2 - y1) * fraction / 2,
    )
    c0, c1 = max(0, int(cx - hw)), min(w, int(cx + hw) + 1)
    r0, r1 = max(0, int(cy - hh)), min(h, int(cy + hh) + 1)
    patch = depth_m[r0:r1, c0:c1]
    good = patch[np.isfinite(patch) & (patch > 0.0)]
    return float(np.percentile(good, percentile)) if good.size >= 5 else None


def iou(a, b):
    """Intersection over union of two (x1, y1, x2, y2, ...) boxes."""
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def choose_box(cands, last, w, h, by_confidence=False, min_iou=0.3):
    """The box to follow among cands ((x1, y1, x2, y2, label, conf)): the most confident
    (by_confidence), else the one overlapping last most (IoU >= min_iou), else the
    largest."""
    if not cands:
        return None
    if by_confidence:
        return max(cands, key=lambda c: c[5])
    if last is not None:
        best = max(cands, key=lambda c: iou(c, last))
        if iou(best, last) >= min_iou:
            return best
    # nothing followed yet: the largest box, the nearest plant (the one the arm is at). The
    # box nearest the image centre picked plants further back on the rack, which sit nearer
    # the horizon than the target when the camera is below it.
    return max(cands, key=lambda c: (c[2] - c[0]) * (c[3] - c[1]))


def depth_at(depth_m, u, v, window):
    """Median of the valid (> 0, finite) depths in a window x window square at (u, v), or None."""
    h, w = depth_m.shape[:2]
    r = max(0, int(window) // 2)
    patch = depth_m[
        max(0, v - r) : min(h, v + r + 1), max(0, u - r) : min(w, u + r + 1)
    ]
    good = patch[np.isfinite(patch) & (patch > 0.0)]
    return float(np.median(good)) if good.size else None


class YoloDepthTracker(Node):
    def __init__(self):
        super().__init__("yolo_depth_tracker")
        d = self.declare_parameter
        d("model", "yolo11s.pt")
        d("target_class", "")
        d("min_confidence", 0.5)
        d("depth_window", 5)
        d(
            "depth_mode", "center"
        )  # center: median around the box centre; box: see depth_in_box
        d("box_fraction", 0.5)
        d("box_percentile", 25.0)
        d("color_topic", "/camera/color/image_raw/compressed")
        d("depth_topic", "/camera/aligned_depth_to_color/image_raw/compressedDepth")
        d("out_domain_id", -1)
        d("device", "")
        d("show", False)
        d("select", "track")  # track | confidence (see best_box)
        d("track_timeout_s", 1.0)
        d(
            "annotated_topic", ""
        )  # '' = off; else a bgr8 Image with the box and distance (demo video)
        self.p = lambda n: self.get_parameter(n).value

        from ultralytics import YOLO  # slow import: after the parameters are checked

        self.model = YOLO(self.p("model"))
        self.names = self.model.names
        target = self.p("target_class").strip()
        if target and target not in self.names.values():
            raise ValueError(
                f'target_class "{target}" is not in the model: {sorted(self.names.values())}'
            )

        out_domain = int(self.p("out_domain_id"))
        self.out_node = None
        if out_domain >= 0:
            self.out_ctx = rclpy.Context()
            rclpy.init(context=self.out_ctx, domain_id=out_domain)
            self.out_node = Node("yolo_depth_tracker_out", context=self.out_ctx)
            self.pub = self.out_node.create_publisher(Float32, "/object_distance", 10)
        else:
            self.pub = self.create_publisher(Float32, "/object_distance", 10)

        # .../compressed: CompressedImage; anything else: a raw Image. On the Go2's camera
        # service the compressed colour topic carried nothing (2026-11-18), the raw one ~10 Hz.
        color_type = (
            CompressedImage if self.p("color_topic").endswith("/compressed") else Image
        )
        color = Subscriber(
            self,
            color_type,
            self.p("color_topic"),
            qos_profile=qos_profile_sensor_data,
        )
        # .../compressedDepth: CompressedImage; anything else: a raw Image (the simulation).
        depth_type = (
            CompressedImage
            if self.p("depth_topic").endswith("/compressedDepth")
            else Image
        )
        depth = Subscriber(
            self,
            depth_type,
            self.p("depth_topic"),
            qos_profile=qos_profile_sensor_data,
        )
        # Colour ~10 Hz, compressed depth ~1.3 Hz and later (PNG on the Jetson): a colour
        # queue of 30 still holds the frame that matches a late depth image.
        self.sync = ApproximateTimeSynchronizer(
            [color, depth], queue_size=30, slop=0.05
        )
        self.sync.registerCallback(self.on_frames)
        self.last_box, self.last_box_t = None, 0.0
        self.annotated_pub = (
            self.create_publisher(Image, self.p("annotated_topic"), 2)
            if self.p("annotated_topic")
            else None
        )
        self.frames = 0
        self.pairs = 0
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f'YOLO {self.p("model")}, target "{target or "any"}"; /object_distance on domain '
            f'{out_domain if out_domain >= 0 else "of the camera"}'
        )

    def report(self):
        """Every 10 s: synced colour+depth pairs, so a silent /object_distance can be told apart."""
        self.get_logger().info(
            f"{self.pairs / 10.0:.1f} colour+depth pairs/s, {self.frames} detections so far"
        )
        self.pairs = 0

    def on_frames(self, color_msg, depth_msg):
        self.pairs += 1
        try:
            img = decode_color(color_msg)
            depth = decode_depth(depth_msg)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return
        if depth.shape[:2] != img.shape[:2]:
            self.get_logger().warn(
                f"depth {depth.shape[:2]} and colour {img.shape[:2]} differ: "
                "is the depth topic the aligned one?",
                throttle_duration_sec=5.0,
            )
            return

        box = self.best_box(img)
        dist = None
        if box is not None:
            x1, y1, x2, y2, label, conf = box
            u, v = int(round((x1 + x2) / 2)), int(round((y1 + y2) / 2))
            if self.p("depth_mode") == "box":
                dist = depth_in_box(
                    depth,
                    (x1, y1, x2, y2),
                    float(self.p("box_fraction")),
                    float(self.p("box_percentile")),
                )
            else:
                dist = depth_at(depth, u, v, self.p("depth_window"))
            if dist is not None:
                self.pub.publish(Float32(data=dist))
            self.frames += 1
            if self.frames % 30 == 1:
                self.get_logger().info(
                    f'{label} {conf:.2f} at ({u}, {v}): '
                    f'{f"{dist:.3f} m" if dist is not None else "no depth"}'
                )
        if self.p("show") or self.annotated_pub is not None:
            self.draw(img, box, dist, color_msg.header)

    def best_box(self, img):
        """(x1, y1, x2, y2, label, confidence) of the tracked wanted detection, or None.

        select:=track (default): stay on the object followed so far (the box overlapping the
        last one most, IoU >= 0.3); when there is none (start, or lost for more than
        track_timeout_s) take the largest box, the nearest plant (the one the arm is at).
        With several plants in view, picking the most confident box each frame jumps from
        plant to plant (seen in the simulation with the camera on the claw).
        select:=confidence: the most confident box, every frame."""
        kw = {"verbose": False, "conf": float(self.p("min_confidence"))}
        if self.p("device"):
            kw["device"] = self.p("device")
        result = self.model(img, **kw)[0]
        target = self.p("target_class").strip()
        cands = []
        for xyxy, cls, conf in zip(
            result.boxes.xyxy.tolist(),
            result.boxes.cls.tolist(),
            result.boxes.conf.tolist(),
        ):
            label = self.names[int(cls)]
            if target and label != target:
                continue
            cands.append((*xyxy, label, conf))
        h, w = img.shape[:2]
        best = choose_box(
            cands,
            self.last_box if self.p("select") == "track" else None,
            w,
            h,
            by_confidence=self.p("select") == "confidence",
        )
        now = time.monotonic()
        if best is not None:
            self.last_box, self.last_box_t = best, now
        elif self.last_box is not None and now - self.last_box_t > float(
            self.p("track_timeout_s")
        ):
            self.last_box = None
        return best

    def draw(self, img, box, dist, header=None):
        if box is not None:
            x1, y1, x2, y2, label, conf = box
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
            cv2.drawMarker(img, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
            text = f"{label} {conf:.2f} " + (
                f"{dist:.3f} m" if dist is not None else "no depth"
            )
            cv2.putText(
                img,
                text,
                (int(x1), max(15, int(y1) - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
        if self.annotated_pub is not None:
            out = Image(
                header=header,
                height=img.shape[0],
                width=img.shape[1],
                encoding="bgr8",
                step=img.shape[1] * 3,
                data=img.tobytes(),
            )
            self.annotated_pub.publish(out)
        if self.p("show"):
            cv2.imshow("yolo_depth_tracker", img)
            cv2.waitKey(1)

    def shutdown(self):
        if self.out_node is not None:
            self.out_node.destroy_node()
            # Ctrl+C already shut every context down (rclpy's signal handler).
            rclpy.try_shutdown(context=self.out_ctx)


def main():
    rclpy.init()
    node = YoloDepthTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
