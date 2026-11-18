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
depth_window (5), color_topic, depth_topic, out_domain_id (-1), device ('' = auto),
show (false: an OpenCV window with the box and distance).
"""

import struct

import cv2
import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def decode_color(msg):
    """sensor_msgs/CompressedImage (JPEG/PNG) -> BGR uint8 image."""
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
        d("color_topic", "/camera/color/image_raw/compressed")
        d("depth_topic", "/camera/aligned_depth_to_color/image_raw/compressedDepth")
        d("out_domain_id", -1)
        d("device", "")
        d("show", False)
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

        color = Subscriber(
            self,
            CompressedImage,
            self.p("color_topic"),
            qos_profile=qos_profile_sensor_data,
        )
        depth = Subscriber(
            self,
            CompressedImage,
            self.p("depth_topic"),
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = ApproximateTimeSynchronizer([color, depth], queue_size=5, slop=0.05)
        self.sync.registerCallback(self.on_frames)
        self.frames = 0
        self.get_logger().info(
            f'YOLO {self.p("model")}, target "{target or "any"}"; /object_distance on domain '
            f'{out_domain if out_domain >= 0 else "of the camera"}'
        )

    def on_frames(self, color_msg, depth_msg):
        try:
            img = decode_color(color_msg)
            depth = decode_compressed_depth(depth_msg)
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
            dist = depth_at(depth, u, v, self.p("depth_window"))
            if dist is not None:
                self.pub.publish(Float32(data=dist))
            self.frames += 1
            if self.frames % 30 == 1:
                self.get_logger().info(
                    f'{label} {conf:.2f} at ({u}, {v}): '
                    f'{f"{dist:.3f} m" if dist is not None else "no depth"}'
                )
        if self.p("show"):
            self.draw(img, box, dist)

    def best_box(self, img):
        """(x1, y1, x2, y2, label, confidence) of the most confident wanted detection, or None."""
        kw = {"verbose": False, "conf": float(self.p("min_confidence"))}
        if self.p("device"):
            kw["device"] = self.p("device")
        result = self.model(img, **kw)[0]
        target = self.p("target_class").strip()
        best = None
        for xyxy, cls, conf in zip(
            result.boxes.xyxy.tolist(),
            result.boxes.cls.tolist(),
            result.boxes.conf.tolist(),
        ):
            label = self.names[int(cls)]
            if target and label != target:
                continue
            if best is None or conf > best[5]:
                best = (*xyxy, label, conf)
        return best

    def draw(self, img, box, dist):
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
        cv2.imshow("yolo_depth_tracker", img)
        cv2.waitKey(1)

    def shutdown(self):
        if self.out_node is not None:
            self.out_node.destroy_node()
            rclpy.shutdown(context=self.out_ctx)


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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
