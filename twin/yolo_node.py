#!/usr/bin/env python3
"""YOLO11 detections on the Go2 camera, turned into Nav2 obstacles.

Every 1 / rate_hz s the newest camera frame goes through YOLO (ultralytics,
CPU). All detections are published as vision_msgs/Detection2DArray. Each
detection whose class is in obstacle_classes is also placed in 3D:

  * LiDAR first: /lidar3d/registered is projected into the image and the
    20th-percentile depth of the points inside the box is the distance.
  * Ground plane otherwise (too few LiDAR points, e.g. glass or a thin chair
    leg): the ray through the bottom-centre of the box is intersected with the
    floor (base_link z - base_height).

and becomes a box of points (box width x obstacle_depth x obstacle_height)
standing on the floor, published as PointCloud2 in map_frame. An empty cloud
is published when nothing is seen, so the costmap layer can clear old marks.
Nav2 uses /yolo/obstacles only once it is an observation source of the
costmap (scripts/tools/add-yolo-costmap-source.sh).

Distances are only as good as the camera intrinsics (go2_camera_info.py) and
the base_link -> camera TF. The stock COCO model has no "door" class; pass a
custom model (model:=...) to detect doors or glass.

Subscribes: /camera/image_raw, /camera/camera_info, /lidar3d/registered, TF
Publishes:  /yolo/detections  (vision_msgs/Detection2DArray)
            /yolo/obstacles   (sensor_msgs/PointCloud2, map_frame)
            /yolo/markers     (visualization_msgs/MarkerArray, labelled boxes)
            /yolo/image       (sensor_msgs/Image, annotated; publish_image)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformException, TransformListener

DEFAULT_OBSTACLES = ('person,bicycle,chair,couch,potted plant,bed,dining table,toilet,tv,'
                     'laptop,suitcase,bench,refrigerator,backpack,dog,cat')


def transform_matrix(tf):
    """geometry_msgs/Transform -> (3x3 rotation, translation)."""
    q, t = tf.rotation, tf.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R, np.array([t.x, t.y, t.z])


def cloud_xyz(msg):
    """PointCloud2 -> Nx3 float array (finite points only)."""
    off = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'} <= off.keys() or msg.point_step == 0:
        return np.empty((0, 3))
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [off['x'], off['y'], off['z']], 'itemsize': msg.point_step})
    raw = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
    pts = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(np.float64)
    return pts[np.isfinite(pts).all(axis=1)]


def image_bgr(msg):
    """sensor_msgs/Image (bgr8 / rgb8 / mono8) -> HxWx3 uint8 BGR, or None."""
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding in ('bgr8', 'rgb8'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img
    if msg.encoding == 'mono8':
        img = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return np.repeat(img[:, :, None], 3, axis=2)
    return None


class YoloObstacles(Node):
    def __init__(self):
        super().__init__('go2_yolo')
        p = self.declare_parameter
        p('image_topic', '/camera/image_raw')
        p('camera_info_topic', '/camera/camera_info')
        p('cloud_topic', '/lidar3d/registered')
        p('model', '/opt/yolo/yolo11s.pt')
        p('rate_hz', 2.0)
        p('confidence', 0.4)
        p('image_size', 640)
        p('threads', 2)
        p('obstacle_classes', DEFAULT_OBSTACLES)   # comma list, or "all"
        p('map_frame', 'lidar3d_map')
        p('robot_base_frame', 'base_link')
        p('base_height', 0.30)        # m, base_link above the floor (Go2 standing)
        p('max_range', 6.0)           # m, farther detections are not placed
        p('min_lidar_points', 5)
        p('obstacle_height', 1.0)     # m, height of the injected box
        p('obstacle_depth', 0.3)      # m, thickness behind the visible face
        p('point_spacing', 0.05)      # m
        p('publish_image', True)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731

        self.map_frame, self.base_frame = gp('map_frame'), gp('robot_base_frame')
        self.conf, self.imgsz = float(gp('confidence')), int(gp('image_size'))
        self.base_height, self.max_range = float(gp('base_height')), float(gp('max_range'))
        self.min_lidar = int(gp('min_lidar_points'))
        self.obs_h, self.obs_d = float(gp('obstacle_height')), float(gp('obstacle_depth'))
        self.spacing = max(0.02, float(gp('point_spacing')))
        self.publish_image = bool(gp('publish_image'))
        classes = str(gp('obstacle_classes')).strip()
        self.obstacle_classes = None if classes == 'all' else {
            c.strip() for c in classes.split(',') if c.strip()}

        import torch
        from ultralytics import YOLO
        torch.set_num_threads(max(1, int(gp('threads'))))
        self.model = YOLO(gp('model'))
        self.names = self.model.names

        self.image = None
        self.image_seq = 0
        self.done_seq = 0
        self.info = None
        self.cloud = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.det_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.obs_pub = self.create_publisher(PointCloud2, '/yolo/obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/yolo/markers', 10)
        self.img_pub = self.create_publisher(Image, '/yolo/image', qos_profile_sensor_data)
        self.create_subscription(Image, gp('image_topic'), self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, gp('camera_info_topic'), self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, gp('cloud_topic'), self.on_cloud,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / max(0.1, float(gp('rate_hz'))), self.tick)
        self.get_logger().info(
            f'YOLO {gp("model")} on {gp("image_topic")} at {gp("rate_hz")} Hz, '
            f'obstacles: {classes}')

    def on_image(self, msg):
        self.image = msg
        self.image_seq += 1

    def on_info(self, msg):
        self.info = msg

    def on_cloud(self, msg):
        self.cloud = msg

    # ------------------------------------------------------------ main loop
    def tick(self):
        if self.image is None or self.image_seq == self.done_seq:
            return
        self.done_seq = self.image_seq
        msg = self.image
        img = image_bgr(msg)
        if img is None:
            self.get_logger().warn(f'unsupported image encoding {msg.encoding!r}', once=True)
            return

        res = self.model.predict(img, imgsz=self.imgsz, conf=self.conf, verbose=False)[0]
        boxes = res.boxes
        xyxy = boxes.xyxy.cpu().numpy() if len(boxes) else np.empty((0, 4))
        cls = boxes.cls.cpu().numpy().astype(int) if len(boxes) else np.empty(0, int)
        score = boxes.conf.cpu().numpy() if len(boxes) else np.empty(0)
        labels = [self.names[int(c)] for c in cls]

        self.publish_detections(msg.header, xyxy, labels, score)
        if self.publish_image:
            out = Image()
            out.header = msg.header
            plotted = res.plot()
            out.height, out.width = plotted.shape[:2]
            out.encoding, out.step = 'bgr8', plotted.shape[1] * 3
            out.data = plotted.tobytes()
            self.img_pub.publish(out)

        keep = [i for i, n in enumerate(labels)
                if self.obstacle_classes is None or n in self.obstacle_classes]
        self.publish_obstacles(msg, xyxy[keep], [labels[i] for i in keep])

    def publish_detections(self, header, xyxy, labels, score):
        arr = Detection2DArray()
        arr.header = header
        for (x1, y1, x2, y2), name, s in zip(xyxy, labels, score):
            d = Detection2D()
            d.header = header
            c = d.bbox.center
            cx, cy = float((x1 + x2) / 2), float((y1 + y2) / 2)
            if hasattr(c, 'position'):      # vision_msgs 4.x (Humble)
                c.position.x, c.position.y = cx, cy
            else:                           # vision_msgs 3.x
                c.x, c.y = cx, cy
            d.bbox.size_x, d.bbox.size_y = float(x2 - x1), float(y2 - y1)
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id, h.hypothesis.score = name, float(s)
            d.results.append(h)
            arr.detections.append(d)
        self.det_pub.publish(arr)

    # ------------------------------------------------------------ 3D placement
    def publish_obstacles(self, img_msg, xyxy, labels):
        header = img_msg.header
        cam_frame = header.frame_id
        out_header = type(header)()
        out_header.stamp, out_header.frame_id = header.stamp, self.map_frame
        points, markers = [], [Marker(action=Marker.DELETEALL, ns='yolo')]
        markers[0].header = out_header

        if len(xyxy) and self.info is not None:
            try:
                R_mc, t_mc = transform_matrix(self.tf_buffer.lookup_transform(
                    self.map_frame, cam_frame, Time()).transform)
                base = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, Time()).transform.translation
            except TransformException as e:
                self.get_logger().warn(f'no TF {self.map_frame} -> {cam_frame}: {e}',
                                       throttle_duration_sec=10.0)
                xyxy = []
            else:
                floor_z = base.z - self.base_height
                k = self.info.k
                fx, fy, cx, cy = k[0], k[4], k[2], k[5]
                # The image may be resized relative to camera_info.
                sx = img_msg.width / self.info.width if self.info.width else 1.0
                sy = img_msg.height / self.info.height if self.info.height else 1.0
                fx, cx, fy, cy = fx * sx, cx * sx, fy * sy, cy * sy
                cam_pts = self.lidar_in_camera(R_mc, t_mc)
                for i, ((x1, y1, x2, y2), name) in enumerate(zip(xyxy, labels)):
                    depth, src = self.box_depth(cam_pts, (x1, y1, x2, y2),
                                                (fx, fy, cx, cy), R_mc, t_mc, floor_z)
                    if depth is None or depth > self.max_range:
                        continue
                    corners = [R_mc @ np.array([(u - cx) / fx * dd, (y2 - cy) / fy * dd, dd]) + t_mc
                               for dd in (depth, depth + self.obs_d) for u in (x1, x2)]
                    points.extend(self.box_points(corners, floor_z))
                    markers.extend(self.box_markers(i, corners, floor_z, name, depth, src,
                                                    out_header))

        self.obs_pub.publish(create_cloud_xyz32(out_header, points))
        self.marker_pub.publish(MarkerArray(markers=markers))

    def lidar_in_camera(self, R_mc, t_mc):
        """Latest cloud (assumed in map_frame) in the camera optical frame, Nx3, z > 0.2."""
        if self.cloud is None:
            return np.empty((0, 3))
        pts = cloud_xyz(self.cloud)
        if self.cloud.header.frame_id and self.cloud.header.frame_id != self.map_frame:
            try:
                R, t = transform_matrix(self.tf_buffer.lookup_transform(
                    self.map_frame, self.cloud.header.frame_id, Time()).transform)
                pts = pts @ R.T + t
            except TransformException:
                return np.empty((0, 3))
        cam = (pts - t_mc) @ R_mc        # R_mc^T (p - t)
        return cam[cam[:, 2] > 0.2]

    def box_depth(self, cam_pts, box, intr, R_mc, t_mc, floor_z):
        x1, y1, x2, y2 = box
        fx, fy, cx, cy = intr
        if len(cam_pts):
            u = fx * cam_pts[:, 0] / cam_pts[:, 2] + cx
            v = fy * cam_pts[:, 1] / cam_pts[:, 2] + cy
            inside = (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
            if np.count_nonzero(inside) >= self.min_lidar:
                return float(np.percentile(cam_pts[inside, 2], 20)), 'lidar'
        # Ground plane: ray through the bottom-centre of the box meets the floor.
        ray = R_mc @ np.array([((x1 + x2) / 2 - cx) / fx, (y2 - cy) / fy, 1.0])
        if ray[2] >= -1e-3:
            return None, None                # box bottom at or above the horizon
        s = (floor_z - t_mc[2]) / ray[2]     # camera-z depth, since ray has z = 1
        return (float(s), 'floor') if s > 0 else (None, None)

    def box_points(self, corners, floor_z):
        """Filled box: the 4 floor corners (near-left, near-right, far-left, far-right)."""
        nl, nr, fl, fr = (c[:2] for c in corners)
        n_w = max(2, int(math.ceil(np.linalg.norm(nr - nl) / self.spacing)) + 1)
        n_d = max(2, int(math.ceil(self.obs_d / self.spacing)) + 1)
        n_h = max(2, int(math.ceil(self.obs_h / (2 * self.spacing))) + 1)
        pts = []
        for a in np.linspace(0.0, 1.0, n_w):
            near, far = nl + a * (nr - nl), fl + a * (fr - fl)
            for b in np.linspace(0.0, 1.0, n_d):
                x, y = near + b * (far - near)
                for z in np.linspace(floor_z + 0.05, floor_z + self.obs_h, n_h):
                    pts.append((float(x), float(y), float(z)))
        return pts

    def box_markers(self, i, corners, floor_z, name, depth, src, header):
        nl, nr, fl, fr = (c[:2] for c in corners)
        centre = (nl + nr + fl + fr) / 4.0
        width = float(np.linalg.norm(nr - nl))
        yaw = math.atan2(nr[1] - nl[1], nr[0] - nl[0])
        cube = Marker(ns='yolo', id=2 * i + 1, type=Marker.CUBE, action=Marker.ADD)
        cube.header = header
        cube.pose.position.x, cube.pose.position.y = float(centre[0]), float(centre[1])
        cube.pose.position.z = floor_z + self.obs_h / 2.0
        cube.pose.orientation.z, cube.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cube.scale.x, cube.scale.y, cube.scale.z = max(width, 0.05), self.obs_d, self.obs_h
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = 1.0, 0.5, 0.0, 0.35
        text = Marker(ns='yolo', id=2 * i + 2, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        text.header = header
        text.pose.position.x, text.pose.position.y = cube.pose.position.x, cube.pose.position.y
        text.pose.position.z = floor_z + self.obs_h + 0.2
        text.pose.orientation.w = 1.0
        text.scale.z = 0.2
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f'{name} {depth:.1f} m ({src})'
        return [cube, text]


def main():
    rclpy.init()
    node = YoloObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
