#!/usr/bin/env python3
"""
sim_robot_bridge.py: stands in for the Go2's Jetson in simulation.

The real robot gives the laptop (DDS domain 64, via robot_relay_wifi.py):
  /lidar3d/odom        nav_msgs/Odometry   lidar *sensor* pose in lidar3d_map
  /lidar3d/registered  PointCloud2         world-frame points (lidar3d_map)
  /lidar3d/path        nav_msgs/Path
  leg joints           (from /lf/lowstate; here /joint_states directly)
and accepts motion on TCP :17999 (go2_cmd_vel_tcp.py, 12-byte "!fff").

This node produces the same from Gazebo (through ros_gz_bridge):
  /sim/odom        <- gz /model/go2/odometry   (base pose)
  /sim/lidar/points<- gz lidar, in the sensor frame
  /sim/cmd_vel     -> gz /model/go2/cmd_vel    (VelocityControl)

The laptop's go2_odom_tf turns /lidar3d/odom into base_link using the
measured sensor_from_base, so this node publishes
  T_map_sensor = T_map_base * inverse(sensor_from_base).
Every cloud is stamped with an odom stamp that was already published, because
the self-filter looks up TF at the exact cloud stamp.

The maths and the TCP server have no ROS dependency (tested off-robot).
"""
import argparse
import json
import math
import os
import socket
import struct
import sys
import threading
import time

import numpy as np

VEL = struct.Struct("!fff")   # same wire format as go2_cmd_vel_tcp.py


# ------------------------------------------------------------- pose maths
def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_rotate(q, v):
    x, y, z, w = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty,
            v[1] + w * ty + z * tx - x * tz,
            v[2] + w * tz + x * ty - y * tx)


def compose(a, b):
    """(p, q) * (p, q): T_a * T_b."""
    pa, qa = a
    pb, qb = b
    r = quat_rotate(qa, pb)
    return (pa[0] + r[0], pa[1] + r[1], pa[2] + r[2]), quat_mul(qa, qb)


def inverse(t):
    p, q = t
    qi = quat_conj(q)
    r = quat_rotate(qi, p)
    return (-r[0], -r[1], -r[2]), qi


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quat_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def drift_step(est, prev_true, true, k_yaw, k_scale):
    """Advance a drifting pose estimate by one true motion step (planar).

    Point-LIO-like drift for testing: every radian turned adds k_yaw rad of
    heading error (always the same sign, like an uncorrected gyro bias), and
    every metre walked is reported as (1 + k_scale) m. est, prev_true and true
    are (p, q); the result keeps true's height and roll/pitch.
    """
    d = compose(inverse(prev_true), true)              # motion in the previous body frame
    dyaw = yaw_of(d[1])
    dx, dy = d[0][0] * (1 + k_scale), d[0][1] * (1 + k_scale)
    eyaw = yaw_of(est[1])
    c, s_ = math.cos(eyaw), math.sin(eyaw)
    nyaw = eyaw + dyaw + k_yaw * abs(dyaw)
    # Re-attach true roll/pitch to the drifted heading.
    tilt = quat_mul(quat_conj(quat_yaw(yaw_of(true[1]))), true[1])
    return ((est[0][0] + c * dx - s_ * dy, est[0][1] + s_ * dx + c * dy, true[0][2]),
            quat_mul(quat_yaw(nyaw), tilt))


def rot_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def transform_points(t, pts):
    """Apply (p, q) to an Nx3 array."""
    p, q = t
    return pts @ rot_matrix(q).T + np.asarray(p)


# ------------------------------------------------------------- PointCloud2
_DT = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def cloud_xyz(fields, point_step, data, big_endian=False):
    """Nx3 float64 from PointCloud2 parts (fields = [(name, offset, datatype)])."""
    byte_order = ">" if big_endian else "<"
    names, formats, offsets = [], [], []
    for name, offset, datatype in fields:
        if name in ("x", "y", "z"):
            names.append(name)
            formats.append(byte_order + _DT[datatype])
            offsets.append(offset)
    if names != ["x", "y", "z"] and sorted(names) != ["x", "y", "z"]:
        raise ValueError(f"cloud lacks x/y/z fields: {names}")
    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": point_step})
    arr = np.frombuffer(bytes(data), dtype=dtype)
    pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
    return pts[np.isfinite(pts).all(axis=1)]


def xyz_cloud_bytes(pts):
    return np.ascontiguousarray(pts, dtype="<f4").tobytes()


# ------------------------------------------------------------- TCP motion
class CmdVelTcpServer:
    """Accepts go2_cmd_vel_tcp.py clients; holds the newest velocity.

    Mirrors the robot side: clamps to GO2_MAX_LINEAR / GO2_MAX_ANGULAR and
    reports zero once no frame has arrived for `timeout` seconds.
    """

    def __init__(self, host="0.0.0.0", port=17999, timeout=0.5, max_linear=1.2, max_angular=1.5,
                 log=print):
        self.host, self.port, self.timeout = host, port, timeout
        self.max_linear, self.max_angular, self.log = max_linear, max_angular, log
        self._lock = threading.Lock()
        self._vel = (0.0, 0.0, 0.0)
        self._stamp = 0.0
        self.frames = 0
        self._stop = threading.Event()
        self._sock = None
        self._thread = None

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(1)
        self._sock.settimeout(0.5)
        self.port = self._sock.getsockname()[1]
        self._thread = threading.Thread(target=self._serve, name="cmd_vel_tcp", daemon=True)
        self._thread.start()
        self.log(f"cmd_vel TCP server started on port {self.port}")

    def stop(self):
        self._stop.set()
        if self._sock:
            self._sock.close()
        if self._thread:
            self._thread.join(timeout=2)

    def _clamp(self, vx, vy, vyaw):
        lim = self.max_linear
        return (max(-lim, min(lim, vx)), max(-lim, min(lim, vy)),
                max(-self.max_angular, min(self.max_angular, vyaw)))

    def _serve(self):
        while not self._stop.is_set():
            try:
                conn, addr = self._sock.accept()
            except (socket.timeout, OSError):
                continue
            self.log(f"cmd_vel client connected from {addr[0]}")
            conn.settimeout(self.timeout)
            buf = b""
            with conn:
                while not self._stop.is_set():
                    try:
                        chunk = conn.recv(4096)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not chunk:
                        break
                    buf += chunk
                    while len(buf) >= VEL.size:
                        vx, vy, vyaw = VEL.unpack(buf[:VEL.size])
                        buf = buf[VEL.size:]
                        if all(math.isfinite(v) for v in (vx, vy, vyaw)):
                            with self._lock:
                                self._vel = self._clamp(vx, vy, vyaw)
                                self._stamp = time.monotonic()
                                self.frames += 1
            self.log("cmd_vel client disconnected; stopping robot")
            with self._lock:
                self._vel, self._stamp = (0.0, 0.0, 0.0), 0.0

    def current(self, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            if now - self._stamp > self.timeout:
                return (0.0, 0.0, 0.0)
            return self._vel


# ------------------------------------------------------------- ROS node
def run_node(params, args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import JointState, PointCloud2, PointField

    sensor_from_base = params["sensor_from_base"]
    t_sensor_base = (tuple(sensor_from_base[:3]), tuple(sensor_from_base[3:]))
    t_base_sensor_virtual = inverse(t_sensor_base)                 # what the laptop expects
    t_base_lidar = (tuple(params["lidar_offset_xyz"]), (0.0, 0.0, 0.0, 1.0))  # the gz lidar link

    class Bridge(Node):
        def __init__(self):
            super().__init__("go2_sim_bridge")
            self.odom_pub = self.create_publisher(Odometry, "/lidar3d/odom", qos_profile_sensor_data)
            self.cloud_pub = self.create_publisher(PointCloud2, "/lidar3d/registered", qos_profile_sensor_data)
            self.path_pub = self.create_publisher(Path, "/lidar3d/path", 10)
            self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.cmd_pub = self.create_publisher(Twist, "/sim/cmd_vel", 10)
            self.create_subscription(Odometry, "/sim/odom", self.on_odom, 50)
            self.create_subscription(PointCloud2, "/sim/lidar/points", self.on_points, qos_profile_sensor_data)
            self.base = None              # (p, q) of base as reported (drifted if enabled)
            self.true_base = None         # (p, q) of base in the sim world (ground truth)
            self.drift = (args.drift_yaw, args.drift_scale)
            if any(self.drift):
                self.get_logger().warn(
                    f"DRIFT ON: {args.drift_yaw:+.3f} rad heading error per rad turned, "
                    f"{args.drift_scale:+.1%} distance scale (/lidar3d/* are no longer ground truth)")
            self.last_odom_stamp = None   # stamp of the newest *published* /lidar3d/odom
            self.path = Path()
            self.path.header.frame_id = "lidar3d_map"
            self.last_path_xy = None
            self.cloud_count = 0
            self.odom_count = 0
            self.ready = False
            self.tcp = CmdVelTcpServer(port=args.port, timeout=args.cmd_timeout,
                                       max_linear=args.max_linear, max_angular=args.max_angular,
                                       log=lambda m: self.get_logger().info(m))
            self.tcp.start()
            self.create_timer(1.0 / 20, self.publish_cmd)
            self.create_timer(1.0 / 30, self.publish_joints)
            self.create_timer(1.0, self.publish_path)
            self.create_timer(5.0, self.report)
            self.frames_reported = 0

        def on_odom(self, msg):
            p, o = msg.pose.pose.position, msg.pose.pose.orientation
            true = ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
            if not any(self.drift) or self.true_base is None:
                self.base = true
            else:
                self.base = drift_step(self.base, self.true_base, true, *self.drift)
            self.true_base = true
            sp, sq = compose(self.base, t_base_sensor_virtual)
            out = Odometry()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = "lidar3d_map"
            out.child_frame_id = "lidar3d_sensor"
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = sp
            (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
             out.pose.pose.orientation.z, out.pose.pose.orientation.w) = sq
            out.twist = msg.twist
            self.odom_pub.publish(out)
            self.last_odom_stamp = out.header.stamp
            self.odom_count += 1
            xy = (self.base[0][0], self.base[0][1])
            if self.last_path_xy is None or math.dist(xy, self.last_path_xy) > 0.1:
                ps = PoseStamped()
                ps.header = out.header
                ps.pose = out.pose.pose
                self.path.poses.append(ps)
                self.last_path_xy = xy

        def on_points(self, msg):
            if self.base is None or self.last_odom_stamp is None:
                return
            fields = [(f.name, f.offset, f.datatype) for f in msg.fields]
            try:
                pts = cloud_xyz(fields, msg.point_step, msg.data, msg.is_bigendian)
            except ValueError as e:
                self.get_logger().warn(str(e), throttle_duration_sec=5.0)
                return
            world = transform_points(compose(self.base, t_base_lidar), pts)
            out = PointCloud2()
            out.header.stamp = self.last_odom_stamp
            out.header.frame_id = "lidar3d_map"
            out.height, out.width = 1, len(world)
            out.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                          for i, n in enumerate("xyz")]
            out.is_bigendian, out.point_step = False, 12
            out.row_step, out.is_dense = 12 * len(world), True
            out.data = xyz_cloud_bytes(world)
            self.cloud_pub.publish(out)
            self.cloud_count += 1
            if not self.ready and self.odom_count:
                self.ready = True
                print("SIM READY: /lidar3d/odom + /lidar3d/registered publishing", flush=True)

        def publish_cmd(self):
            vx, vy, vyaw = self.tcp.current()
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(vyaw)
            self.cmd_pub.publish(t)

        def publish_joints(self):
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(params["leg_joint_names"])
            js.position = [float(v) for v in params["leg_joint_positions"]]
            self.joint_pub.publish(js)

        def publish_path(self):
            if self.path.poses:
                self.path.header.stamp = self.get_clock().now().to_msg()
                self.path_pub.publish(self.path)

        def report(self):
            frames = self.tcp.frames - self.frames_reported
            self.frames_reported = self.tcp.frames
            self.get_logger().info(f"5s: odom={self.odom_count} clouds={self.cloud_count} cmd_vel tcp={frames}")
            self.odom_count = self.cloud_count = 0

    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.tcp.stop()
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", required=True, help="sim_params.json from build_model.py")
    ap.add_argument("--port", type=int, default=17999)
    ap.add_argument("--cmd-timeout", type=float, default=0.5)
    ap.add_argument("--max-linear", type=float, default=1.2)
    ap.add_argument("--max-angular", type=float, default=1.5)
    ap.add_argument("--drift-yaw", type=float, default=float(os.environ.get("SIM_DRIFT_YAW", "0") or 0),
                    help="heading error added per radian turned (e.g. 0.02); 0 = ground truth")
    ap.add_argument("--drift-scale", type=float, default=float(os.environ.get("SIM_DRIFT_SCALE", "0") or 0),
                    help="distance scale error (e.g. 0.03 = 3%% long); 0 = ground truth")
    args = ap.parse_args()
    with open(args.params) as f:
        params = json.load(f)
    run_node(params, args)


if __name__ == "__main__":
    sys.exit(main())
