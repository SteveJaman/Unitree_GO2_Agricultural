#!/usr/bin/env python3
"""
go2_lidar3d_level.py: keep the 3D map and the dog's pose level with the body IMU.

Point-LIO on the dog runs gyro-only (no accelerometer, by design: leg impacts), so it
has no gravity reference. When its attitude estimate is knocked over it never comes
back: on 2026-11-12 (run-20261112-190610) it swung 70 deg in half a second while the
dog stood still, and from then on the pose and every scan were stored tilted; the dog
sank under the floor in RViz and the web app at ~45 deg.

The Go2's own body IMU (/lf/lowstate, imu_state.quaternion) is gravity-referenced.
This node compares the two: v, the body IMU's "up" seen through Point-LIO's pose, should
stay where it was at the start (v0, measured over the first baseline_s seconds; Point-LIO's
output frame is floor-aligned then). The rotation C that takes v back to v0 is the tilt
Point-LIO has lost; it has no yaw part (heading and position tracking stay Point-LIO's).
Points map as C p + offset; the offset changes with C so the dog's corrected position stays
put when C changes (only its later motion is turned).
C is low-pass filtered (tau_s: gait sway and timing between the two streams are noise),
and follows at once when it jumps by more than snap_deg (a divergence).

The dog's relay publishes Point-LIO's topics under /lidar3d/raw/ (GO2_LIDAR3D_RAW=1,
scripts/tools/relay_lidar3d_raw.py); this node republishes them levelled under their
usual names, so every consumer (go2_odom_tf, Nav2, RTAB-Map, RViz, the dashboard) is
unchanged:
  /lidar3d/raw/odom        -> /lidar3d/odom        pose rotated by C
  /lidar3d/raw/registered  -> /lidar3d/registered  points rotated by C (map-frame clouds)
  /lidar3d/raw/path        -> /lidar3d/path        poses rotated by C
  /lidar3d/level/correction_deg (std_msgs/Float32)  the tilt being removed
Before the first body IMU message everything passes through unchanged.

hold_height (default true): the dog's height stays at its start height, and each scan
moves with it: a flat-floor constraint (lab, greenhouse rows). After a divergence
Point-LIO's position drifts up or down too; the tilt alone does not fix that. Turn it off
(GO2_LIDAR3D_HOLD_HEIGHT=0) to map ramps or stairs.

Parameters: sensor_from_base [x y z qx qy qz qw] (as go2_odom_tf), baseline_s 3.0,
tau_s 2.0, snap_deg 10.0, warn_deg 5.0, hold_height true, lowstate_topic /lf/lowstate.
"""

import math

import numpy as np

UP = np.array([0.0, 0.0, 1.0])


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(m):
    """(x, y, z, w) of a rotation matrix."""
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
            0.25 * s,
        )
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = [0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    return (q[0], q[1], q[2], (m[k, j] - m[j, k]) / s)


def rotvec_to_matrix(r):
    angle = float(np.linalg.norm(r))
    if angle < 1e-12:
        return np.eye(3)
    k = r / angle
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * kx @ kx


def rotvec_between(a, b):
    """Rotation vector of the smallest rotation taking unit vector a onto b."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    axis = np.cross(a, b)
    s, c = float(np.linalg.norm(axis)), float(np.clip(np.dot(a, b), -1.0, 1.0))
    if s < 1e-12:
        return np.zeros(3)  # parallel (antiparallel cannot happen for a tilt < 180 deg)
    return axis / s * math.atan2(s, c)


class LevelCorrector:
    """The tilt correction, without ROS: feed body IMU attitudes and Point-LIO poses."""

    def __init__(
        self,
        sensor_from_base_quat=(0.0, 0.0, 0.0, 1.0),
        baseline_s=3.0,
        tau_s=2.0,
        snap_deg=10.0,
        hold_height=True,
    ):
        self.r_sb = quat_to_matrix(*sensor_from_base_quat)
        self.baseline_s = baseline_s
        self.tau_s = tau_s
        self.snap = math.radians(snap_deg)
        self.up_body = None  # gravity "up" in the body frame, from the body IMU
        self.v0 = None
        self._v_sum = np.zeros(3)
        self._t_start = None
        self._t_last = None
        self.rot = np.zeros(3)  # current correction (rotation vector, map frame)
        self.matrix = np.eye(3)
        self.offset = np.zeros(
            3
        )  # translation keeping the corrected position continuous
        self.hold_height = hold_height
        self.z0 = None  # the dog's height at the start (hold_height)
        self.snapped = 0

    def on_imu(self, w, x, y, z):
        """Body attitude (world <- body), gravity-referenced; only its tilt is used."""
        self.up_body = quat_to_matrix(x, y, z, w).T @ UP

    def apply(self, p):
        """A map-frame point (Point-LIO) in the levelled map."""
        return self.matrix @ np.asarray(p, float) + self.offset

    def on_pose(self, t, q_map_sensor, position=(0.0, 0.0, 0.0)):
        """Point-LIO pose (orientation x, y, z, w; position) at time t (s). Returns the
        correction matrix; positions map through apply(). When the correction changes, the
        offset changes with it so that the dog's corrected position does not move: only
        motion after that is turned by the new correction (a rotation about the map origin
        would throw the dog metres up or down after a large tilt)."""
        if self.up_body is None:
            return self.matrix
        v = quat_to_matrix(*q_map_sensor) @ self.r_sb @ self.up_body
        if self.v0 is None:
            if self._t_start is None:
                self._t_start = t
            self._v_sum += v
            if t - self._t_start >= self.baseline_s:
                self.v0 = self._v_sum / np.linalg.norm(self._v_sum)
                self.z0 = float(position[2])
            return self.matrix
        target = rotvec_between(v, self.v0)
        dt = 0.0 if self._t_last is None else max(0.0, t - self._t_last)
        self._t_last = t
        if np.linalg.norm(target - self.rot) > self.snap:
            self.rot = target
            self.snapped += 1
        else:
            self.rot = self.rot + min(1.0, dt / self.tau_s) * (target - self.rot)
        new = rotvec_to_matrix(self.rot)
        p = np.asarray(position, float)
        self.offset = self.offset + (self.matrix - new) @ p
        self.matrix = new
        if self.hold_height and self.z0 is not None:
            # A flat floor: the dog stays at its start height; scans move with it. After a
            # divergence Point-LIO's position climbs too (2026-11-12: +1.3 m standing still).
            self.offset[2] += self.z0 - float((new @ p + self.offset)[2])
        return self.matrix

    @property
    def degrees(self):
        return math.degrees(float(np.linalg.norm(self.rot)))


def rotate_cloud(data, point_step, offsets, matrix, translation=(0.0, 0.0, 0.0)):
    """PointCloud2 data (bytes) with float32 x, y, z at offsets, rotated by matrix, then moved."""
    buf = bytearray(data)
    n = len(buf) // point_step
    if n == 0:
        return bytes(buf)
    cols = [
        np.ndarray((n,), dtype="<f4", buffer=buf, offset=o, strides=(point_step,))
        for o in offsets
    ]
    xyz = np.stack(cols).astype(np.float64)
    out = (matrix @ xyz + np.asarray(translation, float).reshape(3, 1)).astype(
        np.float32
    )
    for c, row in zip(cols, out):
        c[:] = row
    return bytes(buf)


def main():
    import rclpy
    from nav_msgs.msg import Odometry, Path
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Float32
    from unitree_go.msg import LowState

    class Level(Node):
        def __init__(self):
            super().__init__("go2_lidar3d_level")
            p = self.declare_parameter
            p(
                "sensor_from_base",
                [0.267276, 0.0, -0.120571, 0.0, -0.991341, 0.0, 0.131316],
            )
            p("baseline_s", 3.0)
            p("tau_s", 2.0)
            p("snap_deg", 10.0)
            p("warn_deg", 5.0)
            p("lowstate_topic", "/lf/lowstate")
            p("hold_height", True)
            v = lambda n: self.get_parameter(n).value  # noqa: E731
            sfb = v("sensor_from_base")
            self.level = LevelCorrector(
                tuple(sfb[3:7]),
                v("baseline_s"),
                v("tau_s"),
                v("snap_deg"),
                bool(v("hold_height")),
            )
            self.warn_deg = float(v("warn_deg"))
            self.odom_frame = None
            self.warned = False
            self.counts = {"odom": 0, "cloud": 0, "imu": 0}
            q = qos_profile_sensor_data
            self.pub_odom = self.create_publisher(Odometry, "/lidar3d/odom", q)
            self.pub_cloud = self.create_publisher(
                PointCloud2, "/lidar3d/registered", q
            )
            self.pub_path = self.create_publisher(Path, "/lidar3d/path", q)
            self.pub_deg = self.create_publisher(
                Float32, "/lidar3d/level/correction_deg", 10
            )
            self.create_subscription(Odometry, "/lidar3d/raw/odom", self.on_odom, q)
            self.create_subscription(
                PointCloud2, "/lidar3d/raw/registered", self.on_cloud, q
            )
            self.create_subscription(Path, "/lidar3d/raw/path", self.on_path, q)
            self.create_subscription(
                LowState, v("lowstate_topic"), self.on_lowstate, 10
            )
            self.create_timer(5.0, self.report)
            self.get_logger().info(
                "Levelling /lidar3d/raw/* with the body IMU -> /lidar3d/*"
            )

        def on_lowstate(self, msg):
            w, x, y, z = (float(a) for a in msg.imu_state.quaternion)
            self.level.on_imu(w, x, y, z)
            self.counts["imu"] += 1

        def on_odom(self, msg):
            o = msg.pose.pose.orientation
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            before = self.level.snapped
            pos = msg.pose.pose.position
            m = self.level.on_pose(t, (o.x, o.y, o.z, o.w), (pos.x, pos.y, pos.z))
            self.odom_frame = msg.header.frame_id
            px, py, pz = self.level.apply((pos.x, pos.y, pos.z))
            pos.x, pos.y, pos.z = float(px), float(py), float(pz)
            qx, qy, qz, qw = matrix_to_quat(m @ quat_to_matrix(o.x, o.y, o.z, o.w))
            o.x, o.y, o.z, o.w = float(qx), float(qy), float(qz), float(qw)
            self.pub_odom.publish(msg)
            self.counts["odom"] += 1
            if self.level.snapped != before:
                self.get_logger().warn(
                    f"Point-LIO tilt jumped: {self.level.degrees:.1f} deg off the body IMU's gravity; "
                    "correcting (map before and after this moment may not line up exactly)"
                )
            if self.level.degrees > self.warn_deg and not self.warned:
                self.warned = True
                self.get_logger().warn(
                    f"Point-LIO tilt {self.level.degrees:.1f} deg off gravity: levelling it"
                )
            elif self.level.degrees < self.warn_deg / 2:
                self.warned = False

        def on_cloud(self, msg):
            if (
                self.odom_frame
                and msg.header.frame_id == self.odom_frame
                and self.level.degrees > 0.0
            ):
                off = {
                    f.name: f.offset
                    for f in msg.fields
                    if f.datatype == PointField.FLOAT32
                }
                if all(k in off for k in "xyz"):
                    msg.data = rotate_cloud(
                        msg.data,
                        msg.point_step,
                        (off["x"], off["y"], off["z"]),
                        self.level.matrix,
                        self.level.offset,
                    )
            self.pub_cloud.publish(msg)
            self.counts["cloud"] += 1

        def on_path(self, msg):
            m = self.level.matrix
            for ps in msg.poses:
                p, o = ps.pose.position, ps.pose.orientation
                p.x, p.y, p.z = (float(a) for a in self.level.apply((p.x, p.y, p.z)))
                qx, qy, qz, qw = matrix_to_quat(m @ quat_to_matrix(o.x, o.y, o.z, o.w))
                o.x, o.y, o.z, o.w = float(qx), float(qy), float(qz), float(qw)
            self.pub_path.publish(msg)

        def report(self):
            self.pub_deg.publish(Float32(data=float(self.level.degrees)))
            state = (
                "waiting for the body IMU"
                if self.level.up_body is None
                else "measuring the start (keep the dog still)"
                if self.level.v0 is None
                else f"correction {self.level.degrees:.1f} deg, jumps {self.level.snapped}"
            )
            c = self.counts
            self.get_logger().info(
                f"{state}; 5 s: odom {c['odom']}, clouds {c['cloud']}, imu {c['imu']}"
            )
            for k in c:
                c[k] = 0

    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    node = Level()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
