#!/usr/bin/env python3
"""Publish fake PointCloud2, Odometry, and TF for testing live_mapping.sh.

--arm: a fake D1 arm for the web dashboard instead (scripts/tools/mock_arm_twin.sh
runs it): /d1_arm/joint_states at home, and on {"op": "orbit"} from
/d1_arm/command (the dashboard's Orbit Scan button) the same sweep as
scripts/nodes/orbit_scan.py: scan pose, base -90 -> +90 deg in 30 steps with
the wrist counter-rotating, then home. /d1_arm/status reports it like
scripts/nodes/arm_state_bridge.py. --tf also places the robot (lidar3d_map ->
base_link) and walks it to each /goal_pose (0.3 m/s, turning to face the goal's
yaw on arrival), so the dashboard's Inspect mode and inspection_mission.py run
end to end; {"op": "scan_target"} plays a short aim-and-sweep as job "scan",
{"op": "periscope"} rises into arm_periscope.py's pose and holds it (job
"periscope", running until stopped or a scan / orbit takes over).
"""

import argparse
import json
import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

# As scripts/nodes/orbit_scan.py (degrees, joints 0-5).
SCAN_POSE = [0.0, -40.0, 60.0, 0.0, 30.0, 0.0]
BASE_START, BASE_END, STEPS, WRIST_GAIN = -90.0, 90.0, 30, 0.8
PERISCOPE_POSE = [0.0, 10.0, -45.0, 0.0, 80.0, 0.0]  # as scripts/nodes/arm_periscope.py
ARM_JOINTS = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7_1', 'Joint7_2']


class MockSim(Node):
    def __init__(self):
        super().__init__('mock_sim')
        self.cloud_pub = self.create_publisher(PointCloud2, '/registered_scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/state_estimation', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.t = 0.0
        self.create_timer(0.1, self.tick)  # 10 Hz
        self.get_logger().info('mock_sim: publishing /registered_scan, /state_estimation, /tf')

    def tick(self):
        self.t += 0.1
        # Move in a circle, radius 3m
        x = 3.0 * math.cos(0.2 * self.t)
        y = 3.0 * math.sin(0.2 * self.t)
        yaw = 0.2 * self.t + math.pi / 2

        # Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'body'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        self.odom_pub.publish(odom)

        # Point cloud: ring + 4 walls
        pts = []
        for i in range(180):
            a = math.radians(i * 2)
            pts.append((x + 4.0 * math.cos(a), y + 4.0 * math.sin(a), 0.5))
        for i in range(0, 100):
            t = i / 100.0 * 4
            if t < 1:   px, py = -10 + 20 * t, -10
            elif t < 2: px, py = 10, -10 + 20 * (t - 1)
            elif t < 3: px, py = 10 - 20 * (t - 2), 10
            else:       px, py = -10, 10 - 20 * (t - 3)
            pts.append((px, py, 0.5))

        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = 'map'
        cloud.height = 1
        cloud.width = len(pts)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = 12 * len(pts)
        cloud.is_dense = True
        cloud.data = b''.join(struct.pack('fff', *p) for p in pts)
        self.cloud_pub.publish(cloud)

        # TF: map -> body -> base_link
        tf1 = TransformStamped()
        tf1.header.stamp = self.get_clock().now().to_msg()
        tf1.header.frame_id = 'map'
        tf1.child_frame_id = 'body'
        tf1.transform.translation.x = x
        tf1.transform.translation.y = y
        tf1.transform.rotation.z = math.sin(yaw / 2)
        tf1.transform.rotation.w = math.cos(yaw / 2)
        self.tf_broadcaster.sendTransform(tf1)

        tf2 = TransformStamped()
        tf2.header.stamp = self.get_clock().now().to_msg()
        tf2.header.frame_id = 'body'
        tf2.child_frame_id = 'base_link'
        tf2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf2)


class MockArm(Node):
    """Fake D1 arm: home pose, and a timed orbit scan when the dashboard asks."""

    MOVE_S, STEP_S = 2.5, 0.5  # seconds: big moves, each sweep step

    def __init__(self, tf):
        super().__init__('mock_arm')
        self.js_pub = self.create_publisher(JointState, '/d1_arm/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/d1_arm/status', 10)
        self.create_subscription(String, '/d1_arm/command', self.on_command, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if tf else None
        if tf:
            self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.base = [0.0, 0.0, 0.0]  # x, y, yaw of the fake dog
        self.goal = None
        self.job = None
        self.hold = False  # periscope: keep 'running' after the move, until stopped
        self.pose = [0.0] * 6
        self.plan = []  # [(t_end, start, target)] still to run
        self.state = 'idle'
        self.create_timer(0.05, self.tick)  # 20 Hz
        self.create_timer(1.0, self.status)
        self.get_logger().info('mock_arm: /d1_arm/joint_states, /d1_arm/status; '
                               'orbit on /d1_arm/command' + (', TF lidar3d_map -> base_link' if tf else ''))

    def on_command(self, msg):
        try:
            op = json.loads(msg.data).get('op')
        except (ValueError, AttributeError):
            return
        if op in ('scan_target', 'orbit') and self.job == 'periscope' and self.state == 'running':
            self.plan, self.state, self.hold = [], 'stopped', False   # a real scan takes over
        if op == 'periscope' and not self.plan and self.state != 'running':
            self.start_plan('periscope', [list(PERISCOPE_POSE)], step=self.STEP_S)
            self.hold = True
            return
        if op == 'scan_target' and not self.plan:
            # Stand-in for arm_ik_scanner.py (no ikpy here): look forward-down, sweep, home.
            look = [0.0, 25.0, -12.0, 0.0, 47.0, 0.0]
            sweep = [[a, *look[1:]] for a in (0.0, 10.0, 5.0, 0.0, -5.0, -10.0, 0.0)]
            self.start_plan('scan', [look] + sweep + [[0.0] * 6], step=0.5)
            return
        if op == 'orbit' and not self.plan:
            targets = [list(SCAN_POSE)]
            for i in range(STEPS):
                base = BASE_START + (BASE_END - BASE_START) * i / (STEPS - 1)
                t = list(SCAN_POSE)
                t[0], t[5] = base, -base * WRIST_GAIN
                targets.append(t)
            targets.append([0.0] * 6)
            self.start_plan('orbit', targets, step=self.STEP_S)
        elif op == 'stop' and (self.plan or self.hold):
            self.hold = False
            self.plan = [(time.monotonic() + self.MOVE_S, self.MOVE_S, list(self.pose), [0.0] * 6)]
            self.state = 'stopped'
            self.get_logger().info('orbit scan stopped: homing')

    def start_plan(self, job, targets, step):
        now, start = time.monotonic(), list(self.pose)
        for n, t in enumerate(targets):
            dur = self.MOVE_S if n in (0, 1, len(targets) - 1) else step
            now += dur
            self.plan.append((now, dur, start, t))
            start = t
        self.job, self.state = job, 'running'
        self.get_logger().info(f'{job}: {len(targets)} moves, {now - time.monotonic():.0f} s')

    def on_goal(self, msg):
        q = msg.pose.orientation
        self.goal = (msg.pose.position.x, msg.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        self.get_logger().info(f'goal ({self.goal[0]:.2f}, {self.goal[1]:.2f}): walking')

    def walk(self, dt):
        """Fake Nav2: turn towards the goal, walk at 0.3 m/s, then turn to its yaw."""
        x, y, yaw = self.base
        gx, gy, gyaw = self.goal
        d = math.hypot(gx - x, gy - y)
        turn = lambda a: (a + math.pi) % (2 * math.pi) - math.pi  # noqa: E731
        if d > 0.05:
            want = math.atan2(gy - y, gx - x)
            yaw += max(-0.6 * dt, min(0.6 * dt, turn(want - yaw)))
            if abs(turn(want - yaw)) < 0.3:
                step = min(d, 0.3 * dt)
                x, y = x + math.cos(yaw) * step, y + math.sin(yaw) * step
        elif abs(turn(gyaw - yaw)) > 0.02:
            yaw += max(-0.6 * dt, min(0.6 * dt, turn(gyaw - yaw)))
        else:
            self.goal = None
        self.base = [x, y, yaw]

    def tick(self):
        now = time.monotonic()
        if self.goal:
            self.walk(0.05)
        while self.plan and now >= self.plan[0][0]:
            self.pose = list(self.plan.pop(0)[3])
            if not self.plan and self.state == 'running' and not self.hold:
                self.state = 'done'
        if self.plan:
            t_end, dur, start, target = self.plan[0]
            a = min(1.0, max(0.0, 1.0 - (t_end - now) / dur))
            a = a * a * (3 - 2 * a)  # ease in/out like a servo move
            self.pose = [s + (t - s) * a for s, t in zip(start, target)]
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ARM_JOINTS
        js.position = [math.radians(v) for v in self.pose] + [0.0, 0.0]
        self.js_pub.publish(js)
        if self.tf_broadcaster:
            tf = TransformStamped()
            tf.header.stamp = js.header.stamp
            tf.header.frame_id = 'lidar3d_map'
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = self.base[0], self.base[1]
            tf.transform.translation.z = 0.32  # standing trunk height
            tf.transform.rotation.z = math.sin(self.base[2] / 2)
            tf.transform.rotation.w = math.cos(self.base[2] / 2)
            self.tf_broadcaster.sendTransform(tf)

    def status(self):
        self.status_pub.publish(String(data=json.dumps(
            {'arm': True, 'orbit': self.state, 'job': self.job, 'commands': True, 'mock': True})))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--arm', action='store_true', help='fake D1 arm for the dashboard instead of the legacy topics')
    ap.add_argument('--tf', action='store_true', help='with --arm: also publish lidar3d_map -> base_link')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = MockArm(args.tf) if args.arm else MockSim()
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
