#!/usr/bin/env python3
"""
cmu_bridge.py

Makes the CMU autonomy_stack_go2 Unity simulator look like the real Go2 as
seen by the go2-nav2-wifi laptop stack, so lidar3d_planning.launch.py,
lidar3d_controller.launch.py and frontier_explorer.py run unmodified.

  CMU sim (vehicleSimulator + Unity)        go2-nav2-wifi contract
  ----------------------------------        -----------------------------------
  /state_estimation  (map -> sensor)   -->  TF lidar3d_map -> base_link
                                            TF base_link -> lidar3d_sensor (static)
                                            /lidar3d/body_odom (lidar3d_map -> base_link)
  /registered_scan   (frame "map")     -->  /lidar3d/registered
                                            /lidar3d/obstacle_points
  /sim/cmd_vel  TwistStamped           <--  /cmd_vel  Twist (velocity_smoother)

Notes
  * The sim's vehicleSimulator subscribes to the absolute topic /cmd_vel with
    TwistStamped, which collides with Nav2's Twist /cmd_vel. The sim launch
    remaps it to /sim/cmd_vel; this node converts between the two.
  * vehicleSimulator holds the last commanded speed forever. This node sends
    zero velocity when /cmd_vel goes quiet for cmd_timeout seconds, which
    mirrors the real robot's sport-bridge timeout.
  * /lidar3d/odom is deliberately NOT published. On the real robot it is the
    raw lidar pose, and go2_odom_tf turns it into base_link using the measured
    Go2 lidar mount. The sim has no such mount, so this node publishes the
    base_link outputs (TF + /lidar3d/body_odom) directly instead.
  * The real stack runs the URDF self-filter to make /lidar3d/obstacle_points.
    The sim cloud has no robot body in it, so it is passed through unchanged.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def clamp(value, limit):
    """Clamp value to [-limit, limit]; limit <= 0 disables clamping."""
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


def twist_to_sim(twist, max_vx, max_vy, max_wz):
    """Return (vx, vy, wz) for the sim from a Nav2 Twist, with speed limits."""
    vals = (twist.linear.x, twist.linear.y, twist.angular.z)
    if not all(math.isfinite(v) for v in vals):
        return 0.0, 0.0, 0.0
    return (clamp(vals[0], max_vx), clamp(vals[1], max_vy), clamp(vals[2], max_wz))


class CmuSimBridge(Node):
    def __init__(self):
        super().__init__('go2_cmu_sim_bridge')

        p = self.declare_parameter
        p('sim_map_frame', 'map')
        p('map_frame', 'lidar3d_map')
        p('base_frame', 'base_link')
        p('sensor_frame', 'lidar3d_sensor')
        p('sim_odom_topic', '/state_estimation')
        p('sim_cloud_topic', '/registered_scan')
        p('sim_cmd_topic', '/sim/cmd_vel')
        p('nav_cmd_topic', '/cmd_vel')
        p('body_odom_topic', '/lidar3d/body_odom')
        p('registered_topic', '/lidar3d/registered')
        p('obstacle_topic', '/lidar3d/obstacle_points')
        # The sim's "sensor" frame sits vehicleHeight (0.366 m) above terrain.
        # Go2 base_link stands ~0.30 m high, so put base_link this far below it.
        p('sensor_above_base', 0.066)
        p('cmd_timeout', 0.5)
        p('cmd_rate_hz', 20.0)
        p('max_vx', 0.5)
        p('max_vy', 0.3)
        p('max_wz', 1.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.sim_map_frame = g('sim_map_frame')
        self.map_frame = g('map_frame')
        self.base_frame = g('base_frame')
        self.sensor_frame = g('sensor_frame')
        self.sensor_above_base = float(g('sensor_above_base'))
        self.cmd_timeout = float(g('cmd_timeout'))
        self.max_vx = float(g('max_vx'))
        self.max_vy = float(g('max_vy'))
        self.max_wz = float(g('max_wz'))

        self.tf_br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self.publish_static_sensor_tf()

        self.body_odom_pub = self.create_publisher(Odometry, g('body_odom_topic'), 10)
        self.registered_pub = self.create_publisher(
            PointCloud2, g('registered_topic'), qos_profile_sensor_data)
        self.obstacle_pub = self.create_publisher(
            PointCloud2, g('obstacle_topic'), qos_profile_sensor_data)
        self.sim_cmd_pub = self.create_publisher(TwistStamped, g('sim_cmd_topic'), 5)

        self.create_subscription(Odometry, g('sim_odom_topic'), self.on_sim_odom, 10)
        self.create_subscription(
            PointCloud2, g('sim_cloud_topic'), self.on_sim_cloud, qos_profile_sensor_data)
        self.create_subscription(Twist, g('nav_cmd_topic'), self.on_nav_cmd, 10)

        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_cmd_time = None
        self.sent_stop = False
        self.warned_frames = set()
        self.odom_count = 0
        self.cloud_count = 0
        self.create_timer(1.0 / float(g('cmd_rate_hz')), self.tick_cmd)
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f'bridging {g("sim_odom_topic")}, {g("sim_cloud_topic")} -> '
            f'{self.map_frame}/{self.base_frame}; {g("nav_cmd_topic")} -> {g("sim_cmd_topic")}')

    # --- TF / odometry -----------------------------------------------------
    def publish_static_sensor_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.sensor_frame
        t.transform.translation.z = self.sensor_above_base
        t.transform.rotation.w = 1.0
        self.static_br.sendTransform(t)

    def on_sim_odom(self, msg: Odometry):
        self.odom_count += 1
        pose = msg.pose.pose

        # vehicleSimulator publishes at 200 Hz; that is plenty for TF and
        # wasteful for odometry consumers, so keep TF at full rate and odom
        # at 50 Hz.
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z - self.sensor_above_base
        t.transform.rotation = pose.orientation
        self.tf_br.sendTransform(t)

        if self.odom_count % 4:
            return
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = t.transform.translation.x
        out.pose.pose.position.y = t.transform.translation.y
        out.pose.pose.position.z = t.transform.translation.z
        out.pose.pose.orientation = pose.orientation
        # vehicleSimulator twist is already in the body frame (fwd, left, yaw).
        out.twist.twist.linear.x = msg.twist.twist.linear.x
        out.twist.twist.linear.y = msg.twist.twist.linear.y
        out.twist.twist.angular.z = msg.twist.twist.angular.z
        self.body_odom_pub.publish(out)

    # --- Point cloud -------------------------------------------------------
    def on_sim_cloud(self, msg: PointCloud2):
        self.cloud_count += 1
        frame = msg.header.frame_id.lstrip('/')
        if frame == self.sim_map_frame or frame == '':
            msg.header.frame_id = self.map_frame
        elif frame not in self.warned_frames:
            # Unknown frame: forward as-is so it is visible in RViz, but the
            # costmap will drop it unless that frame exists in TF.
            self.warned_frames.add(frame)
            self.get_logger().warn(
                f'cloud frame "{frame}" is not the sim map frame '
                f'"{self.sim_map_frame}"; forwarding unchanged')
        self.registered_pub.publish(msg)
        self.obstacle_pub.publish(msg)

    # --- Velocity ----------------------------------------------------------
    def on_nav_cmd(self, msg: Twist):
        self.last_cmd = twist_to_sim(msg, self.max_vx, self.max_vy, self.max_wz)
        self.last_cmd_time = self.get_clock().now()
        self.publish_sim_cmd(*self.last_cmd)
        self.sent_stop = False

    def tick_cmd(self):
        if self.last_cmd_time is None:
            return
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout and not self.sent_stop:
            self.publish_sim_cmd(0.0, 0.0, 0.0)
            self.sent_stop = True

    def publish_sim_cmd(self, vx, vy, wz):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'vehicle'
        out.twist.linear.x = vx
        out.twist.linear.y = vy
        out.twist.angular.z = wz
        self.sim_cmd_pub.publish(out)

    def report(self):
        self.get_logger().info(
            f'sim odom msgs={self.odom_count} clouds={self.cloud_count} '
            f'last_cmd=({self.last_cmd[0]:.2f}, {self.last_cmd[2]:.2f})')
        if self.odom_count == 0:
            self.get_logger().warn('no /state_estimation yet - is vehicleSimulator running?')
        elif self.cloud_count == 0:
            self.get_logger().warn('no /registered_scan yet - is the Unity Model.x86_64 '
                                   'running and connected to ros_tcp_endpoint?')


def main(args=None):
    rclpy.init(args=args)
    node = CmuSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_sim_cmd(0.0, 0.0, 0.0)
        except Exception:  # noqa: BLE001 - shutting down
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
