#!/usr/bin/env python3
"""Turn clicks in the Foxglove 3D panel into Nav2 goals.

Foxglove's "Publish point" tool sends a PointStamped in the panel's display
frame. This node transforms it into the Nav2 map frame, flattens it, aims the
goal along the robot -> point direction and hands it to goal_pose_nav.

Subscribes: /clicked_point       (geometry_msgs/PointStamped, any TF frame)
Publishes:  /goal_pose           (geometry_msgs/PoseStamped in map_frame)
            /click_goal/marker   (visualization_msgs/Marker, the active goal)
"""
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2)
from tf2_ros import Buffer, TransformException, TransformListener


class ClickToGoal(Node):
    def __init__(self):
        super().__init__('go2_click_to_goal')
        self.declare_parameter('input_topic', '/clicked_point')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('marker_topic', '/click_goal/marker')
        self.declare_parameter('map_frame', 'lidar3d_map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('max_goal_distance', 0.0)   # 0 = no limit

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('robot_base_frame').value
        self.max_dist = float(self.get_parameter('max_goal_distance').value)
        in_topic = self.get_parameter('input_topic').value
        goal_topic = self.get_parameter('goal_topic').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.marker_pub = self.create_publisher(
            Marker, self.get_parameter('marker_topic').value, 10)
        self.create_subscription(PointStamped, in_topic, self.on_click, 10)
        self.get_logger().info(
            f'click to goal: {in_topic} -> {goal_topic} in {self.map_frame}')

    def on_click(self, msg):
        # Clicks are UI events: use the newest TF, not the click's own stamp.
        msg.header.stamp = Time().to_msg()
        try:
            p = self.tf_buffer.transform(
                msg, self.map_frame, timeout=Duration(seconds=0.5)).point
        except TransformException as e:
            self.get_logger().warn(
                f'cannot transform click from {msg.header.frame_id!r} to {self.map_frame}: {e}')
            return

        yaw = 0.0
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()).transform.translation
            dx, dy = p.x - t.x, p.y - t.y
            dist = math.hypot(dx, dy)
            if self.max_dist > 0.0 and dist > self.max_dist:
                self.get_logger().warn(
                    f'goal {dist:.1f} m away exceeds max_goal_distance={self.max_dist} m; ignored')
                return
            if dist > 1e-3:
                yaw = math.atan2(dy, dx)
        except TransformException:
            self.get_logger().warn(
                f'no {self.map_frame} -> {self.base_frame} TF; goal yaw set to 0')

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = p.x
        goal.pose.position.y = p.y
        goal.pose.position.z = 0.0   # Nav2 plans in 2D
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(goal)
        self.publish_marker(goal)
        self.get_logger().info(
            f'goal ({p.x:.2f}, {p.y:.2f}) yaw {math.degrees(yaw):.0f} deg '
            f'[clicked in {msg.header.frame_id}]')

    def publish_marker(self, goal):
        m = Marker()
        m.header = goal.header
        m.ns = 'click_goal'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = goal.pose
        m.scale.x, m.scale.y, m.scale.z = 0.6, 0.12, 0.12
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 1.0
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    node = ClickToGoal()
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
