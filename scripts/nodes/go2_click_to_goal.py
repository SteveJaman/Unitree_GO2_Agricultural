#!/usr/bin/env python3
"""Turn clicks in the Foxglove 3D panel into safe Nav2 goals.

Foxglove's "Publish point" tool sends a PointStamped in the panel's display
frame. A click often lands on a desk, a wall or the inflation around them, so
the point is transformed into the costmap frame and snapped to the nearest
navigable floor before it becomes a goal:

  1. BFS outward from the clicked cell, ring by ring (8-connected), up to
     max_snap_distance.
  2. A cell qualifies if it and every neighbour within clearance_cells
     (1 -> a 3x3 block) are known and cost < free_threshold. Unknown (-1)
     is never safe.
  3. Of the qualifying cells in the first ring that has any, the one nearest
     the click (Euclidean) wins.

The goal faces along the robot -> goal direction. No costmap yet, or no safe
cell within reach, means no goal is sent.

Subscribes: /clicked_point          (geometry_msgs/PointStamped, any TF frame)
            /global_costmap/costmap (nav_msgs/OccupancyGrid, 0-100, -1 unknown)
Publishes:  /goal_pose              (geometry_msgs/PoseStamped, costmap frame)
            /clicked_goal_markers   (visualization_msgs/MarkerArray: grey
                                     cylinder at the click, green arrow at the goal)
"""
import math
from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2)
from tf2_ros import Buffer, TransformException, TransformListener

NEIGHBOURS_8 = ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1))


class ClickToGoal(Node):
    def __init__(self):
        super().__init__('go2_click_to_goal')
        self.declare_parameter('input_topic', '/clicked_point')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('marker_topic', '/clicked_goal_markers')
        self.declare_parameter('map_frame', 'lidar3d_map')   # used if the costmap has no frame
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('free_threshold', 50)         # cost < this is floor
        self.declare_parameter('clearance_cells', 1)         # 1 = 3x3 block must be free
        self.declare_parameter('max_snap_distance', 2.0)     # m from the click
        self.declare_parameter('max_goal_distance', 0.0)     # m from the robot, 0 = no limit
        self.declare_parameter('floor_z', 0.0)               # marker height in the map frame

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('robot_base_frame').value
        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.clearance = max(0, int(self.get_parameter('clearance_cells').value))
        self.max_snap = float(self.get_parameter('max_snap_distance').value)
        self.max_dist = float(self.get_parameter('max_goal_distance').value)
        self.floor_z = float(self.get_parameter('floor_z').value)
        in_topic = self.get_parameter('input_topic').value
        costmap_topic = self.get_parameter('costmap_topic').value
        goal_topic = self.get_parameter('goal_topic').value

        self.costmap = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10)

        # QoS for costmap: match Nav2's costmap publisher (as frontier_explorer.py)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.create_subscription(OccupancyGrid, costmap_topic, self.on_costmap, map_qos)
        self.create_subscription(PointStamped, in_topic, self.on_click, 10)
        self.get_logger().info(
            f'click to goal: {in_topic} -> {goal_topic}, snapped on {costmap_topic} '
            f'(cost < {self.free_threshold}, {2 * self.clearance + 1}x{2 * self.clearance + 1} '
            f'clearance, within {self.max_snap} m)')

    def on_costmap(self, msg):
        self.costmap = msg

    # ------------------------------------------------------------ grid search
    def cell_free(self, data, w, h, x, y):
        if not (0 <= x < w and 0 <= y < h):
            return False
        c = data[y * w + x]
        return 0 <= c < self.free_threshold

    def cell_safe(self, data, w, h, x, y):
        r = self.clearance
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if not self.cell_free(data, w, h, x + dx, y + dy):
                    return False
        return True

    def snap_to_floor(self, grid, wx, wy):
        """Nearest safe cell centre to world (wx, wy), or None."""
        info = grid.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        data = grid.data
        # Clicks off the grid start from the nearest edge cell.
        cx = min(max(int(math.floor((wx - ox) / res)), 0), w - 1)
        cy = min(max(int(math.floor((wy - oy) / res)), 0), h - 1)
        max_ring = int(math.ceil(self.max_snap / res))

        seen = {(cx, cy)}
        ring = [(cx, cy)]
        for _ in range(max_ring + 1):
            safe = [c for c in ring if self.cell_safe(data, w, h, c[0], c[1])]
            if safe:
                bx, by = min(safe, key=lambda c: (
                    (ox + (c[0] + 0.5) * res - wx) ** 2 + (oy + (c[1] + 0.5) * res - wy) ** 2))
                gx, gy = ox + (bx + 0.5) * res, oy + (by + 0.5) * res
                if math.hypot(gx - wx, gy - wy) <= self.max_snap + res:
                    return gx, gy
                return None
            nxt = []
            for x, y in ring:
                for dx, dy in NEIGHBOURS_8:
                    n = (x + dx, y + dy)
                    if n not in seen and 0 <= n[0] < w and 0 <= n[1] < h:
                        seen.add(n)
                        nxt.append(n)
            if not nxt:
                break
            ring = nxt
        return None

    # ------------------------------------------------------------ click
    def on_click(self, msg):
        grid = self.costmap
        if grid is None:
            self.get_logger().warn('no costmap received yet; click ignored')
            return
        frame = grid.header.frame_id or self.map_frame

        # Clicks are UI events: use the newest TF, not the click's own stamp.
        msg.header.stamp = Time().to_msg()
        try:
            p = self.tf_buffer.transform(msg, frame, timeout=Duration(seconds=0.5)).point
        except TransformException as e:
            self.get_logger().warn(
                f'cannot transform click from {msg.header.frame_id!r} to {frame}: {e}')
            return

        snapped = self.snap_to_floor(grid, p.x, p.y)
        if snapped is None:
            self.get_logger().warn(
                f'no navigable floor within {self.max_snap} m of ({p.x:.2f}, {p.y:.2f}); '
                'no goal sent')
            self.publish_markers(frame, p, None)
            return
        gx, gy = snapped

        yaw = 0.0
        try:
            t = self.tf_buffer.lookup_transform(frame, self.base_frame, Time()).transform.translation
            dx, dy = gx - t.x, gy - t.y
            dist = math.hypot(dx, dy)
            if self.max_dist > 0.0 and dist > self.max_dist:
                self.get_logger().warn(
                    f'goal {dist:.1f} m away exceeds max_goal_distance={self.max_dist} m; ignored')
                return
            if dist > 1e-3:
                yaw = math.atan2(dy, dx)
        except TransformException:
            self.get_logger().warn(f'no {frame} -> {self.base_frame} TF; goal yaw set to 0')

        goal = PoseStamped()
        goal.header.frame_id = frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = 0.0   # Nav2 plans in 2D
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(goal)
        self.publish_markers(frame, p, goal)

        shift = math.hypot(gx - p.x, gy - p.y)
        self.get_logger().info(
            f'goal ({gx:.2f}, {gy:.2f}) yaw {math.degrees(yaw):.0f} deg '
            f'[clicked ({p.x:.2f}, {p.y:.2f}) in {msg.header.frame_id}'
            + (f', snapped {shift:.2f} m to free floor]' if shift > grid.info.resolution else ']'))

    # ------------------------------------------------------------ markers
    def publish_markers(self, frame, click, goal):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()

        c = Marker()
        c.header.frame_id, c.header.stamp = frame, stamp
        c.ns, c.id = 'clicked_goal', 0
        c.type, c.action = Marker.CYLINDER, Marker.ADD
        c.pose.position.x, c.pose.position.y, c.pose.position.z = click.x, click.y, click.z
        c.pose.orientation.w = 1.0
        c.scale.x, c.scale.y, c.scale.z = 0.3, 0.3, 0.5
        c.color.r, c.color.g, c.color.b, c.color.a = 0.6, 0.6, 0.6, 0.4
        arr.markers.append(c)

        a = Marker()
        a.header.frame_id, a.header.stamp = frame, stamp
        a.ns, a.id = 'clicked_goal', 1
        a.type = Marker.ARROW
        if goal is None:
            a.action = Marker.DELETE
        else:
            a.action = Marker.ADD
            a.pose.position.x, a.pose.position.y = goal.pose.position.x, goal.pose.position.y
            a.pose.position.z = self.floor_z
            a.pose.orientation = goal.pose.orientation
            a.scale.x, a.scale.y, a.scale.z = 0.6, 0.12, 0.12
            a.color.r, a.color.g, a.color.b, a.color.a = 0.1, 0.9, 0.2, 1.0
        arr.markers.append(a)

        self.marker_pub.publish(arr)


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
