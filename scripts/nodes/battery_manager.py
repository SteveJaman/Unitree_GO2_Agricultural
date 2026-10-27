#!/usr/bin/env python3
"""Battery monitor and low-battery return to the dock (wifi_3d_nav.sh).

Watches the Go2 battery (unitree_go/LowState bms_state.soc). The Wi-Fi relay
forwards the low-rate copy /lf/lowstate; /lowstate is also subscribed in case
the full-rate topic is bridged instead. Whichever arrives is used. The level
is always republished as sensor_msgs/BatteryState on /battery_state (for the
web dashboard). The docking below only runs with auto_dock (--ai):

  soc < low_soc (15 %)    -> /explore/resume False (explorer stops, cancels its
                             goal), then /goal_pose to the dock (default the
                             map origin = where the session started). The goal
                             is re-sent every resend_period s until the robot
                             is within dock_tolerance.
  while soc < resume_soc  -> any /explore/resume True is answered with False:
          (20 %)             the explorer cannot be restarted.
  soc >= resume_soc       -> released. The explorer is NOT restarted
                             automatically; click START again.

Runs inside go2-lidar3d (it needs the unitree_go messages).

Subscribes: /lf/lowstate, /lowstate (unitree_go/LowState), /explore/resume, TF
Publishes:  /battery_state (sensor_msgs/BatteryState, percentage 0-1, voltage),
            /explore/resume (std_msgs/Bool), /goal_pose (geometry_msgs/PoseStamped),
            /battery_manager/status (std_msgs/String, 1 Hz)
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from unitree_go.msg import LowState


class BatteryManager(Node):
    def __init__(self):
        super().__init__('battery_manager')
        self.declare_parameter('lowstate_topics', ['/lf/lowstate', '/lowstate'])
        self.declare_parameter('auto_dock', True)        # False = monitor only
        self.declare_parameter('low_soc', 15.0)
        self.declare_parameter('resume_soc', 20.0)
        self.declare_parameter('dock_x', 0.0)
        self.declare_parameter('dock_y', 0.0)
        self.declare_parameter('dock_yaw', 0.0)
        self.declare_parameter('dock_tolerance', 0.5)   # m
        self.declare_parameter('resend_period', 30.0)   # s
        self.declare_parameter('map_frame', 'lidar3d_map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('resume_topic', '/explore/resume')
        self.declare_parameter('goal_topic', '/goal_pose')
        gp = lambda k: self.get_parameter(k).value  # noqa: E731

        self.low = float(gp('low_soc'))
        self.high = float(gp('resume_soc'))
        self.dock = (float(gp('dock_x')), float(gp('dock_y')), float(gp('dock_yaw')))
        self.tol = float(gp('dock_tolerance'))
        self.resend = float(gp('resend_period'))
        self.map_frame, self.base_frame = gp('map_frame'), gp('robot_base_frame')
        self.auto_dock = bool(gp('auto_dock'))

        self.soc = None
        self.voltage = float('nan')
        self.soc_topic = None
        self.last_soc_time = None
        self.state = 'NORMAL'          # NORMAL | RETURNING | DOCKED
        self.goal_due = None           # time the next dock goal is sent
        self.start = self.get_clock().now()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.resume_pub = self.create_publisher(Bool, gp('resume_topic'), 10)
        self.goal_pub = self.create_publisher(PoseStamped, gp('goal_topic'), 10)
        self.status_pub = self.create_publisher(String, '/battery_manager/status', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        for t in gp('lowstate_topics'):
            self.create_subscription(LowState, t, lambda m, t=t: self.on_lowstate(m, t),
                                     qos_profile_sensor_data)
        self.create_subscription(Bool, gp('resume_topic'), self.on_resume, 10)
        self.create_timer(1.0, self.tick)
        if not self.auto_dock:
            self.get_logger().info('battery monitor only (auto_dock false): /battery_state')
            return
        self.get_logger().info(
            f'battery manager: dock below {self.low:.0f} %, explorer blocked until '
            f'{self.high:.0f} %, dock ({self.dock[0]:.2f}, {self.dock[1]:.2f}) in {self.map_frame}')

    # ------------------------------------------------------------ inputs
    def on_lowstate(self, msg, topic):
        soc = getattr(getattr(msg, 'bms_state', None), 'soc', None)
        if soc is None:
            return
        if self.soc_topic != topic:
            self.soc_topic = topic
            self.get_logger().info(f'battery from {topic}: {soc} %')
        self.soc = float(soc)
        self.voltage = float(getattr(msg, 'power_v', float('nan')))
        self.last_soc_time = self.get_clock().now()

    def on_resume(self, msg):
        if msg.data and self.auto_dock and self.state != 'NORMAL':
            self.get_logger().warn(
                f'explorer START refused: battery {self.soc:.0f} % < {self.high:.0f} %')
            self.pause_explorer()

    # ------------------------------------------------------------ loop
    def tick(self):
        now = self.get_clock().now()
        if self.last_soc_time is None or (now - self.last_soc_time).nanoseconds > 30e9:
            if (now - self.start).nanoseconds > 30e9:
                self.get_logger().warn(
                    'no battery data on /lf/lowstate or /lowstate for 30 s '
                    '(is the robot relay forwarding it?)', throttle_duration_sec=60.0)
            self.publish_status(now)
            return

        self.publish_battery(now)
        if not self.auto_dock:
            self.publish_status(now)
            return

        if self.state == 'NORMAL' and self.soc < self.low:
            self.get_logger().warn(f'battery {self.soc:.0f} % < {self.low:.0f} %: '
                                   'pausing the explorer and returning to the dock')
            self.state = 'RETURNING'
            self.pause_explorer()
            # Let the explorer finish its stop before the dock goal goes out.
            self.goal_due = now.nanoseconds + 2e9
        elif self.state != 'NORMAL' and self.soc >= self.high:
            self.get_logger().info(f'battery {self.soc:.0f} % >= {self.high:.0f} %: '
                                   'explorer may be restarted (click START)')
            self.state = 'NORMAL'
            self.goal_due = None

        if self.state == 'RETURNING':
            dist = self.distance_to_dock()
            if dist is not None and dist <= self.tol:
                self.get_logger().info(f'at the dock ({dist:.2f} m): swap or charge the battery')
                self.state = 'DOCKED'
            elif self.goal_due is not None and now.nanoseconds >= self.goal_due:
                self.send_dock_goal(dist)
                self.goal_due = now.nanoseconds + self.resend * 1e9
        self.publish_status(now)

    # ------------------------------------------------------------ actions
    def pause_explorer(self):
        self.resume_pub.publish(Bool(data=False))

    def distance_to_dock(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()).transform.translation
        except TransformException:
            return None
        return math.hypot(t.x - self.dock[0], t.y - self.dock[1])

    def send_dock_goal(self, dist):
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = self.dock[0], self.dock[1]
        goal.pose.orientation.z = math.sin(self.dock[2] / 2.0)
        goal.pose.orientation.w = math.cos(self.dock[2] / 2.0)
        self.goal_pub.publish(goal)
        self.get_logger().info(
            'dock goal sent' + (f' ({dist:.1f} m away)' if dist is not None else ''))

    def publish_battery(self, now):
        b = BatteryState()
        b.header.stamp = now.to_msg()
        b.percentage = self.soc / 100.0
        b.voltage = self.voltage
        b.present = True
        b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery_pub.publish(b)

    def publish_status(self, now):
        soc = 'unknown' if self.soc is None else f'{self.soc:.0f}%'
        state = self.state if self.auto_dock else 'MONITOR'
        self.status_pub.publish(String(data=f'{state} soc={soc}'))


def main():
    rclpy.init()
    node = BatteryManager()
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
