#!/usr/bin/env python3
"""
go2_teleop.py: drive the Go2 from the web dashboard without upsetting its
localization.

The dashboard's Drive pad sends a joystick (-1..1 per axis) while it is held.
This node turns it into /cmd_vel, the same topic Nav2's velocity smoother
feeds, so the dog walks through the usual motion bridge
(go2_cmd_vel_tcp.py -> sport API) in its normal gait. Nothing switches
modes, stands the dog up or down, or uses the Unitree app or remote, so
Point-LIO keeps tracking. Its IMU only sees what Nav2 itself would ask for:

  * the same speed limits as Nav2 (0.30 m/s forward, 0.30 rad/s turning; the
    slow turns keep the IMU from drifting), a slower reverse;
  * acceleration ramps on every axis: no jolt when a button is pressed or
    released;
  * deadman: no joystick message for `deadman` s (finger off, tab closed,
    Wi-Fi drop) ramps to a stop. go2_cmd_vel_tcp.py on top sends zero once
    /cmd_vel is silent for 0.5 s.

Manual control (op "take") first takes the dog from Nav2: it pauses the
explorer (/explore/resume False), cancels the Nav2 planner and controller
goals, and then cancels any goal that starts while manual control lasts
(survey, battery dock, tap-to-go, VLM). The stick is ignored for `take_delay`
s, while the velocity smoother sends its last stop. Op "release" stops the
dog and hands it back; exploration is not restarted automatically (press
Auto Explore). Op "stop" is an immediate stop that keeps manual control.

Subscribes: /teleop/command  (std_msgs/String JSON {"op": "take"|"release"|"stop"})
            /teleop/joy      (geometry_msgs/Twist: linear.x fwd, linear.y left,
                              angular.z left turn; each -1..1)
            /follow_path/_action/status, /compute_path_to_pose/_action/status
Publishes:  /cmd_vel         (geometry_msgs/Twist, only while driving or stopping)
            /teleop/status   (std_msgs/String JSON, 2 Hz)
            /explore/resume  (std_msgs/Bool False, on take)

Run in go2-3d by twin/start_inside.sh (TWIN_TELEOP=1, the default).
"""

import json
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

NAV_ACTIONS = ("/follow_path", "/compute_path_to_pose")
RUNNING = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)


def ramp(current, target, step):
    """Move `current` toward `target` by at most `step`."""
    if target > current:
        return min(target, current + step)
    return max(target, current - step)


class Teleop(Node):
    def __init__(self):
        super().__init__("go2_teleop")
        p = self.declare_parameter
        p("cmd_topic", "/cmd_vel")
        p("max_vx", 0.30)  # m/s forward (Nav2's desired_linear_vel)
        p("max_vx_back", 0.15)  # m/s reverse: the dog cannot see behind it
        p("max_vy", 0.20)  # m/s sideways
        p("max_wz", 0.30)  # rad/s (Nav2's rotate_to_heading_angular_vel)
        p("acc_lin", 0.40)  # m/s^2
        p("acc_ang", 0.60)  # rad/s^2
        p("rate", 20.0)  # Hz
        p("deadman", 0.35)  # s without a joystick message -> stop
        p("take_delay", 1.2)  # s after taking control before the stick acts
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.limits = {
            "vx": gp("max_vx"),
            "vx_back": gp("max_vx_back"),
            "vy": gp("max_vy"),
            "wz": gp("max_wz"),
        }
        self.acc_lin, self.acc_ang = gp("acc_lin"), gp("acc_ang")
        self.period = 1.0 / gp("rate")
        self.deadman, self.take_delay = gp("deadman"), gp("take_delay")

        self.active = False
        self.active_since = 0.0
        self.joy = (0.0, 0.0, 0.0)
        self.joy_time = 0.0
        self.vel = [0.0, 0.0, 0.0]  # vx, vy, wz actually sent
        self.stop_sends = 0  # zero commands still to send after stopping
        self.note = "Nav2 in control"
        self.last_cancel = 0.0

        self.cmd = self.create_publisher(Twist, gp("cmd_topic"), 10)
        self.status_pub = self.create_publisher(String, "/teleop/status", 10)
        self.resume_pub = self.create_publisher(Bool, "/explore/resume", 10)
        self.cancel = [
            self.create_client(CancelGoal, f"{a}/_action/cancel_goal")
            for a in NAV_ACTIONS
        ]
        self.create_subscription(String, "/teleop/command", self.on_command, 10)
        self.create_subscription(Twist, "/teleop/joy", self.on_joy, 10)
        for a in NAV_ACTIONS:
            self.create_subscription(
                GoalStatusArray, f"{a}/_action/status", self.on_nav_status, 10
            )
        self.create_timer(self.period, self.tick)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f"web drive ready: {self.limits['vx']:.2f} m/s, {self.limits['wz']:.2f} rad/s "
            f"on {gp('cmd_topic')}"
        )

    # ---------------------------------------------------------------- input
    def on_command(self, msg):
        try:
            op = json.loads(msg.data).get("op")
        except (ValueError, AttributeError):
            op = msg.data.strip()
        if op == "take":
            if not self.active:
                self.active, self.active_since = True, time.monotonic()
                self.joy_time = 0.0
                self.resume_pub.publish(Bool(data=False))
                self.cancel_nav()
                self.note = "manual control: Nav2 paused"
                self.get_logger().info(
                    "manual control: explorer paused, Nav2 goals cancelled"
                )
        elif op == "release":
            if self.active:
                self.active = False
                self.joy_time = 0.0
                self.note = "Nav2 in control (press Auto Explore to resume)"
                self.get_logger().info("manual control released")
        elif op == "stop":
            self.joy_time = 0.0
            self.vel = [0.0, 0.0, 0.0]
            self.stop_sends = 5
            self.note = "stopped"
        self.publish_status()

    def on_joy(self, msg):
        if not self.active:
            return
        clamp = lambda v: max(-1.0, min(1.0, float(v)))  # noqa: E731
        self.joy = (clamp(msg.linear.x), clamp(msg.linear.y), clamp(msg.angular.z))
        self.joy_time = time.monotonic()

    def on_nav_status(self, msg):
        """While driving by hand, nothing else may start a Nav2 leg."""
        if not self.active or not any(s.status in RUNNING for s in msg.status_list):
            return
        now = time.monotonic()
        if now - self.last_cancel > 1.0:
            self.last_cancel = now
            self.get_logger().info(
                "a Nav2 goal started during manual control: cancelled"
            )
            self.cancel_nav()

    def cancel_nav(self):
        # A zero goal id and stamp cancels every goal of the action server.
        for cli in self.cancel:
            if cli.service_is_ready():
                cli.call_async(CancelGoal.Request())

    # --------------------------------------------------------------- output
    def target(self, now):
        if not self.active or now - self.active_since < self.take_delay:
            return (0.0, 0.0, 0.0)
        if now - self.joy_time > self.deadman:
            return (0.0, 0.0, 0.0)
        jx, jy, jw = self.joy
        lim = self.limits
        vx = jx * (lim["vx"] if jx >= 0 else lim["vx_back"])
        return (vx, jy * lim["vy"], jw * lim["wz"])

    def tick(self):
        now = time.monotonic()
        tx, ty, tw = self.target(now)
        lin, ang = self.acc_lin * self.period, self.acc_ang * self.period
        was_moving = any(abs(v) > 1e-4 for v in self.vel)
        self.vel = [
            ramp(self.vel[0], tx, lin),
            ramp(self.vel[1], ty, lin),
            ramp(self.vel[2], tw, ang),
        ]
        moving = any(abs(v) > 1e-4 for v in self.vel)
        if was_moving and not moving:
            self.vel = [0.0, 0.0, 0.0]
            self.stop_sends = 5  # make sure the stop arrives
        if not moving and self.stop_sends <= 0:
            return  # silent: Nav2 (or the bridge's zero heartbeat) has /cmd_vel
        if not moving:
            self.stop_sends -= 1
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = self.vel
        self.cmd.publish(out)

    def publish_status(self):
        now = time.monotonic()
        moving = any(abs(v) > 1e-4 for v in self.vel)
        note = self.note
        if self.active and now - self.active_since < self.take_delay:
            note = "taking control from Nav2..."
        elif moving:
            note = "driving"
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "active": self.active,
                        "moving": moving,
                        "vx": round(self.vel[0], 3),
                        "vy": round(self.vel[1], 3),
                        "wz": round(self.vel[2], 3),
                        "limits": self.limits,
                        "note": note,
                    }
                )
            )
        )


def main():
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Never leave the dog walking on the last command.
        if any(abs(v) > 1e-4 for v in node.vel):
            node.cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
