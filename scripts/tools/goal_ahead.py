#!/usr/bin/env python3
"""
goal_ahead.py: drive the dog a set distance straight ahead through the session's Nav2,
and stop it cleanly. For traversability tests (walk over a test object).

This stack has no bt_navigator (no NavigateToPose): goals go to go2_goal_pose_nav on
/goal_pose (PoseStamped, lidar3d_map), which calls the planner (compute_path_to_pose)
and the controller (follow_path). This script publishes one goal there, logs the gait
supervisor's /gait/status while the dog walks, and stops when:
  * the dog is within --tolerance of the goal (done),
  * --timeout passes, or
  * it gets SIGINT / SIGTERM (Ctrl+C, kill, docker exec ... pkill -INT).
On any stop other than arrival it cancels ALL goals of follow_path and
compute_path_to_pose (an empty CancelGoal request), so the controller stops and the
velocity smoother brings the dog to a halt. Only this process exits.

  goal_ahead.py --distance 2.0          # 2 m along the dog's current heading
  goal_ahead.py --cancel                # cancel any running goal and exit (the abort)

Run it inside go2-lidar3d with the session's DDS settings (source /tmp/go2-trav/env.sh),
detached, so stopping it cannot touch the session: see docs/hardware/traversability_handoff.md.
"""

import argparse
import json
import math
import signal
import sys
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

STOP = {"why": None}


def on_signal(signum, _frame):
    STOP["why"] = signal.Signals(signum).name


def cancel_all(node, log):
    """Cancel every goal of the controller and the planner. True if both answered."""
    ok = True
    for action in ("/follow_path", "/compute_path_to_pose"):
        cli = node.create_client(CancelGoal, f"{action}/_action/cancel_goal")
        if not cli.wait_for_service(timeout_sec=3.0):
            log(f"cancel: {action} not available")
            ok = False
            continue
        fut = cli.call_async(CancelGoal.Request())  # zero id and stamp: all goals
        end = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if fut.done():
            n = len(fut.result().goals_canceling)
            log(f"cancel: {action}: {n} goal(s) canceling")
        else:
            log(f"cancel: {action}: no answer in 3 s")
            ok = False
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--distance", type=float, default=2.0, help="m ahead (max 3)")
    ap.add_argument("--timeout", type=float, default=60.0, help="s before cancelling")
    ap.add_argument("--tolerance", type=float, default=0.30, help="m from the goal counts as done")
    ap.add_argument("--map-frame", default="lidar3d_map")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--cancel", action="store_true", help="only cancel running goals")
    args = ap.parse_args()
    if not 0.2 <= args.distance <= 3.0:
        sys.exit("--distance must be 0.2 to 3.0 m")

    # Python's handlers, not rclpy's, so a signal only sets STOP and the cancel still runs.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    node = Node("goal_ahead")

    def log(msg):
        print(f"[goal_ahead {time.strftime('%H:%M:%S')}] {msg}", flush=True)

    if args.cancel:
        ok = cancel_all(node, log)
        node.destroy_node()
        rclpy.shutdown()
        return 0 if ok else 1

    tf = Buffer()
    TransformListener(tf, node)
    gait = {"last": None}

    def on_gait(msg):
        try:
            g = json.loads(msg.data)
        except ValueError:
            return
        key = (g.get("gait"), g.get("backend_state"))
        if key != gait["last"]:
            gait["last"] = key
            log(f"gait {g.get('gait')} (costmap {g.get('costmap_gait')}, clusters {g.get('cluster_gait')}, "
                f"cost {g.get('cost')}, backend {g.get('backend_state')})")

    node.create_subscription(String, "/gait/status", on_gait, 10)
    pub = node.create_publisher(PoseStamped, "/goal_pose", 10)

    end = time.monotonic() + 10.0
    while not tf.can_transform(args.map_frame, args.base_frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.1)
        if STOP["why"] or time.monotonic() > end:
            log(f"no TF {args.map_frame} -> {args.base_frame}: is the session up? Nothing sent.")
            return 1
    t = tf.lookup_transform(args.map_frame, args.base_frame, Time()).transform
    q = t.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    x0, y0 = t.translation.x, t.translation.y
    goal = PoseStamped()
    goal.header.frame_id = args.map_frame
    goal.pose.position.x = x0 + args.distance * math.cos(yaw)
    goal.pose.position.y = y0 + args.distance * math.sin(yaw)
    goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)

    end = time.monotonic() + 5.0
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if STOP["why"] or time.monotonic() > end:
            log("nobody subscribes to /goal_pose (go2_goal_pose_nav down?). Nothing sent.")
            return 1
    goal.header.stamp = node.get_clock().now().to_msg()
    pub.publish(goal)
    log(f"goal sent: from ({x0:.2f}, {y0:.2f}) {args.distance:.1f} m ahead to "
        f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}), heading {math.degrees(yaw):.0f} deg")

    rc, start = 0, time.monotonic()
    while True:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            p = tf.lookup_transform(args.map_frame, args.base_frame, Time()).transform.translation
            left = math.hypot(goal.pose.position.x - p.x, goal.pose.position.y - p.y)
        except Exception:
            left = float("nan")
        if left <= args.tolerance:
            log(f"arrived ({left:.2f} m from the goal)")
            break
        if STOP["why"] or time.monotonic() - start > args.timeout:
            why = STOP["why"] or f"timeout {args.timeout:.0f} s"
            log(f"stopping ({why}, {left:.2f} m left): cancelling the goal")
            rc = 0 if cancel_all(node, log) else 1
            break
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
