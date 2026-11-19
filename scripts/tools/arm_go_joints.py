#!/usr/bin/env python3
"""Move the D1 arm to a joint pose through MoveIt (collision-checked), then exit.

  ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1 python3 scripts/tools/arm_go_joints.py 0 0.25 0.30 0 -0.55 0
  ... --velocity 0.2 --timeout 60

Six joint values in radians (Joint1..Joint6), as MoveIt sees them (the model's signs;
moveit_d1_bridge.py applies ARM_JOINT_SIGNS). Plans and executes with MoveGroup
(/move_action), so the move avoids the dog and the arm itself. Exit 0 when the arm got
there, 1 otherwise. The arm then holds the pose (the bridge keeps it enabled).
"""

import argparse
import sys
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions
from rclpy.action import ActionClient

JOINTS = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
LIMITS = {"Joint1": 2.36, "Joint2": 1.57, "Joint3": 1.57, "Joint4": 2.36, "Joint5": 1.57, "Joint6": 2.36}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("q", type=float, nargs=6, help="Joint1..Joint6 in rad")
    ap.add_argument("--group", default="arm")
    ap.add_argument("--velocity", type=float, default=0.2, help="velocity and acceleration scaling (0-1)")
    ap.add_argument("--timeout", type=float, default=60.0, help="s for planning plus the move")
    args = ap.parse_args()
    for j, v in zip(JOINTS, args.q):
        if abs(v) > LIMITS[j] - 0.02:
            sys.exit(f"{j} = {v:+.3f} rad is at or past its limit (+/-{LIMITS[j]}): pick a pose inside it")
    if not 0.01 <= args.velocity <= 1.0:
        sys.exit("--velocity must be 0.01-1")

    rclpy.init()
    node = rclpy.create_node("arm_go_joints")
    client = ActionClient(node, MoveGroup, "/move_action")

    def log(msg):
        print(f"[arm_go_joints] {msg}", flush=True)

    if not client.wait_for_server(timeout_sec=30.0):
        log("MoveIt's /move_action is not up (is the MoveIt launch running on this ROS_DOMAIN_ID?)")
        return 1
    req = MotionPlanRequest()
    req.group_name = args.group
    req.num_planning_attempts = 5
    req.allowed_planning_time = 10.0
    req.max_velocity_scaling_factor = args.velocity
    req.max_acceleration_scaling_factor = args.velocity
    req.start_state.is_diff = True
    req.goal_constraints = [
        Constraints(
            joint_constraints=[
                JointConstraint(joint_name=j, position=v, tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
                for j, v in zip(JOINTS, args.q)
            ]
        )
    ]
    goal = MoveGroup.Goal(request=req, planning_options=PlanningOptions(plan_only=False, replan=True, replan_attempts=2))
    log("moving to " + " ".join(f"{v:+.2f}" for v in args.q) + f" rad at {args.velocity:.0%} speed")
    end = time.monotonic() + args.timeout
    sent = client.send_goal_async(goal)
    while not sent.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    handle = sent.result() if sent.done() else None
    if handle is None or not handle.accepted:
        log("MoveIt did not accept the goal")
        return 1
    result = handle.get_result_async()
    while not result.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not result.done():
        log(f"no result in {args.timeout:.0f} s: cancelling")
        handle.cancel_goal_async()
        return 1
    code = result.result().result.error_code.val
    if code != 1:
        log(f"failed: MoveIt error code {code}")
        return 1
    log("there: the arm holds this pose")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
