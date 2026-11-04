#!/usr/bin/env python3
"""moveit_d1_bridge.py against a simulated D1: no arm, no network, nothing moves.

  source /opt/ros/humble/setup.bash
  ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 python3 -m pytest tests/test_moveit_bridge.py

python3 tests/test_moveit_bridge.py --serve runs the bridge on the simulated arm
until Ctrl+C, in place of the real one, for a whole-stack check with
demo.launch.py hardware:=real. --signs "..." sets the joint signs.
"""

import argparse
import json
import math
import os
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "nodes"))
import moveit_d1_bridge as bridge  # noqa: E402

JOINTS = bridge.MOVEIT_JOINTS
Result = bridge.FollowJointTrajectory.Result


class FakeLink:
    """The Go2 bridge: plays traj_points by itself, as arm_bridge_wifi.py does."""

    hello = '{"hello": ["traj", "traj_points"]}'

    def __init__(self, arm):
        self.arm, self.sent = arm, []

    def write(self, text):
        m = json.loads(text)
        self.sent.append(m)
        if "traj_points" in m:
            spec = m["traj_points"]
            self.arm.playback = (time.monotonic(), spec["dt"], spec["points"])
        elif "traj" in m:
            self.arm.command = list(m["traj"]["target"])
            self.arm.playback = None
        elif "traj_stop" in m:
            self.arm.playback = None


class FakeArm:
    """The D1 as this bridge sees it. Telemetry at 20 Hz, following its command
    instantly unless `blocked` holds it where it is."""

    ok = True

    def __init__(self, angles=(0, 0, 0, 0, 0, 0), robot=False, enabled=True):
        self.seq = 0
        self.pos = [float(a) for a in angles]
        self.command = list(self.pos)
        self.gripper = -29.5
        self.setpoints, self.calls = [], []
        self.enabled, self.powered = enabled, enabled
        self.blocked = self.silent = False
        self.playback = None
        self.next_telemetry = 0.0
        self.link = FakeLink(self) if robot else None

    def _write(self, text):
        m = json.loads(text)
        if m["funcode"] == 2:
            data = m["data"]
            self.setpoints.append(data)
            self.command = [data[f"angle{j}"] for j in range(6)]

    def _send(self, funcode, data=None, repeat=3):
        self.seq += 1
        self._write(json.dumps({"funcode": funcode, "data": data}))

    def _step(self):
        if self.playback:
            t0, dt, points = self.playback
            i = int((time.monotonic() - t0) / dt)
            self.command = points[min(i, len(points) - 1)]
            if i >= len(points):
                self.playback = None
        if not self.blocked:
            self.pos = list(self.command)

    def _telemetry(self):
        self._step()
        data = {f"angle{j}": a for j, a in enumerate(self.pos)}
        return {"address": bridge.ADDR_TELEM, "funcode": 1, "data": {**data, "angle6": self.gripper}}

    def _take(self):
        now = time.monotonic()
        if self.silent or now < self.next_telemetry:
            return []
        self.next_telemetry = now + 0.05
        return [json.dumps(self._telemetry())]

    def angles(self, timeout=2.0):
        return None if self.silent else self._telemetry()

    def state(self, timeout=2.0):
        return {"data": {"power_status": int(self.powered), "enable_status": int(self.enabled)}}

    def power(self, on):
        self.calls.append("power")
        self.powered = bool(on)

    def enable(self, mode=80000):
        self.calls.append("enable")
        self.enabled = True


class FakeGoal:
    def __init__(self, trajectory):
        self.request = SimpleNamespace(trajectory=trajectory)
        self.is_cancel_requested = False
        self.state = None

    def succeed(self):
        self.state = "succeeded"

    def abort(self):
        self.state = "aborted"

    def canceled(self):
        self.state = "canceled"


def trajectory(start_deg, end_deg, seconds, names=JOINTS, steps=20):
    """A smooth (quintic) move, like a time-parameterised MoveIt trajectory, in radians."""
    start, end = np.radians(start_deg), np.radians(end_deg)
    msg = JointTrajectory(joint_names=list(names))
    for i in range(steps + 1):
        s = i / steps
        k = 10 * s**3 - 15 * s**4 + 6 * s**5
        dk = (30 * s**2 - 60 * s**3 + 30 * s**4) / seconds
        t = s * seconds
        msg.points.append(
            JointTrajectoryPoint(
                positions=list(start + (end - start) * k),
                velocities=list((end - start) * dk),
                time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9)),
            )
        )
    return msg


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def make(signs=(1, 1, 1, 1, 1, 1), start_stow=False, **arm):
    a = FakeArm(**arm)
    return bridge.MoveItD1Bridge(list(signs), arm=a, start_stow=start_stow), a


def test_signs_must_be_six_plus_or_minus_one():
    assert bridge.parse_signs("1 -1 1 1 1 -1") == [1, -1, 1, 1, 1, -1]
    for bad in ("", "1 1 1", "1 1 1 1 1 2", None):
        with pytest.raises(ValueError):
            bridge.parse_signs(bad)


def test_resample_keeps_moveit_timing_and_order():
    t = trajectory([0] * 6, [10, 20, 30, 0, 0, 0], 2.0, names=list(reversed(JOINTS)))
    points = bridge.resample(t, 50.0)
    assert 100 <= len(points) <= 102
    assert np.allclose(np.degrees(points[-1]), [0, 0, 0, 30, 20, 10])
    assert np.degrees(np.abs(np.diff(points, axis=0))).max() < 1.0  # tiny steps, no jumps


def test_resample_refuses_a_missing_joint():
    with pytest.raises(ValueError):
        bridge.resample(trajectory([0] * 5, [1] * 5, 1.0, names=JOINTS[:5]), 50.0)


def test_enabled_arm_is_never_re_enabled():
    node, arm = make(enabled=True)
    assert arm.calls == []
    node.destroy_node()


def test_released_arm_is_powered_and_enabled_once():
    node, arm = make(enabled=False)
    assert arm.calls == ["power", "enable"]
    node.destroy_node()


def test_wired_goal_streams_at_moveit_timing():
    node, arm = make()
    goal = FakeGoal(trajectory([0] * 6, [12, -10, 8, 0, 5, 0], 2.0))
    t0 = time.monotonic()
    result = node.run_goal(goal)
    took = time.monotonic() - t0
    assert goal.state == "succeeded" and result.error_code == Result.SUCCESSFUL
    played = [s for s in arm.setpoints if s["mode"] == 0]
    assert len(played) >= 95, "not streamed at 50 Hz"
    assert 2.0 <= took < 4.5, took
    assert all(s["angle6"] == -29.5 for s in arm.setpoints), "the gripper must stay where it was"
    assert np.allclose(arm.pos, [12, -10, 8, 0, 5, 0], atol=0.1)
    node.destroy_node()


def test_on_the_robot_the_whole_trajectory_goes_in_one_message():
    node, arm = make(robot=True)
    goal = FakeGoal(trajectory([0] * 6, [10] * 6, 2.0))
    node.run_goal(goal)
    sent = [m for m in arm.link.sent if "traj_points" in m]
    assert goal.state == "succeeded"
    assert len(sent) == 1 and sent[0]["traj_points"]["mode"] == 0
    assert len(sent[0]["traj_points"]["points"]) >= 100
    assert not [s for s in arm.setpoints if s["mode"] == 0], "Wi-Fi streaming makes the arm shake"
    node.destroy_node()


def test_signs_reach_the_firmware_and_come_back():
    node, arm = make(signs=(-1, 1, 1, 1, 1, -1))
    node.run_goal(FakeGoal(trajectory([0] * 6, [10, 10, 0, 0, 0, 10], 1.0)))
    assert np.allclose(arm.pos, [-10, 10, 0, 0, 0, -10], atol=0.1)
    node.poll_angles()
    moveit = np.array(node.measured) * node.signs
    assert np.allclose(moveit[[0, 1, 5]], [10, 10, 10], atol=0.2)
    node.destroy_node()


def test_refuses_a_start_away_from_the_arm():
    node, arm = make(angles=(40, 0, 0, 0, 0, 0))
    time.sleep(0.06)
    goal = FakeGoal(trajectory([0] * 6, [5] * 6, 1.0))
    result = node.run_goal(goal)
    assert goal.state == "aborted" and result.error_code == Result.PATH_TOLERANCE_VIOLATED
    assert arm.setpoints == []
    node.destroy_node()


def test_refuses_too_fast_and_out_of_limits():
    node, arm = make()
    fast, far = FakeGoal(trajectory([0] * 6, [80] * 6, 1.0)), FakeGoal(trajectory([0] * 6, [0, 120, 0, 0, 0, 0], 8.0))
    assert node.run_goal(fast).error_code == Result.INVALID_GOAL
    assert node.run_goal(far).error_code == Result.INVALID_GOAL
    assert arm.setpoints == []
    node.destroy_node()


def test_cancel_stops_at_once_and_holds():
    node, arm = make(robot=True)
    goal = FakeGoal(trajectory([0] * 6, [20] * 6, 4.0))

    def cancel_soon(_name):
        goal.is_cancel_requested = len(arm.link.sent) > 0 and time.monotonic() - t0 > 0.5

    t0 = time.monotonic()
    real_sleep, bridge.time.sleep = bridge.time.sleep, lambda s: (cancel_soon(None), real_sleep(s))[1]
    try:
        node.run_goal(goal)
    finally:
        bridge.time.sleep = real_sleep
    assert goal.state == "canceled"
    assert time.monotonic() - t0 < 1.5
    assert any("traj_stop" in m for m in arm.link.sent), "the robot's playback must stop"
    node.destroy_node()


def test_blocked_arm_aborts():
    node, arm = make()
    arm.blocked = True
    goal = FakeGoal(trajectory([0] * 6, [25] * 6, 2.0))
    result = node.run_goal(goal)
    assert goal.state == "aborted" and result.error_code == Result.PATH_TOLERANCE_VIOLATED
    assert "behind" in result.error_string
    node.destroy_node()


def test_lost_telemetry_aborts():
    node, arm = make()
    time.sleep(0.06)
    node.poll_angles()
    goal = FakeGoal(trajectory([0] * 6, [20] * 6, 4.0))
    arm.silent = True
    result = node.run_goal(goal)
    assert goal.state == "aborted" and "telemetry" in result.error_string
    node.destroy_node()


def test_goal_short_of_the_target_is_a_failure_not_a_success():
    node, arm = make()
    goal = FakeGoal(trajectory([0] * 6, [10] * 6, 1.0))
    real_move = bridge.smooth_move
    bridge.smooth_move = lambda *a, **k: SimpleNamespace(reached=False, final_error=4.2)
    try:
        result = node.run_goal(goal)
    finally:
        bridge.smooth_move = real_move
    assert goal.state == "aborted" and result.error_code == Result.GOAL_TOLERANCE_VIOLATED
    node.destroy_node()


FOLD = bridge.FOLDED_ANGLES[:6]
STOW_DEG = [math.degrees(a) for a in bridge.STOW_RAD]


def test_start_stow_moves_a_folded_arm_to_stow():
    node, arm = make(angles=FOLD, start_stow=True)
    assert np.allclose(arm.pos, STOW_DEG, atol=0.1)
    assert all(s["angle6"] == -29.5 for s in arm.setpoints)
    node.destroy_node()


def test_without_start_stow_a_folded_arm_stays():
    node, arm = make(angles=FOLD)
    assert arm.setpoints == [] and arm.pos == [float(a) for a in FOLD]
    node.destroy_node()


def test_start_stow_leaves_any_other_pose_alone():
    node, arm = make(angles=(30, -20, 40, 0, 10, 0), start_stow=True)
    assert arm.setpoints == []
    node.destroy_node()


def test_start_stow_does_nothing_at_stow():
    node, arm = make(angles=STOW_DEG, start_stow=True)
    assert arm.setpoints == []
    node.destroy_node()


def test_start_stow_refuses_a_wrong_sign():
    with pytest.raises(RuntimeError, match="ARM_JOINT_SIGNS"):
        make(signs=(1, -1, 1, 1, 1, 1), angles=FOLD, start_stow=True)


def test_start_stow_fails_if_the_arm_does_not_get_there():
    real_move = bridge.smooth_move
    bridge.smooth_move = lambda *a, **k: SimpleNamespace(reached=False, final_error=9.0)
    try:
        with pytest.raises(RuntimeError, match="not reached"):
            make(angles=FOLD, start_stow=True)
    finally:
        bridge.smooth_move = real_move


def test_second_goal_is_refused_while_moving():
    node, _ = make()
    assert node.on_goal(None) == bridge.GoalResponse.ACCEPT
    assert node.on_goal(None) == bridge.GoalResponse.REJECT
    node.destroy_node()


def serve():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serve", action="store_true")
    ap.add_argument("--signs", default="1 1 1 1 1 1")
    args = ap.parse_args()
    rclpy.init()
    # Start at the sim's stow pose, in firmware degrees.
    signs = bridge.parse_signs(args.signs)
    stow = [math.degrees(a) * s for a, s in zip((0, -1.396, 0.785, 0, 1.396, 0), signs)]
    node = bridge.MoveItD1Bridge(signs, arm=FakeArm(angles=stow))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    serve()
