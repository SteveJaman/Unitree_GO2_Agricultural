#!/usr/bin/env python3
"""moveit_d1_bridge.py against a simulated D1: no arm, no network, nothing moves.

  source /opt/ros/humble/setup.bash
  ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 python3 -m pytest tests/test_moveit_bridge.py

python3 tests/test_moveit_bridge.py --serve runs the bridge on the simulated arm
until Ctrl+C, in place of the real one, for a whole-stack check with
demo.launch.py hardware:=real, or for arm_moveit.sh with ARM_BRIDGE_CMD.
--signs "..." sets the joint signs, --start fold rests the arm on its fold, and
ARM_START_STOW=1 is read as by the real bridge. --glitch S sends one bad
telemetry frame every S seconds.
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
        self.delay = 0.0  # s behind the commands
        self.history = []
        self.playback = None
        self.next_telemetry = 0.0
        self.bad_frames, self.bad_at = 0, 0.0  # that many frames, from then, read wrong
        self.glitch_every = 0.0  # s between single bad frames
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
        if self.blocked:
            return
        now = time.monotonic()
        self.history.append((now, list(self.command)))
        while len(self.history) > 1 and self.history[1][0] <= now - self.delay:
            self.history.pop(0)
        self.pos = list(self.history[0][1])

    def _telemetry(self):
        self._step()
        data = {f"angle{j}": a for j, a in enumerate(self.pos)}
        return {"address": bridge.ADDR_TELEM, "funcode": 1, "data": {**data, "angle6": self.gripper}}

    def _take(self):
        now = time.monotonic()
        if self.silent or now < self.next_telemetry:
            return []
        self.next_telemetry = now + 0.05
        frame = self._telemetry()
        if self.glitch_every and now >= self.bad_at and not self.bad_frames:
            self.bad_frames = 1
        if self.bad_frames and now >= self.bad_at:
            self.bad_frames -= 1
            frame["data"]["angle1"] -= 76.6
            self.bad_at = now + self.glitch_every
        return [json.dumps(frame)]

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
    goal = FakeGoal(trajectory([0] * 6, [45] * 6, 3.0))
    result = node.run_goal(goal)
    assert goal.state == "aborted" and result.error_code == Result.PATH_TOLERANCE_VIOLATED
    assert "behind" in result.error_string
    node.destroy_node()


def test_one_bad_frame_at_the_end_of_a_move_does_not_stop_it():
    # The real failure: 2.6 s, 18 deg, one frame 76.6 deg off right at the goal.
    node, arm = make(robot=True)
    published = []
    node.js_pub.publish = lambda msg: published.append(list(msg.position))
    arm.bad_frames, arm.bad_at = 1, time.monotonic() + 2.3
    goal = FakeGoal(trajectory([0] * 6, [18] * 6, 2.6))
    node.run_goal(goal)
    assert goal.state == "succeeded"
    assert arm.bad_frames == 0  # it was really sent
    assert max(abs(math.degrees(p[1])) for p in published) < 25  # and never reached MoveIt
    node.destroy_node()


def test_bad_frame_before_a_goal_does_not_refuse_it():
    node, arm = make(angles=(10, 10, 10, 10, 10, 10))
    time.sleep(0.06)
    node.poll_angles()
    arm.bad_frames, arm.bad_at = 1, 0.0
    time.sleep(0.06)
    goal = FakeGoal(trajectory([10] * 6, [11] * 6, 0.5))
    node.run_goal(goal)
    assert goal.state == "succeeded"
    node.destroy_node()


def test_a_jump_that_stays_is_believed():
    # Someone pushed the arm: three frames in a row, and the path check gets it.
    node, arm = make()
    time.sleep(0.06)
    node.poll_angles()
    arm.pos = [60.0] * 6
    arm.command = list(arm.pos)
    arm.history.clear()
    for frames in (1, 2, 3):
        time.sleep(0.06)
        node.poll_angles()
        assert (node.measured[0] == 60.0) == (frames == bridge.SPIKE_CONFIRM)
    node.destroy_node()


def test_a_real_block_is_reported_with_its_joint():
    node, arm = make()
    arm.blocked = True
    goal = FakeGoal(trajectory([0] * 6, [0, 45, 0, 0, 0, 0], 3.0))
    result = node.run_goal(goal)
    assert goal.state == "aborted"
    assert "Joint2" in result.error_string and "measured 0.0" in result.error_string
    node.destroy_node()


def test_offsets_shift_the_firmware_and_come_back():
    # The real wrist roll reads 90 deg off the model (2026-11-11): firmware = sign x (MoveIt + offset).
    arm = FakeArm(angles=(0, 0, 0, 0, 0, 90))
    node = bridge.MoveItD1Bridge([1, 1, 1, 1, 1, -1], arm=arm, offsets=[0, 0, 0, 0, 0, -90])
    time.sleep(0.06)
    node.poll_angles()
    published = []
    node.js_pub.publish = lambda msg: published.append(list(msg.position))
    node.publish_joint_states()
    assert abs(math.degrees(published[-1][5])) < 0.5  # firmware 90 is MoveIt 0
    goal = FakeGoal(trajectory([0] * 6, [0, 0, 0, 0, 0, 20], 2.0))
    node.run_goal(goal)
    assert goal.state == "succeeded"
    assert abs(arm.pos[5] - 70.0) < 1.0  # -1 x (20 - 90)
    node.destroy_node()


def test_start_stow_with_a_wrist_offset():
    # Stow is 90 deg of wrist roll away from the fold then: roll joints don't count as too far.
    arm = FakeArm(angles=FOLD)
    node = bridge.MoveItD1Bridge([1, 1, 1, 1, 1, -1], arm=arm, start_stow=True, offsets=[0, 0, 0, 0, 0, -90])
    stow = (np.array(bridge.STOW_RAD) * bridge.RAD2DEG + node.offsets) * node.signs
    assert np.abs(np.array(arm.pos) - stow).max() < 3.0
    node.destroy_node()


def test_offsets_must_be_six_angles():
    assert bridge.parse_offsets("") == [0.0] * 6
    assert bridge.parse_offsets("0 0 0 0 0 -90")[5] == -90.0
    for bad in ("0 0 0", "a b c d e f", "0 0 0 0 0 200"):
        with pytest.raises(ValueError):
            bridge.parse_offsets(bad)


def test_arm_a_second_late_is_not_blocked():
    # Seen on the real arm over Wi-Fi at wrist speed.
    node, arm = make()
    arm.delay = 0.8
    goal = FakeGoal(trajectory([0] * 6, [0, 0, 0, 40, 0, 0], 2.5))
    node.run_goal(goal)
    assert goal.state == "succeeded"
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


def test_start_stow_accepts_a_fold_with_the_wrist_rolled():
    # Read from the real arm after a sign test: joint 5 left at +30.
    node, arm = make(angles=(-5.5, -88.8, 91.8, 8.3, 19.9, 29.9), start_stow=True)
    assert np.allclose(arm.pos, STOW_DEG, atol=0.1)
    node.destroy_node()


def test_ctrl_c_during_the_stow_move_stops_the_robot():
    real_move = bridge.smooth_move

    def interrupted(*a, **k):
        raise KeyboardInterrupt

    bridge.smooth_move = interrupted
    arm = FakeArm(angles=FOLD, robot=True)
    try:
        with pytest.raises(KeyboardInterrupt):
            bridge.MoveItD1Bridge([1] * 6, arm=arm, start_stow=True)
    finally:
        bridge.smooth_move = real_move
    assert any("traj_stop" in m for m in arm.link.sent)


def test_start_stow_needs_the_shoulder_on_the_fold():
    angles = list(FOLD)
    angles[1] += 12
    node, arm = make(angles=angles, start_stow=True)
    assert arm.setpoints == []
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
    ap.add_argument("--signs", default=os.environ.get("ARM_JOINT_SIGNS", "1 1 1 1 1 1"))
    ap.add_argument("--start", choices=["stow", "fold"], default="stow")
    ap.add_argument("--glitch", type=float, default=0.0)
    args = ap.parse_args()
    rclpy.init()
    # Start at the sim's stow pose, in firmware degrees.
    signs = bridge.parse_signs(args.signs)
    stow = [math.degrees(a) * s for a, s in zip((0, -1.396, 0.785, 0, 1.396, 0), signs)]
    start = FOLD if args.start == "fold" else stow
    arm = FakeArm(angles=start, enabled=False)
    arm.glitch_every = args.glitch
    node = bridge.MoveItD1Bridge(
        signs,
        arm=arm,
        start_stow=os.environ.get("ARM_START_STOW") == "1",
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    serve()
