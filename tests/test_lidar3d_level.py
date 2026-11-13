#!/usr/bin/env python3
"""Tests for scripts/nodes/go2_lidar3d_level.py (no ROS): the body-IMU tilt correction.

A synthetic run: the dog walks level on a flat floor while Point-LIO's attitude estimate
is knocked over by 70 deg halfway (as on 2026-11-12, run-20261112-190610) and its
position climbs after that. The body IMU says level throughout.

  python3 -m pytest -q tests/test_lidar3d_level.py
"""
import math
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "nodes"))
import go2_lidar3d_level as L  # noqa: E402

SFB = (0.0, -0.991341, 0.0, 0.131316)   # go2_odom_tf's sensor_from_base rotation


def rot_x(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def rot_z(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def run(hold_height=True, jump_deg=70.0):
    """(times, corrected body tilt deg, corrected z) for the synthetic run."""
    r_sb = L.quat_to_matrix(*SFB)
    c = L.LevelCorrector(SFB, hold_height=hold_height)
    c.on_imu(1.0, 0.0, 0.0, 0.0)          # body IMU: level (yaw is ignored)
    out = []
    for i in range(0, 1200):
        t = i * 0.05                      # 60 s at 20 Hz
        yaw = 0.3 * t                     # turning slowly the whole time
        true_body = rot_z(yaw)
        err = rot_x(jump_deg) if t >= 30.0 else np.eye(3)   # Point-LIO knocked over at 30 s
        r_mb = err @ true_body            # Point-LIO's (wrong) body attitude
        q_ms = L.matrix_to_quat(r_mb @ r_sb.T)
        p = np.array([0.02 * t, 0.0, 0.30 + (0.02 * (t - 30.0) if t >= 30.0 else 0.0)])
        m = c.on_pose(t, q_ms, p)
        corrected = m @ r_mb
        tilt = math.degrees(math.acos(np.clip((corrected.T @ L.UP) @ L.UP, -1, 1)))
        out.append((t, tilt, c.apply(p)[2]))
    return np.array(out), c


def test_quaternion_round_trip():
    m = rot_z(40) @ rot_x(-25)
    assert np.allclose(L.quat_to_matrix(*L.matrix_to_quat(m)), m, atol=1e-9)


def test_rotvec_between():
    a, b = np.array([0.0, 0.3, 1.0]), L.UP
    m = L.rotvec_to_matrix(L.rotvec_between(a, b))
    assert np.allclose(m @ (a / np.linalg.norm(a)), b, atol=1e-9)
    assert np.allclose(L.rotvec_between(b, b), 0.0)


def test_passes_through_before_body_imu():
    c = L.LevelCorrector(SFB)
    m = c.on_pose(0.0, L.matrix_to_quat(rot_x(30)), (1.0, 2.0, 3.0))
    assert np.allclose(m, np.eye(3)) and np.allclose(c.apply((1.0, 2.0, 3.0)), (1.0, 2.0, 3.0))


def test_level_run_needs_no_correction():
    rows, c = run(jump_deg=0.0)
    assert rows[:, 1].max() < 0.5 and c.degrees < 0.5 and c.snapped == 0


def test_divergence_is_levelled():
    rows, c = run()
    after = rows[rows[:, 0] > 31.0]
    assert after[:, 1].max() < 1.0, after[:, 1].max()   # level again within a second
    assert c.snapped >= 1 and abs(c.degrees - 70.0) < 1.0


def test_hold_height_keeps_the_floor():
    rows, _ = run(hold_height=True)
    assert np.ptp(rows[60:, 2]) < 1e-6                     # the height never moves
    rows, _ = run(hold_height=False)
    assert rows[-1, 2] > 0.5                                # Point-LIO's climb shows through


def test_no_position_jump_at_the_correction():
    rows, _ = run(hold_height=False)
    i = int(np.argmax(rows[:, 0] >= 30.0))
    assert abs(rows[i, 2] - rows[i - 1, 2]) < 0.01


def test_rotate_cloud():
    data = struct.pack("<3f4x", 1.0, 0.0, 0.0) + struct.pack("<3f4x", 0.0, 1.0, 0.5)
    out = L.rotate_cloud(data, 16, (0, 4, 8), rot_z(90), (0.0, 0.0, 1.0))
    p0 = struct.unpack_from("<3f", out, 0)
    p1 = struct.unpack_from("<3f", out, 16)
    assert np.allclose(p0, (0.0, 1.0, 1.0), atol=1e-6)
    assert np.allclose(p1, (-1.0, 0.0, 1.5), atol=1e-6)
    assert out[12:16] == data[12:16]                        # other bytes untouched
