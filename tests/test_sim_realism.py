"""SIM_REALISM=1 body motion of the simulated Go2 (gazebo/sim/sim_robot_bridge.py)."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gazebo", "sim"))
import sim_robot_bridge as b  # noqa: E402


def run(motion, cmd, seconds, dt=0.02):
    out = []
    for _ in range(int(seconds / dt)):
        out.append(motion.step(dt, cmd))
    return out


def test_velocity_lags_and_is_acceleration_limited():
    m = b.BodyMotion()
    out = run(m, (0.5, 0.0, 0.0), 0.2)
    vx = [v[0] for v, _ in out]
    assert vx[0] < 0.05                         # no step change
    assert all(b2 >= a for a, b2 in zip(vx, vx[1:]))
    assert max(b2 - a for a, b2 in zip(vx, vx[1:])) <= 1.2 * 0.02 + 1e-9
    out = run(m, (0.5, 0.0, 0.0), 2.0)
    assert out[-1][0][0] == pytest.approx(0.5, abs=0.01)


def test_bob_while_walking_and_breathing_when_standing():
    m = b.BodyMotion()
    walk = run(m, (0.35, 0.0, 0.0), 4.0)[-60:]  # last 1.2 s: two strides at full speed
    dz = [o[0] for _, o in walk]
    roll = [o[2] for _, o in walk]
    assert max(dz) - min(dz) == pytest.approx(2 * 0.008, rel=0.15)
    assert max(roll) - min(roll) == pytest.approx(2 * math.radians(1.2), rel=0.15)
    stand = run(m, (0.0, 0.0, 0.0), 8.0)[-200:]
    dz = [o[0] for _, o in stand]
    assert 0.001 < max(dz) - min(dz) < 0.004    # breathing: ~1.5 mm, not still, not walking


def test_acceleration_pitches_the_nose_up():
    m = b.BodyMotion()
    _, off = run(m, (0.5, 0.0, 0.0), 0.3)[-1]
    assert off[3] < -math.radians(0.3)          # pitch negative = nose up


def test_realism_cmd_holds_height_and_level():
    level = ((0.0, 0.0, 0.30), (0.0, 0.0, 0.0, 1.0))
    lin, ang = b.realism_cmd(level, 0.2, 0.0, 0.0, 4.0, 0.30, (0, 0, 0, 0, 0, 0))
    assert lin[0] == pytest.approx(0.2) and abs(lin[2]) < 1e-9 and max(map(abs, ang)) < 1e-9
    low = ((0.0, 0.0, 0.28), (0.0, 0.0, 0.0, 1.0))
    lin, _ = b.realism_cmd(low, 0.0, 0.0, 0.0, 4.0, 0.30, (0, 0, 0, 0, 0, 0))
    assert lin[2] == pytest.approx(4.0 * 0.02)  # climbs back to the nominal height
    _, ang = b.realism_cmd(level, 0.0, 0.0, 0.0, 4.0, 0.30, (0, 0, math.radians(2), 0, 0, 0))
    assert ang[0] > 0                           # rolls towards the target roll


def test_wrist_stamps_pair_and_never_go_back():
    st = {"offset": None, "frames": {}}
    s = 1_000_000_000
    a = b.wrist_wall_stamp(st, 10 * s, 100 * s)
    assert b.wrist_wall_stamp(st, 10 * s, 100 * s + 50_000_000) == a      # depth of the same frame
    c = b.wrist_wall_stamp(st, 10 * s + 100_000_000, 100 * s + 150_000_000)
    assert c == a + 100_000_000                                            # fixed offset
    # sim at half speed: the wall clock runs away; re-anchoring moves forward only
    t = 100 * s
    last = c
    for k in range(2, 80):
        t += 200_000_000
        w = b.wrist_wall_stamp(st, 10 * s + k * 100_000_000, t)
        assert w > last
        last = w
    assert t - last < 2_100_000_000                                        # stays within max_lag of now
