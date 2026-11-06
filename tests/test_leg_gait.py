#!/usr/bin/env python3
"""Pure-logic tests for the kinematic gait: no ROS or Gazebo."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gazebo", "sim"))
from leg_gait import HIP_OFFSET, LEGS, STAND_HEIGHT, THIGH_Y, Trot, leg_fk, leg_ik  # noqa: E402


def close(a, b, tol=1e-3):
    assert abs(a - b) < tol, (a, b)


def test_ik_stand_is_the_builders_standing_pose():
    for leg in LEGS:
        side = THIGH_Y if leg.endswith("L") else -THIGH_Y
        hip, thigh, calf = leg_ik(leg, 0.0, side, -STAND_HEIGHT)
        close(hip, 0.0)
        close(thigh, 0.9)
        close(calf, -1.8)


def test_fk_inverts_ik_over_the_workspace():
    for leg in LEGS:
        side = THIGH_Y if leg.endswith("L") else -THIGH_Y
        for x in (-0.1, 0.0, 0.12):
            for dy in (-0.03, 0.03):
                for z in (-0.28, -0.22, -0.16):
                    fx, fy, fz = leg_fk(leg, *leg_ik(leg, x, side + dy, z))
                    close(fx, x)
                    close(fy, side + dy)
                    close(fz, z)


def run(gait, seconds, vx, wz, lift):
    log = []
    for _ in range(int(seconds / 0.02)):
        _, feet = gait.step(0.02, vx, 0.0, wz, lift)
        log.append(feet)
    return log


def test_swing_reaches_the_commanded_lift():
    log = run(Trot(), 3.0, 0.3, 0.0, 0.12)
    for leg in LEGS:
        close(max(f[leg][2] + STAND_HEIGHT for f in log), 0.12, 0.005)


def test_stance_foot_stays_planted_in_the_world():
    g = Trot()
    run(g, 1.0, 0.3, 0.0, 0.1)
    dt, x, track = 0.02, 0.0, []
    for _ in range(int(g.period / dt) * 2):
        _, feet = g.step(dt, 0.3, 0.0, 0.0, 0.1)
        x += 0.3 * dt
        if abs(feet["FL"][2] + STAND_HEIGHT) < 1e-6:
            track.append(x + feet["FL"][0])
    assert track
    groups, cur = [], [track[0]]
    for a, b in zip(track, track[1:]):
        if abs(b - a) > 0.02:
            groups.append(cur)
            cur = []
        cur.append(b)
    groups.append(cur)
    assert max(max(c) - min(c) for c in groups) < 0.02


def test_diagonal_pairs_swing_together():
    log = run(Trot(), 3.0, 0.3, 0.0, 0.1)
    for f in log[-100:]:
        close(f["FL"][2], f["RR"][2])
        close(f["FR"][2], f["RL"][2])
        assert min(f["FL"][2], f["FR"][2]) + STAND_HEIGHT < 1e-6


def test_standing_robot_settles_with_feet_down():
    g = Trot()
    run(g, 2.0, 0.3, 0.0, 0.1)
    for feet in run(g, 1.0, 0.0, 0.0, 0.1)[-5:]:
        for leg in LEGS:
            close(feet[leg][2], -STAND_HEIGHT)


def test_a_foot_never_lands_inside_a_marked_object():
    g = Trot(period=1.0)
    bx = [0.0]

    def avoid(leg, x, y):
        wx = bx[0] + x
        return 0.60 < wx < 0.72        # a 12 cm deep object across the path

    for _ in range(int(6.0 / 0.02)):
        _, feet = g.step(0.02, 0.3, 0.0, 0.0, 0.12, avoid)
        bx[0] += 0.3 * 0.02
        for leg in LEGS:
            x, _, z = feet[leg]
            if abs(z + STAND_HEIGHT) < 1e-6:
                assert not 0.60 < bx[0] + x < 0.72, (leg, bx[0], x)
        if bx[0] > 1.2:
            break


def test_swing_rises_before_it_travels_and_lands_after():
    g = Trot(period=1.0)
    run(g, 2.0, 0.3, 0.0, 0.12)
    track = []
    for _ in range(100):
        _, feet = g.step(0.02, 0.3, 0.0, 0.0, 0.12)
        track.append(feet["FL"])
    hip = HIP_OFFSET["FL"][0]
    swing = [(f[0] - hip, f[2] + STAND_HEIGHT) for f in track if f[2] + STAND_HEIGHT > 1e-6]
    start, end = min(p[0] for p in swing), max(p[0] for p in swing)
    middle = [z for x, z in swing if start + 0.02 < x < end - 0.02]
    assert middle and min(middle) > 0.11      # at full height while it crosses


def test_turning_in_place_moves_feet_in_opposite_directions():
    g = Trot()
    a = run(g, 1.0, 0.0, 0.7, 0.05)
    side = {l: max(abs(f[l][1] - (HIP_OFFSET[l][1] + (THIGH_Y if l.endswith("L") else -THIGH_Y))) for f in a) for l in LEGS}
    assert min(side.values()) > 0.01


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
