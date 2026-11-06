#!/usr/bin/env python3
"""Pure-logic tests for object_box_fusion and the gait height rule: no ROS needed for fusion."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "nodes"))
from object_box_fusion import (  # noqa: E402
    Tracks, classify_height, fuse_detection, ground_point, oriented_box, project)

K = (468.0, 468.0, 320.0, 180.0)


def scene(camera_height=0.33):
    """A 0.10 m tall, 0.6 m wide object 2 m ahead, on a floor of points, with a wall behind."""
    rng = np.random.default_rng(1)
    obj = np.column_stack([rng.uniform(1.95, 2.05, 80), rng.uniform(-0.3, 0.3, 80), rng.uniform(0.03, 0.10, 80)])
    floor = np.column_stack([rng.uniform(1.0, 3.0, 300), rng.uniform(-1, 1, 300), rng.normal(0, 0.002, 300)])
    wall = np.column_stack([np.full(100, 5.0), rng.uniform(-2, 2, 100), rng.uniform(0, 1.0, 100)])
    pts = np.vstack([obj, floor, wall])
    cam = pts - np.array([0.0, 0.0, camera_height])
    return pts, cam, obj


def bbox_of(obj, height=0.33):
    u, v = project(obj - np.array([0, 0, height]), *K)
    return [u.min(), v.min(), u.max(), v.max()]


def test_height_and_position_from_lidar_points():
    pts, cam, obj = scene()
    box = fuse_detection(pts, cam, K, bbox_of(obj), 0.0)
    assert abs(box["height"] - 0.10) < 0.01
    assert abs(box["x"] - 2.0) < 0.05 and abs(box["y"]) < 0.05
    assert abs(max(box["sx"], box["sy"]) - 0.6) < 0.1


def test_floor_and_background_are_excluded():
    pts, cam, obj = scene()
    box = fuse_detection(pts, cam, K, bbox_of(obj), 0.0)
    assert box["n_points"] <= 80


def test_too_few_points_is_unknown_not_safe():
    pts, cam, obj = scene()
    box = fuse_detection(pts[:2], cam[:2], K, bbox_of(obj), 0.0)
    assert box["traverse"] == "unknown"
    assert "height" not in box


def test_classification_bands():
    assert classify_height(0.05) == "step_over"
    assert classify_height(0.20) == "caution"
    assert classify_height(0.25) == "blocked"


def test_oriented_box_follows_a_diagonal_cable():
    t = np.linspace(-1, 1, 50)
    xy = np.column_stack([t * math.cos(0.5), t * math.sin(0.5)])
    _, _, sx, sy, yaw = oriented_box(xy)
    assert abs(sx - 2.0) < 0.05 and sy < 0.02
    assert abs(math.sin(yaw - 0.5)) < 0.02


def test_tracks_merge_one_object_and_keep_two_apart():
    t = Tracks()
    b = {"x": 1.0, "y": 0.0, "z": 0.05, "sx": 0.4, "sy": 0.1, "sz": 0.1, "yaw": 0.0, "height": 0.1, "n_points": 9}
    t.update("hose", 0.8, b, 0.0)
    t.update("hose", 0.8, dict(b, x=1.2), 1.0)
    t.update("hose", 0.8, dict(b, x=4.0), 2.0)
    assert len(t.items) == 2
    assert t.items[0]["n_obs"] == 2
    assert len(t.publishable()) == 1    # a one-off track is held back


def test_tracks_keep_the_high_reading_from_a_partial_view():
    t = Tracks()
    b = {"x": 1.0, "y": 0.0, "z": 0.05, "sx": 0.4, "sy": 0.1, "sz": 0.1, "yaw": 0.0, "height": 0.10, "n_points": 9}
    for h in (0.10, 0.11, 0.42, 0.41, 0.42):
        t.update("barrel", 0.8, dict(b, height=h), 0.0)
    assert t.items[0]["traverse"] == "blocked"


def test_a_small_object_in_front_never_reads_as_the_detected_object():
    """The cone-in-front-of-the-barrel case: only the cone has points in the box."""
    rng = np.random.default_rng(2)
    cone = np.column_stack([rng.uniform(1.9, 2.0, 30), rng.uniform(-0.04, 0.04, 30), rng.uniform(0.04, 0.09, 30)])
    cam = cone - np.array([0.0, 0.0, 0.33])
    u, v = project(np.array([[3.0, -0.3, -0.33], [3.0, 0.3, 0.07]]), *K)
    box = fuse_detection(cone, cam, K, [u.min(), v.min(), u.max(), v.max()], 0.0)
    assert box["traverse"] == "unknown" and "height" not in box


def test_only_the_bottom_of_an_object_gives_unknown_height():
    rng = np.random.default_rng(3)
    low = np.column_stack([rng.uniform(1.95, 2.05, 40), rng.uniform(-0.3, 0.3, 40), rng.uniform(0.03, 0.08, 40)])
    cam = low - np.array([0.0, 0.0, 0.33])
    u, v = project(cam, *K)
    top = 180.0 - 468.0 * (0.50 - 0.33) / 2.0
    assert fuse_detection(low, cam, K, [u.min(), top, u.max(), v.max()], 0.0)["reason"] == "top_not_seen"


def test_points_at_the_wrong_range_never_measure_the_object():
    """A far object's box catches a near low object: the floor ray says it is further."""
    pts, cam, obj = scene()
    box = fuse_detection(pts, cam, K, bbox_of(obj), 0.0, floor_range=4.0)
    assert box["reason"] == "wrong_range"
    assert "height" in fuse_detection(pts, cam, K, bbox_of(obj), 0.0, floor_range=2.1)


def test_duplicates_of_a_long_object_merge():
    t = Tracks()
    b = {"x": 1.0, "y": 0.0, "z": 0.05, "sx": 2.0, "sy": 0.1, "sz": 0.1, "yaw": 0.0, "height": 0.1, "n_points": 9}
    t.update("cable", 0.8, b, 0.0)
    t.update("cable", 0.8, dict(b, x=2.2, sx=0.4), 1.0)
    assert len(t.items) == 1


def test_ground_point_of_a_box_bottom():
    rot, trans = np.eye(3), np.array([0.0, 0.0, -0.33])
    p = ground_point([300, 100, 340, 180 + 468 * 0.33 / 2.0], K, rot, trans, 0.0)
    assert abs(p[0] - 2.0) < 0.01 and abs(p[2]) < 1e-6


def gait_rule():
    """gait_supervisor imports rclpy at module level, so load only the rule's source."""
    src = open(os.path.join(os.path.dirname(__file__), "..", "scripts", "nodes", "gait_supervisor.py")).read()
    start = src.index("def gait_for_height")
    ns = {"math": math}
    exec(src[start:src.index("class GaitSupervisor")], ns)
    return ns["gait_for_height"]


def test_high_step_only_when_the_foot_clears_the_object():
    rule = gait_rule()
    assert rule(None, 0.10, 0.02, 0.25) == "CAREFUL"
    assert rule(0.05, 0.10, 0.02, 0.25) == "HIGH_STEP"
    assert rule(0.08, 0.10, 0.02, 0.25) == "HIGH_STEP"
    assert rule(0.12, 0.10, 0.02, 0.25) == "CAREFUL"
    assert rule(0.30, 0.10, 0.02, 0.25) == "STOP"


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
