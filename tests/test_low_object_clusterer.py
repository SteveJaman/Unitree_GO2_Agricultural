"""Unit tests for scripts/nodes/low_object_clusterer.py (no ROS)."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "nodes"))
import low_object_clusterer as lc  # noqa: E402

RNG = np.random.default_rng(0)


def floor(z0=0.0, n=4000, half=3.0, noise=0.004):
    xy = RNG.uniform(-half, half, (n, 2))
    return np.column_stack([xy, z0 + RNG.normal(0, noise, n)])


def box(cx, cy, sx, sy, h, z0=0.0, step=0.02):
    """Points on the top and sides of a box, like LiDAR returns from an object."""
    xs, ys = np.arange(cx - sx / 2, cx + sx / 2 + 1e-9, step), np.arange(cy - sy / 2, cy + sy / 2 + 1e-9, step)
    top = np.array([[x, y, z0 + h] for x in xs for y in ys])
    zs = np.arange(z0 + 0.01, z0 + h, 0.01)
    side = np.array([[cx - sx / 2, y, z] for y in ys for z in zs]) if len(zs) else np.empty((0, 3))
    return np.vstack([top, side]) if len(side) else top


def test_floor_from_histogram():
    pts = np.vstack([floor(z0=0.031), box(1.0, 0.0, 0.3, 0.3, 0.06, z0=0.031)])
    assert abs(lc.estimate_floor(pts, guess=0.0) - 0.031) < 0.004


def test_floor_falls_back_to_guess_without_points():
    assert lc.estimate_floor(np.empty((0, 3)), guess=0.05) == 0.05
    assert lc.estimate_floor(np.array([[0, 0, 1.0]] * 30), guess=0.0) == 0.0   # nothing near the guess


def test_finds_object_at_6cm():
    pts = np.vstack([floor(), box(1.5, 0.0, 0.33, 0.23, 0.06)])
    cl = lc.cluster_low_objects(pts, 0.0)
    assert len(cl) == 1
    c = cl[0]
    assert abs(c["x"] - 1.5) < 0.1 and abs(c["y"]) < 0.1
    assert 0.05 < c["height"] <= 0.065


def test_finds_keyboard_at_2_4cm():
    # 0.024 m: above h_min 0.012, below the 0.02 m ground tolerance of HeightBandLayer.
    pts = np.vstack([floor(noise=0.002), box(1.0, 0.5, 0.44, 0.12, 0.024)])
    cl = lc.cluster_low_objects(pts, 0.0)
    assert len(cl) == 1 and 0.015 < cl[0]["height"] < 0.03


def test_tall_object_and_neighbours_excluded():
    pts = np.vstack([floor(), box(2.0, 0.0, 0.4, 0.4, 0.9)])    # a chair-height block
    assert lc.cluster_low_objects(pts, 0.0) == []


def test_two_objects_two_clusters():
    pts = np.vstack([floor(), box(1.0, 0.0, 0.2, 0.2, 0.05), box(1.0, 1.0, 0.2, 0.2, 0.10)])
    hs = sorted(round(c["height"], 2) for c in lc.cluster_low_objects(pts, 0.0))
    assert hs == [0.05, 0.10]


def test_floor_error_does_not_make_one_giant_cluster():
    # Floor 3 cm above the assumed floor, dense like 1.5 s of stacked scans
    # (~7 points per 8 cm cell), so every cell joins one floor-sized cluster.
    pts = floor(z0=0.03, n=40000)
    assert lc.cluster_low_objects(pts, 0.0) == []    # max_cells drops the floor-sized cluster


def test_range_filter():
    cl = [{"x": 0.1, "y": 0.0}, {"x": 2.0, "y": 0.0}, {"x": 6.0, "y": 0.0}]
    assert [c["x"] for c in lc.in_range(cl, 0.0, 0.0, 0.25, 5.0)] == [2.0]
