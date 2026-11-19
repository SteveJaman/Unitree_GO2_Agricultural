"""Pure-function tests for the servo demo's data tools (plot_distance, servo_trials_report)
and the tracker's raw depth decoding."""
import importlib.util
import math
import os
import sys

import numpy as np
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load(name, rel):
    spec = importlib.util.spec_from_file_location(name, os.path.join(REPO, rel))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


plot = load("plot_distance", "scripts/tools/plot_distance.py")
report = load("servo_trials_report", "scripts/tools/servo_trials_report.py")


def test_summary_settles_and_overshoot():
    data = {"t": [0, 1, 2, 3, 4, 5], "d": [0.60, 0.56, 0.515, 0.49, 0.50, 0.51],
            "x": [0] * 6, "step": [0] * 6, "state": ["step", "step", "step", "hold", "hold", "hold"]}
    s = plot.summary(data, 0.50, 0.02)
    assert s["start_m"] == pytest.approx(0.60)
    assert s["final_m"] == pytest.approx(0.51)
    assert s["settle_s"] == pytest.approx(2.0)          # inside +/-2 cm from t = 2 s to the end
    assert s["overshoot_m"] == pytest.approx(0.01)      # 0.49: 1 cm past the target
    assert s["steps"] == 3


def test_summary_without_distances():
    data = {"t": [0, 1], "d": [math.nan, math.nan], "x": [0, 0], "step": [0, 0], "state": ["stale", "stale"]}
    assert plot.summary(data, 0.5, 0.02) == {"seen": 0}


def test_ci95_brackets_the_mean():
    m, lo, hi = report.ci95([1.0, 2.0, 3.0, 4.0, 5.0], n=2000)
    assert m == pytest.approx(3.0)
    assert lo < m < hi
    assert math.isnan(report.ci95([math.nan])[0])


def test_tracker_decodes_raw_depth():
    pytest.importorskip("rclpy")
    pytest.importorskip("message_filters")
    sys.path.insert(0, os.path.join(REPO, "scripts", "nodes"))
    tracker = load("yolo_depth_tracker", "scripts/nodes/yolo_depth_tracker.py")
    from sensor_msgs.msg import Image

    d = np.array([[0.5, np.nan], [np.inf, 1.25]], np.float32)
    msg = Image(height=2, width=2, encoding="32FC1", step=8, data=d.tobytes())
    out = tracker.decode_depth(msg)
    assert out.tolist() == [[0.5, 0.0], [0.0, 1.25]]
    mm = np.array([[500, 0], [1000, 2000]], np.uint16)
    msg = Image(height=2, width=2, encoding="16UC1", step=4, data=mm.tobytes())
    assert tracker.decode_depth(msg).tolist() == [[0.5, 0.0], [1.0, 2.0]]


def test_tracker_follows_one_box():
    pytest.importorskip("rclpy")
    pytest.importorskip("message_filters")
    sys.path.insert(0, os.path.join(REPO, "scripts", "nodes"))
    tracker = load("yolo_depth_tracker2", "scripts/nodes/yolo_depth_tracker.py")
    centre = (300, 220, 340, 260, "lettuce", 0.9)
    near = (20, 20, 120, 120, "lettuce", 0.5)
    # start: the largest box (the nearest plant), not the most confident nor the most central
    assert tracker.choose_box([centre, near], None, 640, 480) == near
    side = (20, 20, 120, 120, "lettuce", 0.9)
    # then the one overlapping the last box, even if another is more confident
    moved = (305, 222, 345, 262, "lettuce", 0.45)
    assert tracker.choose_box([side, moved], centre, 640, 480) == moved
    assert tracker.choose_box([side, moved], centre, 640, 480, by_confidence=True) == side
    assert tracker.choose_box([], centre, 640, 480) is None


def test_depth_in_box_reads_the_front_leaves():
    pytest.importorskip("rclpy")
    pytest.importorskip("message_filters")
    sys.path.insert(0, os.path.join(REPO, "scripts", "nodes"))
    tracker = load("yolo_depth_tracker3", "scripts/nodes/yolo_depth_tracker.py")
    d = np.full((100, 100), 0.85, np.float32)      # background behind the plant
    d[30:70, 30:70] = 0.62                          # leaves
    d[48:52, 48:52] = 0.85                          # a gap at the box centre
    assert tracker.depth_at(d, 50, 50, 3) == pytest.approx(0.85)
    assert tracker.depth_in_box(d, (30, 30, 70, 70)) == pytest.approx(0.62)
    assert tracker.depth_in_box(np.zeros((10, 10), np.float32), (0, 0, 10, 10)) is None
