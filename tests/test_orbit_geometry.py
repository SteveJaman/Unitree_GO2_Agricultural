#!/usr/bin/env python3
"""orbit_scanner.py's waypoints and view poses: maths only, no ROS graph, no arm.

  source /opt/ros/humble/setup.bash
  python3 -m pytest tests/test_orbit_geometry.py
"""

import os
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "d1_active_perception")
)
from d1_active_perception.orbit_scanner import (  # noqa: E402
    LINK_TO_OPTICAL,
    look_at,
    orbit_waypoints,
)

TARGET = np.array([0.0, 0.40, 0.15])


def azimuth(point):
    """Degrees around the plant, 0 on the side facing the arm."""
    v = point - TARGET
    facing = np.arctan2(-TARGET[1], -TARGET[0])
    return np.degrees((np.arctan2(v[1], v[0]) - facing + np.pi) % (2 * np.pi) - np.pi)


def test_thirteen_waypoints_span_180_degrees_in_equal_steps():
    points = orbit_waypoints(TARGET, 0.25, -90.0, 90.0, 13, 60.0)
    angles = [azimuth(p) for p in points]
    assert len(points) == 13
    assert angles[0] == pytest.approx(-90.0) and angles[-1] == pytest.approx(90.0)
    assert np.allclose(np.diff(angles), 15.0)


def test_waypoints_keep_the_radius_and_elevation():
    for p in orbit_waypoints(TARGET, 0.25, -90.0, 90.0, 13, 60.0):
        assert np.linalg.norm(p - TARGET) == pytest.approx(0.25)
        assert np.degrees(np.arcsin((p - TARGET)[2] / 0.25)) == pytest.approx(60.0)


def test_negative_elevation_puts_the_camera_below_the_plant():
    assert all(p[2] < TARGET[2] for p in orbit_waypoints(TARGET, 0.15, -60, 60, 5, -15.0))


def test_zero_elevation_is_a_flat_circle():
    assert all(p[2] == pytest.approx(TARGET[2]) for p in orbit_waypoints(TARGET, 0.2, -90, 90, 7))


@pytest.mark.parametrize("eye", [(0.2, 0.3, 0.4), (-0.1, 0.5, 0.2), (0.0, 0.4, 0.5)])
def test_look_at_is_optical_z_forward_x_right_y_down(eye):
    eye = np.array(eye)
    R = Rotation.from_quat(look_at(eye, TARGET)).as_matrix()
    forward = (TARGET - eye) / np.linalg.norm(TARGET - eye)
    assert np.allclose(R[:, 2], forward)
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-9) and np.linalg.det(R) == pytest.approx(1.0)
    assert R[2, 1] < 0.0 or abs(forward[2]) > 0.99  # y points down: the image is upright


def test_optical_to_camera_link_keeps_the_view_axis():
    # camera_link has x along the view, the optical frame z (d1_go2.urdf.xacro).
    view = Rotation.from_quat(look_at(np.array([0.2, 0.3, 0.4]), TARGET))
    link = view * LINK_TO_OPTICAL.inv()
    assert np.allclose(link.as_matrix()[:, 0], view.as_matrix()[:, 2])
