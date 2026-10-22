"""Image region -> ground polygon in the map frame (no ROS imports, unit-tested).

The Go2 front camera has no depth, so a region of the image is placed on the
map by intersecting its pixel rays with a flat ground plane. Accuracy falls
off with distance, so rays are clipped at max_range.

Frames: map is x forward / y left / z up at the robot's yaw; the robot's roll
and pitch are ignored (lidar3d_map is levelled at start-up).
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class CameraModel:
    width: int
    height: int
    hfov_rad: float
    # Mount relative to base_link (metres) and downward pitch (radians).
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    pitch_down: float = 0.0

    @property
    def fx(self) -> float:
        return (self.width / 2.0) / math.tan(self.hfov_rad / 2.0)


@dataclass
class RobotPose:
    x: float
    y: float
    z: float
    yaw: float


def pixel_ray_body(cam: CameraModel, u: float, v: float) -> Tuple[float, float, float]:
    """Unit-free ray direction in base_link axes for pixel (u, v)."""
    f = cam.fx
    # Optical -> body axes: forward, left, up. Square pixels assumed.
    dx, dy, dz = 1.0, -(u - cam.width / 2.0) / f, -(v - cam.height / 2.0) / f
    # Pitch the camera nose-down about the body y axis.
    c, s = math.cos(cam.pitch_down), math.sin(cam.pitch_down)
    return (dx * c + dz * s, dy, -dx * s + dz * c)


def ground_point(cam: CameraModel, pose: RobotPose, u: float, v: float,
                 base_height: float, max_range: float) -> Optional[Tuple[float, float]]:
    """Where pixel (u, v) meets the ground, in map x/y.

    Rays that never reach the ground within max_range (at or above the
    horizon, or too far) are clipped to max_range horizontally from the
    camera, so a region that reaches the horizon becomes a band ending at
    max_range rather than disappearing. Returns None only for rays pointing
    backwards.
    """
    bx, by, bz = pixel_ray_body(cam, u, v)
    if bx <= 1e-6:
        return None
    cy, sy = math.cos(pose.yaw), math.sin(pose.yaw)
    # Camera origin and ray in map axes.
    ox = pose.x + cy * cam.x - sy * cam.y
    oy = pose.y + sy * cam.x + cy * cam.y
    oz = pose.z + cam.z
    dx, dy = cy * bx - sy * by, sy * bx + cy * by
    horiz = math.hypot(dx, dy)
    ground_z = pose.z - base_height
    t_max = max_range / horiz
    if bz < -1e-9:
        t = (ground_z - oz) / bz
        t = t_max if t <= 0.0 or t > t_max else t
    else:
        t = t_max
    return (ox + t * dx, oy + t * dy)


def region_polygon(cam: CameraModel, pose: RobotPose,
                   u0: float, u1: float, v0: float, v1: float,
                   base_height: float, max_range: float,
                   samples: int = 6) -> List[Tuple[float, float]]:
    """Ground polygon (map x/y) for the pixel rectangle [u0,u1] x [v0,v1].

    The rectangle border is sampled so that range clipping (an arc on the
    ground) is followed reasonably closely.
    """
    border = []
    for i in range(samples):            # top edge, left -> right
        border.append((u0 + (u1 - u0) * i / samples, v0))
    for i in range(samples):            # right edge, top -> bottom
        border.append((u1, v0 + (v1 - v0) * i / samples))
    for i in range(samples):            # bottom edge, right -> left
        border.append((u1 - (u1 - u0) * i / samples, v1))
    for i in range(samples):            # left edge, bottom -> top
        border.append((u0, v1 - (v1 - v0) * i / samples))
    pts = [ground_point(cam, pose, u, v, base_height, max_range) for u, v in border]
    return [p for p in pts if p is not None]


def strip_bounds(width: int, height: int, n_strips: int,
                 top_frac: float) -> List[Tuple[int, int, int, int]]:
    """Split the lower part of the image (rows top_frac..1) into vertical strips.

    Returns (u0, u1, v0, v1) per strip, left to right.
    """
    v0 = int(round(height * top_frac))
    out = []
    for i in range(n_strips):
        u0 = int(round(width * i / n_strips))
        u1 = int(round(width * (i + 1) / n_strips)) - 1
        out.append((u0, u1, v0, height - 1))
    return out
