"""
mapping.py

Industry-standard mapping core. Pure Python, no ROS 2 required.

Consumes Point-LIO output (/cloud_registered, /state_estimation) and builds:
  - A log-odds occupancy grid (for /map/occupancy in RViz)
  - An accumulated global point cloud
  - A Poisson-reconstructed mesh (for Unity / MeshLab export)
  - Optional RGB-colored points (when camera images are available)

The occupancy grid uses the log-odds Bayesian update from Probabilistic
Robotics (Thrun et al., section 9.2), which is the same algorithm used by
OctoMap and gmapping.
"""

import math
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Log-odds constants (from Probabilistic Robotics, Table 9.1)
# ---------------------------------------------------------------------------

LOG_ODDS_OCCUPIED = 0.85       # p = 0.7
LOG_ODDS_FREE = -0.4           # p = 0.4
LOG_ODDS_MIN = -2.0            # clamp to avoid runaway
LOG_ODDS_MAX = 3.5             # clamp
LOG_ODDS_UNKNOWN = 0.0         # p = 0.5


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class MapConfig:
    """Tunable parameters for the map accumulator."""

    # World extent (meters). The map is a square centered at the origin.
    size_m: float = 50.0
    resolution_m: float = 0.05

    # Height window (meters). Points outside this Z band are discarded.
    z_min: float = -0.5
    z_max: float = 3.0

    # Occupancy update
    hit_log_odds: float = LOG_ODDS_OCCUPIED
    miss_log_odds: float = LOG_ODDS_FREE

    # Cloud accumulation
    max_points: int = 8_000_000
    downsample_voxel_m: float = 0.03

    # Mesh reconstruction
    poisson_depth: int = 9
    density_quantile: float = 0.05

    # Sensor model
    lidar_min_range: float = 0.2
    lidar_max_range: float = 30.0


# ---------------------------------------------------------------------------
# OccupancyGrid — log-odds Bayesian update
# ---------------------------------------------------------------------------

class OccupancyGrid:
    """
    Top-down 2D occupancy grid using log-odds updates.

    Internal values are float32 log-odds:
      negative -> free
      zero     -> unknown
      positive -> occupied
    """

    def __init__(self, config: Optional[MapConfig] = None):
        self.config = config or MapConfig()
        n = int(self.config.size_m / self.config.resolution_m)
        self._n = n
        self._origin = self.config.size_m / 2.0
        self._log_odds = np.full(
            (n, n), LOG_ODDS_UNKNOWN, dtype=np.float32)
        self._hit_count = np.zeros((n, n), dtype=np.uint32)

    # ------------------------------------------------------------------
    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        col = int((x + self._origin) / self.config.resolution_m)
        row = int((self._origin - y) / self.config.resolution_m)
        return col, row

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self._n and 0 <= row < self._n

    # ------------------------------------------------------------------
    def add_scan(
        self,
        sensor_origin_xy: Tuple[float, float],
        points_xyz: np.ndarray,
        intensities: Optional[np.ndarray] = None,
    ) -> None:
        """
        Update the grid with one LiDAR sweep.

        Points are assumed to be in the MAP frame (i.e. already transformed
        by Point-LIO). The sensor_origin_xy is the robot's position in the
        map frame; it is used for free-space ray casting.
        """
        if len(points_xyz) == 0:
            return

        cfg = self.config
        z = points_xyz[:, 2]
        mask = (z >= cfg.z_min) & (z <= cfg.z_max)
        pts = points_xyz[mask]
        if len(pts) == 0:
            return

        # Mark the sensor origin as free
        c0, r0 = self.world_to_pixel(*sensor_origin_xy)
        if self.in_bounds(c0, r0):
            self._log_odds[r0, c0] += cfg.miss_log_odds
            self._log_odds[r0, c0] = np.clip(
                self._log_odds[r0, c0], LOG_ODDS_MIN, LOG_ODDS_MAX)

        # Mark hit cells
        for p in pts:
            col, row = self.world_to_pixel(p[0], p[1])
            if not self.in_bounds(col, row):
                continue
            self._log_odds[row, col] += cfg.hit_log_odds
            self._log_odds[row, col] = np.clip(
                self._log_odds[row, col], LOG_ODDS_MIN, LOG_ODDS_MAX)
            self._hit_count[row, col] += 1

        # Ray casting: mark cells between sensor and hit as free
        self._cast_free_rays(sensor_origin_xy, pts)

    # ------------------------------------------------------------------
    def _cast_free_rays(
        self, origin_xy: Tuple[float, float], points: np.ndarray
    ) -> None:
        """Bresenham-style ray cast to mark free space."""
        o_col, o_row = self.world_to_pixel(*origin_xy)

        # Subsample points to avoid casting 10k rays per scan
        stride = max(1, len(points) // 180)
        for p in points[::stride]:
            t_col, t_row = self.world_to_pixel(p[0], p[1])
            self._bresenham_free(o_col, o_row, t_col, t_row)

    def _bresenham_free(self, c0: int, r0: int, c1: int, r1: int) -> None:
        """Mark cells along the line as free (excluding endpoints)."""
        dc = abs(c1 - c0)
        dr = abs(r1 - r0)
        sc = 1 if c0 < c1 else -1
        sr = 1 if r0 < r1 else -1
        err = dc - dr
        col, row = c0, r0
        cfg = self.config

        while True:
            if col == c1 and row == r1:
                break
            if self.in_bounds(col, row):
                self._log_odds[row, col] += cfg.miss_log_odds
                self._log_odds[row, col] = np.clip(
                    self._log_odds[row, col], LOG_ODDS_MIN, LOG_ODDS_MAX)
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                col += sc
            if e2 < dc:
                err += dc
                row += sr

    # ------------------------------------------------------------------
    def probability(self) -> np.ndarray:
        """Convert log-odds to probability in [0, 1]."""
        return 1.0 - 1.0 / (1.0 + np.exp(self._log_odds))

    def to_occupancy_grid_data(self) -> np.ndarray:
        """Return int8 array for nav_msgs/OccupancyGrid.
        0 = free, 100 = occupied, -1 = unknown."""
        prob = self.probability()
        out = np.full(prob.shape, -1, dtype=np.int8)
        out[prob > 0.65] = 100
        out[prob < 0.35] = 0
        return out

    def render_gray(self) -> np.ndarray:
        """Return uint8 grayscale. 0 = occupied (dark)."""
        prob = self.probability()
        return ((1.0 - prob) * 255).astype(np.uint8)

    def save_png(self, path: Path) -> None:
        _write_grayscale_png(Path(path), self.render_gray())

    @property
    def shape(self) -> Tuple[int, int]:
        return self._log_odds.shape

    @property
    def stats(self) -> dict:
        prob = self.probability()
        return {
            "size_m": self.config.size_m,
            "resolution_m": self.config.resolution_m,
            "grid_size": self._n,
            "total_hits": int(self._hit_count.sum()),
            "occupied_cells": int((prob > 0.65).sum()),
            "free_cells": int((prob < 0.35).sum()),
            "unknown_cells": int(((prob >= 0.35) & (prob <= 0.65)).sum()),
        }


# ---------------------------------------------------------------------------
# PointCloud accumulator
# ---------------------------------------------------------------------------

class CloudAccumulator:
    """
    Accumulates aligned point clouds with optional RGB color.

    Voxel-downsamples on insertion to keep memory bounded. Uses a
    voxel hash so that points from different scans landing in the
    same voxel are merged (last-write-wins for color).

    This is the same pattern used by RTAB-Map's cloud assembly.
    """

    def __init__(self, config: Optional[MapConfig] = None):
        self.config = config or MapConfig()
        self._points: list = []
        self._colors: list = []
        self._n_points: int = 0

    def add(
        self,
        points_xyz: np.ndarray,
        colors_rgb: Optional[np.ndarray] = None,
    ) -> None:
        if len(points_xyz) == 0:
            return
        if self._n_points >= self.config.max_points:
            return

        stride = max(1, len(points_xyz) // 5000)
        pts = points_xyz[::stride]

        # Voxel downsample this scan
        pts, idx = self._voxel_downsample_with_indices(
            pts, self.config.downsample_voxel_m)

        if len(pts) == 0:
            return

        self._points.append(pts)
        if colors_rgb is not None:
            cols = colors_rgb[::stride]
            # Apply the same indexing from voxel downsample
            cols = cols[idx]
            if len(cols) != len(pts):
                # Safety: truncate to matching length
                cols = cols[:len(pts)]
            self._colors.append(cols)
        self._n_points += len(pts)

    @staticmethod
    def _voxel_downsample_with_indices(
        points: np.ndarray, voxel: float
    ) -> tuple:
        """
        Voxel downsample returning both the downsampled points and the
        indices into the original array.

        This is critical for RGB fusion: we must keep colors aligned with
        the points they came from.
        """
        if len(points) == 0 or voxel <= 0:
            return points, np.arange(len(points))
        keys = np.floor(points[:, :3] / voxel).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        idx = np.sort(idx)  # preserve original order
        return points[idx], idx

    def points(self) -> np.ndarray:
        if not self._points:
            return np.zeros((0, 3), dtype=np.float32)
        return np.vstack(self._points)

    def colors(self) -> Optional[np.ndarray]:
        if not self._colors:
            return None
        return np.vstack(self._colors)

    def has_color(self) -> bool:
        return len(self._colors) > 0 and len(self._colors) == len(self._points)

    def clear(self) -> None:
        self._points.clear()
        self._colors.clear()
        self._n_points = 0

    @property
    def count(self) -> int:
        return self._n_points


# ---------------------------------------------------------------------------
# Mesh reconstruction (Open3D)
# ---------------------------------------------------------------------------

def reconstruct_mesh(
    points: np.ndarray,
    colors: Optional[np.ndarray] = None,
    config: Optional[MapConfig] = None,
    output_path: Optional[Path] = None,
):
    """
    Reconstruct a watertight triangle mesh using Poisson surface
    reconstruction. Requires open3d.

    Returns the Open3D TriangleMesh or None if open3d is unavailable.
    """
    try:
        import open3d as o3d
    except ImportError:
        print("open3d not installed. pip install open3d")
        return None

    cfg = config or MapConfig()

    if len(points) < 100:
        print(f"Too few points ({len(points)}) for mesh reconstruction.")
        return None

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))

    # Downsample and denoise
    pcd = pcd.voxel_down_sample(voxel_size=cfg.downsample_voxel_m)
    pcd, _ = pcd.remove_statistical_outlier(
        nb_neighbors=20, std_ratio=2.0)

    # Estimate normals (required for Poisson)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=cfg.downsample_voxel_m * 4, max_nn=30))

    # Poisson reconstruction
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=cfg.poisson_depth)

    # Drop the low-density envelope (Poisson adds a bubble around the scene)
    densities = np.asarray(densities)
    keep = densities > np.quantile(densities, cfg.density_quantile)
    mesh.remove_vertices_by_mask(~keep)
    mesh.compute_vertex_normals()

    # Transfer colors from the cloud to the mesh
    if colors is not None and pcd.has_colors():
        pcd_colors = np.asarray(pcd.colors)
        tree = o3d.geometry.KDTreeFlann(pcd)
        mesh_colors = np.zeros((len(mesh.vertices), 3))
        for i, v in enumerate(mesh.vertices):
            _, idx, _ = tree.search_knn_vector_3d(v, 1)
            mesh_colors[i] = pcd_colors[idx[0]]
        mesh.vertex_colors = o3d.utility.Vector3dVector(mesh_colors)

    if output_path is not None:
        out = Path(output_path)
        # OBJ preserves vertex colors for Unity import
        o3d.io.write_triangle_mesh(str(out), mesh,
                                    write_vertex_colors=True,
                                    write_ascii=False)
        print(f"Mesh written: {out}")

    return mesh


# ---------------------------------------------------------------------------
# PLY / PNG writers (standard library, no dependencies)
# ---------------------------------------------------------------------------

def write_pointcloud_ply(
    path: Path,
    points: np.ndarray,
    colors: Optional[np.ndarray] = None,
) -> None:
    """Write a binary PLY with XYZ and optional RGB."""
    path = Path(path)
    n = len(points)
    has_color = colors is not None and len(colors) == n

    header_lines = [
        "ply",
        "format binary_little_endian 1.0",
        f"element vertex {n}",
        "property float x",
        "property float y",
        "property float z",
    ]
    if has_color:
        header_lines += [
            "property uchar red",
            "property uchar green",
            "property uchar blue",
        ]
    header_lines.append("end_header")
    header = "\n".join(header_lines) + "\n"

    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        if has_color:
            cols_u8 = (np.clip(colors, 0, 1) * 255).astype(np.uint8)
            buf = bytearray()
            for p, c in zip(points, cols_u8):
                buf += struct.pack("fffBBB",
                                   float(p[0]), float(p[1]), float(p[2]),
                                   int(c[0]), int(c[1]), int(c[2]))
            f.write(bytes(buf))
        else:
            f.write(points.astype(np.float32).tobytes())


def _write_grayscale_png(path: Path, image: np.ndarray) -> None:
    import zlib
    h, w = image.shape
    raw = bytearray()
    for row in image:
        raw.append(0)
        raw.extend(row.tobytes())

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (len(data).to_bytes(4, "big") + tag + data +
                zlib.crc32(tag + data).to_bytes(4, "big"))

    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)
    idat = zlib.compress(bytes(raw), 9)
    png = sig + chunk(b"IHDR", ihdr) + chunk(b"IDAT", idat) + chunk(b"IEND", b"")
    Path(path).write_bytes(png)