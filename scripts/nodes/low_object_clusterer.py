#!/usr/bin/env python3
"""LiDAR-only low-object clusterer: finds objects on the floor without a camera detection.

HeightBandLayer and object_box_fusion need a detection to measure an object, so
very flat items (a keyboard at 0.024 m, a book at 0.05 m) that no detector
reports were missed by both. This node finds them from the LiDAR alone and feeds
the gait supervisor's backstop. Geometry only: no VLM, no labels, no costmap
writes; a cluster can only raise the gait.

Ported from Yassine Salimi's traversability_core.py (branch
protected/ysalimi-VLM-traversability, commit e0715c1): estimate_floor() and
find_candidates() with his values (5 mm floor bins within +-0.15 m of the TF
guess, 8 cm cells, h_min 0.012 m, 2 points per cell, 1.5 s scan stacking).

Pipeline: stack accum_s of map-frame clouds -> floor height (histogram mode near
base_link z - base_height) -> 8 cm grid -> drop cells above h_tall and their
neighbours (walls, furniture) -> connected low cells -> clusters.

Subscribes:  cloud_topic (default /lidar3d/obstacle_points: self-filtered, floor
             kept; /lidar3d/registered still contains the robot's own body)
             TF map_frame -> base_frame
Publishes:   /objects/low_clusters       sensor_msgs/PointCloud2, map frame, one
                                         point per cluster: x, y, z = height above
                                         the floor, radius (footprint half-diagonal)
             /objects/low_clusters_json  std_msgs/String, floor_z and clusters (logging)
With dry_run (default true) clusters are logged, nothing is published.
backend: "gazebo" and "robot" read the same topics; only the cloud source differs.
"""
import json
import math

import numpy as np


# ----------------------------------------------------------------- floor
def estimate_floor(pts, guess, window=0.15, bin_w=0.005, min_pts=20):
    """Floor z = most populated bin_w z-bin within +-window of the TF guess (Yassine's)."""
    if len(pts) == 0:
        return float(guess)
    z = pts[:, 2]
    z = z[np.abs(z - guess) < window]
    if len(z) < min_pts:
        return float(guess)
    bins = np.arange(guess - window, guess + window + bin_w, bin_w)
    counts, edges = np.histogram(z, bins=bins)
    k = int(np.argmax(counts))
    centre = (edges[k] + edges[k + 1]) / 2
    near = z[np.abs(z - centre) < 0.01]
    return float(np.median(near)) if len(near) else float(centre)


# ----------------------------------------------------------------- clustering
def cluster_low_objects(pts, floor_z, cell=0.08, h_min=0.012, h_tall=0.5, h_scan=2.0,
                        min_cell_pts=2, max_cells=400):
    """Low-object clusters in an Nx3 map-frame cloud (Yassine's find_candidates).

    Cells whose highest point exceeds h_tall (walls, furniture) and their 8
    neighbours are excluded, so only free-standing low objects remain. Clusters
    of more than max_cells cells are dropped (a floor error would otherwise make
    the whole floor one cluster). Returns dicts: x, y, height (95th percentile,
    or max under 8 points), n_pts, n_cells, radius, bbox.
    """
    if len(pts) == 0:
        return []
    h = pts[:, 2] - floor_z
    keep = (h > h_min) & (h < h_scan)
    pts, h = pts[keep], h[keep]
    if len(pts) == 0:
        return []
    ij = np.floor(pts[:, :2] / cell).astype(np.int64)
    uniq, inv = np.unique(ij, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    cmax = np.full(len(uniq), -np.inf)
    np.maximum.at(cmax, inv, h)
    cnt = np.bincount(inv, minlength=len(uniq))
    index = {(int(a), int(b)): k for k, (a, b) in enumerate(uniq)}

    blocked = set()
    for (a, b), k in index.items():
        if cmax[k] > h_tall:
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    blocked.add((a + da, b + db))
    cand = {key for key, k in index.items() if key not in blocked and cnt[k] >= min_cell_pts}

    lab = np.full(len(uniq), -1, dtype=np.int64)
    clusters, seen = [], set()
    for start in sorted(cand):
        if start in seen:
            continue
        cid, queue, members = len(clusters), [start], []
        seen.add(start)
        while queue:
            a, b = queue.pop()
            members.append((a, b))
            lab[index[(a, b)]] = cid
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    nb = (a + da, b + db)
                    if nb in cand and nb not in seen:
                        seen.add(nb)
                        queue.append(nb)
        clusters.append(members)

    plab = lab[inv]
    out = []
    for cid, members in enumerate(clusters):
        if len(members) > max_cells:
            continue
        sel = plab == cid
        hs, xy = h[sel], pts[sel, :2]
        height = float(np.percentile(hs, 95)) if len(hs) >= 8 else float(hs.max())
        (x0, y0), (x1, y1) = xy.min(axis=0), xy.max(axis=0)
        out.append({"x": float(xy[:, 0].mean()), "y": float(xy[:, 1].mean()), "height": height,
                    "n_pts": int(sel.sum()), "n_cells": len(members),
                    "radius": float(math.hypot(x1 - x0, y1 - y0) / 2 + cell / 2),
                    "bbox": [[float(x0), float(y0)], [float(x1), float(y1)]]})
    return out


def in_range(clusters, x, y, min_range, max_range):
    return [c for c in clusters if min_range <= math.hypot(c["x"] - x, c["y"] - y) <= max_range]


def main():
    import struct
    import time
    from collections import deque

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    def cloud_xyz(msg):
        off = {f.name: f.offset for f in msg.fields}
        dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                       "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
        raw = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
        p = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float64)
        return p[np.isfinite(p).all(axis=1)]

    class LowObjectClusterer(Node):
        def __init__(self):
            super().__init__("low_object_clusterer")
            p = self.declare_parameter
            p("cloud_topic", "/lidar3d/obstacle_points")
            p("clusters_topic", "/objects/low_clusters")
            p("map_frame", "lidar3d_map")
            p("base_frame", "base_link")
            p("base_height", 0.30)        # base_link above the floor: the floor guess
            p("accum_s", 1.5)
            p("grid_m", 0.08)
            p("h_min_m", 0.012)
            p("h_tall_m", 0.5)
            p("step_limit_m", 0.12)       # the real Go2 foot-lift ceiling; reported per cluster
            p("min_cell_points", 2)
            p("max_cells", 400)
            p("min_range_m", 0.25)        # self-filter residue around the body
            p("max_range_m", 5.0)
            p("rate_hz", 2.0)
            p("dry_run", True)
            p("backend", "gazebo")
            g = lambda k: self.get_parameter(k).value  # noqa: E731
            if g("backend") not in ("gazebo", "robot"):
                raise ValueError(f"backend must be gazebo or robot, not {g('backend')}")
            self.cfg = {k: g(k) for k in ("map_frame", "base_frame", "base_height", "accum_s", "grid_m",
                                          "h_min_m", "h_tall_m", "step_limit_m", "min_cell_points",
                                          "max_cells", "min_range_m", "max_range_m")}
            self.dry_run = bool(g("dry_run"))
            self.clouds = deque()
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.create_subscription(PointCloud2, g("cloud_topic"), self.on_cloud, qos_profile_sensor_data)
            self.pub = self.create_publisher(PointCloud2, g("clusters_topic"), 10)
            self.json_pub = self.create_publisher(String, g("clusters_topic") + "_json", 10)
            self.create_timer(1.0 / float(g("rate_hz")), self.tick)
            self.get_logger().info(
                f"low_object_clusterer dry_run={self.dry_run} backend={g('backend')} cloud={g('cloud_topic')} "
                f"grid={self.cfg['grid_m']} h_min={self.cfg['h_min_m']} step_limit={self.cfg['step_limit_m']}")

        def on_cloud(self, msg):
            now = time.monotonic()
            self.clouds.append((now, cloud_xyz(msg)))
            while self.clouds and now - self.clouds[0][0] > self.cfg["accum_s"]:
                self.clouds.popleft()

        def tick(self):
            c = self.cfg
            if not self.clouds:
                return
            try:
                tf = self.tf.lookup_transform(c["map_frame"], c["base_frame"], Time())
            except TransformException as ex:
                self.get_logger().warn(str(ex), throttle_duration_sec=5.0)
                return
            bx, by, bz = tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z
            pts = np.concatenate([p for _, p in self.clouds])
            pts = pts[np.hypot(pts[:, 0] - bx, pts[:, 1] - by) <= c["max_range_m"]]
            floor_z = estimate_floor(pts, bz - c["base_height"])
            clusters = cluster_low_objects(pts, floor_z, cell=c["grid_m"], h_min=c["h_min_m"],
                                           h_tall=c["h_tall_m"], min_cell_pts=c["min_cell_points"],
                                           max_cells=c["max_cells"])
            clusters = in_range(clusters, bx, by, c["min_range_m"], c["max_range_m"])
            for cl in clusters:
                cl["above_step_limit"] = cl["height"] >= c["step_limit_m"]
            if self.dry_run:
                self.get_logger().info(f"dry_run: floor_z={floor_z:.3f} clusters={len(clusters)}",
                                       throttle_duration_sec=5.0)
                return
            msg = PointCloud2()
            msg.header.frame_id = c["map_frame"]
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height, msg.width = 1, len(clusters)
            msg.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                          for k, n in enumerate(("x", "y", "z", "radius"))]
            msg.point_step, msg.row_step = 16, 16 * len(clusters)
            msg.is_dense = True
            msg.data = b"".join(struct.pack("<ffff", cl["x"], cl["y"], cl["height"], cl["radius"])
                                for cl in clusters)
            self.pub.publish(msg)
            self.json_pub.publish(String(data=json.dumps(
                {"stamp": time.time(), "floor_z": round(floor_z, 4),
                 "clusters": [{k: (round(v, 3) if isinstance(v, float) else v) for k, v in cl.items()
                               if k != "bbox"} for cl in clusters]})))

    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    node = LowObjectClusterer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # SIGINT / SIGTERM from the trial scripts
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
