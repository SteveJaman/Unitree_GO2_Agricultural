#!/usr/bin/env python3
"""Gait supervisor: adapts quadruped gait based on upcoming terrain.

Monitors the local costmap ahead of the robot and commands gait changes
through the Go2's sport client interface (Phase 6) or logs intended
commands in dry_run mode.

Gait modes:
  NORMAL:    default trot gait for free terrain
  HIGH_STEP: raised foot clearance, objects the 0.12 m lift clears with foot_margin (h <= 0.10 m)
  CAREFUL:   slow deliberate gait, 0.10 m < h < lethal_height, or height not measured
  STOP:      halt for lethal obstacles (planner should reroute)

When 3D object boxes are available (object_box_fusion.py) the gait comes from
the tallest object in the corridor ahead, and HIGH_STEP is chosen only if the
raised foot clears it. The costmap decision stays in /gait/status for comparison;
with backstop_enabled the gait is never lower than the costmap's, so an object
the boxes missed still raises the feet.

With use_semantic, a track's label from vlm_heuristic_node.py can raise the gait
(CAUTION: at least CAREFUL, AVOID: STOP; low_confidence, i.e. no LiDAR height:
at least CAREFUL) but never lower it.

The backstop has two sources, both only able to raise the gait: the local
costmap ahead, and LiDAR-only low-object clusters (low_object_clusterer.py) in the
corridor, which find flat items no detector reports. Any cluster ahead means at
least CAREFUL; one at or above cluster_stop_height means STOP. /gait/status has
backstop_raised and backstop_source (costmap | cluster | costmap+cluster) when the
backstop lifted the gait above the boxes' choice.

Backends: "gazebo" publishes /gait/command, which the sim bridge turns into foot
lift. "sport_client" (the robot) is Phase 2 hardware work and raises
NotImplementedError; nothing here imports unitree_sdk2py.

Subscribes:
  /local_costmap/costmap   nav_msgs/OccupancyGrid
  /lidar3d/body_odom       nav_msgs/Odometry
  /objects/boxes_json      std_msgs/String
  /semantic/labels_json    std_msgs/String (use_semantic)
  /objects/low_clusters    sensor_msgs/PointCloud2 (backstop_enabled): x, y, z = height, radius

Publishes:
  /gait/command            std_msgs/String (gait mode name)
  /gait/status             std_msgs/String (detailed status, JSON)

Parameters:
  scan_distance:    how far ahead to scan (default 1.5m)
  scan_width:       lateral scan half-width (default 0.3m)
  check_rate_hz:    evaluation frequency (default 5.0)
  foot_margin:      clearance the raised foot must keep above an object (default 0.02m)
  lethal_height:    object height that means STOP (default 0.25m)
  backstop_enabled: gait is at least the costmap's and the clusters' (default true)
  cluster_stop_height: cluster height that means STOP (default 0.12 m, the foot-lift ceiling)
  cluster_max_age_s: clusters older than this are ignored (default 2.0 s)
  use_semantic:     raise the gait from semantic labels (default false)
  backend:          "gazebo" (default) | "sport_client" (NotImplementedError)
  dry_run:          if true, log commands but don't send them to the backend (default true)
"""
import json
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

COSTMAP_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)

STEP_OVER_MIN = 10
CAUTION_MIN = 60
LETHAL_MIN = 100

GAITS = {
    "NORMAL":    {"foot_height": 0.04, "speed_scale": 1.0},
    "HIGH_STEP": {"foot_height": 0.12, "speed_scale": 0.7},
    "CAREFUL":   {"foot_height": 0.12, "speed_scale": 0.4},
    "STOP":      {"foot_height": 0.04, "speed_scale": 0.0},
}


ORDER = ("NORMAL", "HIGH_STEP", "CAREFUL", "STOP")
SEMANTIC_GAIT = {"CAUTION": "CAREFUL", "AVOID": "STOP"}


def worst(*gaits):
    return max(gaits, key=ORDER.index)


def gait_for_height(height, foot_height, margin, lethal_height):
    if height is None:      # seen, but not measured: never treat it as low
        return "CAREFUL"
    if height + margin <= foot_height + 1e-6:   # 0.10 + 0.02 is 0.12000000000000001
        return "HIGH_STEP"
    return "CAREFUL" if height < lethal_height else "STOP"


def cluster_gait_for(clusters, stop_height):
    """Backstop gait from LiDAR-only clusters in the corridor: CAREFUL for any
    (unlabelled, so never assumed clearable), STOP at or above stop_height."""
    if not clusters:
        return None
    return "STOP" if max(c["height"] for c in clusters) >= stop_height - 1e-9 else "CAREFUL"


def parse_clusters(msg):
    """/objects/low_clusters PointCloud2 (x, y, z = height, radius) -> box-like dicts."""
    off = {f.name: f.offset for f in msg.fields}
    out = []
    for k in range(msg.width * msg.height):
        base = k * msg.point_step
        x, y, h = (struct.unpack_from("<f", msg.data, base + off[n])[0] for n in ("x", "y", "z"))
        r = struct.unpack_from("<f", msg.data, base + off["radius"])[0] if "radius" in off else 0.1
        out.append({"x": x, "y": y, "height": h, "sx": 2 * r, "sy": 2 * r})
    return out


def objects_ahead(boxes, x, y, yaw, scan_distance, scan_width):
    """Boxes whose footprint overlaps the corridor ahead of the body."""
    c, s = math.cos(yaw), math.sin(yaw)
    hits = []
    for b in boxes:
        dx, dy = b["x"] - x, b["y"] - y
        forward = c * dx + s * dy
        lateral = -s * dx + c * dy
        reach = max(b["sx"], b["sy"]) / 2
        if -0.45 - reach < forward < scan_distance + reach and abs(lateral) < scan_width + reach:
            hits.append(b)
    return hits


class GaitSupervisor(Node):
    def __init__(self):
        super().__init__("gait_supervisor")

        self.declare_parameter("scan_distance", 1.5)
        self.declare_parameter("scan_width", 0.3)
        self.declare_parameter("check_rate_hz", 5.0)
        self.declare_parameter("foot_margin", 0.02)
        self.declare_parameter("lethal_height", 0.25)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("backend", "gazebo")
        self.declare_parameter("backstop_enabled", True)
        self.declare_parameter("use_semantic", False)
        self.declare_parameter("semantic_topic", "/semantic/labels_json")
        self.declare_parameter("boxes_topic", "/objects/boxes_json")
        self.declare_parameter("costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("odom_topic", "/lidar3d/body_odom")
        self.declare_parameter("command_topic", "/gait/command")
        self.declare_parameter("status_topic", "/gait/status")
        self.declare_parameter("clusters_topic", "/objects/low_clusters")
        self.declare_parameter("cluster_stop_height", 0.12)
        self.declare_parameter("cluster_max_age_s", 2.0)

        self.backend = self.get_parameter("backend").value
        if self.backend == "sport_client":
            raise NotImplementedError(
                "SportClient backend is Phase 2 (hardware) work. Not implemented in sim.")
        if self.backend != "gazebo":
            raise ValueError(f"backend must be gazebo or sport_client, not {self.backend}")
        self.backstop = bool(self.get_parameter("backstop_enabled").value)
        self.use_semantic = bool(self.get_parameter("use_semantic").value)
        self.semantic = {}
        self.clusters, self.clusters_t = [], 0.0
        self.cluster_stop = float(self.get_parameter("cluster_stop_height").value)
        self.cluster_max_age = float(self.get_parameter("cluster_max_age_s").value)

        self.foot_margin = self.get_parameter("foot_margin").value
        self.lethal_height = self.get_parameter("lethal_height").value
        self.boxes = []
        self.scan_dist = self.get_parameter("scan_distance").value
        self.scan_width = self.get_parameter("scan_width").value
        self.dry_run = self.get_parameter("dry_run").value
        rate = self.get_parameter("check_rate_hz").value

        self.costmap = None
        self.odom = None
        self.current_gait = "NORMAL"

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.create_subscription(OccupancyGrid, g("costmap_topic"), self._cm_cb, COSTMAP_QOS)
        self.create_subscription(Odometry, g("odom_topic"), self._odom_cb, SENSOR_QOS)
        self.create_subscription(String, g("boxes_topic"), self._boxes_cb, 10)
        if self.use_semantic:
            self.create_subscription(String, g("semantic_topic"), self._semantic_cb, 10)
        if self.backstop:
            self.create_subscription(PointCloud2, g("clusters_topic"), self._clusters_cb, 10)

        self.cmd_pub = self.create_publisher(String, g("command_topic"), 10)
        self.status_pub = self.create_publisher(String, g("status_topic"), 10)
        self.create_timer(1.0 / rate, self._evaluate)

        mode = "DRY_RUN" if self.dry_run else "ACTIVE"
        self.get_logger().info(
            "gait_supervisor [%s]: backend=%s scan=%.1fm width=%.1fm backstop=%s semantic=%s"
            % (mode, self.backend, self.scan_dist, self.scan_width, self.backstop, self.use_semantic))

    def _cm_cb(self, msg):
        self.costmap = msg

    def _odom_cb(self, msg):
        self.odom = msg

    def _boxes_cb(self, msg):
        self.boxes = json.loads(msg.data)["boxes"]

    def _semantic_cb(self, msg):
        self.semantic = {o["track_id"]: (o["label"], bool(o.get("low_confidence")))
                         for o in json.loads(msg.data).get("objects", [])}

    def _clusters_cb(self, msg):
        self.clusters, self.clusters_t = parse_clusters(msg), time.monotonic()

    def _get_yaw(self):
        if self.odom is None:
            return 0.0
        q = self.odom.pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _scan_ahead(self):
        if self.costmap is None or self.odom is None:
            return 0

        info = self.costmap.info
        rx = self.odom.pose.pose.position.x
        ry = self.odom.pose.pose.position.y
        yaw = self._get_yaw()

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        max_cost = 0
        res = info.resolution
        n_forward = int(self.scan_dist / res)
        n_lateral = int(self.scan_width / res)

        for fd in range(1, n_forward + 1):
            for ld in range(-n_lateral, n_lateral + 1):
                wx = rx + fd * res * cos_y - ld * res * sin_y
                wy = ry + fd * res * sin_y + ld * res * cos_y

                mx = int((wx - info.origin.position.x) / res)
                my = int((wy - info.origin.position.y) / res)

                if 0 <= mx < info.width and 0 <= my < info.height:
                    c = self.costmap.data[my * info.width + mx]
                    if c > max_cost:
                        max_cost = c

        return max_cost

    def _evaluate(self):
        max_cost = self._scan_ahead()

        if max_cost >= LETHAL_MIN:
            costmap_gait = "STOP"
        elif max_cost >= CAUTION_MIN:
            costmap_gait = "CAREFUL"
        elif max_cost >= STEP_OVER_MIN:
            costmap_gait = "HIGH_STEP"
        else:
            costmap_gait = "NORMAL"

        desired = costmap_gait
        hits, box_gait, semantic_gait, cluster_gait, n_clusters = [], None, None, None, 0
        # With odometry the gait comes from the boxes (NORMAL when none are ahead,
        # including before the first box); the costmap only enters as the backstop.
        if self.odom is not None:
            p = self.odom.pose.pose.position
            hits = objects_ahead(self.boxes, p.x, p.y, self._get_yaw(),
                                 self.scan_dist, self.scan_width) if self.boxes else []
            desired = "NORMAL"
            if hits:
                gaits = [gait_for_height(b["height"] if b["traverse"] != "unknown" else None,
                                         GAITS["HIGH_STEP"]["foot_height"],
                                         self.foot_margin, self.lethal_height) for b in hits]
                desired = worst(*gaits)
                sem = []
                for b in hits if self.use_semantic else []:
                    label, low = self.semantic.get(b.get("id"), (None, False))
                    if label in SEMANTIC_GAIT:
                        sem.append(SEMANTIC_GAIT[label])
                    if low:   # no LiDAR height for it: never less than CAREFUL
                        sem.append("CAREFUL")
                if sem:
                    semantic_gait = worst(*sem)
                    desired = worst(desired, semantic_gait)
            box_gait = desired
            if self.backstop:
                fresh = time.monotonic() - self.clusters_t <= self.cluster_max_age
                ahead = objects_ahead(self.clusters, p.x, p.y, self._get_yaw(), self.scan_dist,
                                      self.scan_width) if fresh else []
                n_clusters = len(ahead)
                cluster_gait = cluster_gait_for(ahead, self.cluster_stop)
                desired = worst(desired, costmap_gait, *([cluster_gait] if cluster_gait else []))
        # The backstop raised the gait above what the boxes (and labels) asked for:
        # the costmap or a LiDAR cluster saw something ahead that the camera path did not.
        backstop_raised = box_gait is not None and desired != box_gait
        source = None
        if backstop_raised:
            source = "+".join(n for n, g in (("costmap", costmap_gait), ("cluster", cluster_gait))
                              if g and ORDER.index(g) > ORDER.index(box_gait)) or None

        if desired != self.current_gait:
            self.current_gait = desired
            self.get_logger().info(
                "gait -> %s (max_cost=%d source=%s dry_run=%s)"
                % (desired, max_cost, source or "boxes", self.dry_run))

            if not self.dry_run:
                self._send_gait_command(desired)

        self.cmd_pub.publish(String(data=desired))
        self.status_pub.publish(String(data=json.dumps({
            "gait": desired, "costmap_gait": costmap_gait, "box_gait": box_gait,
            "semantic_gait": semantic_gait, "backstop": self.backstop,
            "backstop_raised": backstop_raised, "backstop_source": source,
            "cluster_gait": cluster_gait, "clusters_ahead": n_clusters, "cost": max_cost,
            "objects": [b["class"] for b in hits],
            "foot": GAITS[desired]["foot_height"],
            "speed_pct": GAITS[desired]["speed_scale"] * 100})))

    def _send_gait_command(self, gait_name):
        # gazebo: /gait/command (published every cycle above) is the command; the
        # sim bridge sets the foot lift from it. Nothing further to send.
        params = GAITS[gait_name]
        self.get_logger().info(
            "gazebo backend: %s foot_height=%.2f speed_scale=%.1f"
            % (gait_name, params["foot_height"], params["speed_scale"]))


def main(args=None):
    rclpy.init(args=args)
    node = GaitSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
