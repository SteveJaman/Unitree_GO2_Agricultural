#!/usr/bin/env python3
"""
go2_debug_probe.py: read-only health probe for the debug page
(scripts/tools/debug_dashboard.py, http://localhost:3001).

Runs inside go2-lidar3d next to Nav2, on the session's DDS setup. It only
subscribes (never publishes, never moves anything) and prints one JSON line
per second on stdout:

  topics     rate (Hz over the last 5 s) and age of the last message, for
             the topics the "why isn't it moving?" checks need. Only topics
             published on this computer are measured: an extra subscriber to
             a cloud that comes over Wi-Fi from the robot would make the robot
             send it twice.
  pose       lidar3d_map -> base_link from TF: x, y, yaw, TF age, and the
             speed actually walked (from the pose over the last 1.5 s)
  cmd        last /cmd_vel_nav (controller) and /cmd_vel (smoother, recovery,
             Drive pad) and their ages
  nav        latest goal states of /compute_path_to_pose and /follow_path,
             time of the last abort
  costmap    local costmap: highest cost under the footprint and right ahead;
             every 2 s also the grid (base64, 255 = unknown), the footprint
             and the current /plan, for the page's drawing
  status     latest /explore/status, /recovery/status, /teleop/status,
             /vlm_decision, /battery_manager/status, /battery_state, /goal_pose
  rosout     WARN and above from every node (last 200), and counts per node
  nodes      the ROS node names seen (every 5 s)

Stops when stdin closes (the page server went away).
"""

import base64
import json
import math
import sys
import threading
import time
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, Image, JointState, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

MAP, BASE = "lidar3d_map", "base_link"
FOOTPRINT = (
    -0.45,
    0.36,
    -0.20,
    0.20,
)  # x0, x1, y0, y1 in base_link (go2_footprint.yaml)
AHEAD = (0.36, 0.80, -0.22, 0.22)
# Measured topics: (topic, type, raw). raw = never deserialised (clouds, images).
RATE_TOPICS = [
    ("/lidar3d/obstacle_points", PointCloud2, True),
    ("/joint_states", JointState, True),
    ("/cmd_vel_nav", Twist, False),
    ("/cmd_vel", Twist, False),
    ("/plan", Path, False),
    ("/local_costmap/costmap", OccupancyGrid, False),
    ("/global_costmap/costmap", OccupancyGrid, True),
    ("/camera/image_raw", Image, True),
]
STATUS = {
    "/explore/status": String,
    "/recovery/status": String,
    "/teleop/status": String,
    "/vlm_decision": String,
    "/battery_manager/status": String,
}
ACTIONS = ("/compute_path_to_pose", "/follow_path")
STATE_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "unknown",
    GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "executing",
    GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}
LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self):
        super().__init__("go2_debug_probe")
        self.t0 = time.monotonic()
        self.stamps = {t: deque(maxlen=400) for t, _, _ in RATE_TOPICS}
        self.cmd = {}
        self.costmap = None
        self.plan = []
        self.status = {}
        self.battery = None
        self.goal = None
        self.nav = {
            a: {"state": None, "since": None, "last_abort": None} for a in ACTIONS
        }
        self.rosout = deque(maxlen=200)
        self.rosout_counts = {}
        self.nodes = []
        self.poses = deque(maxlen=60)
        self.tf_age = None
        self.tf = Buffer()
        TransformListener(self.tf, self)

        for topic, typ, raw in RATE_TOPICS:
            self.create_subscription(
                typ, topic, self._rate_cb(topic, raw), qos_profile_sensor_data, raw=raw
            )
        for topic, typ in STATUS.items():
            self.create_subscription(typ, topic, self._status_cb(topic), 10)
        for a in ACTIONS:
            self.create_subscription(
                GoalStatusArray,
                f"{a}/_action/status",
                self._action_cb(a),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            BatteryState, "/battery_state", self._battery, qos_profile_sensor_data
        )
        self.create_subscription(PoseStamped, "/goal_pose", self._goal, 10)
        self.create_subscription(Log, "/rosout", self._rosout, 100)
        self.create_timer(0.1, self._sample_pose)
        self.create_timer(1.0, self._emit)
        self.create_timer(5.0, self._list_nodes)
        self.n_emit = 0

    # ------------------------------------------------------------- inputs
    def _rate_cb(self, topic, raw):
        def cb(msg):
            now = time.monotonic()
            self.stamps[topic].append(now)
            if raw:
                return
            if topic in ("/cmd_vel_nav", "/cmd_vel"):
                self.cmd[topic] = (now, msg.linear.x, msg.linear.y, msg.angular.z)
            elif topic == "/local_costmap/costmap":
                self.costmap = msg
            elif topic == "/plan":
                pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
                step = max(1, len(pts) // 150)
                self.plan = [(round(x, 2), round(y, 2)) for x, y in pts[::step]]
                self.plan_frame = msg.header.frame_id

        return cb

    def _status_cb(self, topic):
        def cb(msg):
            self.status[topic] = (time.monotonic(), msg.data)

        return cb

    def _action_cb(self, action):
        def cb(msg):
            if not msg.status_list:
                return
            # Newest goal = latest accept stamp.
            s = max(
                msg.status_list,
                key=lambda g: (g.goal_info.stamp.sec, g.goal_info.stamp.nanosec),
            )
            now = time.monotonic()
            st = self.nav[action]
            name = STATE_NAMES.get(s.status, str(s.status))
            if name != st["state"]:
                st["state"], st["since"] = name, now
                if name == "aborted":
                    st["last_abort"] = now

        return cb

    def _battery(self, msg):
        pct = msg.percentage * 100 if msg.percentage <= 1.0 else msg.percentage
        self.battery = (time.monotonic(), round(pct, 1), round(msg.voltage, 2))

    def _goal(self, msg):
        p = msg.pose.position
        self.goal = (
            time.monotonic(),
            round(p.x, 2),
            round(p.y, 2),
            msg.header.frame_id,
        )

    def _rosout(self, msg):
        if msg.level < 30 or msg.name == "go2_debug_probe":
            return
        level = LEVELS.get(msg.level, str(msg.level))
        self.rosout.append(
            {
                "t": round(time.time(), 1),
                "level": level,
                "node": msg.name,
                "msg": msg.msg[:400],
            }
        )
        c = self.rosout_counts.setdefault(msg.name, {"WARN": 0, "ERROR": 0, "FATAL": 0})
        if level in c:
            c[level] += 1

    def _list_nodes(self):
        try:
            self.nodes = sorted(
                f"{ns.rstrip('/')}/{n}" if ns not in ("/", "") else n
                for n, ns in self.get_node_names_and_namespaces()
            )
        except Exception:  # noqa: BLE001 - graph queries can fail while DDS settles
            pass

    def _sample_pose(self):
        try:
            t = self.tf.lookup_transform(MAP, BASE, Time())
        except Exception:  # noqa: BLE001 - no TF yet
            return
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        self.tf_age = time.time() - stamp
        tr = t.transform.translation
        self.poses.append((time.monotonic(), tr.x, tr.y, yaw_of(t.transform.rotation)))

    # ------------------------------------------------------------ output
    def _speed(self):
        if len(self.poses) < 2:
            return None
        now = self.poses[-1][0]
        old = next((p for p in self.poses if now - p[0] <= 1.5), self.poses[0])
        new = self.poses[-1]
        dt = new[0] - old[0]
        if dt < 0.3:
            return None
        lin = math.hypot(new[1] - old[1], new[2] - old[2]) / dt
        dyaw = math.atan2(math.sin(new[3] - old[3]), math.cos(new[3] - old[3]))
        return {"lin": round(lin, 3), "ang": round(dyaw / dt, 3)}

    def _patch_max(self, grid, info, pose, box):
        x, y, yaw = pose
        x0, x1, y0, y1 = box
        res = info.resolution
        xs, ys = np.meshgrid(np.arange(x0, x1, res / 2), np.arange(y0, y1, res / 2))
        c, s = math.cos(yaw), math.sin(yaw)
        wx, wy = x + c * xs - s * ys, y + s * xs + c * ys
        j = ((wx - info.origin.position.x) / res).astype(int)
        i = ((wy - info.origin.position.y) / res).astype(int)
        ok = (i >= 0) & (j >= 0) & (i < info.height) & (j < info.width)
        if not ok.any():
            return None
        cells = grid[i[ok], j[ok]]
        return {
            "max": int(cells.max()),
            "lethal": int((cells >= 100).sum()),
            "inscribed": int((cells == 99).sum()),
            "unknown": int((cells < 0).sum()),
        }

    def _costmap_info(self, pose, with_grid):
        m = self.costmap
        if m is None:
            return None
        info = m.info
        grid = np.array(m.data, dtype=np.int16).reshape(info.height, info.width)
        out = {
            "frame": m.header.frame_id,
            "res": info.resolution,
            "w": info.width,
            "h": info.height,
            "ox": info.origin.position.x,
            "oy": info.origin.position.y,
        }
        if pose is not None and m.header.frame_id == MAP:
            out["under"] = self._patch_max(grid, info, pose, FOOTPRINT)
            out["ahead"] = self._patch_max(grid, info, pose, AHEAD)
        if with_grid:
            out["grid"] = base64.b64encode(
                np.where(grid < 0, 255, grid).astype(np.uint8).tobytes()
            ).decode("ascii")
        return out

    def _emit(self):
        now = time.monotonic()
        topics = {}
        for t, q in self.stamps.items():
            recent = [x for x in q if now - x <= 5.0]
            topics[t] = {
                "hz": round(len(recent) / 5.0, 2),
                "age": round(now - q[-1], 1) if q else None,
            }
        pose = (
            self.poses[-1][1:] if self.poses and now - self.poses[-1][0] < 2 else None
        )
        self.n_emit += 1
        state = {
            "t": round(time.time(), 1),
            "uptime": round(now - self.t0, 1),
            "topics": topics,
            "pose": None
            if pose is None
            else {
                "x": round(pose[0], 2),
                "y": round(pose[1], 2),
                "yaw": round(pose[2], 3),
                "tf_age": None if self.tf_age is None else round(self.tf_age, 2),
                "speed": self._speed(),
            },
            "cmd": {
                k: {
                    "age": round(now - v[0], 1),
                    "vx": round(v[1], 3),
                    "vy": round(v[2], 3),
                    "wz": round(v[3], 3),
                }
                for k, v in self.cmd.items()
            },
            "nav": {
                a: {
                    "state": s["state"],
                    "for": None if s["since"] is None else round(now - s["since"], 1),
                    "abort_ago": None
                    if s["last_abort"] is None
                    else round(now - s["last_abort"], 1),
                }
                for a, s in self.nav.items()
            },
            "costmap": self._costmap_info(pose, self.n_emit % 2 == 0),
            "plan": self.plan if self.n_emit % 2 == 0 else None,
            "status": {
                k: {"age": round(now - v[0], 1), "data": v[1][:300]}
                for k, v in self.status.items()
            },
            "battery": None
            if self.battery is None
            else {
                "age": round(now - self.battery[0], 1),
                "pct": self.battery[1],
                "v": self.battery[2],
            },
            "goal": None
            if self.goal is None
            else {
                "age": round(now - self.goal[0], 1),
                "x": self.goal[1],
                "y": self.goal[2],
                "frame": self.goal[3],
            },
            "rosout": list(self.rosout)[-60:],
            "rosout_counts": self.rosout_counts,
            "nodes": self.nodes,
        }
        sys.stdout.write(json.dumps(state, separators=(",", ":")) + "\n")
        sys.stdout.flush()


def main():
    rclpy.init()
    node = Probe()
    # The page server holds our stdin: when it goes away, so do we.
    threading.Thread(
        target=lambda: (sys.stdin.read(), rclpy.try_shutdown()), daemon=True
    ).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
