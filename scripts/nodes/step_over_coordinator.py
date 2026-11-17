#!/usr/bin/env python3
"""
step_over_coordinator.py: the dog steps over low objects on its path (wifi_3d_nav.sh
--semantic --step-over-crawl MODE). Runs with the traversability nodes in go2-lidar3d.

The Go2's sport mode cannot raise its feet (FootRaiseHeight 3203), so stepping over is
done by scripts/robot/step_over_crawl.py --step-over on the dog: a slow crawl in low-level
mode, one foot at a time. This node decides WHEN and over WHAT, and hands it off:

  1. pick a candidate from /semantic/labels_json: an object on the dog's path (ahead,
     within lateral_max of the heading), whose LiDAR-measured height is between h_min
     (lower: the normal step clears it) and h_max, whose depth along the heading is at
     most depth_max (the crawl's longest step), and that is not AVOID or protected;
  2. pause Nav2 (cancel /follow_path and /compute_path_to_pose), remembering the goal;
  3. line the dog up: face the object, front feet gap_target before its near edge
     (slow /cmd_vel; gives up after align_timeout_s);
  4. hand off: write REQUEST_DIR/request_<id>.json; the laptop-side
     scripts/tools/step_over_executor.py runs the crawl on the dog over SSH and writes
     result_<id>.json; the crawl lies down, hands sport mode back and stands the dog
     up in ClassicWalk;
  5. resume: publish the remembered goal on /goal_pose again.

MODE (parameter mode):
  dry    only log "would step over ..." (default; safe in any session)
  align  steps 1-3 and 5: pause, line up, resume. No crawl.
  crawl  all of it. The crawl falls back to lying down on any safety stop; then this
         node does NOT resume Nav2 and says why.

A candidate is handed off once; the same track is not picked again for cooldown_s.
"""

import json
import math
import os
import time

# --- Pure geometry (tests/test_step_over_coordinator.py) -------------------------------


def depth_along(size_x, size_y, yaw, heading):
    """Extent (m) of an oriented footprint along the heading; None if not measured."""
    if not size_x or not size_y or size_x <= 0 or size_y <= 0:
        return None
    d = (yaw or 0.0) - heading
    return abs(size_x * math.cos(d)) + abs(size_y * math.sin(d))


def relative(obj, pose):
    """(ahead, lateral) of the object's centre from the robot (x, y, heading)."""
    dx, dy = obj["x"] - pose[0], obj["y"] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return dx * c + dy * s, -dx * s + dy * c


def candidate(objects, pose, p):
    """The nearest object the crawl should step over, or None; with why each other
    object ahead was skipped (for the log). p: dict of the node's parameters."""
    best, skipped = None, []
    for o in objects:
        if o.get("height") is None or o.get("x") is None:
            continue
        ahead, lateral = relative(o, pose)
        depth = depth_along(o.get("size_x"), o.get("size_y"), o.get("yaw"), pose[2])
        near_edge = ahead - (depth or 0.0) / 2
        if (
            not (p["ahead_min"] <= near_edge <= p["ahead_max"])
            or abs(lateral) > p["lateral_max"]
        ):
            continue  # not on the path ahead: ignore quietly
        name = f"{o.get('class', '?')} #{o.get('track_id')}"
        why = None
        if o.get("label") == "AVOID" or o.get("protected"):
            why = "AVOID/protected: go around"
        elif o["height"] < p["h_min"]:
            why = f"{o['height'] * 100:.0f} cm: the normal step clears it"
        elif o["height"] > p["h_max"]:
            why = f"{o['height'] * 100:.0f} cm: taller than the crawl clears ({p['h_max'] * 100:.0f} cm)"
        elif depth is None:
            why = "footprint not measured"
        elif depth > p["depth_max"]:
            why = f"{depth * 100:.0f} cm deep: the crawl's step is too short ({p['depth_max'] * 100:.0f} cm)"
        if why:
            skipped.append((name, why))
            continue
        if best is None or near_edge < best["near_edge"]:
            best = {
                "obj": o,
                "name": name,
                "ahead": ahead,
                "lateral": lateral,
                "depth": depth,
                "near_edge": near_edge,
                "height": o["height"],
            }
    return best, skipped


def align_command(cand, pose, p):
    """(vx, wz, done) to line up: face the object's centre, front feet gap_target before
    its near edge. cand carries the object; pose is the robot now."""
    o = cand["obj"]
    ahead, lateral = relative(o, pose)
    yaw_err = math.atan2(lateral, max(ahead, 1e-3))
    gap = ahead - cand["depth"] / 2 - p["front_foot_x"]
    err = gap - p["gap_target"]
    if abs(yaw_err) > p["yaw_tol"]:
        return 0.0, max(-p["wz_max"], min(p["wz_max"], 1.2 * yaw_err)), False
    if abs(err) <= p["gap_tol"]:
        return 0.0, 0.0, True
    vx = max(-p["vx_max"], min(p["vx_max"], 0.8 * err))
    return vx, max(-p["wz_max"], min(p["wz_max"], 1.0 * yaw_err)), False


def is_clear(cand, pose, rear_foot_x=0.19, margin=0.03):
    """True when the object's far edge is behind the rear feet (the dog is fully past it),
    from the robot's pose now; and the far edge's distance ahead of the body centre (m)."""
    ahead, _lateral = relative(cand["obj"], pose)
    far_edge = ahead + cand["depth"] / 2
    return far_edge < -(rear_foot_x + margin), far_edge


def request_payload(req_id, cand, pose, gap):
    """The JSON the laptop-side executor reads."""
    return {
        "id": req_id,
        "t": time.time(),
        "object": cand["name"],
        "height": round(cand["height"], 3),
        "depth": round(cand["depth"], 3),
        "gap": round(gap, 3),
        "pose": [round(v, 3) for v in pose],
    }


# --- The node ---------------------------------------------------------------------------


def main():
    import rclpy
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import PoseStamped, Twist
    from rclpy.node import Node
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    class Coordinator(Node):
        def __init__(self):
            super().__init__("step_over_coordinator")
            d = self.declare_parameter
            d("mode", "dry")
            d("map_frame", "lidar3d_map")
            d("base_frame", "base_link")
            d("labels_topic", "/semantic/labels_json")
            d("request_dir", "/ws/log/3d/step_over")
            d("h_min", 0.03)  # the dog's normal step clears this (TRAV_STEP_OVER)
            d("h_max", 0.10)  # the crawl's lift is object + 5 cm; tested plans to 0.15
            d("depth_max", 0.10)  # one crossing step reaches about 0.12 m
            d("ahead_min", 0.35)
            d("ahead_max", 1.30)
            d("lateral_max", 0.20)
            d("front_foot_x", 0.19)  # base_link to the front feet, m
            d(
                "gap_target", 0.12
            )  # front feet to the object's near edge (the crawl's plan)
            d("gap_tol", 0.03)
            d("yaw_tol", 0.08)
            d("vx_max", 0.10)
            d("wz_max", 0.35)
            d("align_timeout_s", 25.0)
            d("crawl_timeout_s", 600.0)
            d("cooldown_s", 90.0)
            d("moving_vx", 0.05)  # only while Nav2 drives forward at least this fast
            g = lambda k: self.get_parameter(k).value  # noqa: E731
            self.p = {
                k: g(k)
                for k in (
                    "h_min",
                    "h_max",
                    "depth_max",
                    "ahead_min",
                    "ahead_max",
                    "lateral_max",
                    "front_foot_x",
                    "gap_target",
                    "gap_tol",
                    "yaw_tol",
                    "vx_max",
                    "wz_max",
                )
            }
            self.mode = g("mode")
            if self.mode not in ("dry", "align", "crawl"):
                raise ValueError(f"mode must be dry, align or crawl, not {self.mode}")
            self.map_frame, self.base_frame = g("map_frame"), g("base_frame")
            self.req_dir = g("request_dir")
            os.makedirs(self.req_dir, exist_ok=True)
            self.align_timeout, self.crawl_timeout = (
                g("align_timeout_s"),
                g("crawl_timeout_s"),
            )
            self.cooldown, self.moving_vx = g("cooldown_s"), g("moving_vx")

            self.objects, self.goal, self.last_vx, self.last_vx_t = [], None, 0.0, 0.0
            self.state, self.cand, self.t_state, self.req_id = "idle", None, 0.0, None
            self.done = {}  # track id -> time handed off (or logged, in dry)
            self.tf = Buffer()
            TransformListener(self.tf, self)
            self.create_subscription(String, g("labels_topic"), self.on_labels, 10)
            self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
            self.create_subscription(Twist, "/cmd_vel_nav", self.on_nav_vel, 10)
            self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
            self.cancel = {
                a: self.create_client(CancelGoal, f"{a}/_action/cancel_goal")
                for a in ("/follow_path", "/compute_path_to_pose")
            }
            self.create_timer(0.1, self.tick)
            self.get_logger().info(
                f"step_over_coordinator mode={self.mode}: objects {self.p['h_min'] * 100:.0f}-"
                f"{self.p['h_max'] * 100:.0f} cm high, at most {self.p['depth_max'] * 100:.0f} cm deep, "
                f"{self.p['ahead_min']:.2f}-{self.p['ahead_max']:.2f} m ahead; requests in {self.req_dir}"
            )

        # inputs
        def on_labels(self, msg):
            try:
                self.objects = json.loads(msg.data).get("objects", [])
            except ValueError:
                pass

        def on_goal(self, msg):
            if self.state == "idle":
                self.goal = msg
            elif self.state == "failed":
                # A new goal from the operator after a failed crawl: back to normal.
                self.goal = msg
                self.get_logger().info(
                    "new goal after the failed crawl: watching again"
                )
                self.enter("idle")

        def on_nav_vel(self, msg):
            self.last_vx, self.last_vx_t = msg.linear.x, time.monotonic()

        def pose(self):
            try:
                t = self.tf.lookup_transform(
                    self.map_frame, self.base_frame, Time()
                ).transform
            except Exception:  # noqa: BLE001 - no TF yet: nothing to do
                return None
            q = t.rotation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            return (t.translation.x, t.translation.y, yaw)

        # actions
        def stop(self):
            self.vel_pub.publish(Twist())

        def pause_nav(self):
            for name, cli in self.cancel.items():
                if cli.service_is_ready():
                    cli.call_async(CancelGoal.Request())
                else:
                    self.get_logger().warn(f"cancel: {name} not available")
            self.stop()

        def resume_nav(self, why):
            if self.goal is None:
                self.get_logger().info(f"{why}; no goal to resume")
                return
            self.goal.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(self.goal)
            self.get_logger().info(
                f"{why}; goal resumed ({self.goal.pose.position.x:.2f}, "
                f"{self.goal.pose.position.y:.2f})"
            )

        def enter(self, state):
            self.state, self.t_state = state, time.monotonic()

        def tick(self):
            pose = self.pose()
            if pose is None:
                return
            now = time.monotonic()
            if self.state == "idle":
                moving = now - self.last_vx_t < 1.0 and self.last_vx > self.moving_vx
                if not moving:
                    return
                cand, _skipped = candidate(self.objects, pose, self.p)
                if cand is None:
                    return
                tid = cand["obj"].get("track_id")
                if now - self.done.get(tid, -1e9) < self.cooldown:
                    return
                self.done[tid] = now
                self.get_logger().info(
                    f"{'would step' if self.mode == 'dry' else 'stepping'} over {cand['name']}: "
                    f"{cand['height'] * 100:.0f} cm high, {cand['depth'] * 100:.0f} cm deep, near edge "
                    f"{cand['near_edge']:.2f} m ahead, {cand['lateral']:+.2f} m to the side"
                )
                if self.mode == "dry":
                    return
                self.cand = cand
                self.pause_nav()
                self.enter("align")
            elif self.state == "align":
                vx, wz, done = align_command(self.cand, pose, self.p)
                if done:
                    self.stop()
                    if self.mode == "align":
                        self.get_logger().info(
                            f"lined up in front of {self.cand['name']} (align mode: no crawl)"
                        )
                        self.enter("resume")
                        return
                    gap = (
                        relative(self.cand["obj"], pose)[0]
                        - self.cand["depth"] / 2
                        - self.p["front_foot_x"]
                    )
                    self.req_id = f"{int(time.time())}"
                    path = os.path.join(self.req_dir, f"request_{self.req_id}.json")
                    with open(path + ".tmp", "w") as f:
                        json.dump(request_payload(self.req_id, self.cand, pose, gap), f)
                    os.replace(path + ".tmp", path)
                    self.get_logger().info(f"lined up; crawl requested ({path})")
                    self.enter("crawl")
                elif now - self.t_state > self.align_timeout:
                    self.stop()
                    self.get_logger().warn(
                        f"could not line up with {self.cand['name']} in "
                        f"{self.align_timeout:.0f} s: carrying on without stepping over"
                    )
                    self.enter("resume")
                else:
                    t = Twist()
                    t.linear.x, t.angular.z = vx, wz
                    self.vel_pub.publish(t)
            elif self.state == "crawl":
                path = os.path.join(self.req_dir, f"result_{self.req_id}.json")
                if os.path.exists(path):
                    try:
                        res = json.load(open(path))
                    except ValueError:
                        return
                    if res.get("ok"):
                        clear, far = is_clear(self.cand, pose)
                        if clear:
                            self.get_logger().info(
                                f"stepped over {self.cand['name']}; its far edge is {-far:.2f} m "
                                f"behind the body centre ({res.get('summary', '')})"
                            )
                            self.enter("resume")
                        else:
                            self.get_logger().error(
                                f"PROBLEM: the crawl finished but {self.cand['name']} is not behind "
                                f"the rear feet (far edge {far:+.2f} m from the body centre). "
                                "Nav2 is NOT resumed."
                            )
                            self.enter("failed")
                    else:
                        self.get_logger().error(
                            f"PROBLEM: the crawl over {self.cand['name']} did not finish "
                            f"({res.get('summary', 'no summary')}). Nav2 is NOT resumed: check the dog, "
                            "stand it with the remote, then send a new goal."
                        )
                        self.enter("failed")
                elif now - self.t_state > self.crawl_timeout:
                    self.get_logger().error(
                        "PROBLEM: no crawl result in time; Nav2 is NOT resumed"
                    )
                    self.enter("failed")
            elif self.state == "resume":
                if now - self.t_state > 2.0:
                    self.resume_nav("done")
                    self.cand = None
                    self.enter("idle")
            # "failed": stay put (Nav2 paused) until a new goal arrives (on_goal).

    rclpy.init()
    node = Coordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
