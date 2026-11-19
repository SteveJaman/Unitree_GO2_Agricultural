#!/usr/bin/env python3
"""
servo_demo_trials.py: the arm distance-servo demo in the Gazebo NFT 4-6 world, run as a
set of logged trials (inside go2-sim, started by run_sim_demo.sh).

  python3 servo_demo_trials.py --out /tmp/servo_demo --trials 5 --policies proportional fixed_step open_loop

Once: stow the arm, Nav2 to the rack and park tail-first (full_mission_test.py's
navigation), add the rack to MoveIt's planning scene.
Then each trial, in a shuffled order of (policy, start offset):
  1. arm to the start pose (J2 0.25, J3 0.30, J5 -0.55), MoveIt joint goal;
  2. dog driven on Gazebo ground truth to the parking spot moved by the trial's offset:
     alternately too far (measured ~0.57-0.62 m: the arm approaches) and too close
     (~0.42-0.445 m: it backs off);
  3. distance_servo.py started with that policy (stop_when_settled, run timeout),
     YOLO distances from yolo_depth_tracker.py (running in go2-lidar3d);
  4. while it runs, at 10 Hz: the ground truth (camera pose from the dog's Gazebo pose,
     the arm joints and the arm's kinematics; the true depth at the lettuce's centre
     pixel from the wrist depth image, i.e. what the tracker should measure); and the
     wrist RGB-D before and after every arm move (an "interaction", as in Zhang & Gupta,
     "Push Past Green", CoRL 2023), with the visible lettuce area from colour (green
     leaf hue) and depth, the counterpart of their colour-threshold labels.

Per trial in OUT/trial_NN_<policy>/: servo_log.csv, servo_steps.csv (from the servo),
ground_truth.csv, frames.csv + frames/*.jpg|*.npz (wrist RGB, depth), trial.json.
OUT/trials.json lists them; scripts/tools/servo_trials_report.py makes the figures.
"""
import argparse
import csv
import importlib.util
import json
import math
import os
import random
import subprocess
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("full_mission_test", os.path.join(HERE, "full_mission_test.py"))
fm = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fm)

from geometry_msgs.msg import Point, Pose, Quaternion, Twist  # noqa: E402
from moveit_msgs.msg import CollisionObject, PlanningScene  # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene  # noqa: E402
from shape_msgs.msg import SolidPrimitive  # noqa: E402

START = {"Joint1": 0.0, "Joint2": 0.25, "Joint3": 0.30, "Joint4": 0.0, "Joint5": -0.55, "Joint6": 0.0}
W, H, HFOV = 640, 480, 1.21          # the d435i wrist camera (d1_go2.urdf.xacro)
FX = W / 2 / math.tan(HFOV / 2)


def log(msg):
    print(f"[trials] {msg}", flush=True)


class Wrist:
    """Latest wrist colour and depth from Gazebo transport (same frames the tracker gets via ROS)."""

    def __init__(self, node):
        from gz.msgs10.image_pb2 import Image
        self.lock = threading.Lock()
        self.color = self.depth = None
        node.subscribe(Image, "/wrist_camera/image", self._color)
        node.subscribe(Image, "/wrist_camera/depth_image", self._depth)

    def _color(self, m):
        a = np.frombuffer(m.data, np.uint8)
        if a.size == m.width * m.height * 3:
            with self.lock:
                self.color = (time.time(), a.reshape(m.height, m.width, 3).copy())

    def _depth(self, m):
        a = np.frombuffer(m.data, np.float32)
        if a.size == m.width * m.height:
            d = np.nan_to_num(a.reshape(m.height, m.width).copy(), nan=0.0, posinf=0.0, neginf=0.0)
            with self.lock:
                self.depth = (time.time(), d)

    def latest(self):
        with self.lock:
            return self.color, self.depth


def plant_mask(rgb, depth, max_depth=1.2):
    """Leaf pixels: green hue (excludes the frame's blue-green and the white channels), and
    within max_depth (so plants far behind do not count)."""
    f = rgb.astype(np.float32) / 255.0
    mx, mn = f.max(axis=2), f.min(axis=2)
    r, g, b = f[..., 0], f[..., 1], f[..., 2]
    sat = (mx - mn) / np.maximum(mx, 1e-6)
    hue = np.zeros_like(mx)
    m = (mx == g) & (mx > mn)
    hue[m] = 60.0 * (2.0 + (b[m] - r[m]) / (mx[m] - mn[m]))           # hue of green-dominant pixels, degrees
    green = m & (hue > 70) & (hue < 140) & (sat > 0.3) & (mx > 0.12)
    if depth is not None:
        green &= (depth > 0.05) & (depth < max_depth)
    return green


class Trials:
    def __init__(self, a):
        self.a = a
        ma = argparse.Namespace(world_json=a.world_json, urdf=a.urdf, out=a.out, nav_domain=a.nav_domain,
                                arm_domain=a.arm_domain, skip_nav=a.skip_nav, standoff=None, nav_timeout=a.nav_timeout,
                                stage=0.45, pregrasp=0.3, radius=0.15, orbit_deg=0, look_up_deg=0, vel_scale=a.vel_scale,
                                screen_cameras=[])
        self.m = fm.Mission(ma)
        self.world = self.m.world
        self.wrist = Wrist(self.m.gz.node)
        self.plant = np.array(self.world["target_plant"]["center"], float)

    # ---------------------------------------------------------- frames
    def to_arm(self, p, dog):
        """World point -> MoveIt's world frame: ground under the dog, x along the arm, which
        faces the dog's tail (SIM_D1_MOUNT yaw pi; the MoveIt model faces forward)."""
        yaw = dog[3] + math.pi
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = p[0] - dog[0], p[1] - dog[1]
        return np.array([c * dx + s * dy, -s * dx + c * dy, p[2]])

    def add_scene(self, dog):
        objs = []
        for o in self.world["planning_scene"]:
            co = CollisionObject()
            co.header.frame_id = "world"
            co.id = o["name"]
            co.operation = CollisionObject.ADD
            p = self.to_arm(o["pose"][:3], dog)
            yaw = -(dog[3] + math.pi)
            q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
            kind = SolidPrimitive.BOX if o["type"] == "box" else SolidPrimitive.SPHERE
            co.primitives.append(SolidPrimitive(type=kind, dimensions=list(o["size"])))
            co.primitive_poses.append(Pose(position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                                           orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))
            objs.append(co)
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene(is_diff=True)
        req.scene.world.collision_objects = objs
        res = self.m.wait(self.m.scene.call_async(req), 30)
        return bool(res and res.success)

    def ground_truth(self):
        """Camera pose and the lettuce in the camera frame, from Gazebo's dog pose + joints + FK."""
        dog, q = self.m.gz.dog, self.m.current_q()
        if dog is None or q is None:
            return None
        p_cam, R = self.m.kin.pose(q)                        # camera_link in the arm frame: x out of the lens
        plant = self.to_arm(self.plant, dog)
        rel = R.T @ (plant - p_cam)                          # lettuce centre in camera_link (x fwd, y left, z up)
        u = W / 2 - FX * rel[1] / rel[0] if rel[0] > 0.05 else float("nan")
        v = H / 2 - FX * rel[2] / rel[0] if rel[0] > 0.05 else float("nan")
        return {"dog": dog, "q": q, "cam": p_cam, "axial": float(rel[0]), "range": float(np.linalg.norm(rel)),
                "u": u, "v": v, "tool_x": float(p_cam[0])}

    def true_depth(self, gt, depth, radius=0.13, height=0.11, pct=25.0):
        """The true distance to the front of the target lettuce: the pct-th percentile of the
        rendered depth (exact in Gazebo) over the lettuce's projected outline (a head of
        radius x height around its centre), keeping only depths within the lettuce itself
        (centre depth -radius-0.05 .. +radius), so the background and the plants behind are
        out. Same statistic as the tracker's depth_mode:=box, but on the geometry, not on a
        YOLO box: comparing them measures the detector + depth chain."""
        if depth is None or not (math.isfinite(gt["u"]) and math.isfinite(gt["v"])) or gt["axial"] <= 0.1:
            return float("nan")
        r_px = FX * radius / gt["axial"]
        h_px = FX * height / 2 / gt["axial"]
        u0, u1 = int(max(0, gt["u"] - r_px)), int(min(W, gt["u"] + r_px + 1))
        v0, v1 = int(max(0, gt["v"] - h_px)), int(min(H, gt["v"] + h_px + 1))
        if u1 <= u0 or v1 <= v0:
            return float("nan")
        patch = depth[v0:v1, u0:u1]
        lo, hi = gt["axial"] - radius - 0.05, gt["axial"] + radius
        good = patch[np.isfinite(patch) & (patch > lo) & (patch < hi)]
        return float(np.percentile(good, pct)) if good.size >= 5 else float("nan")

    # ---------------------------------------------------------- motion
    def drive_to(self, x, y, yaw, timeout=60):
        def axis(err, tol, gain, lo, hi):
            return 0.0 if abs(err) < tol else math.copysign(min(hi, max(lo, gain * abs(err))), err)
        t1 = time.monotonic()
        while time.monotonic() - t1 < timeout:
            d = self.m.gz.dog
            c, s = math.cos(d[3]), math.sin(d[3])
            ex, ey = c * (x - d[0]) + s * (y - d[1]), -s * (x - d[0]) + c * (y - d[1])
            eyaw = math.atan2(math.sin(yaw - d[3]), math.cos(yaw - d[3]))
            if abs(ex) < 0.006 and abs(ey) < 0.008 and abs(eyaw) < 0.012:
                break
            cmd = Twist()
            cmd.linear.x = axis(ex, 0.006, 0.8, 0.05, 0.12)
            cmd.linear.y = axis(ey, 0.008, 0.8, 0.05, 0.10)
            cmd.angular.z = axis(eyaw, 0.012, 1.5, 0.08, 0.3)
            self.m.cmd_pub.publish(cmd)
            time.sleep(0.05)
        for _ in range(10):
            self.m.cmd_pub.publish(Twist())
            time.sleep(0.05)
        time.sleep(1.5)
        d = self.m.gz.dog
        return math.hypot(d[0] - x, d[1] - y), math.degrees(math.atan2(math.sin(yaw - d[3]), math.cos(yaw - d[3])))

    def arm_moving(self):
        with self.m.lock:
            if not self.m.samples:
                return False
            v = self.m.samples[-1][3]
        return max(abs(x) for x in v) > 0.01

    # ---------------------------------------------------------- one trial
    def save_frame(self, tdir, n, tag, gt, writer):
        col, dep = self.wrist.latest()
        if col is None:
            return                                   # no wrist frame yet
        ct, rgb = col
        depth = dep[1] if dep else None
        from PIL import Image as PILImage
        name = f"f{n:03d}_{tag}"
        PILImage.fromarray(rgb).save(os.path.join(tdir, "frames", name + ".jpg"), quality=90)
        if depth is not None:
            np.savez_compressed(os.path.join(tdir, "frames", name + ".npz"), depth=depth.astype(np.float16))
        mask = plant_mask(rgb, depth)
        target_px = 0
        if gt and math.isfinite(gt["u"]):
            r = int(max(10, FX * 0.12 / max(gt["axial"], 0.1)))   # ~ the head's radius in pixels
            u, v = int(gt["u"]), int(gt["v"])
            target_px = int(mask[max(0, v - r):v + r, max(0, u - r):u + r].sum())
        writer.writerow({"frame": name, "wall_s": f"{ct:.3f}", "tag": tag, "plant_px": int(mask.sum()),
                         "plant_frac": f"{mask.mean():.4f}", "target_px": target_px,
                         "gt_axial_m": f"{gt['axial']:.4f}" if gt else "", "gt_depth_m": f"{self.true_depth(gt, depth):.4f}" if gt else ""})

    def run_trial(self, idx, policy, offset, wp, disturb=0.0):
        a = self.a
        tdir = os.path.join(a.out, f"trial_{idx:02d}_{policy}")
        os.makedirs(os.path.join(tdir, "frames"), exist_ok=True)
        log(f"trial {idx}: {policy}, start offset {offset * 100:+.1f} cm")
        ok, why, _ = self.m.move_to(self.m.joint_goal(START))
        if not ok:
            log(f"  start pose failed: {why}")
        park_err, yaw_off = self.drive_to(wp[0] - offset, wp[1], wp[2])
        gt0 = None
        for _ in range(50):                          # joints and the dog's pose, after the drive
            gt0 = self.ground_truth()
            if gt0 is not None:
                break
            time.sleep(0.1)
        if gt0 is None:
            raise RuntimeError("no ground truth (joint states or Gazebo pose) after parking")
        log(f"  parked {park_err * 1000:.0f} mm / {yaw_off:+.1f} deg off; lettuce {gt0['axial']:.3f} m ahead of the camera "
            f"(pixel {gt0['u']:.0f}, {gt0['v']:.0f})")

        env = dict(os.environ, ROS_DOMAIN_ID=str(a.arm_domain))
        cmd = ["python3", "-u", a.servo, "--ros-args",
               "-p", f"csv_path:={tdir}/servo_log.csv", "-p", f"steps_csv:={tdir}/servo_steps.csv",
               "-p", f"policy:={policy}", "-p", "stop_when_settled:=true", "-p", f"settle_s:={a.settle_s}",
               "-p", f"run_timeout_s:={a.trial_timeout}", "-p", f"velocity_scaling:={a.vel_scale}",
               "-p", f"measure_after_move_s:={a.measure_after}", "-p", f"median_of:={a.median_of}"]
        if disturb:
            cmd += ["-p", f"min_run_s:={a.disturb_at + 6.0}"]
        t_start = time.time()
        servo_log = open(os.path.join(tdir, "servo.log"), "w")
        servo = subprocess.Popen(cmd, env=env, stdout=servo_log, stderr=subprocess.STDOUT)
        gt_f = open(os.path.join(tdir, "ground_truth.csv"), "w", newline="")
        gtw = csv.writer(gt_f)
        gtw.writerow(["wall_s", "t_s", "gt_axial_m", "gt_range_m", "gt_depth_m", "camera_x_arm_m", "u_px", "v_px",
                      "dog_x", "dog_y", "dog_yaw", "arm_moving"] + fm.ARM_JOINTS)
        fr_f = open(os.path.join(tdir, "frames.csv"), "w", newline="")
        frw = csv.DictWriter(fr_f, fieldnames=["frame", "wall_s", "tag", "plant_px", "plant_frac", "target_px", "gt_axial_m", "gt_depth_m"])
        frw.writeheader()
        n, moving_prev, stop_since = 0, False, None
        disturber = None
        if disturb:
            # the dog slides along its axis: + = away from the rack (the plant gets further)
            def slide():
                time.sleep(a.disturb_at)
                d = self.m.gz.dog
                log(f"  disturbance: the dog slides {disturb * 100:+.1f} cm")
                self.drive_to(d[0] - disturb, d[1], wp[2])
            disturber = threading.Thread(target=slide, daemon=True)
            disturber.start()
        self.save_frame(tdir, n, "start", gt0, frw)
        n += 1
        while servo.poll() is None:
            time.sleep(0.1)
            gt = self.ground_truth()
            if gt is None:
                continue
            _, dep = self.wrist.latest()
            moving = self.arm_moving()
            gtw.writerow([f"{time.time():.3f}", f"{time.time() - t_start:.3f}", f"{gt['axial']:.4f}", f"{gt['range']:.4f}",
                          f"{self.true_depth(gt, dep[1] if dep else None):.4f}", f"{gt['tool_x']:.4f}",
                          f"{gt['u']:.1f}", f"{gt['v']:.1f}", f"{gt['dog'][0]:.4f}", f"{gt['dog'][1]:.4f}",
                          f"{gt['dog'][3]:.4f}", int(moving)] + [f"{gt['q'][j]:.4f}" for j in fm.ARM_JOINTS])
            if moving and not moving_prev:                      # interaction starts: the "before" view
                self.save_frame(tdir, n, "before", gt, frw)
                n += 1
                stop_since = None
            if not moving and moving_prev:
                stop_since = time.time()
            if stop_since and time.time() - stop_since > 0.4:   # settled after the move: the "after" view
                self.save_frame(tdir, n, "after", gt, frw)
                n += 1
                stop_since = None
            moving_prev = moving
            if time.time() - t_start > a.trial_timeout + 30:
                servo.terminate()
        servo_log.close()
        gt1 = self.ground_truth()
        self.save_frame(tdir, n, "end", gt1, frw)
        gt_f.close()
        fr_f.close()
        if disturber is not None:
            disturber.join(timeout=30)
        info = {"trial": idx, "policy": policy, "offset_m": offset, "disturb_m": disturb, "servo_exit": servo.returncode,
                "settled": servo.returncode == 0, "duration_s": time.time() - t_start,
                "park_error_m": park_err, "park_yaw_off_deg": yaw_off,
                "start_gt_axial_m": gt0["axial"], "end_gt_axial_m": gt1["axial"] if gt1 else None,
                "target_distance_m": 0.50, "dir": os.path.basename(tdir)}
        json.dump(info, open(os.path.join(tdir, "trial.json"), "w"), indent=1)
        log(f"  {'settled' if info['settled'] else 'NOT settled (exit ' + str(servo.returncode) + ')'} in "
            f"{info['duration_s']:.1f} s; lettuce "
            + (f"{info['end_gt_axial_m']:.3f} m" if info["end_gt_axial_m"] is not None else "? m") + " ahead of the camera at the end")
        return info

    # ---------------------------------------------------------- the whole run
    def run(self):
        a, m = self.a, self.m
        log("waiting for MoveIt, the joint states and Gazebo")
        for c in (m.move, m.exec):
            c.wait_for_server(timeout_sec=120)
        m.scene.wait_for_service(timeout_sec=60)
        t0 = time.monotonic()
        while (m.current_q() is None or m.dog_pose() is None) and time.monotonic() - t0 < 60:
            time.sleep(0.2)
        ok, why, _ = m.move_to(m.joint_goal(fm.STOW))
        log(f"arm {'stowed' if ok else 'NOT stowed: ' + why}")
        if a.skip_nav:
            wp = tuple(self.world["waypoint"])
            self.drive_to(*wp)
        else:
            m.phase = "Navigation"
            ok, info, _ = m.navigate()
            log(f"navigation {'ok' if ok else 'NOT ok'}: {info['notes']}")
        wp = tuple(self.world["waypoint"])
        log(f"planning scene: {'added' if self.add_scene(m.gz.dog) else 'NOT added'}")

        rng = random.Random(a.seed)
        # half the trials start too far (the arm must approach), half too close (it must back off);
        # the tracker reads the front leaves, ~0.09 m nearer than the lettuce centre the world's
        # START_DIST (0.62 m) is measured to, so offset 0 starts at ~0.52 m measured
        plan = []
        for p in a.policies:
            for k in range(a.trials):
                lo, hi = a.far if k % 2 == 0 else a.near
                # with --disturb: half the slides away from the rack, half towards it
                dist = (a.disturb if (k // 2) % 2 == 0 else -a.disturb) if a.disturb else 0.0
                plan.append((p, rng.uniform(lo, hi), dist))
        rng.shuffle(plan)                                     # interleave the policies (as in the paper's protocol)
        results = []
        for i, (policy, off, dist) in enumerate(plan):
            try:
                results.append(self.run_trial(i, policy, off, wp, dist))
            except Exception as e:  # noqa: BLE001  (one bad trial must not lose the others)
                log(f"trial {i} failed: {e!r}")
            json.dump({"world": self.world["world"], "target_plant": self.world["target_plant"], "args": vars(a),
                       "trials": results}, open(os.path.join(a.out, "trials.json"), "w"), indent=1)
        m.move_to(m.joint_goal(START))
        log(f"done: {len(results)} trials in {a.out}")
        return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world-json", default="/sim/worlds/cropking_nft46.json")
    ap.add_argument("--urdf", default="/tmp/sim-gen/d1_moveit.urdf")
    ap.add_argument("--out", default="/tmp/servo_demo")
    ap.add_argument("--servo", default="/tmp/servo/distance_servo.py")
    ap.add_argument("--nav-domain", type=int, default=64)
    ap.add_argument("--arm-domain", type=int, default=66)
    ap.add_argument("--skip-nav", action="store_true", help="drive straight to the parking spot on ground truth")
    ap.add_argument("--nav-timeout", type=float, default=300.0)
    ap.add_argument("--trials", type=int, default=5, help="per policy")
    ap.add_argument("--policies", nargs="+", default=["proportional", "fixed_step", "open_loop"])
    ap.add_argument("--far", type=float, nargs=2, default=(0.05, 0.10), metavar=("MIN", "MAX"),
                    help="parking moved back by MIN-MAX m (measured start ~0.57-0.62 m: the arm approaches)")
    ap.add_argument("--near", type=float, nargs=2, default=(-0.10, -0.075), metavar=("MIN", "MAX"),
                    help="parking moved in by MIN-MAX m (measured start ~0.42-0.445 m: the arm backs off)")
    ap.add_argument("--settle-s", type=float, default=3.0)
    ap.add_argument("--trial-timeout", type=float, default=60.0)
    ap.add_argument("--vel-scale", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--measure-after", type=float, default=0.6,
                    help="servo: distances count from this long after a move (the effort-controlled arm rings)")
    ap.add_argument("--median-of", type=int, default=5, help="servo: step on the median of this many distances")
    ap.add_argument("--disturb", type=float, default=0.0,
                    help="m: the dog slides this far (+/- alternately) disturb-at s into each trial (0 = none)")
    ap.add_argument("--disturb-at", type=float, default=4.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    t = Trials(a)
    code = 1
    try:
        code = t.run()
    finally:
        for ex in t.m.executors:
            ex.shutdown()
    return code


if __name__ == "__main__":
    sys.stdout.flush()
    os._exit(main())
