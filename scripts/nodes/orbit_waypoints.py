#!/usr/bin/env python3
"""
orbit_waypoints.py: three-shell camera orbit around a plant with the D1 + claw D435i
(Gazebo, inside go2-sim; run_orbit_sweep.sh starts it).

  python3 orbit_waypoints.py --mode ik-frontier --d 0.45 --r 0.30 --out DIR
  python3 orbit_waypoints.py --mode orbit --d 0.45 --r 0.30 --shells all --out DIR --run-id R

Frame (MoveIt's world): the dog at the origin, the arm along +x, the plant centroid at
P = (d, 0, h_p), z up. A camera pose of a waypoint (azimuth az, elevation el, radius r):
    C = P + r (-cos el cos az, cos el sin az, sin el),  the camera's x axis looking at P,
its z axis as near to up as the look-at allows; az = 0 is between the dog and the plant.
Shells (elevations relative to the plant centroid): under -20 deg, eq 0 deg, top +30 deg.
    eq:    az -90 -60 -30 0 30 60 90        (the 180 deg arc)
    top:   az -60 -30 0 30 60   (el +30)    (includes the "top pose at az 0" of the eq set)
    under: az -60 -30 0 30 60   (el -20)
fixed order: eq left to right, then top, then under (no reordering: deferred).

ik-frontier: no motion. With the planning scene for a dog parked at d, MoveIt's /compute_ik
(damped least-squares IK continued from the previous azimuth, collision-checked; see
ik_waypoint) in 5 deg steps from 0 to +-120 per shell until the first failure, then in 1 deg
steps back from there: the limit on each side is the last feasible azimuth, to 1 deg.
Writes frontier.json.

orbit: the arm stows, the dog drives (Gazebo ground truth) to the parking spot for d, the
scene is set, the arm stays folded until the first waypoint, then per waypoint: IK seeded from the previous waypoint's joints; RRTConnect
plan to that joint goal (2.0 s); on failure the reason (IK_FAIL: no IK even without
collisions; COLLISION: IK only without collision checking; TIMEOUT: no plan) is logged
and the next waypoint follows. Execute; settle (max |joint velocity| < 0.01 rad/s for
200 ms, 10 s cap); hold 1 s; capture 5 RGB-D wrist frames with the joint torques and
velocities; time plan, execution and settling; integrate sum_i |tau_i omega_i| dt over
the move and settling (exec_energy_j) and E_hold = sum_i |tau_i| t_hold over the hold
(hold_energy_j, N m s: the gravity-holding cost of E3). Then back to --home. Writes
waypoints.csv and frames/ (jpg, depth npz in m float16, meta json with the camera pose).

Watchdog: if the Gazebo clock does not advance for 10 s, or one waypoint takes over 180 s,
the node exits with code 42 (ABORTED_SIM_HANG); run_orbit_sweep.sh restarts the simulation.
"""

import argparse
import csv
import importlib.util
import json
import math
import os
import sys
import threading
import time

import numpy as np

SCRIPTS = os.environ.get("MISSION_DIR", "/moveit_src/d1_moveit_config/scripts")
_spec = importlib.util.spec_from_file_location(
    "servo_demo_trials", os.path.join(SCRIPTS, "servo_demo_trials.py")
)
sdt = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sdt)
fm = sdt.fm

from moveit_msgs.action import ExecuteTrajectory, MoveGroup  # noqa: E402
from moveit_msgs.msg import (  # noqa: E402
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,  # noqa: E402
    PositionIKRequest,
    RobotState,
)
from moveit_msgs.srv import GetPositionIK, GetStateValidity  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

REACH = 0.605
SHELLS = {
    "eq": (0.0, [-90, -60, -30, 0, 30, 60, 90]),
    "top": (30.0, [-60, -30, 0, 30, 60]),
    "under": (-20.0, [-60, -30, 0, 30, 60]),
}
ORDER = ("eq", "top", "under")
ROLLS = (0.0, 30.0, -30.0)
SUCCESS = 1


def log(msg):
    print(f"[orbit] {msg}", flush=True)


def analytic_limit(d, r, reach=REACH):
    """Largest azimuth (deg) with R(theta) = sqrt((d - r cos)^2 + (r sin)^2) <= reach (180: all)."""
    c = (d * d + r * r - reach * reach) / (2 * d * r)
    if c <= -1:
        return 180.0
    if c >= 1:
        return 0.0 if math.hypot(d - r, 0) > reach else 180.0
    return math.degrees(math.acos(c))


def camera_pose(d, r, h_p, az_deg, el_deg, roll_deg=0.0):
    az, el = math.radians(az_deg), math.radians(el_deg)
    P = np.array([d, 0.0, h_p])
    C = P + r * np.array(
        [-math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]
    )
    q = fm.look_at(C, P)
    if roll_deg:
        hr = math.radians(roll_deg) / 2  # q * (roll about the camera's own x axis)
        q = (
            tuple(fm.quat_mul(q, (math.sin(hr), 0.0, 0.0, math.cos(hr))))
            if hasattr(fm, "quat_mul")
            else _qmul(q, (math.sin(hr), 0.0, 0.0, math.cos(hr)))
        )
    return C, q


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rot_error(R_target, R_now):
    """Axis-angle rotation vector taking R_now to R_target (world frame), and its angle. A
    sum of axis cross products, the usual shortcut, is zero for every 180 deg rotation as
    well, which let the IK converge on a camera turned the wrong way."""
    Re = R_target @ R_now.T
    c = max(-1.0, min(1.0, (np.trace(Re) - 1.0) / 2.0))
    w = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    s = np.linalg.norm(w)
    ang = math.atan2(s, c)
    if s > 1e-9:
        return w / s * ang, ang
    if c > 0:
        return np.zeros(3), 0.0
    k = int(np.argmax(np.diag(Re)))  # 180 deg: the axis from the diagonal
    axis = np.sqrt(np.clip((np.diag(Re) + 1.0) / 2.0, 0.0, None))
    axis[k] = max(axis[k], 1e-9)
    for i in range(3):
        if i != k:
            axis[i] = math.copysign(axis[i], Re[k, i])
    return axis / np.linalg.norm(axis) * math.pi, math.pi


class Orbit(sdt.Trials):
    def __init__(self, a):
        ta = argparse.Namespace(
            world_json=a.world_json,
            urdf=a.urdf,
            out=a.out,
            nav_domain=64,
            arm_domain=66,
            skip_nav=True,
            nav_timeout=60,
            vel_scale=a.vel_scale,
        )
        super().__init__(ta)
        self.a = a
        node = self.m.arm
        self.ik = node.create_client(GetPositionIK, "/compute_ik")
        self.valid = node.create_client(GetStateValidity, "/check_state_validity")
        lim = self.m.kin.limits(a.urdf)
        self.lo = np.array([lim[j][0] for j in fm.ARM_JOINTS]) * 0.98
        self.hi = np.array([lim[j][1] for j in fm.ARM_JOINTS]) * 0.98
        rng = np.random.default_rng(5)
        self.seeds = [np.array([sdt.START[j] for j in fm.ARM_JOINTS]), np.zeros(6)] + [
            rng.uniform(self.lo, self.hi) for _ in range(10)
        ]
        self.js_lock = threading.Lock()
        self.js_buf = []  # (t, q[6], v[6], tau[6])
        node.create_subscription(JointState, "/joint_states", self._js, 100)
        self.last_clock = (time.monotonic(), None)
        from gz.msgs10.clock_pb2 import Clock

        self.m.gz.node.subscribe(
            Clock, f"/world/{self.world['world']}/clock", self._clock
        )
        self.h_p = float(self.world["target_plant"]["center"][2])
        self.plant_xy = self.world["target_plant"]["center"][:2]

    # ------------------------------------------------------------ inputs
    def _js(self, msg):
        if not all(j in msg.name for j in fm.ARM_JOINTS):
            return
        idx = [msg.name.index(j) for j in fm.ARM_JOINTS]
        v = [msg.velocity[i] if msg.velocity else 0.0 for i in idx]
        tau = [msg.effort[i] if len(msg.effort) > max(idx) else 0.0 for i in idx]
        with self.js_lock:
            self.js_buf.append(
                (time.monotonic(), [msg.position[i] for i in idx], v, tau)
            )
            del self.js_buf[:-4000]

    def _clock(self, msg):
        sim = msg.sim.sec + msg.sim.nsec * 1e-9
        if self.last_clock[1] is None or sim > self.last_clock[1]:
            self.last_clock = (time.monotonic(), sim)

    def check_clock(self):
        if time.monotonic() - getattr(self, "wp_t0", time.monotonic()) > 180.0:
            log("ABORTED_SIM_HANG: one waypoint took over 180 s")
            sys.stdout.flush()
            os._exit(42)
        if time.monotonic() - self.last_clock[0] > 10.0:
            log("ABORTED_SIM_HANG: the Gazebo clock has not advanced for 10 s")
            sys.stdout.flush()
            os._exit(42)

    def js_window(self, t0, t1):
        with self.js_lock:
            return [s for s in self.js_buf if t0 <= s[0] <= t1]

    @staticmethod
    def energy(samples):
        """sum_i integral |tau_i omega_i| dt over the samples (J)."""
        e = 0.0
        for (ta, _, va, fa), (tb, _, vb, fb) in zip(samples, samples[1:]):
            dt = tb - ta
            e += dt * sum(abs(f * v) for f, v in zip(fa, va))
        return e

    # ------------------------------------------------------------ geometry
    def virtual_dog(self, d):
        """The dog pose (x, y, z, yaw) that puts the plant d ahead of the arm (arm faces the tail)."""
        return (self.plant_xy[0] - d, self.plant_xy[1], 0.29, math.pi)

    def base_clear(self, d):
        """Does the parked dog's body clear the rack (rear legs 0.37 m behind the centre)?"""
        tub_front = self.world["rack"]["center"][0] + 0.05 - 0.475
        return self.virtual_dog(d)[0] + 0.37 < tub_front - 0.01

    def ik_solve(self, pose, seed, avoid=True, timeout=0.05):
        req = GetPositionIK.Request()
        r = PositionIKRequest()
        r.group_name = "arm"
        r.ik_link_name = fm.EE_LINK
        r.avoid_collisions = avoid
        r.timeout.sec, r.timeout.nanosec = 0, int(timeout * 1e9)
        js = JointState(
            name=list(fm.ARM_JOINTS), position=[float(seed[j]) for j in fm.ARM_JOINTS]
        )
        r.robot_state = RobotState(joint_state=js, is_diff=False)
        ps = PoseStamped()
        ps.header.frame_id = "world"
        (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z) = (
            float(v) for v in pose[0]
        )
        (
            ps.pose.orientation.x,
            ps.pose.orientation.y,
            ps.pose.orientation.z,
            ps.pose.orientation.w,
        ) = (float(v) for v in pose[1])
        r.pose_stamped = ps
        req.ik_request = r
        res = self.m.wait(self.ik.call_async(req), 5.0)
        if res is None or res.error_code.val != SUCCESS:
            return None
        sol = dict(
            zip(res.solution.joint_state.name, res.solution.joint_state.position)
        )
        return (
            {j: sol[j] for j in fm.ARM_JOINTS}
            if all(j in sol for j in fm.ARM_JOINTS)
            else None
        )

    def dls(self, C, R, seeds, iters=80):
        """Damped least-squares IK (camera_link pose, full orientation) from each seed within
        the joint limits; every converged solution."""
        kin = self.m.kin
        out = []

        def err(q):
            p, Rq = kin.pose(dict(zip(fm.ARM_JOINTS, q)))
            dr = rot_error(R, Rq)[0]
            return np.concatenate([C - p, 0.3 * dr])

        for s0 in seeds:
            q = np.clip(np.array(s0, float), self.lo, self.hi)
            for _ in range(iters):
                e = err(q)
                if np.linalg.norm(e[:3]) < 5e-4 and np.linalg.norm(e[3:]) < 3e-3:
                    break
                Jm = np.empty((6, 6))
                for k in range(6):
                    dq = np.zeros(6)
                    dq[k] = 1e-5
                    Jm[:, k] = (err(q + dq) - e) / -1e-5
                q = np.clip(
                    q + Jm.T @ np.linalg.solve(Jm @ Jm.T + 1e-4 * np.eye(6), e),
                    self.lo,
                    self.hi,
                )
            p_now, R_now = kin.pose(dict(zip(fm.ARM_JOINTS, q)))
            if np.linalg.norm(C - p_now) < 1e-3 and rot_error(R, R_now)[1] < 0.02:
                out.append(q)
        return out

    def state_valid(self, q):
        req = GetStateValidity.Request()
        req.robot_state = RobotState(
            joint_state=JointState(
                name=list(fm.ARM_JOINTS), position=[float(x) for x in q]
            )
        )
        req.group_name = "arm"
        res = self.m.wait(self.valid.call_async(req), 5.0)
        return bool(res and res.valid)

    def ik_waypoint(self, d, r, az, el, seed):
        """(joints, roll, reason) for a waypoint: damped least-squares IK continued from the
        seed (the previous waypoint), then from 12 seeds x 3 camera rolls (0, +-30 deg); the solutions
        nearest the seed are checked for collisions with MoveIt; KDL (/compute_ik) last.
        IK_FAIL: no solution within the joint limits; COLLISION: solutions, all colliding."""
        sv = np.array([seed[j] for j in fm.ARM_JOINTS])

        def pose_R(roll):
            C, q = camera_pose(d, r, self.h_p, az, el, roll)
            x, y, z, w = q
            R = np.array(
                [
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
                ]
            )
            return C, R, roll

        tries = [(pose_R(0.0), [sv])] + [
            (pose_R(rl), [sv] + self.seeds) for rl in ROLLS
        ]
        any_sol = False
        for (C, R, roll), seeds in tries:
            sols = self.dls(C, R, seeds)
            if not sols:
                continue
            any_sol = True
            for q in sorted(sols, key=lambda q: np.abs(q - sv).sum()):
                if self.state_valid(q):
                    return dict(zip(fm.ARM_JOINTS, q.tolist())), roll, ""
        for roll in ROLLS:
            q = self.ik_solve(
                camera_pose(d, r, self.h_p, az, el, roll), seed, timeout=0.2
            )
            if q:
                return q, roll, ""
        return None, None, "COLLISION" if any_sol else "IK_FAIL"

    def clear_scene(self):
        """Remove the rack and plant objects a previous run left in MoveIt's planning scene (a
        stale scene made the arm's current state collide, and every plan, even stowing, fail)."""
        from moveit_msgs.msg import CollisionObject, PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene

        objs = []
        for o in self.world["planning_scene"]:
            co = CollisionObject()
            co.header.frame_id = "world"
            co.id = o["name"]
            co.operation = CollisionObject.REMOVE
            objs.append(co)
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene(is_diff=True)
        req.scene.world.collision_objects = objs
        self.m.wait(self.m.scene.call_async(req), 30)

    # ------------------------------------------------------------ ik frontier
    def frontier(self):
        a = self.a
        self.clear_scene()
        self.add_scene(self.virtual_dog(a.d))
        seed0 = dict(sdt.START)
        out = {
            "d": a.d,
            "r": a.r,
            "h_p": self.h_p,
            "analytic_limit_deg": analytic_limit(a.d, a.r),
            "R90": math.hypot(a.d, a.r),
            "base_clear": self.base_clear(a.d),
            "shells": {},
        }
        for shell in ORDER:
            el = SHELLS[shell][0]
            sides = {}
            for sign in (1, -1):
                # coarse 5 deg steps from 0 to the first failure, then 1 deg steps in between:
                # the limit to 1 deg (a gap narrower than 5 deg before it could be missed)
                seed, last_ok, reason, fail_at = seed0, None, "", None
                for k in range(0, 121, 5):
                    self.check_clock()
                    q, _, why = self.ik_waypoint(a.d, a.r, sign * k, el, seed)
                    if q is None:
                        reason, fail_at = why, k
                        break
                    seed, last_ok = q, k
                if fail_at is not None and last_ok is not None:
                    for k in range(last_ok + 1, fail_at):
                        self.check_clock()
                        q, _, why = self.ik_waypoint(a.d, a.r, sign * k, el, seed)
                        if q is None:
                            reason = why
                            break
                        seed, last_ok = q, k
                sides["left" if sign > 0 else "right"] = {
                    "limit_deg": last_ok,
                    "stop_reason": reason or "SWEEP_END",
                }
            lim = [s["limit_deg"] for s in sides.values()]
            out["shells"][shell] = {
                **sides,
                "limit_deg": None if None in lim else min(lim),
                "elevation_deg": el,
            }
            log(
                f"d={a.d} r={a.r} {shell}: limit {out['shells'][shell]['limit_deg']} deg "
                f"(left {sides['left']['limit_deg']}, {sides['left']['stop_reason']}; "
                f"right {sides['right']['limit_deg']}, {sides['right']['stop_reason']})"
            )
        os.makedirs(a.out, exist_ok=True)
        json.dump(out, open(os.path.join(a.out, "frontier.json"), "w"), indent=1)
        return 0

    # ------------------------------------------------------------ orbit
    def plan_exec(self, q):
        """(plan_ok, plan_time, exec_time, reason) for a joint goal, RRTConnect, 2.0 s."""
        r = MotionPlanRequest()
        r.group_name = "arm"
        r.planner_id = (
            "RRTConnect"  # OMPL's RRTConnect (this config names no planner set)
        )
        r.num_planning_attempts = 1
        r.allowed_planning_time = 2.0
        r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = (
            self.a.vel_scale
        )
        r.start_state.is_diff = True
        r.goal_constraints = [
            Constraints(
                joint_constraints=[
                    JointConstraint(
                        joint_name=j,
                        position=float(q[j]),
                        tolerance_above=0.01,
                        tolerance_below=0.01,
                        weight=1.0,
                    )
                    for j in fm.ARM_JOINTS
                ]
            )
        ]
        t0 = time.monotonic()
        h = self.m.wait(
            self.m.move.send_goal_async(
                MoveGroup.Goal(
                    request=r, planning_options=PlanningOptions(plan_only=True)
                )
            ),
            10,
        )
        res = (
            self.m.wait(h.get_result_async(), 15)
            if h is not None and h.accepted
            else None
        )
        plan_time = time.monotonic() - t0
        if res is None or res.result.error_code.val != SUCCESS:
            return False, plan_time, 0.0, "TIMEOUT"
        t1 = time.monotonic()
        he = self.m.wait(
            self.m.exec.send_goal_async(
                ExecuteTrajectory.Goal(trajectory=res.result.planned_trajectory)
            ),
            10,
        )
        er = (
            self.m.wait(he.get_result_async(), 60)
            if he is not None and he.accepted
            else None
        )
        code = er.result.error_code.val if er is not None else "none"
        ok = code == SUCCESS
        return ok, plan_time, time.monotonic() - t1, "" if ok else f"EXEC_FAIL({code})"

    def settle(self, cap=10.0):
        t0 = time.monotonic()
        still = None
        while time.monotonic() - t0 < cap:
            self.check_clock()
            time.sleep(0.02)
            with self.js_lock:
                last = self.js_buf[-1] if self.js_buf else None
            if last is None:
                continue
            if max(abs(v) for v in last[2]) < 0.01:
                still = still or time.monotonic()
                if time.monotonic() - still >= 0.2:
                    break
            else:
                still = None
        return time.monotonic() - t0

    def capture(self, fdir, tag, n=5):
        """n distinct wrist RGB-D frames, each with the camera pose (world and arm frame) and joints."""
        from PIL import Image as PILImage

        got, last_t, metas = 0, None, []
        t0 = time.monotonic()
        while got < n and time.monotonic() - t0 < 10:
            time.sleep(0.03)
            col, dep = self.wrist.latest()
            if col is None or dep is None or col[0] == last_t:
                continue
            last_t = col[0]
            gt = self.ground_truth()
            if gt is None:
                continue
            name = f"{tag}_f{got}"
            PILImage.fromarray(col[1]).save(
                os.path.join(fdir, name + ".jpg"), quality=92
            )
            np.savez_compressed(
                os.path.join(fdir, name + ".npz"), depth=dep[1].astype(np.float16)
            )
            p_cam, R = self.m.kin.pose(gt["q"])
            dog = gt["dog"]
            yaw = dog[3] + math.pi  # arm frame -> world (the arm faces the tail)
            c, s = math.cos(yaw), math.sin(yaw)
            Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            world_p = np.array([dog[0], dog[1], 0.0]) + Rz @ p_cam
            meta = {
                "frame": name,
                "wall_s": col[0],
                "camera_arm": p_cam.tolist(),
                "R_arm": R.tolist(),
                "camera_world": world_p.tolist(),
                "R_world": (Rz @ R).tolist(),
                "fx": sdt.FX,
                "w": sdt.W,
                "h": sdt.H,
                "joints": [gt["q"][j] for j in fm.ARM_JOINTS],
            }
            json.dump(meta, open(os.path.join(fdir, name + ".json"), "w"))
            metas.append(meta)
            got += 1
        return metas

    def orbit(self):
        a = self.a
        rid = a.run_id or f"d{a.d:.2f}_r{a.r:.2f}_{int(time.time())}"
        out = os.path.join(a.out, rid)
        fdir = os.path.join(out, "frames")
        os.makedirs(fdir, exist_ok=True)
        m = self.m
        for c in (m.move, m.exec):
            c.wait_for_server(timeout_sec=120)
        self.ik.wait_for_service(timeout_sec=60)
        t0 = time.monotonic()
        while (
            m.current_q() is None or m.dog_pose() is None
        ) and time.monotonic() - t0 < 60:
            time.sleep(0.2)
        home = dict(zip(fm.ARM_JOINTS, a.home)) if a.home else dict(fm.STOW)
        self.clear_scene()
        ok, why, _ = m.move_to(m.joint_goal(fm.STOW))
        if not ok:
            log(f"{rid}: stow failed ({why}); trying again")
            m.move_to(m.joint_goal(fm.STOW))
        vd = self.virtual_dog(a.d)
        park_err, yaw_off = self.drive_to(vd[0], vd[1], vd[3])
        log(
            f"{rid}: parked {park_err * 1000:.0f} mm / {yaw_off:+.1f} deg off (d={a.d}, r={a.r})"
        )
        self.add_scene(m.gz.dog)
        shells = ORDER if a.shells == "all" else (a.shells,)
        # the arm stays folded (stow) while parked close to the plant: the servo's start pose would
        # put the camera inside the plant at small d; the first waypoint is planned from stow
        rows, seed, k = [], dict(sdt.START), 0
        for shell in shells:
            el, azs = SHELLS[shell]
            for az in azs:
                self.check_clock()
                self.wp_t0 = time.monotonic()
                row = {
                    "run_id": rid,
                    "d": a.d,
                    "r": a.r,
                    "shell": shell,
                    "waypoint_k": k,
                    "azimuth_deg": az,
                    "elevation_deg": el,
                    "ik_success": 0,
                    "plan_success": 0,
                    "fail_reason": "",
                    "plan_time_s": "",
                    "exec_time_s": "",
                    "settle_time_s": "",
                    "hold_energy_j": "",
                    "exec_energy_j": "",
                    "camera_x": "",
                    "camera_y": "",
                    "camera_z": "",
                    "dist_to_plant_m": "",
                    "park_error_m": f"{park_err:.4f}",
                }
                q, roll, why = self.ik_waypoint(a.d, a.r, az, el, seed)
                if q is None:
                    row["fail_reason"] = why
                else:
                    row["ik_success"] = 1
                    ts = time.monotonic()
                    ok, tp, te, why = self.plan_exec(q)
                    row["plan_time_s"] = f"{tp:.3f}"
                    if not ok:
                        row["fail_reason"] = why
                        if why.startswith("EXEC_FAIL"):
                            row["plan_success"] = 1
                    else:
                        row["plan_success"] = 1
                        tsettle = self.settle()
                        te_end = time.monotonic()
                        th = time.monotonic()
                        time.sleep(1.0)  # hold
                        hold = self.js_window(th, time.monotonic())
                        tau_mean = (
                            np.mean([[abs(x) for x in s[3]] for s in hold], axis=0)
                            if hold
                            else np.zeros(6)
                        )
                        metas = self.capture(fdir, f"k{k:02d}_{shell}_az{az:+04d}")
                        row.update(
                            exec_time_s=f"{te:.3f}",
                            settle_time_s=f"{tsettle:.3f}",
                            exec_energy_j=f"{self.energy(self.js_window(ts, te_end)):.4f}",
                            hold_energy_j=f"{float(np.sum(tau_mean)) * 1.0:.4f}",
                        )
                        if metas:
                            cw = np.mean([mm["camera_world"] for mm in metas], axis=0)
                            ca = np.mean([mm["camera_arm"] for mm in metas], axis=0)
                            row.update(
                                camera_x=f"{ca[0]:.4f}",
                                camera_y=f"{ca[1]:.4f}",
                                camera_z=f"{ca[2]:.4f}",
                                dist_to_plant_m=f"{np.linalg.norm(cw - np.array(self.world['target_plant']['center'])):.4f}",
                            )
                        seed = m.current_q() or q
                rows.append(row)
                log(
                    f"{rid} k={k} {shell} az={az:+d} el={el:+.0f}: "
                    + (
                        "ok"
                        if row["plan_success"] and not row["fail_reason"]
                        else row["fail_reason"]
                    )
                )
                k += 1
        m.move_to(m.joint_goal(home))
        with open(os.path.join(out, "waypoints.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        json.dump(
            {
                "run_id": rid,
                "d": a.d,
                "r": a.r,
                "h_p": self.h_p,
                "plant": self.world["target_plant"],
                "plant_center_world": self.world["target_plant"]["center"],
                "dog_parked": list(m.gz.dog[:4]),
                "analytic_limit_deg": analytic_limit(a.d, a.r),
                "base_clear": self.base_clear(a.d),
            },
            open(os.path.join(out, "run.json"), "w"),
            indent=1,
        )
        log(
            f"{rid}: {sum(1 for r_ in rows if r_['plan_success'] and not r_['fail_reason'])}/{len(rows)} waypoints done -> {out}"
        )
        return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--mode", choices=("ik-frontier", "orbit"), default="orbit")
    ap.add_argument(
        "--d", type=float, required=True, help="plant distance from the J1 axis (m)"
    )
    ap.add_argument(
        "--r", type=float, required=True, help="orbit radius around the plant (m)"
    )
    ap.add_argument("--shells", choices=("under", "eq", "top", "all"), default="all")
    ap.add_argument("--out", required=True)
    ap.add_argument("--run-id", default="")
    ap.add_argument(
        "--home",
        type=float,
        nargs=6,
        default=None,
        help="joint pose to retract to (rad); default stow",
    )
    ap.add_argument("--world-json", default="/sim/worlds/cropking_nft46.json")
    ap.add_argument("--urdf", default="/tmp/sim-gen/d1_moveit.urdf")
    ap.add_argument("--vel-scale", type=float, default=0.3)
    a = ap.parse_args()
    o = Orbit(a)
    code = 1
    try:
        code = o.frontier() if a.mode == "ik-frontier" else o.orbit()
    finally:
        for ex in o.m.executors:
            ex.shutdown()
    return code


if __name__ == "__main__":
    sys.stdout.flush()
    os._exit(main())
