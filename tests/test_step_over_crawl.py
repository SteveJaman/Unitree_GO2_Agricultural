"""Unit tests for scripts/robot/step_over_crawl.py --plan (no robot, no SDK)."""
import argparse
import math
import os
import random
import sys

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, "..", "scripts", "robot"))
sys.path.insert(0, os.path.join(HERE, "..", "gazebo", "sim"))
import leg_gait  # noqa: E402
import step_over_crawl as sc  # noqa: E402


def args(**kw):
    d = dict(object_height=0.08, object_depth=0.10, object_distance=0.12, stand_height=0.30,
             clearance=0.05, normal_lift=0.06, stride=0.12, max_stride=0.26, landing_margin=0.04,
             stability_margin=0.025, order="RL,FL,RR,FR", max_cycles=8, t_shift=1.5, t_lift=0.8,
             t_move=1.0, t_lower=0.8, step_over=False, stop_after=0, t_settle=0.0, stance_out=0.0,
             lean_abort=6.0, ff=0.8, mass=17.0, com_x=0.0, com_y=0.0, start_temp=55, max_temp=68,
             recentre_every=4, leg_over=None, resume_session=False)
    d.update(kw)
    return argparse.Namespace(**d)


def test_ik_fk_round_trip():
    rng = random.Random(1)
    for _ in range(300):
        leg = rng.choice(sc.LEGS)
        p = (rng.uniform(-0.15, 0.15), sc.side(leg) * sc.L_ABAD + rng.uniform(-0.05, 0.05),
             rng.uniform(-0.36, -0.15))
        q = sc.leg_ik(leg, *p)
        assert all(abs(a - b) < 1e-6 for a, b in zip(sc.leg_fk(leg, *q), p))


def test_fk_matches_the_sim_leg_model():
    # The sim's leg_gait.py (validated against the Gazebo Go2) and this script agree.
    for leg in sc.LEGS:
        for q in ((0.1, 0.7, -1.4), (-0.2, 0.9, -1.7), (0.0, 0.5, -1.2)):
            a, b = sc.leg_fk(leg, *q), leg_gait.leg_fk(leg, *q)
            assert all(abs(x - y) < 1e-9 for x, y in zip(a, b))


def test_stand_pose_is_close_to_unitrees():
    # go2_stand_example.py stands at thigh 0.67, calf -1.3 (about 0.34 m); 0.30 m bends more.
    q = sc.stand_joints(0.34)
    for i in range(4):
        assert abs(q[3 * i]) < 0.02
        assert abs(q[3 * i + 1] - 0.67) < 0.06 and abs(q[3 * i + 2] + 1.30) < 0.08
    for i, leg in enumerate(sc.LEGS):
        assert not sc.limit_violations(leg, sc.stand_joints(0.30)[3 * i: 3 * i + 3])


def test_out_of_reach_raises():
    try:
        sc.leg_ik("FR", 0.0, -sc.L_ABAD, -0.5)
    except ValueError:
        return
    raise AssertionError("0.5 m below the hip should be out of reach")


def test_triangle_margin_and_minimal_shift():
    tri = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    assert abs(sc.triangle_margin((0.25, 0.25), tri) - 0.25) < 1e-9
    assert sc.triangle_margin((1.0, 1.0), tri) < 0
    p = sc.shift_target((-0.2, 0.3), tri, 0.05)
    assert abs(sc.triangle_margin(p, tri) - 0.05) < 1e-3       # just enough, not the centroid


def test_default_plan_crosses_with_every_leg_and_no_problems():
    plan = sc.Plan(args()).build()
    crossings = [k for k in plan.keys if "over the object" in k["name"]]
    assert sorted(k["name"].split()[0] for k in crossings) == ["FL", "FR", "RL", "RR"]
    for i, k in enumerate(plan.keys):
        q, problems = plan.joints(k)
        assert not problems, (i, k["name"], problems)
        if k["swing"] is not None:
            assert k["margin"] >= 0.025
    assert not plan.uncrossed()
    # The foot over the object is clearance above it, and no foot lands on it.
    for i, k in enumerate(plan.keys):
        for leg, (x, _, z) in k["feet"].items():
            if z == 0.0:
                assert not (plan.obj_near - sc.FOOT_R < x < plan.obj_far + sc.FOOT_R), (i, leg, x)
    lifted = [k for k in plan.keys if "up 13 cm" in k["name"]]
    assert len(lifted) == 4


def test_heights_up_to_15_cm_plan_cleanly():
    for h in (0.03, 0.05, 0.10, 0.12, 0.15):
        plan = sc.Plan(args(object_height=h)).build()
        assert not plan.uncrossed()
        assert not any(plan.joints(k)[1] for k in plan.keys), h


def test_too_deep_object_is_reported():
    plan = sc.Plan(args(object_depth=0.20)).build()
    assert len(plan.uncrossed()) == 4
    assert "too deep" in plan.uncrossed()[0]


def test_lift_plan_every_leg_is_clean_and_returns_to_the_stand():
    a = args(lift_height=0.10, lift_margin=0.05, lift_shift_s=3.0, lift_up_s=2.0, hold=5.0)
    stand = sc.stand_joints(0.30)
    for leg in sc.LEGS:
        keys = sc.lift_plan(a, leg)
        assert [k[0].split()[0] for k in keys][1:4] == [leg, leg, leg]
        for name, _t, q, margin, problems in keys:
            assert not problems, (leg, name, problems)
            if "up" in name or "held" in name:
                assert margin >= 0.05 - 1e-3
        assert all(abs(x - y) < 1e-9 for x, y in zip(keys[-1][2], stand))
        # Only the lifted leg's joints change between "body over" and "up".
        i = sc.LEGS.index(leg)
        before, up = keys[0][2], keys[1][2]
        changed = [j // 3 for j in range(12) if abs(before[j] - up[j]) > 1e-9]
        assert set(changed) == {i}


class FakeLowLevel(sc.LowLevel):
    """LowLevel without the SDK: the joints follow the commands, the IMU reads flat."""

    DT = 0.001   # short steps: the test durations are a few ms

    def __init__(self, iface):
        import types
        self.state_t = 0.0
        self.kp, self.phase, self.log, self.log_n = self.KP, "", None, 0
        self.ki, self.i_corr, self.I_MAX, self.i_fade, self.q_ref = 0.0, [0.0] * 12, 0.35, 0.0, None
        self.ff_target, self.ff_gain, self.ff_height, self.ff_mass, self.ff_com = 0.0, 0.0, 0.30, 17.0, (0.0, 0.0)
        self.tau, self.lean_ref, self.lean_limit, self.max_temp, self.n_checks = [0.0] * 12, None, 0.0, 68, 0
        self.taus = []
        self.mode_name, self.sent, self.calls = "", [], []
        q = list(sc.LIE_DOWN)
        self.state = types.SimpleNamespace(
            motor_state=[types.SimpleNamespace(q=v, temperature=35) for v in q],
            imu_state=types.SimpleNamespace(rpy=[0.0, 0.0, 0.0]))
        self.sport = types.SimpleNamespace(StandUp=lambda: None, BalanceStand=lambda: None)

    def _fresh(self):
        import time as _t
        self.state_t = _t.monotonic()

    def check(self, max_tilt):
        self._fresh()
        super().check(max_tilt)

    def release_sport(self):
        self.mode_name = "mcf"
        self.calls.append("release")

    def hand_back(self):
        self.calls.append("hand_back")

    def send(self, q, kp, kd):
        tau = self.feed_forward() if kp > 0 else [0.0] * 12
        self.taus.append(list(tau))
        for i in range(12):
            self.state.motor_state[i].q = q[i]
        self.sent.append((list(q), kp))
        self._log(q)


def test_lift_hardware_path_on_a_fake_robot(tmp_path, monkeypatch):
    monkeypatch.setattr(sc, "LowLevel", FakeLowLevel)
    made = {}
    orig = FakeLowLevel.__init__

    def init(self, iface):
        orig(self, iface)
        made["ll"] = self
    monkeypatch.setattr(FakeLowLevel, "__init__", init)
    log = tmp_path / "lift.csv"
    a = args(lift="FR", lift_height=0.10, lift_margin=0.075, lift_shift_s=0.02, lift_up_s=0.02,
             hold=0.02, ramp=0.02, max_tilt=15.0, kp_lift=80.0, ki=1.0, iface="eth0", yes=True,
             restand=False, log=str(log), dry=False)
    sc.run_stand(a)
    ll = made["ll"]
    assert ll.calls == ["release", "hand_back"]
    kps = {kp for _q, kp in ll.sent}
    assert 80.0 in kps and 60.0 in kps
    # FR reached its lifted pose, and the run ended back at the lying pose.
    fr_up = sc.lift_plan(a, "FR")[1][2]
    # (commands carry the small integral correction, so compare within 0.02 rad)
    assert any(all(abs(x - y) < 0.02 for x, y in zip(q, fr_up)) for q, _ in ll.sent)
    assert all(abs(x - y) < 1e-6 for x, y in zip(ll.sent[-1][0], sc.LIE_DOWN)) or ll.sent[-1][1] == 0.0
    rows = log.read_text().splitlines()
    assert rows[0].startswith("t,phase,kp,ki,roll_deg,pitch_deg") and len(rows) > 2


def test_tilt_stop_goes_soft_and_still_hands_back(monkeypatch):
    class Tipping(FakeLowLevel):
        def send(self, q, kp, kd):
            super().send(q, kp, kd)
            if self.phase.startswith("FR up"):
                self.state.imu_state.rpy[0] = math.radians(20)   # leans past the 15 deg stop
    made = {}

    def init(self, iface):
        FakeLowLevel.__init__(self, iface)
        made["ll"] = self
    monkeypatch.setattr(Tipping, "__init__", init)
    monkeypatch.setattr(sc, "LowLevel", Tipping)
    a = args(lift="FR", lift_height=0.10, lift_margin=0.075, lift_shift_s=0.02, lift_up_s=0.02,
             hold=0.02, ramp=0.02, max_tilt=15.0, kp_lift=80.0, ki=1.0, iface="eth0", yes=True,
             restand=False, log="", dry=False)
    sc.run_stand(a)
    ll = made["ll"]
    assert ll.calls[-1] == "hand_back"
    assert ll.sent[-1][1] == 0.0          # ended soft (damping), not holding a pose
    # ...after folding into the lying pose with stiffness (no topple from standing height).
    stiff = [q for q, kp in ll.sent if kp > 0]
    assert all(abs(x - y) < 0.05 for x, y in zip(stiff[-1], sc.LIE_DOWN))


def test_integral_correction_removes_steady_sag():
    class Sagging(FakeLowLevel):
        """The rear thighs and calves sit 0.2 rad short of the command (gravity under PD)."""
        def send(self, q, kp, kd):
            sag = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.0, 0.2, 0.2]
            for i in range(12):
                self.state.motor_state[i].q = q[i] - sag[i]
            self.sent.append((list(q), kp))
    ll = Sagging("eth0")
    ll.ki = 20.0                          # fast, for the test
    ref = sc.stand_joints(0.30)
    ll.q_ref = ref
    for _ in range(400):
        ll.send(ll.integrate(ref), 80.0, 5.0)
    err = [r - ll.state.motor_state[i].q for i, r in enumerate(ref)]
    assert max(abs(e) for e in err) < 0.01          # the sag is gone
    assert abs(ll.i_corr[7] - 0.2) < 0.01 and ll.i_corr[0] == 0.0
    # Fading: with ki 0 and i_fade set, the correction decays smoothly to ~0.
    ll.ki, ll.i_fade = 0.0, 0.5
    for _ in range(int(3.0 / ll.DT)):
        ll.integrate(ref)
    assert max(abs(c) for c in ll.i_corr) < 0.01


def test_integral_correction_is_capped():
    class Stuck(FakeLowLevel):
        def send(self, q, kp, kd):
            self.sent.append((list(q), kp))       # the joints never move
    ll = Stuck("eth0")
    ll.ki = 50.0
    ref = [v + 1.0 for v in sc.LIE_DOWN]
    for _ in range(2000):
        ll.integrate(ref)
    assert max(abs(c) for c in ll.i_corr) <= ll.I_MAX + 1e-9


def test_dense_path_has_no_problems_and_clears_the_object():
    plan = sc.Plan(args(stability_margin=0.05)).build()
    samples = sc.dense(plan)
    assert len(samples) > 1000
    assert not [p for s in samples for p in s[3]]
    clear = sc.over_object_clearance(plan, samples)
    assert clear is not None and clear >= 0.05 - 1e-6     # object height + 5 cm at the top
    # Consecutive samples never jump: at most 0.08 rad per 20 ms on any joint.
    for (_, _, q0, _, _), (_, _, q1, _, _) in zip(samples, samples[1:]):
        assert max(abs(a - b) for a, b in zip(q0, q1)) < 0.08


def test_step_over_on_a_fake_robot_stops_with_feet_down(monkeypatch):
    made = {}

    def init(self, iface):
        FakeLowLevel.__init__(self, iface)
        made["ll"] = self
    Fake = type("Fake", (FakeLowLevel,), {"__init__": init, "DT": 0.002})
    monkeypatch.setattr(sc, "LowLevel", Fake)
    a = args(stability_margin=0.05, lift=None, step_over=True, stop_after=3, hold=0.02, ramp=0.02,
             max_tilt=15.0, kp_lift=80.0, ki=1.0, iface="eth0", yes=True, restand=False, log="", dry=False)
    sc.run_stand(a)
    ll = made["ll"]
    assert ll.calls == ["release", "hand_back"]
    plan = sc.Plan(a).build()
    # --stop-after 3 rounds up to keyframe 4 ("RL down"): the swing foot is back on the floor.
    assert plan.keys[4]["name"].endswith("down")
    down_q = plan.joints(plan.keys[4])[0]
    assert any(all(abs(x - y) < 0.02 for x, y in zip(q, down_q)) for q, _ in ll.sent)
    fr_cross = [k for k in plan.keys if "over the object" in k["name"]][0]
    cross_q = plan.joints(fr_cross)[0]
    assert not any(all(abs(x - y) < 0.02 for x, y in zip(q, cross_q)) for q, _ in ll.sent)


def _crawl_run(monkeypatch, cls, **kw):
    made = {}

    def init(self, iface):
        FakeLowLevel.__init__(self, iface)
        made["ll"] = self
    Fake = type("Fake", (cls,), {"__init__": init, "DT": 0.002})
    monkeypatch.setattr(sc, "LowLevel", Fake)
    a = args(stability_margin=0.05, lift=None, step_over=True, stop_after=8, hold=0.02, ramp=0.02,
             max_tilt=15.0, kp_lift=80.0, ki=1.0, iface="eth0", yes=True, restand=False, log="",
             dry=False, t_shift=0.1, t_settle=0.05, t_lift=0.05, t_move=0.05, t_lower=0.05, **kw)
    sc.run_stand(a)
    return made["ll"], a


def test_lean_past_the_limit_lowers_the_body_then_goes_soft(monkeypatch, capsys):
    class Leans(FakeLowLevel):
        def send(self, q, kp, kd):
            super().send(q, kp, kd)
            if self.phase.startswith("6 "):              # leans 7 deg during that keyframe
                self.state.imu_state.rpy[0] = math.radians(7.0)
    ll, _ = _crawl_run(monkeypatch, Leans, lean_abort=6.0)
    out = capsys.readouterr().out
    assert "leaning 7.0 deg from the stand" in out and "lowering the body" in out
    stiff = [q for q, kp in ll.sent if kp > 0]
    assert all(abs(x - y) < 0.05 for x, y in zip(stiff[-1], sc.LIE_DOWN))   # folded down first
    assert ll.sent[-1][1] == 0.0 and ll.calls[-1] == "hand_back"


def test_hot_motor_refuses_to_start(monkeypatch):
    class Hot(FakeLowLevel):
        def __init__(self, iface):
            super().__init__(iface)
            self.state.motor_state[6].temperature = 61
    monkeypatch.setattr(sc, "LowLevel", Hot)
    a = args(lift=None, step_over=False, hold=0.02, ramp=0.02, max_tilt=12.0, kp_lift=80.0, ki=1.0,
             iface="eth0", yes=True, restand=False, log="", dry=False)
    try:
        sc.run_stand(a)
    except SystemExit as ex:
        assert "61 C" in str(ex)
        return
    raise AssertionError("a 61 C motor should refuse to start")


def test_feed_forward_holds_the_weight():
    q = sc.stand_joints(0.30)
    tau = sc.ff_torques(q, 0.30, 17.0)
    # Four feet, centre of mass in the middle: equal knee torques that straighten the legs.
    knees = [tau[3 * i + 2] for i in range(4)]
    assert all(abs(k - knees[0]) < 1e-6 for k in knees) and 5.0 < knees[0] < 8.0
    # Vertical balance: the forces the torques imply add up to the weight.
    forces = sc.support_forces([(sc.hip_xy(l)[0], sc.hip_xy(l)[1]) for l in sc.LEGS], [1, 1, 1, 1],
                               (0.0, 0.0), 17.0 * 9.81)
    assert abs(sum(forces) - 17.0 * 9.81) < 1e-6
    # A lifted foot carries nothing; the other three take the weight.
    a = args(stability_margin=0.05, lift_height=0.10, lift_margin=0.075, lift_shift_s=3.0, lift_up_s=2.0,
             hold=5.0)
    up = sc.lift_plan(a, "FR")[1][2]
    t = sc.ff_torques(up, 0.30, 17.0)
    assert all(abs(v) < 1e-9 for v in t[0:3])
    assert max(abs(v) for v in t) <= 30.0


def test_feed_forward_ramps_in_and_out_without_jumps():
    ll = FakeLowLevel("eth0")
    ll.q_ref = sc.stand_joints(0.30)
    ll.ff_target = 0.8
    for _ in range(int(1.0 / ll.DT) + 50):
        ll.feed_forward()
    assert abs(ll.ff_gain - 0.8) < 1e-9
    gains = []
    ll.ff_target = 0.0
    for _ in range(int(1.0 / ll.DT) + 50):
        ll.feed_forward()
        gains.append(ll.ff_gain)
    assert gains[-1] == 0.0 and max(abs(a - b) for a, b in zip(gains, gains[1:])) <= ll.DT + 1e-12


def test_settle_keyframes_follow_each_shift():
    plan = sc.Plan(args(stability_margin=0.05, t_settle=1.0)).build()
    names = [k["name"] for k in plan.keys]
    for i, n in enumerate(names):
        if n.startswith("shift for"):
            assert names[i + 1] == "settle for " + n.split()[-1]


def test_leg_over_plan_is_clean_and_runs_on_a_fake_robot(monkeypatch):
    a = args(stand_height=0.28, lift_margin=0.075, object_distance=0.06, object_depth=0.05,
             object_height=0.06, landing_margin=0.03, lift_shift_s=0.05, lift_up_s=0.05, t_move=0.05,
             t_settle=0.05, hold=0.02, ramp=0.02, max_tilt=12.0, kp_lift=80.0, ki=1.0, iface="eth0",
             yes=True, restand=False, log="", dry=False, lift=None, leg_over="FR")
    plan = sc.leg_over_plan(a, "FR")
    assert sc.check_leg_over(a, plan) == 0
    made = {}

    def init(self, iface):
        FakeLowLevel.__init__(self, iface)
        made["ll"] = self
    Fake = type("Fake", (FakeLowLevel,), {"__init__": init, "DT": 0.002})
    monkeypatch.setattr(sc, "LowLevel", Fake)
    sc.run_stand(a)
    ll = made["ll"]
    assert ll.calls == ["release", "hand_back"]
    beyond = plan.joints([k for k in plan.keys if k["name"].startswith("FR held beyond")][0])[0]
    assert any(all(abs(x - y) < 0.02 for x, y in zip(q, beyond)) for q, _ in ll.sent)


def test_dense_bodies_line_up_with_the_samples():
    plan = sc.Plan(args(stability_margin=0.05)).build()
    samples, bodies = sc.dense(plan), sc.dense_bodies(plan)
    assert len(samples) == len(bodies)
    # The body ends where the last keyframe puts it.
    assert all(abs(a - b) < 1e-9 for a, b in zip(bodies[-1], plan.keys[-1]["body"]))
