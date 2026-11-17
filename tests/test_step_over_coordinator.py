"""Tests for the step-over hand-off: scripts/nodes/step_over_coordinator.py (pure parts)
and scripts/tools/step_over_executor.py (no ROS, no robot)."""
import json
import math
import os
import sys
import time

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, "..", "scripts", "nodes"))
sys.path.insert(0, os.path.join(HERE, "..", "scripts", "tools"))
import step_over_coordinator as co  # noqa: E402
import step_over_executor as ex  # noqa: E402

P = dict(h_min=0.03, h_max=0.10, depth_max=0.10, ahead_min=0.35, ahead_max=1.30, lateral_max=0.20,
         front_foot_x=0.19, gap_target=0.12, gap_tol=0.03, yaw_tol=0.08, vx_max=0.10, wz_max=0.35)


def obj(x, y, h, sx=0.08, sy=0.20, yaw=0.0, label="CAUTION", cls="book", tid=1, protected=False):
    return {"x": x, "y": y, "height": h, "size_x": sx, "size_y": sy, "yaw": yaw, "label": label,
            "class": cls, "track_id": tid, "protected": protected}


def test_depth_along_the_heading():
    assert abs(co.depth_along(0.08, 0.20, 0.0, 0.0) - 0.08) < 1e-9
    assert abs(co.depth_along(0.08, 0.20, math.pi / 2, 0.0) - 0.20) < 1e-9
    assert co.depth_along(0.0, 0.2, 0.0, 0.0) is None


def test_picks_a_low_shallow_object_on_the_path():
    pose = (0.0, 0.0, 0.0)
    c, skipped = co.candidate([obj(0.8, 0.05, 0.06)], pose, P)
    assert c is not None and abs(c["near_edge"] - 0.76) < 1e-9 and c["height"] == 0.06
    assert not skipped


def test_skips_what_it_should_not_step_over():
    pose = (0.0, 0.0, 0.0)
    objs = [obj(0.8, 0.0, 0.02, tid=1),                                    # the normal step clears it
            obj(0.8, 0.0, 0.14, tid=2),                                    # too tall
            obj(0.8, 0.0, 0.06, sx=0.25, tid=3),                           # too deep
            obj(0.8, 0.0, 0.06, cls="potted plant", protected=True, tid=4),  # protected
            obj(0.8, 0.0, 0.06, label="AVOID", tid=5),
            obj(0.8, 0.6, 0.06, tid=6),                                    # beside the path: ignored
            obj(-0.8, 0.0, 0.06, tid=7)]                                   # behind: ignored
    c, skipped = co.candidate(objs, pose, P)
    assert c is None
    reasons = dict(skipped)
    assert "normal step" in reasons["book #1"] and "taller" in reasons["book #2"]
    assert "deep" in reasons["book #3"] and "protected" in reasons["potted plant #4"]
    assert "AVOID" in reasons["book #5"] and len(skipped) == 5


def test_nearest_of_two_and_heading_matters():
    pose = (1.0, 1.0, math.pi / 2)                     # facing +y
    c, _ = co.candidate([obj(1.0, 2.2, 0.06, yaw=math.pi / 2, tid=1),
                         obj(1.05, 1.7, 0.06, yaw=math.pi / 2, tid=2)], pose, P)
    assert c["obj"]["track_id"] == 2


def test_align_turns_first_then_drives_to_the_gap():
    c, _ = co.candidate([obj(0.8, 0.10, 0.06)], (0.0, 0.0, 0.0), P)
    vx, wz, done = co.align_command(c, (0.0, 0.0, 0.0), P)
    assert vx == 0.0 and wz > 0 and not done                    # object to the left: turn left
    yaw = math.atan2(0.10, 0.8)
    vx, wz, done = co.align_command(c, (0.0, 0.0, yaw), P)
    assert vx > 0 and not done                                  # facing it, too far: forward
    # Standing where the front feet are gap_target before the near edge: done.
    d = math.hypot(0.8, 0.10) - 0.04 - 0.19 - 0.12
    pose = (d * math.cos(yaw), d * math.sin(yaw), yaw)
    assert co.align_command(c, pose, P)[2]


def test_executor_command_and_ranges():
    cmd = ex.crawl_command({"id": "17", "height": 0.06, "depth": 0.08, "gap": 0.12})
    assert "--step-over --object-height 0.060 --object-depth 0.080 --object-distance 0.120" in cmd
    assert "--resume-session" in cmd and "nav_17.csv" in cmd
    for bad in ({"id": 1, "height": 0.30, "depth": 0.08, "gap": 0.1},
                {"id": 1, "height": 0.06, "depth": 0.08, "gap": 0.9}):
        try:
            ex.crawl_command(bad)
        except ValueError:
            continue
        raise AssertionError(f"{bad} should be refused")


def test_executor_judges_the_crawl_output():
    ok, s = ex.judge(0, "[crawl] running keyframes\n[crawl] crawl done: roll +0.5 pitch -0.3 deg since the stand\n")
    assert ok and "crawl done" in s
    ok, s = ex.judge(0, "[crawl] 16 shift for FR\n[crawl] STOP: leaning 6.2 deg from the stand: lowering the body\n")
    assert not ok and "STOP" in s
    ok, s = ex.judge(1, "[crawl] a motor is at 67 C (> 55): let the dog cool first, not started\n")
    assert not ok and "not started" in s


def test_executor_handles_a_request_once_and_refuses_stale_ones(tmp_path):
    req = tmp_path / "request_5.json"
    req.write_text(json.dumps({"id": "5", "t": time.time(), "object": "book #1", "height": 0.06,
                               "depth": 0.08, "gap": 0.12}))
    res = ex.handle(str(req), "unitree@x", True, 30.0)
    assert res["ok"] and not req.exists() and (tmp_path / "request_5.json.taken").exists()
    assert json.loads((tmp_path / "result_5.json").read_text())["ok"]
    old = tmp_path / "request_6.json"
    old.write_text(json.dumps({"id": "6", "t": time.time() - 120, "height": 0.06, "depth": 0.08, "gap": 0.1}))
    res = ex.handle(str(old), "unitree@x", True, 30.0)
    assert not res["ok"] and "older" in res["summary"]


def test_resume_only_once_the_object_is_behind_the_rear_feet():
    c, _ = co.candidate([obj(0.8, 0.0, 0.06)], (0.0, 0.0, 0.0), P)     # far edge at 0.84 m
    assert not co.is_clear(c, (0.0, 0.0, 0.0))[0]
    assert not co.is_clear(c, (0.9, 0.0, 0.0))[0]                     # body past it, rear feet not
    assert co.is_clear(c, (1.10, 0.0, 0.0))[0]                        # 0.84 < 1.10 - 0.22
