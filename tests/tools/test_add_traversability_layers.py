#!/usr/bin/env python3
"""Tests for scripts/tools/add-traversability-layers.sh against a mock go2-nav2-wifi.

  python3 tests/tools/test_add_traversability_layers.py      (bash, python3-yaml; no ROS)

The fixture (tests/tools/fixtures/go2-nav2-wifi) is a copy of the real files the
script edits, taken 2026-11-09: lidar3d_planner.yaml with the 2026-11-06 global
height band, the lab-fixed lidar3d_controller.launch.py, setup.py, and the configs
the launch file reads. Each test copies it to a temp dir marked as a mock (no
rebuild) and runs the script with TARGET_REPO pointing there; the real clone is
never touched. The patched launch file is executed with stubbed launch modules, so
the local costmap Nav2 would get is checked, not just the text.
"""
import filecmp
import hashlib
import os
import shutil
import subprocess
import sys
import tempfile
import types

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
SCRIPT = os.path.join(REPO, "scripts", "tools", "add-traversability-layers.sh")
FIXTURE = os.path.join(HERE, "fixtures", "go2-nav2-wifi")
EDITED = ["go2_nav2/config/lidar3d_planner.yaml", "go2_nav2/launch/lidar3d_controller.launch.py",
          "go2_nav2/setup.py"]


def mock(legacy=True):
    d = tempfile.mkdtemp(prefix="mock-go2-nav2-wifi-")
    shutil.copytree(FIXTURE, d, dirs_exist_ok=True)
    open(os.path.join(d, ".mock-go2-nav2-wifi"), "w").close()
    if not legacy:   # the same planner without the global height band
        p = os.path.join(d, EDITED[0])
        doc = yaml.safe_load(open(p))
        g = doc["global_costmap"]["global_costmap"]["ros__parameters"]
        g["plugins"].remove("height_band_layer")
        g.pop("height_band_layer")
        yaml.safe_dump(doc, open(p, "w"), sort_keys=False, default_flow_style=False)
    orig = tempfile.mkdtemp(prefix="orig-")
    shutil.copytree(d, orig, dirs_exist_ok=True)
    return d, orig


def run(target, *args):
    env = dict(os.environ, TARGET_REPO=target, NAV2_WIFI_DIR="/nonexistent-real-clone")
    r = subprocess.run(["bash", SCRIPT, *args], env=env, capture_output=True, text=True)
    assert r.returncode == 0, f"{args}: exit {r.returncode}\n{r.stdout}\n{r.stderr}"
    return r.stdout + r.stderr


def digest(target):
    out = {}
    for root, _, files in os.walk(target):
        for f in files:
            p = os.path.join(root, f)
            out[os.path.relpath(p, target)] = hashlib.sha256(open(p, "rb").read()).hexdigest()
    return out


def global_params(target):
    doc = yaml.safe_load(open(os.path.join(target, EDITED[0])))
    return doc["global_costmap"]["global_costmap"]["ros__parameters"]


def local_costmap(target):
    """Run the (patched) launch file's configure() with stub launch modules; return
    local_costmap.ros__parameters from the params file it writes for controller_server."""
    captured = {}

    class Node:
        def __init__(self, **kw):
            if kw.get("name") == "controller_server":
                captured["path"] = kw["parameters"][0]

    class Anything:
        def __init__(self, *a, **kw):
            pass

    stubs = {
        "ament_index_python": types.ModuleType("ament_index_python"),
        "ament_index_python.packages": types.SimpleNamespace(
            get_package_share_directory=lambda _p: os.path.join(target, "go2_nav2")),
        "launch": types.SimpleNamespace(LaunchDescription=Anything),
        "launch.actions": types.SimpleNamespace(OpaqueFunction=Anything, RegisterEventHandler=Anything),
        "launch.event_handlers": types.SimpleNamespace(OnShutdown=Anything),
        "launch_ros": types.ModuleType("launch_ros"),
        "launch_ros.actions": types.SimpleNamespace(Node=Node),
    }
    saved = {k: sys.modules.get(k) for k in stubs}
    sys.modules.update(stubs)
    try:
        ns = {"__name__": "launch_under_test"}
        path = os.path.join(target, EDITED[1])
        exec(compile(open(path).read(), path, "exec"), ns)
        ns["configure"](None)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    doc = yaml.safe_load(open(captured["path"]))
    os.unlink(captured["path"])
    return doc["local_costmap"]["local_costmap"]["ros__parameters"]


def test_unpatched_fixture_is_the_old_layout():
    d, _ = mock()
    g = global_params(d)
    assert g["plugins"] == ["obstacle_layer", "height_band_layer", "inflation_layer"]
    assert g["height_band_layer"]["step_over_height_max"] == 0.15
    assert local_costmap(d)["plugins"] == ["obstacle_layer", "height_band_layer", "inflation_layer"]


def test_apply_layout():
    d, orig = mock()
    out = run(d)
    assert "EXTERNAL repo" in out and "mock target: go2_nav2 is not rebuilt" in out
    g = global_params(d)
    assert g["plugins"] == ["obstacle_layer", "semantic_layer", "inflation_layer"]
    assert "height_band_layer" not in g
    assert g["semantic_layer"]["max_range"] == 15.0
    loc = local_costmap(d)
    assert loc["plugins"] == ["obstacle_layer", "height_band_layer", "semantic_layer", "inflation_layer"]
    hb = loc["height_band_layer"]
    assert (hb["step_over_height_max"], hb["lethal_height_min"], hb["floor_estimation"], hb["min_range"]) == \
        (0.12, 0.22, "histogram", 0.25)
    assert loc["semantic_layer"]["max_range"] == 5.0          # local version overrides the global copy
    # The launch file's own local overrides still apply on top of the merge.
    assert loc["inflation_layer"]["inflation_radius"] == 0.55
    assert loc["obstacle_layer"]["lidar"]["topic"] == "/lidar3d/obstacle_points"
    setup = open(os.path.join(d, EDITED[2])).read()
    assert "config/lidar3d_local_layers.yaml" in setup
    assert filecmp.cmp(os.path.join(d, EDITED[0] + ".pre-legacy-heightband"), os.path.join(orig, EDITED[0]),
                       shallow=False)


def test_apply_is_idempotent():
    d, _ = mock()
    run(d)
    before = digest(d)
    out = run(d)
    assert "Already applied" in out
    assert digest(d) == before


def test_undo_restores_launch_and_setup_exactly_and_keeps_global_clean():
    d, orig = mock()
    run(d)
    run(d, "--undo")
    for f in EDITED[1:]:
        assert filecmp.cmp(os.path.join(d, f), os.path.join(orig, f), shallow=False), f
    g = global_params(d)
    assert g["plugins"] == ["obstacle_layer", "inflation_layer"]       # old band not restored
    ref = yaml.safe_load(open(os.path.join(orig, EDITED[0])))["global_costmap"]["global_costmap"]["ros__parameters"]
    ref["plugins"].remove("height_band_layer")
    ref.pop("height_band_layer")
    assert g == ref                                  # otherwise identical to the original
    assert not os.path.exists(os.path.join(d, "go2_nav2/config/lidar3d_local_layers.yaml"))
    left = [f for f in digest(d) if f.endswith(".pre-traversability")]
    assert left == []
    assert local_costmap(d)["plugins"] == ["obstacle_layer", "inflation_layer"]


def test_undo_twice_and_undo_without_apply_are_noops():
    d, _ = mock(legacy=False)
    before = digest(d)
    assert "Nothing to undo" in run(d, "--undo")
    assert digest(d) == before
    run(d)
    run(d, "--undo")
    after = digest(d)
    assert "Nothing to undo" in run(d, "--undo")
    assert digest(d) == after


def test_without_legacy_band_apply_undo_is_byte_identical():
    d, orig = mock(legacy=False)
    run(d)
    run(d, "--undo")
    assert digest(d) == digest(orig)


def test_undo_alone_removes_legacy_band_once():
    d, orig = mock()
    out = run(d, "--undo")
    assert "Legacy cleanup" in out
    assert global_params(d)["plugins"] == ["obstacle_layer", "inflation_layer"]
    assert filecmp.cmp(os.path.join(d, EDITED[0] + ".pre-legacy-heightband"), os.path.join(orig, EDITED[0]),
                       shallow=False)
    for f in EDITED[1:]:
        assert filecmp.cmp(os.path.join(d, f), os.path.join(orig, f), shallow=False), f


def test_check_changes_nothing():
    d, _ = mock()
    before = digest(d)
    out = run(d, "--check")
    assert "legacy band    : present" in out
    assert digest(d) == before


def test_missing_target_fails_clearly():
    env = dict(os.environ, TARGET_REPO="/nonexistent-target", NAV2_WIFI_DIR="/nonexistent")
    r = subprocess.run(["bash", SCRIPT], env=env, capture_output=True, text=True)
    assert r.returncode != 0 and "not found" in r.stderr


if __name__ == "__main__":
    n = f = 0
    for name in sorted(k for k in dir() if k.startswith("test_")):
        n += 1
        try:
            globals()[name]()
            print("ok  ", name)
        except Exception as ex:   # noqa: BLE001
            f += 1
            print("FAIL", name, "-", ex)
    print(f"{n} tests, {f} failed")
    sys.exit(1 if f else 0)
