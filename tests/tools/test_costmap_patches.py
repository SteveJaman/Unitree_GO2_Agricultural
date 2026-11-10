#!/usr/bin/env python3
"""The three patches to go2-nav2-wifi's lidar3d_planner.yaml working together:
nav2_voxel_layer.py (VoxelLayer), nav2_camera_source.py (front-camera source) and
add-traversability-layers.sh (height band / semantic layout).

  python3 tests/tools/test_costmap_patches.py      (bash, python3-yaml; no ROS)

Uses the same mock as test_add_traversability_layers.py (real files copied
2026-11-09; the planner already has the YOLO source "lidar yolo" and the
relaxed costmap). Sessions are replayed in wifi_3d_nav.sh's order: the
traversability step, then wifi_nav.sh's camera source and voxel layer steps.
"""
import itertools
import os
import subprocess
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_add_traversability_layers as T  # noqa: E402

TOOLS = os.path.join(T.REPO, "scripts", "tools")
PLANNER = "go2_nav2/config/lidar3d_planner.yaml"


def tool(name, mode, target):
    r = subprocess.run([sys.executable, os.path.join(TOOLS, name), mode, os.path.join(target, PLANNER)],
                       capture_output=True, text=True)
    assert r.returncode == 0, f"{name} {mode}: {r.stdout}{r.stderr}"
    return r.stdout


def voxel(mode, target):
    return tool("nav2_voxel_layer.py", mode, target)


def camera(mode, target):
    return tool("nav2_camera_source.py", mode, target)


def text(target):
    return open(os.path.join(target, PLANNER)).read()


def obstacle(target):
    return T.global_params(target)["obstacle_layer"]


def session(target, semantic, cam, vox):
    """One wifi_3d_nav.sh start: traversability step, then camera, then voxel."""
    T.run(target, *([] if semantic else ["--undo"]))
    camera("--enable" if cam else "--disable", target)
    voxel("--enable" if vox else "--disable", target)


def test_voxel_enable_layout():
    d, _ = T.mock()
    out = voxel("--enable", d)
    assert "enabled" in out
    o = obstacle(d)
    assert o["plugin"] == "nav2_costmap_2d::VoxelLayer"
    assert (o["z_resolution"], o["z_voxels"], o["origin_z"], o["max_obstacle_height"], o["mark_threshold"],
            o["publish_voxel_map"]) == (0.1, 16, 0.0, 1.5, 0, True)
    assert o["lidar"]["topic"] == "/lidar3d/obstacle_points"
    assert o["lidar"]["max_obstacle_height"] == 1.5
    assert o["lidar"]["min_obstacle_height"] == 0.15          # floor rejection unchanged
    assert o["observation_sources"].split() == ["lidar", "yolo"]
    assert o["yolo"]["topic"] == "/yolo/obstacles"


def test_voxel_disable_byte_exact_and_idempotent():
    d, orig = T.mock()
    before = text(d)
    voxel("--enable", d)
    once = text(d)
    assert "already enabled" in voxel("--enable", d)
    assert text(d) == once
    voxel("--disable", d)
    assert text(d) == before
    assert "not enabled" in voxel("--disable", d)
    assert text(d) == before


def test_camera_source_with_yolo_source_present():
    # Before the fix, enable required exactly "observation_sources: lidar" and failed here.
    d, _ = T.mock()
    before = text(d)
    camera("--enable", d)
    o = obstacle(d)
    assert o["observation_sources"].split() == ["lidar", "yolo", "camera"]
    assert o["camera"]["topic"] == "/camera/obstacle_points" and o["camera"]["clearing"] is False
    camera("--disable", d)
    assert text(d) == before


def test_camera_and_voxel_any_order():
    for on, off in itertools.product(itertools.permutations((camera, voxel)), repeat=2):
        d, _ = T.mock()
        before = text(d)
        for f in on:
            f("--enable", d)
        o = obstacle(d)
        assert o["plugin"] == "nav2_costmap_2d::VoxelLayer"
        assert o["observation_sources"].split() == ["lidar", "yolo", "camera"]
        for f in off:
            f("--disable", d)
        assert text(d) == before, (on, off)


def test_traversability_apply_keeps_other_patches_tags():
    d, _ = T.mock()
    camera("--enable", d)
    voxel("--enable", d)
    T.run(d)                              # text edits only: the tags must survive
    g = T.global_params(d)
    assert g["plugins"] == ["obstacle_layer", "semantic_layer", "inflation_layer"]
    assert g["obstacle_layer"]["plugin"] == "nav2_costmap_2d::VoxelLayer"
    assert "go2-voxel-layer" in text(d) and "go2-camera-obstacles" in text(d)
    camera("--disable", d)
    voxel("--disable", d)
    o = obstacle(d)
    assert o["plugin"] == "nav2_costmap_2d::ObstacleLayer" and "camera" not in o
    assert o["lidar"]["topic"] == "/lidar3d/registered"


def test_session_sequences_end_where_they_should():
    # Reference: the planner after the one-time legacy cleanup, nothing else on.
    ref_dir, _ = T.mock()
    T.run(ref_dir, "--undo")
    clean = text(ref_dir)
    flags = [(s, c, v) for s in (0, 1) for c in (0, 1) for v in (0, 1)]
    for a, b in itertools.product(flags, repeat=2):
        d, _ = T.mock()
        session(d, *a)
        session(d, *b)
        g = T.global_params(d)
        o = g["obstacle_layer"]
        s, c, v = b
        assert ("semantic_layer" in g["plugins"]) == bool(s), (a, b)
        assert ("camera" in o["observation_sources"].split()) == bool(c), (a, b)
        assert (o["plugin"] == "nav2_costmap_2d::VoxelLayer") == bool(v), (a, b)
        assert "height_band_layer" not in g["plugins"], (a, b)
        session(d, 0, 0, 0)
        assert text(d) == clean, (a, b)


def test_local_costmap_nav2_would_build():
    d, _ = T.mock()
    session(d, 1, 1, 1)
    loc = T.local_costmap(d)
    assert loc["plugins"] == ["obstacle_layer", "height_band_layer", "semantic_layer", "inflation_layer"]
    o = loc["obstacle_layer"]
    assert o["plugin"] == "nav2_costmap_2d::VoxelLayer"
    assert o["lidar"]["topic"] == "/lidar3d/obstacle_points"
    assert o["observation_sources"].split() == ["lidar", "yolo", "camera"]
    assert (o["z_resolution"], o["z_voxels"], o["publish_voxel_map"]) == (0.1, 16, True)


if __name__ == "__main__":
    n = f = 0
    for name in sorted(k for k in dir() if k.startswith("test_")):
        n += 1
        try:
            globals()[name]()
            print("ok  ", name)
        except Exception as ex:   # noqa: BLE001
            f += 1
            print("FAIL", name, "-", repr(ex)[:400])
    print(f"{n} tests, {f} failed")
    sys.exit(1 if f else 0)
