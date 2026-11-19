"""scripts/nodes/detections_to_distance.py: which 3D detection becomes /object_distance."""
import os
import sys
import types
from types import SimpleNamespace as NS

import pytest

pytest.importorskip("rclpy")
if "vision_msgs.msg" not in sys.modules:  # the host has no vision_msgs; pick() needs none
    vm = types.ModuleType("vision_msgs")
    vm.msg = types.ModuleType("vision_msgs.msg")
    vm.msg.Detection3DArray = object
    sys.modules.update({"vision_msgs": vm, "vision_msgs.msg": vm.msg})
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "nodes"))
import detections_to_distance as dd  # noqa: E402


def det(cls, score, z, x=0.0, y=0.0):
    hyp = NS(hypothesis=NS(class_id=cls, score=score))
    return NS(results=[hyp], bbox=NS(center=NS(position=NS(x=x, y=y, z=z))))


def test_nearest_wanted_detection():
    dets = [det("bottle", 0.9, 0.80), det("bottle", 0.6, 0.55), det("person", 0.95, 0.40)]
    assert dd.pick(dets, "bottle", 0.4)[4] == 0.55          # nearest bottle, not the person
    assert dd.pick(dets, "", 0.4)[0] == "person"            # any class: the nearest of all


def test_skips_weak_empty_and_bad_depth():
    dets = [det("bottle", 0.3, 0.5), det("bottle", 0.9, 0.0), det("bottle", 0.9, float("nan")),
            NS(results=[], bbox=None)]
    assert dd.pick(dets, "bottle", 0.4) is None
