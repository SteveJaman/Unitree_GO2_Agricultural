"""Unit tests for scripts/nodes/semantic_labeler.py (no ROS, no Ollama)."""
import os
import struct
import sys
import zlib

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "nodes"))
import semantic_labeler as sl  # noqa: E402


def test_parse_json_answer():
    label, conf, reason = sl.parse_answer('{"object": "chair", "height_cm": 80, "label": "AVOID", "confidence": 0.95}')
    assert (label, conf) == ("AVOID", 0.95)
    assert "chair" in reason


def test_parse_raises_label_to_own_height():
    label, _, reason = sl.parse_answer('{"object": "side table", "height_cm": 20, "label": "STEP_OVER", "confidence": 0.8}')
    assert label == "CAUTION" and "raised" in reason
    label, _, _ = sl.parse_answer('{"object": "bin", "height_cm": 40, "label": "CAUTION", "confidence": 0.8}')
    assert label == "AVOID"


def test_parse_never_lowers_label():
    label, _, _ = sl.parse_answer('{"object": "cable", "height_cm": 3, "label": "CAUTION", "confidence": 0.9}')
    assert label == "CAUTION"


def test_parse_garbage_is_unknown():
    assert sl.parse_answer("no idea")[:2] == ("UNKNOWN", 0.0)
    assert sl.parse_answer('{"label": "FLY", "confidence": 1}')[:2] == ("UNKNOWN", 0.0)


def test_combine_never_below_geometry():
    assert sl.combine("STEP_OVER", 0.99, "AVOID", 0.7) == ("AVOID", "geometry")
    assert sl.combine("STEP_OVER", 0.99, "CAUTION", 0.7) == ("CAUTION", "geometry")


def test_combine_vlm_can_raise():
    assert sl.combine("CAUTION", 0.9, "STEP_OVER", 0.7) == ("CAUTION", "vlm")
    assert sl.combine("AVOID", 0.9, "CAUTION", 0.7) == ("AVOID", "vlm")


def test_combine_low_confidence_ignored():
    assert sl.combine("AVOID", 0.5, "STEP_OVER", 0.7) == ("STEP_OVER", "geometry")


def test_combine_unmeasured_only_avoid_passes():
    assert sl.combine("STEP_OVER", 0.9, "UNKNOWN", 0.7) == ("UNKNOWN", "none")
    assert sl.combine("CAUTION", 0.9, "UNKNOWN", 0.7) == ("UNKNOWN", "none")
    assert sl.combine("AVOID", 0.9, "UNKNOWN", 0.7) == ("AVOID", "vlm")


def test_geometry_label_bands():
    assert sl.geometry_label({"height": 0.04}) == "STEP_OVER"
    assert sl.geometry_label({"height": 0.20}) == "CAUTION"
    assert sl.geometry_label({"height": 0.30}) == "AVOID"
    assert sl.geometry_label({"height": None}) == "UNKNOWN"


def test_png_is_valid():
    rgb = np.random.default_rng(0).integers(0, 255, (7, 5, 3), dtype=np.uint8)
    png = sl.png_bytes(rgb)
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    w, h = struct.unpack(">II", png[16:24])
    assert (w, h) == (5, 7)
    idat_len = struct.unpack(">I", png[33:37])[0]
    raw = zlib.decompress(png[41:41 + idat_len])
    rows = np.frombuffer(raw, np.uint8).reshape(7, 1 + 5 * 3)
    assert (rows[:, 1:].reshape(7, 5, 3) == rgb).all()


def test_associate_picks_detection_containing_projection():
    rot, trans = np.eye(3), np.zeros(3)   # camera at the map origin, looking along +x
    box = {"id": 1, "class": "cable", "x": 2.0, "y": 0.0, "z": 0.03, "height": 0.06}
    K = (468.0, 468.0, 320.0, 180.0)
    hit = {"class": "cable", "bbox": [300, 170, 340, 200]}
    other_class = {"class": "shoe", "bbox": [300, 170, 340, 200]}
    far = {"class": "cable", "bbox": [0, 0, 20, 20]}
    assert sl.associate(box, [other_class, far, hit], rot, trans, K) is hit
    assert sl.associate(box, [other_class, far], rot, trans, K) is None
    behind = dict(box, x=-2.0)
    assert sl.associate(behind, [hit], rot, trans, K) is None


def test_crop_bbox_margin_and_clamp():
    img = np.zeros((360, 640, 3), np.uint8)
    crop, (u0, v0, u1, v1) = sl.crop_bbox(img, [0, 0, 100, 50])
    assert (u0, v0) == (0, 0) and u1 > 100 and v1 > 50
    assert crop.shape == (v1 - v0, u1 - u0, 3)
    assert sl.crop_bbox(img, [10, 10, 15, 15]) is None
