"""Yassine Salimi's decide() (traversability_core.py, commit e0715c1) as a reference
model for the safety property our fusion must share: geometry always wins, and
the VLM can only make an object more restricted, never less.

Our rule (scripts/nodes/semantic_labeler.combine) differs in its bands (0.12 /
0.22 m and a CAUTION level) and in requiring LiDAR before a VLM AVOID is lethal;
these tests check the property, not equal outputs.
"""
import itertools
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "nodes"))
import semantic_labeler as sl  # noqa: E402

RANK_REF = {"traversable": 0, "caution": 1, "avoid": 2}


def decide(height, verdict, step_limit, use_vlm=True):
    """Ported verbatim (logic) from traversability_core.decide."""
    if height >= step_limit:
        return "avoid"
    if not use_vlm:
        return "traversable"
    if verdict is None:
        return "caution"
    if verdict["label"] == "avoid" or verdict.get("fragile"):
        return "avoid"
    if verdict["label"] == "step_over":
        return "traversable"
    return "caution"


HEIGHTS = [0.0, 0.03, 0.06, 0.10, 0.12, 0.13, 0.20, 0.22, 0.30, 0.9]
VLM = [None, "STEP_OVER", "CAUTION", "AVOID"]


def test_reference_geometry_wins():
    for h, v in itertools.product(HEIGHTS, [None, {"label": "step_over"}, {"label": "caution"}]):
        if h >= 0.12:
            assert decide(h, v, 0.12) == "avoid"


def test_reference_vlm_only_restricts():
    for h in HEIGHTS:
        base = RANK_REF[decide(h, {"label": "step_over"}, 0.12)]
        for v in ({"label": "caution"}, {"label": "avoid"}, {"label": "step_over", "fragile": True}):
            assert RANK_REF[decide(h, v, 0.12)] >= base


def test_ours_geometry_wins():
    # A measured tall object is AVOID whatever the VLM says.
    for h, v in itertools.product(HEIGHTS, VLM):
        geo = sl.geometry_label({"height": h})
        final, _, _ = sl.combine(v, 1.0, geo, 0.7)
        assert sl.SEVERITY[final] >= sl.SEVERITY[geo]
        if h >= 0.22:
            assert final == "AVOID"


def test_ours_vlm_only_restricts():
    # Same property as the reference: a VLM answer never lowers the label below
    # what the same object gets with a STEP_OVER answer.
    for h in HEIGHTS:
        geo = sl.geometry_label({"height": h})
        base = sl.SEVERITY[sl.combine("STEP_OVER", 1.0, geo, 0.7)[0]]
        for v in ("CAUTION", "AVOID"):
            assert sl.SEVERITY[sl.combine(v, 1.0, geo, 0.7)[0]] >= base


def test_ours_fragile_low_object_is_restricted():
    # The laptop case: low by LiDAR, AVOID by the VLM -> lethal (two signals).
    assert sl.combine("AVOID", 1.0, sl.geometry_label({"height": 0.06}), 0.7)[0] == "AVOID"
    assert decide(0.06, {"label": "avoid", "fragile": True}, 0.12) == "avoid"
