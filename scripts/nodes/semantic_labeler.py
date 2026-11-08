"""Semantic traversability labels for fused 3D boxes: crop, ask a VLM, combine with geometry.

Pure functions (numpy + standard library), shared by vlm_heuristic_node.py and
tests/traversability/eval_vlm_frames.py so the node and the offline check run
the same code.

Labels, from least to most severe:
  STEP_OVER  low and solid: walk over it with a high step
  CAUTION    low, but it may tangle, roll, tip or break: slow, careful gait
  AVOID      plan around it
  UNKNOWN    no usable answer; contributes no cost

Fusion (geometry from LiDAR, meaning from the VLM; never the reverse):
  * LiDAR measured the height: the band gives cost_geometric (<= 0.12 m
    STEP_OVER 40, 0.12-0.22 CAUTION 200, >= 0.22 AVOID 254); the VLM label gives
    cost_semantic; the published label is the more severe of the two. Lethal
    from the VLM needs both signals: LiDAR says something is there, the VLM
    says avoid it.
  * No LiDAR height: any VLM label is published as CAUTION (200), flagged
    low_confidence; the gait for it is CAREFUL. A single signal never makes a
    cell lethal, and an unmeasured object is never assumed to be clutter.
The VLM's own height estimate is not used anywhere. The detector's class and
score are given to the VLM to verify, not to adopt: it is asked whether they
match what it sees and to classify independently.

Crops come from the detector's own 2D box for the track (associate()), not from
projecting the track: an unmeasured track's position is a floor-ray estimate and
its projection can miss the object entirely.
"""
import base64
import json
import re
import struct
import time
import urllib.request
import zlib

import numpy as np

LABELS = ("STEP_OVER", "CAUTION", "AVOID", "UNKNOWN")
SEVERITY = {"UNKNOWN": -1, "STEP_OVER": 0, "CAUTION": 1, "AVOID": 2}
GEOMETRY = {"step_over": "STEP_OVER", "caution": "CAUTION", "blocked": "AVOID", "lethal": "AVOID"}
UNKNOWN_SIZE = 0.3   # footprint assumed for a track whose extent was not measured, m

# Prompt variants compared offline on 30 saved frames (eval_vlm_frames.py --prompt);
# PROMPT_ID is the one the node ships. {cls} and {score} are the detector's.
#   verify_oneword      the verification prompt as specified: detector class as a claim
#                       to check, one-word answer (no confidence: scored 1.0)
#   verify_defs_first   label definitions first, detector class after, no "say so if
#                       wrong" nudge; JSON with a confidence
#   object_first        no detector class; name the object, then label (JSON). Given
#                       the class as plain context the VLM adopted it ("person" on a shoe).
# The JSON variants name the object before the label: label-first, qwen2.5vl
# answered STEP_OVER for every crop.
_DEFS = (
    "STEP_OVER: safe clutter (cable, mat, shoe, low object), safe to step over with a 12 cm foot lift.\n"
    "AVOID: tall barrier, liquid, or fragile object; do not touch.\n"
    "CAUTION: uncertain, potentially fragile, or slow-and-careful only.\n")
PROMPTS = {
    "verify_oneword": (
        "Another detector identified this as '{cls}' with confidence {score:.2f}. Looking at the image, "
        "does that match what you see? Classify independently as STEP_OVER (safe clutter: cable, mat, "
        "shoe, low object; safe to step over with 12 cm foot lift), AVOID (tall barrier, liquid, "
        "fragile object; do not touch), or CAUTION (uncertain, potentially fragile, or "
        "slow-and-careful only). If the detector's suggestion is wrong, say so. Answer with one word."),
    "verify_defs_first": (
        "This image is from the front camera of a small quadruped robot dog walking on an indoor floor. "
        "Labels:\n" + _DEFS +
        "A detector suggested '{cls}' (confidence {score:.2f}); use it only if it matches what you see. "
        "Answer with JSON only, in this order: "
        '{{"object": "what you see", "label": "STEP_OVER or AVOID or CAUTION", "confidence": 0.0-1.0}}'),
    "object_first": (
        "This image is from the front camera of a small quadruped robot dog walking on an indoor "
        "floor. Its feet can lift about 12 cm. What is the object in the image? First identify it, "
        "then choose how the robot should treat it:\n"
        "AVOID if it is taller than about 22 cm (furniture such as chairs, tables, shelves, cabinets, "
        "bins, bags), a person or animal, or valuable or fragile and must not be stepped on "
        "(electronics, glass).\n"
        "CAUTION if it is low but could tangle the feet, roll, tip over, spill or break "
        "(cables, cups, bottles, shoes).\n"
        "STEP_OVER only if it is low (under 12 cm), flat and solid (a mat, a board, a power strip, a book).\n"
        "UNKNOWN if no object is clearly visible.\n"
        "Answer with JSON only, in this order: "
        '{{"object": "what it is", "label": "AVOID or CAUTION or STEP_OVER or UNKNOWN", '
        '"confidence": 0.0-1.0}}'),
}
PROMPT_ID = "verify_oneword"   # locked by the offline comparison (traversability_sim.md, "VLM prompt selection")
PROMPT = PROMPTS[PROMPT_ID]


def rotation_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def associate(box, dets, rot, trans, K, max_px=60.0):
    """The detection (same class) that belongs to this track in the current image, or None."""
    fx, fy, cx, cy = K
    z = (box.get("z") or 0.0) if box.get("height") is not None else 0.05
    p = rot @ np.array([box["x"], box["y"], z]) + trans
    if p[0] < 0.2:
        return None
    u, v = cx - fx * p[1] / p[0], cy - fy * p[2] / p[0]
    best, dist = None, max_px
    for d in dets:
        if d.get("class") != box.get("class"):
            continue
        x1, y1, x2, y2 = d["bbox"]
        inside = x1 <= u <= x2 and y1 <= v <= y2
        dd = 0.0 if inside else float(np.hypot(u - (x1 + x2) / 2, v - (y1 + y2) / 2))
        if dd < dist:
            best, dist = d, dd
    return best


def crop_bbox(image, bbox, margin=0.2, min_px=16):
    """Window around a 2D detection box with a margin, or None when too small."""
    h, w = image.shape[:2]
    x1, y1, x2, y2 = bbox
    pu, pv = margin * (x2 - x1) + 6, margin * (y2 - y1) + 6
    u0, u1 = int(max(0, x1 - pu)), int(min(w, x2 + pu))
    v0, v1 = int(max(0, y1 - pv)), int(min(h, y2 + pv))
    if x2 - x1 < min_px or y2 - y1 < min_px or u1 <= u0 or v1 <= v0:
        return None
    return image[v0:v1, u0:u1], (u0, v0, u1, v1)


def to_rgb(image, encoding):
    img = image[:, :, :3]
    return img[:, :, ::-1] if str(encoding).startswith("bgr") else img


def png_bytes(rgb):
    """RGB uint8 array to PNG, standard library only (the Nav2 container has no cv2/PIL)."""
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    h, w = rgb.shape[:2]
    raw = b"".join(b"\x00" + rgb[r].tobytes() for r in range(h))

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def upscale(rgb, min_side=224):
    """Nearest-neighbour upscale of small crops so the VLM sees more than a few pixels."""
    h, w = rgb.shape[:2]
    k = int(np.ceil(min_side / max(1, min(h, w))))
    return rgb if k <= 1 else np.repeat(np.repeat(rgb, k, axis=0), k, axis=1)


def parse_answer(text):
    """(label, confidence, reason) from the model's text; UNKNOWN, 0.0 if unusable.
    A bare one-word answer (verify_oneword) has no confidence and is scored 1.0."""
    word = text.strip().strip(".!\"'").upper().replace(" ", "_").replace("-", "_")
    if word in LABELS:
        return word, 1.0, "one-word answer"
    try:
        m = re.search(r"\{.*\}", text, re.S)
        d = json.loads(m.group(0) if m else text)
        label = str(d.get("label", "")).strip().upper().replace(" ", "_").replace("-", "_")
        conf = max(0.0, min(1.0, float(d.get("confidence", 0.0))))
        if label in LABELS:
            reason = str(d.get("object", "?"))[:80]
            if d.get("detector_correct") is False:
                reason += " (detector wrong)"
            return label, conf, reason
    except (ValueError, AttributeError, TypeError):
        pass
    return "UNKNOWN", 0.0, "unparseable: " + text[:120]


def ask_ollama(url, model, rgb, cls, score, timeout_s, prompt_id=None):
    """One crop to Ollama /api/chat. Returns (label, confidence, reason, latency_s)."""
    pid = prompt_id or PROMPT_ID
    body = {"model": model, "stream": False, "keep_alive": "30m", "options": {"temperature": 0},
            "messages": [{"role": "user", "content": PROMPTS[pid].format(cls=cls, score=float(score)),
                          "images": [base64.b64encode(png_bytes(upscale(rgb))).decode()]}]}
    if pid != "verify_oneword":
        body["format"] = "json"
    req = urllib.request.Request(url.rstrip("/") + "/api/chat", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=timeout_s) as r:
        text = json.loads(r.read())["message"]["content"]
    return (*parse_answer(text), time.time() - t0)


def geometry_label(box, step_over_max=0.12, lethal_min=0.22):
    h = box.get("height")
    if h is None:
        return "UNKNOWN"
    if h <= step_over_max:
        return "STEP_OVER"
    return "CAUTION" if h < lethal_min else "AVOID"


def combine(vlm_label, vlm_conf, geo_label, confidence_min):
    """(label, source, low_confidence). See the module docstring for the rule."""
    vlm = vlm_label if vlm_label and vlm_conf >= confidence_min else "UNKNOWN"
    if geo_label == "UNKNOWN":
        if vlm == "UNKNOWN":
            return "UNKNOWN", "none", True
        # Unmeasured: any VLM label is CAUTION. Lethal needs LiDAR as well.
        return "CAUTION", "vlm_unmeasured", True
    if SEVERITY[vlm] > SEVERITY[geo_label]:
        return vlm, "vlm", False
    return geo_label, ("geometry" if SEVERITY[geo_label] > SEVERITY[vlm] else "agree"), False


def radius(box, margin=0.05):
    sx, sy = box.get("sx") or 0.0, box.get("sy") or 0.0
    if sx <= 0 or sy <= 0:
        return UNKNOWN_SIZE / 2 + margin
    return float(np.hypot(sx, sy) / 2 + margin)
