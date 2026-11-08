"""Semantic traversability labels for fused 3D boxes: crop, ask a VLM, combine with geometry.

Pure functions (numpy + standard library), shared by vlm_heuristic_node.py and
tests/traversability/eval_vlm_frames.py so the node and the offline check run
the same code.

Labels, from least to most severe:
  STEP_OVER  low and solid: walk over it with a high step
  CAUTION    low, but it may tangle, roll, tip or break: slow, careful gait
  AVOID      plan around it
  UNKNOWN    no usable answer; contributes no cost

The final label is never less severe than the LiDAR height band says, so the
VLM can raise a cost (a cable is low but tangles) but never lower one (a
backpack it calls "soft" is still 0.44 m tall). With no measured height only a
VLM AVOID is used: nothing confirms that an object the VLM calls low is low.

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

# The model names the object and estimates its height before it picks a label:
# asked for the label first (with format=json) qwen2.5vl answered STEP_OVER for
# every crop, chairs included.
PROMPT = (
    "This image is from the front camera of a small quadruped robot dog (legs about 30 cm long) "
    "walking on an indoor floor. The detector says the object is a '{cls}'. First identify the "
    "object and estimate its real height above the floor in centimetres. Then choose how the "
    "robot should treat it:\n"
    "AVOID if it is taller than 25 cm (furniture such as chairs, tables, shelves, cabinets, bins, "
    "bags), a person or animal, or must not be touched.\n"
    "CAUTION if it is 15-25 cm tall, or low but could tangle the feet, roll, tip over, spill or "
    "break (cables, cups, bottles, shoes).\n"
    "STEP_OVER only if it is under 15 cm, flat and solid (a mat, a board, a power strip, a book).\n"
    "UNKNOWN if no object is clearly visible.\n"
    "Answer with JSON only, in this order: "
    '{{"object": "what it is", "height_cm": number, '
    '"label": "AVOID or CAUTION or STEP_OVER or UNKNOWN", "confidence": 0.0-1.0}}'
)


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


def parse_answer(text, step_over_max=0.15, lethal_min=0.25):
    """(label, confidence, reason) from the model's text; UNKNOWN, 0.0 if unusable.

    The label is raised to the band of the model's own height estimate when the
    two disagree (e.g. "side table, 20 cm, STEP_OVER" becomes CAUTION).
    """
    try:
        m = re.search(r"\{.*\}", text, re.S)
        d = json.loads(m.group(0) if m else text)
        label = str(d.get("label", "")).strip().upper().replace(" ", "_").replace("-", "_")
        conf = max(0.0, min(1.0, float(d.get("confidence", 0.0))))
        reason = f"{d.get('object', '?')}, {d.get('height_cm', '?')} cm"
        if label not in LABELS:
            raise ValueError(label)
        try:
            h = float(d.get("height_cm")) / 100.0
            own = geometry_label({"height": h}, step_over_max, lethal_min)
            if label != "UNKNOWN" and SEVERITY[own] > SEVERITY[label]:
                reason += f", {label} raised to {own} by its own height"
                label = own
        except (TypeError, ValueError):
            pass
        return label, conf, reason
    except (ValueError, AttributeError, TypeError):
        pass
    return "UNKNOWN", 0.0, "unparseable: " + text[:120]


def ask_ollama(url, model, rgb, cls, timeout_s):
    """One crop to Ollama /api/chat. Returns (label, confidence, reason, latency_s)."""
    body = {"model": model, "stream": False, "format": "json", "keep_alive": "30m",
            "options": {"temperature": 0},
            "messages": [{"role": "user", "content": PROMPT.format(cls=cls),
                          "images": [base64.b64encode(png_bytes(upscale(rgb))).decode()]}]}
    req = urllib.request.Request(url.rstrip("/") + "/api/chat", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=timeout_s) as r:
        text = json.loads(r.read())["message"]["content"]
    return (*parse_answer(text), time.time() - t0)


def geometry_label(box, step_over_max=0.15, lethal_min=0.25):
    h = box.get("height")
    if h is None:
        return "UNKNOWN"
    if h <= step_over_max:
        return "STEP_OVER"
    return "CAUTION" if h < lethal_min else "AVOID"


def combine(vlm_label, vlm_conf, geo_label, confidence_min):
    """Final label and where it came from. See the module docstring for the rule."""
    vlm = vlm_label if vlm_label and vlm_conf >= confidence_min else "UNKNOWN"
    if geo_label == "UNKNOWN" and vlm != "AVOID":
        vlm = "UNKNOWN"
    if SEVERITY[vlm] > SEVERITY[geo_label]:
        return vlm, "vlm"
    if geo_label == "UNKNOWN":
        return "UNKNOWN", "none"
    return geo_label, ("geometry" if SEVERITY[geo_label] > SEVERITY[vlm] else "agree")


def radius(box, margin=0.05):
    sx, sy = box.get("sx") or 0.0, box.get("sy") or 0.0
    if sx <= 0 or sy <= 0:
        return UNKNOWN_SIZE / 2 + margin
    return float(np.hypot(sx, sy) / 2 + margin)
