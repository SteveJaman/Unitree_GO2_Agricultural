"""Vision-language backends: ask a yes/no question about an image.

  OllamaBackend : a local Ollama server (default http://127.0.0.1:11434),
                  e.g. the go2-ollama Docker container running moondream.
  MockBackend   : deterministic stand-in for tests; answers "yes" when enough
                  of the image falls in an HSV colour range.

Both take a BGR numpy image and return (answer: bool, raw_text: str).
"""
import base64
import json
import urllib.error
import urllib.request
from typing import Dict, Tuple

import cv2
import numpy as np


def parse_yes_no(text: str) -> bool:
    """True only for an answer that starts with yes. Small VLMs ramble, so
    look at the first word and treat anything unclear as "no" (no cost)."""
    words = text.strip().lower().replace(',', ' ').replace('.', ' ').split()
    return bool(words) and words[0] in ('yes', 'y', 'true')


class OllamaBackend:
    def __init__(self, url: str, model: str, timeout_s: float = 120.0,
                 max_side: int = 448):
        self.url = url.rstrip('/') + '/api/generate'
        self.model = model
        self.timeout_s = timeout_s
        self.max_side = max_side

    def ask(self, image_bgr: np.ndarray, question: str, rule: Dict = None) -> Tuple[bool, str]:
        img = image_bgr
        scale = self.max_side / float(max(img.shape[:2]))
        if scale < 1.0:  # smaller images are much faster on CPU
            img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise RuntimeError('JPEG encode failed')
        body = json.dumps({
            'model': self.model,
            'prompt': question,
            'images': [base64.b64encode(jpg.tobytes()).decode()],
            'stream': False,
            'options': {'temperature': 0.0, 'num_predict': 8},
        }).encode()
        req = urllib.request.Request(self.url, data=body,
                                     headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
            text = json.loads(resp.read().decode()).get('response', '')
        return parse_yes_no(text), text.strip()


class MockBackend:
    """Colour-fraction stand-in. rule: {hsv_lo: [h,s,v], hsv_hi: [h,s,v],
    min_fraction: 0.2}. A query with no rule always answers "no"."""

    def ask(self, image_bgr: np.ndarray, question: str, rule: Dict = None) -> Tuple[bool, str]:
        if not rule:
            return False, 'no (no mock rule)'
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(rule['hsv_lo'], np.uint8),
                           np.array(rule['hsv_hi'], np.uint8))
        frac = float(np.count_nonzero(mask)) / mask.size
        yes = frac >= float(rule.get('min_fraction', 0.2))
        return yes, f"{'yes' if yes else 'no'} (mock {frac:.2f})"


def make_backend(kind: str, url: str, model: str, timeout_s: float, max_side: int):
    if kind == 'ollama':
        return OllamaBackend(url, model, timeout_s, max_side)
    if kind == 'mock':
        return MockBackend()
    raise ValueError(f'unknown backend {kind!r} (use ollama or mock)')


def ollama_ready(url: str, model: str, timeout_s: float = 3.0) -> Tuple[bool, str]:
    """Check the server is up and the model is pulled."""
    try:
        with urllib.request.urlopen(url.rstrip('/') + '/api/tags', timeout=timeout_s) as r:
            names = [m.get('name', '') for m in json.loads(r.read().decode()).get('models', [])]
    except (urllib.error.URLError, OSError, ValueError) as exc:
        return False, f'Ollama not reachable at {url}: {exc}'
    if not any(n == model or n.split(':')[0] == model for n in names):
        return False, f'model {model!r} not pulled (have: {", ".join(names) or "none"})'
    return True, 'ok'
