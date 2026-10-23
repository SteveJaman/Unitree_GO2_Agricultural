#!/usr/bin/env python3
"""Ask a local vision-language model about the camera view (wifi_3d_nav.sh --ai).

At most every `period` s the newest camera frame is JPEG-encoded and sent to
an Ollama server (/api/generate, default llava:7b). A new request starts only
after the previous one has answered, so on a slow CPU the real rate is the
model's speed (llava:7b on a VM CPU: tens of seconds per frame).

The answer is reduced to one word: the first of HYDROPONIC / BLOCKED / CLEAR
it contains, else UNKNOWN. Nothing is published while the server is
unreachable. The decision is informational: no node drives on it yet.

Subscribes: /camera/image_raw  (sensor_msgs/Image, bgr8 / rgb8 / mono8)
Publishes:  /vlm_decision      (std_msgs/String: HYDROPONIC | BLOCKED | CLEAR | UNKNOWN)
            /vlm_decision/raw  (std_msgs/String: the model's full answer)
"""
import base64
import json
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

DEFAULT_PROMPT = ("You are a robot dog. Do you see a hydroponic station? Or is the path "
                  "ahead blocked by a desk? Answer 'HYDROPONIC', 'BLOCKED', or 'CLEAR'.")
DECISIONS = ('HYDROPONIC', 'BLOCKED', 'CLEAR')


def image_bgr(msg):
    """sensor_msgs/Image (bgr8 / rgb8 / mono8) -> HxWx3 uint8 BGR, or None."""
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding in ('bgr8', 'rgb8'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img
    if msg.encoding == 'mono8':
        img = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return np.repeat(img[:, :, None], 3, axis=2)
    return None


def decide(text):
    """First decision keyword in the answer (by position), else UNKNOWN.

    An answer naming all three just repeated the options: UNKNOWN.
    """
    up = text.upper()
    hits = [(up.find(d), d) for d in DECISIONS if d in up]
    if not hits or len(hits) == len(DECISIONS):
        return 'UNKNOWN'
    return min(hits)[1]


class VlmReasoning(Node):
    def __init__(self):
        super().__init__('vlm_reasoning')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('url', 'http://localhost:11434/api/generate')
        self.declare_parameter('model', 'llava:7b')
        self.declare_parameter('prompt', DEFAULT_PROMPT)
        self.declare_parameter('period', 5.0)          # s between requests (minimum)
        self.declare_parameter('timeout', 120.0)       # s per request
        self.declare_parameter('max_width', 672)       # px, frames are downscaled to this
        gp = lambda k: self.get_parameter(k).value  # noqa: E731

        self.url, self.model, self.prompt = gp('url'), gp('model'), gp('prompt')
        self.timeout = float(gp('timeout'))
        self.max_width = int(gp('max_width'))
        self.image = None
        self.busy = False
        self.lock = threading.Lock()

        self.pub = self.create_publisher(String, '/vlm_decision', 10)
        self.raw_pub = self.create_publisher(String, '/vlm_decision/raw', 10)
        self.create_subscription(Image, gp('image_topic'), self.on_image, qos_profile_sensor_data)
        self.create_timer(float(gp('period')), self.tick)
        self.get_logger().info(f'VLM {self.model} at {self.url} every >= {gp("period")} s '
                               f'on {gp("image_topic")}')

    def on_image(self, msg):
        self.image = msg

    def tick(self):
        with self.lock:
            if self.busy or self.image is None:
                return
            self.busy = True
            msg, self.image = self.image, None
        threading.Thread(target=self.ask, args=(msg,), daemon=True).start()

    def ask(self, msg):
        try:
            img = image_bgr(msg)
            if img is None:
                self.get_logger().warn(f'unsupported image encoding {msg.encoding!r}',
                                       throttle_duration_sec=60.0)
                return
            if img.shape[1] > self.max_width:
                h = int(img.shape[0] * self.max_width / img.shape[1])
                img = cv2.resize(img, (self.max_width, h), interpolation=cv2.INTER_AREA)
            ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                return
            body = json.dumps({
                'model': self.model, 'prompt': self.prompt, 'stream': False,
                'images': [base64.b64encode(jpg.tobytes()).decode('ascii')],
            }).encode()
            req = urllib.request.Request(self.url, data=body,
                                         headers={'Content-Type': 'application/json'})
            t0 = time.monotonic()
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                answer = json.loads(r.read().decode()).get('response', '').strip()
            decision = decide(answer)
            self.pub.publish(String(data=decision))
            self.raw_pub.publish(String(data=answer))
            self.get_logger().info(f'{decision} ({time.monotonic() - t0:.1f} s): {answer[:120]!r}')
        except urllib.error.HTTPError as e:
            detail = e.read().decode(errors='replace')[:200]
            self.get_logger().warn(f'Ollama HTTP {e.code}: {detail} '
                                   f'(model pulled? ollama pull {self.model})',
                                   throttle_duration_sec=30.0)
        except (urllib.error.URLError, OSError, TimeoutError) as e:
            self.get_logger().warn(f'Ollama not reachable at {self.url}: {e}',
                                   throttle_duration_sec=30.0)
        except (ValueError, KeyError) as e:
            self.get_logger().warn(f'bad Ollama reply: {e}', throttle_duration_sec=30.0)
        finally:
            with self.lock:
                self.busy = False


def main():
    rclpy.init()
    node = VlmReasoning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
