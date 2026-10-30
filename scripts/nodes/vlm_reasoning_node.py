#!/usr/bin/env python3
"""Ask a local vision-language model about the camera view (wifi_3d_nav.sh --ai).

At most every `period` s the newest camera frame is JPEG-encoded and sent to
an Ollama server (/api/generate, default llava:7b). A new request starts only
after the previous one has answered, so on a slow CPU the real rate is the
model's speed (llava:7b: seconds per frame on the GPU, tens of seconds on CPU only).

The answer is reduced to one word: the first of HYDROPONIC / BLOCKED / CLEAR
it contains, else UNKNOWN. Nothing is published while the server is
unreachable.

Action (act_on_blocked): after `blocked_confirmations` BLOCKED answers in a
row (default 2: one misread frame must not abort a mission):
  blocked_action "sidestep" (default): the same frame goes back to the model
     with VERIFY_PROMPT ("CLEAR, or BLOCKED and more room LEFT or RIGHT?").
     Unless that second look also says BLOCKED, nothing happens (llava:7b
     calls a corridor of cabinets "blocked" more often than not). Otherwise
     the side goes to go2_stuck_recovery.py on /recovery/sidestep. It acts
     only while Nav2 is driving, at most twice in two minutes; it checks that
     side (then the other) is free in the local costmap, pauses Nav2, steps
     round and lets Nav2 continue from there. If neither side is free (or no
     answer on /recovery/sidestep_result within 20 s), the dock action below
     runs; "not driving" / "stepped enough" answers do nothing.
  blocked_action "dock": publish False on /explore/resume (the explorer
     stops), then 1 s later /goal_pose to the dock (dock_x, dock_y, dock_yaw
     in map_frame, default the origin = where the session started). The
     explorer is not restarted automatically.
Then it waits `cooldown` s (`sidestep_cooldown` after a sidestep) before it
can act again. CLEAR / HYDROPONIC / UNKNOWN reset the count. Nothing happens
while the dashboard's Drive pad has manual control (/teleop/status).

Lidar check (/vlm_verify_request): the LiDAR sometimes sees obstacles that
are not there (reflections, dust, a smeared scan). When Nav2 gives up on a
leg with the costmap blocked right ahead, go2_stuck_recovery.py asks here
first: the newest frame and VERIFY_PROMPT, answered ahead of the routine
check on /vlm_verify_result {"id", "decision": CLEAR|BLOCKED|UNKNOWN,
"side": "left"|"right"|null, "raw"}. CLEAR = a LiDAR ghost: recovery clears
the costmaps and retries; BLOCKED = real: it sidesteps to `side`.

Room naming: a room id on /vlm_room_request (sent by the dashboard when
"Name this area" opens) is answered next, ahead of the routine check, with
the newest frame and ROOM_PROMPT; the answer is cut to a short name and
published as {"room_id", "suggestion", "raw"} on /vlm_room_suggestion. The
camera shows what is in front of the robot, so a suggestion is only
meaningful while the robot is in (or looking into) that room.

Subscribes: /camera/image_raw  (sensor_msgs/Image, bgr8 / rgb8 / mono8)
            /vlm_room_request  (std_msgs/String: room id)
            /vlm_verify_request (std_msgs/String JSON {"id"})
            /recovery/sidestep_result, /teleop/status (std_msgs/String JSON)
Publishes:  /vlm_decision      (std_msgs/String: HYDROPONIC | BLOCKED | CLEAR | UNKNOWN)
            /vlm_room_suggestion (std_msgs/String JSON)
            /vlm_decision/raw  (std_msgs/String: the model's full answer)
            /vlm_verify_result (std_msgs/String JSON)
            /recovery/sidestep (std_msgs/String JSON {"side", "source"})
            /explore/resume    (std_msgs/Bool, False before docking)
            /goal_pose         (geometry_msgs/PoseStamped, the dock)
"""

import base64
import json
import math
import re
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

DEFAULT_PROMPT = (
    "You are a robot dog. Do you see a hydroponic station? Or is the path "
    "ahead blocked by a desk? Answer 'HYDROPONIC', 'BLOCKED', or 'CLEAR'."
)
DECISIONS = ("HYDROPONIC", "BLOCKED", "CLEAR")
# Asked about the view straight ahead, to tell a real obstacle from a LiDAR
# ghost and to pick the side to step round it.
VERIFY_PROMPT = (
    "You are the front camera of a small robot dog, 40 cm wide. Look at the floor "
    "straight ahead, up to about one metre. Is anything in the way (a wall, "
    "furniture, a box, a person, a plant)? Answer with exactly one of: "
    "'CLEAR' if the dog can walk straight ahead; 'BLOCKED LEFT' if it is "
    "blocked and there is more free floor on the left; 'BLOCKED RIGHT' if it "
    "is blocked and there is more free floor on the right."
)
ROOM_PROMPT = (
    "Based on this image, what should I name this room? (e.g., Kitchen, Lab, Office). "
    "Answer with the room name only, at most three words."
)


def room_name(text):
    """'This looks like a lab office.' -> 'Lab Office' (first line, <= 3 words)."""
    line = (text.strip().splitlines() or [""])[0]
    line = re.sub(
        r"^(this|it)\s+(is|looks like|appears to be|seems to be)\s+",
        "",
        line,
        flags=re.I,
    )
    line = re.sub(r"^(a|an|the)\s+", "", line, flags=re.I)
    words = re.findall(r"[A-Za-z][A-Za-z'\-]*", line)[:3]
    return " ".join(w.capitalize() for w in words)


def verify_answer(text):
    """'BLOCKED LEFT ...' -> ("BLOCKED", "left"); 'CLEAR' -> ("CLEAR", None).

    BLOCKED wins over CLEAR when both appear ("not clear: blocked").
    """
    up = text.upper()
    if "BLOCKED" in up or "NOT CLEAR" in up:
        tail = up[up.find("BLOCKED") :] if "BLOCKED" in up else up
        left, right = tail.find("LEFT"), tail.find("RIGHT")
        side = None
        if left >= 0 and (right < 0 or left < right):
            side = "left"
        elif right >= 0:
            side = "right"
        return "BLOCKED", side
    if "CLEAR" in up:
        return "CLEAR", None
    return "UNKNOWN", None


def image_bgr(msg):
    """sensor_msgs/Image (bgr8 / rgb8 / mono8) -> HxWx3 uint8 BGR, or None."""
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding in ("bgr8", "rgb8"):
        img = buf.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(
            msg.height, msg.width, 3
        )
        return img[:, :, ::-1].copy() if msg.encoding == "rgb8" else img
    if msg.encoding == "mono8":
        img = buf.reshape(msg.height, msg.step)[:, : msg.width]
        return np.repeat(img[:, :, None], 3, axis=2)
    return None


def decide(text):
    """First decision keyword in the answer (by position), else UNKNOWN.

    An answer naming all three just repeated the options: UNKNOWN.
    """
    up = text.upper()
    hits = [(up.find(d), d) for d in DECISIONS if d in up]
    if not hits or len(hits) == len(DECISIONS):
        return "UNKNOWN"
    return min(hits)[1]


# Raw camera frames (~1.5 MB, ~1100 UDP packets) are read reliable: a best-effort
# reader loses the whole frame when one packet is dropped (go2_camera_relay.py
# publishes them reliable for that reason).
IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1
)


class VlmReasoning(Node):
    def __init__(self):
        super().__init__("vlm_reasoning")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("url", "http://localhost:11434/api/generate")
        self.declare_parameter("model", "llava:7b")
        self.declare_parameter("prompt", DEFAULT_PROMPT)
        self.declare_parameter("period", 5.0)  # s between requests (minimum)
        self.declare_parameter("timeout", 120.0)  # s per request
        self.declare_parameter("max_width", 672)  # px, frames are downscaled to this
        self.declare_parameter("act_on_blocked", True)
        self.declare_parameter("blocked_confirmations", 2)
        self.declare_parameter("cooldown", 60.0)  # s after acting
        self.declare_parameter("dock_x", 0.0)
        self.declare_parameter("dock_y", 0.0)
        self.declare_parameter("dock_yaw", 0.0)
        self.declare_parameter("map_frame", "lidar3d_map")
        self.declare_parameter("room_prompt", ROOM_PROMPT)
        self.declare_parameter("verify_prompt", VERIFY_PROMPT)
        self.declare_parameter("blocked_action", "sidestep")  # sidestep | dock
        self.declare_parameter("sidestep_cooldown", 20.0)  # s after a sidestep
        self.declare_parameter("sidestep_timeout", 20.0)  # s to wait for recovery
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.room_prompt = gp("room_prompt")
        self.pending_room = None  # room id waiting for a name suggestion
        self.verify_prompt = gp("verify_prompt")
        self.pending_verify = None  # request id from go2_stuck_recovery.py
        self.blocked_action = gp("blocked_action")
        self.sidestep_cooldown = float(gp("sidestep_cooldown"))
        self.sidestep_timeout = float(gp("sidestep_timeout"))
        self.sidestep_done = threading.Event()
        self.sidestep_result = None
        self.manual = False  # Drive pad in control: no automatic action
        self.last_image = None  # newest frame, kept for room requests

        self.act = bool(gp("act_on_blocked"))
        self.confirmations = max(1, int(gp("blocked_confirmations")))
        self.cooldown = float(gp("cooldown"))
        self.dock = (float(gp("dock_x")), float(gp("dock_y")), float(gp("dock_yaw")))
        self.map_frame = gp("map_frame")
        self.blocked_count = 0
        self.last_action = -math.inf

        self.url, self.model, self.prompt = gp("url"), gp("model"), gp("prompt")
        self.timeout = float(gp("timeout"))
        self.max_width = int(gp("max_width"))
        self.image = None
        self.busy = False
        self.lock = threading.Lock()

        self.pub = self.create_publisher(String, "/vlm_decision", 10)
        self.raw_pub = self.create_publisher(String, "/vlm_decision/raw", 10)
        self.resume_pub = self.create_publisher(Bool, "/explore/resume", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.room_pub = self.create_publisher(String, "/vlm_room_suggestion", 10)
        self.verify_pub = self.create_publisher(String, "/vlm_verify_result", 10)
        self.sidestep_pub = self.create_publisher(String, "/recovery/sidestep", 10)
        self.create_subscription(String, "/vlm_room_request", self.on_room_request, 10)
        self.create_subscription(
            String, "/vlm_verify_request", self.on_verify_request, 10
        )
        self.create_subscription(
            String, "/recovery/sidestep_result", self.on_sidestep_result, 10
        )
        self.create_subscription(String, "/teleop/status", self.on_teleop, 10)
        self.create_subscription(Image, gp("image_topic"), self.on_image, IMAGE_QOS)
        self.create_timer(float(gp("period")), self.tick)
        self.get_logger().info(
            f'VLM {self.model} at {self.url} every >= {gp("period")} s '
            f'on {gp("image_topic")}'
        )
        if self.act:
            first = (
                "sidestep round it, else " if self.blocked_action == "sidestep" else ""
            )
            self.get_logger().info(
                f"{self.confirmations} BLOCKED in a row -> {first}pause explorer, dock at "
                f"({self.dock[0]:.2f}, {self.dock[1]:.2f}) in {self.map_frame}"
            )

    def on_image(self, msg):
        self.image = msg
        self.last_image = msg

    def on_room_request(self, msg):
        """Dashboard opened 'Name this area' for a room: answer before routine checks."""
        self.pending_room = msg.data.strip() or None
        self.tick()

    def on_verify_request(self, msg):
        """go2_stuck_recovery.py: is the obstacle the LiDAR reports ahead real?"""
        try:
            self.pending_verify = json.loads(msg.data).get("id")
        except (ValueError, AttributeError):
            self.pending_verify = msg.data.strip() or "?"
        self.tick()

    def on_sidestep_result(self, msg):
        try:
            self.sidestep_result = json.loads(msg.data)
        except ValueError:
            return
        self.sidestep_done.set()

    def on_teleop(self, msg):
        try:
            self.manual = bool(json.loads(msg.data).get("active"))
        except ValueError:
            pass

    def tick(self):
        with self.lock:
            if self.busy:
                return
            room, verify = self.pending_room, self.pending_verify
            if verify is not None and self.last_image is not None:
                self.pending_verify, room = None, None
                msg = self.last_image
            elif room and self.last_image is not None:
                self.pending_room, verify = None, None
                msg = self.last_image
            elif self.image is not None:
                room, verify = None, None
                msg, self.image = self.image, None
            else:
                return
            self.busy = True
        threading.Thread(target=self.ask, args=(msg, room, verify), daemon=True).start()

    def jpeg(self, msg):
        """The frame as a downscaled JPEG, or None."""
        img = image_bgr(msg)
        if img is None:
            self.get_logger().warn(
                f"unsupported image encoding {msg.encoding!r}",
                throttle_duration_sec=60.0,
            )
            return None
        if img.shape[1] > self.max_width:
            h = int(img.shape[0] * self.max_width / img.shape[1])
            img = cv2.resize(img, (self.max_width, h), interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpg if ok else None

    def query(self, prompt, jpg):
        """The model's answer about one JPEG (raises on HTTP / network errors)."""
        body = json.dumps(
            {
                "model": self.model,
                "prompt": prompt,
                "stream": False,
                "images": [base64.b64encode(jpg.tobytes()).decode("ascii")],
            }
        ).encode()
        req = urllib.request.Request(
            self.url, data=body, headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.loads(r.read().decode()).get("response", "").strip()

    def ask(self, msg, room=None, verify=None):
        try:
            jpg = self.jpeg(msg)
            if jpg is None:
                return
            t0 = time.monotonic()
            if verify is not None:
                answer = self.query(self.verify_prompt, jpg)
                decision, side = verify_answer(answer)
                self.verify_pub.publish(
                    String(
                        data=json.dumps(
                            {
                                "id": verify,
                                "decision": decision,
                                "side": side,
                                "raw": answer[:200],
                            }
                        )
                    )
                )
                self.get_logger().info(
                    f"lidar check {verify}: {decision}{' ' + side if side else ''} "
                    f"({time.monotonic() - t0:.1f} s): {answer[:120]!r}"
                )
                return
            answer = self.query(self.room_prompt if room else self.prompt, jpg)
            if room:
                name = room_name(answer)
                self.room_pub.publish(
                    String(
                        data=json.dumps(
                            {"room_id": room, "suggestion": name, "raw": answer[:200]}
                        )
                    )
                )
                self.get_logger().info(
                    f"room {room}: suggest {name!r} ({time.monotonic() - t0:.1f} s)"
                )
                return
            decision = decide(answer)
            self.pub.publish(String(data=decision))
            self.raw_pub.publish(String(data=answer))
            self.get_logger().info(
                f"{decision} ({time.monotonic() - t0:.1f} s): {answer[:120]!r}"
            )
            self.on_decision(decision, jpg)
        except urllib.error.HTTPError as e:
            detail = e.read().decode(errors="replace")[:200]
            self.get_logger().warn(
                f"Ollama HTTP {e.code}: {detail} "
                f"(model pulled? ollama pull {self.model})",
                throttle_duration_sec=30.0,
            )
        except (urllib.error.URLError, OSError, TimeoutError) as e:
            self.get_logger().warn(
                f"Ollama not reachable at {self.url}: {e}", throttle_duration_sec=30.0
            )
        except (ValueError, KeyError) as e:
            self.get_logger().warn(f"bad Ollama reply: {e}", throttle_duration_sec=30.0)
        finally:
            with self.lock:
                self.busy = False

    # ------------------------------------------------------------ action
    def on_decision(self, decision, jpg=None):
        """Debounced BLOCKED -> step round it (VLM picks the side), else dock.

        Runs in the request's worker thread, so waiting here only holds back
        the next routine check.
        """
        if not self.act or self.manual:
            self.blocked_count = 0
            return
        if decision != "BLOCKED":
            self.blocked_count = 0
            return
        self.blocked_count += 1
        now = time.monotonic()
        if self.blocked_count < self.confirmations:
            self.get_logger().info(
                f"BLOCKED {self.blocked_count}/{self.confirmations}, waiting to confirm"
            )
            return
        if now - self.last_action < self.cooldown:
            return
        self.last_action = now
        self.blocked_count = 0
        if self.blocked_action == "sidestep" and jpg is not None:
            outcome = self.sidestep(jpg)
            if outcome != "failed":
                # Stepped, or nothing to do: the next action may come sooner
                # than after docking.
                self.last_action = (
                    time.monotonic() - self.cooldown + self.sidestep_cooldown
                )
                return
        self.get_logger().warn(
            "path BLOCKED (confirmed): pausing the explorer, returning to the dock"
        )
        self.resume_pub.publish(Bool(data=False))
        time.sleep(1.0)  # worker thread: let the explorer finish its stop first
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = self.dock[0], self.dock[1]
        goal.pose.orientation.z = math.sin(self.dock[2] / 2.0)
        goal.pose.orientation.w = math.cos(self.dock[2] / 2.0)
        self.goal_pub.publish(goal)

    def sidestep(self, jpg):
        """Ask again (and which side is freer), have go2_stuck_recovery.py step there.

        "stepped"; "skipped" (the second look does not agree, the dog is not
        navigating, or it already stepped round enough): nothing happens;
        "failed" (dock instead): no side free in the costmap, or no answer.
        """
        try:
            answer = self.query(self.verify_prompt, jpg)
        except (urllib.error.URLError, OSError, TimeoutError, ValueError) as e:
            self.get_logger().warn(f"side question failed: {e}")
            answer = ""
        decision, side = verify_answer(answer)
        if decision != "BLOCKED":
            # llava calls a corridor of cabinets "blocked by a desk" more often
            # than not; acting needs two different prompts to agree.
            self.get_logger().info(
                f"BLOCKED not confirmed by the second look ({decision}, "
                f"{answer[:80]!r}): not acting"
            )
            return "skipped"
        side = side or "auto"
        self.get_logger().warn(
            f"path BLOCKED (confirmed): sidestepping {side} "
            f"(second look: {decision}, {answer[:80]!r})"
        )
        self.sidestep_done.clear()
        self.sidestep_result = None
        self.sidestep_pub.publish(
            String(data=json.dumps({"side": side, "source": "vlm"}))
        )
        if not self.sidestep_done.wait(self.sidestep_timeout):
            self.get_logger().warn(
                "no answer from go2_stuck_recovery.py (not running?)"
            )
            return "failed"
        r = self.sidestep_result or {}
        if r.get("skip"):
            self.get_logger().info(f"no sidestep: {r.get('note', '?')}")
            return "skipped"
        if not r.get("ok"):
            self.get_logger().warn(f"sidestep not possible: {r.get('note', '?')}")
            return "failed"
        self.get_logger().info(f"stepped {r.get('side')}: Nav2 re-plans from there")
        return "stepped"


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


if __name__ == "__main__":
    main()
