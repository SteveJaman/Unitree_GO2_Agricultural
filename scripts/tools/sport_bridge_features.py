#!/usr/bin/env python3
"""
sport_bridge_features.py: automatic fall recovery and the firmware's obstacle
avoidance for go2-nav2-wifi's dog-side sport bridge (ws/scripts/robot_sport_bridge.py,
copied to the dog by mapping.sh every session).

  * Fall recovery (GO2_SPORT_AUTO_RECOVERY, default 1): the bridge sends sport
    AutoRecoverySet(true) (api 2054) at start; the firmware then stands the dog
    back up by itself after a fall (the remote's L2 + X, done automatically).
  * Obstacle avoidance (GO2_SPORT_AVOID, default 0; wifi_3d_nav.sh --avoid): the
    remote's "Free Avoid". The bridge switches the obstacles_avoid service on, has
    it take its walk commands from the API, and sends Nav2's velocity there
    (/api/obstacles_avoid/request, Move) instead of sport Move: the dog slows,
    steers or stops for what its own L1 lidar sees ahead. The manual: forward
    obstacles only, it does not back up. At the end it is switched off again; without
    --avoid it is switched off at the start (the switch survives a reboot: a session that
    died without its cleanup left it on, 2026-11-12).
    Every reply of that service, and of AutoRecoverySet, is logged (code 0 = ok).

  sport_bridge_features.py NAV2_WIFI_DIR          # apply (idempotent)
  sport_bridge_features.py --check NAV2_WIFI_DIR

Edits tagged with TAG; the original is kept as robot_sport_bridge.py.pre-features.
Run by wifi_nav.sh before every session, like mapping_stop_timeouts.py (which
passes the two variables to the dog: ssh does not pass the environment).
"""

import os
import shutil
import sys

TAG = "  # go2-agri: recovery/avoid"
FILE = os.path.join("ws", "scripts", "robot_sport_bridge.py")

METHODS = '''    def _init_features(self) -> None:{t}
        """Fall recovery and obstacle avoidance (sport_bridge_features.py)."""{t}
        if self._auto_recovery:{t}
            self._reply_call(self._pub, API_AUTORECOVERY_SET, {{"data": True}}){t}
            self.get_logger().info("AutoRecoverySet(true): the dog stands up by itself after a fall"){t}
        if self._avoid:{t}
            self._reply_call(self._avoid_pub, AVOID_SWITCH_SET, {{"enable": True}}){t}
            time.sleep(0.3){t}
            self._reply_call(self._avoid_pub, AVOID_USE_API, {{"is_remote_commands_from_api": True}}){t}
            self.get_logger().info("Obstacle avoidance on: walking through /api/obstacles_avoid (forward only)"){t}
        else:{t}
            # Off unless asked for: the switch survives a reboot, so a session that died{t}
            # without its cleanup would otherwise leave it on (seen 2026-11-12).{t}
            self._reply_call(self._avoid_pub, AVOID_USE_API, {{"is_remote_commands_from_api": False}}){t}
            self._reply_call(self._avoid_pub, AVOID_SWITCH_SET, {{"enable": False}}){t}
        time.sleep(0.3){t}
{t}
    def _reply_call(self, pub, api_id: int, data: dict) -> None:{t}
        req = make_request(api_id, json.dumps(data)){t}
        req.header.identity.id = int(time.time() * 1000) % 2**31{t}
        req.header.policy.noreply = False{t}
        self._sent_ids.add(req.header.identity.id){t}
        pub.publish(req){t}
{t}
    def _on_reply(self, msg) -> None:{t}
        # Only replies to this bridge's own requests: other clients on the dog poll{t}
        # the avoidance switch (api 1002) about 10 times a second.{t}
        api = msg.header.identity.api_id{t}
        if msg.header.identity.id in self._sent_ids:{t}
            self._sent_ids.discard(msg.header.identity.id){t}
            code = msg.header.status.code{t}
            log = self.get_logger().info if code == 0 else self.get_logger().warn{t}
            log(f"reply api {{api}}: code {{code}}{{' (ok)' if code == 0 else ' (refused)'}} {{msg.data}}"){t}
{t}
    def _move(self, vx: float, vy: float, vyaw: float) -> None:{t}
        if self._avoid:{t}
            self._avoid_pub.publish(make_request(AVOID_MOVE, json.dumps({{"x": vx, "y": vy, "yaw": vyaw, "mode": 0}}))){t}
        else:{t}
            self._send(API_MOVE, json.dumps({{"x": vx, "y": vy, "z": vyaw}})){t}
{t}
    def _halt(self, final: bool = False) -> None:{t}
        if self._avoid:{t}
            self._avoid_pub.publish(make_request(AVOID_MOVE, json.dumps({{"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": 0}}))){t}
            if final:{t}
                self._reply_call(self._avoid_pub, AVOID_USE_API, {{"is_remote_commands_from_api": False}}){t}
                self._reply_call(self._avoid_pub, AVOID_SWITCH_SET, {{"enable": False}}){t}
                self.get_logger().info("Obstacle avoidance off"){t}
        self._send(API_STOP_MOVE){t}
{t}
    def _on_cmd_vel(self, msg: Twist) -> None:
'''.format(t=TAG)

EDITS = [
    ("from unitree_api.msg import Request\n",
     "from unitree_api.msg import Request, Response" + TAG + "\n"),
    ('TOPIC_REQUEST = "/api/sport/request"\n',
     'TOPIC_REQUEST = "/api/sport/request"\n'
     "API_AUTORECOVERY_SET = 2054" + TAG + "\n"
     "# Firmware obstacle avoidance (unitree_sdk2 obstacles_avoid_api)." + TAG + "\n"
     "AVOID_SWITCH_SET, AVOID_SWITCH_GET, AVOID_MOVE, AVOID_USE_API = 1001, 1002, 1003, 1004" + TAG + "\n"
     'TOPIC_AVOID = "/api/obstacles_avoid/request"' + TAG + "\n"),
    ('        walk_mode = os.environ.get("GO2_SPORT_WALK_MODE", "free").lower()\n',
     '        walk_mode = os.environ.get("GO2_SPORT_WALK_MODE", "free").lower()\n'
     '        self._auto_recovery = os.environ.get("GO2_SPORT_AUTO_RECOVERY", "1") != "0"' + TAG + "\n"
     '        self._avoid = os.environ.get("GO2_SPORT_AVOID", "0") not in ("", "0")' + TAG + "\n"
     "        self._sent_ids = set()" + TAG + "\n"
     "        self._avoid_pub = self.create_publisher(Request, TOPIC_AVOID, 10)" + TAG + "\n"
     '        self.create_subscription(Response, "/api/obstacles_avoid/response", self._on_reply, 10)' + TAG + "\n"
     '        self.create_subscription(Response, "/api/sport/response", self._on_reply, 10)' + TAG + "\n"),
    ('            self._send(API_FREE_WALK)\n'
     '            self.get_logger().info("FreeWalk")\n'
     '        time.sleep(0.4)\n',
     '            self._send(API_FREE_WALK)\n'
     '            self.get_logger().info("FreeWalk")\n'
     '        time.sleep(0.4)\n'
     "        self._init_features()" + TAG + "\n"),
    ("    def _on_cmd_vel(self, msg: Twist) -> None:\n", METHODS),
    ("                self._send(API_STOP_MOVE)\n                self._stopped = True\n",
     "                self._halt()" + TAG + "\n                self._stopped = True\n"),
    ('        self._send(API_MOVE, json.dumps({"x": vx, "y": vy, "z": vyaw}))\n',
     "        self._move(vx, vy, vyaw)" + TAG + "\n"),
    ("                node._send(API_STOP_MOVE)\n",
     "                node._halt(final=True)" + TAG + "\n"),
]


def main():
    args = sys.argv[1:]
    check = "--check" in args
    args = [a for a in args if a != "--check"]
    if len(args) != 1:
        sys.exit(__doc__)
    path = os.path.join(args[0], FILE)
    if not os.path.isfile(path):
        sys.exit(f"[sport] {path} not found")
    text = open(path).read()
    todo = [(a, b) for a, b in EDITS if b not in text]
    backup = path + ".pre-features"
    if todo and TAG in text and os.path.exists(backup):
        # An earlier version of these edits: redo them all from the original.
        text = open(backup).read()
        todo = list(EDITS)
    if not todo:
        print("[sport] fall recovery / obstacle avoidance ... OK")
        return 0
    missing = [a for a, _ in todo if text.count(a) != 1]
    if missing:
        print("[sport] WARN: robot_sport_bridge.py changed upstream; not applied for:")
        for a in missing:
            print("         " + a.strip().splitlines()[0][:90])
        return 1
    if check:
        print(f"[sport] fall recovery / obstacle avoidance ... MISSING ({len(todo)} edits)")
        return 1
    if not os.path.exists(backup):
        shutil.copy2(path, backup)
    for a, b in todo:
        text = text.replace(a, b)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    shutil.copymode(path, tmp)
    os.replace(tmp, path)
    print(f"[sport] robot_sport_bridge.py: fall recovery / obstacle avoidance added ({len(todo)} edits; "
          f"original {os.path.basename(backup)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
