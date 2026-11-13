#!/usr/bin/env python3
"""
walk_onboard.py: walk the Go2 straight ahead for a fixed time, run ON THE DOG.

For step-clearance tests where the laptop's Wi-Fi link is unreliable: the walk and
its stop happen on the robot, so a Wi-Fi drop cannot leave it walking. Publishes
sport Move requests (the same /api/sport/request the session's robot_sport_bridge.py
uses) at 20 Hz for --duration s, then StopMove. SIGINT / SIGTERM send StopMove at once.
Run it with no session up (nothing else commanding the dog), the dog standing and
someone holding the remote. Source ~/go2-nav2-lidar3d-onboard/robot-source-unitree-ros.sh.

  python3 walk_onboard.py --speed 0.15 --duration 10              # current walk mode
  python3 walk_onboard.py --mode classic --speed 0.15 --duration 10
  python3 walk_onboard.py --mode free --stop-only                 # switch mode, no walking
  python3 walk_onboard.py --mode classic --avoid --speed 0.15 --duration 10
  python3 walk_onboard.py --avoid --stop-only     # avoidance on and off again, no walking

--avoid walks through the firmware's obstacle-avoidance service (the remote's
"Free Avoid", /api/obstacles_avoid/request) instead of plain sport Move: it switches
avoidance on, takes the walk commands from the API, and sends the same velocity there;
the dog itself slows, side-steps or backs off from what its lidar sees. At the end it
switches avoidance off again. Every service reply (code 0 = ok) is printed.
"""

import argparse
import json
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response

API_STOP_MOVE, API_MOVE, API_FREE_WALK, API_CLASSIC_WALK = 1003, 1008, 2045, 2049
MAX_SPEED, MAX_DURATION = 0.30, 20.0
# Obstacle-avoidance service (unitree_sdk2 obstacles_avoid_api): switch, move, take API commands.
AVOID_SWITCH_SET, AVOID_SWITCH_GET, AVOID_MOVE, AVOID_USE_API = 1001, 1002, 1003, 1004
STOP = {"why": None}


def on_signal(signum, _frame):
    STOP["why"] = signal.Signals(signum).name


def request(api_id, parameter=None, reply=False):
    r = Request()
    r.header.identity.id = int(time.time() * 1000) % 2**31
    r.header.identity.api_id = api_id
    r.header.policy.noreply = not reply
    if parameter is not None:
        r.parameter = parameter
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=0.15, help="m/s forward (max 0.30)")
    ap.add_argument("--duration", type=float, default=10.0, help="s (max 20)")
    ap.add_argument("--mode", choices=["keep", "free", "classic"], default="keep",
                    help="walk mode to switch to first, the dog standing still")
    ap.add_argument("--avoid", action="store_true", help="walk through the firmware obstacle avoidance")
    ap.add_argument("--stop-only", action="store_true", help="only switch the mode / stop; no walking")
    args = ap.parse_args()
    if not 0.0 < args.speed <= MAX_SPEED or not 0.0 < args.duration <= MAX_DURATION:
        sys.exit("--speed 0..%.2f m/s and --duration 0..%.0f s" % (MAX_SPEED, MAX_DURATION))
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    def log(msg):
        print("[walk_onboard %s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)

    rclpy.init()
    node = Node("walk_onboard")
    pub = node.create_publisher(Request, "/api/sport/request", 10)
    avoid = node.create_publisher(Request, "/api/obstacles_avoid/request", 10)
    replies = []
    node.create_subscription(Response, "/api/obstacles_avoid/response",
                             lambda m: replies.append(m), 10)

    def avoid_call(api_id, parameter, wait=2.0):
        avoid.publish(request(api_id, parameter, reply=True))
        end = time.monotonic() + wait
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            for m in list(replies):
                if m.header.identity.api_id == api_id:
                    replies.remove(m)
                    log("avoid api %d -> code %d %s" % (api_id, m.header.status.code, m.data))
                    return m.header.status.code
        log("avoid api %d -> no reply in %.0f s" % (api_id, wait))
        return None
    end = time.monotonic() + 5.0
    while pub.get_subscription_count() == 0 and time.monotonic() < end:
        time.sleep(0.1)
    if pub.get_subscription_count() == 0:
        print("no sport service on /api/sport/request: nothing sent", flush=True)
        return 1

    pub.publish(request(API_STOP_MOVE))
    time.sleep(0.3)
    if args.mode == "free":
        pub.publish(request(API_FREE_WALK))
        log("FreeWalk")
        time.sleep(1.5)
    elif args.mode == "classic":
        pub.publish(request(API_CLASSIC_WALK, json.dumps({"data": True})))
        log("ClassicWalk(true)")
        time.sleep(1.5)
    if args.avoid:
        avoid_call(AVOID_SWITCH_GET, "{}")
        if avoid_call(AVOID_SWITCH_SET, json.dumps({"enable": True})) != 0:
            log("obstacle avoidance did not switch on: not walking")
            return 1
        avoid_call(AVOID_USE_API, json.dumps({"is_remote_commands_from_api": True}))
    if args.stop_only or STOP["why"]:
        pub.publish(request(API_STOP_MOVE))
        if args.avoid:
            avoid_call(AVOID_USE_API, json.dumps({"is_remote_commands_from_api": False}))
            avoid_call(AVOID_SWITCH_SET, json.dumps({"enable": False}))
        return 0

    log("walking %.2f m/s for %.1f s (about %.1f m)" % (args.speed, args.duration, args.speed * args.duration))
    move = json.dumps({"x": args.speed, "y": 0.0, "z": 0.0})
    still = json.dumps({"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": 0})
    start = time.monotonic()
    while time.monotonic() - start < args.duration and not STOP["why"]:
        if args.avoid:
            avoid.publish(request(AVOID_MOVE, json.dumps({"x": args.speed, "y": 0.0, "yaw": 0.0, "mode": 0})))
        else:
            pub.publish(request(API_MOVE, move))
        time.sleep(0.05)
    for _ in range(5):
        if args.avoid:
            avoid.publish(request(AVOID_MOVE, still))
        pub.publish(request(API_STOP_MOVE))
        time.sleep(0.05)
    if args.avoid:
        avoid_call(AVOID_USE_API, json.dumps({"is_remote_commands_from_api": False}))
        avoid_call(AVOID_SWITCH_SET, json.dumps({"enable": False}))
    log("stopped (%s) after %.1f s" % (STOP["why"] or "done", time.monotonic() - start))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
