#!/usr/bin/env python3
"""
walk_straight.py: walk the dog straight ahead at a fixed slow speed, then stop.

For step-clearance tests: it publishes /cmd_vel directly (no Nav2, so the dog walks
over test objects instead of planning around them) at 20 Hz for --duration s, then
zero. On SIGINT / SIGTERM it sends zero at once and exits; only this process stops.
The dog-side bridge stops the dog by itself 0.5 s after /cmd_vel goes quiet.

Run inside go2-lidar3d with the session's DDS settings, detached:
  docker exec -d go2-lidar3d bash -c 'source /opt/ros/humble/setup.bash; source /tmp/go2-trav/env.sh; \
      exec setsid python3 -u /tmp/walk_straight.py --speed 0.15 --duration 8 > /tmp/walk.log 2>&1'
Abort: docker exec go2-lidar3d pkill -INT -f walk_straight.py
"""

import argparse
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions

MAX_SPEED, MAX_DURATION = 0.30, 20.0
STOP = {"why": None}


def on_signal(signum, _frame):
    STOP["why"] = signal.Signals(signum).name


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=0.15, help="m/s forward (max 0.30)")
    ap.add_argument("--duration", type=float, default=8.0, help="s of walking (max 20)")
    args = ap.parse_args()
    if not 0.0 < args.speed <= MAX_SPEED or not 0.0 < args.duration <= MAX_DURATION:
        sys.exit(f"--speed 0..{MAX_SPEED} m/s and --duration 0..{MAX_DURATION} s")

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    node = rclpy.create_node("walk_straight")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    def log(msg):
        print(f"[walk_straight {time.strftime('%H:%M:%S')}] {msg}", flush=True)

    end = time.monotonic() + 3.0
    while pub.get_subscription_count() == 0 and time.monotonic() < end and not STOP["why"]:
        time.sleep(0.1)
    if pub.get_subscription_count() == 0:
        log("nobody subscribes to /cmd_vel (session down?). Nothing sent.")
        return 1
    go = Twist()
    go.linear.x = args.speed
    log(f"walking {args.speed:.2f} m/s for {args.duration:.1f} s (about {args.speed * args.duration:.1f} m)")
    start = time.monotonic()
    while time.monotonic() - start < args.duration and not STOP["why"]:
        pub.publish(go)
        time.sleep(0.05)
    for _ in range(10):  # zero, a few times
        pub.publish(Twist())
        time.sleep(0.05)
    log(f"stopped ({STOP['why'] or 'done'}) after {time.monotonic() - start:.1f} s")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
