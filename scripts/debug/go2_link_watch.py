#!/usr/bin/env python3
"""
go2_link_watch.py: say in the terminal when the dog's data is not reaching
this computer (wifi_nav.sh / wifi_3d_nav.sh, real robot only).

Runs inside go2-lidar3d next to Nav2, on the session's DDS setup. It only
subscribes to /tf and watches lidar3d_map -> base_link, which go2_odom_tf
publishes here from the dog's /lidar3d/odom. /tf is published on this
computer: listening to it costs no Wi-Fi (an extra subscriber to a topic that
comes from the robot would make the robot send it twice).

Why: when the dog -> computer direction of the Wi-Fi stops delivering, Nav2
never gets the dog's position, the costmaps keep waiting, and after ~78 s the
shared SSH link to the robot times out and the session closes RViz with
everything else. On 2026-10-30 every run that got the position within ~15 s
kept going; every run that did not, closed. This makes that visible at once:

  * no position within warn_after (20 s) of the start: a warning that says
    what is wrong and what to do, repeated every remind_every (30 s)
  * the position arrives: one OK line, with how long it took
  * later, no position for stall_after (5 s): a stall warning, then a line
    when it comes back, with the length of the gap

Never publishes, never moves anything. Stops on SIGINT/SIGTERM.
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage


def say(text):
    print(f"[link] {text}", flush=True)


class LinkWatch(Node):
    def __init__(self):
        super().__init__("go2_link_watch")
        p = self.declare_parameter
        p("parent_frame", "lidar3d_map")
        p("child_frame", "base_link")
        p("warn_after", 20.0)
        p("remind_every", 30.0)
        p("stall_after", 5.0)

        def gp(name):
            return self.get_parameter(name).value

        self.parent, self.child = gp("parent_frame"), gp("child_frame")
        self.warn_after, self.remind_every = gp("warn_after"), gp("remind_every")
        self.stall_after = gp("stall_after")

        self.start = time.monotonic()
        self.last = None  # monotonic time of the last dog pose
        self.stalled_since = None
        self.next_warn = self.start + self.warn_after

        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos)
        self.create_timer(1.0, self.tick)

    def on_tf(self, msg):
        for t in msg.transforms:
            if (
                t.header.frame_id.lstrip("/") == self.parent
                and t.child_frame_id.lstrip("/") == self.child
            ):
                break
        else:
            return
        now = time.monotonic()
        if self.last is None:
            say(
                f"Data from the dog arriving after {now - self.start:.0f} s ........ OK"
            )
        elif self.stalled_since is not None:
            say(f"Data from the dog back after a {now - self.stalled_since:.0f} s gap")
        self.last, self.stalled_since = now, None

    def tick(self):
        now = time.monotonic()
        if self.last is None:
            if now >= self.next_warn:
                self.next_warn = now + self.remind_every
                say(f"WARNING: no data from the dog after {now - self.start:.0f} s.")
                say("  Nav2 has no position for the dog, and the session will close")
                say("  (RViz too) once the SSH link to the robot times out (~78 s).")
                say("  The dog -> computer Wi-Fi is not delivering. Press Ctrl+C and")
                say("  start again; an Ethernet cable to the router avoids it.")
            return
        gap = now - self.last
        if self.stalled_since is None and gap >= self.stall_after:
            self.stalled_since = self.last
            say(
                f"WARNING: no data from the dog for {gap:.0f} s (Wi-Fi stall);"
                " the session closes if this lasts ~60 s"
            )


def main():
    rclpy.init()
    node = LinkWatch()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
