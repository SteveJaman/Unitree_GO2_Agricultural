#!/usr/bin/env python3
"""
contact_monitor.py: log every time the simulated Go2 (body, legs, head or the
D1 arm) touches anything but the floor. Runs inside the go2-sim container
next to Gazebo, for tests/sim_benchmark.py.

Needs the sim started with SIM_CONTACTS=1: start_inside.sh then gives every
robot link a contact sensor, adds the Contact system to the world and lists
the sensors' topics (Gazebo's scoped names) in /tmp/sim-gen/contact_topics.txt.

  python3 contact_monitor.py --out /tmp/contacts.jsonl --topics $(cat /tmp/sim-gen/contact_topics.txt)

One JSON line per contact message with a non-floor collision:
  {"t": wall time, "sim_t": sim time, "link": "base", "robot": "<robot collision>",
   "other": "<model>::<link>::<collision>", "x":, "y":, "z":, "depth":}
"""

import argparse
import json
import signal
import sys
import time

from gz.msgs10.contacts_pb2 import Contacts
from gz.transport13 import Node

FLOOR = ("ground::",)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/contacts.jsonl")
    ap.add_argument("--topics", nargs="+", required=True, help="contact sensor topics")
    a = ap.parse_args()
    out = open(a.out, "a", buffering=1)
    node = Node()

    def on_contacts(link):
        def cb(msg):
            sim_t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            for c in msg.contact:
                names = (c.collision1.name, c.collision2.name)
                robot = next((n for n in names if n.startswith("go2::")), names[0])
                other = names[1] if robot == names[0] else names[0]
                if other.startswith(FLOOR) or other.startswith("go2::"):
                    continue
                p = c.position[0] if len(c.position) else None
                out.write(json.dumps({
                    "t": round(time.time(), 3), "sim_t": round(sim_t, 3), "link": link,
                    "robot": robot, "other": other,
                    "x": round(p.x, 3) if p else None, "y": round(p.y, 3) if p else None,
                    "z": round(p.z, 3) if p else None,
                    "depth": round(c.depth[0], 4) if len(c.depth) else None}) + "\n")
        return cb

    for topic in a.topics:
        link = topic.split("/link/")[1].split("/")[0] if "/link/" in topic else topic
        if not node.subscribe(Contacts, topic, on_contacts(link)):
            print(f"[contacts] could not subscribe to {topic}", file=sys.stderr)
    print(f"[contacts] watching {len(a.topics)} links -> {a.out}", flush=True)
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    while not stop:
        time.sleep(0.2)


if __name__ == "__main__":
    main()
