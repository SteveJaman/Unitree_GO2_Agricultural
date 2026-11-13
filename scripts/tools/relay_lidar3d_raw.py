#!/usr/bin/env python3
"""
relay_lidar3d_raw.py: let the dog's relay hand Point-LIO's topics to the levelling node.

With GO2_LIDAR3D_RAW=1 (wifi_3d_nav.sh sets it on the real dog), go2-nav2-wifi's relay on
the dog publishes /lidar3d/odom, /lidar3d/registered and /lidar3d/path to this computer
as /lidar3d/raw/odom, /lidar3d/raw/registered and /lidar3d/raw/path, and
scripts/nodes/go2_lidar3d_level.py republishes them, levelled with the body IMU, under
their usual names. Without it (or GO2_LIDAR3D_RAW=0) nothing changes.

  relay_lidar3d_raw.py NAV2_WIFI_DIR          # apply (idempotent)
  relay_lidar3d_raw.py --check NAV2_WIFI_DIR

Edits (tagged TAG; originals kept as <file>.pre-raw):
  * ws/scripts/robot_relay_wifi.py: the relay publisher (it runs on the dog, copied
    there by mapping.sh every session) renames the three topics when GO2_LIDAR3D_RAW=1.
  * mapping.sh: passes GO2_LIDAR3D_RAW to the dog (ssh does not pass the environment).
Run by wifi_nav.sh before every session, like mapping_stop_timeouts.py.
"""

import os
import shutil
import sys

TAG = "  # go2-agri: lidar3d raw"

RELAY = os.path.join("ws", "scripts", "robot_relay_wifi.py")
MAPPING = "mapping.sh"

EDITS = {
    RELAY: [(
        "            pubs.append(node.create_publisher(msg_type, topic, _qos(qos_kind)))\n",
        '            if os.environ.get("GO2_LIDAR3D_RAW") == "1" and topic in (' + TAG + "\n"
        '                    "/lidar3d/odom", "/lidar3d/registered", "/lidar3d/path"):' + TAG + "\n"
        '                topic = "/lidar3d/raw/" + topic.rsplit("/", 1)[1]' + TAG + "\n"
        "            pubs.append(node.create_publisher(msg_type, topic, _qos(qos_kind)))\n",
    )],
    MAPPING: [(
        '  "exec env GO2_HOST_IP=$robot_host_ip GO2_RELAY_DOMAIN_ID=$relay_domain ',
        '  "exec env GO2_LIDAR3D_RAW=${GO2_LIDAR3D_RAW:-0} GO2_HOST_IP=$robot_host_ip GO2_RELAY_DOMAIN_ID=$relay_domain ',
    )],
}


def patch(root, rel, edits, check):
    path = os.path.join(root, rel)
    if not os.path.isfile(path):
        print(f"[raw] {path} not found")
        return 1
    text = open(path).read()
    todo = [(a, b) for a, b in edits if b not in text]
    if not todo:
        return 0
    missing = [a for a, _ in todo if text.count(a) != 1]
    if missing:
        print(f"[raw] WARN: {rel} changed upstream; not applied for:")
        for a in missing:
            print("         " + a.strip().splitlines()[0][:90])
        return 1
    if check:
        print(f"[raw] {rel}: MISSING ({len(todo)} edits)")
        return 1
    backup = path + ".pre-raw"
    if not os.path.exists(backup):
        shutil.copy2(path, backup)
    for a, b in todo:
        text = text.replace(a, b)
    # New file + rename: a running bash reads its script as it goes.
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    shutil.copymode(path, tmp)
    os.replace(tmp, path)
    print(f"[raw] {rel}: {len(todo)} edits (original {os.path.basename(backup)})")
    return 0


def main():
    args = sys.argv[1:]
    check = "--check" in args
    args = [a for a in args if a != "--check"]
    if len(args) != 1:
        sys.exit(__doc__)
    bad = sum(patch(args[0], rel, edits, check) for rel, edits in EDITS.items())
    if not bad:
        print("[raw] relay /lidar3d/raw/* (GO2_LIDAR3D_RAW) .... OK")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
