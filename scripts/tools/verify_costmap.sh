#!/usr/bin/env python3
"""
verify_costmap.py — check that /global_costmap/costmap is publishing.

Run on the VM inside the container, or from Windows against a remote
Docker daemon if you have one configured.

Usage:
    python verify_costmap.py
    python verify_costmap.py --topic /global_costmap/costmap
"""
import argparse
import subprocess
import sys

CONTAINER = "go2-lidar3d"


def run(cmd):
    """Run a command inside the container and return stdout."""
    full = ["docker", "exec", "-u", "root", CONTAINER, "bash", "-lc", cmd]
    try:
        result = subprocess.run(
            full, capture_output=True, text=True, timeout=20
        )
        return result.stdout.strip(), result.returncode
    except subprocess.TimeoutExpired:
        return "", 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/global_costmap/costmap")
    ap.add_argument("--container", default=CONTAINER)
    args = ap.parse_args()

    global CONTAINER
    CONTAINER = args.container
    topic = args.topic

    source = (
        "source /opt/ros/humble/setup.bash && "
        "source /ws/install/setup.bash 2>/dev/null && "
    )

    print(f"=== {topic} ===")
    out, rc = run(source + f"timeout 3 ros2 topic info {topic} --verbose 2>/dev/null")
    print(out if out else "  NOT FOUND")

    print(f"\n=== rate ===")
    out, rc = run(source + f"timeout 5 ros2 topic hz {topic} 2>/dev/null")
    print(out if out else "  no messages")

    print(f"\n=== sample ===")
    out, rc = run(source + f"timeout 3 ros2 topic echo {topic} --once 2>/dev/null")
    if out:
        for line in out.splitlines()[:20]:
            print("  " + line)
    else:
        print("  no data")


if __name__ == "__main__":
    main()