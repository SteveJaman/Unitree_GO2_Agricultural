#!/usr/bin/env python3
"""Label points clicked in Foxglove and save them (wifi_3d_nav.sh --save-location).

Interactive: run it in its own terminal while a session is up. Each point
published from Foxglove (Publish -> Point, /clicked_point) is transformed into
map_frame, printed, and you are asked for a label ("Dock",
"Hydroponic_Station", ...). Empty input skips the point; reusing a label
replaces it. Saved to config/semantic_locations.json:

  {"Dock": {"x": 0.0, "y": 0.0, "z": 0.0, "frame": "lidar3d_map",
            "rtabmap_map": {"x": ..., "y": ..., "z": ...},  # if that TF exists
            "saved": "2026-10-23T14:02:11"}}

lidar3d_map starts where the dog stood at session start, so saved points line
up with a later session only if the dog starts from the same spot. The
rtabmap_map copy is stable across sessions of one RTAB-Map database once
RTAB-Map has linked them (wifi_3d_nav.sh --resume).

go2_click_to_goal also listens to /clicked_point, so a clicked point is driven
to as well. To label without driving, set Foxglove's Publish topic to
/label_point and run with: --ros-args -p topic:=/label_point

Subscribes: /clicked_point (geometry_msgs/PointStamped), TF
"""
import datetime
import json
import os
import queue
import tempfile
import threading

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2)
from tf2_ros import Buffer, TransformException, TransformListener


class SaveLocation(Node):
    def __init__(self):
        super().__init__('save_location')
        self.declare_parameter('topic', '/clicked_point')
        self.declare_parameter('map_frame', 'lidar3d_map')
        self.declare_parameter('file', '/config/semantic_locations.json')
        self.map_frame = self.get_parameter('map_frame').value
        self.file = self.get_parameter('file').value
        self.topic = self.get_parameter('topic').value
        self.points = queue.Queue()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PointStamped, self.topic, self.points.put, 10)

    def to_frame(self, msg, frame):
        msg.header.stamp = Time().to_msg()   # UI event: newest TF
        try:
            p = self.tf_buffer.transform(msg, frame, timeout=Duration(seconds=0.5)).point
            return {'x': round(p.x, 3), 'y': round(p.y, 3), 'z': round(p.z, 3)}
        except TransformException:
            return None


def load(path):
    try:
        with open(path) as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except FileNotFoundError:
        return {}
    except (OSError, ValueError) as e:
        raise SystemExit(f'cannot read {path}: {e} (fix or move it, then retry)')


def save(path, data):
    """Write atomically, so a crash never leaves half a file."""
    d = os.path.dirname(path) or '.'
    os.makedirs(d, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=d, prefix='.semantic_locations.')
    with os.fdopen(fd, 'w') as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write('\n')
    os.replace(tmp, path)


def main():
    rclpy.init()
    node = SaveLocation()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    data = load(node.file)
    print(f'Saving to {node.file} ({len(data)} location(s): {", ".join(sorted(data)) or "none"})')
    print(f'Click a point in Foxglove (Publish -> Point on {node.topic}). Ctrl+C to quit.')
    try:
        while True:
            msg = node.points.get()
            src = msg.header.frame_id
            p = node.to_frame(msg, node.map_frame)
            if p is None:
                print(f'  ! cannot transform the click from {src!r} to {node.map_frame}; skipped')
                continue
            print(f'\nPoint in {node.map_frame}: x={p["x"]:.2f} y={p["y"]:.2f} z={p["z"]:.2f}')
            label = input('  Label (e.g. Dock, Hydroponic_Station; Enter = skip): ').strip()
            if not label:
                print('  skipped')
                continue
            label = label.replace(' ', '_')
            entry = dict(p, frame=node.map_frame,
                         saved=datetime.datetime.now().isoformat(timespec='seconds'))
            rt = node.to_frame(msg, 'rtabmap_map')
            if rt is not None:
                entry['rtabmap_map'] = rt
            replaced = label in data
            data[label] = entry
            save(node.file, data)
            print(f'  {"replaced" if replaced else "saved"} {label!r} -> {node.file}')
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
