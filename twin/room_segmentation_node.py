#!/usr/bin/env python3
"""Room segmentation and room tagging for the 3D twin (robot-vacuum style).

Every `period` s, when the costmap changed, the free space of
/global_costmap/costmap is split into rooms:

  1. free = known cells with cost < free_threshold
  2. seeds = free cells farther than door_width / 2 from any non-free cell
     (an erosion: doorways narrower than door_width disappear, rooms stay),
     connected components bigger than min_room_area
  3. every free cell joins its nearest seed (watershed on the distance
     map), so each room ends at its doorway
  4. polygon = simplified outer contour; goal = the room's most open cell
     (maximum distance to walls: unlike the centroid it is always inside
     the room and on free floor)

Room IDs (room_1, room_2, ...) stay stable while the map grows: a new
segment takes the ID of the previous room it overlaps most. Names and
merges come from the dashboard on /semantic_rooms/command and are saved
with the polygons to config/semantic_rooms.json, which is also reloaded on
start (IDs match a new session if the dog starts at the same dock).

Subscribes: /global_costmap/costmap  (nav_msgs/OccupancyGrid)
            /semantic_rooms/command  (std_msgs/String JSON:
                {"op": "rename", "id": "room_2", "label": "Lab Desk"}
                    (+ "source": "auto", "category", "objects", "vlm_suggestion"
                     from the survey; an auto name never replaces a user name)
                {"op": "merge", "ids": ["room_2", "room_3"]}
                {"op": "unmerge", "id": "room_2"}
                {"op": "resegment"})
Publishes:  /semantic_rooms          (visualization_msgs/MarkerArray, one colour per room)
            /semantic_rooms/json     (std_msgs/String, the room map; latched)
"""
import datetime
import json
import math
import os
import tempfile

import cv2
import numpy as np

PALETTE = [  # distinct, light colours (RGB 0-1)
    (0.35, 0.65, 1.00), (0.40, 0.85, 0.55), (1.00, 0.70, 0.30), (0.85, 0.45, 0.95),
    (1.00, 0.45, 0.50), (0.30, 0.85, 0.85), (0.95, 0.85, 0.35), (0.60, 0.60, 1.00),
]


# ------------------------------------------------------------------ algorithm
def free_mask(data, width, height, free_threshold):
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    return ((grid >= 0) & (grid < free_threshold)).astype(np.uint8)


def segment(free, resolution, door_width=0.9, min_room_area=1.5):
    """Label image (0 = not a room, 1..n = rooms) of a free-space mask."""
    if free.sum() == 0:
        return np.zeros(free.shape, np.int32), 0
    dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * resolution
    # A doorway exactly door_width wide peaks at door_width / 2 in its middle;
    # the 1.5-cell margin keeps rounding from bridging two rooms there.
    seeds = (dist > door_width / 2 + 1.5 * resolution).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(seeds, connectivity=8)
    min_cells = min_room_area / (resolution * resolution)
    markers = np.zeros(free.shape, np.int32)
    k = 0
    for i in range(1, n):
        # The seed is the room shrunk by door_width/2 on every side; compare
        # its area with a lower bound of the whole room.
        if stats[i, cv2.CC_STAT_AREA] >= min_cells * 0.15:
            k += 1
            markers[labels == i] = k
    if k == 0:   # one open space with no room-sized core: a single room
        markers[free > 0] = 1
        return markers, 1
    # Watershed over the inverted distance map: free cells flow to the
    # nearest seed, borders form at the narrow places (doorways).
    # Grow all seeds through free space one cell per step until they meet
    # (geodesic, so nothing leaks through walls): borders fall at doorways,
    # where seeds from both sides meet. Free space no seed can reach stays 0.
    lab = markers.astype(np.float32)
    free_b = free > 0
    kernel = np.ones((3, 3), np.uint8)
    for _ in range(free.shape[0] + free.shape[1]):
        todo = free_b & (lab == 0)
        if not todo.any():
            break
        grown = cv2.dilate(lab, kernel)
        new = todo & (grown > 0)
        if not new.any():
            break
        lab[new] = grown[new]
    ws = lab.astype(np.int32)
    # Drop rooms that ended up too small (corridor stubs): give them away.
    for r in range(1, k + 1):
        if 0 < np.count_nonzero(ws == r) < min_cells:
            ring = cv2.dilate((ws == r).astype(np.uint8), np.ones((3, 3), np.uint8)) & (ws != r)
            neigh = ws[ring > 0]
            neigh = neigh[neigh > 0]
            ws[ws == r] = np.bincount(neigh).argmax() if neigh.size else 0
    return ws, k


def describe(mask, resolution, origin, simplify=0.15):
    """Polygon (world xy), goal point (most open cell), area of a room mask."""
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    c = cv2.approxPolyDP(c, max(1.0, simplify / resolution), True)
    ox, oy = origin
    poly = [[round(float(ox + (p[0][0] + 0.5) * resolution), 3), round(float(oy + (p[0][1] + 0.5) * resolution), 3)] for p in c]
    if len(poly) < 3:
        return None
    dist = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5)
    gy, gx = np.unravel_index(int(np.argmax(dist)), dist.shape)
    return {
        'polygon': poly,
        'center': [round(float(ox + (gx + 0.5) * resolution), 3), round(float(oy + (gy + 0.5) * resolution), 3)],
        'area': round(float(mask.sum()) * resolution * resolution, 2),
    }


def rasterize(poly, shape, resolution, origin):
    """Polygon (world) -> mask in the current grid (for ID matching)."""
    ox, oy = origin
    pts = np.array([[(x - ox) / resolution, (y - oy) / resolution] for x, y in poly], np.int32)
    m = np.zeros(shape, np.uint8)
    if len(pts) >= 3:
        cv2.fillPoly(m, [pts], 1)
    return m


def match_ids(labels, k, previous, shape, resolution, origin, next_id):
    """Map label 1..k -> stable room id via overlap with previous polygons."""
    prev_masks = {rid: rasterize(r['polygon'], shape, resolution, origin)
                  for rid, r in previous.items() if r.get('polygon')}
    taken, ids = set(), {}
    pairs = []
    for lab in range(1, k + 1):
        m = labels == lab
        if not m.any():
            continue
        for rid, pm in prev_masks.items():
            ov = np.count_nonzero(m & (pm > 0))
            if ov:
                pairs.append((ov / max(1, np.count_nonzero(m)), lab, rid))
    for score, lab, rid in sorted(pairs, reverse=True):   # best overlaps first
        if lab not in ids and rid not in taken and score > 0.2:
            ids[lab] = rid
            taken.add(rid)
    for lab in range(1, k + 1):
        if lab not in ids and (labels == lab).any():
            ids[lab] = f'room_{next_id}'
            next_id += 1
    return ids, next_id


def adjacent(a, b, gap_cells=3):
    """True if two room masks touch (within a few cells, i.e. through a doorway)."""
    k = np.ones((2 * gap_cells + 1, 2 * gap_cells + 1), np.uint8)
    return bool((cv2.dilate(a.astype(np.uint8), k) & b.astype(np.uint8)).any())


# ------------------------------------------------------------------ ROS node
def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point

    class RoomSegmentation(Node):
        def __init__(self):
            super().__init__('room_segmentation')
            p = self.declare_parameter
            p('costmap_topic', '/global_costmap/costmap')
            p('file', '/config/semantic_rooms.json')
            p('free_threshold', 50)
            p('door_width', 0.9)        # m: openings narrower than this split rooms
            p('min_room_area', 1.5)     # m^2
            p('period', 10.0)           # s between segmentations
            gp = lambda k: self.get_parameter(k).value  # noqa: E731
            self.file = gp('file')
            self.free_threshold = int(gp('free_threshold'))
            self.door_width = float(gp('door_width'))
            self.min_room_area = float(gp('min_room_area'))

            self.grid = None
            self.dirty = False
            self.masks = {}             # room id -> mask in the last grid
            self.state = self.load()

            latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
            self.marker_pub = self.create_publisher(MarkerArray, '/semantic_rooms', latched)
            self.json_pub = self.create_publisher(String, '/semantic_rooms/json', latched)
            self.create_subscription(OccupancyGrid, gp('costmap_topic'), self.on_grid, latched)
            self.create_subscription(String, '/semantic_rooms/command', self.on_command, 10)
            self.create_timer(float(gp('period')), self.tick)
            self.Marker, self.MarkerArray, self.Point, self.String = Marker, MarkerArray, Point, String
            self.publish()
            self.get_logger().info(
                f'rooms: {gp("costmap_topic")} -> /semantic_rooms, doorways < {self.door_width} m, '
                f'{len(self.state["rooms"])} saved room(s) in {self.file}')

        # ---------------------------------------------------------- state
        def load(self):
            try:
                with open(self.file) as f:
                    s = json.load(f)
                s.setdefault('rooms', {})
                s.setdefault('merges', {})
                s.setdefault('next_id', 1 + max([int(r.split('_')[-1]) for r in s['rooms']] + [0]))
                if isinstance(s['rooms'], list):     # tolerate the list form
                    s['rooms'] = {r['id']: r for r in s['rooms']}
                return s
            except (OSError, ValueError, KeyError):
                return {'frame': 'lidar3d_map', 'rooms': {}, 'merges': {}, 'next_id': 1}

        def save(self):
            out = dict(self.state, updated=datetime.datetime.now().isoformat(timespec='seconds'))
            d = os.path.dirname(self.file) or '.'
            try:
                os.makedirs(d, exist_ok=True)
                fd, tmp = tempfile.mkstemp(dir=d, prefix='.semantic_rooms.')
                with os.fdopen(fd, 'w') as f:
                    json.dump(out, f, indent=2)
                    f.write('\n')
                os.replace(tmp, self.file)
            except OSError as e:
                self.get_logger().warn(f'cannot save {self.file}: {e}', throttle_duration_sec=60.0)

        # ---------------------------------------------------------- inputs
        def on_grid(self, msg):
            self.grid = msg
            self.dirty = True
            if not self.state.get('frame'):
                self.state['frame'] = msg.header.frame_id

        def on_command(self, msg):
            try:
                cmd = json.loads(msg.data)
                op = cmd.get('op')
            except (ValueError, AttributeError):
                self.get_logger().warn(f'bad command: {msg.data!r}')
                return
            rooms, merges = self.state['rooms'], self.state['merges']
            if op == 'rename' and cmd.get('id') in rooms:
                room = rooms[cmd['id']]
                source = cmd.get('source', 'user')
                # The survey names rooms automatically; a name a person typed wins.
                if source == 'auto' and room.get('label') and room.get('label_source', 'user') == 'user':
                    self.get_logger().info(f'{cmd["id"]}: keeping the user name {room["label"]!r}')
                else:
                    room['label'] = str(cmd.get('label', '')).strip()[:60]
                    room['label_source'] = source
                for k in ('category', 'objects', 'vlm_suggestion'):
                    if k in cmd:
                        room[k] = cmd[k]
                self.get_logger().info(f'{cmd["id"]} named {room["label"]!r} ({source})')
            elif op == 'merge':
                ids = [i for i in cmd.get('ids', []) if i in rooms]
                if len(ids) != 2 or ids[0] == ids[1]:
                    return self.reject('merge needs two different rooms')
                a, b = ids
                if a in self.masks and b in self.masks and not adjacent(self.masks[a], self.masks[b]):
                    return self.reject(f'{a} and {b} are not adjacent')
                keep, gone = (a, b) if a < b else (b, a)
                merges[gone] = keep
                for k, v in list(merges.items()):     # re-point chains
                    if v == gone:
                        merges[k] = keep
                if not rooms[keep].get('label') and rooms[gone].get('label'):
                    rooms[keep]['label'] = rooms[gone]['label']
                self.get_logger().info(f'merged {gone} into {keep}')
                self.dirty = True
            elif op == 'unmerge' and cmd.get('id'):
                for k in [k for k, v in merges.items() if v == cmd['id']]:
                    del merges[k]
                self.dirty = True
            elif op == 'resegment':
                self.dirty = True
            else:
                return self.reject(f'unknown command {cmd!r}')
            if self.dirty:
                self.tick()
            else:
                self.save()
                self.publish()

        def reject(self, why):
            self.get_logger().warn(why)
            self.publish(error=why)

        # ---------------------------------------------------------- work
        def tick(self):
            if not self.dirty or self.grid is None:
                return
            self.dirty = False
            g = self.grid
            info = g.info
            res, origin = info.resolution, (info.origin.position.x, info.origin.position.y)
            free = free_mask(g.data, info.width, info.height, self.free_threshold)
            labels, k = segment(free, res, self.door_width, self.min_room_area)
            # Merged rooms are matched as one, so they keep one ID.
            previous = self.merged_view(self.state['rooms'])
            ids, self.state['next_id'] = match_ids(labels, k, previous, labels.shape, res, origin,
                                                   self.state['next_id'])
            merges = self.state['merges']
            masks = {}
            for lab, rid in ids.items():
                rid = merges.get(rid, rid)
                m = labels == lab
                masks[rid] = masks[rid] | m if rid in masks else m
            rooms = self.state['rooms']
            for rid, m in masks.items():
                d = describe(m, res, origin)
                if d is None:
                    continue
                rooms.setdefault(rid, {'id': rid, 'label': ''})
                rooms[rid].update(d, id=rid, seen=True)
            self.masks = masks
            # Rooms not seen in this map keep their last polygon (and name).
            for rid in rooms:
                rooms[rid]['seen'] = rid in masks
            self.save()
            self.publish()

        def merged_view(self, rooms):
            merges = self.state['merges']
            return {rid: r for rid, r in rooms.items() if rid not in merges}

        # ---------------------------------------------------------- output
        def visible_rooms(self):
            merges = self.state['merges']
            out = []
            for rid, r in sorted(self.state['rooms'].items(), key=lambda kv: int(kv[0].split('_')[-1])):
                if rid in merges or not r.get('polygon'):
                    continue
                # Once a map arrived, only rooms in it are shown (the file keeps the rest).
                if self.grid is not None and not r.get('seen', True):
                    continue
                out.append(dict(r, color=PALETTE[(int(rid.split('_')[-1]) - 1) % len(PALETTE)],
                                merged_from=sorted(k for k, v in merges.items() if v == rid)))
            return out

        def publish(self, error=None):
            rooms = self.visible_rooms()
            frame = self.state.get('frame') or 'lidar3d_map'
            self.json_pub.publish(self.String(data=json.dumps({
                'frame': frame, 'rooms': rooms, 'error': error,
                'updated': datetime.datetime.now().isoformat(timespec='seconds')})))
            arr = self.MarkerArray()
            clear = self.Marker()
            clear.header.frame_id = frame
            clear.action = self.Marker.DELETEALL
            arr.markers.append(clear)
            for i, r in enumerate(rooms):
                m = self.Marker()
                m.header.frame_id = frame
                m.ns, m.id = 'rooms', 2 * i
                m.type, m.action = self.Marker.LINE_STRIP, self.Marker.ADD
                m.scale.x = 0.05
                m.pose.orientation.w = 1.0
                (m.color.r, m.color.g, m.color.b), m.color.a = r['color'], 0.9
                m.points = [self.Point(x=float(x), y=float(y), z=0.02) for x, y in r['polygon'] + r['polygon'][:1]]
                t = self.Marker()
                t.header.frame_id = frame
                t.ns, t.id = 'rooms', 2 * i + 1
                t.type, t.action = self.Marker.TEXT_VIEW_FACING, self.Marker.ADD
                t.pose.position.x, t.pose.position.y, t.pose.position.z = r['center'][0], r['center'][1], 0.5
                t.pose.orientation.w = 1.0
                t.scale.z = 0.3
                t.color.r = t.color.g = t.color.b = t.color.a = 1.0
                t.text = r.get('label') or r['id'].replace('_', ' ').title()
                arr.markers += [m, t]
            self.marker_pub.publish(arr)

    rclpy.init()
    node = RoomSegmentation()
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
