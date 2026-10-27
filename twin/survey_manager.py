#!/usr/bin/env python3
"""Autonomous survey: explore, partition, visit and categorize every room,
save, go home. No user input (robot-vacuum "fast mapping", for a building).

  EXPLORING    /explore/resume True; wait until the frontier explorer
               reports state=DONE (no reachable frontier left) or
               explore_timeout. Every YOLO object placed meanwhile
               (/yolo/objects) is remembered with its map position.
  SEGMENTING   ask the room node to re-segment the finished map, wait for it
  VISITING     rooms in nearest-first order: /goal_pose to the room's open
               centre; skipped after room_timeout or no progress
  SCANNING     spin 360 deg there (fast-tick Twist on /cmd_vel_nav) while YOLO
               keeps placing objects; ask the VLM for a name twice
  NAMING       room type from the objects that lie INSIDE the room polygon
               (bed -> Bedroom, refrigerator/oven -> Kitchen, couch+tv ->
               Living Room, toilet -> Bathroom, many potted plants -> Plant
               Area, ...); if no rule is confident, the VLM's suggestion; else
               the room keeps its number. Sent as an "auto" rename, so a name
               a person typed is never overwritten.
  RETURNING    /goal_pose to the dock
  DONE         config/survey_report.json (rooms, types, evidence, times) and
               config/home_map.json (the RTAB-Map database of this survey)

Low battery: when the battery manager docks the robot (RETURNING / DOCKED),
the survey pauses and resumes the step it was in once the battery is back.

Commands: /survey/command  std_msgs/String JSON {"op": "start" | "stop"}
          (auto_start: begin by itself once the explorer is up)
Status:   /survey/status   std_msgs/String JSON, 1 Hz (state, progress, rooms)
"""
import datetime
import json
import math
import os
import tempfile
import time

# ------------------------------------------------------------------ room typing
# COCO classes YOLO11 detects, weighted as evidence for a room type. A type
# needs ROOM_TYPE_MIN; the strongest wins. Agricultural: a plant area is many
# potted plants (hydroponic racks look like that to COCO).
ROOM_RULES = {
    'Bedroom':     {'bed': 3.0},
    'Bathroom':    {'toilet': 3.0, 'sink': 1.0, 'toothbrush': 1.0, 'hair drier': 1.0},
    'Kitchen':     {'refrigerator': 3.0, 'oven': 3.0, 'microwave': 2.0, 'toaster': 2.0, 'sink': 1.0},
    'Living Room': {'couch': 2.0, 'tv': 1.5, 'remote': 0.5},
    'Dining Room': {'dining table': 2.0, 'chair': 0.5},
    'Office':      {'laptop': 2.0, 'keyboard': 1.5, 'mouse': 1.0, 'tv': 0.5, 'chair': 0.3, 'book': 0.3},
    'Plant Area':  {'potted plant': 1.0},
}
ROOM_TYPE_MIN = 2.5


def point_in_polygon(x, y, poly):
    """Ray casting; poly = [[x, y], ...]."""
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-12) + x1:
            inside = not inside
    return inside


def objects_in_room(memory, poly):
    """Class -> count of remembered objects (deduplicated cells) inside poly."""
    counts = {}
    for (cls, _, _), o in memory.items():
        if point_in_polygon(o['x'], o['y'], poly):
            counts[cls] = counts.get(cls, 0) + 1
    return counts


def room_type(counts):
    """(type, score) of the best rule, or (None, best score)."""
    best, best_score = None, 0.0
    for name, weights in ROOM_RULES.items():
        score = sum(w * counts.get(cls, 0) for cls, w in weights.items())
        if score > best_score:
            best, best_score = name, score
    return (best, best_score) if best_score >= ROOM_TYPE_MIN else (None, best_score)


def choose_label(counts, vlm_names):
    """Room name and why: confident object rule, else the VLM, else none."""
    t, score = room_type(counts)
    if t:
        return t, f'objects ({", ".join(f"{n} {c}" for c, n in sorted(counts.items(), key=lambda kv: -kv[1]))})'
    names = [v for v in vlm_names if v]
    if names:
        return max(set(names), key=names.count), 'vlm'
    return None, 'no evidence'


def visit_order(rooms, start):
    """Nearest-neighbour tour over room goals from `start` (x, y)."""
    todo, order, here = list(rooms), [], start
    while todo:
        nxt = min(todo, key=lambda r: math.hypot(r['center'][0] - here[0], r['center'][1] - here[1]))
        order.append(nxt)
        todo.remove(nxt)
        here = nxt['center']
    return order


def remember(memory, objects, cell=0.5):
    """Merge /yolo/objects into memory keyed (class, cell x, cell y)."""
    for o in objects:
        key = (o['class'], round(o['x'] / cell), round(o['y'] / cell))
        m = memory.get(key)
        if m is None:
            memory[key] = dict(o, seen=1)
        else:
            m['seen'] += 1


def save_json(path, data):
    d = os.path.dirname(path) or '.'
    os.makedirs(d, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=d, prefix='.' + os.path.basename(path) + '.')
    with os.fdopen(fd, 'w') as f:
        json.dump(data, f, indent=2)
        f.write('\n')
    os.replace(tmp, path)


# ------------------------------------------------------------------ ROS node
def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from geometry_msgs.msg import PoseStamped, Twist
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformException, TransformListener

    class SurveyManager(Node):
        def __init__(self):
            super().__init__('survey_manager')
            p = self.declare_parameter
            p('auto_start', False)
            p('explore_timeout', 1800.0)     # s
            p('room_timeout', 120.0)         # s to reach one room
            p('arrive_tolerance', 0.7)       # m
            p('spin_rate', 0.4)              # rad/s
            p('vlm_wait', 45.0)              # s after the spin for a VLM answer
            p('dock_x', 0.0)
            p('dock_y', 0.0)
            p('map_frame', 'lidar3d_map')
            p('base_frame', 'base_link')
            p('report_file', '/config/survey_report.json')
            p('home_map_file', '/config/home_map.json')
            p('database', '')
            gp = lambda k: self.get_parameter(k).value  # noqa: E731
            for k in ('explore_timeout', 'room_timeout', 'arrive_tolerance', 'spin_rate', 'vlm_wait'):
                setattr(self, k, float(gp(k)))
            self.auto_start = bool(gp('auto_start'))
            self.dock = (float(gp('dock_x')), float(gp('dock_y')))
            self.map_frame, self.base_frame = gp('map_frame'), gp('base_frame')
            self.report_file, self.home_map_file = gp('report_file'), gp('home_map_file')
            self.database = gp('database')

            self.state, self.paused_from = 'IDLE', None
            self.entered = time.time()
            self.started = None
            self.explore = {}
            self.explore_seen = None
            self.battery = ''
            self.battery_ok_since = None
            self.rooms, self.rooms_time = [], 0.0
            self.memory = {}
            self.order, self.idx = [], 0
            self.results = []
            self.vlm = {}                 # room id -> [suggestions]
            self.spin_until = 0.0
            self.progress = (None, 0.0)   # (pose, time) for no-progress detection
            self.note = ''
            self.last_resume = 0.0
            self.asked_twice = False
            self.vlm_alive = False          # any VLM answer seen in this survey
            self.vlm_waited = False         # one full vlm_wait already spent

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)
            self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
            self.room_cmd_pub = self.create_publisher(String, '/semantic_rooms/command', 10)
            self.vlm_pub = self.create_publisher(String, '/vlm_room_request', 10)
            self.status_pub = self.create_publisher(String, '/survey/status', 10)
            self.create_subscription(String, '/survey/command', self.on_command, 10)
            self.create_subscription(String, '/explore/status', self.on_explore, 10)
            self.create_subscription(String, '/battery_manager/status', self.on_battery, 10)
            self.create_subscription(String, '/semantic_rooms/json', self.on_rooms, 10)
            self.create_subscription(String, '/yolo/objects', self.on_objects, 10)
            self.create_subscription(String, '/vlm_room_suggestion', self.on_vlm, 10)
            self.Bool, self.String, self.Twist, self.PoseStamped = Bool, String, Twist, PoseStamped
            self.Time, self.TransformException = Time, TransformException
            self.create_timer(0.5, self.tick)
            self.create_timer(0.05, self.fast_tick)
            self.create_timer(1.0, self.publish_status)
            self.get_logger().info(f'survey manager ready (auto_start={self.auto_start})')

        # ---------------------------------------------------------- inputs
        def on_command(self, msg):
            try:
                op = json.loads(msg.data).get('op')
            except (ValueError, AttributeError):
                op = msg.data.strip()
            if op == 'start' and self.state in ('IDLE', 'DONE', 'STOPPED'):
                self.start()
            elif op == 'stop':
                self.stop('stopped by the user')

        def on_explore(self, msg):
            self.explore = dict(kv.split('=', 1) for kv in msg.data.split() if '=' in kv)
            self.explore_seen = time.time()

        def on_battery(self, msg):
            self.battery = msg.data.split(' ')[0]

        def on_rooms(self, msg):
            try:
                self.rooms = [r for r in json.loads(msg.data).get('rooms', []) if r.get('polygon')]
                self.rooms_time = time.time()
            except ValueError:
                pass

        def on_objects(self, msg):
            try:
                remember(self.memory, json.loads(msg.data).get('objects', []))
            except (ValueError, KeyError, TypeError):
                pass

        def on_vlm(self, msg):
            try:
                d = json.loads(msg.data)
                self.vlm.setdefault(d.get('room_id'), []).append(d.get('suggestion', ''))
                self.vlm_alive = True
            except ValueError:
                pass

        # ---------------------------------------------------------- helpers
        def pose(self):
            try:
                t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, self.Time()).transform.translation
                return (t.x, t.y)
            except self.TransformException:
                return None

        def goto(self, x, y):
            here = self.pose()
            yaw = math.atan2(y - here[1], x - here[0]) if here else 0.0
            g = self.PoseStamped()
            g.header.frame_id = self.map_frame
            g.header.stamp = self.get_clock().now().to_msg()
            g.pose.position.x, g.pose.position.y = float(x), float(y)
            g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.goal_pub.publish(g)
            self.progress = (here, time.time())

        def arrived(self, x, y):
            here = self.pose()
            return here is not None and math.hypot(here[0] - x, here[1] - y) < self.arrive_tolerance

        def stuck(self, limit=30.0):
            here, since = self.progress
            now_pose = self.pose()
            if now_pose and (here is None or math.hypot(now_pose[0] - here[0], now_pose[1] - here[1]) > 0.3):
                self.progress = (now_pose, time.time())
                return False
            return time.time() - since > limit

        def enter(self, state, note=''):
            self.get_logger().info(f'survey: {self.state} -> {state}' + (f' ({note})' if note else ''))
            self.state, self.entered, self.note = state, time.time(), note

        def start(self):
            self.started = time.time()
            self.results, self.memory, self.vlm, self.idx = [], {}, {}, 0
            self.enter('EXPLORING', 'mapping until no frontier is left')

        def stop(self, why):
            self.spin_until = 0.0
            self.cmd_pub.publish(self.Twist())
            self.resume_pub.publish(self.Bool(data=False))
            self.enter('STOPPED', why)

        def battery_low(self):
            return self.battery in ('RETURNING', 'DOCKED')

        # ---------------------------------------------------------- state machine
        def tick(self):
            now, age = time.time(), time.time() - self.entered
            if self.state == 'IDLE':
                if self.auto_start and self.explore_seen and now - self.explore_seen < 5:
                    self.start()
                return
            if self.state in ('DONE', 'STOPPED'):
                return

            # Battery manager took over: wait, then carry on where we were.
            if self.state != 'PAUSED' and self.battery_low():
                self.spin_until = 0.0
                self.paused_from = self.state
                self.enter('PAUSED', 'low battery: the battery manager is docking')
                return
            if self.state == 'PAUSED':
                if self.battery_low():
                    self.battery_ok_since = None
                elif self.battery_ok_since is None:
                    self.battery_ok_since = now
                elif now - self.battery_ok_since > 10:
                    # The battery manager drove the robot to the dock: a room
                    # step restarts by driving back to that room.
                    resume = self.paused_from or 'EXPLORING'
                    if resume == 'SCANNING':
                        resume = 'VISITING'
                    self.enter(resume, 'battery OK again')
                    self.battery_ok_since = None
                    if resume == 'VISITING':
                        self.visit_current()
                    elif resume == 'RETURNING':
                        self.goto(*self.dock)
                    elif resume == 'SEGMENTING':
                        self.request_rooms('battery OK again')
                return

            if self.state == 'EXPLORING':
                st = self.explore.get('state')
                if st == 'DONE':
                    self.resume_pub.publish(self.Bool(data=False))
                    self.request_rooms()
                elif age > self.explore_timeout:
                    self.resume_pub.publish(self.Bool(data=False))
                    self.request_rooms('explore timeout: partitioning what is mapped')
                elif self.explore.get('enabled') != 'True' and now - self.last_resume > 5:
                    self.resume_pub.publish(self.Bool(data=True))     # (re)start the explorer
                    self.last_resume = now
                    if not self.explore_seen and age > 60:
                        self.request_rooms('no explorer running: partitioning what is mapped')

            elif self.state == 'SEGMENTING':
                if self.rooms and self.rooms_time >= self.entered:
                    here = self.pose() or self.dock
                    self.order = visit_order(self.rooms, here)
                    self.idx = 0
                    self.enter('VISITING', f'{len(self.order)} rooms')
                    self.visit_current()
                elif age > 30:
                    self.enter('RETURNING', 'no rooms found')
                    self.goto(*self.dock)

            elif self.state == 'VISITING':
                room = self.order[self.idx]
                if self.arrived(*room['center']):
                    self.enter('SCANNING', room.get('label') or room['id'])
                    self.asked_twice = False
                    self.spin_until = now + 2 * math.pi / self.spin_rate
                    self.vlm_pub.publish(self.String(data=room['id']))
                elif age > self.room_timeout or self.stuck():
                    self.finish_room(room, skipped=True)

            elif self.state == 'SCANNING':
                room = self.order[self.idx]
                spin = 2 * math.pi / self.spin_rate
                if self.spin_until and now > self.spin_until - spin / 2 and not self.asked_twice:
                    self.vlm_pub.publish(self.String(data=room['id']))     # second view, other half
                    self.asked_twice = True
                # No VLM answer in a whole wait (no Ollama?): do not wait per room again.
                wait = self.vlm_wait if (self.vlm_alive or not self.vlm_waited) else 3.0
                if not self.spin_until and (self.vlm.get(room['id']) or age > spin + wait):
                    if age > spin + wait:
                        self.vlm_waited = True
                    self.finish_room(room)

            elif self.state == 'RETURNING':
                if self.arrived(*self.dock) or age > self.room_timeout:
                    self.write_report()
                    self.enter('DONE', f'{len(self.results)} rooms surveyed')

        def fast_tick(self):
            if not self.spin_until:
                return
            if time.time() < self.spin_until and self.state == 'SCANNING':
                t = self.Twist()
                t.angular.z = self.spin_rate
                self.cmd_pub.publish(t)
            else:
                self.spin_until = 0.0
                self.cmd_pub.publish(self.Twist())

        def request_rooms(self, note='map complete'):
            self.enter('SEGMENTING', note)       # first: a reply is then never "older"
            self.room_cmd_pub.publish(self.String(data=json.dumps({'op': 'resegment'})))

        def visit_current(self):
            room = self.order[self.idx]
            self.goto(*room['center'])

        def finish_room(self, room, skipped=False):
            counts = objects_in_room(self.memory, room['polygon'])
            label, why = choose_label(counts, self.vlm.get(room['id'], []))
            rtype, score = room_type(counts)
            result = {'id': room['id'], 'previous_label': room.get('label', ''), 'label': label,
                      'reason': why, 'type': rtype, 'type_score': round(score, 1), 'objects': counts,
                      'vlm': self.vlm.get(room['id'], []), 'visited': not skipped,
                      'area': room.get('area'), 'center': room['center']}
            self.results.append(result)
            if label:
                self.room_cmd_pub.publish(self.String(data=json.dumps({
                    'op': 'rename', 'id': room['id'], 'label': label, 'source': 'auto',
                    'category': rtype, 'objects': counts,
                    'vlm_suggestion': (self.vlm.get(room['id']) or [''])[-1]})))
            self.get_logger().info(f'survey: {room["id"]} -> {label!r} [{why}]' + (' (not reached)' if skipped else ''))
            self.idx += 1
            if self.idx < len(self.order):
                self.enter('VISITING', f'room {self.idx + 1}/{len(self.order)}')
                self.visit_current()
            else:
                self.enter('RETURNING', 'all rooms surveyed')
                self.goto(*self.dock)

        def write_report(self):
            stamp = datetime.datetime.now().isoformat(timespec='seconds')
            report = {'finished': stamp, 'duration_s': round(time.time() - (self.started or time.time())),
                      'database': self.database, 'frame': self.map_frame, 'rooms': self.results,
                      'objects_seen': len(self.memory)}
            try:
                save_json(self.report_file, report)
                if self.database:
                    save_json(self.home_map_file, {'database': self.database, 'surveyed': stamp,
                                                   'rooms': len(self.results)})
            except OSError as e:
                self.get_logger().warn(f'cannot write the survey report: {e}')

        def publish_status(self):
            room = self.order[self.idx] if self.state in ('VISITING', 'SCANNING') and self.idx < len(self.order) else None
            self.status_pub.publish(self.String(data=json.dumps({
                'state': self.state, 'note': self.note,
                'elapsed_s': round(time.time() - self.started) if self.started else 0,
                'room': (room.get('label') or room['id']) if room else None,
                'room_index': self.idx + 1 if room else None, 'rooms_total': len(self.order),
                'results': [{'id': r['id'], 'label': r['label'], 'reason': r['reason']} for r in self.results],
                'objects_seen': len(self.memory),
                'explorer': self.explore.get('state'),
            })))

    rclpy.init()
    node = SurveyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(node.Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
