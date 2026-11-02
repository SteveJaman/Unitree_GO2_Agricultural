#!/usr/bin/env python3
"""
build_lab_world.py: write lab_office.sdf, an indoor research lab / office for
exploration and obstacle-avoidance tests, and lab_office.json (rooms,
furniture footprints, tour goals) for scripts/tools/nav_compare.

  y = 5.5 +---------------------+--------------+--------------+
          | N1 open office      | N2 lab       | N3 conference|
          | desks, cubicle wall,D island,      | table +      |
          | island, bookshelf   | bench        | 6 chairs     |
  y = 1.0 +--------D------------+-----D--------+----D---------+
          |  * spawn (-4.5, 0)    corridor  (bin, cart)        |
  y = -1  +---D-------+----D----+-----D--------+-----D--------+
          | S1 break  | S2      | S3 workshop  | S4 office    |
          | sofa,     | storage | benches,     | desk, shelf  |
          | cafe table| shelves | pallet, cart |              |
  y = -5.5+-----------+---------+--------------+--------------+
        x = -7       -3         0              4              7

14 x 11 m, inside the 30 x 30 m rolling global costmap wherever the robot is.
Walls reach a 2.7 m ceiling, the cubicle wall is 1.3 m. The ceiling matters:
the sim lidar looks up to 32.5 deg, and without one its rays passed over the
walls, hit nothing and cleared the costmap on their way, so rooms the dog
had never seen counted as mapped (the first pilot run reported 669 m² known
and "Exploration complete" after 75 s).
Doors are 1.2 m (N1 <-> N2: 1.0 m). Aisles are 0.9 to 1.6 m; some corners
are deliberately closed off by furniture (the conference table's ends, the
sofa side of the break room) so the dog must map them from a distance.

Furniture: 3DGEMS / Gazebo Models & Worlds Collection models (desks, chairs,
tables, shelves, cabinets, sofa), downloaded by ./gazebo/setup_gazebo.sh into
gazebo/models/furniture/ (not in git); start_inside.sh puts that folder on
GZ_SIM_RESOURCE_PATH. Heavy meshes (plant_1: 532k triangles, shelves_high,
table_conference_1) are left out, and so are kitchen_chair (30k), office_chair
(44k) and computer_case (14k): the lidar is rendered in software, and with
them the sim ran at 0.67x real time instead of 1.0x (2026-11-01, 16 cores).
Boxes, the pallet, the carts and the cubicle wall are SDF primitives.

Every piece is placed by the centre of its footprint (FOOTPRINT holds each
model's measured mesh bounds), so the aisle widths above are exact. Before
writing, the floor plan is rasterized and every room and tour goal is
checked reachable from the spawn for a disc of CHECK_RADIUS.

Usage: python3 build_lab_world.py [--check] [--plan PNG]
  --check  report only, write nothing
  --plan   also draw the floor plan (docs/images/lab_office/floorplan.png)
"""
import argparse
import json
import math
import os
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
SDF_OUT = os.path.join(HERE, "lab_office.sdf")
JSON_OUT = os.path.join(HERE, "lab_office.json")

X0, X1, Y0, Y1 = -7.0, 7.0, -5.5, 5.5     # outer wall centre lines
Y_N, Y_S = 1.0, -1.0                      # corridor walls
WALL_T, WALL_H, CUBE_H = 0.1, 2.7, 1.3   # walls reach the ceiling
SPAWN = (-4.5, 0.0, 0.0)
CHECK_RADIUS = 0.30                        # robot_radius 0.22 + margin

# Walls as (x0, y0, x1, y1, [(door_from, door_to), ...]) along their length.
WALLS = [
    (X0, Y1, X1, Y1, []),                                     # north
    (X0, Y0, X1, Y0, []),                                     # south
    (X0, Y0, X0, Y1, []),                                     # west
    (X1, Y0, X1, Y1, []),                                     # east
    (X0, Y_N, X1, Y_N, [(-3.4, -2.2), (0.4, 1.6), (3.6, 4.8)]),
    (X0, Y_S, X1, Y_S, [(-4.8, -3.6), (-2.1, -0.9), (1.4, 2.6), (4.9, 6.1)]),
    (-1.0, Y_N, -1.0, Y1, [(1.8, 2.8)]),                      # N1 | N2
    (3.0, Y_N, 3.0, Y1, []),                                  # N2 | N3
    (-3.0, Y0, -3.0, Y_S, []),                                # S1 | S2
    (0.0, Y0, 0.0, Y_S, []),                                  # S2 | S3
    (4.0, Y0, 4.0, Y_S, []),                                  # S3 | S4
]

ROOMS = {   # interior rectangles (x0, x1, y0, y1) for "rooms entered"
    "corridor": (X0, X1, Y_S, Y_N),
    "N1 open office": (X0, -1.0, Y_N, Y1),
    "N2 lab": (-1.0, 3.0, Y_N, Y1),
    "N3 conference": (3.0, X1, Y_N, Y1),
    "S1 break room": (X0, -3.0, Y0, Y_S),
    "S2 storage": (-3.0, 0.0, Y0, Y_S),
    "S3 workshop": (0.0, 4.0, Y0, Y_S),
    "S4 office": (4.0, X1, Y0, Y_S),
}

# Mesh bounds in the model frame (x0, x1, y0, y1, height), measured from the
# collection's meshes with trimesh (visual pose and scale applied).
FOOTPRINT = {
    "desk_brown": (-0.94, 0.83, -0.38, 0.42, 0.72),
    "desk_yellow": (-0.91, 0.86, -0.41, 0.39, 0.72),
    "desk_drawer": (-0.26, 0.30, -0.84, 0.98, 0.76),
    "table_conference_2": (-1.36, 1.34, -0.61, 0.59, 0.71),
    "cafe_table": (-0.457, 0.457, -0.457, 0.457, 0.775),
    "chair_1": (-0.32, 0.22, -0.23, 0.25, 0.82),
    "chair_2": (-0.18, 0.20, -0.24, 0.28, 0.87),
    "bookshelf_large": (-0.80, 0.86, -0.21, 0.21, 1.84),
    "file_cabinet_large": (-0.29, 0.33, -0.19, 0.19, 0.66),
    "file_cabinet_small": (-0.26, 0.35, -0.36, 0.39, 0.65),
    "drawer_white": (-0.08, 0.47, -0.09, 0.65, 0.89),
    "trash_bin": (-0.12, 0.12, -0.12, 0.12, 0.30),
    "sofa_set_1": (-1.21, 1.20, -0.37, 0.37, 0.68),
    "side_table_1": (-0.23, 0.22, -0.18, 0.27, 0.64),
    "monitor_1": (-0.31, 0.29, -0.06, 0.04, 0.44),
}

# (model, footprint centre x, y, yaw deg, room)
FURNITURE = [
    # N1 open office: desks on the north wall, a cubicle wall, an island
    ("desk_brown", -6.0, 5.03, 0, "N1"), ("desk_yellow", -4.1, 5.03, 0, "N1"),
    ("chair_1", -6.0, 4.30, 180, "N1"), ("chair_1", -4.1, 4.30, 180, "N1"),
    ("desk_brown", -2.0, 5.03, 0, "N1"), ("chair_1", -2.0, 4.30, 180, "N1"),
    ("desk_yellow", -5.3, 2.60, 0, "N1"), ("chair_2", -5.3, 1.90, 90, "N1"),
    ("bookshelf_large", -6.73, 3.50, 90, "N1"), ("trash_bin", -1.30, 1.30, 0, "N1"),
    # N2 lab: island (long table), bench on the east wall, stools, cabinets
    ("table_conference_2", 0.55, 3.60, 90, "N2"), ("desk_drawer", 2.62, 3.60, 0, "N2"),
    ("chair_2", 0.20, 5.12, 90, "N2"), ("chair_2", 0.90, 5.12, 90, "N2"),
    ("file_cabinet_large", 2.55, 5.22, 0, "N2"), ("drawer_white", -0.60, 5.05, 0, "N2"),
    ("trash_bin", -0.70, 1.35, 0, "N2"),
    # N3 conference: table, three chairs a side; its ends are 0.6 m from the walls
    ("table_conference_2", 5.0, 3.50, 0, "N3"),
    ("chair_2", 4.2, 4.45, -90, "N3"), ("chair_2", 5.0, 4.45, -90, "N3"), ("chair_2", 5.8, 4.45, -90, "N3"),
    ("chair_2", 4.2, 2.55, 90, "N3"), ("chair_2", 5.0, 2.55, 90, "N3"), ("chair_2", 5.8, 2.55, 90, "N3"),
    ("side_table_1", 6.60, 5.10, 0, "N3"), ("trash_bin", 3.35, 5.15, 0, "N3"),
    # S1 break room: sofa, cafe table and chairs, fridge (drawer unit)
    ("sofa_set_1", -6.57, -3.40, 90, "S1"), ("side_table_1", -6.60, -1.60, 0, "S1"),
    ("cafe_table", -5.10, -3.30, 0, "S1"),
    ("chair_2", -5.10, -2.45, 90, "S1"), ("chair_2", -5.10, -4.15, -90, "S1"),
    ("drawer_white", -3.40, -5.05, 0, "S1"), ("trash_bin", -3.30, -1.40, 0, "S1"),
    # S2 storage: two bookshelves, cabinets (boxes below)
    ("bookshelf_large", -2.73, -2.90, 90, "S2"), ("bookshelf_large", -2.73, -4.62, 90, "S2"),
    ("file_cabinet_large", -0.25, -3.00, 90, "S2"), ("file_cabinet_large", -0.25, -3.70, 90, "S2"),
    # S3 workshop: two benches on the south wall, chair, PC (pallet, cart below)
    ("desk_brown", 1.0, -5.03, 180, "S3"), ("desk_yellow", 3.0, -5.03, 180, "S3"),
    ("chair_1", 1.0, -4.30, 0, "S3"),
    # S4 office
    ("desk_yellow", 6.55, -3.30, 90, "S4"), ("chair_1", 5.75, -3.30, 0, "S4"),
    ("bookshelf_large", 5.20, -5.24, 0, "S4"), ("file_cabinet_small", 4.40, -4.30, 0, "S4"),
    ("trash_bin", 6.70, -1.35, 0, "S4"),
    # corridor
    ("trash_bin", -1.0, 0.80, 0, "corridor"), ("side_table_1", 6.70, 0.70, 0, "corridor"),
]
MODELS = sorted({f[0] for f in FURNITURE} | {"monitor_1"})   # what setup_gazebo.sh downloads
# Monitors stand on desks (not in the floor plan): (x, y, z, yaw deg)
MONITORS = [(-6.0, 5.25, 0.72, 180), (-4.1, 5.25, 0.72, 180), (-2.0, 5.25, 0.72, 180),
            (1.0, -5.25, 0.72, 0), (3.0, -5.25, 0.72, 0), (6.75, -3.30, 0.72, -90)]

# SDF boxes: (name, x, y, yaw deg, sx, sy, sz, rgb)
CARDBOARD, GREY, BLUE, WOOD = (0.62, 0.48, 0.30), (0.55, 0.57, 0.60), (0.25, 0.40, 0.65), (0.55, 0.42, 0.28)
BOXES = [
    ("cubicle_wall", -2.95, 4.225, 0, 0.06, 2.45, CUBE_H, (0.45, 0.50, 0.58)),
    ("boxes_1", -1.50, -4.90, 0, 0.70, 0.50, 0.60, CARDBOARD),
    ("boxes_2", -0.60, -5.00, 0, 0.50, 0.50, 0.45, CARDBOARD),
    ("pallet", 2.40, -3.00, 0, 1.00, 0.80, 0.35, WOOD),
    ("tool_cart", 0.50, -2.20, 0, 0.50, 0.80, 0.90, BLUE),
    ("corridor_cart", 3.40, -0.60, 0, 0.80, 0.50, 0.90, GREY),
    ("pc_tower_1", 2.80, 1.50, 0, 0.20, 0.48, 0.44, (0.15, 0.15, 0.17)),
    ("pc_tower_2", 3.75, -4.40, 0, 0.20, 0.48, 0.44, (0.15, 0.15, 0.17)),
]

# Navigation tour after exploration: one goal per room, then back to the spawn.
TOUR = [
    ("N1 open office", -4.0, 3.55), ("N2 lab", 1.75, 3.60), ("N3 conference", 5.0, 1.70),
    ("S4 office", 5.0, -2.50), ("S3 workshop", 3.35, -3.60), ("S2 storage", -1.50, -3.20),
    ("S1 break room", -4.0, -3.30), ("start", SPAWN[0], SPAWN[1]),
]


# ------------------------------------------------------------------ geometry
def rect_corners(cx, cy, sx, sy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]


def wall_boxes():
    """Wall pieces between doors: (name, cx, cy, yaw, length, height)."""
    out = []
    for i, (x0, y0, x1, y1, doors) in enumerate(WALLS):
        horiz = y0 == y1
        a, b = (x0, x1) if horiz else (y0, y1)
        # extend by half a thickness so corners close
        a, b = a - WALL_T / 2, b + WALL_T / 2
        cuts = [a] + [v for d in sorted(doors) for v in d] + [b]
        for j in range(0, len(cuts), 2):
            p, q = cuts[j], cuts[j + 1]
            if q - p < 1e-3:
                continue
            m = (p + q) / 2
            cx, cy = (m, y0) if horiz else (x0, m)
            out.append((f"wall_{i}_{j // 2}", cx, cy, 0.0 if horiz else math.pi / 2, q - p, WALL_H))
    return out


def furniture_pose(model, cx, cy, yaw_deg):
    """Model origin so that the footprint centre lands on (cx, cy)."""
    x0, x1, y0, y1, h = FOOTPRINT[model]
    lx, ly = (x0 + x1) / 2, (y0 + y1) / 2
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    return cx - (c * lx - s * ly), cy - (s * lx + c * ly), yaw, (x1 - x0, y1 - y0, h)


def obstacles():
    """Every floor obstacle as a rotated rectangle: dict(name, kind, cx, cy, yaw, sx, sy, h)."""
    obs = [dict(name=n, kind="wall", cx=cx, cy=cy, yaw=yaw, sx=L, sy=WALL_T, h=h)
           for n, cx, cy, yaw, L, h in wall_boxes()]
    counts = {}
    for model, cx, cy, yaw_deg, room in FURNITURE:
        counts[model] = counts.get(model, 0) + 1
        _, _, yaw, (sx, sy, h) = furniture_pose(model, cx, cy, yaw_deg)
        obs.append(dict(name=f"{model}_{counts[model]}", kind=model, room=room, cx=cx, cy=cy,
                        yaw=yaw, sx=sx, sy=sy, h=h))
    for name, cx, cy, yaw_deg, sx, sy, sz, _ in BOXES:
        obs.append(dict(name=name, kind="box", cx=cx, cy=cy, yaw=math.radians(yaw_deg), sx=sx, sy=sy, h=sz))
    return obs


# ------------------------------------------------------------------ check
def raster(obs, res=0.05, inflate=0.0):
    nx, ny = int(round((X1 - X0) / res)), int(round((Y1 - Y0) / res))
    grid = [[False] * nx for _ in range(ny)]
    for o in obs:
        c, s = math.cos(o["yaw"]), math.sin(o["yaw"])
        hx, hy = o["sx"] / 2 + inflate, o["sy"] / 2 + inflate
        r = math.hypot(hx, hy)
        i0, i1 = max(0, int((o["cx"] - r - X0) / res)), min(nx, int((o["cx"] + r - X0) / res) + 1)
        j0, j1 = max(0, int((o["cy"] - r - Y0) / res)), min(ny, int((o["cy"] + r - Y0) / res) + 1)
        for j in range(j0, j1):
            y = Y0 + (j + 0.5) * res - o["cy"]
            for i in range(i0, i1):
                x = X0 + (i + 0.5) * res - o["cx"]
                u, v = c * x + s * y, -s * x + c * y
                if abs(u) <= hx and abs(v) <= hy:
                    grid[j][i] = True
    return grid, res


def reachable(grid, res, start):
    ny, nx = len(grid), len(grid[0])
    si, sj = int((start[0] - X0) / res), int((start[1] - Y0) / res)
    seen = [[False] * nx for _ in range(ny)]
    if grid[sj][si]:
        return seen
    q = deque([(si, sj)])
    seen[sj][si] = True
    while q:
        i, j = q.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < nx and 0 <= b < ny and not seen[b][a] and not grid[b][a]:
                seen[b][a] = True
                q.append((a, b))
    return seen


def check(obs):
    occ, res = raster(obs)
    free_cells = sum(not v for row in occ for v in row)
    inflated, _ = raster(obs, inflate=CHECK_RADIUS)
    seen = reachable(inflated, res, SPAWN)
    ok = True
    print(f"interior {(X1 - X0) * (Y1 - Y0):.0f} m², floor free of furniture/walls "
          f"{free_cells * res * res:.1f} m², reachable by the robot centre (r {CHECK_RADIUS} m) "
          f"{sum(v for row in seen for v in row) * res * res:.1f} m²")
    for name, (x0, x1, y0, y1) in ROOMS.items():
        n = sum(seen[j][i] for j in range(int((y0 - Y0) / res), int((y1 - Y0) / res))
                for i in range(int((x0 - X0) / res), int((x1 - X0) / res)))
        print(f"  {name:16s} reachable {n * res * res:5.1f} m²")
        ok &= n > 0
    for name, x, y in TOUR:
        hit = seen[int((y - Y0) / res)][int((x - X0) / res)]
        print(f"  tour goal {name:16s} ({x:+.2f}, {y:+.2f}) {'reachable' if hit else 'NOT REACHABLE'}")
        ok &= hit
    return ok, free_cells * res * res


# ------------------------------------------------------------------ SDF
def box_model(name, cx, cy, yaw, sx, sy, sz, rgb):
    col = f"{rgb[0]:.2f} {rgb[1]:.2f} {rgb[2]:.2f} 1"
    geo = f"<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"
    return (f'    <model name="{name}"><static>true</static><pose>{cx:.3f} {cy:.3f} {sz / 2:.3f} 0 0 {yaw:.4f}</pose>\n'
            f'      <link name="link"><collision name="c">{geo}</collision>\n'
            f'        <visual name="v">{geo}<material><ambient>{col}</ambient><diffuse>{col}</diffuse></material></visual></link>\n'
            f'    </model>\n')


def include(model, name, x, y, z, yaw):
    return (f'    <include><uri>model://{model}</uri><name>{name}</name>'
            f'<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose><static>true</static></include>\n')


def write_sdf():
    parts = [f"""<?xml version="1.0" ?>
<!--
  lab_office.sdf: indoor research lab / office for the Go2 + D1 simulation.
  GENERATED by build_lab_world.py: edit the layout there and rerun.
  14 x 11 m, 2.7 m ceiling: a corridor, three rooms north (open office, lab, conference),
  four south (break room, storage, workshop, office). Furniture from the
  3DGEMS / Gazebo Models & Worlds Collection (./gazebo/setup_gazebo.sh
  downloads it to gazebo/models/furniture). Spawn: {SPAWN[0]} {SPAWN[1]} {SPAWN[2]}.
  @RENDER_ENGINE@ is substituted by start_inside.sh (ogre2 or ogre).
-->
<sdf version="1.9">
  <world name="lab_office">
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>@RENDER_ENGINE@</render_engine>
    </plugin>
    <scene>
      <ambient>0.7 0.7 0.7 1</ambient>
      <background>0.8 0.82 0.85 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.88 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -1.0</direction>
    </light>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.62 0.62 0.60 1</ambient><diffuse>0.62 0.62 0.60 1</diffuse></material>
        </visual>
      </link>
    </model>
"""]
    parts.append('    <model name="walls"><static>true</static>\n')
    for name, cx, cy, yaw, length, h in wall_boxes():
        geo = f"<geometry><box><size>{length:.3f} {WALL_T} {h}</size></box></geometry>"
        parts.append(f'      <link name="{name}"><pose>{cx:.3f} {cy:.3f} {h / 2} 0 0 {yaw:.4f}</pose>'
                     f'<collision name="c">{geo}</collision><visual name="v">{geo}'
                     '<material><ambient>0.85 0.85 0.82 1</ambient><diffuse>0.85 0.85 0.82 1</diffuse></material></visual></link>\n')
    parts.append("    </model>\n")
    geo = f"<geometry><box><size>{X1 - X0 + WALL_T:.2f} {Y1 - Y0 + WALL_T:.2f} 0.05</size></box></geometry>"
    parts.append(f'    <model name="ceiling"><static>true</static>'
                 f'<pose>{(X0 + X1) / 2} {(Y0 + Y1) / 2} {WALL_H + 0.025} 0 0 0</pose>\n'
                 f'      <link name="link"><collision name="c">{geo}</collision><visual name="v">{geo}'
                 '<cast_shadows>false</cast_shadows><material><ambient>0.92 0.92 0.9 1</ambient>'
                 '<diffuse>0.92 0.92 0.9 1</diffuse></material></visual></link>\n    </model>\n')
    counts = {}
    for model, cx, cy, yaw_deg, _ in FURNITURE:
        counts[model] = counts.get(model, 0) + 1
        x, y, yaw, _ = furniture_pose(model, cx, cy, yaw_deg)
        parts.append(include(model, f"{model}_{counts[model]}", x, y, 0.0, yaw))
    for k, (cx, cy, z, yaw_deg) in enumerate(MONITORS, 1):
        x, y, yaw, _ = furniture_pose("monitor_1", cx, cy, yaw_deg)
        parts.append(include("monitor_1", f"monitor_{k}", x, y, z, yaw))
    for name, cx, cy, yaw_deg, sx, sy, sz, rgb in BOXES:
        parts.append(box_model(name, cx, cy, math.radians(yaw_deg), sx, sy, sz, rgb))
    parts.append("  </world>\n</sdf>\n")
    with open(SDF_OUT, "w", newline="\n") as f:
        f.write("".join(parts))


def plot_plan(obs, path):
    """Floor plan PNG (needs matplotlib): walls, furniture, rooms, spawn, tour goals."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    fig, ax = plt.subplots(figsize=(11, 8.6))
    fig.patch.set_facecolor("#fcfcfb")
    ax.set_facecolor("#f3f2ee")
    for o in obs:
        pts = rect_corners(o["cx"], o["cy"], o["sx"], o["sy"], o["yaw"])
        if o["kind"] == "wall":
            ax.add_patch(Polygon(pts, closed=True, fc="#2b2a28", ec="none"))
        else:
            ax.add_patch(Polygon(pts, closed=True, fc="#cfcdc6" if o["kind"] == "box" else "#b9c9dc",
                                 ec="#52514e", lw=0.6))
            ax.text(o["cx"], o["cy"], o["kind"].replace("_", " ").replace("table conference 2", "table"),
                    fontsize=5.5, ha="center", va="center", color="#0b0b0b")
    for name, (x0, x1, y0, y1) in ROOMS.items():
        if name != "corridor":
            ax.text((x0 + x1) / 2, (Y1 + 0.15) if y0 > 0 else (Y0 - 0.15), name, fontsize=9, weight="bold",
                    color="#52514e", ha="center", va="bottom" if y0 > 0 else "top")
    for k, (name, x, y) in enumerate(TOUR, 1):
        ax.plot(x, y, "o", ms=14, mfc="#eb6834", mec="#fcfcfb", mew=1.5, zorder=5)
        ax.text(x, y, str(k), ha="center", va="center", fontsize=8, color="white", weight="bold", zorder=6)
    ax.plot(SPAWN[0], SPAWN[1], "s", ms=11, mfc="#1baf7a", mec="#fcfcfb", mew=1.5, zorder=5)
    ax.annotate("spawn", (SPAWN[0], SPAWN[1]), xytext=(9, -14), textcoords="offset points", fontsize=8,
                color="#0b0b0b")
    ax.set_xlim(X0 - 0.3, X1 + 0.3)
    ax.set_ylim(Y0 - 0.7, Y1 + 0.7)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("lab_office: 14 x 11 m, 2.7 m ceiling. Blue = 3DGEMS furniture, grey = boxes, "
                 "orange = tour goals in order", loc="left", fontsize=10)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    print(f"wrote {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="check reachability only, write nothing")
    ap.add_argument("--plan", metavar="PNG", help="also draw the floor plan (needs matplotlib)")
    a = ap.parse_args()
    obs = obstacles()
    if a.plan:
        plot_plan(obs, a.plan)
    ok, free_area = check(obs)
    if not ok:
        raise SystemExit("layout check failed: a room or tour goal is not reachable")
    if a.check:
        return
    write_sdf()
    meta = {
        "world": "lab_office", "generated_by": "gazebo/worlds/build_lab_world.py",
        "bounds": [X0, X1, Y0, Y1], "spawn": list(SPAWN), "free_area_m2": round(free_area, 1),
        "rooms": {k: list(v) for k, v in ROOMS.items()},
        "tour": [{"name": n, "x": x, "y": y} for n, x, y in TOUR],
        "obstacles": [{k: (round(v, 4) if isinstance(v, float) else v) for k, v in o.items()} for o in obs],
        "models": MODELS,
    }
    with open(JSON_OUT, "w", newline="\n") as f:
        json.dump(meta, f, indent=1)
        f.write("\n")
    print(f"wrote {SDF_OUT} and {JSON_OUT}")


if __name__ == "__main__":
    main()
