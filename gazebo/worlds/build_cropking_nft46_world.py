#!/usr/bin/env python3
"""
build_cropking_nft46_world.py: write cropking_nft46.sdf, a lab bay with our CropKing
NFT 4-6 as it stands in the lab (photos IMG_3562-3564, 2026-11-18), and
cropking_nft46.json (rack geometry, the demo's target lettuce, the dog's tail-first
parking waypoint, observer cameras, MoveIt collision objects) for run_sim_demo.sh and
src/d1_moveit_config/scripts/full_mission_test.py.

  y = 2.5 +------------------------------------------+
          |                          | | | | | |     |  6 channels along y, side by side in x
          |  [dock]  ->  waypoint    |N F T 4-6|     |  green frame around them, LED bars on top
          |  (0, 0)    (tail +x)     | | | | | |     |  front channel at x = RACK_X - 0.43
          |                          feeder loops (-y)|
  y = -2.5+------------------------------------------+
         x = -1.5                                  x = 5

The rack, as in the photos:
  - six white 4 ft NFT channels (CropKing NFT 4-6: 6 x 4 ft, 36 sites), square planting
    holes 8 in on centre, side by side with a ~2.5 in gap, resting on two green cross
    rails; they stick out past the frame at the feed end (-y), where a black feeder tube
    loops into each one, and a little at the other end;
  - a green perforated square-steel frame (slotted tube): four posts ~1.68 m high, rails
    at channel height on all four sides, a top frame carrying a black Spider Farmer LED
    bar and a second, white light bar;
  - a black reservoir tub on the floor under the channels, a clear tray on it, the
    supply hose from it to the rack;
  - a white sensor box with a small board and camera on two of the middle channels;
  - only a few plants: the demo's target lettuce (front channel), a romaine head, and
    small seedlings; the other holes are empty.
Sizes are estimated from the photos against the 4 ft channels and the 31 in channel
height of the NFT 4-6 listing (cropking.com/system/nft-4-6).

Lettuce: the generated butterhead mesh (build_lettuce_model.py, model://lettuce), each
plant with the gz Label system (label 1) for scripts/tools/lettuce_dataset.py.

Parking: the D1 is mounted facing the dog's tail, so the dog parks tail to the rack.
The waypoint is chosen so that, with the arm in run_sim_demo.sh's start pose (J2 0.25,
J3 0.30, J5 -0.55: the camera 0.40 m behind the dog's centre, 0.67 m up, looking
level), the target lettuce is START_DIST from the camera, and distance_servo.py can
close to its 0.50 m target inside its 0.15 m travel limit.

Usage: python3 build_cropking_nft46_world.py  (run build_lettuce_model.py once first)
"""
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build_nft46_assets as assets  # noqa: E402  (meshes and textures: model://nft46)

HERE = os.path.dirname(os.path.abspath(__file__))
IN = 0.0254

# Room (lab bay)
X0, X1, Y0, Y1, WALL_H = -1.5, 5.0, -2.5, 2.5, 2.7

# Our NFT 4-6
RACK_X, RACK_Y = 3.0, 0.0
TOP_H = 31 * IN                                    # channel tops
CH_N, CH_LEN, CH_W, CH_H = 6, 48 * IN, 4.0 * IN, 1.75 * IN
CH_GAP = 2.75 * IN
CH_PITCH = CH_W + CH_GAP
SITES, SITE_PITCH = 6, 8 * IN
HOLE = 1.0 * IN                                    # square planting hole
TUBE = 1.25 * IN                                   # green perforated square steel tube
POST_H = 66 * IN
FRAME_X = (CH_N - 1) / 2 * CH_PITCH + CH_W / 2 + TUBE / 2 + 0.01   # posts just outside the outer channels
FRAME_Y = 16.5 * IN                                # posts at y = +/- this (33 in deep frame)
CH_Y = RACK_Y - 0.11                               # channel centre: 0.30 m out past the feed end (-y)
TARGET_CHANNEL, TARGET_SITE = 0, 3                 # front channel
PLANTS = {                                         # (channel, site): (scale xy, scale z); the rest are empty holes
    (0, 3): (1.0, 1.0),                            # the demo's target lettuce
    (4, 4): (0.75, 1.35),                          # romaine head
    (0, 0): (0.38, 0.45), (1, 4): (0.42, 0.5), (2, 2): (0.45, 0.5),
    (2, 5): (0.35, 0.4), (3, 3): (0.4, 0.5), (5, 1): (0.33, 0.4),
}
LETTUCE_LABEL = 1                                  # gz Label system: boundingbox camera (lettuce dataset)

# Arm start pose (run_sim_demo.sh) -> camera, in the dog frame with the arm facing the tail
CAM_BEHIND, CAM_Z = 0.426, 0.721                  # D435i on the claw (SIM_ARM_EE=claw)
START_DIST = 0.62                                  # camera -> target lettuce at the start (m)

DOCK = (0.0, 0.0)
GREEN = (0.06, 0.36, 0.22)
WHITE, BLACK, CONCRETE, ALU = (0.95, 0.95, 0.94), (0.06, 0.06, 0.07), (0.70, 0.70, 0.69), (0.82, 0.83, 0.85)


def channel_x(i):
    return RACK_X - (CH_N - 1) / 2 * CH_PITCH + i * CH_PITCH


def site_y(k):
    return CH_Y - (SITES - 1) / 2 * SITE_PITCH + k * SITE_PITCH


# ------------------------------------------------------------------ SDF bits
def mat(rgb, a=1.0, spec=0.1):
    c = f"{rgb[0]:.2f} {rgb[1]:.2f} {rgb[2]:.2f} {a}"
    return f"<material><ambient>{c}</ambient><diffuse>{c}</diffuse><specular>{spec} {spec} {spec} 1</specular></material>"


def link(name, pose, geom, rgb, collide=True, alpha=1.0):
    g = f"<geometry>{geom}</geometry>"
    col = f'<collision name="c">{g}</collision>' if collide else ""
    return (f'      <link name="{name}"><pose>{" ".join(f"{v:.4f}" for v in pose)}</pose>{col}'
            f'<visual name="v">{g}{mat(rgb, alpha)}</visual></link>\n')


def mesh_link(name, pose, mesh, collision=None, scale=None, emissive=None):
    """A link drawn with a model://nft46 mesh (its own material and texture), with an
    optional box collision (sx, sy, sz) in the link frame."""
    sc = f"<scale>{scale[0]} {scale[1]} {scale[2]}</scale>" if scale else ""
    col = f'<collision name="c"><geometry>{box(*collision)}</geometry></collision>' if collision else ""
    return (f'      <link name="{name}"><pose>{" ".join(f"{v:.4f}" for v in pose)}</pose>{col}'
            f'<visual name="v"><geometry><mesh><uri>model://nft46/meshes/{mesh}.obj</uri>{sc}</mesh></geometry>'
            f'</visual></link>\n')


def tube(name, center, length, axis):
    """A green slotted steel tube (assets.tube_mesh) along axis x, y or z."""
    rot = {"z": (0, 0, 0), "x": (0, math.pi / 2, 0), "y": (math.pi / 2, 0, 0)}[axis]
    size = {"z": (TUBE, TUBE, length), "x": (length, TUBE, TUBE), "y": (TUBE, length, TUBE)}[axis]
    mesh = assets.tube_mesh(length)
    # collision in the world axes: a separate unrotated box
    return (f'      <link name="{name}"><pose>{center[0]:.4f} {center[1]:.4f} {center[2]:.4f} 0 0 0</pose>'
            f'<collision name="c"><geometry>{box(*size)}</geometry></collision>'
            f'<visual name="v"><pose>0 0 0 {rot[0]:.4f} {rot[1]:.4f} {rot[2]:.4f}</pose><geometry><mesh>'
            f'<uri>model://nft46/meshes/{mesh}.obj</uri></mesh></geometry></visual></link>\n')


def box(sx, sy, sz):
    return f"<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>"


def cyl(r, length):
    return f"<cylinder><radius>{r:.4f}</radius><length>{length:.4f}</length></cylinder>"


def model(name, links):
    return f'    <model name="{name}"><static>true</static>\n{"".join(links)}    </model>\n'


def camera(name, eye, target, hfov=1.1, w=960, h=600, rate=10):
    """A fixed observer camera (gz renders it only while something subscribes: the demo video)."""
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    return (f'    <model name="cam_{name}"><static>true</static><pose>{eye[0]} {eye[1]} {eye[2]} 0 {pitch:.4f} {yaw:.4f}</pose>'
            f'<link name="l"><sensor name="{name}" type="camera"><update_rate>{rate}</update_rate><always_on>true</always_on>'
            f'<topic>observer/{name}</topic><camera><horizontal_fov>{hfov}</horizontal_fov>'
            f'<image><width>{w}</width><height>{h}</height></image><clip><near>0.05</near><far>30</far></clip>'
            f'</camera></sensor></link></model>\n')


def arch(prefix, x, y0, z0, radius, r_tube, n=7):
    """A tube looping up and back down in the y-z plane (feeder line), as cylinder segments."""
    L = []
    for s in range(n):
        a0, a1 = math.pi * s / n, math.pi * (s + 1) / n
        p0 = (y0 + radius - radius * math.cos(a0), z0 + radius * math.sin(a0))
        p1 = (y0 + radius - radius * math.cos(a1), z0 + radius * math.sin(a1))
        cy, cz = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
        seg = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        roll = -math.atan2(p1[0] - p0[0], p1[1] - p0[1])   # cylinder axis (z) along the segment
        L.append(link(f"{prefix}_{s}", (x, cy, cz, roll, 0, 0), cyl(r_tube, seg + 0.002), BLACK, False))
    return L


# ------------------------------------------------------------------ the rack
def rack():
    L = []
    rail_z = TOP_H - CH_H - TUBE / 2                # the channels rest on the cross rails
    xs, ys = (RACK_X - FRAME_X, RACK_X + FRAME_X), (RACK_Y - FRAME_Y, RACK_Y + FRAME_Y)
    # frame: posts, rails at channel height and at the top, both ways
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            L.append(tube(f"post_{i}{j}", (x, y, POST_H / 2), POST_H, "z"))
            L.append(link(f"foot_{i}{j}", (x, y, 0.004, 0, 0, 0), cyl(0.02, 0.008), BLACK, False))
        L.append(tube(f"rail_y_{i}", (x, RACK_Y, rail_z), 2 * FRAME_Y - TUBE, "y"))
        L.append(tube(f"top_y_{i}", (x, RACK_Y, POST_H - TUBE / 2), 2 * FRAME_Y - TUBE, "y"))
    for j, y in enumerate(ys):
        L.append(tube(f"rail_x_{j}", (RACK_X, y, rail_z), 2 * FRAME_X + TUBE, "x"))
        L.append(tube(f"top_x_{j}", (RACK_X, y, POST_H + TUBE / 2), 2 * FRAME_X + TUBE, "x"))
    # short centre brace under the feed-end rail (as in the photos)
    L.append(tube("brace_mid", (RACK_X, ys[0], rail_z - 0.12), 0.24, "z"))
    # channels: the NFT profile (lipped cover, tapered sides, closed ends), square holes on top
    for i in range(CH_N):
        x = channel_x(i)
        L.append(mesh_link(f"channel_{i}", (x, CH_Y, TOP_H, 0, 0, 0), "channel",
                           collision=None))
        L.append(f'      <link name="channel_{i}_col"><pose>{x:.4f} {CH_Y:.4f} {TOP_H - CH_H / 2:.4f} 0 0 0</pose>'
                 f'<collision name="c"><geometry>{box(CH_W, CH_LEN, CH_H)}</geometry></collision></link>\n')
        for k in range(SITES):
            if (i, k) not in PLANTS:
                L.append(link(f"hole_{i}_{k}", (x, site_y(k), TOP_H + 0.0006, 0, 0, 0), box(HOLE, HOLE, 0.001), BLACK, False))
        # feeder tube looping into the channel near its feed end
        L += arch(f"feeder_{i}", x + 0.02, CH_Y - CH_LEN / 2 + 0.05, TOP_H, 0.07, 0.0035)
    # light bars hung from the top frame: black Spider Farmer bar and a white bar
    bar_z = POST_H - 0.22
    L.append(link("led_spider_farmer", (RACK_X, RACK_Y + 0.05, bar_z, 0, 0, 0), box(2 * FRAME_X - 0.05, 0.075, 0.035), (0.10, 0.10, 0.11), False))
    L.append(f'      <link name="led_glow"><pose>{RACK_X:.4f} {RACK_Y + 0.05:.4f} {bar_z - 0.0185:.4f} 0 0 0</pose>'
             f'<visual name="v"><geometry>{box(2 * FRAME_X - 0.12, 0.05, 0.002)}</geometry><material>'
             f'<ambient>1 1 0.95 1</ambient><diffuse>1 1 0.95 1</diffuse><emissive>1 0.98 0.92 1</emissive>'
             f'</material></visual></link>\n')
    L.append(link("led_white", (RACK_X, RACK_Y - 0.25, bar_z + 0.03, 0, 0, 0), box(2 * FRAME_X - 0.10, 0.04, 0.025), ALU, False))
    for side in (-1, 1):
        L.append(link(f"hanger_{side + 1}", (RACK_X + side * (FRAME_X - 0.08), RACK_Y + 0.05, (bar_z + POST_H) / 2, 0, 0, 0),
                      cyl(0.002, POST_H - bar_z), BLACK, False))
    # reservoir tub on the floor, clear tray on it, supply hose up to the feed end
    L.append(link("reservoir", (RACK_X + 0.05, RACK_Y + 0.02, 0.17, 0, 0, 0), box(0.95, 0.66, 0.34), BLACK))
    L.append(link("reservoir_rim", (RACK_X + 0.05, RACK_Y + 0.02, 0.345, 0, 0, 0), box(1.0, 0.71, 0.012), (0.10, 0.10, 0.11)))
    L.append(link("tray", (RACK_X + 0.18, RACK_Y - 0.05, 0.40, 0.25, 0, 0.3), box(0.55, 0.30, 0.05), (0.75, 0.72, 0.62), False, alpha=0.55))
    L.append(link("hose", (RACK_X - 0.20, ys[0] - 0.05, (0.34 + rail_z) / 2, 0, 0, 0), cyl(0.009, rail_z - 0.34), BLACK, False))
    # sensor box with a small board and camera, on the middle channels near the drain end
    sx, sy = (channel_x(2) + channel_x(3)) / 2, site_y(SITES - 1) + 0.04
    L.append(link("sensor_box", (sx, sy, TOP_H + 0.04, 0, 0, 0), box(0.12, 0.10, 0.08), WHITE))
    L.append(link("sensor_board", (sx - 0.02, sy - 0.07, TOP_H + 0.015, 0, 0, 0), box(0.09, 0.06, 0.03), (0.15, 0.12, 0.10), False))
    L.append(link("sensor_cam", (sx - 0.02, sy - 0.07, TOP_H + 0.06, 0, 0.6, 0), box(0.025, 0.025, 0.05), BLACK, False))
    return model("nft_4_6_rack", L)


def plants(rng):
    """One lettuce mesh per planted site; returns (sdf, {site: (x, y, base_z, sxy, sz)})."""
    out, sites = [], {}
    for (i, k), (sxy, sz) in PLANTS.items():
        x, y = channel_x(i), site_y(k)
        base = TOP_H + 0.002
        yaw = rng.uniform(-math.pi, math.pi)
        sites[(i, k)] = (x, y, base, sxy, sz)
        out.append(f'    <model name="lettuce_{i}_{k}"><static>true</static><pose>{x:.4f} {y:.4f} {base:.4f} 0 0 {yaw:.3f}</pose>'
                   f'<link name="head"><visual name="v"><geometry><mesh><uri>model://lettuce/meshes/lettuce.obj</uri>'
                   f'<scale>{sxy:.3f} {sxy:.3f} {sz:.3f}</scale></mesh></geometry></visual>'
                   f'<collision name="c"><pose>0 0 {0.055 * sz:.3f} 0 0 0</pose><geometry>{cyl(0.10 * sxy, 0.11 * sz)}</geometry></collision>'
                   f'</link><plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>{LETTUCE_LABEL}</label></plugin>'
                   f'</model>\n')
    return "".join(out), sites


def room():
    L = []
    t = 0.1
    for name, (cx, cy, sx, sy) in {"north": ((X0 + X1) / 2, Y1, X1 - X0 + t, t), "south": ((X0 + X1) / 2, Y0, X1 - X0 + t, t),
                                   "west": (X0, (Y0 + Y1) / 2, t, Y1 - Y0 + t), "east": (X1, (Y0 + Y1) / 2, t, Y1 - Y0 + t)}.items():
        L.append(link(f"wall_{name}", (cx, cy, WALL_H / 2, 0, 0, 0), box(sx, sy, WALL_H), (0.90, 0.90, 0.88)))
    L.append(link("ceiling", ((X0 + X1) / 2, (Y0 + Y1) / 2, WALL_H + 0.025, 0, 0, 0), box(X1 - X0 + t, Y1 - Y0 + t, 0.05), (0.93, 0.93, 0.92)))
    # the grey wall band of the lab (cable raceway with sockets) at about 1 m
    for name, (cx, cy, sx, sy) in {"n": ((X0 + X1) / 2, Y1 - 0.06, X1 - X0 - 0.1, 0.03), "s": ((X0 + X1) / 2, Y0 + 0.06, X1 - X0 - 0.1, 0.03),
                                   "w": (X0 + 0.06, (Y0 + Y1) / 2, 0.03, Y1 - Y0 - 0.1)}.items():
        L.append(link(f"raceway_{name}", (cx, cy, 1.05, 0, 0, 0), box(sx, sy, 0.14), (0.62, 0.63, 0.64), False))
    # lab benches along the back wall, as around the real rack
    for j, y in enumerate((-1.2, 1.4)):
        L.append(link(f"bench_{j}", (X1 - 0.4, y, 0.45, 0, 0, 0), box(0.7, 1.2, 0.9), (0.93, 0.93, 0.92)))
        L.append(link(f"bench_top_{j}", (X1 - 0.4, y, 0.915, 0, 0, 0), box(0.74, 1.24, 0.03), (0.08, 0.08, 0.08)))
    return model("bay", L)


def dock():
    x, y = DOCK
    L = [link("pad", (x, y, 0.002, 0, 0, 0), box(0.90, 0.70, 0.004), (0.95, 0.78, 0.10), collide=False),
         link("pad_stripe", (x - 0.25, y, 0.0045, 0, 0, 0), box(0.10, 0.70, 0.001), BLACK, collide=False),
         link("charging_post", (x - 0.65, y, 0.25, 0, 0, 0), box(0.12, 0.30, 0.50), (0.20, 0.22, 0.25))]
    return model("dock", L)


def main():
    assets.build()
    rng = random.Random(46)
    plant_sdf, sites = plants(rng)
    tx, ty, base = sites[(TARGET_CHANNEL, TARGET_SITE)][:3]
    tz = base + 0.055                                   # middle of the head (the mesh is 0.11 m tall)
    front_edge = RACK_X - FRAME_X - TUBE / 2
    # Tail to the rack (+x): dog centre CAM_BEHIND + START_DIST in front of the lettuce
    wx = tx - CAM_BEHIND - START_DIST
    waypoint = (wx, ty, math.pi)
    standoff = front_edge - wx
    eye_side = (wx + 0.45, ty - 1.45, 1.05)
    eye_front = (wx - 0.55, ty + 0.9, 1.15)
    eye_over = (X0 + 0.3, Y0 + 0.5, 2.3)
    parts = [f"""<?xml version="1.0" ?>
<!--
  cropking_nft46.sdf: lab bay with our CropKing NFT 4-6 (green frame, LED bars), lettuce, a dock.
  GENERATED by build_cropking_nft46_world.py: edit there and rerun.
  @RENDER_ENGINE@ is substituted by start_inside.sh (ogre2 or ogre).
  model://lettuce is gazebo/worlds/models/lettuce (build_lettuce_model.py).
-->
<sdf version="1.9">
  <world name="cropking_nft46">
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
    <scene><ambient>0.7 0.7 0.7 1</ambient><background>0.85 0.86 0.86 1</background><shadows>true</shadows></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.88 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.2 -1.0</direction>
    </light>
    <light type="spot" name="grow_light">
      <cast_shadows>false</cast_shadows><pose>{RACK_X} {RACK_Y + 0.05} {POST_H - 0.25} 0 0 0</pose><direction>0 0 -1</direction>
      <diffuse>0.95 0.95 1.0 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>4</range><constant>0.4</constant><linear>0.2</linear><quadratic>0.05</quadratic></attenuation>
      <spot><inner_angle>0.9</inner_angle><outer_angle>1.4</outer_angle><falloff>1.0</falloff></spot>
    </light>
    <model name="ground"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
      <visual name="visual"><geometry><mesh><uri>model://nft46/meshes/floor.obj</uri></mesh></geometry></visual>
    </link></model>
""", room(), dock(), rack(), plant_sdf,
             camera("side", eye_side, (wx + 0.55, ty, 0.65)),
             camera("front", eye_front, (tx - 0.2, ty, tz - 0.05), hfov=0.95),
             camera("overview", eye_over, (2.4, 0.2, 0.6), hfov=1.2),
             "  </world>\n</sdf>\n"]
    with open(os.path.join(HERE, "cropking_nft46.sdf"), "w", newline="\n") as f:
        f.write("".join(parts))

    # Collision objects for MoveIt's planning scene (world frame), near the arm's side
    def b(name, cx, cy, cz, sx, sy, sz):
        return {"name": name, "type": "box", "pose": [cx, cy, cz, 0, 0, 0], "size": [sx, sy, sz]}
    rail_z = TOP_H - CH_H - TUBE / 2
    scene = [b(f"channel_{i}", channel_x(i), CH_Y, TOP_H - CH_H / 2, CH_W, CH_LEN, CH_H) for i in range(CH_N)]
    scene.append(b("front_rail", RACK_X - FRAME_X, RACK_Y, rail_z, TUBE, 2 * FRAME_Y + TUBE, TUBE))
    scene += [b(f"post_front_{j}", RACK_X - FRAME_X, y, POST_H / 2, TUBE, TUBE, POST_H)
              for j, y in enumerate((RACK_Y - FRAME_Y, RACK_Y + FRAME_Y))]
    for (i, k), (x, y, bz, sxy, sz) in sites.items():
        scene.append({"name": f"lettuce_{i}_{k}", "type": "sphere", "pose": [x, y, bz + 0.055 * sz, 0, 0, 0], "size": [0.10 * sxy]})
    meta = {
        "world": "cropking_nft46", "generated_by": "gazebo/worlds/build_cropking_nft46_world.py",
        "system": "CropKing NFT 4-6 as in the lab: 6 x 4 ft channels (36 sites, 8 in on centre) on a green slotted-steel "
                  "frame with LED bars, 31 in to the channel tops, reservoir tub underneath",
        "bounds": [X0, X1, Y0, Y1], "dock": [DOCK[0], DOCK[1], 0.0],
        "rack": {"center": [RACK_X, RACK_Y], "front_edge_x": front_edge, "channel_top_z": TOP_H,
                 "channel_x": [channel_x(i) for i in range(CH_N)], "site_y": [site_y(k) for k in range(SITES)],
                 "planted": [list(s) for s in PLANTS]},
        "target_plant": {"channel": TARGET_CHANNEL, "site": TARGET_SITE, "center": [tx, ty, tz], "radius": 0.12},
        "waypoint": list(waypoint), "standoff_m": standoff,
        "arm_start": {"joints": [0.0, 0.25, 0.30, 0.0, -0.55, 0.0], "camera_behind_dog_m": CAM_BEHIND,
                      "camera_z_m": CAM_Z, "start_distance_m": START_DIST},
        "observer_cameras": {"side": "observer/side", "front": "observer/front", "overview": "observer/overview"},
        "planning_scene": scene,
    }
    with open(os.path.join(HERE, "cropking_nft46.json"), "w", newline="\n") as f:
        json.dump(meta, f, indent=1)
        f.write("\n")
    print(f"target lettuce ({tx:.3f}, {ty:.3f}, {tz:.3f}); waypoint ({wx:.3f}, {ty:.3f}, yaw 180), "
          f"{standoff:.3f} m from the rack's front posts; camera starts {START_DIST:.2f} m from the lettuce, "
          f"{tz - CAM_Z:+.3f} m below its middle")


if __name__ == "__main__":
    main()
