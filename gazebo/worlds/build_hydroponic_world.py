#!/usr/bin/env python3
"""
build_hydroponic_world.py: write hydroponic_station.sdf, a greenhouse bay with
a CropKing NFT 8-4 hydroponic system and a docking pad, and
hydroponic_station.json (rack geometry, plant sites, the mission's target
plant and waypoints, observer cameras) for the full-mission test
(src/d1_moveit_config/scripts/full_mission_test.py).

  y = 3 +---------------------------------------------+
        |                     light rack               |
        |   [dock]  ->  waypoint   |=| CropKing NFT 8-4 |  4 channels x 12 sites
        |   (0,0)      (1.89, ys)  |=| centred (3.0, 0) |  long axis along y
        |                          |=|  reservoir under  |
  y = -3+---------------------------------------------+
       x = -2                                        x = 6

CropKing NFT 8-4, at its real size (CropKing / retailer listings, 2026-11):
four 8 ft NFT channels with removable top covers, 1 in square plant sites
8 in on centre (48 sites), aluminium stand 40 in x 103 in, 34 in high
(56 in with the light support rack), supply manifold at the high end, return
gutter at the low end into a 25 gal reservoir with a 400 GPH pump. Assumed
where the listings say nothing: 4.5 in x 2 in channels (CropKing's standard
4.5 in channel), 10 in channel pitch, 1 % slope, 6 stand legs.

Lettuce heads (green spheres on a stem in a net cup) sit in every site; the
mission's target is the front channel's site at y = TARGET_Y. The dock is a
painted pad (no collision: the sim's feet hover 1 cm above the floor) with a
charging post behind it. Walls reach a 2.7 m ceiling, as in lab_office.

Usage: python3 build_hydroponic_world.py
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
IN = 0.0254

# Room
X0, X1, Y0, Y1, WALL_H = -2.0, 6.0, -3.0, 3.0, 2.7

# CropKing NFT 8-4
RACK_X, RACK_Y = 3.0, 0.0
STAND_W, STAND_L, STAND_H = 40 * IN, 103 * IN, 34 * IN      # x, y, height to the channel tops
LIGHT_H = 56 * IN
CH_N, CH_LEN, CH_W, CH_H = 4, 96 * IN, 4.5 * IN, 2 * IN
CH_PITCH = 10 * IN
SITES, SITE_PITCH = 12, 8 * IN
SLOPE = 0.01                                                   # high end at +y (supply manifold)
TUBE = 1.5 * IN                                                # aluminium square tube
CUP_R, CUP_H, STEM_H, HEAD_R = 0.02, 0.02, 0.02, 0.055
TARGET_CHANNEL, TARGET_SITE = 0, 8                             # front channel, site at y = +0.508

DOCK = (0.0, 0.0)
STANDOFF = 0.60                    # mission waypoint: dog base 0.6 m in front of the stand's front edge

ALU, PVC, BLACK, GREEN, WHITE = (0.75, 0.76, 0.78), (0.95, 0.95, 0.93), (0.08, 0.08, 0.09), (0.30, 0.65, 0.22), (0.97, 0.97, 0.95)


def channel_x(i):
    return RACK_X - (CH_N - 1) / 2 * CH_PITCH + i * CH_PITCH


def site_y(k):
    return RACK_Y - (SITES - 1) / 2 * SITE_PITCH + k * SITE_PITCH


def top_z(y):
    """Channel cover height at y (1 % slope, STAND_H at the rack's middle)."""
    return STAND_H + SLOPE * (y - RACK_Y)


def head_center(i, k):
    y = site_y(k)
    return channel_x(i), y, top_z(y) + CUP_H / 2 + STEM_H + HEAD_R


# ------------------------------------------------------------------ SDF bits
def mat(rgb, a=1.0):
    c = f"{rgb[0]:.2f} {rgb[1]:.2f} {rgb[2]:.2f} {a}"
    return f"<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>"


def link(name, pose, geom, rgb, collide=True):
    g = f"<geometry>{geom}</geometry>"
    col = f'<collision name="c">{g}</collision>' if collide else ""
    return (f'      <link name="{name}"><pose>{" ".join(f"{v:.4f}" for v in pose)}</pose>{col}'
            f'<visual name="v">{g}{mat(rgb)}</visual></link>\n')


def box(sx, sy, sz):
    return f"<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>"


def cyl(r, length):
    return f"<cylinder><radius>{r:.4f}</radius><length>{length:.4f}</length></cylinder>"


def sphere(r):
    return f"<sphere><radius>{r:.4f}</radius></sphere>"


def model(name, links):
    return f'    <model name="{name}"><static>true</static>\n{"".join(links)}    </model>\n'


def camera(name, eye, target, hfov=1.1, w=1280, h=800):
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    return (f'    <model name="cam_{name}"><static>true</static><pose>{eye[0]} {eye[1]} {eye[2]} 0 {pitch:.4f} {yaw:.4f}</pose>'
            f'<link name="l"><sensor name="{name}" type="camera"><update_rate>2</update_rate><always_on>true</always_on>'
            f'<topic>observer/{name}</topic><camera><horizontal_fov>{hfov}</horizontal_fov>'
            f'<image><width>{w}</width><height>{h}</height></image><clip><near>0.05</near><far>30</far></clip>'
            f'</camera></sensor></link></model>\n')


# ------------------------------------------------------------------ the rack
def rack():
    L = []
    x_lo, x_hi = RACK_X - STAND_W / 2 + TUBE / 2, RACK_X + STAND_W / 2 - TUBE / 2
    y_lo, y_hi = RACK_Y - STAND_L / 2 + TUBE / 2, RACK_Y + STAND_L / 2 - TUBE / 2
    rail_z = STAND_H - CH_H - TUBE / 2 - 0.005          # channels rest on the top rails
    # legs (6), top rails along y and x, lower rails
    for i, x in enumerate((x_lo, x_hi)):
        for j, y in enumerate((y_lo, RACK_Y, y_hi)):
            L.append(link(f"leg_{i}{j}", (x, y, rail_z / 2, 0, 0, 0), box(TUBE, TUBE, rail_z), ALU))
        L.append(link(f"rail_y_{i}", (x, RACK_Y, rail_z, 0, 0, 0), box(TUBE, STAND_L, TUBE), ALU))
        L.append(link(f"low_rail_y_{i}", (x, RACK_Y, 0.15, 0, 0, 0), box(TUBE, STAND_L, TUBE), ALU))
    for j, y in enumerate((y_lo, RACK_Y, y_hi)):
        L.append(link(f"rail_x_{j}", (RACK_X, y, rail_z, 0, 0, 0), box(STAND_W, TUBE, TUBE), ALU))
    # channels: body + white top cover, tilted for the slope (+y high)
    roll = -math.atan(SLOPE)        # rotation about x: +y end up
    for i in range(CH_N):
        x = channel_x(i)
        L.append(link(f"channel_{i}", (x, RACK_Y, top_z(RACK_Y) - CH_H / 2, roll, 0, 0),
                      box(CH_W, CH_LEN, CH_H - 0.004), WHITE))
        L.append(link(f"cover_{i}", (x, RACK_Y, top_z(RACK_Y) - 0.002, roll, 0, 0),
                      box(CH_W + 0.004, CH_LEN, 0.004), PVC))
    # supply manifold (1 in PVC) at the high end with a feeder into each channel
    y_man = RACK_Y + CH_LEN / 2 + 0.03
    z_man = top_z(y_man) + 0.03
    L.append(link("supply_manifold", (RACK_X, y_man, z_man, 0, math.pi / 2, 0), cyl(0.0167, STAND_W - 0.05), PVC))
    for i in range(CH_N):
        L.append(link(f"feeder_{i}", (channel_x(i), y_man - 0.02, z_man - 0.02, 0, 0, 0), cyl(0.004, 0.05), BLACK, False))
    # return gutter at the low end, drain to the reservoir
    y_gut = RACK_Y - CH_LEN / 2 - 0.05
    L.append(link("return_gutter", (RACK_X, y_gut, top_z(y_gut) - 0.06, 0, 0, 0), box(STAND_W - 0.06, 0.10, 0.08), PVC))
    # 25 gal reservoir under the low end (black tank, pump inside), drain and supply line
    res_x, res_y, res_z = RACK_X + 0.08, RACK_Y - STAND_L / 2 + 0.40, 0.19
    L.append(link("reservoir", (res_x, res_y, res_z, 0, 0, 0), box(0.70, 0.45, 0.38), BLACK))
    L.append(link("drain", (RACK_X + 0.35, y_gut, (0.38 + top_z(y_gut) - 0.10) / 2, 0, 0, 0),
                  cyl(0.024, top_z(y_gut) - 0.10 - 0.38), PVC))
    L.append(link("supply_floor", (RACK_X + 0.42, (res_y + y_man) / 2, 0.03, math.pi / 2, 0, 0), cyl(0.0167, y_man - res_y), PVC))
    L.append(link("supply_riser", (RACK_X + 0.42, y_man, z_man / 2, 0, 0, 0), cyl(0.0167, z_man), PVC))
    # light support rack (56 in): uprights at the ends, top bar, two LED fixtures
    for j, y in enumerate((y_lo, y_hi)):
        L.append(link(f"light_post_{j}", (RACK_X, y, (rail_z + LIGHT_H) / 2, 0, 0, 0), box(TUBE, TUBE, LIGHT_H - rail_z), ALU))
    L.append(link("light_bar", (RACK_X, RACK_Y, LIGHT_H, 0, 0, 0), box(TUBE, STAND_L, TUBE), ALU))
    for j, y in enumerate((-0.6, 0.6)):
        L.append(link(f"led_{j}", (RACK_X, y, LIGHT_H - 0.12, 0, 0, 0), box(0.30, 1.10, 0.04), (0.92, 0.92, 0.95)))
    return model("cropking_nft_8_4", L)


def plants():
    L = []
    for i in range(CH_N):
        for k in range(SITES):
            x, y, zc = head_center(i, k)
            top = top_z(y)
            n = f"{i}_{k}"
            L.append(link(f"cup_{n}", (x, y, top, 0, 0, 0), cyl(CUP_R, CUP_H), BLACK))
            L.append(link(f"stem_{n}", (x, y, top + CUP_H / 2 + STEM_H / 2, 0, 0, 0), cyl(0.006, STEM_H), (0.45, 0.70, 0.30)))
            L.append(link(f"head_{n}", (x, y, zc, 0, 0, 0), sphere(HEAD_R), GREEN))
    return model("lettuce", L)


def room():
    L = []
    t = 0.1
    for name, (cx, cy, sx, sy) in {"north": ((X0 + X1) / 2, Y1, X1 - X0 + t, t), "south": ((X0 + X1) / 2, Y0, X1 - X0 + t, t),
                                   "west": (X0, (Y0 + Y1) / 2, t, Y1 - Y0 + t), "east": (X1, (Y0 + Y1) / 2, t, Y1 - Y0 + t)}.items():
        L.append(link(f"wall_{name}", (cx, cy, WALL_H / 2, 0, 0, 0), box(sx, sy, WALL_H), (0.86, 0.87, 0.85)))
    L.append(link("ceiling", ((X0 + X1) / 2, (Y0 + Y1) / 2, WALL_H + 0.025, 0, 0, 0), box(X1 - X0 + t, Y1 - Y0 + t, 0.05), (0.93, 0.93, 0.92)))
    return model("bay", L)


def dock():
    x, y = DOCK
    L = [link("pad", (x, y, 0.002, 0, 0, 0), box(0.90, 0.70, 0.004), (0.95, 0.78, 0.10), collide=False),
         link("pad_stripe", (x - 0.25, y, 0.0045, 0, 0, 0), box(0.10, 0.70, 0.001), BLACK, collide=False),
         link("charging_post", (x - 0.65, y, 0.25, 0, 0, 0), box(0.12, 0.30, 0.50), (0.20, 0.22, 0.25))]
    return model("dock", L)


def main():
    tx, ty, tz = head_center(TARGET_CHANNEL, TARGET_SITE)
    front_edge = RACK_X - STAND_W / 2
    # Tail to the rack: the D1 is mounted facing backward, as on the real dog.
    waypoint = (front_edge - STANDOFF, ty, math.pi)
    eye_side = (waypoint[0] + 0.15, ty - 1.75, 1.30)
    eye_front = (waypoint[0] - 0.35, ty + 1.15, 1.25)
    eye_over = (X0 + 0.4, Y0 + 0.4, 2.45)
    parts = [f"""<?xml version="1.0" ?>
<!--
  hydroponic_station.sdf: greenhouse bay with a CropKing NFT 8-4 and a dock.
  GENERATED by build_hydroponic_world.py: edit there and rerun.
  @RENDER_ENGINE@ is substituted by start_inside.sh (ogre2 or ogre).
-->
<sdf version="1.9">
  <world name="hydroponic_station">
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
    <scene><ambient>0.75 0.75 0.75 1</ambient><background>0.8 0.85 0.9 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.88 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.3 0.2 -1.0</direction>
    </light>
    <model name="ground"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
      <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>{mat((0.58, 0.58, 0.56))}</visual>
    </link></model>
""", room(), dock(), rack(), plants(),
             camera("side", eye_side, (waypoint[0] + 0.45, ty, 0.70)),
             camera("front", eye_front, (tx - 0.15, ty, tz - 0.05), hfov=0.9),
             camera("overview", eye_over, (2.4, 0.2, 0.6), hfov=1.2),
             "  </world>\n</sdf>\n"]
    with open(os.path.join(HERE, "hydroponic_station.sdf"), "w", newline="\n") as f:
        f.write("".join(parts))

    # Collision objects for MoveIt's planning scene (world frame), near the target
    def b(name, cx, cy, cz, sx, sy, sz, roll=0.0):
        return {"name": name, "type": "box", "pose": [cx, cy, cz, roll, 0, 0], "size": [sx, sy, sz]}
    rail_z = STAND_H - CH_H - TUBE / 2 - 0.005
    scene = [b(f"channel_{i}", channel_x(i), RACK_Y, top_z(RACK_Y) - CH_H / 2, CH_W, CH_LEN, CH_H, -math.atan(SLOPE))
             for i in range(CH_N)]
    scene.append(b("front_rail", RACK_X - STAND_W / 2 + TUBE / 2, RACK_Y, rail_z, TUBE, STAND_L, TUBE))
    scene += [b(f"leg_front_{j}", RACK_X - STAND_W / 2 + TUBE / 2, y, rail_z / 2, TUBE, TUBE, rail_z)
              for j, y in enumerate((RACK_Y - STAND_L / 2 + TUBE / 2, RACK_Y, RACK_Y + STAND_L / 2 - TUBE / 2))]
    for i in range(2):
        for k in range(SITES):
            x, y, zc = head_center(i, k)
            if abs(y - ty) < 0.9:
                scene.append({"name": f"lettuce_{i}_{k}", "type": "sphere", "pose": [x, y, zc, 0, 0, 0], "size": [HEAD_R]})
    meta = {
        "world": "hydroponic_station", "generated_by": "gazebo/worlds/build_hydroponic_world.py",
        "system": "CropKing NFT 8-4 (real size: 40 x 103 in stand, 34 in to the channel tops, 4 x 8 ft channels, 48 sites 8 in on centre)",
        "bounds": [X0, X1, Y0, Y1], "dock": [DOCK[0], DOCK[1], 0.0],
        "rack": {"center": [RACK_X, RACK_Y], "front_edge_x": front_edge, "channel_top_z": STAND_H,
                 "channel_x": [channel_x(i) for i in range(CH_N)], "site_y": [site_y(k) for k in range(SITES)]},
        "target_plant": {"channel": TARGET_CHANNEL, "site": TARGET_SITE, "center": [tx, ty, tz], "radius": HEAD_R},
        "waypoint": list(waypoint), "standoff_m": STANDOFF,
        "observer_cameras": {"side": "observer/side", "front": "observer/front", "overview": "observer/overview"},
        "planning_scene": scene,
    }
    with open(os.path.join(HERE, "hydroponic_station.json"), "w", newline="\n") as f:
        json.dump(meta, f, indent=1)
        f.write("\n")
    print(f"target plant ({tx:.3f}, {ty:.3f}, {tz:.3f}); waypoint ({waypoint[0]:.3f}, {waypoint[1]:.3f}); "
          f"plant {tx - waypoint[0]:.3f} m behind the dog, in front of the arm")


if __name__ == "__main__":
    main()
