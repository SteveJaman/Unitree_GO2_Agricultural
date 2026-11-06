#!/usr/bin/env python3
"""build_items_world.py: writes cluttered_items.sdf and cluttered_items.json.

The cluttered arena rebuilt with real objects: scanned household items (Google
Scanned Objects on Gazebo Fuel), Fuel models, the lab's 3DGEMS furniture, and a
modelled power strip (Fuel has none). Run ./gazebo/setup_gazebo.sh --items once.

Visuals are the real meshes. Collisions are boxes from the measured mesh bounds:
they are what the feet and trunk touch, and the JSON holds the same boxes as the
ground truth for tests/traversability/analyze.py. Ground uses collide bitmask 1,
so feet (SIM_GAIT=1, bitmask 2) touch objects and never the floor.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = "cluttered_items"
ARENA = (-2.0, 12.0, -4.5, 4.5)
SPAWN = (0.0, 0.0, 0.0)
GOALS = [("goal_1_mid", 5.0, 0.0), ("goal_2_end", 10.0, 0.0), ("goal_3_return", 0.0, 0.0)]

MUG_W = "Threshold_Porcelain_Coffee_Mug_All_Over_Bead_White"
MUG_B = "Cole_Hardware_Mug_Classic_Blue"
BOOK = "Eat_to_Live_The_Amazing_NutrientRich_Program_for_Fast_and_Sustained_Weight_Loss_Revised_Edition_Book"
SCANS = "https://fuel.gazebosim.org/1.0/GoogleResearch/models"

# name, class, source, x, y, yaw deg, (sx, sy, height) from the mesh, options
# source "scan:DIR" is meshes/model.obj, "mesh:DIR/FILE" any other mesh, "model:NAME"
# is an included 3DGEMS model, "strip" is the modelled power strip.
ITEMS = [
    # Step over
    ("power_strip_1", "power_strip", "strip", 1.9, 0.75, 90, (0.30, 0.055, 0.04), {}),
    ("keyboard_1", "keyboard", "scan:Kanex_MultiSync_Wireless_Keyboard", 1.9, -0.75, 90, (0.442, 0.124, 0.024), {}),
    ("book_1", "book", "scan:" + BOOK, 4.1, 0.65, 80, (0.141, 0.21, 0.051), {"roll": 90, "lift": 0.029}),
    ("saucer_1", "plant_saucer", "scan:Cole_Hardware_Plant_Saucer_Brown_125", 4.2, -0.7, 0, (0.311, 0.31, 0.046), {}),
    ("cable_coil_1", "cable", "scan:Shaxon_100_Molded_Category_6_RJ45RJ45_Shielded_Patch_Cord_White", 5.7, 0.15, 0, (0.175, 0.175, 0.061), {}),
    ("pot_1", "plant_pot", "scan:Ecoforms_Plant_Container_B4_Har", 7.6, -0.4, 0, (0.101, 0.101, 0.072), {}),
    ("mug_blue_1", "cup", "scan:" + MUG_B, 6.3, -0.2, 30, (0.166, 0.126, 0.095), {}),
    ("power_strip_2", "power_strip", "strip", 7.0, 0.55, 80, (0.30, 0.055, 0.04), {}),
    ("mug_white_1", "cup", "scan:" + MUG_W, 6.6, 0.95, 200, (0.131, 0.097, 0.115), {}),
    ("shoe_1", "shoe", "scan:ASICS_GEL1140V_WhiteRoyalSilver", 8.4, 0.1, 60, (0.106, 0.281, 0.124), {}),
    ("plastic_cup_1", "cup", "mesh:Plastic_Cup/meshes/plastic_cup.dae", 8.0, -0.9, 0, (0.075, 0.075, 0.13), {"lift": 0.065}),
    # Caution
    ("bottle_1", "bottle", "mesh:Water_Bottle/meshes/WaterBottle.glb", 5.0, -1.6, 0, (0.087, 0.087, 0.208), {"roll": 90, "scale": 0.8, "lift": 0.104}),
    # Blocked
    ("shelf_1", "bookshelf", "model:bookshelf_large", 3.0, 0.0, 0, (1.66, 0.42, 1.84), {"origin": (0.03, 0.0)}),
    ("chair_1", "chair", "model:chair_2", 4.2, 2.3, -90, (0.38, 0.52, 0.87), {"origin": (0.01, 0.02)}),
    ("cabinet_1", "file_cabinet", "model:file_cabinet_small", 6.0, 2.6, 0, (0.61, 0.75, 0.65), {"origin": (0.045, 0.015)}),
    ("trash_bin_1", "trash_bin", "model:trash_bin", 8.2, 1.8, 0, (0.24, 0.24, 0.30), {"origin": (0.0, 0.0)}),
    ("backpack_1", "backpack", "scan:Jansport_School_Backpack_Blue_Streak", 5.6, 1.5, 120, (0.358, 0.369, 0.44), {}),
    ("toolbox_1", "toolbox", "mesh:Toolbox/meshes/Toolbox.dae", 6.8, -1.7, 20, (0.51, 0.218, 0.299), {"scale": 0.01}),
    ("side_table_1", "side_table", "model:side_table_1", 9.0, -2.6, 0, (0.45, 0.45, 0.64), {"origin": (-0.005, 0.045)}),
]


def box_col(sx, sy, sz, name="c"):
    return (f'<collision name="{name}"><pose>0 0 {sz / 2:.4f} 0 0 0</pose><geometry><box>'
            f'<size>{sx} {sy} {sz}</size></box></geometry></collision>')


def box_vis(name, size, z, color, x=0.0, y=0.0):
    r, g, b = color
    return (f'<visual name="{name}"><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose><geometry><box><size>{size}</size>'
            f'</box></geometry><material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material></visual>')


def strip_visuals():
    """A six-outlet power strip with a switch and a cord: the box is the collision."""
    out = [box_vis("body", "0.30 0.055 0.04", 0.02, (0.93, 0.93, 0.90))]
    for i in range(6):
        x = -0.115 + i * 0.041
        out.append(box_vis(f"socket{i}", "0.03 0.032 0.002", 0.0405, (0.18, 0.18, 0.20), x=x))
    out.append(box_vis("switch", "0.02 0.03 0.006", 0.042, (0.85, 0.12, 0.10), x=0.133))
    for i in range(5):
        y = 0.0 if i else 0.0
        out.append(box_vis(f"cord{i}", "0.14 0.009 0.009", 0.0045, (0.08, 0.08, 0.08), x=-0.21 - i * 0.135, y=y))
    return "".join(out)


def item_sdf(it):
    name, _cls, src, x, y, yaw, (sx, sy, sz), opt = it
    yaw_r = math.radians(yaw)
    kind, _, ref = src.partition(":")
    if kind == "model":
        ox, oy = opt["origin"]
        c, s = math.cos(yaw_r), math.sin(yaw_r)
        px, py = x - (c * ox - s * oy), y - (s * ox + c * oy)
        return (f'    <include><uri>model://{ref}</uri><name>{name}</name><static>true</static>'
                f'<pose>{px:.3f} {py:.3f} 0 0 0 {yaw_r:.4f}</pose></include>\n')
    if kind == "strip":
        visual = strip_visuals()
    else:
        uri = f"model://{ref}/meshes/model.obj" if kind == "scan" else f"model://{ref}"
        scale = opt.get("scale", 1.0)
        roll = math.radians(opt.get("roll", 0))
        visual = (f'<visual name="v"><pose>0 0 {opt.get("lift", 0.0)} {roll:.4f} 0 0</pose><geometry><mesh>'
                  f'<uri>{uri}</uri><scale>{scale} {scale} {scale}</scale></mesh></geometry></visual>')
    return (f'    <model name="{name}"><static>true</static><pose>{x} {y} 0 0 0 {yaw_r:.4f}</pose>'
            f'<link name="link">{box_col(sx, sy, sz)}{visual}</link></model>\n')


def wall(name, x, y, sx, sy):
    return (f'    <model name="{name}"><static>true</static><pose>{x} {y} 0.25 0 0 0</pose><link name="link">'
            f'<collision name="c"><geometry><box><size>{sx} {sy} 0.5</size></box></geometry></collision>'
            f'<visual name="v"><geometry><box><size>{sx} {sy} 0.5</size></box></geometry>'
            f'<material><ambient>0.72 0.72 0.70 1</ambient><diffuse>0.76 0.76 0.74 1</diffuse></material></visual>'
            f'</link></model>\n')


def world():
    x0, x1, y0, y1 = ARENA
    cx, cy, lx, ly = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
    walls = [("wall_north", cx, y1, lx, 0.1), ("wall_south", cx, y0, lx, 0.1),
             ("wall_west", x0, cy, 0.1, ly), ("wall_east", x1, cy, 0.1, ly)]
    return f"""<?xml version="1.0" ?>
<!-- Generated by build_items_world.py. Spawn: SIM_SPAWN="0 0 0". @RENDER_ENGINE@ is substituted by start_inside.sh. -->
<sdf version="1.9">
  <world name="{OUT}">
    <physics name="4ms" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>@RENDER_ENGINE@</render_engine></plugin>
    <scene><ambient>0.75 0.75 0.75 1</ambient><background>0.8 0.82 0.85 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.88 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.3 0.2 -1.0</direction></light>
    <model name="ground"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        <surface><contact><collide_bitmask>0x01</collide_bitmask></contact></surface></collision>
      <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        <material><ambient>0.62 0.60 0.56 1</ambient><diffuse>0.66 0.64 0.60 1</diffuse></material></visual>
    </link></model>
{''.join(wall(*w) for w in walls)}{''.join(item_sdf(i) for i in ITEMS)}  </world>
</sdf>
"""


def truth():
    objects = []
    for name, cls, _src, x, y, yaw, (sx, sy, sz), _opt in ITEMS:
        objects.append({"name": name, "class": cls, "x": x, "y": y, "z": round(sz / 2, 4),
                        "yaw": round(math.radians(yaw), 4), "sx": sx, "sy": sy, "sz": sz, "height": sz})
    x0, x1, y0, y1 = ARENA
    walls = [{"name": "wall_north", "x": (x0 + x1) / 2, "y": y1, "sx": x1 - x0, "sy": 0.1},
             {"name": "wall_south", "x": (x0 + x1) / 2, "y": y0, "sx": x1 - x0, "sy": 0.1},
             {"name": "wall_west", "x": x0, "y": 0.0, "sx": 0.1, "sy": y1 - y0},
             {"name": "wall_east", "x": x1, "y": 0.0, "sx": 0.1, "sy": y1 - y0}]
    return {"world": OUT, "spawn": list(SPAWN), "goals": GOALS, "objects": objects, "walls": walls}


FUEL = [("GoogleResearch", MUG_W), ("GoogleResearch", MUG_B), ("OpenRobotics", "Plastic Cup"),
        ("GoogleResearch", "ASICS_GEL1140V_WhiteRoyalSilver"), ("GoogleResearch", BOOK),
        ("GoogleResearch", "Kanex_MultiSync_Wireless_Keyboard"),
        ("GoogleResearch", "Cole_Hardware_Plant_Saucer_Brown_125"),
        ("GoogleResearch", "Ecoforms_Plant_Container_B4_Har"),
        ("GoogleResearch", "Jansport_School_Backpack_Blue_Streak"), ("OpenRobotics", "Toolbox"),
        ("GoogleResearch", "Shaxon_100_Molded_Category_6_RJ45RJ45_Shielded_Patch_Cord_White"),
        ("iche033", "Water Bottle")]


def download():
    """Fetch the Fuel models into gazebo/models/items (not in git)."""
    import io
    import urllib.parse
    import urllib.request
    import zipfile
    root = os.path.join(HERE, "..", "models", "items")
    os.makedirs(root, exist_ok=True)
    for owner, name in FUEL:
        target = os.path.join(root, name.replace(" ", "_"))
        if os.path.isdir(target):
            continue
        q = urllib.parse.quote(name)
        url = f"https://fuel.gazebosim.org/1.0/{owner}/models/{q}/tip/{q}.zip"
        req = urllib.request.Request(url, headers={"User-Agent": "go2-sim-setup"})
        zipfile.ZipFile(io.BytesIO(urllib.request.urlopen(req, timeout=120).read())).extractall(target)
        print("installed", name)


def main():
    if "--download" in sys.argv:
        download()
        return
    with open(os.path.join(HERE, OUT + ".sdf"), "w", newline="\n") as f:
        f.write(world())
    with open(os.path.join(HERE, OUT + ".json"), "w", newline="\n") as f:
        json.dump(truth(), f, indent=1)
    print(f"wrote {OUT}.sdf and .json: {len(ITEMS)} objects")


if __name__ == "__main__":
    main()
