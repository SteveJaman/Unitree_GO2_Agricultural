#!/usr/bin/env python3
"""
build_plant_inspection_world.py: write plant_inspection.sdf, a small room with
a bench of potted plants behind the robot, and plant_inspection.json (where
the plants are). It gives the plant detector (scripts/runtime/plant_detection.sh)
something a COCO-trained model can recognise: the other worlds' plants are
spheres and cylinders.

The D1 is mounted facing the tail, so the bench is behind the robot's default
spawn (SIM_SPAWN "-4.5 0 0"): with the arm at MoveIt's "home" pose the wrist
camera (0.79 m up, looking back) sees the middle three plants about 1.7 m away.

        y
        ^      wall
   +----|------------------------------+
   |  [bench: pots along y]            |
   |        <- camera   [Go2 ->]       |   robot at (-4.5, 0), heading +x
   +-----------------------------------+--> x

Each plant is a terracotta pot with soil (modelled here) and foliage from a
mesh model: plant_1, plant_2 and plant_3, 3DGEMS models from the Gazebo Models
& Worlds Collection (leonhartyao/gazebo_models_worlds_collection, GPL-3.0), at
the pinned commit setup_gazebo.sh uses for the lab furniture. They have no
textures: the leaves are flat greens. YOLOv5s (COCO) labels them "potted plant"
with a confidence of 0.5 to 0.8 from the wrist camera's distance.
They are downloaded into gazebo/models/plants (not in git):
  ./gazebo/setup_gazebo.sh --plants        (= this script with --download)

Usage: python3 build_plant_inspection_world.py [--download]
"""
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = "plant_inspection"

COLLECTION_URL = "https://github.com/leonhartyao/gazebo_models_worlds_collection.git"
COLLECTION_COMMIT = "cce115b82691b7c529a02f47e5efa391145a4ca1"   # same pin as setup_gazebo.sh
COLLECTION_MODELS = ["plant_1", "plant_2", "plant_3"]

ROBOT = (-4.5, 0.0, 0.0)                 # run_sim.sh's default SIM_SPAWN
X0, X1, Y0, Y1, WALL_H = -7.4, -2.5, -2.4, 2.4, 2.4
BENCH_X, BENCH_D, BENCH_L, BENCH_H = -6.60, 0.50, 3.9, 0.45   # centre x, depth (x), length (y), top height
POT_R, POT_H = 0.11, 0.17
SOIL_DROP = 0.015

TERRACOTTA, SOIL, WOOD, WALL, FLOOR = (0.72, 0.36, 0.22), (0.20, 0.14, 0.09), (0.62, 0.50, 0.36), (0.90, 0.90, 0.87), (0.55, 0.55, 0.53)

# Foliage models. Their scale is fixed by each model's own model.sdf, and their meshes are
# not centred on the model origin. "offset" moves the stem onto the pot axis and its base
# onto the soil (model frame, m); "size" is the foliage once there: width, height above the
# soil. Both measured from top and side renders.
FOLIAGE = {
    "plant_1": {"offset": (0.0, 0.0, 0.0), "size": (0.45, 0.27)},        # thin arching stems
    "plant_2": {"offset": (0.0, 0.0, 0.0), "size": (1.00, 0.72)},        # long blades
    "plant_3": {"offset": (-0.29, -0.33, -0.17), "size": (0.60, 0.42)},  # broad heart-shaped leaves
}

# name, foliage, y along the bench, yaw of the foliage (rad)
PLANTS = [
    ("plant_a", "plant_3", -1.60, 0.0),
    ("plant_b", "plant_1", -0.80, 0.0),
    ("plant_c", "plant_2", 0.00, 0.0),
    ("plant_d", "plant_3", 0.80, 2.2),
    ("plant_e", "plant_1", 1.60, 2.6),
]


def mat(rgb):
    c = f"{rgb[0]:.2f} {rgb[1]:.2f} {rgb[2]:.2f} 1"
    return f"<material><ambient>{c}</ambient><diffuse>{c}</diffuse><specular>0.05 0.05 0.05 1</specular></material>"


def link(name, pose, geom, rgb, collide=True):
    g = f"<geometry>{geom}</geometry>"
    col = f'<collision name="c">{g}</collision>' if collide else ""
    return (f'      <link name="{name}"><pose>{" ".join(f"{v:.4f}" for v in pose)}</pose>{col}'
            f'<visual name="v">{g}{mat(rgb)}</visual></link>\n')


def box(sx, sy, sz):
    return f"<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>"


def cyl(r, length):
    return f"<cylinder><radius>{r:.4f}</radius><length>{length:.4f}</length></cylinder>"


def model(name, links):
    return f'    <model name="{name}"><static>true</static>\n{"".join(links)}    </model>\n'


def room():
    t, cx, cy = 0.1, (X0 + X1) / 2, (Y0 + Y1) / 2
    walls = [("w", X0 - t / 2, cy, t, Y1 - Y0 + 2 * t), ("e", X1 + t / 2, cy, t, Y1 - Y0 + 2 * t),
             ("s", cx, Y0 - t / 2, X1 - X0, t), ("n", cx, Y1 + t / 2, X1 - X0, t)]
    return model("room", [link(f"wall_{n}", (x, y, WALL_H / 2, 0, 0, 0), box(sx, sy, WALL_H), WALL) for n, x, y, sx, sy in walls])


def bench():
    top = 0.04
    links = [link("top", (BENCH_X, 0, BENCH_H - top / 2, 0, 0, 0), box(BENCH_D, BENCH_L, top), WOOD)]
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        links.append(link(f"leg_{i}", (BENCH_X + sx * (BENCH_D / 2 - 0.04), sy * (BENCH_L / 2 - 0.06), (BENCH_H - top) / 2, 0, 0, 0),
                          box(0.05, 0.05, BENCH_H - top), WOOD))
    return model("bench", links)


def soil_z():
    return BENCH_H + POT_H - SOIL_DROP


def plant(name, foliage, y, yaw):
    """A pot with soil (one static model) and the foliage model standing in it."""
    rim = 0.012
    pot = model(f"{name}_pot", [
        link("pot", (BENCH_X, y, BENCH_H + POT_H / 2, 0, 0, 0), cyl(POT_R, POT_H), TERRACOTTA),
        link("rim", (BENCH_X, y, BENCH_H + POT_H - rim / 2, 0, 0, 0), cyl(POT_R + 0.008, rim), TERRACOTTA, collide=False),
        link("soil", (BENCH_X, y, soil_z(), 0, 0, 0), cyl(POT_R - 0.008, 0.004), SOIL, collide=False),
    ])
    ox, oy, dz = FOLIAGE[foliage]["offset"]
    dx = ox * math.cos(yaw) - oy * math.sin(yaw)      # the offset turns with the foliage
    dy = ox * math.sin(yaw) + oy * math.cos(yaw)
    leaves = (f'    <include><uri>model://{foliage}</uri><name>{name}</name><static>true</static>'
              f'<pose>{BENCH_X + dx:.4f} {y + dy:.4f} {soil_z() + dz:.4f} 0 0 {yaw:.4f}</pose></include>\n')
    return pot + leaves


def truth():
    plants = []
    for name, foliage, y, _ in PLANTS:
        w, h = FOLIAGE[foliage]["size"]
        top = soil_z() + h
        plants.append({"name": name, "foliage": foliage, "class": "potted plant",
                       "base": [BENCH_X, y, BENCH_H], "center": [BENCH_X, y, (BENCH_H + top) / 2],
                       "size": [max(w, 2 * POT_R), max(w, 2 * POT_R), top - BENCH_H]})
    return {"world": OUT, "generated_by": "gazebo/worlds/build_plant_inspection_world.py",
            "bounds": [X0, X1, Y0, Y1], "spawn": list(ROBOT),
            "bench": {"center_x": BENCH_X, "top_z": BENCH_H, "length_y": BENCH_L},
            "plants": plants}


def world():
    parts = [f"""<?xml version="1.0" ?>
<!--
  plant_inspection.sdf: potted plants on a bench behind the robot, for the plant detector.
  GENERATED by build_plant_inspection_world.py: edit there and rerun.
  @RENDER_ENGINE@ is substituted by start_inside.sh (ogre2 or ogre).
  Needs gazebo/models/plants (./gazebo/setup_gazebo.sh --plants).
-->
<sdf version="1.9">
  <world name="{OUT}">
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
    <scene><ambient>0.7 0.7 0.7 1</ambient><background>0.8 0.85 0.9 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.88 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.5 0.2 -1.0</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.35 0.35 0.35 1</diffuse><specular>0 0 0 1</specular><direction>-1.0 -0.3 -0.4</direction>
    </light>
    <model name="ground"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
      <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>{mat(FLOOR)}</visual>
    </link></model>
""", room(), bench()]
    parts += [plant(*p) for p in PLANTS]
    parts.append("  </world>\n</sdf>\n")
    return "".join(parts)


def download():
    """Fetch the foliage models into gazebo/models/plants (not in git)."""
    import shutil
    import subprocess
    import tempfile
    root = os.path.join(HERE, "..", "models", "plants")
    os.makedirs(root, exist_ok=True)

    missing = [m for m in COLLECTION_MODELS if not os.path.isfile(os.path.join(root, m, "model.sdf"))]
    if missing:
        with tempfile.TemporaryDirectory() as tmp:
            def git(*a):
                subprocess.run(["git", "-C", tmp, *a], check=True)
            git("init", "-q")
            git("remote", "add", "origin", COLLECTION_URL)
            git("config", "core.sparseCheckout", "true")
            with open(os.path.join(tmp, ".git", "info", "sparse-checkout"), "w") as f:
                f.write("/LICENSE\n" + "".join(f"/models/{m}/\n" for m in missing))
            git("fetch", "-q", "--depth", "1", "--filter=blob:none", "origin", COLLECTION_COMMIT)
            git("checkout", "-q", "FETCH_HEAD")
            for m in missing:
                shutil.copytree(os.path.join(tmp, "models", m), os.path.join(root, m))
                # Gazebo Harmonic cannot read the Gazebo Classic material script some of the
                # models name, and then renders them grey: drop it, the model's own colours apply.
                sdf = os.path.join(root, m, "model.sdf")
                text = open(sdf).read()
                with open(sdf, "w", newline="\n") as f:
                    f.write(re.sub(r"\s*<script>.*?</script>", "", text, flags=re.S))
            shutil.copy(os.path.join(tmp, "LICENSE"), os.path.join(root, "LICENSE.gazebo_models_worlds_collection"))
        print("installed", *missing)

    print("plant models in", os.path.normpath(root))


def main():
    if "--download" in sys.argv:
        download()
        return
    with open(os.path.join(HERE, OUT + ".sdf"), "w", newline="\n") as f:
        f.write(world())
    with open(os.path.join(HERE, OUT + ".json"), "w", newline="\n") as f:
        json.dump(truth(), f, indent=1)
        f.write("\n")
    print(f"wrote {OUT}.sdf and .json: {len(PLANTS)} plants")


if __name__ == "__main__":
    main()
