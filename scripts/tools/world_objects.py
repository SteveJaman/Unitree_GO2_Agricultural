#!/usr/bin/env python3
"""Object list for the sim's ground-truth detector (sim_box_detector.py).

  world_objects.py gazebo/worlds/WORLD.json OUT.json

Accepts both world truth formats:
  * an "objects" list (cluttered_items.json, build_items_world.py), used as is;
  * "obstacles" (lab_office.json, build_lab_world.py): walls, 3DGEMS furniture and
    SDF boxes; the non-wall ones are converted.
OUT.json gets {world, spawn, objects, stand_in}. stand_in is the comma list of
classes the stand-in reports: the ones YOLO lacks (YOLO covers chair, couch,
dining table, ...). The sim's map frame is its world frame, so positions carry
over unchanged. Exit 3 when the world has no object list (e.g. agri_rooms).
Used by wifi_3d_nav.sh --sim --semantic and tests/traversability/lab_semantic.sh.
"""
import json
import sys

# lab_office kind -> detection class.
KIND_CLASS = {
    "chair_1": "chair", "chair_2": "chair",
    "desk_yellow": "desk", "desk_brown": "desk", "desk_drawer": "drawer_unit",
    "drawer_white": "drawer_unit",
    "table_conference_2": "dining table", "cafe_table": "dining table",
    "sofa_set_1": "couch",
    "bookshelf_large": "bookshelf",
    "file_cabinet_large": "file_cabinet", "file_cabinet_small": "file_cabinet",
    "side_table_1": "side_table",
    "trash_bin": "trash_bin",
}
BOX_CLASS = {"cubicle_wall": "partition", "pallet": "pallet", "tool_cart": "cart",
             "corridor_cart": "cart", "pc_tower": "pc_tower", "boxes": "cardboard_box"}

# Classes the stand-in reports. cluttered_items: the list of the n = 10 trials
# (laptop too: the modelled laptop is boxes, which YOLO does not recognise).
STAND_IN_ITEMS = ("power_strip,plant_saucer,cable,plant_pot,shoe,toolbox,trash_bin,side_table,"
                  "bookshelf,file_cabinet,laptop")
STAND_IN = ("trash_bin,side_table,bookshelf,file_cabinet,desk,drawer_unit,partition,pallet,cart,"
            "pc_tower,cardboard_box")


def box_class(name):
    for prefix, cls in BOX_CLASS.items():
        if name.startswith(prefix):
            return cls
    return "box"


def convert(layout):
    """Objects in sim_box_detector's format, or None when the world has none."""
    if "objects" in layout:
        return layout["objects"]
    if "obstacles" not in layout:
        return None
    out = []
    for o in layout["obstacles"]:
        if o["kind"] == "wall":
            continue
        cls = box_class(o["name"]) if o["kind"] == "box" else KIND_CLASS.get(o["kind"], o["kind"])
        out.append({"name": o["name"], "class": cls, "x": o["cx"], "y": o["cy"], "z": o["h"] / 2,
                    "yaw": o["yaw"], "sx": o["sx"], "sy": o["sy"], "sz": o["h"], "height": o["h"]})
    return out


def main():
    layout = json.load(open(sys.argv[1]))
    objects = convert(layout)
    if objects is None:
        print(f"{sys.argv[1]}: no object list", file=sys.stderr)
        return 3
    stand_in = STAND_IN_ITEMS if "objects" in layout else STAND_IN
    with open(sys.argv[2], "w") as f:
        json.dump({"world": layout.get("world"), "spawn": layout.get("spawn"), "objects": objects,
                   "stand_in": stand_in}, f, indent=1)
    print(f"{len(objects)} objects -> {sys.argv[2]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
