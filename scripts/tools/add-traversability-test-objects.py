#!/usr/bin/env python3
"""Add the traversability test objects to gazebo/worlds/build_rooms_world.py.

Safe to run repeatedly: it removes any earlier version of these objects first.
Then regenerate the world:  python3 gazebo/worlds/build_rooms_world.py

  cable_1    0.03 m   no collision  -> should be stepped over
  mat_1      0.02 m   no collision  -> should be stepped over
  phone_1    0.08 m   collision     -> low but fragile: avoid
  toolbox_1  0.12 m   collision     -> above the step limit: avoid
  vase_1     0.30 m   collision     -> tall and fragile: avoid

No-collision objects are a stand-in for stepping: the sim base only glides, so it
cannot lift its feet. Positions are relative to the corridor wall (Y_CORR) and
the room-2 door (ROOM_DOORS_X[1]); check them in Gazebo and adjust below.
"""
import os
import re
import sys

PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                    'gazebo', 'worlds', 'build_rooms_world.py')
BEGIN, END = '# --- traversability test objects (begin) ---', '# --- traversability test objects (end) ---'

COLORS = '''CABLE_RGB = "0.05 0.05 0.05"
MAT_RGB = "0.15 0.35 0.65"
FRAGILE_RGB = "0.85 0.10 0.10"
TOOLBOX_RGB = "0.90 0.75 0.10"
'''
HELPER = '''def visual_box_link(name, x, y, z, sx, sy, sz, rgb):
    """Like box_link but with no collision: the dog drives through it
    (stand-in for stepping over, since the sim base only glides)."""
    return f"""      <link name="{name}">
        <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
        <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>
      </link>
"""


'''
OBJECTS = f'''        {BEGIN}
        visual_box_link("cable_1", ROOM_DOORS_X[1], Y_CORR + 1.2, 0.015, 0.9, 0.03, 0.03, CABLE_RGB),
        visual_box_link("mat_1", ROOM_DOORS_X[1], Y_CORR + 2.4, 0.010, 1.0, 0.8, 0.02, MAT_RGB),
        box_link("toolbox_1", ROOM_DOORS_X[1] - 0.3, Y_CORR + 3.6, 0.06, 0.5, 0.3, 0.12, TOOLBOX_RGB),
        box_link("phone_1", ROOM_DOORS_X[1] + 0.5, Y_CORR + 3.6, 0.04, 0.15, 0.08, 0.08, FRAGILE_RGB),
        box_link("vase_1", ROOM_DOORS_X[1] + 0.8, Y_CORR + 2.0, 0.15, 0.12, 0.12, 0.30, FRAGILE_RGB),
        {END}
'''

ANCHOR = '        box_link("planter", 2.6, 4.35, 0.25, 0.5, 0.5, 0.5, CRATE_RGB),\n'

s = open(PATH).read()
# Remove any earlier version (marked block, or the unmarked first draft).
s = re.sub(r'^[ \t]*' + re.escape(BEGIN) + r'.*?' + re.escape(END) + r'\n', '', s, flags=re.S | re.M)
s = re.sub(r'^.*"(?:cable_1|mat_1|vase_1|phone_1|toolbox_1)".*\n', '', s, flags=re.M)
s = re.sub(r'^[ \t]*# (?:Traversability test objects|Step-over test objects|Fragile / tall).*\n', '', s, flags=re.M)
# Drop old colour lines so they are re-added with the right values.
s = re.sub(r'^(?:CABLE_RGB|MAT_RGB|FRAGILE_RGB|TOOLBOX_RGB) = .*\n', '', s, flags=re.M)
if s.count(ANCHOR) != 1:
    sys.exit('ERROR: planter line not found exactly once; build_rooms_world.py differs from the expected version.')
bench = 'BENCH_RGB = "0.35 0.35 0.40"\n'
if s.count(bench) != 1:
    sys.exit('ERROR: BENCH_RGB line not found.')
s = s.replace(bench, bench + COLORS)
if 'def visual_box_link' not in s:
    if s.count('def wall(') != 1:
        sys.exit('ERROR: "def wall(" not found exactly once.')
    s = s.replace('def wall(', HELPER + 'def wall(')
s = s.replace(ANCHOR, ANCHOR + OBJECTS)
open(PATH, 'w').write(s)
print('patched ' + os.path.normpath(PATH))
