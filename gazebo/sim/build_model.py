#!/usr/bin/env python3
"""
build_model.py: build the simulated Go2 + D1 model for Gazebo Harmonic.

Inputs are the official, unmodified Unitree descriptions downloaded by
setup_gazebo.sh:
  models/go2_description  (unitreerobotics/unitree_ros, BSD-3-Clause)
  models/d1_description   (Unitree D1 gripper-version URDF package)

Output (in --out):
  go2_d1.urdf       URDF with Gazebo Harmonic extensions
  sim_params.json   constants shared with the bridge and the arm emulator

What the builder changes:
  * Go2 legs become fixed joints at a standing pose ("gliding base"): the body
    is moved by gz VelocityControl from /cmd_vel; there is no gait.
  * The D1 is attached to the Go2 trunk (--mount), with names prefixed "d1_".
    Its URDF ships effort=0 velocity=0 on every joint, which would make the
    joint controllers apply no force, so real limits are patched in.
  * A virtual lidar is placed where the stack's measured mount puts the
    sensor (inverse of sensor_from_base from go2-nav2-wifi
    go2_nav2/config/lidar3d_robot_viz.yaml), level, with a wide vertical FOV.
  * A camera is placed at Unitree's front_camera mount.
  * Gazebo plugins: VelocityControl, OdometryPublisher, JointStatePublisher
    and one JointPositionController per D1 joint.

Uses only the Python standard library, so it can be run and checked anywhere.
"""
import argparse
import json
import math
import os
import posixpath
import re
import sys
import xml.etree.ElementTree as ET

# T_sensor_base (xyz, quaternion xyzw). The laptop's go2_odom_tf turns the lidar
# pose into base_link with go2-nav2-wifi go2_nav2/config/lidar3d_robot_viz.yaml,
# so the sim reads the same file (--sensor-yaml); a mismatch tilts the body in
# RViz and skews Nav2's heading. Fallback when the file is missing: the official
# Go2 URDF lidar mount (radar_joint rpy 0 2.8782 0, inverted), which is what
# scripts/tools/apply-lab-nav-fixes.sh writes into that file.
SENSOR_FROM_BASE = [0.267276, 0.0, -0.120571, 0.0, -0.991341, 0.0, 0.131316]
SENSOR_YAML_KEY = re.compile(r"^\s*sensor_from_base\s*:\s*\[([^\]]*)\]", re.MULTILINE)

# Standing pose (rad). Gives a trunk height of ~0.29 m with Go2 leg lengths.
STAND = {"hip": 0.0, "thigh": 0.9, "calf": -1.8}
LEG_JOINT = re.compile(r"^(FL|FR|RL|RR)_(hip|thigh|calf)_joint$")
LEG_ORDER = [f"{leg}_{part}_joint" for leg in ("FR", "FL", "RR", "RL")
             for part in ("hip", "thigh", "calf")]

# D1: protocol gripper angle (servo degrees, 0 = closed) -> finger travel.
GRIPPER_OPEN_DEG = 65.0
FINGER_TRAVEL_M = 0.03
D1_REVOLUTE = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
D1_FINGERS = ["Joint7_1", "Joint7_2"]

MODEL_NAME = "go2"
CLEARANCE_M = 0.01   # feet hover this far above the ground (no contact jitter)


# --------------------------------------------------------------- 3D maths
def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def rpy_of(R):
    p = math.asin(max(-1.0, min(1.0, -R[2][0])))
    if abs(math.cos(p)) > 1e-9:
        return math.atan2(R[2][1], R[2][2]), p, math.atan2(R[1][0], R[0][0])
    return math.atan2(-R[1][2], R[1][1]), p, 0.0


def rot_axis(axis, a):
    n = math.sqrt(sum(v * v for v in axis)) or 1.0
    x, y, z = (v / n for v in axis)
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def rot_quat(x, y, z, w):
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mv(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def floats(s, n=3, default=0.0):
    vals = [float(v) for v in (s or "").split()]
    return (vals + [default] * n)[:n]


def fmt(vals):
    return " ".join(f"{v:.6g}" for v in vals)


# ------------------------------------------------------------- URDF helpers
def strip_ros1_extensions(robot):
    """Drop ROS 1 gazebo/transmission blocks; Harmonic plugins are added below."""
    for tag in ("gazebo", "transmission"):
        for el in list(robot.findall(tag)):
            robot.remove(el)


def rewrite_meshes(robot, package, mesh_root):
    prefix = f"package://{package}/"
    for mesh in robot.iter("mesh"):
        fn = mesh.get("filename", "")
        if fn.startswith(prefix):
            # Gazebo runs on Linux: build a POSIX path whatever OS runs this script.
            mesh.set("filename", "file://" + posixpath.join(mesh_root, package, fn[len(prefix):]))


def origin_of(joint):
    o = joint.find("origin")
    if o is None:
        o = ET.SubElement(joint, "origin")
    return o


def fix_leg_joints(robot, stand):
    """Turn leg joints into fixed joints at the standing angles."""
    fixed = {}
    for j in robot.findall("joint"):
        m = LEG_JOINT.match(j.get("name", ""))
        if not m:
            continue
        q = stand[m.group(2)]
        o = origin_of(j)
        axis = floats(j.find("axis").get("xyz") if j.find("axis") is not None else "1 0 0")
        R = mm(rot_rpy(*floats(o.get("rpy"))), rot_axis(axis, q))
        o.set("rpy", fmt(rpy_of(R)))
        j.set("type", "fixed")
        for tag in ("axis", "limit", "dynamics", "safety_controller"):
            for el in list(j.findall(tag)):
                j.remove(el)
        fixed[j.get("name")] = q
    if len(fixed) != 12:
        raise SystemExit(f"expected 12 Go2 leg joints, found {len(fixed)}: {sorted(fixed)}")
    return fixed


def link_pose_chain(robot, root):
    """World (root-relative) pose of every link through the (now fixed) joints."""
    children = {}
    for j in robot.findall("joint"):
        children.setdefault(j.find("parent").get("link"), []).append(j)
    poses = {root: ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0])}
    stack = [root]
    while stack:
        parent = stack.pop()
        Rp, tp = poses[parent]
        for j in children.get(parent, []):
            o = j.find("origin")
            R = rot_rpy(*floats(o.get("rpy") if o is not None else ""))
            t = floats(o.get("xyz") if o is not None else "")
            child = j.find("child").get("link")
            poses[child] = (mm(Rp, R), [a + b for a, b in zip(tp, mv(Rp, t))])
            stack.append(child)
    return poses


def lowest_point(robot, poses):
    """Lowest collision point (sphere/box/cylinder approximations) below the trunk."""
    lowest = 0.0
    for link in robot.findall("link"):
        name = link.get("name")
        if name not in poses:
            continue
        R, t = poses[name]
        for col in link.findall("collision"):
            o = col.find("origin")
            c = mv(R, floats(o.get("xyz") if o is not None else ""))
            z = t[2] + c[2]
            g = col.find("geometry")
            if g is None:
                continue
            if g.find("sphere") is not None:
                z -= float(g.find("sphere").get("radius"))
            elif g.find("cylinder") is not None:
                z -= float(g.find("cylinder").get("radius"))
            elif g.find("box") is not None:
                z -= max(floats(g.find("box").get("size"))) / 2
            lowest = min(lowest, z)
    return lowest


def add_link(robot, name, mass=0.01):
    link = ET.SubElement(robot, "link", name=name)
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", value=f"{mass}")
    ET.SubElement(inertial, "inertia", ixx="1e-5", ixy="0", ixz="0", iyy="1e-5", iyz="0", izz="1e-5")
    return link


def add_fixed(robot, name, parent, child, xyz, rpy=(0, 0, 0)):
    j = ET.SubElement(robot, "joint", name=name, type="fixed")
    ET.SubElement(j, "parent", link=parent)
    ET.SubElement(j, "child", link=child)
    ET.SubElement(j, "origin", xyz=fmt(xyz), rpy=fmt(rpy))


def import_d1(robot, d1_root, mesh_root, parent, mount):
    """Merge the D1 URDF under `parent`, names prefixed d1_, real limits patched."""
    d1 = ET.parse(os.path.join(d1_root, "urdf", "d1_description.urdf")).getroot()
    strip_ros1_extensions(d1)
    rewrite_meshes(d1, "d1_description", mesh_root)
    names = {l.get("name") for l in d1.findall("link")}
    for el in d1:
        if el.tag == "link":
            el.set("name", "d1_" + el.get("name"))
        elif el.tag == "joint":
            el.set("name", "d1_" + el.get("name"))
            for side in ("parent", "child"):
                s = el.find(side)
                s.set("link", "d1_" + s.get("link"))
            lim = el.find("limit")
            if lim is not None:
                prismatic = el.get("type") == "prismatic"
                lim.set("effort", "20" if prismatic else "30")
                lim.set("velocity", "0.2" if prismatic else "3.14")
            if el.get("type") in ("revolute", "prismatic") and el.find("dynamics") is None:
                ET.SubElement(el, "dynamics", damping="0.5", friction="0.0")
        elif el.tag == "material":
            el.set("name", "d1_" + el.get("name", "mat"))
        robot.append(el)
    if "base_link" not in names:
        raise SystemExit("D1 URDF has no base_link")
    add_fixed(robot, "d1_mount_joint", parent, "d1_base_link", mount[:3], mount[3:])
    limits = {}
    for j in d1.findall("joint"):
        lim = j.find("limit")
        if lim is not None:
            limits[j.get("name")] = [float(lim.get("lower")), float(lim.get("upper"))]
    return limits


def gz(robot, xml_text):
    robot.append(ET.fromstring(xml_text))


def load_sensor_from_base(path):
    """sensor_from_base from a go2_odom_tf params file; the built-in default if absent."""
    if not path or not os.path.isfile(path):
        print(f"WARN: {path or 'no --sensor-yaml'} not found; using the official Go2 URDF lidar mount",
              file=sys.stderr)
        return list(SENSOR_FROM_BASE), "default (Go2 URDF)"
    with open(path) as f:
        m = SENSOR_YAML_KEY.search(f.read())   # first uncommented value
    vals = [float(v) for v in m.group(1).split(",")] if m else []
    if len(vals) != 7:
        sys.exit(f"{path}: expected 'sensor_from_base: [x, y, z, qx, qy, qz, qw]'")
    return vals, path


def build(go2_root, d1_root, mesh_root, out_dir, mount, stand, sensor_from_base):
    go2_urdf = os.path.join(go2_root, "urdf", "go2_description.urdf")
    robot = ET.parse(go2_urdf).getroot()
    robot.set("name", MODEL_NAME)
    strip_ros1_extensions(robot)
    rewrite_meshes(robot, "go2_description", mesh_root)
    stand_angles = fix_leg_joints(robot, stand)

    roots = {l.get("name") for l in robot.findall("link")} - {j.find("child").get("link") for j in robot.findall("joint")}
    if roots != {"base"}:
        raise SystemExit(f"expected Go2 root link 'base', found {roots}")

    poses = link_pose_chain(robot, "base")
    spawn_z = -lowest_point(robot, poses) + CLEARANCE_M

    # Virtual lidar at the measured sensor position, level, wide vertical FOV.
    t, q = sensor_from_base[:3], sensor_from_base[3:]
    R = rot_quat(*q)
    lidar_xyz = [-sum(R[r][c] * t[r] for r in range(3)) for c in range(3)]
    add_link(robot, "sim_lidar")
    add_fixed(robot, "sim_lidar_joint", "base", "sim_lidar", lidar_xyz)

    cam = next((j for j in robot.findall("joint") if j.get("name") == "front_camera_joint"), None)
    cam_xyz = floats(cam.find("origin").get("xyz")) if cam is not None else [0.327, 0.0, 0.043]
    add_link(robot, "sim_camera")
    add_fixed(robot, "sim_camera_joint", "base", "sim_camera", cam_xyz)

    d1_limits = import_d1(robot, d1_root, mesh_root, "base", mount)
    d1_joints = ["d1_" + n for n in D1_REVOLUTE + D1_FINGERS]
    missing = [n for n in d1_joints if n not in {j.get("name") for j in robot.findall("joint")}]
    if missing:
        raise SystemExit(f"D1 joints missing: {missing}")

    gz(robot, f"""<gazebo reference="sim_lidar"><sensor name="lidar3d" type="gpu_lidar">
  <topic>/sim/lidar</topic><gz_frame_id>sim_lidar</gz_frame_id>
  <update_rate>10</update_rate><always_on>1</always_on><visualize>false</visualize>
  <lidar><scan>
    <horizontal><samples>180</samples><resolution>1</resolution><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
    <vertical><samples>16</samples><resolution>1</resolution><min_angle>-0.4363</min_angle><max_angle>0.6981</max_angle></vertical>
  </scan><range><min>0.15</min><max>25.0</max><resolution>0.01</resolution></range></lidar>
</sensor></gazebo>""")
    gz(robot, """<gazebo reference="sim_camera"><sensor name="front_camera" type="camera">
  <topic>/camera/image_raw</topic><gz_frame_id>camera_link</gz_frame_id>
  <update_rate>10</update_rate><always_on>1</always_on>
  <camera><horizontal_fov>1.2</horizontal_fov><image><width>640</width><height>360</height><format>R8G8B8</format></image>
  <clip><near>0.05</near><far>40</far></clip></camera>
</sensor></gazebo>""")

    plugins = [
        f"""<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
  <topic>/model/{MODEL_NAME}/cmd_vel</topic></plugin>""",
        f"""<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
  <odom_frame>sim_world</odom_frame><robot_base_frame>base</robot_base_frame>
  <odom_topic>/model/{MODEL_NAME}/odometry</odom_topic><odom_publish_frequency>50</odom_publish_frequency>
  <dimensions>3</dimensions></plugin>""",
        """<plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
  <topic>/sim/d1/joint_state</topic>""" + "".join(f"<joint_name>{n}</joint_name>" for n in d1_joints) + "</plugin>",
    ]
    for n in d1_joints:
        prismatic = n.startswith("d1_Joint7")
        plugins.append(f"""<plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
  <joint_name>{n}</joint_name><topic>/sim/d1/{n}/cmd_pos</topic>
  <use_velocity_commands>true</use_velocity_commands><p_gain>{20 if prismatic else 8}</p_gain>
  <cmd_max>{0.2 if prismatic else 3.0}</cmd_max><cmd_min>{-0.2 if prismatic else -3.0}</cmd_min></plugin>""")
    gz(robot, "<gazebo>" + "".join(plugins) + "</gazebo>")

    # Warn (don't fail) about meshes missing on this machine: --mesh-root may
    # describe another machine, but inside the sim container they must exist.
    missing = sorted({m.get("filename")[len("file://"):] for m in robot.iter("mesh")
                      if m.get("filename", "").startswith("file://")
                      and not os.path.isfile(m.get("filename")[len("file://"):])})
    if missing:
        print(f"WARN: {len(missing)} mesh file(s) not found here, e.g. {missing[0]}", file=sys.stderr)

    os.makedirs(out_dir, exist_ok=True)
    if hasattr(ET, "indent"):
        ET.indent(robot, space="  ")
    urdf_path = os.path.join(out_dir, "go2_d1.urdf")
    ET.ElementTree(robot).write(urdf_path, encoding="utf-8", xml_declaration=True)

    params = {
        "model_name": MODEL_NAME,
        "spawn_z": round(spawn_z, 4),
        "sensor_from_base": sensor_from_base,
        "lidar_offset_xyz": [round(v, 5) for v in lidar_xyz],
        "camera_offset_xyz": cam_xyz,
        "leg_joint_names": LEG_ORDER,
        "leg_joint_positions": [stand_angles[n] for n in LEG_ORDER],
        "d1_joints": d1_joints,
        "d1_limits_rad": d1_limits,          # keys already carry the d1_ prefix
        "gripper_open_deg": GRIPPER_OPEN_DEG,
        "finger_travel_m": FINGER_TRAVEL_M,
        "mount": mount,
    }
    with open(os.path.join(out_dir, "sim_params.json"), "w") as f:
        json.dump(params, f, indent=2)
    return urdf_path, params


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    here = os.path.dirname(os.path.abspath(__file__))
    models = os.path.join(here, "..", "models")
    ap.add_argument("--go2", default=os.path.join(models, "go2_description"))
    ap.add_argument("--d1", default=os.path.join(models, "d1_description"))
    ap.add_argument("--mesh-root", default=None,
                    help="directory containing go2_description/ and d1_description/ as seen by Gazebo "
                         "(default: the parent of --go2)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--mount", default=os.environ.get("SIM_D1_MOUNT", "0.0 0 0.057 0 0 0"),
                    help="D1 base pose on the Go2 trunk: 'x y z roll pitch yaw' (m, rad). "
                         "Default: centre of the trunk top")
    ap.add_argument("--sensor-yaml", default=os.environ.get("SIM_SENSOR_YAML", os.path.expanduser(
                        "~/files/go2-nav2-wifi/go2_nav2/config/lidar3d_robot_viz.yaml")),
                    help="go2_odom_tf params file to take sensor_from_base from, so the sim's lidar "
                         "mount matches the laptop stack (default: the go2-nav2-wifi checkout)")
    a = ap.parse_args()
    mount = floats(a.mount, 6)
    mesh_root = a.mesh_root or os.path.dirname(os.path.abspath(a.go2))
    mesh_root = mesh_root.replace("\\", "/").rstrip("/")
    for p in (a.go2, a.d1):
        if not os.path.isdir(p):
            sys.exit(f"missing model directory: {p} (run gazebo/setup_gazebo.sh)")
    sensor_from_base, source = load_sensor_from_base(a.sensor_yaml)
    urdf, params = build(a.go2, a.d1, mesh_root, a.out, mount, STAND, sensor_from_base)
    print(f"wrote {urdf}")
    print(f"sensor_from_base from {source}")
    print(f"spawn_z={params['spawn_z']} m  lidar at {params['lidar_offset_xyz']}  "
          f"D1 joints: {len(params['d1_joints'])}")


if __name__ == "__main__":
    main()
