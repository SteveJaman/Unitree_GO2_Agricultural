#!/usr/bin/env python3
"""The D1 arm on the Go2, as a URDF for robot_state_publisher (start_inside.sh).

Turns the dashboard's copy of Unitree's d1_description into TF frames that sit
next to the Go2's own:

  * every link is prefixed "d1_" (the D1's root is called base_link, like the
    Go2's); joint names stay Joint1..Joint7_2, as on /d1_arm/joint_states
  * base_link (the Go2) -> d1_base_link: the mount, 0.057 m above the trunk
    origin (as gazebo/sim/build_model.py --mount and the dashboard)
  * d1_Link6 -> wrist_camera_link -> wrist_camera_optical_frame: the wrist
    camera (not mounted yet). Its pose on Link6 is "x y z roll pitch yaw"
    (TWIN_WRIST_CAMERA_TF, default 0.09 m along the gripper axis, Link6 +z,
    looking along it); the optical frame is z forward, x right, y down
  * visuals, collisions and inertials are dropped: TF only

Usage: d1_tf_urdf.py <d1_description.urdf> [camera xyz rpy] > d1_tf.urdf
"""
import sys
import xml.etree.ElementTree as ET

MOUNT_XYZ = '0 0 0.057'


def main():
    src = sys.argv[1]
    cam = (sys.argv[2] if len(sys.argv) > 2 else '0 0 0.09 0 0 0').split()
    if len(cam) != 6:
        sys.exit('camera pose must be "x y z roll pitch yaw"')
    d1 = ET.parse(src).getroot()
    robot = ET.Element('robot', name='d1_on_go2')
    ET.SubElement(robot, 'link', name='base_link')
    for el in d1:
        if el.tag == 'link':
            ET.SubElement(robot, 'link', name='d1_' + el.get('name'))
        elif el.tag == 'joint':
            j = ET.SubElement(robot, 'joint', name=el.get('name'), type=el.get('type'))
            for child in el:
                if child.tag in ('parent', 'child'):
                    ET.SubElement(j, child.tag, link='d1_' + child.get('link'))
                elif child.tag in ('origin', 'axis', 'limit'):
                    j.append(child)

    def fixed(name, parent, child, xyz, rpy='0 0 0'):
        ET.SubElement(robot, 'link', name=child)
        j = ET.SubElement(robot, 'joint', name=name, type='fixed')
        ET.SubElement(j, 'parent', link=parent)
        ET.SubElement(j, 'child', link=child)
        ET.SubElement(j, 'origin', xyz=xyz, rpy=rpy)

    robot.remove(robot.find("link[@name='d1_base_link']"))
    fixed('d1_mount', 'base_link', 'd1_base_link', MOUNT_XYZ)
    fixed('wrist_camera_mount', 'd1_Link6', 'wrist_camera_link', ' '.join(cam[:3]), ' '.join(cam[3:]))
    # Optical frame (REP 103): z forward along the view axis, which is the camera link's z already.
    fixed('wrist_camera_optical', 'wrist_camera_link', 'wrist_camera_optical_frame', '0 0 0')
    ET.indent(robot)
    sys.stdout.write(ET.tostring(robot, encoding='unicode') + '\n')


if __name__ == '__main__':
    main()
