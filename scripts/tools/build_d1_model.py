#!/usr/bin/env python3
"""Build the dashboard's D1 arm model from Unitree's d1_description package.

Reads the URDF and its STL meshes (~130k triangles, 6.4 MB), decimates each
link (quadric), and writes what the web dashboard's urdf-loader reads:

  <out>/urdf/d1_description.urdf   the URDF, unchanged
  <out>/meshes/<Link>.STL          binary STL, decimated
  <out>/LICENSE.md                 where the files come from

The package is BSD licensed (Unitree Robotics). gazebo/setup_gazebo.sh
--models downloads it (sha256-checked) to gazebo/models/d1_description.
Needs Open3D, so run it in the go2-3d image:

  docker run --rm --user "$(id -u):$(id -g)" \\
      -v "$PWD/gazebo/models/d1_description":/desc:ro -v "$PWD":/repo \\
      go2-3d:local python3 /repo/scripts/tools/build_d1_model.py /desc /repo/twin/webapp/public/d1_description
"""
import os
import shutil
import sys
import xml.etree.ElementTree as ET

import open3d as o3d

# Triangle budget per link: the whole arm stays near 9k triangles, light next
# to the ~25k of the Go2 on a machine without 3D acceleration.
BUDGET = {'base_link': 600, 'Link1': 1600, 'Link2': 2000, 'Link3': 1200, 'Link4': 1400,
          'Link5': 1000, 'Link6': 1000, 'Link7_1': 250, 'Link7_2': 250}


def convert(src, dst, budget):
    mesh = o3d.io.read_triangle_mesh(src)
    before = len(mesh.triangles)
    if before > budget:
        mesh = mesh.simplify_quadric_decimation(budget)
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()
    if not o3d.io.write_triangle_mesh(dst, mesh, write_ascii=False):
        sys.exit(f'cannot write {dst}')
    return before, len(mesh.triangles)


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    desc, out = sys.argv[1], sys.argv[2]
    urdf = os.path.join(desc, 'urdf', 'd1_description.urdf')
    if not os.path.isfile(urdf):
        sys.exit(f'{urdf} not found (gazebo/setup_gazebo.sh --models downloads it)')
    os.makedirs(os.path.join(out, 'urdf'), exist_ok=True)
    os.makedirs(os.path.join(out, 'meshes'), exist_ok=True)
    shutil.copyfile(urdf, os.path.join(out, 'urdf', 'd1_description.urdf'))

    total = [0, 0]
    for link in ET.parse(urdf).getroot().findall('link'):
        mesh = link.find('visual/geometry/mesh')
        if mesh is None:
            continue
        name = os.path.basename(mesh.get('filename'))
        before, after = convert(os.path.join(desc, 'meshes', name), os.path.join(out, 'meshes', name),
                                BUDGET.get(link.get('name'), 1000))
        total[0] += before
        total[1] += after
        print(f'{name:14s} {before:7d} -> {after:6d} triangles')
    print(f'{"total":14s} {total[0]:7d} -> {total[1]:6d} triangles')

    with open(os.path.join(out, 'LICENSE.md'), 'w') as f:
        f.write('# D1 arm model\n\n'
                "Unitree Robotics' d1_description (D1 arm, gripper version), BSD license\n"
                '(package.xml). Meshes decimated for the web dashboard by\n'
                'scripts/tools/build_d1_model.py; the URDF is unchanged.\n')


if __name__ == '__main__':
    main()
