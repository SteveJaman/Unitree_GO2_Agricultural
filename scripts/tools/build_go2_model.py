#!/usr/bin/env python3
"""Build the dashboard's Go2 model from Unitree's official go2_description.

Reads the URDF and its COLLADA meshes (~400k triangles, 25 MB), decimates
each part (quadric, per material so colours stay), and writes what the web
dashboard loads:

  <out>/<mesh>.ply   binary PLY: positions, normals, linear vertex colours
  <out>/go2.json     links, revolute joints (origin, axis, limits), visuals

The meshes are BSD licensed (go2_description, Unitree Robotics). Needs
Open3D (Assimp reads .dae), so run it in the go2-3d image:

  docker run --rm --user "$(id -u):$(id -g)" \\
      -v ~/files/go2-nav2-wifi/go2_description:/desc:ro -v "$PWD":/repo \\
      go2-3d:local python3 /repo/scripts/tools/build_go2_model.py /desc /repo/twin/webapp/public/go2
"""
import json
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import open3d as o3d

# Triangle budget per mesh: the whole dog stays near 25k triangles, cheap
# enough for a machine without 3D acceleration.
BUDGET = {'base': 6000, 'hip': 1500, 'thigh': 1500, 'thigh_mirror': 1500,
          'calf': 900, 'calf_mirror': 900, 'foot': 300}


def floats(s, n=3):
    return [float(v) for v in (s or ' '.join(['0'] * n)).split()]


def node_matrix(src):
    """The scene node's 4x4 transform. Each go2_description .dae has one node
    that turns the Blender export (y up) into the URDF frame; Open3D's
    reader ignores it."""
    with open(src, encoding='utf-8') as f:
        m = re.search(r'<node[^>]*>\s*<matrix[^>]*>([^<]+)</matrix>', f.read())
    return np.array(m.group(1).split(), float).reshape(4, 4) if m else np.eye(4)


def convert(src, dst, budget):
    model = o3d.io.read_triangle_model(src)
    node = node_matrix(src)
    total = sum(len(m.mesh.triangles) for m in model.meshes) or 1
    out = o3d.geometry.TriangleMesh()
    for mi in model.meshes:
        mesh = mi.mesh.transform(node)
        target = max(40, int(budget * len(mesh.triangles) / total))
        if len(mesh.triangles) > target:
            mesh = mesh.simplify_quadric_decimation(target)
        mesh.remove_degenerate_triangles()
        mesh.remove_unreferenced_vertices()
        mesh.compute_vertex_normals()
        rgb = np.clip(np.asarray(model.materials[mi.material_idx].base_color[:3]), 0, 1)
        mesh.paint_uniform_color(rgb)
        out += mesh
    o3d.io.write_triangle_mesh(dst, out, write_ascii=False, write_vertex_normals=True,
                               write_vertex_colors=True)
    return len(out.triangles)


def main():
    desc, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    urdf = ET.parse(os.path.join(desc, 'urdf', 'go2_description.urdf')).getroot()

    joints = []
    for j in urdf.findall('joint'):
        o = j.find('origin')
        entry = {
            'name': j.get('name'), 'type': j.get('type'),
            'parent': j.find('parent').get('link'), 'child': j.find('child').get('link'),
            'xyz': floats(o.get('xyz') if o is not None else None),
            'rpy': floats(o.get('rpy') if o is not None else None),
        }
        if j.get('type') != 'fixed':
            entry['axis'] = floats(j.find('axis').get('xyz'))
            lim = j.find('limit')
            if lim is not None:
                entry['limit'] = [float(lim.get('lower', 0)), float(lim.get('upper', 0))]
        joints.append(entry)

    visuals, meshes = [], {}
    for link in urdf.findall('link'):
        v = link.find('visual')
        m = v.find('geometry/mesh') if v is not None else None
        if m is None:
            continue
        name = os.path.splitext(os.path.basename(m.get('filename')))[0]
        o = v.find('origin')
        visuals.append({'link': link.get('name'), 'mesh': f'{name}.ply',
                        'xyz': floats(o.get('xyz') if o is not None else None),
                        'rpy': floats(o.get('rpy') if o is not None else None)})
        if name not in meshes:
            meshes[name] = convert(os.path.join(desc, 'dae', f'{name}.dae'),
                                   os.path.join(out, f'{name}.ply'), BUDGET.get(name, 1500))
            print(f'{name}: {meshes[name]} triangles')

    with open(os.path.join(out, 'go2.json'), 'w') as f:
        json.dump({'source': 'go2_description (Unitree Robotics, BSD)', 'root': 'base_link',
                   'joints': joints, 'visuals': visuals}, f, indent=1)
        f.write('\n')
    print(f'{len(visuals)} visuals, {sum(meshes[v["mesh"][:-4]] for v in visuals)} triangles in the dog')


if __name__ == '__main__':
    main()
