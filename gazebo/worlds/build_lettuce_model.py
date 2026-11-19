#!/usr/bin/env python3
"""
build_lettuce_model.py: generate a butterhead lettuce mesh for Gazebo (no download).

Writes gazebo/worlds/models/lettuce/:
  meshes/lettuce.obj, meshes/lettuce.mtl   the head, z up, origin at the base of the stem
  meshes/lettuce_leaf.png                  leaf texture (4 shade columns, outer dark -> inner pale)
  model.config, model.sdf                  so it also works as a standalone model

The head is ~0.22 m across and ~0.14 m tall (a market butterhead from an NFT channel).
Leaves are placed on a golden-angle spiral, outer to inner: the outer ones long, open and
drooping, the inner ones short, upright and curled over the heart. Each leaf is a curved
strip (spine bent in its own vertical plane), cupped across its width, with ruffled
edges, drawn on both sides (Gazebo culls back faces: shared vertices, a normal set per side). Deterministic (fixed seed): rerun
to regenerate exactly the same files.

No lettuce exists on Gazebo Fuel (searched 2026-11-18); this keeps the asset in the
repo and free of licences.

Usage: python3 build_lettuce_model.py [--leaves 22] [--grid 12 8]
"""
import argparse
import math
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "models", "lettuce")
GOLDEN = math.radians(137.508)
SHADES = 4                  # texture columns: 0 = outer leaves (dark) ... 3 = heart (pale)


def leaf_texture(path, size=512):
    """Leaf texture atlas: SHADES columns, each a leaf (U across, V base -> tip)."""
    rng = np.random.default_rng(7)
    col_w = size // SHADES
    img = np.zeros((size, size, 3), np.float32)
    vv = np.linspace(0.0, 1.0, size)[:, None]                    # 0 = base (row 0 at the top = V 1 in OBJ)
    vv = 1.0 - vv
    for s in range(SHADES):
        uu = np.linspace(-1.0, 1.0, col_w)[None, :]
        pale = s / (SHADES - 1)
        # bright yellow-green, as the lab's hydroponic lettuce (photos IMG_3562-3564)
        tip = np.array([0.34, 0.62, 0.16]) * (1 - pale) + np.array([0.66, 0.86, 0.32]) * pale
        base = np.array([0.78, 0.90, 0.52]) * (1 - pale) + np.array([0.90, 0.95, 0.64]) * pale
        t = np.clip(vv ** 0.6, 0, 1)[..., None]
        c = base * (1 - t) + tip * t
        edge = np.clip(np.abs(uu) ** 3, 0, 1)[..., None]           # darker towards the edges
        c = c * (1 - 0.25 * edge * t)
        # midrib (pale, wide at the base) and veins running out from it
        rib_w = 0.05 + 0.10 * (1 - vv)
        rib = np.exp(-(uu / rib_w) ** 2)[..., None]
        c = c * (1 - 0.6 * rib) + np.array([0.90, 0.95, 0.75]) * 0.6 * rib
        vein = 0.5 + 0.5 * np.cos(2 * math.pi * (vv * 9 - np.abs(uu) * 1.3))
        vein = (vein ** 40)[..., None] * (1 - edge) * 0.25
        c = c * (1 - vein) + np.array([0.85, 0.92, 0.65]) * vein
        img[:, s * col_w:(s + 1) * col_w] = c
    img += rng.normal(0, 0.025, img.shape)                         # leaf cell texture
    im = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(0.6))
    # a few darker blotches, as on a real leaf
    d = ImageDraw.Draw(im, "RGBA")
    for _ in range(140):
        x, y, r = rng.integers(0, size), rng.integers(0, size), rng.integers(2, 7)
        d.ellipse((x - r, y - r, x + r, y + r), fill=(30, 70, 20, 25))
    im.save(path)


def leaf(i, n, nu, nv, rng):
    """Vertices (nu*nv, 3), uv (nu*nv, 2) and per-vertex surface normals of leaf i of n."""
    t = i / max(1, n - 1)                       # 0 outer ... 1 heart
    az = i * GOLDEN + rng.normal(0, 0.08)
    length = 0.135 * (1 - 0.55 * t) * rng.uniform(0.92, 1.08)
    width = 0.095 * (1 - 0.45 * t) * rng.uniform(0.9, 1.1)
    elev0 = math.radians(18 + 72 * t ** 0.8)    # angle of the leaf base above the horizontal
    bend = math.radians(55 - 150 * t)           # + droops outward, - curls over the heart
    cup = 0.55 + 0.6 * t                        # how much the sides lift
    ruffle = 0.012 * (1 - 0.5 * t)
    r0 = 0.004 + 0.010 * (1 - t)
    e_r = np.array([math.cos(az), math.sin(az), 0.0])
    e_t = np.array([-math.sin(az), math.cos(az), 0.0])
    e_z = np.array([0.0, 0.0, 1.0])

    us = np.linspace(0.0, 1.0, nu)
    vs = np.linspace(-1.0, 1.0, nv)
    # spine: integrate the bending direction in the (e_r, e_z) plane
    spine = [r0 * e_r + np.array([0, 0, 0.012])]
    dirs = []
    for k, u in enumerate(us):
        th = elev0 - bend * u ** 1.4
        d = math.cos(th) * e_r + math.sin(th) * e_z
        dirs.append((d, -math.sin(th) * e_r + math.cos(th) * e_z))
        if k:
            spine.append(spine[-1] + d * length * (us[k] - us[k - 1]))
    phase = rng.uniform(0, 2 * math.pi)
    P = np.zeros((nu, nv, 3))
    for a, u in enumerate(us):
        w = width * (u ** 0.55) * (1 - u ** 2.5) ** 0.5 + 0.004   # obovate: widest past the middle
        d, nrm = dirs[a]
        for b, v in enumerate(vs):
            lat = v * w
            lift = cup * lat ** 2 / max(width, 1e-6)
            wave = ruffle * math.sin(9 * u * math.pi + 5 * v + phase) * abs(v) ** 2.5 * u
            P[a, b] = spine[a] + lat * e_t + (lift + wave) * nrm
    # vertex normals from the grid
    du = np.gradient(P, axis=0)
    dv = np.gradient(P, axis=1)
    N = np.cross(dv, du)
    N /= np.linalg.norm(N, axis=2, keepdims=True) + 1e-12
    shade = min(SHADES - 1, int(t * SHADES))
    U = (shade + (vs + 1) / 2 * 0.96 + 0.02) / SHADES
    tex_v, tex_u = np.meshgrid(us, U, indexing="ij")     # (nu, nv): V base -> tip, U across the column
    UV = np.stack([tex_u, tex_v], axis=-1)
    return P.reshape(-1, 3), UV.reshape(-1, 2), N.reshape(-1, 3)


def build(leaves, nu, nv):
    os.makedirs(os.path.join(OUT, "meshes"), exist_ok=True)
    rng = np.random.default_rng(42)
    leaf_texture(os.path.join(OUT, "meshes", "lettuce_leaf.png"))
    V, T, N, F = [], [], [], []           # F: (vertex index, normal index) per corner
    for i in range(leaves):
        p, uv, nrm = leaf(i, leaves, nu, nv, rng)
        base, nbase = len(V), len(N)
        V.extend(p)
        T.extend(uv)
        N.extend(nrm)                        # front normals, then the back's (flipped)
        N.extend(-nrm)
        for a in range(nu - 1):
            for b in range(nv - 1):
                q = [a * nv + b, (a + 1) * nv + b, (a + 1) * nv + b + 1, a * nv + b + 1]
                for tri in ((q[0], q[1], q[2]), (q[0], q[2], q[3])):
                    F.append([(base + k, nbase + k) for k in tri])                          # front
                    F.append([(base + k, nbase + len(p) + k) for k in tri[::-1]])           # back: reversed winding
    V = np.array(V)
    with open(os.path.join(OUT, "meshes", "lettuce.mtl"), "w", newline="\n") as f:
        f.write("# generated by build_lettuce_model.py\nnewmtl leaf\nKa 0.6 0.6 0.6\nKd 1.0 1.0 1.0\nKs 0.05 0.05 0.05\n"
                "Ns 8\nd 1.0\nillum 2\nmap_Kd lettuce_leaf.png\n")
    with open(os.path.join(OUT, "meshes", "lettuce.obj"), "w", newline="\n") as f:
        f.write("# butterhead lettuce, generated by gazebo/worlds/build_lettuce_model.py (z up, metres)\n")
        f.write("mtllib lettuce.mtl\no lettuce\n")
        f.writelines(f"v {x:.4f} {y:.4f} {z:.4f}\n" for x, y, z in V)
        f.writelines(f"vt {u:.3f} {v:.3f}\n" for u, v in T)
        # both sides of a leaf share positions and texture coordinates, with their own normals
        # (Gazebo culls back faces, so each side is drawn only from its side): < 500 kB
        f.writelines(f"vn {x:.3f} {y:.3f} {z:.3f}\n" for x, y, z in N)
        f.write("usemtl leaf\ns 1\n")
        f.writelines("f " + " ".join(f"{v + 1}/{v + 1}/{n + 1}" for v, n in face) + "\n" for face in F)
    with open(os.path.join(OUT, "model.config"), "w", newline="\n") as f:
        f.write('<?xml version="1.0"?>\n<model>\n  <name>lettuce</name>\n  <version>1.0</version>\n'
                '  <sdf version="1.9">model.sdf</sdf>\n  <description>Butterhead lettuce, generated by '
                'gazebo/worlds/build_lettuce_model.py</description>\n</model>\n')
    with open(os.path.join(OUT, "model.sdf"), "w", newline="\n") as f:
        f.write('<?xml version="1.0"?>\n<sdf version="1.9">\n  <model name="lettuce"><static>true</static>\n'
                '    <link name="head">\n'
                '      <visual name="v"><geometry><mesh><uri>meshes/lettuce.obj</uri></mesh></geometry></visual>\n'
                '      <collision name="c"><pose>0 0 0.07 0 0 0</pose><geometry><cylinder><radius>0.09</radius>'
                '<length>0.12</length></cylinder></geometry></collision>\n'
                '    </link>\n  </model>\n</sdf>\n')
    lo, hi = V.min(axis=0), V.max(axis=0)
    print(f"lettuce: {leaves} leaves, {len(V)} vertices, {len(F)} triangles; "
          f"size {hi[0] - lo[0]:.3f} x {hi[1] - lo[1]:.3f} x {hi[2] - lo[2]:.3f} m (z {lo[2]:.3f}..{hi[2]:.3f})")
    return lo, hi


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--leaves", type=int, default=22)
    ap.add_argument("--grid", type=int, nargs=2, default=(12, 8), metavar=("ALONG", "ACROSS"))
    a = ap.parse_args()
    build(a.leaves, *a.grid)


if __name__ == "__main__":
    main()
