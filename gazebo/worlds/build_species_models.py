#!/usr/bin/env python3
"""
build_species_models.py: geometric stand-in plant models for the species ladder (no downloads).

Writes gazebo/models/<species>_plant/ for basil, spinach, tomato and strawberry:
  meshes/<species>.obj + .mtl   leaves (textured, both sides), stems, fruit; z up, origin at
                                the base of the plant (the net cup's rim)
  meshes/<species>_leaf.png     the leaf texture (shade gradient, midrib, veins)
  model.config, model.sdf       model://<species>_plant, with the gz Label system (label below)
  README.md                     what the model is and is not
These are GEOMETRIC STAND-INS: plausible size, leaf shape, arrangement and colour for a young
hydroponic plant of each species, built from parametric leaf surfaces, tubes and spheres.
They are not botanical reconstructions. The lettuce of the control is the separate generated
butterhead (build_lettuce_model.py, model://lettuce, label 1).

  species     label  height  canopy r  form
  basil         2    0.30 m  0.12 m    3 stems, opposite leaf pairs turning 90 deg per node
  spinach       3    0.20 m  0.15 m    rosette of spoon-shaped leaves on petioles
  tomato        4    0.60 m  0.20 m    main stem, alternate compound leaves, 2 trusses of fruit
  strawberry    5    0.18 m  0.16 m    crown of trifoliate leaves, red fruit hanging low

Deterministic (fixed seeds). Prints each model's measured height and canopy radius, which
config/species_config.yaml records.

Usage: python3 build_species_models.py [--species basil spinach ...]
"""
import argparse
import math
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(os.path.dirname(HERE), "models")
LABELS = {"lettuce": 1, "basil": 2, "spinach": 3, "tomato": 4, "strawberry": 5}


# ------------------------------------------------------------------ geometry
class Mesh:
    """Vertices, texture coords, normals and faces per material, written as one OBJ."""

    def __init__(self):
        self.v, self.t, self.n = [], [], []
        self.faces = {}                      # material -> [((v, t, n), ...), ...]

    def add_surface(self, P, UV, N, mat, double=True):
        """A grid surface (nu, nv, 3) drawn on both sides (shared vertices, two normal sets)."""
        nu, nv = P.shape[:2]
        bv, bt, bn = len(self.v), len(self.t), len(self.n)
        self.v.extend(P.reshape(-1, 3))
        self.t.extend(UV.reshape(-1, 2))
        self.n.extend(N.reshape(-1, 3))
        if double:
            self.n.extend(-N.reshape(-1, 3))
        f = self.faces.setdefault(mat, [])
        for a in range(nu - 1):
            for b in range(nv - 1):
                q = [a * nv + b, (a + 1) * nv + b, (a + 1) * nv + b + 1, a * nv + b + 1]
                for tri in ((q[0], q[1], q[2]), (q[0], q[2], q[3])):
                    f.append([(bv + k, bt + k, bn + k) for k in tri])
                    if double:
                        f.append([(bv + k, bt + k, bn + nu * nv + k) for k in tri[::-1]])

    def add_tube(self, p0, p1, r0, r1, mat, sides=6):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        ax = p1 - p0
        L = np.linalg.norm(ax)
        if L < 1e-6:
            return
        ax /= L
        ref = np.array([0, 0, 1.0]) if abs(ax[2]) < 0.9 else np.array([1.0, 0, 0])
        e1 = np.cross(ax, ref)
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(ax, e1)
        ang = np.linspace(0, 2 * math.pi, sides + 1)
        ring = np.stack([np.cos(ang), np.sin(ang)], 1)
        P = np.stack([[p0 + r0 * (c * e1 + s * e2) for c, s in ring], [p1 + r1 * (c * e1 + s * e2) for c, s in ring]])
        N = np.stack([[c * e1 + s * e2 for c, s in ring]] * 2)
        UV = np.zeros((2, sides + 1, 2))
        self.add_surface(P, UV, N, mat, double=False)

    def add_sphere(self, c, r, mat, squash=(1, 1, 1), nu=7, nv=9):
        th = np.linspace(0, math.pi, nu)
        ph = np.linspace(0, 2 * math.pi, nv)
        T, PH = np.meshgrid(th, ph, indexing="ij")
        N = np.stack([np.sin(T) * np.cos(PH), np.sin(T) * np.sin(PH), np.cos(T)], -1)
        P = np.asarray(c) + r * N * np.asarray(squash)
        self.add_surface(P, np.zeros(P.shape[:2] + (2,)), N, mat, double=False)

    def bounds(self):
        v = np.array(self.v)
        return v.min(0), v.max(0)

    def write(self, folder, name, materials):
        os.makedirs(os.path.join(folder, "meshes"), exist_ok=True)
        with open(os.path.join(folder, "meshes", f"{name}.mtl"), "w", newline="\n") as f:
            for m, spec in materials.items():
                kd = spec.get("kd", (1, 1, 1))
                f.write(f"newmtl {m}\nKa {kd[0]} {kd[1]} {kd[2]}\nKd {kd[0]} {kd[1]} {kd[2]}\n"
                        f"Ks {spec.get('ks', 0.08)} {spec.get('ks', 0.08)} {spec.get('ks', 0.08)}\nNs {spec.get('ns', 12)}\n")
                if spec.get("map"):
                    f.write(f"map_Kd {spec['map']}\n")
        with open(os.path.join(folder, "meshes", f"{name}.obj"), "w", newline="\n") as f:
            f.write(f"# {name}: geometric stand-in, generated by gazebo/worlds/build_species_models.py (z up, metres)\n")
            f.write(f"mtllib {name}.mtl\no {name}\n")
            f.writelines(f"v {x:.4f} {y:.4f} {z:.4f}\n" for x, y, z in self.v)
            f.writelines(f"vt {u:.3f} {v:.3f}\n" for u, v in self.t)
            f.writelines(f"vn {x:.3f} {y:.3f} {z:.3f}\n" for x, y, z in self.n)
            for m, faces in self.faces.items():
                f.write(f"usemtl {m}\n")
                f.writelines("f " + " ".join(f"{a + 1}/{b + 1}/{c + 1}" for a, b, c in face) + "\n" for face in faces)


def leaf(m, base, direction, up, length, width, rng, mat="leaf", bend=0.3, cup=0.4, ruffle=0.0,
         shape="ovate", nu=9, nv=5, petiole=0.0, serrate=0.0, shade=0):
    """One leaf blade along direction (unit, in 3D) from base, its face towards up, curving
    down by bend (rad over the length), cupped across, optionally on a petiole."""
    d = np.asarray(direction, float)
    d /= np.linalg.norm(d)
    up = np.asarray(up, float)
    side = np.cross(d, up)
    if np.linalg.norm(side) < 1e-6:
        side = np.cross(d, [1.0, 0, 0])
    side /= np.linalg.norm(side)
    nrm = np.cross(side, d)
    base = np.asarray(base, float)
    if petiole > 0:
        m.add_tube(base, base + d * petiole, 0.0018, 0.0014, "stem", sides=5)
        base = base + d * petiole
    us = np.linspace(0, 1, nu)
    vs = np.linspace(-1, 1, nv)
    P = np.zeros((nu, nv, 3))
    pos, ang = base.copy(), 0.0
    phase = rng.uniform(0, 6.28)
    for a, u in enumerate(us):
        if a:
            ang = bend * u
            step = (math.cos(ang) * d - math.sin(ang) * nrm) * length / (nu - 1)
            pos = pos + step
        if shape == "spoon":
            w = width * (u ** 1.2) * (1 - u ** 3) ** 0.6
        elif shape == "lanceolate":
            w = width * math.sin(math.pi * min(u * 1.1, 1.0)) * (1 - 0.3 * u)
        else:                                    # ovate
            w = width * (u ** 0.6) * (1 - u ** 2) ** 0.55
        w += 0.0015
        for b, v in enumerate(vs):
            lat = v * w * (1 + serrate * math.sin(30 * u + phase) * abs(v))
            lift = cup * (v * w) ** 2 / max(width, 1e-6) + ruffle * math.sin(12 * u + 4 * v + phase) * abs(v) ** 2
            P[a, b] = pos + lat * side + lift * nrm
    du, dv = np.gradient(P, axis=0), np.gradient(P, axis=1)
    N = np.cross(dv, du)
    N /= np.linalg.norm(N, axis=2, keepdims=True) + 1e-12
    tex_v, tex_u = np.meshgrid(us, (shade + (vs + 1) / 2 * 0.96 + 0.02) / 2, indexing="ij")
    m.add_surface(P, np.stack([tex_u, tex_v], -1), N, mat)


def leaf_texture(path, dark, light, size=256, vein=0.25):
    """Two columns (0: darker, 1: lighter leaves), V base -> tip, midrib and veins."""
    rng = np.random.default_rng(11)
    img = np.zeros((size, size, 3), np.float32)
    vv = 1.0 - np.linspace(0, 1, size)[:, None]
    for s in range(2):
        uu = np.linspace(-1, 1, size // 2)[None, :]
        tip = np.array(dark) * (1 - 0.4 * s) + np.array(light) * 0.4 * s
        base = np.array(light)
        t = np.clip(vv ** 0.7, 0, 1)[..., None]
        c = base * (1 - t) + tip * t
        rib = np.exp(-(uu / 0.06) ** 2)[..., None]
        c = c * (1 - 0.45 * rib) + np.array(light) * 1.15 * 0.45 * rib
        vn = (0.5 + 0.5 * np.cos(2 * math.pi * (vv * 7 - np.abs(uu) * 1.2))) ** 30
        c = c * (1 - vein * vn[..., None]) + np.array(light) * vein * vn[..., None]
        img[:, s * size // 2:(s + 1) * size // 2] = c
    img += rng.normal(0, 0.02, img.shape)
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(0.6)).save(path)


# ------------------------------------------------------------------ species
def basil(m, rng):
    for s in range(3):
        az0 = s * 2.1 + rng.uniform(-0.2, 0.2)
        top = np.array([0.03 * math.cos(az0), 0.03 * math.sin(az0), 0.27 + rng.uniform(-0.03, 0.02)])
        m.add_tube([0, 0, 0], top, 0.004, 0.0025, "stem")
        for k in range(5):                       # nodes, opposite pairs, decussate
            f = 0.25 + 0.17 * k
            node = top * f
            size = 1.0 - 0.13 * k
            for side in (0, 1):
                az = az0 + k * math.pi / 2 + side * math.pi
                d = np.array([math.cos(az), math.sin(az), 0.35 + 0.1 * k])
                leaf(m, node, d, [0, 0, 1], 0.065 * size, 0.035 * size, rng, bend=0.6, cup=0.9,
                     shape="ovate", petiole=0.012, shade=1)
        for j in range(4):                       # the growing tip
            az = az0 + j * math.pi / 2
            leaf(m, top, [math.cos(az), math.sin(az), 1.2], [0, 0, 1], 0.028, 0.016, rng, bend=0.4, cup=1.0, shade=1)


def spinach(m, rng):
    n = 14
    for i in range(n):
        t = i / (n - 1)
        az = i * math.radians(137.5)
        elev = math.radians(30 + 45 * t)
        d = np.array([math.cos(az) * math.cos(elev), math.sin(az) * math.cos(elev), math.sin(elev)])
        L = 0.11 * (1 - 0.35 * t) * rng.uniform(0.9, 1.1)
        leaf(m, [0, 0, 0.01], d, [0, 0, 1], L, 0.055 * (1 - 0.3 * t), rng, bend=0.7 - 0.4 * t, cup=0.5,
             ruffle=0.004, shape="spoon", petiole=0.06 * (1 - 0.3 * t), shade=0 if t < 0.6 else 1)


def tomato(m, rng):
    pts = [np.array([0, 0, 0.0])]
    for k in range(1, 7):                        # zigzag main stem
        pts.append(np.array([0.012 * (-1) ** k, 0.008 * (-1) ** (k + 1), 0.1 * k]))
    for a, b in zip(pts, pts[1:]):
        m.add_tube(a, b, 0.006, 0.005, "stem")
    for k in range(1, 7):                        # alternate compound leaves
        az = k * math.radians(150)
        node = pts[k] - np.array([0, 0, 0.03])
        d = np.array([math.cos(az), math.sin(az), 0.25])
        d /= np.linalg.norm(d)
        rach = node + d * 0.18 * (1.1 - 0.08 * k)
        m.add_tube(node, rach, 0.0025, 0.0015, "stem")
        side = np.cross(d, [0, 0, 1.0])
        for j, f in enumerate((0.35, 0.65)):     # leaflet pairs + the terminal one
            p = node + (rach - node) * f
            for sgn in (-1, 1):
                leaf(m, p, sgn * side + 0.2 * d + [0, 0, 0.1], [0, 0, 1], 0.05, 0.025, rng, bend=0.5,
                     cup=0.4, serrate=0.15, shape="ovate", shade=0)
        leaf(m, rach, d + [0, 0, -0.1], [0, 0, 1], 0.055, 0.028, rng, bend=0.5, cup=0.4, serrate=0.15, shade=0)
    for k, colors in ((2, ("red", "red", "orange", "green_fruit")), (4, ("orange", "green_fruit", "green_fruit", "green_fruit"))):
        az = k * math.radians(150) + math.pi
        base = pts[k] + np.array([0.03 * math.cos(az), 0.03 * math.sin(az), -0.02])
        for j, col in enumerate(colors):         # a truss of cherry tomatoes
            c = base + np.array([0.025 * j * math.cos(az), 0.025 * j * math.sin(az), -0.03 - 0.008 * j])
            m.add_tube(base, c, 0.0015, 0.0012, "stem", sides=4)
            m.add_sphere(c, 0.016, col)


def strawberry(m, rng):
    for i in range(9):                           # trifoliate leaves on petioles from the crown
        az = i * math.radians(137.5)
        elev = math.radians(rng.uniform(45, 70))
        d = np.array([math.cos(az) * math.cos(elev), math.sin(az) * math.cos(elev), math.sin(elev)])
        tip = np.array([0, 0, 0.015]) + d * rng.uniform(0.09, 0.13)
        m.add_tube([0, 0, 0.015], tip, 0.0022, 0.0016, "stem", sides=5)
        out = np.array([d[0], d[1], 0.15])
        side = np.cross(out, [0, 0, 1.0])
        side /= np.linalg.norm(side) + 1e-9
        for rot in (-0.7, 0.0, 0.7):             # three leaflets
            ld = out * math.cos(rot) + side * math.sin(rot)
            leaf(m, tip, ld, [0, 0, 1], 0.05, 0.03, rng, bend=0.25, cup=0.5, serrate=0.25, shape="ovate", shade=i % 2)
    for j in range(5):                           # fruit hanging low, under the leaves
        az = j * 1.3 + 0.4
        r = rng.uniform(0.07, 0.11)
        c = np.array([r * math.cos(az), r * math.sin(az), rng.uniform(-0.01, 0.03)])
        m.add_tube([0, 0, 0.02], c + [0, 0, 0.012], 0.0012, 0.001, "stem", sides=4)
        m.add_sphere(c, 0.013, "red" if j % 4 else "white_fruit", squash=(1, 1, 1.35))
        m.add_sphere(c + [0, 0, 0.016], 0.007, "sepal", squash=(1.2, 1.2, 0.3))


SPECIES = {
    "basil": dict(fn=basil, seed=21, dark=(0.18, 0.50, 0.12), light=(0.55, 0.80, 0.30),
                  stem=(0.30, 0.48, 0.18), desc="Genovese basil: 3 stems, opposite leaf pairs turning 90 deg per node"),
    "spinach": dict(fn=spinach, seed=22, dark=(0.08, 0.30, 0.08), light=(0.35, 0.58, 0.22),
                    stem=(0.45, 0.62, 0.30), desc="spinach: a rosette of spoon-shaped leaves on petioles"),
    "tomato": dict(fn=tomato, seed=23, dark=(0.14, 0.36, 0.10), light=(0.42, 0.62, 0.26),
                   stem=(0.32, 0.45, 0.18), desc="young cherry tomato: zigzag stem, compound leaves, two trusses of fruit"),
    "strawberry": dict(fn=strawberry, seed=24, dark=(0.12, 0.38, 0.10), light=(0.45, 0.68, 0.28),
                       stem=(0.42, 0.50, 0.22), desc="day-neutral strawberry: trifoliate leaves on a crown, fruit hanging low"),
}
FRUIT = {"red": (0.80, 0.08, 0.06), "orange": (0.90, 0.45, 0.08), "green_fruit": (0.45, 0.62, 0.20),
         "white_fruit": (0.92, 0.88, 0.70), "sepal": (0.20, 0.45, 0.12)}


def build(name):
    sp = SPECIES[name]
    rng = np.random.default_rng(sp["seed"])
    m = Mesh()
    sp["fn"](m, rng)
    folder = os.path.join(MODELS, f"{name}_plant")
    os.makedirs(os.path.join(folder, "meshes"), exist_ok=True)
    leaf_texture(os.path.join(folder, "meshes", f"{name}_leaf.png"), sp["dark"], sp["light"])
    mats = {"leaf": dict(map=f"{name}_leaf.png", ks=0.12, ns=20), "stem": dict(kd=sp["stem"])}
    mats.update({k: dict(kd=v, ks=0.4, ns=60) for k, v in FRUIT.items() if k in m.faces})
    m.write(folder, name, mats)
    lo, hi = m.bounds()
    v = np.array(m.v)
    height, radius = float(hi[2]), float(np.max(np.hypot(v[:, 0], v[:, 1])))
    label = LABELS[name]
    with open(os.path.join(folder, "model.config"), "w", newline="\n") as f:
        f.write(f'<?xml version="1.0"?>\n<model>\n  <name>{name}_plant</name>\n  <version>1.0</version>\n'
                f'  <sdf version="1.9">model.sdf</sdf>\n  <description>Geometric stand-in for {sp["desc"]} '
                f'(gazebo/worlds/build_species_models.py)</description>\n</model>\n')
    with open(os.path.join(folder, "model.sdf"), "w", newline="\n") as f:
        f.write(f'<?xml version="1.0"?>\n<sdf version="1.9">\n  <model name="{name}_plant"><static>true</static>\n'
                f'    <link name="plant">\n'
                f'      <visual name="v"><geometry><mesh><uri>meshes/{name}.obj</uri></mesh></geometry></visual>\n'
                f'      <collision name="c"><pose>0 0 {height / 2:.3f} 0 0 0</pose><geometry><cylinder>'
                f'<radius>{radius * 0.8:.3f}</radius><length>{height:.3f}</length></cylinder></geometry></collision>\n'
                f'    </link>\n'
                f'    <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>{label}</label></plugin>\n'
                f'  </model>\n</sdf>\n')
    with open(os.path.join(folder, "README.md"), "w", newline="\n") as f:
        f.write(f"# {name}_plant: geometric stand-in\n\n"
                f"A geometric stand-in for {sp['desc']}, generated by `gazebo/worlds/build_species_models.py` "
                "(rerun it to regenerate exactly). It is **not a botanical reconstruction**: leaf shape, "
                "arrangement, size and colour are plausible for a young hydroponic plant, built from parametric "
                "leaf surfaces, tubes and spheres, without measured leaf geometry, texture photographs or growth "
                "stages.\n\n"
                f"- `model://{name}_plant`, origin at the base of the plant (the net cup's rim), z up\n"
                f"- height {height:.3f} m, canopy radius {radius:.3f} m (measured from the mesh)\n"
                f"- gz Label system label {label} (species dataset: scripts/tools/lettuce_dataset.py --species)\n")
    print(f"{name}: height {height:.3f} m, canopy radius {radius:.3f} m, {len(m.v)} vertices, "
          f"{sum(len(f) for f in m.faces.values())} triangles")
    return height, radius


def preview_world():
    """gazebo/worlds/species_preview.sdf: the five species in a row (render_world.sh species_preview)."""
    items = [("lettuce", "model://lettuce")] + [(n, f"model://{n}_plant") for n in SPECIES]
    inc = "".join(f'    <include><uri>{u}</uri><name>{n}</name><pose>3.0 {-0.8 + 0.4 * i:.2f} 0.30 0 0 0</pose></include>\n'
                  for i, (n, u) in enumerate(items))
    pots = "".join(f'    <model name="pot_{n}"><static>true</static><pose>3.0 {-0.8 + 0.4 * i:.2f} 0.15 0 0 0</pose><link name="l">'
                   f'<visual name="v"><geometry><cylinder><radius>0.07</radius><length>0.30</length></cylinder></geometry>'
                   f'<material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material></visual></link></model>\n'
                   for i, (n, _) in enumerate(items))
    sdf = f"""<?xml version="1.0" ?>
<!-- species_preview.sdf: GENERATED by build_species_models.py. Lettuce (control) and the stand-ins. -->
<sdf version="1.9">
  <world name="species_preview">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>@RENDER_ENGINE@</render_engine></plugin>
    <scene><ambient>0.7 0.7 0.7 1</ambient><background>0.92 0.93 0.94 1</background><shadows>true</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.95 0.95 0.92 1</diffuse><direction>-0.4 0.2 -1.0</direction></light>
    <model name="ground"><static>true</static><link name="l"><visual name="v"><geometry><plane><normal>0 0 1</normal>
      <size>20 20</size></plane></geometry><material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
      </visual></link></model>
{pots}{inc}  </world>
</sdf>
"""
    open(os.path.join(HERE, "species_preview.sdf"), "w", newline="\n").write(sdf)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--species", nargs="*", default=list(SPECIES))
    for name in ap.parse_args().species:
        build(name)
    preview_world()


if __name__ == "__main__":
    main()
