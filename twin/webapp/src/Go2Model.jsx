import { useEffect, useMemo, useRef, useState } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { PLYLoader } from 'three/examples/jsm/loaders/PLYLoader.js';

// Unitree's official Go2 (go2_description, BSD), decimated to ~23k triangles
// by scripts/tools/build_go2_model.py: public/go2/go2.json + *.ply.
const BASE = './go2/';
// Unitree's standing pose (hip, thigh, calf), used until /joint_states arrives.
const STAND = { hip: 0, thigh: 0.67, calf: -1.3 };

let modelPromise = null;
/** go2.json and every mesh it names, loaded once for the whole page. */
function loadModel() {
  if (!modelPromise) {
    modelPromise = fetch(`${BASE}go2.json`)
      .then((r) => { if (!r.ok) throw new Error(`go2.json: HTTP ${r.status}`); return r.json(); })
      .then(async (desc) => {
        const loader = new PLYLoader();
        const files = [...new Set(desc.visuals.map((v) => v.mesh))];
        const geoms = await Promise.all(files.map((f) => loader.loadAsync(`${BASE}${f}`)));
        return { desc, geometry: Object.fromEntries(files.map((f, i) => [f, geoms[i]])) };
      });
    modelPromise.catch(() => { modelPromise = null; });   // retry on the next mount
  }
  return modelPromise;
}

/** URDF origin (xyz, rpy = fixed-axis roll, pitch, yaw) -> position + quaternion. */
function setOrigin(obj, xyz, rpy) {
  obj.position.set(...xyz);
  obj.quaternion.setFromEuler(new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX'));
}

/**
 * The Go2 with its 12 leg joints driven by `joints` (a ref to a Map
 * name -> angle, filled from /joint_states). Angles are smoothed toward the
 * newest value every frame, so 20 Hz joint data still moves fluidly.
 * Renders `fallback` while loading or if the model files are missing.
 */
export default function Go2Model({ joints, lambert = false, fallback = null }) {
  const [model, setModel] = useState(null);
  const [failed, setFailed] = useState(false);
  useEffect(() => {
    let alive = true;
    loadModel().then((m) => alive && setModel(m), (e) => {
      console.warn('Go2 model not loaded:', e);
      if (alive) setFailed(true);
    });
    return () => { alive = false; };
  }, []);

  const material = useMemo(() => (lambert
    ? new THREE.MeshLambertMaterial({ vertexColors: true })
    : new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.55, metalness: 0.25 })), [lambert]);
  useEffect(() => () => material.dispose(), [material]);

  // Build the link tree once: base_link -> joint frame -> moving group -> child link.
  const built = useMemo(() => {
    if (!model) return null;
    const { desc, geometry } = model;
    const links = new Map([[desc.root, new THREE.Group()]]);
    const movers = [];
    const link = (name) => {
      if (!links.has(name)) links.set(name, new THREE.Group());
      return links.get(name);
    };
    for (const j of desc.joints) {
      const frame = new THREE.Group();
      setOrigin(frame, j.xyz, j.rpy);
      const child = link(j.child);
      if (j.type === 'revolute' || j.type === 'continuous') {
        const mover = new THREE.Group();
        frame.add(mover);
        mover.add(child);
        const part = Object.keys(STAND).find((k) => j.name.endsWith(`_${k}_joint`));
        const angle = part ? STAND[part] : 0;
        movers.push({ name: j.name, mover, axis: new THREE.Vector3(...j.axis).normalize(), angle, limit: j.limit });
      } else {
        frame.add(child);
      }
      link(j.parent).add(frame);
    }
    for (const v of desc.visuals) {
      const mesh = new THREE.Mesh(geometry[v.mesh], material);
      setOrigin(mesh, v.xyz, v.rpy);
      link(v.link).add(mesh);
    }
    return { root: links.get(desc.root), movers };
  }, [model, material]);

  useFrame((_, dt) => {
    if (!built) return;
    const a = 1 - Math.exp(-dt * 18);
    const live = joints?.current;
    for (const m of built.movers) {
      let target = live?.get(m.name);
      if (target === undefined) target = m.angle;
      if (m.limit) target = Math.min(m.limit[1], Math.max(m.limit[0], target));
      m.angle += (target - m.angle) * a;
      m.mover.quaternion.setFromAxisAngle(m.axis, m.angle);
    }
  });

  if (!built) return failed ? fallback : null;
  return <primitive object={built.root} />;
}
