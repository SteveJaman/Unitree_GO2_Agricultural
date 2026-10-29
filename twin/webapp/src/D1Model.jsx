import { useEffect, useMemo, useState } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';

// Unitree's D1 arm (d1_description, BSD), meshes decimated to ~9k triangles
// by scripts/tools/build_d1_model.py: public/d1_description/.
const URDF = './d1_description/urdf/d1_description.urdf';
const PACKAGES = { d1_description: './d1_description' };
// The wrist camera's field of view, drawn from the gripper along Link6 +z.
const CAMERA = { offset: 0.09, range: 0.45, fovDeg: 60 };
// Without live angles (no arm_state_bridge, arm off): the arm's real resting
// pose, FOLDED_ANGLES of scripts/nodes/d1_arm_calibration.py (deg, joints 0-5),
// drawn translucent so it never passes for a live reading.
const FOLDED_DEG = [1.8, -88.8, 91.8, 1.3, 23.0, 0.6];
const FOLDED = Object.fromEntries(FOLDED_DEG.map((d, i) => [`Joint${i + 1}`, THREE.MathUtils.degToRad(d)]));
const LIVE_MS = 2000;

let robotPromise = null;
/** The URDF and its meshes, loaded once for the whole page. */
function loadRobot() {
  if (!robotPromise) {
    robotPromise = new Promise((resolve, reject) => {
      const manager = new THREE.LoadingManager();
      const loader = new URDFLoader(manager);
      loader.packages = PACKAGES;
      loader.loadMeshCb = (path, mgr, done) => new STLLoader(mgr).load(
        path, (geom) => done(new THREE.Mesh(geom)), undefined, (err) => done(null, err));
      let robot = null;
      manager.onLoad = () => (robot ? resolve(robot) : reject(new Error('D1 URDF did not load')));
      manager.onError = (url) => reject(new Error(`D1 model: cannot load ${url}`));
      loader.load(URDF, (r) => { robot = r; });
    });
    robotPromise.catch(() => { robotPromise = null; });   // retry on the next mount
  }
  return robotPromise;
}

/** Translucent cone from the camera along +z, apex at the lens. */
function cameraCone() {
  const { offset, range, fovDeg } = CAMERA;
  const radius = range * Math.tan(THREE.MathUtils.degToRad(fovDeg / 2));
  const geom = new THREE.ConeGeometry(radius, range, 40, 1, true);
  geom.translate(0, -range / 2, 0);   // apex at the origin, base towards -y
  geom.rotateX(-Math.PI / 2);         // ... then towards +z
  const cone = new THREE.Group();
  cone.add(new THREE.Mesh(geom, new THREE.MeshBasicMaterial({
    color: '#38bdf8', transparent: true, opacity: 0.16, depthWrite: false, side: THREE.DoubleSide,
  })));
  const rim = new THREE.LineLoop(
    new THREE.CircleGeometry(radius, 40).toNonIndexed(),
    new THREE.LineBasicMaterial({ color: '#7dd3fc', transparent: true, opacity: 0.7 }));
  rim.position.z = range;
  cone.add(rim);
  cone.position.z = offset;
  cone.name = 'd1_camera_cone';
  return cone;
}

/**
 * The D1 arm on the Go2's back, its joints driven by `joints` (a ref to a Map
 * name -> position, filled from /d1_arm/joint_states; `seen`: a ref to the
 * time of the last message). Positions glide toward the newest value every
 * frame, so 10 Hz telemetry still sweeps smoothly, like the dog's pose. With
 * no message for 2 s the arm settles into its folded resting pose, translucent.
 * Renders nothing while loading or if the model files are missing.
 */
export default function D1Model({ joints, seen, lambert = false, cone = true }) {
  const [robot, setRobot] = useState(null);
  useEffect(() => {
    let alive = true;
    loadRobot().then((r) => alive && setRobot(r), (e) => console.warn('D1 model not loaded:', e));
    return () => { alive = false; };
  }, []);

  const material = useMemo(() => (lambert
    ? new THREE.MeshLambertMaterial({ color: '#d7dbe2' })
    : new THREE.MeshStandardMaterial({ color: '#d7dbe2', roughness: 0.45, metalness: 0.3 })), [lambert]);
  useEffect(() => () => material.dispose(), [material]);

  const built = useMemo(() => {
    if (!robot) return null;
    robot.traverse((o) => { if (o.isMesh && o.name !== 'd1_camera_cone') o.material = material; });
    const link6 = robot.links.Link6;
    let coneObj = link6?.getObjectByName('d1_camera_cone');
    if (link6 && !coneObj) {
      coneObj = cameraCone();
      link6.add(coneObj);
    }
    const movers = Object.values(robot.joints)
      .filter((j) => j.jointType !== 'fixed')
      .map((j) => ({ joint: j, value: 0, lo: Number(j.limit.lower), hi: Number(j.limit.upper) }));
    return { robot, movers, coneObj };
  }, [robot, material]);

  useFrame((_, dt) => {
    if (!built) return;
    const a = 1 - Math.exp(-dt * 12);
    const isLive = !!seen && Date.now() - seen.current < LIVE_MS;
    if (material.transparent === isLive) {   // live: solid; resting guess: translucent
      material.transparent = !isLive;
      material.opacity = isLive ? 1 : 0.4;
      material.depthWrite = isLive;
      material.needsUpdate = true;
    }
    if (built.coneObj) built.coneObj.visible = cone && isLive;
    const live = isLive ? joints?.current : null;
    for (const m of built.movers) {
      let target = live?.get(m.joint.name);
      if (target === undefined) target = FOLDED[m.joint.name] ?? 0;
      if (Number.isFinite(m.lo) && Number.isFinite(m.hi) && m.hi > m.lo) {
        target = Math.min(m.hi, Math.max(m.lo, target));
      }
      m.value += (target - m.value) * a;
      m.joint.setJointValue(m.value);
    }
  });

  if (!built) return null;
  return <primitive object={built.robot} />;
}
