import { useEffect, useMemo, useRef, useState } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { Grid, Html, OrbitControls } from '@react-three/drei';
import * as THREE from 'three';
import { useTopic } from './RosProvider.jsx';
import { TfTree, toMatrix } from './ros/tf.js';
import { decodeCloud } from './ros/pointcloud.js';

// ROS is z-up, three.js is y-up: everything ROS lives in this rotated group.
const ROS_TO_THREE = [-Math.PI / 2, 0, 0];

/**
 * 3D view: dense map (/rtabmap/cloud_map), optional live scan, Nav2 /plan,
 * the robot from TF (fixedFrame -> base_link), the active /goal_pose and
 * YOLO boxes. With clickToGo, a click on the floor calls onPick({x, y}) in
 * fixedFrame. Subscriptions are made here, outside <Canvas>, so they do not
 * depend on React context crossing into the three.js renderer.
 */
export default function Scene({ fixedFrame, baseFrame, layers, clickToGo, follow, onPick, onStats }) {
  const tf = useMemo(() => new TfTree(), []);
  const [cloud, setCloud] = useState(null);
  const [scan, setScan] = useState(null);
  const [plan, setPlan] = useState(null);
  const [goal, setGoal] = useState(null);
  const [yolo, setYolo] = useState(new Map());
  const [pick, setPick] = useState(null);

  useTopic('/tf', 'tf2_msgs/msg/TFMessage', (m) => tf.add(m), { queueLength: 50 });
  useTopic('/tf_static', 'tf2_msgs/msg/TFMessage', (m) => tf.add(m), { queueLength: 50 });
  useTopic('/rtabmap/cloud_map', 'sensor_msgs/msg/PointCloud2', (m) => {
    const c = decodeCloud(m, 600000);
    if (c) setCloud(c);
  }, { throttle: 2000, compression: 'cbor', enabled: layers.map });
  useTopic('/lidar3d/registered', 'sensor_msgs/msg/PointCloud2', (m) => {
    const c = decodeCloud(m, 60000);
    if (c) setScan(c);
  }, { throttle: 500, compression: 'cbor', enabled: layers.scan });
  useTopic('/plan', 'nav_msgs/msg/Path', setPlan, { throttle: 500, enabled: layers.plan });
  useTopic('/goal_pose', 'geometry_msgs/msg/PoseStamped', (m) => { setGoal(m); setPick(null); });
  useTopic('/yolo/markers', 'visualization_msgs/msg/MarkerArray', (m) => {
    setYolo((prev) => applyMarkers(prev, m));
  }, { throttle: 250, enabled: layers.yolo });

  const stats = useRef(onStats);
  stats.current = onStats;
  useEffect(() => {
    stats.current?.({ mapPoints: cloud?.count ?? 0, mapTotal: cloud?.total ?? 0 });
  }, [cloud]);

  const handlePick = (p) => {
    setPick(p);
    onPick?.(p);
  };

  return (
    <Canvas camera={{ position: [-6, 9, 8], fov: 50, near: 0.05, far: 1000 }} dpr={[1, 2]}
            style={{ cursor: clickToGo ? 'crosshair' : 'grab' }}>
      <color attach="background" args={['#0b0f14']} />
      <ambientLight intensity={0.75} />
      <directionalLight position={[6, 12, 4]} intensity={0.9} />
      <Grid infiniteGrid cellSize={1} sectionSize={5} cellColor="#1f2a36" sectionColor="#2f4256"
            fadeDistance={80} position={[0, -0.001, 0]} />
      <OrbitControls makeDefault enableDamping />
      <group rotation={ROS_TO_THREE}>
        <axesHelper args={[1]} />
        {layers.map && cloud && <Points tf={tf} fixed={fixedFrame} cloud={cloud} size={0.05} />}
        {layers.scan && scan && <Points tf={tf} fixed={fixedFrame} cloud={scan} size={0.03} tint="#4ade80" />}
        {layers.plan && plan && <PathLine tf={tf} fixed={fixedFrame} path={plan} />}
        {goal && <Pose tf={tf} fixed={fixedFrame} msg={goal} color="#22c55e" />}
        {layers.yolo && [...yolo.values()].map((m) => (
          <YoloBox key={`${m.ns}/${m.id}`} tf={tf} fixed={fixedFrame} marker={m} />
        ))}
        <Robot tf={tf} fixed={fixedFrame} base={baseFrame} follow={follow} />
        {pick && <PickRing point={pick} />}
        <ClickPlane enabled={clickToGo} onPick={handlePick} />
      </group>
    </Canvas>
  );
}

/** A child group placed at fixed <- frame every frame; hidden until TF resolves. */
function InFrame({ tf, fixed, frame, children }) {
  const ref = useRef();
  useFrame(() => {
    const g = ref.current;
    if (!g) return;
    const m = tf.lookup(fixed, frame || fixed);
    g.visible = !!m;
    if (m) g.matrix.copy(m);
  });
  return <group ref={ref} matrixAutoUpdate={false} visible={false}>{children}</group>;
}

function Points({ tf, fixed, cloud, size, tint }) {
  const geometry = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(cloud.positions, 3));
    if (!tint) g.setAttribute('color', new THREE.BufferAttribute(cloud.colors, 3));
    g.computeBoundingSphere();
    return g;
  }, [cloud, tint]);
  useEffect(() => () => geometry.dispose(), [geometry]);
  return (
    <InFrame tf={tf} fixed={fixed} frame={cloud.frame}>
      <points geometry={geometry}>
        <pointsMaterial size={size} sizeAttenuation vertexColors={!tint} color={tint || '#ffffff'} />
      </points>
    </InFrame>
  );
}

function PathLine({ tf, fixed, path }) {
  const geometry = useMemo(() => {
    const pts = (path.poses || []).map((p) => new THREE.Vector3(
      p.pose.position.x, p.pose.position.y, p.pose.position.z + 0.05));
    return new THREE.BufferGeometry().setFromPoints(pts);
  }, [path]);
  useEffect(() => () => geometry.dispose(), [geometry]);
  return (
    <InFrame tf={tf} fixed={fixed} frame={path.header?.frame_id}>
      <line geometry={geometry}>
        <lineBasicMaterial color="#facc15" linewidth={2} />
      </line>
    </InFrame>
  );
}

function Pose({ tf, fixed, msg, color }) {
  const matrix = useMemo(() => toMatrix(msg.pose.position, msg.pose.orientation), [msg]);
  return (
    <InFrame tf={tf} fixed={fixed} frame={msg.header?.frame_id}>
      <group matrix={matrix} matrixAutoUpdate={false}>
        <mesh position={[0.25, 0, 0.05]} rotation={[0, 0, -Math.PI / 2]}>
          <coneGeometry args={[0.12, 0.3, 16]} />
          <meshStandardMaterial color={color} />
        </mesh>
        <mesh position={[0, 0, 0.02]} rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.2, 0.2, 0.02, 32]} />
          <meshStandardMaterial color={color} transparent opacity={0.5} />
        </mesh>
      </group>
    </InFrame>
  );
}

/** Go2-sized stand-in (0.70 x 0.31 x 0.40 m) at fixed <- base_link. */
function Robot({ tf, fixed, base, follow }) {
  const ref = useRef();
  const world = useMemo(() => new THREE.Vector3(), []);
  useFrame((state) => {
    const g = ref.current;
    if (!g) return;
    const m = tf.lookup(fixed, base);
    g.visible = !!m;
    if (!m) return;
    g.matrix.copy(m);
    if (follow && state.controls) {
      g.updateWorldMatrix(true, false);
      world.setFromMatrixPosition(g.matrixWorld);
      const c = state.controls;
      const delta = world.clone().sub(c.target);
      c.target.add(delta);
      state.camera.position.add(delta);
    }
  });
  const legs = [[0.25, 0.12], [0.25, -0.12], [-0.25, 0.12], [-0.25, -0.12]];
  return (
    <group ref={ref} matrixAutoUpdate={false} visible={false}>
      <mesh>
        <boxGeometry args={[0.7, 0.31, 0.18]} />
        <meshStandardMaterial color="#e5e7eb" metalness={0.2} roughness={0.5} />
      </mesh>
      <mesh position={[0.38, 0, 0.03]}>
        <boxGeometry args={[0.1, 0.22, 0.12]} />
        <meshStandardMaterial color="#111827" />
      </mesh>
      <mesh position={[0.6, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
        <coneGeometry args={[0.06, 0.18, 12]} />
        <meshStandardMaterial color="#38bdf8" emissive="#0ea5e9" emissiveIntensity={0.6} />
      </mesh>
      {legs.map(([x, y]) => (
        <mesh key={`${x},${y}`} position={[x, y, -0.2]} rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.03, 0.03, 0.26, 8]} />
          <meshStandardMaterial color="#6b7280" />
        </mesh>
      ))}
    </group>
  );
}

function YoloBox({ tf, fixed, marker }) {
  const matrix = useMemo(() => toMatrix(marker.pose.position, marker.pose.orientation), [marker]);
  if (marker.type === 9) {   // TEXT_VIEW_FACING
    return (
      <InFrame tf={tf} fixed={fixed} frame={marker.header?.frame_id}>
        <group matrix={matrix} matrixAutoUpdate={false}>
          <Html center className="yolo-label">{marker.text}</Html>
        </group>
      </InFrame>
    );
  }
  if (marker.type !== 1) return null;   // only CUBE otherwise
  const c = marker.color;
  return (
    <InFrame tf={tf} fixed={fixed} frame={marker.header?.frame_id}>
      <mesh matrix={matrix} matrixAutoUpdate={false}>
        <boxGeometry args={[marker.scale.x, marker.scale.y, marker.scale.z]} />
        <meshStandardMaterial color={new THREE.Color(c.r, c.g, c.b)} transparent
                              opacity={Math.max(0.25, c.a)} />
      </mesh>
    </InFrame>
  );
}

function PickRing({ point }) {
  const ref = useRef();
  useFrame(({ clock }) => {
    if (ref.current) ref.current.scale.setScalar(1 + 0.25 * Math.sin(clock.elapsedTime * 6));
  });
  return (
    <mesh ref={ref} position={[point.x, point.y, 0.03]}>
      <ringGeometry args={[0.15, 0.22, 32]} />
      <meshBasicMaterial color="#f97316" side={THREE.DoubleSide} />
    </mesh>
  );
}

/** Invisible z = 0 plane of the fixed frame that turns clicks into floor points. */
function ClickPlane({ enabled, onPick }) {
  const ref = useRef();
  return (
    <mesh ref={ref} visible
          onClick={(e) => {
            if (!enabled || e.delta > 4) return;       // ignore orbit drags
            e.stopPropagation();
            const p = ref.current.parent.worldToLocal(e.point.clone());
            onPick({ x: p.x, y: p.y });
          }}>
      <planeGeometry args={[2000, 2000]} />
      <meshBasicMaterial transparent opacity={0} depthWrite={false} side={THREE.DoubleSide} />
    </mesh>
  );
}

/** visualization_msgs/MarkerArray into a ns/id map (ADD, DELETE, DELETEALL). */
function applyMarkers(prev, msg) {
  const next = new Map(prev);
  for (const m of msg.markers || []) {
    const key = `${m.ns}/${m.id}`;
    if (m.action === 3) next.clear();             // DELETEALL
    else if (m.action === 2) next.delete(key);    // DELETE
    else next.set(key, m);                        // ADD / MODIFY
  }
  return next;
}
