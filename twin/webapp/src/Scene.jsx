import { memo, useEffect, useMemo, useRef, useState } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { Grid, Html, Line, OrbitControls } from '@react-three/drei';
import * as THREE from 'three';
import { PLYLoader } from 'three/examples/jsm/loaders/PLYLoader.js';
import { useTopic } from './RosProvider.jsx';
import { TfTree, toMatrix } from './ros/tf.js';
import { QUALITY } from './ros/quality.js';
import { fetchConfigJson } from './ros/config.js';
import Go2Model from './Go2Model.jsx';
import D1Model from './D1Model.jsx';

// D1 base on the Go2 trunk (base_link): centre of the trunk top, as in
// gazebo/sim/build_model.py --mount and the MoveIt model (d1_go2.urdf.xacro).
const D1_MOUNT = [0, 0, 0.057];
// ROS is z-up, three.js is y-up: everything ROS lives in this rotated group.
const ROS_TO_THREE = [-Math.PI / 2, 0, 0];
// Light blue/grey "architectural scan" surface.
const SURFACE = '#c3d1df';
const ROUTE = '#22d3ee';
const PIN = '#ff3b5c';
// 3D labels stay under the UI panels and dialogs (drei's default is ~16M).
const LABEL_Z = [500, 0];

/**
 * Full-screen 3D view of the twin.
 *  view 'solid'  /rtabmap/cloud_map through a voxel grid filter, drawn as lit,
 *                shadow-casting cubes (one InstancedMesh: one draw call)
 *  view 'mesh'   a Poisson mesh exported by wifi_3d_nav.sh --export --mesh
 *                (maps/<db>_mesh.ply), lit and shadowed; works without a robot
 *  view 'points' the raw cloud
 * Plus the robot (smoothed, red navigation arrow), the waypoint pin and the
 * glowing Nav2 route to it, YOLO boxes with floating labels, VLM
 * "HYDROPONIC" sightings, the dock and the named locations of
 * config/semantic_locations.json. Subscriptions are made here, outside
 * <Canvas>, so they do not depend on React context crossing into the
 * three.js renderer. Point clouds are decoded and voxelized in a Web Worker
 * (ros/cloudWorker.js); `quality` picks the render settings (ros/quality.js).
 * Memoized: the dashboard re-renders for every status change, the 3D scene
 * only when its own props change.
 */
export default memo(Scene);

function Scene({
  fixedFrame, baseFrame, layers, view, meshUrl, textured, shadows, voxelSize, quality = 'high',
  assets, clickToGo, follow, waypoint, onPick, onGoal, onStats,
  rooms, roomsInteractive, selectedRooms = [], onRoomClick,
}) {
  const tf = useMemo(() => new TfTree(), []);
  const [cloud, setCloud] = useState(null);
  const [scan, setScan] = useState(null);
  const [plan, setPlan] = useState(null);
  const [yolo, setYolo] = useState(new Map());
  const [sightings, setSightings] = useState([]);
  const [places, setPlaces] = useState([]);

  useTopic('/tf', 'tf2_msgs/msg/TFMessage', (m) => tf.add(m), { queueLength: 50 });
  useTopic('/tf_static', 'tf2_msgs/msg/TFMessage', (m) => tf.add(m), { queueLength: 50 });
  // Leg angles (go2_joint_state_bridge, from /lf/lowstate): a ref, not state,
  // so 20 Hz joint data moves the model without re-rendering React.
  const joints = useRef(new Map());
  useTopic('/joint_states', 'sensor_msgs/msg/JointState', (m) => {
    (m.name || []).forEach((n, i) => joints.current.set(n, m.position[i]));
  }, { throttle: 50 });
  // D1 arm angles (arm_state_bridge.py, from the arm's rt/arm_Feedback).
  const armJoints = useRef(new Map());
  useTopic('/d1_arm/joint_states', 'sensor_msgs/msg/JointState', (m) => {
    (m.name || []).forEach((n, i) => armJoints.current.set(n, m.position[i]));
  }, { throttle: 50 });
  const q = QUALITY[quality] || QUALITY.high;
  const voxelOpts = useMemo(() => (view === 'solid'
    ? { size: voxelSize === 'auto' ? 0.06 : Number(voxelSize), budget: q.voxelBudget } : null),
  [view, voxelSize, q.voxelBudget]);
  const [voxels, setVoxels] = useState(null);
  const decode = useCloudWorker({ setCloud, setScan, setVoxels }, voxelOpts);
  useTopic('/rtabmap/cloud_map', 'sensor_msgs/msg/PointCloud2', (m) => decode('map', m, 600000),
    { throttle: 2000, compression: 'cbor', enabled: layers.map && view !== 'mesh' });
  useTopic('/lidar3d/registered', 'sensor_msgs/msg/PointCloud2', (m) => decode('scan', m, 60000),
    { throttle: 500, compression: 'cbor', enabled: layers.scan });
  useTopic('/plan', 'nav_msgs/msg/Path', setPlan, { throttle: 300 });
  // The goal Nav2 actually got (click-to-go snaps clicks to free floor).
  useTopic('/goal_pose', 'geometry_msgs/msg/PoseStamped', (m) => onGoal?.(m));
  useTopic('/yolo/markers', 'visualization_msgs/msg/MarkerArray', (m) => {
    setYolo((prev) => applyMarkers(prev, m));
  }, { throttle: 250, enabled: layers.yolo });
  // The VLM says what it sees, not where: a HYDROPONIC answer is pinned
  // 1.5 m ahead of the robot (the camera's view), merged within 1.5 m.
  useTopic('/vlm_decision', 'std_msgs/msg/String', (m) => {
    if (m.data !== 'HYDROPONIC') return;
    const pose = tf.lookup(fixedFrame, baseFrame);
    if (!pose) return;
    const p = new THREE.Vector3(1.5, 0, 0).applyMatrix4(pose);
    setSightings((prev) => (prev.some((s) => Math.hypot(s.x - p.x, s.y - p.y) < 1.5)
      ? prev : [...prev, { x: p.x, y: p.y, label: 'Hydroponic station' }]));
  }, { enabled: layers.labels });

  // A new waypoint drops the old route until Nav2 plans the new one.
  const waypointId = waypoint?.id;
  useEffect(() => { if (waypointId) setPlan(null); }, [waypointId]);

  // Named locations saved with --save-location (served as config/).
  useEffect(() => {
    if (!layers.labels) return undefined;
    let alive = true;
    const load = () => fetchConfigJson(assets, 'semantic_locations.json')
      .then((d) => alive && setPlaces(Object.entries(d || {}).map(([label, e]) => ({ label, ...e }))))
      .catch(() => {});
    load();
    const id = setInterval(load, 10000);
    return () => { alive = false; clearInterval(id); };
  }, [assets, layers.labels]);

  const [meshInfo, setMeshInfo] = useState(null);
  const stats = useRef(onStats);
  stats.current = onStats;
  useEffect(() => {
    stats.current?.({
      mapPoints: cloud?.count ?? 0,
      voxels: voxels?.count ?? 0,
      triangles: view === 'solid' ? (voxels?.count ?? 0) * 12 : view === 'mesh' ? (meshInfo?.triangles ?? 0) : 0,
    });
  }, [cloud, voxels, meshInfo, view]);

  const floorZ = view === 'solid' && voxels ? voxels.floorZ : view === 'mesh' && meshInfo ? meshInfo.floorZ : null;
  // Rooms lie on top of the floor surface: above the floor voxels' tops in Solid view.
  const roomZ = view === 'solid' && voxels ? voxels.floorZ + voxels.size + 0.02 : (floorZ ?? 0) + 0.03;

  return (
    <Canvas shadows={shadows} camera={{ position: [-7, 10, 9], fov: 45, near: 0.05, far: 1000 }}
            dpr={q.dpr} gl={{ antialias: q.antialias }} style={{ cursor: clickToGo ? 'crosshair' : 'grab' }}>
      <color attach="background" args={['#05070b']} />
      <fog attach="fog" args={['#05070b', 35, 110]} />
      <hemisphereLight args={['#e8f0f8', '#1e2733', 0.75]} />
      <ambientLight intensity={0.2} />
      <directionalLight position={[12, 22, 8]} intensity={1.6} castShadow={shadows}
                        shadow-mapSize={[q.shadowMapSize, q.shadowMapSize]} shadow-bias={-0.0005} shadow-normalBias={0.02}
                        shadow-camera-left={-30} shadow-camera-right={30}
                        shadow-camera-top={30} shadow-camera-bottom={-30}
                        shadow-camera-near={1} shadow-camera-far={80} />
      <directionalLight position={[-10, 8, -12]} intensity={0.35} />
      {shadows && <SlowShadows />}
      {/* Under the scanned floor (ROS z is three.js y), never drawn through it. */}
      <Grid infiniteGrid cellSize={1} sectionSize={5} cellColor="#121a24" sectionColor="#1d2b3b"
            fadeDistance={70} position={[0, (floorZ ?? 0) - 0.02, 0]} />
      <OrbitControls makeDefault enableDamping maxPolarAngle={Math.PI * 0.49} />
      <FpsCounter onFps={(fps) => stats.current?.({ fps })} />
      <group rotation={ROS_TO_THREE}>
        {layers.map && view === 'solid' && voxels && voxels.count > 0 && (
          <InFrame tf={tf} fixed={fixedFrame} frame={voxels.frame}>
            <Voxels voxels={voxels} textured={textured} lambert={q.material === 'lambert'} />
            {shadows && <ShadowFloor z={floorZ} />}
          </InFrame>
        )}
        {layers.map && view === 'points' && cloud && <Points tf={tf} fixed={fixedFrame} cloud={cloud} size={0.05} />}
        {layers.map && view === 'mesh' && meshUrl && (
          <InFrame tf={tf} fixed={fixedFrame} frame="rtabmap_map" fallbackIdentity>
            <ScanMesh url={meshUrl} textured={textured} onInfo={setMeshInfo} />
            {shadows && meshInfo && <ShadowFloor z={meshInfo.floorZ} />}
          </InFrame>
        )}
        {layers.scan && scan && <Points tf={tf} fixed={fixedFrame} cloud={scan} size={0.03} tint="#4ade80" />}
        {layers.rooms && rooms?.rooms?.length > 0 && (
          <InFrame tf={tf} fixed={fixedFrame} frame={rooms.frame} fallbackIdentity>
            {rooms.rooms.map((r) => (
              <RoomShape key={r.id} room={r} z={roomZ} interactive={roomsInteractive}
                         selected={selectedRooms.includes(r.id)} onClick={onRoomClick} />
            ))}
          </InFrame>
        )}
        {plan && waypoint && <Route tf={tf} fixed={fixedFrame} path={plan} />}
        {waypoint && <WaypointPin key={waypoint.id} x={waypoint.x} y={waypoint.y} />}
        <Dock />
        {layers.yolo && [...yolo.values()].map((m) => (
          <YoloBox key={`${m.ns}/${m.id}`} tf={tf} fixed={fixedFrame} marker={m} />
        ))}
        {layers.labels && sightings.map((s, i) => (
          <Pin key={`vlm-${i}`} x={s.x} y={s.y} z={floorZ ?? 0} label={s.label} kind="vlm" />
        ))}
        {layers.labels && places.filter((p) => (!p.frame || p.frame === fixedFrame) && !/^dock$/i.test(p.label)).map((p) => (
          <Pin key={`place-${p.label}`} x={p.x} y={p.y} z={floorZ ?? p.z ?? 0} label={p.label.replace(/_/g, ' ')} kind="place" />
        ))}
        <Robot tf={tf} fixed={fixedFrame} base={baseFrame} follow={follow} joints={joints} lambert={q.material === 'lambert'}
               armJoints={layers.arm !== false ? armJoints : null} />
        <ClickPlane enabled={clickToGo} onPick={onPick} />
      </group>
    </Canvas>
  );
}

/**
 * A child group placed at fixed <- frame every frame; hidden until TF
 * resolves (or shown untransformed with fallbackIdentity, e.g. an exported
 * mesh viewed without a running session).
 */
function InFrame({ tf, fixed, frame, fallbackIdentity = false, children }) {
  const ref = useRef();
  useFrame(() => {
    const g = ref.current;
    if (!g) return;
    const m = tf.lookup(fixed, frame || fixed);
    if (m) g.matrix.copy(m);
    else if (fallbackIdentity) g.matrix.identity();
    g.visible = !!m || fallbackIdentity;
  });
  return <group ref={ref} matrixAutoUpdate={false} visible={false}>{children}</group>;
}

/** Voxel cubes: one InstancedMesh, light blue/grey or camera colours. */
function Voxels({ voxels, textured, lambert }) {
  const ref = useRef();
  const geometry = useMemo(() => new THREE.BoxGeometry(1, 1, 1), []);
  // Lambert (lite): no PBR terms per pixel, much cheaper on a CPU renderer.
  const material = useMemo(() => (lambert
    ? new THREE.MeshLambertMaterial({ color: SURFACE })
    : new THREE.MeshStandardMaterial({ color: SURFACE, roughness: 0.85, metalness: 0.05 })), [lambert]);
  useEffect(() => () => { geometry.dispose(); material.dispose(); }, [geometry, material]);

  useEffect(() => {
    const mesh = ref.current;
    if (!mesh) return;
    const m = new THREE.Matrix4();
    const s = voxels.size * 0.96;   // small gaps read as surface texture
    const color = new THREE.Color();
    const useColor = textured && voxels.hasColor;
    for (let i = 0; i < voxels.count; i++) {
      m.makeScale(s, s, s).setPosition(voxels.centers[3 * i], voxels.centers[3 * i + 1], voxels.centers[3 * i + 2]);
      mesh.setMatrixAt(i, m);
      if (useColor) mesh.setColorAt(i, color.setRGB(voxels.colors[3 * i], voxels.colors[3 * i + 1], voxels.colors[3 * i + 2]));
      else mesh.setColorAt(i, color.set(SURFACE));
    }
    mesh.count = voxels.count;
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    mesh.computeBoundingSphere();
  }, [voxels, textured, material]);

  return <instancedMesh ref={ref} args={[geometry, material, voxels.count]} castShadow receiveShadow />;
}

/** Exported Poisson mesh (binary PLY with normals and optional colours). */
function ScanMesh({ url, textured, onInfo }) {
  const [geometry, setGeometry] = useState(null);
  const [error, setError] = useState(null);
  useEffect(() => {
    let alive = true;
    setError(null);
    new PLYLoader().load(url, (g) => {
      if (!alive) { g.dispose(); return; }
      if (!g.attributes.normal) g.computeVertexNormals();
      g.computeBoundingBox();
      const tris = g.index ? g.index.count / 3 : g.attributes.position.count / 3;
      onInfo?.({ triangles: tris, floorZ: g.boundingBox.min.z, hasColor: !!g.attributes.color });
      setGeometry(g);
    }, undefined, (e) => alive && setError(String(e?.message || e)));
    return () => { alive = false; };
  }, [url, onInfo]);
  useEffect(() => () => geometry?.dispose(), [geometry]);
  if (error) return <Html zIndexRange={LABEL_Z} center className="scene-msg">The 3D scan could not be loaded</Html>;
  if (!geometry) return <Html zIndexRange={LABEL_Z} center className="scene-msg">Loading 3D scan…</Html>;
  const colored = textured && !!geometry.attributes.color;
  return (
    <mesh geometry={geometry} castShadow receiveShadow>
      {/* keyed: vertexColors is compiled into the shader, a prop change alone is ignored */}
      <meshStandardMaterial key={colored ? 'rgb' : 'plain'} color={colored ? '#ffffff' : SURFACE} vertexColors={colored}
                            roughness={0.85} metalness={0.05} side={THREE.DoubleSide} />
    </mesh>
  );
}

/** Invisible floor that only shows the shadows cast on it. */
function ShadowFloor({ z }) {
  if (z === null || z === undefined) return null;
  return (
    <mesh position={[0, 0, z + 0.002]} receiveShadow>
      <planeGeometry args={[200, 200]} />
      <shadowMaterial opacity={0.35} />
    </mesh>
  );
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

/** Nav2 /plan as a thick neon line: bright core + wide translucent glow. */
function Route({ tf, fixed, path }) {
  const points = useMemo(() => (path.poses || []).map((p) => [
    p.pose.position.x, p.pose.position.y, 0.08]), [path]);
  if (points.length < 2) return null;
  return (
    <InFrame tf={tf} fixed={fixed} frame={path.header?.frame_id}>
      <Line points={points} color={ROUTE} lineWidth={16} transparent opacity={0.18} depthWrite={false} />
      <Line points={points} color={ROUTE} lineWidth={7} transparent opacity={0.35} depthWrite={false} />
      <Line points={points} color="#cffafe" lineWidth={2.5} />
    </InFrame>
  );
}

/** Glowing waypoint: floating sphere on a light beam, pulsing ground ring. */
function WaypointPin({ x, y }) {
  const ring = useRef();
  const orb = useRef();
  const pos = useRef(new THREE.Vector3(x, y, 0));
  const group = useRef();
  useFrame(({ clock }, dt) => {
    const t = clock.elapsedTime;
    // Glide to a snapped goal instead of jumping.
    pos.current.lerp(new THREE.Vector3(x, y, 0), 1 - Math.exp(-dt * 8));
    if (group.current) group.current.position.copy(pos.current);
    if (ring.current) {
      const s = 1 + (t * 0.8 % 1) * 1.6;
      ring.current.scale.setScalar(s);
      ring.current.material.opacity = 0.6 * (1 - (t * 0.8 % 1));
    }
    if (orb.current) orb.current.position.z = 0.75 + Math.sin(t * 2.5) * 0.06;
  });
  // Position is driven by useFrame (initialised from the first x, y).
  return (
    <group ref={group} position={pos.current.toArray()}>
      <mesh ref={ring} position={[0, 0, 0.02]}>
        <ringGeometry args={[0.16, 0.22, 48]} />
        <meshBasicMaterial color={PIN} transparent depthWrite={false} side={THREE.DoubleSide} />
      </mesh>
      <mesh position={[0, 0, 0.02]}>
        <circleGeometry args={[0.12, 32]} />
        <meshBasicMaterial color={PIN} transparent opacity={0.55} side={THREE.DoubleSide} />
      </mesh>
      <mesh position={[0, 0, 0.37]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[0.012, 0.012, 0.7, 8]} />
        <meshBasicMaterial color={PIN} transparent opacity={0.7} />
      </mesh>
      <group ref={orb} position={[0, 0, 0.75]}>
        <mesh>
          <sphereGeometry args={[0.13, 32, 16]} />
          <meshBasicMaterial color={PIN} />
        </mesh>
        <mesh>
          <sphereGeometry args={[0.26, 32, 16]} />
          <meshBasicMaterial color={PIN} transparent opacity={0.18} depthWrite={false}
                             blending={THREE.AdditiveBlending} />
        </mesh>
      </group>
    </group>
  );
}

/**
 * A room on the floor: translucent fill, outline, name. Clicks are taken
 * only when interactive (room editing); otherwise they pass through to the
 * tap-to-go floor below.
 */
function RoomShape({ room, z, interactive, selected, onClick }) {
  const [hover, setHover] = useState(false);
  const geometry = useMemo(() => new THREE.ShapeGeometry(
    new THREE.Shape(room.polygon.map(([x, y]) => new THREE.Vector2(x, y)))), [room.polygon]);
  useEffect(() => () => geometry.dispose(), [geometry]);
  const outline = useMemo(() => [...room.polygon, room.polygon[0]].map(([x, y]) => [x, y, 0.005]), [room.polygon]);
  const color = new THREE.Color(...room.color);
  const opacity = selected ? 0.5 : hover && interactive ? 0.36 : 0.2;
  return (
    <group position={[0, 0, z]}>
      <mesh geometry={geometry}
            onClick={(e) => {
              if (!interactive || e.delta > 4) return;   // let tap-to-go have it
              e.stopPropagation();
              onClick?.(room);
            }}
            onPointerOver={() => interactive && setHover(true)}
            onPointerOut={() => setHover(false)}>
        <meshBasicMaterial color={color} transparent opacity={opacity} depthWrite={false} side={THREE.DoubleSide} />
      </mesh>
      <Line points={outline} color={color} lineWidth={selected ? 4 : 2} transparent opacity={0.9} />
      <group position={[room.center[0], room.center[1], 0.25]}>
        <Html zIndexRange={LABEL_Z} center className={`tag tag-room ${selected ? 'selected' : ''}`} style={{ '--room': `#${color.getHexString()}` }}>
          {room.label || `Room ${room.id.split('_').pop()}`}
        </Html>
      </group>
    </group>
  );
}

/** The dock: the session's start pose, lidar3d_map origin. */
function Dock() {
  return (
    <group>
      <mesh position={[0, 0, 0.01]}>
        <ringGeometry args={[0.32, 0.38, 64]} />
        <meshBasicMaterial color="#a78bfa" transparent opacity={0.8} side={THREE.DoubleSide} />
      </mesh>
      <group position={[0, 0, 0.5]}>
        <Html zIndexRange={LABEL_Z} center className="tag tag-dock">Dock</Html>
      </group>
    </group>
  );
}

/**
 * The official Go2 model with live leg joints (Go2Model), a red navigation
 * arrow and a footprint disc; a Go2-sized box stand-in while the model loads
 * or if its files are missing. It casts no shadow (shadows are redrawn once
 * a second); the blue disc below it marks its footprint instead.
 * Smoothed: TF updates arrive at ~10-20 Hz, so the pose glides toward the
 * newest one (exponential lerp / slerp) instead of jumping; jumps > 2 m
 * (relocalization) snap.
 */
function Robot({ tf, fixed, base, follow, joints, lambert, armJoints }) {
  const ref = useRef();
  const s = useMemo(() => ({
    pos: new THREE.Vector3(), quat: new THREE.Quaternion(), tpos: new THREE.Vector3(),
    tquat: new THREE.Quaternion(), scale: new THREE.Vector3(), init: false, world: new THREE.Vector3(),
  }), []);
  useFrame((state, dt) => {
    const g = ref.current;
    if (!g) return;
    const m = tf.lookup(fixed, base);
    g.visible = !!m;
    if (!m) return;
    m.decompose(s.tpos, s.tquat, s.scale);
    if (!s.init || s.pos.distanceTo(s.tpos) > 2) {
      s.pos.copy(s.tpos);
      s.quat.copy(s.tquat);
      s.init = true;
    } else {
      const a = 1 - Math.exp(-dt * 10);
      s.pos.lerp(s.tpos, a);
      s.quat.slerp(s.tquat, a);
    }
    g.matrix.compose(s.pos, s.quat, s.scale.set(1, 1, 1));
    if (follow && state.controls) {
      g.updateWorldMatrix(true, false);
      s.world.setFromMatrixPosition(g.matrixWorld);
      const c = state.controls;
      const delta = s.world.clone().sub(c.target).multiplyScalar(1 - Math.exp(-dt * 4));
      c.target.add(delta);
      state.camera.position.add(delta);
    }
  });
  return (
    <group ref={ref} matrixAutoUpdate={false} visible={false}>
      <Go2Model joints={joints} lambert={lambert} fallback={<StandIn />} />
      {armJoints && (
        <group position={D1_MOUNT}>
          <D1Model joints={armJoints} lambert={lambert} />
        </group>
      )}
      {/* Red navigation arrow above the body, pointing along +x (heading);
          above the D1 arm (~0.5 m tall) when it is shown. */}
      <group position={[0, 0, armJoints ? 0.75 : 0.4]}>
        <mesh position={[-0.05, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
          <cylinderGeometry args={[0.05, 0.05, 0.5, 16]} />
          <meshStandardMaterial color="#ef4444" emissive="#b91c1c" emissiveIntensity={0.7} />
        </mesh>
        <mesh position={[0.32, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
          <coneGeometry args={[0.14, 0.28, 24]} />
          <meshStandardMaterial color="#ef4444" emissive="#b91c1c" emissiveIntensity={0.7} />
        </mesh>
      </group>
      <mesh position={[0, 0, -0.32]}>
        <circleGeometry args={[0.55, 48]} />
        <meshBasicMaterial color="#38bdf8" transparent opacity={0.15} depthWrite={false} side={THREE.DoubleSide} />
      </mesh>
    </group>
  );
}

/** Go2-sized box (0.70 x 0.31 x 0.40 m), used when the model cannot load. */
function StandIn() {
  const legs = [[0.25, 0.12], [0.25, -0.12], [-0.25, 0.12], [-0.25, -0.12]];
  return (
    <group>
      <mesh>
        <boxGeometry args={[0.7, 0.31, 0.18]} />
        <meshStandardMaterial color="#e5e7eb" metalness={0.2} roughness={0.5} />
      </mesh>
      <mesh position={[0.38, 0, 0.03]}>
        <boxGeometry args={[0.1, 0.22, 0.12]} />
        <meshStandardMaterial color="#111827" />
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
  if (marker.type === 9) {   // TEXT_VIEW_FACING: floats above its box (yolo_node.py)
    return (
      <InFrame tf={tf} fixed={fixed} frame={marker.header?.frame_id}>
        <group matrix={matrix} matrixAutoUpdate={false}>
          <Html zIndexRange={LABEL_Z} center className="tag tag-yolo">{prettyYolo(marker.text)}</Html>
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
                              opacity={Math.max(0.25, c.a)} depthWrite={false} />
      </mesh>
    </InFrame>
  );
}

/** "chair 2.1 m (lidar)" -> "Chair · 2.1 m": no internals in the UI. */
function prettyYolo(text = '') {
  const m = text.match(/^(.*?)\s+([\d.]+ m)/);
  const name = (m ? m[1] : text).replace(/\s*\(.*\)\s*$/, '');
  return m ? `${name.charAt(0).toUpperCase()}${name.slice(1)} · ${m[2]}` : name;
}

/** Floating label on a thin stem (named locations, VLM sightings). */
function Pin({ x, y, z, label, kind }) {
  const color = kind === 'vlm' ? '#22c55e' : '#38bdf8';
  return (
    <group position={[x, y, z]}>
      <mesh position={[0, 0, 0.75]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[0.015, 0.015, 1.5, 8]} />
        <meshBasicMaterial color={color} />
      </mesh>
      <mesh position={[0, 0, 0.01]}>
        <circleGeometry args={[0.18, 32]} />
        <meshBasicMaterial color={color} transparent opacity={0.6} side={THREE.DoubleSide} />
      </mesh>
      <group position={[0, 0, 1.6]}>
        <Html zIndexRange={LABEL_Z} center className={`tag tag-${kind}`}>{label}</Html>
      </group>
    </group>
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
            onPick?.({ x: p.x, y: p.y });
          }}>
      <planeGeometry args={[2000, 2000]} />
      <meshBasicMaterial transparent opacity={0} depthWrite={false} side={THREE.DoubleSide} />
    </mesh>
  );
}

/**
 * Shadows redrawn once a second instead of every frame. The shadow pass
 * draws the whole map a second time; the map changes every 2 s at most.
 */
function SlowShadows({ period = 1 }) {
  const gl = useThree((s) => s.gl);
  const last = useRef(-Infinity);
  useEffect(() => {
    gl.shadowMap.autoUpdate = false;
    gl.shadowMap.needsUpdate = true;
    return () => { gl.shadowMap.autoUpdate = true; };
  }, [gl]);
  useFrame(({ clock }) => {
    if (clock.elapsedTime - last.current < period) return;
    last.current = clock.elapsedTime;
    gl.shadowMap.needsUpdate = true;
  });
  return null;
}

/**
 * Point cloud decoding in ros/cloudWorker.js. Returns decode(kind, msg, maxPoints).
 * A map message that arrives while the worker is busy waits; a newer one
 * replaces it, so the worker never falls behind. Changing the voxel options
 * re-filters the last map without a new message.
 */
function useCloudWorker({ setCloud, setScan, setVoxels }, voxelOpts) {
  const w = useRef(null);
  const busy = useRef({ map: false, scan: false });
  const pending = useRef({ map: null, scan: null });
  const opts = useRef(voxelOpts);
  opts.current = voxelOpts;

  const send = (kind, msg, maxPoints) => {
    if (!w.current) return;
    if (busy.current[kind]) { pending.current[kind] = [msg, maxPoints]; return; }
    busy.current[kind] = true;
    w.current.postMessage({ op: 'cloud', kind, msg, maxPoints, voxel: kind === 'map' ? opts.current : null });
  };

  useEffect(() => {
    const worker = new Worker(new URL('./ros/cloudWorker.js', import.meta.url), { type: 'module' });
    worker.onmessage = ({ data }) => {
      const { kind } = data;
      if ('cloud' in data) {
        busy.current[kind] = false;
        if (data.cloud) (kind === 'map' ? setCloud : setScan)(data.cloud);
      }
      if (kind === 'map' && 'voxels' in data) setVoxels(data.voxels);
      const next = pending.current[kind];
      if (next && !busy.current[kind]) { pending.current[kind] = null; send(kind, ...next); }
    };
    w.current = worker;
    return () => { worker.terminate(); w.current = null; };
  }, []);   // eslint-disable-line react-hooks/exhaustive-deps

  // New voxel size or view: re-filter the map the worker already has.
  useEffect(() => {
    if (!voxelOpts) setVoxels(null);
    w.current?.postMessage({ op: 'voxel', voxel: voxelOpts });
  }, [voxelOpts, setVoxels]);

  return send;
}

/** Frames per second, reported once a second. */
function FpsCounter({ onFps }) {
  const acc = useRef({ frames: 0, t: 0 });
  const cb = useRef(onFps);
  cb.current = onFps;
  useFrame((_, dt) => {
    const a = acc.current;
    a.frames += 1;
    a.t += dt;
    if (a.t >= 1) {
      cb.current?.(Math.round(a.frames / a.t));
      a.frames = 0;
      a.t = 0;
    }
  });
  return null;
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
