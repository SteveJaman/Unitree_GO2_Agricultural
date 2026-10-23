import { useCallback, useEffect, useState } from 'react';
import { RosProvider, usePublisher, useRos } from './RosProvider.jsx';
import Scene from './Scene.jsx';
import StatusBar from './StatusBar.jsx';
import CameraFeed from './CameraFeed.jsx';
import ControlPanel from './ControlPanel.jsx';
import MapMenu from './MapMenu.jsx';
import VlmBadge from './VlmBadge.jsx';

// Nav2 plans in lidar3d_map (Point-LIO); RTAB-Map's rtabmap_map is its parent.
const params = new URLSearchParams(window.location.search);
const FIXED_FRAME = params.get('frame') || 'lidar3d_map';
const BASE_FRAME = params.get('base') || 'base_link';
// Where maps/ (databases, *_mesh.ply) and config/ are served: the dashboard's
// own server by default; ?assets=http://<VM IP>:3000/ for the dev server.
const ASSETS = params.get('assets') || './';
// The dock is the session's start pose (wifi_3d_nav.sh starts at the dock).
const DOCK = { x: 0, y: 0 };

export default function App() {
  return (
    <RosProvider>
      <Dashboard />
    </RosProvider>
  );
}

/** *_mesh.ply files in maps/ (python http.server directory listing), newest first. */
function useMeshes() {
  const [meshes, setMeshes] = useState([]);
  useEffect(() => {
    let alive = true;
    const load = () => fetch(`${ASSETS}maps/`, { cache: 'no-store' })
      .then((r) => (r.ok ? r.text() : ''))
      .then((html) => {
        const names = [...html.matchAll(/href="([^"?#]+_mesh\.ply)"/g)].map((m) => decodeURIComponent(m[1]));
        if (alive) setMeshes([...new Set(names)].sort().reverse());
      })
      .catch(() => {});
    load();
    const id = setInterval(load, 15000);
    return () => { alive = false; clearInterval(id); };
  }, []);
  return meshes;
}

function Dashboard() {
  const { status } = useRos();
  const connected = status === 'connected';
  const [layers, setLayers] = useState({ map: true, scan: false, yolo: true, labels: true });
  const [view, setView] = useState(params.get('view') || 'solid');
  const [textured, setTextured] = useState(true);
  const [shadows, setShadows] = useState(true);
  const [voxelSize, setVoxelSize] = useState('auto');
  const [clickToGo, setClickToGo] = useState(true);
  const [follow, setFollow] = useState(false);
  const [stats, setStats] = useState({});
  const [toast, setToast] = useState(null);
  const [waypoint, setWaypoint] = useState(null);   // { id, x, y }
  const meshes = useMeshes();
  const [meshFile, setMeshFile] = useState('');
  const sendClick = usePublisher('/clicked_point', 'geometry_msgs/msg/PointStamped');
  const sendGoal = usePublisher('/goal_pose', 'geometry_msgs/msg/PoseStamped');
  const sendResume = usePublisher('/explore/resume', 'std_msgs/msg/Bool');

  useEffect(() => {
    if (!meshFile && meshes.length) setMeshFile(meshes[0]);
  }, [meshes, meshFile]);

  const onStats = useCallback((s) => setStats((prev) => ({ ...prev, ...s })), []);
  const say = useCallback((text, tone = 'info') => {
    const id = Date.now();
    setToast({ id, text, tone });
    setTimeout(() => setToast((t) => (t && t.id === id ? null : t)), 3200);
  }, []);

  // Click on the floor: pin at once, go2_click_to_goal snaps it to safe
  // floor and sends the Nav2 goal; the pin then glides to that goal.
  const onPick = ({ x, y }) => {
    const ok = sendClick({
      header: { frame_id: FIXED_FRAME, stamp: { sec: 0, nanosec: 0 } },
      point: { x, y, z: 0 },
    });
    if (!ok) { say('Not connected to the robot', 'warn'); return; }
    setWaypoint({ id: Date.now(), x, y });
    say('Heading to the selected point');
  };

  // Any goal Nav2 receives (snapped click, dock, explorer frontier) moves the pin.
  const onGoal = useCallback((m) => {
    const f = (m.header?.frame_id || '').replace(/^\//, '');
    if (f && f !== FIXED_FRAME) return;
    const { x, y } = m.pose.position;
    setWaypoint((w) => (w && Math.hypot(w.x - x, w.y - y) < 3 ? { ...w, x, y } : { id: Date.now(), x, y }));
  }, []);

  const setExplore = (on) => {
    if (!sendResume({ data: on })) { say('Not connected to the robot', 'warn'); return; }
    say(on ? 'Auto explore started' : 'Auto explore stopped');
  };

  // Pause the explorer first, or its next frontier would override the dock goal.
  const returnToDock = () => {
    if (!sendResume({ data: false })) { say('Not connected to the robot', 'warn'); return; }
    setTimeout(() => sendGoal({
      header: { frame_id: FIXED_FRAME, stamp: { sec: 0, nanosec: 0 } },
      pose: { position: { x: DOCK.x, y: DOCK.y, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
    }), 800);
    setWaypoint({ id: Date.now(), x: DOCK.x, y: DOCK.y });
    say('Returning to dock');
  };

  const meshUrl = meshFile ? `${ASSETS}maps/${encodeURIComponent(meshFile)}` : null;

  return (
    <div className="app">
      <Scene fixedFrame={FIXED_FRAME} baseFrame={BASE_FRAME} layers={layers} view={view}
             meshUrl={meshUrl} textured={textured} shadows={shadows} voxelSize={voxelSize}
             assets={ASSETS} clickToGo={clickToGo && connected} follow={follow}
             waypoint={waypoint} onPick={onPick} onGoal={onGoal} onStats={onStats} />

      <MapMenu view={view} setView={setView} meshes={meshes} meshFile={meshFile} setMeshFile={setMeshFile}
               voxelSize={voxelSize} setVoxelSize={setVoxelSize} textured={textured} setTextured={setTextured}
               shadows={shadows} setShadows={setShadows} layers={layers} setLayers={setLayers} stats={stats} />
      <StatusBar />
      <VlmBadge />
      <ControlPanel connected={connected} onExplore={setExplore} onDock={returnToDock}
                    clickToGo={clickToGo} setClickToGo={setClickToGo} follow={follow} setFollow={setFollow}
                    hasWaypoint={!!waypoint} onClearWaypoint={() => setWaypoint(null)} />
      <CameraFeed />

      {view === 'mesh' && meshes.length === 0 && (
        <div className="glass hint">No saved 3D scan yet. It is created after a mapping run.</div>
      )}
      {toast && <div key={toast.id} className={`glass toast ${toast.tone}`}>{toast.text}</div>}
      {!connected && view !== 'mesh' && (
        <div className="connecting glass">
          <span className="spinner" /> Connecting to the robot…
        </div>
      )}
    </div>
  );
}
