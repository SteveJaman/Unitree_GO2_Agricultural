import { useCallback, useEffect, useLayoutEffect, useRef, useState } from 'react';
import { RosProvider, usePublisher, useRos, useTopic } from './RosProvider.jsx';
import RoomsPanel from './RoomsPanel.jsx';
import RoomNameModal from './RoomNameModal.jsx';
import { normalizeRooms, roomName } from './ros/rooms.js';
import { initialQuality, saveQuality } from './ros/quality.js';
import { fetchConfigJson } from './ros/config.js';
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

/**
 * Publish the bottom-left controls' height as --controls-h on the app, so the
 * top-left settings column stops above them on any screen (styles.css).
 */
function useControlsHeight(appRef) {
  useLayoutEffect(() => {
    const app = appRef.current;
    const controls = app?.querySelector('.controls');
    if (!controls || typeof ResizeObserver === 'undefined') return undefined;
    const ro = new ResizeObserver(() => app.style.setProperty('--controls-h', `${controls.offsetHeight}px`));
    ro.observe(controls);
    return () => ro.disconnect();
  }, [appRef]);
}

/** A callback with a stable identity that always runs the latest closure. */
function useStable(fn) {
  const ref = useRef(fn);
  ref.current = fn;
  return useCallback((...args) => ref.current(...args), []);
}

function Dashboard() {
  const { status } = useRos();
  const connected = status === 'connected';
  const appRef = useRef(null);
  useControlsHeight(appRef);
  const [quality, setQualityState] = useState(initialQuality);
  const lite = quality === 'lite';
  const [layers, setLayers] = useState({ map: true, scan: false, yolo: true, labels: true, rooms: true, arm: true });
  const [view, setView] = useState(params.get('view') || 'solid');
  const [textured, setTextured] = useState(true);
  const [shadows, setShadows] = useState(!lite);
  const setQuality = (q) => {
    setQualityState(q);
    saveQuality(q);
    setShadows(q !== 'lite');
  };
  const [voxelSize, setVoxelSize] = useState('auto');
  const [clickToGo, setClickToGoState] = useState(true);
  // Inspect mode: clicks pick a 3D target for inspection_mission.py instead of a goal.
  const [inspectMode, setInspectModeState] = useState(false);
  const [inspectTarget, setInspectTarget] = useState(null);
  const setClickToGo = (on) => { setClickToGoState(on); if (on) setInspectModeState(false); };
  const setInspectMode = (on) => { setInspectModeState(on); if (on) setClickToGoState(false); };
  const [follow, setFollow] = useState(false);
  const [stats, setStats] = useState({});
  const [toast, setToast] = useState(null);
  const [waypoint, setWaypoint] = useState(null);   // { id, x, y }
  const meshes = useMeshes();
  const [meshFile, setMeshFile] = useState('');
  const sendClick = usePublisher('/clicked_point', 'geometry_msgs/msg/PointStamped');
  const sendInspect = usePublisher('/inspect_point', 'geometry_msgs/msg/PointStamped');
  const sendInspectCmd = usePublisher('/inspection/command', 'std_msgs/msg/String');
  const sendGoal = usePublisher('/goal_pose', 'geometry_msgs/msg/PoseStamped');
  const sendResume = usePublisher('/explore/resume', 'std_msgs/msg/Bool');
  const sendRoomCmd = usePublisher('/semantic_rooms/command', 'std_msgs/msg/String');
  const sendRoomAsk = usePublisher('/vlm_room_request', 'std_msgs/msg/String');

  // ------------------------------------------------------------ rooms
  const [rooms, setRooms] = useState(normalizeRooms(null));
  const [roomsOpen, setRoomsOpen] = useState(true);
  const [editing, setEditing] = useState(false);
  const [merging, setMerging] = useState(false);
  const [selected, setSelected] = useState([]);
  const [naming, setNaming] = useState(null);          // room being named
  const [suggestions, setSuggestions] = useState({});  // room id -> VLM name
  const [asking, setAsking] = useState(null);          // room id awaiting the VLM
  const liveRooms = useRef(false);

  useTopic('/semantic_rooms/json', 'std_msgs/msg/String', (m) => {
    try {
      const d = normalizeRooms(JSON.parse(m.data));
      liveRooms.current = true;
      setRooms(d);
      if (d.error) say(d.error.replace(/room_(\d+)/g, 'Room $1'), 'warn');
    } catch { /* ignore a malformed message */ }
  });
  useTopic('/vlm_room_suggestion', 'std_msgs/msg/String', (m) => {
    try {
      const d = JSON.parse(m.data);
      if (d.suggestion) setSuggestions((s) => ({ ...s, [d.room_id]: d.suggestion }));
      setAsking((a) => (a === d.room_id ? null : a));
    } catch { /* ignore */ }
  });
  // Offline (--view, or before the room node answers): the saved room map.
  useEffect(() => {
    let alive = true;
    const load = () => !liveRooms.current && fetchConfigJson(ASSETS, 'semantic_rooms.json')
      .then((d) => alive && d && !liveRooms.current && setRooms(normalizeRooms(d)))
      .catch(() => {});
    load();
    const id = setInterval(load, 20000);
    return () => { alive = false; clearInterval(id); };
  }, []);
  // Editing rooms takes over clicks from tap-to-go.
  useEffect(() => { if (editing) { setClickToGoState(false); setInspectModeState(false); } }, [editing]);

  const askRoom = (id) => {
    if (!sendRoomAsk({ data: id })) return;
    setAsking(id);
    setTimeout(() => setAsking((a) => (a === id ? null : a)), 90000);
  };
  const openNaming = (room) => {
    setNaming(room);
    if (!suggestions[room.id]) askRoom(room.id);
  };
  const onRoomClick = useStable((room) => {
    if (merging) {
      setSelected((s) => (s.includes(room.id) ? s.filter((x) => x !== room.id) : [...s, room.id].slice(-2)));
      return;
    }
    openNaming(room);
  });
  const saveName = (name) => {
    if (!sendRoomCmd({ data: JSON.stringify({ op: 'rename', id: naming.id, label: name }) })) {
      say('Not connected to the robot', 'warn');
      return;
    }
    setRooms((d) => ({ ...d, rooms: d.rooms.map((r) => (r.id === naming.id ? { ...r, label: name } : r)) }));
    say(`Saved “${name}”`);
    setNaming(null);
  };
  const mergeRooms = () => {
    if (selected.length !== 2) return;
    const [a, b] = selected.map((id) => rooms.rooms.find((r) => r.id === id));
    sendRoomCmd({ data: JSON.stringify({ op: 'merge', ids: selected }) });
    say(`Merging ${roomName(a)} and ${roomName(b)}`);
    setMerging(false);
    setSelected([]);
  };
  // Drive to the room's most open point; snapped like a click, explorer paused first.
  const goToRoom = (room) => {
    if (!sendResume({ data: false })) { say('Not connected to the robot', 'warn'); return; }
    const [x, y] = room.center;
    setTimeout(() => sendClick({
      header: { frame_id: rooms.frame || FIXED_FRAME, stamp: { sec: 0, nanosec: 0 } },
      point: { x, y, z: 0 },
    }), 800);
    setWaypoint({ id: Date.now(), x, y });
    say(`Heading to ${roomName(room)}`);
  };

  useEffect(() => {
    if (!meshFile && meshes.length) setMeshFile(meshes[0]);
  }, [meshes, meshFile]);

  const onStats = useCallback((s) => setStats((prev) => ({ ...prev, ...s })), []);
  function say(text, tone = 'info') {
    const id = Date.now();
    setToast({ id, text, tone });
    setTimeout(() => setToast((t) => (t && t.id === id ? null : t)), 3200);
  }

  // Inspect: the exact 3D point under the cursor -> inspection_mission.py
  // (walk there facing it, then the arm's IK scan).
  const onInspect = ({ x, y, z, surface }) => {
    const ok = sendInspect({
      header: { frame_id: FIXED_FRAME, stamp: { sec: 0, nanosec: 0 } },
      point: { x, y, z },
    });
    if (!ok) { say('Not connected to the robot', 'warn'); return; }
    setInspectTarget({ x, y, z });
    say(surface ? `Inspecting the point at ${z.toFixed(2)} m` : 'Inspecting a point on the floor (no map surface there)');
  };
  const cancelInspect = () => {
    sendInspectCmd({ data: JSON.stringify({ op: 'cancel' }) });
    setInspectTarget(null);
    say('Inspection cancelled');
  };

  // Click: the exact 3D point (map surface, else floor). go2_click_to_goal
  // snaps it to safe floor and sends the Nav2 goal; the pin glides to it.
  const onPick = useStable(({ x, y, z = 0, surface }) => {
    if (inspectMode) { onInspect({ x, y, z, surface }); return; }
    const ok = sendClick({
      header: { frame_id: FIXED_FRAME, stamp: { sec: 0, nanosec: 0 } },
      point: { x, y, z },
    });
    if (!ok) { say('Not connected to the robot', 'warn'); return; }
    setWaypoint({ id: Date.now(), x, y });
    say('Heading to the selected point');
  });

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
    <div ref={appRef} className={`app ${lite ? 'lite' : ''}`}>
      {/* keyed: antialiasing is fixed when the WebGL context is created */}
      <Scene key={quality} quality={quality} fixedFrame={FIXED_FRAME} baseFrame={BASE_FRAME} layers={layers} view={view}
             meshUrl={meshUrl} textured={textured} shadows={shadows} voxelSize={voxelSize}
             assets={ASSETS} clickToGo={(clickToGo || inspectMode) && connected} follow={follow}
             inspectTarget={inspectTarget}
             waypoint={waypoint} onPick={onPick} onGoal={onGoal} onStats={onStats}
             rooms={rooms} roomsInteractive={connected && (editing || merging || (!clickToGo && !inspectMode))}
             selectedRooms={selected} onRoomClick={onRoomClick} />

      <MapMenu view={view} setView={setView} meshes={meshes} meshFile={meshFile} setMeshFile={setMeshFile}
               voxelSize={voxelSize} setVoxelSize={setVoxelSize} textured={textured} setTextured={setTextured}
               shadows={shadows} setShadows={setShadows} layers={layers} setLayers={setLayers} stats={stats}
               quality={quality} setQuality={setQuality} />
      <StatusBar />
      <VlmBadge />
      <ControlPanel connected={connected} onExplore={setExplore} onDock={returnToDock}
                    clickToGo={clickToGo} setClickToGo={setClickToGo} follow={follow} setFollow={setFollow}
                    inspectMode={inspectMode} setInspectMode={setInspectMode} onCancelInspect={cancelInspect}
                    hasWaypoint={!!waypoint} onClearWaypoint={() => setWaypoint(null)}
                    rooms={rooms.rooms} onGoRoom={goToRoom} />
      <CameraFeed />
      <RoomsPanel rooms={rooms.rooms} connected={connected} open={roomsOpen} setOpen={setRoomsOpen}
                  editing={editing} setEditing={setEditing} merging={merging}
                  setMerging={(m) => { setMerging(m); setSelected([]); }}
                  selected={selected} onRename={openNaming} onGo={goToRoom}
                  onMerge={mergeRooms} onCancelMerge={() => { setMerging(false); setSelected([]); }} />
      {naming && (
        <RoomNameModal key={naming.id} room={naming} suggestion={suggestions[naming.id]}
                       asking={asking === naming.id} onAsk={() => askRoom(naming.id)}
                       onSave={saveName} onClose={() => setNaming(null)} />
      )}

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
