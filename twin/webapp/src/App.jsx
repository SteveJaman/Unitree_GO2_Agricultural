import { useState } from 'react';
import { RosProvider, usePublisher, useRos } from './RosProvider.jsx';
import Scene from './Scene.jsx';
import StatusBar from './StatusBar.jsx';
import CameraFeed from './CameraFeed.jsx';

// Nav2 plans in lidar3d_map (Point-LIO); RTAB-Map's rtabmap_map is its parent.
const params = new URLSearchParams(window.location.search);
const FIXED_FRAME = params.get('frame') || 'lidar3d_map';
const BASE_FRAME = params.get('base') || 'base_link';

export default function App() {
  return (
    <RosProvider>
      <Dashboard />
    </RosProvider>
  );
}

function Dashboard() {
  const { status } = useRos();
  const [layers, setLayers] = useState({ map: true, scan: false, plan: true, yolo: true });
  const [clickToGo, setClickToGo] = useState(false);
  const [follow, setFollow] = useState(false);
  const [stats, setStats] = useState({ mapPoints: 0 });
  const [toast, setToast] = useState(null);
  const sendClick = usePublisher('/clicked_point', 'geometry_msgs/msg/PointStamped');

  const toggle = (k) => setLayers((l) => ({ ...l, [k]: !l[k] }));
  // Same path as Foxglove's "Publish point": go2_click_to_goal snaps it to
  // safe floor and sends the Nav2 goal.
  const onPick = ({ x, y }) => {
    const ok = sendClick({
      header: { frame_id: FIXED_FRAME, stamp: { sec: 0, nanosec: 0 } },
      point: { x, y, z: 0 },
    });
    setToast(ok ? `Goal requested at (${x.toFixed(2)}, ${y.toFixed(2)})` : 'Not connected');
    setTimeout(() => setToast(null), 3000);
  };

  return (
    <div className="app">
      <StatusBar mapPoints={stats.mapPoints} />
      <main>
        <div className="viewport">
          <Scene fixedFrame={FIXED_FRAME} baseFrame={BASE_FRAME} layers={layers}
                 clickToGo={clickToGo} follow={follow} onPick={onPick} onStats={setStats} />
          <div className="toolbar">
            <button className={`btn ${clickToGo ? 'active' : ''}`} onClick={() => setClickToGo((v) => !v)}
                    disabled={status !== 'connected'} title="Click the floor to send the dog there">
              {clickToGo ? 'Click-to-go: ON' : 'Click-to-go'}
            </button>
            <button className={`btn ${follow ? 'active' : ''}`} onClick={() => setFollow((v) => !v)}>
              Follow robot
            </button>
            {[['map', '3D map'], ['scan', 'Live scan'], ['plan', 'Plan'], ['yolo', 'YOLO']].map(([k, label]) => (
              <label key={k} className="check">
                <input type="checkbox" checked={layers[k]} onChange={() => toggle(k)} /> {label}
              </label>
            ))}
            <span className="frame">frame {FIXED_FRAME}</span>
          </div>
          {toast && <div className="toast">{toast}</div>}
          {status !== 'connected' && (
            <div className="overlay">
              Connecting to rosbridge… start the session with <code>./scripts/runtime/wifi_3d_nav.sh</code>
            </div>
          )}
        </div>
        <aside className="side">
          <CameraFeed />
          <div className="help">
            <b>Mouse</b>: left drag orbit · right drag pan · wheel zoom.<br />
            <b>Click-to-go</b>: turn on, then click the floor; clicks on furniture are moved to the
            nearest safe floor. Each click replaces the current goal.
          </div>
        </aside>
      </main>
    </div>
  );
}
