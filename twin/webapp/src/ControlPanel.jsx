import { useLatest, usePublisher } from './RosProvider.jsx';
import SurveyCard from './SurveyCard.jsx';

/**
 * Bottom-left controls: Auto Explore toggle (/explore/resume, state from
 * /explore/status), Return to Dock, tap-to-go and follow.
 */
export default function ControlPanel({
  connected, onExplore, onDock, clickToGo, setClickToGo, follow, setFollow, hasWaypoint, onClearWaypoint,
  rooms = [], onGoRoom,
}) {
  const [status] = useLatest('/explore/status', 'std_msgs/msg/String', { throttle: 500 });
  const exploring = /enabled=True/.test(status?.data || '');
  const state = status?.data?.match(/state=(\S+)/)?.[1];
  const sub = !connected ? 'Robot offline'
    : exploring ? (state === 'RETURNING' ? 'Heading home' : 'Mapping the area')
      : state === 'DONE' ? 'Area complete' : 'Ready';

  return (
    <div className="controls glass">
      <SurveyCard connected={connected} />
      <button className={`explore ${exploring ? 'on' : ''}`} disabled={!connected}
              onClick={() => onExplore(!exploring)}>
        <span className="explore-icon">{exploring ? '■' : '▶'}</span>
        <span className="explore-text">
          <span className="explore-title">{exploring ? 'Stop Exploring' : 'Auto Explore'}</span>
          <span className="explore-sub">{sub}</span>
        </span>
      </button>
      <button className="action" disabled={!connected} onClick={onDock}>
        <span className="action-icon">⌂</span> Return to Dock
      </button>
      <OrbitButton connected={connected} />
      {rooms.length > 0 && (
        <label className="room-go">
          <span className="action-icon">➜</span>
          <select value="" disabled={!connected}
                  onChange={(e) => { const r = rooms.find((x) => x.id === e.target.value); if (r) onGoRoom(r); }}>
            <option value="" disabled>Navigate to room…</option>
            {rooms.map((r) => (
              <option key={r.id} value={r.id}>{r.label || `Room ${r.id.split('_').pop()}`}</option>
            ))}
          </select>
        </label>
      )}
      <div className="chips">
        <button className={`chip ${clickToGo ? 'active' : ''}`} onClick={() => setClickToGo(!clickToGo)}
                title="Click the floor to send the robot there">
          ⌖ Tap to go
        </button>
        <button className={`chip ${follow ? 'active' : ''}`} onClick={() => setFollow(!follow)}
                title="Keep the camera on the robot">
          ◎ Follow
        </button>
        {hasWaypoint && <button className="chip" onClick={onClearWaypoint} title="Hide the waypoint">✕ Clear</button>}
      </div>
    </div>
  );
}

/**
 * D1 arm orbit scan (/d1_arm/command -> arm_state_bridge.py runs
 * start_arm.sh orbit; mock_sim.py --arm fakes it). State from /d1_arm/status.
 */
function OrbitButton({ connected }) {
  const [status] = useLatest('/d1_arm/status', 'std_msgs/msg/String', { throttle: 500 });
  const send = usePublisher('/d1_arm/command', 'std_msgs/msg/String');
  let st = null;
  try { st = status ? JSON.parse(status.data) : null; } catch { st = null; }
  const running = st?.orbit === 'running';
  const ready = connected && st?.arm && st?.commands !== false;
  const note = !connected ? 'Robot offline'
    : !st ? 'Arm bridge not running'
      : !st.arm ? 'Arm offline'
        : st.commands === false ? 'Commands off' : running ? 'Sweeping ±90°'
          : st.orbit === 'done' ? 'Last scan complete' : st.orbit === 'failed' ? 'Last scan failed'
            : st.mock ? 'Simulated arm' : 'Sweep around the object';
  const onClick = () => {
    if (running) { send({ data: JSON.stringify({ op: 'stop' }) }); return; }
    if (!st?.mock && !window.confirm('The D1 arm will sweep its base ±90° around the object. Is the space around the arm clear?')) return;
    send({ data: JSON.stringify({ op: 'orbit' }) });
  };
  return (
    <button className={`action orbit ${running ? 'on' : ''}`} disabled={!ready && !running} onClick={onClick}
            title={note}>
      <span className="action-icon">{running ? '■' : '◠'}</span>
      <span className="orbit-text">
        <span>{running ? 'Stop Orbit Scan' : 'Orbit Scan'}</span>
        <span className="orbit-sub">{note}</span>
      </span>
    </button>
  );
}
