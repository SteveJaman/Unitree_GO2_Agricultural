import { useLatest } from './RosProvider.jsx';

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
