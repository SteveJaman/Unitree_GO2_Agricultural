import { useEffect, useState } from 'react';
import { useLatest, useRos } from './RosProvider.jsx';

/** Seconds since `t` (ms), refreshed every second. */
export function useAge(t) {
  const [now, setNow] = useState(Date.now());
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);
  return t ? Math.round((now - t) / 1000) : null;
}

/** Battery icon, filled to the level; green / amber below 30 % / red below 15 %. */
function BatteryIcon({ pct, charging }) {
  const level = pct === null ? 0 : Math.max(0, Math.min(100, pct));
  const color = pct === null ? '#64748b' : pct < 15 ? '#ff3b5c' : pct < 30 ? '#f5a524' : '#34d399';
  return (
    <svg width="34" height="18" viewBox="0 0 34 18" aria-hidden="true">
      <rect x="1" y="1" width="28" height="16" rx="4" fill="none" stroke="rgba(255,255,255,.55)" strokeWidth="1.6" />
      <rect x="30.5" y="6" width="2.5" height="6" rx="1.2" fill="rgba(255,255,255,.55)" />
      <rect x="3.5" y="3.5" width={Math.max(0, 23 * level / 100)} height="11" rx="2.2" fill={color} />
      {charging && <path d="M16 4 L11.5 10 H15 L13.5 14 L19 8 H15.5 Z" fill="#fff" />}
    </svg>
  );
}

/**
 * Top-right status: link to the robot, speed, battery (from /battery_state;
 * red below 15 %, with the docking state when the battery manager acts).
 */
export default function StatusBar() {
  const { status } = useRos();
  const [battery, batteryTime] = useLatest('/battery_state', 'sensor_msgs/msg/BatteryState');
  const [dock] = useLatest('/battery_manager/status', 'std_msgs/msg/String');
  const [cmd] = useLatest('/cmd_vel', 'geometry_msgs/msg/Twist', { throttle: 200 });
  const age = useAge(batteryTime);

  const pct = battery && Number.isFinite(battery.percentage) && age !== null && age < 30
    ? Math.round(battery.percentage * 100) : null;
  const dockState = dock?.data?.split(' ')[0];
  const docking = dockState === 'RETURNING' || dockState === 'DOCKED';
  const speed = cmd ? Math.hypot(cmd.linear.x, cmd.linear.y) : 0;
  const link = status === 'connected' ? 'Online' : status === 'connecting' ? 'Connecting' : 'Offline';

  return (
    <div className="status glass">
      <div className="status-item" title={`Robot link: ${link}`}>
        <span className={`dot ${status}`} />
        <span className="status-text">{link}</span>
      </div>
      <div className="status-sep" />
      <div className="status-item" title="Speed">
        <span className="status-value">{speed.toFixed(1)}</span>
        <span className="status-unit">m/s</span>
      </div>
      <div className="status-sep" />
      <div className={`status-item ${pct !== null && pct < 15 ? 'low' : ''}`}
           title={docking ? (dockState === 'DOCKED' ? 'At the dock' : 'Low battery: returning to dock') : 'Battery'}>
        <BatteryIcon pct={pct} />
        <span className="status-value">{pct === null ? '--' : pct}</span>
        <span className="status-unit">%</span>
        {docking && <span className="pill warn">{dockState === 'DOCKED' ? 'Docked' : 'Docking'}</span>}
      </div>
    </div>
  );
}
