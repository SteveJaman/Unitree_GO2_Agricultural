import { useEffect, useState } from 'react';
import { useLatest, usePublisher, useRos } from './RosProvider.jsx';

const DECISION_COLORS = { CLEAR: '#22c55e', BLOCKED: '#ef4444', HYDROPONIC: '#38bdf8', UNKNOWN: '#9ca3af' };

/** Seconds since `t` (ms), refreshed every second. */
function useAge(t) {
  const [now, setNow] = useState(Date.now());
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);
  return t ? Math.round((now - t) / 1000) : null;
}

/**
 * Top bar: connection, battery (/battery_state + /battery_manager/status),
 * commanded speed (/cmd_vel), VLM decision (/vlm_decision), explorer state
 * and START / STOP buttons (/explore/resume).
 */
export default function StatusBar({ mapPoints }) {
  const { status, url } = useRos();
  const [battery, batteryTime] = useLatest('/battery_state', 'sensor_msgs/msg/BatteryState');
  const [dock] = useLatest('/battery_manager/status', 'std_msgs/msg/String');
  const [cmd] = useLatest('/cmd_vel', 'geometry_msgs/msg/Twist', { throttle: 200 });
  const [vlm, vlmTime] = useLatest('/vlm_decision', 'std_msgs/msg/String');
  const [explore] = useLatest('/explore/status', 'std_msgs/msg/String', { throttle: 500 });
  const sendResume = usePublisher('/explore/resume', 'std_msgs/msg/Bool');
  const batteryAge = useAge(batteryTime);
  const vlmAge = useAge(vlmTime);

  const pct = battery && Number.isFinite(battery.percentage) ? Math.round(battery.percentage * 100) : null;
  const batteryColor = pct === null ? '#9ca3af' : pct < 15 ? '#ef4444' : pct < 30 ? '#f59e0b' : '#22c55e';
  const dockState = dock?.data?.split(' ')[0];
  const speed = cmd ? Math.hypot(cmd.linear.x, cmd.linear.y) : null;
  const decision = vlm?.data || null;
  const exploreState = explore?.data?.match(/state=(\S+)/)?.[1];

  return (
    <header className="statusbar">
      <div className="brand">Go2 Digital Twin</div>
      <Item label="rosbridge" title={url}>
        <span className={`dot ${status}`} /> {status}
      </Item>
      <Item label="Battery" title={dock?.data}>
        <span style={{ color: batteryColor }}>{pct === null ? '—' : `${pct} %`}</span>
        {battery && Number.isFinite(battery.voltage) && battery.voltage > 0 &&
          <small> {battery.voltage.toFixed(1)} V</small>}
        {dockState && dockState !== 'NORMAL' && dockState !== 'MONITOR' &&
          <span className="badge warn">{dockState}</span>}
        {batteryAge !== null && batteryAge > 10 && <small className="stale"> ({batteryAge} s old)</small>}
      </Item>
      <Item label="Speed (cmd)">
        {speed === null ? '—' : `${speed.toFixed(2)} m/s`}
        {cmd && Math.abs(cmd.angular.z) > 0.01 && <small> {cmd.angular.z.toFixed(2)} rad/s</small>}
      </Item>
      <Item label="VLM" title="/vlm_decision (informational)">
        {decision
          ? <span className="badge" style={{ background: DECISION_COLORS[decision] || '#6b7280' }}>{decision}</span>
          : '—'}
        {vlmAge !== null && <small> {vlmAge} s ago</small>}
      </Item>
      <Item label="Map">{mapPoints ? `${(mapPoints / 1000).toFixed(0)}k pts` : '—'}</Item>
      <Item label="Explorer">
        {exploreState || '—'}
        <button className="btn go" onClick={() => sendResume({ data: true })}
                disabled={status !== 'connected'}>START</button>
        <button className="btn stop" onClick={() => sendResume({ data: false })}
                disabled={status !== 'connected'}>STOP</button>
      </Item>
    </header>
  );
}

function Item({ label, title, children }) {
  return (
    <div className="item" title={title}>
      <span className="label">{label}</span>
      <span className="value">{children}</span>
    </div>
  );
}
