import { useEffect, useRef, useState } from 'react';
import { useTopic } from './RosProvider.jsx';

const LOOK = {
  HYDROPONIC: { icon: '🌱', title: 'Hydroponic station spotted', tone: 'good' },
  BLOCKED: { icon: '⛔', title: 'Path blocked', tone: 'bad' },
  CLEAR: { icon: '✓', title: 'Path clear', tone: 'ok' },
};

// What go2_stuck_recovery.py is doing (/recovery/status), shown as a toast.
const RECOVERY = {
  verifying: { icon: '🔍', title: 'Checking the obstacle with the camera', tone: 'ok' },
  ghost: { icon: '👻', title: 'LiDAR ghost: camera sees the way clear', tone: 'good' },
  sidestep: { icon: '↔', title: 'Stepping round the obstacle', tone: 'bad' },
  blocked: { icon: '⛔', title: 'Path blocked', tone: 'bad' },
  stuck: { icon: '⛔', title: 'Robot stuck', tone: 'bad' },
};

/**
 * AI vision (/vlm_decision) at the top centre: a large toast for 5 s when
 * the decision changes to something noteworthy, then a small badge. Recovery
 * steps (camera check of a LiDAR obstacle, sidesteps) show as toasts too.
 */
export default function VlmBadge() {
  const [decision, setDecision] = useState(null);
  const [toast, setToast] = useState(null);
  const last = useRef(null);
  const timer = useRef(null);

  useTopic('/vlm_decision', 'std_msgs/msg/String', (m) => {
    const d = LOOK[m.data] ? m.data : null;
    if (!d) return;
    setDecision(d);
    if (d !== last.current && d !== 'CLEAR') {
      clearTimeout(timer.current);
      setToast({ d, id: Date.now() });
      timer.current = setTimeout(() => setToast(null), 5000);
    }
    last.current = d;
  });
  useTopic('/recovery/status', 'std_msgs/msg/String', (m) => {
    let r = null;
    try { r = JSON.parse(m.data); } catch { return; }
    if (!RECOVERY[r?.state]) return;
    clearTimeout(timer.current);
    setToast({ r, id: Date.now() });
    timer.current = setTimeout(() => setToast(null), 6000);
  });
  useEffect(() => () => clearTimeout(timer.current), []);

  if (toast?.r) {
    const l = RECOVERY[toast.r.state];
    return (
      <div key={toast.id} className={`vlm-toast glass ${l.tone}`}>
        <span className="vlm-icon">{l.icon}</span>
        <div>
          <div className="vlm-title">{l.title}</div>
          <div className="vlm-sub">{toast.r.note}</div>
        </div>
      </div>
    );
  }
  if (toast) {
    const l = LOOK[toast.d];
    return (
      <div key={toast.id} className={`vlm-toast glass ${l.tone}`}>
        <span className="vlm-icon">{l.icon}</span>
        <div>
          <div className="vlm-title">{l.title}</div>
          <div className="vlm-sub">{toast.d === 'BLOCKED' ? 'Checking the way round' : 'AI vision'}</div>
        </div>
      </div>
    );
  }
  if (!decision) return null;
  const l = LOOK[decision];
  return (
    <div className={`vlm-badge glass ${l.tone}`} title="AI vision">
      <span>{l.icon}</span> {l.title}
    </div>
  );
}
