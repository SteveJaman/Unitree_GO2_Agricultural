import { useEffect, useRef, useState } from 'react';
import { useTopic } from './RosProvider.jsx';

const LOOK = {
  HYDROPONIC: { icon: '🌱', title: 'Hydroponic station spotted', tone: 'good' },
  BLOCKED: { icon: '⛔', title: 'Path blocked', tone: 'bad' },
  CLEAR: { icon: '✓', title: 'Path clear', tone: 'ok' },
};

/**
 * AI vision (/vlm_decision) at the top centre: a large toast for 5 s when
 * the decision changes to something noteworthy, then a small badge.
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
  useEffect(() => () => clearTimeout(timer.current), []);

  if (toast) {
    const l = LOOK[toast.d];
    return (
      <div key={toast.id} className={`vlm-toast glass ${l.tone}`}>
        <span className="vlm-icon">{l.icon}</span>
        <div>
          <div className="vlm-title">{l.title}</div>
          <div className="vlm-sub">{toast.d === 'BLOCKED' ? 'Pausing exploration · returning to dock' : 'AI vision'}</div>
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
