import { go, useHub, worstLevel } from './hub.js';

/**
 * Map / Decisions / System / Settings, in the status bar. The System dot is the worst
 * check (go2_hub.py /api/state). Hidden when the page is not served by the hub.
 */
export default function HubNav({ route }) {
  const [state, err] = useHub('/api/state', 5000);
  const hub = err?.status !== 404;     // System and Settings need go2_hub.py; Decisions does not
  const level = worstLevel(state?.checks);
  const tab = (id, label, extra) => (
    <button className={`hub-tab ${route === id || route.startsWith(`${id}/`) ? 'on' : ''}`}
            onClick={() => go(id)} aria-current={route === id ? 'page' : undefined}>
      {label}{extra}
    </button>
  );
  return (
    <div className="hub-nav" role="navigation" aria-label="Dashboard">
      {tab('map', 'Map')}
      {tab('decisions', 'Decisions')}
      {hub && tab('system', 'System', level && <span className={`hub-dot ${level}`} aria-label={`System: ${level}`} />)}
      {hub && tab('settings', 'Settings')}
    </div>
  );
}
