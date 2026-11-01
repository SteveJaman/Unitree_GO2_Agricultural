import { useEffect, useMemo, useState } from 'react';
import { api, copyText, download, go, useHub } from './hub.js';

const LEVEL = {
  ok: { mark: '✓', word: 'OK' },
  warn: { mark: '!', word: 'Check' },
  bad: { mark: '✕', word: 'Blocked' },
  info: { mark: 'i', word: 'Note' },
};

function NoHub() {
  return (
    <div className="hub-empty glass">
      System needs the dashboard hub, which <code>./scripts/runtime/wifi_3d_nav.sh</code> starts (not with
      <code>--no-hub</code> or <code>--view</code>).
    </div>
  );
}

function Checks({ checks }) {
  const [open, setOpen] = useState(null);
  return (
    <section className="hub-card glass">
      <div className="hub-card-head">
        <h2>Why isn't it moving?</h2>
        <span className="muted small">checked in order, LiDAR to legs · every 2 s</span>
      </div>
      {checks.map((c, i) => {
        const L = LEVEL[c.level] || LEVEL.info;
        const isOpen = open === i;
        return (
          <div key={i} className={`check ${c.level} ${isOpen ? 'open' : ''}`}>
            <button className="check-row" onClick={() => setOpen(isOpen ? null : i)} aria-expanded={isOpen}>
              <span className="check-mark">{L.mark}</span>
              <span className="check-title">{c.title}</span>
              <span className="check-word">{L.word}</span>
            </button>
            {isOpen && c.detail && <div className="check-detail">{c.detail}</div>}
          </div>
        );
      })}
    </section>
  );
}

function Topics({ topics }) {
  const rows = Object.entries(topics || {});
  return (
    <section className="hub-card glass">
      <h2>Topics</h2>
      {rows.length === 0 ? <span className="muted small">No live data yet.</span> : (
        <div className="table-scroll">
          <table className="hub-table">
            <thead><tr><th>Topic</th><th>Rate</th><th>Age</th></tr></thead>
            <tbody>
              {rows.map(([t, v]) => {
                const stale = v.age === null || v.age > 3;
                return (
                  <tr key={t} className={stale ? 'stale' : ''}>
                    <td className="mono">{t}</td>
                    <td>{v.hz} Hz</td>
                    <td>{v.age === null ? 'never' : `${v.age} s`}</td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      )}
    </section>
  );
}

function Tiles({ health }) {
  const ping = health.ping || {};
  const wifi = health.wifi || {};
  const tiles = [
    { label: 'Robot ping', value: ping.avg_ms != null ? `${Math.round(ping.avg_ms)} ms` : '—',
      note: health.robot_ip ? `${health.robot_ip} · ${ping.loss ?? '?'}% lost` : 'robot IP unknown',
      tone: ping.loss >= 100 ? 'bad' : ping.loss > 0 ? 'warn' : ping.avg_ms != null ? 'ok' : '' },
    { label: 'Wi-Fi signal', value: wifi.signal_dbm != null ? `${wifi.signal_dbm} dBm` : wifi.signal_pct != null ? `${wifi.signal_pct}%` : '—',
      note: wifi.ssid || health.iface || '', tone: wifi.signal_dbm != null && wifi.signal_dbm < -75 ? 'warn' : '' },
    { label: 'Containers', value: `${(health.containers || []).filter((c) => /^Up/.test(c.status)).length} up`,
      note: (health.containers || []).map((c) => c.name).join(', '), tone: '' },
    { label: 'Ollama (VLM)', value: health.ollama ? 'On' : 'Off',
      note: health.ollama ? health.ollama.join(', ') : 'fix: ollama serve', tone: health.ollama ? 'ok' : 'warn' },
    { label: 'UDP buffers', value: health.rmem_max ? `${Math.round(health.rmem_max / 1048576)} MB` : '—',
      note: health.rmem_max && health.rmem_max < 8e6 ? 'too small for the camera' : 'camera frames safe',
      tone: health.rmem_max && health.rmem_max < 8e6 ? 'warn' : 'ok' },
  ];
  return (
    <div className="tiles">
      {tiles.map((t) => (
        <div key={t.label} className={`tile glass ${t.tone}`}>
          <span className="muted small">{t.label}</span>
          <span className="tile-value">{t.value}</span>
          <span className="muted small tile-note">{t.note}</span>
        </div>
      ))}
    </div>
  );
}

function Report({ onClose }) {
  const [text, setText] = useState('');
  const [state, setState] = useState('');
  useEffect(() => {
    api('/api/report').then(setText).catch((e) => setText(`Could not build the report: ${e.message}`));
    const onKey = (e) => { if (e.key === 'Escape') onClose(); };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, []);
  return (
    <div className="modal-backdrop" onClick={onClose}>
      <div className="modal glass report" role="dialog" aria-modal="true" aria-labelledby="report-title"
           onClick={(e) => e.stopPropagation()}>
        <div className="hub-card-head">
          <h2 id="report-title">Debug report</h2>
          <span className="muted small">Markdown · paste it into a chat or an issue</span>
        </div>
        <pre className="report-text">{text || 'Collecting…'}</pre>
        <div className="modal-actions">
          <button className="chip" onClick={onClose}>Close</button>
          <button className="chip" disabled={!text}
                  onClick={() => { download(`go2-report-${new Date().toISOString().slice(0, 16).replace(/[:T]/g, '-')}.md`, text); setState('saved'); }}>
            {state === 'saved' ? 'Saved' : 'Download .md'}
          </button>
          <button className="primary" disabled={!text}
                  onClick={async () => setState((await copyText(text)) ? 'copied' : 'copy-failed')}>
            {state === 'copied' ? 'Copied' : state === 'copy-failed' ? 'Copy blocked: use Download' : 'Copy to clipboard'}
          </button>
        </div>
      </div>
    </div>
  );
}

function Logs() {
  const [list, listErr] = useHub('/api/logs', 15000);
  const [id, setId] = useState(null);
  const [problems, setProblems] = useState(false);
  const [follow, setFollow] = useState(true);
  const [log] = useHub(id ? `/api/log?id=${encodeURIComponent(id)}&n=400` : null, follow ? 3000 : 0);
  const groups = useMemo(() => {
    const where = { host: 'This computer', container: 'Container go2-lidar3d', 'container-ros': 'Nav2 (go2-lidar3d)', robot: 'The robot' };
    const out = {};
    for (const l of list || []) (out[where[l.where] || l.where] ||= []).push(l);
    return Object.entries(out);
  }, [list]);
  useEffect(() => {
    if (!id && list?.length) setId((list.find((l) => l.id === '3d:boot') || list[0]).id);
  }, [list, id]);
  if (listErr?.status === 404) return <NoHub />;
  const lines = (log?.text || '').split('\n').filter((l) => !problems || /WARN|ERROR|FATAL|FAIL|Traceback/.test(l));
  const current = (list || []).find((l) => l.id === id);
  return (
    <div className="logs">
      <nav className="logs-list glass" aria-label="Logs">
        {groups.map(([g, items]) => (
          <div key={g} className="logs-group">
            <span className="muted small logs-group-name">{g}</span>
            {items.map((l) => (
              <button key={l.id} className={`logs-item ${l.id === id ? 'on' : ''}`} onClick={() => setId(l.id)}
                      aria-pressed={l.id === id}>
                {l.label.replace(/^3D twin: boot$/, 'Start-up (boot.log)')}
              </button>
            ))}
          </div>
        ))}
      </nav>
      <section className="logs-view glass">
        <div className="hub-card-head">
          <h2>{current ? current.label : 'Logs'}</h2>
          <div className="row-gap">
            <button className={`chip ${!problems ? 'active' : ''}`} onClick={() => setProblems(false)}>All</button>
            <button className={`chip ${problems ? 'active' : ''}`} onClick={() => setProblems(true)}>Warnings and errors</button>
            <label className="muted small check-label">
              <input type="checkbox" checked={follow} onChange={(e) => setFollow(e.target.checked)} /> Follow
            </label>
          </div>
        </div>
        <pre className="log-text">
          {lines.map((l, i) => (
            <span key={i} className={/ERROR|FATAL|Traceback/.test(l) ? 'err' : /WARN|FAIL/.test(l) ? 'warn' : ''}>{l}{'\n'}</span>
          ))}
        </pre>
      </section>
    </div>
  );
}

/** System tab: health and the "why isn't it moving?" chain, every log, the debug report. */
export default function SystemView({ sub }) {
  const [state, err] = useHub('/api/state', 2000);
  const [report, setReport] = useState(false);
  const noHub = err?.status === 404;
  return (
    <div className="hub-page">
      <div className="hub-page-head">
        <div className="segmented" role="tablist">
          <button className={sub !== 'logs' ? 'active' : ''} onClick={() => go('system')} role="tab" aria-selected={sub !== 'logs'}>Health</button>
          <button className={sub === 'logs' ? 'active' : ''} onClick={() => go('system/logs')} role="tab" aria-selected={sub === 'logs'}>Logs</button>
        </div>
        {!noHub && <button className="primary" onClick={() => setReport(true)}>Copy debug report</button>}
      </div>
      {noHub ? <NoHub /> : sub === 'logs' ? <Logs /> : (
        <div className="hub-grid">
          {state ? <Checks checks={state.checks} /> : <section className="hub-card glass muted">Loading…</section>}
          <div className="hub-side">
            <Topics topics={state?.probe?.state?.topics} />
            {state && <Tiles health={state.health || {}} />}
            {state?.probe?.note && state.probe.note !== 'live' && <span className="muted small">Probe: {state.probe.note}</span>}
          </div>
        </div>
      )}
      {report && <Report onClose={() => setReport(false)} />}
    </div>
  );
}
