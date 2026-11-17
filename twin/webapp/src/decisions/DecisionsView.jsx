import { useMemo, useState } from 'react';
import { download, go } from '../hub.js';
import { BANDS, HOW, LABEL_BAND, RESULTS } from './content.js';
import { explain, nearCutoff, useDecisions } from './useDecisions.js';

/**
 * Decisions tab (#/decisions, #/decisions/how, #/decisions/results): what the robot
 * decided about each object and why, how it decides, and the results. Written for
 * reviewers and visitors as much as for the operator.
 */
export default function DecisionsView({ sub }) {
  const d = useDecisions();
  const tabs = [['', 'Live'], ['how', 'How it decides'], ['results', 'Results']];
  return (
    <div className="hub-page dx">
      <div className="hub-page-head">
        <div className="dx-head">
          <h1 className="hub-title">Decisions</h1>
          <span className="muted small">step over, go carefully, or keep away: object by object</span>
        </div>
        <div className="dx-tabs" role="tablist">
          {tabs.map(([id, label]) => (
            <button key={id} role="tab" aria-selected={sub === id} className={`chip ${sub === id ? 'active' : ''}`}
                    onClick={() => go(id ? `decisions/${id}` : 'decisions')}>{label}</button>
          ))}
        </div>
      </div>
      {sub === 'how' ? <How /> : sub === 'results' ? <Results /> : <Live d={d} />}
    </div>
  );
}

// --- Live ---------------------------------------------------------------------------------

const GAIT_PLAIN = {
  NORMAL: 'walking normally',
  HIGH_STEP: 'high steps (an item it can clear)',
  CAREFUL: 'slow and careful',
  STOP: 'stopped: blocked ahead',
};

function Band({ label }) {
  const b = LABEL_BAND[label] || BANDS.unknown;
  return <span className={`dx-band ${b.tone}`}>{b.name}</span>;
}

function Live({ d }) {
  const counts = useMemo(() => {
    const c = { STEP_OVER: 0, CAUTION: 0, AVOID: 0, UNKNOWN: 0 };
    d.objects.forEach((o) => { c[o.label] = (c[o.label] || 0) + 1; });
    return c;
  }, [d.objects]);
  const gait = d.gait?.gait;
  return (
    <>
      {d.demo && (
        <div className="dx-note glass small">
          Sample data: the robot is not connected, so a scripted example plays. Live data replaces it as soon as a
          session with <code>--semantic</code> is running.
        </div>
      )}
      <div className="dx-tiles">
        <Tile label="Gait now" value={gait || '—'} note={GAIT_PLAIN[gait] || 'no gait decision yet'}
              tone={gait === 'STOP' ? 'bad' : gait === 'CAREFUL' ? 'warn' : gait ? 'good' : ''} />
        <Tile label="Objects tracked" value={d.objects.length}
              note={`${counts.STEP_OVER} step over · ${counts.CAUTION} careful · ${counts.AVOID} must avoid`} />
        <Tile label="Step-over" value={d.stepOver ? d.stepOver.mode : 'off'}
              note={d.stepOver ? `${d.stepOver.state}${d.stepOver.object ? `: ${d.stepOver.object}` : ''}` : 'not running in this session'} />
        <Tile label="Step limit" value={`${(d.limits.step * 100).toFixed(0)} cm`}
              note={`must avoid from ${d.limits.lethal * 100} cm · heights ±${d.heightErrorCm} cm (median)`} />
      </div>
      <div className="hub-grid">
        <ObjectsCard d={d} />
        <div className="hub-side">
          <Timeline events={d.events} />
          <ProtectedCard d={d} />
        </div>
      </div>
    </>
  );
}

function Tile({ label, value, note, tone = '' }) {
  return (
    <div className={`dx-tile glass ${tone}`}>
      <span className="muted small">{label}</span>
      <strong>{value}</strong>
      <span className="muted small">{note}</span>
    </div>
  );
}

function ObjectsCard({ d }) {
  const [open, setOpen] = useState(null);
  const rows = [...d.objects].sort((a, b) => (a.x ?? 0) - (b.x ?? 0));
  return (
    <section className="hub-card glass">
      <div className="hub-card-head">
        <h2>What the robot sees and decides</h2>
        <span className="muted small">click a row for the reason</span>
      </div>
      {rows.length === 0 ? <span className="muted small">No objects tracked yet.</span> : (
        <div className="table-scroll">
          <table className="hub-table dx-table">
            <thead>
              <tr><th>Object</th><th>Measured height</th><th className="dx-col-vlm">VLM said</th><th>Decision</th></tr>
            </thead>
            <tbody>
              {rows.map((o) => {
                const cut = nearCutoff(o.height, d.limits);
                const isOpen = open === o.track_id;
                return [
                  <tr key={o.track_id} className={`dx-row ${isOpen ? 'open' : ''}`} onClick={() => setOpen(isOpen ? null : o.track_id)}>
                    <td>
                      <span className="dx-obj">{o.class}</span>
                      {o.protected && <span className="dx-tag" title="On the operator's protected list">protected</span>}
                    </td>
                    <td>
                      {o.height == null ? <span className="muted">not measured</span> : `${(o.height * 100).toFixed(1)} cm`}
                      {cut != null && <span className="dx-tag warn" title="Within 1.5 cm of a cutoff: the measurement error could put it on either side">near {cut * 100} cm cutoff</span>}
                    </td>
                    <td className="dx-col-vlm">{o.vlm ? <Band label={o.vlm} /> : <span className="muted">{o.protected ? 'not asked' : '—'}</span>}</td>
                    <td><Band label={o.label} /></td>
                  </tr>,
                  isOpen && (
                    <tr key={`${o.track_id}-why`} className="dx-why"><td colSpan={4}>{explain(o, d.limits)}</td></tr>
                  ),
                ];
              })}
            </tbody>
          </table>
        </div>
      )}
      <div className="dx-legend small muted">
        {[BANDS.stepOver, BANDS.careful, BANDS.avoid].map((b) => (
          <span key={b.key}><span className={`dx-band ${b.tone}`}>{b.name}</span> {b.plain}</span>
        ))}
      </div>
    </section>
  );
}

const KIND_NAME = { seen: 'Object', label: 'Decision', gait: 'Gait', step: 'Step-over' };

function Timeline({ events }) {
  const [filter, setFilter] = useState('all');
  const shown = events.filter((e) => filter === 'all' || e.kind === filter || (filter === 'seen' && e.kind === 'label'));
  const save = (fmt) => {
    const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    if (fmt === 'json') {
      download(`decisions-${stamp}.json`, JSON.stringify(events, null, 2), 'application/json');
      return;
    }
    const esc = (v) => `"${String(v ?? '').replace(/"/g, '""')}"`;
    const csv = ['time,kind,level,decision,why',
      ...events.map((e) => [new Date(e.t * 1000).toISOString(), e.kind, e.level, e.text, e.why].map(esc).join(','))].join('\n');
    download(`decisions-${stamp}.csv`, csv, 'text/csv');
  };
  return (
    <section className="hub-card glass">
      <div className="hub-card-head">
        <h2>Timeline</h2>
        <span className="dx-actions">
          <button className="chip small" onClick={() => save('csv')} disabled={!events.length}>CSV</button>
          <button className="chip small" onClick={() => save('json')} disabled={!events.length}>JSON</button>
        </span>
      </div>
      <div className="chips">
        {[['all', 'All'], ['seen', 'Objects'], ['gait', 'Gait'], ['step', 'Step-over']].map(([id, label]) => (
          <button key={id} className={`chip small ${filter === id ? 'active' : ''}`} onClick={() => setFilter(id)}>{label}</button>
        ))}
      </div>
      <ol className="dx-events">
        {shown.length === 0 && <li className="muted small">Nothing yet.</li>}
        {shown.slice(0, 60).map((e, i) => (
          <li key={`${e.t}-${i}`} className={`dx-event ${e.level}`}>
            <span className="dx-time mono">{new Date(e.t * 1000).toLocaleTimeString()}</span>
            <span className="dx-kind">{KIND_NAME[e.kind] || e.kind}</span>
            <span className="dx-text">{e.text}</span>
            {e.why && <span className="dx-sub muted small">{e.why}</span>}
          </li>
        ))}
      </ol>
    </section>
  );
}

function ProtectedCard({ d }) {
  const [input, setInput] = useState('');
  const [msg, setMsg] = useState(null);
  const list = d.protectedList || [];
  const apply = (next) => {
    const r = d.setProtected(next);
    setMsg(r === 'sent' ? 'Applied to this session.' : r === 'sample' ? 'Sample only: not connected.' : 'Not connected to the robot.');
  };
  const add = () => {
    const v = input.trim().toLowerCase().replace(/_/g, ' ');
    if (!v || list.includes(v)) return;
    apply([...list, v]);
    setInput('');
  };
  return (
    <section className="hub-card glass">
      <div className="hub-card-head">
        <h2>Protected object types</h2>
        <span className="muted small">the operator decides</span>
      </div>
      <span className="muted small">
        Detected objects of these types are never stepped on: must avoid once measured, careful before. The model can
        add caution, never remove it.
      </span>
      <div className="chips">
        {list.length === 0 && <span className="muted small">None: only height and the model decide.</span>}
        {list.map((c) => (
          <span key={c} className="dx-pchip">
            {c}
            <button aria-label={`Remove ${c}`} onClick={() => apply(list.filter((x) => x !== c))}>{'×'}</button>
          </span>
        ))}
      </div>
      <form className="dx-add" onSubmit={(e) => { e.preventDefault(); add(); }}>
        <input value={input} onChange={(e) => setInput(e.target.value)} placeholder="Add a type, e.g. backpack" maxLength={40} />
        <button className="chip small go" type="submit" disabled={!input.trim()}>Add</button>
      </form>
      {msg && <span className="muted small">{msg} The default list is in scripts/config/protected_classes.yaml.</span>}
    </section>
  );
}

// --- How it decides ---------------------------------------------------------------------

function How() {
  return (
    <div className="dx-cols">
      {HOW.map((s) => (
        <section key={s.id} className={`hub-card glass dx-read ${s.id === 'example' ? 'wide' : ''}`}>
          <h2>{s.title}</h2>
          {s.body?.map((p, i) => <p key={i}>{p}</p>)}
          {s.terms && (
            <dl className="dx-terms">
              {s.terms.map(([t, def]) => [<dt key={t}>{t}</dt>, <dd key={`${t}-d`}>{def}</dd>])}
            </dl>
          )}
          {s.steps && (
            <ol className="dx-steps">
              {s.steps.map(([t, txt], i) => (
                <li key={t}><span className="dx-step-n">{i + 1}</span><div><strong>{t}</strong><p>{txt}</p></div></li>
              ))}
            </ol>
          )}
        </section>
      ))}
    </div>
  );
}

// --- Results ------------------------------------------------------------------------------

function Bars({ data }) {
  return (
    <div className="dx-bars" role="img" aria-label={data.map((r) => `${r.label} ${Math.round((100 * r.value) / r.of)} percent`).join(', ')}>
      {data.map((r) => {
        const pct = (100 * r.value) / r.of;
        return (
          <div key={r.label} className="dx-bar-row">
            <span className="dx-bar-label">{r.label}</span>
            <span className="dx-bar-track"><span className="dx-bar" style={{ width: `${pct}%` }} /></span>
            <span className="dx-bar-val"><strong>{Math.round(pct)}%</strong> <span className="muted small">{r.value}/{r.of}</span></span>
          </div>
        );
      })}
      <div className="dx-axis small muted"><span>0%</span><span>50%</span><span>100%</span></div>
    </div>
  );
}

function Results() {
  return (
    <div className="dx-cols">
      {RESULTS.map((r) => (
        <section key={r.id} className={`hub-card glass dx-read ${r.chart || r.pair ? 'wide' : ''}`}>
          <div className="hub-card-head">
            <h2>{r.q}</h2>
            <span className={`dx-tag ${r.real ? 'good' : ''}`}>{r.real ? 'Real robot' : 'Simulation, n = 10 per mode'}</span>
          </div>
          <p className="dx-headline">{r.headline}</p>
          {r.chart && <Bars data={r.chart} />}
          {r.pair && (
            <table className="hub-table">
              <thead><tr><th>Object</th><th>Measured</th><th>Model alone (trials)</th><th>With the protected list</th></tr></thead>
              <tbody>{r.pair.map((p) => <tr key={p.name}><td>{p.name}</td><td>{p.height}</td><td>{p.model}</td><td>{p.now}</td></tr>)}</tbody>
            </table>
          )}
          {r.detail.map((p, i) => <p key={i} className="small">{p}</p>)}
        </section>
      ))}
    </div>
  );
}
