import { useEffect, useState } from 'react';
import { api, setCode, useHub } from './hub.js';

const PRESET_TEXT = { safe: 'default, open rooms', wide: 'crowded spaces', tight: 'narrow rows, doorways' };

const fmt = (v, step) => (v === null || v === undefined ? '—' : Number(v).toFixed(step < 0.1 ? 2 : 1));
const same = (a, b) => a !== null && a !== undefined && b !== null && b !== undefined && Math.abs(a - b) < 1e-6;

/**
 * Settings tab: Nav2 values changed live through go2_hub.py (/api/settings).
 * Read-only until the session code (printed by wifi_3d_nav.sh) is entered.
 */
export default function SettingsView() {
  const [data, err, , put] = useHub('/api/settings', 0);
  const [auth, setAuth] = useState(null);
  const [codeInput, setCodeInput] = useState('');
  const [draft, setDraft] = useState({});
  const [save, setSave] = useState(false);
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState(null);
  const [confirmTight, setConfirmTight] = useState(false);

  const checkAuth = () => api('/api/auth').then(setAuth).catch(() => setAuth(null));
  useEffect(() => { checkAuth(); }, []);
  useEffect(() => { if (data?.values) setDraft(data.values); }, [data]);

  if (err?.status === 404) {
    return (
      <div className="hub-page">
        <div className="hub-empty glass">Settings need the dashboard hub, which <code>./scripts/runtime/wifi_3d_nav.sh</code> starts.</div>
      </div>
    );
  }
  const locked = !auth || (auth.code_required && !auth.ok);
  const live = data?.values || {};
  const defs = data?.defs || [];
  const pending = defs.filter((d) => draft[d.key] !== undefined && !same(draft[d.key], live[d.key]));

  const unlock = async () => {
    setCode(codeInput);
    const a = await api('/api/auth').catch(() => null);
    setAuth(a);
    setMsg(a?.ok ? null : { tone: 'bad', text: 'Wrong code. The terminal shows it next to the dashboard address.' });
  };
  const send = async (path, body) => {
    setBusy(true); setMsg(null);
    try {
      const out = await api(path, { method: 'POST', body });
      const failed = Object.keys(out.set_errors || {});
      setMsg(failed.length
        ? { tone: 'warn', text: `Applied, except: ${failed.map((k) => k.split('|')[1]).join(', ')} (not available in this drive mode?)` }
        : { tone: 'ok', text: 'Applied. The values below are what Nav2 reports now.' });
      put(out);   // the answer already holds Nav2's values after the change
    } catch (e) {
      setMsg({ tone: 'bad', text: e.status === 403 ? 'The session code is needed to change settings.' : e.message });
      if (e.status === 403) { setCode(''); checkAuth(); }
    }
    setBusy(false);
  };
  const preset = (name) => {
    const p = data.presets[name];
    setDraft({ ...draft, ...p });
  };
  const activePreset = data?.presets && Object.entries(data.presets)
    .find(([, p]) => Object.entries(p).every(([k, v]) => same(draft[k], v)))?.[0];

  return (
    <div className="hub-page">
      <div className="hub-page-head">
        <h1 className="hub-title">Settings</h1>
        <span className={`lock-pill ${locked ? 'locked' : 'open'}`}>
          <span className="dot" /> {locked ? 'Read-only' : 'Changes allowed'}
        </span>
      </div>

      {locked && (
        <div className="lock-bar glass">
          <span>Settings are read-only. Enter the session code printed in the terminal at start to change them.</span>
          <label className="code-label">Code
            <input value={codeInput} onChange={(e) => setCodeInput(e.target.value.replace(/\D/g, '').slice(0, 8))}
                   onKeyDown={(e) => e.key === 'Enter' && unlock()} inputMode="numeric" aria-label="Session code" />
          </label>
          <button className="primary" onClick={unlock} disabled={!codeInput}>Unlock</button>
        </div>
      )}
      {!data && !err && <div className="lock-bar glass" role="status">Reading the values from Nav2…</div>}
      {err && err.status !== 404 && <div className="lock-bar glass bad">Could not reach the hub: {err.message}</div>}
      {data?.error && <div className="lock-bar glass warn">{data.error}: the values appear once Nav2 is running.</div>}
      {msg && <div className={`lock-bar glass ${msg.tone}`} role="status">{msg.text}</div>}

      <div className="hub-grid">
        <section className="hub-card glass">
          <div className="hub-card-head">
            <h2>Collision threshold</h2>
            <span className="muted small">How far Nav2 keeps the dog from walls and obstacles. Applied live, no restart.</span>
          </div>
          <div className="presets">
            {Object.entries(data?.presets || {}).map(([name, p]) => (
              <button key={name} className={`preset ${activePreset === name ? 'on' : ''}`} disabled={locked}
                      aria-pressed={activePreset === name}
                      onClick={() => (name === 'tight' ? setConfirmTight(true) : preset(name))}>
                <span className="preset-name">{name[0].toUpperCase() + name.slice(1)}</span>
                <span className="muted small">{p.inflation.toFixed(2)} m · {p.padding.toFixed(2)} m</span>
                <span className="muted small">{PRESET_TEXT[name]}</span>
              </button>
            ))}
          </div>
          {defs.map((d) => {
            const v = draft[d.key];
            const changed = !same(v, live[d.key]);
            return (
              <div key={d.key} className="setting">
                <div className="setting-head">
                  <label htmlFor={`set-${d.key}`}>{d.name} <span className="mono muted small">{[...new Set(d.params)].join(' · ')}{d.params.length > 1 && new Set(d.params).size === 1 ? ' (both costmaps)' : ''}</span></label>
                  <span className="setting-value">{fmt(v, d.step)} {d.unit}</span>
                </div>
                <input id={`set-${d.key}`} type="range" min={d.min} max={d.max} step={d.step}
                       value={v ?? d.min} disabled={locked || v === null || v === undefined}
                       onChange={(e) => setDraft({ ...draft, [d.key]: parseFloat(e.target.value) })} />
                <div className="setting-foot muted small">
                  <span>{d.help} ({fmt(d.min, d.step)}–{fmt(d.max, d.step)} {d.unit})</span>
                  <span className={changed ? 'pending' : 'applied'}>
                    {live[d.key] === null || live[d.key] === undefined ? 'not available'
                      : changed ? `not applied yet (Nav2 now ${fmt(live[d.key], d.step)})` : `Nav2 now ${fmt(live[d.key], d.step)} ✓`}
                  </span>
                </div>
              </div>
            );
          })}
        </section>

        <section className="hub-card glass hub-side">
          <h2>Apply</h2>
          <span className="muted small">
            {pending.length ? `${pending.length} change(s) waiting: ${pending.map((d) => d.name).join(', ')}.` : 'Nav2 is using the values shown.'}
          </span>
          <button className="primary" disabled={locked || busy || !pending.length}
                  onClick={() => send('/api/settings', { values: Object.fromEntries(pending.map((d) => [d.key, draft[d.key]])), save })}>
            {busy ? 'Applying…' : 'Apply to Nav2 now'}
          </button>
          <button className="chip" disabled={locked || busy || !data?.defaults}
                  onClick={() => send('/api/settings/reset', { save })}>Reset to session defaults</button>
          <label className="check-label">
            <input type="checkbox" checked={save} disabled={locked} onChange={(e) => setSave(e.target.checked)} />
            Also save as my defaults (config/dashboard_settings.json, applied at the next start)
          </label>
          {data?.defaults && (
            <span className="muted small">Session started with: {defs.map((d) => `${d.name.toLowerCase()} ${fmt(data.defaults[d.key], d.step)}`).join(' · ')}</span>
          )}
          <div className="history">
            <span className="muted small history-title">Recent changes</span>
            {(data?.history || []).length === 0 && <span className="muted small">none this session</span>}
            {(data?.history || []).map((h, i) => <span key={i} className="small">{h}</span>)}
          </div>
        </section>
      </div>

      {confirmTight && (
        <div className="modal-backdrop" onClick={() => setConfirmTight(false)}>
          <div className="modal glass" role="dialog" aria-modal="true" aria-labelledby="tight-title" onClick={(e) => e.stopPropagation()}>
            <h2 id="tight-title">Use the Tight threshold?</h2>
            <p>0.45 m inflation lets the dog pass close to walls and furniture. Use it only for narrow spaces, with the remote in hand.</p>
            <div className="modal-actions">
              <button className="chip" onClick={() => setConfirmTight(false)}>Cancel</button>
              <button className="primary warn" onClick={() => { preset('tight'); setConfirmTight(false); }}>Use Tight</button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
