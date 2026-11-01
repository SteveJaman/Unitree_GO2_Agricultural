import { useEffect, useState } from 'react';
import { go, useHub } from './hub.js';

const MARK = { ok: '✓', warn: '!', fail: '✕', run: '…', wait: '·' };

/**
 * Start-up progress over the map (go2_hub.py /api/boot): the stages
 * wifi_3d_nav.sh reports plus the live session, and every warning with its
 * fix. Goes away once everything is up (or when hidden); a failure stays.
 */
export default function BootOverlay() {
  const [hidden, setHidden] = useState(false);
  const [boot, error] = useHub('/api/boot', 1500, !hidden);
  const [done, setDone] = useState(false);

  useEffect(() => {
    if (!boot?.ready) return undefined;
    const id = setTimeout(() => setDone(true), 2500);
    return () => clearTimeout(id);
  }, [boot?.ready]);

  if (hidden || done || error || !boot) return null;   // no hub: nothing to show
  const elapsed = boot.started ? Math.max(0, Math.round(Date.now() / 1000 - boot.started)) : null;

  return (
    <div className="boot-overlay">
      <div className="boot-card glass" role="status" aria-live="polite">
        <div className="boot-head">
          <h2>{boot.failed ? 'Start-up stopped' : boot.ready ? 'Everything is up' : 'Starting the robot stack'}</h2>
          {elapsed !== null && <span className="muted">{elapsed} s</span>}
        </div>
        {boot.stages.map((s) => (
          <div key={s.key} className={`boot-step ${s.state}`}>
            <span className="boot-mark">{MARK[s.state] || '·'}</span>
            <span className="boot-text">
              <span className="boot-name">{s.name}</span>
              <span className="muted small">{s.detail || (s.state === 'wait' ? 'waiting' : s.state === 'run' ? 'in progress' : '')}</span>
            </span>
          </div>
        ))}
        {boot.warnings.length > 0 && (
          <div className="boot-warn">
            <span className="boot-warn-title">{boot.warnings.length} warning{boot.warnings.length > 1 ? 's' : ''}</span>
            {boot.warnings.slice(-4).map((w, i) => (
              <div key={i} className="boot-warn-item">
                <span>{w.text}</span>
                {w.more.map((m, j) => <code key={j}>{m}</code>)}
              </div>
            ))}
          </div>
        )}
        <div className="boot-foot">
          <button className="link" onClick={() => go('system')}>Open System for details</button>
          <button className="chip" onClick={() => setHidden(true)}>Hide</button>
        </div>
      </div>
    </div>
  );
}
