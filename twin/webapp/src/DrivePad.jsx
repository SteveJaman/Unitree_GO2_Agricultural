import { useCallback, useEffect, useRef, useState } from 'react';
import { useLatest, usePublisher } from './RosProvider.jsx';

const SEND_MS = 100;   // joystick rate while held (go2_teleop.py stops after 0.35 s of silence)
const KEYS = {         // key -> [forward, left, turn-left]
  w: [1, 0, 0], arrowup: [1, 0, 0], s: [-1, 0, 0], arrowdown: [-1, 0, 0],
  a: [0, 0, 1], arrowleft: [0, 0, 1], d: [0, 0, -1], arrowright: [0, 0, -1],
  q: [0, 1, 0], e: [0, -1, 0],
};

/**
 * Manual drive (go2_teleop.py): take control from Nav2, then a joystick
 * (forward/back + turn), hold-to-strafe buttons, STOP, and WASD / arrows
 * (Q/E strafe, Space stop). The robot walks at Nav2's own gentle limits, so
 * its localization keeps tracking. Letting go stops it; closing the pad
 * hands it back to Nav2.
 */
export default function DrivePad({ connected }) {
  const [open, setOpen] = useState(false);
  const [status] = useLatest('/teleop/status', 'std_msgs/msg/String', { throttle: 250 });
  const sendCmd = usePublisher('/teleop/command', 'std_msgs/msg/String');
  const sendJoy = usePublisher('/teleop/joy', 'geometry_msgs/msg/Twist');
  let st = null;
  try { st = status ? JSON.parse(status.data) : null; } catch { st = null; }
  const active = !!st?.active;
  const ready = connected && !!st;

  // The stick (pointer) and the keyboard each hold an axis set; the sum is sent.
  const stick = useRef([0, 0, 0]);
  const strafe = useRef(0);
  const keys = useRef(new Set());
  const timer = useRef(null);
  const [knob, setKnob] = useState([0, 0]);

  const current = () => {
    let [x, y, w] = stick.current;
    y += strafe.current;
    for (const k of keys.current) { const v = KEYS[k]; x += v[0]; y += v[1]; w += v[2]; }
    const c = (v) => Math.max(-1, Math.min(1, v));
    return [c(x), c(y), c(w)];
  };
  const publish = useCallback(() => {
    const [x, y, w] = current();
    sendJoy({ linear: { x, y, z: 0 }, angular: { x: 0, y: 0, z: w } });
    return x || y || w;
  }, [sendJoy]);   // eslint-disable-line react-hooks/exhaustive-deps
  // Send while any input is held; one last zero when everything is let go.
  const pump = useCallback(() => {
    if (timer.current) return;
    publish();
    timer.current = setInterval(() => { if (!publish()) { clearInterval(timer.current); timer.current = null; } }, SEND_MS);
  }, [publish]);
  const halt = useCallback(() => {
    stick.current = [0, 0, 0]; strafe.current = 0; keys.current.clear(); setKnob([0, 0]);
    clearInterval(timer.current); timer.current = null;
    publish();
  }, [publish]);

  const take = () => sendCmd({ data: JSON.stringify({ op: 'take' }) });
  const release = useCallback(() => { halt(); sendCmd({ data: JSON.stringify({ op: 'release' }) }); }, [halt, sendCmd]);
  const stop = () => { halt(); sendCmd({ data: JSON.stringify({ op: 'stop' }) }); };

  // Keyboard, only while the pad is open and in control (not while typing).
  useEffect(() => {
    if (!open || !active) return undefined;
    const typing = (e) => /input|textarea|select/i.test(e.target?.tagName || '');
    const down = (e) => {
      if (typing(e)) return;
      const k = e.key.toLowerCase();
      if (k === ' ') { e.preventDefault(); stop(); return; }
      if (!KEYS[k]) return;
      e.preventDefault();
      keys.current.add(k);
      pump();
    };
    const up = (e) => { keys.current.delete(e.key.toLowerCase()); };
    const blur = () => halt();
    window.addEventListener('keydown', down);
    window.addEventListener('keyup', up);
    window.addEventListener('blur', blur);
    document.addEventListener('visibilitychange', blur);
    return () => {
      window.removeEventListener('keydown', down);
      window.removeEventListener('keyup', up);
      window.removeEventListener('blur', blur);
      document.removeEventListener('visibilitychange', blur);
    };
  }, [open, active, pump, halt]);   // eslint-disable-line react-hooks/exhaustive-deps

  // Never leave the robot in manual control behind a closed pad.
  useEffect(() => () => { clearInterval(timer.current); }, []);
  const close = () => { if (active) release(); setOpen(false); };

  // Joystick: up = forward, left = turn left.
  const pad = useRef(null);
  const onStick = (e) => {
    if (!active || (e.type === 'pointermove' && !e.currentTarget.hasPointerCapture(e.pointerId))) return;
    if (e.type === 'pointerdown') e.currentTarget.setPointerCapture(e.pointerId);
    const r = pad.current.getBoundingClientRect();
    const rad = r.width / 2;
    let dx = (e.clientX - r.left - rad) / rad;
    let dy = (e.clientY - r.top - rad) / rad;
    const len = Math.hypot(dx, dy);
    if (len > 1) { dx /= len; dy /= len; }
    const dead = (v) => (Math.abs(v) < 0.12 ? 0 : v);
    stick.current = [dead(-dy), 0, dead(-dx)];
    setKnob([dx, dy]);
    pump();
  };
  const endStick = () => { stick.current = [0, 0, 0]; setKnob([0, 0]); };
  const hold = (dir) => ({
    onPointerDown: (e) => { if (!active) return; e.currentTarget.setPointerCapture(e.pointerId); strafe.current = dir; pump(); },
    onPointerUp: () => { strafe.current = 0; },
    onPointerCancel: () => { strafe.current = 0; },
  });

  if (!open) {
    return (
      <button className="drive-fab glass" disabled={!ready} onClick={() => setOpen(true)}
              title={!connected ? 'Robot offline' : !st ? 'Web drive not running (go2_teleop.py)' : 'Drive the robot by hand'}>
        <span className="action-icon">✥</span> Drive
      </button>
    );
  }
  const lim = st?.limits;
  return (
    <div className={`drive glass ${active ? 'on' : ''}`}>
      <div className="drive-head">
        <span className="drive-title">Manual drive</span>
        <button className="chip small" onClick={close} title="Hand the robot back to Nav2 and close">✕</button>
      </div>
      <div className="drive-sub">
        {!ready ? 'Robot offline' : st.note}
        {lim && <> · {lim.vx.toFixed(2)} m/s, {lim.wz.toFixed(2)} rad/s max</>}
      </div>
      {!active ? (
        <button className="action primary" disabled={!ready} onClick={take}
                title="Pauses Auto Explore and cancels the current Nav2 goal">
          Take control
        </button>
      ) : (
        <>
          <div className="drive-body">
            <button className="drive-strafe" aria-label="Sidestep left (Q)" {...hold(1)}>◀</button>
            <div ref={pad} className="drive-stick" onPointerDown={onStick} onPointerMove={onStick}
                 onPointerUp={endStick} onPointerCancel={endStick}>
              <div className="drive-knob" style={{ transform: `translate(${knob[0] * 50}%, ${knob[1] * 50}%)` }} />
            </div>
            <button className="drive-strafe" aria-label="Sidestep right (E)" {...hold(-1)}>▶</button>
          </div>
          <button className="drive-stop" onClick={stop}>STOP</button>
          <div className="drive-row">
            <span className="drive-keys">W A S D · Q E sidestep · Space stop</span>
            <button className="chip small" onClick={release} title="Stop and hand the robot back to Nav2">Release</button>
          </div>
        </>
      )}
    </div>
  );
}
