import { useEffect, useRef, useState } from 'react';
import { cssColor, roomName } from './ros/rooms.js';

const IDEAS = ['Hydroponic Station', 'Lab Desk', 'Greenhouse', 'Storage', 'Office', 'Hallway'];

/**
 * "Name this area": the VLM's suggestion (if any) becomes the placeholder;
 * Tab or the suggestion chip takes it. Enter saves, Escape cancels.
 */
export default function RoomNameModal({ room, suggestion, asking, onAsk, onSave, onClose }) {
  const [value, setValue] = useState(room.label || '');
  const input = useRef();
  useEffect(() => { input.current?.focus(); }, []);

  const placeholder = suggestion || (asking ? 'Asking the AI…' : 'e.g. Lab Desk');
  const save = () => {
    const name = (value || suggestion || '').trim();
    if (name) onSave(name);
  };
  return (
    <div className="modal-backdrop" onMouseDown={(e) => e.target === e.currentTarget && onClose()}>
      <div className="modal glass" role="dialog" aria-label="Name this area">
        <div className="modal-head">
          <span className="swatch big" style={{ background: cssColor(room.color) }} />
          <div>
            <div className="modal-title">Name this area</div>
            <div className="modal-sub">{roomName(room)} · {Math.round(room.area)} m²</div>
          </div>
        </div>
        <input ref={input} className="modal-input" value={value} placeholder={placeholder} maxLength={60}
               onChange={(e) => setValue(e.target.value)}
               onKeyDown={(e) => {
                 if (e.key === 'Enter') save();
                 if (e.key === 'Escape') onClose();
                 if (e.key === 'Tab' && !value && suggestion) { e.preventDefault(); setValue(suggestion); }
               }} />
        <div className="modal-ideas">
          {suggestion && (
            <button className="chip small ai" onClick={() => setValue(suggestion)} title="AI suggestion from the camera">
              ✨ {suggestion}
            </button>
          )}
          {!suggestion && (
            <button className="chip small ai" onClick={onAsk} disabled={asking}
                    title="Ask the AI (uses what the camera sees now)">
              {asking ? 'Asking…' : '✨ Suggest'}
            </button>
          )}
          {IDEAS.map((i) => <button key={i} className="chip small" onClick={() => setValue(i)}>{i}</button>)}
        </div>
        <div className="modal-note">AI suggestions use the live camera: best while the robot is in this area.</div>
        <div className="modal-actions">
          <button className="action" onClick={onClose}>Cancel</button>
          <button className="action primary" onClick={save} disabled={!(value || suggestion)}>Save</button>
        </div>
      </div>
    </div>
  );
}
