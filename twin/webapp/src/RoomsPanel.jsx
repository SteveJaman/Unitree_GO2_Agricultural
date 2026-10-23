import { cssColor, roomName } from './ros/rooms.js';

/**
 * Rooms list (top right, under the status): name, area, Go; Edit mode lets
 * the map's rooms be clicked to name them, and Merge joins two adjacent rooms.
 */
export default function RoomsPanel({
  rooms, connected, open, setOpen, editing, setEditing, merging, setMerging,
  selected, onRename, onGo, onMerge, onCancelMerge,
}) {
  if (!open) {
    return (
      <button className="rooms-fab glass" onClick={() => setOpen(true)} title="Rooms">
        ▦ Rooms{rooms.length ? <span className="count">{rooms.length}</span> : null}
      </button>
    );
  }
  const pick = merging ? selected.map((id) => rooms.find((r) => r.id === id)).filter(Boolean) : [];
  return (
    <div className="rooms glass">
      <div className="rooms-head">
        <span className="rooms-title">Rooms</span>
        <button className={`chip small ${editing ? 'active' : ''}`} disabled={!connected}
                onClick={() => { setEditing(!editing); setMerging(false); }}
                title="Click rooms on the map to name them">
          {editing ? 'Done' : 'Edit'}
        </button>
        <button className="icon" onClick={() => { setOpen(false); setEditing(false); setMerging(false); }} title="Close">✕</button>
      </div>

      {rooms.length === 0 && (
        <div className="rooms-empty">No rooms yet. They appear as the robot maps the area.</div>
      )}

      <div className="rooms-list">
        {rooms.map((r) => (
          <div key={r.id} className={`room-row ${selected.includes(r.id) ? 'selected' : ''}`}>
            <span className="swatch" style={{ background: cssColor(r.color) }} />
            <span className="room-name">{roomName(r)}</span>
            <span className="room-area">{Math.round(r.area)} m²</span>
            {editing
              ? <button className="chip small" onClick={() => onRename(r)} disabled={!connected}>Name</button>
              : <button className="chip small go" onClick={() => onGo(r)} disabled={!connected}>Go</button>}
          </div>
        ))}
      </div>

      {editing && (
        <div className="rooms-foot">
          {!merging && (
            <>
              <span className="hint-text">Click a room on the map to name it.</span>
              <button className="chip small" disabled={rooms.length < 2} onClick={() => setMerging(true)}>⧉ Merge rooms</button>
            </>
          )}
          {merging && (
            <>
              <span className="hint-text">
                {pick.length < 2
                  ? `Select two adjacent rooms on the map (${pick.length}/2)`
                  : `Merge ${roomName(pick[0])} + ${roomName(pick[1])}?`}
              </span>
              <div className="row-buttons">
                <button className="chip small" onClick={onCancelMerge}>Cancel</button>
                <button className="chip small active" disabled={pick.length < 2} onClick={onMerge}>Merge</button>
              </div>
            </>
          )}
        </div>
      )}
    </div>
  );
}
