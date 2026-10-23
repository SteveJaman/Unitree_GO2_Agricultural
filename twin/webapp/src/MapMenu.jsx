import { useState } from 'react';

const VIEWS = [['solid', 'Solid'], ['mesh', '3D Scan'], ['points', 'Points']];
const k = (n) => (n >= 1e6 ? `${(n / 1e6).toFixed(1)}M` : n >= 1e3 ? `${Math.round(n / 1e3)}k` : `${n}`);
const prettyScan = (f) => {
  const m = f.match(/(\d{4})(\d{2})(\d{2})-(\d{2})(\d{2})/);
  return m ? `${m[1]}-${m[2]}-${m[3]} ${m[4]}:${m[5]}` : f.replace(/_mesh\.ply$/, '');
};

/** Top-left: brand, map style, and a settings popover (layers, quality). */
export default function MapMenu({
  view, setView, meshes, meshFile, setMeshFile, voxelSize, setVoxelSize,
  textured, setTextured, shadows, setShadows, layers, setLayers, stats,
}) {
  const [open, setOpen] = useState(false);
  const toggle = (key) => setLayers((l) => ({ ...l, [key]: !l[key] }));
  return (
    <div className="mapmenu">
      <div className="brand glass">
        <span className="brand-mark" />
        <span>Go2 <b>Digital Twin</b></span>
      </div>
      <div className="segmented glass">
        {VIEWS.map(([key, label]) => (
          <button key={key} className={view === key ? 'active' : ''} onClick={() => setView(key)}>{label}</button>
        ))}
        <button className={`gear ${open ? 'active' : ''}`} onClick={() => setOpen(!open)} title="Map settings">⚙</button>
      </div>
      {open && (
        <div className="menu glass">
          {view === 'mesh' && (
            <label className="row">Scan
              <select value={meshFile} onChange={(e) => setMeshFile(e.target.value)}>
                {meshes.length === 0 && <option value="">none yet</option>}
                {meshes.map((m) => <option key={m} value={m}>{prettyScan(m)}</option>)}
              </select>
            </label>
          )}
          {view === 'solid' && (
            <label className="row">Detail
              <select value={voxelSize} onChange={(e) => setVoxelSize(e.target.value)}>
                <option value="auto">Auto</option>
                <option value="0.05">High</option>
                <option value="0.1">Medium</option>
                <option value="0.2">Low</option>
              </select>
            </label>
          )}
          <Toggle label="Real colours" on={textured} set={() => setTextured(!textured)} />
          <Toggle label="Shadows" on={shadows} set={() => setShadows(!shadows)} />
          <div className="menu-sep" />
          <Toggle label="Map" on={layers.map} set={() => toggle('map')} />
          <Toggle label="Live scan" on={layers.scan} set={() => toggle('scan')} />
          <Toggle label="Detected objects" on={layers.yolo} set={() => toggle('yolo')} />
          <Toggle label="Places & labels" on={layers.labels} set={() => toggle('labels')} />
          <div className="menu-foot">
            {stats.triangles ? `${k(stats.triangles)} triangles · ` : ''}{stats.fps ? `${stats.fps} fps` : ''}
          </div>
        </div>
      )}
    </div>
  );
}

function Toggle({ label, on, set }) {
  return (
    <button className="row toggle" onClick={set}>
      {label}
      <span className={`switch ${on ? 'on' : ''}`}><span /></span>
    </button>
  );
}
