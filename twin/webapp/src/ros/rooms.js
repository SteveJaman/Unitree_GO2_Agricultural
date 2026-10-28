// Room map from /semantic_rooms/json (live) or config/semantic_rooms.json
// (saved file, used offline, e.g. wifi_3d_nav.sh --view).

const PALETTE = [
  [0.35, 0.65, 1.0], [0.4, 0.85, 0.55], [1.0, 0.7, 0.3], [0.85, 0.45, 0.95],
  [1.0, 0.45, 0.5], [0.3, 0.85, 0.85], [0.95, 0.85, 0.35], [0.6, 0.6, 1.0],
];
const num = (id) => Number(String(id).split('_').pop()) || 0;

/** -> { frame, rooms: [{ id, label, polygon, center, area, color, merged_from }], error } */
export function normalizeRooms(data) {
  if (!data) return { frame: 'lidar3d_map', rooms: [], error: null };
  const merges = data.merges || {};
  const list = Array.isArray(data.rooms) ? data.rooms : Object.values(data.rooms || {});
  const rooms = list
    .filter((r) => r && r.polygon && r.polygon.length >= 3 && !merges[r.id] && r.seen !== false)
    .map((r) => ({
      ...r,
      color: r.color || PALETTE[(num(r.id) - 1 + PALETTE.length) % PALETTE.length],
      merged_from: r.merged_from || Object.keys(merges).filter((k) => merges[k] === r.id),
    }))
    .sort((a, b) => num(a.id) - num(b.id));
  return { frame: data.frame || 'lidar3d_map', rooms, error: data.error || null };
}

export const roomName = (r) => (r?.label ? r.label : `Room ${num(r?.id)}`);

export const cssColor = (c) => `rgb(${c.map((v) => Math.round(v * 255)).join(',')})`;
