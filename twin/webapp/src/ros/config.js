// config/*.json written by the session (semantic_locations.json after the
// first --save-location, semantic_rooms.json after the first segmentation).

/**
 * The parsed file, or null while it does not exist yet. The folder listing
 * (python http.server) is checked first: polling a missing file directly
 * logs a 404 in the browser console every time.
 */
export async function fetchConfigJson(assets, name) {
  const dir = await fetch(`${assets}config/`, { cache: 'no-store' });
  if (!dir.ok || !(await dir.text()).includes(`href="${name}"`)) return null;
  const r = await fetch(`${assets}config/${name}`, { cache: 'no-store' });
  return r.ok ? r.json() : null;
}
