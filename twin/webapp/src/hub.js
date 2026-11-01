// The dashboard's own backend, go2_hub.py (started by wifi_3d_nav.sh): the
// System and Settings tabs. Same origin as the page. Served by go2-3d alone
// (--no-hub, --view) there is no /api: the tabs say so.
import { useEffect, useState } from 'react';

const CODE_KEY = 'go2-hub-code';

/** Route and session code from the address: #/system, #/settings, #code=4821. */
export function parseHash(hash = window.location.hash) {
  const h = hash.replace(/^#/, '');
  const code = h.match(/(?:^|[?&/])code=(\d{4,8})/)?.[1] || null;
  const route = h.match(/^\/(system(?:\/logs)?|settings)/)?.[1] || 'map';
  return { route, code };
}

export function getCode() {
  try { return sessionStorage.getItem(CODE_KEY) || ''; } catch { return ''; }
}

export function setCode(code) {
  try { if (code) sessionStorage.setItem(CODE_KEY, code); else sessionStorage.removeItem(CODE_KEY); } catch { /* private window */ }
}

export function go(route) {
  window.location.hash = route === 'map' ? '' : `/${route}`;
}

export async function api(path, { method = 'GET', body } = {}) {
  const res = await fetch(path, {
    method,
    headers: { 'Content-Type': 'application/json', 'X-Go2-Code': getCode() },
    body: body === undefined ? undefined : JSON.stringify(body),
    cache: 'no-store',
  });
  const type = res.headers.get('Content-Type') || '';
  const data = type.includes('json') ? await res.json() : await res.text();
  if (!res.ok) {
    const err = new Error((data && data.error) || `${res.status} ${res.statusText}`);
    err.status = res.status;
    throw err;
  }
  return data;
}

/**
 * Poll a hub endpoint every `ms` while `on`. Returns [data, error, refresh];
 * error.status 404 (no /api) means the page is not served by the hub.
 */
export function useHub(path, ms, on = true) {
  const [data, setData] = useState(null);
  const [error, setError] = useState(null);
  const [n, setN] = useState(0);
  useEffect(() => {
    if (!on || !path) return undefined;
    let alive = true;
    let timer = null;
    const load = async () => {
      try {
        const d = await api(path);
        if (alive) { setData(d); setError(null); }
      } catch (e) {
        if (alive) setError(e);
      }
      if (alive && ms) timer = setTimeout(load, ms);
    };
    load();
    return () => { alive = false; clearTimeout(timer); };
  }, [path, ms, on, n]);
  // refresh() loads again; put(d) shows data the caller already has (a POST's answer)
  return [data, error, () => setN((x) => x + 1), setData];
}

/** Worst level of the System checks: 'bad' | 'warn' | 'ok' | null (no hub). */
export function worstLevel(checks) {
  if (!checks) return null;
  if (checks.some((c) => c.level === 'bad')) return 'bad';
  if (checks.some((c) => c.level === 'warn')) return 'warn';
  return 'ok';
}

/** Copy text: the clipboard API needs https or localhost; else a hidden textarea. */
export async function copyText(text) {
  try {
    if (navigator.clipboard && window.isSecureContext) {
      await navigator.clipboard.writeText(text);
      return true;
    }
  } catch { /* fall through */ }
  const ta = document.createElement('textarea');
  ta.value = text;
  ta.setAttribute('readonly', '');
  ta.style.position = 'fixed';
  ta.style.opacity = '0';
  document.body.appendChild(ta);
  ta.select();
  let ok = false;
  try { ok = document.execCommand('copy'); } catch { ok = false; }
  document.body.removeChild(ta);
  return ok;
}

export function download(name, text, type = 'text/markdown') {
  const url = URL.createObjectURL(new Blob([text], { type }));
  const a = document.createElement('a');
  a.href = url;
  a.download = name;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
