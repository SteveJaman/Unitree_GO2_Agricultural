// Render quality. A VM without 3D acceleration (VirtualBox, llvmpipe,
// SwiftShader) draws every pixel and triangle on the CPU, where the full
// look (2x resolution, antialiasing, 80k lit cubes, live shadows, blurred
// glass panels) runs at a few fps. 'lite' trades those for frame rate.
//
// Chosen by ?quality=lite|high, else the last choice in the settings menu,
// else 'lite' when the browser reports a software renderer.

const KEY = 'go2-twin-quality';

/** The WebGL renderer string, and whether it is a software (CPU) renderer. */
export function detectRenderer() {
  try {
    const c = document.createElement('canvas');
    const gl = c.getContext('webgl2') || c.getContext('webgl');
    if (!gl) return { renderer: 'none', software: true };
    const ext = gl.getExtension('WEBGL_debug_renderer_info');
    const renderer = String(ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER));
    gl.getExtension('WEBGL_lose_context')?.loseContext();
    // Firefox: "llvmpipe, or similar"; Chrome in VirtualBox: "SVGA3D; build: RELEASE; LLVM;".
    return { renderer, software: /llvmpipe|softpipe|swiftshader|software|llvm|basic render/i.test(renderer) };
  } catch {
    return { renderer: 'unknown', software: false };
  }
}

export function initialQuality() {
  const q = new URLSearchParams(window.location.search).get('quality');
  if (q === 'lite' || q === 'high') return q;
  try {
    const saved = window.localStorage.getItem(KEY);
    if (saved === 'lite' || saved === 'high') return saved;
  } catch { /* storage blocked */ }
  return detectRenderer().software ? 'lite' : 'high';
}

export function saveQuality(q) {
  try { window.localStorage.setItem(KEY, q); } catch { /* storage blocked */ }
}

/** Per-quality render settings used by Scene. */
export const QUALITY = {
  high: { dpr: [1, 2], antialias: true, voxelBudget: 80000, shadowMapSize: 2048, material: 'standard' },
  lite: { dpr: 1, antialias: false, voxelBudget: 25000, shadowMapSize: 1024, material: 'lambert' },
};
