// sensor_msgs/PointCloud2 -> typed arrays for THREE.Points.

// Camera colours are sRGB; three.js expects linear vertex colours.
const SRGB_TO_LINEAR = Float32Array.from({ length: 256 }, (_, i) => {
  const c = i / 255;
  return c <= 0.04045 ? c / 12.92 : ((c + 0.055) / 1.055) ** 2.4;
});

/** rosbridge sends uint8[] as base64 (JSON) or a typed array (CBOR). */
export function toBytes(data) {
  if (data instanceof Uint8Array) return data;
  if (ArrayBuffer.isView(data)) return new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
  if (Array.isArray(data)) return Uint8Array.from(data);
  if (typeof data === 'string') {
    const bin = atob(data);
    const out = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
    return out;
  }
  return new Uint8Array(0);
}

/**
 * Decode x/y/z (+ packed rgb/rgba when present). Every `stride`-th point is
 * kept so at most maxPoints are drawn. Without colour, points are coloured
 * by height (z, ROS frame). Returns null if the cloud has no x/y/z.
 */
export function decodeCloud(msg, maxPoints = 400000) {
  const fields = Object.fromEntries((msg.fields || []).map((f) => [f.name, f]));
  if (!fields.x || !fields.y || !fields.z) return null;
  const bytes = toBytes(msg.data);
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const le = !msg.is_bigendian;
  const width = msg.width || 0;
  const n = width * (msg.height || 0);
  const step = msg.point_step;
  const rowStep = msg.row_step || width * step;
  const stride = Math.max(1, Math.ceil(n / maxPoints));
  const rgb = fields.rgb || fields.rgba;
  const ox = fields.x.offset, oy = fields.y.offset, oz = fields.z.offset;

  const cap = Math.ceil(n / stride);
  const positions = new Float32Array(cap * 3);
  const colors = new Float32Array(cap * 3);
  let k = 0;
  let zMin = Infinity, zMax = -Infinity;
  for (let i = 0; i < n; i += stride) {
    const o = Math.floor(i / width) * rowStep + (i % width) * step;
    if (o + step > bytes.byteLength) break;
    const x = dv.getFloat32(o + ox, le);
    const y = dv.getFloat32(o + oy, le);
    const z = dv.getFloat32(o + oz, le);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
    positions[3 * k] = x;
    positions[3 * k + 1] = y;
    positions[3 * k + 2] = z;
    if (rgb) {
      // PCL packs rgb as bytes b, g, r, (a) in a 32-bit field.
      const c = o + rgb.offset;
      colors[3 * k] = SRGB_TO_LINEAR[bytes[c + 2]];
      colors[3 * k + 1] = SRGB_TO_LINEAR[bytes[c + 1]];
      colors[3 * k + 2] = SRGB_TO_LINEAR[bytes[c]];
    }
    if (z < zMin) zMin = z;
    if (z > zMax) zMax = z;
    k++;
  }
  if (!rgb) heightColors(positions, colors, k, zMin, zMax);
  return {
    positions: positions.subarray(0, 3 * k),
    colors: colors.subarray(0, 3 * k),
    count: k,
    total: n,
    hasColor: !!rgb,
    frame: (msg.header && msg.header.frame_id) || '',
  };
}

/**
 * Voxel grid filter: one voxel per occupied size^3 cell (centre + mean
 * colour). If more than maxVoxels remain, the size grows by 1.25x and the
 * grid is rebuilt, so the triangle budget (12 per voxel cube) holds no
 * matter how big the map gets. Returns centres in the cloud's frame, plus
 * floorZ (2nd percentile of z) for the shadow-catching floor.
 */
export function voxelize(cloud, size = 0.08, maxVoxels = 80000, minPoints = 1) {
  const { positions, colors, count } = cloud;
  const OFF = 65536, K = 131072;            // keys stay below 2^53
  for (let attempt = 0; attempt < 12; attempt++, size *= 1.25) {
    const inv = 1 / size;
    const cells = new Map();
    for (let i = 0; i < count; i++) {
      const ix = Math.floor(positions[3 * i] * inv) + OFF;
      const iy = Math.floor(positions[3 * i + 1] * inv) + OFF;
      const iz = Math.floor(positions[3 * i + 2] * inv) + OFF;
      const key = (ix * K + iy) * K + iz;
      let c = cells.get(key);
      if (!c) {
        if (cells.size >= maxVoxels * 1.5) break;   // clearly too fine: next size
        c = [ix, iy, iz, 0, 0, 0, 0];
        cells.set(key, c);
      }
      c[3] += colors[3 * i]; c[4] += colors[3 * i + 1]; c[5] += colors[3 * i + 2]; c[6]++;
    }
    let kept = 0;
    for (const c of cells.values()) if (c[6] >= minPoints) kept++;
    if (kept > maxVoxels && attempt < 11) continue;

    const centers = new Float32Array(kept * 3);
    const vcolors = new Float32Array(kept * 3);
    const zs = new Float32Array(kept);
    let j = 0;
    for (const c of cells.values()) {
      if (c[6] < minPoints) continue;
      centers[3 * j] = (c[0] - OFF + 0.5) * size;
      centers[3 * j + 1] = (c[1] - OFF + 0.5) * size;
      centers[3 * j + 2] = (c[2] - OFF + 0.5) * size;
      vcolors[3 * j] = c[3] / c[6]; vcolors[3 * j + 1] = c[4] / c[6]; vcolors[3 * j + 2] = c[5] / c[6];
      zs[j] = centers[3 * j + 2];
      j++;
    }
    const sorted = zs.slice().sort();
    const floorZ = kept ? sorted[Math.floor(kept * 0.02)] - size / 2 : 0;
    return { centers, colors: vcolors, count: kept, size, floorZ, hasColor: cloud.hasColor, frame: cloud.frame };
  }
  return { centers: new Float32Array(0), colors: new Float32Array(0), count: 0, size, floorZ: 0, hasColor: false, frame: cloud.frame };
}

function heightColors(positions, colors, count, zMin, zMax) {
  const span = Math.max(1e-3, zMax - zMin);
  for (let i = 0; i < count; i++) {
    const t = (positions[3 * i + 2] - zMin) / span;          // 0 floor .. 1 top
    colors[3 * i] = 0.15 + 0.85 * t;                          // blue -> orange
    colors[3 * i + 1] = 0.45 + 0.35 * Math.sin(Math.PI * t);
    colors[3 * i + 2] = 0.95 - 0.8 * t;
  }
}
