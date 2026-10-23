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
    frame: (msg.header && msg.header.frame_id) || '',
  };
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
