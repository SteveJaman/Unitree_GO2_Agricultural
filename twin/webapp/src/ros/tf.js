import { Matrix4, Quaternion, Vector3 } from 'three';

const ONE = new Vector3(1, 1, 1);
const strip = (f) => (f || '').replace(/^\//, '');

/** geometry_msgs Transform or Pose -> Matrix4 (ROS frame, z up). */
export function toMatrix(translation, rotation) {
  return new Matrix4().compose(
    new Vector3(translation.x, translation.y, translation.z),
    new Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
    ONE,
  );
}

/**
 * Minimal TF tree from /tf and /tf_static (no tf2_web_republisher needed).
 * Keeps the newest transform per child frame; lookup(fixed, frame) returns
 * the Matrix4 that maps `frame` coordinates into `fixed`, or null when the
 * two frames are not connected (yet).
 */
export class TfTree {
  constructor() {
    this.parent = new Map();
    this.transform = new Map();
  }

  add(msg) {
    for (const t of msg.transforms || []) {
      const child = strip(t.child_frame_id);
      this.parent.set(child, strip(t.header.frame_id));
      this.transform.set(child, toMatrix(t.transform.translation, t.transform.rotation));
    }
  }

  has(frame) {
    return this.parent.has(strip(frame));
  }

  /** frame -> root of its tree: { matrix: T_root_frame, root } */
  toRoot(frame) {
    const m = new Matrix4();
    const seen = new Set();
    let f = strip(frame);
    while (this.parent.has(f) && !seen.has(f)) {
      seen.add(f);
      m.premultiply(this.transform.get(f));
      f = this.parent.get(f);
    }
    return { matrix: m, root: f };
  }

  lookup(fixed, frame) {
    fixed = strip(fixed);
    frame = strip(frame);
    if (fixed === frame) return new Matrix4();
    const a = this.toRoot(frame);
    const b = this.toRoot(fixed);
    if (a.root !== b.root) return null;
    return b.matrix.invert().multiply(a.matrix);
  }

  frames() {
    return [...new Set([...this.parent.keys(), ...this.parent.values()])].sort();
  }
}
