// Web Worker: PointCloud2 decoding and the voxel grid filter, off the main
// thread. A 300k-point /rtabmap/cloud_map takes long enough on a VM's CPU to
// freeze the 3D view every time the map updates.
//
// in:  { op: 'cloud', kind: 'map' | 'scan', msg, maxPoints, voxel: { size, budget } | null }
//      { op: 'voxel', voxel: { size, budget } | null }   (re-filter the last map)
// out: { kind, cloud?, voxels? }   typed arrays transferred, not copied
import { decodeCloud, voxelize } from './pointcloud.js';

let lastMap = null;

function voxelsOf(cloud, voxel) {
  return cloud && voxel ? voxelize(cloud, voxel.size, voxel.budget) : null;
}

function buffers(...objs) {
  return objs.filter(Boolean).flatMap((o) => [o.positions, o.colors, o.centers]
    .filter((a) => a && a.buffer && a.byteLength)
    .map((a) => a.buffer));
}

self.onmessage = ({ data }) => {
  if (data.op === 'voxel') {
    const voxels = voxelsOf(lastMap, data.voxel);
    // The map stays here for the next re-filter; only the new voxels move.
    self.postMessage({ kind: 'map', voxels }, buffers(voxels && { centers: voxels.centers, colors: voxels.colors }));
    return;
  }
  const cloud = decodeCloud(data.msg, data.maxPoints);
  if (!cloud) { self.postMessage({ kind: data.kind, cloud: null }); return; }
  if (data.kind !== 'map') {
    self.postMessage({ kind: data.kind, cloud }, buffers(cloud));
    return;
  }
  // Keep a private copy of the map for re-filtering; transfer the decoded one.
  lastMap = { ...cloud, positions: cloud.positions.slice(), colors: cloud.colors.slice() };
  const voxels = voxelsOf(cloud, data.voxel);
  self.postMessage({ kind: 'map', cloud, voxels }, buffers(cloud, voxels && { centers: voxels.centers, colors: voxels.colors }));
};
