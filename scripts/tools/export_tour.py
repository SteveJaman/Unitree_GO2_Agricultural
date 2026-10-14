#!/usr/bin/env python3
"""
export_tour.py — convert an RTAB-Map database (or PLY) into a mesh.

Two modes:
  - Default: launch rtabmap-databaseViewer to export the PLY manually,
             then this script meshes it.
  - --from-ply: skip the viewer, use an existing PLY.

Usage:
    python3 scripts/tools/export_tour.py ~/.ros/tour.db -o tour.ply
    python3 scripts/tools/export_tour.py tour.ply --from-ply -o tour_mesh.ply

Requires: open3d (pip install open3d)
"""
import argparse
import pathlib
import subprocess
import sys


def check_open3d():
    try:
        import open3d  # noqa: F401
    except ImportError:
        sys.exit(
            "open3d not installed. Run:\n"
            "    pip install open3d\n"
            "or inside the container:\n"
            "    docker exec -u root go2-lidar3d pip3 install open3d"
        )


def open_viewer(db_path):
    print(f"Opening rtabmap-databaseViewer with {db_path}")
    print()
    print("In the viewer:")
    print("  File -> Export -> Export 3D clouds")
    print("  Save as a .ply file")
    print("  Then re-run this script with that file and --from-ply")
    print()
    try:
        subprocess.run(
            ["ros2", "run", "rtabmap_ros", "rtabmap-databaseViewer", str(db_path)],
            check=False,
        )
    except FileNotFoundError:
        sys.exit("rtabmap-databaseViewer not found. Run this inside the container.")


def mesh_ply(ply_path, out_path, voxel, depth, remove_outliers):
    import numpy as np
    import open3d as o3d

    print(f"Loading {ply_path}")
    pcd = o3d.io.read_point_cloud(str(ply_path))
    print(f"  {len(pcd.points)} points")

    if remove_outliers:
        print(f"Removing outliers...")
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        print(f"  {len(pcd.points)} points after")

    if voxel > 0:
        print(f"Downsampling at voxel_size={voxel}")
        pcd = pcd.voxel_down_sample(voxel_size=voxel)
        print(f"  {len(pcd.points)} points")

    print(f"Poisson reconstruction (depth={depth})...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth
    )

    print("Removing low-density vertices...")
    densities = np.asarray(densities)
    cutoff = np.quantile(densities, 0.01)
    mesh.remove_vertices_by_mask(densities < cutoff)

    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(str(out_path), mesh)
    print(f"Wrote {out_path}")
    print(f"  {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", help=".db (RTAB-Map database) or .ply")
    ap.add_argument("-o", "--output", default="tour_mesh.ply",
                    help="output mesh path (default: tour_mesh.ply)")
    ap.add_argument("--from-ply", action="store_true",
                    help="input is already a PLY; skip the viewer")
    ap.add_argument("--voxel", type=float, default=0.02,
                    help="voxel downsample size, 0 to disable (default: 0.02)")
    ap.add_argument("--depth", type=int, default=9,
                    help="Poisson octree depth (default: 9)")
    ap.add_argument("--no-outlier-removal", action="store_true",
                    help="skip statistical outlier removal")
    args = ap.parse_args()

    src = pathlib.Path(args.input)
    if not src.exists():
        sys.exit(f"not found: {src}")

    if not args.from_ply:
        if src.suffix != ".db":
            sys.exit("expected a .db file, or pass --from-ply")
        open_viewer(src)
        print()
        print(f"After exporting the PLY, re-run:")
        print(f"  python3 {__file__} <exported.ply> --from-ply -o {args.output}")
        return

    check_open3d()
    mesh_ply(
        src,
        args.output,
        voxel=args.voxel,
        depth=args.depth,
        remove_outliers=not args.no_outlier_removal,
    )


if __name__ == "__main__":
    main()