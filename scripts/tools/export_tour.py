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
    # what wifi_3d_nav.sh --export --mesh runs (web dashboard "Mesh" mode):
    python3 scripts/tools/export_tour.py cloud.ply --from-ply -o db_mesh.ply --max-triangles 300000

Meshing: outlier removal -> voxel downsample -> normals (estimated if the
PLY has none; Poisson needs them) -> Poisson -> drop vertices farther than
--max-gap from any scan point (the "balloon" Poisson grows over unseen
areas; a distance cut keeps sparse real surfaces such as LiDAR floors,
which a density cut would remove) -> crop to the cloud's bounding box ->
optional quadric decimation to --max-triangles. Vertex colours carry over
when the cloud has them.

Requires: open3d (pip install open3d; included in the go2-3d image)
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


def mesh_ply(ply_path, out_path, voxel, depth, remove_outliers, trim=0.0, max_triangles=0, max_gap=0.0):
    import numpy as np
    import open3d as o3d

    print(f"Loading {ply_path}")
    pcd = o3d.io.read_point_cloud(str(ply_path))
    print(f"  {len(pcd.points)} points, colours={pcd.has_colors()}, normals={pcd.has_normals()}")
    if len(pcd.points) < 100:
        sys.exit("too few points to mesh")

    if remove_outliers:
        print("Removing outliers...")
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        print(f"  {len(pcd.points)} points after")

    if voxel > 0:
        print(f"Downsampling at voxel_size={voxel}")
        pcd = pcd.voxel_down_sample(voxel_size=voxel)
        print(f"  {len(pcd.points)} points")

    if not pcd.has_normals():
        # Poisson needs oriented normals; LiDAR scan exports often have none.
        # Indoors the scan was taken from inside the rooms, so normals are
        # turned toward the scan's centre (fast; the tangent-plane MST
        # orientation stalls on clouds of a few hundred thousand points).
        radius = max(voxel, 0.02) * 4
        print(f"Estimating normals (radius {radius:.2f} m)...")
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
        pcd.orient_normals_towards_camera_location(pcd.get_axis_aligned_bounding_box().get_center())

    print(f"Poisson reconstruction (depth={depth})...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth
    )

    # Poisson closes the surface over unseen space. Keep only vertices close
    # to a measured point: unlike a density cut this keeps sparsely sampled
    # but real surfaces (LiDAR floors are hit at grazing angles).
    max_gap = max_gap or max(voxel, 0.02) * 5
    verts = o3d.geometry.PointCloud(mesh.vertices)
    gap = np.asarray(verts.compute_point_cloud_distance(pcd))
    print(f"Removing vertices more than {max_gap:.2f} m from the scan "
          f"({np.count_nonzero(gap > max_gap)} of {len(gap)})...")
    mesh.remove_vertices_by_mask(gap > max_gap)
    if trim > 0:
        print(f"Removing the lowest-density {trim:.0%} of vertices...")
        densities = np.asarray(densities)[gap <= max_gap]
        mesh.remove_vertices_by_mask(densities < np.quantile(densities, trim))
    # Crop to the scan's box, grown by the gap: floors and outer walls lie on
    # the box faces, and the smooth Poisson surface sits just outside them.
    box = pcd.get_axis_aligned_bounding_box()
    mesh = mesh.crop(o3d.geometry.AxisAlignedBoundingBox(box.min_bound - max_gap, box.max_bound + max_gap))

    if max_triangles and len(mesh.triangles) > max_triangles:
        print(f"Decimating {len(mesh.triangles)} -> {max_triangles} triangles...")
        mesh = mesh.simplify_quadric_decimation(max_triangles)

    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(str(out_path), mesh, write_ascii=False)
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
    ap.add_argument("--max-gap", type=float, default=0.0,
                    help="drop mesh vertices farther than this (m) from any scan point; "
                         "0 = 5 x voxel (default)")
    ap.add_argument("--trim", type=float, default=0.0,
                    help="also drop this fraction of lowest-density Poisson vertices (default: 0)")
    ap.add_argument("--max-triangles", type=int, default=0,
                    help="decimate to at most this many triangles, 0 = keep all (default: 0)")
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
        trim=args.trim,
        max_triangles=args.max_triangles,
        max_gap=args.max_gap,
    )


if __name__ == "__main__":
    main()