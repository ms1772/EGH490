#!/usr/bin/env python3
"""lidar_to_obstacles.py — CLI entry for the perception pipeline.

Reads a point cloud, removes the ground, voxelizes, merges by Chebyshev
clustering, splits low-fill clusters along the median axis, inflates each
leaf by --safety, and writes a dict pkl drop-in for DECK_GA_QuickNav.py.

Usage example:
    python perception/lidar_to_obstacles.py \\
        --input  path/to/survey.las \\
        --output data/obstacles/obstacles_lidar.pkl \\
        --voxel 0.3 --safety 0.5 --ground-method ransac \\
        --auto-center --seed 42
"""

from __future__ import annotations

import argparse
import pickle
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from perception.io_utils import load_cloud, write_obstacles_dict, write_obstacles_txt
from perception.ground_filter import remove_ground
from perception.adaptive_aabb import extract_obstacles
from perception.viz import save_matplotlib_snapshot, show_open3d_viewer


def _adaptive_voxel(points: np.ndarray) -> float:
    extent = float(np.max(points.max(axis=0) - points.min(axis=0)))
    return float(np.clip(extent / 1000.0, 0.1, 1.0))


def _coord_frame_warning(
    centres: np.ndarray,
    waypoints_pkl: Optional[Path],
    fallback_threshold: float = 50.0,
) -> None:
    """Warn loudly if extracted obstacle bbox is far from waypoint bbox.

    The most common silent-wrong-output failure: lidar cloud in UTM-like
    coordinates, waypoints near origin. Planner runs but sees zero in-range
    obstacles and produces a no-avoidance plan.
    """
    if len(centres) == 0:
        return

    obs_centre = centres.mean(axis=0)
    obs_extent = centres.max(axis=0) - centres.min(axis=0)

    if waypoints_pkl is not None and Path(waypoints_pkl).exists():
        with Path(waypoints_pkl).open("rb") as f:
            wp_arr = np.asarray(pickle.load(f), dtype=float)
        if wp_arr.ndim == 2 and wp_arr.shape[1] == 3 and len(wp_arr) > 0:
            wp_centre = wp_arr.mean(axis=0)
            gap = float(np.linalg.norm(obs_centre - wp_centre))
            if gap > fallback_threshold:
                print(
                    f"\n=== COORDINATE-FRAME WARNING ===\n"
                    f"Extracted obstacles centred at {obs_centre.round(2)}, "
                    f"extent {obs_extent.round(2)}.\n"
                    f"Waypoints centred at {wp_centre.round(2)}.\n"
                    f"Distance between centres: {gap:.1f} m (> {fallback_threshold} m threshold).\n"
                    f"The planner will likely see zero in-range obstacles and silently produce\n"
                    f"a no-avoidance plan. Use --translate or --auto-center to recentre.\n"
                    f"================================\n",
                    file=sys.stderr,
                )
            else:
                print(f"[coord-check] obstacle vs waypoint gap {gap:.2f} m (ok).")
        return

    # No waypoints supplied: just print extracted bbox so the user can eyeball it.
    print(
        f"[coord-check] extracted obstacle bbox centre={obs_centre.round(2)}, "
        f"extent={obs_extent.round(2)}. No --waypoints-pkl supplied."
    )


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Extract obstacle AABBs from a point cloud for DECK-GA + QuickNav.",
        allow_abbrev=False,
    )
    ap.add_argument("--input", required=True, help="Point-cloud file (.pcd .ply .xyz .las .laz .npy .bin)")
    ap.add_argument("--output", required=True, help="Output dict-pkl path")
    ap.add_argument("--output-txt", default=None,
                    help="Optional sidecar 6-column CSV for human inspection. Defaults to <output>.txt")

    ap.add_argument("--voxel", type=float, default=None,
                    help="Voxel resolution in metres. Default: clip(scene_extent/1000, 0.1, 1.0).")
    ap.add_argument("--safety", type=float, default=0.5,
                    help="Per-axis inflation margin in metres (default 0.5)")
    ap.add_argument("--fill-threshold", type=float, default=0.05,
                    help="Min fill ratio before split (default 0.05, vegetation-aware)")
    ap.add_argument("--max-split-depth", type=int, default=3,
                    help="Median-axis recursion cap (default 3)")
    ap.add_argument("--min-voxels-per-leaf", type=int, default=8)

    ap.add_argument("--ground-method", choices=["ransac", "z_threshold", "none"],
                    default="ransac")
    ap.add_argument("--ground-z", type=float, default=None,
                    help="Floor z for z_threshold. Auto-pick (5th pctl) if omitted.")
    ap.add_argument("--ransac-distance", type=float, default=None,
                    help="RANSAC inlier threshold. Default: equal to --voxel.")

    ap.add_argument("--translate", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    default=[0.0, 0.0, 0.0])
    ap.add_argument("--auto-center", action="store_true",
                    help="Translate so cloud xy centroid is at origin (overrides --translate xy).")
    ap.add_argument("--waypoints-pkl", default=None,
                    help="Optional waypoints pkl for coord-frame sanity warning.")

    ap.add_argument("--seed", type=int, default=42)

    ap.add_argument("--viz", action="store_true", help="Open interactive Open3D viewer.")
    ap.add_argument("--no-viz", dest="viz", action="store_false")
    ap.set_defaults(viz=False)
    ap.add_argument("--snapshot-dir", default=str(Path(__file__).resolve().parent / "results"))

    args = ap.parse_args()

    input_path = Path(args.input).expanduser()
    output_path = Path(args.output).expanduser()

    print(f"[1/6] Loading cloud: {input_path}")
    t0 = time.time()
    points = load_cloud(input_path)
    print(f"      {len(points)} points loaded in {time.time() - t0:.2f}s")

    if args.auto_center:
        offset = -points[:, :2].mean(axis=0)
        translate = np.array([offset[0], offset[1], 0.0])
    else:
        translate = np.asarray(args.translate, dtype=float)

    if np.any(translate != 0.0):
        points = points + translate
        print(f"[2/6] Translated by {translate.round(3).tolist()}")
    else:
        print(f"[2/6] No translation applied")

    cloud_extent_xyz = [
        (float(points[:, k].min()), float(points[:, k].max())) for k in range(3)
    ]

    voxel = float(args.voxel) if args.voxel is not None else _adaptive_voxel(points)
    print(f"[3/6] Voxel resolution: {voxel:.3f} m")

    ransac_distance = float(args.ransac_distance) if args.ransac_distance is not None else voxel
    print(f"[4/6] Ground removal ({args.ground_method})")
    t0 = time.time()
    filtered, gnd_info = remove_ground(
        points,
        method=args.ground_method,
        ransac_distance=ransac_distance,
        ground_z=args.ground_z,
        seed=int(args.seed),
    )
    print(
        f"      {gnd_info['n_in']} -> {gnd_info['n_out']} points "
        f"(-{gnd_info['n_in'] - gnd_info['n_out']}) in {time.time() - t0:.2f}s"
    )
    if "warning" in gnd_info:
        print(f"      WARNING: {gnd_info['warning']}", file=sys.stderr)

    print(f"[5/6] Adaptive AABB extraction")
    t0 = time.time()
    centres, sizes = extract_obstacles(
        filtered,
        r_voxel=voxel,
        s_safety=float(args.safety),
        tau_fill=float(args.fill_threshold),
        max_split_depth=int(args.max_split_depth),
        min_voxels_per_leaf=int(args.min_voxels_per_leaf),
    )
    print(f"      {len(centres)} obstacles extracted in {time.time() - t0:.2f}s")

    if len(centres) > 0:
        total_vol = float(np.sum(np.prod(2.0 * sizes, axis=1)))
        print(f"      total inflated volume: {total_vol:.2f} m^3")

    _coord_frame_warning(
        centres, Path(args.waypoints_pkl) if args.waypoints_pkl else None
    )

    meta = {
        "source": "lidar",
        "input_cloud": str(input_path),
        "params": {
            "voxel": voxel,
            "safety": float(args.safety),
            "fill_threshold": float(args.fill_threshold),
            "max_split_depth": int(args.max_split_depth),
            "min_voxels_per_leaf": int(args.min_voxels_per_leaf),
            "ground_method": args.ground_method,
            "ground_z": args.ground_z,
            "ransac_distance": ransac_distance,
            "seed": int(args.seed),
        },
        "safety_used": float(args.safety),
        "translate": translate.tolist(),
        "cloud_extent": cloud_extent_xyz,
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }

    print(f"[6/6] Writing output")
    write_obstacles_dict(output_path, centres, sizes, meta)
    print(f"      pkl  -> {output_path}")

    txt_path = Path(args.output_txt) if args.output_txt else output_path.with_suffix(".txt")
    write_obstacles_txt(txt_path, centres, sizes)
    print(f"      txt  -> {txt_path}")

    snapshot_dir = Path(args.snapshot_dir).expanduser()
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    snap_path = snapshot_dir / f"{input_path.stem}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
    save_matplotlib_snapshot(filtered, centres, sizes, snap_path,
                             title=f"{input_path.name} -> {len(centres)} obstacles")
    print(f"      png  -> {snap_path}")

    if args.viz:
        print("Opening Open3D viewer (close window to continue)...")
        show_open3d_viewer(filtered, centres, sizes,
                           window_name=f"{input_path.name} -> {len(centres)} obstacles")

    print("Done.")


if __name__ == "__main__":
    main()
