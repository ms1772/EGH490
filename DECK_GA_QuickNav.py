#!/usr/bin/env python3
"""
DECK_GA_QuickNav.py

Combined pipeline: DECK-GA path planning + QuickNav obstacle avoidance.

Steps:
1) Load 3D waypoints from --points_pkl (Nx3 array).
2) Load obstacle centres from --obstacles_pkl (Nx3 array, same format as points pkl).
3) Shift all points to positive space (algorithm stability).
4) Run DCKmeans clustering + GA optimisation per UAV (same as DECK_GA.py).
5) Shift deckga_paths back to original coordinates.
6) Run apply_quicknav() per UAV path against original-space obstacles.
7) Save deckga_quicknav_output.pkl with both deckga_paths and quicknav_paths.

Output pickle is a strict superset of DECK_GA.py's output, so all existing
downstream scripts that read deckga_paths continue to work unchanged.

Usage examples:
  python3 DECK_GA_QuickNav.py \\
      --points_pkl data/points/points_current.pkl \\
      --obstacles_pkl data/obstacles/obstacles_current.pkl \\
      --obs_size 5.0 \\
      --out_pkl deckga_ros2/data/deckga_quicknav_output.pkl

  # Generate obstacles with the existing point generator:
  python3 data/points/generate_points_xyz.py \\
      --n 10 --x 0 80 --y 0 80 --z 5 40 \\
      --out data/obstacles/obstacles_current.pkl
"""

import argparse
import pickle
import time
from pathlib import Path

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from DCKmeans import manual_kmeans_clustering
from GA_path_planning import ga_3d_pathplanning
from QuickNav import apply_quicknav

try:
    from perception.octomap_to_obstacles import octomap_to_aabbs
except Exception:  # pragma: no cover - only relevant when --octomap is used
    octomap_to_aabbs = None

matplotlib.rc("font", family="sans-serif")


# ----------------------------
# Helpers (same as DECK_GA.py)
# ----------------------------

def load_points(pkl_path: Path) -> np.ndarray:
    with pkl_path.open("rb") as f:
        arr = pickle.load(f)
    arr = np.asarray(arr, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f"Points must be Nx3. Got shape={arr.shape} from {pkl_path}")
    return arr


def load_obstacles_from_pkl(pkl_path: Path, fallback_size: float):
    """Auto-detect dict-pkl vs legacy Nx3 obstacle file.

    Returns
    -------
    centers : (N, 3) ndarray
    sizes   : (N, 3) ndarray  — per-axis half-sizes

    Dict format: {'centers': (N,3), 'sizes': (N,3) or (N,), 'meta': {...}}
    Legacy:     bare (N,3) numpy array of centres; sizes filled from fallback_size.
    """
    with pkl_path.open("rb") as f:
        obj = pickle.load(f)

    if isinstance(obj, dict) and "centers" in obj and "sizes" in obj:
        centers = np.asarray(obj["centers"], dtype=float)
        sizes = np.asarray(obj["sizes"], dtype=float)
        if sizes.ndim == 1:
            sizes = np.tile(sizes[:, None], (1, 3))
        if centers.ndim != 2 or centers.shape[1] != 3:
            raise ValueError(f"dict centers must be (N,3), got {centers.shape}")
        if sizes.shape != centers.shape:
            raise ValueError(
                f"dict sizes shape {sizes.shape} doesn't match centers {centers.shape}"
            )
        print(f"  (dict-format obstacles: per-axis sizes embedded; --obs_size ignored)")
        return centers, sizes

    arr = np.asarray(obj, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f"Legacy obstacles must be Nx3. Got shape={arr.shape} from {pkl_path}")
    sizes = np.full((len(arr), 3), float(fallback_size))
    return arr, sizes


def calculate_path_distance(path: np.ndarray) -> float:
    path = np.asarray(path, dtype=float)
    if len(path) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def parse_start_points(s: str, num_uavs: int) -> np.ndarray:
    sp = []
    for item in s.split(";"):
        xyz = [float(v) for v in item.split(",")]
        if len(xyz) != 3:
            raise ValueError("Each start point must be x,y,z")
        sp.append(xyz)
    arr = np.asarray(sp, dtype=float)
    if arr.shape != (num_uavs, 3):
        raise ValueError(f"start_points must be shape ({num_uavs},3), got {arr.shape}")
    return arr


def shift_to_positive(all_points: np.ndarray):
    mins = np.min(all_points, axis=0)
    offset = np.where(mins < 0.0, -mins, 0.0)
    return all_points + offset, offset


def ensure_closed_tour(path: np.ndarray) -> np.ndarray:
    path = np.asarray(path, dtype=float)
    if len(path) == 0:
        return path
    if not np.allclose(path[0], path[-1]):
        path = np.vstack([path, path[0]])
    return path


def maybe_save_fig(fig, save_dir, name: str):
    if save_dir is None:
        return
    save_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_dir / f"{name}.png", bbox_inches="tight", dpi=300)


# ----------------------------
# Main
# ----------------------------

def main():
    repo_root = Path(__file__).resolve().parent

    ap = argparse.ArgumentParser(
        description="DECK-GA + QuickNav obstacle avoidance pipeline"
    )
    ap.add_argument("--points_pkl", default="data/points/points_current.pkl",
                    help="Pkl file with Nx3 waypoint array")
    ap.add_argument("--obstacles_pkl", default=None,
                    help="Pkl file with Nx3 or dict obstacles (mutually exclusive with --octomap)")
    ap.add_argument("--octomap", default=None,
                    help="OctoMap .bt file; obstacles extracted in-process "
                         "(mutually exclusive with --obstacles_pkl)")
    ap.add_argument("--octomap-safety", dest="octomap_safety", type=float, default=0.5,
                    help="Safety inflation (m) applied when building AABBs from octomap")
    ap.add_argument("--octomap-fill-threshold", dest="octomap_fill_threshold",
                    type=float, default=0.05,
                    help="Fill-ratio threshold for AABB splitting from octomap")
    ap.add_argument("--octomap-max-split-depth", dest="octomap_max_split_depth",
                    type=int, default=3,
                    help="Max recursive median-axis split depth for octomap AABBs")
    ap.add_argument("--obs_size", type=float, default=5.0,
                    help="Uniform half-size for all obstacle cubes (default: 5.0)")
    ap.add_argument("--out_pkl", default="deckga_ros2/data/deckga_quicknav_output.pkl",
                    help="Output pickle path")
    ap.add_argument("--num_uavs", type=int, default=3)
    ap.add_argument("--start_points", default="10,0,10;50,20,10;90,0,10",
                    help='Semicolon-separated: "x,y,z;x,y,z;..." (must match num_uavs)')
    ap.add_argument("--no_plot", action="store_true", help="Disable matplotlib plots")
    ap.add_argument("--save_fig_dir", default=None,
                    help="If set, save figures (PNG) into this directory")
    ap.add_argument("--plot_kmeans", action="store_true")
    ap.add_argument("--plot_per_uav", action="store_true")

    args = ap.parse_args()

    def resolve(p):
        p = Path(p)
        return p if p.is_absolute() else (repo_root / p).resolve()

    if (args.obstacles_pkl is None) == (args.octomap is None):
        ap.error("exactly one of --obstacles_pkl or --octomap must be supplied")

    points_path = resolve(args.points_pkl)
    out_path = resolve(args.out_pkl)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    save_dir = resolve(args.save_fig_dir) if args.save_fig_dir else None

    # -------------------------------------------------------
    # Step 1: Load waypoints + obstacles
    # -------------------------------------------------------
    points = load_points(points_path)

    if args.obstacles_pkl is not None:
        obstacles_path = resolve(args.obstacles_pkl)
        obstacle_xyz, obs_size = load_obstacles_from_pkl(
            obstacles_path, fallback_size=args.obs_size
        )
        obs_source_str = f"pkl   : {obstacles_path}"
    else:
        if octomap_to_aabbs is None:
            ap.error("octomap support unavailable — perception.octomap_to_obstacles failed to import")
        octomap_path = resolve(args.octomap)
        obstacle_xyz, obs_size, _octo_meta = octomap_to_aabbs(
            octomap_path,
            s_safety=args.octomap_safety,
            tau_fill=args.octomap_fill_threshold,
            max_split_depth=args.octomap_max_split_depth,
        )
        obs_source_str = (
            f"octomap: {octomap_path}  "
            f"(res={_octo_meta['resolution']}, "
            f"leaves={_octo_meta['n_occupied_leaves']})"
        )

    num_points = len(points)
    num_uavs = args.num_uavs
    start_points = parse_start_points(args.start_points, num_uavs)

    # obs_size summary line — adaptive to scalar vs per-axis content.
    if len(obs_size) == 0:
        size_str = "none"
    elif np.allclose(obs_size, obs_size[0, 0]):
        size_str = f"uniform={float(obs_size[0, 0]):.3f}"
    else:
        smin, smax = float(obs_size.min()), float(obs_size.max())
        size_str = f"per-axis range=[{smin:.3f}, {smax:.3f}]"

    print("\n--- INPUT ---")
    print(f"Points file    : {points_path}  ({num_points} pts)")
    print(f"Obstacles {obs_source_str}  ({len(obstacle_xyz)} obstacles, {size_str})")
    print(f"Num UAVs       : {num_uavs}")
    print(f"Start pts      : {start_points}")

    # -------------------------------------------------------
    # Validate: no waypoint or start point falls inside an obstacle cube
    # -------------------------------------------------------
    all_check_pts = np.vstack([points, start_points])
    conflicts = []
    for oi, (oc, os_) in enumerate(zip(obstacle_xyz, obs_size)):
        pmin = oc - os_
        pmax = oc + os_
        inside = np.all((all_check_pts >= pmin) & (all_check_pts <= pmax), axis=1)
        for pi in np.where(inside)[0]:
            tag = f"start[{pi - num_points}]" if pi >= num_points else f"waypoint[{pi}]"
            conflicts.append(f"  {tag} {all_check_pts[pi]} is inside obstacle {oi} at {oc}")
    if conflicts:
        raise ValueError(
            "One or more waypoints/start-points fall inside an obstacle cube:\n"
            + "\n".join(conflicts)
            + "\nRegenerate points or obstacles so they do not overlap."
        )

    # -------------------------------------------------------
    # Step 2: Shift into positive space (waypoints + starts only;
    #         obstacles stay in original coords for QuickNav)
    # -------------------------------------------------------
    all_in = np.vstack([start_points, points])
    all_shifted, offset_used = shift_to_positive(all_in)
    start_shifted = all_shifted[:num_uavs]
    points_shifted = all_shifted[num_uavs:]

    # -------------------------------------------------------
    # Step 3: DCKmeans
    # -------------------------------------------------------
    t0 = time.time()
    clusters, centroids = manual_kmeans_clustering(points_shifted, num_uavs)
    print(f"\nDCKmeans done in {time.time() - t0:.3f} s")
    print("Centroids:\n", centroids)

    if (not args.no_plot) and args.plot_kmeans:
        fig_km = plt.figure()
        ax_km = fig_km.add_subplot(111, projection="3d")
        ax_km.set_title("DCKmeans Clustering (shifted space)")
        colors = cm.rainbow(np.linspace(0, 1, num_uavs))
        for i, cluster in enumerate(clusters):
            cp = np.asarray(cluster, dtype=float)
            ax_km.scatter(cp[:, 0], cp[:, 1], cp[:, 2], color=colors[i], label=f"Cluster {i+1}")
        ax_km.scatter(centroids[:, 0], centroids[:, 1], centroids[:, 2],
                      c="black", s=100, marker="x", label="Centroids")
        ax_km.legend()
        maybe_save_fig(fig_km, save_dir, "kmeans_clustering")
        plt.show()

    # -------------------------------------------------------
    # Step 4: Add start points + GA optimisation per UAV
    # -------------------------------------------------------
    clusters_with_start = []
    for i, cluster in enumerate(clusters):
        cluster_arr = np.asarray(cluster, dtype=float)
        clusters_with_start.append(np.vstack([start_shifted[i], cluster_arr]))

    raw_lengths = [calculate_path_distance(c) for c in clusters_with_start]
    print("\n--- Raw path lengths (before GA, shifted) ---")
    for i, d in enumerate(raw_lengths):
        print(f"  UAV {i}: {d:.3f}")

    optimized_paths_shifted = []
    dega_lengths = []
    t0 = time.time()

    for i, cluster_points in enumerate(clusters_with_start):
        opt = ga_3d_pathplanning(cluster_points)
        opt = ensure_closed_tour(opt)
        optimized_paths_shifted.append(opt)
        dega_lengths.append(calculate_path_distance(opt))
        print(f"  UAV {i} GA done, length={dega_lengths[-1]:.3f}")

    print(f"GA done in {time.time() - t0:.3f} s")

    # -------------------------------------------------------
    # Step 5: Shift deckga_paths back to original coords
    # -------------------------------------------------------
    deckga_paths = [np.asarray(p, dtype=float) - offset_used for p in optimized_paths_shifted]

    # -------------------------------------------------------
    # Step 6: QuickNav obstacle avoidance (original coords)
    # -------------------------------------------------------
    print("\n--- QuickNav obstacle avoidance ---")
    quicknav_paths = []
    final_lengths = []
    all_detected_indices = []

    for i, path in enumerate(deckga_paths):
        print(f"  UAV {i}: running QuickNav on {len(path)}-point path ...")
        qn_path = apply_quicknav(path, obstacle_xyz, obs_size)
        quicknav_paths.append(qn_path)
        fl = calculate_path_distance(qn_path)
        final_lengths.append(fl)

        # Record which obstacle indices were detected along this UAV's original path
        from QuickNav_detection_3D import obstacle_detection as _det
        _, detected_obs, _ = _det(path, obstacle_xyz, obs_size, visualize=False)
        detected_idx = []
        for ob in detected_obs:
            idxs = np.where((obstacle_xyz == ob).all(axis=1))[0]
            if len(idxs) > 0:
                detected_idx.append(int(idxs[0]))
        all_detected_indices.append(detected_idx)

        print(f"    {len(path)} pts -> {len(qn_path)} pts after avoidance, length={fl:.3f}")
        print(f"    Obstacles detected: {detected_idx}")

    # -------------------------------------------------------
    # Step 7: Save output pickle
    # -------------------------------------------------------
    out = {
        # Same keys as DECK_GA.py (backward compat)
        "deckga_paths": deckga_paths,
        "raw_lengths": raw_lengths,
        "dega_lengths": dega_lengths,
        "centroids": centroids,
        "offset_used": offset_used,
        "num_uavs": num_uavs,
        "num_points": int(num_points),
        "points_file": str(points_path),
        # QuickNav additions
        "quicknav_paths": quicknav_paths,
        "obstacle_xyz": obstacle_xyz,
        "obs_size": obs_size,
        "detected_indices": all_detected_indices,
        "final_lengths": final_lengths,
    }

    with out_path.open("wb") as f:
        pickle.dump(out, f)

    print(f"\n--- Saved output to: {out_path} ---")
    print("Keys:", sorted(out.keys()))

    # -------------------------------------------------------
    # Step 8: Plotting
    # -------------------------------------------------------
    if not args.no_plot:
        from QuickNav_detection_3D import draw_cube  # local: drawn only when plotting

        fig_combined = plt.figure(figsize=(5, 3.5), dpi=200)
        ax = fig_combined.add_subplot(111, projection="3d")
        ax.set_title("DECK-GA + QuickNav Paths", fontsize=10)
        colors = cm.rainbow(np.linspace(0, 1, num_uavs))

        # Draw obstacles: centre marker + per-axis box wireframe
        for obs_c, obs_s in zip(obstacle_xyz, obs_size):
            ax.scatter(*obs_c, c="red", marker="x", s=60)
            draw_cube(ax, obs_c, obs_s)

        # Draw deckga paths (dashed) and quicknav paths (solid)
        for i in range(len(deckga_paths)):
            dp = np.asarray(deckga_paths[i], dtype=float)
            qp = np.asarray(quicknav_paths[i], dtype=float)
            ax.plot(dp[:, 0], dp[:, 1], dp[:, 2], "--", color=colors[i], alpha=0.4,
                    label=f"UAV {i} DECKGA")
            ax.plot(qp[:, 0], qp[:, 1], qp[:, 2], "-", color=colors[i],
                    label=f"UAV {i} QuickNav")
            ax.scatter(qp[0, 0], qp[0, 1], qp[0, 2], c="red", s=40, marker="o")

        ax.set_xlabel("X", fontsize=9)
        ax.set_ylabel("Y", fontsize=9)
        ax.set_zlabel("Z", fontsize=9)
        maybe_save_fig(fig_combined, save_dir, "deckga_quicknav_combined")
        plt.show()

        if args.plot_per_uav:
            for i in range(len(quicknav_paths)):
                fig = plt.figure()
                ax2 = fig.add_subplot(111, projection="3d")
                qp = np.asarray(quicknav_paths[i], dtype=float)
                ax2.plot(qp[:, 0], qp[:, 1], qp[:, 2], "*-", label=f"UAV {i+1} QuickNav")
                ax2.scatter(qp[0, 0], qp[0, 1], qp[0, 2], c="red", s=100, marker="o",
                            label="Start")
                for obs_c, obs_s in zip(obstacle_xyz, obs_size):
                    ax2.scatter(*obs_c, c="red", marker="x", s=60)
                    draw_cube(ax2, obs_c, obs_s)
                ax2.set_title(f"QuickNav Path UAV {i+1}")
                ax2.set_xlabel("X"); ax2.set_ylabel("Y"); ax2.set_zlabel("Z")
                ax2.legend()
                maybe_save_fig(fig, save_dir, f"quicknav_uav{i+1}_path")
                plt.show()

    print("\n--- Summary ---")
    for i in range(num_uavs):
        print(f"  UAV {i}:  raw={raw_lengths[i]:.2f}  dega={dega_lengths[i]:.2f}  "
              f"quicknav={final_lengths[i]:.2f}  obstacles_hit={all_detected_indices[i]}")


if __name__ == "__main__":
    main()
