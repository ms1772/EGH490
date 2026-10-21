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
# Campaign arms
# ----------------------------

def _run_arm(*, args, points, start_points, obstacle_xyz, obs_size,
             num_uavs, arm_label):
    """Allocation + ordering for the non-legacy arms.

    Everything here works in ORIGINAL coordinates and carries node INDICES, not
    coordinates. Both are deliberate:

    * Original coordinates, because the cost matrix must be built where the
      obstacles actually are. DECK_GA_QuickNav shifts waypoints into positive
      space but leaves obstacles untouched; Euclidean distance survives that
      translation so it has never mattered, but detour cost does not. Building
      the matrix in shifted space does not raise -- it makes every pair look
      clear, and the obstacle-aware arms silently become the Euclidean ones.
      `assert_detours_found` below is the tripwire.

    * Indices, because the published path loses them immediately
      (manual_kmeans_clustering returns point arrays, DCKmeans.py:69) and
      recovering them later means matching float triples.

    Node layout matches `all_in` in the legacy path exactly:
        index i             -> start point i        (i < num_uavs)
        index num_uavs + j  -> waypoint j

    Returns (deckga_paths, dega_lengths, raw_lengths, centroids, labels,
             uav_of_cluster, n_unreachable, cost_meta, stage_times).
    """
    import numpy as _np

    from planner import balance as _balance
    from planner import cost_matrix as _cm
    from planner import kmedoids as _km
    from planner import timing as _timing
    from planner import tour as _tour

    if args.cluster != "kmedoids":
        # This path is k-medoids by construction: it needs labels and a
        # precomputed cost matrix, neither of which DCKmeans provides. Rather
        # than silently substituting the clusterer the caller did not ask for,
        # refuse. The five campaign arms never need kmeans off the legacy path.
        raise ValueError(
            f"--cluster {args.cluster} is only supported on the legacy path "
            f"(all other campaign flags at their defaults). Combining it with "
            f"--cost {args.cost} / --objective {args.objective} is not a "
            f"defined arm; use --cluster kmedoids."
        )

    st = {}
    rng = _np.random.default_rng(args.planner_seed
                                 if args.planner_seed is not None else 0)
    tm = (_timing.TimeModel.parse(args.time_model) if args.time_model
          else _timing.GAZEBO_SPEED1)

    nodes = _np.vstack([start_points, points])
    start_nodes = _np.arange(num_uavs)
    wp_nodes = _np.arange(num_uavs, num_uavs + len(points))

    # ---- cost matrix ----------------------------------------------------
    t0 = time.time()
    cost = _cm.load_or_build(
        args.cost, nodes, obstacle_xyz, obs_size,
        cache_dir=args.cost_cache_dir,
    )
    st["cost_matrix_s"] = time.time() - t0

    # The node layout is load-bearing: index i is start i, index num_uavs + j is
    # waypoint j, matching `all_in` on the legacy path. Every lookup below
    # (start_nodes, wp_nodes, medoids_global) assumes it. A matrix of the wrong
    # size means the layout assumption is broken and the indices address the
    # wrong points -- which produces a plausible-looking plan for the wrong
    # scene rather than an error.
    n_expected = num_uavs + len(points)
    if cost.length.shape != (n_expected, n_expected):
        raise AssertionError(
            f"cost matrix is {cost.length.shape}, expected "
            f"{(n_expected, n_expected)} for {num_uavs} start(s) + "
            f"{len(points)} waypoint(s)"
        )
    cost.assert_detours_found(len(obstacle_xyz))
    print(f"\nCost matrix ({args.cost}) in {st['cost_matrix_s']:.1f} s: "
          f"{cost.meta.get('n_detour_edges', 0)} detour edge(s), "
          f"{cost.meta.get('n_infeasible_edges', 0)} infeasible, "
          f"cache_hit={cost.meta.get('cache_hit')}")

    # A waypoint that cannot be reached from anything pins whichever UAV owns
    # it as permanently critical and makes a min-max balancer thrash. The
    # planner's own overlap check catches "inside a box" but not "walled in by
    # boxes"; measured 5 such nodes at n=100 with 100 obstacles.
    all_unreachable = [int(i) for i in cost.unreachable_nodes()]
    bad_starts = [i for i in all_unreachable if i < num_uavs]
    if bad_starts:
        raise ValueError(
            f"UNREACHABLE WAYPOINTS: start point(s) {bad_starts} have no feasible "
            f"edge to anything. A UAV that cannot leave its start cannot fly the "
            f"mission; the scene or the start points are wrong."
        )
    unreachable = [i for i in all_unreachable if i >= num_uavs]
    if unreachable:
        print(f"UNREACHABLE WAYPOINTS: dropping {len(unreachable)} node(s) "
              f"that no feasible edge reaches: {unreachable[:8]}")
        wp_nodes = _np.array([n for n in wp_nodes if n not in set(unreachable)])
        if len(wp_nodes) < num_uavs:
            raise ValueError(
                f"UNREACHABLE WAYPOINTS: only {len(wp_nodes)} reachable waypoint(s) "
                f"remain for {num_uavs} UAVs. The scene is too dense to plan."
            )

    # ---- allocation ------------------------------------------------------
    t0 = time.time()
    sub = cost.length[_np.ix_(wp_nodes, wp_nodes)]
    km = _km.kmedoids(sub, num_uavs, rng=rng, n_init=5)
    labels_full = _np.full(len(nodes), -1, dtype=int)
    labels_full[wp_nodes] = km.labels
    medoids_global = _np.array([int(wp_nodes[m]) for m in km.medoids])
    st["cluster_s"] = time.time() - t0
    print(f"k-medoids in {st['cluster_s']:.2f} s, cluster sizes "
          f"{_np.bincount(km.labels, minlength=num_uavs).tolist()}")

    # ---- balance ---------------------------------------------------------
    t0 = time.time()
    # One config for BOTH objectives. `closed_tours` in particular must match:
    # it decides whether a tour's cost includes the return leg to the start, so
    # a minsum arm evaluated open against a minmax arm evaluated closed would
    # differ by a whole edge per UAV and the 2x2 ablation would be measuring
    # that instead of the objective.
    cfg = _balance.BalanceConfig(max_seconds=float(args.balance_max_seconds))
    if args.objective == "minmax":
        res = _balance.balance_makespan(cost, tm, labels_full, start_nodes,
                                        medoids=medoids_global, cfg=cfg, rng=rng)
        labels_full = res.labels
        uav_of_cluster = res.uav_of_cluster
        print(f"balance: makespan {res.makespan0:.1f} -> {res.makespan:.1f} s "
              f"({res.improvement:+.1%}), {res.n_moves} move(s), {res.terminated}")
    else:
        # minsum still chooses a cluster->UAV pairing -- otherwise the arms
        # would differ by whether the pairing was optimised at all rather than
        # by the objective -- but it optimises the SUM, matching its own
        # objective. Pairing on the max here would make this arm partly min-max
        # and would understate A2-over-A1 and A4-over-A3.
        ev = _balance._Evaluator(cost, tm, start_nodes, num_uavs,
                                 cfg.closed_tours)
        uav_of_cluster, _times = _balance.best_pairing(ev, labels_full,
                                                       objective="minsum")
    st["balance_s"] = time.time() - t0

    # ---- relabel into UAV space -----------------------------------------
    # EVERYTHING DOWNSTREAM IS INDEXED BY UAV, NOT BY CLUSTER. The legacy path
    # sets that convention at line 549: `vstack([start_shifted[i], cluster_i])`,
    # so `deckga_paths[i]` is UAV i's tour starting from start point i, and the
    # executor hands `paths[i]` to drone i.
    #
    # k-medoids labels are arbitrary cluster ids, and `best_pairing` /
    # `balance_makespan` then choose which UAV flies which cluster -- that
    # permutation is the entire point of the min-max arms. Emitting the lists in
    # cluster order would hand drone j a tour whose first waypoint is drone
    # `uav_of_cluster[j]`'s start point: every aircraft would open the mission by
    # transiting to someone else's start, and the pairing optimisation would be
    # silently discarded.
    #
    # (It would NOT corrupt the offline makespan aggregate, since max over tours
    # is order-invariant, which is exactly why this could have reached Gazebo
    # unnoticed.)
    #
    # Remapping the labels once, here, is safer than carrying the permutation
    # through three list-building sites: afterwards `labels == i` means "the
    # waypoints UAV i flies" and the index is the UAV everywhere below.
    uav_of_cluster = _np.asarray(uav_of_cluster, dtype=int)
    if sorted(uav_of_cluster.tolist()) != list(range(num_uavs)):
        raise AssertionError(
            f"uav_of_cluster must be a permutation of 0..{num_uavs - 1}, "
            f"got {uav_of_cluster.tolist()}"
        )
    labels_uav = _np.full_like(labels_full, -1)
    for j in range(num_uavs):
        labels_uav[labels_full == j] = int(uav_of_cluster[j])
    labels_full = labels_uav

    # ---- tours -----------------------------------------------------------
    t0 = time.time()
    ga_iters = int(args.ga_iters) if args.ga_iters else int(1e5)
    deckga_paths, dega_lengths, raw_lengths = [], [], []
    for u in range(num_uavs):
        members = _np.flatnonzero(labels_full == u).tolist()
        start = int(start_nodes[u])
        # Per-UAV seed so the tours are independent but reproducible.
        rng_j = _np.random.default_rng(
            (args.planner_seed if args.planner_seed is not None else 0) + 1000 + u)
        gres = _tour.ga_tour(cost, members, start, num_iter=ga_iters,
                             rng=rng_j, points_xyz=nodes)
        # ga_tour puts `start` at local index 0 and relies on the GA rotating
        # its route to begin at row 0. If that ever stops holding, the tour
        # still contains the right nodes in a valid order, so nothing raises --
        # the aircraft simply launches toward some waypoint in the middle of its
        # route and returns to its start last. Cheap to check, invisible if not.
        # `route_indices` is ALREADY CLOSED: GA_path_planning.py:252 returns
        # `concatenate([idx, idx[:1]])`, so the start node appears again as the
        # final entry and the array is len(members) + 2 long. Checking raw
        # uniqueness here would therefore always fail.
        route = _np.asarray(gres.route_indices, dtype=int)
        if int(route[0]) != start:
            raise AssertionError(
                f"UAV {u}: tour begins at node {int(route[0])}, not its start "
                f"node {start}; GA rotation assumption in ga_tour broken"
            )
        if int(route[-1]) != start:
            raise AssertionError(
                f"UAV {u}: tour ends at node {int(route[-1])}, not back at its "
                f"start node {start}; the route is not a closed tour"
            )
        body = route[:-1]
        if len(set(body.tolist())) != len(body):
            raise AssertionError(
                f"UAV {u}: tour revisits a node ({len(body)} stops, "
                f"{len(set(body.tolist()))} unique)"
            )
        if len(body) != len(members) + 1:
            raise AssertionError(
                f"UAV {u}: tour visits {len(body)} node(s) but the cluster has "
                f"{len(members)} waypoint(s) plus 1 start"
            )
        path = ensure_closed_tour(nodes[route])
        deckga_paths.append(path)
        dega_lengths.append(calculate_path_distance(path))
        raw_lengths.append(calculate_path_distance(nodes[[start] + members]))
        print(f"  UAV {u} tour: {len(members)} wp, length={dega_lengths[-1]:.3f}")
    st["ga_s"] = time.time() - t0
    print(f"tours in {st['ga_s']:.1f} s")

    centroids = _np.array([
        nodes[_np.flatnonzero(labels_full == u)].mean(axis=0)
        if (labels_full == u).any() else _np.zeros(3)
        for u in range(num_uavs)
    ])

    return (deckga_paths, dega_lengths, raw_lengths, centroids,
            labels_full, uav_of_cluster, len(unreachable),
            dict(cost.meta), st)


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
    # --- campaign arms ------------------------------------------------------
    # Omit every one of these and the planner takes an explicit legacy branch
    # that is byte-for-byte what it has always done. See _is_legacy below --
    # the guarantee is a single auditable `if`, not "the general path with
    # default parameters", because merely BUILDING a cost matrix consumes
    # global-RNG draws and would shift everything downstream of DCKmeans'
    # np.random.seed(42).
    ap.add_argument("--cluster", default="kmeans", choices=["kmeans", "kmedoids"],
                    help="Allocation. kmeans is DCKmeans, the published method. "
                         "kmedoids works on any cost matrix (k-means needs a "
                         "Euclidean mean, which a detour metric has no notion of) "
                         "and cannot produce an empty cluster.")
    ap.add_argument("--cost", default="euclidean", choices=["euclidean", "quicknav"],
                    help="Edge metric for BOTH clustering and tour ordering. "
                         "quicknav prices each edge by the length of the avoided "
                         "route rather than the straight line.")
    ap.add_argument("--objective", default="minsum", choices=["minsum", "minmax"],
                    help="minsum is total fleet distance, the published objective. "
                         "minmax is makespan -- the longest tour -- which is what "
                         "actually determines when a concurrent mission ends.")
    ap.add_argument("--planner_seed", type=int, default=None,
                    help="Explicit RNG seed for clustering and the GA. Without it "
                         "the planner is reproducible only by accident, via the "
                         "np.random.seed(42) inside DCKmeans.")
    ap.add_argument("--arm", default=None, help="Label recorded in the pickle.")
    ap.add_argument("--time_model", default=None, metavar="V_EFF,T_STOP",
                    help="Execution time model. Default comes from planner.timing.")
    ap.add_argument("--cost_cache_dir", default=None,
                    help="Reuse a prebuilt cost matrix. The matrix depends only on "
                         "the scene, so one build serves every arm and seed of it.")
    ap.add_argument("--ga_iters", type=int, default=None, help="Default 100000.")
    ap.add_argument("--ga_defect", default="fixed", choices=["fixed", "broken"],
                    help="'broken' reproduces the original flip-mutation defect, so "
                         "the before/after comparison stays runnable.")
    ap.add_argument("--balance_max_seconds", type=float, default=20.0)
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
    # Which arm is this?
    #
    # The legacy predicate is deliberately explicit and deliberately total: if
    # ANY campaign flag is non-default we take the new path, and otherwise we
    # run the original code verbatim. Nothing is "the same code with different
    # parameters" -- that phrasing is how a byte-identity guarantee quietly
    # stops being true.
    # -------------------------------------------------------
    _is_legacy = (
        args.cluster == "kmeans"
        and args.cost == "euclidean"
        and args.objective == "minsum"
        and args.planner_seed is None
        and args.ga_iters is None
        and args.ga_defect == "fixed"
        and args.cost_cache_dir is None
    )
    arm_label = args.arm or ("A0_published" if _is_legacy else
                             f"{args.cluster}_{args.cost}_{args.objective}")
    stage_times = {}
    cost_meta = {}
    time_model_used = None
    labels_out = None
    uav_of_cluster_out = None
    n_unreachable = 0
    print(f"Arm            : {arm_label}"
          f"  (cluster={args.cluster} cost={args.cost} obj={args.objective}"
          f" seed={args.planner_seed} legacy={_is_legacy})")

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
    # Step 3: allocation
    #
    # The non-legacy arms run an entirely separate, INDEX-CARRYING path and
    # return early from this block, because the published path loses indices at
    # the first step: manual_kmeans_clustering returns coordinates, not labels
    # (DCKmeans.py:69). Recovering them afterwards would mean matching floats,
    # which is exactly the bug the new path exists to avoid.
    #
    # The new path also skips shift_to_positive entirely and works in ORIGINAL
    # coordinates. The shift exists for k-means numerical stability; k-medoids
    # on a precomputed matrix does not need it, and keeping one frame removes
    # the trap where the cost matrix is built where the obstacles are NOT.
    # -------------------------------------------------------
    if not _is_legacy:
        deckga_paths, dega_lengths, raw_lengths, centroids, labels_out, \
            uav_of_cluster_out, n_unreachable, cost_meta, _st = _run_arm(
                args=args,
                points=points,
                start_points=start_points,
                obstacle_xyz=obstacle_xyz,
                obs_size=obs_size,
                num_uavs=num_uavs,
                arm_label=arm_label,
            )
        stage_times.update(_st)
        offset_used = np.zeros(3, dtype=float)   # no shift on this path
        from planner import timing as _t
        time_model_used = (_t.TimeModel.parse(args.time_model) if args.time_model
                           else _t.GAZEBO_SPEED1)
    else:
        t0 = time.time()
        clusters, centroids = manual_kmeans_clustering(points_shifted, num_uavs)
        print(f"\nDCKmeans done in {time.time() - t0:.3f} s")
        print("Centroids:\n", centroids)
        stage_times["cluster_s"] = time.time() - t0

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

    # Legacy only. The arm path already produced deckga_paths above, in
    # original coordinates, with its indices carried through, so none of
    # the shifted-space machinery below applies to it.
    if _is_legacy:
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
        stage_times["ga_s"] = time.time() - t0

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
        # --- campaign additions, strictly additive ---------------------------
        # Nothing downstream reads these yet except the new metrics;
        # compute_all_metrics requires only 6 keys and _measure_quicknav_only 3,
        # so every existing consumer is unaffected.
        "arm": arm_label,
        "cluster_method": args.cluster,
        "cost_kind": args.cost,
        "objective": args.objective,
        "planner_seed": args.planner_seed,
        "ga_defect": args.ga_defect,
        "ga_iters": int(args.ga_iters) if args.ga_iters else int(1e5),
        "is_legacy": bool(_is_legacy),
        # The allocation itself, which DCKmeans never returned. Without it
        # nothing downstream can say WHICH waypoints a UAV was given, only
        # where it flew.
        #
        # `labels` is in UAV SPACE: `labels == i` are the waypoints flown by
        # UAV i, matching `deckga_paths[i]` and `start_points[i]`. The
        # cluster->UAV permutation has ALREADY been applied (see _run_arm), so
        # do not index through `uav_of_cluster` again -- that is kept only as
        # provenance, recording which k-medoids cluster id became which UAV.
        # -1 marks a node that is not an assigned waypoint (a start point, or a
        # waypoint dropped as unreachable).
        "labels": labels_out,
        "uav_of_cluster": uav_of_cluster_out,
        # Not previously saved at all, so nothing could verify which start went
        # with which path.
        "start_points": start_points,
        "time_model": (time_model_used.as_dict() if time_model_used else None),
        "cost_matrix_meta": cost_meta,
        "n_unreachable_waypoints": int(n_unreachable),
        # Real per-stage timings, so the harness stops inferring DECK-GA time by
        # subtracting a re-measured QuickNav time and a 0.3 s guess.
        "stage_times_s": stage_times,
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
