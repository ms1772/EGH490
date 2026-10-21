"""Pure metric functions consumed by sweep_offline.py.

No I/O, no subprocess, no ROS. Safe to unit-test with hand-rolled fixtures.

Re-uses upstream functions where possible:
  - QuickNav_detection_3D.obstacle_detection for residual collision counting
  - QuickNav_detection_3D.line_intersects_cube for segment-level checks
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, List, Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from QuickNav_detection_3D import line_intersects_cube, obstacle_detection  # noqa: E402


def deduplicate_preserve_order(path: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Identical to QuickNav.apply_quicknav's nested deduplicate_preserve_order.

    Removes any point that is approximately equal to ANY earlier point in the
    route (not just the immediately preceding one). This is what QuickNav does
    to its output, so we apply it to deckga_paths too before comparing distance
    — otherwise the TSP tour's return-to-start (a duplicate of the first point)
    gets removed only from quicknav_paths, producing a misleading negative
    'avoidance penalty'.
    """
    p = np.asarray(path, dtype=float)
    if len(p) == 0:
        return p
    out = [p[0]]
    for q in p[1:]:
        if not any(np.allclose(q, r, atol=eps) for r in out):
            out.append(q)
    return np.array(out)


def path_distance(path: np.ndarray, dedup: bool = False) -> float:
    """Cumulative Euclidean distance along an Nx3 waypoint sequence.

    If dedup=True, applies QuickNav-style dedup first. Use this when comparing
    a deckga_path against a quicknav_path (which has already been dedup'd
    internally by apply_quicknav).
    """
    p = np.asarray(path, dtype=float)
    if dedup:
        p = deduplicate_preserve_order(p)
    if len(p) < 2:
        return 0.0
    diffs = np.diff(p, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def fleet_distance(paths: Sequence[np.ndarray]) -> float:
    return float(sum(path_distance(p) for p in paths))


def detections_per_uav(detected_indices: Sequence[Sequence[int]]) -> List[int]:
    """Number of obstacle detections per UAV — taken directly from the pickle."""
    return [int(len(idxs)) for idxs in detected_indices]


def avoidances_per_uav(
    deckga_paths: Sequence[np.ndarray],
    quicknav_paths: Sequence[np.ndarray],
) -> List[int]:
    """Inserted vertex count per UAV: len(quicknav) - len(deckga). Each detour
    inserts 2 vertices in the typical case; reported as raw inserted-vertex count
    to match the §4.2 phrasing of 'avoidance applications'."""
    out = []
    for d, q in zip(deckga_paths, quicknav_paths):
        out.append(int(max(0, len(q) - len(d))))
    return out


def coerce_sizes(obs_size, n: int) -> np.ndarray:
    """Coerce obs_size to length-n array. Accepts scalar, (n,), or (n, 3)."""
    arr = np.asarray(obs_size)
    if arr.ndim == 0:
        return np.full(n, float(arr))
    if arr.ndim == 1 and len(arr) == n:
        return arr
    if arr.ndim == 2 and arr.shape == (n, 3):
        return arr
    raise ValueError(f"Unsupported obs_size shape {arr.shape} for n={n} obstacles")


def residual_collisions_per_uav(
    quicknav_paths: Sequence[np.ndarray],
    obstacle_xyz: np.ndarray,
    obs_size,
) -> List[int]:
    """Re-run obstacle_detection on the QuickNav-avoided paths. Any non-zero
    return is an avoidance failure (bug A/B/C/D regression)."""
    obstacle_xyz = np.asarray(obstacle_xyz)
    n_obs = len(obstacle_xyz)
    if n_obs == 0:
        return [0 for _ in quicknav_paths]
    sizes = coerce_sizes(obs_size, n_obs)
    out = []
    for path in quicknav_paths:
        _, new_ob, _ = obstacle_detection(
            np.asarray(path, dtype=float),
            obstacle_xyz,
            sizes,
            visualize=False,
        )
        out.append(int(len(new_ob)))
    return out


def _time_model_from(pickle_data: dict):
    """The time model the planner used, or the campaign default.

    Single source of truth: planner.timing. Redefining v_eff/t_stop here would
    let the planner optimise against one pair of constants while the report
    scores against another, and every number would be quietly wrong.
    """
    from planner.timing import GAZEBO_SPEED1, TimeModel

    tm = pickle_data.get("time_model")
    if isinstance(tm, dict) and "v_eff" in tm:
        return TimeModel(v_eff=float(tm["v_eff"]), t_stop=float(tm.get("t_stop", 0.0)))
    return GAZEBO_SPEED1


def _balance_ratio(values) -> float:
    """max / mean. Scale-free, so it compares across problem sizes."""
    v = [float(x) for x in values if x is not None]
    if not v:
        return 0.0
    mean = sum(v) / len(v)
    return (max(v) / mean) if mean > 0 else 1.0


# Keys compute_all_metrics exports that are scalars, i.e. CSV-able. Used by
# sweep_offline to assert its header covers them -- DictWriter(extrasaction=
# "ignore") drops unknown keys SILENTLY, so a new metric that nobody adds to
# the header simply never appears and nothing complains.
CSV_EXPORTED_KEYS = [
    "num_uavs",
    "distance_baseline_total_m", "distance_avoided_total_m",
    "detections_total", "avoidances_total", "residual_collisions_total",
    "max_tour_baseline_m", "max_tour_avoided_m",
    "makespan_baseline_s", "makespan_avoided_s",
    "balance_ratio_baseline", "balance_ratio_avoided",
    "wp_avoided_max",
    "time_model_v_eff", "time_model_t_stop",
]


def compute_all_metrics(pickle_data: dict) -> Dict[str, object]:
    """Given a loaded DECK_GA_QuickNav output dict, compute every metric.

    Required pickle keys: deckga_paths, quicknav_paths, obstacle_xyz, obs_size,
    detected_indices, num_uavs.
    """
    deckga_paths = pickle_data["deckga_paths"]
    quicknav_paths = pickle_data["quicknav_paths"]
    obstacle_xyz = np.asarray(pickle_data["obstacle_xyz"]) if len(pickle_data["obstacle_xyz"]) > 0 \
        else np.zeros((0, 3))
    obs_size = pickle_data["obs_size"]
    detected_indices = pickle_data["detected_indices"]
    num_uavs = int(pickle_data["num_uavs"])

    # dedup=True on baseline because apply_quicknav internally dedups the route
    # it returns; comparing against a non-dedup'd baseline is apples-to-oranges
    # and produces spurious negative penalties when the TSP tour's return-to-start
    # leg gets stripped only on one side.
    dist_baseline = [path_distance(p, dedup=True) for p in deckga_paths]
    dist_avoided = [path_distance(p) for p in quicknav_paths]
    detect_n = detections_per_uav(detected_indices)
    avoid_n = avoidances_per_uav(deckga_paths, quicknav_paths)
    residual_n = residual_collisions_per_uav(quicknav_paths, obstacle_xyz, obs_size)

    # --- makespan -----------------------------------------------------------
    # The fleet metric. UAVs fly concurrently, so the mission ends when the
    # SLOWEST one lands -- the max, never the sum. Total distance hides this
    # completely: the committed run gz_n30_o0_s0 flew 329 / 322 / 526 m, a
    # makespan 34% above the balanced mean, and the totals look unremarkable.
    #
    # The time model comes from the pickle when the planner recorded one, so
    # the metric is scored with the SAME constants the planner optimised
    # against. Falling back to two different numbers in two places is how a
    # campaign ends up reporting something nobody optimised.
    tm = _time_model_from(pickle_data)
    wp_baseline = [max(0, len(np.asarray(p)) - 1) for p in deckga_paths]
    wp_avoided = [max(0, len(np.asarray(p)) - 1) for p in quicknav_paths]
    time_baseline = [tm.tour_time(d, n) for d, n in zip(dist_baseline, wp_baseline)]
    time_avoided = [tm.tour_time(d, n) for d, n in zip(dist_avoided, wp_avoided)]

    out = {
        "num_uavs": num_uavs,
        "distance_baseline_per_uav_m": dist_baseline,
        "distance_avoided_per_uav_m": dist_avoided,
        "distance_baseline_total_m": float(sum(dist_baseline)),
        "distance_avoided_total_m": float(sum(dist_avoided)),
        "detections_per_uav": detect_n,
        "detections_total": int(sum(detect_n)),
        "avoidances_per_uav": avoid_n,
        "avoidances_total": int(sum(avoid_n)),
        "residual_collisions_per_uav": residual_n,
        "residual_collisions_total": int(sum(residual_n)),

        # makespan in distance and in calibrated time
        "max_tour_baseline_m": float(max(dist_baseline)) if dist_baseline else 0.0,
        "max_tour_avoided_m": float(max(dist_avoided)) if dist_avoided else 0.0,
        "makespan_baseline_s": float(max(time_baseline)) if time_baseline else 0.0,
        "makespan_avoided_s": float(max(time_avoided)) if time_avoided else 0.0,
        "tour_time_baseline_per_uav_s": time_baseline,
        "tour_time_avoided_per_uav_s": time_avoided,

        # how unevenly the work was shared; 1.0 is perfect
        "balance_ratio_baseline": _balance_ratio(dist_baseline),
        "balance_ratio_avoided": _balance_ratio(dist_avoided),

        # waypoint counts drive the t_stop term and change under balancing
        "wp_baseline_per_uav": wp_baseline,
        "wp_avoided_per_uav": wp_avoided,
        "wp_avoided_max": int(max(wp_avoided)) if wp_avoided else 0,

        "time_model_v_eff": tm.v_eff,
        "time_model_t_stop": tm.t_stop,
    }
    return out


def penetration_depth(segment_a: np.ndarray, segment_b: np.ndarray,
                       cube_center: np.ndarray, cube_half_size,
                       n_samples: int = 20) -> float:
    """Max distance from any sample on segment a->b to the nearest cube face,
    measured only at samples that are INSIDE the cube. Returns 0.0 if the
    segment never enters the cube.

    cube_half_size: scalar or 3-vector.
    """
    a = np.asarray(segment_a, dtype=float)
    b = np.asarray(segment_b, dtype=float)
    c = np.asarray(cube_center, dtype=float)
    if np.isscalar(cube_half_size):
        h = np.array([float(cube_half_size)] * 3)
    else:
        h = np.asarray(cube_half_size, dtype=float)

    pmin = c - h
    pmax = c + h

    ts = np.linspace(0.0, 1.0, n_samples)
    max_depth = 0.0
    for t in ts:
        p = a + (b - a) * t
        if np.all(p >= pmin) and np.all(p <= pmax):
            # Distance to nearest face along each axis is min(p - pmin, pmax - p).
            face_dist = np.minimum(p - pmin, pmax - p).min()
            if face_dist > max_depth:
                max_depth = float(face_dist)
    return max_depth
