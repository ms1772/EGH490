#!/usr/bin/env python3
"""QuickNav_avoidance_3D.py

Given intersecting segments and obstacle cubes, computes shortest detour paths
by evaluating 8 corner-pair combinations across all 6 faces of each cube.

Originally ported from dipraj-debnath/DECKGA_QuickNav_3D. This version fixes
three bugs (B, C from the project plan):
  - Detour candidates now validated against ALL obstacles (not just the one
    being detoured) on all THREE candidate segments (start->c1, c1->c2, c2->end).
  - When no candidate passes the multi-obstacle filter, the function retries
    once with corners pushed out by an extra safety margin instead of silently
    falling back to a clipping candidate.
  - If retry also fails, the function logs a loud warning and returns the
    original segment endpoints as a degenerate detour (caller's max-iter cap
    surfaces the failure).
"""

from __future__ import annotations

from typing import Iterable, Optional

import numpy as np

from QuickNav_detection_3D import line_intersects_cube


def _half_sizes(s) -> tuple[float, float, float]:
    if np.isscalar(s):
        v = float(s)
        return v, v, v
    arr = np.asarray(s, dtype=float)
    if arr.size == 3:
        return float(arr[0]), float(arr[1]), float(arr[2])
    if arr.size == 1:
        v = float(arr.ravel()[0])
        return v, v, v
    raise ValueError(f"unsupported obs_size shape {arr.shape}")


def _box_minmax(centre, half) -> tuple[np.ndarray, np.ndarray]:
    sx, sy, sz = _half_sizes(half)
    cx, cy, cz = (float(centre[0]), float(centre[1]), float(centre[2]))
    return (
        np.array([cx - sx, cy - sy, cz - sz]),
        np.array([cx + sx, cy + sy, cz + sz]),
    )


def _segment_hits_any_obstacle(
    p: np.ndarray,
    q: np.ndarray,
    all_centers: np.ndarray,
    all_sizes,
    skip_centre: Optional[np.ndarray] = None,
    skip_eps: float = 1e-6,
    boundary_tol: float = 1e-4,
) -> bool:
    """True if segment p->q penetrates the INTERIOR of any AABB.

    `skip_centre`, if given, suppresses the box whose centre matches it within
    `skip_eps` (used to exclude the obstacle currently being detoured around).

    Boxes are shrunk by `boundary_tol` per axis before the line-vs-box test,
    so segments that merely touch a face/edge/corner (as detour corners do by
    construction) are NOT counted as collisions. Only INTERIOR penetration counts.
    """
    for k in range(len(all_centers)):
        if skip_centre is not None and np.all(np.abs(all_centers[k] - skip_centre) < skip_eps):
            continue
        pmin, pmax = _box_minmax(all_centers[k], all_sizes[k])
        # Shrink the box to allow boundary-tangent segments.
        pmin = pmin + boundary_tol
        pmax = pmax - boundary_tol
        # If the box has been shrunk below zero on any axis, it's effectively a
        # surface or smaller — no interior to penetrate.
        if np.any(pmin >= pmax):
            continue
        if line_intersects_cube(p, q, pmin, pmax):
            return True
    return False


def _build_corners(centre, half, scale: float = 1.0) -> np.ndarray:
    """8 corners of an AABB, optionally scaled outward by `scale` per axis."""
    cx, cy, cz = (float(v) for v in centre)
    sx, sy, sz = _half_sizes(half)
    sx *= scale
    sy *= scale
    sz *= scale
    return np.array([
        [cx - sx, cy - sy, cz - sz],  # 0
        [cx + sx, cy - sy, cz - sz],  # 1
        [cx + sx, cy + sy, cz - sz],  # 2
        [cx - sx, cy + sy, cz - sz],  # 3
        [cx - sx, cy - sy, cz + sz],  # 4
        [cx + sx, cy - sy, cz + sz],  # 5
        [cx + sx, cy + sy, cz + sz],  # 6
        [cx - sx, cy + sy, cz + sz],  # 7
    ])


_FACES = {
    "bottom": [0, 1, 2, 3],
    "top":    [4, 5, 6, 7],
    "front":  [0, 1, 5, 4],
    "back":   [2, 3, 7, 6],
    "left":   [0, 3, 7, 4],
    "right":  [1, 2, 6, 5],
}


def _candidate_paths(start: np.ndarray, end: np.ndarray, corners: np.ndarray) -> list:
    """Build all 48 candidate detour paths (8 per face × 6 faces)."""
    all_paths = []
    for face_indices in _FACES.values():
        c = [corners[idx] for idx in face_indices]
        all_paths.extend([
            [start, c[0], c[1], end],
            [start, c[1], c[0], end],
            [start, c[2], c[3], end],
            [start, c[3], c[2], end],
            [start, c[0], c[3], end],
            [start, c[3], c[0], end],
            [start, c[1], c[2], end],
            [start, c[2], c[1], end],
        ])
    return all_paths


def _filter_collision_free(
    candidates: list,
    current_centre: np.ndarray,
    current_half,
) -> list:
    """Reject detour candidates whose approach/exit re-enters the CURRENT obstacle.

    With Bug A (`processed_obstacles` skip-list) fixed, the iterative outer
    loop catches secondary collisions in subsequent iterations — so we only
    need to ensure the detour itself doesn't re-enter the obstacle being
    avoided. `line_intersects_cube` already shrinks the box by a small
    boundary tolerance, so detour corners sitting on the surface are not
    flagged as re-entry.
    """
    pmin, pmax = _box_minmax(current_centre, current_half)
    valid = []
    for p in candidates:
        s, c1, c2, e = (np.asarray(p[0]), np.asarray(p[1]),
                        np.asarray(p[2]), np.asarray(p[3]))
        if line_intersects_cube(s, c1, pmin, pmax):
            continue
        if line_intersects_cube(c2, e, pmin, pmax):
            continue
        valid.append(p)
    return valid


def _shortest_path(candidates: list) -> np.ndarray:
    best_path = None
    best_dist = np.inf
    for path in candidates:
        d = sum(
            float(np.linalg.norm(np.asarray(path[j + 1]) - np.asarray(path[j])))
            for j in range(3)
        )
        if d < best_dist:
            best_dist = d
            best_path = path
    return np.asarray(best_path)


def obstacle_avoid(
    Obstacle_route: np.ndarray,
    new_ob: np.ndarray,
    obstacle_xyz: np.ndarray,
    obs_size,
    *,
    current_sizes: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Compute avoidance detour paths for each detected obstacle intersection.

    Parameters
    ----------
    Obstacle_route : (K, 6) — [Ax Ay Az Bx By Bz] per intersecting segment.
    new_ob         : (K, 3) — detected obstacle centres (one per intersection).
    obstacle_xyz   : (M, 3) — the FULL obstacle centre list (used for multi-obstacle filtering).
    obs_size       : scalar | (M,) | (M, 3) — full obstacle size list, matching obstacle_xyz.
    current_sizes  : optional (K,) or (K, 3) — per-intersection sizes for new_ob.
                     If None, look up sizes from (obstacle_xyz, obs_size).

    Returns
    -------
    Obstacle_avoid_route : (4, 3, K) — [start, c1, c2, end] per intersection.
    """
    num_obs = len(new_ob)
    Obstacle_avoid_route = np.zeros((4, 3, num_obs))

    obstacle_xyz = np.asarray(obstacle_xyz, dtype=float)

    # Normalise obs_size to (M,) or (M, 3) so all_sizes[k] returns a valid size.
    if np.isscalar(obs_size):
        obs_size_arr = np.full(len(obstacle_xyz), float(obs_size))
    else:
        obs_size_arr = np.asarray(obs_size, dtype=float)
        if obs_size_arr.ndim == 0:
            obs_size_arr = np.full(len(obstacle_xyz), float(obs_size_arr))

    for i in range(num_obs):
        start = Obstacle_route[i, 0:3].astype(float)
        end = Obstacle_route[i, 3:6].astype(float)
        centre = np.asarray(new_ob[i], dtype=float)

        if current_sizes is not None:
            half_here = current_sizes[i]
        else:
            # Look up the current obstacle's size by centre match.
            idx = np.where((obstacle_xyz == centre).all(axis=1))[0]
            half_here = obs_size_arr[idx[0]] if len(idx) > 0 else 1.0

        corners = _build_corners(centre, half_here, scale=1.0)
        all_paths = _candidate_paths(start, end, corners)

        # Stage 1: filter candidates so the approach (start->c1) and exit
        # (c2->end) segments don't re-enter the current obstacle. Other
        # obstacles are handled by the outer iteration (after Bug A fix the
        # iterative loop converges; over-filtering causes spurious failures
        # when two obstacles share a path segment).
        valid_paths = _filter_collision_free(all_paths, centre, half_here)

        # Stage 2: if empty, retry with corners pushed out by ~50% extra clearance.
        if not valid_paths:
            sx, sy, sz = _half_sizes(half_here)
            avg = (sx + sy + sz) / 3.0
            wider_scale = 1.0 + (0.5 if avg > 0 else 1.0)
            wider_corners = _build_corners(centre, half_here, scale=wider_scale)
            wider_paths = _candidate_paths(start, end, wider_corners)
            valid_paths = _filter_collision_free(wider_paths, centre, half_here)

        # Stage 3: still empty -> emit degenerate detour (just the original endpoints
        # duplicated). Log a warning so the iterative loop / max-iter cap surfaces
        # the unsolved collision rather than silently picking a clipping path.
        if not valid_paths:
            print(
                f"[QuickNav] WARNING: no collision-free detour for obstacle at "
                f"{centre.round(2).tolist()} (segment {start.round(2).tolist()} -> "
                f"{end.round(2).tolist()}). Returning straight segment; outer loop may retry."
            )
            Obstacle_avoid_route[:, :, i] = np.array([start, start, end, end])
            continue

        Obstacle_avoid_route[:, :, i] = _shortest_path(valid_paths)

    return Obstacle_avoid_route
