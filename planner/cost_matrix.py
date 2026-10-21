"""Pairwise cost oracle: what it really costs to fly from waypoint i to waypoint j.

DECK-GA measures every edge with a ruler. Two waypoints on opposite sides of an
obstacle are "adjacent" to k-means and to the GA, and QuickNav only discovers
otherwise afterwards -- by which time the allocation and the visiting order are
already fixed. This module answers the same question with the obstacle field
taken into account, by asking QuickNav itself for the avoided route between each
pair and measuring that.

The result drops straight into `ga_3d_pathplanning(..., dmat=...)`, because the
GA's search loop touches only the cost matrix and integer permutations.

--------------------------------------------------------------------------
THE FOUR THINGS THAT WILL BITE YOU
--------------------------------------------------------------------------

1. AN UNSOLVABLE PAIR LOOKS EXACTLY LIKE A FREE ONE.
   When QuickNav cannot find a detour it emits the degenerate route
   [start, start, end, end] (QuickNav_avoidance_3D.py:248), and the final
   `deduplicate_preserve_order` (QuickNav.py:151) collapses that back to
   [start, end] -- a straight line of exactly Euclidean length. Verified
   end-to-end: a segment through a wall returns 2 points, length 10.0, against a
   Euclidean 10.0, with 1 residual detection.

   So cost alone CANNOT distinguish "nothing in the way" from "impossible", and
   a TSP solver handed that matrix would preferentially route through walls.
   Every returned polyline is therefore re-run through detection, and anything
   still colliding is flagged infeasible and given a large FINITE penalty.
   Finite, not np.inf: an inf would leave the GA's globalMin at inf.

2. SYMMETRY IS MANDATORY, NOT COSMETIC.
   The GA's flip operator is a segment reversal, and 2-opt's delta evaluation is
   only valid when C[i,j] == C[j,i]. QuickNav is *almost* symmetric -- measured
   0/780 asymmetric pairs on a 40-point scene -- but not guaranteed:
   `_shortest_path` breaks ties with a strict `<` and keeps the first candidate
   in enumeration order, and the sort at QuickNav.py:93 is a non-stable
   quicksort. Computing only the upper triangle and mirroring makes symmetry
   exact by construction and halves the work.

3. THE MATRIX IS NOT A METRIC.
   Measured 346 triangle violations on that same scene, worst excess 11% of the
   direct cost. Detours must route via two corners of ONE obstacle's AABB and
   face-diagonal corner pairs are excluded, so cost(i,k) can exceed
   cost(i,j) + cost(j,k). The GA does not care -- it has no construction
   heuristic that assumes the triangle inequality -- but k-medoids quality
   bounds and any insertion heuristic do. Recorded in meta, asserted nowhere.

4. THE SUM OF PAIRWISE COSTS IS A SURROGATE, NOT THE FLOWN COST.
   `apply_quicknav` on a whole tour installs only one detour per segment per
   iteration, and detour corners become real waypoints subject to further
   detection. So sum(C[i,j]) over a tour differs from running QuickNav on that
   tour. It is a far better surrogate than Euclidean, but it must be validated
   against the real thing rather than assumed equal.

--------------------------------------------------------------------------
COORDINATE FRAME -- read this before using it in DECK_GA_QuickNav.py
--------------------------------------------------------------------------
That script shifts waypoints and start points into positive space and then
clusters and runs the GA in SHIFTED coordinates, while obstacles stay in
ORIGINAL coordinates throughout. Euclidean distance is invariant under that
translation, which is why it has never mattered. Detour cost is not.

Build this matrix in ORIGINAL coordinates. Indices are frame-invariant, so the
GA can still run on shifted points; only the costs must be computed where the
obstacles actually are. Getting this wrong does not raise -- it makes every pair
look clear, silently degrading the obstacle-aware arms into Euclidean ones.
`assert_detours_found` exists to catch exactly that.
"""
from __future__ import annotations

import contextlib
import hashlib
import io
import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

COST_EUCLIDEAN = "euclidean"
COST_QUICKNAV = "quicknav"


# ---------------------------------------------------------------------------
def _detect(route, obstacle_xyz, obs_size):
    """Collision detection with visualize=False pinned.

    `obstacle_detection` defaults visualize=True (QuickNav_detection_3D.py:14).
    In WSL with no DISPLAY matplotlib falls back to Agg and plt.show() returns
    immediately, so forgetting the keyword does not hang or raise -- it silently
    builds a 3D figure per pair across thousands of pairs. Never call upstream
    directly from this module.
    """
    from QuickNav_detection_3D import obstacle_detection

    return obstacle_detection(route, obstacle_xyz, obs_size, visualize=False)


def normalise_obs_size(obs_size, n_obstacles: int) -> np.ndarray:
    """Coerce to (M, 3) float64 at the boundary.

    QuickNav.py:75 indexes obs_size, OUTSIDE its try block, so a Python float
    raises TypeError only once the first collision occurs -- i.e. a scene with no
    detours passes and a cluttered one explodes.
    """
    arr = np.asarray(obs_size, dtype=float)
    if arr.ndim == 0:
        return np.full((n_obstacles, 3), float(arr))
    if arr.ndim == 1:
        if arr.size == n_obstacles:
            return np.tile(arr[:, None], (1, 3))
        if arr.size == 3:
            return np.tile(arr[None, :], (n_obstacles, 1))
        raise ValueError(f"obs_size shape {arr.shape} matches neither {n_obstacles} nor 3")
    if arr.shape == (n_obstacles, 3):
        return arr.astype(float, copy=True)
    raise ValueError(f"obs_size must be scalar, (M,), (3,) or (M,3); got {arr.shape}")


@dataclass
class CostMatrices:
    """Symmetric pairwise costs over a fixed node list.

    length         (m, m) float64  route length, penalty where infeasible
    extra_vertices (m, m) int16    detour vertices QuickNav inserts
    infeasible     (m, m) bool     still collides after avoidance
    """

    kind: str
    length: np.ndarray
    extra_vertices: np.ndarray
    infeasible: np.ndarray
    penalty: float
    meta: dict = field(default_factory=dict)

    @property
    def n(self) -> int:
        return int(self.length.shape[0])

    def time_matrix(self, time_model) -> np.ndarray:
        """Convert to flight time. Each detour vertex is an extra stop."""
        return (self.length / time_model.v_eff
                + self.extra_vertices.astype(float) * time_model.t_stop)

    def submatrix(self, idx) -> np.ndarray:
        """Costs restricted to `idx`, for handing one cluster to the GA."""
        idx = np.asarray(idx, dtype=int)
        return self.length[np.ix_(idx, idx)]

    def unreachable_nodes(self) -> np.ndarray:
        """Nodes whose every edge is infeasible -- walled in, not merely blocked.

        These poison an allocation: the penalty makes whichever UAV owns one
        permanently the critical UAV, and a min-max balancer then thrashes trying
        to move it. The planner's own overlap check catches "inside a box" but
        not "surrounded by boxes".
        """
        if self.n < 2:
            return np.zeros(0, dtype=int)
        # The diagonal is set True, not False, so that `.all(axis=1)` tests only
        # the off-diagonal entries. Masking it to False instead would put a
        # guaranteed False in every row and the predicate could never fire --
        # silently reporting that nothing is ever walled in.
        masked = self.infeasible.copy()
        np.fill_diagonal(masked, True)
        return np.flatnonzero(masked.all(axis=1))

    def assert_detours_found(self, n_obstacles: int) -> None:
        """Guard against the coordinate-frame trap described in the module docstring."""
        if self.kind != COST_QUICKNAV or n_obstacles <= 0:
            return
        if int(self.meta.get("n_detour_edges", 0)) == 0:
            raise ValueError(
                "quicknav cost matrix found ZERO detours across every pair, with "
                f"{n_obstacles} obstacles present. Expected roughly 12% of pairs at "
                "M=20 and 45% at M=40. The usual cause is building the matrix in "
                "SHIFTED coordinates while the obstacles are in ORIGINAL ones, which "
                "makes every segment look clear and silently turns this arm into the "
                "Euclidean one."
            )


# ---------------------------------------------------------------------------
def euclidean_cost(nodes_xyz) -> CostMatrices:
    p = np.asarray(nodes_xyz, dtype=float)
    m = p.shape[0]
    d = np.sqrt(((p[:, None, :] - p[None, :, :]) ** 2).sum(-1))
    np.fill_diagonal(d, 0.0)
    return CostMatrices(
        kind=COST_EUCLIDEAN,
        length=d,
        extra_vertices=np.zeros((m, m), dtype=np.int16),
        infeasible=np.zeros((m, m), dtype=bool),
        penalty=0.0,
        meta={"kind": COST_EUCLIDEAN, "n_nodes": m, "n_detour_edges": 0,
              "n_infeasible_edges": 0, "n_triangle_violations": 0,
              "build_s": 0.0, "cache_hit": False},
    )


def quicknav_cost(nodes_xyz, obstacle_xyz, obs_size, *, penalty_scale: float = 10.0,
                  count_triangle_violations: bool = True,
                  progress=None) -> CostMatrices:
    """Pairwise avoided-route cost. Upper triangle only, then mirrored."""
    from QuickNav import apply_quicknav

    t0 = time.perf_counter()
    p = np.asarray(nodes_xyz, dtype=float)
    m = p.shape[0]
    obstacle_xyz = np.asarray(obstacle_xyz, dtype=float).reshape(-1, 3)
    sizes = normalise_obs_size(obs_size, len(obstacle_xyz))

    length = np.zeros((m, m), dtype=float)
    extra = np.zeros((m, m), dtype=np.int16)
    infeas = np.zeros((m, m), dtype=bool)

    n_detour = 0
    n_infeasible = 0

    # QuickNav prints on its degenerate-detour and max-iteration paths. Those are
    # swallowed here ONLY because feasibility is established by re-detection
    # below rather than by reading stdout.
    sink = io.StringIO()
    for i in range(m):
        for j in range(i + 1, m):
            seg = np.array([p[i], p[j]], dtype=float)

            if len(obstacle_xyz) == 0:
                route = seg
            else:
                with contextlib.redirect_stdout(sink):
                    _, hits, _ = _detect(seg, obstacle_xyz, sizes)
                    if len(hits) == 0:
                        route = seg          # clear: skip avoidance entirely
                    else:
                        route = np.asarray(apply_quicknav(seg, obstacle_xyz, sizes),
                                           dtype=float)
                        _, residual, _ = _detect(route, obstacle_xyz, sizes)
                        if len(residual) > 0:
                            infeas[i, j] = infeas[j, i] = True
                            n_infeasible += 1

            if route.shape[0] < 2:           # coincident endpoints
                route = seg
            d = float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum())
            v = max(0, route.shape[0] - 2)
            if v > 0:
                n_detour += 1

            length[i, j] = length[j, i] = d
            extra[i, j] = extra[j, i] = v
        if progress is not None:
            progress(i + 1, m)

    np.fill_diagonal(length, 0.0)

    # The penalty is scaled off the problem's own geometry -- the largest
    # straight-line separation in the node set -- not off the feasible route
    # lengths. Those can be empty or tiny (a two-node scene whose only pair is
    # blocked has NO feasible off-diagonal entries), and a penalty that collapses
    # to the same magnitude as a real edge is worse than useless: the solver
    # would route through the wall for free.
    span = float(np.sqrt(((p[:, None, :] - p[None, :, :]) ** 2).sum(-1)).max()) if m > 1 else 0.0
    penalty = float(penalty_scale) * (span if span > 0 else 1.0)
    length[infeas] = penalty

    n_tri = _count_triangle_violations(length) if count_triangle_violations else -1

    return CostMatrices(
        kind=COST_QUICKNAV,
        length=length,
        extra_vertices=extra,
        infeasible=infeas,
        penalty=penalty,
        meta={
            "kind": COST_QUICKNAV,
            "n_nodes": m,
            "n_obstacles": int(len(obstacle_xyz)),
            "n_pairs": m * (m - 1) // 2,
            "n_detour_edges": n_detour,
            "n_infeasible_edges": n_infeasible,
            "n_unreachable_nodes": 0,     # filled by the caller after construction
            "n_triangle_violations": n_tri,
            "penalty": penalty,
            "penalty_scale": float(penalty_scale),
            "build_s": time.perf_counter() - t0,
            "cache_hit": False,
        },
    )


def _count_triangle_violations(c: np.ndarray, tol: float = 1e-9) -> int:
    """How often C[i,k] > C[i,j] + C[j,k]. Informational; O(m^3) but vectorised."""
    m = c.shape[0]
    if m > 220:          # the full O(m^3) broadcast would be multi-GB
        return -1
    via = c[:, :, None] + c[None, :, :]        # (i, j, k)
    best = via.min(axis=1)
    return int(((c - best) > tol).sum())


def build(kind: str, nodes_xyz, obstacle_xyz=None, obs_size=None, **kw) -> CostMatrices:
    if kind == COST_EUCLIDEAN:
        return euclidean_cost(nodes_xyz)
    if kind == COST_QUICKNAV:
        if obstacle_xyz is None:
            raise ValueError("quicknav cost needs obstacle_xyz")
        return quicknav_cost(nodes_xyz, obstacle_xyz, obs_size, **kw)
    raise ValueError(f"unknown cost kind {kind!r}; expected {COST_EUCLIDEAN} or {COST_QUICKNAV}")


# ------------------------------------------------------------------ caching
def cache_key(kind: str, nodes_xyz, obstacle_xyz, obs_size, penalty_scale: float) -> str:
    """Content hash. Points and obstacles are identical across every arm and
    planner seed of a scene, so one build serves all of them."""
    h = hashlib.sha256()
    h.update(kind.encode())
    for arr in (nodes_xyz, obstacle_xyz, obs_size):
        a = np.ascontiguousarray(np.asarray(arr, dtype=float))
        h.update(str(a.shape).encode())
        h.update(a.tobytes())
    h.update(f"{float(penalty_scale):.12g}".encode())
    return h.hexdigest()[:32]


def load_or_build(kind: str, nodes_xyz, obstacle_xyz, obs_size, cache_dir,
                  *, penalty_scale: float = 10.0, require_hit: bool = False,
                  **kw) -> CostMatrices:
    """Reuse a cached matrix when the scene matches, else build and store it.

    require_hit turns a miss into an error. Worth setting for a campaign: a
    silent miss means every trial rebuilds the matrix, and the only symptom is a
    sweep that takes all night.
    """
    if cache_dir is None:
        if require_hit:
            raise ValueError("require_hit=True but no cache_dir was given")
        return build(kind, nodes_xyz, obstacle_xyz, obs_size,
                     penalty_scale=penalty_scale, **kw)

    cache_dir = Path(cache_dir)
    key = cache_key(kind, nodes_xyz,
                    obstacle_xyz if obstacle_xyz is not None else np.zeros((0, 3)),
                    obs_size if obs_size is not None else 0.0,
                    penalty_scale)
    npz = cache_dir / f"cost_{kind}_{key}.npz"

    if npz.exists():
        z = np.load(npz, allow_pickle=False)
        meta = json.loads(str(z["meta_json"]))
        meta["cache_hit"] = True
        return CostMatrices(kind=kind, length=z["length"],
                            extra_vertices=z["extra_vertices"],
                            infeasible=z["infeasible"],
                            penalty=float(z["penalty"]), meta=meta)

    if require_hit:
        raise FileNotFoundError(
            f"no cached cost matrix at {npz}. Pre-warm the cache for every scene "
            f"before the sweep instead of letting trials build it."
        )

    c = build(kind, nodes_xyz, obstacle_xyz, obs_size,
              penalty_scale=penalty_scale, **kw)
    cache_dir.mkdir(parents=True, exist_ok=True)
    # np.savez_compressed APPENDS '.npz' when the name does not already end in
    # it, so a '.npz.tmp' target silently becomes '.npz.tmp.npz' and the rename
    # below fails. Write through an open handle, which it leaves alone.
    tmp = npz.with_name(npz.name + ".tmp")
    with tmp.open("wb") as fh:
        np.savez_compressed(fh, length=c.length, extra_vertices=c.extra_vertices,
                            infeasible=c.infeasible, penalty=c.penalty,
                            meta_json=json.dumps(c.meta))
    tmp.replace(npz)       # atomic, so a killed pre-warm cannot leave a torn file
    return c
