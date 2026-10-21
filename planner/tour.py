"""Fast tour construction and improvement, for use INSIDE the balancing loop.

The balancer evaluates hundreds of candidate reallocations. Running the real GA
for each is not an option -- measured at roughly 12 s per cluster at 1e5
iterations, a few hundred candidates across three clusters would take hours. This
module provides a nearest-neighbour construction plus a first-improvement 2-opt
pass that lands within a few percent of the GA in about a millisecond.

TWO RULES THAT MATTER MORE THAN THE ALGORITHM
---------------------------------------------
1. SYMMETRY. The 2-opt delta below prices only the two edges that change,
   because reversing the interior of a segment leaves its internal cost
   untouched. That is true if and only if C[i,j] == C[j,i]. On an asymmetric
   matrix the delta silently misprices every move and the search converges to
   tours that are wrong by its own objective. `two_opt` therefore refuses a
   non-symmetric matrix rather than quietly producing nonsense.

2. ESTIMATOR CONSISTENCY. The same estimator must score the incumbent and every
   candidate. Mixing -- GA cost for the incumbent, this for the candidates --
   makes the incumbent look better than every alternative by construction, and
   the balancer silently becomes a no-op that reports success.

CLOSED VS OPEN
--------------
Closed by default, because the GA minimises a closed tour (its evaluator adds
the wrap edge, GA_path_planning.py) and the estimator's job is to RANK
allocations the way the GA eventually will. Note the flown avoided path is open
-- `apply_quicknav`'s final dedup strips the return leg -- so the reported
makespan and this estimate are not the same quantity, and should not be
conflated in the writeup.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


@dataclass
class TourEstimate:
    seq: list              # node indices, starting at the start node
    length: float          # total cost under the matrix it was built with
    n_extra_vertices: int  # detour vertices implied by the surrogate
    n_seg: int             # legs flown
    time_s: float
    n_infeasible_edges: int


def _check_symmetric(c: np.ndarray, tol: float = 1e-9) -> None:
    if c.ndim != 2 or c.shape[0] != c.shape[1]:
        raise ValueError(f"cost matrix must be square, got {c.shape}")
    if not np.allclose(c, c.T, atol=tol, rtol=0.0):
        raise ValueError(
            "cost matrix is not symmetric. 2-opt's delta evaluation prices only "
            "the two changed edges, which assumes reversing a segment leaves its "
            "interior cost unchanged -- false for an asymmetric matrix. Mirror "
            "the upper triangle before calling."
        )


def nn_tour(c: np.ndarray, nodes: Sequence[int], start: int) -> list:
    """Nearest-neighbour construction from `start` over `nodes`."""
    remaining = [int(n) for n in nodes if int(n) != int(start)]
    seq = [int(start)]
    cur = int(start)
    while remaining:
        nxt = min(remaining, key=lambda k: c[cur, k])
        remaining.remove(nxt)
        seq.append(nxt)
        cur = nxt
    return seq


def two_opt(c: np.ndarray, seq: Sequence[int], *, closed: bool = True,
            max_passes: int = 50, eps: float = 1e-12) -> list:
    """First-improvement 2-opt with position 0 pinned.

    Position 0 is the UAV's start point and is not a free variable.
    """
    _check_symmetric(c)
    t = [int(x) for x in seq]
    m = len(t)
    if m < 4:
        return t

    for _ in range(max_passes):
        improved = False
        for i in range(1, m - 1):
            for j in range(i + 1, m):
                a, b = t[i - 1], t[i]
                d_node = t[j]
                if j + 1 < m:
                    e = t[j + 1]
                elif closed:
                    e = t[0]
                else:
                    e = None

                if e is None:
                    # Open path, reversing the tail: only the entry edge changes.
                    delta = c[a, d_node] - c[a, b]
                else:
                    delta = (c[a, d_node] + c[b, e]) - (c[a, b] + c[d_node, e])

                if delta < -eps:
                    t[i:j + 1] = t[i:j + 1][::-1]
                    improved = True
        if not improved:
            break
    return t


def path_cost(c: np.ndarray, seq: Sequence[int], *, closed: bool = True) -> float:
    t = [int(x) for x in seq]
    if len(t) < 2:
        return 0.0
    total = float(sum(c[t[k], t[k + 1]] for k in range(len(t) - 1)))
    if closed:
        total += float(c[t[-1], t[0]])
    return total


def _sum_over(mat: np.ndarray, seq: Sequence[int], closed: bool):
    t = [int(x) for x in seq]
    if len(t) < 2:
        return 0
    total = sum(mat[t[k], t[k + 1]] for k in range(len(t) - 1))
    if closed:
        total += mat[t[-1], t[0]]
    return total


def build_tour(c: np.ndarray, nodes: Sequence[int], start: int, *,
               closed: bool = True, max_passes: int = 50) -> list:
    return two_opt(c, nn_tour(c, nodes, start), closed=closed, max_passes=max_passes)


def estimate_cost(cost, nodes: Sequence[int], start: int, time_model, *,
                  closed: bool = True, max_passes: int = 50) -> TourEstimate:
    """Construct a tour over `nodes` from `start` and price it in seconds.

    `cost` is a planner.cost_matrix.CostMatrices. Using it rather than a bare
    array is what lets the estimate charge for detour vertices as well as
    distance, and report how many infeasible edges the tour was forced through.
    """
    members = [int(n) for n in nodes]
    if int(start) not in members:
        members = [int(start)] + members

    if len(members) <= 1:
        return TourEstimate(seq=members, length=0.0, n_extra_vertices=0,
                            n_seg=0, time_s=0.0, n_infeasible_edges=0)

    seq = build_tour(cost.length, members, int(start),
                     closed=closed, max_passes=max_passes)
    length = path_cost(cost.length, seq, closed=closed)
    extra = int(_sum_over(cost.extra_vertices, seq, closed))
    infeasible = int(_sum_over(cost.infeasible.astype(int), seq, closed))
    n_seg = len(seq) - 1 + (1 if closed else 0)

    time_s = time_model.tour_time(length, n_seg + extra)
    return TourEstimate(seq=seq, length=length, n_extra_vertices=extra,
                        n_seg=n_seg, time_s=time_s,
                        n_infeasible_edges=infeasible)


def ga_tour(cost, nodes: Sequence[int], start: int, *, num_iter: int,
            rng=None, pop_size: int = 16, points_xyz: Optional[np.ndarray] = None):
    """Run the real GA over the sub-matrix for one cluster.

    Returns the GAResult from GA_path_planning. The sub-matrix slice is what
    makes an obstacle-aware tour possible at all: the GA never sees coordinates,
    only this matrix.
    """
    from GA_path_planning import ga_3d_pathplanning

    members = [int(n) for n in nodes]
    if int(start) not in members:
        members = [int(start)] + members
    else:                      # the GA rotates to begin at row 0, so start first
        members = [int(start)] + [n for n in members if n != int(start)]

    idx = np.asarray(members, dtype=int)
    sub = cost.length[np.ix_(idx, idx)]
    xyz = (np.zeros((len(idx), 3)) if points_xyz is None
           else np.asarray(points_xyz, dtype=float)[idx])

    res = ga_3d_pathplanning(xyz, dmat=sub, num_iter=num_iter, pop_size=pop_size,
                             rng=rng, return_route=True, verbose=False)
    # Map the GA's local indices back onto the caller's node ids.
    res.route_indices = idx[res.route_indices]
    return res
