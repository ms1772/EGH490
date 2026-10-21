"""k-medoids clustering over a precomputed cost matrix.

WHY NOT k-MEANS
---------------
k-means works by averaging coordinates, which only means something under
Euclidean distance. Hand it a detour-cost matrix and there is no "mean route".
k-medoids picks an actual node as each cluster's representative, so it works
with any dissimilarity -- which is the whole reason the obstacle-aware arms are
possible at all.

Three hazards in DCKmeans.py disappear as a side effect, and they are worth
stating because they are live bugs, not stylistic preferences:

* EMPTY CLUSTERS. DCKmeans.py:58 parks an empty cluster's centroid at
  np.zeros(3) -- in the shifted frame that is a CORNER of the bounding box, so
  on the next iteration it steals whatever points are nearest that corner. It
  also returns np.array([]) with shape (0,), not (0,3), which breaks any
  downstream `cluster[:, 0]`. Here a medoid is always a member of its own
  cluster, because C[m, m] == 0 and no other node can beat that, so a cluster can
  never empty. `flight_ops/lab_config/o134_mission/mission_spec.py:137-141`
  currently designs its waypoint layout by hand to dodge this.

* k-means++ WITH D^1 WEIGHTING. DCKmeans.py:17 samples proportional to distance,
  not squared distance. Weaker spread than the textbook algorithm it names.

* SILENT UNDER-SEEDING. The inverse-CDF loop at DCKmeans.py:20-23 can fall
  through without appending -- on float error, or when all distances are zero --
  returning FEWER than k centroids, so the planner silently runs with fewer
  UAVs than asked for. `rng.choice(..., p=...)` cannot do that.

DCKmeans.py is deliberately left untouched. It has five importers, one of which
decides which physical bay each aircraft flies in O-134.

NON-METRIC COSTS
----------------
The detour matrix violates the triangle inequality (measured: 346 violations on
a 40-point scene, worst excess 11%). k-medoids still terminates and still
decreases its objective monotonically -- it only ever compares "cost to this
medoid" against "cost to that medoid" and never composes two edges. The usual
quality BOUNDS do not survive, but the algorithm does.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class KMedoidsResult:
    labels: np.ndarray        # (n,) cluster index per node
    medoids: np.ndarray       # (k,) node index acting as each cluster's medoid
    cost: float               # sum of each node's cost to its medoid
    n_iter: int
    n_init_used: int


def assign_labels(c: np.ndarray, medoids) -> np.ndarray:
    """Nearest medoid for every node. Ties go to the lowest medoid index."""
    med = np.asarray(medoids, dtype=int)
    labels = np.argmin(c[:, med], axis=1).astype(int)
    # A medoid must belong to its own cluster. With duplicate or zero-cost nodes
    # the argmin above can hand it to an earlier medoid, which would let a
    # cluster empty out -- the exact failure this class exists to prevent.
    for j, m in enumerate(med):
        labels[m] = j
    return labels


def _total_cost(c: np.ndarray, labels: np.ndarray, medoids: np.ndarray) -> float:
    return float(sum(c[labels == j, m].sum() for j, m in enumerate(medoids)))


def _seed_medoids(c: np.ndarray, k: int, rng) -> np.ndarray:
    """k-medoids++ with D^2 weighting.

    rng.choice with an explicit probability vector cannot silently return fewer
    than k seeds, unlike the hand-rolled inverse CDF in DCKmeans.
    """
    n = c.shape[0]
    first = int(rng.integers(n))
    medoids = [first]
    for _ in range(1, k):
        d = c[:, medoids].min(axis=1)
        d2 = d ** 2
        d2[medoids] = 0.0
        total = d2.sum()
        if total <= 0.0:
            # Every remaining node is at zero cost from a chosen medoid
            # (duplicates). Fill deterministically rather than sampling.
            remaining = [i for i in range(n) if i not in medoids]
            medoids.append(int(remaining[0]))
        else:
            medoids.append(int(rng.choice(n, p=d2 / total)))
    return np.array(medoids, dtype=int)


def kmedoids(c: np.ndarray, k: int, *, rng, n_init: int = 5,
             max_iter: int = 100) -> KMedoidsResult:
    """Voronoi-iteration k-medoids. Deterministic given `rng`."""
    c = np.asarray(c, dtype=float)
    n = c.shape[0]
    if c.ndim != 2 or c.shape[1] != n:
        raise ValueError(f"cost matrix must be square, got {c.shape}")
    if not (1 <= k <= n):
        raise ValueError(f"need 1 <= k <= n; got k={k}, n={n}")

    best = None
    for init in range(n_init):
        medoids = _seed_medoids(c, k, rng)
        labels = assign_labels(c, medoids)
        prev_cost = np.inf

        for it in range(max_iter):
            # Update: the member minimising total cost to its own cluster.
            for j in range(k):
                members = np.flatnonzero(labels == j)
                if members.size == 0:
                    continue          # unreachable by construction; see docstring
                sub = c[np.ix_(members, members)]
                medoids[j] = int(members[np.argmin(sub.sum(axis=1))])

            labels = assign_labels(c, medoids)
            cost = _total_cost(c, labels, medoids)
            if cost >= prev_cost - 1e-12:
                prev_cost = cost
                break
            prev_cost = cost

        if best is None or prev_cost < best.cost:
            best = KMedoidsResult(labels=labels.copy(), medoids=medoids.copy(),
                                  cost=float(prev_cost), n_iter=it + 1,
                                  n_init_used=init + 1)

    return best


def clusters_from_labels(points_xyz, labels, k: int) -> list:
    """Split points by label, ALWAYS returning (m, 3) arrays.

    DCKmeans returns np.array([]) with shape (0,) for an empty cluster, which
    breaks `cluster[:, 0]` downstream. Shape is forced here even though
    k-medoids cannot produce an empty cluster, so the contract does not depend
    on that argument holding.
    """
    p = np.asarray(points_xyz, dtype=float)
    labels = np.asarray(labels, dtype=int)
    out = []
    for j in range(k):
        members = p[labels == j]
        out.append(members if members.size else np.zeros((0, 3), dtype=float))
    return out


def indices_from_labels(labels, k: int) -> list:
    """Node INDICES per cluster.

    Indices, not coordinates, are what the rest of the pipeline carries: the
    cost matrix is built in original coordinates while the GA may run on shifted
    ones, and recovering indices by float-matching coordinates afterwards is how
    that becomes a silent geometry bug.
    """
    labels = np.asarray(labels, dtype=int)
    return [np.flatnonzero(labels == j) for j in range(k)]
