"""Frozen, verbatim copies of the planner internals as they stood on 2026-10-08.

NOT imported from the live modules on purpose. These are the reference the
vectorisation and the flip-defect fix are measured against, and they have to
stay valid after those modules change -- which rules out `import`, and rules out
`git revert` as an A/B mechanism (a revert-based comparison stops being
reproducible the moment it is merged).

Nothing in the repo should import this except tests. Do not "keep it in sync":
the whole point is that it does not move.

Source: GA_path_planning.py lines 1-99 and DCKmeans.py lines 1-70, copied with
only the renames below.
"""
from __future__ import annotations

import numpy as np


# ---------------------------------------------------------------------------
# GA_path_planning.ga_3d_pathplanning, frozen 2026-10-08.
# Changes from the original: renamed; `verbose` added to silence the two
# unconditional print blocks at the end (they are I/O, not algorithm); the
# `flip_broken` switch exposes the defect at :59-64 so the A/B is reproducible.
# The RNG call sequence is otherwise byte-for-byte identical.
# ---------------------------------------------------------------------------
def ga_3d_pathplanning_reference(xyz, *, num_iter=int(1e5), flip_broken=True,
                                 verbose=False):
    N = xyz.shape[0]
    popSize = 16
    numIter = int(num_iter)

    a = np.meshgrid(np.arange(N), np.arange(N))
    dmat = np.sqrt(np.sum((xyz[a[0], :] - xyz[a[1], :]) ** 2, axis=2))

    popSize = 4 * int(np.ceil(popSize / 4))
    numIter = max(1, round(numIter))

    pop = np.zeros((popSize, N), dtype=int)
    pop[0, :] = np.arange(N)
    for k in range(1, popSize):
        pop[k, :] = np.random.permutation(N)

    globalMin = np.inf
    distHistory = np.zeros(numIter)
    optRoute = pop[0, :].copy()

    for it in range(numIter):
        totalDist = np.zeros(popSize)

        for p in range(popSize):
            d = dmat[pop[p, -1], pop[p, 0]]
            for k in range(1, N):
                d += dmat[pop[p, k - 1], pop[p, k]]
            totalDist[p] = d

        minDist = np.min(totalDist)
        index = np.argmin(totalDist)
        distHistory[it] = minDist

        if minDist < globalMin:
            globalMin = minDist
            optRoute = pop[index, :]

        randomOrder = np.random.permutation(popSize)
        newPop = np.zeros((popSize, N), dtype=int)
        for p in range(0, popSize, 4):
            rtes = pop[randomOrder[p:p + 4], :]
            dists = totalDist[randomOrder[p:p + 4]]
            idx = np.argmin(dists)
            bestOf4Route = rtes[idx, :]
            routeInsertionPoints = np.sort(np.random.randint(0, N, size=2))
            I, J = routeInsertionPoints

            tmpPop = np.zeros((4, N), dtype=int)
            tmpPop[0, :] = bestOf4Route

            # THE DEFECT. The original writes only the reversed slice into a row
            # that is still zero-filled, so every index outside [I, J] stays 0,
            # the integrity guard below sees duplicates, and the child becomes a
            # uniform random permutation ~97% of the time at N=35. Compare with
            # the swap and slide rows, which both .copy() the parent first.
            if flip_broken:
                if I < J:
                    tmpPop[1, I:J + 1] = bestOf4Route[I:J + 1][::-1]
                elif I > J:
                    tmpPop[1, J:I + 1] = bestOf4Route[J:I + 1][::-1]
                else:
                    tmpPop[1, :] = bestOf4Route
            else:
                tmpPop[1, :] = bestOf4Route.copy()
                tmpPop[1, I:J + 1] = bestOf4Route[I:J + 1][::-1]

            tmpPop[2, :] = bestOf4Route.copy()
            tmpPop[2, [I, J]] = bestOf4Route[[J, I]]

            tmpPop[3, :] = bestOf4Route.copy()
            tmpPop[3, I:J + 1] = np.roll(bestOf4Route[I:J + 1], shift=-1)

            for i in range(4):
                if len(np.unique(tmpPop[i, :])) != N:
                    tmpPop[i, :] = np.random.permutation(N)

            newPop[p:p + 4, :] = tmpPop
        pop = newPop

    final_route = xyz[optRoute, :]
    start_point = np.where(np.all(final_route == xyz[0], axis=1))[0][0]
    rearranged_route = np.concatenate(
        (final_route[start_point:], final_route[:start_point + 1])
    )

    if verbose:
        print("\n--- GA Input ---")
        print("Waypoints:")
        print(xyz)
        print("\n--- GA Output ---")
        print("Optimized Route:")
        print(rearranged_route)

    return rearranged_route


def ga_totaldist_reference(pop, dmat, popSize, N):
    """The tour-length evaluator exactly as GA_path_planning.py:30-37 computes it.

    Broken out so the vectorisation can be compared bitwise without running a
    whole GA. The accumulation ORDER is the thing under test: the closing edge
    is added first, then each forward edge in sequence.
    """
    totalDist = np.zeros(popSize)
    for p in range(popSize):
        d = dmat[pop[p, -1], pop[p, 0]]
        for k in range(1, N):
            d += dmat[pop[p, k - 1], pop[p, k]]
        totalDist[p] = d
    return totalDist


# ---------------------------------------------------------------------------
# DCKmeans.manual_kmeans_clustering, frozen 2026-10-08. Verbatim apart from the
# rename and dropping the convergence print.
# ---------------------------------------------------------------------------
def _initialize_centroids_kmeans_plus_plus_reference(points, num_clusters):
    centroids = [points[np.random.choice(len(points))]]
    for _ in range(1, num_clusters):
        distances = np.min(
            [np.linalg.norm(points - c, axis=1) for c in centroids], axis=0
        )
        probabilities = distances / np.sum(distances)
        cumulative_probabilities = np.cumsum(probabilities)
        r = np.random.rand()
        for i, p in enumerate(cumulative_probabilities):
            if r < p:
                centroids.append(points[i])
                break
    return np.array(centroids)


def manual_kmeans_clustering_reference(points, num_clusters,
                                       max_iterations=100000, tolerance=1e-4):
    np.random.seed(42)

    centroids = _initialize_centroids_kmeans_plus_plus_reference(points, num_clusters)
    prev_centroids = np.zeros_like(centroids)

    for _iteration in range(max_iterations):
        distances = np.linalg.norm(points[:, np.newaxis] - centroids, axis=2)
        labels = np.argmin(distances, axis=1)

        # Built as Python lists, exactly as the original. This is what makes an
        # empty cluster come back as np.array([]) with shape (0,) rather than
        # (0, 3) -- the shape that breaks downstream cluster[:, 0] indexing.
        clusters = [[] for _ in range(num_clusters)]
        for i, label in enumerate(labels):
            clusters[label].append(points[i])

        centroids = []
        for cluster in clusters:
            if len(cluster) > 0:
                centroids.append(np.mean(cluster, axis=0))
            else:
                centroids.append(np.zeros(3))
        centroids = np.array(centroids)

        centroid_shift = np.linalg.norm(centroids - prev_centroids)
        if centroid_shift < tolerance:
            break
        prev_centroids = centroids.copy()

    clusters = [np.array(cluster) for cluster in clusters]
    return clusters, centroids
