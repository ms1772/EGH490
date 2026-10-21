"""Proves the frozen reference copies reproduce the live modules, today.

This test is only meaningful BEFORE the planner work lands -- it is the evidence
that `tests/reference_impl.py` is a faithful snapshot. Once GA_path_planning.py
is vectorised and the flip defect is fixed, the GA comparison here is expected
to fail and gets marked xfail with the step number that changed it. The k-means
comparison must keep passing forever, because DCKmeans.py is not being modified.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from tests.reference_impl import (  # noqa: E402
    ga_3d_pathplanning_reference,
    ga_totaldist_reference,
    manual_kmeans_clustering_reference,
)


@pytest.mark.parametrize("seed", [0, 1, 2])
@pytest.mark.parametrize("n", [6, 12])
def test_frozen_ga_matches_live_ga(seed, n):
    """Same global seed in, same tour out.

    Bounded at 400 generations since Step 3 added num_iter. Lockstep over 400
    generations implies lockstep forever: if totalDist is bitwise identical and
    no RNG call is added or removed, the two implementations stay in step by
    induction on the generation. The unbounded 1e5 version was confirmed
    identical once, by hand, before the vectorisation landed.
    """
    from GA_path_planning import ga_3d_pathplanning

    xyz = np.random.default_rng(seed).random((n, 3)) * 100.0

    np.random.seed(seed)
    live = ga_3d_pathplanning(xyz, num_iter=400, verbose=False, flip_broken=True)

    np.random.seed(seed)
    ref = ga_3d_pathplanning_reference(xyz, num_iter=400, flip_broken=True)

    assert np.array_equal(live, ref), "frozen GA reference has drifted from the live module"


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_frozen_kmeans_matches_live_kmeans(seed):
    from DCKmeans import manual_kmeans_clustering

    pts = np.random.default_rng(seed).random((30, 3)) * 100.0

    live_c, live_cent = manual_kmeans_clustering(pts, 3)
    ref_c, ref_cent = manual_kmeans_clustering_reference(pts, 3)

    assert np.array_equal(live_cent, ref_cent)
    assert len(live_c) == len(ref_c)
    for a, b in zip(live_c, ref_c):
        assert a.shape == b.shape, "empty-cluster shape differs -- (0,) vs (0,3) matters"
        assert np.array_equal(a, b)


def test_frozen_totaldist_matches_the_live_inner_loop():
    """The evaluator helper must reproduce GA_path_planning.py:30-37 exactly.

    Everything the vectorisation is measured against rests on this one.
    """
    rng = np.random.default_rng(0)
    for n in (4, 11, 35):
        xyz = rng.random((n, 3)) * 100.0
        a = np.meshgrid(np.arange(n), np.arange(n))
        dmat = np.sqrt(np.sum((xyz[a[0], :] - xyz[a[1], :]) ** 2, axis=2))
        pop = np.array([rng.permutation(n) for _ in range(16)])

        # Recomputed inline, the way the live module does it.
        expected = np.zeros(16)
        for p in range(16):
            d = dmat[pop[p, -1], pop[p, 0]]
            for k in range(1, n):
                d += dmat[pop[p, k - 1], pop[p, k]]
            expected[p] = d

        got = ga_totaldist_reference(pop, dmat, 16, n)
        assert got.tobytes() == expected.tobytes()
