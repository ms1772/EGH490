"""k-medoids properties, including the three DCKmeans failures it removes."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from planner.cost_matrix import euclidean_cost  # noqa: E402
from planner.kmedoids import (  # noqa: E402
    assign_labels,
    clusters_from_labels,
    indices_from_labels,
    kmedoids,
)


def _scene(n, seed=0, scale=100.0):
    pts = np.random.default_rng(seed).random((n, 3)) * scale
    return pts, euclidean_cost(pts).length


# ------------------------------------------------------------------ contract
@pytest.mark.parametrize("k", [2, 3, 4, 6])
def test_never_produces_an_empty_cluster(k):
    """The property DCKmeans.py:58 fails, across many scenes."""
    for seed in range(40):
        _, c = _scene(30, seed=seed)
        res = kmedoids(c, k, rng=np.random.default_rng(seed), n_init=3)
        counts = np.bincount(res.labels, minlength=k)
        assert (counts > 0).all(), f"empty cluster at k={k}, seed={seed}: {counts}"


def test_seeding_always_returns_exactly_k_medoids():
    """DCKmeans' inverse-CDF loop can silently under-seed; this cannot."""
    for seed in range(200):
        _, c = _scene(12, seed=seed)
        res = kmedoids(c, 4, rng=np.random.default_rng(seed), n_init=1)
        assert len(set(res.medoids.tolist())) == 4


def test_handles_duplicate_points():
    """All-identical nodes make every distance zero -- the degenerate case that
    sends a probability-weighted sampler to NaN."""
    pts = np.zeros((10, 3))
    c = euclidean_cost(pts).length
    res = kmedoids(c, 3, rng=np.random.default_rng(0), n_init=2)
    assert len(set(res.medoids.tolist())) == 3
    assert (np.bincount(res.labels, minlength=3) > 0).all()


def test_medoid_belongs_to_its_own_cluster():
    for seed in range(20):
        _, c = _scene(25, seed=seed)
        res = kmedoids(c, 3, rng=np.random.default_rng(seed))
        for j, m in enumerate(res.medoids):
            assert res.labels[m] == j


def test_deterministic_given_the_same_generator():
    _, c = _scene(40, seed=3)
    a = kmedoids(c, 3, rng=np.random.default_rng(7))
    b = kmedoids(c, 3, rng=np.random.default_rng(7))
    assert np.array_equal(a.labels, b.labels)
    assert np.array_equal(a.medoids, b.medoids)


def test_does_not_touch_the_global_rng():
    """Critical. The published arm's reproducibility rests on DCKmeans seeding
    the GLOBAL stream with 42; a new module perturbing it would silently break
    that, and nothing else would notice."""
    _, c = _scene(20, seed=1)
    np.random.seed(12345)
    before = np.random.get_state()
    kmedoids(c, 3, rng=np.random.default_rng(0), n_init=3)
    after = np.random.get_state()
    assert before[0] == after[0]
    assert np.array_equal(before[1], after[1])
    assert before[2:] == after[2:]


def test_rejects_bad_arguments():
    _, c = _scene(5)
    with pytest.raises(ValueError):
        kmedoids(c, 0, rng=np.random.default_rng(0))
    with pytest.raises(ValueError):
        kmedoids(c, 6, rng=np.random.default_rng(0))
    with pytest.raises(ValueError):
        kmedoids(np.zeros((3, 4)), 2, rng=np.random.default_rng(0))


def test_k_equals_one_and_k_equals_n():
    _, c = _scene(6, seed=2)
    one = kmedoids(c, 1, rng=np.random.default_rng(0))
    assert set(one.labels.tolist()) == {0}
    full = kmedoids(c, 6, rng=np.random.default_rng(0))
    assert sorted(full.labels.tolist()) == list(range(6))


# ------------------------------------------------------------------- quality
def test_recovers_obviously_separated_groups():
    pts = np.vstack([
        np.random.default_rng(0).random((10, 3)) + np.array([0, 0, 0]),
        np.random.default_rng(1).random((10, 3)) + np.array([100, 0, 0]),
        np.random.default_rng(2).random((10, 3)) + np.array([0, 100, 0]),
    ])
    c = euclidean_cost(pts).length
    res = kmedoids(c, 3, rng=np.random.default_rng(0), n_init=5)
    for group in (range(0, 10), range(10, 20), range(20, 30)):
        assert len(set(res.labels[list(group)].tolist())) == 1


def test_cost_is_consistent_with_the_returned_labels():
    _, c = _scene(30, seed=5)
    res = kmedoids(c, 3, rng=np.random.default_rng(5))
    recomputed = sum(c[res.labels == j, m].sum() for j, m in enumerate(res.medoids))
    assert res.cost == pytest.approx(recomputed)


def test_more_inits_never_hurts():
    _, c = _scene(35, seed=8)
    one = kmedoids(c, 3, rng=np.random.default_rng(2), n_init=1)
    many = kmedoids(c, 3, rng=np.random.default_rng(2), n_init=8)
    assert many.cost <= one.cost + 1e-9


def test_tolerates_a_non_metric_cost_matrix():
    """The detour matrix violates the triangle inequality by construction."""
    rng = np.random.default_rng(0)
    n = 20
    c = rng.random((n, n)) * 10
    c = (c + c.T) / 2          # symmetric but emphatically not a metric
    np.fill_diagonal(c, 0.0)
    res = kmedoids(c, 3, rng=np.random.default_rng(1), n_init=3)
    assert (np.bincount(res.labels, minlength=3) > 0).all()


# -------------------------------------------------------------------- splits
def test_clusters_from_labels_always_returns_n_by_3():
    pts = np.random.default_rng(0).random((9, 3))
    labels = np.array([0, 0, 0, 1, 1, 1, 2, 2, 2])
    out = clusters_from_labels(pts, labels, 3)
    assert all(a.shape[1] == 3 for a in out)
    # The shape DCKmeans gets wrong on an empty cluster
    empty = clusters_from_labels(pts, np.zeros(9, dtype=int), 3)
    assert empty[1].shape == (0, 3)


def test_indices_from_labels_partition_the_nodes():
    labels = np.array([2, 0, 1, 2, 0])
    idx = indices_from_labels(labels, 3)
    assert idx[0].tolist() == [1, 4]
    assert idx[1].tolist() == [2]
    assert idx[2].tolist() == [0, 3]
    assert sorted(np.concatenate(idx).tolist()) == list(range(5))


def test_assign_labels_matches_nearest_medoid():
    _, c = _scene(20, seed=9)
    medoids = np.array([0, 7, 13])
    labels = assign_labels(c, medoids)
    for i in range(20):
        if i not in medoids:
            assert labels[i] == int(np.argmin(c[i, medoids]))
