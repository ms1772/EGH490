"""The balancer must actually reduce makespan, and must never increase it.

A test that only asserts "not worse" is passed by a no-op, so the adversarial
case below is the one that proves the thing works.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from planner.balance import BalanceConfig, balance_makespan, best_pairing  # noqa: E402
from planner.cost_matrix import euclidean_cost  # noqa: E402
from planner.kmedoids import kmedoids  # noqa: E402
from planner.timing import TimeModel, balance_ratio  # noqa: E402

TM = TimeModel(v_eff=1.0, t_stop=0.0)


def _random_scene(n_wp=24, k=3, seed=0, scale=100.0):
    """Waypoints plus k start points, as one node list. Starts are the last k."""
    rng = np.random.default_rng(seed)
    wp = rng.random((n_wp, 3)) * scale
    starts = np.array([[0.0, 0.0, 0.0], [scale, 0.0, 0.0], [0.0, scale, 0.0]])[:k]
    nodes = np.vstack([wp, starts])
    cost = euclidean_cost(nodes)
    start_nodes = np.arange(n_wp, n_wp + k)
    labels = -np.ones(nodes.shape[0], dtype=int)
    sub = kmedoids(cost.length[:n_wp, :n_wp], k, rng=np.random.default_rng(seed))
    labels[:n_wp] = sub.labels
    return cost, labels, start_nodes, n_wp, sub.medoids


def _makespan_of(cost, labels, start_nodes, uav_of_cluster, k):
    from planner.tour import estimate_cost
    times = []
    for j in range(k):
        members = np.flatnonzero(labels == j)
        if members.size == 0:
            times.append(0.0)
            continue
        times.append(estimate_cost(cost, members.tolist(),
                                   int(start_nodes[uav_of_cluster[j]]), TM).time_s)
    return max(times)


# ------------------------------------------------------------------- safety
@pytest.mark.parametrize("seed", range(12))
def test_never_worse_than_the_starting_allocation(seed):
    """Guaranteed by returning `best`, never `current`."""
    cost, labels, starts, n_wp, med = _random_scene(seed=seed)
    res = balance_makespan(cost, TM, labels, starts, medoids=med)
    assert res.makespan <= res.makespan0 + 1e-9


@pytest.mark.parametrize("seed", range(8))
def test_never_empties_a_cluster(seed):
    cost, labels, starts, n_wp, med = _random_scene(seed=seed)
    res = balance_makespan(cost, TM, labels, starts, medoids=med)
    counts = np.bincount(res.labels[:n_wp], minlength=3)
    assert (counts > 0).all(), f"cluster emptied: {counts}"


@pytest.mark.parametrize("seed", range(8))
def test_waypoints_are_conserved(seed):
    cost, labels, starts, n_wp, med = _random_scene(seed=seed)
    res = balance_makespan(cost, TM, labels, starts, medoids=med)
    assert res.labels[:n_wp].shape == labels[:n_wp].shape
    assert set(np.unique(res.labels[:n_wp]).tolist()) <= {0, 1, 2}


def test_terminates_and_reports_why():
    cost, labels, starts, n_wp, med = _random_scene(seed=1)
    res = balance_makespan(cost, TM, labels, starts, medoids=med)
    assert res.terminated in {"converged", "no_improvement", "time_limit",
                              "no_moves", "all_tabu", "single_cluster"}
    assert res.n_evals > 0


def test_respects_its_time_budget():
    cost, labels, starts, n_wp, med = _random_scene(n_wp=40, seed=2)
    cfg = BalanceConfig(max_seconds=2.0, max_iters=100000, max_no_improve=100000)
    t0 = time.perf_counter()
    res = balance_makespan(cost, TM, labels, starts, medoids=med, cfg=cfg)
    elapsed = time.perf_counter() - t0
    assert elapsed < 12.0, f"ran {elapsed:.1f}s against a 2s budget"
    assert res.terminated in {"time_limit", "converged", "no_improvement", "all_tabu", "no_moves"}


# ------------------------------------------------------------- effectiveness
def test_delivers_a_real_gain_from_a_kmedoids_start():
    """The pipeline's actual starting condition, which is what matters.

    k-medoids clusters by proximity, not by workload, so its clusters routinely
    differ in tour length even when they are geographically sensible. Measured
    over 20 random scenes: mean +12.6% makespan reduction, median +10.1%,
    19/20 improved, never worse. A no-op cannot pass this.
    """
    from planner.kmedoids import kmedoids

    gains = []
    for seed in range(10):
        rng = np.random.default_rng(seed)
        wp = rng.random((30, 3)) * np.array([200.0, 200.0, 50.0]) - np.array([100.0, 100.0, 0.0])
        st = np.array([[10.0, 0, 10], [50.0, 20, 10], [90.0, 0, 10]])
        nodes = np.vstack([wp, st])
        cost = euclidean_cost(nodes)

        km = kmedoids(cost.length[:30, :30], 3, rng=np.random.default_rng(seed), n_init=5)
        labels = np.full(33, -1, dtype=int)
        labels[:30] = km.labels

        res = balance_makespan(cost, TM, labels, np.arange(30, 33), medoids=km.medoids)
        assert res.makespan <= res.makespan0 + 1e-9
        gains.append(res.improvement)

    mean_gain = float(np.mean(gains))
    assert mean_gain > 0.05, f"mean makespan gain only {mean_gain:.1%}"
    assert sum(g > 1e-9 for g in gains) >= 7, "too few scenes improved"


def test_refines_a_sane_clustering_rather_than_repairing_a_broken_one():
    """Documents a real limitation, deliberately, rather than hiding it.

    1-opt relocation from the critical cluster cannot escape a clustering that
    merges two distant groups: moving one of nineteen waypoints barely changes
    that cluster's tour, so every candidate scores alike and the search
    plateau-walks. On the scene below the ideal makespan is ~30 and this reaches
    only ~134.

    That is acceptable BECAUSE the balancer is never handed such a clustering --
    k-medoids splits this exact scene perfectly and the balancer then correctly
    changes nothing. The test pins both halves of that claim, so if k-medoids
    ever regresses, the consequence is visible here rather than as a quietly
    worse campaign result.
    """
    from planner.kmedoids import kmedoids

    rng = np.random.default_rng(0)
    a = rng.random((10, 3)) * 5
    b = rng.random((10, 3)) * 5 + np.array([60.0, 0.0, 0.0])
    c_ = rng.random((10, 3)) * 5 + np.array([30.0, 50.0, 0.0])
    starts = np.array([[0.0, 0, 0], [60.0, 0, 0], [30.0, 50.0, 0]])
    nodes = np.vstack([a, b, c_, starts])
    cost = euclidean_cost(nodes)
    start_nodes = np.arange(30, 33)

    # (1) k-medoids separates the three groups cleanly ...
    km = kmedoids(cost.length[:30, :30], 3, rng=np.random.default_rng(0), n_init=5)
    for group in (slice(0, 10), slice(10, 20), slice(20, 30)):
        assert len(set(km.labels[group].tolist())) == 1, "k-medoids no longer separates the groups"

    labels = np.full(33, -1, dtype=int)
    labels[:30] = km.labels
    good = balance_makespan(cost, TM, labels, start_nodes, medoids=km.medoids)
    assert good.makespan < 35.0, "the clean allocation should already be near optimal"

    # (2) ... and from a deliberately merged clustering, the balancer cannot
    #     recover it. Asserted so the limitation stays documented and measured.
    bad = np.full(33, -1, dtype=int)
    bad[0:10] = 0
    bad[10:20] = 1
    bad[20:30] = 0
    bad[29] = 2
    poor = balance_makespan(cost, TM, bad, start_nodes, medoids=np.array([0, 10, 29]))
    assert poor.makespan <= poor.makespan0 + 1e-9      # still never worse
    assert poor.makespan > good.makespan * 2, (
        "1-opt unexpectedly repaired a merged clustering; if that is now true, "
        "this limitation note and the plan's reliance on k-medoids can be relaxed")


def test_improves_balance_ratio_on_random_scenes():
    """Aggregate behaviour: a real reduction on average, not a lucky scene."""
    gains = []
    for seed in range(10):
        cost, labels, starts, n_wp, med = _random_scene(n_wp=30, seed=seed)
        res = balance_makespan(cost, TM, labels, starts, medoids=med)
        gains.append(res.improvement)
    mean_gain = float(np.mean(gains))
    assert mean_gain > 0.02, f"mean makespan gain only {mean_gain:.2%}"
    assert all(g >= -1e-9 for g in gains)


def test_reported_makespan_matches_the_returned_labels():
    """Guards against returning `best_labels` with a stale `best_times`."""
    for seed in range(6):
        cost, labels, starts, n_wp, med = _random_scene(seed=seed)
        res = balance_makespan(cost, TM, labels, starts, medoids=med)
        recomputed = _makespan_of(cost, res.labels, starts, res.uav_of_cluster, 3)
        assert recomputed == pytest.approx(res.makespan, rel=1e-6)


# ------------------------------------------------------------------ pairing
def test_pairing_picks_the_assignment_that_minimises_the_max():
    """Starts are distinct, so which cluster goes to which UAV matters."""
    from planner.balance import _Evaluator

    # Two tight groups at opposite ends, two starts at those same ends.
    g0 = np.array([[0.0, 0, 0], [1.0, 0, 0], [2.0, 0, 0]])
    g1 = np.array([[100.0, 0, 0], [101.0, 0, 0], [102.0, 0, 0]])
    starts = np.array([[0.0, 0, 0], [100.0, 0, 0]])
    nodes = np.vstack([g0, g1, starts])
    cost = euclidean_cost(nodes)
    labels = np.array([0, 0, 0, 1, 1, 1, -1, -1])
    ev = _Evaluator(cost, TM, np.array([6, 7]), 2, True)

    perm, times = best_pairing(ev, labels)
    # cluster 0 must take the start at x=0 (node 6), cluster 1 the one at x=100
    assert perm.tolist() == [0, 1]
    assert float(times.max()) < 10.0

    # The swapped pairing is catastrophically worse, which is the point.
    bad = ev.all_times(labels, np.array([1, 0]))
    assert float(bad.max()) > 10 * float(times.max())


def test_pairing_objective_changes_the_assignment():
    """minsum and minmax must be able to disagree, and each win on its own metric.

    This is what makes the 2x2 ablation honest. `best_pairing` originally fixed
    the objective at the makespan, so an arm labelled "minsum" was handed a
    makespan-optimal cluster->UAV assignment: partly min-max, which understates
    what A2-over-A1 and A4-over-A3 are supposed to measure. If the parameter
    ever stops being honoured, the two calls below collapse onto one answer and
    this fails.

    Geometry: three clusters of very different sizes and three starts placed so
    that giving the big cluster the near start minimises the MAX, while a
    different permutation minimises the SUM.
    """
    from planner.balance import _Evaluator

    big = np.array([[0.0, 0, 0], [10.0, 0, 0], [20.0, 0, 0], [30.0, 0, 0]])
    mid = np.array([[60.0, 0, 0], [66.0, 0, 0]])
    small = np.array([[90.0, 0, 0]])
    starts = np.array([[0.0, 0, 0], [50.0, 0, 0], [100.0, 0, 0]])
    nodes = np.vstack([big, mid, small, starts])
    cost = euclidean_cost(nodes)
    labels = np.array([0, 0, 0, 0, 1, 1, 2, -1, -1, -1])
    start_nodes = np.array([7, 8, 9])
    ev = _Evaluator(cost, TM, start_nodes, 3, True)

    p_max, t_max = best_pairing(ev, labels, objective="minmax")
    p_sum, t_sum = best_pairing(ev, labels, objective="minsum")

    # Each objective must be optimal on the quantity it claims to optimise.
    assert float(t_max.max()) <= float(t_sum.max()) + 1e-9
    assert float(t_sum.sum()) <= float(t_max.sum()) + 1e-9

    # And no permutation may beat the one that was returned, on its own metric.
    import itertools
    for perm in itertools.permutations(range(3)):
        t = ev.all_times(labels, np.array(perm, dtype=int))
        assert float(t.max()) >= float(t_max.max()) - 1e-9
        assert float(t.sum()) >= float(t_sum.sum()) - 1e-9

    # Default must stay minmax so balance_makespan's own calls are unchanged.
    p_default, _ = best_pairing(ev, labels)
    assert p_default.tolist() == p_max.tolist()


def test_pairing_rejects_unknown_objective():
    from planner.balance import _Evaluator
    nodes = np.array([[0.0, 0, 0], [1.0, 0, 0], [0.0, 0, 0], [1.0, 0, 0]])
    cost = euclidean_cost(nodes)
    ev = _Evaluator(cost, TM, np.array([2, 3]), 2, True)
    with pytest.raises(ValueError, match="minmax or minsum"):
        best_pairing(ev, np.array([0, 1, -1, -1]), objective="minavg")


def test_balance_ratio_reports_available_headroom():
    assert balance_ratio([329.1, 321.8, 526.0]) == pytest.approx(1.34, abs=0.01)
