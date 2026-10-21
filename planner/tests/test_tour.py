"""The cheap estimator must be fast, correct, and -- critically -- correlated
with the GA, since the balancer uses it as a stand-in for what will actually fly.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from planner.cost_matrix import euclidean_cost  # noqa: E402
from planner.timing import TimeModel  # noqa: E402
from planner.tour import (  # noqa: E402
    build_tour,
    estimate_cost,
    ga_tour,
    nn_tour,
    path_cost,
    two_opt,
)

TM = TimeModel(v_eff=1.0, t_stop=0.0)


def _scene(n, seed=0):
    pts = np.random.default_rng(seed).random((n, 3)) * 100.0
    return pts, euclidean_cost(pts)


# ------------------------------------------------------------------ structure
def test_nn_tour_visits_every_node_once_from_the_start():
    _, c = _scene(12)
    seq = nn_tour(c.length, range(12), start=5)
    assert seq[0] == 5
    assert sorted(seq) == list(range(12))


def test_two_opt_pins_position_zero():
    _, c = _scene(14, seed=2)
    seq = build_tour(c.length, range(14), start=3)
    assert seq[0] == 3
    assert sorted(seq) == list(range(14))


def test_two_opt_never_worsens_the_tour():
    for seed in range(8):
        _, c = _scene(15, seed=seed)
        start = nn_tour(c.length, range(15), start=0)
        improved = two_opt(c.length, start)
        assert path_cost(c.length, improved) <= path_cost(c.length, start) + 1e-9


def test_two_opt_untangles_a_known_crossing():
    """A unit square visited in a crossing order; 2-opt must find the perimeter."""
    pts = np.array([[0.0, 0, 0], [1.0, 0, 0], [1.0, 1, 0], [0.0, 1, 0]])
    c = euclidean_cost(pts)
    crossed = [0, 2, 1, 3]            # length 2 + 2*sqrt(2)
    fixed = two_opt(c.length, crossed)
    assert path_cost(c.length, fixed) == pytest.approx(4.0)


def test_two_opt_refuses_an_asymmetric_matrix():
    """The delta evaluation is only valid on symmetric costs."""
    c = np.array([[0.0, 1.0, 5.0],
                  [9.0, 0.0, 1.0],
                  [1.0, 1.0, 0.0]])
    with pytest.raises(ValueError, match="not symmetric"):
        two_opt(c, [0, 1, 2, 0][:3])


def test_tiny_inputs_are_handled():
    _, c = _scene(3)
    assert two_opt(c.length, [0]) == [0]
    assert two_opt(c.length, [0, 1]) == [0, 1]
    assert path_cost(c.length, [0]) == 0.0


def test_open_and_closed_costs_differ_by_the_return_leg():
    pts = np.array([[0.0, 0, 0], [3.0, 0, 0], [3.0, 4, 0]])
    c = euclidean_cost(pts)
    seq = [0, 1, 2]
    assert path_cost(c.length, seq, closed=False) == pytest.approx(7.0)
    assert path_cost(c.length, seq, closed=True) == pytest.approx(12.0)


# -------------------------------------------------------------------- pricing
def test_estimate_prices_detour_vertices_as_extra_stops():
    from planner.cost_matrix import CostMatrices

    m = 3
    length = np.array([[0.0, 10.0, 10.0],
                       [10.0, 0.0, 10.0],
                       [10.0, 10.0, 0.0]])
    extra = np.zeros((m, m), dtype=np.int16)
    extra[0, 1] = extra[1, 0] = 2
    cost = CostMatrices(kind="quicknav", length=length, extra_vertices=extra,
                        infeasible=np.zeros((m, m), bool), penalty=0.0, meta={})

    tm = TimeModel(v_eff=1.0, t_stop=3.0)
    est = estimate_cost(cost, [0, 1, 2], start=0, time_model=tm, closed=True)
    # 3 legs plus the 2 detour vertices on the 0-1 edge
    assert est.n_extra_vertices == 2
    assert est.time_s == pytest.approx(est.length / 1.0 + (est.n_seg + 2) * 3.0)


def test_estimate_counts_infeasible_edges_it_was_forced_through():
    from planner.cost_matrix import CostMatrices

    m = 3
    infeas = np.zeros((m, m), bool)
    infeas[0, 1] = infeas[1, 0] = True
    cost = CostMatrices(kind="quicknav", length=np.ones((m, m)) * 5.0,
                        extra_vertices=np.zeros((m, m), np.int16),
                        infeasible=infeas, penalty=50.0, meta={})
    np.fill_diagonal(cost.length, 0.0)
    est = estimate_cost(cost, [0, 1, 2], start=0, time_model=TM)
    assert est.n_infeasible_edges >= 1


def test_estimate_handles_a_single_node_cluster():
    _, c = _scene(5)
    est = estimate_cost(c, [2], start=2, time_model=TM)
    assert est.time_s == 0.0
    assert est.n_seg == 0


def test_estimate_inserts_the_start_when_absent():
    _, c = _scene(8)
    est = estimate_cost(c, [1, 2, 3], start=7, time_model=TM)
    assert est.seq[0] == 7
    assert set(est.seq) == {7, 1, 2, 3}


# ------------------------------------------------------- against the real GA
def test_estimator_is_fast():
    _, c = _scene(34, seed=11)
    t0 = time.perf_counter()
    for _ in range(20):
        estimate_cost(c, range(34), start=0, time_model=TM)
    per_call = (time.perf_counter() - t0) / 20
    assert per_call < 0.25, f"{per_call*1000:.1f} ms per estimate is too slow for the balance loop"


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_estimator_tracks_the_ga(seed):
    """Within a sane band of the real optimiser.

    If the estimator ranked allocations differently from the GA, the balancer
    would be optimising a quantity nobody flies.
    """
    pts, c = _scene(20, seed=seed)
    est = estimate_cost(c, range(20), start=0, time_model=TM)
    res = ga_tour(c, range(20), start=0, num_iter=4000,
                  rng=np.random.default_rng(seed), points_xyz=pts)
    ratio = est.length / res.opt_cost
    assert 0.80 < ratio < 1.35, f"estimator/GA ratio {ratio:.3f} out of band"


def test_ga_tour_returns_caller_node_ids():
    """The GA works on a submatrix with local indices; they must be mapped back."""
    pts, c = _scene(12, seed=4)
    nodes = [9, 4, 7, 1, 3]
    res = ga_tour(c, nodes, start=9, num_iter=300,
                  rng=np.random.default_rng(0), points_xyz=pts)
    assert set(res.route_indices.tolist()) <= set(nodes)
    assert res.route_indices[0] == 9
    assert res.route_indices[-1] == 9          # closed
