"""Soundness of the pairwise cost oracle.

The load-bearing test here is `test_infeasible_pair_is_flagged_not_silently_euclidean`.
Every other property can be eyeballed; that one cannot, because an unsolvable
pair and a completely clear pair produce numerically IDENTICAL costs, and the
only thing separating them is the re-detection step.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from planner.cost_matrix import (  # noqa: E402
    COST_EUCLIDEAN,
    COST_QUICKNAV,
    build,
    cache_key,
    euclidean_cost,
    load_or_build,
    normalise_obs_size,
    quicknav_cost,
)
from planner.timing import TimeModel  # noqa: E402


# ------------------------------------------------------------------ fixtures
def _one_box_between():
    """Two nodes with a small box squarely between them -- solvable detour."""
    nodes = np.array([[0.0, 0.0, 1.0], [10.0, 0.0, 1.0]])
    centres = np.array([[5.0, 0.0, 1.0]])
    sizes = np.array([[1.0, 1.0, 1.0]])
    return nodes, centres, sizes


def _unsolvable():
    """The geometry from tests/test_quicknav_fixes.py:159-162.

    QuickNav cannot route around this and returns the ORIGINAL straight segment,
    whose length is exactly Euclidean.
    """
    nodes = np.array([[0.0, 0.0, 1.0], [10.0, 0.0, 1.0]])
    centres = np.array([[9.0, 0.0, 1.0]])
    sizes = np.array([[3.0, 3.0, 3.0]])
    return nodes, centres, sizes


# -------------------------------------------------------------- obs_size coercion
def test_normalise_obs_size_accepts_every_shape():
    assert normalise_obs_size(5.0, 3).shape == (3, 3)
    assert np.allclose(normalise_obs_size(5.0, 3), 5.0)
    assert normalise_obs_size(np.array([1.0, 2.0, 3.0]), 3).shape == (3, 3)
    assert normalise_obs_size(np.ones((4, 3)), 4).shape == (4, 3)


def test_normalise_obs_size_rejects_nonsense():
    with pytest.raises(ValueError):
        normalise_obs_size(np.ones((2, 2)), 2)


# ----------------------------------------------------------------- euclidean
def test_euclidean_is_symmetric_zero_diagonal():
    nodes = np.random.default_rng(0).random((12, 3)) * 50
    c = euclidean_cost(nodes)
    assert np.array_equal(c.length, c.length.T)
    assert np.allclose(np.diag(c.length), 0.0)
    assert c.extra_vertices.sum() == 0
    assert not c.infeasible.any()


# ------------------------------------------------------------------ quicknav
def test_clear_pair_equals_euclidean_exactly():
    nodes = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
    centres = np.array([[0.0, 50.0, 0.0]])          # far away
    c = quicknav_cost(nodes, centres, np.array([[1.0, 1.0, 1.0]]))
    assert c.length[0, 1] == pytest.approx(10.0, abs=1e-12)
    assert c.extra_vertices[0, 1] == 0
    assert not c.infeasible.any()


def test_detour_pair_is_longer_and_collision_free():
    nodes, centres, sizes = _one_box_between()
    c = quicknav_cost(nodes, centres, sizes)
    assert c.length[0, 1] > 10.0
    assert c.extra_vertices[0, 1] >= 1
    assert not c.infeasible[0, 1]


def test_infeasible_pair_is_flagged_not_silently_euclidean():
    """THE test. Without re-detection this pair reports a plain 10.0 -- the
    cheapest possible edge -- and a TSP solver would route straight through."""
    nodes, centres, sizes = _unsolvable()

    # Establish the premise: QuickNav really does hand back the bare segment.
    from QuickNav import apply_quicknav
    from planner.cost_matrix import _detect
    import contextlib, io
    with contextlib.redirect_stdout(io.StringIO()):
        route = np.asarray(apply_quicknav(nodes.copy(), centres, sizes), dtype=float)
        _, residual, _ = _detect(route, centres, sizes)
    assert route.shape[0] == 2
    assert float(np.linalg.norm(route[1] - route[0])) == pytest.approx(10.0)
    assert len(residual) > 0, "fixture no longer unsolvable; pick another geometry"

    c = quicknav_cost(nodes, centres, sizes, penalty_scale=10.0)
    assert c.infeasible[0, 1] and c.infeasible[1, 0]
    assert c.length[0, 1] == pytest.approx(c.penalty)
    assert c.length[0, 1] > 10.0
    assert c.meta["n_infeasible_edges"] == 1


def test_penalty_is_finite():
    """np.inf would leave the GA's globalMin at inf and optRoute unbound."""
    nodes, centres, sizes = _unsolvable()
    c = quicknav_cost(nodes, centres, sizes)
    assert np.all(np.isfinite(c.length))


def test_symmetry_is_exact_by_construction():
    rng = np.random.default_rng(3)
    nodes = rng.random((14, 3)) * 60
    centres = rng.random((6, 3)) * 60
    c = quicknav_cost(nodes, centres, np.full((6, 3), 4.0))
    assert np.array_equal(c.length, c.length.T)
    assert np.array_equal(c.extra_vertices, c.extra_vertices.T)
    assert np.array_equal(c.infeasible, c.infeasible.T)
    assert np.allclose(np.diag(c.length), 0.0)


def test_dominates_euclidean():
    """QuickNav can only lengthen a segment. A shorter one means the splice or
    the dedup corrupted an endpoint."""
    rng = np.random.default_rng(4)
    nodes = rng.random((12, 3)) * 60
    centres = rng.random((5, 3)) * 60
    c = quicknav_cost(nodes, centres, np.full((5, 3), 5.0))
    e = euclidean_cost(nodes)
    ok = ~c.infeasible
    assert np.all(c.length[ok] >= e.length[ok] - 1e-9)


def test_unreachable_nodes_detected():
    """A node every edge of which is infeasible pins whichever UAV owns it as
    permanently critical, so it has to be found before clustering.

    Unit-tested on a hand-built matrix rather than via QuickNav geometry: the
    predicate is what needs proving, and constructing a genuinely walled-in node
    out of AABBs is fiddly and would make the test about the fixture instead.
    """
    from planner.cost_matrix import CostMatrices

    m = 4
    infeas = np.zeros((m, m), dtype=bool)
    infeas[2, :] = True          # node 2 cannot reach anything
    infeas[:, 2] = True
    np.fill_diagonal(infeas, False)
    c = CostMatrices(kind=COST_QUICKNAV, length=np.ones((m, m)),
                     extra_vertices=np.zeros((m, m), dtype=np.int16),
                     infeasible=infeas, penalty=99.0, meta={})
    assert c.unreachable_nodes().tolist() == [2]


def test_unreachable_is_empty_when_everything_is_reachable():
    """Guards the diagonal-masking bug: with the diagonal masked to False the
    predicate can never fire, and this would be the only symptom."""
    nodes = np.random.default_rng(12).random((6, 3)) * 40
    c = euclidean_cost(nodes)
    assert c.unreachable_nodes().size == 0


def test_a_node_blocked_from_only_some_peers_is_not_unreachable():
    from planner.cost_matrix import CostMatrices

    m = 4
    infeas = np.zeros((m, m), dtype=bool)
    infeas[1, 3] = infeas[3, 1] = True
    c = CostMatrices(kind=COST_QUICKNAV, length=np.ones((m, m)),
                     extra_vertices=np.zeros((m, m), dtype=np.int16),
                     infeasible=infeas, penalty=99.0, meta={})
    assert c.unreachable_nodes().size == 0


def test_frame_trap_guard_fires_when_nothing_detours():
    """Building in the wrong coordinate frame makes every pair look clear."""
    nodes = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
    far = np.array([[500.0, 500.0, 500.0]])
    c = quicknav_cost(nodes, far, np.array([[1.0, 1.0, 1.0]]))
    assert c.meta["n_detour_edges"] == 0
    with pytest.raises(ValueError, match="ZERO detours"):
        c.assert_detours_found(n_obstacles=1)


def test_guard_silent_when_there_are_no_obstacles():
    nodes = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
    c = quicknav_cost(nodes, np.zeros((0, 3)), np.zeros((0, 3)))
    c.assert_detours_found(n_obstacles=0)          # must not raise


# ------------------------------------------------------------------- derived
def test_time_matrix_charges_for_detour_vertices():
    nodes, centres, sizes = _one_box_between()
    c = quicknav_cost(nodes, centres, sizes)
    tm = TimeModel(v_eff=2.0, t_stop=1.0)
    t = c.time_matrix(tm)
    expected = c.length[0, 1] / 2.0 + c.extra_vertices[0, 1] * 1.0
    assert t[0, 1] == pytest.approx(expected)


def test_submatrix_round_trip():
    nodes = np.random.default_rng(6).random((10, 3)) * 30
    c = euclidean_cost(nodes)
    idx = [7, 2, 5]
    sub = c.submatrix(idx)
    assert sub.shape == (3, 3)
    assert sub[0, 1] == pytest.approx(c.length[7, 2])
    assert np.array_equal(sub, sub.T)


# --------------------------------------------------------------------- cache
def test_cache_key_is_content_addressed():
    a = np.random.default_rng(8).random((5, 3))
    obs = np.random.default_rng(9).random((2, 3))
    k1 = cache_key(COST_QUICKNAV, a, obs, 5.0, 10.0)
    k2 = cache_key(COST_QUICKNAV, a.copy(), obs.copy(), 5.0, 10.0)
    assert k1 == k2
    moved = a.copy()
    moved[0, 0] += 1e-6
    assert cache_key(COST_QUICKNAV, moved, obs, 5.0, 10.0) != k1
    assert cache_key(COST_EUCLIDEAN, a, obs, 5.0, 10.0) != k1


def test_cache_round_trip_and_hit_flag(tmp_path):
    rng = np.random.default_rng(10)
    nodes = rng.random((10, 3)) * 40
    obs = rng.random((3, 3)) * 40
    sizes = np.full((3, 3), 4.0)

    first = load_or_build(COST_QUICKNAV, nodes, obs, sizes, tmp_path)
    assert first.meta["cache_hit"] is False

    second = load_or_build(COST_QUICKNAV, nodes, obs, sizes, tmp_path)
    assert second.meta["cache_hit"] is True
    assert np.array_equal(first.length, second.length)
    assert np.array_equal(first.extra_vertices, second.extra_vertices)
    assert np.array_equal(first.infeasible, second.infeasible)
    assert first.penalty == pytest.approx(second.penalty)


def test_require_hit_turns_a_miss_into_an_error(tmp_path):
    nodes = np.random.default_rng(11).random((4, 3))
    with pytest.raises(FileNotFoundError, match="Pre-warm"):
        load_or_build(COST_QUICKNAV, nodes, np.zeros((0, 3)), np.zeros((0, 3)),
                      tmp_path, require_hit=True)


def test_build_rejects_unknown_kind():
    with pytest.raises(ValueError, match="unknown cost kind"):
        build("manhattan", np.zeros((3, 3)))
