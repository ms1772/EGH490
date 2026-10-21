"""Step 1: the vectorised tour evaluator must be BITWISE identical.

Not "close". The GA takes argmin over these values (GA_path_planning.py:43-45),
so a one-ulp difference can select a different individual, which changes how many
RNG draws that generation consumes, which diverges every subsequent generation.
Floating-point equality is the actual requirement here, not a strict choice.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from tests.reference_impl import ga_totaldist_reference  # noqa: E402


def _vectorised(pop, dmat):
    """The form now live in GA_path_planning.py."""
    cols = np.concatenate(
        [dmat[pop[:, -1], pop[:, 0]][:, None], dmat[pop[:, :-1], pop[:, 1:]]],
        axis=1,
    )
    return np.cumsum(cols, axis=1)[:, -1]


def _sum_axis1(pop, dmat):
    """The obvious form, kept only to document why it is NOT used."""
    return dmat[pop[:, :-1], pop[:, 1:]].sum(1) + dmat[pop[:, -1], pop[:, 0]]


def _random_case(rng, n, pop_size=16):
    xyz = rng.random((n, 3)) * 100.0
    a = np.meshgrid(np.arange(n), np.arange(n))
    dmat = np.sqrt(np.sum((xyz[a[0], :] - xyz[a[1], :]) ** 2, axis=2))
    pop = np.array([rng.permutation(n) for _ in range(pop_size)])
    return pop, dmat


@pytest.mark.parametrize("n", [2, 3, 4, 11, 35, 101])
def test_vectorised_evaluator_is_bitwise_identical(n):
    rng = np.random.default_rng(1234 + n)
    for _ in range(60):
        pop, dmat = _random_case(rng, n)
        ref = ga_totaldist_reference(pop, dmat, pop.shape[0], n)
        got = _vectorised(pop, dmat)
        assert got.tobytes() == ref.tobytes(), f"not bitwise identical at N={n}"


def test_single_node_degenerate_case():
    """N=1: no forward edges at all. The scalar loop returns dmat[0,0] == 0."""
    dmat = np.zeros((1, 1))
    pop = np.zeros((16, 1), dtype=int)
    ref = ga_totaldist_reference(pop, dmat, 16, 1)
    assert _vectorised(pop, dmat).tobytes() == ref.tobytes()


def test_sum_axis1_form_is_not_bitwise_identical():
    """Documents the trap, so nobody 'simplifies' the cumsum away later.

    This asserts the obvious one-liner DIFFERS, which is the whole reason the
    live code uses cumsum. If this ever starts passing, numpy changed its
    summation strategy and the comment in GA_path_planning.py needs revisiting.
    """
    rng = np.random.default_rng(7)
    mismatches = 0
    trials = 100
    for _ in range(trials):
        pop, dmat = _random_case(rng, 35)
        ref = ga_totaldist_reference(pop, dmat, 16, 35)
        if _sum_axis1(pop, dmat).tobytes() != ref.tobytes():
            mismatches += 1
    assert mismatches > trials // 2, (
        "expected .sum(axis=1) to differ from sequential accumulation in the "
        "last bits; it no longer does"
    )


def _scalar_fixed_generation(pop, total_dist, n, pop_size, perm, randint):
    """The generation step written out serially, with the flip fixed.

    This is what the vectorised block in GA_path_planning.py must reproduce
    exactly -- including the order the RNG is consumed in.
    """
    order = perm(pop_size)
    new_pop = np.zeros((pop_size, n), dtype=int)
    for p in range(0, pop_size, 4):
        rtes = pop[order[p:p + 4], :]
        dists = total_dist[order[p:p + 4]]
        best = rtes[np.argmin(dists), :]
        I, J = np.sort(randint(0, n, 2))
        t = np.zeros((4, n), dtype=int)
        t[0, :] = best
        t[1, :] = best.copy()
        t[1, I:J + 1] = best[I:J + 1][::-1]
        t[2, :] = best.copy()
        t[2, [I, J]] = best[[J, I]]
        t[3, :] = best.copy()
        t[3, I:J + 1] = np.roll(best[I:J + 1], shift=-1)
        new_pop[p:p + 4, :] = t
    return new_pop


def _vectorised_generation(pop, total_dist, n, pop_size, perm, randint):
    """Mirror of the block now live in GA_path_planning.py."""
    order = perm(pop_size)
    g = pop_size // 4
    ij = np.empty((g, 2), dtype=int)
    for k in range(g):
        ij[k] = np.sort(randint(0, n, 2))

    rtes = pop[order].reshape(g, 4, n)
    dists = total_dist[order].reshape(g, 4)
    best = rtes[np.arange(g), np.argmin(dists, axis=1), :]

    icol, jcol = ij[:, 0][:, None], ij[:, 1][:, None]
    col = np.arange(n)[None, :]
    inseg = (col >= icol) & (col <= jcol)

    kids = np.empty((g, 4, n), dtype=int)
    kids[:, 0, :] = best
    kids[:, 1, :] = np.take_along_axis(best, np.where(inseg, icol + jcol - col, col), axis=1)
    rows = np.arange(g)
    sw = best.copy()
    sw[rows, ij[:, 0]] = best[rows, ij[:, 1]]
    sw[rows, ij[:, 1]] = best[rows, ij[:, 0]]
    kids[:, 2, :] = sw
    seglen = jcol - icol + 1
    kids[:, 3, :] = np.take_along_axis(
        best, np.where(inseg, icol + ((col - icol + 1) % seglen), col), axis=1)
    return kids.reshape(pop_size, n)


@pytest.mark.parametrize("n", [2, 3, 4, 11, 35, 101])
def test_vectorised_generation_step_matches_scalar(n):
    """Same RNG stream in, identical population out.

    The mirrored/modular gathers must reproduce the slice reversal and np.roll
    exactly, and the (I, J) draws must be consumed in the same order -- which is
    why the live code keeps those four draws as a Python loop.
    """
    rng_seed = 2024 + n
    for trial in range(25):
        pop, dmat = _random_case(np.random.default_rng(rng_seed + trial), n)
        total = _vectorised(pop, dmat)

        np.random.seed(trial)
        a = _scalar_fixed_generation(
            pop, total, n, 16,
            np.random.permutation,
            lambda lo, hi, size: np.random.randint(lo, hi, size=size))

        np.random.seed(trial)
        b = _vectorised_generation(
            pop, total, n, 16,
            np.random.permutation,
            lambda lo, hi, size: np.random.randint(lo, hi, size=size))

        assert np.array_equal(a, b), f"generation step differs at N={n}, trial {trial}"


@pytest.mark.parametrize("n", [3, 11, 35])
def test_vectorised_children_are_permutations(n):
    """Belt and braces: the gathers must not duplicate or drop a city."""
    rng = np.random.default_rng(99)
    for _ in range(200):
        pop, dmat = _random_case(rng, n)
        total = _vectorised(pop, dmat)
        np.random.seed(0)
        out = _vectorised_generation(
            pop, total, n, 16,
            np.random.permutation,
            lambda lo, hi, size: np.random.randint(lo, hi, size=size))
        for row in out:
            assert len(np.unique(row)) == n
