"""Step 2: the flip mutation must produce a valid permutation.

The original wrote only the reversed slice into a zero-filled row, so the child
was not a permutation and the integrity guard at the end of the mutation block
silently replaced it with a uniform random restart. These tests pin the fix and
prove the guard is now dead code.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))


def _children(best, I, J, *, broken):
    """Reproduce one group's four children exactly as the live module builds them."""
    n = len(best)
    tmp = np.zeros((4, n), dtype=int)
    tmp[0, :] = best
    if broken:
        if I < J:
            tmp[1, I:J + 1] = best[I:J + 1][::-1]
        else:
            tmp[1, :] = best
    else:
        tmp[1, :] = best.copy()
        tmp[1, I:J + 1] = best[I:J + 1][::-1]
    tmp[2, :] = best.copy()
    tmp[2, [I, J]] = best[[J, I]]
    tmp[3, :] = best.copy()
    tmp[3, I:J + 1] = np.roll(best[I:J + 1], shift=-1)
    return tmp


def test_all_four_operators_preserve_permutations_when_fixed():
    """20,000 random (N, I, J) draws x 4 children. Zero may be invalid.

    This is what makes the np.unique integrity guard removable: with the fix,
    every operator is permutation-preserving by construction.
    """
    rng = np.random.default_rng(1)
    invalid = 0
    for _ in range(20_000):
        n = int(rng.integers(3, 60))
        best = rng.permutation(n)
        I, J = np.sort(rng.integers(0, n, size=2))
        for child in _children(best, I, J, broken=False):
            if len(np.unique(child)) != n:
                invalid += 1
    assert invalid == 0, f"{invalid} invalid children from the fixed operators"


def test_broken_flip_is_overwhelmingly_invalid():
    """Quantifies the defect, and guards against 'fixing' the broken path.

    At N=35 the flip child is a valid permutation only when I==J (probability
    1/N) or when the slice spans the whole row, I==0 and J==N-1 (2/N^2).
    """
    rng = np.random.default_rng(2)
    n, trials = 35, 20_000
    invalid = 0
    for _ in range(trials):
        best = rng.permutation(n)
        I, J = np.sort(rng.integers(0, n, size=2))
        child = _children(best, I, J, broken=True)[1]
        if len(np.unique(child)) != n:
            invalid += 1
    rate = invalid / trials
    assert 0.95 < rate < 0.99, f"expected ~96.9% invalid at N=35, measured {rate:.3%}"


def test_live_module_defaults_to_fixed():
    import GA_path_planning

    assert GA_path_planning.FLIP_BROKEN_DEFAULT is False


def test_broken_path_still_reproduces_the_frozen_reference():
    """The A/B must stay runnable now that the fix is merged.

    A git-revert-based comparison stops being reproducible the moment it lands;
    this keeps the original behaviour reachable by flag, forever.
    """
    from GA_path_planning import ga_3d_pathplanning

    from tests.reference_impl import ga_3d_pathplanning_reference

    xyz = np.random.default_rng(5).random((6, 3)) * 100.0

    np.random.seed(5)
    live_broken = ga_3d_pathplanning(xyz, flip_broken=True, num_iter=400, verbose=False)
    np.random.seed(5)
    ref = ga_3d_pathplanning_reference(xyz, flip_broken=True, num_iter=400)

    assert np.array_equal(live_broken, ref)


def test_fixed_differs_from_broken():
    """Sanity: the fix actually changes the search, at a size where it matters."""
    from GA_path_planning import ga_3d_pathplanning

    xyz = np.random.default_rng(11).random((20, 3)) * 100.0

    np.random.seed(0)
    broken = ga_3d_pathplanning(xyz, flip_broken=True, num_iter=2000, verbose=False)
    np.random.seed(0)
    fixed = ga_3d_pathplanning(xyz, flip_broken=False, num_iter=2000, verbose=False)

    assert not np.array_equal(broken, fixed)
