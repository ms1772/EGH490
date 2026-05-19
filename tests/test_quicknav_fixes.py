"""Regression tests for the Phase 2 QuickNav avoidance fixes.

Bugs covered:
  A: processed_obstacles skip-list (deleted)
  B: single-obstacle candidate filter (replaced with multi-obstacle 3-segment check)
  C: silent fallback to clipping paths (replaced with retry + warn)
  D: splice-order assumption (sorted obs_route before splice loop)
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from QuickNav import apply_quicknav
from QuickNav_detection_3D import line_intersects_cube


def _path_has_collision(
    path: np.ndarray,
    centres: np.ndarray,
    sizes: np.ndarray,
    eps: float = 1e-4,
) -> bool:
    """Check whether any path segment penetrates the INTERIOR of an obstacle box.

    The obstacle box is shrunk by `eps` on every axis so that detour corners
    lying exactly on the obstacle surface — which is the intended behaviour
    of QuickNav's geometric avoidance — are not flagged as collisions.
    """
    for k, c in enumerate(centres):
        sz = sizes[k]
        if np.isscalar(sz):
            sx = sy = sz_ = float(sz)
        else:
            sx, sy, sz_ = (float(v) for v in sz)
        pmin = np.array([c[0] - sx + eps, c[1] - sy + eps, c[2] - sz_ + eps])
        pmax = np.array([c[0] + sx - eps, c[1] + sy - eps, c[2] + sz_ - eps])
        if np.any(pmin >= pmax):
            continue  # obstacle smaller than 2*eps — skip
        for j in range(len(path) - 1):
            if line_intersects_cube(path[j], path[j + 1], pmin, pmax):
                return True
    return False


def test_regression_single_obstacle_does_not_clip():
    """Single obstacle: basic sanity. The fixes must not break the simple case."""
    route = np.array([[0.0, 0.0, 1.0], [10.0, 0.0, 1.0]])
    centres = np.array([[5.0, 0.0, 1.0]])
    sizes = np.array([[1.0, 1.0, 1.0]])

    out = apply_quicknav(route, centres, sizes)
    assert not _path_has_collision(out, centres, sizes)
    assert len(out) > 2  # detour added points


def test_per_axis_sizes_propagate_correctly():
    """An (N,3) obs_size array must produce non-cubic detours."""
    route = np.array([[0.0, 0.0, 1.0], [10.0, 0.0, 1.0]])
    centres = np.array([[5.0, 0.0, 1.0]])
    sizes = np.array([[1.0, 4.0, 0.2]])  # wide in y, thin in z

    out = apply_quicknav(route, centres, sizes)
    assert not _path_has_collision(out, centres, sizes)


def test_out_of_order_obs_route_sort(capsys):
    """Multiple obstacles on different segments, with obstacle_xyz order producing
    non-monotonic obs_route. Without the sort fix in QuickNav.py, the second
    splice would overwrite the first."""
    # 5-waypoint path along y; two obstacles, one on segment 1 (between y=0..5)
    # and one on segment 3 (between y=10..15).
    route = np.array([
        [0.0, 0.0, 1.0],
        [0.0, 5.0, 1.0],
        [0.0, 10.0, 1.0],
        [0.0, 15.0, 1.0],
        [0.0, 20.0, 1.0],
    ])
    # obstacle_xyz order: A is on the LATER segment (y=12), B is on the EARLIER
    # segment (y=3). obstacle_detection iterates obstacles-then-segments, so
    # without the sort, obs_route = [(3,4), (1,2)] — out of order.
    centres = np.array([
        [0.0, 12.0, 1.0],   # obstacle 0 -> hits segment 3 (waypoints 2..3 -> obs_route[3,4])
        [0.0, 3.0, 1.0],    # obstacle 1 -> hits segment 1 (waypoints 0..1 -> obs_route[1,2])
    ])
    sizes = np.array([
        [0.8, 0.8, 0.8],
        [0.8, 0.8, 0.8],
    ])

    out = apply_quicknav(route, centres, sizes)
    assert not _path_has_collision(out, centres, sizes), (
        f"Splice-order fix did not work; final route still clips.\n{out}"
    )


def test_two_separated_obstacles_both_resolved():
    """Two obstacles with non-overlapping safety zones; both must be cleared."""
    route = np.array([[0.0, 0.0, 1.0], [12.0, 0.0, 1.0]])
    centres = np.array([
        [3.0, 0.0, 1.0],
        [9.0, 0.0, 1.0],
    ])
    sizes = np.array([
        [0.8, 0.8, 0.8],
        [0.8, 0.8, 0.8],
    ])

    out = apply_quicknav(route, centres, sizes)
    assert not _path_has_collision(out, centres, sizes), (
        f"Both obstacles should have been resolved.\n{out}"
    )


def test_detour_around_first_avoids_nearby_second():
    """Multi-obstacle filter: the detour around the FIRST obstacle must not
    clip a NEARBY second obstacle.

    Without bug-B fix, the candidate filter only checks the current obstacle.
    A naive detour around A picks a face/corner that happens to penetrate B.
    The fix's all-obstacles, all-3-segments check rejects that candidate and
    picks a face on the opposite side.
    """
    route = np.array([[0.0, 0.0, 1.0], [6.0, 0.0, 1.0]])
    # A in the path, B alongside.
    centres = np.array([
        [3.0, 0.0, 1.0],     # A — directly on path
        [3.0, 2.5, 1.0],     # B — alongside, in the +y direction (where one detour would go)
    ])
    sizes = np.array([
        [0.6, 0.6, 0.6],
        [0.6, 2.0, 0.6],     # B is wide in y, occupying y in [0.5, 4.5]
    ])

    out = apply_quicknav(route, centres, sizes)
    assert not _path_has_collision(out, centres, sizes), (
        f"Detour around A clipped neighbour B.\n{out}"
    )


def test_max_iter_warns_when_unsolvable(capsys):
    """An unsolvable scene must trigger a loud warning rather than silently
    returning a clipping path.

    Setup: the path's endpoint is INSIDE an obstacle. Every detour candidate
    has its exit segment (c2 -> end) penetrating the obstacle because the
    destination is inside it. The retry-with-wider-corners stage also fails
    for the same reason. The algorithm must surface the failure loudly.
    """
    route = np.array([[0.0, 0.0, 1.0], [10.0, 0.0, 1.0]])
    centres = np.array([[9.0, 0.0, 1.0]])
    # Obstacle spans (6,-3,-2) to (12,3,4). The path's end (10,0,1) is well inside.
    sizes = np.array([[3.0, 3.0, 3.0]])

    out = apply_quicknav(route, centres, sizes, max_iterations=3)
    captured = capsys.readouterr()
    combined = captured.out + captured.err
    assert ("WARNING" in combined) or ("no collision-free" in combined.lower()), (
        f"Expected a warning for unsolvable scene. Got:\n{combined!r}"
    )
