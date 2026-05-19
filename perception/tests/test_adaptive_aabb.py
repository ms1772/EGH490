"""Tests for perception/adaptive_aabb.py."""

from __future__ import annotations

import numpy as np
import pytest

from perception.adaptive_aabb import (
    chebyshev_clusters,
    extract_obstacles,
    voxelize,
)
from perception.tests.fixtures.synth_cloud import three_boxes, trunk_and_canopy


def _bbox_overlap_iou(c1, h1, c2, h2) -> float:
    """3D IoU between two AABBs given (centre, half_sizes)."""
    min1, max1 = c1 - h1, c1 + h1
    min2, max2 = c2 - h2, c2 + h2
    lo = np.maximum(min1, min2)
    hi = np.minimum(max1, max2)
    inter_dims = np.clip(hi - lo, a_min=0, a_max=None)
    inter = float(np.prod(inter_dims))
    vol1 = float(np.prod(2 * h1))
    vol2 = float(np.prod(2 * h2))
    union = vol1 + vol2 - inter
    return inter / union if union > 0 else 0.0


def _gt_match(extracted_centre, extracted_half, gt_boxes, s_safety, r_voxel):
    """Return the closest ground-truth box index whose centre lies inside the
    extracted AABB inflated by s_safety + r_voxel."""
    eps = s_safety + r_voxel
    for i, (gc, gh) in enumerate(gt_boxes):
        if np.all(np.abs(extracted_centre - gc) <= extracted_half + eps):
            return i
    return -1


def test_voxelize_canonical_centres():
    """Canonical centres are at (i + 0.5) * r_voxel for integer i."""
    rng = np.random.default_rng(0)
    pts = rng.uniform(low=0.0, high=1.0, size=(100, 3))
    r = 0.1
    centres = voxelize(pts, r)
    # All centres must be of the form (i + 0.5) * r — fractional part should be 0.5
    frac = (centres / r) - np.floor(centres / r)
    assert np.allclose(frac, 0.5, atol=1e-9)


def test_chebyshev_clusters_touching_voxels_merge():
    # Three voxels in a row, spacing exactly the merge threshold.
    r, s = 0.2, 0.5
    T = r + 2 * s
    centres = np.array([
        [0.0, 0.0, 0.0],
        [T, 0.0, 0.0],
        [2 * T, 0.0, 0.0],
        [10.0, 0.0, 0.0],  # well separated
    ])
    labels = chebyshev_clusters(centres, T)
    # First three share a cluster; the fourth is its own.
    assert labels[0] == labels[1] == labels[2]
    assert labels[3] != labels[0]


def test_chebyshev_clusters_diagonal_touch():
    """Voxels exactly on the Chebyshev boundary should merge."""
    r, s = 0.2, 0.5
    T = r + 2 * s
    centres = np.array([
        [0.0, 0.0, 0.0],
        [T, T, T],  # Chebyshev distance = T (touching)
    ])
    labels = chebyshev_clusters(centres, T)
    assert labels[0] == labels[1]


def test_recovers_known_boxes():
    """End-to-end: synthetic cloud of 3 boxes + ground → recover 3 AABBs."""
    points, gt_boxes = three_boxes(seed=42)
    # Drop the ground manually so we focus on extraction (ground filter has its own test).
    points = points[points[:, 2] > 0.1]

    r_voxel, s_safety = 0.1, 0.3
    centres, sizes = extract_obstacles(
        points,
        r_voxel=r_voxel,
        s_safety=s_safety,
        tau_fill=0.05,
        max_split_depth=2,
    )

    # Every ground-truth box must be matched by at least one extracted AABB.
    matched = set()
    for c, h in zip(centres, sizes):
        idx = _gt_match(c, h, gt_boxes, s_safety, r_voxel)
        if idx >= 0:
            matched.add(idx)

    assert matched == {0, 1, 2}, (
        f"Failed to recover all 3 ground-truth boxes. Matched: {matched}. "
        f"Extracted {len(centres)} obstacles."
    )


def test_trunk_canopy_split_occurs():
    """Trunk + canopy with low τ_fill must split (at least 2 leaf AABBs)."""
    points, _ = trunk_and_canopy(seed=42)
    points = points[points[:, 2] > 0.1]

    centres, sizes = extract_obstacles(
        points,
        r_voxel=0.1,
        s_safety=0.2,
        tau_fill=0.05,
        max_split_depth=3,
    )
    # The merged trunk+canopy cluster has fill ratio well below 0.05;
    # the algorithm must split at least once.
    assert len(centres) >= 2, (
        f"Expected at least 2 leaf AABBs from trunk+canopy split; got {len(centres)}."
    )


def test_trunk_canopy_force_merge():
    """With τ_fill=0, the trunk+canopy cluster must NOT split."""
    points, _ = trunk_and_canopy(seed=42)
    points = points[points[:, 2] > 0.1]

    centres, _ = extract_obstacles(
        points,
        r_voxel=0.1,
        s_safety=0.2,
        tau_fill=0.0,         # never split
        max_split_depth=3,
    )
    assert len(centres) == 1, (
        f"With τ_fill=0 expected exactly 1 merged AABB; got {len(centres)}."
    )


def test_safety_inflation_monotone():
    """Total inflated volume strictly increases with s_safety; count non-increasing."""
    points, _ = three_boxes(seed=42)
    points = points[points[:, 2] > 0.1]

    last_vol = -1.0
    last_count = 10 ** 9
    for s in (0.1, 0.3, 0.7, 1.5):
        centres, sizes = extract_obstacles(
            points,
            r_voxel=0.1,
            s_safety=s,
            tau_fill=0.05,
            max_split_depth=2,
        )
        vol = float(np.sum(np.prod(2 * sizes, axis=1)))
        count = len(centres)
        assert vol > last_vol, f"Volume not monotone increasing at s={s} (was {last_vol}, now {vol})"
        assert count <= last_count, f"Obstacle count grew at s={s} ({last_count} -> {count})"
        last_vol = vol
        last_count = count


def test_extract_empty_cloud():
    """Empty input → empty output, no exceptions."""
    centres, sizes = extract_obstacles(
        np.zeros((0, 3)), r_voxel=0.2, s_safety=0.5
    )
    assert centres.shape == (0, 3)
    assert sizes.shape == (0, 3)


def test_extract_single_point():
    """A single-point cloud → exactly one AABB centred there."""
    pts = np.array([[1.0, 2.0, 3.0]])
    centres, sizes = extract_obstacles(pts, r_voxel=0.2, s_safety=0.5)
    assert len(centres) == 1
    # Centre should be near the voxel centre (within r_voxel/2).
    assert np.allclose(centres[0], [1.0, 2.0, 3.0], atol=0.2)
    # Half-sizes: r_voxel/2 + s_safety = 0.1 + 0.5 = 0.6 per axis.
    assert np.allclose(sizes[0], [0.6, 0.6, 0.6], atol=1e-9)
