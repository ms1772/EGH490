"""Synthetic point-cloud fixtures for perception tests.

Each generator returns (points, ground_truth_aabbs). points is (M,3).
ground_truth_aabbs is a list of (centre (3,), half_sizes (3,)) tuples
describing the boxes the points were sampled from — BEFORE any safety
inflation. Tests compare extracted AABBs against this with a tolerance
that accounts for r_voxel + s_safety.
"""

from __future__ import annotations

from typing import List, Tuple

import numpy as np


def _sample_box(
    centre: np.ndarray, half: np.ndarray, n: int, rng: np.random.Generator
) -> np.ndarray:
    """Sample n points uniformly inside an axis-aligned box."""
    lows = centre - half
    highs = centre + half
    return rng.uniform(low=lows, high=highs, size=(n, 3))


def three_boxes(seed: int = 0) -> Tuple[np.ndarray, List[Tuple[np.ndarray, np.ndarray]]]:
    """Three well-separated boxes: tall column, wide slab, cube. Plus ground.

    Returns (points (M,3), [(centre, half_sizes), ...])
    """
    rng = np.random.default_rng(seed)

    boxes = [
        (np.array([0.0, 0.0, 2.5]),  np.array([0.2, 0.2, 2.5])),   # tall thin column
        (np.array([5.0, 0.0, 0.5]),  np.array([2.0, 2.0, 0.25])),  # wide flat slab
        (np.array([0.0, 5.0, 1.0]),  np.array([0.5, 0.5, 0.5])),   # cube
    ]

    point_clouds = [_sample_box(c, h, 300, rng) for c, h in boxes]

    # Ground plane: 20m x 20m at z=0 (thin slab).
    ground = rng.uniform(low=[-10, -10, -0.05], high=[10, 10, 0.05], size=(2000, 3))

    # Mild Gaussian noise on obstacle points.
    obs_points = np.vstack(point_clouds)
    obs_points = obs_points + rng.normal(scale=0.02, size=obs_points.shape)

    pts = np.vstack([obs_points, ground])
    return pts, boxes


def trunk_and_canopy(seed: int = 0) -> Tuple[np.ndarray, List[Tuple[np.ndarray, np.ndarray]]]:
    """Thin tall trunk + wide flat canopy above it. Models the 'tree' case.

    The two boxes overlap in z at z ≈ 4..5 so the merged Chebyshev cluster
    spans the union. Fill ratio of the merged AABB is low; a median-axis
    split should bisect along z first.
    """
    rng = np.random.default_rng(seed)

    trunk_centre = np.array([0.0, 0.0, 2.5])
    trunk_half = np.array([0.15, 0.15, 2.5])

    canopy_centre = np.array([0.0, 0.0, 4.5])
    canopy_half = np.array([2.0, 2.0, 0.5])

    trunk_pts = _sample_box(trunk_centre, trunk_half, 200, rng)
    canopy_pts = _sample_box(canopy_centre, canopy_half, 400, rng)

    obs_points = np.vstack([trunk_pts, canopy_pts])
    obs_points = obs_points + rng.normal(scale=0.02, size=obs_points.shape)

    ground = rng.uniform(low=[-5, -5, -0.05], high=[5, 5, 0.05], size=(1000, 3))

    pts = np.vstack([obs_points, ground])
    return pts, [(trunk_centre, trunk_half), (canopy_centre, canopy_half)]


def floor_only(seed: int = 0) -> np.ndarray:
    """A pure ground plane for ground-filter tests."""
    rng = np.random.default_rng(seed)
    return rng.uniform(low=[-10, -10, -0.05], high=[10, 10, 0.05], size=(5000, 3))


def floor_with_boxes(seed: int = 0) -> Tuple[np.ndarray, np.ndarray]:
    """Ground plane + box obstacles. Returns (points, labels).

    labels: 1 for obstacle points, 0 for ground. Used to score ground filter
    precision/recall.
    """
    rng = np.random.default_rng(seed)
    ground = rng.uniform(low=[-10, -10, -0.05], high=[10, 10, 0.05], size=(5000, 3))
    box1 = _sample_box(np.array([0.0, 0.0, 1.0]), np.array([0.5, 0.5, 1.0]), 1000, rng)
    box2 = _sample_box(np.array([3.0, 3.0, 0.5]), np.array([0.3, 0.3, 0.5]), 500, rng)
    pts = np.vstack([ground, box1, box2])
    labels = np.concatenate([
        np.zeros(len(ground), dtype=int),
        np.ones(len(box1) + len(box2), dtype=int),
    ])
    return pts, labels
