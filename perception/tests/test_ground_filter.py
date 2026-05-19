"""Tests for perception/ground_filter.py."""

from __future__ import annotations

import importlib.util

import numpy as np
import pytest

from perception.ground_filter import remove_ground
from perception.tests.fixtures.synth_cloud import floor_only, floor_with_boxes


_has_open3d = importlib.util.find_spec("open3d") is not None
requires_open3d = pytest.mark.skipif(not _has_open3d, reason="open3d not installed")


@requires_open3d
def test_ransac_removes_floor():
    """RANSAC removes ≥95% of ground points and retains ≥95% of obstacle points."""
    pts, labels = floor_with_boxes(seed=42)
    filtered, info = remove_ground(pts, method="ransac", ransac_distance=0.2, seed=42)

    # Build a mask for filtered points by matching back to original.
    # Simpler: compute precision/recall against labels by counting.
    # remove_ground returns filtered points but not their original indices,
    # so we reconstruct by point-set membership using a kdtree.
    from scipy.spatial import cKDTree
    tree = cKDTree(pts)
    dists, idx = tree.query(filtered, k=1)
    # Map filtered points back to their original label.
    keep_labels = labels[idx]

    n_obs_total = int(labels.sum())
    n_gnd_total = int((1 - labels).sum())
    n_obs_kept = int(keep_labels.sum())
    n_gnd_kept = int((1 - keep_labels).sum())

    obs_recall = n_obs_kept / n_obs_total
    gnd_removed_frac = 1.0 - (n_gnd_kept / n_gnd_total)

    assert obs_recall >= 0.95, f"Obstacle retention {obs_recall:.2%} < 95%"
    assert gnd_removed_frac >= 0.95, f"Ground removal {gnd_removed_frac:.2%} < 95%"


@requires_open3d
def test_ransac_seed_determinism():
    """Same seed → identical output."""
    pts, _ = floor_with_boxes(seed=42)
    f1, _ = remove_ground(pts, method="ransac", ransac_distance=0.2, seed=42)
    f2, _ = remove_ground(pts, method="ransac", ransac_distance=0.2, seed=42)
    assert f1.shape == f2.shape
    assert np.allclose(np.sort(f1, axis=0), np.sort(f2, axis=0))


def test_z_threshold_basic():
    """z_threshold drops everything at/below ground_z."""
    pts = np.array([
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.05],
        [0.0, 0.0, 0.5],
        [0.0, 0.0, 2.0],
    ])
    out, info = remove_ground(pts, method="z_threshold", ground_z=0.1)
    assert info["method"] == "z_threshold"
    assert info["n_out"] == 2
    assert np.all(out[:, 2] > 0.1)


def test_z_threshold_auto_pick():
    """No ground_z → auto-pick 5th percentile + 0.05 margin."""
    pts = floor_only(seed=0)  # z near 0
    out, info = remove_ground(pts, method="z_threshold", ground_z=None)
    # Most ground points should be removed, only the upper tail above the
    # auto-picked threshold survives.
    assert info["n_out"] < info["n_in"]
    assert "ground_z" in info


def test_none_passthrough():
    pts = np.random.default_rng(0).uniform(size=(50, 3))
    out, info = remove_ground(pts, method="none")
    assert info["method"] == "none"
    assert out.shape == pts.shape
    assert np.allclose(out, pts)


def test_unknown_method_raises():
    pts = np.zeros((1, 3))
    with pytest.raises(ValueError):
        remove_ground(pts, method="bogus")


@requires_open3d
def test_ransac_warns_on_vertical_plane():
    """A cloud whose dominant plane is a wall should trigger a warning."""
    # Sample points on a vertical plane (x = 0).
    rng = np.random.default_rng(0)
    wall = rng.uniform(low=[-0.05, -10, -10], high=[0.05, 10, 10], size=(3000, 3))
    out, info = remove_ground(wall, method="ransac", ransac_distance=0.2, seed=42)
    assert "warning" in info or info["normal_z"] < 0.7
