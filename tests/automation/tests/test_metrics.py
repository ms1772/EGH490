"""Unit tests for metrics.py — pure functions, no ROS, no subprocess."""

from __future__ import annotations

import numpy as np
import pytest

from tests.automation import metrics


def test_path_distance_simple():
    p = np.array([[0, 0, 0], [3, 4, 0]], dtype=float)
    assert metrics.path_distance(p) == pytest.approx(5.0)


def test_path_distance_multi_segment():
    p = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [1, 1, 1]], dtype=float)
    assert metrics.path_distance(p) == pytest.approx(3.0)


def test_path_distance_degenerate():
    assert metrics.path_distance(np.array([[1, 2, 3]], dtype=float)) == 0.0
    assert metrics.path_distance(np.zeros((0, 3))) == 0.0


def test_fleet_distance():
    paths = [
        np.array([[0, 0, 0], [3, 4, 0]], dtype=float),
        np.array([[0, 0, 0], [0, 0, 5]], dtype=float),
    ]
    assert metrics.fleet_distance(paths) == pytest.approx(10.0)


def test_detections_per_uav():
    detected_indices = [[0, 1, 2], [], [5]]
    assert metrics.detections_per_uav(detected_indices) == [3, 0, 1]


def test_avoidances_per_uav():
    deckga = [np.zeros((5, 3)), np.zeros((3, 3)), np.zeros((10, 3))]
    quicknav = [np.zeros((9, 3)), np.zeros((3, 3)), np.zeros((14, 3))]
    assert metrics.avoidances_per_uav(deckga, quicknav) == [4, 0, 4]


def test_residual_collisions_empty_obstacles():
    quicknav = [np.array([[0, 0, 0], [10, 0, 0]], dtype=float)]
    assert metrics.residual_collisions_per_uav(quicknav, np.zeros((0, 3)), 1.0) == [0]


def test_residual_collisions_clean_path():
    quicknav = [np.array([[0, 0, 0], [10, 0, 0]], dtype=float)]
    obstacles = np.array([[5, 5, 0]], dtype=float)  # 5 m to the side
    sizes = np.array([1.0])
    assert metrics.residual_collisions_per_uav(quicknav, obstacles, sizes) == [0]


def test_residual_collisions_direct_hit():
    quicknav = [np.array([[0, 0, 0], [10, 0, 0]], dtype=float)]
    obstacles = np.array([[5, 0, 0]], dtype=float)  # directly on the path
    sizes = np.array([1.0])
    assert metrics.residual_collisions_per_uav(quicknav, obstacles, sizes) == [1]


def test_residual_collisions_scalar_obs_size():
    """Scalar obs_size should be broadcast across all obstacles."""
    quicknav = [np.array([[0, 0, 0], [10, 0, 0]], dtype=float)]
    obstacles = np.array([[5, 0, 0]], dtype=float)
    assert metrics.residual_collisions_per_uav(quicknav, obstacles, 1.0) == [1]


def test_compute_all_metrics_no_obstacles():
    data = {
        "deckga_paths": [np.array([[0, 0, 0], [3, 4, 0]], dtype=float)],
        "quicknav_paths": [np.array([[0, 0, 0], [3, 4, 0]], dtype=float)],
        "obstacle_xyz": np.zeros((0, 3)),
        "obs_size": np.zeros((0,)),
        "detected_indices": [[]],
        "num_uavs": 1,
    }
    m = metrics.compute_all_metrics(data)
    assert m["distance_baseline_total_m"] == pytest.approx(5.0)
    assert m["distance_avoided_total_m"] == pytest.approx(5.0)
    assert m["detections_total"] == 0
    assert m["avoidances_total"] == 0
    assert m["residual_collisions_total"] == 0


def test_compute_all_metrics_with_avoidance():
    """Sanity: QuickNav added 2 inserted vertices, residual collisions = 0."""
    data = {
        "deckga_paths": [np.array([[0, 0, 0], [10, 0, 0]], dtype=float)],
        # Same start/end, but detoured around the obstacle via +y
        "quicknav_paths": [np.array([
            [0, 0, 0],
            [4, 3, 0],
            [6, 3, 0],
            [10, 0, 0],
        ], dtype=float)],
        "obstacle_xyz": np.array([[5, 0, 0]], dtype=float),
        "obs_size": np.array([1.0]),
        "detected_indices": [[0]],
        "num_uavs": 1,
    }
    m = metrics.compute_all_metrics(data)
    assert m["detections_total"] == 1
    assert m["avoidances_total"] == 2  # 2 inserted vertices
    assert m["residual_collisions_total"] == 0
    assert m["distance_avoided_total_m"] > m["distance_baseline_total_m"]


def test_penetration_depth_clean():
    assert metrics.penetration_depth(
        np.array([0, 0, 0]), np.array([10, 0, 0]),
        np.array([5, 5, 0]), 1.0,
    ) == 0.0


def test_penetration_depth_through_center():
    # Segment passes through cube center; max penetration ~= half-size (1.0)
    # At n_samples=201 the central sample lands exactly on the centre.
    d = metrics.penetration_depth(
        np.array([0, 0, 0]), np.array([10, 0, 0]),
        np.array([5, 0, 0]), 1.0, n_samples=201,
    )
    assert d == pytest.approx(1.0, abs=0.02)


def test_penetration_depth_per_axis_size():
    """3-vector size: cube is thin in y/z, wide in x."""
    d = metrics.penetration_depth(
        np.array([0, 0, 0]), np.array([10, 0, 0]),
        np.array([5, 0, 0]),
        np.array([1.0, 0.5, 0.5]),
        n_samples=50,
    )
    assert d == pytest.approx(0.5, abs=0.1)
