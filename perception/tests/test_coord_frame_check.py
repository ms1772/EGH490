"""Tests for the coordinate-frame sanity warning in lidar_to_obstacles.py."""

from __future__ import annotations

import pickle
import sys
from pathlib import Path

import numpy as np
import pytest


# Import the module under test directly so we can call its helper.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from perception.lidar_to_obstacles import _coord_frame_warning  # noqa: E402


def _write_waypoints(path: Path, centre):
    """Write a synthetic Nx3 waypoint pkl centred on `centre`."""
    rng = np.random.default_rng(0)
    pts = rng.uniform(low=-1, high=1, size=(20, 3)) + np.asarray(centre)
    with path.open("wb") as f:
        pickle.dump(pts, f)


def test_warns_when_obstacles_far_from_waypoints(capsys, tmp_path: Path):
    """Obstacles at UTM-like coords + waypoints at origin → warning."""
    obstacles_centre = np.array([500_000.0, 6_900_000.0, 50.0])
    centres = np.tile(obstacles_centre, (5, 1)) + np.random.default_rng(0).normal(scale=2, size=(5, 3))

    wp_path = tmp_path / "wps.pkl"
    _write_waypoints(wp_path, centre=np.array([0.0, 0.0, 10.0]))

    _coord_frame_warning(centres, wp_path)
    captured = capsys.readouterr()
    combined = captured.out + captured.err
    assert "COORDINATE-FRAME WARNING" in combined


def test_no_warn_when_close(capsys, tmp_path: Path):
    centres = np.array([[1.0, 1.0, 1.0], [2.0, 2.0, 2.0]])
    wp_path = tmp_path / "wps.pkl"
    _write_waypoints(wp_path, centre=np.array([0.0, 0.0, 0.0]))

    _coord_frame_warning(centres, wp_path)
    captured = capsys.readouterr()
    combined = captured.out + captured.err
    assert "COORDINATE-FRAME WARNING" not in combined


def test_no_waypoints_prints_bbox(capsys):
    centres = np.array([[1.0, 1.0, 1.0], [3.0, 4.0, 5.0]])
    _coord_frame_warning(centres, None)
    captured = capsys.readouterr()
    combined = captured.out + captured.err
    assert "coord-check" in combined


def test_empty_centres_silent(capsys):
    _coord_frame_warning(np.zeros((0, 3)), None)
    captured = capsys.readouterr()
    # No output expected.
    assert captured.out == "" and captured.err == ""
