"""Tests for perception/io_utils.py — round-trips and format detection."""

from __future__ import annotations

import pickle
from pathlib import Path

import numpy as np
import pytest

from perception.io_utils import (
    load_obstacles_any,
    write_obstacles_dict,
    write_obstacles_txt,
)


def test_dict_format_roundtrip(tmp_path: Path):
    centres = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    sizes = np.array([[0.5, 0.5, 0.5], [1.0, 0.3, 0.7]])
    meta = {"source": "lidar", "params": {"voxel": 0.2}}

    out = tmp_path / "obs.pkl"
    write_obstacles_dict(out, centres, sizes, meta)

    c, s = load_obstacles_any(out)
    assert np.allclose(c, centres)
    assert np.allclose(s, sizes)


def test_legacy_nx3_format_with_fallback(tmp_path: Path):
    arr = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    out = tmp_path / "legacy_obs.pkl"
    with out.open("wb") as f:
        pickle.dump(arr, f)

    c, s = load_obstacles_any(out, fallback_size=7.0)
    assert np.allclose(c, arr)
    assert s.shape == (2, 3)
    assert np.all(s == 7.0)


def test_dict_1d_sizes_promoted_to_3d(tmp_path: Path):
    """A dict pkl with sizes shape (N,) should be tiled to (N,3)."""
    centres = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    sizes_1d = np.array([0.4, 0.6])

    out = tmp_path / "obs_1d.pkl"
    with out.open("wb") as f:
        pickle.dump({"centers": centres, "sizes": sizes_1d, "meta": {}}, f)

    c, s = load_obstacles_any(out)
    assert s.shape == (2, 3)
    assert np.allclose(s[0], [0.4, 0.4, 0.4])
    assert np.allclose(s[1], [0.6, 0.6, 0.6])


def test_write_dict_shape_validation(tmp_path: Path):
    out = tmp_path / "bad.pkl"
    with pytest.raises(ValueError):
        write_obstacles_dict(out, centers=np.zeros((3, 2)), sizes=np.zeros((3, 3)), meta={})
    with pytest.raises(ValueError):
        write_obstacles_dict(out, centers=np.zeros((3, 3)), sizes=np.zeros((2, 3)), meta={})


def test_write_txt_six_columns(tmp_path: Path):
    centres = np.array([[1.0, 2.0, 3.0]])
    sizes = np.array([[0.5, 0.7, 0.9]])
    out = tmp_path / "obs.txt"
    write_obstacles_txt(out, centres, sizes)
    lines = out.read_text().splitlines()
    header = lines[0]
    row = lines[1]
    assert header == "cx,cy,cz,sx,sy,sz"
    assert len(row.split(",")) == 6


def test_legacy_invalid_shape_raises(tmp_path: Path):
    out = tmp_path / "bad_shape.pkl"
    with out.open("wb") as f:
        pickle.dump(np.zeros((5, 2)), f)
    with pytest.raises(ValueError):
        load_obstacles_any(out, fallback_size=1.0)
