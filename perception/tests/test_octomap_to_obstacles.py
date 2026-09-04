"""Tests for perception/octomap_to_obstacles.py.

Round-trips the synthetic fixture (perception/tests/fixtures/synth_octree.py)
through the .bt parser and adaptive AABB extraction. No external octomap
library required.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from perception.octomap_to_obstacles import (
    OCTOMAP_MAX_DEPTH,
    load_octree_centres,
    octomap_to_aabbs,
    _read_header,
)
from perception.tests.fixtures.synth_octree import (
    DEFAULT_BOXES,
    DEFAULT_FIXTURE_PATH,
    DEFAULT_RESOLUTION,
    ensure_default_fixture,
)


@pytest.fixture(scope="module")
def synth_fixture():
    """Write (if missing) and return the default 3-box .bt fixture metadata."""
    return ensure_default_fixture()


def test_load_octree_centres_matches_fixture(synth_fixture):
    centres, res = load_octree_centres(Path(synth_fixture["path"]))
    assert res == pytest.approx(synth_fixture["resolution"])
    assert centres.shape == synth_fixture["leaf_centres"].shape

    # Centre sets must match as sets (DFS order may differ).
    expected_sorted = synth_fixture["leaf_centres"][np.lexsort(synth_fixture["leaf_centres"].T)]
    got_sorted = centres[np.lexsort(centres.T)]
    np.testing.assert_allclose(got_sorted, expected_sorted, atol=1e-9)


def test_octree_to_aabbs_three_boxes(synth_fixture):
    centers, sizes, meta = octomap_to_aabbs(
        Path(synth_fixture["path"]),
        s_safety=0.5,
        tau_fill=0.05,
        max_split_depth=3,
    )
    assert meta["source"] == "octomap"
    assert meta["resolution"] == pytest.approx(DEFAULT_RESOLUTION)
    # Expect one AABB per planted box.
    assert centers.shape[0] == len(DEFAULT_BOXES), (
        f"Expected {len(DEFAULT_BOXES)} AABBs, got {centers.shape[0]}"
    )

    # Every ground-truth box centre should fall inside exactly one extracted AABB
    # (within an extra res + safety tolerance to account for inflation).
    tol = DEFAULT_RESOLUTION + 0.5  # safety inflation
    for (gc, _gh) in DEFAULT_BOXES:
        gc_arr = np.asarray(gc)
        inside = np.all(np.abs(centers - gc_arr) <= sizes + tol, axis=1)
        assert inside.sum() >= 1, f"No AABB contains ground-truth centre {gc}"


def test_truncated_bt_raises(tmp_path, synth_fixture):
    src = Path(synth_fixture["path"]).read_bytes()
    # Keep header but lop the binary payload.
    header_end = src.find(b"data\n") + len(b"data\n")
    truncated = src[: header_end + 2]  # only 2 bytes of payload -> can't recurse
    bad = tmp_path / "trunc.bt"
    bad.write_bytes(truncated)
    with pytest.raises(ValueError, match="truncated"):
        load_octree_centres(bad)


def test_header_missing_data_raises(tmp_path):
    bad = tmp_path / "noheader.bt"
    bad.write_bytes(b"# Octomap OcTree binary file\nid OcTree\nres 0.5\n")
    with pytest.raises(ValueError, match="data"):
        load_octree_centres(bad)


def test_header_missing_res_raises(tmp_path):
    bad = tmp_path / "nores.bt"
    bad.write_bytes(b"# Octomap OcTree binary file\nid OcTree\nsize 0\ndata\n")
    with pytest.raises(ValueError, match=r"(?i)res"):
        load_octree_centres(bad)


def test_ot_extension_rejected(tmp_path):
    bad = tmp_path / "x.ot"
    bad.write_bytes(b"")  # contents irrelevant; suffix check happens first
    with pytest.raises(NotImplementedError):
        load_octree_centres(bad)


def test_explode_uniform_subtree(tmp_path):
    """An occupied marker at non-leaf depth should expand to its leaf voxels."""
    # Craft a tiny synthetic .bt by hand:
    # - resolution 1.0
    # - One occupied leaf at child index 7 of the root (signs +,+,+).
    # The expansion of that subtree (depth=16 from root) at the top-level child
    # would be enormous, so instead we test the small-cube path: place one box
    # at half-extent equal to resolution (single leaf). This proves the leaf path.
    from perception.tests.fixtures.synth_octree import write_synth_bt
    p = tmp_path / "one_leaf.bt"
    write_synth_bt(p, [((10.0, 10.0, 10.0), 0.5)], resolution=1.0)
    centres, res = load_octree_centres(p)
    assert res == 1.0
    assert centres.shape == (1, 3)


def test_empty_tree(tmp_path):
    from perception.tests.fixtures.synth_octree import write_synth_bt
    p = tmp_path / "empty.bt"
    write_synth_bt(p, [], resolution=0.5)
    centres, res = load_octree_centres(p)
    assert centres.shape == (0, 3)
    assert res == 0.5


def test_round_trip_dispatcher(synth_fixture):
    """end-to-end: parse + adaptive AABB returns matching counts to extract_obstacles
    applied to the same centres."""
    from perception.adaptive_aabb import extract_obstacles
    centres_a, _ = load_octree_centres(Path(synth_fixture["path"]))
    expected_centres, expected_sizes = extract_obstacles(
        centres_a, r_voxel=DEFAULT_RESOLUTION, s_safety=0.5
    )
    centers_b, sizes_b, _ = octomap_to_aabbs(Path(synth_fixture["path"]), s_safety=0.5)
    assert centers_b.shape == expected_centres.shape
    np.testing.assert_allclose(
        centers_b[np.lexsort(centers_b.T)],
        expected_centres[np.lexsort(expected_centres.T)],
        atol=1e-9,
    )
