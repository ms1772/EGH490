"""Pins the O-134 hardware mission against planner changes.

`flight_ops/lab_config/o134_mission/mission_spec.py` imports DCKmeans and uses
the CLUSTER ORDER to decide which physical bay of the netted volume each
aircraft starts in (`start_points_for`, mission_spec.py:192-197). The planner
pairs start i with cluster i by index. So a change to the clusterer -- or even
to the order it happens to return clusters in -- silently swaps which drone
flies which part of the room, with no error and no test failure anywhere else.

The golden values below were captured on 2026-10-08, before any planner work
began. The cluster order is [bay R, bay M, bay L]: not sorted, not alphabetical,
not guessable. That is exactly why it is pinned rather than recomputed.

These tests need numpy only -- no ROS, no WSL -- so they run everywhere.
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
MISSION_DIR = REPO / "flight_ops" / "lab_config" / "o134_mission"

# Captured 2026-10-08 from a clean checkout.
GOLDEN_START_POINTS = "-3.080,-2.750,1.000;1.700,-2.750,1.000;-0.820,-2.750,1.000"
GOLDEN_CLUSTER_BAYS = ["R", "M", "L"]
GOLDEN_CLUSTER_SIZES = [3, 3, 3]


def _load_mission_spec():
    if not MISSION_DIR.exists():
        pytest.skip("o134_mission not present in this checkout")
    sys.path.insert(0, str(REPO))
    sys.path.insert(0, str(MISSION_DIR))
    spec = importlib.util.spec_from_file_location(
        "o134_mission_spec", MISSION_DIR / "mission_spec.py"
    )
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_start_points_string_unchanged():
    """The exact --start_points handed to the planner for the lab mission.

    If this fails, the aircraft will fly a different bay assignment than the
    run-sheet, the floor tape and the crew brief describe.
    """
    spec = _load_mission_spec()
    _points, _centers, _halves, starts, _clusters, _env = spec.geometry()
    assert spec.fmt_start_points(starts) == GOLDEN_START_POINTS


def test_cluster_order_and_sizes_unchanged():
    """Cluster index -> bay mapping, and the 3/3/3 split the planner requires.

    A non-3/3/3 split makes DECK_GA_QuickNav.py:298 vstack a (0,) array onto a
    (3,) start point and die; mission_spec.py:137-141 documents the waypoint
    layout chosen to prevent it.
    """
    spec = _load_mission_spec()
    _points, _centers, _halves, _starts, clusters, _env = spec.geometry()

    sizes = [len(c) for c in clusters]
    assert sizes == GOLDEN_CLUSTER_SIZES, f"cluster sizes drifted: {sizes}"

    bays = [spec.bay_of(float(c[:, 0].mean())) for c in clusters]
    assert bays == GOLDEN_CLUSTER_BAYS, (
        f"cluster ORDER changed: {bays} != {GOLDEN_CLUSTER_BAYS}. "
        f"Each aircraft would start in a different bay than briefed."
    )


def test_mission_spec_does_not_import_the_new_planner_package():
    """The lab mission must keep using DCKmeans, not the new allocators.

    k-medoids and the makespan balancer reorder clusters by construction. Wiring
    them into the hardware config -- even as a drive-by improvement -- changes
    the physical bay assignment. This blocks the 'while I was in there' refactor.
    """
    source = (MISSION_DIR / "mission_spec.py").read_text(encoding="utf-8")
    for banned in ("planner.kmedoids", "planner.balance", "from planner import"):
        assert banned not in source, (
            f"mission_spec.py imports {banned!r}; that silently changes which "
            f"drone flies which bay in O-134."
        )
