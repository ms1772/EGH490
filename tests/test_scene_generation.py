"""Scene generation: rejection sampling must make dense scenes planner-legal.

`DECK_GA_QuickNav.py:463-477` rejects an ENTIRE scene if any waypoint or start
point falls inside an obstacle AABB, and the containment test is inclusive
(`>=` / `<=`), so a point exactly on a face is a conflict. At n=100/M=100 a few
waypoints land inside an obstacle by pure chance; the sweep's four-seed retry
loop then rerolls and often fails again, recording `skipped_overlap`.

Two things break without rejection sampling, and the second is the one that
would quietly corrupt results rather than visibly fail:

1. The dense arms of the campaign do not run at all.
2. A scene lost to overlap is lost for EVERY arm that draws it, and the retry
   loop mutates `obstacles.pkl` in place with a fresh seed -- so two arms of the
   same `scene_id` can end up planning against different obstacle sets. The
   paired cross-arm comparison assumes they do not.

These tests therefore pin both the rejection property and the determinism that
makes the pairing valid.
"""
from __future__ import annotations

import pickle
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
GEN = REPO_ROOT / "data" / "points" / "generate_points_xyz.py"

HALF = 5.0                      # OBS_SIZE_DEFAULT
MARGIN = 0.05                   # sweep_offline.OBSTACLE_EXCLUDE_MARGIN
STARTS = np.array([[10.0, 0.0, 10.0], [50.0, 20.0, 10.0], [90.0, 0.0, 10.0]])


def _gen(tmp_path, n, bx, by, bz, seed, name, excl=None):
    out = tmp_path / name
    cmd = [sys.executable, str(GEN), "--n", str(n),
           "--x", str(bx[0]), str(bx[1]), "--y", str(by[0]), str(by[1]),
           "--z", str(bz[0]), str(bz[1]), "--seed", str(seed), "--out", str(out)]
    if excl is not None:
        cmd += ["--exclude_pkl", str(excl), "--exclude_halfsize", str(HALF),
                "--exclude_margin", str(MARGIN)]
    proc = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO_ROOT)
    assert proc.returncode == 0, f"generator failed:\n{proc.stdout}\n{proc.stderr}"
    with out.open("rb") as f:
        return np.asarray(pickle.load(f), dtype=float).reshape(-1, 3), out


def _conflicts(check_pts, obstacles, half=HALF):
    """The planner's own test, transcribed from DECK_GA_QuickNav.py:463-471."""
    n = 0
    for oc in obstacles:
        inside = np.all((check_pts >= oc - half) & (check_pts <= oc + half), axis=1)
        n += int(inside.sum())
    return n


@pytest.mark.parametrize("n_wp,n_obs", [(30, 20), (100, 50), (100, 100)])
@pytest.mark.parametrize("obs_z", [(5.0, 25.0), (0.0, 50.0)])
def test_rejection_sampling_leaves_no_conflicts(tmp_path, n_wp, n_obs, obs_z):
    wp, wp_pkl = _gen(tmp_path, n_wp, (-100, 100), (-100, 100), (0, 50), 0,
                      f"wp_{n_wp}.pkl")
    excl = tmp_path / f"excl_{n_wp}_{obs_z[1]:g}.pkl"
    with excl.open("wb") as f:
        pickle.dump(np.vstack([wp, STARTS]), f)

    obs, _ = _gen(tmp_path, n_obs, (-90, 90), (-90, 90), obs_z, 777,
                  f"obs_{n_obs}.pkl", excl=excl)
    assert obs.shape == (n_obs, 3)
    assert _conflicts(np.vstack([wp, STARTS]), obs) == 0


def test_a_start_inside_a_box_is_a_conflict():
    """Deterministic statement of WHY starts must be in the exclusion set.

    The planner validates `vstack([points, start_points])`
    (DECK_GA_QuickNav.py:463), so a box covering a start aborts the scene just
    as surely as one covering a waypoint. Hand-built rather than generated: the
    point is the rule, and a random draw would only reproduce it with some
    probability.
    """
    on_start = STARTS[:1].copy()                      # box centred on start 0
    assert _conflicts(STARTS, on_start) == 1

    # Exactly on a face still conflicts -- the test is inclusive (>=, <=), which
    # is why the harness adds OBSTACLE_EXCLUDE_MARGIN on top of the half-size.
    on_face = STARTS[:1] + np.array([[HALF, 0.0, 0.0]])
    assert _conflicts(STARTS, on_face) == 1

    just_clear = STARTS[:1] + np.array([[HALF * 2 + 0.01, 0.0, 0.0]])
    assert _conflicts(STARTS, just_clear) == 0


def test_sweep_builds_exclusion_set_from_waypoints_and_starts(tmp_path):
    """`_write_exclusion_pkl` must emit the planner's full check set.

    This is the wiring the campaign depends on: the generator can only avoid
    points it is told about, so if the harness passed waypoints alone, every
    scene would still be one unlucky draw away from an abort that takes the
    whole scene -- and therefore every arm -- with it.
    """
    sys.path.insert(0, str(REPO_ROOT))
    from tests.automation import config
    from tests.automation.sweep_offline import _write_exclusion_pkl

    wp = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]])
    points_pkl = tmp_path / "points.pkl"
    with points_pkl.open("wb") as f:
        pickle.dump(wp, f)

    trial = config.TrialSpec(trial_id="t", n_points=3, n_obstacles=5,
                             seed=0, num_uavs=3)
    out = _write_exclusion_pkl(trial, points_pkl, tmp_path / "excl.pkl")
    with out.open("rb") as f:
        got = np.asarray(pickle.load(f), dtype=float)

    expected_starts = np.array(
        [[float(v) for v in item.split(",")] for item in trial.start_points.split(";")]
    )
    assert got.shape == (len(wp) + len(expected_starts), 3)
    for row in wp:
        assert any(np.allclose(row, g) for g in got), f"waypoint {row} missing"
    for row in expected_starts:
        assert any(np.allclose(row, g) for g in got), f"start {row} missing"


def test_exclusion_set_rejects_start_count_mismatch(tmp_path):
    """A start/num_uavs mismatch must fail here, not later in the planner.

    `parse_start_points` requires shape (num_uavs, 3) exactly and does not
    truncate, so a mismatch is fatal anyway -- catching it before generation
    saves a wasted scene build inside an unattended campaign.
    """
    sys.path.insert(0, str(REPO_ROOT))
    from tests.automation import config
    from tests.automation.sweep_offline import _write_exclusion_pkl

    points_pkl = tmp_path / "p.pkl"
    with points_pkl.open("wb") as f:
        pickle.dump(np.array([[0.0, 0.0, 0.0]]), f)

    trial = config.TrialSpec(trial_id="t", n_points=1, n_obstacles=1,
                             seed=0, num_uavs=2)
    with pytest.raises(RuntimeError, match="start_points has 3 entries"):
        _write_exclusion_pkl(trial, points_pkl, tmp_path / "e.pkl")


def test_generation_is_deterministic_across_arms(tmp_path):
    """Same seed -> byte-identical scene, which is what makes pairing valid.

    Two arms of one `scene_id` generate independently; they must agree. This is
    the cheap half of gate G4 (the other half hashes the files the sweep
    actually wrote).
    """
    wp_a, _ = _gen(tmp_path, 60, (-100, 100), (-100, 100), (0, 50), 3, "a_wp.pkl")
    wp_b, _ = _gen(tmp_path, 60, (-100, 100), (-100, 100), (0, 50), 3, "b_wp.pkl")
    assert np.array_equal(wp_a, wp_b)

    excl = tmp_path / "e.pkl"
    with excl.open("wb") as f:
        pickle.dump(np.vstack([wp_a, STARTS]), f)
    o_a, _ = _gen(tmp_path, 40, (-90, 90), (-90, 90), (5, 25), 99, "a_o.pkl", excl=excl)
    o_b, _ = _gen(tmp_path, 40, (-90, 90), (-90, 90), (5, 25), 99, "b_o.pkl", excl=excl)
    assert np.array_equal(o_a, o_b)


def test_impossible_density_fails_loudly(tmp_path):
    """Over-dense scenes must raise, not silently place overlapping boxes.

    `--max_redraws` exists so an unsatisfiable request is reported instead of
    being quietly relaxed; a silent fallback here would hand the planner a scene
    it rejects, hours into an unattended campaign.
    """
    # 40 waypoints inside a 4 m box, then obstacles with half-size 5 m confined
    # to that same box: every candidate centre contains a waypoint.
    wp = np.column_stack([
        np.linspace(0, 4, 40), np.linspace(0, 4, 40), np.linspace(10, 14, 40)
    ])
    excl = tmp_path / "dense.pkl"
    with excl.open("wb") as f:
        pickle.dump(wp, f)

    out = tmp_path / "impossible.pkl"
    proc = subprocess.run(
        [sys.executable, str(GEN), "--n", "5",
         "--x", "0", "4", "--y", "0", "4", "--z", "10", "14",
         "--seed", "1", "--out", str(out),
         "--exclude_pkl", str(excl), "--exclude_halfsize", str(HALF),
         "--exclude_margin", str(MARGIN), "--max_redraws", "50"],
        capture_output=True, text=True, cwd=REPO_ROOT,
    )
    assert proc.returncode != 0, "expected a loud failure, got success"
    assert "could not place box" in (proc.stdout + proc.stderr)
    assert not out.exists(), "no output file may be written on failure"
