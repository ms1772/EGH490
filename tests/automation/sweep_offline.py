#!/usr/bin/env python3
"""Stage 0 + Stage 1: Offline matrix sweep driver.

Per trial:
  1. Generate (or copy) waypoint pickle
  2. Generate (or copy) obstacle pickle
  3. Run DECK_GA_QuickNav.py as a subprocess, captured + timed
  4. In-process re-measure of QuickNav-only timing for the deckga/quicknav split
  5. Compute metrics from the output pickle
  6. Append a row to offline_master.csv (atomic per-row write -> resumable)

Usage:
    python tests/automation/sweep_offline.py --matrix smoke
    python tests/automation/sweep_offline.py --matrix report
    python tests/automation/sweep_offline.py --matrix report --resume <run_dir>
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import pickle
import shutil
import subprocess
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tests.automation import config, metrics  # noqa: E402
from tests.automation.config import TrialSpec  # noqa: E402

# 180 s was sized for the Euclidean planner alone. The obstacle-aware arms add
# a pairwise cost matrix -- measured 519 s at n=100 with 100 obstacles -- and the
# GA itself runs ~12 s per cluster. A cache hit makes most trials fast, but the
# first trial of a cold scene pays the build, and a timeout there would be
# recorded as a generic subprocess_error with nothing to point at.
# Raised from 1200 for the larger fleets. Planner cost grows super-linearly in
# fleet size (46 s -> 92 s per trial from three to six UAVs, measured), and at
# twelve UAVs with 100 obstacles the cost matrix alone can reach 519 s. A
# timeout is recorded as a FAILED trial, so a limit set too low does not slow
# the campaign down -- it silently deletes its largest data points.
PLANNER_TIMEOUT_S = 3600
PLANNER_SUBPROCESS_OVERHEAD_S = 0.3  # empirical Python startup cost, subtracted from DECK-GA split


# ---------- precondition probe ----------

def probe_preconditions() -> None:
    missing = []
    try:
        import numpy  # noqa: F401
    except ImportError:
        missing.append("numpy")
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        missing.append("matplotlib")
    # DECK_GA_QuickNav.py must exist
    if not (REPO_ROOT / "DECK_GA_QuickNav.py").exists():
        missing.append("DECK_GA_QuickNav.py (run from repo root)")
    if not (REPO_ROOT / "data" / "points" / "generate_points_xyz.py").exists():
        missing.append("data/points/generate_points_xyz.py")
    if missing:
        for m in missing:
            print(f"MISSING: {m}", file=sys.stderr)
        sys.exit(1)


# ---------- row schema ----------

CSV_HEADER = [
    "trial_id", "n_points", "n_obstacles", "seed", "num_uavs", "status",
    "distance_baseline_total_m", "distance_avoided_total_m",
    "distance_baseline_uav0_m", "distance_baseline_uav1_m", "distance_baseline_uav2_m",
    "distance_avoided_uav0_m", "distance_avoided_uav1_m", "distance_avoided_uav2_m",
    "detections_uav0", "detections_uav1", "detections_uav2", "detections_total",
    "avoidances_uav0", "avoidances_uav1", "avoidances_uav2", "avoidances_total",
    "residual_collisions_uav0", "residual_collisions_uav1", "residual_collisions_uav2",
    "residual_collisions_total",
    "planning_time_total_s", "planning_time_deckga_s", "planning_time_quicknav_s",
    "quicknav_warning",
    # --- campaign additions -------------------------------------------------
    # Which arm produced this row, and on which scene. scene_id is what makes a
    # cross-arm paired comparison possible: arms differ, the scene must not.
    "arm", "cluster_method", "cost_kind", "objective", "planner_seed",
    "scene_id", "obs_size",
    # Makespan is the fleet metric; total distance hides a 34% imbalance.
    "max_tour_baseline_m", "max_tour_avoided_m",
    "makespan_baseline_s", "makespan_avoided_s",
    "balance_ratio_baseline", "balance_ratio_avoided", "wp_avoided_max",
    "time_model_v_eff", "time_model_t_stop",
    # Cost-oracle health. n_detour_edges == 0 with obstacles present means the
    # matrix was built in the wrong coordinate frame and the obstacle-aware arm
    # silently degenerated into the Euclidean one.
    "n_detour_edges", "n_infeasible_edges", "n_unreachable_waypoints",
    "cost_cache_hit",
    # Real stage timings from the planner, not a residual subtraction.
    "t_cost_matrix_s", "t_cluster_s", "t_balance_s", "t_ga_s", "t_quicknav_s",
    "trial_dir", "started_at", "finished_at", "error",
]


def _assert_header_covers_metrics() -> None:
    """Fail loudly if a metric would be dropped on the floor.

    csv.DictWriter(extrasaction="ignore") discards unknown keys SILENTLY. Add a
    metric to compute_all_metrics, forget the header, and the column simply
    never appears -- no error, no warning, and the gap is only noticed when the
    analysis comes up empty. Checked at import so it cannot reach a campaign.
    """
    missing = [k for k in metrics.CSV_EXPORTED_KEYS if k not in CSV_HEADER]
    if missing:
        raise RuntimeError(
            f"CSV_HEADER is missing exported metric(s) {missing}. They would be "
            f"silently dropped by DictWriter(extrasaction='ignore')."
        )


_assert_header_covers_metrics()


def _per_uav_fields(values, n=3):
    out = list(values) + [""] * max(0, n - len(values))
    return out[:n]


def _append_row(csv_path: Path, row: dict) -> None:
    new_file = not csv_path.exists()
    with csv_path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_HEADER, extrasaction="ignore")
        if new_file:
            writer.writeheader()
        writer.writerow(row)
        f.flush()
        os.fsync(f.fileno())


def _existing_ok_trial_ids(csv_path: Path) -> set:
    """Trial IDs with status=ok. Non-ok rows are not treated as 'done' — they
    get retried, since the user almost always wants a non-ok row re-attempted
    (e.g. after a bug fix)."""
    if not csv_path.exists():
        return set()
    with csv_path.open("r", encoding="utf-8") as f:
        return {r["trial_id"] for r in csv.DictReader(f) if r.get("status") == "ok"}


def _purge_non_ok_rows(csv_path: Path) -> int:
    """Drop any row whose status != 'ok' so the re-attempt writes fresh."""
    if not csv_path.exists():
        return 0
    with csv_path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return 0
    keep = [r for r in rows if r.get("status") == "ok"]
    dropped = len(rows) - len(keep)
    if dropped == 0:
        return 0
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_HEADER, extrasaction="ignore")
        w.writeheader()
        w.writerows(keep)
    return dropped


# ---------- generation ----------

def _generate_points(trial: TrialSpec, out_pkl: Path, log_path: Path) -> None:
    """Run generate_points_xyz.py for waypoints. Uses planner-scope bounds."""
    cmd = [
        sys.executable,
        str(REPO_ROOT / "data" / "points" / "generate_points_xyz.py"),
        "--n", str(trial.n_points),
        "--x", str(config.WAYPOINT_BOUNDS_X[0]), str(config.WAYPOINT_BOUNDS_X[1]),
        "--y", str(config.WAYPOINT_BOUNDS_Y[0]), str(config.WAYPOINT_BOUNDS_Y[1]),
        "--z", str(config.WAYPOINT_BOUNDS_Z[0]), str(config.WAYPOINT_BOUNDS_Z[1]),
        "--seed", str(trial.seed),
        "--out", str(out_pkl),
    ]
    with log_path.open("a", encoding="utf-8") as logf:
        logf.write(f"$ {' '.join(cmd)}\n")
        proc = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO_ROOT)
        logf.write(proc.stdout)
        logf.write(proc.stderr)
    if proc.returncode != 0:
        raise RuntimeError(f"generate_points_xyz failed (exit {proc.returncode}); see {log_path}")


# Clearance added to obs_size when rejecting obstacle placements. The planner's
# containment test is INCLUSIVE (`>=` / `<=`, DECK_GA_QuickNav.py:468), so a
# waypoint landing exactly on a face is a conflict; a bare half-size comparison
# would leave that boundary case live. Small relative to OBS_SIZE_DEFAULT=5.0,
# so it costs almost no placement freedom.
#
# Lives here rather than in config.py deliberately: config.py is imported by
# sweep_gazebo.py, and this was added while a Gazebo calibration was mid-flight.
OBSTACLE_EXCLUDE_MARGIN = 0.05


def _write_exclusion_pkl(trial: TrialSpec, points_pkl: Path, out_pkl: Path) -> Path:
    """Waypoints + start points, the exact set the planner validates.

    `DECK_GA_QuickNav.py:463` checks `vstack([points, start_points])`, so
    excluding only the waypoints would still let an obstacle swallow a start
    point and abort the scene. Order does not matter -- the generator only tests
    containment -- but the CONTENTS must match that line or the rejection
    sampling is incomplete.
    """
    import pickle as _pickle

    import numpy as _np

    with points_pkl.open("rb") as f:
        wp = _np.asarray(_pickle.load(f), dtype=float).reshape(-1, 3)

    sp = _np.asarray(
        [[float(v) for v in item.split(",")] for item in trial.start_points.split(";")],
        dtype=float,
    ).reshape(-1, 3)
    # parse_start_points demands shape (num_uavs, 3) exactly and does NOT
    # truncate, so a mismatch here is a config bug that would fail the planner
    # anyway -- catch it before burning a generation.
    if sp.shape[0] != trial.num_uavs:
        raise RuntimeError(
            f"{trial.trial_id}: start_points has {sp.shape[0]} entries but "
            f"num_uavs={trial.num_uavs}"
        )

    with out_pkl.open("wb") as f:
        _pickle.dump(_np.vstack([wp, sp]), f)
    return out_pkl


def _generate_obstacles(trial: TrialSpec, seed_offset: int, out_pkl: Path, log_path: Path,
                        points_pkl: Optional[Path] = None) -> None:
    """Run generate_points_xyz.py for obstacles. Different bounds; decorrelated seed.

    When `points_pkl` is given, obstacle centres are drawn by REJECTION SAMPLING
    against the waypoints and start points, so no obstacle can contain one. At
    n=100/M=100 roughly 4.5 waypoints land inside an obstacle by pure chance,
    which trips the planner's hard abort; the four-seed retry loop below then
    rerolls and usually fails again, recording `skipped_overlap`. The dense arms
    of the campaign do not run at all without this.

    It also makes the scene a deterministic function of (seed, n_points,
    n_obstacles): because the retry loop never has to fire, two arms of the same
    `scene_id` are generated identically instead of diverging onto different
    obstacle sets.
    """
    cmd = [
        sys.executable,
        str(REPO_ROOT / "data" / "points" / "generate_points_xyz.py"),
        "--n", str(trial.n_obstacles),
        "--x", str(config.OBSTACLE_BOUNDS_X[0]), str(config.OBSTACLE_BOUNDS_X[1]),
        "--y", str(config.OBSTACLE_BOUNDS_Y[0]), str(config.OBSTACLE_BOUNDS_Y[1]),
        "--z", str(config.OBSTACLE_BOUNDS_Z[0]), str(config.OBSTACLE_BOUNDS_Z[1]),
        "--seed", str(trial.seed + seed_offset),
        "--out", str(out_pkl),
    ]
    if points_pkl is not None and points_pkl.exists():
        excl_pkl = _write_exclusion_pkl(trial, points_pkl, out_pkl.parent / "exclusion_points.pkl")
        cmd += [
            "--exclude_pkl", str(excl_pkl),
            "--exclude_halfsize", str(trial.obs_size),
            "--exclude_margin", str(OBSTACLE_EXCLUDE_MARGIN),
        ]
    with log_path.open("a", encoding="utf-8") as logf:
        logf.write(f"$ {' '.join(cmd)}\n")
        proc = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO_ROOT)
        logf.write(proc.stdout)
        logf.write(proc.stderr)
    if proc.returncode != 0:
        raise RuntimeError(f"generate_points_xyz (obstacles) failed (exit {proc.returncode}); see {log_path}")


def _prepare_inputs(trial: TrialSpec, trial_dir: Path) -> tuple[Path, Path, int]:
    """Ensure points.pkl and obstacles.pkl exist for this trial. Returns paths
    and the obstacle seed_offset actually used (relevant for retries)."""
    points_pkl = trial_dir / "points.pkl"
    obstacles_pkl = trial_dir / "obstacles.pkl"
    log = trial_dir / "planner.log"

    # Points
    if trial.points_fixture is not None:
        shutil.copy(trial.points_fixture, points_pkl)
    else:
        _generate_points(trial, points_pkl, log)

    # Obstacles
    if trial.obstacles_fixture is not None:
        shutil.copy(trial.obstacles_fixture, obstacles_pkl)
        seed_offset_used = config.SEED_OBSTACLE_OFFSET
    elif trial.n_obstacles == 0:
        shutil.copy(REPO_ROOT / "tests" / "automation" / "fixtures" / "obstacles_empty.pkl", obstacles_pkl)
        seed_offset_used = config.SEED_OBSTACLE_OFFSET
    else:
        _generate_obstacles(trial, config.SEED_OBSTACLE_OFFSET, obstacles_pkl, log,
                            points_pkl=points_pkl)
        seed_offset_used = config.SEED_OBSTACLE_OFFSET

    return points_pkl, obstacles_pkl, seed_offset_used


# ---------- planner subprocess ----------

class PlannerOverlapError(Exception):
    """Raised when DECK_GA_QuickNav reports a waypoint/start inside an obstacle."""


def _is_legacy_trial(trial: TrialSpec) -> bool:
    """Mirror of `DECK_GA_QuickNav._is_legacy`, for argv construction.

    MUST STAY IN SYNC with that predicate. It is an AND over every campaign
    flag, so any one of them leaking into A0's argv silently reroutes the
    published arm away from the branch the byte-identity guarantee depends on.
    `--arm` and `--no_plot` are deliberately absent from it, which is why they
    are safe to pass unconditionally.
    """
    return (trial.cluster == "kmeans"
            and trial.cost_kind == "euclidean"
            and trial.objective == "minsum"
            and trial.planner_seed is None)


def _run_planner(trial: TrialSpec, points_pkl: Path, obstacles_pkl: Path,
                 out_pkl: Path, log_path: Path,
                 cost_cache_dir: Optional[Path] = None) -> float:
    """Subprocess-call DECK_GA_QuickNav.py, return total wall time. Raises
    PlannerOverlapError on the known 'inside an obstacle cube' validation,
    or RuntimeError on any other non-zero exit."""
    cmd = [
        sys.executable, str(REPO_ROOT / "DECK_GA_QuickNav.py"),
        "--points_pkl", str(points_pkl),
        "--obstacles_pkl", str(obstacles_pkl),
        "--obs_size", str(trial.obs_size),
        "--num_uavs", str(trial.num_uavs),
        "--start_points", trial.start_points,
        "--no_plot",
        "--out_pkl", str(out_pkl),
    ]
    # Campaign arm flags, appended only when NON-DEFAULT, so an unchanged
    # TrialSpec produces the exact argv the planner has always received. That
    # is what makes the A0 byte-identity guarantee hold end to end rather than
    # only inside the planner.
    if trial.cluster != "kmeans":
        cmd += ["--cluster", trial.cluster]
    if trial.cost_kind != "euclidean":
        cmd += ["--cost", trial.cost_kind]
    if trial.objective != "minsum":
        cmd += ["--objective", trial.objective]
    if trial.planner_seed is not None:
        cmd += ["--planner_seed", str(trial.planner_seed)]
    if trial.arm:
        cmd += ["--arm", trial.arm]
    # The cache directory is ALSO a campaign flag as far as the planner is
    # concerned: `_is_legacy` (DECK_GA_QuickNav.py:513) is an AND that includes
    # `cost_cache_dir is None`. Passing it to the published arm diverts A0 off
    # the byte-identical legacy branch into `_run_arm`, which then refuses
    # `--cluster kmeans` and fails the trial outright. A0 builds no cost matrix,
    # so it has no use for the cache anyway.
    if cost_cache_dir is not None and not _is_legacy_trial(trial):
        cmd += ["--cost_cache_dir", str(cost_cache_dir)]
    with log_path.open("a", encoding="utf-8") as logf:
        logf.write(f"\n$ {' '.join(cmd)}\n")
    t0 = time.perf_counter()
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True,
            cwd=REPO_ROOT, timeout=PLANNER_TIMEOUT_S,
        )
    except subprocess.TimeoutExpired as e:
        with log_path.open("a", encoding="utf-8") as logf:
            logf.write(f"\n[harness] TIMEOUT after {PLANNER_TIMEOUT_S}s\n")
            if e.stdout:
                logf.write(e.stdout if isinstance(e.stdout, str) else e.stdout.decode("utf-8", "replace"))
            if e.stderr:
                logf.write(e.stderr if isinstance(e.stderr, str) else e.stderr.decode("utf-8", "replace"))
        raise RuntimeError(f"planner timeout after {PLANNER_TIMEOUT_S}s")
    elapsed = time.perf_counter() - t0
    with log_path.open("a", encoding="utf-8") as logf:
        logf.write(proc.stdout)
        logf.write(proc.stderr)
    if proc.returncode != 0:
        combined = (proc.stdout or "") + (proc.stderr or "")
        if "inside an obstacle" in combined:
            raise PlannerOverlapError(combined.split("inside an obstacle", 1)[0][-200:])
        raise RuntimeError(f"planner exit {proc.returncode}; see {log_path}")
    return elapsed


def _measure_quicknav_only(out_pkl: Path) -> float:
    """In-process re-run of apply_quicknav on the loaded deckga_paths to get
    a clean QuickNav-only wall-clock (excludes Python startup, GA timing).
    """
    from QuickNav import apply_quicknav  # noqa: E402
    with out_pkl.open("rb") as f:
        data = pickle.load(f)
    obstacle_xyz = data["obstacle_xyz"]
    obs_size = data["obs_size"]
    deckga_paths = data["deckga_paths"]
    t0 = time.perf_counter()
    for path in deckga_paths:
        apply_quicknav(path, obstacle_xyz, obs_size)
    return time.perf_counter() - t0


# ---------- trial execution ----------

def _detect_quicknav_warning(log_path: Path) -> str:
    if not log_path.exists():
        return ""
    with log_path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if "[QuickNav] WARNING" in line:
                return line.strip()
    return ""


def _trial_to_row(i: int, trial: TrialSpec, run_dir: Path, n_total: int) -> dict:
    """One trial -> one CSV row, with every failure contained.

    `_run_single_trial` guards its subprocess block and its metrics block, but
    NOT the per-UAV flattening, the warning scan or the row assembly -- and the
    sweep loop itself was unguarded. In an unattended campaign a single
    malformed trial would abort the whole sweep hours in, and its partial row
    would never be written, so the failure would not even be visible in the CSV
    afterwards.

    Record it, keep going. A campaign that loses one trial is a campaign with
    one missing row; a campaign that dies at 3am is a lost night.

    Returns the row with a private `_elapsed_s` the caller pops before writing.
    """
    trial_dir = run_dir / "trials" / trial.trial_id
    t_trial = time.perf_counter()
    print(f"[{i}/{n_total}] {trial.trial_id} n={trial.n_points} "
          f"obs={trial.n_obstacles} seed={trial.seed} "
          f"arm={trial.arm or 'legacy'}...", flush=True)
    try:
        row = _run_single_trial(trial, trial_dir,
                                cost_cache_dir=run_dir / "cost_cache")
    except KeyboardInterrupt:
        raise
    except BaseException as exc:                          # noqa: BLE001
        row = {
            "trial_id": trial.trial_id,
            "n_points": trial.n_points,
            "n_obstacles": trial.n_obstacles,
            "seed": trial.seed,
            "num_uavs": trial.num_uavs,
            "status": "harness_error",
            "trial_dir": str(trial_dir),
            "started_at": datetime.now().isoformat(timespec="seconds"),
            "finished_at": datetime.now().isoformat(timespec="seconds"),
            "error": f"{type(exc).__name__}: {exc}",
        }
        try:
            trial_dir.mkdir(parents=True, exist_ok=True)
            (trial_dir / "harness_error.txt").write_text(
                traceback.format_exc(), encoding="utf-8")
        except Exception:
            pass              # never let error handling raise
    row["_elapsed_s"] = time.perf_counter() - t_trial
    return row


def _run_single_trial(trial: TrialSpec, trial_dir: Path,
                      cost_cache_dir: Optional[Path] = None) -> dict:
    trial_dir.mkdir(parents=True, exist_ok=True)
    log_path = trial_dir / "planner.log"
    out_pkl = trial_dir / "deckga_quicknav_output.pkl"

    started_at = datetime.now().isoformat(timespec="seconds")
    row: dict = {
        "trial_id": trial.trial_id, "n_points": trial.n_points,
        "n_obstacles": trial.n_obstacles, "seed": trial.seed, "num_uavs": trial.num_uavs,
        "trial_dir": str(trial_dir), "started_at": started_at,
    }

    # Retry loop on overlap (up to 3 obstacle regenerations with different seeds)
    seed_offsets_to_try = [
        config.SEED_OBSTACLE_OFFSET,
        config.SEED_OBSTACLE_OFFSET + 100_000,
        config.SEED_OBSTACLE_OFFSET + 200_000,
        config.SEED_OBSTACLE_OFFSET + 300_000,
    ]

    planner_time_total = 0.0
    attempt_ok = False
    last_overlap_msg = ""
    for attempt_idx, seed_offset in enumerate(seed_offsets_to_try):
        try:
            points_pkl, obstacles_pkl, _ = _prepare_inputs(trial, trial_dir)
            # If retry: regenerate obstacles only (points are deterministic)
            if attempt_idx > 0 and trial.obstacles_fixture is None and trial.n_obstacles > 0:
                _generate_obstacles(trial, seed_offset, obstacles_pkl, log_path,
                                    points_pkl=points_pkl)
            planner_time_total = _run_planner(trial, points_pkl, obstacles_pkl,
                                              out_pkl, log_path,
                                              cost_cache_dir=cost_cache_dir)
            attempt_ok = True
            break
        except PlannerOverlapError as e:
            last_overlap_msg = str(e)
            with log_path.open("a", encoding="utf-8") as logf:
                logf.write(f"\n[harness] overlap on attempt {attempt_idx + 1}; will retry with new seed.\n")
            # Fixtures-only trial: do not retry, can't change the fixture
            if trial.obstacles_fixture is not None or trial.n_obstacles == 0:
                break
            continue
        except Exception as e:
            row.update({
                "status": "subprocess_error",
                "error": str(e),
                "finished_at": datetime.now().isoformat(timespec="seconds"),
            })
            return row

    if not attempt_ok:
        row.update({
            "status": "skipped_overlap",
            "error": last_overlap_msg or "waypoint/start inside obstacle after all retries",
            "finished_at": datetime.now().isoformat(timespec="seconds"),
        })
        return row

    # In-process timing split
    try:
        quicknav_time = _measure_quicknav_only(out_pkl)
    except Exception as e:
        quicknav_time = float("nan")
        with log_path.open("a", encoding="utf-8") as logf:
            logf.write(f"[harness] WARN: in-process QuickNav timing failed: {e}\n")

    deckga_time = max(0.0, planner_time_total - quicknav_time - PLANNER_SUBPROCESS_OVERHEAD_S)

    # Compute metrics
    try:
        with out_pkl.open("rb") as f:
            data = pickle.load(f)
        m = metrics.compute_all_metrics(data)
        # If a fixture was used, the TrialSpec's n_points/n_obstacles are
        # placeholders (0). Override with the actual values from the pickle so
        # downstream aggregation groups correctly.
        if trial.points_fixture is not None:
            row["n_points"] = int(data.get("num_points", trial.n_points))
        if trial.obstacles_fixture is not None or trial.n_obstacles == 0:
            row["n_obstacles"] = int(len(data["obstacle_xyz"]))
    except Exception as e:
        row.update({
            "status": "parse_error",
            "error": f"{type(e).__name__}: {e}\n{traceback.format_exc()}",
            "finished_at": datetime.now().isoformat(timespec="seconds"),
        })
        return row

    qn_warning = _detect_quicknav_warning(log_path)
    n_uav = m["num_uavs"]
    dist_b = _per_uav_fields(m["distance_baseline_per_uav_m"], n=max(n_uav, 3))
    dist_a = _per_uav_fields(m["distance_avoided_per_uav_m"], n=max(n_uav, 3))
    det = _per_uav_fields(m["detections_per_uav"], n=max(n_uav, 3))
    av = _per_uav_fields(m["avoidances_per_uav"], n=max(n_uav, 3))
    res = _per_uav_fields(m["residual_collisions_per_uav"], n=max(n_uav, 3))

    row.update({
        "status": "ok",
        "distance_baseline_total_m": m["distance_baseline_total_m"],
        "distance_avoided_total_m": m["distance_avoided_total_m"],
        "distance_baseline_uav0_m": dist_b[0],
        "distance_baseline_uav1_m": dist_b[1],
        "distance_baseline_uav2_m": dist_b[2],
        "distance_avoided_uav0_m": dist_a[0],
        "distance_avoided_uav1_m": dist_a[1],
        "distance_avoided_uav2_m": dist_a[2],
        "detections_uav0": det[0], "detections_uav1": det[1], "detections_uav2": det[2],
        "detections_total": m["detections_total"],
        "avoidances_uav0": av[0], "avoidances_uav1": av[1], "avoidances_uav2": av[2],
        "avoidances_total": m["avoidances_total"],
        "residual_collisions_uav0": res[0],
        "residual_collisions_uav1": res[1],
        "residual_collisions_uav2": res[2],
        "residual_collisions_total": m["residual_collisions_total"],
        "planning_time_total_s": planner_time_total,
        "planning_time_deckga_s": deckga_time,
        "planning_time_quicknav_s": quicknav_time,
        "quicknav_warning": qn_warning,
        "finished_at": datetime.now().isoformat(timespec="seconds"),
        "error": "",
    })

    # --- campaign columns ---------------------------------------------------
    # These were computed and the header was widened for them, but nothing ever
    # copied them into the row, so every campaign column came out EMPTY. The
    # import-time header assertion did not catch it: it checks that CSV_HEADER
    # *contains* each exported key, not that any row ever supplies one. A sweep
    # therefore looked completely healthy -- status=ok, 59 columns -- while
    # carrying none of the data the campaign exists to collect.
    #
    # Driven off metrics.CSV_EXPORTED_KEYS rather than a second hand-written
    # list, so a metric added there cannot be silently forgotten here.
    for k in metrics.CSV_EXPORTED_KEYS:
        if k in m:
            row[k] = m[k]

    # Which arm, and which scene. `scene_id` is what makes the paired cross-arm
    # comparison possible: the arm differs, the scene must not.
    row.update({
        "arm": trial.arm or "",
        "cluster_method": trial.cluster,
        "cost_kind": trial.cost_kind,
        "objective": trial.objective,
        "planner_seed": "" if trial.planner_seed is None else trial.planner_seed,
        "scene_id": trial.scene_id,
        "obs_size": trial.obs_size,
    })

    # Cost-oracle health and real per-stage timings, straight from the planner
    # rather than inferred by subtraction. n_detour_edges == 0 with obstacles
    # present is the tripwire for a matrix built in the wrong frame, which makes
    # an obstacle-aware arm degenerate into the Euclidean one without raising.
    cmeta = data.get("cost_matrix_meta") or {}
    stages = data.get("stage_times_s") or {}
    row.update({
        "n_detour_edges": cmeta.get("n_detour_edges", ""),
        "n_infeasible_edges": cmeta.get("n_infeasible_edges", ""),
        "n_unreachable_waypoints": data.get("n_unreachable_waypoints", ""),
        "cost_cache_hit": cmeta.get("cache_hit", ""),
        "t_cost_matrix_s": stages.get("cost_matrix_s", ""),
        "t_cluster_s": stages.get("cluster_s", ""),
        "t_balance_s": stages.get("balance_s", ""),
        "t_ga_s": stages.get("ga_s", ""),
        "t_quicknav_s": stages.get("quicknav_s", ""),
    })
    return row


def _check_campaign_columns(rows: list) -> list:
    """Complaints about ok rows that carry no campaign data. [] means healthy.

    `_assert_header_covers_metrics` runs at import and proves only that
    CSV_HEADER *contains* each exported metric key. It cannot notice that no
    row ever supplies one -- which is exactly what happened: the sweep reported
    healthy trials with all 59 columns present and every campaign column blank,
    because the row assembly was never taught to copy them. A schema check
    cannot catch a data failure, so this checks the data.
    """
    required = ["scene_id", "cluster_method", "cost_kind", "objective",
                "max_tour_avoided_m", "makespan_avoided_s",
                "balance_ratio_avoided"]
    problems = []
    for r in rows:
        if r.get("status") != "ok":
            continue
        blank = [k for k in required if r.get(k) in (None, "")]
        if blank:
            problems.append(f"{r.get('trial_id')}: blank {blank}")
        # An obstacle-aware arm that found no detours did not price any
        # obstacle, so it is silently the Euclidean arm wearing a new label --
        # the single most damaging way this campaign could produce clean-looking
        # but meaningless results.
        if r.get("cost_kind") == "quicknav":
            try:
                n_obs = int(float(r.get("n_obstacles") or 0))
                n_det = int(float(r.get("n_detour_edges") or 0))
            except (TypeError, ValueError):
                continue
            if n_obs > 0 and n_det == 0:
                problems.append(
                    f"{r.get('trial_id')}: quicknav cost found 0 detour edges "
                    f"against {n_obs} obstacles -- this arm has degenerated "
                    f"into the euclidean one")
    return problems


# ---------- smoke assertions ----------

def _smoke_assertions(rows: list, run_dir: Path) -> bool:
    """Per the plan's Stage 0 assertion contract. Returns True if all pass."""
    print("\n=== SMOKE ASSERTIONS ===")
    all_pass = True

    # (a) all trials have residual_collisions_total == 0
    for r in rows:
        rc = r.get("residual_collisions_total")
        if r["status"] != "ok":
            print(f"  [FAIL] trial {r['trial_id']} status={r['status']} (expected ok)")
            all_pass = False
            continue
        if rc not in (0, "0", 0.0):
            print(f"  [FAIL] trial {r['trial_id']} residual_collisions_total={rc} (expected 0)")
            all_pass = False
        else:
            print(f"  [ok ] trial {r['trial_id']} residual_collisions_total=0")

    # (b) no [QuickNav] WARNING in any log
    for r in rows:
        w = r.get("quicknav_warning") or ""
        if w:
            print(f"  [FAIL] trial {r['trial_id']} QuickNav warning: {w}")
            all_pass = False

    # (c) and (d) overlap-fixture trial specific
    overlap_row = next((r for r in rows if r["trial_id"] == "smoke_overlap_bugA"), None)
    if overlap_row is None:
        print("  [warn] no overlap-fixture trial found, skipping bug-A assertions")
    elif overlap_row["status"] != "ok":
        print(f"  [FAIL] overlap trial status={overlap_row['status']}")
        all_pass = False
    else:
        # Load the pickle and check that quicknav differs from deckga for at least one UAV
        trial_dir = Path(overlap_row["trial_dir"])
        with (trial_dir / "deckga_quicknav_output.pkl").open("rb") as f:
            data = pickle.load(f)
        differs = False
        for d, q in zip(data["deckga_paths"], data["quicknav_paths"]):
            if d.shape != q.shape or not np.allclose(d, q):
                differs = True
                break
        if differs:
            print("  [ok ] overlap trial: quicknav_paths differs from deckga_paths (avoidance ran)")
        else:
            print("  [FAIL] overlap trial: quicknav_paths IDENTICAL to deckga_paths "
                  "(avoidance did not run -- bug A may have regressed)")
            all_pass = False
        # residual_collisions==0 already covered by (a) — that's the direct bug-A proof

    print(f"=== SMOKE {'PASS' if all_pass else 'FAIL'} ===\n")
    return all_pass


# ---------- main ----------

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", required=True, choices=sorted(config.MATRICES))
    parser.add_argument("--run-dir", default=None,
                        help="Existing run dir to resume into. If omitted, a new timestamped dir is created.")
    parser.add_argument("--jobs", type=int, default=1, metavar="N",
                        help="Run N scene groups concurrently. The sweep is "
                             "single-threaded otherwise and pins one core of "
                             "twenty; trials of one scene always stay on one "
                             "worker so they share the cost-matrix cache "
                             "instead of racing to rebuild it. Default 1 "
                             "(serial), which is what timing metrics should be "
                             "measured with.")
    parser.add_argument("--results-root",
                        default=str(REPO_ROOT / "tests" / "automation" / "results"))
    args = parser.parse_args()

    probe_preconditions()

    if args.run_dir:
        run_dir = Path(args.run_dir).resolve()
        run_dir.mkdir(parents=True, exist_ok=True)
        print(f"[resume] using existing run dir: {run_dir}")
    else:
        ts = datetime.now().strftime("%Y%m%d-%H%M%S")
        run_dir = Path(args.results_root) / ts
        run_dir.mkdir(parents=True, exist_ok=True)
        print(f"[new] run dir: {run_dir}")

    trials = config.resolve_matrix(args.matrix)
    csv_path = run_dir / "offline_master.csv"
    jsonl_path = run_dir / "sweep.jsonl"
    dropped = _purge_non_ok_rows(csv_path)
    if dropped:
        print(f"[resume] dropped {dropped} prior non-ok row(s) so they retry")
    done_ids = _existing_ok_trial_ids(csv_path)
    if done_ids:
        print(f"[resume] skipping {len(done_ids)} already-completed trial(s)")

    rows_this_run = []
    t_sweep_start = time.perf_counter()

    pending = [(i, t) for i, t in enumerate(trials, 1)
               if t.trial_id not in done_ids]
    for i, trial in enumerate(trials, 1):
        if trial.trial_id in done_ids:
            print(f"[{i}/{len(trials)}] {trial.trial_id} -- already done, skipping")

    # Rows are written AS EACH TRIAL FINISHES, including from worker threads,
    # not batched when a scene group completes. Per-row persistence is what
    # makes the sweep resumable on `trial_id`: a campaign killed mid-scene must
    # keep the trials it already paid for. `_append_row` opens, appends and
    # fsyncs, so one lock around it is enough to keep rows from interleaving.
    _write_lock = threading.Lock()

    def _persist(row: dict) -> None:
        # Guarded because a disk that filled mid-campaign must not take the
        # remaining trials with it.
        try:
            with _write_lock:
                _append_row(csv_path, row)
                with jsonl_path.open("a", encoding="utf-8") as jf:
                    jf.write(json.dumps(row, default=str) + "\n")
                rows_this_run.append(row)
        except Exception as exc:                          # noqa: BLE001
            print(f"    [warn] could not persist row: {type(exc).__name__}: {exc}")

    if args.jobs > 1:
        # PARALLELISM IS GROUPED BY SCENE, not spread over individual trials.
        #
        # Measured: the serial sweep pins exactly one core of twenty (4-5% user
        # on 20 cores), and the GA is ~12 s per cluster per 1e5 iterations, so
        # S3 is hours of one-core work with 19 cores idle.
        #
        # But trials of the same scene share a cost matrix, and scattering them
        # across workers would have several of them miss the cache and build it
        # simultaneously -- at n=100/M=100 that is a 519 s build duplicated.
        # Keeping a scene on one worker means its first trial builds the matrix
        # and the rest hit it, so grouping both removes the race and keeps the
        # cache useful. Parallelism then comes from running different scenes at
        # once, of which the campaign has 60.
        #
        # Threads, not processes: each trial's real work is already in a
        # subprocess, so the pool only waits on them and the GIL is irrelevant.
        from collections import OrderedDict                     # noqa: PLC0415
        from concurrent.futures import ThreadPoolExecutor, as_completed  # noqa: PLC0415

        groups: "OrderedDict[str, list]" = OrderedDict()
        for i, trial in pending:
            groups.setdefault(trial.scene_id, []).append((i, trial))

        def _run_group(items):
            """One scene's trials, in order, each persisted as it finishes."""
            for i, t in items:
                row = _trial_to_row(i, t, run_dir, len(trials))
                print(f"  [{row.get('trial_id')}] {row.get('status')} "
                      f"in {row.pop('_elapsed_s', 0.0):.1f}s  "
                      f"residual={row.get('residual_collisions_total','?')}",
                      flush=True)
                _persist(row)

        print(f"[parallel] {len(pending)} trial(s) in {len(groups)} scene "
              f"group(s), {args.jobs} worker(s)")
        with ThreadPoolExecutor(max_workers=args.jobs) as ex:
            futs = {ex.submit(_run_group, items): scene
                    for scene, items in groups.items()}
            for fut in as_completed(futs):
                scene = futs[fut]
                try:
                    fut.result()
                except BaseException as exc:              # noqa: BLE001
                    # `_trial_to_row` already contains per-trial failures, so
                    # reaching here means the group itself broke. Its remaining
                    # trials are simply absent; say which scene, loudly.
                    print(f"  [scene {scene}] GROUP FAILED after its persisted "
                          f"rows: {type(exc).__name__}: {exc}")
    else:
        for i, trial in pending:
            row = _trial_to_row(i, trial, run_dir, len(trials))
            print(f"{row.get('status')} in {row.pop('_elapsed_s', 0.0):.1f}s  "
                  f"residual={row.get('residual_collisions_total','?')}")
            _persist(row)


    print(f"\n[done] {len(rows_this_run)} new trial(s) in {time.perf_counter() - t_sweep_start:.1f}s")
    print(f"       CSV: {csv_path}")

    # Checked AFTER every row is written, so a failure here is visible without
    # costing the data already collected -- the point is to fail the gate, not
    # the campaign.
    col_problems = _check_campaign_columns(rows_this_run)
    if col_problems:
        print(f"\n[FAIL] {len(col_problems)} row(s) carry no usable campaign data:")
        for p in col_problems[:20]:
            print(f"  {p}")
        if len(col_problems) > 20:
            print(f"  ... and {len(col_problems) - 20} more")
        sys.exit(3)
    if rows_this_run:
        print(f"       campaign columns populated on all "
              f"{sum(1 for r in rows_this_run if r.get('status') == 'ok')} ok row(s)")

    # Smoke matrix triggers assertions; non-smoke matrices skip them
    if args.matrix == "smoke":
        # Re-read all rows (including any previously-done ones if resuming) for asserts
        all_rows = list(rows_this_run)
        if not all_rows:
            with csv_path.open("r", encoding="utf-8") as f:
                all_rows = list(csv.DictReader(f))
            # Coerce types for the few fields we assert on
            for r in all_rows:
                rc = r.get("residual_collisions_total", "")
                if rc not in ("", None):
                    try:
                        r["residual_collisions_total"] = int(float(rc))
                    except (ValueError, TypeError):
                        pass
        ok = _smoke_assertions(all_rows, run_dir)
        if not ok:
            sys.exit(2)


if __name__ == "__main__":
    main()
