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

PLANNER_TIMEOUT_S = 180
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
    "trial_dir", "started_at", "finished_at", "error",
]


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


def _generate_obstacles(trial: TrialSpec, seed_offset: int, out_pkl: Path, log_path: Path) -> None:
    """Run generate_points_xyz.py for obstacles. Different bounds; decorrelated seed."""
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
        _generate_obstacles(trial, config.SEED_OBSTACLE_OFFSET, obstacles_pkl, log)
        seed_offset_used = config.SEED_OBSTACLE_OFFSET

    return points_pkl, obstacles_pkl, seed_offset_used


# ---------- planner subprocess ----------

class PlannerOverlapError(Exception):
    """Raised when DECK_GA_QuickNav reports a waypoint/start inside an obstacle."""


def _run_planner(trial: TrialSpec, points_pkl: Path, obstacles_pkl: Path,
                 out_pkl: Path, log_path: Path) -> float:
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


def _run_single_trial(trial: TrialSpec, trial_dir: Path) -> dict:
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
                _generate_obstacles(trial, seed_offset, obstacles_pkl, log_path)
            planner_time_total = _run_planner(trial, points_pkl, obstacles_pkl, out_pkl, log_path)
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
    return row


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
    for i, trial in enumerate(trials, 1):
        if trial.trial_id in done_ids:
            print(f"[{i}/{len(trials)}] {trial.trial_id} -- already done, skipping")
            continue
        trial_dir = run_dir / "trials" / trial.trial_id
        t_trial = time.perf_counter()
        print(f"[{i}/{len(trials)}] {trial.trial_id} n={trial.n_points} obs={trial.n_obstacles} seed={trial.seed}...", end=" ", flush=True)
        row = _run_single_trial(trial, trial_dir)
        elapsed = time.perf_counter() - t_trial
        rc = row.get("residual_collisions_total", "?")
        print(f"{row['status']} in {elapsed:.1f}s  residual={rc}")
        _append_row(csv_path, row)
        with jsonl_path.open("a", encoding="utf-8") as jf:
            jf.write(json.dumps(row, default=str) + "\n")
        rows_this_run.append(row)

    print(f"\n[done] {len(rows_this_run)} new trial(s) in {time.perf_counter() - t_sweep_start:.1f}s")
    print(f"       CSV: {csv_path}")

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
