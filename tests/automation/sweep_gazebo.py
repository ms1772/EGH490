#!/usr/bin/env python3
"""Stage 4: Gazebo execution driver.

Two modes:
  - full_gui:  one smoke trial; spawns RViz/ground-station; pauses for manual
               Enter; runs executor; records bag; runs collision audit.
  - headless:  N trials; spawns Aerostack2 only; uses discovered topics from
               full_gui run; runs executor; records bag; runs audit.

Re-uses the offline run dir for trial pickles. If a pickle is missing for a
trial, the driver auto-runs sweep_offline for just that trial first.

Usage (after sweep_offline has produced the trial pickles):
    python tests/automation/sweep_gazebo.py --matrix gazebo_smoke --mode full_gui \\
        --offline-run-dir tests/automation/results/<ts>
    python tests/automation/sweep_gazebo.py --matrix gazebo_subset --mode headless \\
        --offline-run-dir tests/automation/results/<ts>
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tests.automation import config, gazebo_runner, collision_audit  # noqa: E402
from tests.automation.config import TrialSpec  # noqa: E402

GAZEBO_CSV_HEADER = [
    "trial_id", "n_points", "n_obstacles", "seed", "num_uavs", "mode", "status",
    "planned_makespan_s", "executed_path_phase_s", "mission_makespan_s",
    "executed_total_arm_to_land_s", "executed_total_dist_m",
    "executed_uav0_dist_m", "executed_uav1_dist_m", "executed_uav2_dist_m",
    "audit_verdict", "audit_intersections", "audit_scrapes", "audit_clean",
    "bag_path", "trial_dir", "started_at", "finished_at", "error",
]


def _append_row(csv_path: Path, row: dict) -> None:
    new_file = not csv_path.exists()
    with csv_path.open("a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=GAZEBO_CSV_HEADER, extrasaction="ignore")
        if new_file:
            w.writeheader()
        w.writerow(row)
        f.flush()
        os.fsync(f.fileno())


def _existing_ok_trial_ids(csv_path: Path) -> set:
    """Trials with status=ok in the CSV. Used to skip on resume.

    Trials with a non-ok status (executor_timeout, launch_timeout, etc.) are
    NOT skipped — those should be retried with whatever new settings the user
    has chosen (e.g. a higher --executor-timeout)."""
    if not csv_path.exists():
        return set()
    with csv_path.open("r", encoding="utf-8") as f:
        return {r["trial_id"] for r in csv.DictReader(f) if r.get("status") == "ok"}


def _purge_non_ok_rows(csv_path: Path) -> int:
    """Rewrite CSV keeping only status=ok rows. Returns count of dropped rows.
    Called once at the start of resumed runs so the retry actually re-runs them."""
    if not csv_path.exists():
        return 0
    with csv_path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
        fieldnames = f.readline  # noqa: just keeping reader exhausted
    if not rows:
        return 0
    keep = [r for r in rows if r.get("status") == "ok"]
    dropped = len(rows) - len(keep)
    if dropped == 0:
        return 0
    # Rewrite using the same header (CSV_HEADER constant is the canonical schema)
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=GAZEBO_CSV_HEADER, extrasaction="ignore")
        w.writeheader()
        w.writerows(keep)
    return dropped


def _ensure_pickle(trial: TrialSpec, offline_run_dir: Path) -> Path:
    """Find the trial's deckga_quicknav_output.pkl; if missing, run sweep_offline
    for just this matrix to populate it."""
    pkl = offline_run_dir / "trials" / trial.trial_id / "deckga_quicknav_output.pkl"
    if pkl.exists():
        return pkl
    # Auto-run: just this matrix into the same offline_run_dir (it skips existing trials)
    matrix_name = None
    for name in ("gazebo_smoke", "gazebo_subset"):
        if trial.trial_id in [t.trial_id for t in config.resolve_matrix(name)]:
            matrix_name = name
            break
    if matrix_name is None:
        raise FileNotFoundError(f"no pickle for {trial.trial_id} and not in any gazebo matrix")
    print(f"[gazebo] pickle missing for {trial.trial_id}; running offline planner first...")
    proc = subprocess.run(
        [sys.executable, str(REPO_ROOT / "tests" / "automation" / "sweep_offline.py"),
         "--matrix", matrix_name, "--run-dir", str(offline_run_dir)],
        cwd=REPO_ROOT,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"sweep_offline failed for {trial.trial_id}")
    if not pkl.exists():
        raise FileNotFoundError(f"pickle still missing after offline planner: {pkl}")
    return pkl


def _parse_executor_summary(log_dir: Path, trial_id: str) -> Optional[dict]:
    """Find the run_<trial_id>_<timestamp>_summary.csv produced by deckga_execute.py.

    Picks the most-recent if multiple exist (e.g. retries)."""
    matches = sorted(log_dir.glob(f"run_{trial_id}_*_summary.csv"))
    if not matches:
        return None
    summary_csv = matches[-1]
    with summary_csv.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    return rows[0]


def _wait_for_manual_confirmation(prompt: str) -> bool:
    """Prompt the user. Returns False on EOF/Ctrl-D (assume non-interactive)."""
    print(prompt)
    try:
        input()
        return True
    except (EOFError, KeyboardInterrupt):
        return False


def _run_single_gazebo_trial(trial: TrialSpec, mode: str,
                              run_dir: Path, offline_run_dir: Path,
                              run_ts: str, no_prompt: bool = False,
                              scrape_threshold: float = 0.10,
                              executor_timeout_s: float = gazebo_runner.EXECUTOR_BACKSTOP_S) -> dict:
    started_at = datetime.now().isoformat(timespec="seconds")
    trial_dir = run_dir / "gazebo" / trial.trial_id
    trial_dir.mkdir(parents=True, exist_ok=True)
    row: dict = {
        "trial_id": trial.trial_id, "n_points": trial.n_points,
        "n_obstacles": trial.n_obstacles, "seed": trial.seed,
        "num_uavs": trial.num_uavs, "mode": mode,
        "started_at": started_at, "trial_dir": str(trial_dir),
    }

    # 1. Ensure pickle
    try:
        pkl = _ensure_pickle(trial, offline_run_dir)
    except Exception as e:
        row.update({"status": "missing_pickle", "error": str(e),
                    "finished_at": datetime.now().isoformat(timespec="seconds")})
        return row

    # 2. Launch lifecycle
    print(f"  [launch] mode={mode} ...", flush=True)
    if mode == "full_gui":
        lc = gazebo_runner.launch_full_gui(pkl, log_dir=trial_dir)
    elif mode == "headless":
        lc = gazebo_runner.launch_headless(log_dir=trial_dir)
    else:
        raise ValueError(f"unknown mode {mode!r}")

    # 3. Wait for /clock
    t_launch = time.perf_counter()
    print(f"  [wait] /clock (up to {gazebo_runner.READINESS_CLOCK_TIMEOUT_S}s)...", flush=True)
    if not gazebo_runner.wait_for_clock():
        gazebo_runner.teardown(lc, log_dir=trial_dir)
        row.update({"status": "launch_timeout", "error": "/clock never appeared",
                    "finished_at": datetime.now().isoformat(timespec="seconds")})
        return row

    # Settle for behaviour servers to initialise
    print(f"  [settle] {gazebo_runner.POST_READY_SETTLE_S}s for behaviour servers...", flush=True)
    time.sleep(gazebo_runner.POST_READY_SETTLE_S)

    # 4. Discover pose topics (full_gui) or read from cache (headless)
    if mode == "full_gui":
        pose_topics = gazebo_runner.discover_pose_topics(num_uavs=trial.num_uavs)
        time_to_ready_s = time.perf_counter() - t_launch
        gazebo_runner.write_discovered_topics(run_dir, pose_topics, time_to_ready_s)
        print(f"  [discover] pose topics: {pose_topics}", flush=True)
    else:
        cached = gazebo_runner.read_discovered_topics(offline_run_dir) or \
                 gazebo_runner.read_discovered_topics(run_dir)
        if cached is None:
            print("  [warn] no discovered_topics.json found; falling back to defaults", flush=True)
            pose_topics = gazebo_runner.DEFAULT_POSE_TOPICS[:trial.num_uavs]
        else:
            pose_topics = cached["pose_topics"]
        # Confirm at least one is up before bag recorder
        ready = False
        for t in pose_topics:
            if gazebo_runner.wait_for_pose(t, timeout_s=10):
                ready = True
                break
        if not ready:
            gazebo_runner.teardown(lc, log_dir=trial_dir)
            row.update({"status": "launch_timeout",
                        "error": f"no pose on {pose_topics}",
                        "finished_at": datetime.now().isoformat(timespec="seconds")})
            return row

    # 5. Start bag recorder
    bag_proc, bag_home = gazebo_runner.start_bag_recorder(
        pose_topics, run_ts, trial.trial_id, log_path=trial_dir / "bag_recorder.log"
    )
    lc.bag_proc = bag_proc
    lc.bag_path = bag_home
    time.sleep(2.0)  # let recorder open the topic subscriptions

    # 6. Manual checkpoint (full_gui smoke only)
    if mode == "full_gui" and not no_prompt:
        proceed = _wait_for_manual_confirmation(
            "\n[smoke] Aerostack2 + ground station + bag recorder up.\n"
            "Verify in RViz that 3 drones spawned, paths render in green, "
            "obstacles render as cyan cubes.\n"
            "Press Enter to launch executor (or Ctrl-C to abort)."
        )
        if not proceed:
            gazebo_runner.stop_bag_recorder(bag_proc)
            gazebo_runner.teardown(lc, log_dir=trial_dir)
            row.update({"status": "aborted", "error": "user aborted at manual checkpoint",
                        "finished_at": datetime.now().isoformat(timespec="seconds")})
            return row

    # 7. Run executor
    print(f"  [exec] running deckga_execute (backstop {executor_timeout_s:.0f}s)...", flush=True)
    rc, status = gazebo_runner.run_executor(
        deckga_pkl=pkl, log_dir=trial_dir, trial_id=trial.trial_id,
        num_uavs=trial.num_uavs, backstop_s=executor_timeout_s,
    )
    print(f"  [exec] -> rc={rc} status={status}", flush=True)
    row["status"] = status

    # 8. Stop bag recorder
    print("  [bag] stopping recorder (SIGINT)...", flush=True)
    gazebo_runner.stop_bag_recorder(bag_proc)

    # 9. Teardown
    print("  [teardown]...", flush=True)
    teardown_report = gazebo_runner.teardown(lc, log_dir=trial_dir)
    if teardown_report.get("orphans_killed"):
        print(f"  [teardown] killed orphan PIDs: {teardown_report['orphans_killed']}", flush=True)

    # 10. Copy bag to results
    bag_results = trial_dir / "poses.bag"
    bag_ok = gazebo_runner.copy_bag(bag_home, bag_results)
    if bag_ok:
        row["bag_path"] = str(bag_results)
        # Optional: delete the staging copy to save space
        try:
            shutil.rmtree(bag_home, ignore_errors=True)
        except Exception:
            pass

    # 11. Parse executor summary
    summary = _parse_executor_summary(trial_dir, trial.trial_id)
    if summary:
        for k in ("planned_makespan_s", "executed_path_phase_s", "mission_makespan_s",
                  "executed_total_arm_to_land_s", "executed_total_dist_m",
                  "executed_uav0_dist_m", "executed_uav1_dist_m", "executed_uav2_dist_m"):
            if k in summary:
                row[k] = summary[k]

    # 12. Run collision audit
    if bag_ok and status == "ok":
        try:
            audit = collision_audit.audit_bag(
                bag_results, pkl,
                pose_topics=pose_topics,
                scrape_threshold=scrape_threshold,
            )
            audit_path = trial_dir / "collision_audit.json"
            audit_path.write_text(json.dumps(audit, indent=2))
            row["audit_verdict"] = audit["verdict"]
            row["audit_intersections"] = audit["totals"]["intersection"]
            row["audit_scrapes"] = audit["totals"]["scrape"]
            row["audit_clean"] = audit["totals"]["clean"]
            print("")
            print(collision_audit.render_markdown_summary(audit, trial_id=trial.trial_id))
        except Exception as e:
            row["audit_verdict"] = "ERROR"
            row["error"] = (row.get("error") or "") + f"; audit error: {e}"
            print(f"  [audit] ERROR: {e}", flush=True)
    else:
        row["audit_verdict"] = "skipped" if status != "ok" else "no_bag"

    row["finished_at"] = datetime.now().isoformat(timespec="seconds")
    return row


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", required=True,
                        choices=["gazebo_smoke", "gazebo_subset"])
    parser.add_argument("--mode", required=True, choices=["full_gui", "headless"])
    parser.add_argument("--offline-run-dir", required=True,
                        help="Existing offline sweep run dir (provides trial pickles)")
    parser.add_argument("--no-prompt", action="store_true",
                        help="full_gui mode: skip manual Enter checkpoint (auto-launch executor)")
    parser.add_argument("--scrape-threshold", type=float, default=0.10)
    parser.add_argument("--executor-timeout", type=float,
                        default=gazebo_runner.EXECUTOR_BACKSTOP_S,
                        help="Per-trial executor backstop in seconds. Default %(default)s. "
                             "Bump higher for n=100 trials if they hit the limit.")
    args = parser.parse_args()

    offline_run_dir = Path(args.offline_run_dir).resolve()
    if not offline_run_dir.exists():
        print(f"[err] offline run dir not found: {offline_run_dir}", file=sys.stderr)
        sys.exit(1)

    # Gazebo results live alongside the offline run dir for joinability
    run_dir = offline_run_dir
    run_ts = run_dir.name

    trials = config.resolve_matrix(args.matrix)
    csv_path = run_dir / "gazebo_master.csv"
    dropped = _purge_non_ok_rows(csv_path)
    if dropped:
        print(f"[gazebo] dropped {dropped} prior non-ok row(s) so they retry with new settings")
    done = _existing_ok_trial_ids(csv_path)
    print(f"[gazebo] matrix={args.matrix} mode={args.mode} "
          f"trials={len(trials)} already_done={len(done)}")
    if args.mode == "headless":
        cached = gazebo_runner.read_discovered_topics(run_dir)
        if cached is None:
            print("[gazebo] WARN: no discovered_topics.json from a prior full_gui smoke. "
                  "Will fall back to default topic names per trial.")

    for i, trial in enumerate(trials, 1):
        if trial.trial_id in done:
            print(f"[{i}/{len(trials)}] {trial.trial_id} -- already done, skipping")
            continue
        print(f"\n[{i}/{len(trials)}] {trial.trial_id} n={trial.n_points} obs={trial.n_obstacles}")
        try:
            row = _run_single_gazebo_trial(
                trial, args.mode, run_dir, offline_run_dir, run_ts,
                no_prompt=args.no_prompt,
                scrape_threshold=args.scrape_threshold,
                executor_timeout_s=args.executor_timeout,
            )
        except KeyboardInterrupt:
            print("\n[gazebo] interrupted; aborting sweep")
            sys.exit(130)
        _append_row(csv_path, row)
        print(f"  -> {row['status']}  audit={row.get('audit_verdict', '-')}", flush=True)

    print(f"\n[gazebo] done. CSV: {csv_path}")


if __name__ == "__main__":
    main()
