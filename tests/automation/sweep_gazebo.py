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
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tests.automation import config, gazebo_runner, collision_audit  # noqa: E402
from tests.automation.campaign import guards  # noqa: E402
from tests.automation.config import TrialSpec  # noqa: E402

GAZEBO_CSV_HEADER = [
    "trial_id", "n_points", "n_obstacles", "seed", "num_uavs", "mode", "status",
    "planned_makespan_s", "executed_path_phase_s", "mission_makespan_s",
    "executed_total_arm_to_land_s", "executed_total_dist_m",
    "executed_uav0_dist_m", "executed_uav1_dist_m", "executed_uav2_dist_m",
    "audit_verdict", "audit_intersections", "audit_scrapes", "audit_clean",
    # Recorded, not assumed: a machine that cannot deliver the requested factor
    # would otherwise log a 5x run that actually ran at 2x.
    "rtf_requested", "rtf_achieved", "rtf_note",
    # Capture provenance. Without these in the header, DictWriter
    # (extrasaction="ignore") drops them silently -- the same failure that
    # left every campaign column blank in the offline sweep.
    "capture_dir", "capture_frames",
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
    # campaign_gazebo FIRST and included at all: its trial ids are gzc_*,
    # while the offline method matrix uses cmp_*, so NONE of the 80 Gazebo
    # trials share an id with an offline trial. Without this entry every
    # campaign Gazebo trial fails as missing_pickle -- an entire overnight
    # window for nothing. The driver also pre-generates them in a dedicated
    # stage so this fallback stays a safety net rather than the mechanism.
    for name in ("campaign_gazebo", "gazebo_smoke", "gazebo_subset"):
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
    # The [0-9] is load-bearing. Trial ids now carry arm suffixes, and `_`
    # (0x5f) sorts AFTER digits -- so a bare `*` glob on run_gzc_n30_o0_s0_*
    # would also match run_gzc_n30_o0_s0_A4_qn_minmax_*, and matches[-1] would
    # attach ANOTHER ARM's flight data to this trial. Anchoring on the
    # timestamp digit keeps each arm's summary to itself.
    matches = sorted(log_dir.glob(f"run_{trial_id}_[0-9]*_summary.csv"))
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
                              executor_timeout_s: float = gazebo_runner.EXECUTOR_BACKSTOP_S,
                              rtf: float = gazebo_runner.DEFAULT_RTF,
                              capture_dir: Optional[Path] = None,
                              capture_period_s: float = 0.0) -> dict:
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

    # 3b. Real-time factor.
    #
    # Applied AFTER /clock (the set_physics service does not exist before the
    # world is up) and BEFORE the settle, so the behaviour servers initialise at
    # the rate the mission will actually run at.
    #
    # The achieved factor is measured and recorded, never assumed: asking for 5x
    # on a machine that can only deliver 2x would otherwise be silently logged
    # as a 5x run. Stage 0 rejects any factor the machine does not actually hit.
    if rtf and float(rtf) != 1.0:
        ok = gazebo_runner.set_real_time_factor(float(rtf))
        print(f"  [rtf] requested {rtf}x -> service {'accepted' if ok else 'REFUSED'}",
              flush=True)
        if not ok:
            row["rtf_note"] = "set_physics refused"
        time.sleep(gazebo_runner.RTF_SETTLE_S)

    # Settle for behaviour servers to initialise
    print(f"  [settle] {gazebo_runner.POST_READY_SETTLE_S}s for behaviour servers...", flush=True)
    time.sleep(gazebo_runner.POST_READY_SETTLE_S)

    achieved_rtf = gazebo_runner.measure_real_time_factor()
    row["rtf_requested"] = float(rtf) if rtf else 1.0
    row["rtf_achieved"] = achieved_rtf
    print(f"  [rtf] achieved {achieved_rtf if achieved_rtf is not None else 'unknown'}",
          flush=True)

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

    # 6b. Visual capture, which MUST happen inside this window.
    #
    # `teardown` SIGKILLs ign-gazebo, rviz2 and the marker nodes between trials
    # (gazebo_runner.py:464-479), so anything fired afterwards finds nothing to
    # photograph. The GUI is up even in "headless" mode -- `launch_headless`
    # only skips the ground station and the two RViz marker nodes and still runs
    # the same `launch_as2.bash -m` -- so `/gui/screenshot` is available on
    # every trial and capture needs no separate GUI stage.
    #
    # Frames are collected on a daemon thread because `run_executor` blocks for
    # the whole mission. A periodic PNG series is used instead of a live screen
    # recorder so the video can be assembled afterwards with no live session,
    # which is what "never re-run a simulation for a picture" actually needs.
    cap_stop = threading.Event()
    cap_thread = None
    cap_series = None
    if capture_dir is not None:
        from tests.automation.campaign import capture as _capture  # noqa: PLC0415
        capture_dir.mkdir(parents=True, exist_ok=True)
        if not _capture.service_available():
            print("  [capture] WARNING: /gui/screenshot not advertised; "
                  "skipping capture for this trial", flush=True)
            capture_dir = None
        else:
            ok = _capture.screenshot(capture_dir / f"{trial.trial_id}_00_preflight.png")
            print(f"  [capture] preflight shot: {'ok' if ok else 'FAILED'}", flush=True)
            if capture_period_s > 0:
                cap_series = _capture.FrameSeries(
                    out_dir=capture_dir, period_s=capture_period_s,
                    prefix=f"{trial.trial_id}_frame")

                def _cap_loop():
                    while not cap_stop.wait(0.05):
                        try:
                            cap_series.tick()
                        except Exception:         # noqa: BLE001
                            pass   # a failed frame must never disturb the trial

                cap_thread = threading.Thread(target=_cap_loop, daemon=True)
                cap_thread.start()
                print(f"  [capture] frame series every {capture_period_s:.1f}s",
                      flush=True)

    # 7. Run executor
    print(f"  [exec] running deckga_execute (backstop {executor_timeout_s:.0f}s)...", flush=True)
    rc, status = gazebo_runner.run_executor(
        deckga_pkl=pkl, log_dir=trial_dir, trial_id=trial.trial_id,
        num_uavs=trial.num_uavs, backstop_s=executor_timeout_s,
    )
    print(f"  [exec] -> rc={rc} status={status}", flush=True)
    row["status"] = status

    # 7b. Final frame BEFORE teardown, while the aircraft are still landed in
    # view. After teardown there is no renderer left to ask.
    if capture_dir is not None:
        from tests.automation.campaign import capture as _capture  # noqa: PLC0415
        cap_stop.set()
        if cap_thread is not None:
            cap_thread.join(timeout=15)
        ok = _capture.screenshot(capture_dir / f"{trial.trial_id}_99_landed.png")
        n_frames = len(cap_series.frames) if cap_series else 0
        print(f"  [capture] landed shot: {'ok' if ok else 'FAILED'}; "
              f"{n_frames} frame(s)", flush=True)
        row["capture_dir"] = str(capture_dir)
        row["capture_frames"] = n_frames

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
    # Derived from config.MATRICES, not a hardcoded pair. The literal list
    # predated the campaign matrices, so `--matrix campaign_gazebo` was rejected
    # by argparse before anything ran -- the Gazebo stage would have died
    # instantly on an invalid-choice error. Deriving it means a new matrix is
    # usable from both sweeps the moment it is defined, as sweep_offline
    # already does.
    parser.add_argument("--matrix", required=True,
                        choices=sorted(config.MATRICES))
    parser.add_argument("--mode", required=True, choices=["full_gui", "headless"])
    parser.add_argument("--offline-run-dir", required=True,
                        help="Existing offline sweep run dir (provides trial pickles)")
    parser.add_argument("--no-prompt", action="store_true",
                        help="full_gui mode: skip manual Enter checkpoint (auto-launch executor)")
    parser.add_argument("--scrape-threshold", type=float, default=0.10)
    parser.add_argument("--capture-trials", default="",
                        help="Comma-separated trial_ids to capture stills and "
                             "frames for. Capture must run INSIDE a trial "
                             "window because teardown SIGKILLs the renderer "
                             "between trials; the GUI is up even in headless "
                             "mode, so no separate GUI stage is needed.")
    parser.add_argument("--capture-period", type=float, default=2.0,
                        metavar="S",
                        help="Seconds between frames on captured trials. 0 "
                             "means pre/post stills only. Frames are plain "
                             "PNGs, assembled into video afterwards with no "
                             "live session. Default %(default)s.")
    parser.add_argument("--executor-timeout", type=float,
                        default=gazebo_runner.EXECUTOR_BACKSTOP_S,
                        help="Per-trial executor backstop in seconds. Default %(default)s. "
                             "Bump higher for n=100 trials if they hit the limit.")
    parser.add_argument("--run-dir", default=None,
                        help="Where Gazebo results go. Defaults to --offline-run-dir, "
                             "which keeps them joinable with the offline trials. Set it "
                             "separately when several runs share one set of plan pickles "
                             "but must not overwrite each other's per-trial artefacts -- "
                             "e.g. the Stage 0 RTF calibration, where every run uses the "
                             "same trial_id by design.")
    parser.add_argument("--only-trial", default=None,
                        help="Run just this trial_id from the matrix. For calibration "
                             "and for re-running a single failure.")
    parser.add_argument("--rtf", type=float, default=gazebo_runner.DEFAULT_RTF,
                        help="Gazebo real-time factor. Default %(default)s. The world "
                             "template pins the sim to 1.0, which is a configured "
                             "throttle, not a CPU limit; this lifts it at runtime via "
                             "/world/empty/set_physics. Do NOT raise it without running "
                             "the Stage 0 calibration first -- at RTF N the controllers "
                             "must sustain 50*N Hz, and if they cannot the aircraft "
                             "flies a different path and every result is contaminated. "
                             "The achieved factor is measured and recorded per trial.")
    args = parser.parse_args()

    if args.rtf <= 0:
        parser.error("--rtf must be positive; 0.0 is silently ignored by Gazebo, "
                     "not treated as 'unlimited'")

    offline_run_dir = Path(args.offline_run_dir).resolve()
    if not offline_run_dir.exists():
        print(f"[err] offline run dir not found: {offline_run_dir}", file=sys.stderr)
        sys.exit(1)

    # Gazebo results live alongside the offline run dir for joinability, unless
    # the caller separates them (see --run-dir).
    run_dir = Path(args.run_dir).resolve() if args.run_dir else offline_run_dir
    run_dir.mkdir(parents=True, exist_ok=True)
    run_ts = run_dir.name

    trials = config.resolve_matrix(args.matrix)
    if args.only_trial:
        trials = [t for t in trials if t.trial_id == args.only_trial]
        if not trials:
            print(f"[err] --only-trial {args.only_trial!r} is not in matrix "
                  f"{args.matrix!r}", file=sys.stderr)
            sys.exit(1)
    # Validated against the resolved matrix rather than trusted: a typo in a
    # capture id would otherwise mean the campaign finishes with no pictures and
    # nothing said about it, and these trials cannot be re-flown cheaply.
    capture_ids = {t.strip() for t in args.capture_trials.split(",") if t.strip()}
    if capture_ids:
        known = {t.trial_id for t in trials}
        unknown = sorted(capture_ids - known)
        if unknown:
            print(f"[err] --capture-trials not in matrix {args.matrix!r}: "
                  f"{unknown}", file=sys.stderr)
            sys.exit(1)
        print(f"[gazebo] capturing {len(capture_ids)} trial(s): "
              f"{sorted(capture_ids)}")

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

        # Mid-run guards, BETWEEN TRIALS. The driver's preflight runs once per
        # stage, so on an 80-trial 16-hour stage a disk that fills at trial 50
        # would go unnoticed until the stage ended -- with the remaining trials
        # recorded as failures for the wrong reason. A background job in this
        # session was already reaped for low memory, so RAM is checked too.
        #
        # Stopping cleanly beats pressing on: every completed trial is already
        # persisted and `_existing_ok_trial_ids` resumes from it, so aborting
        # here costs only the trials not yet run.
        disk = guards.check_disk(run_dir)
        if not disk.ok:
            print(f"  [guard] ABORT: {disk.detail}", flush=True)
            print(f"[gazebo] stopping before {trial.trial_id}; "
                  f"{len(trials) - i + 1} trial(s) remain for a resumed run",
                  flush=True)
            break
        ram = guards.check_ram()
        if not ram.ok:
            # Almost always a simulator that outlived its trial. Reclaim and
            # re-check once before giving up on the stage.
            killed = guards.kill_sim_orphans()
            print(f"  [guard] low RAM ({ram.detail}); killed {killed} "
                  f"orphan(s), waiting 60 s", flush=True)
            time.sleep(60)
            ram = guards.check_ram()
            if not ram.ok:
                print(f"  [guard] ABORT: still low -- {ram.detail}", flush=True)
                break
        if guards.find_sim_orphans():
            killed = guards.kill_sim_orphans()
            print(f"  [guard] killed {killed} orphan sim process(es) left by "
                  f"the previous trial", flush=True)

        try:
            row = _run_single_gazebo_trial(
                trial, args.mode, run_dir, offline_run_dir, run_ts,
                no_prompt=args.no_prompt,
                scrape_threshold=args.scrape_threshold,
                executor_timeout_s=args.executor_timeout,
                rtf=args.rtf,
                capture_dir=(run_dir / "media" / trial.trial_id
                             if trial.trial_id in capture_ids else None),
                capture_period_s=args.capture_period,
            )
        except KeyboardInterrupt:
            print("\n[gazebo] interrupted; aborting sweep")
            sys.exit(130)
        _append_row(csv_path, row)
        print(f"  -> {row['status']}  audit={row.get('audit_verdict', '-')}", flush=True)

    print(f"\n[gazebo] done. CSV: {csv_path}")


if __name__ == "__main__":
    main()
