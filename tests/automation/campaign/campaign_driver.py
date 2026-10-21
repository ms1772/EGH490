"""Campaign orchestrator: runs every stage to completion, unattended.

DESIGN RULES, each one earned
-----------------------------
* **Never block on input.** `sweep_gazebo`'s manual checkpoint returns False on
  EOF and that aborts the trial, so a detached full_gui run without
  `--no-prompt` fails every single trial instantly. Stages that use the GUI pass
  it explicitly.

* **Every stage is resumable and the driver is restartable.** Re-running picks
  up at the first stage without a `.done` marker; the sweeps themselves skip
  completed trials by `trial_id`. Nothing is recomputed that already succeeded.

* **One stage at a time.** Measured the hard way: running the test suite
  alongside a Gazebo trial pushed its achieved RTF to 0.969 and its makespan 8%
  above the historical value for the identical plan. On a calibration whose
  acceptance threshold is 2%, concurrent work is the difference between
  accepting and rejecting. Stages are strictly serial.

* **Guards between trials, not just at startup.** See guards.py.

* **The log is appended, never truncated**, and a heartbeat file carries enough
  state to answer "is it alive and how far in" without reading the log.

* **A failed stage does not necessarily stop the campaign.** Offline stages are
  cheap to redo; the Gazebo stage is not, so it continues past individual trial
  failures and only aborts if the failure rate says something systemic is wrong.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable, List, Optional

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tests.automation.campaign import guards  # noqa: E402

HEARTBEAT_PERIOD_S = 60.0


@dataclass
class Stage:
    name: str
    argv: List[str]
    timeout_s: float
    needs_sim: bool = False
    # A stage may fail without ending the campaign when later stages do not
    # depend on it (e.g. a capture pass).
    required: bool = True
    env: dict = field(default_factory=dict)


class Campaign:
    def __init__(self, run_dir: Path, *, dry_run: bool = False):
        self.run_dir = run_dir
        self.dir = run_dir / "campaign"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.log_path = self.dir / "campaign.log"
        self.hb_path = self.dir / "heartbeat.json"
        self.dry_run = dry_run
        self.t0 = time.time()
        self.state = {"stage": None, "stage_index": 0, "n_stages": 0,
                      "started_at": datetime.now().isoformat(timespec="seconds")}
        self._stop_hb = threading.Event()
        self._hb_thread: Optional[threading.Thread] = None

    # -- logging ----------------------------------------------------------
    def log(self, msg: str) -> None:
        stamp = datetime.now().isoformat(timespec="seconds")
        line = f"[{stamp}] {msg}"
        print(line, flush=True)
        try:
            with self.log_path.open("a", encoding="utf-8") as f:
                f.write(line + "\n")
        except Exception:
            pass          # logging must never take the campaign down

    # -- heartbeat --------------------------------------------------------
    def _heartbeat_once(self) -> None:
        try:
            free = guards.check_disk(self.run_dir)
            ram = guards.check_ram()
            payload = dict(self.state)
            payload.update({
                "updated_at": datetime.now().isoformat(timespec="seconds"),
                "elapsed_s": round(time.time() - self.t0, 1),
                "elapsed_h": round((time.time() - self.t0) / 3600.0, 2),
                "disk": free.detail,
                "ram": ram.detail,
                "pid": os.getpid(),
            })
            tmp = self.hb_path.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            tmp.replace(self.hb_path)       # atomic; a reader never sees a torn file
        except Exception:
            pass

    def _heartbeat_loop(self) -> None:
        while not self._stop_hb.wait(HEARTBEAT_PERIOD_S):
            self._heartbeat_once()

    def start_heartbeat(self) -> None:
        self._heartbeat_once()
        self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._hb_thread.start()

    def stop_heartbeat(self) -> None:
        self._stop_hb.set()
        self._heartbeat_once()

    # -- stages -----------------------------------------------------------
    def marker(self, stage: Stage) -> Path:
        return self.dir / f"{stage.name}.done"

    def run_stage(self, stage: Stage, index: int, total: int) -> bool:
        mark = self.marker(stage)
        if mark.exists():
            self.log(f"stage {stage.name}: already done, skipping "
                     f"({mark.read_text().strip()[:60]})")
            return True

        self.state.update({"stage": stage.name, "stage_index": index,
                           "n_stages": total})
        self._heartbeat_once()

        checks = guards.preflight(self.run_dir, need_sim=stage.needs_sim)
        all_ok, fatal, text = guards.summarise(checks)
        self.log(f"stage {stage.name}: preflight\n{text}")
        if fatal:
            self.log(f"stage {stage.name}: ABORT -- fatal precondition")
            return False

        if stage.needs_sim:
            killed = guards.kill_sim_orphans()
            if killed:
                self.log(f"stage {stage.name}: killed {killed} orphan sim process(es) "
                         f"before starting")

        env = os.environ.copy()
        env.update(stage.env)
        cmd = [sys.executable] + stage.argv

        self.log(f"stage {stage.name}: START  timeout={stage.timeout_s/3600:.1f}h")
        self.log(f"  $ {' '.join(cmd)}")
        if self.dry_run:
            self.log(f"stage {stage.name}: DRY RUN, not executed")
            return True

        t0 = time.time()
        try:
            proc = subprocess.run(cmd, cwd=REPO_ROOT, env=env,
                                  timeout=stage.timeout_s)
            rc = proc.returncode
        except subprocess.TimeoutExpired:
            self.log(f"stage {stage.name}: TIMEOUT after {stage.timeout_s/3600:.1f}h")
            rc = -1
        except KeyboardInterrupt:
            self.log(f"stage {stage.name}: interrupted")
            raise
        dt = time.time() - t0

        if stage.needs_sim:
            killed = guards.kill_sim_orphans()
            if killed:
                self.log(f"stage {stage.name}: cleaned {killed} orphan(s) afterwards")

        if rc == 0:
            mark.write_text(
                f"ok rc=0 elapsed={dt:.0f}s at "
                f"{datetime.now().isoformat(timespec='seconds')}\n",
                encoding="utf-8")
            self.log(f"stage {stage.name}: DONE in {dt/60:.1f} min")
            return True

        self.log(f"stage {stage.name}: FAILED rc={rc} after {dt/60:.1f} min")
        return not stage.required

    def run(self, stages: List[Stage]) -> int:
        self.log("=" * 70)
        self.log(f"campaign start -- run_dir={self.run_dir}")
        self.log(f"stages: {', '.join(s.name for s in stages)}")
        self.start_heartbeat()
        try:
            for i, stage in enumerate(stages, 1):
                if not self.run_stage(stage, i, len(stages)):
                    self.log(f"campaign STOPPED at stage {stage.name}")
                    return 1
            self.log(f"campaign COMPLETE in "
                     f"{(time.time() - self.t0)/3600:.2f} h")
            return 0
        finally:
            self.stop_heartbeat()


def default_capture_trials(matrix_name: str) -> List[str]:
    """The trials that get screenshots: the published arm and the full
    reformulation, at both fleet sizes, in an obstacle-bearing scene.

    Derived from the resolved matrix rather than hardcoded, so renaming an arm
    or changing the densities cannot leave this pointing at ids that no longer
    exist -- which would mean a finished campaign with no figures and nothing
    said about it.

    A0 vs A4 is the comparison the thesis makes, obs=20 is the densest Gazebo
    condition (so avoidance is actually visible), and the lowest seed keeps it
    to one scene per fleet size.
    """
    from tests.automation import config                        # noqa: PLC0415
    trials = config.resolve_matrix(matrix_name)
    if not trials:
        return []
    want_arms = ("A0_published", "A4_qn_minmax")
    obs = max(t.n_obstacles for t in trials)
    seed = min(t.seed for t in trials)
    out = []
    for n in sorted({t.n_points for t in trials}):
        for arm in want_arms:
            hit = [t for t in trials
                   if t.n_points == n and t.n_obstacles == obs
                   and t.seed == seed and (t.arm or "") == arm]
            if hit:
                out.append(hit[0].trial_id)
    return out


def build_stages(run_dir: Path, offline_run_dir: Path, rtf: float,
                 gazebo_matrix: str, executor_timeout: float,
                 jobs: int = 8, capture_trials: Optional[List[str]] = None,
                 capture_period: float = 0.0) -> List[Stage]:
    A = "tests/automation"
    return [
        # The pilot's 39 trial ids are a STRICT SUBSET of campaign_method's 780,
        # and both write the same run dir, so the gate costs nothing: resume
        # skips its rows when s3 runs. Only 3 scenes exist here, and parallelism
        # is per scene group, so more than 3 workers would idle.
        Stage("s1_pilot",
              [f"{A}/sweep_offline.py", "--matrix", "campaign_pilot",
               "--jobs", str(min(jobs, 3)), "--run-dir", str(run_dir)],
              timeout_s=3600),
        # 780 trials, ~37 s each (the GA is ~34 s of that and dominates), so
        # ~8 h serial against ~1 h at 8 workers. 60 scene groups, so 8 workers
        # stay busy throughout.
        Stage("s3_offline_method",
              [f"{A}/sweep_offline.py", "--matrix", "campaign_method",
               "--jobs", str(jobs), "--run-dir", str(run_dir)],
              timeout_s=8 * 3600),
        # The Gazebo matrix uses its OWN trial ids (gzc_*), which share none
        # with the offline method matrix (cmp_*). Its plans therefore have to be
        # planned explicitly; without this stage every Gazebo trial would abort
        # as missing_pickle. 80 trials, cheap, and it keeps the planning cost out
        # of the expensive serial Gazebo stage.
        Stage("s4_gazebo_plans",
              [f"{A}/sweep_offline.py", "--matrix", "campaign_gazebo",
               "--jobs", str(jobs), "--run-dir", str(offline_run_dir)],
              timeout_s=2 * 3600),
        Stage("s5_gazebo",
              [f"{A}/sweep_gazebo.py", "--matrix", gazebo_matrix,
               "--mode", "headless", "--no-prompt",
               "--offline-run-dir", str(offline_run_dir),
               "--run-dir", str(run_dir),
               "--rtf", str(rtf),
               "--capture-trials", ",".join(capture_trials or []),
               "--capture-period", str(capture_period),
               "--executor-timeout", str(executor_timeout)],
              timeout_s=40 * 3600, needs_sim=True),
        Stage("s7_aggregate",
              [f"{A}/aggregate.py", "--run-dir", str(run_dir)],
              timeout_s=1800),
        Stage("s7_report",
              [f"{A}/report.py", "--run-dir", str(run_dir)],
              timeout_s=1800, required=False),
    ]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--offline-run-dir", default=None,
                    help="defaults to --run-dir")
    ap.add_argument("--rtf", type=float, default=1.0,
                    help="from Stage 0 calibration; do not guess")
    ap.add_argument("--gazebo-matrix", default="campaign_gazebo",
                    help="80 trials: 5 arms x 8 original conditions x 2 seeds.")
    ap.add_argument("--capture-trials", default=None,
                    help="Comma-separated trial_ids to screenshot. Default: "
                         "A0 and A4 at both fleet sizes in the densest Gazebo "
                         "condition, derived from the matrix.")
    ap.add_argument("--capture-period", type=float, default=0.0, metavar="S",
                    help="Seconds between frames on captured trials. DEFAULT 0 "
                         "= stills only (pre-flight and landed), which is "
                         "deliberate: a frame series fires `ign service` "
                         "subprocesses DURING the mission, and the captured "
                         "trials are also data rows feeding the paired "
                         "comparison. Stills are taken outside the flight "
                         "window and perturb nothing. Shoot the video as a "
                         "separate dedicated trial whose row is discarded.")
    ap.add_argument("--jobs", type=int, default=8, metavar="N",
                    help="Offline sweep workers. The sweep is otherwise "
                         "single-threaded and pins one core of twenty. Gazebo "
                         "stages stay serial regardless of this: a concurrent "
                         "load measurably changed a trial's makespan.")
    ap.add_argument("--executor-timeout", type=float, default=3600.0)
    ap.add_argument("--only-stage", default=None,
                    help="run just this stage (by name)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    run_dir = Path(args.run_dir).resolve()
    offline = Path(args.offline_run_dir).resolve() if args.offline_run_dir else run_dir
    run_dir.mkdir(parents=True, exist_ok=True)

    if args.capture_trials is None:
        capture = default_capture_trials(args.gazebo_matrix)
    else:
        capture = [x.strip() for x in args.capture_trials.split(",") if x.strip()]
    stages = build_stages(run_dir, offline, args.rtf, args.gazebo_matrix,
                          args.executor_timeout, jobs=args.jobs,
                          capture_trials=capture,
                          capture_period=args.capture_period)
    if args.only_stage:
        stages = [s for s in stages if s.name == args.only_stage]
        if not stages:
            print(f"no stage named {args.only_stage!r}", file=sys.stderr)
            return 2

    return Campaign(run_dir, dry_run=args.dry_run).run(stages)


if __name__ == "__main__":
    raise SystemExit(main())
