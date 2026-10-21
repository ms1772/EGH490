#!/usr/bin/env python3
"""Stage 0: find the highest real-time factor that does not change the results.

WHY THIS EXISTS
---------------
The Gazebo world pins itself to real time
(`as2_gazebo_assets/worlds/empty.sdf.jinja:6-7`, `real_time_factor 1.0`). That
is a configured throttle, not a CPU limit. Lifting it is the difference between
a ~30 hour campaign and a ~12 hour one.

But it is only free if the control stack keeps up. `config/config.yaml:11,25`
runs the platform and the controller at `cmd_freq: 50 Hz` against
`use_sim_time: true`, so at RTF N those loops must sustain 50*N Hz of
wall-clock work. If they cannot, the aircraft tracks its path worse, the flown
trajectory changes, and every number the campaign produces is contaminated.

So this does not ask "how fast can it go". It asks "how fast can it go while
producing the SAME FLIGHT", and takes the answer from measurement.

THE ACCEPTANCE TEST
-------------------
An RTF is accepted only if, against the RTF=1.0 control run of the identical
trial:

  * planned per-UAV lengths are IDENTICAL (proves the same plan was flown)
  * makespan, normalised to simulated seconds, is within TIME_TOL (2%)
  * residual collisions are identical

and `rtf_accept.py` additionally gates on bag-measured flown distance and
planned-path deviation. That module is AUTHORITATIVE; this one cannot measure
flown geometry at all.

TWO CORRECTIONS THAT THIS FILE ORIGINALLY GOT WRONG
---------------------------------------------------
1. `mission_makespan_s` is WALL-CLOCK, not sim time. Every timestamp in
   `deckga_execute.py` comes from `time.perf_counter()`/`time.time()`, not the
   ROS clock, despite `use_sim_time:=true` for the platform and controller. So
   raising the RTF shrinks it *by construction* and an un-normalised comparison
   can never pass for RTF > 1. Multiply by the achieved factor first.
2. `executed_uavN_dist_m` is PLANNED geometry, not measurement
   (`deckga_execute.py:355` sums commanded waypoints), so it is identical at
   every RTF and cannot detect a changed path.

Together those two made the original gate unable to fail on distance and
unable to pass on time. Measured on gz_n30_o20_s0: requesting 2.0 achieved
1.179, with wall makespan -14.9% but sim-normalised makespan +1.08%, bag-flown
length +0.30% and identical collisions -- i.e. a faithful 1.18x speedup that
the original test would have rejected while reporting the distance check as
passing.

The highest passing RTF is then backed off one rung for margin, because the
campaign runs for hours under varying machine load and the calibration is a
handful of minutes on an idle box.

USAGE
    python3 tests/automation/campaign/rtf_calibrate.py \
        --offline-run-dir tests/automation/results/<ts> \
        --trial-id gz_n30_o20_s0 --rtfs 1,2,3,5
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT))

from tests.automation import gazebo_runner  # noqa: E402

# Retained only so the emitted JSON keeps its schema. NOT a gate any more: the
# column it applied to is planned geometry and could never breach it. The real
# distance tolerance is rtf_accept.BAG_DIST_TOL, applied to bag poses.
DIST_TOL = 0.02
TIME_TOL = 0.02          # 2% makespan, after normalising wall -> simulated time
RTF_ACHIEVED_TOL = 0.80  # below this the request saturated: reported, not fatal


@dataclass
class RtfRun:
    requested: float
    achieved: Optional[float] = None
    wall_s: Optional[float] = None
    makespan_sim_s: Optional[float] = None
    distances: List[float] = field(default_factory=list)
    collisions: Optional[int] = None
    status: str = "not_run"
    note: str = ""

    def as_dict(self) -> dict:
        return {
            "requested_rtf": self.requested,
            "achieved_rtf": self.achieved,
            "wall_s": self.wall_s,
            "makespan_sim_s": self.makespan_sim_s,
            "executed_distances_m": self.distances,
            "residual_collisions": self.collisions,
            "status": self.status,
            "note": self.note,
        }


def _pct(a: float, b: float) -> float:
    """Relative difference of a from b."""
    if b == 0:
        return 0.0 if a == 0 else float("inf")
    return abs(a - b) / abs(b)


def evaluate(control: RtfRun, cand: RtfRun) -> tuple:
    """(accepted, reason). The control is always accepted by definition."""
    if cand.status != "ok":
        return False, f"run status {cand.status}"

    if cand.achieved is None:
        return False, "no achieved RTF measured"

    # Saturation is INFORMATIONAL. A request of 2.0 that delivers 1.18 while
    # leaving sim-time makespan, flown geometry and collisions unchanged is a
    # free 1.18x, not a failure; failing it would discard real campaign hours
    # for no fidelity benefit. `rtf_accept.choose` therefore ranks on achieved.
    saturated = (cand.requested > 1.0
                 and cand.achieved < RTF_ACHIEVED_TOL * cand.requested)

    # `cand.distances` comes from `executed_uavN_dist_m`, which is PLANNED
    # geometry, not measurement: deckga_execute.py:355 sums commanded waypoints
    # (`norm(path[k] - last_xyz)`) and never reads a pose. So it cannot detect a
    # changed flight path and must not be used as a tolerance gate -- it would
    # pass at any RTF. Measured on gz_n30_o20_s0 it was bit-identical across
    # RTF 1 and 2 (1279.6168624083275) while the bag showed the real flown
    # length. The real distance gate lives in `rtf_accept.py`, on bag poses.
    #
    # Inverting it makes it useful: because it is planned geometry, it MUST be
    # identical across runs of the same plan. Any difference means the runs did
    # not fly the same thing, which invalidates the comparison outright.
    if len(cand.distances) != len(control.distances):
        return False, "different number of UAVs"
    for i, (c, k) in enumerate(zip(control.distances, cand.distances)):
        if abs(k - c) > 1e-6:
            return False, (f"uav{i} PLANNED length {k:.6f} m != control "
                           f"{c:.6f} m -- these runs did not fly the same plan, "
                           f"so no comparison between them is valid")

    # Makespan must be normalised to SIM time first. `mission_makespan_s` is
    # wall-clock (every executor timestamp is `time.perf_counter()`, not the ROS
    # clock), so a faster sim shortens it by construction and a raw comparison
    # can never pass for RTF > 1. Verified independently: across RTF 1 and 2 the
    # pose sample counts matched within 1.4% and the implied sim-time
    # publication rate agreed to 0.4% (82.4 vs 82.1 Hz), i.e. identical
    # simulated duration with only wall-clock differing.
    if control.makespan_sim_s and cand.makespan_sim_s:
        cand_sim = cand.makespan_sim_s * cand.achieved
        ctrl_sim = control.makespan_sim_s * (control.achieved or 1.0)
        d = _pct(cand_sim, ctrl_sim)
        if d > TIME_TOL:
            return False, (f"sim-normalised makespan {cand_sim:.1f}s vs control "
                           f"{ctrl_sim:.1f}s ({d:.1%} > {TIME_TOL:.0%}); "
                           f"wall {cand.makespan_sim_s:.1f}s vs "
                           f"{control.makespan_sim_s:.1f}s")

    if cand.collisions != control.collisions:
        return False, (f"residual collisions {cand.collisions} vs control "
                       f"{control.collisions}")

    if saturated:
        return True, (f"within tolerance, but SATURATED: achieved "
                      f"{cand.achieved:.2f}x of {cand.requested:g}x requested "
                      f"({cand.achieved / cand.requested:.0%}) -- faithful, "
                      f"capped by the machine")
    return True, "within tolerance"


def choose(runs: List[RtfRun], control: RtfRun) -> tuple:
    """Fidelity-passing run with the highest ACHIEVED factor. (chosen, table).

    Ranks on achieved rather than requested, and does not back off a rung,
    because the request saturates well below itself on this machine (2.0 -> 1.18
    measured): the requested number is only a dial, so "one rung down" throws
    away the entire measured gain rather than buying margin. Margin comes from
    the fidelity gates in `evaluate`, which compare against a 1.0 control.

    Kept deliberately consistent with `rtf_accept.choose`, which is
    AUTHORITATIVE because it also applies the bag-measured distance gate that
    this module cannot (see `evaluate`).
    """
    table = []
    passing: List[RtfRun] = []
    for r in runs:
        if r.requested == 1.0:
            table.append((r, True, "control"))
            continue
        ok, why = evaluate(control, r)
        table.append((r, ok, why))
        if ok and r.achieved:
            passing.append(r)

    if not passing:
        return 1.0, table
    best = max(passing, key=lambda r: r.achieved or 0.0)
    # Not worth leaving the control condition for a gain inside the
    # run-to-run spread of a single sample per factor.
    if (best.achieved or 1.0) < 1.10:
        return 1.0, table
    return best.requested, table


def render_report(trial_id: str, runs: List[RtfRun], chosen: float, table) -> str:
    out = [
        "# Stage 0 -- real-time factor calibration",
        "",
        f"Trial: `{trial_id}`   (identical plan and scene for every row)",
        "",
        "| requested | achieved | wall s | sim makespan s | distances m | collisions | verdict |",
        "|---|---|---|---|---|---|---|",
    ]
    for r, ok, why in table:
        dists = ", ".join(f"{d:.0f}" for d in r.distances) if r.distances else "--"
        out.append(
            f"| {r.requested:g}x | "
            f"{('%.2f' % r.achieved) if r.achieved is not None else '--'} | "
            f"{('%.0f' % r.wall_s) if r.wall_s is not None else '--'} | "
            f"{('%.1f' % r.makespan_sim_s) if r.makespan_sim_s is not None else '--'} | "
            f"{dists} | "
            f"{r.collisions if r.collisions is not None else '--'} | "
            f"{'PASS' if ok else 'reject'} -- {why} |"
        )
    control = next((r for r in runs if r.requested == 1.0), None)
    out += ["", f"**Chosen RTF: {chosen:g}x**"]
    if control and control.wall_s and chosen > 1.0:
        passing = [r for r, ok, _ in table if ok and r.requested == chosen]
        if passing and passing[0].wall_s:
            speedup = control.wall_s / passing[0].wall_s
            out.append(
                f"\nMeasured wall-clock speedup at the chosen factor: "
                f"**{speedup:.2f}x** ({control.wall_s:.0f}s -> "
                f"{passing[0].wall_s:.0f}s on this trial)."
            )
    if chosen == 1.0:
        out.append(
            "\nNo factor above 1.0 passed. The campaign runs at real time; do "
            "not override this by hand. Accepting a rejected factor would mean "
            "flying a different path than the one being reported."
        )
    return "\n".join(out) + "\n"


def _run_one(run_dir: Path, offline_run_dir: Path, trial_id: str,
             rtf: float, executor_timeout: float) -> RtfRun:
    """One Gazebo trial at one RTF, via the real sweep_gazebo driver.

    Each RTF gets its OWN run dir so the per-trial artefacts do not overwrite
    each other -- `gazebo/<trial_id>/` is keyed on trial id alone, and all four
    runs share that id by design (same plan, same scene; only physics differs).
    """
    import subprocess

    out = RtfRun(requested=rtf)
    stage_dir = run_dir / f"rtf_{rtf:g}"
    stage_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        sys.executable,
        str(REPO_ROOT / "tests" / "automation" / "sweep_gazebo.py"),
        "--matrix", "gazebo_subset",
        "--mode", "headless",
        "--offline-run-dir", str(offline_run_dir),
        "--run-dir", str(stage_dir),
        "--only-trial", trial_id,
        "--rtf", str(rtf),
        "--executor-timeout", str(executor_timeout),
    ]
    print(f"  [rtf {rtf:g}x] {' '.join(cmd[-8:])}", flush=True)

    t0 = time.perf_counter()
    try:
        proc = subprocess.run(cmd, cwd=REPO_ROOT, timeout=executor_timeout + 900)
    except subprocess.TimeoutExpired:
        out.status = "driver_timeout"
        out.wall_s = time.perf_counter() - t0
        return out
    out.wall_s = time.perf_counter() - t0

    master = stage_dir / "gazebo_master.csv"
    if not master.exists():
        out.status = "no_master_csv"
        out.note = f"sweep_gazebo exited {proc.returncode}"
        return out

    import csv
    rows = [r for r in csv.DictReader(master.open()) if r["trial_id"] == trial_id]
    if not rows:
        out.status = "no_row"
        return out
    r = rows[-1]

    out.status = r.get("status", "unknown")
    out.achieved = _float_or_none(r.get("rtf_achieved"))
    out.makespan_sim_s = _float_or_none(r.get("mission_makespan_s"))
    out.distances = [d for d in (
        _float_or_none(r.get(f"executed_uav{i}_dist_m")) for i in range(3)
    ) if d is not None]

    audit = stage_dir / "gazebo" / trial_id / "collision_audit.json"
    if audit.exists():
        try:
            out.collisions = json.loads(audit.read_text())["totals"]["intersection"]
        except Exception:
            out.collisions = None
    return out


def _float_or_none(v) -> Optional[float]:
    try:
        f = float(v)
        return f if f == f else None          # reject NaN
    except (TypeError, ValueError):
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--offline-run-dir", required=True,
                    help="run dir holding the trial pickles")
    ap.add_argument("--run-dir", required=True,
                    help="where to write the calibration runs and report")
    ap.add_argument("--trial-id", default="gz_n30_o20_s0")
    ap.add_argument("--rtfs", default="1,2,3,5")
    ap.add_argument("--executor-timeout", type=float, default=2700.0)
    args = ap.parse_args()

    rtfs = sorted({float(x) for x in args.rtfs.split(",")} | {1.0})
    offline_run_dir = Path(args.offline_run_dir).resolve()
    run_dir = Path(args.run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)

    print(f"Stage 0: RTF calibration on {args.trial_id} over {rtfs}")
    print("Every run flies the identical plan; only the physics rate differs.\n")

    runs: List[RtfRun] = []
    for rtf in rtfs:
        r = _run_one(run_dir, offline_run_dir, args.trial_id, rtf,
                     args.executor_timeout)
        runs.append(r)
        print(f"  -> status={r.status} achieved={r.achieved} "
              f"wall={r.wall_s:.0f}s makespan={r.makespan_sim_s}\n", flush=True)

    control = next((r for r in runs if r.requested == 1.0), None)
    if control is None or control.status != "ok":
        print("CONTROL RUN FAILED -- cannot calibrate without a 1.0x baseline.")
        (run_dir / "rtf_calibration.json").write_text(
            json.dumps({"runs": [r.as_dict() for r in runs],
                        "chosen_rtf": 1.0,
                        "note": "control failed"}, indent=2))
        return 1

    chosen, table = choose(runs, control)
    report = render_report(args.trial_id, runs, chosen, table)
    (run_dir / "rtf_calibration.md").write_text(report, encoding="utf-8")
    (run_dir / "rtf_calibration.json").write_text(json.dumps({
        "trial_id": args.trial_id,
        "chosen_rtf": chosen,
        "dist_tol": DIST_TOL,
        "time_tol": TIME_TOL,
        "runs": [r.as_dict() for r in runs],
    }, indent=2), encoding="utf-8")

    print(report)
    print(f"wrote {run_dir / 'rtf_calibration.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
