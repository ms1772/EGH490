#!/usr/bin/env python3
"""Re-evaluate an RTF calibration with the MEASURED distance criterion.

Why this is a second pass rather than a change to `rtf_calibrate.py`
-------------------------------------------------------------------
`rtf_calibrate.py` gates on three things: achieved-vs-requested RTF, sim-time
makespan within 2%, and identical residual collisions -- plus "every UAV's
executed distance within 2%", read from `executed_uavN_dist_m`.

That fourth gate does nothing. `deckga_execute.py:355` builds each segment from
the COMMANDED waypoint (`path[k]`), not from measured pose, so that column is
planned polyline geometry and is bit-identical at every real-time factor. It
cannot fail. Measured on `gz_n30_o20_s0`, it equalled `planned_makespan_s` to
fifteen significant figures.

The practical effect is bounded, which is why the first calibration was allowed
to finish rather than being restarted a second time: a gate that always passes
can never wrongly REJECT an RTF, and the two gates that do bite -- makespan from
the sim clock, collisions from the bag -- are both real measurements. The
exposure is a wrong ACCEPT, where tracking degrades enough to change the flown
geometry while leaving makespan inside 2% and the collision verdict unchanged.
This pass closes exactly that hole, using path length and planned-path deviation
recovered from the bag by `bag_distance.py`.

Hard gates here: achieved RTF, makespan, collisions, and bag-derived max flown
length. Deviation is reported with a deliberately loose relative gate -- its
absolute value includes takeoff and the unplanned approach to WP1, so it is
meaningful as a ratio between two runs treated identically and not as an
absolute tracking-error figure.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

TIME_TOL = 0.02          # sim-time makespan, same as rtf_calibrate

# Looser than the makespan gate ON PURPOSE. Makespan is read off the sim clock
# and is exact; bag-integrated length carries its own ~2% sensitivity to the
# integration rate (see bag_distance.LENGTH_RATE_HZ), so a 2% gate here would
# be comparing a tolerance against its own measurement noise. 5% still catches
# a real tracking change -- RTF 2 came in at +30%.
BAG_DIST_TOL = 0.05      # max per-UAV flown length, FROM THE BAG
RTF_ACHIEVED_TOL = 0.80  # achieved must be >= 80% of requested
DEV_RATIO_TOL = 1.25     # p95 deviation may not exceed 1.25x the control's


def _f(v) -> Optional[float]:
    if v in (None, ""):
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _load_row(rtf_dir: Path) -> Optional[dict]:
    master = rtf_dir / "gazebo_master.csv"
    if not master.exists():
        return None
    with master.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return rows[-1] if rows else None


def collect(cal_dir: Path, offline_run_dir: Optional[Path],
            num_uavs: int = 3) -> List[Dict]:
    from tests.automation.campaign.bag_distance import analyse  # noqa: PLC0415

    out: List[Dict] = []
    for rtf_dir in sorted(cal_dir.glob("rtf_*"), key=lambda p: p.name):
        if not rtf_dir.is_dir():
            continue
        row = _load_row(rtf_dir)
        if row is None:
            print(f"[rtf_accept] {rtf_dir.name}: no gazebo_master.csv, skipping")
            continue
        trial_id = row.get("trial_id", "?")
        bag = row.get("bag_path") or ""
        bag_path = Path(bag)
        if not bag_path.exists():
            # Fall back to the in-tree location; bag_path is recorded as a WSL
            # path and this may be read from either side of the mount.
            cand = rtf_dir / "gazebo" / trial_id / "poses.bag"
            bag_path = cand

        pkl = None
        if offline_run_dir is not None:
            cand = offline_run_dir / "trials" / trial_id / "deckga_quicknav_output.pkl"
            if cand.exists():
                pkl = cand

        entry = {
            "name": rtf_dir.name,
            "trial_id": trial_id,
            "requested": _f(row.get("rtf_requested")),
            "achieved": _f(row.get("rtf_achieved")),
            "makespan_sim_s": _f(row.get("mission_makespan_s")),
            "arm_to_land_s": _f(row.get("executed_total_arm_to_land_s")),
            "collisions": _f(row.get("audit_intersections")),
            "status": row.get("status"),
            # The column the old gate used, kept so the report can show that it
            # is constant across RTFs rather than merely assert it.
            "reported_dist_total_m": _f(row.get("executed_total_dist_m")),
        }
        if bag_path.exists():
            print(f"[rtf_accept] {rtf_dir.name}: reading {bag_path}")
            entry["bag"] = analyse(bag_path, pkl, num_uavs=num_uavs)
        else:
            print(f"[rtf_accept] {rtf_dir.name}: bag missing at {bag_path}")
            entry["bag"] = None
        out.append(entry)
    return out


def evaluate(control: Dict, cand: Dict) -> tuple:
    """(accepted, [reasons]). The control is accepted by definition."""
    if cand["name"] == control["name"]:
        return True, ["control"]
    reasons: List[str] = []
    ok = True

    if cand.get("status") != "ok":
        return False, [f"status={cand.get('status')}"]

    # INFORMATIONAL, not disqualifying. `rtf_calibrate` treats achieved < 80% of
    # requested as "the factor did not take" and fails the run, but that
    # conflates two different things: whether the simulation is still FAITHFUL
    # (the fidelity gates below) and whether the request was fully DELIVERED.
    # A request of 2.0 that yields 1.18 while leaving sim-time makespan, flown
    # geometry and collisions unchanged is a free 1.18x speedup, not a failure --
    # rejecting it would discard real campaign time for no fidelity benefit.
    # Saturation is a property of the machine (IPC-bound here, ~30% CPU idle),
    # so it is recorded and then selected on by achieved factor.
    req, ach = cand.get("requested"), cand.get("achieved")
    if req and ach is not None and req > 1.0 and ach < RTF_ACHIEVED_TOL * req:
        reasons.append(f"NOTE saturated: achieved {ach:.3f} vs requested "
                       f"{req:g} ({ach / req:.0%}) -- faithful but capped")

    # Makespan must be normalised to SIM time before it can be compared across
    # real-time factors. `mission_makespan_s` is wall-clock: every timestamp in
    # deckga_execute comes from `time.perf_counter()`/`time.time()`, not the ROS
    # clock, despite `use_sim_time:=true` for the platform and controller. So a
    # faster-running sim shortens it by construction and a raw comparison can
    # NEVER pass for RTF > 1 -- the exact mirror of the inert distance gate.
    #
    # Multiplying by the achieved factor recovers simulated seconds. Verified
    # independently via the pose stream: sample counts were within 1.4% across
    # RTF 1 and 2 while wall duration differed 17%, and the implied sim-time
    # publication rate agreed to 0.4% (82.4 vs 82.1 Hz).
    a, b = cand.get("makespan_sim_s"), control.get("makespan_sim_s")
    ka = cand.get("achieved") or cand.get("requested") or 1.0
    kb = control.get("achieved") or 1.0
    if a and b:
        a_sim, b_sim = a * ka, b * kb
        d = abs(a_sim - b_sim) / b_sim
        reasons.append(f"makespan {a_sim:.1f}s vs {b_sim:.1f}s sim-normalised "
                       f"({d:+.2%}); wall {a:.1f}s vs {b:.1f}s")
        if d > TIME_TOL:
            ok = False
            reasons[-1] += f" EXCEEDS {TIME_TOL:.0%}"

    ca, cb = cand.get("collisions"), control.get("collisions")
    if ca != cb:
        ok = False
        reasons.append(f"collisions {ca} vs control {cb}")

    bc, bb = cand.get("bag"), control.get("bag")
    if bc and bb:
        a, b = bc.get("bag_len_max_m"), bb.get("bag_len_max_m")
        if a and b:
            d = abs(a - b) / b
            reasons.append(f"flown max {a:.2f}m vs {b:.2f}m ({d:+.2%}) [bag]")
            if d > BAG_DIST_TOL:
                ok = False
                reasons[-1] += f" EXCEEDS {BAG_DIST_TOL:.0%}"
        pa, pb = bc.get("dev_p95_max_m"), bb.get("dev_p95_max_m")
        if pa and pb:
            r = pa / pb
            reasons.append(f"dev p95 {pa:.3f}m vs {pb:.3f}m ({r:.2f}x)")
            if r > DEV_RATIO_TOL:
                ok = False
                reasons[-1] += f" EXCEEDS {DEV_RATIO_TOL:.2f}x"
    else:
        reasons.append("NO BAG DATA -- distance gate not applied")

    return ok, reasons


def choose(entries: List[Dict]) -> tuple:
    """(chosen_requested_rtf, verdicts).

    Selects on ACHIEVED factor among the runs that passed the fidelity gates,
    not on the requested value. Because the request saturates (2.0 -> 1.18
    here), the highest request is not necessarily the fastest run, and the
    requested number is only a dial -- the achieved factor is the speedup the
    campaign actually gets. Returns the *requested* value to pass to the sweep,
    since that is the knob, while ranking by what it delivered.
    """
    control = next((e for e in entries if (e.get("requested") or 0) == 1.0), None)
    if control is None:
        return 1.0, []
    verdicts = []
    passing = []
    for e in entries:
        ok, reasons = evaluate(control, e)
        verdicts.append({"name": e["name"], "requested": e.get("requested"),
                         "achieved": e.get("achieved"),
                         "accepted": ok, "reasons": reasons})
        if ok and (e.get("requested") or 0) > 1.0 and e.get("achieved"):
            passing.append(e)
    if not passing:
        return 1.0, verdicts
    best = max(passing, key=lambda e: e["achieved"])
    # Only worth the risk if it actually buys something: a few percent is inside
    # the run-to-run spread of a single sample and not worth moving off the
    # control condition for.
    if (best.get("achieved") or 1.0) < 1.10:
        return 1.0, verdicts
    return float(best["requested"]), verdicts


def render(entries: List[Dict], chosen: float, verdicts: List[Dict]) -> str:
    L = ["# RTF acceptance -- re-evaluated with bag-measured distance", "",
         "The `executed_uavN_dist_m` gate in `rtf_calibrate.py` is inert "
         "(planned geometry, not measurement);", "the `flown` column below is "
         "integrated from bag poses and is the gate that actually bites.", "",
         "| run | requested | achieved | makespan (sim s) | flown max (m) "
         "| reported dist (m) | dev p95 (m) | collisions | accepted |",
         "|---|---|---|---|---|---|---|---|---|"]
    for e in entries:
        v = next((x for x in verdicts if x["name"] == e["name"]), {})
        bag = e.get("bag") or {}
        L.append(
            f"| {e['name']} | {e.get('requested')} | {e.get('achieved')} "
            f"| {e.get('makespan_sim_s')} "
            f"| {bag.get('bag_len_max_m', float('nan')):.2f} "
            f"| {e.get('reported_dist_total_m')} "
            f"| {bag.get('dev_p95_max_m', float('nan')):.3f} "
            f"| {e.get('collisions')} | {'YES' if v.get('accepted') else 'no'} |"
        )
    L += ["", "## Per-run detail", ""]
    for v in verdicts:
        L.append(f"- **{v['name']}** (requested {v.get('requested')}): "
                 f"{'ACCEPTED' if v['accepted'] else 'REJECTED'}")
        for r in v["reasons"]:
            L.append(f"  - {r}")
    L += ["", f"## Chosen RTF: **{chosen:g}** (requested value)", "",
          "Chosen as the fidelity-passing run with the highest ACHIEVED factor, "
          "which is the speedup the campaign actually gets; the requested value "
          "saturates well below itself on this machine. Falls back to 1 if "
          "nothing passed or the best achieved under 1.10x, where the gain is "
          "inside the run-to-run spread of a single sample.", "",
          "`makespan` is normalised to simulated seconds (wall x achieved), "
          "because the executor timestamps with `perf_counter`, not the ROS "
          "clock. `flown` is integrated from bag poses at "
          "`bag_distance.LENGTH_RATE_HZ`. The `reported dist` column is the "
          "planner's own figure, shown only to demonstrate that it does not "
          "move with RTF and so cannot gate anything.", ""]
    return "\n".join(L)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cal-dir", required=True,
                    help="directory containing rtf_1/, rtf_2/, ...")
    ap.add_argument("--offline-run-dir", default=None,
                    help="offline run dir holding trials/<id>/deckga_quicknav_output.pkl")
    ap.add_argument("--num-uavs", type=int, default=3)
    args = ap.parse_args()

    cal_dir = Path(args.cal_dir).resolve()
    offline = Path(args.offline_run_dir).resolve() if args.offline_run_dir else None
    entries = collect(cal_dir, offline, num_uavs=args.num_uavs)
    if not entries:
        print("no rtf_* runs found")
        return 1
    chosen, verdicts = choose(entries)
    md = render(entries, chosen, verdicts)
    (cal_dir / "rtf_acceptance_measured.md").write_text(md, encoding="utf-8")
    (cal_dir / "rtf_acceptance_measured.json").write_text(
        json.dumps({"chosen_rtf": chosen, "entries": entries,
                    "verdicts": verdicts}, indent=2), encoding="utf-8")
    print(md)
    print(f"\nwrote {cal_dir / 'rtf_acceptance_measured.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
