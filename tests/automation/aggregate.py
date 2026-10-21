#!/usr/bin/env python3
"""Stage 2: aggregate offline_master.csv into summary.csv (mean/std per condition).

Groups by (n_points, n_obstacles); filters status=ok; computes mean, std, n,
plus paired (avoided - baseline) penalty with CI.

Usage:
    python tests/automation/aggregate.py --run-dir <run_dir>
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional

try:
    from scipy import stats as _scipy_stats
    _HAS_SCIPY = True
except ImportError:
    _scipy_stats = None
    _HAS_SCIPY = False


# The dimensions a summary row is grouped by. Declared once rather than
# hardcoded at the key-construction site, because the campaign adds `arm` and
# the old two-dimension assumption was baked into the group key AND its unpack
# AND six label-building sites in report.py.
#
# `arm` is last so that sorted() still orders by condition first, which keeps
# the existing table layout readable.
# `num_uavs` is a grouping dimension, not a constant. It was 3 everywhere until
# the fleet matrix, which sweeps {1,2,3}; without it here, aggregate() pools all
# three fleet sizes into one mean and reports a number describing no actual
# condition. Rows that predate the fleet matrix all carry num_uavs=3, so adding
# it does not resplit any existing group.
GROUP_KEYS = ["n_points", "n_obstacles", "num_uavs", "arm"]


def _group_key(row: dict) -> tuple:
    """Tuple of the grouping dimensions, tolerant of older CSVs.

    Rows written before the campaign have no `arm` column. Defaulting them to
    'published_legacy' rather than '' keeps them visible as their own group
    instead of silently pooling them with a new arm's rows."""
    out = []
    for k in GROUP_KEYS:
        v = row.get(k)
        if k in ("n_points", "n_obstacles", "num_uavs"):
            out.append(int(float(v)) if v not in (None, "") else -1)
        else:
            out.append(v if v else "published_legacy")
    return tuple(out)


NUMERIC_METRICS = [
    "distance_baseline_total_m",
    "distance_avoided_total_m",
    "detections_total",
    "avoidances_total",
    "residual_collisions_total",
    "planning_time_total_s",
    "planning_time_deckga_s",
    "planning_time_quicknav_s",
    # campaign additions -- makespan is the fleet metric, total distance hides it
    "max_tour_baseline_m",
    "max_tour_avoided_m",
    "makespan_baseline_s",
    "makespan_avoided_s",
    "balance_ratio_baseline",
    "balance_ratio_avoided",
    "n_detour_edges",
    "n_infeasible_edges",
    "n_unreachable_waypoints",
]


def _safe_float(s):
    if s is None or s == "":
        return float("nan")
    try:
        return float(s)
    except (ValueError, TypeError):
        return float("nan")


def _mean_std(values: List[float]) -> tuple:
    clean = [v for v in values if not math.isnan(v)]
    if not clean:
        return float("nan"), float("nan"), 0
    n = len(clean)
    m = sum(clean) / n
    if n > 1:
        var = sum((v - m) ** 2 for v in clean) / (n - 1)
        s = math.sqrt(var)
    else:
        s = 0.0
    return m, s, n


def _ci95(values: List[float]) -> tuple:
    """95% CI of the mean. Uses scipy t if available, else normal approximation."""
    clean = [v for v in values if not math.isnan(v)]
    n = len(clean)
    if n < 2:
        return float("nan"), float("nan")
    m = sum(clean) / n
    var = sum((v - m) ** 2 for v in clean) / (n - 1)
    s = math.sqrt(var)
    se = s / math.sqrt(n)
    if _HAS_SCIPY:
        tcrit = _scipy_stats.t.ppf(0.975, df=n - 1)
    else:
        tcrit = 1.96
    return m - tcrit * se, m + tcrit * se


def _paired_pvalue(diffs: List[float]) -> float:
    """Paired t-test p-value for H0: diff mean == 0. NaN if scipy unavailable
    or sample too small."""
    clean = [v for v in diffs if not math.isnan(v)]
    if not _HAS_SCIPY or len(clean) < 2:
        return float("nan")
    res = _scipy_stats.ttest_1samp(clean, popmean=0.0)
    return float(res.pvalue)


REFERENCE_ARM = "A0_published"

# Metrics compared arm-against-reference. Makespan first: it is the claim.
PAIRED_METRICS = [
    "makespan_avoided_s",
    "max_tour_avoided_m",
    "distance_avoided_total_m",
    "balance_ratio_avoided",
    "planning_time_total_s",
]


def paired_vs_reference(run_dir: Path, reference_arm: str = REFERENCE_ARM) -> Optional[Path]:
    """Per-scene paired comparison of each arm against the reference.

    PAIRED, not grouped means: every comparison is between two arms planning
    THE SAME SCENE with the same planner seed. That is the only way the
    difference is attributable to the method rather than to the scene, and it
    is why `scene_id` exists and excludes the arm.

    The existing penalty block in `aggregate()` is a within-row comparison
    (avoided vs baseline on one trial). This is across rows, so it is a new
    function rather than an extra column.

    Writes arm_vs_ref_summary.csv. Returns None when there is nothing to pair,
    which is the normal case for a single-arm run.
    """
    master = run_dir / "offline_master.csv"
    if not master.exists():
        return None

    with master.open("r", encoding="utf-8") as f:
        rows = [r for r in csv.DictReader(f) if r.get("status") == "ok"]
    if not rows:
        return None

    def scene_of(r: dict) -> str:
        """Always RECONSTRUCTED, never read from the stored column.

        The stored `scene_id` is a derived value written at sweep time, so it
        reflects whatever definition was current then. Rows written before
        num_uavs joined the scene key carry a value that collides across fleet
        sizes -- and trusting it would pair a 1-UAV run against a 3-UAV one and
        report the difference as a method effect. Deriving it here from the
        row's own dimensions means the pairing is correct for any row, however
        old, and cannot drift again.

        Must match TrialSpec.scene_id exactly, including the non-default-only
        `_u` suffix.
        """
        obs = r.get("obs_size") or "5"
        sid = (f"n{r.get('n_points')}_o{r.get('n_obstacles')}"
               f"_z{float(obs):g}_s{r.get('seed')}")
        try:
            u = int(float(r.get("num_uavs") or 3))
        except (TypeError, ValueError):
            u = 3
        if u != 3:
            sid += f"_u{u}"
        return sid

    # (scene, planner_seed) -> arm -> row
    cells: dict = {}
    for r in rows:
        arm = r.get("arm") or "published_legacy"
        key = (scene_of(r), r.get("planner_seed") or "")
        cells.setdefault(key, {})[arm] = r

    # Fleet size is reported SEPARATELY, never pooled.
    #
    # Only some arms are swept over fleet size (the fleet matrix runs A0 and A4
    # only), so a pooled mean compares arms over different populations. Measured
    # here: pooling put A4 at -10.2% over 300 pairs against A2 at -13.6% over
    # 180, which reads as "A2 beats A4" and is simply false -- A4's figure was
    # diluted by the 1-UAV cases, where a min-max objective has nothing to
    # balance and correctly does nothing. At matched fleet size A4 leads.
    def uav_of(r: dict) -> str:
        try:
            return str(int(float(r.get("num_uavs") or 3)))
        except (TypeError, ValueError):
            return "3"

    arms = sorted({(r.get("arm") or "published_legacy") for r in rows})
    others = [a for a in arms if a != reference_arm]
    if reference_arm not in arms or not others:
        return None

    fleet_sizes = sorted({uav_of(r) for r in rows}, key=int)

    out_rows = []
    for arm in others:
      for uav in fleet_sizes:
        for metric in PAIRED_METRICS:
            diffs, rels = [], []
            for key, by_arm in cells.items():
                # A planner seed only exists on the stochastic arms, so pair the
                # reference by scene alone when its seed slot is empty.
                ref = by_arm.get(reference_arm)
                if ref is None:
                    ref = next((v.get(reference_arm)
                                for k, v in cells.items()
                                if k[0] == key[0] and reference_arm in v), None)
                cand = by_arm.get(arm)
                if ref is None or cand is None:
                    continue
                # Both sides must be the same fleet size, or the "difference"
                # is partly a difference in the problem rather than the method.
                if uav_of(cand) != uav or uav_of(ref) != uav:
                    continue
                a, b = _safe_float(cand.get(metric)), _safe_float(ref.get(metric))
                if a != a or b != b:          # NaN
                    continue
                diffs.append(a - b)
                if b != 0:
                    rels.append((a - b) / abs(b))
            if not diffs:
                continue
            m, sd, n = _mean_std(diffs)
            lo, hi = _ci95(diffs)
            rm, _rs, _rn = _mean_std(rels)
            out_rows.append({
                "arm": arm,
                "reference": reference_arm,
                "num_uavs": uav,
                "metric": metric,
                "n_pairs": n,
                "mean_delta": m,
                "std_delta": sd,
                "ci95_lo": lo,
                "ci95_hi": hi,
                "mean_rel_change": rm,
                "pvalue_paired_t": _paired_pvalue(diffs),
                "improved": sum(1 for d in diffs if d < 0),
                "worsened": sum(1 for d in diffs if d > 0),
            })

    if not out_rows:
        return None
    out_csv = run_dir / "arm_vs_ref_summary.csv"
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(out_rows[0].keys()))
        w.writeheader()
        w.writerows(out_rows)
    return out_csv


def aggregate(run_dir: Path) -> Path:
    csv_in = run_dir / "offline_master.csv"
    if not csv_in.exists():
        raise FileNotFoundError(f"missing {csv_in}")
    out_csv = run_dir / "summary.csv"

    # group by (n_points, n_obstacles); collect per-trial metric vectors
    groups: Dict[tuple, List[dict]] = defaultdict(list)
    total_rows = 0
    ok_rows = 0
    with csv_in.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            total_rows += 1
            if r.get("status") != "ok":
                continue
            ok_rows += 1
            groups[_group_key(r)].append(r)

    summary_rows = []
    for key, rows in sorted(groups.items(), key=lambda kv: [str(x) for x in kv[0]]):
        row_out = dict(zip(GROUP_KEYS, key))
        row_out["n_trials"] = len(rows)
        for col in NUMERIC_METRICS:
            vals = [_safe_float(r.get(col)) for r in rows]
            m, s, n = _mean_std(vals)
            row_out[f"{col}_mean"] = m
            row_out[f"{col}_std"] = s
            row_out[f"{col}_n"] = n
        # Paired (avoided - baseline) penalty
        diffs = [
            _safe_float(r["distance_avoided_total_m"]) - _safe_float(r["distance_baseline_total_m"])
            for r in rows
        ]
        m, s, n = _mean_std(diffs)
        lo, hi = _ci95(diffs)
        row_out["penalty_mean_m"] = m
        row_out["penalty_std_m"] = s
        row_out["penalty_ci95_lo_m"] = lo
        row_out["penalty_ci95_hi_m"] = hi
        row_out["penalty_pvalue"] = _paired_pvalue(diffs)
        # Avoidance-failure flags (any trial with residual > 0 or QuickNav warning)
        failures = [r for r in rows if _safe_float(r.get("residual_collisions_total")) > 0
                    or (r.get("quicknav_warning") or "")]
        row_out["avoidance_failures"] = len(failures)
        summary_rows.append(row_out)

    if not summary_rows:
        print(f"[aggregate] WARN: no ok rows in {csv_in} (total={total_rows})")

    fieldnames = list(summary_rows[0].keys()) if summary_rows else [
        "n_points", "n_obstacles", "n_trials"
    ]
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)

    print(f"[aggregate] {len(summary_rows)} condition(s) from {ok_rows}/{total_rows} ok rows")
    print(f"[aggregate] wrote {out_csv}")
    return out_csv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--reference-arm", default=REFERENCE_ARM,
                        help="Arm that every other arm is compared against.")
    args = parser.parse_args()
    run_dir = Path(args.run_dir).resolve()
    aggregate(run_dir)

    # Cross-arm paired comparison. Silently absent for single-arm runs, which
    # is every historical run dir -- so this stays backward compatible.
    paired = paired_vs_reference(run_dir, args.reference_arm)
    if paired:
        print(f"[aggregate] wrote {paired}")
    else:
        print("[aggregate] no cross-arm pairs to compare (single-arm run)")


if __name__ == "__main__":
    main()
