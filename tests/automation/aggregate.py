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
from typing import Dict, List

try:
    from scipy import stats as _scipy_stats
    _HAS_SCIPY = True
except ImportError:
    _scipy_stats = None
    _HAS_SCIPY = False


NUMERIC_METRICS = [
    "distance_baseline_total_m",
    "distance_avoided_total_m",
    "detections_total",
    "avoidances_total",
    "residual_collisions_total",
    "planning_time_total_s",
    "planning_time_deckga_s",
    "planning_time_quicknav_s",
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
            key = (int(r["n_points"]), int(r["n_obstacles"]))
            groups[key].append(r)

    summary_rows = []
    for (n_pts, n_obs), rows in sorted(groups.items()):
        row_out = {"n_points": n_pts, "n_obstacles": n_obs, "n_trials": len(rows)}
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
    args = parser.parse_args()
    aggregate(Path(args.run_dir).resolve())


if __name__ == "__main__":
    main()
