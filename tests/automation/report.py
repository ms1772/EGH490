#!/usr/bin/env python3
"""Stage 3: render the markdown report from offline_master.csv + summary.csv.

Auto-detects gazebo_master.csv and adds an Executed-vs-Planned table when present.

Usage:
    python tests/automation/report.py --run-dir <run_dir>
"""

from __future__ import annotations

import argparse
import csv
import math
import platform
import subprocess
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

# Force non-interactive backend before importing pyplot — works in WSL with no DISPLAY.
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def _safe_float(s):
    if s is None or s == "":
        return float("nan")
    try:
        return float(s)
    except (ValueError, TypeError):
        return float("nan")


def _fmt(v, prec=2):
    if v is None or v == "" or (isinstance(v, float) and math.isnan(v)):
        return "—"
    if isinstance(v, float):
        return f"{v:.{prec}f}"
    return str(v)


def _git_sha(repo_root: Path) -> str:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"], capture_output=True, text=True,
            cwd=repo_root, timeout=5,
        )
        if out.returncode == 0:
            return out.stdout.strip()[:12]
    except Exception:
        pass
    return "unknown"


def _git_dirty(repo_root: Path) -> bool:
    try:
        out = subprocess.run(
            ["git", "status", "--porcelain"], capture_output=True, text=True,
            cwd=repo_root, timeout=5,
        )
        return bool(out.stdout.strip())
    except Exception:
        return False


def _read_rows(csv_path: Path) -> List[dict]:
    with csv_path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _make_plots(summary: List[dict], offline: List[dict], fig_dir: Path) -> Dict[str, Path]:
    fig_dir.mkdir(parents=True, exist_ok=True)
    plots: Dict[str, Path] = {}

    # Distance vs obstacles, one panel per n_points
    by_npts: Dict[int, List[dict]] = defaultdict(list)
    for s in summary:
        by_npts[int(s["n_points"])].append(s)
    if by_npts:
        fig, axs = plt.subplots(1, len(by_npts), figsize=(5 * len(by_npts), 4), sharey=False, squeeze=False)
        axs = axs[0]
        for ax, (n_pts, srows) in zip(axs, sorted(by_npts.items())):
            srows = sorted(srows, key=lambda r: int(r["n_obstacles"]))
            xs = [int(r["n_obstacles"]) for r in srows]
            base = [_safe_float(r["distance_baseline_total_m_mean"]) for r in srows]
            base_std = [_safe_float(r["distance_baseline_total_m_std"]) for r in srows]
            av = [_safe_float(r["distance_avoided_total_m_mean"]) for r in srows]
            av_std = [_safe_float(r["distance_avoided_total_m_std"]) for r in srows]
            ax.errorbar(xs, base, yerr=base_std, fmt="o-", label="baseline (deckga)", capsize=3)
            ax.errorbar(xs, av, yerr=av_std, fmt="s-", label="avoided (quicknav)", capsize=3)
            ax.set_xlabel("# obstacles")
            ax.set_ylabel("fleet distance (m)")
            ax.set_title(f"n_points = {n_pts}")
            ax.grid(True, alpha=0.3)
            ax.legend()
        fig.suptitle("Fleet distance vs obstacle count (mean ± std)")
        fig.tight_layout()
        p = fig_dir / "distance_vs_obstacles.png"
        fig.savefig(p, dpi=120)
        plt.close(fig)
        plots["distance_vs_obstacles"] = p

    # Planning time breakdown (stacked bar per condition)
    if summary:
        labels = [f"n={s['n_points']}, obs={s['n_obstacles']}" for s in summary]
        deckga = [_safe_float(s["planning_time_deckga_s_mean"]) for s in summary]
        qn = [_safe_float(s["planning_time_quicknav_s_mean"]) for s in summary]
        fig, ax = plt.subplots(figsize=(max(8, len(labels) * 0.7), 4))
        x = range(len(labels))
        ax.bar(x, deckga, label="DECK-GA")
        ax.bar(x, qn, bottom=deckga, label="QuickNav")
        ax.set_xticks(list(x))
        ax.set_xticklabels(labels, rotation=45, ha="right")
        ax.set_ylabel("planning time (s)")
        ax.set_title("Planning-time breakdown per condition (mean)")
        ax.grid(True, alpha=0.3, axis="y")
        ax.legend()
        fig.tight_layout()
        p = fig_dir / "planning_time_breakdown.png"
        fig.savefig(p, dpi=120)
        plt.close(fig)
        plots["planning_time_breakdown"] = p

    # Avoidance penalty per condition with error bars
    if summary:
        labels = [f"n={s['n_points']}, obs={s['n_obstacles']}" for s in summary]
        pen = [_safe_float(s["penalty_mean_m"]) for s in summary]
        pen_std = [_safe_float(s["penalty_std_m"]) for s in summary]
        fig, ax = plt.subplots(figsize=(max(8, len(labels) * 0.7), 4))
        ax.bar(range(len(labels)), pen, yerr=pen_std, capsize=4)
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels(labels, rotation=45, ha="right")
        ax.set_ylabel("avoidance penalty (m)")
        ax.set_title("Avoidance penalty per condition (mean ± std)")
        ax.grid(True, alpha=0.3, axis="y")
        ax.axhline(0, color="k", linewidth=0.5)
        fig.tight_layout()
        p = fig_dir / "avoidance_penalty.png"
        fig.savefig(p, dpi=120)
        plt.close(fig)
        plots["avoidance_penalty"] = p

    # Residual collisions bar (should be all zeros)
    if summary:
        labels = [f"n={s['n_points']}, obs={s['n_obstacles']}" for s in summary]
        rc = [_safe_float(s["residual_collisions_total_mean"]) for s in summary]
        fig, ax = plt.subplots(figsize=(max(8, len(labels) * 0.7), 3.5))
        ax.bar(range(len(labels)), rc, color="#cc4444")
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels(labels, rotation=45, ha="right")
        ax.set_ylabel("mean residual collisions")
        ax.set_title("Residual collisions per condition (zero == avoidance success)")
        ax.grid(True, alpha=0.3, axis="y")
        fig.tight_layout()
        p = fig_dir / "collisions.png"
        fig.savefig(p, dpi=120)
        plt.close(fig)
        plots["collisions"] = p

    return plots


def _metric_table(summary: List[dict]) -> str:
    by_npts: Dict[int, List[dict]] = defaultdict(list)
    for s in summary:
        by_npts[int(s["n_points"])].append(s)
    out_lines = []
    for n_pts, rows in sorted(by_npts.items()):
        rows = sorted(rows, key=lambda r: int(r["n_obstacles"]))
        out_lines.append(f"\n### Waypoints = {n_pts}\n")
        out_lines.append(
            "| n_obs | trials | Baseline dist (m) | Avoided dist (m) | Penalty (m) | "
            "Detect | Avoid | Resid coll | Plan total (s) | DECK-GA (s) | QuickNav (s) |"
        )
        out_lines.append(
            "|------:|-------:|------------------:|-----------------:|-----------:|"
            "-------:|------:|-----------:|---------------:|------------:|-------------:|"
        )
        for r in rows:
            out_lines.append(
                f"| {r['n_obstacles']} | {r['n_trials']} "
                f"| {_fmt(_safe_float(r['distance_baseline_total_m_mean']))} ± {_fmt(_safe_float(r['distance_baseline_total_m_std']))} "
                f"| {_fmt(_safe_float(r['distance_avoided_total_m_mean']))} ± {_fmt(_safe_float(r['distance_avoided_total_m_std']))} "
                f"| {_fmt(_safe_float(r['penalty_mean_m']))} ± {_fmt(_safe_float(r['penalty_std_m']))} "
                f"| {_fmt(_safe_float(r['detections_total_mean']), 1)} "
                f"| {_fmt(_safe_float(r['avoidances_total_mean']), 1)} "
                f"| {_fmt(_safe_float(r['residual_collisions_total_mean']), 1)} "
                f"| {_fmt(_safe_float(r['planning_time_total_s_mean']))} "
                f"| {_fmt(_safe_float(r['planning_time_deckga_s_mean']))} "
                f"| {_fmt(_safe_float(r['planning_time_quicknav_s_mean']))} |"
            )
    return "\n".join(out_lines)


def _failures_section(offline: List[dict]) -> str:
    fail_rows = [
        r for r in offline
        if r.get("status") == "ok" and (
            _safe_float(r.get("residual_collisions_total")) > 0
            or (r.get("quicknav_warning") or "")
        )
    ]
    skip_rows = [r for r in offline if r.get("status") != "ok"]
    lines = ["\n## Avoidance failures & non-OK trials\n"]
    if not fail_rows and not skip_rows:
        lines.append(f"No avoidance failures observed across {len(offline)} trials.\n")
        return "\n".join(lines)
    if fail_rows:
        lines.append(f"**{len(fail_rows)}** trial(s) with residual collisions or QuickNav warning:\n")
        lines.append("| trial_id | n_pts | n_obs | seed | residual | warning |")
        lines.append("|----------|------:|------:|-----:|---------:|---------|")
        for r in fail_rows:
            lines.append(
                f"| {r['trial_id']} | {r['n_points']} | {r['n_obstacles']} | {r['seed']} "
                f"| {r['residual_collisions_total']} | {(r.get('quicknav_warning') or '').replace('|', '\\|')} |"
            )
        lines.append("")
    if skip_rows:
        lines.append(f"**{len(skip_rows)}** trial(s) did not complete successfully:\n")
        lines.append("| trial_id | status | error |")
        lines.append("|----------|--------|-------|")
        for r in skip_rows:
            err = (r.get("error") or "").replace("|", "\\|").replace("\n", " ")
            if len(err) > 160:
                err = err[:160] + "…"
            lines.append(f"| {r['trial_id']} | {r['status']} | {err} |")
        lines.append("")
    return "\n".join(lines)


def _paired_table(summary: List[dict]) -> str:
    lines = ["\n## Paired comparison (deckga vs quicknav)\n",
             "| n_pts | n_obs | Mean penalty (m) | 95% CI | p-value (paired t) |",
             "|------:|------:|-----------------:|--------|--------------------|"]
    for s in sorted(summary, key=lambda r: (int(r["n_points"]), int(r["n_obstacles"]))):
        lo = _safe_float(s.get("penalty_ci95_lo_m"))
        hi = _safe_float(s.get("penalty_ci95_hi_m"))
        pv = _safe_float(s.get("penalty_pvalue"))
        ci = f"[{_fmt(lo)}, {_fmt(hi)}]" if not (math.isnan(lo) or math.isnan(hi)) else "—"
        pv_s = _fmt(pv, 4) if not math.isnan(pv) else "—"
        lines.append(
            f"| {s['n_points']} | {s['n_obstacles']} "
            f"| {_fmt(_safe_float(s['penalty_mean_m']))} ± {_fmt(_safe_float(s['penalty_std_m']))} "
            f"| {ci} | {pv_s} |"
        )
    return "\n".join(lines)


def _gazebo_table(gz_rows: List[dict], offline_by_id: Dict[str, dict]) -> str:
    lines = ["\n## Executed vs Planned (Gazebo subset)\n",
             "| trial_id | n_pts | n_obs | mode | Planned dist (m) | Executed dist (m) | Δ (m) | Mission makespan (s) |",
             "|----------|------:|------:|------|-----------------:|------------------:|------:|---------------------:|"]
    for r in gz_rows:
        offline = offline_by_id.get(r.get("trial_id"), {})
        plan_dist = _safe_float(offline.get("distance_avoided_total_m"))
        exec_dist = _safe_float(r.get("executed_total_dist_m"))
        delta = exec_dist - plan_dist if not (math.isnan(plan_dist) or math.isnan(exec_dist)) else float("nan")
        lines.append(
            f"| {r['trial_id']} | {r.get('n_points', '?')} | {r.get('n_obstacles', '?')} "
            f"| {r.get('mode', '?')} | {_fmt(plan_dist)} | {_fmt(exec_dist)} | {_fmt(delta)} "
            f"| {_fmt(_safe_float(r.get('mission_makespan_s')))} |"
        )
    return "\n".join(lines)


def render(run_dir: Path) -> Path:
    offline_csv = run_dir / "offline_master.csv"
    summary_csv = run_dir / "summary.csv"
    if not offline_csv.exists() or not summary_csv.exists():
        raise FileNotFoundError(
            f"missing offline_master.csv or summary.csv in {run_dir} -- run aggregate.py first"
        )

    offline = _read_rows(offline_csv)
    summary = _read_rows(summary_csv)
    repo_root = Path(__file__).resolve().parents[2]
    plots = _make_plots(summary, offline, run_dir / "figures")

    n_ok = sum(1 for r in offline if r.get("status") == "ok")
    status_breakdown = defaultdict(int)
    for r in offline:
        status_breakdown[r.get("status", "?")] += 1

    md = []
    md.append("# DECK-GA + QuickNav simulation results\n")
    md.append("## Run metadata\n")
    md.append(f"- Run directory: `{run_dir}`")
    md.append(f"- Generated: {datetime.now().isoformat(timespec='seconds')}")
    md.append(f"- Git SHA: `{_git_sha(repo_root)}` {'(DIRTY)' if _git_dirty(repo_root) else ''}")
    md.append(f"- Host: {platform.node()} ({platform.system()} {platform.release()})")
    md.append(f"- Total trials: {len(offline)}  |  OK: {n_ok}")
    md.append("- Status breakdown: " + ", ".join(f"{k}={v}" for k, v in sorted(status_breakdown.items())))
    md.append("")

    md.append("## Per-condition metric tables\n")
    md.append(_metric_table(summary))
    md.append("")

    md.append("\n## Plots\n")
    for name, path in plots.items():
        rel = path.relative_to(run_dir)
        md.append(f"### {name.replace('_', ' ').title()}")
        md.append(f"![{name}]({rel.as_posix()})\n")

    md.append(_failures_section(offline))
    md.append(_paired_table(summary))

    # Gazebo subset table if present
    gz_csv = run_dir / "gazebo_master.csv"
    if gz_csv.exists():
        gz_rows = _read_rows(gz_csv)
        offline_by_id = {r["trial_id"]: r for r in offline}
        md.append(_gazebo_table(gz_rows, offline_by_id))

    md.append("\n## Appendix\n")
    md.append(f"- [offline_master.csv]({offline_csv.relative_to(run_dir).as_posix()})")
    md.append(f"- [summary.csv]({summary_csv.relative_to(run_dir).as_posix()})")
    if gz_csv.exists():
        md.append(f"- [gazebo_master.csv]({gz_csv.relative_to(run_dir).as_posix()})")

    out = run_dir / "report.md"
    out.write_text("\n".join(md), encoding="utf-8")
    print(f"[report] wrote {out}  ({len(plots)} figure(s))")
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    args = parser.parse_args()
    render(Path(args.run_dir).resolve())


if __name__ == "__main__":
    main()
