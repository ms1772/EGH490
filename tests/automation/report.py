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
        fig, ax = plt.subplots(figsize=(min(26, max(8, len(labels) * 0.7)), 4))
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
        fig, ax = plt.subplots(figsize=(min(26, max(8, len(labels) * 0.7)), 4))
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
        fig, ax = plt.subplots(figsize=(min(26, max(8, len(labels) * 0.7)), 3.5))
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

    hm = _plot_makespan_by_arm(summary, fig_dir)
    if hm is not None:
        plots["makespan_by_arm"] = hm

    return plots


def _plot_makespan_by_arm(summary: List[dict], fig_dir: Path) -> Optional[Path]:
    """THE headline figure: makespan per arm against obstacle density.

    The campaign had no plot of its own main result -- the arm comparison
    existed only as tables. The three legacy bar charts plot one bar per
    condition, which was readable at 8 conditions and is not at 76: they came
    out 6383 px wide, one bar per (n, density, fleet, arm).

    This is a line per ARM instead, faceted by waypoint count, which is the
    comparison the thesis actually makes and stays readable however many
    conditions accumulate. Restricted to the 3-UAV rows so the fleet sweep's
    1- and 2-UAV conditions cannot be silently averaged in beside them.
    """
    rows = [r for r in summary
            if r.get("arm") and str(r.get("num_uavs", "3")) == "3"]
    if not rows or len({r["arm"] for r in rows}) < 2:
        return None

    by_n: Dict[int, List[dict]] = defaultdict(list)
    for r in rows:
        by_n[int(float(r["n_points"]))].append(r)

    fig, axs = plt.subplots(1, len(by_n), figsize=(6 * len(by_n), 4.2),
                            squeeze=False)
    axs = axs[0]
    for ax, (n_pts, rs) in zip(axs, sorted(by_n.items())):
        for arm in sorted({r["arm"] for r in rs}):
            pts = sorted((r for r in rs if r["arm"] == arm),
                         key=lambda r: int(float(r["n_obstacles"])))
            xs = [int(float(r["n_obstacles"])) for r in pts]
            ys = [_safe_float(r.get("makespan_avoided_s_mean")) for r in pts]
            es = [_safe_float(r.get("makespan_avoided_s_std")) for r in pts]
            style = "-o" if arm.endswith("minmax") else "--s"
            lw = 2.2 if arm.endswith("minmax") else 1.3
            ax.errorbar(xs, ys, yerr=es, fmt=style, linewidth=lw, capsize=3,
                        label=arm, alpha=0.9)
        ax.set_xlabel("# obstacles")
        ax.set_ylabel("makespan (s)")
        ax.set_title(f"n_points = {n_pts}  (3 UAVs)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("Makespan by arm — solid = min-max objective, dashed = min-sum")
    fig.tight_layout()
    out = fig_dir / "makespan_by_arm.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def _makespan_by_arm(summary: list) -> str:
    """Makespan per arm -- the fleet metric the campaign is actually about.

    Kept separate from the per-condition table because that one is organised
    around the avoidance penalty (baseline vs avoided WITHIN a trial) and this
    one is organised around the arms (method vs method ACROSS trials).
    Collapsing them would put two different comparisons in one table.
    """
    rows = [r for r in summary if r.get("arm")]
    if not rows or len({r["arm"] for r in rows}) < 2:
        return ""

    def pm(r, col):
        try:
            return f"{float(r.get(col + '_mean')):.1f} ± {float(r.get(col + '_std')):.1f}"
        except (TypeError, ValueError):
            return "--"

    def one(r, col):
        try:
            return f"{float(r.get(col + '_mean')):.3f}"
        except (TypeError, ValueError):
            return "--"

    out = [
        "",
        "## Makespan by arm",
        "",
        "Makespan is the LONGEST tour, not the total. A fleet finishes when its",
        "slowest aircraft lands, so fleet distance can look unchanged while the",
        "mission takes materially longer.",
        "",
        "| arm | n_pts | n_obs | trials | makespan avoided (s) | max tour (m) "
        "| fleet distance (m) | balance ratio |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for r in sorted(rows, key=lambda x: (int(float(x.get("n_points") or 0)),
                                         int(float(x.get("n_obstacles") or 0)),
                                         str(x.get("arm")))):
        out.append(
            f"| {r['arm']} | {r.get('n_points')} | {r.get('n_obstacles')} "
            f"| {r.get('n_trials')} | {pm(r, 'makespan_avoided_s')} "
            f"| {pm(r, 'max_tour_avoided_m')} | {pm(r, 'distance_avoided_total_m')} "
            f"| {one(r, 'balance_ratio_avoided')} |"
        )
    out.append("")
    return "\n".join(out)


def _arm_vs_reference_table(run_dir) -> str:
    """Paired arm-against-baseline comparison -- the headline result.

    Every row pairs two arms on the SAME scene, so a difference is attributable
    to the method rather than to which scenes each arm happened to draw.
    Negative deltas are improvements for every metric listed.

    `improved/worsened` counts individual scenes, which is what distinguishes a
    consistent effect from a mean dragged around by one or two outliers.
    """
    csv_path = run_dir / "arm_vs_ref_summary.csv"
    if not csv_path.exists():
        return ""
    rows = _read_rows(csv_path)
    if not rows:
        return ""

    def num(v, spec=".2f"):
        try:
            return format(float(v), spec)
        except (TypeError, ValueError):
            return "--"

    def pct(v):
        try:
            return f"{float(v) * 100:.1f}%"
        except (TypeError, ValueError):
            return "--"

    out = [
        "",
        "## Paired comparison: each arm vs the published baseline",
        "",
        "Paired per scene AND per fleet size; negative is better.",
        "",
        "Fleet size is never pooled. Only A0 and A4 are swept over it, so a "
        "pooled mean would compare arms over different populations: pooling "
        "put A4 at -10.2% against A2's -13.6%, reading as 'A2 beats A4', "
        "purely because A4's figure was diluted by the 1-UAV cases where a "
        "min-max objective has nothing to balance. At matched fleet size A4 "
        "leads. The 1-UAV row is a deliberate negative control -- the method "
        "SHOULD do nothing there.",
        "",
        "| arm | UAVs | metric | pairs | mean delta | 95% CI | mean rel. "
        "| improved | worsened | p (paired t) |",
        "|---|---:|---|---:|---:|---|---:|---:|---:|---:|",
    ]
    for r in rows:
        out.append(
            f"| {r['arm']} | {r.get('num_uavs', '3')} | {r['metric']} | {r['n_pairs']} "
            f"| {num(r['mean_delta'])} "
            f"| [{num(r['ci95_lo'])}, {num(r['ci95_hi'])}] "
            f"| {pct(r['mean_rel_change'])} "
            f"| {r['improved']} | {r['worsened']} "
            f"| {num(r['pvalue_paired_t'], '.4f')} |"
        )
    out.append("")
    return "\n".join(out)


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
            # Escaped OUTSIDE the f-string: a backslash in an f-string
            # expression is only legal from Python 3.12 (PEP 701), and the
            # campaign runs on WSL's 3.10. Windows here is on 3.14, which
            # accepts it -- so this parsed locally and failed in the only
            # environment that matters.
            warn = (r.get("quicknav_warning") or "").replace("|", "\\|")
            lines.append(
                f"| {r['trial_id']} | {r['n_points']} | {r['n_obstacles']} | {r['seed']} "
                f"| {r['residual_collisions_total']} | {warn} |"
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


def _gazebo_by_arm(gz_rows: List[dict], offline_by_id: Dict[str, dict]) -> str:
    """Per-arm Gazebo makespan -- the transfer check, which is the point of S5.

    The offline sweep is the measurement; Gazebo asks a narrower question: does
    the planned makespan gain survive real controllers, or is it an artefact of
    the planner's own time model? That is an ARM-vs-ARM question, so an 80-row
    per-trial dump does not answer it and this table exists alongside it.

    `mission_makespan_s` is wall-clock (every executor timestamp is
    `time.perf_counter()`), which is directly comparable here only because the
    whole campaign flies at RTF 1 -- achieved 0.9917 on the control. Mixing real-
    time factors would require normalising by the achieved factor first.
    """
    by_arm: Dict[str, List[float]] = {}
    planned_by_arm: Dict[str, List[float]] = {}
    for r in gz_rows:
        if r.get("status") != "ok":
            continue
        off = offline_by_id.get(r.get("trial_id"), {})
        arm = (off.get("arm") or "").strip() or "unknown"
        mk = _safe_float(r.get("mission_makespan_s"))
        if not math.isnan(mk):
            by_arm.setdefault(arm, []).append(mk)
        pm = _safe_float(off.get("makespan_avoided_s"))
        if not math.isnan(pm):
            planned_by_arm.setdefault(arm, []).append(pm)
    if len(by_arm) < 2:
        return ""

    def mean(v):
        return sum(v) / len(v) if v else float("nan")

    def std(v):
        if len(v) < 2:
            return 0.0
        m = mean(v)
        return math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))

    ref = "A0_published"
    ref_mk = mean(by_arm.get(ref, [])) if ref in by_arm else float("nan")
    ref_pl = mean(planned_by_arm.get(ref, [])) if ref in planned_by_arm else float("nan")

    lines = [
        "\n## Gazebo transfer check: flown makespan by arm\n",
        "Does the planned makespan gain survive real controllers? "
        "`planned` is the planner's own estimate, `flown` is the mission as "
        "executed. Both deltas are against `A0_published`; agreement between "
        "them is the transfer result.\n",
        "| arm | trials | flown makespan (s) | Δ vs A0 | planned makespan (s) | Δ vs A0 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for arm in sorted(by_arm):
        v = by_arm[arm]
        p = planned_by_arm.get(arm, [])
        d_f = ((mean(v) - ref_mk) / ref_mk * 100.0
               if ref_mk and not math.isnan(ref_mk) else float("nan"))
        d_p = ((mean(p) - ref_pl) / ref_pl * 100.0
               if ref_pl and not math.isnan(ref_pl) else float("nan"))
        lines.append(
            f"| {arm} | {len(v)} | {mean(v):.1f} ± {std(v):.1f} "
            f"| {'--' if math.isnan(d_f) else format(d_f, '+.1f') + '%'} "
            f"| {'--' if not p else f'{mean(p):.1f} ± {std(p):.1f}'} "
            f"| {'--' if math.isnan(d_p) else format(d_p, '+.1f') + '%'} |"
        )
    return "\n".join(lines)


def _gazebo_table(gz_rows: List[dict], offline_by_id: Dict[str, dict]) -> str:
    """Per-trial Gazebo detail.

    NOTE ON THE DISTANCE COLUMNS. `executed_total_dist_m` is NOT a measurement:
    `deckga_execute.py:355` builds each segment from the commanded waypoint
    (`norm(path[k] - last_xyz)`) and never reads a pose, so it is planned
    polyline geometry. It was previously differenced against the planner's own
    distance and labelled "Δ", which compared planned against planned and
    printed a near-zero number that looked like flawless tracking. It is
    labelled honestly here; real flown distance comes from
    `campaign/bag_distance.py`, which integrates bag poses.
    """
    lines = ["\n## Gazebo per-trial detail\n",
             "`commanded dist` is planned geometry, not a measurement -- see "
             "the note in the source. Use `bag_distance.py` for flown "
             "distance.\n",
             "| trial_id | n_pts | n_obs | mode | status | planner dist (m) | commanded dist (m) | mission makespan (s) | collisions |",
             "|----------|------:|------:|------|--------|-----------------:|-------------------:|---------------------:|-----------:|"]
    for r in gz_rows:
        offline = offline_by_id.get(r.get("trial_id"), {})
        plan_dist = _safe_float(offline.get("distance_avoided_total_m"))
        cmd_dist = _safe_float(r.get("executed_total_dist_m"))
        lines.append(
            f"| {r['trial_id']} | {r.get('n_points', '?')} | {r.get('n_obstacles', '?')} "
            f"| {r.get('mode', '?')} | {r.get('status', '?')} | {_fmt(plan_dist)} "
            f"| {_fmt(cmd_dist)} "
            f"| {_fmt(_safe_float(r.get('mission_makespan_s')))} "
            f"| {r.get('audit_intersections', '?')} |"
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

    md.append(_makespan_by_arm(summary))
    md.append(_arm_vs_reference_table(run_dir))
    md.append(_failures_section(offline))
    md.append(_paired_table(summary))

    # Gazebo subset table if present
    gz_csv = run_dir / "gazebo_master.csv"
    if gz_csv.exists():
        gz_rows = _read_rows(gz_csv)
        offline_by_id = {r["trial_id"]: r for r in offline}
        # Arm summary first: it is the transfer result. The per-trial table is
        # 80 rows of supporting detail.
        md.append(_gazebo_by_arm(gz_rows, offline_by_id))
        md.append(_gazebo_table(gz_rows, offline_by_id))

    md.append("\n## Appendix\n")
    md.append(f"- [offline_master.csv]({offline_csv.relative_to(run_dir).as_posix()})")
    md.append(f"- [summary.csv]({summary_csv.relative_to(run_dir).as_posix()})")
    if gz_csv.exists():
        md.append(f"- [gazebo_master.csv]({gz_csv.relative_to(run_dir).as_posix()})")
    arm_csv = run_dir / "arm_vs_ref_summary.csv"
    if arm_csv.exists():
        md.append(f"- [arm_vs_ref_summary.csv]({arm_csv.relative_to(run_dir).as_posix()})")

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
