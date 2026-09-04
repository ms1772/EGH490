#!/usr/bin/env python3
"""Stage 4a.5: post-flight collision audit from ros2 bag + trial pickle.

For each UAV trajectory recorded in the bag, classify each segment between
adjacent (downsampled) pose samples as:
  - clean:        no intersection with any obstacle cube
  - scrape:       penetration depth < scrape_threshold * half_size (within margin)
  - intersection: penetration depth >= scrape_threshold * half_size (real collision)

Default scrape_threshold = 0.10 (10% of the obstacle's half-size). At the
default obs_size=5.0, that means depth < 0.5 m is "within control noise +
safety margin".

Outputs:
  - <trial_dir>/collision_audit.json
  - stdout markdown summary block

Usage:
    python tests/automation/collision_audit.py \\
        --bag <path>/poses.bag \\
        --pickle <trial>/deckga_quicknav_output.pkl \\
        --out <trial_dir>/collision_audit.json \\
        [--scrape-threshold 0.10]
"""

from __future__ import annotations

import argparse
import json
import pickle
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from QuickNav_detection_3D import line_intersects_cube  # noqa: E402
from tests.automation.metrics import penetration_depth, coerce_sizes  # noqa: E402


def read_bag_poses(bag_path: Path, topic: str,
                   downsample_hz: float = 10.0) -> List[Tuple[float, np.ndarray]]:
    """Read pose timeseries from a rosbag2 bag for one topic. Returns
    [(t_seconds, np.array([x,y,z]))] downsampled to ~downsample_hz.

    Tries rosbag2_py; raises clear error if unavailable.
    """
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        raise RuntimeError(
            f"rosbag2_py / rclpy not available ({e}). Source ROS2 humble first."
        )

    reader = SequentialReader()
    storage_opts = StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_opts = ConverterOptions(input_serialization_format="cdr",
                                      output_serialization_format="cdr")
    reader.open(storage_opts, converter_opts)

    type_map = {ti.name: ti.type for ti in reader.get_all_topics_and_types()}
    if topic not in type_map:
        return []
    msg_type = get_message(type_map[topic])

    out: List[Tuple[float, np.ndarray]] = []
    min_dt = 1.0 / max(0.1, downsample_hz)
    last_t = -1e9
    reader.set_filter(__import__("rosbag2_py").StorageFilter(topics=[topic]))
    while reader.has_next():
        topic_name, raw, t_ns = reader.read_next()
        t = t_ns / 1e9
        if t - last_t < min_dt:
            continue
        last_t = t
        msg = deserialize_message(raw, msg_type)
        # PoseStamped: msg.pose.position. Pose: msg.position.
        pos = getattr(msg, "pose", msg)
        if hasattr(pos, "pose"):
            pos = pos.pose
        if hasattr(pos, "position"):
            pos = pos.position
        out.append((t, np.array([pos.x, pos.y, pos.z], dtype=float)))
    return out


def audit_trajectory(samples: List[Tuple[float, np.ndarray]],
                      obstacle_xyz: np.ndarray, obs_size,
                      scrape_threshold: float = 0.10) -> Dict:
    """Audit one UAV trajectory. Returns dict with counts + per-incident list."""
    n_obs = len(obstacle_xyz)
    sizes = coerce_sizes(obs_size, n_obs) if n_obs > 0 else np.zeros((0,))

    counts = {"clean": 0, "scrape": 0, "intersection": 0}
    incidents = []
    n_segments = max(0, len(samples) - 1)

    for k in range(n_segments):
        t_a, a = samples[k]
        t_b, b = samples[k + 1]
        seg_classification = "clean"
        worst_event = None
        for oi in range(n_obs):
            c = np.asarray(obstacle_xyz[oi], dtype=float)
            sz = sizes[oi]
            if np.isscalar(sz):
                h_arr = np.array([float(sz)] * 3)
                ref_half = float(sz)
            else:
                h_arr = np.asarray(sz, dtype=float)
                ref_half = float(h_arr.min())  # use min so a wide-but-thin cube
                                                 # still gets a sane threshold
            pmin = c - h_arr
            pmax = c + h_arr
            if not line_intersects_cube(a, b, pmin, pmax):
                continue
            depth = penetration_depth(a, b, c, h_arr, n_samples=20)
            depth_frac = depth / max(ref_half, 1e-9)
            kind = "scrape" if depth_frac < scrape_threshold else "intersection"
            event = {
                "obstacle_idx": oi,
                "t_start_s": float(t_a),
                "t_end_s": float(t_b),
                "depth_m": float(depth),
                "depth_frac_of_half_size": float(depth_frac),
                "kind": kind,
            }
            if (worst_event is None or
                    event["depth_frac_of_half_size"] > worst_event["depth_frac_of_half_size"]):
                worst_event = event
            if kind == "intersection":
                seg_classification = "intersection"
            elif seg_classification != "intersection":
                seg_classification = "scrape"
        counts[seg_classification] += 1
        if worst_event is not None:
            incidents.append(worst_event)

    return {
        "n_segments": n_segments,
        "n_samples": len(samples),
        "counts": counts,
        "incidents": incidents,
        "scrape_threshold_frac_of_half_size": scrape_threshold,
    }


def audit_bag(bag_path: Path, pickle_path: Path,
              pose_topics: Optional[List[str]] = None,
              scrape_threshold: float = 0.10,
              downsample_hz: float = 10.0) -> Dict:
    """Audit every UAV trajectory in the bag against the obstacles from the
    DECK_GA_QuickNav pickle. Returns aggregate dict."""
    with pickle_path.open("rb") as f:
        data = pickle.load(f)
    obstacle_xyz = np.asarray(data["obstacle_xyz"]) if len(data["obstacle_xyz"]) > 0 \
        else np.zeros((0, 3))
    obs_size = data["obs_size"]
    num_uavs = int(data["num_uavs"])

    if pose_topics is None:
        pose_topics = [f"/drone{i}/self_localization/pose" for i in range(num_uavs)]

    per_uav = []
    for i in range(num_uavs):
        topic = pose_topics[i] if i < len(pose_topics) else \
            f"/drone{i}/self_localization/pose"
        try:
            samples = read_bag_poses(bag_path, topic, downsample_hz=downsample_hz)
        except RuntimeError as e:
            per_uav.append({"uav": i, "topic": topic, "error": str(e),
                            "n_segments": 0, "n_samples": 0,
                            "counts": {"clean": 0, "scrape": 0, "intersection": 0},
                            "incidents": []})
            continue
        result = audit_trajectory(samples, obstacle_xyz, obs_size, scrape_threshold)
        result["uav"] = i
        result["topic"] = topic
        per_uav.append(result)

    totals = {"clean": 0, "scrape": 0, "intersection": 0}
    for u in per_uav:
        for k in totals:
            totals[k] += u["counts"][k]

    return {
        "scrape_threshold_frac_of_half_size": scrape_threshold,
        "totals": totals,
        "per_uav": per_uav,
        "verdict": "PASS" if totals["intersection"] == 0 else "FAIL",
    }


def render_markdown_summary(audit: Dict, trial_id: str = "?") -> str:
    lines = [f"=== COLLISION AUDIT: trial {trial_id} ==="]
    for u in audit["per_uav"]:
        c = u["counts"]
        if "error" in u:
            lines.append(f"UAV{u['uav']}: ERROR reading bag — {u['error']}")
            continue
        verdict = "PASS" if c["intersection"] == 0 else "FAIL"
        scrape_note = f" ({c['scrape']} scrapes within margin)" if c["scrape"] > 0 else ""
        lines.append(
            f"UAV{u['uav']}: {u['n_segments']} segments  "
            f"| clean={c['clean']}  scrape={c['scrape']}  intersection={c['intersection']}  "
            f"-> {verdict}{scrape_note}"
        )
        for inc in u["incidents"]:
            tag = inc["kind"]
            lines.append(
                f"     {tag}: obstacle {inc['obstacle_idx']}, "
                f"depth {inc['depth_m']:.3f} m "
                f"({inc['depth_frac_of_half_size'] * 100:.1f}% of half-size) "
                f"at t={inc['t_start_s']:.1f}s"
            )
    t = audit["totals"]
    lines.append("")
    lines.append(
        f"OVERALL: {audit['verdict']} "
        f"({t['intersection']} intersections, {t['scrape']} scrapes within "
        f"{int(audit['scrape_threshold_frac_of_half_size'] * 100)}% margin)"
    )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="Path to the rosbag2 bag dir")
    parser.add_argument("--pickle", required=True, help="DECK_GA_QuickNav output pickle")
    parser.add_argument("--out", required=True, help="Output JSON path")
    parser.add_argument("--trial-id", default="?")
    parser.add_argument("--scrape-threshold", type=float, default=0.10)
    parser.add_argument("--downsample-hz", type=float, default=10.0)
    parser.add_argument("--pose-topics", default=None,
                        help="Comma-separated pose topic names (one per UAV). "
                             "Default: /drone0/.../pose, /drone1/..., /drone2/...")
    args = parser.parse_args()

    pose_topics = None
    if args.pose_topics:
        pose_topics = [t.strip() for t in args.pose_topics.split(",")]

    audit = audit_bag(
        Path(args.bag).resolve(),
        Path(args.pickle).resolve(),
        pose_topics=pose_topics,
        scrape_threshold=args.scrape_threshold,
        downsample_hz=args.downsample_hz,
    )

    out = Path(args.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(audit, indent=2))

    print(render_markdown_summary(audit, trial_id=args.trial_id))
    print(f"\n[audit] wrote {out}")
    sys.exit(0 if audit["verdict"] == "PASS" else 3)


if __name__ == "__main__":
    main()
