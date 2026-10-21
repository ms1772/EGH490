#!/usr/bin/env python3
"""Executed path length and tracking deviation, computed FROM THE BAG.

WHY THIS EXISTS
---------------
`executed_uavN_dist_m` in the executor's summary CSV is NOT an executed
distance. `deckga_execute.py:355` computes each segment as

    seg_dist = norm(path[k] - last_xyz)

i.e. from the COMMANDED waypoints, never from measured pose. `cum_dist_m`
therefore accumulates planned polyline geometry, and `executed_uavN_dist_m` is
that sum. On `gz_n30_o20_s0` it equalled `planned_makespan_s` to fifteen
significant figures -- planned length and "executed" length are the same number
by construction, not by good tracking.

The consequence for Stage 0 is specific: the plan's RTF acceptance criterion
"executed distance per UAV within 2%" is VACUOUS against that column. It is
identical at every real-time factor no matter how badly tracking degrades,
because no measurement enters it. It would pass at RTF 100.

This module supplies the measurement that criterion needs. Poses come from the
bag -- the same source `collision_audit.py` already trusts for the residual
collision verdict -- so a degraded RTF shows up as a changed flown length and a
widened deviation from the planned polyline.

WINDOWING
---------
The planned polyline is WP1..WPn; the aircraft's start point is not in it, so
takeoff and the approach to WP1 are flown but unplanned, and their samples
deviate by tens of metres legitimately. Rather than guess a path-phase window,
every statistic here is computed over the WHOLE bag and compared only against
another run computed the same way. Identical treatment on both sides is what
makes an RTF-vs-RTF comparison valid; an absolute deviation figure from this
tool is not a tracking-error claim and must not be reported as one.

SAMPLE RATE IS THE WHOLE BALL GAME
----------------------------------
A naive full-rate integral of pose is not a path length, it is a noise
measurement. Measured on `gz_n30_o20_s0`: the pose stream runs at 82-97 Hz, so
at ~1 m/s true motion is ~1.2 cm per sample, which is *smaller* than the
estimator's own jitter. Integrating every sample gave 2732 m against a 558.7 m
planned tour -- 4.9x inflation, almost all of it noise. A 2 cm deadband made it
worse rather than better: it sits above the real per-sample step, so it
discarded genuine motion while still passing noise spikes.

The fix is to integrate at a rate where real displacement dominates jitter.
At 5 Hz each step is ~20 cm against a few cm of noise, and corner-cutting is
negligible because inter-waypoint legs are tens of metres. This is the same
reason `collision_audit.py` defaults to 10 Hz.

`bag_len_m` (at LENGTH_RATE_HZ) is the number to quote. `len_by_rate` carries
2 / 5 / 10 Hz and full rate so the sensitivity is visible and the choice is
auditable rather than asserted. Deviation is measured at full rate, where noise
adds a small symmetric spread to a distance-to-polyline statistic instead of
biasing it the way it biases a path integral.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# Rate the quoted path length is integrated at, chosen from the measured rate
# sweep rather than picked a priori. On gz_n30_o20_s0 (UAV2, planned 558.7 m):
#
#     2 Hz 662.8 | 5 Hz 676.6 | 10 Hz 718.3 | full (82 Hz) 2732.1
#
# The integral is stable at the low end and diverges at the high end, which is
# the signature of noise accumulation rather than recovered geometry: 2->5 Hz
# moves it 2.1%, 5->10 Hz 6.2%, 10 Hz->full 280%. 2 Hz is ~50 cm per step at
# 1 m/s, while inter-waypoint legs here average ~56 m, so chord loss is well
# under 1% even with a direction change at every waypoint.
LENGTH_RATE_HZ = 2.0

# Chunk size for the point-to-polyline reduction. Bounds peak memory at
# chunk * n_segments * 3 floats; Gazebo is already holding most of WSL's 7.8 GB
# when this runs inside a trial window.
CHUNK = 2000

# `read_bag_poses` has no "off" switch for decimation: it computes
# `min_dt = 1.0 / max(0.1, downsample_hz)` unconditionally, so None raises and
# its 10 Hz default would throw away most of a ~50 Hz pose stream. A huge rate
# gives min_dt ~ 1 us, which keeps every sample.
NO_DOWNSAMPLE_HZ = 1.0e6

# Reported alongside the quoted value so the rate sensitivity is on the record.
# 1 Hz is included as the lower bracket: if the quoted 2 Hz figure were losing
# real geometry to chord-cutting, 1 Hz would drop away from it noticeably.
DIAGNOSTIC_RATES = (1.0, 2.0, 5.0, 10.0, NO_DOWNSAMPLE_HZ)

# The key the trials were actually FLOWN with. `load_deckga_pkl` defaults to
# "deckga_paths", but `sweep_gazebo.py` invokes the executor with
# `--path_key quicknav_paths`, so defaulting here would measure deviation
# against the straight-line DECK-GA tour the aircraft never flew -- and would
# report a large bogus deviation at every RTF.
FLOWN_PATH_KEY = "quicknav_paths"


def _load_planned(pickle_path: Path, num_uavs: int) -> List[np.ndarray]:
    """Planned paths in the SAME frame the executor commanded them in.

    Imports the executor's own `load_paths`/`transform_paths` rather than
    reimplementing the transform: the shift-detection in `_auto_is_shifted` and
    the scale/z-offset/clamp chain are easy to reproduce slightly wrong, and a
    frame mismatch would make every deviation number below meaningless while
    still looking plausible.
    """
    from deckga_ros2.deckga_execute import (  # noqa: PLC0415
        DEFAULT_SCALE_XY, DEFAULT_SCALE_Z, DEFAULT_Z_MIN, DEFAULT_Z_OFFSET,
        TransformConfig, load_deckga_pkl, transform_paths,
    )
    raw, offset_used = load_deckga_pkl(str(pickle_path), path_key=FLOWN_PATH_KEY)
    tf = TransformConfig(
        coord_mode="original",
        scale_xy=DEFAULT_SCALE_XY,
        scale_z=DEFAULT_SCALE_Z,
        z_offset=DEFAULT_Z_OFFSET,
        z_min=DEFAULT_Z_MIN,
    )
    paths = transform_paths(raw, offset_used, tf)
    return [np.asarray(p, dtype=float).reshape(-1, 3) for p in paths[:num_uavs]]


def _polyline_deviation(samples: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Per-sample distance to the nearest point on the planned polyline."""
    if poly.shape[0] < 2 or samples.shape[0] == 0:
        return np.zeros(samples.shape[0], dtype=float)
    a = poly[:-1]                       # (S,3) segment starts
    b = poly[1:]                        # (S,3) segment ends
    ab = b - a
    denom = np.einsum("ij,ij->i", ab, ab)
    denom = np.where(denom < 1e-12, 1.0, denom)

    out = np.empty(samples.shape[0], dtype=float)
    for lo in range(0, samples.shape[0], CHUNK):
        p = samples[lo:lo + CHUNK]                       # (C,3)
        ap = p[:, None, :] - a[None, :, :]               # (C,S,3)
        t = np.einsum("csj,sj->cs", ap, ab) / denom      # (C,S)
        t = np.clip(t, 0.0, 1.0)
        closest = a[None, :, :] + t[:, :, None] * ab[None, :, :]
        d = np.linalg.norm(p[:, None, :] - closest, axis=2)
        out[lo:lo + CHUNK] = d.min(axis=1)
    return out


def _decimate(ts: np.ndarray, xyz: np.ndarray, rate_hz: float) -> np.ndarray:
    """Keep the first sample in each 1/rate window. Matches read_bag_poses."""
    if rate_hz >= NO_DOWNSAMPLE_HZ:
        return xyz
    min_dt = 1.0 / max(0.1, rate_hz)
    keep = []
    last = -1.0e18
    for i, t in enumerate(ts):
        if t - last >= min_dt:
            keep.append(i)
            last = t
    return xyz[keep]


def _length(xyz: np.ndarray) -> float:
    if xyz.shape[0] < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())


def uav_stats(bag_path: Path, topic: str, planned: Optional[np.ndarray],
              length_rate_hz: float = LENGTH_RATE_HZ) -> Dict:
    from tests.automation.collision_audit import read_bag_poses  # noqa: PLC0415

    # Read once at full rate, then decimate in memory -- re-reading the bag per
    # rate would quadruple the I/O for identical data.
    samples = read_bag_poses(bag_path, topic, downsample_hz=NO_DOWNSAMPLE_HZ)
    if not samples:
        return {"topic": topic, "error": "no samples", "n_samples": 0}

    ts = np.asarray([t for t, _ in samples], dtype=float)
    xyz = np.asarray([p for _, p in samples], dtype=float).reshape(-1, 3)

    rates = sorted(set(DIAGNOSTIC_RATES) | {float(length_rate_hz)})
    len_by_rate = {}
    for r in rates:
        key = "full" if r >= NO_DOWNSAMPLE_HZ else f"{r:g}hz"
        len_by_rate[key] = _length(_decimate(ts, xyz, r))

    out = {
        "topic": topic,
        "n_samples": int(xyz.shape[0]),
        "duration_s": float(ts[-1] - ts[0]) if ts.size > 1 else 0.0,
        "sample_rate_hz": (float(xyz.shape[0]) / float(ts[-1] - ts[0])
                           if ts.size > 1 and ts[-1] > ts[0] else 0.0),
        "length_rate_hz": float(length_rate_hz),
        "bag_len_m": float(len_by_rate[f"{float(length_rate_hz):g}hz"]),
        "bag_len_full_rate_m": float(len_by_rate["full"]),
        "len_by_rate": len_by_rate,
    }
    if planned is not None and planned.shape[0] >= 2:
        dev = _polyline_deviation(xyz, planned)
        seg = np.linalg.norm(np.diff(planned, axis=0), axis=1)
        out.update({
            "planned_len_m": float(seg.sum()),
            "dev_mean_m": float(dev.mean()),
            "dev_p95_m": float(np.percentile(dev, 95)),
            "dev_max_m": float(dev.max()),
        })
    return out


def analyse(bag_path: Path, pickle_path: Optional[Path], num_uavs: int = 3,
            pose_topics: Optional[Sequence[str]] = None,
            length_rate_hz: float = LENGTH_RATE_HZ) -> Dict:
    if pose_topics is None:
        pose_topics = [f"/drone{i}/self_localization/pose" for i in range(num_uavs)]
    planned: List[Optional[np.ndarray]] = [None] * num_uavs
    if pickle_path is not None and pickle_path.exists():
        try:
            got = _load_planned(pickle_path, num_uavs)
            for i, p in enumerate(got):
                planned[i] = p
        except Exception as e:                                   # noqa: BLE001
            print(f"[bag_distance] planned paths unavailable ({e}); "
                  f"length only, no deviation")

    per_uav = [uav_stats(bag_path, pose_topics[i], planned[i], length_rate_hz)
               for i in range(num_uavs)]
    ok = [u for u in per_uav if "bag_len_m" in u]
    return {
        "bag_path": str(bag_path),
        "num_uavs": num_uavs,
        "length_rate_hz": float(length_rate_hz),
        "per_uav": per_uav,
        "bag_len_total_m": float(sum(u["bag_len_m"] for u in ok)),
        "bag_len_max_m": float(max((u["bag_len_m"] for u in ok), default=0.0)),
        "dev_max_m": float(max((u.get("dev_max_m", 0.0) for u in ok), default=0.0)),
        "dev_p95_max_m": float(max((u.get("dev_p95_m", 0.0) for u in ok), default=0.0)),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--pickle", default=None,
                    help="deckga_quicknav_output.pkl, for planned-path deviation")
    ap.add_argument("--num-uavs", type=int, default=3)
    ap.add_argument("--length-rate", type=float, default=LENGTH_RATE_HZ,
                    help="Hz the quoted length is integrated at (default %(default)s)")
    ap.add_argument("--out", default=None, help="write JSON here")
    args = ap.parse_args()

    res = analyse(Path(args.bag),
                  Path(args.pickle) if args.pickle else None,
                  num_uavs=args.num_uavs, length_rate_hz=args.length_rate)
    for u in res["per_uav"]:
        if "error" in u:
            print(f"  {u['topic']}: ERROR {u['error']}")
            continue
        rates = "  ".join(f"{k}={v:.1f}" for k, v in u["len_by_rate"].items())
        line = (f"  {u['topic']}: flown={u['bag_len_m']:.2f} m "
                f"@{u['length_rate_hz']:g}Hz  n={u['n_samples']} "
                f"({u['sample_rate_hz']:.1f} Hz)  T={u['duration_s']:.1f} s")
        if "planned_len_m" in u:
            line += (f"\n      planned={u['planned_len_m']:.2f} m  "
                     f"ratio={u['bag_len_m'] / max(u['planned_len_m'], 1e-9):.3f}  "
                     f"dev mean={u['dev_mean_m']:.3f} p95={u['dev_p95_m']:.3f} "
                     f"max={u['dev_max_m']:.3f} m")
        line += f"\n      len by rate: {rates}"
        print(line)
    print(f"TOTAL flown {res['bag_len_total_m']:.2f} m   "
          f"max per-UAV {res['bag_len_max_m']:.2f} m")
    if args.out:
        Path(args.out).write_text(json.dumps(res, indent=2), encoding="utf-8")
        print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
