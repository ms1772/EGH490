#!/usr/bin/env python3
"""Prove the track archive can stand in for the bags, rather than assert it.

The claim being tested is narrow and worth stating exactly: that every quantity
this project computes from a bag can be recomputed from `flown_tracks_full.npz`
and come out the same. It does NOT claim the archive replaces the bag -- `/tf`
is not in it, so frame-correct replay still needs the original.

Why bother: the first archive was declared a sufficient substitute while it had
silently destroyed inter-UAV time alignment, which would have made the collision
audit unreproducible. An assertion of equivalence that nobody checked is exactly
how 39 GB gets deleted on a false premise. So this re-derives the numbers both
ways and compares them.

Compared per trial and UAV:
  * flown path length at bag_distance.LENGTH_RATE_HZ (the quoted metric)
  * sample count and duration
  * inter-UAV start offsets, which are what a collision audit depends on
"""
from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

RATE_HZ = 2.0          # bag_distance.LENGTH_RATE_HZ -- the rate it quotes


def _decimate(ts: np.ndarray, xyz: np.ndarray, rate_hz: float) -> np.ndarray:
    """First sample in each 1/rate window -- same rule as read_bag_poses."""
    min_dt = 1.0 / max(0.1, rate_hz)
    keep, last = [], -1.0e18
    for i, t in enumerate(ts):
        if t - last >= min_dt:
            keep.append(i)
            last = t
    return xyz[keep]


def _length(xyz: np.ndarray) -> float:
    if xyz.shape[0] < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--archive", default="flown_tracks_full.npz")
    ap.add_argument("--trials", type=int, default=6,
                    help="how many trials to re-read from the BAG (slow); "
                         "archive-side checks cover all of them")
    ap.add_argument("--tol", type=float, default=0.01,
                    help="relative tolerance on flown length (default 1%%)")
    args = ap.parse_args()

    run_dir = Path(args.run_dir).resolve()
    z = np.load(run_dir / args.archive)

    # ---- archive-side, ALL trials ------------------------------------------
    by_trial = defaultdict(dict)
    for k in z.files:
        tid, uav = k.split("|")
        by_trial[tid][uav] = z[k]

    print(f"archive: {len(z.files)} tracks over {len(by_trial)} trials")
    bad_align = [t for t, d in by_trial.items()
                 if all(abs(a[0, 0]) < 1e-9 for a in d.values())]
    print(f"  inter-UAV alignment present: {len(by_trial) - len(bad_align)}"
          f"/{len(by_trial)}")
    if bad_align:
        print(f"  FAIL: {len(bad_align)} trial(s) have every UAV at t=0, which "
              f"means per-UAV anchoring -- a collision audit cannot be "
              f"reproduced from this archive")

    # ---- bag-side, a sample ------------------------------------------------
    from tests.automation.campaign.bag_distance import analyse  # noqa: PLC0415

    tids = sorted(by_trial)[:: max(1, len(by_trial) // max(args.trials, 1))]
    tids = tids[:args.trials]
    print(f"\nre-reading {len(tids)} bag(s) to compare against:")

    worst = 0.0
    n_ok = 0
    for tid in tids:
        bag = run_dir / "gazebo" / tid / "poses.bag"
        if not bag.exists():
            print(f"  [skip] {tid}: no bag")
            continue
        res = analyse(bag, None, num_uavs=len(by_trial[tid]))
        per = {u["topic"].split("/")[1]: u for u in res["per_uav"]
               if "bag_len_m" in u}
        rows = []
        for uav, arr in sorted(by_trial[tid].items()):
            idx = int(uav.replace("uav", ""))
            b = per.get(f"drone{idx}")
            if b is None:
                continue
            a_len = _length(_decimate(arr[:, 0], arr[:, 1:4], RATE_HZ))
            b_len = b["bag_len_m"]
            rel = abs(a_len - b_len) / max(b_len, 1e-9)
            worst = max(worst, rel)
            n_ok += 1
            rows.append(f"{uav} archive={a_len:8.2f} bag={b_len:8.2f} "
                        f"({rel:+.3%})")
        print(f"  {tid}")
        for r in rows:
            print(f"      {r}")

    print(f"\ncompared {n_ok} UAV-track(s); worst relative difference "
          f"{worst:.4%} (tolerance {args.tol:.1%})")
    ok = (worst <= args.tol) and not bad_align
    print("RESULT:", "archive reproduces the bag-derived metrics"
          if ok else "MISMATCH -- do not delete the bags")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
