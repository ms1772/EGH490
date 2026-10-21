#!/usr/bin/env python3
"""Distil every trial's flown trajectory into one compact archive.

WHY
---
The campaign's bags total ~39 GB (80 trials, 370-630 MB each) on a disk with
~29 GB free. Almost all of that is `/tf`, which was added to the record list so
a trial could be replayed frame-correct in RViz later; the *analysis* only ever
reads the three `self_localization/pose` topics.

That makes the bags simultaneously the single largest artefact and -- for every
use we actually have -- almost entirely redundant. But they cannot simply be
deleted: they are the only record of what the aircraft really did, and
re-creating them means re-flying 23 hours of simulation, which is precisely
what this campaign exists to avoid ever needing again.

So this extracts the part that matters. This script never deletes anything.

WHAT THE ARCHIVE IS AND IS NOT
------------------------------
Be honest about this before trusting it instead of the bags. At the default
settings the archive keeps POSITION ONLY, decimated, and therefore CANNOT
reproduce:

  * anything needing orientation -- the quaternion is dropped unless
    `--with-orientation` is passed;
  * anything above the stored rate (the bags carry 82-97 Hz);
  * `/tf` / `/tf_static`, so no frame-correct RViz replay;
  * the collision audit, if per-UAV time bases are not shared (see below).

An early version subtracted each UAV's OWN first timestamp, which quietly broke
the last of those: three tracks each starting at zero cannot say where the
aircraft were at the same instant, and inter-UAV simultaneity is exactly what a
collision audit needs. Every UAV in a trial is now anchored to one epoch,
recorded as `t0_epoch_s`.

Run with `--with-orientation --rate 100` for an archive that is a genuine
substitute for the pose topics (~0.5 GB for 80 trials, against 39 GB of bags).
Even then it is not a substitute for the BAG, because `/tf` is not in it.

FORMAT
------
One `.npz` per run, keys `<trial_id>|uav<i>`, each an (N,4) array of
[t, x, y, z] -- or (N,8) with [t, x, y, z, qx, qy, qz, qw] when orientation is
kept. `t` is seconds since that trial's shared epoch, so UAVs are directly
comparable. float64, because the cost is trivial and float32 quietly loses
precision on absolute timestamps. A JSON sidecar records the rate, the column
layout, the epoch and the source bag sizes, so a later reader can tell what it
is looking at without the bags.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

STORE_RATE_HZ = 10.0


def _dir_size(p: Path) -> int:
    try:
        return sum(f.stat().st_size for f in p.rglob("*") if f.is_file())
    except OSError:
        return 0


def read_bag_multi(bag_path: Path, topics, downsample_hz: float,
                   with_orientation: bool = False) -> Dict[str, list]:
    """All requested topics in ONE pass over the bag.

    `collision_audit.read_bag_poses` opens and scans the bag per topic, which
    is fine for a 30 MB bag but not here: these are 370-630 MB and live on
    /mnt/c, where WSL's 9p filesystem makes every scan expensive. Three scans
    per trial across 80 trials was going to take hours.

    The storage filter is still applied, so /tf -- which is the bulk of the
    file and the reason it is this large -- is never deserialised.
    """
    from rclpy.serialization import deserialize_message        # noqa: PLC0415
    from rosbag2_py import (ConverterOptions, SequentialReader,  # noqa: PLC0415
                            StorageFilter, StorageOptions)
    from rosidl_runtime_py.utilities import get_message        # noqa: PLC0415

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
                ConverterOptions(input_serialization_format="cdr",
                                 output_serialization_format="cdr"))
    type_map = {ti.name: ti.type for ti in reader.get_all_topics_and_types()}
    present = [t for t in topics if t in type_map]
    if not present:
        return {}
    msg_types = {t: get_message(type_map[t]) for t in present}
    reader.set_filter(StorageFilter(topics=present))

    min_dt = 1.0 / max(0.1, downsample_hz)
    last = {t: -1e18 for t in present}
    out: Dict[str, list] = {t: [] for t in present}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t = t_ns / 1e9
        if t - last[topic] < min_dt:
            continue
        last[topic] = t
        m = deserialize_message(raw, msg_types[topic])
        p = m.pose.position
        if with_orientation:
            q = m.pose.orientation
            out[topic].append((t, (p.x, p.y, p.z, q.x, q.y, q.z, q.w)))
        else:
            out[topic].append((t, (p.x, p.y, p.z)))
    return out


def extract(run_dir: Path, num_uavs: int = 3,
            rate_hz: float = STORE_RATE_HZ,
            with_orientation: bool = False,
            out_name: str = "flown_tracks.npz",
            part_name: str = "flown_tracks_parts") -> Path:
    gz_dir = run_dir / "gazebo"
    bags = sorted(gz_dir.glob("*/poses.bag"))
    if not bags:
        raise SystemExit(f"no bags under {gz_dir}")

    # PER-TRIAL PERSISTENCE, not one write at the end.
    #
    # The first version of this accumulated every track in memory and saved a
    # single archive after the last bag. It was killed by a signal at bag 24 of
    # 80 -- no traceback, just gone -- and took 23 bags of work with it. Reading
    # 39 GB takes long enough that "lose everything on death" is not an
    # acceptable failure mode, and it is the same lesson already applied to the
    # sweep, which persists each trial's row as it completes.
    #
    # Each trial now lands in its own small .npz the moment it is read, so a
    # death costs one bag, and a re-run skips what is already on disk.
    part_dir = run_dir / part_name
    part_dir.mkdir(parents=True, exist_ok=True)

    manifest = {
        "rate_hz": rate_hz,
        "num_uavs": num_uavs,
        "with_orientation": with_orientation,
        "columns": ("t,x,y,z,qx,qy,qz,qw" if with_orientation else "t,x,y,z"),
        "time_base": "seconds since the trial epoch t0_epoch_s "
                     "(shared by all UAVs in the trial)",
        "source_bag_bytes": 0,
        "trials": {},
    }

    t0 = time.perf_counter()
    skipped = 0
    for i, bag in enumerate(bags, 1):
        tid = bag.parent.name
        size = _dir_size(bag)
        manifest["source_bag_bytes"] += size

        part = part_dir / f"{tid}.npz"
        if part.exists():
            skipped += 1
            try:
                with np.load(part) as z:
                    manifest["trials"][tid] = {
                        "bag_bytes": size,
                        "uavs": {k: {"n": int(z[k].shape[0])} for k in z.files},
                    }
            except Exception:                                 # noqa: BLE001
                part.unlink(missing_ok=True)
            else:
                continue
        topics = [f"/drone{u}/self_localization/pose" for u in range(num_uavs)]
        per_uav = {}
        t_bag = time.perf_counter()
        try:
            got = read_bag_multi(bag, topics, rate_hz,
                                 with_orientation=with_orientation)
        except Exception as e:                                # noqa: BLE001
            manifest["trials"][tid] = {"bag_bytes": size,
                                       "error": str(e)[:160]}
            print(f"  [{i}/{len(bags)}] {tid}  FAILED: {type(e).__name__}: {e}",
                  flush=True)
            continue
        # One epoch for the whole trial: the earliest sample across all UAVs.
        trial_t0 = min((s[0][0] for s in (got.get(t) or [[None]] for t in topics)
                        if s and s[0] is not None and s[0][0] is not None),
                       default=0.0)
        manifest["trials"].setdefault(tid, {})["t0_epoch_s"] = float(trial_t0)

        this_trial: Dict[str, np.ndarray] = {}
        for u, topic in enumerate(topics):
            samples = got.get(topic) or []
            if not samples:
                per_uav[f"uav{u}"] = {"n": 0}
                continue
            ts = np.asarray([t for t, _ in samples], dtype=float)
            cols = len(samples[0][1])
            vals = np.asarray([p for _, p in samples], dtype=float).reshape(-1, cols)
            # TIME IS RELATIVE TO THE TRIAL, NOT TO EACH UAV.
            #
            # The first version subtracted each UAV's own first timestamp, which
            # silently destroyed inter-UAV alignment: three tracks each starting
            # at zero cannot tell you where the aircraft were AT THE SAME MOMENT,
            # so the collision audit -- the one analysis that needs all three
            # simultaneously -- could not be reproduced from the archive at all.
            # Anchoring every UAV to one per-trial epoch keeps them comparable,
            # and `t0_epoch_s` below preserves the absolute clock.
            ts = ts - trial_t0
            this_trial[f"uav{u}"] = np.column_stack([ts, vals]).astype(np.float64)
            per_uav[f"uav{u}"] = {"n": int(ts.size),
                                  "t_start_s": round(float(ts[0]), 3),
                                  "duration_s": round(float(ts[-1] - ts[0]), 2)}
        # Written immediately, through a temp file so a kill mid-write cannot
        # leave a truncated .npz that a later run would trust and skip.
        tmp = part.with_suffix(".npz.tmp")
        with tmp.open("wb") as fh:
            np.savez_compressed(fh, **this_trial)
        tmp.replace(part)
        del got, this_trial
        # MERGE, never replace. A bare assignment here silently discarded the
        # `t0_epoch_s` written a few lines above, because it rebuilds the dict
        # from scratch -- the archive kept correct RELATIVE alignment (the
        # arrays are already anchored) but lost the absolute epoch for all 80
        # trials, with nothing to indicate it had happened.
        manifest["trials"].setdefault(tid, {}).update(
            {"bag_bytes": size, "uavs": per_uav})
        print(f"  [{i}/{len(bags)}] {tid}  {size/2**20:.0f} MB -> "
              f"{sum(v.get('n', 0) for v in per_uav.values())} samples "
              f"in {time.perf_counter() - t_bag:.0f}s", flush=True)

    # Merge the per-trial parts into one archive. Done at the end from files on
    # disk, so it costs no memory during the long read and can be re-run alone.
    arrays: Dict[str, np.ndarray] = {}
    for part in sorted(part_dir.glob("*.npz")):
        tid = part.stem
        try:
            with np.load(part) as z:
                for k in z.files:
                    arrays[f"{tid}|{k}"] = z[k]
        except Exception as e:                                # noqa: BLE001
            print(f"  [warn] unreadable part {part.name}: {e}")

    out = run_dir / out_name
    np.savez_compressed(out, **arrays)
    (run_dir / (Path(out_name).stem + "_manifest.json")).write_text(
        json.dumps(manifest, indent=2), encoding="utf-8")

    src = manifest["source_bag_bytes"]
    dst = out.stat().st_size
    print(f"\nwrote {out}")
    print(f"  {len(arrays)} track(s) from {len(bags)} trial(s) "
          f"({skipped} already on disk) in {time.perf_counter() - t0:.0f}s")
    print(f"  {src/2**30:.1f} GB of bags -> {dst/2**20:.1f} MB archive "
          f"({src/max(dst,1):.0f}x smaller)")
    print("  bags are now OPTIONAL for analysis; nothing has been deleted.")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--num-uavs", type=int, default=3)
    ap.add_argument("--rate", type=float, default=STORE_RATE_HZ)
    ap.add_argument("--with-orientation", action="store_true",
                    help="also store the pose quaternion")
    ap.add_argument("--out-name", default="flown_tracks.npz")
    ap.add_argument("--part-name", default="flown_tracks_parts")
    args = ap.parse_args()
    extract(Path(args.run_dir).resolve(), num_uavs=args.num_uavs,
            rate_hz=args.rate, with_orientation=args.with_orientation,
            out_name=args.out_name, part_name=args.part_name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
