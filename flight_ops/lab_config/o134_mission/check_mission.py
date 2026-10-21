#!/usr/bin/env python3
"""Offline geometry gate for the O-134 mission. Nothing flies until this passes.

It loads the produced plan THROUGH deckga_execute.py's own loader and transform,
so what is checked here is byte-for-byte what the aircraft would be commanded to
fly -- not a re-derivation that could drift from it.

  python3 check_mission.py [--pkl o134_mission.pkl] [--path_key both] [-v]

Exit 0 = every check passed. Exit 1 = do not fly.

WHAT IT IS ACTUALLY LOOKING FOR
  1. Envelope. Every vertex of every path, INCLUDING the QuickNav detour
     vertices, inside the usable box. Detour vertices are corners of the
     obstacle AABB and nothing in QuickNav knows the room exists, so a badly
     placed obstacle puts a commanded waypoint outside the net or into the
     floor. This is the check that matters most.
  2. Spacing. Waypoints closer than MIN_SPACING_M are not distinguishable in
     flight and, with --ensure_reach, invite a go_to that confirms instantly or
     not at all.
  3. Clearance. quicknav_paths must clear the PHYSICAL props by a real margin,
     and deckga_paths must NOT -- if the baseline does not collide there is no
     comparison to publish.
  4. Endings. quicknav_paths are not closed tours (apply_quicknav's final
     deduplicate_preserve_order drops the point ensure_closed_tour added), so
     each avoided flight lands at its last waypoint, not at the pad. The crew
     needs to know where, and that the floor there is clear.
  5. Takeoff columns. The pad sits under each path's start point; the vertical
     column from the floor to cruise must not intersect an obstacle.
"""
from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]

MIN_SPACING_M = 0.80        # between commanded waypoints
MIN_ANY_STEP_M = 0.25       # any commanded hop shorter than this is a red flag
ENVELOPE_MARGIN_M = 0.00    # the envelope is already an inset of the measured room
MIN_PROP_CLEARANCE_M = 0.30 # quicknav paths vs the physical props


def _load_executor():
    """Import deckga_execute.py by path, so its loader/transform are the ones used."""
    path = REPO / "deckga_ros2" / "deckga_execute.py"
    spec = importlib.util.spec_from_file_location("deckga_execute_for_check", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def seg_box_distance(p, q, lo, hi, samples=400):
    """Min distance from segment p->q to an AABB. Sampled, which is fine here:
    at 400 samples a 6 m segment is resolved to 15 mm, far below any margin we
    would act on, and it cannot report a clearance that is larger than the truth
    by more than that."""
    t = np.linspace(0.0, 1.0, samples)[:, None]
    pts = p[None, :] + t * (q - p)[None, :]
    d = np.maximum(np.maximum(lo[None, :] - pts, pts - hi[None, :]), 0.0)
    return float(np.linalg.norm(d, axis=1).min())


def path_box_distance(path, lo, hi):
    return min(seg_box_distance(np.asarray(path[i], float),
                                np.asarray(path[i + 1], float), lo, hi)
               for i in range(len(path) - 1))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pkl", default=str(HERE / "o134_mission.pkl"))
    ap.add_argument("--speed", type=float, default=0.4,
                    help="only used to report an estimated flight time")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    sys.path.insert(0, str(REPO))
    sys.path.insert(0, str(HERE))
    import mission_spec as spec

    dx = _load_executor()

    pkl = Path(args.pkl)
    if not pkl.exists():
        print(f"FAIL  no plan at {pkl} -- run make_mission.sh first")
        return 1

    (ex0, ex1), (ey0, ey1), (ez0, ez1) = spec.ENVELOPE
    m = ENVELOPE_MARGIN_M
    env_lo = np.array([ex0 + m, ey0 + m, ez0 + m])
    env_hi = np.array([ex1 - m, ey1 - m, ez1 - m])

    # Model boxes (what the planner avoided) and physical boxes (what is
    # actually in the room, and what a collision would be against).
    model_lo, model_hi, prop_lo, prop_hi, names = [], [], [], [], []
    for o in spec.OBSTACLES:
        c = np.asarray(o["center"], float)
        h = np.asarray(o["half"], float)
        model_lo.append(c - h)
        model_hi.append(c + h)
        ph = np.asarray(o["prop_half_xy"], float)
        z0, z1 = o["prop_z"]
        prop_lo.append(np.array([c[0] - ph[0], c[1] - ph[1], spec.FLOOR_Z + z0]))
        prop_hi.append(np.array([c[0] + ph[0], c[1] + ph[1], spec.FLOOR_Z + z1]))
        names.append(o["name"])

    fails, warns = [], []

    def fail(msg):
        fails.append(msg)
        print(f"  FAIL  {msg}")

    def warn(msg):
        warns.append(msg)
        print(f"  warn  {msg}")

    print(f"plan      {pkl}")
    print(f"envelope  x {env_lo[0]:+.2f}..{env_hi[0]:+.2f}  "
          f"y {env_lo[1]:+.2f}..{env_hi[1]:+.2f}  z {env_lo[2]:.2f}..{env_hi[2]:.2f}"
          f"   (floor reads {spec.FLOOR_Z:+.2f})")
    for n, lo, hi, plo, phi in zip(names, model_lo, model_hi, prop_lo, prop_hi):
        print(f"obstacle  {n:9s} model x {lo[0]:+.2f}..{hi[0]:+.2f} "
              f"y {lo[1]:+.2f}..{hi[1]:+.2f} z {lo[2]:.2f}..{hi[2]:.2f}  |  "
              f"prop x {plo[0]:+.2f}..{phi[0]:+.2f} y {plo[1]:+.2f}..{phi[1]:+.2f} "
              f"z {plo[2]:.2f}..{phi[2]:.2f}")

    tf = dx.TransformConfig(coord_mode="original", scale_xy=1.0, scale_z=1.0,
                            z_offset=0.0, z_min=0.0)

    loaded = {}
    for key in ("deckga_paths", "quicknav_paths"):
        raw, offset_used = dx.load_deckga_pkl(str(pkl), path_key=key)
        loaded[key] = dx.transform_paths(raw, offset_used, tf)

    n_paths = len(loaded["quicknav_paths"])
    if len(loaded["deckga_paths"]) != n_paths:
        fail(f"deckga_paths has {len(loaded['deckga_paths'])} paths but "
             f"quicknav_paths has {n_paths}")

    # ---- per path -------------------------------------------------------
    for i in range(n_paths):
        qn = np.asarray(loaded["quicknav_paths"][i], float)
        bl = np.asarray(loaded["deckga_paths"][i], float)
        print(f"\n--- path {i} --- baseline {len(bl)} pts, avoided {len(qn)} pts")

        # 1. envelope, every vertex of the path that will actually be flown
        for j, p in enumerate(qn):
            if np.any(p < env_lo) or np.any(p > env_hi):
                fail(f"path {i} quicknav vertex {j} {np.round(p, 3).tolist()} "
                     f"is OUTSIDE the envelope")
        for j, p in enumerate(bl):
            if np.any(p < env_lo) or np.any(p > env_hi):
                fail(f"path {i} baseline vertex {j} {np.round(p, 3).tolist()} "
                     f"is OUTSIDE the envelope")

        # which vertices QuickNav added
        detours = [p for p in qn
                   if not any(np.allclose(p, q, atol=1e-6) for q in bl)]
        print(f"  detour vertices : {len(detours)}")
        if args.verbose:
            for p in detours:
                print(f"      {np.round(p, 3).tolist()}  "
                      f"(floor clearance {p[2] - spec.FLOOR_Z:+.2f} m)")
        if not detours:
            fail(f"path {i} has no detour vertices -- QuickNav changed nothing, "
                 f"so this path demonstrates no avoidance")

        # 2. spacing
        steps = np.linalg.norm(np.diff(qn, axis=0), axis=1)
        if len(steps):
            print(f"  step length     : {steps.min():.2f} .. {steps.max():.2f} m "
                  f"(total {steps.sum():.2f} m, ~{steps.sum() / args.speed:.0f} s "
                  f"at {args.speed:.1f} m/s)")
            if steps.min() < MIN_ANY_STEP_M:
                fail(f"path {i} has a commanded hop of {steps.min():.3f} m "
                     f"(< {MIN_ANY_STEP_M} m)")
            elif steps.min() < MIN_SPACING_M:
                warn(f"path {i} shortest hop is {steps.min():.2f} m "
                     f"(< {MIN_SPACING_M} m); expected between a waypoint and an "
                     f"adjacent detour corner, check it is not two waypoints")
        bsteps = np.linalg.norm(np.diff(bl, axis=0), axis=1)
        if len(bsteps) and bsteps.min() < MIN_SPACING_M:
            fail(f"path {i} baseline has two waypoints {bsteps.min():.2f} m apart "
                 f"(< {MIN_SPACING_M} m)")

        # 3. clearance -- avoided must clear the props, baseline must not.
        #    "Hit" is judged against the prop grown by the airframe half-width:
        #    a path centre 0.03 m from the pillar is a strike, because the
        #    aircraft is 0.30 m wide. Comparing the bare prop to the path
        #    centreline would call that a miss.
        hit_any = False
        for n, lo, hi, plo, phi in zip(names, model_lo, model_hi, prop_lo, prop_hi):
            d_model = path_box_distance(qn, lo, hi)
            d_prop = path_box_distance(qn, plo, phi)
            d_base = path_box_distance(bl, plo, phi)
            would_strike = d_base <= spec.DRONE_HALF_XY
            hit_any = hit_any or would_strike
            print(f"  vs {n:9s}  avoided: {d_prop:.2f} m from the prop "
                  f"({d_model:.3f} m from the model box) | "
                  f"baseline: {d_base:.2f} m "
                  f"-> {'WOULD STRIKE' if would_strike else 'clear'}")
            if d_prop < MIN_PROP_CLEARANCE_M:
                fail(f"path {i} avoided route passes {d_prop:.2f} m from {n} "
                     f"(< {MIN_PROP_CLEARANCE_M} m)")
        if not hit_any:
            fail(f"path {i} baseline strikes nothing -- this path has no "
                 f"collision to contrast against the avoided run")

        # The headline length comparison is a trap. apply_quicknav ends with
        # deduplicate_preserve_order, which drops the closing point that
        # ensure_closed_tour added, so quicknav_paths are OPEN and deckga_paths
        # are CLOSED. Comparing them as printed makes avoidance look like it
        # SHORTENED the route. Compare open against open.
        len_closed = float(np.sum(np.linalg.norm(np.diff(bl, axis=0), axis=1)))
        bl_open = bl[:-1] if np.allclose(bl[0], bl[-1]) else bl
        len_open = float(np.sum(np.linalg.norm(np.diff(bl_open, axis=0), axis=1)))
        len_qn = float(np.sum(np.linalg.norm(np.diff(qn, axis=0), axis=1)))
        print(f"  length          : baseline {len_open:.2f} m open "
              f"({len_closed:.2f} m closed) vs avoided {len_qn:.2f} m open "
              f"-> avoidance costs {len_qn - len_open:+.2f} m "
              f"({100.0 * (len_qn - len_open) / len_open:+.1f}%)")
        if len_qn < len_open:
            warn(f"path {i} avoided route is SHORTER than the open baseline "
                 f"-- check before putting this in a figure")

        # 4. where it ends
        closed = bool(np.allclose(qn[0], qn[-1]))
        print(f"  lands at        : x {qn[-1][0]:+.2f}  y {qn[-1][1]:+.2f}  "
              f"z {qn[-1][2]:.2f}  ({qn[-1][2] - spec.FLOOR_Z:.2f} m above floor)"
              f"{'  [closed tour]' if closed else '  [NOT back at the pad]'}")
        for n, plo, phi in zip(names, prop_lo, prop_hi):
            d = seg_box_distance(qn[-1], np.array([qn[-1][0], qn[-1][1], spec.FLOOR_Z]),
                                 plo, phi)
            if d < MIN_PROP_CLEARANCE_M:
                fail(f"path {i} descends to land {d:.2f} m from {n}")

    # ---- takeoff columns -------------------------------------------------
    print("\n--- takeoff columns (pad sits under each path's start point) ---")
    points, centers, halves, starts, clusters, _env = spec.geometry()
    for i, s in enumerate(starts):
        col_lo = np.array([s[0], s[1], spec.FLOOR_Z])
        col_hi = np.array([s[0], s[1], float(s[2])])
        worst = None
        for n, lo, hi in zip(names, model_lo, model_hi):
            d = seg_box_distance(col_lo, col_hi, lo, hi)
            worst = d if worst is None else min(worst, d)
            if d <= 0.0:
                fail(f"takeoff column at start {i} ({s[0]:+.2f}, {s[1]:+.2f}) "
                     f"passes through {n}")
        print(f"  start {i} ({s[0]:+.2f}, {s[1]:+.2f}) -> climb to {s[2]:.2f} m: "
              f"{worst:.2f} m clear of the nearest MODEL box, i.e. "
              f"{worst + spec.INFLATE_XY:.2f} m from the prop itself")

    # ---- verdict ---------------------------------------------------------
    print(f"\n==== {len(fails)} failure(s), {len(warns)} warning(s) ====")
    for f in fails:
        print(f"  FAIL  {f}")
    if not fails:
        print("  GEOMETRY GATE PASSED -- cleared to fly this plan")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
