#!/usr/bin/env python3
"""O-134 mission geometry -- the single source of truth for the indoor flights.

Emits the two input pickles DECK_GA_QuickNav.py consumes, plus the
--start_points string, so the plan the aircraft flies is fully reproducible
from this one file.

RUN IT WITH THE SAME PYTHON THAT WILL FLY IT.
  The ground station executes deckga_execute.py under the WSL ROS 2 Humble
  python (numpy 1.21). A pickle written by numpy 2.x raises
  "ModuleNotFoundError: No module named 'numpy._core'" when that python tries
  to read it, and the mission simply cannot be loaded on the day. Measured
  2026-09-17: deckga_quicknav_synth.pkl in this repo is already unreadable for
  exactly that reason. make_mission.sh sources /opt/ros/humble/setup.bash for
  this reason and should be preferred over calling this file directly.

--------------------------------------------------------------------------
FRAME
--------------------------------------------------------------------------
Everything here is in the OptiTrack/Motive frame, in metres, and is flown with
the executor's identity transform (--coord_mode original, scales 1.0, no
z_offset). Nothing is scaled. Scaling a QuickNav path shrinks its clearance
while the physical obstacle keeps its size, which turns the avoidance margin
into a meaningless number; the README's old scale_xy=0.05 recipe must not come
back.

FLOOR_Z is not zero. Measured 2026-09-17 from bag monday_1246, drone stationary
on the pad, 940 samples: z = +0.2703 m, spread 4 mm. So the mocap origin sits
below the floor (or the rigid-body pivot sits above it -- the two are
indistinguishable from here and it does not matter). Ground clearance of any
waypoint is z - FLOOR_Z, and every altitude in this file is an absolute mocap z,
never a height above the ground.

This also settles MBD-5 in lab_config/o134_project/config/volume_guard.yaml:
box_center_m is [0,0,0] there, which asserts the mocap origin is at the CENTRE
of the volume including vertically. The measurement says it is at the floor, so
that value should be [0.0, 0.0, 2.0]. Left for the crew to change on the card,
because it is a safety file and this is not the place to edit it silently.

--------------------------------------------------------------------------
ENVELOPE
--------------------------------------------------------------------------
The room's extent in the mocap frame is NOT the nominal 8 x 6 m box centred on
the origin. Bag monday_1235 is a deliberate perimeter walk with the drone in
hand, and it traces x -3.83..+2.30, y -3.75..-1.25 -- no centred 8 x 6 box is
consistent with that, so the origin is not the room centre in xy. ENVELOPE
below is that measured loop inset by ~0.4 m. It is deliberately conservative:
it is the region that is known to be inside the net AND known to be tracked,
which is a stronger guarantee than a number off a floor plan.

If the crew measures the real usable box on the day and it is larger, do NOT
scale this mission up. Either fly it as-is (it is sized to be a complete
validation on its own) or move it with --offset, which preserves every
clearance exactly.
"""
from __future__ import annotations

import argparse
import pickle
from pathlib import Path

import numpy as np

# --------------------------------------------------------------------------
# Measured constants
# --------------------------------------------------------------------------
FLOOR_Z = 0.27          # mocap z reported by an aircraft sitting on the floor
CRUISE_Z = 1.00         # nominal mission altitude (0.73 m above the floor)

# Usable box: ((x0, x1), (y0, y1), (z0, z1)) in mocap metres.
# z ceiling is 1.45 on purpose: it stays inside volume_guard's fence under BOTH
# readings of box_center_m (|z| < 1.5 if the centre is [0,0,0], 0.5 < z < 3.5 if
# it is [0,0,2]), so a mis-set guard cannot fence the mission out of the air.
ENVELOPE = ((-3.40, 1.90), (-3.30, -1.30), (0.70, 1.45))

# Horizontal inflation applied to every physical prop before it is handed to
# the planner: 0.30 m drone half-width (450-class, including props) + 0.25 m
# safety. QuickNav routes THROUGH the corners of the box it is given, with zero
# additional clearance -- the margin has to be baked into the box or it does
# not exist.
INFLATE_XY = 0.55

# Half-width of the airframe itself, props included. INFLATE_XY is this plus
# 0.25 m of safety. It is broken out because the two are used for different
# questions: INFLATE_XY sizes the box the planner avoids, while DRONE_HALF_XY
# answers "would this trajectory actually have struck the prop", which is the
# question the baseline-vs-avoided comparison rests on. A path 0.03 m from the
# pillar is not a near miss, it is a strike -- the aircraft is 0.30 m wide.
DRONE_HALF_XY = 0.30

# --------------------------------------------------------------------------
# Obstacles
# --------------------------------------------------------------------------
# Every QuickNav detour vertex is one of the 8 corners of the AABB
# (QuickNav_avoidance_3D._build_corners, scale 1.0, or 1.5 on the retry path).
# There are no mid-height vertices, so a detour can only ever sit at cz - sz or
# cz + sz. That is what shapes the two boxes below: each one's z extents are
# chosen against the FLIGHT BAND, not against the physical prop, so that the
# cheap face is the one we want flown and it lands inside the envelope.
#
#   A: model top is far above the ceiling -> "over the top" is never shortest
#      -> the detour is forced onto the bottom face at z = 0.95, a flat skirt
#      just under cruise.
#   B: model top is just above cruise and the model bottom is below the floor
#      -> "under it" is never shortest -> the detour climbs to z = 1.30.
#
# `prop` is what the crew actually places, and `prop_z` is the physical height
# above the floor that the prop must satisfy for the model to be honest.
OBSTACLES = [
    dict(
        name="A_pillar",
        center=(-2.05, -2.30, 1.775),
        half=(0.70, 0.70, 0.825),            # x -2.75..-1.35  y -3.00..-1.60  z 0.95..2.60
        prop="slim upright: light stand / tripod with a foam tube, "
             "<= 0.30 m across",
        prop_half_xy=(0.15, 0.15),           # physical footprint, half-sizes
        prop_z=(0.00, 1.50),                 # must reach ABOVE the 1.45 ceiling
        forces="horizontal detour (bottom face, z = 0.95)",
    ),
    dict(
        name="B_lowbox",
        center=(0.55, -2.30, 0.70),
        half=(0.85, 0.75, 0.60),             # x -0.30..+1.40  y -3.05..-1.55  z 0.10..1.30
        prop="box / crate ~0.60 m tall, footprint <= 0.60 x 0.40 m",
        prop_half_xy=(0.30, 0.20),           # physical footprint, half-sizes
        prop_z=(0.00, 0.60),                 # must stay BELOW the 1.30 flyover
        forces="climb over the top (top face, z = 1.30)",
    ),
]

# --------------------------------------------------------------------------
# Waypoints
# --------------------------------------------------------------------------
# The two obstacle footprints cut the room into three bays in x:
#   L: x < -2.75        M: -1.35 < x < -0.30        R: x > +1.40
# Three waypoints sit in each bay, 0.85 m apart in y (the >= 0.80 m rule: any
# closer and two waypoints are not distinguishable in flight). Three tight,
# well-separated groups is also what keeps DCKmeans at 3/3/3 -- an empty
# cluster makes DECK_GA_QuickNav.py:298 vstack a (0,) array onto a (3,) start
# point and die.
#
# Altitudes alternate between 0.90 / 1.00 / 1.10 so the DECK-GA tours are
# genuinely three-dimensional and the QuickNav z excursions still stand out
# against them.
WAYPOINTS = [
    (-3.05, -3.15, 1.00), (-3.10, -2.30, 0.90), (-3.05, -1.45, 1.10),   # bay L
    (-0.85, -3.15, 1.10), (-0.80, -2.30, 1.00), (-0.85, -1.45, 0.90),   # bay M
    (+1.65, -3.15, 0.90), (+1.70, -2.30, 1.10), (+1.65, -1.45, 1.00),   # bay R
]

# Each cluster is given a start point in a DIFFERENT bay, so reaching its own
# waypoints is impossible without crossing an obstacle -- the avoidance is
# structural, not a lucky accident of where the GA happened to route.
#   cluster in bay L -> start in bay M   (crosses A)
#   cluster in bay M -> start in bay R   (crosses B)
#   cluster in bay R -> start in bay L   (crosses A and B)
# The pairing is resolved from each cluster's own mean x rather than from a
# fixed ordering, because DECK_GA_QuickNav.py pairs start i with cluster i BY
# INDEX and the DCKmeans ordering is not something this file should assume.
START_IN_BAY = {"L": "M", "M": "R", "R": "L"}
START_POS = {
    "L": (-3.08, -2.75, 1.00),
    "M": (-0.82, -2.75, 1.00),
    "R": (+1.70, -2.75, 1.00),
}

# The aircraft takes off from a pad placed directly UNDER the start point of
# the path being flown, and the pad is moved between flights. That is not
# decoration: deckga_execute.py takes off and then go_to()s path[0] along a
# straight line that nothing has checked for obstacles. From the pad used on
# 2026-09-15 at (-0.95, -0.65), that transit line passes straight through
# obstacle A's box. Putting the pad under the start point makes the first move
# essentially vertical and removes the hazard entirely.
PAD_UNDER_START = True


# --------------------------------------------------------------------------
def bay_of(x: float) -> str:
    """Which of the three bays an x coordinate falls in."""
    return "L" if x < -1.5 else ("M" if x < 0.4 else "R")


def cluster_waypoints(points: np.ndarray, num_uavs: int = 3):
    """Cluster exactly as the planner will, so starts can be paired correctly."""
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from DCKmeans import manual_kmeans_clustering
    clusters, _ = manual_kmeans_clustering(points, num_uavs)
    return clusters


def start_points_for(clusters) -> np.ndarray:
    return np.asarray(
        [START_POS[START_IN_BAY[bay_of(float(np.asarray(c, dtype=float)[:, 0].mean()))]]
         for c in clusters],
        dtype=float,
    )


def geometry(offset=(0.0, 0.0, 0.0)):
    """The whole mission, optionally translated. Translation is the only
    permitted transform: it preserves every clearance exactly."""
    off = np.asarray(offset, dtype=float)
    points = np.asarray(WAYPOINTS, dtype=float) + off
    centers = np.asarray([o["center"] for o in OBSTACLES], dtype=float) + off
    halves = np.asarray([o["half"] for o in OBSTACLES], dtype=float)
    (x0, x1), (y0, y1), (z0, z1) = ENVELOPE
    env = ((x0 + off[0], x1 + off[0]),
           (y0 + off[1], y1 + off[1]),
           (z0 + off[2], z1 + off[2]))
    clusters = cluster_waypoints(points)
    starts = start_points_for(clusters)
    return points, centers, halves, starts, clusters, env


def fmt_start_points(starts: np.ndarray) -> str:
    return ";".join(f"{p[0]:.3f},{p[1]:.3f},{p[2]:.3f}" for p in starts)


def main() -> int:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out_dir", default=str(here),
                    help="where to write o134_points.pkl / o134_obstacles.pkl")
    ap.add_argument("--offset", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                    metavar=("DX", "DY", "DZ"),
                    help="translate the whole mission, in metres. Use this, "
                         "never scaling, if the measured room sits elsewhere.")
    ap.add_argument("--print_only", action="store_true",
                    help="report the geometry without writing any pickle")
    args = ap.parse_args()

    points, centers, halves, starts, clusters, env = geometry(args.offset)

    (x0, x1), (y0, y1), (z0, z1) = env
    print(f"envelope     x {x0:+.2f}..{x1:+.2f}   y {y0:+.2f}..{y1:+.2f}   "
          f"z {z0:.2f}..{z1:.2f}   (floor reads {FLOOR_Z:+.2f})")
    print(f"waypoints    {len(points)}   altitudes "
          f"{sorted(set(np.round(points[:, 2], 2).tolist()))}")
    sizes = [len(np.asarray(c)) for c in clusters]
    print(f"clusters     {sizes}" + ("" if sorted(sizes) == [3, 3, 3]
                                     else "   *** NOT 3/3/3 -- planner will crash ***"))
    for o, c, h in zip(OBSTACLES, centers, halves):
        lo, hi = c - h, c + h
        print(f"obstacle {o['name']:9s} x {lo[0]:+.2f}..{hi[0]:+.2f}  "
              f"y {lo[1]:+.2f}..{hi[1]:+.2f}  z {lo[2]:.2f}..{hi[2]:.2f}   "
              f"-> {o['forces']}")
    for i, (s, c) in enumerate(zip(starts, clusters)):
        cx = float(np.asarray(c, dtype=float)[:, 0].mean())
        print(f"start {i}      {np.round(s, 2).tolist()}  (bay {bay_of(float(s[0]))}) "
              f"-> cluster in bay {bay_of(cx)}")
    print(f"\n--start_points \"{fmt_start_points(starts)}\"")

    if args.print_only:
        return 0

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    pts_pkl = out_dir / "o134_points.pkl"
    with pts_pkl.open("wb") as f:
        pickle.dump(np.ascontiguousarray(points, dtype=np.float64), f)

    # Dict format, with per-axis HALF-sizes. This is not a style choice: the
    # legacy bare-Nx3 format takes its size from --obs_size, whose default is
    # 5.0, which would make each of these a 10 m cube and abort the run on the
    # waypoint-inside-obstacle check before the GA ever starts.
    obs_pkl = out_dir / "o134_obstacles.pkl"
    with obs_pkl.open("wb") as f:
        pickle.dump({"centers": np.ascontiguousarray(centers, dtype=np.float64),
                     "sizes": np.ascontiguousarray(halves, dtype=np.float64),
                     "meta": {"frame": "mocap", "units": "m",
                              "floor_z": FLOOR_Z,
                              "inflate_xy": INFLATE_XY,
                              "envelope": env,
                              "props": [o["prop"] for o in OBSTACLES]}}, f)

    print(f"\nwrote {pts_pkl}")
    print(f"wrote {obs_pkl}")
    print(f"numpy {np.__version__} -- the flight-day python must be able to read these")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
