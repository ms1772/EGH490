"""Generate the adversarial 'overlapping obstacles' fixture for the Stage 0 smoke test.

Designed to exercise bug A: iter 1's detour around obstacle A is geometrically
forced to splice through obstacle B's cube, so without iterative re-detection
the final path still penetrates B.

Layout (all coords in metres):

  - 3 UAVs, each with a 3-waypoint path. The middle waypoint of UAV 0 is placed
    so the segment from waypoint 0 -> waypoint 1 crosses obstacle A.
  - Obstacle A is on that segment. Obstacle B is placed adjacent to A in the
    direction the naive single-iteration detour would route, so the detour
    around A penetrates B.
  - UAV 1 and UAV 2 have simpler clear paths (sanity).

Run directly to (re)generate the pickles:
    python tests/automation/fixtures/smoke_overlap.py
"""

from __future__ import annotations

import pickle
from pathlib import Path

import numpy as np

FIXTURES_DIR = Path(__file__).resolve().parent

# Waypoints: 9 points total, 3 per UAV cluster. K-means in DECK_GA_QuickNav
# will assign these to 3 clusters based on the planner's hardcoded start_points
# (10,0,10; 50,20,10; 90,0,10). We place waypoints near each start_point so the
# clustering is unambiguous.
POINTS = np.array([
    # Cluster near (10,0,10) -- UAV 0 will fly through obstacle region
    [5.0, 0.0, 10.0],
    [20.0, 0.0, 10.0],   # this segment (start -> here) crosses obstacle A
    [25.0, 5.0, 10.0],
    # Cluster near (50,20,10) -- UAV 1, clear
    [45.0, 18.0, 10.0],
    [55.0, 20.0, 10.0],
    [50.0, 25.0, 10.0],
    # Cluster near (90,0,10) -- UAV 2, clear
    [85.0, 0.0, 10.0],
    [95.0, 0.0, 10.0],
    [90.0, 5.0, 10.0],
], dtype=float)

# Two overlapping obstacle cubes placed on UAV 0's first segment, with their
# safety cubes (half-size = 1.0) abutting each other so any naive detour around
# A in +y direction must cross B.
OBSTACLES = np.array([
    [12.0, 0.0, 10.0],   # Obstacle A: on segment from (5,0,10)->(20,0,10)
    [12.0, 1.6, 10.0],   # Obstacle B: 1.6 m in +y from A (with half_size 1.0,
                          # the cubes' safety boxes touch but don't fully overlap.
                          # A detour around A that picks the +y face will route
                          # through B's cube.)
], dtype=float)


def main() -> None:
    points_pkl = FIXTURES_DIR / "smoke_overlap_points.pkl"
    obs_pkl = FIXTURES_DIR / "smoke_overlap_obstacles.pkl"

    with points_pkl.open("wb") as f:
        pickle.dump(POINTS, f)
    with obs_pkl.open("wb") as f:
        pickle.dump(OBSTACLES, f)

    print(f"Wrote {points_pkl} ({POINTS.shape})")
    print(f"Wrote {obs_pkl} ({OBSTACLES.shape})")


if __name__ == "__main__":
    main()
