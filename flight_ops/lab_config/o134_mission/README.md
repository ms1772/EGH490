# O-134 mission

The waypoints, obstacles and start points for the indoor validation flights,
and the two scripts that turn them into a plan and then refuse to let a bad one
fly.

```
mission_spec.py    the geometry, and the only place to edit it
make_mission.sh    spec -> DECK-GA + QuickNav -> geometry gate   (~2 min)
check_mission.py   the geometry gate, on its own                 (~2 s)
o134_points.pkl    generated: 9 waypoints, Nx3 float64
o134_obstacles.pkl generated: dict of centres + per-axis half-sizes
o134_mission.pkl   generated: the plan, with deckga_paths and quicknav_paths
```

Build it the day before a session, not on the flight line: the GA runs its full
`numIter` no matter how few waypoints it is given, so `make_mission.sh` takes
about two minutes.

## Flying it

One aircraft flies all three allocated paths, one per battery:

```
python3 deckga_ros2/deckga_execute.py \
  --deckga_pkl flight_ops/lab_config/o134_mission/o134_mission.pkl \
  --path_key quicknav_paths --num_uavs 1 --path_index 0 \
  --ensure_reach --wp_settle_s 1.0 --first_wp_wait \
  --takeoff_z 1.0 --speed 0.35 --pre_arm_wait_s 5 \
  --run_tag M1_p0_quicknav
```

`--path_index` picks which path of the plan to fly. Without it, `--num_uavs 1`
truncates to the first path and paths 1 and 2 cannot be reached at all. The
selection happens after the coordinate transform, so the route flown is
identical to what a full three-drone run would fly.

Leave every transform argument alone. The plan is already in metres in the
mocap frame and the executor's defaults are the identity.

## Things that are not obvious

**The floor is not z = 0.** An aircraft sitting on the floor reports
z = +0.27 in the mocap frame (measured from bag `monday_1246`, 940 stationary
samples, 4 mm spread). Every altitude here is an absolute mocap z; ground
clearance is `z - 0.27`.

**The mocap origin is not the middle of the room.** Bag `monday_1235` is a
perimeter walk with the drone in hand and it traces x −3.83..+2.30,
y −3.75..−1.25. No 8 × 6 m box centred on the origin contains that, so the
nominal room dimensions cannot be used to place waypoints. `ENVELOPE` is that
measured loop inset by ~0.4 m — the region known to be both inside the net and
tracked.

**Never scale this mission.** Scaling a QuickNav path shrinks its clearance
while the physical obstacle keeps its size, so the avoidance margin stops
meaning anything. If the room turns out to sit somewhere else relative to the
origin, move the mission instead: `make_mission.sh --offset DX DY DZ` preserves
every clearance exactly.

**Every detour vertex is a corner of an obstacle box.** QuickNav builds its
candidates from the eight AABB corners and nothing in it knows the room exists,
so an obstacle placed near a wall puts a *commanded waypoint* outside the net.
There are also no mid-height vertices — a detour can only sit at `cz - sz` or
`cz + sz` — which is why the two obstacle boxes are sized against the flight
band rather than against the props:

- **A** is modelled with its top far above the ceiling, so going over it is
  never the shortest detour and the route is forced into a flat skirt around
  the side at z = 0.95.
- **B** is modelled with its top just above cruise and its bottom below the
  floor, so going under it is never shortest and the route climbs to z = 1.30.

The obstacle boxes are the physical props inflated by 0.55 m per horizontal
axis (0.30 m airframe half-width plus 0.25 m safety). QuickNav routes *through*
the corners of the box it is given, with no clearance of its own — if the
margin is not in the box, it does not exist.

**The avoided paths are not closed tours.** `apply_quicknav` finishes with
`deduplicate_preserve_order`, which drops the closing point that
`ensure_closed_tour` added. So a `quicknav_paths` flight lands at its last
waypoint, not back at the pad, and comparing its length against the closed
`deckga_paths` length makes avoidance look like it *shortened* the route.
`check_mission.py` prints the open-against-open comparison and says where each
route ends.

**The baseline cannot be flown with the props in place.** `deckga_paths` is the
un-avoided route and it passes through both obstacle boxes — the gate confirms
it would strike them. Fly it with the obstacles physically removed and make the
collision claim geometrically afterwards, against the recorded positions.

**The pad moves between flights.** It goes directly under the start point of
the path being flown. `deckga_execute.py` takes off and then `go_to`s the first
waypoint along a straight line nothing has checked; from the pad used on
2026-09-15 that line runs through obstacle A.

## Afterwards

```
python3 tests/automation/collision_audit.py --bag ~/bags/M1_p0_quicknav --pickle flight_ops/lab_config/o134_mission/o134_mission.pkl --pose-topics /drone0/self_localization/pose --trial-id M1_p0_quicknav --out ~/bags/M1_p0_quicknav.json
```

`--pose-topics` has to be given. The tool defaults its topic list from the
pickle's `num_uavs`, which is 3, so on a single-aircraft bag it would look for
`/drone1/...` and `/drone2/...` that were never recorded.

The audit measures the flown trajectory against `obstacle_xyz` / `obs_size` in
the pickle -- the INFLATED model boxes, not the props. A run that shows a small
penetration of the model box may still have cleared the physical obstacle by
tens of centimetres; `check_mission.py` prints both distances for the planned
route, and the same distinction applies to the flown one.

## What rig R1 can and cannot rehearse

Measured 2026-09-17. R1 comes up clean -- `session_bringup.sh --sim --drones
drone0` reaches PREFLIGHT GREEN, 6/6, and the platform arms and goes offboard
on command.

**It cannot rehearse this mission past takeoff.** `fake_mocap.py` publishes a
fixed 20 s circle and nothing the platform does changes it, so
`self_localization/pose` orbits at a constant height no matter what is
commanded. `safe_takeoff(..., wait=True)` then waits for an arrival that cannot
happen, and the run hangs there indefinitely -- observed sitting at takeoff for
12 minutes with the aircraft armed, offboard and orbiting at z = 2.2.

This is broader than "R1 cannot rehearse `--ensure_reach`": takeoff blocks
whether or not that flag is given, because takeoff is always `wait=True`. So
nothing downstream of takeoff -- waypoint order, the CSV summary, the landing
phase -- can be exercised on the rig at all.

`timeout` did not end the run either: the process ignored SIGTERM while blocked
in the behaviour call and needed SIGKILL. The script installs a handler for
Ctrl-C (SIGINT) only. On the day this is not a theoretical point -- **the RC
kill switch, not the keyboard, is the interrupt.**

What is left to validate the ground-station side, and what was used instead:

- argument parsing and `--path_index` bounds -- `deckga_execute.py --help` and
  an out-of-range index
- the slice itself -- proven against a real 3-path plan by importing
  `load_deckga_pkl` / `transform_paths` directly and comparing each
  `--path_index N` selection against path N of the full run, byte for byte
- the flown geometry -- `check_mission.py`, which uses those same two functions

Which means **the four-waypoint square in O-134 is the first end-to-end test of
`deckga_execute.py` on anything.** Fly it first, low and small, before the
mission flights.
