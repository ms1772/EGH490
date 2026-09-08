# Indoor Flight Run-Sheet — O-134

Three Holybro X500 V2, PX4 v1.17, Aerostack2, OptiTrack localisation.
Netted volume ~8 x 6 m, ~4 m usable height.

Crew: pilot on sticks, GCS operator, safety observer watching the aircraft.
Pilot and safety observer are never the same person. Abort call is
KNOCK IT OFF — anyone, any time, no justification needed.

Annexes: crew brief, emergency actions, callouts.

Status: commands are not filled in yet and nothing here has been rehearsed,
so nothing here is flyable yet.

## How to run this

Ground checks G1-G4 are gates. They run in order and none is skipped.

Flight tests are not. Each one is a superset of the ones before it, so start
at the highest test the aircraft is cleared for and only drop back when
something fails. A failure tells you where to bisect:

    F2 fails  ->  try F2a (square), then F2b (go-to), then F2c (takeoff/land)
    F5 fails  ->  try two drones, then back to single
    F6 fails  ->  try F4 (one drone, one obstacle)

Two exceptions that are never skipped, because they are about the airframe
rather than the software:

- G4 motor check, on any airframe that has not run since it was last touched
- F1 manual hover, once per airframe, before that airframe flies autonomously

If a test fails twice, stop for the day. Reproduce it in sim, fix it, come
back. Sessions get burned by debugging, not by failed tests.

No config files are edited in the lab. Everything comes from
`flight_ops/lab_config/` via `session_bringup.sh`.

## Before anything

    [ ] Snapshot taken, gates passed:  snapshot.py --label <session>-pre --check
    [ ] Rollback files present in the snapshot folder
    [ ] Crew brief read, roles agreed
    [ ] Volume clear of people and gear
    [ ] Batteries charged and logged
    [ ] Fire containment present
    [ ] Session stop time agreed

Before every flight test, without exception:

    [ ] preflight_check.py all green
    [ ] RC kill switch tested on the ground, on this airframe, this battery
    [ ] Recording running

---

# Ground checks — props off

## G1  Power on and links

1. Power the aircraft. FC boots, no unusual tones.
2. `ping 10.41.10.2` from the Jetson. Replies.
3. Start the agent: `MicroXRCEAgent udp4 -p 8888`. Reports a session.
4. `ros2 topic list`. Expect 65 `/fmu/*` topics, unversioned names
   (`vehicle_status`, not `vehicle_status_v1`). Capture the list — this is
   the only place we get ground truth on it.
5. `ros2 topic info -v` on the topics the platform uses. Publisher and
   subscriber counts non-zero. Topics existing is not enough.
6. Record battery voltage and airframe ID.

Stop if: any link is down. Set that airframe aside, carry on with the others,
fix it at the bench rather than here.

## G2  Mocap

1. Create and name a rigid body per airframe in Motive. Write the names down
   exactly — a typo here is a crash, not an inconvenience.
2. Enable VRPN streaming. Also enable NatNet as a backup while you are in there.
3. Start the bridge. `/mocap/rigid_bodies` publishing.
4. Aircraft still on a stand, record 10 minutes. Want ≥95 Hz, no gap over
   100 ms, position noise under 2 mm.
5. Carry each airframe around the perimeter, into all four corners, floor
   level and full height. Log every dropout and where it happened. This is
   the coverage map — every dead zone found here is a crash that does not
   happen later.
6. Cover the markers by hand. The bridge should stop publishing that body
   rather than emit a stale pose, and `/mocap/health` should show it untracked.
7. Record all three bodies moving through the full volume, including
   deliberate occlusions. This bag gets replayed off-site.
8. Repeat step 4 with all three Jetsons streaming, to see what wifi
   contention does.

Stop if: rate below 95 Hz or gaps over 100 ms. Switch to NatNet and repeat.
If both are bad, indoor flight does not go ahead — better to find that here.

## G3  EKF2 on vision

1. Apply `px4_indoor_params.yaml` through QGC MAVLink Console.
   `param set`, `param save`, reboot.
2. Read back every parameter. QGC on the lab laptop is older than the
   firmware and throws errors on writes that actually worked, so the popup
   means nothing and the read-back means everything.
3. Confirm `EKF2_EV_CTRL`, `EKF2_HGT_REF` = 3 (Vision), `EKF2_GPS_CTRL` = 0,
   `EKF2_EV_DELAY` = 50.
4. Set `UXRCE_DDS_NS_IDX` per airframe: 0, 1, 2. Reboot. Topics become
   `/uav_0/fmu/...` etc.
5. Carry the aircraft around by hand. `vehicle_local_position` tracks mocap.
6. Compare `self_localization/pose` against raw mocap. Should agree closely.
7. Watch EKF2 innovations for 10 minutes. Bounded, no drift.
8. Check the aircraft would accept Position mode. Do not arm.
9. Cut the mocap feed. Visual odometry publishing must stop, PX4 must declare
   a vision timeout, and volume_guard must alarm.

Stop if: step 9 does not produce a timeout. That means a dead mocap link
still looks healthy to PX4, which is the failure this whole setup exists to
prevent. No flying until it is fixed.

## G4  Motors — props on, restrained

Nobody within two metres. Kill switch tested first.

1. Arm restrained. All four spin, correct directions.
2. Step the throttle up in increments. No desync, no stutter, nothing
   sounding wrong.
3. Watch EKF2 innovations through the sweep. Still bounded.
4. Mocap keeps tracking under vibration.
5. Note the thrust needed to hover — this sets `max_thrust` in the platform
   config, which is currently an upstream default and almost certainly wrong
   for this airframe.
6. Disarm. Confirm stopped. Inspect props and mounts.

Stop if: any desync, innovation spike or tracking dropout. That airframe does
not fly today.

---

# Flight tests

## F1  Manual hover — once per airframe

Pilot flies it. No autonomy.

1. Lift to 0.5 m. Hold ten seconds. Land. Inspect.
2. Lift to 1 m. Hold thirty seconds. Should sit still — no drift, no
   oscillation, no hunting.
3. Small manual moves around the middle of the volume. Predictable, returns
   to where you put it.
4. Land.

Stop if: any drift, any oscillation, anything within a metre of the net.

The 0.5 m step is there because ground effect and net-proximity behaviour are
the two things no amount of simulation predicts.

## F2  Autonomous mission, one drone, no obstacles

Start here once F1 is done. This covers takeoff, go-to, path following and
the planner in one run.

1. Read back the test ID and the `--path_key` value before sending. F2 and F3
   differ only in that argument.
2. Run the DECK-GA plan with `--path_key deckga_paths`.
3. Watch it fly the waypoints in order.
4. Land.

Pass: completes the plan, path matches what was planned, nothing heads for
the net.

If it fails, drop back:

- F2a — four-waypoint square via follow-path
- F2b — single go-to waypoint
- F2c — AS2 takeoff and land only

Whichever one passes tells you where the problem is.

## F3  Same mission, obstacle-aware path key

Identical to F2 in every way except `--path_key quicknav_paths`, with no
obstacles in the scene.

The flown path should be indistinguishable from F2. That is the whole point:
it proves obstacle-aware versus baseline is a runtime switch and not a code
change, which is the main architectural claim of the report. Cheap test,
worth a battery on its own.

## F4  One drone, one obstacle

1. Place the obstacle. Measure it. Write the numbers on the sheet.
2. Check the box shows up in RViz where the real thing actually is.
3. Fly the obstacle-aware plan. It should go around.
4. Land.

Pass: clears the obstacle with margin, and `collision_audit.py` finds no
intersections afterwards.

Separately, and not blocking any of this: run a LiDAR scan of the same
obstacle and compare the boxes it produces against the hand measurements.
That closes the never-yet-run LiDAR-to-planner chain without any flight
depending on it.

## F5  Three drones, no obstacles

Each airframe needs F1 done first. Everything else can be skipped into.

1. On the ground, check all three report positions in the same frame — compare
   each `self_localization/pose` against mocap before anything leaves the
   floor. The frame origin used to be decided by a startup race, so this is
   the check that proves the fix held.
2. Take off staggered.
3. Run the full DECK-GA allocation.
4. Land staggered.

Pass: all three fly their own tours, separation holds.

If it fails, try two drones. If that fails, back to single.

## F6  Three drones, obstacles

Same as F5 with the obstacle field in place and the obstacle-aware path key.
This is the one that matters for the thesis.

Pass: all three complete, zero collisions in the audit, and the numbers come
out comparable to the simulation results in section 4.

If it fails, drop to F4 and add drones back one at a time.

---

# After the session

    [ ] Batteries disconnected and logged
    [ ] Bags copied off and checksummed
    [ ] collision_audit.py run over the session
    [ ] Sheets signed and scanned
    [ ] Post-session snapshot against the pre-session one
    [ ] Any parameter drift explained in writing
    [ ] Rollback applied if the hardware goes back to someone else
    [ ] Anomalies written up today, not next week
