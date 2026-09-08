# Outdoor Flight Run-Sheet — SERF Brisbane

**Multi-UAV 3D path planning with integrated obstacle avoidance — EGH490-2**
GPS localisation, open site, three Holybro X500 V2.

> **STATUS: SKELETON.** Downstream of the indoor campaign. Card objectives and
> criteria are drafted; commands are filled in once the indoor campaign has
> proven the stack and the GPS-specific gates have passed off-site.
> Every `Proven on:` field currently reads *not yet proven*.

---

## Prerequisites — NOT covered by this document

This run-sheet is an operational procedure only. It is **not** a risk assessment
and it is **not** an approvals record. Before any card here is valid, the
following must be in place and are tracked elsewhere:

- Site access and induction for SERF
- Airspace authorisation appropriate to the site and altitude
- PIC currency and any required certification
- Insurance and university operational approval
- Landowner / facility notification
- Emergency services access and a nominated first-aid responder

**If any of the above is unresolved, this document does not apply.**

---

## Why outdoor matters to the thesis

O-134 offers ~200 m³. The §4 simulation matrix flew ~1200 m paths across 30
waypoints and ~2400 m across 100. The indoor campaign therefore validates the
*mechanism* on a deliberately reduced scenario; **SERF is where the simulation
scale actually transfers**, and where the comparison against the §4 baseline
becomes a fair one.

It is also where the LiDAR obstacle front-end is exercised for real, closing the
combined LiDAR → plan → fly chain on a scenario big enough to be meaningful.

---

## Envelope

| | |
|---|---|
| Localisation | GPS + EKF2, standard PX4 outdoor configuration |
| Geofence | Surveyed on site, set before first flight — `lab_config/geofence_serf.yaml` |
| Altitude | Per airspace authorisation; geofence vertical limit set below it |
| Wind limit | Stated in the weather go/no-go below; hard limit, not a judgement call |
| Aircraft | As indoor, GPS parameter set instead of vision |

The parameter difference from indoor is significant and goes through the same
snapshot / managed-parameter / rollback discipline: `EKF2_GPS_CTRL` restored,
`EKF2_EV_CTRL` cleared, `EKF2_HGT_REF` off vision, indoor velocity and tilt
limits released, RTL altitudes set to sane outdoor values.

---

## Weather go / no-go — before every session

Checked at the site, out loud, recorded on the first card. **Any single no-go
stops the session.**

- [ ] Wind mean and gust below the stated limit
- [ ] No precipitation, none forecast within the session window
- [ ] Visibility adequate for visual line of sight throughout
- [ ] Cloud base above the intended operating altitude
- [ ] Temperature within battery and airframe limits
- [ ] No electrical activity within the region

---

## Session structure

| Session | Cards | Purpose |
|---|---|---|
| **O-A — Site and systems** | O-01 → O-05 | Survey, geofence, failsafes, comms. No autonomous flight. |
| **O-B — Single aircraft** | O-06 → O-11 | Build-up to a single-UAV obstacle-aware mission |
| **O-C — Multi-aircraft** | O-12 → O-13 | Three-UAV allocation, then obstacle field |

Read `common/crew_brief.md` aloud before each. `common/emergency_actions.md` and
`common/callouts.md` apply, with the outdoor differences noted per card.

---

## Cards

### O-01 — Site survey and geofence setup
**Objective.** Establish the operating area, hazards, and the geofence before
anything flies. **Proven on:** *not yet proven*
Walk the site. Identify obstacles, terrain, overhead hazards, public access,
and the emergency landing area. Survey the geofence polygon and vertical limit.
Set and verify on each aircraft by read-back.
**Pass.** Geofence set and read back on all aircraft; hazards briefed to crew.

### O-02 — Power-on and systems check
**Objective.** Same intent as indoor I-01, GPS configuration. **Proven on:** *not yet proven*
Boot, links up, uXRCE-DDS session, AS2 platform matched pub/sub, versioned topic
names present, battery logged.
**Pass.** All links up on all airframes.

### O-03 — GPS quality and EKF2 aiding check
**Objective.** Confirm position quality is adequate before trusting it.
**Proven on:** *not yet proven*
Satellite count, HDOP/EPH, EKF2 innovations, position stability with the
aircraft stationary for 10 minutes. Confirm home position set correctly.
**Pass.** Stated thresholds met and stable. **Abort.** Marginal GPS is a no-go,
not a "watch it".

### O-04 — RTL and failsafe verification
**Objective.** Prove the failsafes **before** any autonomous card, not after.
**Proven on:** *not yet proven*
Verify RC-loss action, GCS-loss action, offboard-loss action, low-battery
action, and geofence action. Where safe, demonstrate rather than assume —
RC-loss and geofence in particular.
**Pass.** Every failsafe produces its intended action, observed.
**Abort.** Any failsafe not behaving as configured ends the session.

> Outdoor RTL is a genuine return, unlike indoors where it is a manoeuvre. That
> makes this card both more useful and more necessary here.

### O-05 — Comms range check
**Objective.** Establish usable RC and telemetry range within the operating area.
**Proven on:** *not yet proven*
Walk the aircraft to the geofence extremities on the ground; monitor RC RSSI and
telemetry link quality. Identify any shadowed regions.
**Pass.** Adequate margin at every point inside the geofence.

### O-06 — Manual hover
**Objective.** First outdoor flight. Position mode, PIC on sticks.
**Proven on:** *not yet proven*
Low hover, then working altitude. Assess stability and wind response.
**Pass.** Stable hold; predictable response.

### O-07 — AS2 takeoff and land behavior
**Objective.** First autonomous command outdoors. **Proven on:** *not yet proven*
As indoor I-06.

### O-08 — AS2 go-to and follow-path
**Objective.** Position tracking and multi-waypoint following at outdoor scale.
**Proven on:** *not yet proven*
Single waypoint, then a square sized to the site.

### O-09 — LiDAR obstacle survey
**Objective.** Produce the obstacle set for the obstacle-aware cards from real
sensor data — the card that closes the combined chain. **Proven on:** *not yet proven*
1. Survey the obstacle field with the LiDAR.
2. Run `perception/lidar_to_obstacles.py` — voxel 0.2–0.5 m outdoor,
   `--auto-center`, seeded.
3. Verify the derived AABBs against hand measurements of the same obstacles.
4. Verify in RViz against the real placement before any flight uses them.
**Pass.** Derived AABBs agree with hand measurements within tolerance.
**Data.** Point cloud, obstacle pickle, comparison table.

> This is a ground card. The flight cards that follow consume its output, but no
> flight depends on the LiDAR pipeline succeeding — hand-measured obstacles
> remain the fallback, exactly as indoors.

### O-10 — DECK-GA path, baseline, single aircraft
**Objective.** Obstacle-naive baseline at outdoor scale. **Proven on:** *not yet proven*
`--path_key deckga_paths`. Read back the card ID and the path key.

### O-11 — QuickNav path with LiDAR-derived obstacles, single aircraft
**Objective.** Obstacle avoidance on sensor-derived obstacles.
**Proven on:** *not yet proven*
`--path_key quicknav_paths`, obstacles from O-09.
**Pass.** Obstacles cleared with margin; `collision_audit.py` reports zero
intersections.

> Consider inserting the outdoor equivalent of indoor **I-10** — the
> zero-obstacle null-change check — between O-10 and O-11 if site time allows.
> It is the cheapest proof that only the path key changed.

### O-12 — Three drones, full DECK-GA allocation
**Objective.** Multi-UAV at a scale where the §4 comparison is fair.
**Proven on:** *not yet proven*
Staggered takeoff, allocation, staggered recovery. Separation monitored
throughout.

### O-13 — Three drones, LiDAR-derived obstacle field
**Objective.** End-to-end acceptance for the outdoor campaign, and for the
thesis: sensor-derived obstacles, multi-UAV allocation, obstacle-aware
execution, all in one chain. **Proven on:** *not yet proven*
**Pass.** All three complete; zero collisions in the audit; the §4 metric set
reproducible from the bags at a scale comparable to the simulation.

---

## Post-session

As `common/crew_brief.md` §5, plus:

- Bags audited with `collision_audit.py`
- Metrics compared against the §4 simulation baseline — path length, avoidance
  penalty, detections, avoidances, residual collisions, planner runtime
- Parameter rollback applied if the airframes return to indoor configuration
