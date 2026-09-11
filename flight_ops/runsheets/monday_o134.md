# Monday O-134 — first ROS 2 + OptiTrack + Aerostack2 session, one aircraft

Booking 11:00–13:30. Three people: PIC, GCS+DATA, SAFETY. drone0 only. No obstacles.
Objective, in order, stop wherever it stops: VRPN streams to the drone → RViz shows
it moving → estimator → EKF2 accepts vision → manual hover in Position → AS2
takeoff/land → one go-to → (square). Annexes: crew_brief, emergency_actions, callouts.

Hard times: last ARM 13:05. Last battery disconnected 13:15. Out 13:30.
Two-failure rule per block. Abort word: KNOCK IT OFF.

Machines. `[GS]` = WSL Ubuntu-22.04 terminal on the laptop (~/.bashrc sets
RMW cyclonedds, domain 0). `[JETSON]` = `ssh jetson@<jetson-ip>` from the GS, then
`source ~/as2_o134_ws/setup_env.sh` in EVERY new ssh shell, no exceptions.
`[MOTIVE]` = lab PC. `[QGC]` = QGroundControl on the laptop, USB to the FC.
Once per GS terminal: `R="/mnt/c/Users/mitch/Multi-UAV Project/ROS2_MultiUAV_3D-main"`
Once per Jetson shell: `M=~/as2_o134_ws/flight_ops/deploy/mocap_bringup.sh`

Fill in on arrival, never assume:
Motive/VRPN ip ________ (EGH450 used 131.181.135.150:3883)  GS ip ________ (was 10.88.27.7)
Jetson ip ________ (was 10.88.38.136, DHCP)   FC = 10.41.10.2 on the Jetson's wired link.

## 0:00–0:20  Arrival and setup  (11:00–11:20)

    [ ] Crew brief read aloud, roles, go/no-go poll. Stop time written here: 13:05 last arm.
    [ ] Volume clear, fire bag out, packs voltage-checked and logged
    [ ] Aircraft on its stand in the volume, PROPS OFF, 5+ markers on, asymmetric
    [ ] [QGC] Parameters → Tools → Save to file → C:\Users\mitch\Downloads\drone0.params
    [ ] [GS] python3 "$R/flight_ops/snapshot.py" --label monday-pre --check --px4-params drone0:/mnt/c/Users/mitch/Downloads/drone0.params  → 0
        (without the dump --check passes vacuously; its ssh to 10.88.51.230 in expected_state.yaml is stale and may time out — harmless)

    [GS]     ip -brief addr; ping -c 3 <motive-ip>; ping -c 3 <jetson-ip>
    [JETSON] source ~/as2_o134_ws/setup_env.sh; ip -brief addr; ping -c 3 <motive-ip>; ping -c 3 10.41.10.2
    [JETSON] ls -l $M ~/as2_o134_ws/flight_ops/session_bringup.sh ~/as2_o134_ws/flight_ops/tmux_project.py

Motive PC may drop ping and still stream; carry on. If the `ls` fails, Friday's
copy was missed: do the "Friday software prep" block of monday_gear.md now
(scp the flight_ops files + the mocap script, chmod; ~15 min with wifi). No
tmuxinator on the Jetson: the launcher runs the same project files through
tmux_project.py. Do not go past G2 without them.

## 0:20–0:30  G1  Power and links — props off, PACK IN  (11:20–11:30)

Pack first. The RC receiver (and the telemetry radio) are powered from the
battery only: on USB alone the peripheral rail sags and both drop out, so a
receiver that is dead on USB is not a fault. RC_MAP_KILL_SW is 0 = nothing
mapped. FIRST THING, with a pack connected:
    [QGC]  Vehicle Setup → Radio: bars move with the sticks (calibrate if not).
           Flight Modes: mode channel → Stabilized / Altitude / Position on the
           3-pos switch; Kill switch → a 2-pos switch. Console: `param show
           RC_MAP_KILL_SW` non-zero. `param save`. Pilot shows SAFETY the motion.
    [ ] KILL TEST, props off: arm in Stabilized, motors turn, flip kill → motors
        stop at once. Does not stop = no props today.

Links. The launcher starts the uXRCE agent itself in G3 (window `agent` of
o134_drone0); do NOT start one by hand. Prove the FC link now without it:
    [JETSON] bash ~/as2_o134_ws/flight_ops/deploy/verify_jetson.sh --drone-ns drone0 --gs-host <gs-ip> --with-fmu
             # JETSON GREEN; fmu_topics "65 topics under this prefix". Starts and stops its own agent.
    [QGC]    MAVLink Console, `param show` each: UXRCE_DDS_NS_IDX 0, SYS_HAS_MAG 0, SYS_HAS_GPS 0,
             EKF2_EV_CTRL 15, EKF2_HGT_REF 3, EKF2_GPS_CTRL 0, EKF2_EV_DELAY 50, COM_OBL_RC_ACT 4,
             NAV_DLL_ACT 0 (set today via the console; 3 = "No connection to the GCS", no arming).
             The QGC popup means nothing; the read-back means everything.

## 0:30–0:55  G2  Mocap  (11:30–11:55)

Motive, for someone who has not touched it in years:
1. `[MOTIVE]` Open Motive. File → Open → the lab's current project/calibration
   (ask the tech which). View → Camera Calibration: date and result quality. If
   cameras were moved since, it needs wanding — the tech's job, ask first.
2. Aircraft in the volume, LEVEL, NOSE ALONG MOTIVE +X (the calibration square's
   long arm; ask). All markers visible as white dots in the Perspective view.
3. Drag-select the 5+ markers → right-click → Rigid Body → Create From Selected
   Markers. Assets pane: rename to exactly `drone0`. Created nose-along-+X is what
   makes yaw 0 = nose along +X. Created pointing elsewhere: delete, re-orient, redo.
4. View → Data Streaming pane: Local Interface = the wired IP, WRITE IT ABOVE as
   <motive-ip>. Up Axis: Z Up. Rigid Bodies: ticked. VRPN Streaming Engine:
   Broadcast Frame Data ON, port 3883. NatNet: ON, Unicast (backup only, no client).

The chain, GS first:
    [GS]     bash "$R/flight_ops/deploy/mocap_bringup.sh" --server <motive-ip> --gs
    [JETSON] bash $M --server <motive-ip> --drone drone0
Both end `MOCAP OK`. If not, the script says what it saw and stays up; it
reconnects by itself when Motive streams, so fix Motive and re-run with
`--status`. "receiving trackers ['X']": the body is not named drone0 — rename it.

Frame check, write the answers here (this decides MBD-5 and the guard's fence):
    [GS] ros2 topic echo /vrpn_mocap/drone0/pose --field pose.position
    on the floor at the calibration-square origin:  z = ______ m
      (≈ +0.1, the marker height ⇒ Motive origin is on the FLOOR ⇒ volume_guard
       box_center is wrong and its ceiling is +1.5 m today: every flight stays at
       1.0 m, yaml fixed Tuesday.  ≈ −1.9 ⇒ mid-height origin, box_center is right)
    walked 1 m toward ____________: x changed ______ ⇒ +X points toward ____________
    walked 1 m toward ____________: y changed ______ ⇒ +Y points toward ____________
    lifted 1 m: z changed ______ (if y moved instead: Up Axis is not Z Up, step 4)
    orientation: nose +X → z≈0 w≈1; nose turned to +Y → z≈0.71 w≈0.71.  OK? ____
    origin vs the net: centre / offset ______ m toward ____________

RViz, the "it moves" check, raw VRPN on the GS:
    [GS] rviz2    → Fixed Frame: map → Add → By topic → /vrpn_mocap/drone0/pose → Pose
Walk it; the arrow follows. If WSLg refuses a display, the echo above is the proof.

Coverage and staleness:
    [JETSON] ros2 topic hz /drone0/mocap/rigid_bodies       # ≥95 Hz, leave running
    [GS]     ros2 topic echo /mocap/health
Four corners, floor and 2 m: hz never below 95, health stays tracked. Cover all
markers with the cloth: hz STOPS (no stale republish), health tracked:false within
~0.3 s. Uncover: both recover. Dead spots: __________________

Jetson gets nothing but the GS does (wifi subnet cannot reach Motive): degraded
fallback, single aircraft, today only —
    [GS] bash "$R/flight_ops/deploy/mocap_bringup.sh" --server <motive-ip> --drone drone0
publishes /drone0/mocap/rigid_bodies from the GS over wifi DDS (deploy/README §2
Option A: one wifi hiccup starves the aircraft). Neither machine after 15 min of
Motive: no props today; stay and characterise mocap.

## 0:55–1:10  G3  Estimator and EKF2 on vision — props off, pack connected  (11:55–12:10)

    [JETSON] bash ~/as2_o134_ws/flight_ops/session_bringup.sh --hardware --role drone --drones drone0
    [GS]     cd "$R" && bash flight_ops/session_bringup.sh --hardware --role ground --drones drone0 --jetson-peer <jetson-ip>
Both print the post-bringup preflight: want PREFLIGHT GREEN. px4_params SKIPs
without a QGC dump (fine). battery FAILs without a flight pack: connect one.
dds_link RED = UXRCE_DDS_NS_IDX / fmu_prefix disagree; see the read-back. RViz
crashes on WSLg: add `--no-rviz`.
The Jetson launcher starts the agent (window `agent`, first) and prints
`fmu: drone0 /uav_0: 65 /uav_0/fmu topics, vehicle_odometry publishers=1`. A red
NO FLIGHT CONTROLLER DATA banner (0 topics): read the agent pane — `tmux attach -t
o134_drone0`, window agent, it must say "session established" — ping 10.41.10.2,
re-run the G1 verify. Only if the agent pane itself is dead: `session_stop.sh`,
`tmux new-session -d -s xrce_agent 'MicroXRCEAgent udp4 -p 8888'`, relaunch; the
launcher reuses a running agent and says so.
    [JETSON] ros2 topic hz /uav_0/fmu/out/vehicle_odometry            # ~100 Hz
    [GS]     ros2 topic list | grep -c /uav_0/fmu/                     # 65 = GS sees the Jetson
GS count 0 while the Jetson says 65: wifi DDS discovery is down. The ground
launcher renders the Cyclone config (eth1 pin, DontRoute, peer) from
`--jetson-peer`: confirm it is today's Jetson ip, `ros2 daemon stop`, recount.
Still 0: G2/G3 from the Jetson side only, no RViz, no F2 (the GS can neither
command nor guard what it cannot see). F1 may still go ahead.

    [JETSON] ros2 topic hz /drone0/self_localization/pose               # ~100 Hz
    [JETSON] ros2 topic echo --once /drone0/mocap_health                # tracked true, name_misses 0
    [JETSON] ros2 topic hz /uav_0/fmu/in/vehicle_visual_odometry        # ~100 Hz = platform feeding PX4
    [JETSON] ros2 topic echo --once /uav_0/fmu/out/vehicle_local_position_v1 | grep -E "^(x|y|z|heading|eph|epv|xy_valid|z_valid|v_xy_valid|heading_good_for_control):"
PX4 is NED: mocap +x appears as PX4 y, mocap +y as PX4 x, mocap +z as PX4 −z.
Nose along mocap +X: heading ≈ 1.57 and QGC's compass reads East. All *_valid
true, eph/epv < 0.1. `heading_good_for_control: false` is EXPECTED (the platform
sends pose_frame FRD and v1.17 never sets yaw_align on that path) and does NOT
block Position mode — bench 2026-09-11: PX4 accepted Position, nav_state 2.
Carry it 1 m each way: PX4 follows. QGC: Ready to Fly, and Position mode is
selectable (do not arm).
    [GS] ros2 topic echo /volume_guard/status        # every condition OK, dry_run true
Leave it 10 min on the stand. DATA logs any drift and every guard line.

Step 9, never skipped: cover all markers 5 s. `vehicle_visual_odometry` hz stops
within 0.1 s, `mocap_health` tracked:false, PX4 drops the *_valid flags in a few
s, QGC complains about position, the guard prints "[DRY RUN] would LAND/DISARM".
Uncover, then reset the latched guard:
    [GS] ros2 service call /volume_guard/reset_all std_srvs/srv/Trigger
No timeout in step 9: STOP. A dead mocap link that looks healthy is the crash this
whole setup exists to prevent. Nothing flies.

Guard decision, out loud, written here. F1: DRY RUN. F2: DRY RUN unless the
10-min watch printed zero would-have-tripped lines, in which case the crew may
`--stop` the ground role and relaunch it with `--guard-arm` (a false DISARM at 1 m
drops the aircraft; the dwell times have never met real mocap jitter).
Decision: __________  by: __________

## 1:10–1:30  F1  Manual hover — props on  (12:10–12:30)

Both stacks stay up; GCS touches nothing during F1. Nobody within 2 m.
    [ ] PREFLIGHT GREEN   [ ] KILL CHECKED: arm restrained, flip kill, motors stop, this pack
    [ ] Restrained spin-up: throttle stepped, four the right way, no stutter, mocap hz still ≥95
    [ ] RECORDING, own terminal: [GS] mkdir -p ~/bags && ros2 bag record -a -o ~/bags/monday_$(date +%H%M)
    [ ] VOLUME CLEAR
Stabilized: lift to 0.5 m, 10 s, land, inspect. Position: 1 m, 30 s, sits still.
Small moves. Land. SAFETY talks continuously (HOLDING / DRIFTING <dir>).
Stop if: DRIFTING in Position, any oscillation, within 1 m of the net, or
Position refuses (heading not aligned = EKF not on vision yaw: back to G3).
Failed twice: pack up. Never go to F2 on a Stabilized-only hover.

## 1:30–2:10  F2c takeoff/land, then F2b one go-to  (12:30–13:10)

Fresh pack. PREFLIGHT GREEN, KILL CHECKED, RECORDING, VOLUME CLEAR, every flight.
PIC holds the transmitter; the mode switch to Position takes it back from AS2 at
any moment; kill is kill. Snippets proven on rig R1 (sim) 11 Sept: offboard, arm,
takeoff goal accepted; nothing below has flown a real aircraft.

    [GS] cat > /tmp/f2c.py <<'EOF'
import rclpy
from as2_python_api.drone_interface import DroneInterface
rclpy.init()
d = DroneInterface(drone_id="drone0", verbose=True)
print("offboard:", d.offboard()); print("arm:", d.arm())
print("takeoff:", d.takeoff(height=1.0, speed=0.5))
input("F2c: hovering. Enter = land ")
print("land:", d.land(speed=0.5))
d.shutdown(); rclpy.shutdown()
EOF
    [GS] python3 /tmp/f2c.py
Read back "F2c, drone0, takeoff 1.0 m" first. Calls: OFFBOARD, F2c RUNNING, F2c
COMPLETE, LANDING, DISARMED. Pass: climbs to 1 m, holds without hunting, lands
on the land detector, disarms within 2 s.
Stop if: leaps or sags on takeoff (MBD-1 max_thrust — reads like tuning, is not),
hunts more than ±0.2 m, lands hard. Twice: done for the day.

F2b, one go-to, 1 m along the +X you wrote down in G2, same height:
    [GS] cat > /tmp/f2b.py <<'EOF'
import rclpy
from as2_python_api.drone_interface import DroneInterface
rclpy.init()
d = DroneInterface(drone_id="drone0", verbose=True)
print("offboard:", d.offboard()); print("arm:", d.arm())
print("takeoff:", d.takeoff(height=1.0, speed=0.5))
input("F2b: Enter = go to (1, 0, 1) ")
print("go_to:", d.go_to.go_to_point([1.0, 0.0, 1.0], speed=0.5))
input("arrived. Enter = land ")
print("land:", d.land(speed=0.5))
d.shutdown(); rclpy.shutdown()
EOF
    [GS] python3 /tmp/f2b.py
Assumes the pad is at the origin; otherwise add the pad's x, y to the point.
Pass: 1 m in the expected direction, stops within 0.2 m, no overshoot toward the net.
Stop if: it moves any direction other than the one on this sheet — land, do not
"try the other axis"; that is G2 step 3 (body orientation) or the frame check.

Optional square only if F2b passed first time and it is before 12:55: replace the
go_to line with four, [1,0,1] → [1,1,1] → [0,1,1] → [0,0,1], 0.5 m/s. Origin
off-centre by more than 1 m (G2 frame check): no square today.

## 2:10–2:30  Pack-down  (13:10–13:30)

    [GS]     bash "$R/flight_ops/session_stop.sh"; bash "$R/flight_ops/deploy/mocap_bringup.sh" --stop --gs
    [JETSON] bash ~/as2_o134_ws/flight_ops/session_stop.sh; bash $M --stop --drone drone0
             (session_stop stops the agent the launcher started; only a hand-started one needs `tmux kill-session -t xrce_agent`)
    [ ] Packs off, logged. Bag copied to "$R/flight_ops/snapshots/" and checksummed.
    [ ] [QGC] save params again → drone0_post.params; [GS] python3 "$R/flight_ops/snapshot.py" --label monday-post --px4-params drone0:/mnt/c/Users/mitch/Downloads/drone0_post.params --baseline "$R"/flight_ops/snapshots/<monday-pre dir>
    [ ] This sheet: every blank filled, signed, photographed. Anomalies written tonight.
    [ ] Motive: leave streaming on and the rigid body `drone0` in the project.

Abort, whole session: any KNOCK IT OFF; any block failed twice; mocap below 95 Hz
or a gap over 100 ms on the Jetson while airborne; a guard line nobody can
explain; 13:05 with no flight started; anyone uneasy at the go/no-go.
