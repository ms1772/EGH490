# O-134 Flight Session — Step by Step

Do the steps in order. Do not skip. If a step fails twice, stop for the day and
write down what happened. The abort word is KNOCK IT OFF — anyone can say it,
any time.

## Three terminals you will use

- **GS terminal** — a WSL Ubuntu terminal on the laptop.
- **DRONE terminal** — another WSL terminal, logged into the drone with `ssh jetson@10.88.38.136`. If that address does not work, look at the drone's screen or run `hostname -I` on it to get today's address.
- **QGC** — QGroundControl on the laptop, USB cable to the flight controller.

Paste these first, once, in each terminal. Put in the Motive PC address when you have it (step 3).

- GS terminal:
```
R="/mnt/c/Users/mitch/Multi-UAV Project/ROS2_MultiUAV_3D-main"; JETSON=10.88.38.136; MOTIVE=131.181.135.150
```
- DRONE terminal (after ssh):
```
source ~/as2_o134_ws/setup_env.sh; M=~/as2_o134_ws/flight_ops/deploy/mocap_bringup.sh; MOTIVE=131.181.135.150
```
- Every time you open a new terminal or new ssh, paste its line again.

## Step 1 — Arrive, power up, radio (props OFF)

- Drone on its stand in the middle of the net, props off, 5 or more reflective markers on it in a non-symmetrical pattern.
- Battery in. Plug the USB into the flight controller. Open QGC. Wait for it to connect.
- QGC: Parameters → Tools → Save to file → save as `C:\Users\mitch\Downloads\drone0.params`. This is the backup of every setting before we touch anything.
- QGC: Vehicle Setup → Radio. Turn on the transmitter (model Quad_QCR). Move the sticks. The bars on screen must move.
  - If they do not move, the receiver is not bound. Find the small receiver board on the drone, hold its bind button while powering the drone on, then on the transmitter: MDL → MODEL SETUP → Internal RF → Bind. The receiver light goes solid. Power both off and on. Try again.
- QGC: Vehicle Setup → Flight Modes. Set the mode switch so one position is Position mode. Set Kill switch to a two-position switch. Note which switch it is. Show the safety person.
- QGC: Analyze Tools → MAVLink Console. Paste this, then check each answer matches:
```
param show RC_MAP_KILL_SW
param show NAV_DLL_ACT
param show EKF2_EV_CTRL
param show EKF2_HGT_REF
param show UXRCE_DDS_NS_IDX
```
  - RC_MAP_KILL_SW must NOT be 0. NAV_DLL_ACT = 0. EKF2_EV_CTRL = 15. EKF2_HGT_REF = 3. UXRCE_DDS_NS_IDX = 0.
  - If NAV_DLL_ACT is not 0: `param set NAV_DLL_ACT 0` then `param save`.
- Kill switch test, props still off: arm the drone in Stabilized mode with the transmitter (throttle down, yaw right, hold). Motors spin. Flip the kill switch. Motors must stop instantly. Disarm. If the kill switch does not stop the motors, no flying today.

## Step 2 — Check the drone computer talks to the flight controller

- DRONE terminal:
```
bash ~/as2_o134_ws/flight_ops/deploy/verify_jetson.sh --drone-ns drone0 --gs-host 10.88.27.7 --with-fmu
```
- Wait for it to finish. The last line must say `JETSON GREEN`. Two rows may fail and are fine: battery (if pack not in yet) and px4_params.
- If it is RED for anything else, read the failed row. Most likely the drone is not on the lab wifi — check with `ip -brief addr` in the DRONE terminal; it needs an address starting 10.88.

## Step 3 — Set up the motion capture (Motive PC)

- Open Motive. File → Open the lab's current calibration project. Ask the lab tech which one if unsure.
- Put the drone in the middle of the volume, level, nose pointing along the +X arm of the calibration square on the floor. Ask which way that is. Write it down — everything later depends on it.
- All the markers show as white dots in the 3D view. If some are missing, move the drone slightly or check a camera is not blocked.
- Drag a box around all the drone's markers → right-click → Rigid Body → Create From Selected Markers.
- In the Assets list, rename the new rigid body to exactly `drone0` (lowercase, zero). This name must match or nothing works.
- View → Data Streaming. Set: Local Interface = the PC's wired address (write it down, this is MOTIVE). Up Axis = Z Up. Rigid Bodies = ticked. VRPN Streaming Engine → Broadcast Frame Data = ON, port 3883. Also turn NatNet ON as a backup.
- Go back to both terminals and set MOTIVE to that address:
```
MOTIVE=<the address you wrote down>
```

## Step 4 — Start mocap streaming on both computers

- GS terminal:
```
bash "$R/flight_ops/deploy/mocap_bringup.sh" --server $MOTIVE --gs
```
- DRONE terminal:
```
bash $M --server $MOTIVE --drone drone0
```
- Both must end with `MOCAP OK`.
- If either says NO VRPN DATA: it tells you why. Usually Motive streaming is off, the address is wrong, or the rigid body is not named drone0. Fix it in Motive, then run the same command again with `--status` on the end.
- If the drone gets nothing but the GS does, the drone's wifi cannot reach the Motive PC. Ask the tech. Do not go past step 5 without the drone receiving mocap.

## Step 5 — Check the mocap data by eye

- GS terminal — this prints the drone's position over and over for 6 seconds:
```
timeout 6 ros2 topic echo /vrpn_mocap/drone0/pose
```
- Look at `position`. Pick the drone up and move it while this runs: x, y, z change. Put it back.
- Now find out where Motive's zero is and which way is X. Run the echo again and:
  - Read z with the drone on the floor. Write it down: z = ______. If it is about 0.1, Motive's zero is on the floor. If it is about −1.9, zero is in the middle of the room.
  - Walk 1 m in some direction. Whichever of x or y went UP by 1, that direction is +X or +Y. Write it down: +X points toward ______, +Y points toward ______.
- RViz check, GS terminal:
```
rviz2 &
```
  - In RViz: Fixed Frame box (top left) → type `map`. Add → By topic → `/vrpn_mocap/drone0/pose` → Pose → OK. Walk the drone around. The arrow follows it. Turn the drone; the arrow turns.
  - If RViz will not open, skip it. The echo above already proved the data.
- Rate and drop-out check. DRONE terminal:
```
timeout 6 ros2 topic hz /drone0/mocap/rigid_bodies
```
  - Must say about 100 Hz (anything above 95 is fine). Carry the drone to each corner of the net, down to the floor, up to 2 m, running this each time. Note any spot where the rate drops or stops. Do not fly there.
- Cover ALL the markers with a cloth and run it again: it must say no new messages. Uncover: back to 100. This proves a lost mocap signal does not get faked.

## Step 6 — Start the drone software and the ground software

- DRONE terminal:
```
bash ~/as2_o134_ws/flight_ops/session_bringup.sh --hardware --role drone --drones drone0
```
  - Wait about 30 seconds. It prints a line starting `fmu:` — it must say `65 /uav_0/fmu topics`. If it prints a red NO FLIGHT CONTROLLER DATA banner, run `ping -c 3 10.41.10.2`; if that works, run step 2 again, then this again.
- GS terminal:
```
cd "$R" && bash flight_ops/session_bringup.sh --hardware --role ground --drones drone0 --jetson-peer $JETSON
```
  - This also opens RViz showing the drone. If RViz crashes, run the command again with `--no-rviz` on the end.
- Both print a preflight table at the end. Want: PREFLIGHT GREEN. battery may FAIL until a fresh pack is in. px4_params SKIP is fine.
- GS terminal — does the ground station see the drone?
```
ros2 topic list | grep -c /uav_0/fmu/
```
  - Must print 65. If it prints 0: run `ros2 daemon stop; ros2 daemon start` and try again. Still 0: the laptop cannot see the drone over wifi — check JETSON is today's address, and that the laptop is on the QUT wifi with an address starting 10.88.

## Step 7 — Check the state estimator and the flight controller agree (props OFF)

- DRONE terminal. This runs five checks in a row and takes about 30 seconds:
```
timeout 6 ros2 topic hz /drone0/self_localization/pose; ros2 topic echo --once /drone0/mocap_health; timeout 6 ros2 topic hz /uav_0/fmu/in/vehicle_visual_odometry; timeout 6 ros2 topic hz /uav_0/fmu/out/vehicle_odometry; ros2 topic echo --once /uav_0/fmu/out/vehicle_local_position_v1 | grep -E "^(x|y|z|heading|xy_valid|z_valid|v_xy_valid):"
```
  - First hz: about 100. mocap_health: `tracked: true`, `name_misses: 0`. Second and third hz: about 100. Last block: `xy_valid: true`, `z_valid: true`, `v_xy_valid: true`.
  - If xy_valid is false, wait 20 seconds and run the last command again. Still false: the flight controller is not taking the mocap. Check step 1's parameter read-back.
- QGC: the message "No valid position estimate" must be GONE. "No valid global position" stays — that is GPS, we have none, ignore it.
- Pick the drone up and carry it 1 m. Run the last command again: x or y changed by about 1. Note: the flight controller uses a different axis convention, so mocap X shows up as PX4 y and mocap Y as PX4 x. That is normal.
- GS terminal — the safety watchdog:
```
timeout 6 ros2 topic echo /volume_guard/status
```
  - Every condition OK. `dry_run: true`.
- Now the most important test. Cover all the markers with the cloth for 5 seconds, then uncover. DRONE terminal while covered:
```
timeout 6 ros2 topic hz /uav_0/fmu/in/vehicle_visual_odometry
```
  - Must say no new messages while covered. QGC must show the position warning come back. Uncover: run it again, back to 100, warning goes away.
  - If the rate keeps going while covered: STOP. Nothing flies today. Tell me.
- Reset the watchdog after the cover test, GS terminal:
```
ros2 service call /volume_guard/reset_all std_srvs/srv/Trigger
```
- Leave everything running for 10 minutes with the drone still. Watch QGC and RViz. The drone should not drift on screen.

## Step 8 — Manual hover in Position mode (props ON)

- Fresh battery. Props on. Everyone 2 m back. Nobody inside the net.
- Kill switch test again with this battery: arm restrained, motors spin, flip kill, motors stop.
- Start recording, in a NEW GS terminal (paste the R= line first):
```
mkdir -p ~/bags && ros2 bag record -a -o ~/bags/monday_$(date +%H%M)
```
  - Leave it running. Ctrl-C to stop at the end of the day.
- Pilot: mode switch to Stabilized. Arm. Lift to about half a metre. Hold 10 seconds. Land. Disarm. Look the drone over.
- Pilot: mode switch to Position. Arm. Lift to 1 m. Let go of the sticks (centred). The drone must sit still for 30 seconds — no drifting, no wobbling.
- Small stick moves forward, back, left, right. It moves and stops where you leave it.
- Land. Disarm.
- Stop if: it drifts in Position mode, wobbles, gets within 1 m of the net, or Position mode refuses to engage. Do not go to step 9 unless Position mode held still.

## Step 9 — Computer takeoff and land

- Fresh battery if the last one is below 15.2 V. Kill test. Props on. Everyone back.
- Pilot keeps the transmitter in hand the whole time. Flipping the mode switch to Position takes control back from the computer at any moment. Kill is kill.
- GS terminal (a new one, paste the R= line first). This creates the script and runs it:
```
cat > /tmp/f2c.py <<'EOF'
import rclpy
from as2_python_api.drone_interface import DroneInterface
rclpy.init()
d = DroneInterface(drone_id="drone0", verbose=True)
print("offboard:", d.offboard()); print("arm:", d.arm())
print("takeoff:", d.takeoff(height=1.0, speed=0.5))
input("Hovering. Press Enter to land ")
print("land:", d.land(speed=0.5))
d.shutdown(); rclpy.shutdown()
EOF
python3 /tmp/f2c.py
```
- Say out loud "takeoff, drone0, 1 metre" before pressing Enter. The drone climbs to 1 m and holds. Press Enter. It lands and disarms.
- Stop if: it jumps or sags on takeoff, bounces up and down more than 20 cm, or lands hard. Twice: done for the day.

## Step 10 — One waypoint

- Same checks as step 9. This goes 1 m along mocap +X at 1 m height, then lands.
- **Put the drone on the pad mark at x -0.80, y -2.30 first.** Those are Motive numbers, not "the middle of the net". Motive's zero is NOT in the middle of this room — the floor reads z = +0.27, and the area we know is inside the net and tracked runs x -3.4 to +1.9 and y -3.3 to -1.3. Everything below uses those numbers.
- Heights are absolute Motive numbers. The floor reads 0.27, so "z = 1.00" means about 0.73 m of air under the drone.
```
cat > /tmp/f2b.py <<'EOF'
import rclpy
from as2_python_api.drone_interface import DroneInterface
rclpy.init()
d = DroneInterface(drone_id="drone0", verbose=True)
print("offboard:", d.offboard()); print("arm:", d.arm())
print("takeoff:", d.takeoff(height=1.0, speed=0.5))
input("Hovering. Press Enter to fly to (0.20, -2.30, 1.00) ")
print("go_to:", d.go_to.go_to_point([0.20, -2.30, 1.00], speed=0.5))
input("Arrived. Press Enter to land ")
print("land:", d.land(speed=0.5))
d.shutdown(); rclpy.shutdown()
EOF
python3 /tmp/f2b.py
```
- It should move 1 m in the +X direction and stop. If it moves in ANY other direction: press Enter to land immediately. That means the rigid body was created pointing the wrong way in step 3 — do not try again today.
- It does not matter exactly where the drone took off from. The target is an absolute Motive position, so it flies to the same spot in the room whatever pad it started on.

## Step 11 — Four waypoints (only if step 10 worked first try)

- Same checks. Fresh battery. This flies a 1 m square at 1 m height, then lands.
```
cat > /tmp/f2d.py <<'EOF'
import rclpy
from as2_python_api.drone_interface import DroneInterface
rclpy.init()
d = DroneInterface(drone_id="drone0", verbose=True)
print("offboard:", d.offboard()); print("arm:", d.arm())
print("takeoff:", d.takeoff(height=1.0, speed=0.5))
input("Hovering. Press Enter to fly the square ")
for p in ([-1.30, -2.80, 1.00], [-0.30, -2.80, 1.00],
          [-0.30, -1.80, 1.00], [-1.30, -1.80, 1.00]):
    print("go_to", p, ":", d.go_to.go_to_point(p, speed=0.5))
input("Square done. Press Enter to land ")
print("land:", d.land(speed=0.5))
d.shutdown(); rclpy.shutdown()
EOF
python3 /tmp/f2d.py
```
- Watch each leg. It goes 1 m along X, then 1 m along Y, then back along X, then back along Y. Lands at the last corner, not at the pad.
- This square sits inside the area we measured as tracked and inside the net, with about 1.5 m to spare on every side. Do not move it to nicer round numbers — round numbers here are outside the room.
- **This is the gate.** If the square flies clean first try, go on to step 12. If it does not, stop: the mission flights are the same machinery over a bigger area.

## Read this before step 12 — what changes now

Steps 1 to 11 all had a human finger on the Enter key before anything moved. **The mission flights do not.** You start the command and the drone takes off, flies the whole route and lands on its own. The only way to interrupt it is the pilot's transmitter — mode switch to Position takes control back, kill switch stops the motors.

- The pilot holds the transmitter, thumb near the switch, for every one of these. No exceptions.
- Everybody else is outside the net, watching the drone, not the screen.
- Say the flight name and the path number out loud before pressing Enter.
- If the drone arms, the propellers spin up and then it just sits on the pad for more than 30 seconds: takeoff is not confirming. Pilot takes it back in Position mode. Do not wait and do not Ctrl-C and hope — the script has been seen to ignore a kill signal while it is waiting on the drone.
- All the numbers below are Motive positions in metres. The floor reads z = 0.27, so z = 1.00 is about 0.73 m of air.

## Step 12 — Put the obstacles in the room

- Two objects, taped to the floor at these exact spots. Measure them with a tape from the two Motive axes you identified in step 5.

| What | Centre (x, y) | Must be |
|---|---|---|
| **A — the tall one** | −2.05, −2.30 | at least 1.5 m tall, no more than 0.30 m across. A light stand or tripod with a foam tube taped to it is ideal. |
| **B — the low one** | +0.55, −2.30 | about 0.60 m tall, footprint no bigger than 0.60 × 0.40 m. A box or a crate. |

- A must be **tall** and B must be **low**. That is the whole point: the drone is supposed to go *around* A and *over* B. If A is short the drone will try to fly over it.
- Nothing else inside the net. No cases, no chairs, no people.
- Tape a cross on the floor at each of these as well. They are where the drone starts and where it finishes.

| Mark | (x, y) | What it is |
|---|---|---|
| PAD-0 | −3.08, −2.75 | take off here for flight M1 and M2 |
| PAD-1 | +1.70, −2.75 | take off here for flight M3 |
| PAD-2 | −0.82, −2.75 | take off here for flight M4 |
| END-0 | −0.30, −1.55 | M1 lands here |
| END-1 | +1.40, −1.55 | M3 lands here |
| END-2 | −1.35, −1.60 | M4 lands here |

- **The drone does not come back to its pad.** Each route finishes at its last waypoint and lands there. Keep the END marks clear and keep people away from them.

## Step 13 — Check the mission on the ground (nothing flies)

- GS terminal:
```
cd "$R" && python3 flight_ops/lab_config/o134_mission/check_mission.py --verbose
```
- Last line must say `GEOMETRY GATE PASSED`. If it does not, do not fly the mission — go back to the four-waypoint square and call it a day.
- It prints where each route goes, how far it is, and where it lands. Read the three `lands at` lines out loud to the crew.
- If the room measured differently to the numbers in step 12 — say the net is somewhere else relative to Motive's zero — the mission can be shifted without changing any clearance:
```
cd "$R" && bash flight_ops/lab_config/o134_mission/make_mission.sh --offset 0.3 0 0
```
  - That takes about two minutes and re-runs the check at the end. Do **not** try to make the mission bigger or smaller. Moving it is safe, resizing it is not.

## Step 14 — Flight M1: path 0 with obstacle avoidance (the one that matters)

This is the result the whole project is for. If the battery situation gets tight, this one and step 15 are the two to protect.

- Fresh battery. Props on. Drone on **PAD-0** (−3.08, −2.75), facing the same way it has been all day.
- Start a fresh recording in the bag terminal (Ctrl-C the old one first):
```
ros2 bag record -a -o ~/bags/M1_p0_quicknav
```
- GS terminal:
```
cd "$R" && python3 deckga_ros2/deckga_execute.py \
  --deckga_pkl flight_ops/lab_config/o134_mission/o134_mission.pkl \
  --path_key quicknav_paths --num_uavs 1 --path_index 0 \
  --ensure_reach --wp_settle_s 1.0 --first_wp_wait \
  --takeoff_z 1.0 --speed 0.35 --pre_arm_wait_s 5 \
  --run_tag M1_p0_quicknav
```
- What you should see, in order: it climbs to about 1 m over PAD-0, swings out to the far side of obstacle A and slides past it at a steady height, runs the length of the room, does three waypoints down the right-hand end, then comes back and **lifts over obstacle B** before landing on END-0.
- Stop it if: it heads straight at either obstacle, it goes above head height, it gets within 1 m of the net, or it stops responding and hovers for more than 20 seconds.
- `--path_key quicknav_paths` is doing the work here. Leave it off and you get the un-avoided route, which is step 15 and needs the obstacles taken out first.

## Step 15 — Flight M2: path 0 without avoidance — **OBSTACLES OUT OF THE ROOM**

- **Take both obstacles out of the net before this flight.** This route is the one DECK-GA produces with no obstacle avoidance at all, and it goes straight through where A and B were standing. Flying it with them in place is a crash you scheduled in advance.
- Leave the floor tape down. That is what you photograph afterwards.
- Fresh battery. Drone back on **PAD-0**.
- Bag terminal:
```
ros2 bag record -a -o ~/bags/M2_p0_deckga_baseline
```
- GS terminal — same command, two things changed:
```
cd "$R" && python3 deckga_ros2/deckga_execute.py \
  --deckga_pkl flight_ops/lab_config/o134_mission/o134_mission.pkl \
  --path_key deckga_paths --num_uavs 1 --path_index 0 \
  --ensure_reach --wp_settle_s 1.0 --first_wp_wait \
  --takeoff_z 1.0 --speed 0.35 --pre_arm_wait_s 5 \
  --run_tag M2_p0_deckga_baseline
```
- This one *is* a closed loop — it comes back and lands near PAD-0.
- It will fly straight over both tape marks. That is the point. The two flights together are the comparison: same waypoints, same drone, same day, one route avoids and one does not.
- **Put the obstacles back** before step 16.

## Step 16 — Flight M3: path 1 with obstacle avoidance

- Obstacles back in place. Fresh battery. Drone on **PAD-1** (+1.70, −2.75).
- Bag terminal:
```
ros2 bag record -a -o ~/bags/M3_p1_quicknav
```
- GS terminal — only `--path_index` changes:
```
cd "$R" && python3 deckga_ros2/deckga_execute.py \
  --deckga_pkl flight_ops/lab_config/o134_mission/o134_mission.pkl \
  --path_key quicknav_paths --num_uavs 1 --path_index 1 \
  --ensure_reach --wp_settle_s 1.0 --first_wp_wait \
  --takeoff_z 1.0 --speed 0.35 --pre_arm_wait_s 5 \
  --run_tag M3_p1_quicknav
```
- This route lifts over obstacle B twice — once on the way out and once on the way back. Lands on **END-1**.

## Step 17 — Flight M4: path 2 with obstacle avoidance

- Fresh battery. Drone on **PAD-2** (−0.82, −2.75).
- Bag terminal:
```
ros2 bag record -a -o ~/bags/M4_p2_quicknav
```
- GS terminal:
```
cd "$R" && python3 deckga_ros2/deckga_execute.py \
  --deckga_pkl flight_ops/lab_config/o134_mission/o134_mission.pkl \
  --path_key quicknav_paths --num_uavs 1 --path_index 2 \
  --ensure_reach --wp_settle_s 1.0 --first_wp_wait \
  --takeoff_z 1.0 --speed 0.35 --pre_arm_wait_s 5 \
  --run_tag M4_p2_quicknav
```
- This one goes round obstacle A on both legs, at a steady height. Lands on **END-2**.
- With M1, M3 and M4 done, one drone has flown all three of the routes DECK-GA split the job into. That is the multi-drone allocation, flown one at a time.

## If the batteries run out — what to protect

Fly them in this order and stop wherever you have to:

1. Step 11, the four-waypoint square — without this nothing below is safe to try
2. **Step 14 (M1)** — the avoidance result
3. **Step 15 (M2)** — the comparison. After this one the project has its main figure.
4. Step 16 (M3)
5. Step 17 (M4)

Steps 16 and 17 complete the three-route story. Steps 14 and 15 are the ones that cannot be left out.

## After each mission flight

- Ctrl-C the bag recording before swapping the battery. One bag per flight, named as above.
- Check the terminal printed a summary and wrote a CSV into `results_csv/`.
- Photograph the room with the tape marks and the obstacles in place, from two corners. Do this once and do it properly — it is the figure that shows the room was real.

Not on the day, but write it down now so it is not lost — each bag is checked afterwards with:

```
cd "$R" && python3 tests/automation/collision_audit.py --bag ~/bags/M1_p0_quicknav --pickle flight_ops/lab_config/o134_mission/o134_mission.pkl --pose-topics /drone0/self_localization/pose --trial-id M1_p0_quicknav --out ~/bags/M1_p0_quicknav.json
```

`--pose-topics` is not optional here. The plan says three drones, so without it the tool goes looking for drone1 and drone2 in a bag that only ever had drone0.

## Pack down

- Stop the recording: Ctrl-C in the bag terminal.
- GS terminal:
```
bash "$R/flight_ops/session_stop.sh"; bash "$R/flight_ops/deploy/mocap_bringup.sh" --stop --gs
```
- DRONE terminal:
```
bash ~/as2_o134_ws/flight_ops/session_stop.sh; bash $M --stop --drone drone0
```
- Batteries out. QGC: save parameters again as `drone0_post.params`.
- Leave Motive streaming on with the drone0 rigid body saved in the project.
- Write down anything odd while you remember it.

## If things go wrong — quick list

- Drone terminal says "command not found" or "No such file": you forgot to paste the `source ~/as2_o134_ws/setup_env.sh` line in that terminal.
- GS terminal ros2 commands do nothing: `ros2 daemon stop; ros2 daemon start`.
- mocap_bringup says NO VRPN DATA: Motive streaming is off, wrong MOTIVE address, or the rigid body is not called drone0.
- Ground station sees 0 topics from the drone: wrong JETSON address, or laptop not on the lab wifi.
- QGC still says "No valid position estimate" after step 7: run the step 1 parameter read-back; EKF2_EV_CTRL must be 15 and EKF2_HGT_REF must be 3.
- Position mode will not engage: the flight controller has no position yet — back to step 7.
- Drone drifts steadily in one direction in flight: mocap lost. Land now. Check the dead-spot notes from step 5.
- `deckga_execute.py` stops with `No module named 'numpy._core'`: the mission file was built by the wrong Python. Rebuild it with `bash "$R/flight_ops/lab_config/o134_mission/make_mission.sh"` — that takes two minutes and uses the right one.
- `--path_index N is out of range`: you typed a number above 2. There are three routes, 0, 1 and 2.
- The drone flies to a waypoint and just sits there: `--ensure_reach` is waiting for an arrival it is not going to get. Pilot takes it back in Position mode and lands. Do not wait it out.
- The drone lands somewhere you did not expect: that is normal for the three avoidance flights — they finish at their last waypoint, not at the pad. The END marks from step 12 are where to expect it.
