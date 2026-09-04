# `o134_project` — the Aerostack2 project for real indoor flight in O-134

This is the hardware sibling of
`aerostack_examples/02_examples_gazebo_project/project_gazebo/`. Same shape —
a `config/` directory, a `tmuxinator/` directory, a ground-station RViz
config — so anyone who has driven the Aerostack2 reference project can read
this one. Two things are different, and both are deliberate:

* **`use_sim_time` is `false` everywhere.** Nothing in this stack publishes
  `/clock`. Motion capture, the FMU and the controller all run on wall time.
* **There is no Gazebo anything.** No world file, no models, no
  `as2_gazebo_assets`, no simulation config threaded through the platform
  launch.

Nothing here is launched by hand. Everything is started by
`flight_ops/session_bringup.sh`, which is the same file off-site and in the
lab, because *no configuration is edited inside O-134*.

```bash
# rig R1, on the ground station, no hardware at all
flight_ops/session_bringup.sh --sim

# the same chain, three aircraft
flight_ops/session_bringup.sh --sim --drones drone0,drone1,drone2

# O-134, one aircraft, guard live
flight_ops/session_bringup.sh --hardware --drones drone0 --guard-arm

flight_ops/session_bringup.sh --stop
```

---

## 1. Layout, and why it is one shared file plus launch arguments

```
o134_project/
├── config/
│   ├── drones.yaml                 <-- THE per-aircraft table. The only file
│   │                                   where the three airframes differ.
│   ├── platform_pixhawk.yaml       shared   as2_platform_pixhawk   (--hardware)
│   ├── platform_sim.yaml           shared   multirotor simulator   (--sim)
│   ├── mocap_pose_guarded.yaml     shared   the S1 estimator plugin
│   ├── motion_controller.yaml      shared   controller_manager node
│   ├── pid_speed_controller.yaml   shared   controller gains
│   ├── motion_behaviors.yaml       shared   takeoff / land / go_to / follow_path
│   ├── trajectory_generation.yaml  shared   polynomial trajectory behaviour
│   ├── volume_guard.yaml           shared   S2 watchdog thresholds
│   └── fake_mocap_r1.yaml          rig R1   synthetic mocap source
├── config_ground_station/
│   └── o134.rviz
└── tmuxinator/
    ├── drone.yaml                  one tmux session per aircraft
    └── ground_station.yaml         one tmux session for the fleet
```

**There are no per-drone parameter files.** Three near-identical YAMLs would
drift apart inside a fortnight, and the drift would be invisible — a diff of
three 100-line files is not something anyone does on a lab afternoon. Instead
the four values that differ live in one 15-line table, `config/drones.yaml`,
where a mismatch is visible by reading four rows side by side.

### What the launch machinery actually supports — checked, not assumed

The obvious alternative, "pass a shared file and a per-drone overlay file",
**does not work**, and it is worth writing down why so nobody tries it again.

`as2_core.LaunchConfigurationFromConfigFile.perform()` — the class every
Aerostack2 launch file uses for `config_file:` / `platform_config_file:` —
does exactly one `open()` on exactly one path:

```python
user_yaml_filename = launch.substitutions.LaunchConfiguration(self.name).perform(context)
...
with open(user_yaml_filename, 'r', encoding='utf-8') as file:
    user_data, _ = read_complete_yaml_text(file.read())
```

One file per node. No list, no comma-separated paths, no second overlay
argument. (Read at
`/opt/ros/humble/local/lib/python3.10/dist-packages/as2_core/launch_configuration_from_config_file.py`.)

But the same method **does** give a real overlay mechanism, one layer up.
`DeclareLaunchArgumentsFromConfigFile` declares a launch argument for every
leaf key in the package's *installed default* config, and `perform()` merges
in this order:

```
installed package default   <-   launch arguments   <-   the user's config file
```

So a per-instance value can be passed as a plain launch argument, provided
**the shared config file does not mention that key** — because the user file
wins over the command line, silently.

That is the layout: one shared file per node, with the per-aircraft keys
deliberately *absent*, and those keys supplied as launch arguments generated
from `drones.yaml`.

**Verified by running it**, not by reading it:

```
$ ros2 launch as2_platform_pixhawk pixhawk_launch.py namespace:=drone1 \
      platform_config_file:=<shared file with no fmu_prefix, no target_system_id> \
      fmu_prefix:=/uav_1 target_system_id:=2

[INFO] [drone1.platform]: External odometry mode: true       <- from the shared file
[INFO] [drone1.platform]: External odometry timeout: 0.100 s <- from the shared file
[INFO] [drone1.platform]: FMU prefix: /uav_1                 <- from the launch argument

$ ros2 param get /drone1/platform fmu_prefix        -> String value is: /uav_1
$ ros2 param get /drone1/platform target_system_id  -> Integer value is: 2
$ ros2 topic list | grep fmu
/uav_1/fmu/in/vehicle_command
/uav_1/fmu/in/vehicle_visual_odometry
/uav_1/fmu/out/vehicle_odometry
...
```

The state estimator needs no such trick: `as2_mocap_guarded`'s own launch
file already exposes `rigid_body_name` and `mocap_topic` as first-class
arguments and applies them *after* both YAMLs.

### One consequence worth stating plainly

`config/platform_pixhawk.yaml` **must never grow an `fmu_prefix` or a
`target_system_id` key.** If it does, all three aircraft are pinned to one FMU
namespace and the launch argument is ignored without a single log line. That
is why both keys carry a comment saying so at the point where someone would
add them.

---

## 2. What differs per aircraft

Four values. That is the whole list.

| | drone0 | drone1 | drone2 | Set where | Consumed by |
|---|---|---|---|---|---|
| **Aerostack2 namespace** | `drone0` | `drone1` | `drone2` | key in `config/drones.yaml` | every node's `namespace:=`; `deckga_execute.py`; `preflight_check.py --drones`; `volume_guard --drones` |
| **`fmu_prefix`** | `/uav_0` | `/uav_1` | `/uav_2` | `config/drones.yaml` → launch argument | `as2_platform_pixhawk` (`--hardware` only) |
| **`target_system_id`** | `1` | `1` | `1` | `config/drones.yaml` → launch argument | `as2_platform_pixhawk`; mirrored into `volume_guard --target-systems` |
| **`rigid_body_name`** | `drone0` | `drone1` | `drone2` | `config/drones.yaml` → launch argument | `mocap_pose_guarded` plugin; mirrored into `volume_guard --rigid-bodies` and `fake_mocap --bodies` |

Two more values are *derived* by `session_bringup.sh` and are not separate
settings:

| Derived | `--sim` | `--hardware` | Why |
|---|---|---|---|
| `mocap_topic` | `/mocap/rigid_bodies` | `/<ns>/mocap/rigid_bodies` | Per **mode**, not per aircraft. In the lab each Jetson runs its own VRPN client and bridge (deploy README §2, Option B), so a wifi dropout starves one aircraft rather than three. On rig R1 there is one synthetic source for the whole rig. |
| volume_guard remaps | none | `-r /<ns>/fmu/in/vehicle_command:=<fmu_prefix>/fmu/in/vehicle_command` | See §4. |

### On the flight controller, not in these files

| Parameter | drone0 | drone1 | drone2 | Notes |
|---|---|---|---|---|
| **`UXRCE_DDS_NS_IDX`** | `0` | `1` | `2` | **This is the one that matters and nothing in this repository writes it.** PX4 v1.17 defines it as "index-based namespace for DDS messages, e.g. uav_0, uav_1" (`src/modules/uxrce_dds_client/module.yaml` @ v1.17.0), default `-1` = no namespace, **`reboot_required`**. Written in QGroundControl, read back with `param show`, then reboot. It is in the managed-parameter allow-list in `lab_config/expected_state.yaml` and is the only entry there with a per-airframe value. |
| `MAV_SYS_ID` | 1 | 1 | 1 | Factory default, **not** in the allow-list, so this project does not change it. It is what `target_system_id` above has to match. See MBD-2. |
| `UXRCE_DDS_CFG` / `PRT` / `KEY` / `DOM_ID` | 1000 / 8888 / 1 / 0 | same | same | Gated by `expected_state.yaml → gates.must_equal`. A mismatch means somebody else's session is live: abort, do not "fix". |
| everything in `lab_config/px4_indoor_params.yaml` | identical | identical | identical | EKF external-vision aiding, envelope, geofence, failsafes. |

**The coupling to check first, every session.** `UXRCE_DDS_NS_IDX = N` on the
FC and `fmu_prefix = "/uav_N"` in `drones.yaml` are two halves of one
decision. If they disagree the platform starts perfectly cleanly, reports
`connected: false` forever, and looks exactly like a network fault.
`drones.yaml` carries an advisory `uxrce_dds_ns_idx` column purely so the two
halves can be eyeballed together; the launcher does not read it.

`preflight_check.py`'s `dds_link` check is what catches a mismatch
mechanically. `session_bringup.sh` runs it once per drone with that drone's
own `--fmu-prefix`, because the tool takes one prefix and this topology has
one per aircraft.

---

## 3. Why these plugin choices

| Choice | Reason |
|---|---|
| `mocap_pose_guarded`, launched from **`as2_mocap_guarded`'s own launch file** | The stock `as2_state_estimator` launch file cannot load an out-of-package plugin: `plugin_name` is declared with `choices=get_available_plugins('as2_state_estimator')`, and the plugin's default config path is hard-wired under the `as2_state_estimator` share directory and `open()`ed unconditionally. Both confirmed by running them — see `deckga_ros2/as2_mocap_guarded/README.md`. The **node** is still the stock `as2_state_estimator_node`. |
| `pid_speed_controller`, not `differential_flatness_controller` | Forced by the platform. `as2_platform_pixhawk` advertises `UNSET`, `ACRO`, `ATTITUDE`, `SPEED+yaw angle`, `SPEED+yaw speed` — POSITION and TRAJECTORY are commented out upstream. A SPEED setpoint in ENU is the only thing the aircraft accepts for waypoint flight. |
| `takeoff_plugin_position`, `land_plugin_speed`, `go_to_plugin_position`, `follow_path_plugin_position` | Position variants for anything that must stop somewhere specific under a 4 m ceiling net; speed for landing, which finishes on the land detector rather than on a floor height it does not have indoors. |
| `plugin_name` on the command line, never in a YAML | A plugin swap changes what is flying the aircraft. It must not be possible by editing a config file that some other tool also writes. |

---

## 4. `volume_guard` and the `/uav_N` mapping — the honest answer

**Question:** `volume_guard.py` must publish its force-disarm to
`/uav_N/fmu/in/vehicle_command`. Can its `--fmu-prefix` express that?

**No.** `resolve_fmu_topic()` is:

```python
def resolve_fmu_topic(prefix: str, namespace: str) -> str:
    prefix = str(prefix or "")
    base = prefix.format(ns=namespace) if "{ns}" in prefix else prefix
    return base + FMU_VEHICLE_COMMAND_TOPIC
```

`{ns}` resolves to the **Aerostack2** namespace, `drone0`, never `uav_0`.
Nor is there a format trick: `'/uav_{ns[5]}'` *would* format to `/uav_0`, but
the substitution is gated on the literal substring test `"{ns}" in prefix`,
which `'/uav_{ns[5]}'` fails, so the prefix would be used verbatim. There is
no string that expresses `drone0 -> /uav_0`.

The three configurations that *are* expressible, and what each costs:

| Configuration | Result |
|---|---|
| `--fmu-prefix /uav_0`, **one drone only** | Correct and exact. `check_disarm_routing()` returns `None` for a single drone, so `--arm` is accepted. |
| `--fmu-prefix /uav_0`, **three drones** | Refused. `check_disarm_routing()` rejects it: three drones on one topic with colliding `target_system` ids means a drone0 trip disarms drone1. Verified: the node exits 2 with that message. Correctly refused — it would also be aimed at the wrong FC. |
| `--drones uav_0,uav_1,uav_2` | Breaks everything else. `pose_topic_template` and `land_action_template` both interpolate the same `{ns}`, so the guard would watch `/uav_0/self_localization/pose` (does not exist) and call `/uav_0/LandBehavior` (does not exist). `pose_timeout` would trip on every aircraft continuously. Strictly worse than useless. |

**What `session_bringup.sh` does instead**, without touching
`volume_guard.py`: keep `--fmu-prefix '/{ns}'`, which gives one distinct
topic per aircraft and satisfies the routing check, and do the rename one
layer down with ROS 2 **static remapping**, which rewrites the topic after the
node has named it:

```bash
python3 flight_ops/nodes/volume_guard.py \
    --drones drone0,drone1,drone2 --fmu-prefix '/{ns}' --arm \
    --ros-args --params-file config/volume_guard.yaml \
    -r /drone0/fmu/in/vehicle_command:=/uav_0/fmu/in/vehicle_command \
    -r /drone1/fmu/in/vehicle_command:=/uav_1/fmu/in/vehicle_command \
    -r /drone2/fmu/in/vehicle_command:=/uav_2/fmu/in/vehicle_command
```

The remap arguments are generated from `drones.yaml`, so the guard's topics
and the platforms' topics cannot disagree.

Verified end to end through the launcher (`--hardware --role ground`):

```
$ ros2 node info /volume_guard
  Publishers:
    /uav_0/fmu/in/vehicle_command: px4_msgs/msg/VehicleCommand
    /uav_1/fmu/in/vehicle_command: px4_msgs/msg/VehicleCommand
    /uav_2/fmu/in/vehicle_command: px4_msgs/msg/VehicleCommand
```

and independently, by `preflight_check.py`'s own graph inspection, which
found the guard's publisher sitting on the right topic waiting for an FMU
that was not there:

```
dds_link  -  FAIL  absent: /uav_0/fmu/out/vehicle_odometry, ... ;
                   no matched endpoint: /uav_0/fmu/in/vehicle_command (pub=1, sub=0)
```

> **The one caveat, because it will bite someone at 2 a.m.** volume_guard's
> start-up log line and its `/volume_guard/status` payload report the
> **pre-remap** name (`/drone0/fmu/...`, `"fmu_prefix": "/{ns}"`), because
> that is the name the node asked for. The remap happens below it, in rcl.
> `ros2 node info /volume_guard` reports the real one. **Trust the graph, not
> the log.**
>
> If that is not acceptable — and for a safety node it is a fair objection —
> the alternative is a one-line change to `volume_guard.py` to accept a
> per-drone prefix map (`--fmu-prefix drone0:/uav_0,...`) the same way
> `--rigid-bodies` and `--target-systems` already do. That file is out of
> scope for this task and was not modified.

---

## 5. MUST BE DETERMINED

Every value below is a placeholder with a defensible provenance, not a
measurement. Each one is also printed by `session_bringup.sh --hardware`, in
a banner, every single launch, so it cannot quietly become the truth.

| | Where | What is unknown | What goes wrong if it stays as it is |
|---|---|---|---|
| **MBD-1** | `config/platform_pixhawk.yaml` → `max_thrust: 15.0` | The static thrust of an X500 V2's four motors, in newtons, on a charged 4S pack. **15.0 N is the `as2_platform_pixhawk` upstream default and has nothing to do with this airframe.** | Aerostack2 uses it to convert commanded newtons into PX4's normalised setpoint. Wrong → the *entire* thrust axis is mis-scaled by a constant. Too low: everything saturates and the aircraft leaps on takeoff. Too high: commands are attenuated, it is sluggish or will not leave the ground, and the altitude integrator winds up trying to fix it. **Both read exactly like a controller tuning problem, and no amount of tuning fixes a wrong unit conversion.** Determine on the bench, props on, airframe restrained — run-sheet card **I-04**. |
| **MBD-2** | `config/drones.yaml` → `target_system_id: 1` (×3) | Each airframe's actual `MAV_SYS_ID`. | `MAV_SYS_ID` is not in the managed-parameter allow-list, so this project does not write it and the PX4 factory value is 1. If the lab has assigned distinct ids, arm/disarm from Aerostack2 is silently ignored by the aircraft. Read all three back in QGC and put them in the table. |
| **MBD-3** | `config/platform_pixhawk.yaml` → `external_odom_timeout_s: 0.1` | Whether the real OptiTrack → bridge → estimator → TF chain has jitter that trips a 100 ms gate. | Listed as unverified in `flight_ops/patches/PATCHES.md`: 0.1 s was chosen as 10 cycles of the 100 Hz timer, not measured. Too tight → the platform stops feeding PX4 on ordinary jitter and hands a healthy aircraft to the external-vision failsafe. Log the inter-arrival distribution on the bench first. It is a runtime parameter for exactly this reason. |
| **MBD-4** | `config/pid_speed_controller.yaml` → every gain | Indoor gains for an X500 V2 on 100 Hz mocap with a 40 ms velocity filter in the loop. | The values are carried verbatim from the Aerostack2 Gazebo reference project — a simulated quadrotor with perfect state. They have never been flown on this airframe. The installed package defaults are all **0.0**, which is why they are not simply left alone: a zero-gain PID comes up green, accepts a goal, and commands zero speed forever. Tune on the tether, one axis at a time, against the `px4_indoor_params.yaml` envelope. |
| **MBD-5** | `config/volume_guard.yaml` → `box_center_m: [0, 0, 0]` (and the matching value in `fake_mocap_r1.yaml`) | Where Motive's origin is, vertically. | `[0,0,0]` says the mocap origin is at the *centre* of the volume, i.e. z = 0 is 2 m above the floor. If Motive's origin is on the floor, this must be `[0, 0, 2]` — otherwise the lower half of the fence is underground and the aircraft is unprotected below 2 m. A ten-second check in Motive on the first lab visit. |
| **MBD-6** | `config/drones.yaml` → `rigid_body_name` | The exact strings Motive streams. | `drone0` / `drone1` / `drone2` is an assumption. Motive commonly streams numeric ids. A mismatch is defect **D1**: the guarded plugin refuses to publish and says so loudly (which is the good outcome), but the session stops until it is fixed. Confirm the spelling — case and punctuation included — against Motive before the first flight. |
| **MBD-7** | `config/mocap_pose_guarded.yaml` → `earth_to_map_* : 0.0` | Whether the lab frame is deliberately offset from the mocap origin. | Identity is the *right default* — it is what makes `earth == map` identical for all three aircraft regardless of start order, and it is what `preflight_check.py`'s `tf_tree` identity assertion checks. Recorded here only so that, if the room frame is ever offset, it is changed for **every** aircraft in the session and the preflight identity check is re-argued rather than switched off. |

Not a placeholder, but worth knowing: `preflight_check.py`'s `as2_nodes`
check defaults to `platform,state_estimator,controller_manager`. Its own
header says to pin that list against the running system on the first lab
visit. With this project the behaviour nodes are also up, so the list can be
widened once someone has confirmed what "normal" looks like.

---

## 6. Verification — what was actually run

Rig R1, WSL2 Ubuntu 22.04, ROS 2 Humble, Aerostack2 1.1.3, `ROS_DOMAIN_ID=0`.
Full three-aircraft chain from one command, no hardware:

```
$ flight_ops/session_bringup.sh --sim --drones drone0,drone1,drone2 --settle 25
```

**Nodes** — 35 excluding TF listeners: per namespace `platform`,
`state_estimator`, `controller_manager`, `TakeoffBehavior`, `LandBehavior`,
`GoToBehavior`, `FollowPathBehavior`, `TrajectoryGeneratorBehavior`,
`alphanumeric_viewer`, `marker_publisher_node`, `robot_state_publisher`; plus
`/fake_mocap` and `/volume_guard`. Every tmux pane `dead=0`.

**Post-bringup preflight** — 16/16:

```
mocap_rate    /mocap/rigid_bodies  PASS  102.4 Hz over 5.0 s, worst 1 s 100.0 Hz,
                                         max gap 12.0 ms, jitter 1.51 ms
rigid_bodies  drone0/1/2           PASS  in 512/512 messages (100.0%)
tf_tree       drone0/1/2           PASS  3 link(s) resolve; coincident:
                                         earth->droneN/map 0.0 cm / 0.00 deg,
                                         droneN/map->droneN/odom 0.0 cm / 0.00 deg
as2_nodes     drone0/1/2           PASS  3 node(s) alive
platform      drone0/1/2           PASS  connected=True, armed=False, offboard=False
pose_delta    drone0               PASS  delta 0.0 cm (est -1.729,0.268,-0.969
                                         vs mocap -1.729,0.268,-0.969)

PREFLIGHT GREEN   16 passed, 0 failed, 0 skipped
```

**TF**, `earth -> droneN/map -> droneN/odom -> droneN/base_link`, resolving
with both upper links identity from parameters (defect D2 closed):

```
$ ros2 run tf2_ros tf2_echo earth drone1/base_link
- Translation: [1.349, 1.114, 0.127]
- Rotation: RPY (degree) [0.000, -0.000, 129.553]

$ ros2 run tf2_ros tf2_echo earth drone1/map          -> [0,0,0] / [0,0,0,1]
$ ros2 run tf2_ros tf2_echo drone1/map drone1/odom    -> [0,0,0] / [0,0,0,1]
```

**Pose tracking the synthetic mocap** — ~100 Hz on all three, and the twist
path alive (0.559 m/s is the tangential speed of the configured circle, not a
constant zero that a broken finite difference would also produce):

```
$ ros2 topic hz /drone0|1|2/self_localization/pose
  min 0.005s  max 0.014s  std dev 0.0018s   (x3)

$ ros2 topic echo --once /drone1/self_localization/twist
  frame_id: drone1/base_link
  linear: x 0.5585836  y -0.0078346  z 0.0274139
```

**Guarded plugin health** — clean on all three:

```json
{"tracked": true, "age": 0.0059, "messages": 8829, "accepted": 8829,
 "name_misses": 0, "rejects": 0,
 "earth_to_map": {"x":0.0,"y":0.0,"z":0.0,"yaw":0.0}}
```

**Volume guard** watching the fleet through this configuration, all seven
conditions `OK`, fence `+/-3.50, 2.50, 1.50 m`, `pose_delta 0.005 m`,
`dry_run: true`.

**Every configured value read back off the running nodes** — the point being
that a config file that is *loaded* is not the same as a config file that is
*applied*, and `as2_core` parses these YAMLs with its own hand-rolled parser
(`read_complete_yaml_text`), not PyYAML, so nested keys are worth confirming
rather than assuming:

```
controller position_control.kp.x  1.0                          <- pid_speed_controller.yaml (nested)
controller use_bypass             True                         <- motion_controller.yaml
controller plugin_name            pid_speed_controller          <- command line
estimator  twist_smooth_filter_cte 0.2                          <- mocap_pose_guarded.yaml
estimator  plugin_name            mocap_pose_guarded            <- the launch file, not a yaml
estimator  rigid_body_name        drone0                        <- drones.yaml -> launch argument
estimator  mocap_topic            /mocap/rigid_bodies           <- derived from the mode
takeoff    plugin_name            takeoff_plugin_position       <- motion_behaviors.yaml
takeoff    takeoff_height         1.0                           <- motion_behaviors.yaml
land       plugin_name            land_plugin_speed             <- motion_behaviors.yaml
go_to      go_to_speed            0.5                           <- motion_behaviors.yaml
follow_path plugin_name           follow_path_plugin_position   <- motion_behaviors.yaml
trajgen    sampling_dt            0.01                          <- trajectory_generation.yaml
guard      box_size_m             [8.0, 6.0, 4.0]               <- volume_guard.yaml
guard      mocap_timeout_s        0.1                           <- volume_guard.yaml
fake_mocap trajectory             circle                        <- fake_mocap_r1.yaml

use_sim_time  estimator False   controller False   takeoff False
```

### A fault, because a chain that has only ever run nominally proves little

A 4 s total mocap dropout injected at runtime on the same running rig:

```
$ ros2 topic pub --once /fake_mocap/inject std_msgs/msg/String "{data: 'dropout+4'}"
```

* `mocap_health` → `{"tracked": false, "age": 1.9703}` — ageing out, not
  freezing at the last good pose.
* `volume_guard` drone0 → `state: DISARM`, `latched: true`,
  `driving_conditions: ["mocap_timeout", "pose_timeout"]`, and in the log
  `[DRY RUN] would FORCE-DISARM drone0 ... THE AIRCRAFT WILL FALL`.
* On recovery: `{"tracked": true, "age": 0.0045}`. The guard stays latched, as
  designed — an incident does not un-happen.

### Hardware-mode wiring, without hardware

`--hardware --role ground` on the same machine, which starts only the guard:

```
$ ros2 node info /volume_guard | grep uav
    /uav_0/fmu/in/vehicle_command: px4_msgs/msg/VehicleCommand
    /uav_1/fmu/in/vehicle_command: px4_msgs/msg/VehicleCommand
    /uav_2/fmu/in/vehicle_command: px4_msgs/msg/VehicleCommand
```

and the post-bringup preflight correctly went **RED** (`exit 1`, stack left
running) with `dds_link ... no matched endpoint: /uav_0/fmu/in/vehicle_command
(pub=1, sub=0)` — the guard publishing into a namespace with no flight
controller on the other end, which is exactly the truth on a desk.

### RViz — the one thing that was NOT proven

`config_ground_station/o134.rviz` is **unverified against a running RViz.**

The ground-station session's `viz` window comes up and stays up: the
`as2_visualization` launch runs, and `robot_state_publisher` and
`marker_publisher_node` start in every namespace. **`rviz2` itself does not**,
in this environment:

```
[ERROR] [rviz2]: RenderingAPIException: Couldn`t open X display :0
                 in GLXGLSupport::getGLDisplay
```

That is WSLg, not the config: RViz needs a display, and the runs above were
driven from a non-interactive `wsl -- bash <script>` invocation that has no
reachable X session. `QT_QPA_PLATFORM=offscreen` does not help — Ogre's GLX
backend still wants a display — and `Xvfb` is not installed on this machine.
The crash is contained: it takes down neither the rest of the `viz` launch nor
any other session.

What *was* checked is that the file is well-formed and structurally valid:
`Fixed Frame: earth`, seven displays (Grid, TF, three `rviz_default_plugins/Pose`
for `/droneN/self_localization/pose`, two MarkerArrays for `/deckga/markers`
and `/deckga/obstacles`), five tools, an Orbit view, two panels.

**So: open it once from an interactive shell before the lab, and put that on a
card.** `--no-rviz` exists precisely so a display problem cannot stop a bringup.

### What rig R1 does *not* prove

The multirotor simulator integrates its own dynamics while the state
estimator is driven by `fake_mocap.py`. The two are independent, on purpose:
the object under test is the mocap → estimator → platform chain and the
launcher's own arithmetic, not a flight model. **There is no loop closed
around this rig, so do not read a stable hover out of it.** Closed-loop
behaviour is a tether card, not a bench card.

---

## 7. Files this project deliberately does not ship

| Not here | Why |
|---|---|
| a frame-name config for the state estimator | `as2_mocap_guarded` already ships one that is byte-identical to upstream. A third copy is exactly the drift this layout exists to avoid. |
| `world_config` / `uav_config` overrides for the simulator | Rig R1 is not a flight model. Overriding a mass or an initial pose would invite someone to read a result out of it. |
| a per-drone RViz config | `config_ground_station/o134.rviz` carries all three drones' pose displays. A namespace that is not running simply shows nothing. |
| anything Gazebo | See the top of this file. |
