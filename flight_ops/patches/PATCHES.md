# `as2_platform_pixhawk` patches for the O134 indoor rig

This directory carries the local delta that makes `as2_platform_pixhawk` build and behave
correctly against the software actually installed on the O134 ground station, plus two
safety changes that are specific to flying in a netted 8 x 6 x 4 m indoor volume.

| File | Purpose |
| --- | --- |
| `as2_platform_pixhawk_o134.patch` | `git diff` against base commit `2b00b77`. Applies cleanly with `git apply`. |
| `apply_platform_patch.sh` | Clone (if needed) -> checkout base -> apply -> build -> PASS/FAIL. Idempotent. |
| `PATCHES.md` | This file. |

Environment the patch was produced and verified against:

| Component | Version | Source |
| --- | --- | --- |
| Ubuntu | 22.04.5 LTS (WSL2) | - |
| ROS 2 | Humble | apt |
| Aerostack2 / `as2_core` / `as2_msgs` | **1.1.3** (`ros-humble-aerostack2 1.1.3-1jammy`) | apt binary |
| `px4_msgs` | **2.0.1**, branch `release/1.17`, commit `86d8239` | source, built once, not touched by this patch |
| `as2_platform_pixhawk` | base commit `2b00b77dcd2a4e3f7f607ef043bc8cb85e215b88` | source + this patch |
| Target autopilot firmware | PX4 **v1.17.0** | - |

Build command (the two flags are not optional on this machine):

```bash
cd ~/as2_o134_ws
set +u
source /opt/ros/humble/setup.bash
source /opt/ros/humble/share/aerostack2/local_setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select as2_platform_pixhawk --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

* `set +u` — the ROS setup scripts dereference unset variables.
* `-DBUILD_TESTING=OFF` — `ament_lint_auto` is not installed here (`/opt/ros/humble/share/ament_lint_auto` does not exist), so the package's test block cannot configure.

---

## Why this base commit

`as2_platform_pixhawk` has no released version that matches this machine's pair of
dependencies. It is squeezed from both sides:

* **From below (`as2_core`)** — the apt-installed Aerostack2 is **1.1.3**, which is older than
  the platform repo's `main`.
* **From above (`px4_msgs`)** — the message package is **2.0.1** (`release/1.17`), which is
  *newer* than anything the platform repo was written against.

`main` (`fcba062`) is too new for the installed `as2_core`. It calls into `as2::AerialPlatform`
APIs that do not exist in 1.1.3, and uses an `as2_msgs::msg::ControlMode` constant that does not
exist in 1.1.3 either:

```
symbol                    used by fcba062   present in as2_core 1.1.3 headers
getParameter<T>                 7 hits              0 files
getBaseFrameId                  1 hit               0 files
getOdomFrameId                  1 hit               0 files
setCommandPoseFrameId           1 hit               0 files
setCommandTwistFrameId          1 hit               0 files
tf_handler_ (protected member)  1 hit               0 files
ControlMode::BODY_RATES         4 hits              not in as2_msgs 1.1.3 ControlMode.msg
```

`/opt/ros/humble/share/as2_msgs/msg/ControlMode.msg` at 1.1.3 defines `ACRO = 6`; `BODY_RATES`
is the name `main` uses after an upstream rename. Making `main` build would mean either
back-porting `as2_core` or forward-porting five platform APIs — both out of scope, and both
would leave us flying software that does not correspond to any upstream revision.

`2b00b77` ("Merge pull request #29 from aerostack2/add_docstrings") is the **newest** revision
that needs none of those APIs. Against it, the *only* compile errors are the eight `px4_msgs`
2.0.1 field renames listed below. That is a small, auditable delta, which is what we want for
a flight-critical component.

Baseline failure at `2b00b77` before this patch — eight errors, all field renames:

```
src/pixhawk_platform.cpp:430:26: error: ... VehicleAttitudeSetpoint ... has no member named 'pitch_body'
src/pixhawk_platform.cpp:431:26: error: ... VehicleAttitudeSetpoint ... has no member named 'roll_body'
src/pixhawk_platform.cpp:432:26: error: ... VehicleAttitudeSetpoint ... has no member named 'yaw_body'
src/pixhawk_platform.cpp:779:35: error: ... SensorGps ... has no member named 'lat'
src/pixhawk_platform.cpp:780:36: error: ... SensorGps ... has no member named 'lon'
src/pixhawk_platform.cpp:781:35: error: ... SensorGps ... has no member named 'alt_ellipsoid';
                                        did you mean 'altitude_ellipsoid_m'?
src/pixhawk_platform.cpp:815:38: error: ... BatteryStatus ... has no member named 'design_capacity'
src/pixhawk_platform.cpp:827:51: error: ... BatteryStatus ... has no member named 'serial_number'
```

---

## Change 1 — `SensorGps` field renames **and a units change**

**File:** `src/pixhawk_platform.cpp`, `px4GpsCallback()`

**Was**

```cpp
nav_sat_fix_msg.latitude  = msg->lat;
nav_sat_fix_msg.longitude = msg->lon;
nav_sat_fix_msg.altitude  = msg->alt_ellipsoid;
...
nav_sat_fix_msg.latitude  = nav_sat_fix_msg.latitude  / 1e7;
nav_sat_fix_msg.longitude = nav_sat_fix_msg.longitude / 1e7;
nav_sat_fix_msg.altitude  = nav_sat_fix_msg.altitude  / 1e3;
```

**Became**

```cpp
nav_sat_fix_msg.latitude  = msg->latitude_deg;
nav_sat_fix_msg.longitude = msg->longitude_deg;
nav_sat_fix_msg.altitude  = msg->altitude_ellipsoid_m;
```

— the three rescaling lines are **deleted**.

**Why.** This is not a pure rename: the fields also changed type and unit. From
`~/as2_o134_ws/src/px4_msgs/msg/SensorGps.msg` at 2.0.1:

```
float64 latitude_deg          # Latitude in degrees, allows centimeter level RTK precision
float64 longitude_deg         # Longitude in degrees, allows centimeter level RTK precision
float64 altitude_msl_m        # Altitude above MSL, meters
float64 altitude_ellipsoid_m  # Altitude above Ellipsoid, meters
```

| Old field | Old type/unit | New field | New type/unit |
| --- | --- | --- | --- |
| `lat` | `int32`, 1e-7 deg | `latitude_deg` | `float64`, degrees |
| `lon` | `int32`, 1e-7 deg | `longitude_deg` | `float64`, degrees |
| `alt_ellipsoid` | `int32`, mm | `altitude_ellipsoid_m` | `float64`, metres |

`sensor_msgs/NavSatFix` wants degrees and metres, so the new fields are already in the right
units. **Keeping the `/1e7` and `/1e3` would have compiled fine and silently produced garbage**
— a Brisbane latitude of -27.47 deg would have been published as -2.747e-6 deg, i.e. a point
in the Gulf of Guinea. This is the one change in the patch that is a correctness trap rather
than a mechanical rename.

`altitude_ellipsoid_m` is used (not `altitude_msl_m`) to preserve the original code's choice of
the ellipsoidal datum, which is also what `NavSatFix.altitude` is specified as.

---

## Change 2 — `BatteryStatus.design_capacity` removed

**File:** `src/pixhawk_platform.cpp`, `px4BatteryCallback()`

**Was** `battery_msg.design_capacity = msg->design_capacity;`

**Became** the assignment is deleted; `sensor_msgs/BatteryState::design_capacity` is left at its
default (`0.0`), with a comment recording why.

**Why.** `BatteryStatus.msg` at px4_msgs 2.0.1 has no `design_capacity` and no equivalent. It
keeps `uint16 capacity  # [mAh] Capacity of the battery when fully charged` (still assigned, as
before) and adds `state_of_health  # [%] FullChargeCapacity/DesignCapacity`, but design capacity
itself is no longer transmitted. Inventing a value would be worse than leaving the field unset,
so it is left unset.

---

## Change 3 — `BatteryStatus.serial_number` removed

**File:** `src/pixhawk_platform.cpp`, `px4BatteryCallback()`

**Was** `battery_msg.serial_number = std::to_string(msg->serial_number);`

**Became** the assignment is deleted; `BatteryState::serial_number` is left at its default
(empty string), with a comment recording why.

**Why.** Same as above — the field is gone from `BatteryStatus.msg` at 2.0.1 with no
replacement. The closest survivor is `manufacture_date`, which is a date, not an identity, so it
is not substituted.

---

## Change 4 — `VehicleAttitudeSetpoint` Euler fields removed

**File:** `src/pixhawk_platform.cpp`, `resetAttitudeSetpoint()`

**Was**

```cpp
px4_attitude_setpoint_.pitch_body = NAN;
px4_attitude_setpoint_.roll_body  = NAN;
px4_attitude_setpoint_.yaw_body   = NAN;

px4_attitude_setpoint_.q_d = std::array<float, 4>{0, 0, 0, 1};
```

**Became** the three Euler assignments are deleted; `q_d` is untouched.

**Why — and which of the two cases this was.** This was the **"`q_d` is already set elsewhere,
so just remove the redundant Euler assignments"** case. **No quaternion conversion was needed
or added.**

`VehicleAttitudeSetpoint.msg` at 2.0.1 is quaternion-only:

```
uint64 timestamp
float32 yaw_sp_move_rate
float32[4] q_d        # Desired quaternion for quaternion control
float32[3] thrust_body
```

The three removed lines only ever wrote `NAN` into a *reset* function; they never carried a
command. The actual attitude command already goes through `q_d`, built from the commanded ENU
orientation in `ownSendCommand()`, `case ControlMode::ATTITUDE`, which runs immediately after
`resetAttitudeSetpoint()`:

```cpp
Eigen::Quaterniond q_ned      = q_enu_to_ned_ * q_enu;
Eigen::Quaterniond q_aircraft = q_ned * q_baselink_to_aircraft_;
px4_attitude_setpoint_.q_d[0] = q_aircraft.w();   // ... [1]=x, [2]=y, [3]=z
```

So the attitude path is functionally unchanged by this patch. Nothing was lost in the removal:
PX4 v1.13+ had already stopped consuming the Euler fields for multicopter attitude control,
2.0.1 simply deleted the dead fields.

---

## Change 5 (SAFETY) — the kill switch published to a topic PX4 does not subscribe to

**File:** `src/pixhawk_platform.cpp`, `ownKillSwitch()`; publisher + include removed from
`include/as2_platform_pixhawk/pixhawk_platform.hpp`

**Was**

```cpp
RCLCPP_ERROR(this->get_logger(), "KILL SWITCH TRIGGERED");
px4_msgs::msg::ManualControlSwitches kill_switch_msg;
kill_switch_msg.kill_switch = true;
px4_manual_control_switches_pub_->publish(kill_switch_msg);   // -> /fmu/in/manual_control_switches
```

**Became**

```cpp
RCLCPP_ERROR(this->get_logger(), "KILL SWITCH TRIGGERED");
static constexpr float PX4_FORCE_DISARM_MAGIC = 21196.0f;
PX4publishVehicleCommand(
  px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
  static_cast<float>(px4_msgs::msg::VehicleCommand::ARMING_ACTION_DISARM),  // param1 = 0
  PX4_FORCE_DISARM_MAGIC);                                                 // param2 = 21196
```

The now-unused `px4_manual_control_switches_pub_` member, its `create_publisher` call and the
`<px4_msgs/msg/manual_control_switches.hpp>` include are removed so nobody can mistake the old
path for a live one. The `RCLCPP_ERROR("KILL SWITCH TRIGGERED")` line is unchanged.

**Why.** `/fmu/in/manual_control_switches` is **not** a uXRCE-DDS subscription in PX4 v1.17, so
the message left the ground station and was dropped on the floor. The kill switch was a silent
no-op. Verified directly against
`PX4-Autopilot/v1.17.0/src/modules/uxrce_dds_client/dds_topics.yaml`: its `subscriptions:`
section has 38 entries, `manual_control_switches` is not among them, while
`/fmu/in/vehicle_command` and `/fmu/in/vehicle_visual_odometry` both are. (`manual_control_input`
is present, but that is `ManualControlSetpoint` — stick axes — not the switches message, and it
would be a per-cycle RC override stream rather than a one-shot kill.)

**Why force-disarm rather than flight termination.** Both were checked in
`~/as2_o134_ws/src/px4_msgs/msg/VehicleCommand.msg`:

```
uint16 VEHICLE_CMD_DO_FLIGHTTERMINATION   = 185
uint16 VEHICLE_CMD_COMPONENT_ARM_DISARM   = 400
int8   ARMING_ACTION_DISARM               = 0
int8   ARMING_ACTION_ARM                  = 1
```

`VEHICLE_CMD_COMPONENT_ARM_DISARM` and `ARMING_ACTION_DISARM` both exist, so the preferred
force-disarm path is available and is what the patch uses. Confirmed in PX4 v1.17.0
`src/modules/commander/Commander.cpp`:

```cpp
const int8_t arming_action = static_cast<int8_t>(lroundf(cmd.param1));
const bool   forced        = (static_cast<int>(lroundf(cmd.param2)) == 21196);
...
} else if (arming_action == vehicle_command_s::ARMING_ACTION_DISARM) {
    arming_res = disarm(arm_disarm_reason, forced);
}
```

`forced == true` is what makes PX4 skip the "must be landed" guard and cut the motors in the
air, which is the whole point of a kill switch. `21196` is a bare literal inside Commander; it
is **not** exported as a constant by `px4_msgs` 2.0.1's `VehicleCommand.msg`, hence the local
`PX4_FORCE_DISARM_MAGIC` constant. The `VEHICLE_CMD_DO_FLIGHTTERMINATION` fallback (param1 > 0.5
in the same file) was therefore not needed; it is also a latched, parameter-dependent state that
usually deploys a chute/servo termination and is harder to recover from on a small indoor
quadrotor.

`target_system` / `target_component` / `source_system` / `source_component` / `from_external` are
not set by hand: the call goes through the existing `PX4publishVehicleCommand()` helper, which
already fills `target_system = target_system_id_`, `target_component = 1`, `source_system = 1`,
`source_component = 1`, `from_external = true` for every arm, disarm and offboard command this
platform sends. Same pattern, same routing.

---

## Change 6 (SAFETY, "S3") — visual-odometry staleness gate

**Files:** `src/pixhawk_platform.cpp` (`externalOdomCb`, new `isExternalOdomFresh`,
`PX4publishVisualOdometry`, constructor), `include/as2_platform_pixhawk/pixhawk_platform.hpp`,
`config/platform_config_file.yaml`

**Was.** When `external_odom` is true, a 100 Hz `create_wall_timer` calls
`PX4publishVisualOdometry()`, which copies `odometry_msg_` (pose from TF, twist from
`self_localization/twist`) into a `VehicleOdometry`, stamps it with
`this->get_clock()->now()`, and publishes to `/fmu/in/vehicle_visual_odometry`. There was **no
freshness check anywhere on that path**.

**The hazard.** The timer re-stamps every message with `now()`, so PX4 cannot tell a live pose
from a dead one. If motion capture drops out, `odometry_msg_` freezes at its last value and the
platform keeps shipping that frozen pose at 100 Hz with *fresh* timestamps. The EKF sees a
healthy, perfectly stationary vision source, no external-vision timeout fires, no failsafe runs
— and the aircraft flies blind while believing it is holding station. In an 8 x 6 x 4 m netted
volume that is a net or wall strike in roughly a second.

**Became.**

* New member state, and a new parameter read in the constructor next to the other parameter
  declarations:

  ```cpp
  this->declare_parameter<double>("external_odom_timeout_s", 0.1);
  external_odom_timeout_s_ = this->get_parameter("external_odom_timeout_s").as_double();
  ```

  Declared **with** a default (0.1 s = 10 timer cycles at 100 Hz) so existing config files keep
  working; the other platform parameters are declared without defaults and are therefore
  mandatory, which would have broken every existing launch. The value is also echoed at
  start-up next to the other `RCLCPP_INFO` banner lines, and added to
  `config/platform_config_file.yaml` for discoverability.

* `externalOdomCb()` stamps the arrival of each usable sample:

  ```cpp
  last_external_odom_time_ = this->get_clock()->now();
  external_odom_received_  = true;
  ```

* `PX4publishVisualOdometry()` returns early — **publishing nothing at all** — when the gate says
  stale. Nothing else about the message contents or the 100 Hz timer rate changed.

* `isExternalOdomFresh()` does the check and the edge-triggered logging:

  ```cpp
  bool fresh = false;
  if (external_odom_received_) {
    const double age = (this->get_clock()->now() - last_external_odom_time_).seconds();
    fresh = (age >= 0.0) && (age < external_odom_timeout_s_);
  }
  if (!fresh && !external_odom_stale_) {          // fresh -> stale edge
    external_odom_stale_ = true;
    RCLCPP_ERROR(...);
  } else if (fresh && external_odom_stale_) {     // stale -> fresh edge
    external_odom_stale_ = false;
    RCLCPP_INFO(...);
  }
  return fresh;
  ```

**Why stop publishing rather than publish something.** Stopping is the *only* action that gets
PX4 to help. PX4's EKF declares an external-vision timeout when the stream dries up and runs its
own failsafe; publishing a held pose, or a zeroed pose, or a NAN pose keeps the source looking
alive (or actively poisons the estimate). The correct behaviour for a source we no longer trust
is to fall silent and let the autopilot's own failsafe machinery take over.

**Two deliberate implementation details.**

1. `external_odom_stale_` is **initialised to `true`.** Before the first sample ever arrives the
   node is, correctly, in the stale state and publishes nothing — but it does not log an ERROR at
   start-up. The first log you see is the `RCLCPP_INFO` recovery line when real odometry starts
   flowing. This also fixes a latent bug at the base commit: before the first callback,
   `odometry_msg_` is default-constructed, and the old code cheerfully streamed that
   all-zeros pose (with an all-zero, non-unit quaternion) to the EKF at 100 Hz.

2. The arrival stamp is taken **inside the `try` block**, after `odometry_msg_` has actually been
   updated — not on bare message receipt. This is a small, deliberate strengthening of the
   requirement. If the TF lookup throws (`tf_handler_->getState`), `odometry_msg_` keeps its
   previous frozen contents even though messages are still arriving on the topic — exactly the
   hazard the gate exists to catch. Stamping on receipt would have left that case open. In the
   normal path (TF healthy) the two are identical.

Logging is strictly edge-triggered: exactly one `RCLCPP_ERROR` on entering stale and one
`RCLCPP_INFO` on recovery, never once per 10 ms cycle.

---

## Verification performed

**Build** — green, zero warnings:

```
Starting >>> as2_platform_pixhawk
Finished <<< as2_platform_pixhawk [39.7s]
Summary: 1 package finished [39.9s]
```

`install/as2_platform_pixhawk/lib/as2_platform_pixhawk/as2_platform_pixhawk_node` exists.

**Patch hygiene** — the patch was re-applied to a fresh clone checked out at `2b00b77`;
`git apply --check` passes and the resulting tree is byte-identical (`diff -r`) to the tree the
patch was cut from.

**Node start-up and the new parameter** — run in the `/drone0` namespace with the package config
plus `-p external_odom:=true -p external_odom_timeout_s:=0.1`:

```
[INFO] [drone0.platform]: Max thrust: 15.000000
[INFO] [drone0.platform]: Min thrust: 0.150000
[INFO] [drone0.platform]: Simulation mode: false
[INFO] [drone0.platform]: External odometry mode: true
[INFO] [drone0.platform]: External odometry timeout: 0.100 s
[INFO] [drone0.platform]: FMU prefix:
```

```
$ ros2 param get /drone0/platform external_odom_timeout_s
Double value is: 0.1
$ ros2 param describe /drone0/platform external_odom_timeout_s
  Type: double
```

**Staleness gate, end to end** — with static transforms `earth -> drone0/odom -> drone0/base_link`
and a synthetic 50 Hz twist on `/drone0/self_localization/twist`:

| Phase | `ros2 topic hz /fmu/in/vehicle_visual_odometry` |
| --- | --- |
| before any external odometry | no messages (gate closed from start-up) |
| while twist is flowing | `average rate: 99.303` (unchanged 100 Hz timer) |
| after the twist is stopped | no messages |

and exactly two log lines across the whole run:

```
[INFO]  External odometry recovered: resuming /fmu/in/vehicle_visual_odometry
[ERROR] EXTERNAL ODOMETRY STALE (no sample for more than 0.100 s): stopped publishing to
        /fmu/in/vehicle_visual_odometry so PX4 can run its external vision failsafe
```

## Not verified — needs the aircraft

Everything below is reasoned from the PX4 v1.17.0 source and the message definitions, and has
**not** been exercised against a real flight controller. All of it belongs on the bench-test
card before the first armed flight:

* **Force-disarm actually cuts the motors in the air.** The command path is verified as far as
  "the topic is a real v1.17 subscription and Commander parses param2 == 21196 as forced". That
  the FMU disarms while airborne, and how fast, needs a props-off bench test and then a tethered
  or low-hover test. Note the kill is now a *disarm*, not a mode change: the aircraft drops.
* **GPS scaling.** No GNSS receiver has been connected, so `latitude_deg` / `longitude_deg` /
  `altitude_ellipsoid_m` have never carried a real fix through this code. The unit reasoning
  comes from `SensorGps.msg`. Indoors this path is unused, but confirm it before any outdoor
  (SERF) session.
* **The 0.1 s timeout value.** Chosen as 10 cycles of the existing 100 Hz timer. Whether the real
  OptiTrack -> `self_localization/twist` -> TF chain has jitter that would trip a 100 ms gate
  spuriously is a measurement, not a derivation. Log the inter-arrival distribution on the bench
  and raise the parameter if needed — it is a runtime parameter for exactly this reason.
* **PX4's response to the stream stopping.** That the EKF raises its external-vision timeout and
  the configured failsafe fires (and which failsafe) depends on `EKF2_EV_*` and `COM_*`
  parameters in `flight_ops/lab_config/px4_indoor_params.yaml`. Verify in SITL or on the bench.
* **Battery fields.** `design_capacity` and `serial_number` are now always empty on
  `sensor_msgs/BatteryState`. Nothing in this stack was found to read them, but any downstream
  consumer or log analysis that did will now see defaults.
