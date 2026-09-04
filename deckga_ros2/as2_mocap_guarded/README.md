# `as2_mocap_guarded` — hardened mocap state estimator (S1)

A separate ROS 2 package that exports a pluginlib plugin, `mocap_pose_guarded::Plugin`,
for the **stock** Aerostack2 `as2_state_estimator` node. It is a hardened fork of that
package's `mocap_pose` plugin (`as2_state_estimator` 1.1.3,
`/opt/ros/humble/include/mocap_pose.hpp`).

**Nothing in the installed `as2_state_estimator` is modified.** The two plugins coexist;
`plugin_name` decides which one loads. The names are deliberately far apart so a config
file or a log line can never be ambiguous about which is running.

Everything the original did is preserved: the mocap sample is treated as already-ENU with
no axis conversion, `map->odom` is always identity, `odom->base_link` is broadcast per
sample, `self_localization/pose` carries the raw sample in the `earth` frame with the
node's own timestamp, and twist is a smoothed finite difference rotated into the body
frame. Five defects are fixed and one health topic is added.

---

## The defects

### D1 — origin pose on name mismatch

`rigid_bodies_callback()` scans `msg->rigidbodies` for `rigid_body_name`, then calls
`process_mocap_pose(pose_msg)` **unconditionally, outside the loop**:

```cpp
auto pose_msg = geometry_msgs::msg::PoseStamped();
pose_msg.header = msg->header;
for (const auto & rigid_body : msg->rigidbodies) {
  if (rigid_body.rigid_body_name == rigid_body_name_) {
    pose_msg.pose = rigid_body.pose;
    break;
  }
}
process_mocap_pose(pose_msg);   // <-- runs whether or not anything matched
```

`geometry_msgs/Quaternion.msg` defaults to `w = 1`, so on a miss `pose_msg` is a perfectly
well-formed pose at position (0, 0, 0) with identity orientation. It is published at full
mocap rate with no warning of any kind. In a netted 8 × 6 × 4 m volume, a drone that
believes it is at the origin flies into the net, and the log says nothing.

**What changed.** A miss returns immediately: no pose, no transform, no twist, ever. The
plugin never synthesises a pose. A throttled `RCLCPP_ERROR` names the body it wanted **and
lists every name actually present in the array** — that list is what turns a 40-minute lab
mystery into a 10-second fix, because the typo is then visible side by side with the
correct spelling. The miss count is also on `mocap_health`.

### D2 — frame origin set by a startup race

`earth_to_map_` is latched from the **first message ever received** and never revised
(`has_earth_to_map_`). Whether the estimator node starts before or after Motive acquires
the body decides whether `earth->map` is identity or that aircraft's resting pose,
*including its yaw*. With three drones started independently you can get three different
world origins in one session, silently. Upstream flags it itself:

```
// TODO(javilinos): MODIFY this to a initial earth to map transform
// (reading initial position from parameters or msgs )
```

**What changed.** `earth->map` is built from parameters (`earth_to_map_x/y/z/yaw`),
**defaulting to identity**, and the static transform is published once in `on_setup()`
before a single mocap sample has been seen. `earth` = `map` = the lab frame,
deterministically, for every drone, regardless of start order. `map->odom` is still
published as identity exactly as before. `get_earth_to_map_transform()` is also overridden
to report the configured value instead of the base class's "warn and return identity".

### D3 — velocity noise, and `static` locals in a member function

Twist is a raw finite difference of position and `twist_smooth_filter_cte` defaults to
`1.0`, which means the filter is **off**. At the O-134 rig's ~2 mm 1σ static noise and
100 Hz:

```
sigma_v = sigma_p * sqrt(2) * f_s = 0.002 * 1.414 * 100  ~=  0.28 m/s
```

0.28 m/s of pure noise into a speed controller.

The original also declares `last_pose` and the body-frame `twist_msg` as `static` locals
**inside a member function**. A function-local static is shared by every instance in the
process and initialised exactly once, on the first call ever — so two estimators in one
component container would differentiate each other's positions.

**What changed.** The statics are per-instance members (`last_position_`,
`has_last_position_`, `twist_body_msg_`), and the first accepted sample now yields exactly
zero velocity by construction rather than by accident. The default
`twist_smooth_filter_cte` is **0.2**:

| α | noise gain √(α/(2−α)) | σ_v | −3 dB corner ≈ f_s·α/2π | group delay ≈ (1−α)/(α·f_s) |
|---|---|---|---|---|
| 1.0 (upstream) | 1.000 | 0.28 m/s | — | 0 ms |
| **0.2 (here)** | **0.333** | **0.094 m/s** | **3.2 Hz** | **40 ms** |
| 0.1 | 0.229 | 0.065 m/s | 1.6 Hz | 90 ms |

A multirotor position/velocity loop runs at roughly 0.5–1.5 Hz bandwidth. A 3.2 Hz corner
passes the whole control-relevant band while cutting differentiation noise threefold, and
40 ms of lag is small against the loop period. α = 0.1 halves the noise again but puts a
1.6 Hz corner and 90 ms of lag *inside* the position loop, where it shows up as sluggish
velocity tracking. 0.2 is the compromise. Set `1.0` to reproduce upstream exactly.

`orientation_smooth_filter_cte` is deliberately **left at 1.0**: the original blends
quaternion components linearly without renormalising, so any value below 1.0 emits a
non-unit quaternion. Keeping the default at 1.0 makes that blend an exact no-op; fixing it
properly means slerp, which is a behaviour change beyond this fork's scope.

### D4 — no input validation

The original used whatever Motive/VRPN emitted.

**What changed.** Before use, every sample is checked for non-finite position/orientation
components and for `| ‖q‖ − 1 | > quaternion_tolerance` (default 1e-3, which also catches
the all-zero quaternion a partially-solved marker set produces). Failures are counted by
category, logged throttled with the offending values, and **dropped**.

A rejected sample is **never** replaced by the last good pose. That is the point: bad data
must age out exactly like no data, so the `age`/`tracked` fields on `mocap_health` are the
single source of truth about liveness. Substituting the last good pose would make a dead
feed look alive — the same class of lie as D1.

### D5 — silent QoS incompatibility (found by running it, not by reading it)

The original subscribes with `rclcpp::QoS(10)`, which is **RELIABLE**. Both mocap4r2's own
drivers and this project's `flight_ops/nodes/vrpn_to_rigidbodies.py` publish
`/mocap/rigid_bodies` **BEST_EFFORT**, which is the ROS convention for a 100 Hz sensor
stream. A reliable subscriber and a best-effort publisher are incompatible: DDS matches
nothing, not one message is delivered, and the estimator sits there looking healthy with an
empty pose topic. Only the *publisher* logs anything, and only if you happen to be reading
its console.

**What changed.** `mocap_qos_reliability` is a parameter, defaulting to `best_effort` so it
matches the sources this stack actually has, and an incompatible-QoS event callback turns
the silence into a named `RCLCPP_ERROR` on the estimator side. Set `reliable` to reproduce
upstream behaviour.

---

## Health topic

`/{namespace}/mocap_health` — **`std_msgs/String` carrying a JSON object**, published on a
**timer** (default 2 Hz), QoS reliable + transient-local.

Why `std_msgs/String` + JSON rather than `diagnostic_msgs/DiagnosticStatus`:

1. **`diagnostic_msgs` is not installed** in this ROS 2 Humble image — there is no
   `/opt/ros/humble/share/diagnostic_msgs`. Depending on it would make the package
   unbuildable on the flight machines, which is exactly the class of surprise this fork
   exists to remove.
2. `DiagnosticStatus` degenerates to a `KeyValue[]` of stringified numbers anyway, so it
   buys type safety it does not actually deliver.
3. The consumers already speak JSON. `preflight_check.py` reads mocap messages by duck
   typing rather than importing message packages, and `fake_mocap.py` already publishes its
   fault timeline as JSON on `/fake_mocap/status`. One decoder covers both.
4. `ros2 topic echo /drone0/mocap_health` is readable on a bench laptop with nothing extra
   built.

**Timer-driven, not callback-driven**, on purpose: a health topic fed from the data
callback goes silent exactly when the thing it monitors fails. This one keeps reporting
`tracked: false` and a growing `age` right through a total mocap outage. Transient-local
durability means a monitor attaching late gets the current state immediately.

```json
{
  "plugin": "mocap_pose_guarded",
  "stamp": 1788485331.950931,
  "rigid_body_name": "drone0",
  "mocap_topic": "/mocap/rigid_bodies",
  "mocap_qos_reliability": "best_effort",
  "tracked": false,
  "age": -1.0,
  "timeout": 0.25,
  "messages": 864,
  "accepted": 0,
  "name_misses": 864,
  "rejects": 0,
  "rejects_non_finite": 0,
  "rejects_quaternion": 0,
  "names_seen": "'drone0_typo', 'drone1', 'drone2'",
  "earth_to_map": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
  "frames": {"earth": "earth", "map": "drone0/map",
             "odom": "drone0/odom", "base": "drone0/base_link"}
}
```

| Field | Meaning |
|---|---|
| `tracked` | `true` iff a sample has been accepted and `age <= timeout` |
| `age` | seconds since the last **accepted** sample; `-1.0` if there has never been one |
| `timeout` | the `mocap_timeout` parameter, so a consumer need not be configured separately |
| `messages` / `accepted` | RigidBodies messages received / samples that produced a pose |
| `name_misses` | D1 counter — configured body absent from the array |
| `rejects`, `rejects_non_finite`, `rejects_quaternion` | D4 counters |
| `names_seen` | body names present at the most recent name miss; the typo, spelled out |
| `earth_to_map`, `frames` | what this instance is actually using, for cross-drone comparison |

`age` and `name_misses` distinguish the three ways to have no pose, which look identical
from outside: the feed is down (`messages` static), the name is wrong (`name_misses`
climbing), or the data is malformed (`rejects` climbing).

---

## How the plugin and its config are resolved — verified, not assumed

**Class name.** The stock node builds its pluginlib lookup name as
`<plugin_name parameter> + "::Plugin"` (confirmed by the adjacent `plugin_name` /
`::Plugin` string literals in `libas2_state_estimator.so`) and constructs

```cpp
pluginlib::ClassLoader<as2_state_estimator_plugin_base::StateEstimatorBase>(
    "as2_state_estimator", "as2_state_estimator_plugin_base::StateEstimatorBase")
```

That ClassLoader scans the ament resource index
`share/ament_index/resource_index/as2_state_estimator__pluginlib__plugin/` across **every**
prefix on `AMENT_PREFIX_PATH`. `pluginlib_export_plugin_description_file(as2_state_estimator
plugins.xml)` in this package's `CMakeLists.txt` registers an entry there, so
`plugin_name: "mocap_pose_guarded"` resolves to `mocap_pose_guarded::Plugin` in
`libmocap_pose_guarded.so` **without touching the installed `as2_state_estimator`**. The
CMake target name is load-bearing: `plugins.xml` says `<library path="mocap_pose_guarded">`,
which pluginlib turns into `libmocap_pose_guarded.so`.

**Config file.** This is where a plugin in a different package comes unstuck, and it is why
this package ships its own launch file. Reading
`/opt/ros/humble/share/as2_state_estimator/launch/state_estimator_launch.py`, the stock
launch file cannot load an out-of-package plugin for two independent reasons:

1. `plugin_name` is declared with `choices=get_available_plugins('as2_state_estimator')`,
   and `as2_core.launch_plugin_utils.get_available_plugins()` parses **only**
   `<as2_state_estimator share>/plugins.xml`.
2. The plugin default config path is hard-wired to
   `<as2_state_estimator share>/plugins/<plugin_name>/config/plugin_default.yaml`, and
   `as2_core.LaunchConfigurationFromConfigFile.perform()` `open()`s that path
   unconditionally — even when the user passes `plugin_config_file:=...`.

Both were confirmed by running them:

```
$ ros2 launch as2_state_estimator state_estimator_launch.py plugin_name:=mocap_pose_guarded namespace:=drone0
[ERROR] [launch.actions.declare_launch_argument]: Argument "plugin_name" provided value
"mocap_pose_guarded" is not valid. Valid options are:
['raw_odometry', 'ground_truth', 'mocap_pose', 'ground_truth_odometry_fuse', '']

$ ros2 launch as2_state_estimator state_estimator_launch.py config_file:=/tmp/s1_cfg.yaml namespace:=drone0
   # /tmp/s1_cfg.yaml contains plugin_name: "mocap_pose_guarded"
[ERROR] [launch]: Caught exception in launch (see debug for traceback): [Errno 2] No such
file or directory:
'/opt/ros/humble/share/as2_state_estimator/plugins/mocap_pose_guarded/config/plugin_default.yaml'
```

The second one is worth noting: the config-file route gets *past* the `choices` check,
because `override_plugin_name_in_context()` writes straight into
`context.launch_configurations` after the argument has already been validated as `''`. It
then dies on the missing yaml instead. Neither path works.

So this package ships `launch/mocap_pose_guarded_state_estimator.launch.py`, which launches
**the same stock node** (`package='as2_state_estimator'`,
`executable='as2_state_estimator_node'`) with `plugin_name` and the two yaml files supplied
directly. `plugin_name` is set in the launch file rather than in a yaml so an inherited
config can never silently swap it back to the stock `mocap_pose`.

### The exact working launch invocation

```bash
set +u
source /opt/ros/humble/setup.bash
source ~/as2_o134_ws/install/setup.bash

ros2 launch as2_mocap_guarded mocap_pose_guarded_state_estimator.launch.py \
    namespace:=drone0 \
    rigid_body_name:=drone0
```

`rigid_body_name` has no usable default and the plugin throws at setup if it is empty —
a wrong or missing name is D1, so refusing to start is the safe outcome.

Per-drone, with everything spelled out:

```bash
ros2 launch as2_mocap_guarded mocap_pose_guarded_state_estimator.launch.py \
    namespace:=drone1 \
    rigid_body_name:=drone1 \
    mocap_topic:=/mocap/rigid_bodies \
    mocap_qos_reliability:=best_effort \
    twist_smooth_filter_cte:=0.2 \
    earth_to_map_x:=0.0 earth_to_map_y:=0.0 earth_to_map_z:=0.0 earth_to_map_yaw:=0.0 \
    log_level:=info
```

Or with a per-drone yaml:

```bash
ros2 launch as2_mocap_guarded mocap_pose_guarded_state_estimator.launch.py \
    namespace:=drone1 plugin_config_file:=/path/to/drone1_mocap.yaml
```

Launching the node directly, without the launch file, also works — this is what the launch
file reduces to:

```bash
ros2 run as2_state_estimator as2_state_estimator_node --ros-args \
    -r __ns:=/drone0 -r __node:=state_estimator \
    -p plugin_name:=mocap_pose_guarded \
    --params-file $(ros2 pkg prefix as2_mocap_guarded)/share/as2_mocap_guarded/config/state_estimator_default.yaml \
    --params-file $(ros2 pkg prefix as2_mocap_guarded)/share/as2_mocap_guarded/plugins/mocap_pose_guarded/config/plugin_default.yaml \
    -p rigid_body_name:=drone0
```

### Build

```bash
set +u
source /opt/ros/humble/setup.bash
cd ~/as2_o134_ws
PYTHONNOUSERSITE=1 colcon build --packages-select as2_mocap_guarded \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

---

## Parameters

New parameters are marked **NEW**. Everything else keeps its upstream name; only the
default of `twist_smooth_filter_cte` changed.

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `mocap_topic` | string | `"/mocap/rigid_bodies"` | `mocap4r2_msgs/RigidBodies` input. Upstream had no code default and threw if unset. |
| `rigid_body_name` | string | `""` → **throws** | Must match the Motive name exactly, case and punctuation included. No usable default on purpose (D1). |
| `mocap_qos_reliability` | string | `"best_effort"` | **NEW** (D5). `best_effort` \| `reliable` \| `system_default`. Upstream is hard-wired `reliable`. |
| `earth_to_map_x` | double | `0.0` | **NEW** (D2). metres |
| `earth_to_map_y` | double | `0.0` | **NEW** (D2). metres |
| `earth_to_map_z` | double | `0.0` | **NEW** (D2). metres |
| `earth_to_map_yaw` | double | `0.0` | **NEW** (D2). radians. All four zero ⇒ `earth->map` identity. |
| `twist_smooth_filter_cte` | double | **`0.2`** | (D3) Upstream default was `1.0` = filter off. `(0, 1]`. |
| `orientation_smooth_filter_cte` | double | `1.0` | Unchanged. Left off deliberately — the blend does not renormalise. |
| `quaternion_tolerance` | double | `0.001` | **NEW** (D4). `\| ‖q‖ − 1 \|` above this ⇒ sample dropped. |
| `mocap_timeout` | double | `0.25` | **NEW**. Seconds without an accepted sample before `tracked: false`. |
| `mocap_health_topic` | string | `"mocap_health"` | **NEW**. Relative ⇒ `/{namespace}/mocap_health`. |
| `mocap_health_rate` | double | `2.0` | **NEW**. Hz. Timer-driven, so it survives mocap silence. |
| `log_throttle_period` | double | `2.0` | **NEW**. Seconds between repeated fault log lines. |

Inherited unchanged from `as2_state_estimator`: `base_frame` (`base_link`),
`global_ref_frame` (`earth`), `odom_frame` (`odom`), `map_frame` (`map`). All but
`global_ref_frame` are namespaced into `<namespace>/<frame>`.

Every new parameter is declared by the plugin when absent, so a yaml that predates this
fork still loads and simply gets the defaults above. Numeric parameters written as YAML
integers (`twist_smooth_filter_cte: 1`) are accepted — upstream's `as_double()` threw on
them.

---

## Verification

All of it run in WSL Ubuntu 22.04, ROS 2 Humble, `ROS_DOMAIN_ID=77`. Stimulus for the name
mismatch is `flight_ops/nodes/fake_mocap.py --mode rigidbodies --fault name_mismatch`,
which renames the first body to `drone0_typo` and leaves the others alone.

### Build

```
Starting >>> as2_mocap_guarded
Finished <<< as2_mocap_guarded [19.6s]
Summary: 1 package finished [20.0s]
```

### The plugin loads in the stock node

```
[INFO] [as2_state_estimator_node-1]: process started with pid [439]
[as2_state_estimator_node-1] [INFO] [drone0.state_estimator]: Construct with name [state_estimator]
[as2_state_estimator_node-1] [INFO] [drone0.state_estimator]: mocap_pose_guarded: tracking
  rigid body 'drone0' on '/mocap/rigid_bodies'; earth->map = [x 0.000 y 0.000 z 0.000
  yaw 0.000] (IDENTITY) from parameters, NOT from the first sample; subscription
  reliability best_effort; twist_smooth_filter_cte 0.200; quaternion_tolerance 1.0e-03;
  mocap_timeout 0.250 s; health on 'mocap_health' at 2.0 Hz
```

No `pluginlib`, `LibraryLoadException` or `CreateClassException` anywhere in the log.

### D1 — name mismatch publishes nothing

Same stimulus, same 6-second window, both plugins:

| | `self_localization/pose` | first pose | estimator log |
|---|---|---|---|
| **stock `mocap_pose`** | **600 msgs, 100.0 Hz** | **(0.0, 0.0, 0.0)** | **0 lines matching error/warn/mismatch** |
| **`mocap_pose_guarded`** | **0 msgs, 0.0 Hz** | — | throttled `MOCAP NAME MISMATCH` every 2 s |

Guarded `mocap_health` during the fault — 864 messages in, 864 misses, nothing accepted,
and the names that were actually there:

```json
{"tracked": false, "age": -1.0, "messages": 864, "accepted": 0, "name_misses": 864,
 "rejects": 0, "names_seen": "'drone0_typo', 'drone1', 'drone2'"}
```

```
[ERROR] [drone0.state_estimator]: MOCAP NAME MISMATCH: rigid body 'drone0' is NOT in the
'/mocap/rigid_bodies' message. Bodies actually present: ['drone0_typo', 'drone1',
'drone2']. Message DROPPED -- no pose, no transform, no twist published (upstream
mocap_pose would have published the ORIGIN here). Misses so far: 604 of 604 messages.
Fix `rigid_body_name` or the name in Motive; they must match exactly, including case and
quoting.
```

Control run, correct name, same window: **600 pose messages at 100.0 Hz**, `tracked: true`,
`age: 0.0038`, 838 accepted, 0 misses, 0 rejects. The drop is specific to the fault, not a
broken test rig.

### D2 — `earth->map` is identity regardless of start order

Body on a 2 m circle, 20 s period, so the pose at attach time is a strong function of when
the estimator started. `ros2 run tf2_ros tf2_echo earth <ns>/map`:

| plugin | order | translation | rotation (RPY deg) |
|---|---|---|---|
| stock `mocap_pose` | estimator first, mocap second | `[2.000, 0.007, 1.500]` | `[0, 0, +90.201]` |
| stock `mocap_pose` | mocap first, estimator second | `[-2.000, -0.019, 1.500]` | `[0, 0, -89.452]` |
| **`mocap_pose_guarded`** | estimator first, mocap second | **`[0.000, 0.000, 0.000]`** | **`[0, 0, 0]`** |
| **`mocap_pose_guarded`** | mocap first, estimator second | **`[0.000, 0.000, 0.000]`** | **`[0, 0, 0]`** |

Two identical stock configurations, differing only in process start order, produce world
origins 4 m apart and 180° rotated. The guarded plugin is identity in both, quaternion
`[0, 0, 0, 1]`, from parameters, before any data arrives.

### D4 — malformed samples are dropped, not substituted

`fake_mocap.py --fault bad_quaternion` (‖q‖ = 1.7):

```json
{"tracked": false, "age": -1.0, "messages": 854, "accepted": 0,
 "name_misses": 0, "rejects": 854, "rejects_non_finite": 0, "rejects_quaternion": 854}
```

0 poses published. `age` stays at `-1.0` rather than freezing at a stale value — the
sample is dropped, not replaced.

```
[ERROR] MOCAP SAMPLE REJECTED for 'drone0': non-unit quaternion (pos 1.7398 0.1890
-0.7176, quat 0.000000 0.000000 1.265317 1.135330, |q| 1.700000, tolerance 1.0e-03).
Sample DROPPED and NOT replaced by the last good pose, so it ages out like no data.
Rejects so far: 0 non-finite, 202 non-unit-quaternion.
```

### D5 — QoS mismatch is named instead of silent

Guarded plugin forced to `mocap_qos_reliability:=reliable` against `fake_mocap.py`'s
best-effort publisher — the configuration the stock plugin is permanently in:

```
[ERROR] [d_grd_qos.state_estimator]: MOCAP QoS INCOMPATIBLE on '/mocap/rigid_bodies': this
subscription's QoS cannot match the publisher (offending policy: RELIABILITY; 1
incompatible publisher(s) total). NO messages will ever be delivered on this topic. Set
the 'mocap_qos_reliability' parameter (best_effort|reliable|system_default) to match the
source.
```

`mocap_health` shows `messages: 0` — the estimator never saw one byte. With the default
`best_effort`, the same setup delivers 100 Hz.

---

## Layout

```
as2_mocap_guarded/
├── CMakeLists.txt
├── package.xml
├── plugins.xml                                  # mocap_pose_guarded::Plugin
├── include/as2_mocap_guarded/
│   └── mocap_pose_guarded.hpp                   # the plugin, inline as upstream
├── src/
│   └── mocap_pose_guarded.cpp                   # PLUGINLIB_EXPORT_CLASS only
├── config/
│   └── state_estimator_default.yaml             # frame names
├── plugins/mocap_pose_guarded/config/
│   └── plugin_default.yaml                      # plugin parameters
└── launch/
    └── mocap_pose_guarded_state_estimator.launch.py
```

`plugins/mocap_pose_guarded/config/plugin_default.yaml` mirrors the directory layout the
stock `state_estimator_launch.py` expects, so the same relative path would work unchanged
if the plugin were ever vendored into `as2_state_estimator` itself.

The workspace copy at `~/as2_o134_ws/src/as2_mocap_guarded/` and this copy are identical.
