# flight_ops/deploy — putting the stack on the three Jetsons

The ground station is finished. What is left is: *get ROS 2 and the flight
software onto the Jetson Orin NX on each of the three Holybro X500 V2s.*

This directory turns that into a scripted, verifiable procedure that produces
the same result three times, instead of three afternoons of improvisation.

> **Governing rule, inherited from `flight_ops/README.md`:**
> *No event happens for the first time inside O-134.* Everything here is bench
> work. A Jetson does not go on an airframe until `verify_jetson.sh` prints
> **`JETSON GREEN`**.

Read [Verified / Unverified](#verified--unverified) before you trust any of it.
Roughly half of this kit has been executed for real; the half that needs a
Jetson in front of it has not, because there was none available when it was
written.

---

## 1. What runs where

This is the single most important thing in this document. Get it wrong and
you will spend a lab session debugging a topology problem instead of flying.

**Three machine classes are involved:**

| | |
|---|---|
| **FC** | Pixhawk on each airframe, PX4 v1.17.0. Not a deployment target — it is flashed, not installed to. |
| **Jetson** | Orin NX on each airframe, user `jetson`, one known at `10.88.51.230`. **The subject of this kit.** |
| **GS** | The ground station: Ubuntu 22.04 in WSL2 on the operator's PC. Already set up. |

### The table

| Component | Runs on | Why there and not elsewhere |
|---|---|---|
| PX4 v1.17.0, `uxrce_dds_client` | **FC** | `UXRCE_DDS_CFG=1000` (Ethernet), `AG_IP=170461697` (=10.41.10.1), `PRT=8888`, `KEY=1`, `DOM_ID=0`. |
| `MicroXRCEAgent udp4 -p 8888` | **each Jetson** | Must sit at the far end of the point-to-point Ethernet link the FC dials into. Running it on the GS would put the FC's 100+ Hz telemetry across wifi with a UDP session that has no reconnect story. No sudo needed. |
| `as2_platform_pixhawk` (patched) | **each Jetson** | It is the thing that publishes `/fmu/in/vehicle_visual_odometry` at 100 Hz and consumes `/fmu/out/*`. Co-locating it with the agent keeps that loop inside one machine. Its kill switch must also work when wifi is gone. |
| `as2_state_estimator` (stock node) | **each Jetson** | Feeds the platform's external-odometry path. Same loop, same machine. |
| `as2_mocap_guarded` — `mocap_pose_guarded::Plugin` (S1) | **each Jetson** | The project's own hardened fork of the stock `mocap_pose` plugin, loaded *into* the stock node via `plugin_name: mocap_pose_guarded`. **This is the plugin that must load, not `mocap_pose`.** It lives in the repo at `deckga_ros2/as2_mocap_guarded/` and is built into `~/as2_o134_ws`. |
| `as2_motion_controller` / `controller_manager` | **each Jetson** | Closes the position loop. Never put a control loop across wifi. |
| `as2_behaviors_*` (takeoff, land, go_to, follow_path) | **each Jetson** | Action servers the GS calls. They survive a momentary wifi dropout; a behaviour running on the GS would not. |
| `vrpn_mocap` client | **each Jetson** *(recommended)* **and GS** | See [the mocap decision](#2-the-mocap-decision-one-bridge-or-four) below. |
| `flight_ops/nodes/vrpn_to_rigidbodies.py` | **each Jetson** *(recommended)* **and GS** | Same decision. Namespaced per drone — see below. |
| `flight_ops/nodes/volume_guard.py` (S2) | **GS** | The independent fence over the whole room. It must see every aircraft at once, and it publishes `px4_msgs/VehicleCommand` to `<prefix>/fmu/in/vehicle_command` **directly** — deliberately not through the platform node, since the platform node is one of the things that can hang. It therefore needs `px4_msgs` on the GS (already built) and reachability to every aircraft's FMU topics over the fleet DDS. |
| `flight_ops/preflight_check.py` (S4) | **GS** | Gates props-on for the fleet. Must see every namespace at once. |
| `flight_ops/snapshot.py` (G0) | **GS** | Reads shared lab state, `ssh`es *to* the Jetsons. Never runs on one. |
| `deckga_ros2/deckga_execute.py` | **GS** | The mission sequencer. Uses `as2_python_api.DroneInterface` against all three namespaces. |
| `deckga_ros2/rviz_paths_node.py`, `rviz_obstacles_node.py`, RViz | **GS** | Visualisation. There is no display on a Jetson, and `ros-humble-desktop` is deliberately **not** installed on them. |
| `DECK_GA_QuickNav.py`, `perception/` | **GS**, offline | Planning happens before the session and writes a `.pkl`. Nothing plans in flight. |
| QGroundControl | **GS** | Parameter read-back, MAVLink console. |
| Motive / OptiTrack | lab PC | Not ours. VRPN server only. |

### The rule behind the table

> **Anything inside a control or safety loop runs on the Jetson.
> Anything that supervises the fleet, or that a human looks at, runs on the GS.**

Wifi is the least reliable link in the system. Every loop that crosses it is a
loop that fails when someone walks between the AP and the aircraft. The three
loops that must never cross it are: FC ↔ platform, mocap → estimator →
platform, and setpoint → controller.

---

## 2. The mocap decision: one bridge, or four?

There is a real choice here, and it is worth stating both sides.

**Option A — one bridge on the GS.** `vrpn_mocap` + `vrpn_to_rigidbodies.py`
run once, on the ground station, publishing `/mocap/rigid_bodies`. All three
Jetsons subscribe to it over wifi DDS.

**Option B — a client per aircraft.** Each Jetson runs its own `vrpn_mocap`
talking directly to Motive, plus its own bridge, publishing
`/<ns>/mocap/rigid_bodies` locally. The GS also runs one, for the volume guard,
RViz and `preflight_check.py`.

| | Option A (one bridge on GS) | Option B (per-Jetson client) |
|---|---|---|
| Configuration surface | One place. One rigid-body name map. | Four places. A typo can differ per aircraft. |
| Load on Motive | 1 VRPN client | 4 VRPN clients |
| Safety-critical transport | 100 Hz RigidBodies over **DDS on wifi**, fanned out ×3 | VRPN unicast, one hop, per aircraft |
| Bandwidth | Small either way (~50 kB/s per subscriber) | Same |
| Failure correlation | **One wifi hiccup starves all three aircraft at once** | Independent per aircraft |
| Where the freshness gates sit | Both gates (`vrpn_to_rigidbodies.py` staleness, `external_odom_timeout_s`) live *upstream* of the wifi hop | Both gates live on the same machine as the consumer, downstream of every hop |
| Precedent | — | **The QUT EGH450 setup ran a VRPN client on each drone's onboard computer, on this lab's infrastructure.** |

### Recommendation: **Option B.**

Four reasons, in order of weight:

1. **It moves a 100 Hz safety-critical stream off cross-machine DDS.** VRPN is
   a lightweight unicast protocol built for exactly this traffic. DDS over
   wifi is not: reliable-QoS retransmission on a lossy link produces latency
   spikes precisely when you least want them, and multicast (which DDS uses
   for discovery, and by default for data) is transmitted at the AP's lowest
   basic rate and is the first thing to drop.

2. **It decorrelates the failure.** Under Option A a single wifi dropout stops
   external odometry on all three aircraft simultaneously. All three then hit
   the platform's `external_odom_timeout_s` gate at the same instant, stop
   publishing to PX4 at the same instant, and hand all three to PX4's external
   vision failsafe at the same instant — inside an 8 × 6 × 4 m net. Under
   Option B one aircraft loses its feed and the other two keep flying.

3. **It puts the freshness gates where the failure is.** This project has two
   deliberate staleness gates: the bridge's *absence-means-stale* invariant,
   and Change 6 in `flight_ops/patches/`. Under Option A both sit on the far
   side of the wifi hop from the consumer, so neither can observe the hop
   itself; only the platform's timeout catches a wifi failure, one gate deep.
   Under Option B every gate is downstream of every hop it is supposed to
   protect against.

4. **It is the configuration already proven in this lab.** EGH450 ran this
   topology on this OptiTrack rig. That is stronger evidence than any argument
   above.

The cost — four copies of the configuration — is exactly what this deployment
kit exists to neutralise: the bundle is byte-identical across the three
aircraft and only `--drone-ns` differs.

### What Option B requires you to do

`vrpn_to_rigidbodies.py` publishes `/mocap/rigid_bodies` by default, and both
the stock `mocap_pose` plugin and `as2_mocap_guarded` default their
`mocap_topic` to `/mocap/rigid_bodies`. On one shared `ROS_DOMAIN_ID=0`
**three unnamespaced bridges would collide**: every drone would see all three
publishers. So namespace them.

On each Jetson (this is `drone0`; change the names for the others):

```bash
source ~/as2_o134_ws/setup_env.sh

# 1. the VRPN client, in this drone's namespace
ros2 run vrpn_mocap client --ros-args -r __ns:=/drone0 \
    -p server:=<motive-ip> -p port:=3883 -p frame_id:=map -p update_freq:=100.0

# 2. the bridge, publishing into this drone's namespace only
python3 ~/as2_o134_ws/flight_ops/nodes/vrpn_to_rigidbodies.py --ros-args \
    -p trackers:="['drone0']" \
    -p input_topic_template:="/drone0/vrpn_mocap/{tracker}/pose" \
    -p output_topic:=/drone0/mocap/rigid_bodies \
    -p stale_timeout_s:=0.1

# 3. the state estimator, with the GUARDED plugin, pointed at that topic
ros2 launch as2_mocap_guarded mocap_pose_guarded_state_estimator.launch.py     namespace:=drone0     rigid_body_name:=drone0     mocap_topic:=/drone0/mocap/rigid_bodies
```

The launch file already exposes `namespace`, `rigid_body_name` and
`mocap_topic` as arguments, and sets `plugin_name` in the launch file rather
than a YAML so it cannot be silently changed. `rigid_body_name` has **no usable
default on purpose** — a wrong or missing name is the exact defect the fork
exists to fix, so the plugin throws at setup rather than quietly tracking
nothing.

One QoS detail that bites here: `as2_mocap_guarded` defaults
`mocap_qos_reliability: best_effort`, which is what `vrpn_to_rigidbodies.py`
publishes. The upstream plugin hard-codes a RELIABLE subscription, which
matches nothing and delivers zero messages in total silence. Do not set it to
`reliable` unless you have changed the publisher too.

The **ground station keeps its own** unnamespaced `vrpn_mocap` +
`vrpn_to_rigidbodies.py` on `/mocap/rigid_bodies`, which is what
`volume_guard.py`, RViz and `preflight_check.py`'s `mocap_rate` check consume.

> **Consequence for `preflight_check.py`, stated honestly.** Its
> `--mocap-topic` takes one topic, so `rigid_bodies` and `pose_delta` validate
> the *ground station's* copy of the feed, not each aircraft's. Until it learns
> per-namespace mocap topics, run it once per drone for those two checks:
>
> ```bash
> python3 flight_ops/preflight_check.py --only "rigid_bodies pose_delta" \
>     --drones drone0 --mocap-topic /drone0/mocap/rigid_bodies
> ```
>
> This is a real gap, listed under [Follow-ups](#8-what-this-kit-does-not-do).

---

## 3. arm64 availability — what actually exists

Checked against the ROS 2 build farm, not assumed.

| Package | arm64 jammy Humble deb? | Evidence |
|---|---|---|
| `ros-humble-aerostack2` | **YES** | `Hbin_ujv8_uJv8__aerostack2__ubuntu_jammy_arm64__binary` — last build #186, success |
| `ros-humble-as2-state-estimator` | **YES** | same job family, #116, success |
| `ros-humble-vrpn-mocap` | **YES** | #91, success |
| `ros-humble-mocap4r2-msgs` | **YES** | #64, success |
| `ros-humble-ros-base`, `rmw-cyclonedds-cpp` | **YES** | ROS 2 core; jammy arm64 is a first-class target |
| **Micro-XRCE-DDS-Agent** | **NO — no package on ANY architecture** | No `microxrcedds_agent` and no `micro_ros_agent` job exists for Humble; both URLs 404 |

**So: Aerostack2 installs from apt on the Jetsons exactly as it did on the
ground station.** `ros-humble-aerostack2` version on the GS is
`1.1.3-1jammy`; `verify_jetson.sh` FAILs if the Jetson gets anything else,
because `flight_ops/patches/PATCHES.md` reasons against the 1.1.3 `as2_core`
API specifically.

**The agent is the exception, and it is the one real fallback in this kit.**
The three candidate paths and why one was chosen:

| Path | Verdict |
|---|---|
| apt | Does not exist. |
| `snap install micro-xrce-dds-agent` | Needs internet *and* snapd on the Jetson, and pins a version we do not control. Rejected. |
| Build from source, superbuild ON | `ExternalProject_Add` git-clones Fast-CDR and Fast-DDS at configure time. **Needs internet.** Rejected as the primary path. |
| **Build from source, `UAGENT_SUPERBUILD=OFF`** | **Chosen.** |

At tag `v2.4.2` the CMakeLists asks for `find_package(fastcdr 1)`,
`find_package(fastrtps 2)` and `find_package(spdlog 1)` when the
`UAGENT_USE_SYSTEM_*` options are ON. ROS 2 Humble already installs
`ros-humble-fastcdr` (1.0.x) and `ros-humble-fastrtps` (2.6.x), and jammy ships
`libspdlog-dev` 1.9.2. All three are satisfied, so the build needs **no network
at all** — which is the whole point on a machine that could not reach github.
`jetson_setup.sh` phase `agent` does exactly this; the superbuild is printed as
the fallback in the failure message, with its internet requirement spelled out.

---

## 4. The files

| File | Runs on | What it is |
|---|---|---|
| `README.md` | — | This. |
| `packages.txt` | — | The apt set, one package per line, each with the reason it is on a flight computer. Parsed by both scripts. |
| `bundle_for_jetson.sh` | **GS** | Produces one tarball with the three source packages (`px4_msgs`, patched `as2_platform_pixhawk`, `as2_mocap_guarded`), the agent source, the `flight_ops` payload, the ROS archive key and a checksum manifest. Prints the `scp` line. |
| `jetson_setup.sh` | **Jetson** | Seven idempotent phases: preflight, network, apt, workspace, build, agent, env. Generates `setup_env.sh`, `cyclonedds.xml` and `rollback.sh`. |
| `verify_jetson.sh` | **Jetson** | 18 checks, one PASS/FAIL table, one verdict. Non-zero exit on any FAIL. Mirrors `preflight_check.py`. |

Nothing else in the repository is modified by any of them.

---

## 5. The procedure, per Jetson

Do this three times. **The bundle is identical for all three — only
`--drone-ns` changes.** Do one aircraft completely, verify it green, and only
then start the next; that way a mistake costs one Jetson, not three.

### Prerequisite: ssh keys, once

No password appears anywhere in this repository and none should be typed into
a script. `snapshot.py` already uses `ssh -o BatchMode=yes`, which *requires*
key auth.

```bash
ssh-keygen -t ed25519 -C "o134-ground-station"     # if you have no key yet
ssh-copy-id jetson@10.88.51.230                    # asks for the password ONCE, interactively
ssh -o BatchMode=yes jetson@10.88.51.230 true && echo "key auth works"
```

**Verify:** the last line prints `key auth works` with no prompt.

---

### Step 1 — Build the bundle (on the GS)

```bash
cd ~/as2_o134_ws                     # anywhere; the script finds the repo itself
"/mnt/c/Users/mitch/Multi-UAV Project/ROS2_MultiUAV_3D-main/flight_ops/deploy/bundle_for_jetson.sh"
```

It refuses to build a bundle whose `as2_platform_pixhawk` is unpatched — both
by `git apply --check --reverse` and by grepping for the two safety changes.
That refusal is the point: an unpatched tree means a kill switch that is a
silent no-op on three aircraft.

**Verify:**

```
OK    O134 patch IS applied to the working tree (git apply --check --reverse)
OK    patch content verified in pixhawk_platform.cpp
OK    src/as2_mocap_guarded staged (workspace copy, identical to deckga_ros2/as2_mocap_guarded)
OK    MANIFEST.sha256: <n> files
OK    ./o134_bundle_<stamp>.tar.gz (2.2M)
```

**Rollback:** delete the tarball. The script writes nothing else.

---

### Step 2 — Deliver it

```bash
scp o134_bundle_<stamp>.tar.gz o134_bundle_<stamp>.tar.gz.sha256 jetson@10.88.51.230:~/
ssh jetson@10.88.51.230 'sha256sum -c ~/o134_bundle_<stamp>.tar.gz.sha256'
ssh jetson@10.88.51.230 'tar -xzf ~/o134_bundle_<stamp>.tar.gz -C ~'
```

**Verify:** `sha256sum -c` prints `OK`. Do not skip this — wifi scp corruption
shows up later as an incomprehensible compile error.

**Rollback:** `ssh jetson@... 'rm -rf ~/o134_bundle_<stamp>*'`

---

### Step 3 — Look before you leap (on the Jetson)

```bash
ssh -t jetson@10.88.51.230
~/o134_bundle_<stamp>/flight_ops/deploy/jetson_setup.sh --drone-ns drone0 --dry-run
```

**Verify:** phase 0 confirms `aarch64` and `jammy`, phase 1 prints the
interface and route table. **Read the route output.** See
[the phantom gateway](#7-the-four-traps-this-kit-exists-to-catch).

**Rollback:** none needed — `--dry-run` changes nothing.

---

### Step 4 — Run it for real

```bash
~/o134_bundle_<stamp>/flight_ops/deploy/jetson_setup.sh \
    --drone-ns drone0 \
    --gs-host  10.88.51.10 \
    --peer     10.88.51.231 \
    --peer     10.88.51.232
```

`--gs-host` and `--peer` become Cyclone DDS unicast peers, so discovery does
not depend on wifi multicast working. Substitute the real addresses.

`sudo` prompts once, at the start, on purpose.

Expect **45–90 minutes**, nearly all of it `px4_msgs` (235 message definitions,
compiled on an Orin NX). Run it under `tmux` so an ssh drop does not kill it.

**Verify, phase by phase:**

| Phase | It worked when you see |
|---|---|
| 0 preflight | `OK architecture aarch64`, `OK Ubuntu 22.04 (jammy)`, bundle info printed |
| 1 network | `OK default route does not go via 10.41.10.254` |
| 2 apt | `OK all required packages present` |
| 3 workspace | `OK MANIFEST.sha256 verifies`, `OK O134 platform patch is present in the source` |
| 4 build | `OK as2_platform_pixhawk built: …/as2_platform_pixhawk_node` and `OK as2_mocap_guarded built: libmocap_pose_guarded.so + pluginlib index entry` |
| 5 agent | `OK MicroXRCEAgent built and runs` |
| 6 env | `OK wrote …/setup_env.sh`, `OK wrote …/cyclonedds.xml` |

Any phase can be re-run alone: `--only "build"`, `--only "agent env"`. A phase
that is already satisfied says so and does nothing.

**Rollback:** `~/as2_o134_ws/.deploy/rollback.sh` (dry run) then `--apply`.

---

### Step 5 — Source the environment

```bash
source ~/as2_o134_ws/setup_env.sh
```

**Verify:** it echoes

```
O134 env: drone0  ROS_DISTRO=humble  RMW=rmw_cyclonedds_cpp  DOMAIN=0
```

> **This is per session, by hand, every time.** Nothing was written to
> `~/.bashrc`, no systemd unit was installed, no cron job was created — rule 6
> of the shared-hardware discipline. A login shell on this Jetson still looks
> like a stock JetPack shell to whoever uses the aircraft next.

**Rollback:** open a new shell.

---

### Step 6 — Verify, without the flight controller

```bash
~/o134_bundle_<stamp>/flight_ops/deploy/verify_jetson.sh --drone-ns drone0 --gs-host 10.88.51.10
```

**Verify:** every row PASS except `fmu_topics` / `fmu_endpoints`, which SKIP.
The verdict line will say `JETSON GREEN … (no --with-fmu: the uXRCE-DDS session
is UNPROVEN)`. That is expected at this point and is **not** sufficient to put
the aircraft on a card.

---

### Step 7 — Verify with the flight controller powered

Power the Pixhawk, connect the Ethernet link, then:

```bash
~/o134_bundle_<stamp>/flight_ops/deploy/verify_jetson.sh \
    --drone-ns drone0 --gs-host 10.88.51.10 --with-fmu \
    --json ~/verify_drone0.json
```

`--with-fmu` starts a `MicroXRCEAgent`, waits for the FC to open a session,
inspects the graph, and kills the agent again. It leaves nothing running.

**Verify:** the two rows that matter —

```
fmu_topics     /fmu/    PASS  N /fmu/ topics visible; all 9 required present
fmu_endpoints  matched  PASS  9/9 matched; timesync_status ~100 Hz
```

and the verdict `JETSON GREEN` with no SUBSET or UNPROVEN qualifier.

`fmu_endpoints` is the one that earns its keep: a `/fmu/` topic can *exist*
because the agent advertised it while the flight controller never matched on
it. Existence is not the check; **matched endpoints** are. If it fails, the
detail line names the four PX4 parameters to read back in QGC —
`UXRCE_DDS_AG_IP=170461697`, `PRT=8888`, `KEY=1`, `DOM_ID=0`. Trust the
read-back, never the QGC popup.

**Verify (fleet level, once all three are done):** on the GS, with all three
Jetsons up, `ros2 node list` shows all three namespaces. If it does not, the
Cyclone interface is the first suspect — see the traps below.

---

### Step 8 — Copy the JSON off, and record it

```bash
scp jetson@10.88.51.230:~/verify_drone0.json ./flight_ops/snapshots/
```

The run-sheet card for this aircraft gets `Proven on: bench, verify_jetson.sh
<UTC>, JETSON GREEN`. **A card with an empty `Proven on:` field cannot be
flown.**

---

## 6. Rollback

Everything this kit installs lives in three places, and the generated
`rollback.sh` removes them in strict reverse order, asking before each step.

```bash
~/as2_o134_ws/.deploy/rollback.sh          # dry run — prints, changes nothing
~/as2_o134_ws/.deploy/rollback.sh --apply  # prompts per step
```

| Reverse order | What goes | Manual equivalent |
|---|---|---|
| 6 env | `setup_env.sh`, `cyclonedds.xml` | `rm -f ~/as2_o134_ws/{setup_env.sh,cyclonedds.xml}` |
| 5 agent | the whole agent tree | `rm -rf ~/xrce_agent` |
| 4 build | build artefacts | `rm -rf ~/as2_o134_ws/{build,install,log}` |
| 3 workspace | the source | `rm -rf ~/as2_o134_ws/{src,flight_ops}` |
| 2 apt | only the packages this kit added | `sudo apt-get purge -y $(cat ~/as2_o134_ws/.deploy/apt_installed_by_deploy.txt)` |
| 2 apt source | the ROS repo entry | `sudo rm -f /etc/apt/sources.list.d/ros2.list /usr/share/keyrings/ros-archive-keyring.gpg` |

`apt_installed_by_deploy.txt` is a `dpkg` diff taken across the apt phase, so
**read it before purging it** — anything else installed in the same window is
in it too. That caveat is printed by the script.

There is nothing else to undo: no `.bashrc` append, no systemd unit, no cron
job, and the only file outside `$HOME` is the apt source entry.

---

## 7. The five traps this kit exists to catch

### 7.1 The phantom gateway — almost certainly why github was unreachable

`10.41.10.254` is advertised on the FC link, and **nothing is there.** The link
is point-to-point: Jetson `10.41.10.1`, FC `10.41.10.2`, no router. If
NetworkManager installs `10.41.10.254` as the *default* route, every outbound
packet — apt, DNS, github — is black-holed, and the wifi looks fine while
nothing works.

`jetson_setup.sh` phase 1 and `verify_jetson.sh`'s `route` check both look for
this by name.

```bash
ip route show default                                  # is 10.41.10.254 in there?
sudo ip route del default via 10.41.10.254 dev enP8p1s0   # session-local fix, reverts on reboot
```

Durable fix, which **does persist** and therefore belongs in a `snapshot.py`
capture before you do it:

```bash
nmcli connection modify <fc-connection> ipv4.never-default yes ipv4.gateway ''
nmcli connection up <fc-connection>
```

### 7.2 Cyclone DDS picks the wrong interface

Cyclone binds **one** interface. With both the wifi and the FC link up it can
pick `enP8p1s0` — whose only peer is a flight controller that speaks XRCE, not
DDS. Every ROS 2 topic then becomes invisible to the ground station, with no
error printed anywhere.

`jetson_setup.sh` phase 6 writes `~/as2_o134_ws/cyclonedds.xml` pinning
`wlP1p1s0`, and `setup_env.sh` exports `CYCLONEDDS_URI` at it. The config also
sets `<AllowMulticast>spdp</AllowMulticast>` — multicast for discovery only,
user data unicast — because wifi multicast is transmitted at the AP's lowest
basic rate and is where a 100 Hz stream goes to die. `--gs-host` / `--peer`
add unicast discovery peers so discovery survives an AP that drops multicast
entirely.

`verify_jetson.sh`'s `fleet_iface` check fails if `cyclonedds.xml` does not
name the interface that is actually up.

### 7.3 JetPack 5 (Ubuntu 20.04) has no ROS 2 Humble

There is no Humble deb for focal. `jetson_setup.sh` stops with exit 2 and
prints the three real options (flash JetPack 6, containerise, or build from
source) rather than half-installing something. Check first:

```bash
ssh jetson@10.88.51.230 '. /etc/os-release; echo $PRETTY_NAME; cat /etc/nv_tegra_release'
```

### 7.4 The agent is not an apt package

Covered in [§3](#3-arm64-availability--what-actually-exists). If phase 5 fails,
read its message: it names the exact cmake line and the internet-requiring
fallback. Do not "just apt install" something — there is nothing to install.

---

### 7.5 Three aircraft, one `ROS_DOMAIN_ID`, one set of `/fmu/` topics

**Read this before the second Jetson goes on an airframe.** It is the one
topology problem this kit surfaces but cannot decide for you.

PX4 publishes bare `/fmu/out/...` and subscribes to bare `/fmu/in/...` by
default. With three aircraft on `ROS_DOMAIN_ID=0` and three agents on one wifi,
**all three use the same topic names.** Two consequences, both bad:

* Every drone's `as2_platform_pixhawk` receives `/fmu/out/vehicle_odometry`
  from **all three** flight controllers, and fuses whatever arrives.
* A disarm published by `volume_guard.py` to `/fmu/in/vehicle_command` reaches
  **every** aircraft on the link. `volume_guard.py` knows this: its
  `check_disarm_routing()` refuses to start when more than one drone shares a
  single `/fmu/in/vehicle_command` without distinct `target_system` ids.

`target_system` (i.e. distinct `MAV_SYS_ID` per airframe) fixes the *command*
direction only. It does nothing for the outbound telemetry collision. So the
real fix is a per-aircraft FMU namespace:

| Lever | Where | Note |
|---|---|---|
| `UXRCE_DDS_NS_IDX` | **PX4 parameter, v1.17+** | Set 0/1/2 per airframe → PX4 publishes under `/uav_0`, `/uav_1`, `/uav_2`. Settable from QGC; no startup-script surgery. **Recommended.** |
| `uxrce_dds_client start -n <ns>` | PX4 startup script | Arbitrary namespace, but means editing the SD-card startup on each FC. |
| `fmu_prefix` | `as2_platform_pixhawk` parameter | Already exists in the patched platform (`declare_parameter<std::string>("fmu_prefix")`). Must match what PX4 publishes. |
| `--fmu-prefix` | `volume_guard.py`, `preflight_check.py`, `verify_jetson.sh` | Literal prefix, or `{ns}` substituted with the drone namespace. |
| `MAV_SYS_ID` | PX4 parameter | Distinct per airframe regardless. Necessary, not sufficient. |

**The naming mismatch you have to decide about.** `UXRCE_DDS_NS_IDX` produces
`/uav_N`, while this project's Aerostack2 namespaces are `drone0/1/2`. The
`{ns}` substitution in `volume_guard.py` and `preflight_check.py` only lines up
if the two agree. Either rename the AS2 namespaces to `uav_0/1/2`, or set the
PX4 namespace via `-n drone0` in the startup script. **This kit does not choose
— it just makes sure whichever you pick is passed consistently:**

```bash
jetson_setup.sh --drone-ns drone0 --fmu-prefix /uav_0 --only env
# -> setup_env.sh exports O134_FMU_PREFIX=/uav_0
# -> verify_jetson.sh picks it up automatically
# -> pass the same string to the platform's fmu_prefix param and to
#    volume_guard.py --fmu-prefix
```

**`UXRCE_DDS_NS_IDX` is not currently in `flight_ops/lab_config/expected_state.yaml`** —
not in `gates.must_equal`, not in `managed_params`. Setting it is therefore a
change to the managed-parameter allow-list and follows the normal
snapshot → change → read-back → rollback discipline. Do not just type it into
QGC.

## 8. What this kit does *not* do

Stated plainly so nobody discovers it at the pad.

1. **It does not configure Aerostack2's per-drone project.** It installs and
   builds `as2_mocap_guarded` and verifies the plugin is loadable, but it does
   not write the per-drone `controller` / `platform` YAML, and
   `flight_ops/lab_config/o134_project/` is still empty. The §2 commands show
   what must change for the namespaced mocap topology; the project itself is a
   separate piece of work.
2. **It does not teach `preflight_check.py` per-namespace mocap topics.** See
   the callout in §2. Until then, run it once per drone for `rigid_bodies` and
   `pose_delta`.
3. **It does not touch the flight controller.** No parameter is written. Use
   `snapshot.py` and QGC for that, and read every value back.
4. **It does not build `session_bringup.sh`.** Still marked *not yet built* in
   `flight_ops/README.md`. `volume_guard.py` now exists and runs on the GS, but
   this kit does not configure it — in particular it does not choose the
   `--fmu-prefix` for you (trap 7.5).
5. **`--with-debs` in `bundle_for_jetson.sh` is a contingency, not a plan.**
   Full offline apt mirroring for arm64 is fragile by nature. The robust path
   is ten minutes of working wifi on the Jetson, which trap 7.1 is usually what
   stands in the way of.
6. **Nothing here has been run against a Jetson.** See below.

---

## Verified / Unverified

Honest status. No Jetson was reachable when this kit was written, so nothing
that requires aarch64 hardware has been executed.

### Verified — actually run, output observed

| What | How it was verified |
|---|---|
| `bundle_for_jetson.sh` end to end | Run for real on the WSL ground station. Produced a 2.2 MB tarball, 496 files. |
| Bundle integrity round trip | `sha256sum -c` on the tarball, then unpacked, then `sha256sum -c MANIFEST.sha256` — all OK. |
| Patch-provenance refusal logic | `git apply --check --reverse` and the content greps both ran against the real `~/as2_o134_ws/src/as2_platform_pixhawk` and passed. |
| `px4_msgs` / platform commit assertions | Confirmed against the real workspace: `86d8239` on `release/1.17` (tag `v1.17.0`), platform base `2b00b77`. |
| Micro-XRCE-DDS-Agent `v2.4.2` clone | Cloned for real, `57d0862`. |
| ROS archive key fetch | Fetched; expiry read as 2030-06-01. |
| `verify_jetson.sh` table, verdict, exit codes | Run on the GS: correctly returned RED (exit 1) on wrong-arch, GREEN on a passing subset, and the SUBSET / UNPROVEN qualifiers appear. |
| `verify_jetson.sh --json` | Written and parsed back with `json.load`. |
| `platform_patch` check | Caught a false positive on its first version (it matched the explanatory comment); tightened to match the publisher member, and re-verified PASS against the real patched source. |
| `jetson_setup.sh` hard guards | Verified it refuses to run on x86_64 with exit 2, **including** under `--only`, after that hole was found and fixed. |
| `packages.txt` parsing | Required/optional split verified: 27 required, 3 optional. |
| `as2_mocap_guarded` is required and shippable | Found in `~/as2_o134_ws/src` **and** `deckga_ros2/`, byte-identical (`diff -rq`), already built into `install/`. The bundle's provenance comparison was written against that observation. |
| Plugin artefact names | `libmocap_pose_guarded.so` and the `as2_state_estimator__pluginlib__plugin/as2_mocap_guarded` resource-index path read from `plugins.xml` and `CMakeLists.txt`, not guessed. |
| Launch arguments for S1 | `namespace`, `rigid_body_name`, `mocap_topic` read from `mocap_pose_guarded_state_estimator.launch.py`. |
| arm64 package availability | Each buildfarm job URL fetched individually. Positives and the two 404s are both real observations. |
| `UAGENT_SUPERBUILD=OFF` dependency versions | Read from the actual `v2.4.2` `CMakeLists.txt`: `fastcdr 1`, `fastrtps 2`, `spdlog 1`. |
| `mocap_topic` / `rigid_body_name` parameter names | Read from the installed `as2_state_estimator` 1.1.3 plugin config on the GS. |
| `bash -n` on all three scripts | Clean under bash 5.2 (msys) and bash 5.1 (Ubuntu 22.04). |
| **Phase 6 output, generated for real** | `phase_env` was called directly on the GS. The generated `setup_env.sh` is `bash -n` clean, **and sourcing it actually worked** — it sourced ROS 2 Humble and Aerostack2 and exported `AS2_DRONE_NS=drone0`, `O134_FMU_PREFIX=/uav_0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=0`, `CYCLONEDDS_URI`. |
| Generated `cyclonedds.xml` | Well-formed XML (`xml.dom.minidom.parse`), with the interface pinned and all three `--gs-host`/`--peer` addresses present as `<Peer>` entries. |
| Generated `rollback.sh` | `bash -n` clean; its dry run prints the six steps in strict reverse order (6 env → 5 agent → 4 build → 3 workspace → 2 apt → apt source). |
| `--fmu-prefix` plumbing | `/uav_0` passed to `jetson_setup.sh` appears in `setup_env.sh` as `O134_FMU_PREFIX`, and `verify_jetson.sh` picks it up from the environment with no extra flag. |

### Unverified — needs a Jetson

| What | Why it could not be checked | What would prove it |
|---|---|---|
| Every phase of `jetson_setup.sh` on aarch64 | No Jetson reachable. It stops at the architecture guard here, by design. | One full run, `--dry-run` first. |
| `as2_mocap_guarded` compiling on the Jetson | It needs `as2_state_estimator` headers from the apt Aerostack2. Compiles on the GS; not compiled on arm64. | Phase 4 on the Jetson. |
| The guarded plugin actually *loading* at runtime | The kit checks the `.so` and the pluginlib resource-index entry exist. It does not start the node. | `ros2 launch as2_mocap_guarded …` and look for `mocap_pose_guarded` in the log. |
| `ros-humble-aerostack2` arm64 **installing cleanly on JetPack** | Buildfarm green proves the deb exists, not that it installs beside NVIDIA's pinned libraries. | `apt-get install -s ros-humble-aerostack2` on the Jetson. |
| The agent building with `UAGENT_SUPERBUILD=OFF` | The version constraints are read from source and are satisfied on paper. Not compiled. | Phase 5 on the Jetson. |
| `px4_msgs` build time and RAM on an Orin NX | The 45–90 min estimate is an estimate. | One build. Watch for OOM; add swap if it dies. |
| `cyclonedds.xml` interface pinning *taking effect* | The file is generated and valid, but no Cyclone process has ever read it, and `wlP1p1s0` / `enP8p1s0` are the names you gave rather than names observed on hardware. | `ip -brief link` on the Jetson, then `ros2 node list` from the GS with the Jetson up. |
| The phantom-gateway diagnosis | Strongly indicated by "no github on the wired route", but not confirmed on the hardware. | `ip route show default` on the Jetson. |
| `fmu_topics` / `fmu_endpoints` | Needs a powered FC. | Step 7. |
| The whole mocap topology (Option B) | Reasoned and precedent-backed, not measured on this rig. | Run both and compare dropout behaviour on the bench with `fake_mocap.py`. |
| `--with-debs` | Written, never executed. | A run with a real `/var/lib/dpkg/status` from a Jetson. |
| The `/fmu/` namespace decision (trap 7.5) | `UXRCE_DDS_NS_IDX` was read from the PX4 v1.17 docs, not set on a flight controller. No aircraft has ever run with a prefix. | Set it on one FC, then `verify_jetson.sh --with-fmu --fmu-prefix /uav_0`. |
| `shellcheck` cleanliness | `shellcheck` is not installed on this machine or in WSL. Only `bash -n` was run. | `sudo apt install shellcheck && shellcheck flight_ops/deploy/*.sh` |

---

## Quick reference

```bash
# ground station
flight_ops/deploy/bundle_for_jetson.sh
scp o134_bundle_*.tar.gz* jetson@10.88.51.230:~/

# jetson
tar -xzf o134_bundle_*.tar.gz -C ~
~/o134_bundle_*/flight_ops/deploy/jetson_setup.sh --drone-ns drone0 --gs-host <gs-ip>
source ~/as2_o134_ws/setup_env.sh
~/o134_bundle_*/flight_ops/deploy/verify_jetson.sh --drone-ns drone0 --with-fmu

# run the agent (no sudo)
MicroXRCEAgent udp4 -p 8888

# undo everything
~/as2_o134_ws/.deploy/rollback.sh --apply
```
