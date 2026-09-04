# flight_ops

Operational tooling and run-sheets for the EGH490-2 flight campaigns: indoor
motion-capture flight in **O-134**, then outdoor GPS flight at **SERF Brisbane**.

## The governing rule

> **No event happens for the first time inside O-134.**
> A lab session executes a script that has already been executed.

O-134 bookings are scarce and every session needs multiple people. Sessions are
the limiting resource, so the entire campaign is arranged to move work *out* of
the lab: onto simulation rigs, onto the bench, onto replayed mocap data.

This is enforced mechanically. Every card in every run-sheet carries a
**`Proven on:`** field naming the rig and gate where that exact procedure was
executed.

> **A card with an empty `Proven on:` field cannot be flown.**

## Layout

| Path | What it is |
|---|---|
| `snapshot.py` | Gate G0. Read-only capture of shared lab state + rollback generation. |
| `preflight_check.py` | S4. Nine checks, one PASS/FAIL table; gates props-on. `--self-test` runs 139 assertions without ROS. |
| `session_bringup.sh` | S7. The single tmux launcher, identical off-site and in the lab. `--sim` runs the whole chain on this PC; `--hardware` flies. `session_stop.sh` tears it down. |
| `nodes/vrpn_to_rigidbodies.py` | S6. VRPN `PoseStamped` -> `mocap4r2_msgs/RigidBodies`, freshness-gated. |
| `nodes/volume_guard.py` | S2. Safety-critical watchdog: seven trip conditions, WARN/LAND/DISARM ladder, force-disarm straight to PX4. |
| `nodes/fake_mocap.py` | Synthetic mocap with eight injectable faults, for rig R1. |
| `lab_config/expected_state.yaml` | Refuse-to-proceed gates + the managed-parameter allow-list. |
| `lab_config/px4_indoor_params.yaml` | S5. Indoor envelope, geofence and failsafe values, verified against PX4 v1.17.0 source. |
| `patches/` | Vendored fixes to `as2_platform_pixhawk`, with evidence. |
| `lab_config/o134_project/` | The Aerostack2 project for real flight. One shared file per node plus per-drone launch arguments. |
| `run_selftests.py` | Runs every `--self-test` in one command. No ROS, no hardware. |
| `deploy/` | Jetson build-out: bundle, setup, verify. |
| `snapshots/<UTC>/` | Timestamped state captures and their generated rollbacks. |
| `runsheets/` | The carried-on-a-clipboard procedures. |
| `rigs/` | Bringup notes for the off-site rigs. |

## Shared-hardware discipline

Anything touching the Holybro airframes, the Pixhawk FC, the Jetsons, or the
OptiTrack rig follows seven rules. `snapshot.py` implements the first four
directly:

1. **Snapshot** every piece of state about to change, into a timestamped
   folder, *before* any modification.
2. **Refuse-to-proceed gates** — abort if the current state suggests someone
   else's setup is live. `gates.must_equal` in `expected_state.yaml`.
3. **Conditional triggers** — never apply a fix preemptively; only when the
   snapshot proves the precondition.
4. **Explicit rollback**, in strict reverse order, with the exact command for
   each step. Generated per snapshot.
5. **Coordination prerequisites** listed up front.
6. **No persistent hooks** — no `.bashrc` appends, no systemd units, no cron.
   Manual `source` per session.
7. **Isolated workspaces** — everything under one removable home subdirectory.

When in doubt whether a resource is shared, treat it as shared.

## Taking a snapshot

Export the parameters from QGroundControl first — a **QGC `.params` export is
strongly preferred** over an `nsh` capture, because it is the only format that
carries parameter *types*, and without types the generated pymavlink rollback
refuses to run.

```bash
# Local record, no hardware present
python3 flight_ops/snapshot.py --label pre-b1 --local-only

# Pre-session capture with gates enforced
python3 flight_ops/snapshot.py --label session0-pre \
    --px4-params droneA:~/dumps/droneA.params \
    --px4-params droneB:~/dumps/droneB.params \
    --check

# End-of-session drift check against the snapshot taken at the start
python3 flight_ops/snapshot.py --label session0-post \
    --px4-params droneA:~/dumps/droneA_after.params \
    --baseline flight_ops/snapshots/20260904T031500Z --check
```

Exit `1` means a `must_equal` gate failed: the hardware is not in the state this
project left it in. **Do not "fix" it — coordinate first.**

Each snapshot with a parameter dump also emits, per airframe:

- `rollback_console_<label>.txt` — paste into **QGC -> Analyze Tools -> MAVLink
  Console**. This is the reliable path: the lab QGC is older than PX4 v1.17 and
  will warn about unknown parameters and reject valid enum values *while the
  write succeeds*. Trust the read-back, never the popup.
- `rollback_<label>.py` — pymavlink equivalent. Int32 parameters are written by
  bit reinterpretation, not cast, which is required above 2^24 (packed IPs).
  Refuses to run if the source capture lacked type information.

No password or credential is ever written into this directory. `snapshot.py`
uses `ssh -o BatchMode=yes`; set up key authentication to the Jetsons first.
