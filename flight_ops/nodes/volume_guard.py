#!/usr/bin/env python3
"""S2: the independent volume watchdog for O-134. The last line of defence.

    OptiTrack -> vrpn_to_rigidbodies -> [this node, on the GROUND STATION]
                                               |
                        LAND (Aerostack2 behavior)   DISARM (straight to PX4)

This is not convenience tooling. It exists because two specific things were
found to be true of the stack it watches, and both of them make the aircraft
look healthy while it is not:

1. `as2_platform_pixhawk`'s kill switch was INERT. It published
   `px4_msgs/ManualControlSwitches` to `/fmu/in/manual_control_switches`, which
   is not one of PX4 v1.17's 38 uXRCE-DDS subscriptions -- the message left the
   ground station and was dropped on the floor. See `patches/PATCHES.md`. The
   patched platform force-disarms instead, but THIS NODE DOES NOT USE IT: an
   emergency action routed through the Aerostack2 platform node is worthless in
   the case where the platform node is the thing that has hung. The DISARM stage
   publishes `px4_msgs/VehicleCommand` to `/fmu/in/vehicle_command` directly.

2. A DEAD MOCAP LINK LOOKS HEALTHY. The visual-odometry path re-stamped a frozen
   pose with `now()` at 100 Hz, so EKF2 saw a perfectly stationary vision source
   and fired no failsafe. The S3 staleness gate fixes that in the platform. This
   node must not assume the gate works -- it is the independent check on it, so
   it measures every age against ITS OWN clock at ITS OWN arrival times and
   never trusts a header stamp.

Consequences, all deliberate:

  * It runs on the ground station, NOT on a drone's Jetson. It is not on the
    aircraft it may have to disarm.
  * It runs on its own timer, subscribes to raw mocap plus one pose topic per
    drone, and depends on nothing else. It never blocks: no service waits, no
    `spin_until_future_complete`, no synchronous action calls.
  * Each drone has an independent state machine. One aircraft tripping never
    acts on another. The routing check at startup refuses to run a
    configuration in which a disarm could reach the wrong airframe.

Trip conditions (all thresholds configurable, see --list-conditions)

    mocap_timeout    raw mocap for this body older than --mocap-timeout (0.1 s)
    mocap_untracked  /mocap/health reports the body NOT TRACKED
    geofence         mocap position at or outside the fenced box
    overspeed        mocap-derived speed at or above --max-speed (2.0 m/s)
    tilt             mocap-derived tilt at or above --max-tilt (35 deg)
    pose_mismatch    self_localization/pose disagrees with raw mocap
    pose_timeout     no self_localization/pose at all for --pose-timeout (0.5 s)

Escalation ladder, per condition: WARN -> LAND -> DISARM

    WARN    log loudly, publish status. No action taken.
    LAND    command the drone's Aerostack2 land behavior. The graceful action.
    DISARM  force-disarm VehicleCommand straight to PX4. THE AIRCRAFT FALLS.

A condition must be CONTINUOUSLY true for `confirm_s` before it reaches the
first stage, and continuously true for another `escalate_s` before each
advance. That dwell is the whole reason a single noisy mocap sample cannot
disarm an aircraft in the air. A disarm is correct for a net strike at speed
and wrong for a marginal trip; the ladder and the dwell are what separate them,
so they are set on the run-sheet card, not guessed here.

Everything latches. A stage, once entered, is never left because the condition
went away -- an incident does not un-happen. Clearing requires an explicit
reset, and a reset while the condition is still true simply re-trips.

Actions are DRY RUN by default: the node logs what it would have sent and sends
nothing. That is the bench default. `--arm` is the explicit, deliberately
awkward flag that makes LAND and DISARM real.

Usage:
    # bench / rig R1 -- logs the actions, sends nothing
    python3 volume_guard.py --drones drone0,drone1,drone2

    # in O-134, actions live, per-drone FMU namespaces
    python3 volume_guard.py --drones drone0,drone1,drone2 --arm \\
        --fmu-prefix '/{ns}' --box 8 6 4 --box-center 0 0 2 --margin 0.5

    # PX4's UXRCE_DDS_NS_IDX namespaces the FMU topics /uav_0, /uav_1,
    # /uav_2 while Aerostack2 namespaces the drones drone0, drone1,
    # drone2. No template spans both, so the mapping is stated, not derived
    python3 volume_guard.py --drones drone0,drone1,drone2 --arm \\
        --fmu-prefix-map drone0=/uav_0,drone1=/uav_1,drone2=/uav_2

    # Motive streams numeric rigid-body ids: map namespace -> body
    python3 volume_guard.py --rigid-bodies drone0:1,drone1:2,drone2:3

    # tighten the fence ladder: no WARN stage, land immediately, disarm 0.5 s on
    python3 volume_guard.py --policy geofence:land+disarm:0.1:0.5

    # clear a latched trip
    ros2 topic pub --once /volume_guard/reset std_msgs/msg/String "{data: 'drone0'}"
    ros2 service call /volume_guard/reset_all std_srvs/srv/Trigger

    python3 volume_guard.py --list-conditions
    python3 volume_guard.py --self-test        # no ROS required

Exit codes:
    0  clean shutdown, or --self-test passed
    1  --self-test failed
    2  the configuration was refused -- the node did not run
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Sequence, Tuple

# --------------------------------------------------------------------------- #
# defaults
# --------------------------------------------------------------------------- #

DEFAULT_DRONES = ("drone0", "drone1", "drone2")

DEFAULT_MOCAP_TOPIC = "/mocap/rigid_bodies"
DEFAULT_HEALTH_TOPIC = "/mocap/health"
DEFAULT_POSE_TEMPLATE = "/{ns}/self_localization/pose"
DEFAULT_STATUS_TOPIC = "/volume_guard/status"
DEFAULT_RESET_TOPIC = "/volume_guard/reset"
DEFAULT_RESET_SERVICE = "/volume_guard/reset_all"
DEFAULT_LAND_ACTION_TEMPLATE = "/{ns}/LandBehavior"

# The one topic that matters. Verified present in PX4 v1.17's dds_topics.yaml
# subscriptions; `/fmu/in/manual_control_switches` is NOT (see PATCHES.md).
FMU_VEHICLE_COMMAND_TOPIC = "/fmu/in/vehicle_command"

# O-134: ~8 x 6 m of floor, ~4 m of usable height, fully netted.
DEFAULT_BOX_M = (8.0, 6.0, 4.0)
DEFAULT_BOX_CENTER_M = (0.0, 0.0, 0.0)
# The fence sits this far INSIDE the net. 0.5 m is a little more than the
# ~0.4 m stopping distance at MPC_XY_VEL_MAX = 1.5 m/s and MPC_ACC_HOR = 3 m/s^2
# (see lab_config/px4_indoor_params.yaml), so an aircraft that trips the fence
# at the configured envelope speed can still stop before the net.
DEFAULT_FENCE_MARGIN_M = 0.5

# Matches external_odom_timeout_s in the patched platform, so this guard and the
# S3 gate agree about what "stale" means. If they ever disagree, the guard is
# the one that must be tighter.
DEFAULT_MOCAP_TIMEOUT_S = 0.1
# Looser than the mocap timeout: self_localization is a derived, lower-rate
# product, and a brief gap there is not the same event as losing the source.
DEFAULT_POSE_TIMEOUT_S = 0.5
# MPC_XY_VEL_MAX is 1.5 m/s indoors. The guard sits ABOVE the commanded
# envelope, not on it: tripping at the commanded limit would fire on every
# normal traverse. 2.0 m/s means the aircraft is doing something the position
# controller was never told to do.
DEFAULT_MAX_SPEED_MPS = 2.0
# MPC_TILTMAX_AIR is 25 deg. Same argument: 35 deg is past what any commanded
# manoeuvre in this room produces.
DEFAULT_MAX_TILT_DEG = 35.0
# preflight_check uses 5 cm for a stationary aircraft on the pad. In flight the
# estimator legitimately lags mocap, so this is the "lost the plot" threshold,
# not an accuracy budget.
DEFAULT_MAX_POSE_DELTA_M = 0.30
# Speed is differenced over this window rather than sample-to-sample: at 100 Hz
# and 2 mm of mocap jitter, a single-sample difference is ~0.3 m/s of noise.
DEFAULT_SPEED_WINDOW_S = 0.1
DEFAULT_HEALTH_TIMEOUT_S = 3.0
# The guard starts before the mocap bridge and the estimator do. Without a
# grace it would latch a trip on every single launch, and a crew that resets a
# latching guard reflexively before every flight has no latching guard at all.
DEFAULT_STARTUP_GRACE_S = 5.0

DEFAULT_RATE_HZ = 50.0
DEFAULT_STATUS_RATE_HZ = 5.0
DEFAULT_LAND_SPEED_MPS = 0.5
# /fmu/in/* is best-effort on the uXRCE-DDS side, so a single datagram can be
# dropped. The last line of defence does not get to depend on one packet.
DEFAULT_DISARM_REPEAT = 5
DEFAULT_TARGET_SYSTEM = 1

# From px4_msgs 2.0.1 VehicleCommand.msg (verified in PATCHES.md):
#   uint16 VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
#   int8   ARMING_ACTION_DISARM             = 0
# 21196 is a bare literal inside PX4's Commander.cpp -- it is NOT exported as a
# message constant, hence the local definition, exactly as the platform patch
# had to do:
#   const bool forced = (static_cast<int>(lroundf(cmd.param2)) == 21196);
# `forced` is what makes Commander skip the "must be landed" guard and cut the
# motors in the air. Without it the disarm is refused while airborne, which is
# precisely the case this node exists for.
PX4_VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
PX4_ARMING_ACTION_DISARM = 0.0
PX4_FORCE_DISARM_MAGIC = 21196.0

# A limit is a limit. At exactly the threshold the guard trips: the tolerance
# leans towards tripping early rather than late, and it stops a value built from
# trigonometry (tilt) or a division (speed) landing one ulp under its own
# threshold and silently doing nothing.
LIMIT_EPS = 1e-9

# --- stages ---------------------------------------------------------------- #

OK = "OK"
WARN = "WARN"
LAND = "LAND"
DISARM = "DISARM"

STAGES: Tuple[str, ...] = (WARN, LAND, DISARM)
STATE_RANK: Dict[str, int] = {OK: 0, WARN: 1, LAND: 2, DISARM: 3}
DEFAULT_LADDER: Tuple[str, ...] = (WARN, LAND, DISARM)

# --- conditions ------------------------------------------------------------ #

C_MOCAP_TIMEOUT = "mocap_timeout"
C_MOCAP_UNTRACKED = "mocap_untracked"
C_GEOFENCE = "geofence"
C_OVERSPEED = "overspeed"
C_TILT = "tilt"
C_POSE_MISMATCH = "pose_mismatch"
C_POSE_TIMEOUT = "pose_timeout"


@dataclass(frozen=True)
class ConditionDoc:
    """One row of --list-conditions: the limit, and why it is where it is."""
    name: str
    limit: str
    why: str


CONDITION_DOCS: Tuple[ConditionDoc, ...] = (
    ConditionDoc(
        C_MOCAP_TIMEOUT, "--mocap-timeout (s)",
        "No raw mocap for this rigid body, measured by ARRIVAL TIME on this\n"
        "     machine's clock. Header stamps are not used: re-stamping a frozen\n"
        "     pose with now() is the exact failure this guard exists to catch."),
    ConditionDoc(
        C_MOCAP_UNTRACKED, "(boolean, from /mocap/health)",
        "The bridge says the body is NOT TRACKED. Independent of the timeout\n"
        "     above, and it fires while other bodies are still streaming --\n"
        "     the single-drone-occlusion case."),
    ConditionDoc(
        C_GEOFENCE, "--box / --box-center / --margin",
        "Mocap position at or outside the box shrunk by the margin. PX4's own\n"
        "     GF_MAX_*_DIST fence is evaluated against GLOBAL position and may\n"
        "     never arm without an EKF origin, which is why this is the PRIMARY\n"
        "     fence and PX4's is the backup."),
    ConditionDoc(
        C_OVERSPEED, "--max-speed (m/s)",
        "Speed differenced from raw mocap over --speed-window, not read from\n"
        "     the estimator: the estimator is one of the things that can be\n"
        "     wrong. Sits above MPC_XY_VEL_MAX so normal traverses do not trip."),
    ConditionDoc(
        C_TILT, "--max-tilt (deg)",
        "Tilt from the mocap rigid-body orientation. Sits above\n"
        "     MPC_TILTMAX_AIR, so it means an attitude excursion nothing\n"
        "     commanded."),
    ConditionDoc(
        C_POSE_MISMATCH, "--max-pose-delta (m)",
        "self_localization/pose disagrees with raw mocap: the estimator has\n"
        "     lost the plot. Evaluated only while BOTH sources are fresh, so a\n"
        "     dropout reports as a dropout and not as a phantom disagreement."),
    ConditionDoc(
        C_POSE_TIMEOUT, "--pose-timeout (s)",
        "No self_localization/pose at all. The estimator, or the whole\n"
        "     Aerostack2 stack for that drone, has stopped."),
)

CONDITIONS: Tuple[str, ...] = tuple(doc.name for doc in CONDITION_DOCS)
CONDITION_INDEX: Dict[str, ConditionDoc] = {d.name: d for d in CONDITION_DOCS}


# --------------------------------------------------------------------------- #
# pure logic -- no ROS, unit-testable, exercised by --self-test
# --------------------------------------------------------------------------- #

def at_or_above(value: float, limit: float) -> bool:
    """Threshold comparison for every numeric trip condition.

    One function so that "at the limit" means the same thing everywhere, and so
    that the direction of the floating-point tolerance is stated once.
    """
    if not math.isfinite(value):
        return True          # a non-finite measurement is not "within limits"
    return value >= limit - LIMIT_EPS


def distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(p) - float(q)) ** 2 for p, q in zip(a, b)))


def tilt_deg_from_quaternion(x: float, y: float, z: float, w: float) -> Optional[float]:
    """Angle between the body up-axis and world up, in degrees.

    Returns None for a quaternion that cannot be normalised. A corrupt
    orientation is reported as UNKNOWN rather than as a trip: rejecting
    malformed samples is the bridge's job (S6), and inventing a tilt from
    garbage would be its own hazard.
    """
    values = (x, y, z, w)
    if not all(math.isfinite(v) for v in values):
        return None
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-6:
        return None
    qx, qy = x / norm, y / norm
    # R[2][2] of the rotation matrix; the third column is the body z axis
    # expressed in the world frame, so its z component is cos(tilt).
    cos_tilt = max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy)))
    return math.degrees(math.acos(cos_tilt))


def health_tracked(health: object, body: str) -> Optional[bool]:
    """Read one body's tracked flag out of a mocap health payload.

    TWO producers publish health JSON, with different shapes, and this guard
    must read either -- pointing it at the wrong one and silently getting
    "unknown" forever would disable a trip condition without saying so:

      /mocap/health          from vrpn_to_rigidbodies.py. NESTED: keyed by VRPN
                             TRACKER name, carrying rigid_body_name inside each
                             entry, because Motive commonly streams numeric ids.
                             Covers every body at once. This is the default and
                             the better source -- it reports what the mocap link
                             is doing, upstream of any estimator.

      /{ns}/mocap_health     from the as2_mocap_guarded plugin. FLAT, one drone,
                             tracked at the top level. Reports what the ESTIMATOR
                             ACCEPTED, so it also goes false on a rejected
                             quaternion or a name miss -- strictly downstream
                             information, useful as a cross-check.

    Resolution order: rigid_body_name inside a nested entry, then the tracker
    key, then a flat top-level payload. Anything unrecognisable is None
    (unknown), never False: a guard that trips on a JSON parse is a guard that
    gets switched off.
    """
    if not isinstance(health, dict):
        return None
    for entry in health.values():
        if not isinstance(entry, dict):
            continue
        if str(entry.get("rigid_body_name", "")) == body:
            value = entry.get("tracked")
            return bool(value) if isinstance(value, bool) else None
    entry = health.get(body)
    if isinstance(entry, dict) and isinstance(entry.get("tracked"), bool):
        return bool(entry["tracked"])
    # Flat per-drone schema. Only trust it when the payload either names this
    # body or names no body at all -- a flat payload for a DIFFERENT drone must
    # not answer for this one.
    if isinstance(health.get("tracked"), bool):
        named = health.get("rigid_body_name")
        if named is None or str(named) == body:
            return bool(health["tracked"])
    return None


@dataclass
class Box:
    """The netted volume, and the fence inside it."""
    size: Tuple[float, float, float] = DEFAULT_BOX_M
    center: Tuple[float, float, float] = DEFAULT_BOX_CENTER_M
    margin: float = DEFAULT_FENCE_MARGIN_M

    def __post_init__(self) -> None:
        self.size = tuple(float(v) for v in self.size)          # type: ignore[assignment]
        self.center = tuple(float(v) for v in self.center)      # type: ignore[assignment]
        self.margin = float(self.margin)
        if len(self.size) != 3 or len(self.center) != 3:
            raise ValueError("box size and centre each need three numbers")
        if min(self.size) <= 0.0:
            raise ValueError("box size must be positive in every axis")
        if self.margin < 0.0:
            raise ValueError("fence margin must be >= 0")
        if min(self.half_extents()) <= 0.0:
            raise ValueError(
                f"margin {self.margin:g} m leaves no flyable volume inside a "
                f"{self.size[0]:g}x{self.size[1]:g}x{self.size[2]:g} m box")

    def half_extents(self) -> Tuple[float, float, float]:
        """Half-extents of the FENCE, i.e. of the box shrunk by the margin."""
        return tuple(0.5 * s - self.margin for s in self.size)   # type: ignore[return-value]

    def outside(self, position: Sequence[float]) -> Tuple[bool, str]:
        """Is this position at or beyond the fence? Returns (outside, why)."""
        half = self.half_extents()
        for axis, value, centre, extent in zip("xyz", position, self.center, half):
            if not math.isfinite(float(value)):
                return True, f"{axis} is not finite"
            offset = abs(float(value) - centre)
            if at_or_above(offset, extent):
                return (True,
                        f"{axis}={float(value):+.2f} m is {offset:.2f} m from centre, "
                        f"fence is {extent:.2f} m")
        return False, ""


@dataclass
class Limits:
    """Every configurable threshold. One object so the node, the status payload
    and the tests all read the same numbers."""
    box: Box = field(default_factory=Box)
    mocap_timeout_s: float = DEFAULT_MOCAP_TIMEOUT_S
    pose_timeout_s: float = DEFAULT_POSE_TIMEOUT_S
    max_speed_mps: float = DEFAULT_MAX_SPEED_MPS
    max_tilt_deg: float = DEFAULT_MAX_TILT_DEG
    max_pose_delta_m: float = DEFAULT_MAX_POSE_DELTA_M
    speed_window_s: float = DEFAULT_SPEED_WINDOW_S
    health_timeout_s: float = DEFAULT_HEALTH_TIMEOUT_S
    startup_grace_s: float = DEFAULT_STARTUP_GRACE_S

    def __post_init__(self) -> None:
        for name in ("mocap_timeout_s", "pose_timeout_s", "max_speed_mps",
                     "max_tilt_deg", "max_pose_delta_m", "speed_window_s",
                     "health_timeout_s", "startup_grace_s"):
            value = float(getattr(self, name))
            setattr(self, name, value)
            if value < 0.0 or not math.isfinite(value):
                raise ValueError(f"{name} must be finite and >= 0")
        if self.speed_window_s <= 0.0:
            raise ValueError("speed_window_s must be > 0")
        if self.max_tilt_deg > 180.0:
            raise ValueError("max_tilt_deg must be <= 180")

    def describe(self) -> Dict[str, object]:
        half = self.box.half_extents()
        return {
            "box_size_m": list(self.box.size),
            "box_center_m": list(self.box.center),
            "fence_margin_m": self.box.margin,
            "fence_half_extents_m": [round(v, 3) for v in half],
            "mocap_timeout_s": self.mocap_timeout_s,
            "pose_timeout_s": self.pose_timeout_s,
            "max_speed_mps": self.max_speed_mps,
            "max_tilt_deg": self.max_tilt_deg,
            "max_pose_delta_m": self.max_pose_delta_m,
            "speed_window_s": self.speed_window_s,
            "startup_grace_s": self.startup_grace_s,
        }


@dataclass(frozen=True)
class ConditionPolicy:
    """How one condition escalates.

    `confirm_s` is the dwell before the FIRST stage; `escalate_s` the dwell
    between stages. Both require the condition to be CONTINUOUSLY true: any
    clean sample restarts the clock, which is what makes a single noisy sample
    incapable of disarming an aircraft.
    """
    name: str
    ladder: Tuple[str, ...] = DEFAULT_LADDER
    confirm_s: float = 0.2
    escalate_s: float = 1.0
    enabled: bool = True

    def __post_init__(self) -> None:
        if self.name not in CONDITION_INDEX:
            raise ValueError(f"unknown condition '{self.name}'; known: "
                             + ", ".join(CONDITIONS))
        if self.enabled and not self.ladder:
            raise ValueError(f"{self.name}: an enabled condition needs a ladder")
        ranks = [STATE_RANK.get(stage) for stage in self.ladder]
        if any(rank is None for rank in ranks):
            raise ValueError(f"{self.name}: ladder stages must be from "
                             + "+".join(s.lower() for s in STAGES))
        if any(b <= a for a, b in zip(ranks, ranks[1:])):
            # An "escalation" ladder that does not escalate is a configuration
            # error, not a preference: the dwell logic only ever moves forward.
            raise ValueError(f"{self.name}: ladder must strictly escalate, got "
                             + "+".join(self.ladder))
        if self.confirm_s < 0.0 or self.escalate_s < 0.0:
            raise ValueError(f"{self.name}: dwell times must be >= 0")

    def describe(self) -> str:
        if not self.enabled:
            return f"{self.name}: DISABLED"
        return (f"{self.name}: {'+'.join(s.lower() for s in self.ladder)} "
                f"confirm {self.confirm_s:g}s escalate {self.escalate_s:g}s")


def default_policies() -> Dict[str, ConditionPolicy]:
    """The shipped ladder. These numbers belong on the run-sheet card; they are
    arguments, not constants, and every one of them is overridable."""
    return {
        # Losing the only localisation source. Land promptly, but the pilot gets
        # a full second of warning before the graceful action and another before
        # the ungraceful one.
        C_MOCAP_TIMEOUT: ConditionPolicy(C_MOCAP_TIMEOUT, DEFAULT_LADDER, 0.2, 1.0),
        C_MOCAP_UNTRACKED: ConditionPolicy(C_MOCAP_UNTRACKED, DEFAULT_LADDER, 0.3, 1.0),
        # The net is 0.5 m away and 1.5 m/s covers that in a third of a second,
        # so the fence ladder is the fastest one here.
        C_GEOFENCE: ConditionPolicy(C_GEOFENCE, DEFAULT_LADDER, 0.1, 0.5),
        # Speed and tilt are differentiated/trigonometric quantities and the
        # noisiest inputs, so they get the longest confirmation.
        C_OVERSPEED: ConditionPolicy(C_OVERSPEED, DEFAULT_LADDER, 0.3, 1.5),
        C_TILT: ConditionPolicy(C_TILT, DEFAULT_LADDER, 0.3, 1.5),
        C_POSE_MISMATCH: ConditionPolicy(C_POSE_MISMATCH, DEFAULT_LADDER, 0.5, 1.5),
        C_POSE_TIMEOUT: ConditionPolicy(C_POSE_TIMEOUT, DEFAULT_LADDER, 0.3, 1.0),
    }


@dataclass
class ConditionTracker:
    """The dwell and latch machine for one condition on one drone."""
    policy: ConditionPolicy
    tripped_since: Optional[float] = None
    stage_index: int = -1               # -1 = below the first stage
    stage_since: float = 0.0
    trips: int = 0                      # how many times it has gone true
    detail: str = ""
    currently_tripped: bool = False

    @property
    def name(self) -> str:
        return self.policy.name

    @property
    def stage(self) -> str:
        if self.stage_index < 0:
            return OK
        return self.policy.ladder[self.stage_index]

    def update(self, tripped: bool, now: float, detail: str = "") -> bool:
        """Feed one evaluation. Returns True if the stage advanced this call."""
        self.currently_tripped = bool(tripped)
        if not tripped:
            # Restart the dwell clock. The stage is NOT lowered -- latching is
            # the point: an incident does not un-happen because the symptom
            # stopped, and mid-incident self-clearing is how a guard talks a
            # crew out of an abort.
            self.tripped_since = None
            return False

        self.detail = detail or self.detail
        if self.tripped_since is None:
            self.tripped_since = now
            self.trips += 1

        if self.stage_index < 0:
            if now - self.tripped_since >= self.policy.confirm_s - LIMIT_EPS:
                self.stage_index = 0
                self.stage_since = now
                return True
            return False

        if self.stage_index >= len(self.policy.ladder) - 1:
            return False

        # Dwell from whichever is later: entering this stage, or the condition
        # becoming true again after a clean patch. A condition that flickers
        # must not accumulate credit towards a disarm.
        since = max(self.stage_since, self.tripped_since)
        if now - since >= self.policy.escalate_s - LIMIT_EPS:
            # One stage per update, never two. If this node stalls for five
            # seconds it must still issue LAND and give it its dwell before
            # DISARM; skipping the graceful action because a timer was late is
            # not a trade this node gets to make.
            self.stage_index += 1
            self.stage_since = now
            return True
        return False

    def reset(self, now: float) -> None:
        self.tripped_since = None
        self.stage_index = -1
        self.stage_since = now
        self.currently_tripped = False
        self.detail = ""

    def status(self, now: float) -> Dict[str, object]:
        return {
            "enabled": self.policy.enabled,
            "tripped": self.currently_tripped,
            "stage": self.stage,
            "ladder": [s for s in self.policy.ladder],
            "confirm_s": self.policy.confirm_s,
            "escalate_s": self.policy.escalate_s,
            "time_in_stage_s": (None if self.stage_index < 0
                                else round(now - self.stage_since, 3)),
            "trips": self.trips,
            "detail": self.detail,
        }


@dataclass
class Measurements:
    """What the guard measured for one drone at one instant. Every field may be
    None, which always means UNKNOWN and never means SAFE."""
    mocap_age_s: float = 0.0
    pose_age_s: float = 0.0
    position: Optional[Tuple[float, float, float]] = None
    speed_mps: Optional[float] = None
    tilt_deg: Optional[float] = None
    pose_delta_m: Optional[float] = None
    tracked: Optional[bool] = None
    mocap_samples: int = 0
    pose_samples: int = 0

    def status(self) -> Dict[str, object]:
        def r(value: Optional[float], places: int = 3) -> Optional[float]:
            return None if value is None else round(float(value), places)
        return {
            "mocap_age_s": r(self.mocap_age_s, 4),
            "pose_age_s": r(self.pose_age_s, 4),
            "position": None if self.position is None
                        else [round(float(v), 3) for v in self.position],
            "speed_mps": r(self.speed_mps),
            "tilt_deg": r(self.tilt_deg, 2),
            "pose_delta_m": r(self.pose_delta_m),
            "tracked": self.tracked,
            "mocap_samples": self.mocap_samples,
            "pose_samples": self.pose_samples,
        }


@dataclass(frozen=True)
class Action:
    """One thing the node must actually do. The core decides; the node acts.

    `repeat` is 0 for the first issue of a stage and counts up for the
    deliberate re-sends of a DISARM.
    """
    drone: str
    stage: str
    conditions: Tuple[str, ...]
    detail: str = ""
    repeat: int = 0

    def describe(self) -> str:
        tag = f" (repeat {self.repeat})" if self.repeat else ""
        drivers = ", ".join(self.conditions) or "unknown"
        return f"{self.stage} {self.drone}{tag}: {drivers}" + (
            f" -- {self.detail}" if self.detail else "")


class DroneGuard:
    """One aircraft's independent state machine.

    Holds only what this guard measured itself. Nothing here is shared with
    another drone, which is what makes "one drone tripping must not act on
    another" a structural property rather than a promise.
    """

    def __init__(self, namespace: str, body: str,
                 policies: Dict[str, ConditionPolicy], started_at: float) -> None:
        self.ns = namespace
        self.body = body
        self.started_at = started_at
        self.conditions: Dict[str, ConditionTracker] = {
            name: ConditionTracker(policies[name]) for name in CONDITIONS
        }
        # (arrival time on OUR clock, position). Header stamps are deliberately
        # not stored: a re-stamped frozen pose is the failure mode.
        self.mocap_history: List[Tuple[float, Tuple[float, float, float]]] = []
        self.mocap_recv_s: Optional[float] = None
        self.mocap_quat: Optional[Tuple[float, float, float, float]] = None
        self.mocap_samples = 0
        self.pose_position: Optional[Tuple[float, float, float]] = None
        self.pose_recv_s: Optional[float] = None
        self.pose_samples = 0
        self.tracked: Optional[bool] = None
        self.tracked_recv_s: Optional[float] = None

        self.state: str = OK
        self.state_since: float = started_at
        self.disarm_sent: int = 0
        self.land_sent: bool = False
        self.log: List[str] = []

    # -- inputs ------------------------------------------------------------ #

    def on_mocap(self, position: Sequence[float],
                 quaternion: Optional[Sequence[float]], now: float,
                 speed_window_s: float) -> None:
        pos = (float(position[0]), float(position[1]), float(position[2]))
        self.mocap_recv_s = now
        self.mocap_samples += 1
        if quaternion is not None:
            self.mocap_quat = (float(quaternion[0]), float(quaternion[1]),
                               float(quaternion[2]), float(quaternion[3]))
        self.mocap_history.append((now, pos))
        # Keep exactly one sample older than the window, so the baseline spans
        # the full window at 100 Hz and still exists at 10 Hz.
        while (len(self.mocap_history) > 2
               and now - self.mocap_history[1][0] > speed_window_s):
            self.mocap_history.pop(0)

    def on_pose(self, position: Sequence[float], now: float) -> None:
        self.pose_position = (float(position[0]), float(position[1]),
                              float(position[2]))
        self.pose_recv_s = now
        self.pose_samples += 1

    def on_health(self, tracked: Optional[bool], now: float) -> None:
        if tracked is None:
            return
        self.tracked = bool(tracked)
        self.tracked_recv_s = now

    # -- measurement ------------------------------------------------------- #

    def measure(self, now: float, limits: Limits) -> Measurements:
        m = Measurements(mocap_samples=self.mocap_samples,
                         pose_samples=self.pose_samples)
        # Never received counts as "aged since the guard started", so a body
        # that never appears trips exactly like one that stopped.
        m.mocap_age_s = now - (self.mocap_recv_s
                               if self.mocap_recv_s is not None else self.started_at)
        m.pose_age_s = now - (self.pose_recv_s
                              if self.pose_recv_s is not None else self.started_at)

        if self.mocap_history:
            m.position = self.mocap_history[-1][1]

        if len(self.mocap_history) >= 2:
            (t0, p0), (t1, p1) = self.mocap_history[0], self.mocap_history[-1]
            dt = t1 - t0
            # Too short to differentiate, or spanning a dropout: report UNKNOWN
            # rather than the enormous or vanishing speed either would produce.
            # The staleness conditions already cover the dropout itself.
            if dt >= 1e-3 and dt <= 3.0 * limits.speed_window_s:
                m.speed_mps = distance(p1, p0) / dt

        if self.mocap_quat is not None:
            m.tilt_deg = tilt_deg_from_quaternion(*self.mocap_quat)

        if (self.tracked_recv_s is not None
                and now - self.tracked_recv_s <= limits.health_timeout_s):
            m.tracked = self.tracked

        # Only compare two FRESH sources. Differencing a stale pose against a
        # live one manufactures a disagreement that is really a dropout, and
        # reporting the wrong condition to the crew is its own failure.
        if (m.position is not None and self.pose_position is not None
                and m.mocap_age_s < limits.mocap_timeout_s
                and m.pose_age_s < limits.pose_timeout_s):
            m.pose_delta_m = distance(self.pose_position, m.position)
        return m

    def evaluate(self, now: float, limits: Limits
                 ) -> Tuple[Measurements, Dict[str, Tuple[bool, str]]]:
        """The trip conditions, evaluated raw. Latching, dwell and the startup
        grace are applied by GuardCore, so status can show both."""
        m = self.measure(now, limits)
        out: Dict[str, Tuple[bool, str]] = {}

        tripped = at_or_above(m.mocap_age_s, limits.mocap_timeout_s)
        seen = "" if self.mocap_recv_s is not None else " (never received)"
        out[C_MOCAP_TIMEOUT] = (
            tripped,
            f"mocap age {m.mocap_age_s * 1000:.0f} ms vs "
            f"{limits.mocap_timeout_s * 1000:.0f} ms{seen}")

        out[C_MOCAP_UNTRACKED] = (
            m.tracked is False,
            f"/mocap/health reports '{self.body}' NOT TRACKED"
            if m.tracked is False else f"tracked={m.tracked}")

        if m.position is None:
            out[C_GEOFENCE] = (False, "no mocap position yet")
        else:
            # The LAST KNOWN position is used even when it is stale: an aircraft
            # that left the fence and then lost tracking has still left the
            # fence.
            outside, why = limits.box.outside(m.position)
            out[C_GEOFENCE] = (outside, why or "inside the fence")

        if m.speed_mps is None:
            out[C_OVERSPEED] = (False, "speed unknown")
        else:
            out[C_OVERSPEED] = (
                at_or_above(m.speed_mps, limits.max_speed_mps),
                f"speed {m.speed_mps:.2f} m/s vs {limits.max_speed_mps:.2f} m/s")

        if m.tilt_deg is None:
            out[C_TILT] = (False, "tilt unknown")
        else:
            out[C_TILT] = (
                at_or_above(m.tilt_deg, limits.max_tilt_deg),
                f"tilt {m.tilt_deg:.1f} deg vs {limits.max_tilt_deg:.1f} deg")

        if m.pose_delta_m is None:
            out[C_POSE_MISMATCH] = (False, "not comparable (a source is stale)")
        else:
            out[C_POSE_MISMATCH] = (
                at_or_above(m.pose_delta_m, limits.max_pose_delta_m),
                f"self_localization is {m.pose_delta_m * 100:.0f} cm from mocap "
                f"vs {limits.max_pose_delta_m * 100:.0f} cm")

        seen = "" if self.pose_recv_s is not None else " (never received)"
        out[C_POSE_TIMEOUT] = (
            at_or_above(m.pose_age_s, limits.pose_timeout_s),
            f"self_localization/pose age {m.pose_age_s * 1000:.0f} ms vs "
            f"{limits.pose_timeout_s * 1000:.0f} ms{seen}")
        return m, out

    # -- state ------------------------------------------------------------- #

    def demanded_stage(self) -> Tuple[str, Tuple[str, ...]]:
        """The highest stage any condition has reached, and which reached it."""
        best = OK
        for tracker in self.conditions.values():
            if STATE_RANK[tracker.stage] > STATE_RANK[best]:
                best = tracker.stage
        if best == OK:
            return OK, ()
        drivers = tuple(sorted(name for name, t in self.conditions.items()
                               if t.stage == best))
        return best, drivers

    def reset(self, now: float) -> None:
        for tracker in self.conditions.values():
            tracker.reset(now)
        self.state = OK
        self.state_since = now
        self.disarm_sent = 0
        self.land_sent = False
        self.note(now, "RESET -- latched state cleared by operator")

    def note(self, now: float, text: str) -> None:
        self.log.append(f"t={now:.3f} {text}")
        del self.log[:-20]          # the last few events, not a flight recorder

    def status(self, now: float, m: Measurements) -> Dict[str, object]:
        _, drivers = self.demanded_stage()
        return {
            "state": self.state,
            "time_in_state_s": round(now - self.state_since, 3),
            "latched": self.state != OK,
            "driving_conditions": list(drivers),
            "tripped_now": sorted(name for name, t in self.conditions.items()
                                  if t.currently_tripped),
            "rigid_body": self.body,
            "measurements": m.status(),
            "conditions": {name: t.status(now)
                           for name, t in self.conditions.items()},
            "disarms_sent": self.disarm_sent,
            "land_sent": self.land_sent,
            "recent": list(self.log[-5:]),
        }


class GuardCore:
    """Every drone's state machine, one clock, no ROS.

    The node feeds it samples and calls `update(now)` on its own timer;
    `update` returns the actions to perform and performs none of them itself.
    That split is what makes the whole escalation ladder testable on a laptop.
    """

    def __init__(self, drones: Sequence[str],
                 rigid_bodies: Optional[Dict[str, str]] = None,
                 limits: Optional[Limits] = None,
                 policies: Optional[Dict[str, ConditionPolicy]] = None,
                 started_at: float = 0.0,
                 disarm_repeat: int = DEFAULT_DISARM_REPEAT) -> None:
        self.drones = [str(d) for d in drones]
        if not self.drones:
            raise ValueError("no drones configured")
        if len(set(self.drones)) != len(self.drones):
            raise ValueError("drone namespaces must be unique")
        self.limits = limits if limits is not None else Limits()
        self.policies = policies if policies is not None else default_policies()
        missing = [c for c in CONDITIONS if c not in self.policies]
        if missing:
            raise ValueError("no policy for condition(s): " + ", ".join(missing))
        self.rigid_bodies = dict(rigid_bodies or {d: d for d in self.drones})
        for ns in self.drones:
            self.rigid_bodies.setdefault(ns, ns)
        self.started_at = float(started_at)
        self.disarm_repeat = max(1, int(disarm_repeat))
        self.body_to_drone: Dict[str, str] = {
            self.rigid_bodies[ns]: ns for ns in self.drones}
        self.guards: Dict[str, DroneGuard] = {
            ns: DroneGuard(ns, self.rigid_bodies[ns], self.policies, self.started_at)
            for ns in self.drones}

    # -- inputs ------------------------------------------------------------ #

    def on_mocap_body(self, body: str, position: Sequence[float],
                      quaternion: Optional[Sequence[float]], now: float) -> bool:
        """Feed one rigid body. Returns False for a body we do not watch."""
        ns = self.body_to_drone.get(str(body))
        if ns is None:
            return False
        self.guards[ns].on_mocap(position, quaternion, now,
                                 self.limits.speed_window_s)
        return True

    def on_pose(self, drone: str, position: Sequence[float], now: float) -> bool:
        guard = self.guards.get(drone)
        if guard is None:
            return False
        guard.on_pose(position, now)
        return True

    def on_health(self, health: object, now: float) -> None:
        for ns, guard in self.guards.items():
            guard.on_health(health_tracked(health, self.rigid_bodies[ns]), now)

    # -- the tick ---------------------------------------------------------- #

    def in_grace(self, now: float) -> bool:
        return (now - self.started_at) < self.limits.startup_grace_s

    def update(self, now: float) -> List[Action]:
        """Advance every drone independently. Returns the actions to perform."""
        grace = self.in_grace(now)
        actions: List[Action] = []
        for ns in self.drones:
            guard = self.guards[ns]
            _, evals = guard.evaluate(now, self.limits)
            for name, tracker in guard.conditions.items():
                tripped, detail = evals[name]
                if grace or not tracker.policy.enabled:
                    tripped = False
                tracker.update(tripped, now, detail)

            demanded, drivers = guard.demanded_stage()
            if STATE_RANK[demanded] > STATE_RANK[guard.state]:
                guard.state = demanded
                guard.state_since = now
                detail = "; ".join(guard.conditions[c].detail for c in drivers)
                action = Action(ns, demanded, drivers, detail, 0)
                guard.note(now, action.describe())
                actions.append(action)
                if demanded == LAND:
                    guard.land_sent = True
                if demanded == DISARM:
                    guard.disarm_sent = 1
            elif guard.state == DISARM and guard.disarm_sent < self.disarm_repeat:
                # Deliberate re-send. /fmu/in/* is best-effort; one lost
                # datagram must not be the difference between a disarm and a
                # net strike.
                actions.append(Action(ns, DISARM, drivers, "re-send",
                                      guard.disarm_sent))
                guard.disarm_sent += 1
        return actions

    def reset(self, target: Optional[str], now: float) -> List[str]:
        """Clear latched state for one drone, or for all of them.

        A reset while the condition is still true simply re-trips after the
        confirm dwell. That is intended: reset means "I have seen it", not
        "it did not happen".
        """
        if target in (None, "", "all", "*"):
            chosen = list(self.drones)
        elif target in self.guards:
            chosen = [target]
        else:
            raise ValueError(f"no such drone '{target}'; configured: "
                             + ", ".join(self.drones))
        for ns in chosen:
            self.guards[ns].reset(now)
        return chosen

    def status(self, now: float) -> Dict[str, object]:
        payload: Dict[str, object] = {
            "t_s": round(now, 3),
            "uptime_s": round(now - self.started_at, 3),
            "startup_grace_active": self.in_grace(now),
            "limits": self.limits.describe(),
            "drones": {},
        }
        for ns, guard in self.guards.items():
            m, _ = guard.evaluate(now, self.limits)
            payload["drones"][ns] = guard.status(now, m)   # type: ignore[index]
        return payload

    def any_latched(self) -> bool:
        return any(g.state != OK for g in self.guards.values())


# --------------------------------------------------------------------------- #
# the disarm, built as data so it can be asserted without px4_msgs
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class DisarmCommand:
    """Exactly the px4_msgs/VehicleCommand this node publishes.

    Built as a plain dataclass so --self-test can assert every field on a
    machine with no ROS. The field values mirror the platform's
    `PX4publishVehicleCommand()` helper (see PATCHES.md, Change 5), because a
    command that PX4 routes differently from the platform's own arm/disarm
    traffic is a command that has not been tested by anything else in the stack.
    """
    command: int = PX4_VEHICLE_CMD_COMPONENT_ARM_DISARM   # 400
    param1: float = PX4_ARMING_ACTION_DISARM              # 0.0 -> disarm
    param2: float = PX4_FORCE_DISARM_MAGIC                # 21196.0 -> forced
    target_system: int = DEFAULT_TARGET_SYSTEM
    target_component: int = 1
    source_system: int = 1
    source_component: int = 1
    from_external: bool = True
    timestamp: int = 0                                    # microseconds


def build_disarm_command(target_system: int = DEFAULT_TARGET_SYSTEM,
                         timestamp_us: int = 0) -> DisarmCommand:
    """The force-disarm. param2 == 21196 is what makes PX4 skip the landed
    check and cut the motors in the air; without it the command is refused in
    flight, which would make this whole node ornamental."""
    return DisarmCommand(target_system=int(target_system),
                         timestamp=int(timestamp_us))


VEHICLE_COMMAND_FIELDS: Tuple[str, ...] = (
    "command", "param1", "param2", "target_system", "target_component",
    "source_system", "source_component", "from_external", "timestamp")


def fill_vehicle_command(msg: object, cmd: DisarmCommand) -> object:
    """Copy the command onto a real px4_msgs/VehicleCommand.

    Every field is asserted present rather than set best-effort: a silently
    dropped `param2` turns a force-disarm into a normal disarm, which PX4
    refuses in flight. That failure would be invisible until the one moment it
    mattered.
    """
    for name in VEHICLE_COMMAND_FIELDS:
        if not hasattr(msg, name):
            raise AttributeError(
                f"VehicleCommand has no field '{name}' -- px4_msgs is not the "
                "2.0.1 (release/1.17) build this node was written against")
        setattr(msg, name, getattr(cmd, name))
    return msg


# --------------------------------------------------------------------------- #
# configuration parsing -- fails on the command line, never at 50 Hz
# --------------------------------------------------------------------------- #

def parse_drones(spec: object) -> List[str]:
    if isinstance(spec, (list, tuple)):
        names = [str(s).strip() for s in spec]
    else:
        names = [s.strip() for s in str(spec or "").split(",")]
    names = [n for n in names if n]
    if not names:
        raise ValueError("no drone namespaces selected")
    if len(set(names)) != len(names):
        raise ValueError("drone namespaces must be unique")
    return names


def parse_rigid_bodies(spec: Optional[str], drones: Sequence[str]) -> Dict[str, str]:
    """Map drone namespace -> mocap rigid_body_name.

    Same grammar as preflight_check.py: 'drone0:1,drone1:2' or positional
    'A,B,C'. Empty means the rigid body is named after the namespace.
    """
    if not spec or not str(spec).strip():
        return {d: d for d in drones}
    entries = [e.strip() for e in str(spec).split(",") if e.strip()]
    if all(":" in e for e in entries):
        mapping: Dict[str, str] = {}
        for entry in entries:
            ns, _, body = entry.partition(":")
            mapping[ns.strip()] = body.strip()
        unknown = sorted(set(mapping) - set(drones))
        if unknown:
            raise ValueError("--rigid-bodies names unknown drone(s): "
                             + ", ".join(unknown))
        missing = [d for d in drones if d not in mapping]
        if missing:
            raise ValueError("--rigid-bodies has no entry for: " + ", ".join(missing))
        if len(set(mapping.values())) != len(mapping):
            raise ValueError("--rigid-bodies maps two drones to the same body")
        return mapping
    if any(":" in e for e in entries):
        raise ValueError("--rigid-bodies must be either all 'ns:body' pairs or "
                         "all positional names, not a mixture")
    if len(entries) != len(drones):
        raise ValueError(f"--rigid-bodies has {len(entries)} names but "
                         f"{len(drones)} drone(s) were selected")
    return dict(zip(drones, entries))


def parse_target_systems(spec: Optional[str], drones: Sequence[str]) -> Dict[str, int]:
    """Map drone namespace -> PX4 target_system id.

    Wrong here means a disarm reaches the wrong airframe, so it is explicit and
    validated rather than inferred.
    """
    if not spec or not str(spec).strip():
        return {d: DEFAULT_TARGET_SYSTEM for d in drones}
    entries = [e.strip() for e in str(spec).split(",") if e.strip()]
    mapping: Dict[str, int] = {d: DEFAULT_TARGET_SYSTEM for d in drones}
    positional = [e for e in entries if ":" not in e]
    if positional and len(positional) != len(entries):
        raise ValueError("--target-systems must be either all 'ns:id' pairs or "
                         "all positional ids, not a mixture")
    if positional:
        if len(positional) != len(drones):
            raise ValueError(f"--target-systems has {len(positional)} ids but "
                             f"{len(drones)} drone(s) were selected")
        entries = [f"{ns}:{value}" for ns, value in zip(drones, positional)]
    for entry in entries:
        ns, _, raw = entry.partition(":")
        ns = ns.strip()
        if ns not in mapping:
            raise ValueError(f"--target-systems names unknown drone '{ns}'")
        try:
            value = int(raw.strip())
        except ValueError:
            raise ValueError(f"--target-systems: '{raw.strip()}' is not an "
                             "integer system id") from None
        if not 1 <= value <= 255:
            raise ValueError("--target-systems: system ids are 1..255")
        mapping[ns] = value
    return mapping


def parse_fmu_prefix_map(spec: Optional[str], drones: Sequence[str]) -> Dict[str, str]:
    """Map drone namespace -> FMU topic prefix, e.g. 'drone0=/uav_0,drone1=/uav_1'.

    Aerostack2 calls the aircraft `drone0`; PX4's UXRCE_DDS_NS_IDX calls the
    SAME aircraft's FMU topics `/uav_0`. Nothing derives one name from the
    other, so no form of --fmu-prefix can express it: a bare prefix aims three
    drones at one topic, and '/{ns}' aims them at topics that do not exist --
    which the guard would then watch, and report as a permanent pose_timeout.
    A static ROS remap can redirect the publisher, but then this node's log
    lines and /volume_guard/status still name the PRE-REMAP topic: a safety
    node telling the operator it publishes somewhere it does not.

    Only the drones named here are overridden; a drone with no entry falls back
    to --fmu-prefix. A drone named here that is NOT being watched is an error,
    because it means the crew believes a mapping is in force that is not.
    """
    if not spec or not str(spec).strip():
        return {}
    entries = [e.strip() for e in str(spec).split(",") if e.strip()]
    mapping: Dict[str, str] = {}
    known = set(str(d) for d in drones)
    for entry in entries:
        ns, sep, raw = entry.partition("=")
        ns = ns.strip()
        if not sep or not ns:
            raise ValueError(f"--fmu-prefix-map: '{entry}' is not an "
                             "'ns=prefix' pair")
        if ns not in known:
            raise ValueError(f"--fmu-prefix-map names unknown drone '{ns}'")
        if ns in mapping:
            raise ValueError(f"--fmu-prefix-map names '{ns}' twice")
        prefix = raw.strip()
        if not prefix:
            raise ValueError(f"--fmu-prefix-map: '{ns}' has an empty prefix; "
                             "omit the entry to fall back to --fmu-prefix")
        mapping[ns] = prefix
    return mapping


def parse_policy_spec(text: str, policies: Dict[str, ConditionPolicy]
                      ) -> Dict[str, ConditionPolicy]:
    """`NAME:LADDER[:CONFIRM[:ESCALATE]]` -> the policies it changes.

    NAME may be `*` to apply to every condition. Any field may be left empty to
    keep the default, e.g. `tilt::0.5` changes only the confirm dwell. LADDER
    `none` disables the condition entirely.
    """
    raw = str(text).strip()
    if not raw:
        raise ValueError("empty policy specification")
    parts = [p.strip() for p in raw.split(":")]
    if len(parts) > 4:
        raise ValueError(f"malformed policy '{raw}'; expected "
                         "NAME:LADDER[:CONFIRM[:ESCALATE]]")
    name = parts[0]
    targets = list(CONDITIONS) if name in ("*", "all") else [name]
    for target in targets:
        if target not in policies:
            raise ValueError(f"unknown condition '{target}'; known: "
                             + ", ".join(CONDITIONS))

    ladder_text = parts[1] if len(parts) > 1 else ""
    confirm_text = parts[2] if len(parts) > 2 else ""
    escalate_text = parts[3] if len(parts) > 3 else ""

    updates: Dict[str, ConditionPolicy] = {}
    for target in targets:
        policy = policies[target]
        changes: Dict[str, object] = {}
        if ladder_text:
            if ladder_text.lower() in ("none", "off", "disabled"):
                changes["enabled"] = False
            else:
                stages = []
                for token in ladder_text.split("+"):
                    token = token.strip().upper()
                    if token not in STAGES:
                        raise ValueError(
                            f"policy '{raw}': unknown stage '{token}'; stages are "
                            + "+".join(s.lower() for s in STAGES))
                    stages.append(token)
                changes["ladder"] = tuple(stages)
                changes["enabled"] = True
        if confirm_text:
            changes["confirm_s"] = _as_float(confirm_text, "confirm dwell")
        if escalate_text:
            changes["escalate_s"] = _as_float(escalate_text, "escalate dwell")
        if not changes:
            raise ValueError(f"policy '{raw}' changes nothing")
        updates[target] = replace(policy, **changes)   # re-runs validation
    return updates


def apply_policy_specs(specs: Sequence[str],
                       policies: Optional[Dict[str, ConditionPolicy]] = None
                       ) -> Dict[str, ConditionPolicy]:
    result = dict(policies if policies is not None else default_policies())
    for spec in specs:
        result.update(parse_policy_spec(spec, result))
    return result


def _as_float(text: str, label: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise ValueError(f"{label}: '{text}' is not a number") from None
    if value < 0.0 or not math.isfinite(value):
        raise ValueError(f"{label}: must be finite and >= 0")
    return value


def resolve_fmu_topic(prefix: str, namespace: str,
                      overrides: Optional[Dict[str, str]] = None) -> str:
    """`--fmu-prefix` follows preflight_check.py exactly: a bare prefix is
    prepended, and a prefix containing '{ns}' is resolved per drone.

    `overrides` is --fmu-prefix-map. An entry for this namespace wins over both
    and is used VERBATIM -- no '{ns}' substitution -- because the whole point of
    the map is the case where no template relates the Aerostack2 namespace to
    the PX4 one. With no entry for this drone the behaviour is unchanged, so
    every caller that passes no overrides keeps exactly the semantics it had.
    """
    if overrides is not None and namespace in overrides:
        return str(overrides[namespace]) + FMU_VEHICLE_COMMAND_TOPIC
    prefix = str(prefix or "")
    base = prefix.format(ns=namespace) if "{ns}" in prefix else prefix
    return base + FMU_VEHICLE_COMMAND_TOPIC


def check_disarm_routing(drones: Sequence[str], fmu_prefix: str,
                         target_systems: Dict[str, int],
                         fmu_prefix_map: Optional[Dict[str, str]] = None
                         ) -> Optional[str]:
    """Refuse a configuration in which a disarm cannot be aimed.

    "One drone tripping must not act on another" is the requirement. A
    VehicleCommand can only reach an aircraft it was not meant for when TWO
    things are true at once: that aircraft listens on the same
    `/fmu/in/vehicle_command` topic, AND it answers to the same
    `target_system`, because PX4 drops a command addressed to another system
    id. So the drones are grouped by the topic each one RESOLVES to -- which is
    exactly what --fmu-prefix-map changes -- and a group is refused only when
    two of its members also share an id.

    Distinct topics therefore end the question: a drone on its own /uav_N
    namespace cannot be hit by another drone's disarm whatever its id, which is
    what the old '{ns}' special case was a narrower way of saying. A shared
    topic with colliding ids is still refused, because there a geofence trip on
    drone0 drops drone1 out of the air.
    """
    if len(drones) < 2:
        return None
    routes: Dict[str, List[str]] = {}
    for ns in drones:
        routes.setdefault(
            resolve_fmu_topic(fmu_prefix, ns, fmu_prefix_map), []).append(ns)
    for topic, sharing in routes.items():
        if len(sharing) < 2:
            continue
        ids = [target_systems.get(ns, DEFAULT_TARGET_SYSTEM) for ns in sharing]
        if len(set(ids)) == len(ids):
            continue
        listed = ", ".join(f"{ns} sys {i}" for ns, i in zip(sharing, ids))
        return (f"{len(sharing)} drones share the single topic {topic} and do "
                f"not have distinct target_system ids ({listed}). "
                "A disarm would reach every aircraft on that link. Give each "
                "drone its own FMU namespace (--fmu-prefix '/{ns}', or "
                "--fmu-prefix-map drone0=/uav_0,drone1=/uav_1 when PX4's "
                "UXRCE_DDS_NS_IDX does not match the Aerostack2 namespaces), "
                "or distinct ids (--target-systems drone0:1,drone1:2,...). "
                "Override only with --allow-ambiguous-disarm, and only if you "
                "know the link is shared.")
    return None


# --------------------------------------------------------------------------- #
# ROS node -- rclpy and px4_msgs are imported HERE, never at module scope
# --------------------------------------------------------------------------- #

def main_ros(cli: argparse.Namespace, argv: Optional[List[str]] = None) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
    except ImportError as exc:
        # A traceback here reads as a broken tool. It is not: ROS simply is not
        # sourced. `--self-test` is the part that runs anywhere.
        print(f"volume_guard: ROS 2 is not available ({exc}). Source the "
              "workspace, or run --self-test / --list-conditions, which need "
              "no ROS.", file=sys.stderr)
        return 2

    class VolumeGuard(Node):
        def __init__(self) -> None:
            super().__init__("volume_guard")

            self.declare_parameter("drones", list(DEFAULT_DRONES))
            self.declare_parameter("rigid_bodies", "")
            self.declare_parameter("target_systems", "")
            self.declare_parameter("mocap_topic", DEFAULT_MOCAP_TOPIC)
            self.declare_parameter("health_topic", DEFAULT_HEALTH_TOPIC)
            self.declare_parameter("pose_topic_template", DEFAULT_POSE_TEMPLATE)
            self.declare_parameter("status_topic", DEFAULT_STATUS_TOPIC)
            self.declare_parameter("reset_topic", DEFAULT_RESET_TOPIC)
            self.declare_parameter("land_action_template",
                                   DEFAULT_LAND_ACTION_TEMPLATE)
            self.declare_parameter("fmu_prefix", "")
            self.declare_parameter("fmu_prefix_map", "")
            self.declare_parameter("box_size_m", list(DEFAULT_BOX_M))
            self.declare_parameter("box_center_m", list(DEFAULT_BOX_CENTER_M))
            self.declare_parameter("fence_margin_m", DEFAULT_FENCE_MARGIN_M)
            self.declare_parameter("mocap_timeout_s", DEFAULT_MOCAP_TIMEOUT_S)
            self.declare_parameter("pose_timeout_s", DEFAULT_POSE_TIMEOUT_S)
            self.declare_parameter("max_speed_mps", DEFAULT_MAX_SPEED_MPS)
            self.declare_parameter("max_tilt_deg", DEFAULT_MAX_TILT_DEG)
            self.declare_parameter("max_pose_delta_m", DEFAULT_MAX_POSE_DELTA_M)
            self.declare_parameter("speed_window_s", DEFAULT_SPEED_WINDOW_S)
            self.declare_parameter("startup_grace_s", DEFAULT_STARTUP_GRACE_S)
            self.declare_parameter("rate_hz", DEFAULT_RATE_HZ)
            self.declare_parameter("status_rate_hz", DEFAULT_STATUS_RATE_HZ)
            self.declare_parameter("land_speed_mps", DEFAULT_LAND_SPEED_MPS)
            self.declare_parameter("disarm_repeat", DEFAULT_DISARM_REPEAT)
            self.declare_parameter("policies", [""])

            # px4_msgs is the whole point of the DISARM stage. Without it the
            # node would come up looking healthy and be unable to do the one
            # thing nothing else in the stack can do, so it refuses instead.
            try:
                from px4_msgs.msg import VehicleCommand
            except ImportError:
                self.get_logger().fatal(
                    "px4_msgs is not available -- the DISARM stage would be "
                    "inert. Source the workspace that builds px4_msgs 2.0.1 "
                    "(release/1.17) before running the volume guard.")
                raise SystemExit(2)
            self._VehicleCommand = VehicleCommand

            try:
                from mocap4r2_msgs.msg import RigidBodies
            except ImportError:
                self.get_logger().fatal(
                    "mocap4r2_msgs is not available -- there is no independent "
                    "position source to guard with. Source the workspace.")
                raise SystemExit(2)

            try:
                drones = parse_drones(
                    cli.drones if cli.drones is not None
                    else list(self.get_parameter("drones")
                              .get_parameter_value().string_array_value))
                self.rigid_bodies = parse_rigid_bodies(
                    _pick(cli.rigid_bodies,
                          self.get_parameter("rigid_bodies").value), drones)
                self.target_systems = parse_target_systems(
                    _pick(cli.target_systems,
                          self.get_parameter("target_systems").value), drones)
                specs = [s for s in self.get_parameter("policies")
                         .get_parameter_value().string_array_value if s]
                specs += list(cli.policy or [])
                policies = apply_policy_specs(specs)
                box = Box(
                    size=tuple(_pick(cli.box,
                                     list(self.get_parameter("box_size_m").value))),
                    center=tuple(_pick(
                        cli.box_center,
                        list(self.get_parameter("box_center_m").value))),
                    margin=float(_pick(cli.margin,
                                       self.get_parameter("fence_margin_m").value)))
                limits = Limits(
                    box=box,
                    mocap_timeout_s=float(_pick(
                        cli.mocap_timeout,
                        self.get_parameter("mocap_timeout_s").value)),
                    pose_timeout_s=float(_pick(
                        cli.pose_timeout,
                        self.get_parameter("pose_timeout_s").value)),
                    max_speed_mps=float(_pick(
                        cli.max_speed, self.get_parameter("max_speed_mps").value)),
                    max_tilt_deg=float(_pick(
                        cli.max_tilt, self.get_parameter("max_tilt_deg").value)),
                    max_pose_delta_m=float(_pick(
                        cli.max_pose_delta,
                        self.get_parameter("max_pose_delta_m").value)),
                    speed_window_s=float(_pick(
                        cli.speed_window,
                        self.get_parameter("speed_window_s").value)),
                    startup_grace_s=float(_pick(
                        cli.startup_grace,
                        self.get_parameter("startup_grace_s").value)))
                self.fmu_prefix = str(_pick(
                    cli.fmu_prefix, self.get_parameter("fmu_prefix").value))
                self.fmu_prefix_map = parse_fmu_prefix_map(
                    _pick(cli.fmu_prefix_map,
                          self.get_parameter("fmu_prefix_map").value), drones)
                rate = float(_pick(cli.rate, self.get_parameter("rate_hz").value))
                status_rate = float(_pick(
                    cli.status_rate, self.get_parameter("status_rate_hz").value))
                if rate <= 0.0 or status_rate <= 0.0:
                    raise ValueError("rate_hz and status_rate_hz must be > 0")
                self.land_speed = float(_pick(
                    cli.land_speed, self.get_parameter("land_speed_mps").value))
                repeat = int(_pick(cli.disarm_repeat,
                                   self.get_parameter("disarm_repeat").value))
                routing_problem = check_disarm_routing(
                    drones, self.fmu_prefix, self.target_systems,
                    self.fmu_prefix_map)
                # Refused only when the actions are real. A dry run sends
                # nothing, so an ambiguous route is a warning there -- but it is
                # a warning the crew must see at the bench rather than discover
                # at the pad, which is why it is phrased as what will happen.
                if routing_problem and cli.arm and not cli.allow_ambiguous_disarm:
                    raise ValueError(routing_problem)
            except (ValueError, TypeError) as exc:
                self.get_logger().fatal(str(exc))
                raise SystemExit(2)

            self.dry_run = not bool(cli.arm)
            if routing_problem and self.dry_run:
                self.get_logger().warn(
                    "AMBIGUOUS DISARM ROUTING (harmless in dry run, REFUSED "
                    "with --arm): " + routing_problem)
            self.core = GuardCore(drones, self.rigid_bodies, limits, policies,
                                  started_at=self.now_s(), disarm_repeat=repeat)

            # BEST_EFFORT/VOLATILE subscribes successfully to reliable and
            # best-effort publishers alike, so a QoS mismatch can never present
            # as a dead mocap feed -- which this node would then treat as a
            # genuine dropout and act on.
            sensor_qos = QoSProfile(depth=20,
                                    history=HistoryPolicy.KEEP_LAST,
                                    reliability=ReliabilityPolicy.BEST_EFFORT,
                                    durability=DurabilityPolicy.VOLATILE)

            self.create_subscription(
                RigidBodies, str(self.get_parameter("mocap_topic").value),
                self.on_mocap, sensor_qos)
            self.create_subscription(
                String, str(self.get_parameter("health_topic").value),
                self.on_health, 10)
            pose_template = str(self.get_parameter("pose_topic_template").value)
            for ns in drones:
                self.create_subscription(
                    PoseStamped, pose_template.format(ns=ns),
                    lambda msg, n=ns: self.on_pose(n, msg), sensor_qos)

            self.status_pub = self.create_publisher(
                String, str(self.get_parameter("status_topic").value), 10)
            self.create_subscription(
                String, str(self.get_parameter("reset_topic").value),
                self.on_reset_topic, 10)
            self.create_service(Trigger, DEFAULT_RESET_SERVICE, self.on_reset_service)

            # One VehicleCommand publisher per drone. Matching the platform's
            # SensorDataQoS-equivalent is what the uXRCE-DDS agent expects on
            # /fmu/in/*; it is also why a disarm is re-sent rather than trusted.
            self.command_pubs = {}
            for ns in drones:
                # Through the same accessor the log lines and the status
                # payload use, so what is reported cannot drift from what is
                # bound.
                topic = self.command_topic(ns)
                self.command_pubs[ns] = self.create_publisher(
                    VehicleCommand, topic, sensor_qos)

            self.land_clients: Dict[str, object] = {}
            self._Land = None
            try:
                from rclpy.action import ActionClient
                from as2_msgs.action import Land
                self._Land = Land
                template = str(self.get_parameter("land_action_template").value)
                for ns in drones:
                    self.land_clients[ns] = ActionClient(
                        self, Land, template.format(ns=ns))
            except ImportError:
                # Degraded, not fatal: LAND stops working, DISARM does not, and
                # a guard that can still disarm is worth far more than one that
                # refused to start.
                self.get_logger().error(
                    "as2_msgs/action/Land is not available -- the LAND stage "
                    "will be logged and skipped. The ladder still reaches "
                    "DISARM. Source the Aerostack2 workspace.")

            self.create_timer(1.0 / rate, self.on_timer)
            self.create_timer(1.0 / status_rate, self.on_status)

            half = limits.box.half_extents()
            self.get_logger().info(
                f"watching {len(drones)} drone(s): "
                + ", ".join(f"{ns}->body '{self.rigid_bodies[ns]}' "
                            f"sys {self.target_systems[ns]} "
                            f"cmd {self.command_topic(ns)}"
                            for ns in drones))
            self.get_logger().info(
                f"fence +/-{half[0]:.2f}, {half[1]:.2f}, {half[2]:.2f} m about "
                f"({limits.box.center[0]:g}, {limits.box.center[1]:g}, "
                f"{limits.box.center[2]:g}); mocap timeout "
                f"{limits.mocap_timeout_s * 1000:.0f} ms; max speed "
                f"{limits.max_speed_mps:g} m/s; max tilt "
                f"{limits.max_tilt_deg:g} deg")
            for name in CONDITIONS:
                self.get_logger().info("  " + policies[name].describe())
            if self.dry_run:
                self.get_logger().warn(
                    "DRY RUN: land and disarm will be LOGGED AND NOT SENT. "
                    "Pass --arm to make the actions real.")
            else:
                self.get_logger().warn(
                    "ACTIONS ARMED: a DISARM will force-disarm the aircraft in "
                    "flight (param2=21196). THE AIRCRAFT WILL FALL.")
            self.get_logger().info(
                f"startup grace {limits.startup_grace_s:g} s; reset with "
                f"ros2 topic pub --once "
                f"{self.get_parameter('reset_topic').value} std_msgs/msg/String "
                "\"{data: 'all'}\"")

        # -- plumbing ------------------------------------------------------ #

        def now_s(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def command_topic(self, ns: str) -> str:
            """The topic this drone's disarm is ACTUALLY published on.

            Every log line, the dry-run message and the status payload go
            through here, so the operator can never be told one topic while
            the publisher is bound to another.
            """
            return resolve_fmu_topic(self.fmu_prefix, ns, self.fmu_prefix_map)

        def on_mocap(self, msg) -> None:
            # Arrival time on OUR clock. The header stamp is not read at all:
            # re-stamping a frozen pose is exactly the failure this guards.
            now = self.now_s()
            for body in getattr(msg, "rigidbodies", []):
                try:
                    name = str(body.rigid_body_name)
                    p = body.pose.position
                    q = body.pose.orientation
                except AttributeError:
                    self.get_logger().warn(
                        "message on the mocap topic is not RigidBodies",
                        throttle_duration_sec=5.0)
                    return
                self.core.on_mocap_body(name, (p.x, p.y, p.z),
                                        (q.x, q.y, q.z, q.w), now)

        def on_pose(self, ns: str, msg) -> None:
            try:
                p = msg.pose.position
            except AttributeError:
                return
            self.core.on_pose(ns, (p.x, p.y, p.z), self.now_s())

        def on_health(self, msg) -> None:
            try:
                payload = json.loads(msg.data)
            except (ValueError, TypeError):
                self.get_logger().warn("unparseable /mocap/health payload",
                                       throttle_duration_sec=10.0)
                return
            self.core.on_health(payload, self.now_s())

        def on_reset_topic(self, msg) -> None:
            self.do_reset((msg.data or "all").strip())

        def on_reset_service(self, request, response):
            count = len(self.do_reset("all"))
            response.success = True
            response.message = f"reset {count} drone(s)"
            return response

        def do_reset(self, target: str) -> List[str]:
            try:
                cleared = self.core.reset(None if target in ("all", "*") else target,
                                          self.now_s())
            except ValueError as exc:
                self.get_logger().warn(f"reset rejected: {exc}")
                return []
            self.get_logger().warn(
                "LATCH RESET for " + ", ".join(cleared)
                + " -- if the condition is still true it will re-trip")
            return cleared

        # -- the tick ------------------------------------------------------ #

        def on_timer(self) -> None:
            for action in self.core.update(self.now_s()):
                self.perform(action)

        def perform(self, action: Action) -> None:
            prefix = "[DRY RUN] would " if self.dry_run else ""
            if action.stage == WARN:
                # WARN never acts, in dry run or armed. It exists so the crew
                # hears the condition before anything is commanded.
                self.get_logger().warn(action.describe())
                return

            if action.stage == LAND:
                self.get_logger().error(prefix + action.describe())
                if not self.dry_run:
                    self.send_land(action.drone)
                return

            self.get_logger().error(
                prefix + "FORCE-" + action.describe()
                + "  -- THE AIRCRAFT WILL FALL")
            cmd = build_disarm_command(
                self.target_systems.get(action.drone, DEFAULT_TARGET_SYSTEM),
                timestamp_us=int(self.get_clock().now().nanoseconds // 1000))
            if self.dry_run:
                self.get_logger().error(
                    f"[DRY RUN] {self.command_topic(action.drone)} "
                    f"command={cmd.command} param1={cmd.param1} "
                    f"param2={cmd.param2} target_system={cmd.target_system}")
                return
            try:
                msg = fill_vehicle_command(self._VehicleCommand(), cmd)
            except AttributeError as exc:
                self.get_logger().fatal(f"cannot build the disarm: {exc}")
                return
            self.command_pubs[action.drone].publish(msg)

        def send_land(self, ns: str) -> None:
            """Fire the land behavior and return immediately.

            Never waits for the server and never waits for the result. If the
            behavior is unavailable or hangs, the ladder keeps running and
            reaches DISARM on its own timer -- which is the entire reason LAND
            is not the terminal stage.
            """
            client = self.land_clients.get(ns)
            if client is None or self._Land is None:
                self.get_logger().error(
                    f"{ns}: no land behavior client -- LAND skipped, the ladder "
                    "will continue to DISARM")
                return
            if not client.server_is_ready():
                self.get_logger().error(
                    f"{ns}: land behavior server is not ready -- LAND skipped, "
                    "the ladder will continue to DISARM")
                return
            goal = self._Land.Goal()
            if hasattr(goal, "land_speed"):
                goal.land_speed = float(self.land_speed)
            future = client.send_goal_async(goal)
            future.add_done_callback(
                lambda fut, n=ns: self.on_land_response(n, fut))

        def on_land_response(self, ns: str, future) -> None:
            try:
                accepted = future.result().accepted
            except Exception as exc:            # noqa: BLE001 -- never raise here
                self.get_logger().error(f"{ns}: land goal failed: {exc}")
                return
            if accepted:
                self.get_logger().warn(f"{ns}: land behavior accepted the goal")
            else:
                self.get_logger().error(
                    f"{ns}: land behavior REJECTED the goal -- the ladder will "
                    "continue to DISARM")

        def on_status(self) -> None:
            payload = self.core.status(self.now_s())
            payload["dry_run"] = self.dry_run
            payload["fmu_prefix"] = self.fmu_prefix
            payload["fmu_prefix_map"] = dict(self.fmu_prefix_map)
            # The RESOLVED topics, not just the prefix they were built
            # from: this is the same string the publisher was created with.
            payload["fmu_command_topics"] = {
                ns: self.command_topic(ns) for ns in self.core.drones}
            payload["target_systems"] = dict(self.target_systems)
            self.status_pub.publish(String(data=json.dumps(payload)))

    rclpy.init(args=argv)
    try:
        node = VolumeGuard()
    except SystemExit as exc:
        rclpy.shutdown()
        return int(exc.code or 2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _pick(cli_value, param_value):
    """CLI wins when given; otherwise the ROS parameter stands."""
    return param_value if cli_value is None else cli_value


# --------------------------------------------------------------------------- #
# --list-conditions
# --------------------------------------------------------------------------- #

def print_conditions() -> None:
    policies = default_policies()
    print("Trip conditions. Every threshold and every ladder is configurable.")
    print()
    print("  Ladder syntax:  --policy NAME:LADDER[:CONFIRM[:ESCALATE]]")
    print("    NAME      a condition, or '*' for all of them")
    print("    LADDER    warn+land+disarm, land+disarm, disarm, ... or 'none'")
    print("              to disable. Stages must strictly escalate.")
    print("    CONFIRM   seconds the condition must be CONTINUOUSLY true before")
    print("              the first stage. This is what stops one noisy sample.")
    print("    ESCALATE  seconds of continuous truth between stages.")
    print("    Empty fields keep the default:  --policy tilt::0.5")
    print()
    for doc in CONDITION_DOCS:
        policy = policies[doc.name]
        print(f"  {doc.name}")
        print(f"     limit: {doc.limit}")
        print(f"     ladder: {'+'.join(s.lower() for s in policy.ladder)}"
              f"   confirm {policy.confirm_s:g} s   escalate {policy.escalate_s:g} s"
              f"   -> DISARM at {policy.confirm_s + policy.escalate_s * (len(policy.ladder) - 1):g} s")
        print(f"     {doc.why}")
        print()
    print("Stages")
    print("  WARN    log loudly and publish status. Nothing is commanded.")
    print("  LAND    the drone's Aerostack2 land behavior. Never waited on.")
    print("  DISARM  px4_msgs/VehicleCommand straight to PX4:")
    print(f"          command={PX4_VEHICLE_CMD_COMPONENT_ARM_DISARM} "
          f"(VEHICLE_CMD_COMPONENT_ARM_DISARM), "
          f"param1={PX4_ARMING_ACTION_DISARM:g} (ARMING_ACTION_DISARM), "
          f"param2={PX4_FORCE_DISARM_MAGIC:g} (PX4 force magic).")
    print(f"          Published on {FMU_VEHICLE_COMMAND_TOPIC}, namespaced by")
    print("          --fmu-prefix, or per drone by --fmu-prefix-map")
    print("          (drone0=/uav_0,drone1=/uav_1) when PX4's UXRCE_DDS_NS_IDX")
    print("          does not match the Aerostack2 namespaces.")
    print("          THE AIRCRAFT FALLS. Actions are DRY RUN unless --arm.")
    print()
    print("Everything latches. Clear with:")
    print(f"  ros2 topic pub --once {DEFAULT_RESET_TOPIC} std_msgs/msg/String "
          "\"{data: 'drone0'}\"")
    print(f"  ros2 service call {DEFAULT_RESET_SERVICE} std_srvs/srv/Trigger")


# --------------------------------------------------------------------------- #
# self-test -- no ROS, runs on the bench laptop
# --------------------------------------------------------------------------- #

def _yaw_pitch_quat(pitch_deg: float) -> Tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) for a pure pitch, so tilt is exactly pitch."""
    half = math.radians(pitch_deg) / 2.0
    return (0.0, math.sin(half), 0.0, math.cos(half))


def _core(drones: Sequence[str] = ("drone0",),
          policies: Optional[Dict[str, ConditionPolicy]] = None,
          **limit_kwargs) -> GuardCore:
    """A core with no startup grace and a hard-edged box, for threshold work."""
    limit_kwargs.setdefault("startup_grace_s", 0.0)
    box = limit_kwargs.pop("box", Box(margin=0.0))
    limits = Limits(box=box, **limit_kwargs)
    return GuardCore(drones, None, limits,
                     policies if policies is not None else default_policies(),
                     started_at=0.0)


def _only(condition: str, ladder: Tuple[str, ...] = DEFAULT_LADDER,
          confirm_s: float = 0.2, escalate_s: float = 1.0
          ) -> Dict[str, ConditionPolicy]:
    """Policies with exactly one condition live, so a ladder test measures the
    ladder and not whichever other condition happened to trip too."""
    policies = default_policies()
    for name in CONDITIONS:
        policies[name] = replace(policies[name], enabled=(name == condition),
                                 ladder=ladder if name == condition
                                 else policies[name].ladder)
        if name == condition:
            policies[name] = replace(policies[name], confirm_s=confirm_s,
                                     escalate_s=escalate_s)
    return policies


def _fires(core: GuardCore, condition: str, now: float, drone: str = "drone0") -> bool:
    _, evals = core.guards[drone].evaluate(now, core.limits)
    return evals[condition][0]


def self_test() -> int:  # noqa: C901 -- a flat list of cases reads better here
    failures: List[str] = []

    def check(label: str, condition: bool) -> None:
        print(f"  [{'PASS' if condition else 'FAIL'}] {label}")
        if not condition:
            failures.append(label)

    print("threshold comparison")
    check("trips exactly at the limit", at_or_above(0.1, 0.1))
    check("does not trip just inside", not at_or_above(0.0999, 0.1))
    check("trips above", at_or_above(0.2, 0.1))
    check("a NaN measurement is never 'within limits'",
          at_or_above(float("nan"), 0.1))

    print("geofence geometry (8 x 6 x 4 m, no margin)")
    box = Box(size=(8.0, 6.0, 4.0), center=(0.0, 0.0, 0.0), margin=0.0)
    check("half extents are half the box", box.half_extents() == (4.0, 3.0, 2.0))
    check("centre is inside", not box.outside((0.0, 0.0, 0.0))[0])
    check("just inside every face", not box.outside((3.999, 2.999, 1.999))[0])
    check("EXACTLY on the x face trips", box.outside((4.0, 0.0, 0.0))[0])
    check("EXACTLY on the y face trips", box.outside((0.0, 3.0, 0.0))[0])
    check("EXACTLY on the z face trips", box.outside((0.0, 0.0, 2.0))[0])
    check("exactly on the corner trips", box.outside((4.0, 3.0, 2.0))[0])
    check("just inside the corner does not",
          not box.outside((3.9999999, 2.9999999, 1.9999999))[0])
    check("negative faces are symmetric",
          box.outside((-4.0, 0.0, 0.0))[0] and not box.outside((-3.999, 0.0, 0.0))[0])
    check("a NaN position is outside", box.outside((float("nan"), 0.0, 0.0))[0])
    check("the reason names the axis", box.outside((4.0, 0.0, 0.0))[1].startswith("x="))

    fenced = Box(size=(8.0, 6.0, 4.0), margin=0.5)
    check("the margin shrinks the fence", fenced.half_extents() == (3.5, 2.5, 1.5))
    check("EXACTLY on the fence trips", fenced.outside((3.5, 0.0, 0.0))[0])
    check("just inside the fence does not", not fenced.outside((3.4999, 0.0, 0.0))[0])
    floored = Box(size=(8.0, 6.0, 4.0), center=(0.0, 0.0, 2.0), margin=0.0)
    check("a floor-referenced box fences the floor",
          floored.outside((0.0, 0.0, 0.0))[0]
          and not floored.outside((0.0, 0.0, 0.1))[0])
    for bad in ({"size": (8.0, 6.0)}, {"size": (8.0, 0.0, 4.0)},
                {"margin": -1.0}, {"margin": 4.0}):
        try:
            Box(**bad)                                     # type: ignore[arg-type]
            check(f"rejects box {bad}", False)
        except ValueError:
            check(f"rejects box {bad}", True)

    print("tilt from a quaternion")

    def _tilt_is(quaternion: Tuple[float, float, float, float],
                 expected: float) -> bool:
        value = tilt_deg_from_quaternion(*quaternion)
        return value is not None and abs(value - expected) < 1e-9

    check("level is 0 deg", _tilt_is((0.0, 0.0, 0.0, 1.0), 0.0))
    check("yaw alone is not tilt",
          _tilt_is((0.0, 0.0, math.sin(0.7), math.cos(0.7)), 0.0))
    check("30 deg pitch reads 30 deg", _tilt_is(_yaw_pitch_quat(30.0), 30.0))
    check("a non-unit quaternion is normalised, not rejected",
          _tilt_is((0.0, 2 * math.sin(math.radians(15)),
                    0.0, 2 * math.cos(math.radians(15))), 30.0))
    check("a zero quaternion is unknown, not level",
          tilt_deg_from_quaternion(0.0, 0.0, 0.0, 0.0) is None)
    check("a NaN quaternion is unknown",
          tilt_deg_from_quaternion(float("nan"), 0.0, 0.0, 1.0) is None)

    print("/mocap/health parsing")
    health = {"1": {"rigid_body_name": "drone0", "tracked": False},
              "2": {"rigid_body_name": "drone1", "tracked": True}}
    check("matches on rigid_body_name", health_tracked(health, "drone0") is False)
    check("reads the other body independently",
          health_tracked(health, "drone1") is True)
    check("an absent body is unknown, not untracked",
          health_tracked(health, "drone2") is None)
    check("falls back to the tracker key",
          health_tracked({"drone0": {"tracked": True}}, "drone0") is True)
    check("garbage is unknown, never a trip",
          health_tracked("not json at all", "drone0") is None
          and health_tracked({"drone0": 7}, "drone0") is None)

    print("condition: mocap_timeout")
    core = _core(mocap_timeout_s=0.1)
    core.on_mocap_body("drone0", (0.0, 0.0, 0.0), None, 0.0)
    check("fresh does not fire", not _fires(core, C_MOCAP_TIMEOUT, 0.09))
    check("EXACTLY at 100 ms fires", _fires(core, C_MOCAP_TIMEOUT, 0.1))
    check("older fires", _fires(core, C_MOCAP_TIMEOUT, 0.5))
    never = _core(mocap_timeout_s=0.1)
    check("a body that never arrives ages from node start",
          not _fires(never, C_MOCAP_TIMEOUT, 0.0)
          and _fires(never, C_MOCAP_TIMEOUT, 0.1))

    print("condition: mocap_untracked")
    core = _core()
    check("no health yet is unknown, not untracked",
          not _fires(core, C_MOCAP_UNTRACKED, 0.0))
    core.on_health({"1": {"rigid_body_name": "drone0", "tracked": True}}, 0.0)
    check("tracked does not fire", not _fires(core, C_MOCAP_UNTRACKED, 0.0))
    core.on_health({"1": {"rigid_body_name": "drone0", "tracked": False}}, 1.0)
    check("NOT TRACKED fires", _fires(core, C_MOCAP_UNTRACKED, 1.0))
    check("a stale health report stops counting",
          not _fires(core, C_MOCAP_UNTRACKED, 1.0 + core.limits.health_timeout_s + 0.1))

    print("condition: geofence")
    core = _core()
    core.on_mocap_body("drone0", (3.999, 0.0, 0.0), None, 0.0)
    check("just inside does not fire", not _fires(core, C_GEOFENCE, 0.0))
    core.on_mocap_body("drone0", (4.0, 0.0, 0.0), None, 0.01)
    check("EXACTLY on the boundary fires", _fires(core, C_GEOFENCE, 0.01))
    check("the last known position is used even when it is stale",
          _fires(core, C_GEOFENCE, 30.0))

    print("condition: overspeed")
    core = _core(max_speed_mps=2.0, speed_window_s=0.1)
    core.on_mocap_body("drone0", (0.0, 0.0, 0.0), None, 0.0)
    check("one sample gives no speed",
          core.guards["drone0"].measure(0.0, core.limits).speed_mps is None)
    core.on_mocap_body("drone0", (0.25, 0.0, 0.0), None, 0.125)
    check("speed is differenced from raw mocap",
          abs((core.guards["drone0"].measure(0.125, core.limits).speed_mps or 0)
              - 2.0) < 1e-12)
    check("EXACTLY at the limit fires", _fires(core, C_OVERSPEED, 0.125))
    slow = _core(max_speed_mps=2.0, speed_window_s=0.1)
    slow.on_mocap_body("drone0", (0.0, 0.0, 0.0), None, 0.0)
    slow.on_mocap_body("drone0", (0.24, 0.0, 0.0), None, 0.125)
    check("just under the limit does not fire", not _fires(slow, C_OVERSPEED, 0.125))
    gapped = _core(max_speed_mps=2.0, speed_window_s=0.1)
    gapped.on_mocap_body("drone0", (0.0, 0.0, 0.0), None, 0.0)
    gapped.on_mocap_body("drone0", (5.0, 0.0, 0.0), None, 5.0)
    check("a speed spanning a dropout is unknown, not a phantom trip",
          gapped.guards["drone0"].measure(5.0, gapped.limits).speed_mps is None
          and not _fires(gapped, C_OVERSPEED, 5.0))

    print("condition: tilt")
    core = _core(max_tilt_deg=35.0)
    core.on_mocap_body("drone0", (0.0, 0.0, 0.0), _yaw_pitch_quat(34.9), 0.0)
    check("just inside does not fire", not _fires(core, C_TILT, 0.0))
    core.on_mocap_body("drone0", (0.0, 0.0, 0.0), _yaw_pitch_quat(35.0), 0.0)
    check("EXACTLY at the limit fires", _fires(core, C_TILT, 0.0))
    core.on_mocap_body("drone0", (0.0, 0.0, 0.0), _yaw_pitch_quat(60.0), 0.0)
    check("beyond the limit fires", _fires(core, C_TILT, 0.0))

    print("condition: pose_mismatch")
    core = _core(max_pose_delta_m=0.3)
    core.on_mocap_body("drone0", (0.0, 0.0, 0.0), None, 0.0)
    check("no estimator pose yet is not a mismatch",
          not _fires(core, C_POSE_MISMATCH, 0.0))
    core.on_pose("drone0", (0.29, 0.0, 0.0), 0.0)
    check("just inside does not fire", not _fires(core, C_POSE_MISMATCH, 0.0))
    core.on_pose("drone0", (0.3, 0.0, 0.0), 0.0)
    check("EXACTLY at the limit fires", _fires(core, C_POSE_MISMATCH, 0.0))
    check("a stale source is reported as a dropout, not a disagreement",
          not _fires(core, C_POSE_MISMATCH, 0.2))

    print("condition: pose_timeout")
    core = _core(pose_timeout_s=0.5)
    core.on_pose("drone0", (0.0, 0.0, 0.0), 0.0)
    check("fresh does not fire", not _fires(core, C_POSE_TIMEOUT, 0.49))
    check("EXACTLY at the timeout fires", _fires(core, C_POSE_TIMEOUT, 0.5))

    print("dwell: one noisy sample cannot trip anything")
    core = _core(policies=_only(C_GEOFENCE, confirm_s=0.2, escalate_s=1.0))
    core.on_mocap_body("drone0", (0.0, 0.0, 0.0), None, 0.0)
    core.update(0.0)
    core.on_mocap_body("drone0", (9.0, 0.0, 0.0), None, 0.01)   # one bad sample
    actions = core.update(0.01)
    check("the bad sample commands nothing", actions == [])
    core.on_mocap_body("drone0", (0.0, 0.0, 0.0), None, 0.02)   # and it is gone
    check("state is still OK", core.guards["drone0"].state == OK)
    for k in range(1, 60):                                      # 0.6 s of clean data
        t = 0.02 + k * 0.01
        core.on_mocap_body("drone0", (0.0, 0.0, 0.0), None, t)
        core.update(t)
    check("still OK 0.6 s later, well past the confirm dwell",
          core.guards["drone0"].state == OK)
    check("the flicker was counted even though it did not trip",
          core.guards["drone0"].conditions[C_GEOFENCE].trips == 1)

    print("dwell: a flicker never accumulates towards a stage")
    core = _core(policies=_only(C_GEOFENCE, confirm_s=0.2, escalate_s=1.0))
    for k in range(40):
        t = k * 0.05
        # alternate outside/inside every sample: continuously true, never.
        x = 9.0 if k % 2 == 0 else 0.0
        core.on_mocap_body("drone0", (x, 0.0, 0.0), None, t)
        core.update(t)
    check("2 s of alternating samples never reaches WARN",
          core.guards["drone0"].state == OK)

    print("escalation ladder advances in order, one stage per dwell")
    core = _core(policies=_only(C_GEOFENCE, confirm_s=0.2, escalate_s=1.0))
    core.on_mocap_body("drone0", (9.0, 0.0, 0.0), None, 0.0)    # outside, and stays
    seen: List[Tuple[float, str]] = []
    t = 0.0
    while t <= 3.0 + 1e-9:
        for action in core.update(t):
            seen.append((round(t, 3), action.stage))
        t = round(t + 0.02, 3)
    stages = [stage for _, stage in seen if stage != DISARM or True]
    check("first stage is WARN", stages[0] == WARN)
    check("the order is WARN, LAND, DISARM",
          [s for s in dict.fromkeys(stages)] == [WARN, LAND, DISARM])
    times = {stage: time for time, stage in reversed(seen)}
    check("WARN at the confirm dwell", abs(times[WARN] - 0.2) < 0.021)
    check("LAND one escalate dwell later", abs(times[LAND] - 1.2) < 0.021)
    check("DISARM one more escalate dwell later", abs(times[DISARM] - 2.2) < 0.021)
    check("no stage is skipped",
          stages.count(WARN) == 1 and stages.count(LAND) == 1)
    check("the disarm is re-sent, because /fmu/in is best-effort",
          stages.count(DISARM) == core.disarm_repeat)
    check("and re-sent a bounded number of times",
          core.guards["drone0"].disarm_sent == core.disarm_repeat)

    print("a ladder may skip stages when configured to")
    core = _core(policies=_only(C_GEOFENCE, ladder=(LAND, DISARM),
                                confirm_s=0.1, escalate_s=0.5))
    core.on_mocap_body("drone0", (9.0, 0.0, 0.0), None, 0.0)
    check("the confirm dwell still applies to a skipped ladder",
          core.update(0.0) == [])
    first = [a.stage for a in core.update(0.1)]
    check("configured first stage is LAND, not WARN", first == [LAND])
    check("and it reaches DISARM after the escalate dwell",
          [a.stage for a in core.update(0.6)] == [DISARM])
    try:
        ConditionPolicy(C_GEOFENCE, (LAND, WARN))
        check("refuses a ladder that de-escalates", False)
    except ValueError:
        check("refuses a ladder that de-escalates", True)
    try:
        ConditionPolicy(C_GEOFENCE, (WARN, WARN))
        check("refuses a ladder that repeats a stage", False)
    except ValueError:
        check("refuses a ladder that repeats a stage", True)

    print("per-drone independence")
    core = _core(("drone0", "drone1", "drone2"),
                 policies=_only(C_GEOFENCE, confirm_s=0.1, escalate_s=0.5))
    acted: List[Action] = []
    t = 0.0
    while t <= 2.0 + 1e-9:
        core.on_mocap_body("drone0", (9.0, 0.0, 0.0), None, t)      # outside
        core.on_mocap_body("drone1", (0.0, 0.0, 0.0), None, t)      # inside
        core.on_mocap_body("drone2", (-1.0, 1.0, 0.5), None, t)     # inside
        acted.extend(core.update(t))
        t = round(t + 0.02, 3)
    check("the offending drone reaches DISARM",
          core.guards["drone0"].state == DISARM)
    check("the others are untouched",
          core.guards["drone1"].state == OK and core.guards["drone2"].state == OK)
    check("no action was ever raised against another drone",
          all(a.drone == "drone0" for a in acted))
    check("their condition trackers never even armed",
          core.guards["drone1"].conditions[C_GEOFENCE].trips == 0)

    print("latching: a trip does not self-clear mid-incident")
    core = _core(policies=_only(C_GEOFENCE, confirm_s=0.1, escalate_s=1.0))
    core.on_mocap_body("drone0", (9.0, 0.0, 0.0), None, 0.0)
    core.update(0.0)
    core.update(0.1)
    check("WARN is reached", core.guards["drone0"].state == WARN)
    core.on_mocap_body("drone0", (0.0, 0.0, 0.0), None, 0.2)     # back inside
    for k in range(1, 200):                                     # 2 s of good data
        t = 0.2 + k * 0.01
        core.on_mocap_body("drone0", (0.0, 0.0, 0.0), None, t)
        core.update(t)
    check("the state stays latched at WARN", core.guards["drone0"].state == WARN)
    check("and does NOT escalate once the condition has gone",
          core.guards["drone0"].conditions[C_GEOFENCE].stage == WARN)
    check("status reports it as latched",
          core.status(2.2)["drones"]["drone0"]["latched"] is True)

    print("reset")
    cleared = core.reset("drone0", 2.5)
    check("reset clears the named drone", cleared == ["drone0"]
          and core.guards["drone0"].state == OK)
    core.on_mocap_body("drone0", (0.0, 0.0, 0.0), None, 2.5)
    core.update(2.5)
    check("and it stays clear while the condition is gone",
          core.guards["drone0"].state == OK)
    core.on_mocap_body("drone0", (9.0, 0.0, 0.0), None, 3.0)
    core.update(3.0)
    check("a fresh trip must serve the confirm dwell again",
          core.guards["drone0"].state == OK)
    core.update(3.1)
    check("then it escalates from the bottom of the ladder again",
          core.guards["drone0"].state == WARN)

    print("reset does not un-happen a live condition")
    core.reset(None, 3.2)
    check("reset all clears every drone",
          all(g.state == OK for g in core.guards.values()))
    core.update(3.2)
    check("still OK on the reset tick", core.guards["drone0"].state == OK)
    core.on_mocap_body("drone0", (9.0, 0.0, 0.0), None, 3.3)
    core.update(3.3)
    core.update(3.4)
    check("and it re-trips because the condition is still true",
          core.guards["drone0"].state == WARN)
    try:
        core.reset("drone9", 3.5)
        check("rejects a reset for an unknown drone", False)
    except ValueError:
        check("rejects a reset for an unknown drone", True)

    print("startup grace")
    limits = Limits(box=Box(margin=0.0), startup_grace_s=1.0)
    core = GuardCore(["drone0"], None, limits,
                     _only(C_MOCAP_TIMEOUT, confirm_s=0.0, escalate_s=0.5),
                     started_at=0.0)
    core.update(0.5)
    check("nothing trips during the grace window",
          core.guards["drone0"].state == OK and core.in_grace(0.5))
    core.update(1.0)
    check("and it arms the instant the grace expires",
          not core.in_grace(1.0) and core.guards["drone0"].state == WARN)

    print("the DISARM message")
    cmd = build_disarm_command(target_system=2, timestamp_us=1234)
    check("command is VEHICLE_CMD_COMPONENT_ARM_DISARM (400)", cmd.command == 400)
    check("param1 is ARMING_ACTION_DISARM (0.0)", cmd.param1 == 0.0)
    check("param2 is the PX4 force magic (21196.0)", cmd.param2 == 21196.0)
    check("param2 is a float, as VehicleCommand declares it",
          isinstance(cmd.param2, float))
    check("target_system is the drone's own id", cmd.target_system == 2)
    check("target_component matches the platform (1)", cmd.target_component == 1)
    check("source_system matches the platform (1)", cmd.source_system == 1)
    check("source_component matches the platform (1)", cmd.source_component == 1)
    check("from_external is true", cmd.from_external is True)
    check("timestamp is carried in microseconds", cmd.timestamp == 1234)
    check("the default target_system is 1", build_disarm_command().target_system == 1)

    class _StubVehicleCommand:
        def __init__(self) -> None:
            for name in VEHICLE_COMMAND_FIELDS:
                setattr(self, name, None)

    stub = fill_vehicle_command(_StubVehicleCommand(), cmd)
    check("every field is copied onto the message",
          all(getattr(stub, n) == getattr(cmd, n) for n in VEHICLE_COMMAND_FIELDS))

    class _OldVehicleCommand:
        def __init__(self) -> None:
            for name in VEHICLE_COMMAND_FIELDS:
                if name != "param2":
                    setattr(self, name, None)

    try:
        fill_vehicle_command(_OldVehicleCommand(), cmd)
        check("refuses a VehicleCommand missing param2", False)
    except AttributeError:
        # Silently dropping param2 turns a force-disarm into an ordinary one,
        # which PX4 refuses in flight. That must be loud, not best-effort.
        check("refuses a VehicleCommand missing param2", True)

    print("disarm routing cannot be ambiguous")
    check("one drone on a bare prefix is fine",
          check_disarm_routing(["drone0"], "", {"drone0": 1}) is None)
    check("per-drone FMU namespaces are unambiguous",
          check_disarm_routing(["drone0", "drone1"], "/{ns}",
                               {"drone0": 1, "drone1": 1}) is None)
    check("a shared topic with distinct system ids is allowed",
          check_disarm_routing(["drone0", "drone1"], "",
                               {"drone0": 1, "drone1": 2}) is None)
    check("a shared topic with colliding ids is REFUSED",
          check_disarm_routing(["drone0", "drone1"], "",
                               {"drone0": 1, "drone1": 1}) is not None)
    check("the FMU topic resolves like preflight_check does",
          resolve_fmu_topic("", "drone0") == "/fmu/in/vehicle_command"
          and resolve_fmu_topic("/{ns}", "drone1")
          == "/drone1/fmu/in/vehicle_command"
          and resolve_fmu_topic("/uav", "drone0") == "/uav/fmu/in/vehicle_command")
    check("no prefix map resolves exactly as it did before",
          resolve_fmu_topic("/{ns}", "drone1", None)
          == resolve_fmu_topic("/{ns}", "drone1")
          and resolve_fmu_topic("/{ns}", "drone1", {})
          == "/drone1/fmu/in/vehicle_command")
    check("a mapped drone takes its prefix VERBATIM",
          resolve_fmu_topic("", "drone0", {"drone0": "/uav_0"})
          == "/uav_0/fmu/in/vehicle_command")
    check("the map beats --fmu-prefix, template included",
          resolve_fmu_topic("/{ns}", "drone0", {"drone0": "/uav_0"})
          == "/uav_0/fmu/in/vehicle_command")
    check("an unmapped drone falls back to --fmu-prefix",
          resolve_fmu_topic("/{ns}", "drone1", {"drone0": "/uav_0"})
          == "/drone1/fmu/in/vehicle_command"
          and resolve_fmu_topic("", "drone1", {"drone0": "/uav_0"})
          == "/fmu/in/vehicle_command")

    px4_ns = {"drone0": "/uav_0", "drone1": "/uav_1", "drone2": "/uav_2"}
    check("a prefix map giving every drone its own topic is unambiguous",
          check_disarm_routing(["drone0", "drone1", "drone2"], "",
                               {"drone0": 1, "drone1": 1, "drone2": 1},
                               px4_ns) is None)
    check("a partial map still separates the mapped drone from the rest",
          check_disarm_routing(["drone0", "drone1"], "",
                               {"drone0": 1, "drone1": 1},
                               {"drone0": "/uav_0"}) is None)
    check("a map aiming two drones at ONE topic with colliding ids is REFUSED",
          check_disarm_routing(["drone0", "drone1"], "",
                               {"drone0": 1, "drone1": 1},
                               {"drone0": "/uav_0", "drone1": "/uav_0"})
          is not None)
    check("the same shared topic with distinct ids is still allowed",
          check_disarm_routing(["drone0", "drone1"], "",
                               {"drone0": 1, "drone1": 2},
                               {"drone0": "/uav_0", "drone1": "/uav_0"}) is None)
    check("an empty map changes nothing, so the bare shared link is REFUSED",
          check_disarm_routing(["drone0", "drone1"], "",
                               {"drone0": 1, "drone1": 1}, {}) is not None)
    check("the refusal names the topic that is actually shared",
          "/uav_0/fmu/in/vehicle_command" in (check_disarm_routing(
              ["drone0", "drone1"], "", {"drone0": 1, "drone1": 1},
              {"drone0": "/uav_0", "drone1": "/uav_0"}) or ""))

    print("configuration parsing")
    check("drones split on commas",
          parse_drones("drone0, drone1") == ["drone0", "drone1"])
    check("drones accept a list", parse_drones(["a", "b"]) == ["a", "b"])
    for bad in ("", ",,", ["a", "a"]):
        try:
            parse_drones(bad)
            check(f"rejects drones {bad!r}", False)
        except ValueError:
            check(f"rejects drones {bad!r}", True)
    check("rigid bodies default to the namespace",
          parse_rigid_bodies(None, ["drone0"]) == {"drone0": "drone0"})
    check("rigid bodies map numeric Motive ids",
          parse_rigid_bodies("drone0:1,drone1:2", ["drone0", "drone1"])
          == {"drone0": "1", "drone1": "2"})
    check("rigid bodies accept positional names",
          parse_rigid_bodies("a,b", ["drone0", "drone1"])
          == {"drone0": "a", "drone1": "b"})
    for bad in ("drone0:1", "drone9:1,drone0:2", "a,b,c", "drone0:1,b",
                "drone0:1,drone1:1"):
        try:
            parse_rigid_bodies(bad, ["drone0", "drone1"])
            check(f"rejects --rigid-bodies '{bad}'", False)
        except ValueError:
            check(f"rejects --rigid-bodies '{bad}'", True)
    check("target systems default to 1",
          parse_target_systems(None, ["drone0", "drone1"])
          == {"drone0": 1, "drone1": 1})
    check("target systems map per drone",
          parse_target_systems("drone0:1,drone1:2", ["drone0", "drone1"])
          == {"drone0": 1, "drone1": 2})
    check("target systems accept positional ids",
          parse_target_systems("3,4", ["drone0", "drone1"])
          == {"drone0": 3, "drone1": 4})
    for bad in ("drone9:1", "drone0:x", "drone0:0", "drone0:256", "1,2,3", "1,drone1:2"):
        try:
            parse_target_systems(bad, ["drone0", "drone1"])
            check(f"rejects --target-systems '{bad}'", False)
        except ValueError:
            check(f"rejects --target-systems '{bad}'", True)
    check("no FMU prefix map means no override at all, not a default",
          parse_fmu_prefix_map(None, ["drone0"]) == {}
          and parse_fmu_prefix_map("   ", ["drone0"]) == {})
    check("the FMU prefix map reads 'ns=prefix' pairs",
          parse_fmu_prefix_map("drone0=/uav_0, drone1=/uav_1",
                               ["drone0", "drone1", "drone2"])
          == {"drone0": "/uav_0", "drone1": "/uav_1"})
    check("a drone with no entry is simply absent, and falls back",
          "drone2" not in parse_fmu_prefix_map(
              "drone0=/uav_0", ["drone0", "drone1", "drone2"]))
    for bad in ("drone0", "drone0:/uav_0", "drone9=/uav_9", "drone0=",
                "=/uav_0", "drone0=/uav_0,drone0=/uav_1"):
        try:
            parse_fmu_prefix_map(bad, ["drone0", "drone1"])
            check(f"rejects --fmu-prefix-map '{bad}'", False)
        except ValueError:
            check(f"rejects --fmu-prefix-map '{bad}'", True)

    print("policy specification")
    policies = apply_policy_specs(["geofence:land+disarm:0.1:0.5"])
    check("ladder, confirm and escalate all applied",
          policies[C_GEOFENCE].ladder == (LAND, DISARM)
          and policies[C_GEOFENCE].confirm_s == 0.1
          and policies[C_GEOFENCE].escalate_s == 0.5)
    check("other conditions are untouched",
          policies[C_TILT] == default_policies()[C_TILT])
    check("an empty field keeps the default",
          apply_policy_specs(["tilt::0.5"])[C_TILT].ladder
          == default_policies()[C_TILT].ladder
          and apply_policy_specs(["tilt::0.5"])[C_TILT].confirm_s == 0.5)
    check("'none' disables a condition",
          apply_policy_specs(["overspeed:none"])[C_OVERSPEED].enabled is False)
    check("'*' applies to every condition",
          all(p.confirm_s == 2.0
              for p in apply_policy_specs(["*::2.0"]).values()))
    check("specs apply in order",
          apply_policy_specs(["*::2.0", "tilt::0.25"])[C_TILT].confirm_s == 0.25)
    for bad in ("", "wobble:warn", "geofence:sideways", "geofence:disarm+warn",
                "geofence::abc", "geofence:::-1", "geofence", "a:b:c:d:e"):
        try:
            apply_policy_specs([bad])
            check(f"rejects --policy '{bad}'", False)
        except ValueError:
            check(f"rejects --policy '{bad}'", True)

    print("a disabled condition is inert")
    core = _core(policies=apply_policy_specs(["*:none"]))
    core.on_mocap_body("drone0", (99.0, 0.0, 0.0), None, 0.0)
    for k in range(200):
        core.update(k * 0.05)
    check("nothing trips when every condition is disabled",
          core.guards["drone0"].state == OK)

    print("status payload")
    core = _core(("drone0", "drone1"),
                 policies=_only(C_GEOFENCE, confirm_s=0.1, escalate_s=0.5))
    core.on_mocap_body("drone0", (9.0, 0.0, 0.0), None, 0.0)
    core.on_mocap_body("drone1", (0.0, 0.0, 0.0), None, 0.0)
    core.on_pose("drone0", (9.0, 0.0, 0.0), 0.0)
    core.on_health({"a": {"rigid_body_name": "drone0", "tracked": True}}, 0.0)
    core.update(0.0)
    core.update(0.2)
    payload = core.status(0.2)
    check("status is JSON serialisable", isinstance(json.dumps(payload), str))
    check("status carries every drone", set(payload["drones"]) == {"drone0", "drone1"})
    d0 = payload["drones"]["drone0"]
    check("status carries the state", d0["state"] == WARN)
    check("status carries time in state", d0["time_in_state_s"] >= 0.0)
    check("status names the tripped condition",
          d0["driving_conditions"] == [C_GEOFENCE]
          and C_GEOFENCE in d0["tripped_now"])
    check("status carries every condition", set(d0["conditions"]) == set(CONDITIONS))
    check("status carries time in stage",
          d0["conditions"][C_GEOFENCE]["time_in_stage_s"] is not None)
    check("status carries the measurements",
          d0["measurements"]["position"] == [9.0, 0.0, 0.0]
          and d0["measurements"]["tracked"] is True)
    check("status carries the limits it is enforcing",
          payload["limits"]["max_speed_mps"] == DEFAULT_MAX_SPEED_MPS)
    check("the untripped drone reports OK",
          payload["drones"]["drone1"]["state"] == OK)

    print("input routing")
    core = _core(("drone0",))
    check("an unmapped rigid body is ignored",
          core.on_mocap_body("someone_elses_body", (0.0, 0.0, 0.0), None, 0.0)
          is False)
    check("a pose for an unknown drone is ignored",
          core.on_pose("drone9", (0.0, 0.0, 0.0), 0.0) is False)
    mapped = GuardCore(["drone0"], {"drone0": "1"}, Limits(box=Box(margin=0.0),
                                                          startup_grace_s=0.0))
    check("a mapped body name reaches the right drone",
          mapped.on_mocap_body("1", (1.0, 2.0, 3.0), None, 0.0) is True
          and mapped.guards["drone0"].measure(0.0, mapped.limits).position
          == (1.0, 2.0, 3.0))

    print()
    if failures:
        print(f"{len(failures)} FAILURE(S): " + "; ".join(failures))
        return 1
    print("all self-tests passed")
    return 0


# --------------------------------------------------------------------------- #
# entry point
# --------------------------------------------------------------------------- #

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="volume_guard.py",
        description="S2: independent volume watchdog for O-134. Actions are "
                    "DRY RUN unless --arm is given. Unrecognised arguments, "
                    "including --ros-args, are passed through to rclpy.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Ladder syntax: --policy NAME:LADDER[:CONFIRM[:ESCALATE]]. "
               "See --list-conditions.")
    ap.add_argument("--drones", default=None,
                    help="comma-separated namespaces (default: %s)"
                         % ",".join(DEFAULT_DRONES))
    ap.add_argument("--rigid-bodies", default=None,
                    help="'ns:body,...' or positional 'A,B,C'; default: the "
                         "rigid body is named after the namespace")
    ap.add_argument("--arm", action="store_true",
                    help="ACTUALLY send land and force-disarm commands. Without "
                         "this the node logs what it would have sent and sends "
                         "nothing.")
    ap.add_argument("--dry-run", action="store_true",
                    help="the default; present so a bench command can say so "
                         "explicitly. Ignored if --arm is also given.")
    ap.add_argument("--allow-ambiguous-disarm", action="store_true",
                    help="permit several drones to share one "
                         "/fmu/in/vehicle_command with colliding target_system "
                         "ids. A disarm would then reach every aircraft on that "
                         "link. Refused by default.")

    fence = ap.add_argument_group("volume")
    fence.add_argument("--box", nargs=3, type=float, default=None,
                       metavar=("X", "Y", "Z"),
                       help="netted volume in metres (default %s)"
                            % " ".join(f"{v:g}" for v in DEFAULT_BOX_M))
    fence.add_argument("--box-center", nargs=3, type=float, default=None,
                       metavar=("X", "Y", "Z"),
                       help="volume centre in metres (default 0 0 0; use "
                            "0 0 2 for a floor-referenced room frame)")
    fence.add_argument("--margin", type=float, default=None, metavar="M",
                       help="how far the fence sits inside the net "
                            f"(default {DEFAULT_FENCE_MARGIN_M:g})")

    limits = ap.add_argument_group("limits")
    limits.add_argument("--mocap-timeout", type=float, default=None, metavar="S",
                        help=f"default {DEFAULT_MOCAP_TIMEOUT_S:g}")
    limits.add_argument("--pose-timeout", type=float, default=None, metavar="S",
                        help=f"default {DEFAULT_POSE_TIMEOUT_S:g}")
    limits.add_argument("--max-speed", type=float, default=None, metavar="MPS",
                        help=f"default {DEFAULT_MAX_SPEED_MPS:g}")
    limits.add_argument("--max-tilt", type=float, default=None, metavar="DEG",
                        help=f"default {DEFAULT_MAX_TILT_DEG:g}")
    limits.add_argument("--max-pose-delta", type=float, default=None, metavar="M",
                        help=f"default {DEFAULT_MAX_POSE_DELTA_M:g}")
    limits.add_argument("--speed-window", type=float, default=None, metavar="S",
                        help="window the mocap speed is differenced over "
                             f"(default {DEFAULT_SPEED_WINDOW_S:g})")
    limits.add_argument("--startup-grace", type=float, default=None, metavar="S",
                        help="no condition may trip for this long after start "
                             f"(default {DEFAULT_STARTUP_GRACE_S:g})")
    limits.add_argument("--policy", action="append", metavar="SPEC", default=None,
                        help="NAME:LADDER[:CONFIRM[:ESCALATE]]; repeatable. "
                             "See --list-conditions.")

    plumbing = ap.add_argument_group("plumbing")
    plumbing.add_argument("--fmu-prefix", default=None,
                          help="prefix for /fmu/ topics; include '{ns}' for a "
                               "per-drone namespace (default: bare /fmu/...)")
    plumbing.add_argument("--fmu-prefix-map", default=None,
                          metavar="NS=PREFIX,...",
                          help="explicit per-drone FMU prefix, e.g. "
                               "'drone0=/uav_0,drone1=/uav_1'. Each prefix is "
                               "used VERBATIM and beats --fmu-prefix; a drone "
                               "with no entry falls back to it. This is the "
                               "PX4 UXRCE_DDS_NS_IDX case, where the FMU "
                               "topics are /uav_0.. and no template relates "
                               "them to the Aerostack2 namespaces.")
    plumbing.add_argument("--target-systems", default=None,
                          help="'ns:id,...' or positional ids; PX4 "
                               f"target_system per drone (default all "
                               f"{DEFAULT_TARGET_SYSTEM})")
    plumbing.add_argument("--rate", type=float, default=None, metavar="HZ",
                          help=f"evaluation rate (default {DEFAULT_RATE_HZ:g})")
    plumbing.add_argument("--status-rate", type=float, default=None, metavar="HZ",
                          help=f"status rate (default {DEFAULT_STATUS_RATE_HZ:g})")
    plumbing.add_argument("--land-speed", type=float, default=None, metavar="MPS",
                          help=f"land behavior speed (default {DEFAULT_LAND_SPEED_MPS:g})")
    plumbing.add_argument("--disarm-repeat", type=int, default=None, metavar="N",
                          help="how many times a disarm is re-sent, because "
                               "/fmu/in is best-effort (default "
                               f"{DEFAULT_DISARM_REPEAT})")

    ap.add_argument("--list-conditions", action="store_true",
                    help="print the trip conditions and the ladder, then exit")
    ap.add_argument("--self-test", action="store_true",
                    help="run the offline test suite and exit (no ROS)")
    return ap


def main() -> int:
    cli, rest = build_parser().parse_known_args()
    if cli.list_conditions:
        print_conditions()
        return 0
    if cli.self_test:
        return self_test()

    # Validate everything that can be validated before rclpy is touched. A
    # safety node that half-starts on a typo, on a machine that may not even
    # have ROS sourced, is worse than one that refuses in a single line.
    for spec in (cli.policy or []):
        try:
            apply_policy_specs([spec])
        except ValueError as exc:
            print(f"volume_guard: --policy {spec}: {exc}", file=sys.stderr)
            print("volume_guard: see --list-conditions", file=sys.stderr)
            return 2
    try:
        drones = parse_drones(cli.drones or ",".join(DEFAULT_DRONES))
        parse_rigid_bodies(cli.rigid_bodies, drones)
        targets = parse_target_systems(cli.target_systems, drones)
        prefix_map = parse_fmu_prefix_map(cli.fmu_prefix_map, drones)
        if cli.box is not None or cli.box_center is not None or cli.margin is not None:
            Box(size=tuple(cli.box or DEFAULT_BOX_M),
                center=tuple(cli.box_center or DEFAULT_BOX_CENTER_M),
                margin=DEFAULT_FENCE_MARGIN_M if cli.margin is None else cli.margin)
        if cli.arm:
            problem = check_disarm_routing(drones, cli.fmu_prefix or "",
                                           targets, prefix_map)
            if problem and not cli.allow_ambiguous_disarm:
                raise ValueError(problem)
    except ValueError as exc:
        print(f"volume_guard: {exc}", file=sys.stderr)
        return 2
    return main_ros(cli, rest or None)


if __name__ == "__main__":
    sys.exit(main())
