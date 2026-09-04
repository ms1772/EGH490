#!/usr/bin/env python3
"""B3: synthetic mocap source with injectable faults, for the bench rigs.

    [this node] --PoseStamped--> vrpn_to_rigidbodies --RigidBodies--> as2_state_estimator
    [this node] --RigidBodies--------------------------------------------^  (--mode rigidbodies)

O-134 bookings are scarce and every session needs a crew, so the whole stack --
including its failure modes -- has to be driven on a desk before anyone flies.
This node stands in for OptiTrack plus the VRPN client: it synthesises poses for
N rigid bodies inside the netted volume and, on command, fails in each of the
ways the real link is known to fail.

WHY THE FAULTS ARE THE POINT
----------------------------
The nominal path is easy and proves almost nothing. Each fault below has a
specific downstream consequence that must be observed at least once off-site:

  * `name_mismatch` is the check for the Aerostack2 mocap_pose bug. When the
    configured rigid_body_name is absent from the array, the stock plugin still
    calls process_mocap_pose() and publishes position (0, 0, 0) with identity
    orientation, at full rate, with no warning. A drone that believes it is at
    the origin flies into the net. This node produces that stimulus on demand.
  * `freeze` is the most dangerous entry in the list, and the reason
    `vrpn_to_rigidbodies` never republishes unchanged data: a frozen stream
    looks perfectly healthy on `ros2 topic hz`. Rate is not liveness.
  * `dropout`, `latency` and `rate_collapse` are what the Wi-Fi, the switch and
    the Jetsons actually do once three airframes stream at once.
  * `occlusion` is a marker set passing behind a prop guard or an operator --
    one aircraft lost while the others keep flying, which is the multi-UAV case
    single-drone testing never reaches.
  * `bad_quaternion` is the malformed-sample path the bridge is supposed to
    reject rather than forward.

Trajectories are closed-form and deterministic -- no RNG anywhere, including the
hover jitter. A bound that holds only in probability is not a bound, and the
self-test asserts that every body stays inside the volume for every trajectory.

Modes
-----
  --mode vrpn          (default) geometry_msgs/PoseStamped on
                       /vrpn_mocap/<tracker>/pose. Exercises the real bridge.
  --mode rigidbodies   mocap4r2_msgs/RigidBodies on /mocap/rigid_bodies.
                       Bypasses the bridge and feeds the estimator directly.

The mode also decides which label matters: in vrpn mode a body's label IS its
topic, so `name_mismatch` starves the bridge's subscription exactly as a Motive
rename would; in rigidbodies mode the label is the rigid_body_name the estimator
matches on, so the same fault reaches the plugin bug directly.

Scheduling and triggering faults
--------------------------------
Scheduled from the command line, timed from node start:

    --fault dropout@10.0+2.0            # starts at t=10 s, lasts 2 s
    --fault NAME[@START][+DURATION][:ARG]

Triggered at runtime, timed from the moment of injection, on a String topic:

    ros2 topic pub --once /fake_mocap/inject std_msgs/msg/String "{data: 'freeze+5'}"
    ros2 topic pub --once /fake_mocap/inject std_msgs/msg/String "{data: 'clear all'}"
    ros2 service call /fake_mocap/clear_faults std_srvs/srv/Trigger    # panic button

A topic rather than a custom service: injection needs one string, it must work
from a bare `ros2 topic pub` on a machine with no extra interface packages
built, and the command grammar is then identical to the CLI. Active faults are
published as JSON on /fake_mocap/status at 1 Hz so a bench log carries the fault
timeline next to whatever the stack did about it.

Usage:
    ros2 run ... fake_mocap.py --trajectory circle --fault freeze@30+5 --ros-args \\
        -p trackers:="['drone0','drone1','drone2']"

    # Straight at the estimator, no bridge in the loop:
    python3 fake_mocap.py --mode rigidbodies --fault name_mismatch@20+10

    python3 fake_mocap.py --list-faults
    python3 fake_mocap.py --self-test       # no ROS required
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Sequence, Tuple

DEFAULT_VRPN_TOPIC_TEMPLATE = "/vrpn_mocap/{tracker}/pose"
DEFAULT_RIGIDBODIES_TOPIC = "/mocap/rigid_bodies"
DEFAULT_INJECT_TOPIC = "/fake_mocap/inject"
DEFAULT_STATUS_TOPIC = "/fake_mocap/status"
DEFAULT_CLEAR_SERVICE = "/fake_mocap/clear_faults"

DEFAULT_TRACKERS = ("drone0", "drone1", "drone2")
DEFAULT_PUBLISH_RATE_HZ = 100.0
DEFAULT_FRAME_ID = "map"
DEFAULT_MODE = "vrpn"
DEFAULT_TRAJECTORY = "circle"

# O-134: ~8 m x 6 m horizontal, ~4 m usable altitude, fully netted. Centred on
# the origin as specified; set box_center_m to (0, 0, 2) for a floor-referenced
# room frame where z is altitude above the floor.
DEFAULT_BOX_M = (8.0, 6.0, 4.0)
DEFAULT_BOX_CENTER_M = (0.0, 0.0, 0.0)
# Keep synthetic motion off the net by this much. The geofence lives further in
# still; this margin only guarantees the *source* never commands the edge.
DEFAULT_MARGIN_M = 0.5
DEFAULT_PERIOD_S = 20.0
# 2 mm matches the static 1-sigma the I-02 card expects from the real rig.
DEFAULT_JITTER_M = 0.002

DEFAULT_LATENCY_S = 0.25
DEFAULT_JUMP_M = (2.0, 0.0, 1.0)
DEFAULT_COLLAPSE_HZ = 5.0
DEFAULT_MISMATCH_SUFFIX = "_typo"
DEFAULT_QUAT_MODE = "nonunit"

MODES = ("vrpn", "rigidbodies")
TRAJECTORIES = ("static", "circle", "lissajous", "hover_jitter")
QUAT_MODES = ("nonunit", "nan", "zero")

# Tick-scheduling comparisons are made on accumulated floats; this slack stops
# a 1-ulp shortfall from silently dropping a frame under rate_collapse.
RATE_EPS_S = 1e-6


# --------------------------------------------------------------------------- #
# fault catalogue
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class FaultDoc:
    """One row of --list-faults: the syntax, and what to watch downstream."""
    name: str
    arg_syntax: str
    default: str
    why: str


FAULT_DOCS: Tuple[FaultDoc, ...] = (
    FaultDoc(
        "dropout", "(none)", "-",
        "Stops publishing entirely. Expect PX4 to declare an EV timeout and\n"
        "     volume_guard to alarm; nothing at all should reach the estimator."),
    FaultDoc(
        "latency", "SECONDS", f"{DEFAULT_LATENCY_S}",
        "Steps the published timestamps back by SECONDS, and delays the pose\n"
        "     with them, so the stream stays self-consistent but arrives late.\n"
        "     This is the EKF2_EV_DELAY mismatch case."),
    FaultDoc(
        "jump", "[BODY=]DX,DY,DZ", ",".join(str(v) for v in DEFAULT_JUMP_M),
        "Teleports the pose by a fixed offset. A jump is deliberately allowed\n"
        "     to leave the box -- that is the whole stimulus. Watch the position\n"
        "     controller's reaction, not the source."),
    FaultDoc(
        "occlusion", "BODY", "first configured body",
        "Removes ONE body from the output while the others continue. The\n"
        "     bridge must drop that body only, and report it untracked."),
    FaultDoc(
        "name_mismatch", "[BODY=]WRONG_NAME", f"first body + '{DEFAULT_MISMATCH_SUFFIX}'",
        "Renames a body to a string nothing is configured for. THE Aerostack2\n"
        "     mocap_pose check: the array stays full-length and full-rate, so\n"
        "     the stock plugin publishes (0,0,0) identity with no warning."),
    FaultDoc(
        "rate_collapse", "HZ", f"{DEFAULT_COLLAPSE_HZ}",
        "Decimates output to HZ. Distinct from dropout: data keeps arriving,\n"
        "     just far too slowly to close a position loop on."),
    FaultDoc(
        "bad_quaternion", "[BODY=]nonunit|nan|zero", DEFAULT_QUAT_MODE,
        "Emits a malformed orientation. The bridge must reject the sample\n"
        "     rather than forward it, and must not resurrect the last good one."),
    FaultDoc(
        "freeze", "[BODY]", "all bodies",
        "Keeps publishing at full rate with an unchanging pose. The most\n"
        "     dangerous fault here: `ros2 topic hz` looks perfect while the link\n"
        "     is dead. Nothing downstream may treat rate as liveness."),
)

FAULT_INDEX: Dict[str, FaultDoc] = {doc.name: doc for doc in FAULT_DOCS}

# Faults whose argument is `[BODY=]VALUE`; the rest take a bare argument.
_BODY_VALUE_FAULTS = ("jump", "name_mismatch", "bad_quaternion")
# Faults whose whole argument is a body name.
_BODY_ONLY_FAULTS = ("occlusion", "freeze")

_SPEC_RE = re.compile(
    r"^(?P<name>[a-z_]+)"
    r"(?:@(?P<start>\d+(?:\.\d*)?))?"
    r"(?:\+(?P<dur>\d+(?:\.\d*)?|inf))?$")


# --------------------------------------------------------------------------- #
# pure logic -- no ROS, unit-testable
# --------------------------------------------------------------------------- #

@dataclass
class Pose6:
    """A pose in the lab frame. Quaternion order matches geometry_msgs."""
    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


def quat_norm(pose: Pose6) -> float:
    """Norm of the orientation. NaN propagates, which is the intent."""
    return math.sqrt(pose.qx * pose.qx + pose.qy * pose.qy
                     + pose.qz * pose.qz + pose.qw * pose.qw)


def yaw_quat(yaw: float) -> Tuple[float, float, float, float]:
    """Yaw-only rotation. Unit by construction, so any non-unit quaternion
    downstream came from the bad_quaternion fault and nowhere else."""
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


@dataclass
class FaultSpec:
    """One scheduled fault. Constructing it validates it.

    Arguments are decoded here rather than at activation time so that a typo on
    the command line fails at startup, not silently at t=10 s in the middle of a
    bench run.
    """
    name: str
    start_s: float = 0.0
    duration_s: float = math.inf
    arg: str = ""
    body: Optional[str] = field(default=None, init=False)   # None = all/default
    value: str = field(default="", init=False)
    number: float = field(default=0.0, init=False)          # latency s / collapse Hz
    offset: Tuple[float, float, float] = field(default=(0.0, 0.0, 0.0), init=False)

    def __post_init__(self) -> None:
        if self.name not in FAULT_INDEX:
            raise ValueError(
                f"unknown fault '{self.name}'; known faults: "
                + ", ".join(FAULT_INDEX))
        if self.start_s < 0.0:
            raise ValueError(f"{self.name}: start must be >= 0")
        if not (self.duration_s > 0.0):
            raise ValueError(f"{self.name}: duration must be > 0")

        arg = self.arg.strip()
        if self.name in _BODY_VALUE_FAULTS and "=" in arg:
            body, _, value = arg.partition("=")
            self.body = body.strip() or None
            self.value = value.strip()
        elif self.name in _BODY_ONLY_FAULTS:
            self.body = arg or None
            self.value = ""
        else:
            self.body = None
            self.value = arg

        if self.name == "latency":
            self.number = _as_float(self.value, DEFAULT_LATENCY_S, "latency seconds")
            if self.number < 0.0:
                raise ValueError("latency: seconds must be >= 0")
        elif self.name == "rate_collapse":
            self.number = _as_float(self.value, DEFAULT_COLLAPSE_HZ, "rate_collapse Hz")
            if self.number <= 0.0:
                raise ValueError("rate_collapse: Hz must be > 0")
        elif self.name == "jump":
            self.offset = _as_triple(self.value, DEFAULT_JUMP_M)
        elif self.name == "bad_quaternion":
            self.value = self.value or DEFAULT_QUAT_MODE
            if self.value not in QUAT_MODES:
                raise ValueError(
                    f"bad_quaternion: mode must be one of {'|'.join(QUAT_MODES)}")

    @property
    def end_s(self) -> float:
        return self.start_s + self.duration_s

    def active_at(self, t: float) -> bool:
        """Half-open interval: active at the start instant, clear at the end."""
        return self.start_s <= t < self.end_s

    def describe(self) -> str:
        span = "inf" if math.isinf(self.duration_s) else f"{self.duration_s:g}"
        tail = f":{self.arg}" if self.arg else ""
        return f"{self.name}@{self.start_s:g}+{span}{tail}"


def _as_float(text: str, default: float, label: str) -> float:
    if not text:
        return default
    try:
        return float(text)
    except ValueError:
        raise ValueError(f"{label}: '{text}' is not a number") from None


def _as_triple(text: str, default: Tuple[float, float, float]
               ) -> Tuple[float, float, float]:
    if not text:
        return default
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 3:
        raise ValueError(f"expected DX,DY,DZ but got '{text}'")
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError:
        raise ValueError(f"expected three numbers but got '{text}'") from None


def parse_fault_spec(text: str) -> FaultSpec:
    """`NAME[@START][+DURATION][:ARG]` -> FaultSpec.

    The argument is split off on the FIRST colon, which is unambiguous because
    the timing part never contains one.
    """
    text = text.strip()
    if not text:
        raise ValueError("empty fault specification")
    head, _, arg = text.partition(":")
    match = _SPEC_RE.match(head.strip())
    if match is None:
        raise ValueError(
            f"malformed fault specification '{text}'; "
            "expected NAME[@START][+DURATION][:ARG]")
    start = float(match.group("start")) if match.group("start") else 0.0
    dur_text = match.group("dur")
    duration = math.inf if dur_text in (None, "inf") else float(dur_text)
    return FaultSpec(name=match.group("name"), start_s=start,
                     duration_s=duration, arg=arg)


@dataclass
class Volume:
    """The netted box. `margin` is how far synthetic motion stays off the net."""
    size: Tuple[float, float, float] = DEFAULT_BOX_M
    center: Tuple[float, float, float] = DEFAULT_BOX_CENTER_M
    margin: float = DEFAULT_MARGIN_M

    def __post_init__(self) -> None:
        # Validated here so a malformed box_size_m parameter reaches the node's
        # fatal-log path as a ValueError instead of an IndexError at 100 Hz.
        if len(self.size) != 3 or len(self.center) != 3:
            raise ValueError("box size and centre each need three numbers")
        if min(self.size) <= 0.0:
            raise ValueError("box size must be positive in every axis")
        if self.margin < 0.0:
            raise ValueError("box margin must be >= 0")

    def half(self) -> Tuple[float, float, float]:
        return (0.5 * self.size[0], 0.5 * self.size[1], 0.5 * self.size[2])

    def usable(self) -> Tuple[float, float, float]:
        """Half-extents every trajectory amplitude is derived from."""
        half = self.half()
        usable = tuple(h - self.margin for h in half)
        if min(usable) <= 0.0:
            raise ValueError(
                f"box_margin_m={self.margin} leaves no room inside a "
                f"{self.size[0]}x{self.size[1]}x{self.size[2]} m box")
        return (usable[0], usable[1], usable[2])

    def contains(self, x: float, y: float, z: float, margin: float = 0.0) -> bool:
        half = self.half()
        for value, centre, extent in zip((x, y, z), self.center, half):
            if not math.isfinite(value):
                return False
            if abs(value - centre) > extent - margin + 1e-9:
                return False
        return True


# Sum-of-sines "noise": deterministic, reproducible across machines, and bounded
# to [-1, 1] because the weights sum to 1. Frequencies are mutually
# incommensurate so the pattern does not visibly repeat.
_NOISE_TERMS = ((0.73, 0.50), (1.91, 0.30), (4.37, 0.20))
_GOLDEN_ANGLE = 2.399963229728653


def bounded_noise(t: float, seed: int) -> float:
    total = 0.0
    for k, (freq, weight) in enumerate(_NOISE_TERMS):
        phase = _GOLDEN_ANGLE * (seed + 1) * (k + 1)
        total += weight * math.sin(2.0 * math.pi * freq * t + phase)
    return total


class TrajectoryGenerator:
    """Closed-form body motion, guaranteed inside the volume.

    Two separation devices keep N bodies apart without any collision checking:
    a per-body phase offset around the horizontal pattern, and a per-body
    horizontal altitude lane. Lanes are sized so that the vertical excursion of
    one body can never reach its neighbour's lane, which makes the minimum
    separation a property of the configuration rather than of the sampling.
    """

    def __init__(self, n_bodies: int, trajectory: str = DEFAULT_TRAJECTORY,
                 volume: Optional[Volume] = None,
                 period_s: float = DEFAULT_PERIOD_S,
                 jitter_m: float = DEFAULT_JITTER_M) -> None:
        if n_bodies < 1:
            raise ValueError("need at least one body")
        if trajectory not in TRAJECTORIES:
            raise ValueError(
                f"unknown trajectory '{trajectory}'; choose from "
                + ", ".join(TRAJECTORIES))
        if period_s <= 0.0:
            raise ValueError("period_s must be > 0")
        self.n = n_bodies
        self.trajectory = trajectory
        self.volume = volume if volume is not None else Volume()
        self.period_s = period_s
        self.jitter_m = abs(jitter_m)

        hx, hy, hz = self.volume.usable()
        self.hx, self.hy, self.hz = hx, hy, hz
        self.w = 2.0 * math.pi / period_s
        # Lane geometry. Lane half-height hz/n, vertical swing 0.4 of that, so
        # adjacent lanes are separated by at least 1.2 * hz/n at all times.
        self.lane_half = hz / self.n
        self.z_amp = 0.4 * self.lane_half
        self.r_orbit = 0.7 * min(hx, hy)
        self.r_slot = 0.5 * min(hx, hy)
        if self.jitter_m > 0.25 * min(hx, hy, self.lane_half):
            raise ValueError("jitter_m is too large for this box and body count")

    def phase(self, index: int) -> float:
        return 2.0 * math.pi * index / self.n

    def lane_z(self, index: int) -> float:
        return (self.volume.center[2] - self.hz
                + (2 * index + 1) * self.lane_half)

    def min_separation_m(self) -> float:
        """Guaranteed vertical separation between any two bodies."""
        if self.n < 2:
            return math.inf
        return 2.0 * self.lane_half - 2.0 * self.z_amp

    def pose(self, index: int, t: float) -> Pose6:
        cx, cy, _ = self.volume.center
        ph = self.phase(index)
        zc = self.lane_z(index)

        if self.trajectory == "static":
            x = cx + self.r_slot * math.cos(ph)
            y = cy + self.r_slot * math.sin(ph)
            z = zc
            yaw = ph

        elif self.trajectory == "circle":
            a = self.w * t + ph
            x = cx + self.r_orbit * math.cos(a)
            y = cy + self.r_orbit * math.sin(a)
            z = zc + self.z_amp * math.sin(a)
            yaw = a + 0.5 * math.pi          # nose along the tangent

        elif self.trajectory == "lissajous":
            a = 3.0 * self.w * t + ph
            b = 2.0 * self.w * t + 2.0 * ph
            x = cx + 0.8 * self.hx * math.sin(a)
            y = cy + 0.8 * self.hy * math.sin(b)
            z = zc + self.z_amp * math.sin(self.w * t + ph)
            # Analytic velocity, so yaw is exact rather than differenced.
            dx = 0.8 * self.hx * 3.0 * self.w * math.cos(a)
            dy = 0.8 * self.hy * 2.0 * self.w * math.cos(b)
            yaw = math.atan2(dy, dx)

        else:  # hover_jitter -- station keeping, not a dead hold
            j = self.jitter_m
            x = cx + self.r_slot * math.cos(ph) + j * bounded_noise(t, 4 * index)
            y = cy + self.r_slot * math.sin(ph) + j * bounded_noise(t, 4 * index + 1)
            z = zc + j * bounded_noise(t, 4 * index + 2)
            yaw = ph + 0.01 * bounded_noise(t, 4 * index + 3)

        qx, qy, qz, qw = yaw_quat(yaw)
        return Pose6(x, y, z, qx, qy, qz, qw)


@dataclass
class Frame:
    """What one tick puts on the wire.

    `publish=False` means nothing is transmitted at all -- that is how dropout
    and rate_collapse differ from every other fault, which alter content.
    """
    publish: bool
    stamp_s: float
    bodies: List[Tuple[str, Pose6]]
    active: Tuple[str, ...] = ()
    reason: str = ""


class MocapSourceCore:
    """Trajectory generation plus the fault state machine. No ROS.

    Faults compose in a fixed order, and the order is the argument for why
    overlapping faults stay sane:

        dropout        -> nothing on the wire; wins over everything
        rate_collapse  -> decides whether this tick transmits at all
        latency        -> shifts the stamp, and the sampled instant with it
        freeze         -> replaces the sampled instant for the frozen bodies
        occlusion      -> removes bodies
        jump           -> displaces position
        bad_quaternion -> corrupts orientation
        name_mismatch  -> relabels, last, so every other fault still targets the
                          body by the name the operator configured
    """

    def __init__(self, bodies: Sequence[str],
                 trajectory: str = DEFAULT_TRAJECTORY,
                 volume: Optional[Volume] = None,
                 rate_hz: float = DEFAULT_PUBLISH_RATE_HZ,
                 period_s: float = DEFAULT_PERIOD_S,
                 jitter_m: float = DEFAULT_JITTER_M,
                 faults: Sequence[FaultSpec] = ()) -> None:
        self.bodies: List[str] = list(bodies)
        if not self.bodies:
            raise ValueError("no bodies configured")
        if len(set(self.bodies)) != len(self.bodies):
            raise ValueError("body names must be unique")
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be > 0")
        self.rate_hz = rate_hz
        self.gen = TrajectoryGenerator(len(self.bodies), trajectory, volume,
                                       period_s=period_s, jitter_m=jitter_m)
        self.faults: List[FaultSpec] = []
        for spec in faults:
            self.add(spec)
        self._last_emit_s: Optional[float] = None

    # -- fault schedule ---------------------------------------------------- #

    def add(self, spec: FaultSpec) -> FaultSpec:
        """Arm a fault. Body names are checked here, where they are known."""
        if spec.body is not None and spec.body not in self.bodies:
            raise ValueError(
                f"{spec.name}: no such body '{spec.body}'; configured bodies "
                "are " + ", ".join(self.bodies))
        if spec.name == "rate_collapse" and spec.number >= self.rate_hz:
            raise ValueError(
                f"rate_collapse: {spec.number:g} Hz is not below the nominal "
                f"{self.rate_hz:g} Hz")
        self.faults.append(spec)
        return spec

    def clear(self, name: Optional[str] = None) -> int:
        """Disarm one fault by name, or everything. Returns how many went."""
        before = len(self.faults)
        if name is None:
            self.faults = []
        else:
            if name not in FAULT_INDEX:
                raise ValueError(f"unknown fault '{name}'")
            self.faults = [f for f in self.faults if f.name != name]
        return before - len(self.faults)

    def inject(self, text: str, now: float) -> str:
        """Runtime command. Start times are relative to NOW, because an operator
        typing at a bench means 'from here', not 'from node start'."""
        text = text.strip()
        if not text:
            raise ValueError("empty command")
        words = text.split(None, 1)
        head = words[0].lower()
        if head == "clear":
            target = words[1].strip() if len(words) > 1 else "all"
            count = self.clear(None if target in ("all", "*") else target)
            return f"cleared {count} fault(s)"
        if head in ("list", "status"):
            return json.dumps(self.status(now))
        parsed = parse_fault_spec(text)
        # replace() re-runs __post_init__, so the shifted copy is re-validated.
        spec = self.add(replace(parsed, start_s=parsed.start_s + now))
        return f"armed {spec.describe()} (now t={now:.3f} s)"

    def active_faults(self, t: float) -> List[FaultSpec]:
        return [f for f in self.faults if f.active_at(t)]

    def status(self, t: float) -> Dict[str, object]:
        active = self.active_faults(t)
        return {
            "t_s": round(t, 4),
            "trajectory": self.gen.trajectory,
            "rate_hz": self.rate_hz,
            "bodies": list(self.bodies),
            "active_faults": [
                {"spec": f.describe(),
                 "remaining_s": (None if math.isinf(f.end_s)
                                 else round(f.end_s - t, 3))}
                for f in active],
            "scheduled_faults": [f.describe() for f in self.faults],
        }

    # -- the tick ---------------------------------------------------------- #

    def sample(self, t: float) -> Frame:
        active = self.active_faults(t)
        names = tuple(dict.fromkeys(f.name for f in active))

        if any(f.name == "dropout" for f in active):
            # Not even a header goes out. Absence is the signal.
            return Frame(False, t, [], names, "dropout")

        collapse = _first(active, "rate_collapse")
        if collapse is not None and self._last_emit_s is not None:
            period = 1.0 / collapse.number
            if (t - self._last_emit_s) < period - RATE_EPS_S:
                return Frame(False, t, [], names, "rate_collapse")
        self._last_emit_s = t

        # Overlapping latency faults add up; anything else would need an
        # arbitrary tie-break, and cumulative delay is what really happens when
        # two links in series both degrade.
        delay = sum(f.number for f in active if f.name == "latency")
        stamp = t - delay

        occluded = {f.body if f.body is not None else self.bodies[0]
                    for f in active if f.name == "occlusion"}
        freezes = [f for f in active if f.name == "freeze"]
        jumps = [f for f in active if f.name == "jump"]
        quats = [f for f in active if f.name == "bad_quaternion"]
        renames = [f for f in active if f.name == "name_mismatch"]

        out: List[Tuple[str, Pose6]] = []
        for index, body in enumerate(self.bodies):
            if body in occluded:
                continue

            # freeze: hold the pose as of the instant the fault started. Rate is
            # untouched, which is exactly what makes this one dangerous.
            sample_t = stamp
            for spec in freezes:
                if spec.body is None or spec.body == body:
                    sample_t = min(sample_t, spec.start_s)
            pose = self.gen.pose(index, sample_t)

            for spec in jumps:
                if spec.body is None or spec.body == body:
                    # A jump may leave the box. That is the stimulus, not a bug.
                    pose = Pose6(pose.x + spec.offset[0],
                                 pose.y + spec.offset[1],
                                 pose.z + spec.offset[2],
                                 pose.qx, pose.qy, pose.qz, pose.qw)

            for spec in quats:
                if spec.body is None or spec.body == body:
                    pose = _corrupt_quat(pose, spec.value)

            label = body
            for spec in renames:
                target = spec.body if spec.body is not None else self.bodies[0]
                if target == body:
                    label = spec.value or (body + DEFAULT_MISMATCH_SUFFIX)

            out.append((label, pose))

        # An empty list is still published in rigidbodies mode: an array with
        # every body missing is precisely the stimulus for the mocap_pose
        # origin-pose bug, so it must reach the plugin.
        return Frame(True, stamp, out, names, "")


def _first(specs: Sequence[FaultSpec], name: str) -> Optional[FaultSpec]:
    for spec in specs:
        if spec.name == name:
            return spec
    return None


def _corrupt_quat(pose: Pose6, mode: str) -> Pose6:
    if mode == "nan":
        nan = float("nan")
        return Pose6(pose.x, pose.y, pose.z, nan, pose.qy, pose.qz, pose.qw)
    if mode == "zero":
        return Pose6(pose.x, pose.y, pose.z, 0.0, 0.0, 0.0, 0.0)
    scale = 1.7                     # comfortably outside any sane norm tolerance
    return Pose6(pose.x, pose.y, pose.z, pose.qx * scale, pose.qy * scale,
                 pose.qz * scale, pose.qw * scale)


def build_body_names(trackers: Sequence[str], rigid_body_names: Sequence[str],
                     mode: str) -> List[str]:
    """Which label the synthetic stream is published under.

    vrpn mode publishes one topic per tracker, so the tracker name is what the
    bridge keys on. rigidbodies mode bypasses the bridge, so the label is the
    rigid_body_name the estimator matches. Getting this backwards is the same
    class of error as the name_mismatch fault, which is why it is one function
    with one test rather than an inline conditional at each publisher.
    """
    if not trackers:
        raise ValueError("no trackers configured")
    if mode not in MODES:
        raise ValueError(f"unknown mode '{mode}'; choose from " + ", ".join(MODES))
    if rigid_body_names and len(rigid_body_names) != len(trackers):
        raise ValueError(
            f"rigid_body_names has {len(rigid_body_names)} entries but "
            f"trackers has {len(trackers)}; they must correspond one-to-one")
    if mode == "vrpn":
        return list(trackers)
    return list(rigid_body_names) if rigid_body_names else list(trackers)


# --------------------------------------------------------------------------- #
# ROS node
# --------------------------------------------------------------------------- #

def main_ros(cli: argparse.Namespace, argv: Optional[List[str]] = None) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.time import Time
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class FakeMocap(Node):
        def __init__(self) -> None:
            super().__init__("fake_mocap")

            self.declare_parameter("trackers", list(DEFAULT_TRACKERS))
            self.declare_parameter("rigid_body_names", [""])
            self.declare_parameter("mode", DEFAULT_MODE)
            self.declare_parameter("trajectory", DEFAULT_TRAJECTORY)
            self.declare_parameter("output_topic_template",
                                   DEFAULT_VRPN_TOPIC_TEMPLATE)
            self.declare_parameter("output_topic", DEFAULT_RIGIDBODIES_TOPIC)
            self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)
            self.declare_parameter("frame_id", DEFAULT_FRAME_ID)
            self.declare_parameter("box_size_m", list(DEFAULT_BOX_M))
            self.declare_parameter("box_center_m", list(DEFAULT_BOX_CENTER_M))
            self.declare_parameter("box_margin_m", DEFAULT_MARGIN_M)
            self.declare_parameter("period_s", DEFAULT_PERIOD_S)
            self.declare_parameter("jitter_m", DEFAULT_JITTER_M)
            self.declare_parameter("faults", [""])
            self.declare_parameter("inject_topic", DEFAULT_INJECT_TOPIC)
            self.declare_parameter("status_topic", DEFAULT_STATUS_TOPIC)
            self.declare_parameter("status_period_s", 1.0)

            trackers = [t for t in self.get_parameter("trackers")
                        .get_parameter_value().string_array_value if t]
            bodies = [b for b in self.get_parameter("rigid_body_names")
                      .get_parameter_value().string_array_value if b]
            if cli.bodies:
                trackers = list(cli.bodies)
                bodies = []
            if not trackers:
                self.get_logger().fatal(
                    "parameter 'trackers' is empty -- nothing to synthesise")
                raise SystemExit(2)

            self.mode = _pick(cli.mode, self.get_parameter("mode").value)
            self.frame_id = self.get_parameter("frame_id").value
            self.template = self.get_parameter("output_topic_template").value

            specs: List[FaultSpec] = []
            scheduled = [f for f in self.get_parameter("faults")
                         .get_parameter_value().string_array_value if f]
            scheduled += list(cli.fault or [])

            # Every configuration error lands in one place and exits 2 with a
            # single fatal line. A node that half-starts on a bad box or an
            # unparseable fault is worse than one that refuses.
            try:
                rate = float(_pick(cli.rate,
                                   self.get_parameter("publish_rate_hz").value))
                volume = Volume(
                    size=tuple(_pick(
                        cli.box, list(self.get_parameter("box_size_m").value))),
                    center=tuple(_pick(
                        cli.box_center,
                        list(self.get_parameter("box_center_m").value))),
                    margin=float(_pick(
                        cli.margin, self.get_parameter("box_margin_m").value)))
                names = build_body_names(trackers, bodies, self.mode)
                self.core = MocapSourceCore(
                    names,
                    trajectory=_pick(cli.trajectory,
                                     self.get_parameter("trajectory").value),
                    volume=volume,
                    rate_hz=rate,
                    period_s=float(_pick(cli.period,
                                         self.get_parameter("period_s").value)),
                    jitter_m=float(_pick(cli.jitter,
                                         self.get_parameter("jitter_m").value)))
                for text in scheduled:
                    specs.append(self.core.add(parse_fault_spec(text)))
            except ValueError as exc:
                self.get_logger().fatal(str(exc))
                raise SystemExit(2)

            # Mocap is high-rate best-effort data; match the VRPN client so the
            # bridge downstream sees the QoS it will see in O-134.
            self.sensor_qos = QoSProfile(depth=10,
                                         reliability=ReliabilityPolicy.BEST_EFFORT,
                                         history=HistoryPolicy.KEEP_LAST)

            self.pubs: Dict[str, object] = {}
            self.rb_pub = None
            if self.mode == "vrpn":
                for name in names:
                    self.pub_for(name)
            else:
                # Imported only on this path: a bench box that just runs the
                # bridge needs geometry_msgs and nothing else built.
                try:
                    from mocap4r2_msgs.msg import RigidBodies, RigidBody
                except ImportError:
                    self.get_logger().fatal(
                        "mode 'rigidbodies' needs mocap4r2_msgs; source the "
                        "workspace or use --mode vrpn")
                    raise SystemExit(2)
                self._RigidBodies = RigidBodies
                self._RigidBody = RigidBody
                self.rb_pub = self.create_publisher(
                    RigidBodies, self.get_parameter("output_topic").value,
                    self.sensor_qos)

            self.status_pub = self.create_publisher(
                String, self.get_parameter("status_topic").value, 10)
            self.create_subscription(
                String, self.get_parameter("inject_topic").value,
                self.on_inject, 10)
            self.create_service(Trigger, DEFAULT_CLEAR_SERVICE, self.on_clear)

            self.t0_ns = self.get_clock().now().nanoseconds
            self.frame_number = 0
            self._prev_active: Tuple[str, ...] = ()
            self.create_timer(1.0 / rate, self.on_timer)
            self.create_timer(float(self.get_parameter("status_period_s").value),
                              self.on_status)

            self.get_logger().info(
                f"mode '{self.mode}', trajectory '{self.core.gen.trajectory}', "
                f"{len(names)} bodies at {rate:g} Hz inside a "
                f"{volume.size[0]:g}x{volume.size[1]:g}x{volume.size[2]:g} m box "
                f"(margin {volume.margin:g} m)")
            self.get_logger().info(
                "SYNTHETIC DATA -- not a tracking system. Faults scheduled: "
                + (", ".join(s.describe() for s in specs) if specs else "none"))
            self.get_logger().info(
                "inject at runtime: ros2 topic pub --once "
                f"{self.get_parameter('inject_topic').value} std_msgs/msg/String "
                "\"{data: 'freeze+5'}\"")

        # -- plumbing ------------------------------------------------------ #

        def pub_for(self, tracker: str):
            """Publisher for one VRPN tracker topic, created on demand.

            On demand because name_mismatch renames a body at runtime, and in
            vrpn mode the name IS the topic -- so the renamed body appears on a
            topic nobody subscribes to, starving the bridge exactly as a rename
            in Motive would.
            """
            pub = self.pubs.get(tracker)
            if pub is None:
                topic = self.template.format(tracker=tracker)
                pub = self.create_publisher(PoseStamped, topic, self.sensor_qos)
                self.pubs[tracker] = pub
                self.get_logger().info(f"publishing {topic}")
            return pub

        def elapsed_s(self) -> float:
            return (self.get_clock().now().nanoseconds - self.t0_ns) * 1e-9

        def stamp_msg(self, rel_s: float):
            ns = self.t0_ns + int(round(rel_s * 1e9))
            return Time(nanoseconds=max(ns, 0)).to_msg()

        def on_timer(self) -> None:
            t = self.elapsed_s()
            frame = self.core.sample(t)
            self.log_edges(frame)
            if not frame.publish:
                return

            stamp = self.stamp_msg(frame.stamp_s)
            if self.mode == "vrpn":
                for name, pose in frame.bodies:
                    msg = PoseStamped()
                    msg.header.stamp = stamp
                    msg.header.frame_id = self.frame_id
                    msg.pose.position.x = pose.x
                    msg.pose.position.y = pose.y
                    msg.pose.position.z = pose.z
                    msg.pose.orientation.x = pose.qx
                    msg.pose.orientation.y = pose.qy
                    msg.pose.orientation.z = pose.qz
                    msg.pose.orientation.w = pose.qw
                    self.pub_for(name).publish(msg)
                return

            out = self._RigidBodies()
            out.header.stamp = stamp
            out.header.frame_id = self.frame_id
            self.frame_number += 1
            out.frame_number = self.frame_number
            for name, pose in frame.bodies:
                body = self._RigidBody()
                body.rigid_body_name = name
                body.pose.position.x = pose.x
                body.pose.position.y = pose.y
                body.pose.position.z = pose.z
                body.pose.orientation.x = pose.qx
                body.pose.orientation.y = pose.qy
                body.pose.orientation.z = pose.qz
                body.pose.orientation.w = pose.qw
                out.rigidbodies.append(body)
            self.rb_pub.publish(out)

        def log_edges(self, frame: Frame) -> None:
            """Log fault transitions, so the bench log says when the stimulus
            started without anyone having to correlate two clocks."""
            if frame.active == self._prev_active:
                return
            started = [n for n in frame.active if n not in self._prev_active]
            ended = [n for n in self._prev_active if n not in frame.active]
            if started:
                self.get_logger().warn("FAULT ACTIVE: " + ", ".join(started))
            if ended:
                self.get_logger().info("fault cleared: " + ", ".join(ended))
            self._prev_active = frame.active

        def on_inject(self, msg) -> None:
            try:
                reply = self.core.inject(msg.data, self.elapsed_s())
            except ValueError as exc:
                self.get_logger().warn(f"rejected injection '{msg.data}': {exc}")
                return
            self.get_logger().warn(f"injection: {reply}")

        def on_clear(self, request, response):
            count = self.core.clear(None)
            response.success = True
            response.message = f"cleared {count} fault(s)"
            self.get_logger().warn(response.message)
            return response

        def on_status(self) -> None:
            payload = self.core.status(self.elapsed_s())
            payload["mode"] = self.mode
            self.status_pub.publish(String(data=json.dumps(payload)))

    rclpy.init(args=argv)
    try:
        node = FakeMocap()
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
# --list-faults
# --------------------------------------------------------------------------- #

def print_faults() -> None:
    print("Fault syntax:  NAME[@START][+DURATION][:ARG]")
    print()
    print("  START      seconds; from node start on the CLI, from the moment of")
    print("             injection on the topic. Default 0.")
    print("  DURATION   seconds, or 'inf'. Default inf -- stays until cleared.")
    print("  ARG        fault-specific; see below. BODY defaults as noted.")
    print()
    print("  --fault dropout@10.0+2.0        CLI, timed from node start")
    print(f"  ros2 topic pub --once {DEFAULT_INJECT_TOPIC} std_msgs/msg/String \\")
    print("      \"{data: 'freeze+5'}\"      runtime, timed from now")
    print("  ... \"{data: 'clear freeze'}\"  or 'clear all'")
    print(f"  ros2 service call {DEFAULT_CLEAR_SERVICE} std_srvs/srv/Trigger")
    print()
    for doc in FAULT_DOCS:
        print(f"  {doc.name}")
        print(f"     arg: {doc.arg_syntax}    default: {doc.default}")
        print(f"     {doc.why}")
        print()
    print("Faults compose. Order of application:")
    print("  dropout > rate_collapse > latency > freeze > occlusion > jump >")
    print("  bad_quaternion > name_mismatch (last, so the others still target")
    print("  the body by its configured name).")


# --------------------------------------------------------------------------- #
# self-test -- no ROS
# --------------------------------------------------------------------------- #

def _dist(a: Pose6, b: Pose6) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def _core(faults: Sequence[str] = (),
          bodies: Sequence[str] = ("drone0", "drone1", "drone2"),
          trajectory: str = "circle") -> MocapSourceCore:
    return MocapSourceCore(bodies, trajectory=trajectory, volume=Volume(),
                           rate_hz=100.0, period_s=7.0,
                           faults=[parse_fault_spec(f) for f in faults])


def _ticks(core: MocapSourceCore, t0: float, count: int,
           dt: float = 0.01) -> List[Frame]:
    return [core.sample(t0 + k * dt) for k in range(count)]


def self_test() -> int:
    failures: List[str] = []

    def check(label: str, condition: bool) -> None:
        print(f"  [{'PASS' if condition else 'FAIL'}] {label}")
        if not condition:
            failures.append(label)

    print("fault specification parsing")
    spec = parse_fault_spec("dropout@10.0+2.0")
    check("parses name, start and duration",
          (spec.name, spec.start_s, spec.duration_s) == ("dropout", 10.0, 2.0))
    bare = parse_fault_spec("freeze")
    check("bare name arms at t=0 indefinitely",
          bare.start_s == 0.0 and math.isinf(bare.duration_s))
    only_dur = parse_fault_spec("dropout+2.5")
    check("duration without start",
          (only_dur.start_s, only_dur.duration_s) == (0.0, 2.5))
    check("start without duration",
          math.isinf(parse_fault_spec("jump@4").duration_s))
    check("decodes latency seconds",
          parse_fault_spec("latency@1+1:0.25").number == 0.25)
    check("decodes jump offset",
          parse_fault_spec("jump:1,2,3").offset == (1.0, 2.0, 3.0))
    check("decodes a per-body target",
          parse_fault_spec("jump:drone1=1,0,0").body == "drone1")
    check("occlusion argument is a bare body name",
          parse_fault_spec("occlusion@2+1:drone1").body == "drone1")
    check("interval is half-open at the end",
          spec.active_at(10.0) and spec.active_at(11.999)
          and not spec.active_at(12.0) and not spec.active_at(9.999))
    for bad in ("nonsense@1+1", "latency:abc", "jump:1,2",
                "bad_quaternion:sideways", "dropout@1+0", ""):
        try:
            parse_fault_spec(bad)
            check(f"rejects '{bad}'", False)
        except ValueError:
            check(f"rejects '{bad}'", True)

    print("trajectories stay inside the box")
    volume = Volume()
    for trajectory in TRAJECTORIES:
        for n in (1, 3):
            gen = TrajectoryGenerator(n, trajectory, volume, period_s=7.0,
                                      jitter_m=0.01)
            inside = margin_ok = unit = True
            min_sep = math.inf
            moved = 0.0
            first = [gen.pose(i, 0.0) for i in range(n)]
            for k in range(2001):
                t = k * 0.01
                poses = [gen.pose(i, t) for i in range(n)]
                for i, pose in enumerate(poses):
                    if not volume.contains(pose.x, pose.y, pose.z):
                        inside = False
                    if not volume.contains(pose.x, pose.y, pose.z,
                                           margin=volume.margin):
                        margin_ok = False
                    if abs(quat_norm(pose) - 1.0) > 1e-9:
                        unit = False
                    moved = max(moved, _dist(pose, first[i]))
                for a in range(n):
                    for b in range(a + 1, n):
                        min_sep = min(min_sep, _dist(poses[a], poses[b]))
            tag = f"{trajectory} n={n}"
            check(f"{tag}: inside the 8x6x4 m box", inside)
            check(f"{tag}: inside the 0.5 m margin too", margin_ok)
            check(f"{tag}: unit quaternions throughout", unit)
            if n > 1:
                floor = 0.9 * gen.min_separation_m()
                check(f"{tag}: bodies stay >{floor:.2f} m apart",
                      min_sep > floor)
            if trajectory == "static":
                check(f"{tag}: holds position exactly", moved == 0.0)
            elif trajectory == "hover_jitter":
                # Station keeping, not free flight: it must move, but never
                # further than the noise bound allows (2 per axis, three axes).
                check(f"{tag}: wanders, but stays within the jitter bound",
                      0.0 < moved <= 2.0 * math.sqrt(3.0) * 0.01 + 1e-9)
            else:
                check(f"{tag}: actually moves", moved > 0.05)
    try:
        TrajectoryGenerator(3, "circle", Volume(margin=4.0))
        check("rejects a margin larger than the box", False)
    except ValueError:
        check("rejects a margin larger than the box", True)
    for bad_box in ({"size": (8.0, 6.0)}, {"size": (8.0, 0.0, 4.0)},
                    {"margin": -1.0}):
        try:
            Volume(**bad_box)
            check(f"rejects box {bad_box}", False)
        except ValueError:
            check(f"rejects box {bad_box}", True)
    off_centre = Volume(center=(0.0, 0.0, 2.0))
    gen = TrajectoryGenerator(3, "circle", off_centre, period_s=7.0)
    check("a floor-referenced centre keeps altitude positive",
          all(gen.pose(i, t * 0.05).z > 0.0
              for i in range(3) for t in range(200)))

    print("dropout")
    core = _core(["dropout@10+2"])
    check("publishes before the fault", core.sample(9.99).publish)
    check("stops at the scheduled start", not core.sample(10.0).publish)
    check("emits no bodies at all while dropped", core.sample(11.0).bodies == [])
    check("still dropped just before the end", not core.sample(11.99).publish)
    check("resumes at the scheduled end", core.sample(12.0).publish)

    print("latency")
    clean, faulty = _core(), _core(["latency@10+2:0.25"])
    frame = faulty.sample(9.99)
    check("stamp is live outside the fault", abs(frame.stamp_s - 9.99) < 1e-9)
    frame = faulty.sample(10.5)
    check("stamp steps back by the delay",
          abs(frame.stamp_s - 10.25) < 1e-9 and frame.publish)
    check("pose is delayed with the stamp",
          frame.bodies[0][1] == clean.sample(10.25).bodies[0][1])
    check("stamp recovers after the fault",
          abs(faulty.sample(12.0).stamp_s - 12.0) < 1e-9)

    print("jump")
    clean, faulty = _core(), _core(["jump@10+2:2,0,1"])
    nominal = clean.sample(10.5).bodies[0][1]
    jumped = faulty.sample(10.5).bodies[0][1]
    check("position displaced by exactly the offset",
          abs(jumped.x - nominal.x - 2.0) < 1e-9
          and abs(jumped.y - nominal.y) < 1e-9
          and abs(jumped.z - nominal.z - 1.0) < 1e-9)
    check("orientation untouched by a jump", jumped.qw == nominal.qw)
    check("all bodies jump when none is named",
          all(abs(f.x - c.x - 2.0) < 1e-9 for (_, f), (_, c)
              in zip(faulty.sample(10.6).bodies, clean.sample(10.6).bodies)))
    check("nominal restored after the fault",
          faulty.sample(12.0).bodies[0][1] == clean.sample(12.0).bodies[0][1])

    print("occlusion")
    clean, faulty = _core(), _core(["occlusion@10+2:drone1"])
    frame = faulty.sample(10.5)
    labels = [name for name, _ in frame.bodies]
    check("named body removed", "drone1" not in labels)
    check("every other body still present", labels == ["drone0", "drone2"])
    reference = dict(clean.sample(10.5).bodies)
    check("the others are bit-for-bit unaffected",
          all(pose == reference[name] for name, pose in frame.bodies))
    check("body returns when the fault ends",
          "drone1" in [n for n, _ in faulty.sample(12.0).bodies])

    print("name_mismatch (the mocap_pose origin-pose check)")
    faulty = _core(["name_mismatch@10+2:drone0=ghost"])
    labels = [n for n, _ in faulty.sample(10.5).bodies]
    check("array stays full length -- this is why the bug is silent",
          len(labels) == 3)
    check("configured name is absent", "drone0" not in labels)
    check("wrong name is present instead", labels == ["ghost", "drone1", "drone2"])
    check("original name returns after the fault",
          "drone0" in [n for n, _ in faulty.sample(12.0).bodies])
    default = _core(["name_mismatch@0+1"])
    labels = [n for n, _ in default.sample(0.5).bodies]
    check("default renames only the first body",
          labels == ["drone0" + DEFAULT_MISMATCH_SUFFIX, "drone1", "drone2"])

    print("rate_collapse")
    faulty = _core(["rate_collapse@10+2:5"])
    frames = _ticks(faulty, 10.0, 200)
    published = sum(1 for f in frames if f.publish)
    check("output decimated to about 5 Hz over 2 s", 9 <= published <= 11)
    check("the frames that do go out carry all bodies",
          all(len(f.bodies) == 3 for f in frames if f.publish))
    check("full rate restored after the fault",
          all(f.publish for f in _ticks(faulty, 12.0, 100)))
    try:
        _core(["rate_collapse@0+1:200"])
        check("rejects a collapse rate above nominal", False)
    except ValueError:
        check("rejects a collapse rate above nominal", True)

    print("freeze (rate is not liveness)")
    clean, faulty = _core(), _core(["freeze@10+4"])
    frames = _ticks(faulty, 10.0, 200)
    check("keeps publishing every single tick",
          len(frames) == 200 and all(f.publish for f in frames))
    check("distinct from dropout: bodies are still there",
          all(len(f.bodies) == 3 for f in frames))
    poses = [f.bodies[0][1] for f in frames]
    check("pose never changes for 2 s", all(p == poses[0] for p in poses))
    check("frozen at the pose of the instant the fault started",
          poses[0] == clean.sample(10.0).bodies[0][1])
    check("motion resumes after the fault",
          faulty.sample(14.5).bodies[0][1] != poses[0])
    one = _core(["freeze@0+2:drone1"])
    a, b = one.sample(0.5), one.sample(1.5)
    frozen = dict(a.bodies)["drone1"] == dict(b.bodies)["drone1"]
    moving = dict(a.bodies)["drone0"] != dict(b.bodies)["drone0"]
    check("a named freeze holds one body while the rest fly",
          frozen and moving)

    print("bad_quaternion")
    faulty = _core(["bad_quaternion@10+2:nonunit"])
    pose = faulty.sample(10.5).bodies[0][1]
    check("emits a genuinely non-unit quaternion",
          abs(quat_norm(pose) - 1.0) > 1e-3)
    check("unit again after the fault",
          abs(quat_norm(faulty.sample(12.0).bodies[0][1]) - 1.0) < 1e-9)
    nan_pose = _core(["bad_quaternion@0+1:nan"]).sample(0.5).bodies[0][1]
    check("nan mode emits a non-finite component", math.isnan(nan_pose.qx))
    targeted = dict(_core(["bad_quaternion@0+1:drone1=zero"]).sample(0.5).bodies)
    check("targets only the named body",
          quat_norm(targeted["drone1"]) == 0.0
          and abs(quat_norm(targeted["drone0"]) - 1.0) < 1e-9)
    check("position survives a corrupt quaternion",
          math.isfinite(nan_pose.x))

    print("overlapping faults compose")
    clean = _core()
    core = _core(["freeze@10+4", "latency@11+4:0.25"])
    core.sample(10.5)
    frame = core.sample(11.5)
    check("freeze + latency: stamp still delayed",
          abs(frame.stamp_s - 11.25) < 1e-9)
    check("freeze + latency: pose still frozen at the freeze instant",
          frame.bodies[0][1] == clean.sample(10.0).bodies[0][1])
    core = _core(["dropout@10+2", "freeze@10+2"])
    check("dropout beats freeze -- nothing on the wire",
          not core.sample(10.5).publish)
    core = _core(["occlusion@0+5:drone0", "name_mismatch@0+5:drone1=ghost"])
    check("occlusion and name_mismatch hit different bodies",
          [n for n, _ in core.sample(1.0).bodies] == ["ghost", "drone2"])
    core = _core(["rate_collapse@0+2:5", "jump@0+2:2,0,1"])
    emitted = [f for f in _ticks(core, 0.0, 200) if f.publish]
    displaced = all(
        abs(f.bodies[0][1].x - _core().sample(f.stamp_s).bodies[0][1].x - 2.0) < 1e-9
        for f in emitted)
    check("rate_collapse + jump: decimated and displaced",
          9 <= len(emitted) <= 11 and displaced)
    core = _core(["latency@0+5:0.1", "latency@0+5:0.2"])
    check("two latency faults add up",
          abs(core.sample(1.0).stamp_s - 0.7) < 1e-9)
    core = _core(["occlusion@0+5:drone0", "occlusion@0+5:drone1",
                  "occlusion@0+5:drone2"])
    frame = core.sample(1.0)
    check("occluding every body still publishes an empty array "
          "(the plugin stimulus)", frame.publish and frame.bodies == [])

    print("runtime injection")
    core = _core()
    core.inject("dropout+2", now=5.0)
    check("injection is timed from now, not from node start",
          core.sample(5.5).publish is False and core.sample(7.0).publish is True)
    check("clear all disarms everything",
          core.inject("clear all", now=7.5).startswith("cleared 1")
          and core.faults == [])
    core.inject("freeze@0+1", now=8.0)
    core.inject("occlusion:drone1", now=8.0)
    check("clear by name removes only that fault",
          core.clear("freeze") == 1
          and [f.name for f in core.faults] == ["occlusion"])
    for bad in ("occlusion:drone9", "wobble+1", "clear nonsense"):
        try:
            core.inject(bad, now=9.0)
            check(f"rejects injection '{bad}'", False)
        except ValueError:
            check(f"rejects injection '{bad}'", True)
    check("status reports the active fault",
          any(entry["spec"].startswith("occlusion")
              for entry in core.status(9.0)["active_faults"]))

    print("mode selects the label that matters")
    check("vrpn mode labels by tracker (the topic name)",
          build_body_names(["1", "2"], ["drone0", "drone1"], "vrpn") == ["1", "2"])
    check("rigidbodies mode labels by rigid_body_name",
          build_body_names(["1", "2"], ["drone0", "drone1"], "rigidbodies")
          == ["drone0", "drone1"])
    check("falls back to tracker names when none are mapped",
          build_body_names(["a"], [], "rigidbodies") == ["a"])
    for args in ((["a", "b"], ["one"], "vrpn"), (["a"], [], "natnet"), ([], [], "vrpn")):
        try:
            build_body_names(*args)
            check(f"rejects {args}", False)
        except ValueError:
            check(f"rejects {args}", True)

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
        prog="fake_mocap.py",
        description="Synthetic mocap source with injectable faults (B3). "
                    "Unrecognised arguments, including --ros-args, are passed "
                    "through to rclpy.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Fault syntax: NAME[@START][+DURATION][:ARG]. "
               "See --list-faults.")
    ap.add_argument("--mode", choices=MODES, default=None,
                    help=f"output message type (default {DEFAULT_MODE})")
    ap.add_argument("--trajectory", choices=TRAJECTORIES, default=None,
                    help=f"body motion (default {DEFAULT_TRAJECTORY})")
    ap.add_argument("--bodies", nargs="+", metavar="NAME", default=None,
                    help="body names; overrides the 'trackers' parameter "
                         f"(default {' '.join(DEFAULT_TRACKERS)})")
    ap.add_argument("--rate", type=float, default=None, metavar="HZ",
                    help=f"publish rate (default {DEFAULT_PUBLISH_RATE_HZ:g})")
    ap.add_argument("--box", nargs=3, type=float, default=None,
                    metavar=("X", "Y", "Z"),
                    help="volume size in metres (default %s)"
                         % " ".join(f"{v:g}" for v in DEFAULT_BOX_M))
    ap.add_argument("--box-center", nargs=3, type=float, default=None,
                    metavar=("X", "Y", "Z"),
                    help="volume centre in metres (default 0 0 0)")
    ap.add_argument("--margin", type=float, default=None, metavar="M",
                    help="how far trajectories stay off the net "
                         f"(default {DEFAULT_MARGIN_M:g})")
    ap.add_argument("--period", type=float, default=None, metavar="S",
                    help=f"trajectory period (default {DEFAULT_PERIOD_S:g})")
    ap.add_argument("--jitter", type=float, default=None, metavar="M",
                    help="hover_jitter amplitude "
                         f"(default {DEFAULT_JITTER_M:g})")
    ap.add_argument("--fault", action="append", metavar="SPEC", default=None,
                    help="schedule a fault, e.g. dropout@10.0+2.0; repeatable")
    ap.add_argument("--list-faults", action="store_true",
                    help="print the fault catalogue and exit")
    ap.add_argument("--self-test", action="store_true",
                    help="run the offline test suite and exit (no ROS)")
    return ap


def main() -> int:
    cli, rest = build_parser().parse_known_args()
    if cli.list_faults:
        print_faults()
        return 0
    if cli.self_test:
        return self_test()
    # Syntax-check the schedule before ROS is touched: a typo should fail here,
    # in one line, not inside a node constructor on a machine that may not even
    # have rclpy sourced. Body names are checked later, where they are known.
    for text in (cli.fault or []):
        try:
            parse_fault_spec(text)
        except ValueError as exc:
            print(f"fake_mocap: --fault {text}: {exc}", file=sys.stderr)
            print("fake_mocap: see --list-faults", file=sys.stderr)
            return 2
    return main_ros(cli, rest or None)


if __name__ == "__main__":
    sys.exit(main())
