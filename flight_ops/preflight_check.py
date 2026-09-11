#!/usr/bin/env python3
"""S4: the props-on gate. One command, one PASS/FAIL table, one verdict.

Nothing is fitted with a propeller until this prints `PREFLIGHT GREEN`. The
crew reads the verdict aloud and does not proceed on anything else.

O-134 bookings are the scarce resource, so the failure modes that waste a
session are enumerated here and checked mechanically, in one 5-second window,
before anyone touches an airframe:

  * the mocap feed is nominally up but delivering 60 Hz with 200 ms holes;
  * `rigid_body_name` has a typo, so the stock mocap_pose plugin publishes the
    ORIGIN at full rate and the aircraft flies into the net believing it is
    stationary at (0, 0, 0);
  * `earth`->`map` latched to a non-identity transform during the startup race,
    so two aircraft hold different ideas of where the room is;
  * a parameter was written, the QGC popup complained, and nobody read it back;
  * the uXRCE-DDS session exists but the platform never matched on it.

Every check is independent, degrades to a FAIL with a readable reason rather
than a traceback, and reports what it actually measured -- the measurement is
the point, not the boolean.

Checks (see --list-checks):

    mocap_rate    rate, jitter and dropout on the RigidBodies feed
    rigid_bodies  each configured rigid_body_name is actually in the array
    tf_tree       earth->map->odom->base_link resolves; earth/map/odom coincide
    px4_params    live parameters match the indoor set BY READ-BACK
    dds_link      required /fmu/ topics exist AND have matched endpoints
    battery       pack voltage above the launch threshold
    as2_nodes     the expected Aerostack2 nodes are alive in each namespace
    platform      the platform reports connected
    pose_delta    self_localization agrees with raw mocap on the ground

Usage:
    # the pad command
    python3 flight_ops/preflight_check.py --drones drone0,drone1,drone2

    # Motive streams numeric body names; map namespace -> rigid body
    python3 flight_ops/preflight_check.py --drones drone0,drone1 \\
        --rigid-bodies drone0:1,drone1:2

    # longer window, machine-readable record alongside the table
    python3 flight_ops/preflight_check.py --window 10 \\
        --json flight_ops/snapshots/preflight_i04.json

    # parameter read-back only -- bench work, no ROS needed
    python3 flight_ops/preflight_check.py --only px4_params \\
        --px4-params drone0:~/dumps/droneA_after.params

    # validate the tool itself, off-site, without ROS sourced
    python3 flight_ops/preflight_check.py --self-test

Exit codes:
    0  every non-skipped check passed -- PREFLIGHT GREEN
    1  at least one check FAILED -- do not fit props
    2  the tool could not verify anything (no ROS, bad arguments, all skipped)

A note on trust: this tool never writes to the flight controller, the Jetsons
or Motive. It only reads. `px4_params` compares a READ-BACK against the
intended set, because the lab QGC is older than PX4 v1.17 and reports failures
on writes that succeeded -- and successes on writes that did not.
"""

from __future__ import annotations

import argparse
import bisect
import datetime as dt
import json
import math
import statistics
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Callable, Dict, List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
FLIGHT_OPS = REPO_ROOT / "flight_ops"
DEFAULT_PX4_EXPECTED = FLIGHT_OPS / "lab_config" / "px4_indoor_params.yaml"

PASS = "PASS"
FAIL = "FAIL"
SKIP = "SKIP"

EXIT_GREEN = 0
EXIT_FAIL = 1
EXIT_UNUSABLE = 2

# --- mocap -----------------------------------------------------------------
DEFAULT_MOCAP_TOPIC = "/mocap/rigid_bodies"
DEFAULT_MIN_MOCAP_RATE_HZ = 95.0
DEFAULT_MAX_MOCAP_GAP_MS = 100.0
# "Sustained" is measured over a sliding one-second window, not just the mean
# over the whole capture: a feed that runs at 130 Hz for 4 s and 40 Hz for 1 s
# averages above threshold while being unflyable for a second of it.
SUSTAINED_WINDOW_S = 1.0
# ...but a 1 s window sitting exactly on the threshold trips on ordinary
# jitter, so allow 10% headroom. This check is about sustained loss, not
# sampling noise.
SUSTAINED_MARGIN = 0.90
# A body must appear in nearly every message. The bridge signals "not tracked"
# by OMITTING a body, so intermittent presence is intermittent tracking.
DEFAULT_BODY_PRESENCE_FRAC = 0.90

# --- frames ----------------------------------------------------------------
DEFAULT_TF_CHAIN = "earth,map,odom,base_link"
# Aerostack2 prefixes per-drone frames with the namespace; `earth` is the one
# frame shared by every aircraft, so it is never prefixed.
DEFAULT_TF_GLOBAL_FRAMES = "earth"
# The pair-wise links that must be IDENTITY, not merely resolvable. A non-zero
# earth->map means the frame-origin latch bug is live and the aircraft do not
# share a room.
DEFAULT_TF_IDENTITY_LINKS = "earth->map,map->odom"
DEFAULT_TF_TOL_M = 0.01
DEFAULT_TF_TOL_DEG = 1.0

# --- PX4 / uXRCE-DDS -------------------------------------------------------
# Verified against PX4 v1.17 dds_topics.yaml. These are the unversioned names
# the bridge advertises; do NOT append `_v1`.
DEFAULT_FMU_TOPICS = (
    "/fmu/out/vehicle_odometry",
    "/fmu/out/vehicle_control_mode",
    # MEASURED on the FC (PX4 v1.17, 11 Sept 2026): battery_status carries a
    # MESSAGE_VERSION and is published as battery_status_v1. Eight of the 65
    # topics are versioned; the rest of this list is not. dds_topics.yaml
    # shows base names only and cannot be used to decide this.
    "/fmu/out/battery_status_v1",
    "/fmu/out/sensor_combined",
    "/fmu/out/timesync_status",
    "/fmu/in/trajectory_setpoint",
    "/fmu/in/offboard_control_mode",
    "/fmu/in/vehicle_command",
    "/fmu/in/vehicle_visual_odometry",
)

# --- Aerostack2 ------------------------------------------------------------
DEFAULT_POSE_TEMPLATE = "/{ns}/self_localization/pose"
DEFAULT_BATTERY_TEMPLATE = "/{ns}/sensor_measurements/battery"
DEFAULT_PLATFORM_TEMPLATE = "/{ns}/platform/info"
# Pin this list against the running system on the first lab visit and record it
# on the card; the default is the minimum set the campaign depends on.
DEFAULT_AS2_NODES = "platform,state_estimator,controller_manager"

# 4S LiPo on the X500 V2: 16.8 V full, 15.2 V is roughly half a pack under no
# load. Below that an indoor sortie is not worth the battery cycle.
DEFAULT_MIN_BATTERY_V = 15.2
DEFAULT_POSE_DELTA_M = 0.05

DEFAULT_WINDOW_S = 5.0
DEFAULT_DISCOVERY_S = 5.0

FLOAT_TOL = 1e-6


# --------------------------------------------------------------------------- #
# results
# --------------------------------------------------------------------------- #

@dataclass
class CheckResult:
    """One row of the table. `data` is whatever the check measured."""
    name: str
    target: str
    status: str
    detail: str
    data: Dict[str, object] = field(default_factory=dict)

    @property
    def passed(self) -> bool:
        return self.status == PASS


@dataclass
class CheckSpec:
    name: str
    summary: str
    run: Optional[Callable] = None
    # px4_params reads a dump or a MAVLink link, so it is usable on the bench
    # with no ROS running at all.
    needs_ros: bool = True


# --------------------------------------------------------------------------- #
# pure logic -- no ROS, unit-testable, exercised by --self-test
# --------------------------------------------------------------------------- #

@dataclass
class RateStats:
    count: int
    duration_s: float
    rate_hz: float
    max_gap_s: float
    mean_gap_s: float
    jitter_ms: float
    min_window_rate_hz: float


def compute_rate_stats(arrivals: Sequence[float],
                       window_start: Optional[float] = None,
                       window_end: Optional[float] = None) -> RateStats:
    """Rate, jitter and worst gap for a sequence of message arrival times.

    When the observation window is given, its edges participate in the gap
    computation. That is the whole point: a feed that delivers 500 messages in
    the first two seconds and then dies has no large INTER-MESSAGE gap, and
    would otherwise report 250 Hz and pass.
    """
    stamps = sorted(arrivals)
    count = len(stamps)

    if window_start is not None and window_end is not None:
        duration = max(window_end - window_start, 0.0)
        edges = [window_start] + stamps + [window_end]
        rate = count / duration if duration > 0 else 0.0
    else:
        duration = (stamps[-1] - stamps[0]) if count >= 2 else 0.0
        edges = list(stamps)
        rate = (count - 1) / duration if duration > 0 else 0.0

    boundary_gaps = [b - a for a, b in zip(edges, edges[1:])] if len(edges) >= 2 else []
    interior_gaps = [b - a for a, b in zip(stamps, stamps[1:])] if count >= 2 else []

    max_gap = max(boundary_gaps) if boundary_gaps else duration
    mean_gap = statistics.fmean(interior_gaps) if interior_gaps else 0.0
    jitter = statistics.pstdev(interior_gaps) * 1000.0 if len(interior_gaps) >= 2 else 0.0

    return RateStats(
        count=count,
        duration_s=duration,
        rate_hz=rate,
        max_gap_s=max_gap,
        mean_gap_s=mean_gap,
        jitter_ms=jitter,
        min_window_rate_hz=windowed_min_rate(
            stamps, SUSTAINED_WINDOW_S, window_start, window_end),
    )


def windowed_min_rate(arrivals: Sequence[float], window_s: float,
                      span_start: Optional[float] = None,
                      span_end: Optional[float] = None) -> float:
    """Worst message rate over any `window_s` slice of the observation span."""
    stamps = sorted(arrivals)
    if window_s <= 0 or not stamps:
        return 0.0
    start = span_start if span_start is not None else stamps[0]
    end = span_end if span_end is not None else stamps[-1]
    span = end - start
    if span <= 0:
        return 0.0
    if span < window_s:
        # Too short to judge sustained behaviour; report the overall rate.
        return len(stamps) / span

    worst = float("inf")
    # Candidate window starts: the span start, plus every arrival. A window
    # whose worst case is not anchored on one of these cannot be the minimum.
    for origin in [start] + stamps:
        if origin + window_s > end + 1e-9:
            break
        lo = bisect.bisect_left(stamps, origin)
        hi = bisect.bisect_left(stamps, origin + window_s)
        worst = min(worst, (hi - lo) / window_s)
    return 0.0 if worst == float("inf") else worst


def evaluate_rate(stats: RateStats, min_rate_hz: float,
                  max_gap_s: float) -> Tuple[bool, str]:
    """PASS/FAIL plus the measurement, always -- a passing check still reports
    the numbers, because a run at 96 Hz is a finding even when it passes."""
    if stats.count < 2:
        return False, (f"only {stats.count} message(s) in {stats.duration_s:.1f} s "
                       f"-- the feed is not running")

    summary = (f"{stats.rate_hz:.1f} Hz over {stats.duration_s:.1f} s, "
               f"worst {SUSTAINED_WINDOW_S:.0f} s {stats.min_window_rate_hz:.1f} Hz, "
               f"max gap {stats.max_gap_s * 1000:.1f} ms, "
               f"jitter {stats.jitter_ms:.2f} ms")

    reasons: List[str] = []
    if stats.rate_hz < min_rate_hz:
        reasons.append(f"mean rate below {min_rate_hz:.0f} Hz")
    if stats.max_gap_s > max_gap_s:
        reasons.append(f"gap exceeds {max_gap_s * 1000:.0f} ms")
    if stats.min_window_rate_hz < min_rate_hz * SUSTAINED_MARGIN:
        reasons.append(f"not sustained ({min_rate_hz * SUSTAINED_MARGIN:.0f} Hz floor)")

    if reasons:
        return False, summary + " -- " + "; ".join(reasons)
    return True, summary


def quat_angle_deg(x: float, y: float, z: float, w: float) -> float:
    """Rotation magnitude of a quaternion, in degrees. Non-finite or degenerate
    input returns 180 deg so it can never be mistaken for identity."""
    values = (x, y, z, w)
    if not all(math.isfinite(v) for v in values):
        return 180.0
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-9:
        return 180.0
    return math.degrees(2.0 * math.acos(min(1.0, abs(w) / norm)))


def transform_is_identity(translation: Sequence[float], quaternion: Sequence[float],
                          tol_m: float = DEFAULT_TF_TOL_M,
                          tol_deg: float = DEFAULT_TF_TOL_DEG) -> Tuple[bool, float, float]:
    """Returns (is_identity, |translation| in m, rotation in deg)."""
    tx, ty, tz = (float(v) for v in translation)
    if not all(math.isfinite(v) for v in (tx, ty, tz)):
        return False, float("inf"), 180.0
    dist = math.sqrt(tx * tx + ty * ty + tz * tz)
    angle = quat_angle_deg(*(float(v) for v in quaternion))
    return (dist <= tol_m and angle <= tol_deg), dist, angle


def pose_distance(a: Sequence[float], b: Sequence[float]) -> float:
    """Euclidean distance, with non-finite input reported as infinite rather
    than silently comparing as small."""
    values = [float(v) for v in a] + [float(v) for v in b]
    if not all(math.isfinite(v) for v in values):
        return float("inf")
    return math.sqrt(sum((float(p) - float(q)) ** 2 for p, q in zip(a, b)))


def qualify_frame(frame: str, namespace: str, global_frames: Sequence[str]) -> str:
    """Namespace-qualify a TF frame name. Global frames are never prefixed."""
    if frame in global_frames or frame.startswith("/"):
        return frame.lstrip("/")
    return f"{namespace.strip('/')}/{frame}"


def chain_pairs(frames: Sequence[str]) -> List[Tuple[str, str]]:
    """[a, b, c] -> [(a, b), (b, c)]"""
    return list(zip(frames, frames[1:]))


def parse_identity_links(spec: str) -> List[Tuple[str, str]]:
    """Parse 'earth->map,map->odom' into pairs of unqualified frame names."""
    pairs: List[Tuple[str, str]] = []
    for entry in (e.strip() for e in spec.split(",")):
        if not entry:
            continue
        if "->" not in entry:
            raise ValueError(f"identity link '{entry}' is not of the form parent->child")
        parent, _, child = entry.partition("->")
        pairs.append((parent.strip(), child.strip()))
    return pairs


def parse_rigid_bodies(spec: Optional[str], drones: Sequence[str]) -> Dict[str, str]:
    """Map drone namespace -> mocap rigid_body_name.

    Accepts 'drone0:1,drone1:2' (explicit) or 'A,B,C' (positional, one per
    drone). Empty means the rigid body is named after the namespace.
    """
    if not spec or not spec.strip():
        return {d: d for d in drones}
    entries = [e.strip() for e in spec.split(",") if e.strip()]
    if all(":" in e for e in entries):
        mapping: Dict[str, str] = {}
        for entry in entries:
            ns, _, body = entry.partition(":")
            mapping[ns.strip()] = body.strip()
        unknown = sorted(set(mapping) - set(drones))
        if unknown:
            raise ValueError(f"--rigid-bodies names unknown drone(s): {', '.join(unknown)}")
        missing = [d for d in drones if d not in mapping]
        if missing:
            raise ValueError(f"--rigid-bodies has no entry for: {', '.join(missing)}")
        return mapping
    if any(":" in e for e in entries):
        raise ValueError("--rigid-bodies must be either all 'ns:body' pairs or all "
                         "positional names, not a mixture")
    if len(entries) != len(drones):
        raise ValueError(f"--rigid-bodies has {len(entries)} names but "
                         f"{len(drones)} drone(s) were selected")
    return dict(zip(drones, entries))


def parse_labelled_path(spec: str) -> Tuple[Optional[str], Path]:
    """Parse 'drone0:/path/to.params'. A bare path returns label None, meaning
    it applies to every drone."""
    if ":" in spec and not Path(spec).exists():
        label, _, raw = spec.partition(":")
        candidate = Path(raw).expanduser()
        # Guard against Windows-style 'C:\...' being read as a label.
        if len(label) > 1:
            return label, candidate
    return None, Path(spec).expanduser()


def values_equal(left: object, right: object) -> bool:
    if isinstance(left, bool) or isinstance(right, bool):
        return bool(left) == bool(right)
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return abs(float(left) - float(right)) <= FLOAT_TOL
    return left == right


def normalise_expected_params(obj: object) -> Dict[str, object]:
    """Accept any of the reasonable YAML shapes for a name->value parameter set.

    Supported, because the file is hand-maintained and the operator should not
    have to remember which shape this tool wanted:
        NAME: value
        params: {NAME: value}
        NAME: {value: v, why: "..."}
        [{name: NAME, value: v}, ...]
        [{NAME: value}, ...]
    """
    if obj is None:
        return {}
    if isinstance(obj, dict):
        for key in ("params", "parameters", "px4_params", "managed_params"):
            nested = obj.get(key)
            if isinstance(nested, (dict, list)):
                return normalise_expected_params(nested)
        out: Dict[str, object] = {}
        for name, value in obj.items():
            if isinstance(value, dict):
                if "value" not in value:
                    raise ValueError(f"parameter '{name}' has no 'value' key")
                out[str(name)] = value["value"]
            else:
                out[str(name)] = value
        return out
    if isinstance(obj, list):
        out = {}
        for entry in obj:
            if isinstance(entry, dict) and "name" in entry:
                if "value" not in entry:
                    raise ValueError(f"parameter '{entry['name']}' has no 'value' key")
                out[str(entry["name"])] = entry["value"]
            elif isinstance(entry, dict) and len(entry) == 1:
                name, value = next(iter(entry.items()))
                if isinstance(value, dict):
                    if "value" not in value:
                        raise ValueError(f"parameter '{name}' has no 'value' key")
                    value = value["value"]
                out[str(name)] = value
            else:
                raise ValueError(f"cannot read parameter entry: {entry!r}")
        return out
    raise ValueError(f"expected a mapping or list of parameters, got {type(obj).__name__}")


def compare_params(expected: Dict[str, object],
                   actual: Dict[str, object]) -> List[Tuple[str, object, object]]:
    """Mismatches as (name, wanted, got). A parameter the vehicle does not
    report at all is a mismatch, not an omission -- a typo'd parameter name in
    the indoor set would otherwise pass silently."""
    mismatches = []
    for name in sorted(expected):
        want = expected[name]
        got = actual.get(name, "<absent>")
        if not values_equal(want, got):
            mismatches.append((name, want, got))
    return mismatches


def parse_param_dump(text: str) -> Dict[str, object]:
    """Read a PX4 parameter read-back.

    Handles the QGC `.params` export (`sysid compid NAME VALUE TYPE`) and the
    nsh `param show -a` / `NAME=VALUE` forms. snapshot.py owns the canonical
    version of this parser including MAVLink type recovery; this one only needs
    values, since the expected set carries no types.
    """
    values: Dict[str, object] = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.replace("=", " ").split()
        if len(fields) >= 5 and fields[0].lstrip("-").isdigit() and fields[1].isdigit():
            name, literal = fields[2], fields[3]
        elif len(fields) >= 2:
            name, literal = fields[0], fields[1]
        else:
            continue
        if not name.replace("_", "").isalnum() or name[0].isdigit():
            continue
        values[name] = _coerce(literal)
    return values


def _coerce(text: str) -> object:
    """Int when the literal has no fractional part, else float, else the text."""
    try:
        value = float(text)
    except ValueError:
        return text
    if value.is_integer() and "." not in text and "e" not in text.lower():
        return int(value)
    return value


def evaluate_endpoints(counts: Dict[str, Tuple[int, int]],
                       required: Sequence[str]) -> Tuple[List[str], List[str]]:
    """Split required topics into (absent, unmatched).

    `ros2 topic info -v` semantics: a topic with a publisher and no subscriber
    is a topic nobody is listening to. Mere existence in the graph proves
    nothing -- the uXRCE-DDS agent advertises before the platform matches.
    """
    absent, unmatched = [], []
    for topic in required:
        if topic not in counts:
            absent.append(topic)
            continue
        pubs, subs = counts[topic]
        if pubs < 1 or subs < 1:
            unmatched.append(f"{topic} (pub={pubs}, sub={subs})")
    return absent, unmatched


def select_checks(available: Sequence[str], only: Optional[str],
                  skip: Optional[str]) -> List[str]:
    """Resolve --only / --skip against the check registry."""
    def split(spec: Optional[str]) -> List[str]:
        return [s.strip() for s in (spec or "").split(",") if s.strip()]

    chosen = list(available)
    only_names, skip_names = split(only), split(skip)
    unknown = sorted(set(only_names + skip_names) - set(available))
    if unknown:
        raise ValueError(f"unknown check(s): {', '.join(unknown)}. "
                         f"Known: {', '.join(available)}")
    if only_names:
        chosen = [name for name in chosen if name in only_names]
    if skip_names:
        chosen = [name for name in chosen if name not in skip_names]
    return chosen


def render_table(results: Sequence[CheckResult]) -> str:
    """Aligned PASS/FAIL table. Details are never truncated -- the reason a
    check failed is the only thing worth reading at the pad."""
    header = ("CHECK", "TARGET", "STATUS", "DETAIL")
    rows = [(r.name, r.target or "-", r.status, r.detail) for r in results]
    if not rows:
        return "no checks were run"
    widths = [max(len(header[i]), max(len(row[i]) for row in rows)) for i in range(3)]
    lines = [
        f"{header[0]:<{widths[0]}}  {header[1]:<{widths[1]}}  "
        f"{header[2]:<{widths[2]}}  {header[3]}",
        f"{'-' * widths[0]}  {'-' * widths[1]}  {'-' * widths[2]}  "
        f"{'-' * max(len(header[3]), 40)}",
    ]
    for name, target, status, detail in rows:
        lines.append(f"{name:<{widths[0]}}  {target:<{widths[1]}}  "
                     f"{status:<{widths[2]}}  {detail}")
    return "\n".join(lines)


def tally(results: Sequence[CheckResult]) -> Dict[str, int]:
    return {
        PASS: sum(1 for r in results if r.status == PASS),
        FAIL: sum(1 for r in results if r.status == FAIL),
        SKIP: sum(1 for r in results if r.status == SKIP),
    }


def exit_code(results: Sequence[CheckResult]) -> int:
    """0 only when something was actually verified and nothing failed.

    An all-skipped run is NOT green. "Exit 0 only if every non-skipped check
    passed" is a necessary condition, not a sufficient one: a gate that returns
    zero having checked nothing is worse than no gate.
    """
    counts = tally(results)
    if counts[FAIL]:
        return EXIT_FAIL
    if counts[PASS] == 0:
        return EXIT_UNUSABLE
    return EXIT_GREEN


def summary_line(results: Sequence[CheckResult], window_s: float = 0.0) -> str:
    counts = tally(results)
    body = (f"{counts[PASS]} passed, {counts[FAIL]} failed, {counts[SKIP]} skipped"
            f"  (window {window_s:.1f} s)")
    code = exit_code(results)
    if code == EXIT_GREEN:
        return f"PREFLIGHT GREEN       {body}"
    if code == EXIT_FAIL:
        return f"PREFLIGHT RED         {body}   DO NOT FIT PROPS"
    return f"PREFLIGHT UNVERIFIED  {body}   NOTHING WAS CHECKED"


# --------------------------------------------------------------------------- #
# capture containers -- filled by the ROS layer, read by the checks
# --------------------------------------------------------------------------- #

@dataclass
class TopicCapture:
    topic: str
    type_name: Optional[str] = None
    arrivals: List[float] = field(default_factory=list)
    last_msg: Optional[object] = None
    error: Optional[str] = None


@dataclass
class MocapCapture:
    topic: str
    type_name: Optional[str] = None
    arrivals: List[float] = field(default_factory=list)
    # rigid_body_name -> number of messages it appeared in. Absence is the
    # bridge's "not tracked" signal, so the COUNT matters, not just presence.
    body_messages: Dict[str, int] = field(default_factory=dict)
    last_pose: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)
    malformed: int = 0
    error: Optional[str] = None


@dataclass
class TfLink:
    parent: str
    child: str
    ok: bool
    translation: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    quaternion: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    error: Optional[str] = None


@dataclass
class Context:
    args: argparse.Namespace
    drones: List[str]
    rigid_bodies: Dict[str, str]
    window_start: float = 0.0
    window_end: float = 0.0
    mocap: MocapCapture = field(default_factory=lambda: MocapCapture(DEFAULT_MOCAP_TOPIC))
    poses: Dict[str, TopicCapture] = field(default_factory=dict)
    batteries: Dict[str, TopicCapture] = field(default_factory=dict)
    platforms: Dict[str, TopicCapture] = field(default_factory=dict)
    endpoints: Dict[str, Tuple[int, int]] = field(default_factory=dict)
    nodes: List[Tuple[str, str]] = field(default_factory=list)
    tf: Dict[str, List[TfLink]] = field(default_factory=dict)
    tf_error: Optional[str] = None
    ros_available: bool = False


# --------------------------------------------------------------------------- #
# the checks
# --------------------------------------------------------------------------- #

def check_mocap_rate(ctx: Context) -> List[CheckResult]:
    cap = ctx.mocap
    if cap.error:
        return [CheckResult("mocap_rate", cap.topic, FAIL, cap.error)]
    stats = compute_rate_stats(cap.arrivals, ctx.window_start, ctx.window_end)
    ok, detail = evaluate_rate(stats, ctx.args.min_mocap_rate,
                               ctx.args.max_mocap_gap_ms / 1000.0)
    if cap.malformed:
        detail += f"; {cap.malformed} malformed message(s)"
        ok = False
    return [CheckResult("mocap_rate", cap.topic, PASS if ok else FAIL,
                        detail, asdict(stats))]


def check_rigid_bodies(ctx: Context) -> List[CheckResult]:
    """The typo check. The stock Aerostack2 mocap_pose plugin calls
    process_mocap_pose() whether or not the configured name matched, so a
    missing name publishes a default-constructed pose at the ORIGIN, at full
    rate, with no warning. This check is the only thing between a typo in a
    launch file and an aircraft flying into the net."""
    cap = ctx.mocap
    results = []
    total = len(cap.arrivals)
    seen = sorted(cap.body_messages)
    for ns in ctx.drones:
        body = ctx.rigid_bodies[ns]
        if cap.error:
            results.append(CheckResult("rigid_bodies", ns, FAIL, cap.error))
            continue
        if total == 0:
            results.append(CheckResult("rigid_bodies", ns, FAIL,
                                       f"no messages on {cap.topic}"))
            continue
        count = cap.body_messages.get(body, 0)
        frac = count / total
        if count == 0:
            results.append(CheckResult(
                "rigid_bodies", ns, FAIL,
                f"'{body}' NEVER appeared in rigidbodies[]; seen: "
                f"{seen or ['<none>']} -- mocap_pose would publish the ORIGIN",
                {"rigid_body_name": body, "present_in": 0, "seen": seen}))
        elif frac < ctx.args.body_presence_frac:
            results.append(CheckResult(
                "rigid_bodies", ns, FAIL,
                f"'{body}' present in only {count}/{total} messages "
                f"({frac * 100:.1f}%) -- intermittent tracking",
                {"rigid_body_name": body, "present_in": count, "messages": total}))
        else:
            results.append(CheckResult(
                "rigid_bodies", ns, PASS,
                f"'{body}' in {count}/{total} messages ({frac * 100:.1f}%)",
                {"rigid_body_name": body, "present_in": count, "messages": total}))
    return results


def check_tf_tree(ctx: Context) -> List[CheckResult]:
    results = []
    identity_links = parse_identity_links(ctx.args.tf_identity_links)
    globals_ = [f.strip() for f in ctx.args.tf_global_frames.split(",") if f.strip()]
    for ns in ctx.drones:
        if ctx.tf_error:
            results.append(CheckResult("tf_tree", ns, FAIL, ctx.tf_error))
            continue
        links = ctx.tf.get(ns, [])
        if not links:
            results.append(CheckResult("tf_tree", ns, FAIL, "no transforms looked up"))
            continue
        problems, notes = [], []
        must_be_identity = {
            (qualify_frame(p, ns, globals_), qualify_frame(c, ns, globals_))
            for p, c in identity_links
        }
        for link in links:
            if not link.ok:
                problems.append(f"{link.parent}->{link.child}: "
                                f"{link.error or 'not resolvable'}")
                continue
            if (link.parent, link.child) in must_be_identity:
                identity, dist, angle = transform_is_identity(
                    link.translation, link.quaternion,
                    ctx.args.tf_tol_m, ctx.args.tf_tol_deg)
                text = (f"{link.parent}->{link.child} "
                        f"{dist * 100:.1f} cm / {angle:.2f} deg")
                if identity:
                    notes.append(text)
                else:
                    # A non-identity earth->map is the frame-origin latch bug:
                    # each aircraft ends up with its own idea of the room.
                    problems.append(text + " NOT identity -- frame-origin latch")
        if problems:
            results.append(CheckResult("tf_tree", ns, FAIL, "; ".join(problems),
                                       {"links": [asdict(e) for e in links]}))
        else:
            detail = f"{len(links)} link(s) resolve"
            if notes:
                detail += "; coincident: " + ", ".join(notes)
            results.append(CheckResult("tf_tree", ns, PASS, detail,
                                       {"links": [asdict(e) for e in links]}))
    return results


def check_px4_params(ctx: Context) -> List[CheckResult]:
    """Read-back comparison. Never trust a write: the lab QGC is older than
    PX4 v1.17 and both warns about writes that succeeded and accepts writes
    that were rejected."""
    expected_path = Path(ctx.args.px4_expected).expanduser()
    try:
        expected = load_expected_params(expected_path)
    except FileNotFoundError:
        return [CheckResult("px4_params", "-", SKIP,
                            f"{expected_path} does not exist yet -- S5 has not "
                            f"produced the indoor parameter set")]
    except (OSError, ValueError, ImportError) as exc:
        return [CheckResult("px4_params", "-", FAIL, f"{expected_path}: {exc}")]
    if not expected:
        return [CheckResult("px4_params", "-", SKIP,
                            f"{expected_path} contains no parameters")]

    sources = resolve_param_sources(ctx)
    if not sources:
        return [CheckResult("px4_params", "-", SKIP,
                            f"{len(expected)} parameter(s) expected but no read-back "
                            f"source given; pass --px4-params LABEL:PATH or "
                            f"--px4-conn (a write is not evidence)")]

    results = []
    for label, (actual, error) in sorted(sources.items()):
        if error:
            results.append(CheckResult("px4_params", label, FAIL, error))
            continue
        mismatches = compare_params(expected, actual)
        if mismatches:
            shown = "; ".join(f"{n}: want {w}, read {g}" for n, w, g in mismatches[:6])
            if len(mismatches) > 6:
                shown += f"; (+{len(mismatches) - 6} more)"
            results.append(CheckResult(
                "px4_params", label, FAIL,
                f"{len(mismatches)}/{len(expected)} mismatched -- {shown}",
                {"mismatches": [{"param": n, "want": w, "read": g}
                                for n, w, g in mismatches]}))
        else:
            results.append(CheckResult(
                "px4_params", label, PASS,
                f"all {len(expected)} parameter(s) read back as intended",
                {"checked": len(expected)}))
    return results


def check_dds_link(ctx: Context) -> List[CheckResult]:
    topics = [t.strip() for t in ctx.args.fmu_topics.split(",") if t.strip()]
    prefix = ctx.args.fmu_prefix
    targets = ctx.drones if "{ns}" in prefix else ["-"]
    results = []
    for target in targets:
        resolved = [prefix.format(ns=target) + t if "{ns}" in prefix else prefix + t
                    for t in topics]
        absent, unmatched = evaluate_endpoints(ctx.endpoints, resolved)
        if absent or unmatched:
            parts = []
            if absent:
                parts.append(f"absent: {', '.join(absent)}")
            if unmatched:
                parts.append(f"no matched endpoint: {', '.join(unmatched)}")
            results.append(CheckResult(
                "dds_link", target, FAIL, "; ".join(parts),
                {"absent": absent, "unmatched": unmatched}))
        else:
            results.append(CheckResult(
                "dds_link", target, PASS,
                f"all {len(resolved)} /fmu/ topics present with matched endpoints",
                {"topics": resolved}))
    return results


def check_battery(ctx: Context) -> List[CheckResult]:
    results = []
    for ns in ctx.drones:
        cap = ctx.batteries.get(ns)
        if cap is None or cap.error:
            detail = cap.error if cap else "not subscribed"
            results.append(CheckResult("battery", ns, FAIL, detail))
            continue
        if cap.last_msg is None:
            results.append(CheckResult("battery", ns, FAIL,
                                       f"no message on {cap.topic} in "
                                       f"{ctx.window_end - ctx.window_start:.1f} s"))
            continue
        voltage = float(getattr(cap.last_msg, "voltage", float("nan")))
        percentage = float(getattr(cap.last_msg, "percentage", float("nan")))
        data = {"voltage_v": voltage, "percentage": percentage,
                "messages": len(cap.arrivals)}
        if not math.isfinite(voltage):
            results.append(CheckResult("battery", ns, FAIL,
                                       "voltage is not a number", data))
            continue
        pct_text = f", {percentage * 100:.0f}%" if math.isfinite(percentage) else ""
        reasons = []
        if voltage < ctx.args.min_battery_v:
            reasons.append(f"below {ctx.args.min_battery_v:.1f} V")
        # The percentage floor is off by default: PX4's estimate is only
        # meaningful once the pack has been characterised for this airframe.
        if (ctx.args.min_battery_pct > 0 and math.isfinite(percentage)
                and percentage < ctx.args.min_battery_pct):
            reasons.append(f"below {ctx.args.min_battery_pct * 100:.0f}%")
        detail = f"{voltage:.2f} V{pct_text}"
        if reasons:
            results.append(CheckResult("battery", ns, FAIL,
                                       detail + " -- " + "; ".join(reasons), data))
        else:
            results.append(CheckResult("battery", ns, PASS, detail, data))
    return results


def check_as2_nodes(ctx: Context) -> List[CheckResult]:
    expected = [n.strip() for n in ctx.args.as2_nodes.split(",") if n.strip()]
    results = []
    for ns in ctx.drones:
        want_ns = "/" + ns.strip("/")
        alive = {name for name, namespace in ctx.nodes
                 if namespace.rstrip("/") == want_ns}
        missing = [n for n in expected if n not in alive]
        if missing:
            results.append(CheckResult(
                "as2_nodes", ns, FAIL,
                f"missing: {', '.join(missing)}; alive in {want_ns}: "
                f"{sorted(alive) or ['<none>']}",
                {"missing": missing, "alive": sorted(alive)}))
        else:
            results.append(CheckResult(
                "as2_nodes", ns, PASS, f"{len(expected)} node(s) alive in {want_ns}",
                {"alive": sorted(alive)}))
    return results


def check_platform(ctx: Context) -> List[CheckResult]:
    results = []
    for ns in ctx.drones:
        cap = ctx.platforms.get(ns)
        if cap is None or cap.error:
            results.append(CheckResult("platform", ns,
                                       FAIL, cap.error if cap else "not subscribed"))
            continue
        if cap.last_msg is None:
            results.append(CheckResult("platform", ns, FAIL,
                                       f"no message on {cap.topic} in "
                                       f"{ctx.window_end - ctx.window_start:.1f} s"))
            continue
        if not hasattr(cap.last_msg, "connected"):
            results.append(CheckResult(
                "platform", ns, FAIL,
                f"{cap.topic} is {cap.type_name}, which has no 'connected' field"))
            continue
        connected = bool(cap.last_msg.connected)
        armed = getattr(cap.last_msg, "armed", None)
        offboard = getattr(cap.last_msg, "offboard", None)
        data = {"connected": connected, "armed": armed, "offboard": offboard}
        detail = (f"connected={connected}"
                  + (f", armed={bool(armed)}" if armed is not None else "")
                  + (f", offboard={bool(offboard)}" if offboard is not None else ""))
        if not connected:
            results.append(CheckResult("platform", ns, FAIL,
                                       detail + " -- platform is not talking to the FC",
                                       data))
        elif armed:
            # Props-off gate: an armed vehicle at S4 means someone is ahead of
            # the run-sheet.
            results.append(CheckResult("platform", ns, FAIL,
                                       detail + " -- ARMED before the preflight gate",
                                       data))
        else:
            results.append(CheckResult("platform", ns, PASS, detail, data))
    return results


def check_pose_delta(ctx: Context) -> List[CheckResult]:
    """The check that catches a frame or naming mismatch on the ground.

    Compares what the estimator believes against mocap truth. It is only
    meaningful alongside tf_tree: earth/map/odom coinciding is what makes the
    two positions directly comparable.
    """
    results = []
    for ns in ctx.drones:
        body = ctx.rigid_bodies[ns]
        cap = ctx.poses.get(ns)
        if cap is None or cap.error:
            results.append(CheckResult("pose_delta", ns,
                                       FAIL, cap.error if cap else "not subscribed"))
            continue
        truth = ctx.mocap.last_pose.get(body)
        if truth is None:
            results.append(CheckResult(
                "pose_delta", ns, FAIL,
                f"no mocap pose for rigid body '{body}' -- nothing to compare against"))
            continue
        if cap.last_msg is None:
            results.append(CheckResult("pose_delta", ns, FAIL,
                                       f"no message on {cap.topic} in "
                                       f"{ctx.window_end - ctx.window_start:.1f} s"))
            continue
        try:
            position = cap.last_msg.pose.position
            estimate = (position.x, position.y, position.z)
        except AttributeError:
            results.append(CheckResult(
                "pose_delta", ns, FAIL,
                f"{cap.topic} is {cap.type_name}, which is not a PoseStamped"))
            continue
        delta = pose_distance(estimate, truth)
        data = {"self_localization": list(estimate), "mocap": list(truth),
                "delta_m": delta}
        detail = (f"delta {delta * 100:.1f} cm  "
                  f"(est {estimate[0]:.3f},{estimate[1]:.3f},{estimate[2]:.3f} vs "
                  f"mocap {truth[0]:.3f},{truth[1]:.3f},{truth[2]:.3f})")
        if delta > ctx.args.pose_delta_m:
            results.append(CheckResult(
                "pose_delta", ns, FAIL,
                detail + f" -- exceeds {ctx.args.pose_delta_m * 100:.0f} cm; suspect "
                         f"frame or rigid_body_name mismatch", data))
        else:
            results.append(CheckResult("pose_delta", ns, PASS, detail, data))
    return results


CHECKS: List[CheckSpec] = [
    CheckSpec("mocap_rate", "RigidBodies feed rate, jitter and dropouts",
              check_mocap_rate),
    CheckSpec("rigid_bodies", "each configured rigid_body_name is in the array",
              check_rigid_bodies),
    CheckSpec("tf_tree", "earth->map->odom->base_link resolves and coincides",
              check_tf_tree),
    CheckSpec("px4_params", "live PX4 parameters match the indoor set (read-back)",
              check_px4_params, needs_ros=False),
    CheckSpec("dds_link", "required /fmu/ topics exist with matched endpoints",
              check_dds_link),
    CheckSpec("battery", "pack voltage above the launch threshold", check_battery),
    CheckSpec("as2_nodes", "expected Aerostack2 nodes alive per namespace",
              check_as2_nodes),
    CheckSpec("platform", "platform reports connected (and not already armed)",
              check_platform),
    CheckSpec("pose_delta", "self_localization agrees with raw mocap",
              check_pose_delta),
]

CHECK_NAMES = [spec.name for spec in CHECKS]


def run_check(spec: CheckSpec, ctx: Context) -> List[CheckResult]:
    """Run one check, converting any escaped exception into a FAIL row.

    Belt and braces on top of each check's own error handling: at the pad, a
    traceback is indistinguishable from a broken tool, and a broken tool stops
    the session just as surely as a broken aircraft.
    """
    try:
        results = spec.run(ctx)
    except Exception as exc:  # noqa: BLE001 -- never let a check raise
        return [CheckResult(spec.name, "-", FAIL,
                            f"check raised {type(exc).__name__}: {exc}")]
    return results or [CheckResult(spec.name, "-", SKIP, "nothing to check")]


# --------------------------------------------------------------------------- #
# parameter read-back sources (no ROS required)
# --------------------------------------------------------------------------- #

def load_expected_params(path: Path) -> Dict[str, object]:
    """Load the intended indoor parameter set. Raises FileNotFoundError when
    S5 has not produced it yet -- that is a SKIP, not a failure."""
    if not path.exists():
        raise FileNotFoundError(path)
    try:
        import yaml
    except ImportError as exc:
        raise ImportError(f"PyYAML is required to read {path.name} ({exc})")
    with path.open("r", encoding="utf-8") as handle:
        return normalise_expected_params(yaml.safe_load(handle))


def resolve_param_sources(ctx: Context) -> Dict[str, Tuple[Dict[str, object],
                                                           Optional[str]]]:
    """label -> (values, error). Dump files first, then a live MAVLink read."""
    sources: Dict[str, Tuple[Dict[str, object], Optional[str]]] = {}
    for spec in ctx.args.px4_params:
        label, path = parse_labelled_path(spec)
        labels = [label] if label else list(ctx.drones) or ["vehicle"]
        try:
            values = parse_param_dump(path.read_text(encoding="utf-8", errors="replace"))
            error = None if values else f"{path} contained no parameters"
        except OSError as exc:
            values, error = {}, f"cannot read {path}: {exc}"
        for name in labels:
            sources[name] = (values, error)

    if ctx.args.px4_conn:
        label = f"mavlink:{ctx.args.px4_conn}"
        try:
            expected = load_expected_params(Path(ctx.args.px4_expected).expanduser())
            sources[label] = (read_params_over_mavlink(
                ctx.args.px4_conn, ctx.args.px4_baud, sorted(expected),
                ctx.args.px4_read_timeout), None)
        except Exception as exc:  # noqa: BLE001 -- a link failure is a FAIL row
            sources[label] = ({}, f"MAVLink read-back failed: "
                                  f"{type(exc).__name__}: {exc}")
    return sources


def read_params_over_mavlink(conn: str, baud: int, names: Sequence[str],
                             timeout_s: float) -> Dict[str, object]:
    """Read parameters one by one over MAVLink. Read-only; nothing is written.

    PX4 packs integer parameters into PARAM_VALUE's float field BYTEWISE, not
    by numeric cast. Reading `param_value` directly corrupts any int above
    2^24 -- which includes the packed IP addresses in the UXRCE_DDS set. The
    same reinterpretation appears on the write side in snapshot.py's generated
    rollback.
    """
    import struct

    from pymavlink import mavutil

    int_types = {
        mavutil.mavlink.MAV_PARAM_TYPE_INT8, mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
        mavutil.mavlink.MAV_PARAM_TYPE_INT16, mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32, mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
    }

    link = mavutil.mavlink_connection(conn, baud=baud)
    try:
        if link.wait_heartbeat(timeout=timeout_s) is None:
            raise TimeoutError(f"no heartbeat on {conn} within {timeout_s:.0f} s")
        values: Dict[str, object] = {}
        for name in names:
            link.mav.param_request_read_send(
                link.target_system, link.target_component, name.encode("ascii"), -1)
            msg = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout_s)
            if msg is None:
                continue
            param_id = msg.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode("ascii", "replace")
            param_id = param_id.rstrip("\x00")
            if msg.param_type in int_types:
                values[param_id] = struct.unpack(
                    "<i", struct.pack("<f", msg.param_value))[0]
            else:
                values[param_id] = float(msg.param_value)
        return values
    finally:
        link.close()


# --------------------------------------------------------------------------- #
# ROS collection layer -- imported lazily so --self-test needs no ROS
# --------------------------------------------------------------------------- #

class RosUnavailable(RuntimeError):
    pass


def _sensor_qos(depth: int = 50):
    """The most permissive subscriber QoS available.

    BEST_EFFORT/VOLATILE matches RELIABLE and TRANSIENT_LOCAL publishers as
    well as best-effort ones, so a QoS incompatibility can never be reported
    to the crew as a dead topic. The cost is that a latched topic published
    once before we subscribed is not replayed -- every topic this tool reads
    publishes at a rate.
    """
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    return QoSProfile(depth=depth,
                      history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def _extract_bodies(msg) -> List[Tuple[str, Tuple[float, float, float]]]:
    """Pull (name, position) out of a mocap4r2_msgs/RigidBodies message.

    Read by duck typing rather than by importing mocap4r2_msgs: the message
    type is resolved from the graph, so an unexpected type on the topic
    becomes a readable FAIL instead of an import error at startup.
    """
    out = []
    for body in getattr(msg, "rigidbodies", []):
        name = getattr(body, "rigid_body_name", None)
        pose = getattr(body, "pose", None)
        position = getattr(pose, "position", None)
        if name is None or position is None:
            raise ValueError("message has no rigid_body_name/pose.position")
        out.append((str(name), (float(position.x), float(position.y),
                                float(position.z))))
    return out


def collect(args: argparse.Namespace, drones: List[str],
            rigid_bodies: Dict[str, str], needed: Sequence[str]) -> Context:
    """One discovery pass, one capture window, one graph snapshot.

    All ROS-facing checks share a single window. Running them in sequence would
    multiply the pad time by nine and, worse, would let a fault appear and
    disappear between checks.
    """
    ctx = Context(args=args, drones=drones, rigid_bodies=rigid_bodies)
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        raise RosUnavailable(
            f"rclpy is not importable ({exc}). Source ROS 2 humble and the "
            f"Aerostack2 workspace first.")

    try:
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RosUnavailable(f"rosidl_runtime_py is not importable ({exc})")

    wanted: Dict[str, str] = {}
    if {"mocap_rate", "rigid_bodies", "pose_delta"} & set(needed):
        wanted[args.mocap_topic] = "mocap"
    for ns in drones:
        if "pose_delta" in needed:
            wanted[args.pose_topic.format(ns=ns)] = f"pose:{ns}"
        if "battery" in needed:
            wanted[args.battery_topic.format(ns=ns)] = f"battery:{ns}"
        if "platform" in needed:
            wanted[args.platform_topic.format(ns=ns)] = f"platform:{ns}"

    ctx.mocap = MocapCapture(args.mocap_topic)
    for ns in drones:
        ctx.poses[ns] = TopicCapture(args.pose_topic.format(ns=ns))
        ctx.batteries[ns] = TopicCapture(args.battery_topic.format(ns=ns))
        ctx.platforms[ns] = TopicCapture(args.platform_topic.format(ns=ns))

    rclpy.init(args=None)
    node = Node("preflight_check")
    try:
        ctx.ros_available = True

        # Discovery: wait for the topics to appear rather than sampling the
        # graph once, because on a freshly launched stack the graph fills over
        # a second or two and a single sample reports a healthy system as dead.
        found: Dict[str, str] = {}
        deadline = time.monotonic() + args.discovery_timeout
        while time.monotonic() < deadline:
            for topic, types in node.get_topic_names_and_types():
                if topic in wanted and topic not in found and types:
                    found[topic] = types[0]
            if len(found) == len(wanted):
                break
            rclpy.spin_once(node, timeout_sec=0.1)

        _install_subscriptions(node, ctx, wanted, found, get_message)

        tf_buffer = None
        # Bound for the lifetime of the capture: the listener must not be
        # collected while the buffer is filling. It deliberately runs without
        # its own spin thread -- the loop below is what serves its callbacks,
        # so the buffer covers exactly the window the other checks measure.
        tf_listener = None
        if "tf_tree" in needed:
            try:
                from tf2_ros import Buffer, TransformListener
                tf_buffer = Buffer()
                tf_listener = TransformListener(tf_buffer, node)
            except Exception as exc:  # noqa: BLE001 -- a FAIL row, not a crash
                tf_buffer = None
                ctx.tf_error = f"tf2_ros unavailable ({type(exc).__name__}: {exc})"

        ctx.window_start = time.monotonic()
        end = ctx.window_start + args.window
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        ctx.window_end = time.monotonic()

        if "dds_link" in needed:
            ctx.endpoints = _endpoint_counts(node)
        if "as2_nodes" in needed:
            ctx.nodes = [(name, namespace)
                         for name, namespace in node.get_node_names_and_namespaces()]
        if "tf_tree" in needed and tf_buffer is not None:
            ctx.tf = _lookup_tf(tf_buffer, drones, args)
        del tf_listener
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001 -- shutdown must never mask a result
            pass
    return ctx


def _install_subscriptions(node, ctx: Context, wanted: Dict[str, str],
                           found: Dict[str, str], get_message) -> None:
    """Subscribe using the type discovered on the graph, so a topic carrying an
    unexpected type fails readably instead of raising on import."""
    qos = _sensor_qos()

    def capture_for(role: str) -> Optional[object]:
        kind, _, ns = role.partition(":")
        if kind == "mocap":
            return ctx.mocap
        return {"pose": ctx.poses, "battery": ctx.batteries,
                "platform": ctx.platforms}[kind].get(ns)

    for topic, role in wanted.items():
        target = capture_for(role)
        if target is None:
            continue
        type_name = found.get(topic)
        if type_name is None:
            target.error = (f"{topic} is not advertised after "
                            f"{ctx.args.discovery_timeout:.0f} s -- is the "
                            f"publisher running?")
            continue
        target.type_name = type_name
        try:
            msg_type = get_message(type_name)
        except (ImportError, ValueError, AttributeError) as exc:
            target.error = f"{topic}: cannot load message type {type_name} ({exc})"
            continue

        if target is ctx.mocap:
            node.create_subscription(
                msg_type, topic, lambda msg: _on_mocap(ctx.mocap, msg), qos)
        else:
            node.create_subscription(
                msg_type, topic,
                lambda msg, cap=target: _on_generic(cap, msg), qos)


def _on_mocap(cap: MocapCapture, msg) -> None:
    cap.arrivals.append(time.monotonic())
    try:
        bodies = _extract_bodies(msg)
    except (ValueError, TypeError, AttributeError):
        cap.malformed += 1
        return
    # Counted once per message even if Motive streams two bodies under the
    # same name, so the presence fraction can never exceed 100% and read as
    # healthy when it is in fact a duplicate-name misconfiguration.
    for name in {n for n, _ in bodies}:
        cap.body_messages[name] = cap.body_messages.get(name, 0) + 1
    for name, position in bodies:
        cap.last_pose[name] = position


def _on_generic(cap: TopicCapture, msg) -> None:
    cap.arrivals.append(time.monotonic())
    cap.last_msg = msg


def _endpoint_counts(node) -> Dict[str, Tuple[int, int]]:
    """Publisher/subscriber counts per topic -- `ros2 topic info -v` semantics
    without shelling out."""
    counts: Dict[str, Tuple[int, int]] = {}
    for topic, _types in node.get_topic_names_and_types():
        try:
            counts[topic] = (node.count_publishers(topic),
                             node.count_subscribers(topic))
        except Exception:  # noqa: BLE001 -- a bad name must not kill the sweep
            counts[topic] = (0, 0)
    return counts


def _lookup_tf(buffer, drones: Sequence[str],
               args: argparse.Namespace) -> Dict[str, List[TfLink]]:
    """Look up each chain link from the buffer filled during the window.

    Lookups use the latest available transform with a zero timeout: a non-zero
    timeout would block waiting for a spin that is no longer happening on this
    thread, turning a missing transform into a hang instead of a FAIL.
    """
    import rclpy.time

    chain = [f.strip() for f in args.tf_chain.split(",") if f.strip()]
    globals_ = [f.strip() for f in args.tf_global_frames.split(",") if f.strip()]
    out: Dict[str, List[TfLink]] = {}
    for ns in drones:
        links: List[TfLink] = []
        for parent, child in chain_pairs(chain):
            p = qualify_frame(parent, ns, globals_)
            c = qualify_frame(child, ns, globals_)
            try:
                tf = buffer.lookup_transform(p, c, rclpy.time.Time())
                t, q = tf.transform.translation, tf.transform.rotation
                links.append(TfLink(p, c, True, (t.x, t.y, t.z),
                                    (q.x, q.y, q.z, q.w)))
            except Exception as exc:  # noqa: BLE001 -- tf2 raises many types
                links.append(TfLink(p, c, False, error=str(exc).strip() or
                                    type(exc).__name__))
        out[ns] = links
    return out


# --------------------------------------------------------------------------- #
# self-test -- all pure logic, synthetic data, no ROS
# --------------------------------------------------------------------------- #

def self_test() -> int:
    failures: List[str] = []

    def check(label: str, condition: bool) -> None:
        print(f"  [{'PASS' if condition else 'FAIL'}] {label}")
        if not condition:
            failures.append(label)

    print("rate statistics")
    clean = [i / 100.0 for i in range(500)]              # 100 Hz for 5 s
    stats = compute_rate_stats(clean, 0.0, 5.0)
    check("counts every message", stats.count == 500)
    check("reports 100 Hz", abs(stats.rate_hz - 100.0) < 0.5)
    check("max gap is one period", abs(stats.max_gap_s - 0.01) < 1e-6)
    check("jitter is zero on a perfect feed", stats.jitter_ms < 1e-6)
    check("sustained rate matches mean",
          abs(stats.min_window_rate_hz - 100.0) < 1.0)

    ok, detail = evaluate_rate(stats, 95.0, 0.100)
    check("clean 100 Hz feed passes", ok)
    check("passing detail still reports the numbers", "100.0 Hz" in detail)

    print("gap detection")
    with_gap = [t for t in clean if not (2.0 < t < 2.15)]  # 150 ms hole
    stats_gap = compute_rate_stats(with_gap, 0.0, 5.0)
    check("finds the 150 ms hole", abs(stats_gap.max_gap_s - 0.15) < 0.02)
    ok, detail = evaluate_rate(stats_gap, 95.0, 0.100)
    check("a 150 ms hole fails", not ok)
    check("failure names the gap", "gap exceeds" in detail)

    print("a feed that dies mid-window")
    # The failure mode that a naive (n-1)/(t_last-t_first) rate misses entirely.
    died = [i / 100.0 for i in range(200)]                # 100 Hz then silence
    stats_died = compute_rate_stats(died, 0.0, 5.0)
    check("rate is computed over the WINDOW, not the data",
          abs(stats_died.rate_hz - 40.0) < 1.0)
    check("the trailing silence is a gap", stats_died.max_gap_s > 2.9)
    ok, _ = evaluate_rate(stats_died, 95.0, 0.100)
    check("a feed that dies fails", not ok)

    print("a feed that starts late")
    late = [2.0 + i / 100.0 for i in range(300)]
    stats_late = compute_rate_stats(late, 0.0, 5.0)
    check("the leading silence is a gap", stats_late.max_gap_s > 1.9)

    print("sustained-rate detection")
    # 130 Hz for 4 s then 40 Hz for 1 s: the mean passes, the second does not.
    fast = [i / 130.0 for i in range(520)]
    slow = [4.0 + i / 40.0 for i in range(40)]
    mixed = fast + slow
    stats_mixed = compute_rate_stats(mixed, 0.0, 5.0)
    check("mean rate alone would pass", stats_mixed.rate_hz > 100.0)
    check("worst second is caught", stats_mixed.min_window_rate_hz < 60.0)
    ok, detail = evaluate_rate(stats_mixed, 95.0, 0.100)
    check("a one-second collapse fails", not ok)
    check("failure names sustainment", "not sustained" in detail)

    print("degenerate input")
    check("empty capture reports zero rate",
          compute_rate_stats([], 0.0, 5.0).rate_hz == 0.0)
    ok, detail = evaluate_rate(compute_rate_stats([], 0.0, 5.0), 95.0, 0.1)
    check("empty capture fails without raising", not ok)
    check("empty capture says the feed is not running", "not running" in detail)
    check("single message fails",
          not evaluate_rate(compute_rate_stats([1.0], 0.0, 5.0), 95.0, 0.1)[0])
    check("windowed rate on empty input is zero", windowed_min_rate([], 1.0) == 0.0)
    check("windowed rate on a short span falls back to overall",
          windowed_min_rate([0.0, 0.1, 0.2], 1.0) > 0.0)

    print("identity transforms")
    ident = (0.0, 0.0, 0.0)
    unit = (0.0, 0.0, 0.0, 1.0)
    check("exact identity is identity", transform_is_identity(ident, unit)[0])
    check("5 mm offset is within tolerance",
          transform_is_identity((0.005, 0.0, 0.0), unit)[0])
    check("20 cm offset is NOT identity",
          not transform_is_identity((0.2, 0.0, 0.0), unit)[0])
    # 90 deg about z: the classic latched-origin yaw.
    yaw90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    identity, dist, angle = transform_is_identity(ident, yaw90)
    check("90 deg yaw is NOT identity", not identity)
    check("90 deg yaw is measured as 90 deg", abs(angle - 90.0) < 1e-6)
    check("0.5 deg yaw is within tolerance",
          transform_is_identity(ident, (0.0, 0.0, math.sin(math.radians(0.25)),
                                        math.cos(math.radians(0.25))))[0])
    check("NaN translation is never identity",
          not transform_is_identity((float("nan"), 0.0, 0.0), unit)[0])
    check("zero quaternion is never identity",
          not transform_is_identity(ident, (0.0, 0.0, 0.0, 0.0))[0])
    check("negated identity quaternion is still identity",
          transform_is_identity(ident, (0.0, 0.0, 0.0, -1.0))[0])

    print("frame naming")
    check("earth is not namespaced",
          qualify_frame("earth", "drone0", ["earth"]) == "earth")
    check("base_link is namespaced",
          qualify_frame("base_link", "drone0", ["earth"]) == "drone0/base_link")
    check("chain becomes pairs",
          chain_pairs(["earth", "map", "odom", "base_link"])
          == [("earth", "map"), ("map", "odom"), ("odom", "base_link")])
    check("identity links parse",
          parse_identity_links("earth->map, map->odom")
          == [("earth", "map"), ("map", "odom")])
    try:
        parse_identity_links("earth:map")
        check("malformed identity link is rejected", False)
    except ValueError:
        check("malformed identity link is rejected", True)

    print("pose delta")
    check("identical poses have zero delta",
          pose_distance((1.0, 2.0, 3.0), (1.0, 2.0, 3.0)) == 0.0)
    check("3-4-5 triangle", abs(pose_distance((0, 0, 0), (3, 4, 0)) - 5.0) < 1e-9)
    check("2 cm delta is under a 5 cm threshold",
          pose_distance((0, 0, 0), (0.02, 0, 0)) < DEFAULT_POSE_DELTA_M)
    check("NaN delta is infinite, never small",
          pose_distance((0, 0, 0), (float("nan"), 0, 0)) == float("inf"))
    # The origin-pose failure mode: mocap_pose publishing (0,0,0) while the
    # aircraft is 2.5 m away.
    check("origin pose against a real position is a large delta",
          pose_distance((0.0, 0.0, 0.0), (1.5, 2.0, 1.0)) > 2.0)

    print("rigid body mapping")
    check("defaults to the namespace name",
          parse_rigid_bodies(None, ["drone0", "drone1"])
          == {"drone0": "drone0", "drone1": "drone1"})
    check("positional names map in order",
          parse_rigid_bodies("A,B", ["drone0", "drone1"])
          == {"drone0": "A", "drone1": "B"})
    check("explicit pairs map by namespace",
          parse_rigid_bodies("drone1:2,drone0:1", ["drone0", "drone1"])
          == {"drone0": "1", "drone1": "2"})
    for bad, why in (("A", "wrong count"), ("drone9:1", "unknown drone"),
                     ("drone0:1,B", "mixed forms")):
        try:
            parse_rigid_bodies(bad, ["drone0", "drone1"])
            check(f"rejects {why}", False)
        except ValueError:
            check(f"rejects {why}", True)

    print("expected parameter shapes")
    flat = {"MPC_XY_VEL_MAX": 1.0, "EKF2_EV_CTRL": 15}
    check("flat mapping", normalise_expected_params(flat) == flat)
    check("nested under params:",
          normalise_expected_params({"params": flat}) == flat)
    check("value/why mapping",
          normalise_expected_params({"GF_ACTION": {"value": 2, "why": "land"}})
          == {"GF_ACTION": 2})
    check("list of name/value dicts",
          normalise_expected_params([{"name": "A", "value": 1},
                                     {"name": "B", "value": 2.5}])
          == {"A": 1, "B": 2.5})
    check("list of single-entry dicts",
          normalise_expected_params([{"A": 1}, {"B": 2.5}]) == {"A": 1, "B": 2.5})
    check("empty file is an empty set", normalise_expected_params(None) == {})
    try:
        normalise_expected_params({"A": {"why": "no value here"}})
        check("rejects an entry with no value", False)
    except ValueError:
        check("rejects an entry with no value", True)

    print("parameter read-back comparison")
    expected = {"MPC_XY_VEL_MAX": 1.0, "EKF2_EV_CTRL": 15, "GF_ACTION": 2}
    check("exact read-back passes",
          compare_params(expected, dict(expected)) == [])
    check("float tolerance absorbs representation",
          compare_params({"A": 1.0}, {"A": 1.0 + 1e-9}) == [])
    check("int and float compare equal", values_equal(15, 15.0))
    check("a real difference is caught",
          compare_params({"A": 1.0}, {"A": 2.0}) == [("A", 1.0, 2.0)])
    absent = compare_params(expected, {"MPC_XY_VEL_MAX": 1.0, "EKF2_EV_CTRL": 15})
    check("an absent parameter is a mismatch, not an omission",
          absent == [("GF_ACTION", 2, "<absent>")])
    check("extra live parameters are ignored",
          compare_params({"A": 1}, {"A": 1, "B": 99}) == [])
    check("mismatches are ordered by name",
          [m[0] for m in compare_params({"B": 1, "A": 1}, {})] == ["A", "B"])

    print("parameter dump parsing")
    qgc = ("# Onboard parameters\n"
           "# MAV ID\tCOMPONENT ID\tPARAM NAME\tVALUE\tTYPE\n"
           "1\t1\tMPC_XY_VEL_MAX\t1.000000\t9\n"
           "1\t1\tEKF2_EV_CTRL\t15\t6\n")
    parsed = parse_param_dump(qgc)
    check("QGC export parses", parsed == {"MPC_XY_VEL_MAX": 1.0,
                                          "EKF2_EV_CTRL": 15})
    check("nsh 'NAME VALUE' parses",
          parse_param_dump("GF_ACTION 2\nRTL_RETURN_ALT 2.5\n")
          == {"GF_ACTION": 2, "RTL_RETURN_ALT": 2.5})
    check("NAME=VALUE parses", parse_param_dump("COM_DISARM_LAND=2.0")
          == {"COM_DISARM_LAND": 2.0})
    check("comments and blank lines are ignored",
          parse_param_dump("# note\n\nA 1\n") == {"A": 1})

    print("endpoint matching")
    counts = {"/fmu/out/vehicle_odometry": (1, 1),
              "/fmu/in/trajectory_setpoint": (1, 0),
              "/fmu/out/battery_status": (0, 1)}
    absent, unmatched = evaluate_endpoints(
        counts, ["/fmu/out/vehicle_odometry", "/fmu/in/trajectory_setpoint",
                 "/fmu/out/battery_status", "/fmu/out/sensor_combined"])
    check("a missing topic is absent", absent == ["/fmu/out/sensor_combined"])
    check("existence without a subscriber is unmatched",
          any("trajectory_setpoint" in u for u in unmatched))
    check("existence without a publisher is unmatched",
          any("battery_status" in u for u in unmatched))
    check("a fully matched topic is neither",
          all("vehicle_odometry" not in u for u in unmatched))
    # Pinned to what the flight controller actually publishes, not to a theory
    # about it. An earlier version of this test asserted the opposite.
    check("battery_status is the versioned name the FC publishes",
          "/fmu/out/battery_status_v1" in DEFAULT_FMU_TOPICS
          and "/fmu/out/battery_status" not in DEFAULT_FMU_TOPICS)
    check("the flight-critical topics are unversioned, as measured",
          all(t in DEFAULT_FMU_TOPICS for t in (
              "/fmu/in/trajectory_setpoint", "/fmu/in/offboard_control_mode",
              "/fmu/in/vehicle_command", "/fmu/in/vehicle_visual_odometry",
              "/fmu/out/vehicle_odometry")))

    print("check selection")
    check("default selects everything", select_checks(CHECK_NAMES, None, None)
          == CHECK_NAMES)
    check("--only narrows",
          select_checks(CHECK_NAMES, "battery,tf_tree", None) == ["tf_tree", "battery"])
    check("--skip removes",
          "battery" not in select_checks(CHECK_NAMES, None, "battery"))
    check("--only then --skip",
          select_checks(CHECK_NAMES, "battery,tf_tree", "battery") == ["tf_tree"])
    try:
        select_checks(CHECK_NAMES, "no_such_check", None)
        check("unknown check name is rejected", False)
    except ValueError:
        check("unknown check name is rejected", True)

    print("table rendering")
    rows = [
        CheckResult("mocap_rate", "/mocap/rigid_bodies", PASS, "99.8 Hz"),
        CheckResult("rigid_bodies", "drone0", FAIL, "'drone0' NEVER appeared"),
        CheckResult("px4_params", "-", SKIP, "no expected set yet"),
    ]
    table = render_table(rows)
    lines = table.splitlines()
    check("header plus separator plus one row each", len(lines) == 5)
    check("columns are aligned",
          len({line.index(PASS) for line in lines if PASS in line}
              | {line.index(FAIL) for line in lines if FAIL in line}) == 1)
    check("detail is not truncated", "'drone0' NEVER appeared" in table)
    check("empty result set renders a sentence, not a crash",
          "no checks" in render_table([]))

    print("exit-code selection")
    all_pass = [CheckResult("a", "-", PASS, ""), CheckResult("b", "-", PASS, "")]
    check("all pass is green", exit_code(all_pass) == EXIT_GREEN)
    check("green summary says GREEN", "PREFLIGHT GREEN" in summary_line(all_pass))
    one_fail = all_pass + [CheckResult("c", "-", FAIL, "")]
    check("any failure is non-zero", exit_code(one_fail) == EXIT_FAIL)
    check("red summary forbids props",
          "DO NOT FIT PROPS" in summary_line(one_fail))
    with_skip = all_pass + [CheckResult("c", "-", SKIP, "")]
    check("a skip alongside passes is still green",
          exit_code(with_skip) == EXIT_GREEN)
    all_skip = [CheckResult("a", "-", SKIP, ""), CheckResult("b", "-", SKIP, "")]
    check("an all-skipped run is NOT green", exit_code(all_skip) == EXIT_UNUSABLE)
    check("all-skipped says nothing was checked",
          "NOTHING WAS CHECKED" in summary_line(all_skip))
    check("no results at all is not green", exit_code([]) == EXIT_UNUSABLE)
    check("a failure outranks a skip",
          exit_code(all_skip + [CheckResult("c", "-", FAIL, "")]) == EXIT_FAIL)
    check("counts are reported",
          tally(one_fail) == {PASS: 2, FAIL: 1, SKIP: 0})

    print("check registry")
    check("nine checks are registered", len(CHECKS) == 9)
    check("every check has a runner", all(spec.run is not None for spec in CHECKS))
    check("check names are unique", len(set(CHECK_NAMES)) == len(CHECKS))
    check("a raising check becomes a FAIL row, not a traceback",
          run_check(CheckSpec("boom", "", lambda ctx: (_ for _ in ()).throw(
              RuntimeError("synthetic"))), None)[0].status == FAIL)
    check("a check returning nothing becomes a SKIP row",
          run_check(CheckSpec("quiet", "", lambda ctx: []), None)[0].status == SKIP)

    print("mocap message ingestion")

    def body(name: str, x: float, y: float, z: float) -> object:
        return SimpleNamespace(rigid_body_name=name, pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z)))

    cap = MocapCapture("/mocap/rigid_bodies")
    _on_mocap(cap, SimpleNamespace(rigidbodies=[body("drone0", 1, 2, 3),
                                                body("drone1", 4, 5, 6)]))
    check("every body in the array is counted",
          cap.body_messages == {"drone0": 1, "drone1": 1})
    check("the position is captured", cap.last_pose["drone1"] == (4.0, 5.0, 6.0))
    _on_mocap(cap, SimpleNamespace(rigidbodies=[body("drone0", 1, 2, 3),
                                                body("drone0", 9, 9, 9)]))
    check("a duplicate name counts once per message",
          cap.body_messages["drone0"] == 2)
    check("presence can never exceed the message count",
          all(n <= len(cap.arrivals) for n in cap.body_messages.values()))
    _on_mocap(cap, SimpleNamespace(rigidbodies=[SimpleNamespace(
        rigid_body_name="drone0")]))
    check("a body with no pose is counted malformed, not raised",
          cap.malformed == 1)
    _on_mocap(cap, SimpleNamespace(rigidbodies=[]))
    check("an empty array is a message with no bodies, not an error",
          cap.malformed == 1 and len(cap.arrivals) == 4)

    print("checks against a synthetic capture")
    # Drives every check body with fabricated captures, so a fault in a check
    # is found here rather than in O-134 with the crew standing around.
    def pose_msg(x: float, y: float, z: float) -> object:
        return SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z)))

    def fresh_ctx() -> Context:
        args = build_parser().parse_args(["--drones", "drone0", "--window", "5"])
        ctx = Context(args=args, drones=["drone0"],
                      rigid_bodies={"drone0": "drone0"},
                      window_start=0.0, window_end=5.0)
        ctx.mocap = MocapCapture("/mocap/rigid_bodies")
        ctx.mocap.arrivals = list(clean)
        ctx.mocap.body_messages = {"drone0": 500}
        ctx.mocap.last_pose = {"drone0": (1.500, 2.000, 0.500)}
        ctx.poses["drone0"] = TopicCapture("/drone0/self_localization/pose")
        ctx.poses["drone0"].last_msg = pose_msg(1.505, 2.003, 0.498)
        ctx.batteries["drone0"] = TopicCapture("/drone0/sensor_measurements/battery")
        ctx.batteries["drone0"].last_msg = SimpleNamespace(voltage=16.2,
                                                           percentage=0.92)
        ctx.platforms["drone0"] = TopicCapture("/drone0/platform/info")
        ctx.platforms["drone0"].last_msg = SimpleNamespace(
            connected=True, armed=False, offboard=False)
        ctx.endpoints = {t: (1, 1) for t in DEFAULT_FMU_TOPICS}
        ctx.nodes = [(n, "/drone0") for n in DEFAULT_AS2_NODES.split(",")]
        globals_ = [DEFAULT_TF_GLOBAL_FRAMES]
        ctx.tf = {"drone0": [
            TfLink(qualify_frame(p, "drone0", globals_),
                   qualify_frame(c, "drone0", globals_), True,
                   (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
            for p, c in chain_pairs(DEFAULT_TF_CHAIN.split(","))]}
        # base_link carries the aircraft's actual position; only the frames
        # above it must coincide.
        ctx.tf["drone0"][-1].translation = (1.5, 2.0, 0.5)
        return ctx

    healthy = fresh_ctx()
    for spec in CHECKS:
        if spec.name == "px4_params":
            continue
        rows = run_check(spec, healthy)
        check(f"{spec.name} passes on a healthy system",
              all(r.status == PASS for r in rows) and len(rows) >= 1)

    missing = fresh_ctx()
    missing.rigid_bodies = {"drone0": "Rigid Body 1"}
    row = run_check(CHECKS[1], missing)[0]
    check("a typo'd rigid_body_name fails", row.status == FAIL)
    check("the typo failure shows the names actually streaming",
          "drone0" in row.detail and "ORIGIN" in row.detail)

    intermittent = fresh_ctx()
    intermittent.mocap.body_messages = {"drone0": 200}
    check("intermittent tracking fails",
          run_check(CHECKS[1], intermittent)[0].status == FAIL)

    latched = fresh_ctx()
    latched.tf["drone0"][0].translation = (0.5, 0.0, 0.0)   # earth->map offset
    row = run_check(CHECKS[2], latched)[0]
    check("a non-identity earth->map fails", row.status == FAIL)
    check("the latch failure is named", "latch" in row.detail)

    broken_tf = fresh_ctx()
    broken_tf.tf["drone0"][2] = TfLink("drone0/odom", "drone0/base_link", False,
                                       error="frame does not exist")
    check("an unresolvable TF link fails",
          run_check(CHECKS[2], broken_tf)[0].status == FAIL)

    no_tf2 = fresh_ctx()
    no_tf2.tf_error = "tf2_ros is not importable"
    check("a missing tf2_ros is a FAIL, not a traceback",
          run_check(CHECKS[2], no_tf2)[0].status == FAIL)

    unmatched = fresh_ctx()
    unmatched.endpoints["/fmu/in/trajectory_setpoint"] = (1, 0)
    row = run_check(CHECKS[4], unmatched)[0]
    check("an unmatched /fmu/ endpoint fails", row.status == FAIL)
    check("the unmatched topic is named", "trajectory_setpoint" in row.detail)

    absent_topic = fresh_ctx()
    del absent_topic.endpoints["/fmu/out/vehicle_odometry"]
    check("an absent /fmu/ topic fails",
          run_check(CHECKS[4], absent_topic)[0].status == FAIL)

    flat = fresh_ctx()
    flat.batteries["drone0"].last_msg = SimpleNamespace(voltage=14.1,
                                                        percentage=0.15)
    check("a flat pack fails", run_check(CHECKS[5], flat)[0].status == FAIL)
    silent = fresh_ctx()
    silent.batteries["drone0"].last_msg = None
    check("a silent battery topic fails",
          run_check(CHECKS[5], silent)[0].status == FAIL)
    nan_battery = fresh_ctx()
    nan_battery.batteries["drone0"].last_msg = SimpleNamespace(
        voltage=float("nan"), percentage=float("nan"))
    check("a NaN voltage fails rather than comparing as small",
          run_check(CHECKS[5], nan_battery)[0].status == FAIL)

    dead_node = fresh_ctx()
    dead_node.nodes = [("platform", "/drone0")]
    row = run_check(CHECKS[6], dead_node)[0]
    check("a missing AS2 node fails", row.status == FAIL)
    check("the missing node is named", "state_estimator" in row.detail)
    wrong_ns = fresh_ctx()
    wrong_ns.nodes = [(n, "/drone1") for n in DEFAULT_AS2_NODES.split(",")]
    check("nodes in another namespace do not count",
          run_check(CHECKS[6], wrong_ns)[0].status == FAIL)

    disconnected = fresh_ctx()
    disconnected.platforms["drone0"].last_msg = SimpleNamespace(
        connected=False, armed=False, offboard=False)
    check("a disconnected platform fails",
          run_check(CHECKS[7], disconnected)[0].status == FAIL)
    armed = fresh_ctx()
    armed.platforms["drone0"].last_msg = SimpleNamespace(
        connected=True, armed=True, offboard=False)
    check("an already-armed vehicle fails the props-off gate",
          run_check(CHECKS[7], armed)[0].status == FAIL)
    wrong_type = fresh_ctx()
    wrong_type.platforms["drone0"].last_msg = SimpleNamespace(status=0)
    wrong_type.platforms["drone0"].type_name = "std_msgs/msg/Int32"
    check("an unexpected platform message type fails readably",
          run_check(CHECKS[7], wrong_type)[0].status == FAIL)

    origin = fresh_ctx()
    origin.poses["drone0"].last_msg = pose_msg(0.0, 0.0, 0.0)
    row = run_check(CHECKS[8], origin)[0]
    check("estimator at the origin while mocap says otherwise fails",
          row.status == FAIL)
    check("the delta failure points at frames or naming",
          "rigid_body_name" in row.detail)
    no_truth = fresh_ctx()
    no_truth.mocap.last_pose = {}
    check("no mocap truth for the body is a FAIL, not a crash",
          run_check(CHECKS[8], no_truth)[0].status == FAIL)
    not_a_pose = fresh_ctx()
    not_a_pose.poses["drone0"].last_msg = SimpleNamespace(data=1)
    check("a non-PoseStamped on the pose topic fails readably",
          run_check(CHECKS[8], not_a_pose)[0].status == FAIL)

    dead_feed = fresh_ctx()
    dead_feed.mocap.error = "/mocap/rigid_bodies is not advertised"
    check("an unadvertised mocap topic fails every mocap check",
          all(run_check(CHECKS[i], dead_feed)[0].status == FAIL for i in (0, 1)))

    no_expected = fresh_ctx()
    no_expected.args.px4_expected = Path("no_such_indoor_params.yaml")
    row = run_check(CHECKS[3], no_expected)[0]
    check("an absent px4_indoor_params.yaml skips gracefully",
          row.status == SKIP)
    check("the skip says why", "does not exist yet" in row.detail)

    print("labelled path parsing")
    label, path = parse_labelled_path("drone0:/tmp/a.params")
    check("label is split off", label == "drone0" and path.name == "a.params")
    check("a bare path has no label",
          parse_labelled_path("/tmp/a.params")[0] is None)
    check("a Windows drive letter is not a label",
          parse_labelled_path("C:/tmp/a.params")[0] is None)

    print()
    if failures:
        print(f"{len(failures)} FAILURE(S): " + "; ".join(failures))
        return 1
    print("all self-tests passed")
    return 0


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drones", default="drone0,drone1,drone2",
                    help="comma-separated namespaces (default: %(default)s)")
    ap.add_argument("--rigid-bodies", default=None,
                    help="'ns:body,...' or positional 'A,B,C'; default: the "
                         "rigid body is named after the namespace")
    ap.add_argument("--window", type=float, default=DEFAULT_WINDOW_S,
                    help="capture window in seconds (default: %(default)s)")
    ap.add_argument("--discovery-timeout", type=float, default=DEFAULT_DISCOVERY_S,
                    help="seconds to wait for topics to appear on the graph")
    ap.add_argument("--json", type=Path, default=None,
                    help="also write machine-readable results here")
    ap.add_argument("--only", default=None, help="run only these checks")
    ap.add_argument("--skip", default=None, help="run everything except these")
    ap.add_argument("--list-checks", action="store_true",
                    help="print the check registry and exit (no ROS needed)")
    ap.add_argument("--self-test", action="store_true",
                    help="exercise all pure logic on synthetic data (no ROS)")

    mocap = ap.add_argument_group("mocap")
    mocap.add_argument("--mocap-topic", default=DEFAULT_MOCAP_TOPIC)
    mocap.add_argument("--min-mocap-rate", type=float,
                       default=DEFAULT_MIN_MOCAP_RATE_HZ)
    mocap.add_argument("--max-mocap-gap-ms", type=float,
                       default=DEFAULT_MAX_MOCAP_GAP_MS)
    mocap.add_argument("--body-presence-frac", type=float,
                       default=DEFAULT_BODY_PRESENCE_FRAC,
                       help="fraction of messages a rigid body must appear in")

    frames = ap.add_argument_group("frames")
    frames.add_argument("--tf-chain", default=DEFAULT_TF_CHAIN)
    frames.add_argument("--tf-global-frames", default=DEFAULT_TF_GLOBAL_FRAMES,
                        help="frames that are NOT namespace-prefixed")
    frames.add_argument("--tf-identity-links", default=DEFAULT_TF_IDENTITY_LINKS,
                        help="links that must be identity, 'parent->child,...'")
    frames.add_argument("--tf-tol-m", type=float, default=DEFAULT_TF_TOL_M)
    frames.add_argument("--tf-tol-deg", type=float, default=DEFAULT_TF_TOL_DEG)
    frames.add_argument("--pose-delta-m", type=float, default=DEFAULT_POSE_DELTA_M,
                        help="max self_localization vs mocap delta (default: %(default)s)")

    px4 = ap.add_argument_group("px4")
    px4.add_argument("--px4-expected", type=Path, default=DEFAULT_PX4_EXPECTED,
                     help="intended indoor parameter set (default: %(default)s)")
    px4.add_argument("--px4-params", action="append", default=[],
                     metavar="LABEL:PATH",
                     help="parameter READ-BACK dump; repeatable. A bare path "
                          "applies to every drone.")
    px4.add_argument("--px4-conn", default=None,
                     help="pymavlink connection string for a live read-back, "
                          "e.g. /dev/ttyACM0 or udp:0.0.0.0:14550")
    px4.add_argument("--px4-baud", type=int, default=57600)
    px4.add_argument("--px4-read-timeout", type=float, default=5.0)
    px4.add_argument("--fmu-prefix", default="",
                     help="prefix for /fmu/ topics; include '{ns}' to check "
                          "them per drone (default: bare /fmu/...)")
    px4.add_argument("--fmu-topics", default=",".join(DEFAULT_FMU_TOPICS),
                     help="required uXRCE-DDS topics (PX4 v1.17 names)")

    as2 = ap.add_argument_group("aerostack2")
    as2.add_argument("--pose-topic", default=DEFAULT_POSE_TEMPLATE)
    as2.add_argument("--battery-topic", default=DEFAULT_BATTERY_TEMPLATE)
    as2.add_argument("--platform-topic", default=DEFAULT_PLATFORM_TEMPLATE)
    as2.add_argument("--as2-nodes", default=DEFAULT_AS2_NODES,
                     help="node names expected in each namespace")
    as2.add_argument("--min-battery-v", type=float, default=DEFAULT_MIN_BATTERY_V)
    as2.add_argument("--min-battery-pct", type=float, default=0.0,
                     help="0 disables; PX4's estimate is only meaningful once "
                          "the pack is characterised")
    return ap


def write_json(path: Path, args: argparse.Namespace, drones: Sequence[str],
               results: Sequence[CheckResult], code: int) -> None:
    payload = {
        "generated_utc": dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "drones": list(drones),
        "window_s": args.window,
        "green": code == EXIT_GREEN,
        "exit_code": code,
        "counts": tally(results),
        "results": [asdict(r) for r in results],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, default=str), encoding="utf-8")


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    if args.self_test:
        return self_test()

    if args.list_checks:
        width = max(len(spec.name) for spec in CHECKS)
        for spec in CHECKS:
            ros = "" if spec.needs_ros else "   (no ROS required)"
            print(f"{spec.name:<{width}}  {spec.summary}{ros}")
        return EXIT_GREEN

    drones = [d.strip() for d in args.drones.split(",") if d.strip()]
    if not drones:
        print("preflight: --drones selected no namespaces", file=sys.stderr)
        return EXIT_UNUSABLE
    try:
        selected = select_checks(CHECK_NAMES, args.only, args.skip)
        rigid_bodies = parse_rigid_bodies(args.rigid_bodies, drones)
    except ValueError as exc:
        print(f"preflight: {exc}", file=sys.stderr)
        return EXIT_UNUSABLE
    if not selected:
        print("preflight: --only/--skip left no checks to run", file=sys.stderr)
        return EXIT_UNUSABLE

    specs = [spec for spec in CHECKS if spec.name in selected]
    ctx = Context(args=args, drones=drones, rigid_bodies=rigid_bodies)
    if any(spec.needs_ros for spec in specs):
        try:
            ctx = collect(args, drones, rigid_bodies, selected)
        except RosUnavailable as exc:
            print(f"preflight: {exc}", file=sys.stderr)
            return EXIT_UNUSABLE
        except Exception as exc:  # noqa: BLE001 -- report, never traceback
            print(f"preflight: capture failed: {type(exc).__name__}: {exc}",
                  file=sys.stderr)
            return EXIT_UNUSABLE

    results: List[CheckResult] = []
    for spec in specs:
        results.extend(run_check(spec, ctx))

    print(render_table(results))
    print()
    print(summary_line(results, args.window))

    code = exit_code(results)
    if args.json:
        try:
            write_json(args.json, args, drones, results, code)
            print(f"[preflight] wrote {args.json}")
        except OSError as exc:
            print(f"[preflight] could not write {args.json}: {exc}", file=sys.stderr)
    return code


if __name__ == "__main__":
    sys.exit(main())
