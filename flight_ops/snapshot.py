#!/usr/bin/env python3
"""Gate G0: read-only snapshot of shared lab state, plus rollback generation.

Captures every piece of state this project is capable of changing -- BEFORE
anything is changed -- into a timestamped folder, then evaluates the
refuse-to-proceed gates in lab_config/expected_state.yaml and emits a rollback
that restores exactly the managed parameters to their captured values.

Nothing here writes to the flight controller, the Jetson, or Motive. Every
source degrades to "unavailable" rather than failing the run, so a snapshot
taken without hardware present is still a valid record.

Sources:
  git         repo HEAD, branch, dirty files                       (always)
  env         ROS/AS2 environment variables                        (always)
  apt         installed ros-humble-* package versions              (WSL)
  ros_graph   ros2 topic list -t, ros2 node list                   (ROS running)
  px4         parameter dump, from a QGC .params export or a
              'param show -a' capture, one per airframe            (--px4-params)
  jetson      ip addr, failed units, uname                         (ssh, BatchMode)
  motive      copy of the Motive project file                      (--motive-project)

Usage:
    # local-only record, no hardware present
    python3 flight_ops/snapshot.py --label pre-b1 --local-only

    # full pre-session capture with two airframe parameter dumps
    python3 flight_ops/snapshot.py --label session0-pre \
        --px4-params droneA:~/dumps/droneA.params \
        --px4-params droneB:~/dumps/droneB.params \
        --baseline flight_ops/snapshots/20260904T031500Z \
        --check

    # end-of-session drift check against the snapshot taken at the start
    python3 flight_ops/snapshot.py --label session0-post \
        --px4-params droneA:~/dumps/droneA_after.params \
        --baseline flight_ops/snapshots/20260904T031500Z --check

Exit codes:
    0  snapshot written, gates passed (or --check not requested)
    1  snapshot written, a must_equal gate FAILED -- do not proceed
    2  snapshot could not be written
"""

from __future__ import annotations

import argparse
import datetime as dt
import fnmatch
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
FLIGHT_OPS = REPO_ROOT / "flight_ops"
DEFAULT_EXPECTED = FLIGHT_OPS / "lab_config" / "expected_state.yaml"
DEFAULT_OUT_ROOT = FLIGHT_OPS / "snapshots"

CMD_TIMEOUT = 20
SSH_TIMEOUT = 15
FLOAT_TOL = 1e-6

ENV_KEYS = (
    "ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "AMENT_PREFIX_PATH",
    "LIBGL_ALWAYS_SOFTWARE", "IGN_RENDERING_ENGINE", "GZ_VERSION",
    "PYTHONNOUSERSITE", "ROS_LOCALHOST_ONLY",
)


# --------------------------------------------------------------------------- #
# results
# --------------------------------------------------------------------------- #

@dataclass
class Source:
    """One collected artifact, or the reason it could not be collected."""
    name: str
    ok: bool
    artifact: Optional[str] = None       # filename inside artifacts/
    error: Optional[str] = None
    sha256: Optional[str] = None
    parsed: Dict[str, object] = field(default_factory=dict)


@dataclass
class GateResult:
    kind: str                            # "must_equal" | "baseline_compare"
    target: str                          # airframe label
    param: str
    expected: object
    actual: object
    passed: bool

    def line(self) -> str:
        mark = "PASS" if self.passed else "FAIL"
        return (f"[{mark}] {self.kind:16s} {self.target}:{self.param} "
                f"expected={self.expected!r} actual={self.actual!r}")


# --------------------------------------------------------------------------- #
# shell helpers
# --------------------------------------------------------------------------- #

def run(cmd: List[str], timeout: int = CMD_TIMEOUT) -> Tuple[bool, str]:
    """Run a command, never raise. Returns (ok, combined output)."""
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, check=False,
        )
    except FileNotFoundError:
        return False, f"command not found: {cmd[0]}"
    except subprocess.TimeoutExpired:
        return False, f"timed out after {timeout}s: {' '.join(cmd)}"
    except OSError as exc:                                     # pragma: no cover
        return False, f"{type(exc).__name__}: {exc}"
    out = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode == 0, out


def run_shell(script: str, timeout: int = CMD_TIMEOUT) -> Tuple[bool, str]:
    """Run a shell snippet through bash -lc so ROS setup files are sourced."""
    return run(["bash", "-lc", script], timeout=timeout)


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


# --------------------------------------------------------------------------- #
# PX4 parameter parsing
# --------------------------------------------------------------------------- #

# QGC .params export: "<sysid>\t<compid>\t<NAME>\t<VALUE>\t<TYPE>"
_QGC_ROW = re.compile(r"^\s*(\d+)\s+(\d+)\s+([A-Z0-9_]+)\s+(-?[\d.eE+]+)\s+(\d+)\s*$")
# nsh 'param show -a': "x   EKF2_EV_DELAY [263,595] : 50.0000"
_NSH_ROW = re.compile(r"^[\sx+*]*([A-Z0-9_]+)\s*(?:\[[^\]]*\])?\s*:\s*(-?[\d.eE+]+)\s*$")
# plain "NAME VALUE" or "NAME=VALUE"
_PLAIN_ROW = re.compile(r"^\s*([A-Z0-9_]+)\s*[=\s]\s*(-?[\d.eE+]+)\s*$")


def _coerce(text: str) -> object:
    """Return an int when the literal has no fractional part, else a float."""
    try:
        value = float(text)
    except ValueError:
        return text
    if value.is_integer() and "." not in text and "e" not in text.lower():
        return int(value)
    return value


def parse_px4_params(text: str) -> Tuple[Dict[str, object], Dict[str, int]]:
    """Parse a PX4 parameter dump in any of three formats.

    Returns (values, mav_types). mav_types is populated only for QGC exports,
    which are the only format that carries type information.
    """
    values: Dict[str, object] = {}
    types: Dict[str, int] = {}
    for raw in text.splitlines():
        line = raw.rstrip()
        if not line or line.lstrip().startswith("#"):
            continue
        match = _QGC_ROW.match(line)
        if match:
            values[match.group(3)] = _coerce(match.group(4))
            types[match.group(3)] = int(match.group(5))
            continue
        match = _NSH_ROW.match(line)
        if match:
            values[match.group(1)] = _coerce(match.group(2))
            continue
        match = _PLAIN_ROW.match(line)
        if match:
            values[match.group(1)] = _coerce(match.group(2))
    return values, types


def values_equal(left: object, right: object) -> bool:
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return abs(float(left) - float(right)) <= FLOAT_TOL
    return left == right


# --------------------------------------------------------------------------- #
# collectors
# --------------------------------------------------------------------------- #

def _write_artifact(art_dir: Path, filename: str, body: str, name: str,
                    parsed: Optional[Dict[str, object]] = None) -> Source:
    dest = art_dir / filename
    dest.write_text(body, encoding="utf-8")
    return Source(name, True, artifact=filename, sha256=sha256_of(dest),
                  parsed=parsed or {})


def collect_git(art_dir: Path) -> Source:
    ok_head, head = run(["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"])
    ok_branch, branch = run(["git", "-C", str(REPO_ROOT), "rev-parse", "--abbrev-ref", "HEAD"])
    _, status = run(["git", "-C", str(REPO_ROOT), "status", "--short"])
    if not (ok_head and ok_branch):
        return Source("git", False, error=(head.strip() or branch.strip()))
    body = (
        f"HEAD    {head.strip()}\n"
        f"branch  {branch.strip()}\n"
        f"dirty   {'yes' if status.strip() else 'no'}\n\n"
        f"{status}"
    )
    return _write_artifact(art_dir, "git.txt", body, "git",
                           parsed={"head": head.strip(), "branch": branch.strip(),
                                   "dirty": bool(status.strip())})


def collect_env(art_dir: Path) -> Source:
    lines = [f"{key}={os.environ.get(key, '<unset>')}" for key in ENV_KEYS]
    return _write_artifact(art_dir, "env.txt", "\n".join(lines) + "\n", "env")


def collect_apt(art_dir: Path) -> Source:
    ok, out = run_shell(
        "dpkg -l 'ros-humble-*' 2>/dev/null | awk '/^ii/ {print $2, $3}' | sort")
    if not ok or not out.strip():
        return Source("apt", False, error="dpkg unavailable or no ros-humble packages")
    return _write_artifact(art_dir, "apt_ros_packages.txt", out, "apt")


def collect_ros_graph(art_dir: Path) -> Source:
    ok_t, topics = run_shell("ros2 topic list -t 2>&1")
    ok_n, nodes = run_shell("ros2 node list 2>&1")
    if not ok_t and not ok_n:
        return Source("ros_graph", False,
                      error="ros2 CLI unavailable or no ROS graph running")
    body = f"# ros2 topic list -t\n{topics}\n\n# ros2 node list\n{nodes}\n"
    return _write_artifact(art_dir, "ros_graph.txt", body, "ros_graph")


def collect_px4(art_dir: Path, label: str, path: Path) -> Source:
    name = f"px4:{label}"
    if not path.exists():
        return Source(name, False, error=f"no such file: {path}")
    try:
        text = path.read_text(errors="replace")
    except OSError as exc:
        return Source(name, False, error=f"unreadable: {exc}")
    values, types = parse_px4_params(text)
    if not values:
        return Source(name, False, error=f"no parameters parsed from {path}")
    return _write_artifact(art_dir, f"px4_{label}.params", text, name,
                           parsed={"values": values, "mav_types": types})


def collect_jetson(art_dir: Path, host: Dict[str, str]) -> Source:
    label = host.get("name", host.get("addr", "jetson"))
    name = f"jetson:{label}"
    target = f"{host['user']}@{host['addr']}"
    remote = (
        "echo '## uname'; uname -a; "
        "echo; echo '## ip addr'; ip -brief addr; "
        "echo; echo '## failed units'; systemctl --failed --no-pager 2>/dev/null | head -20; "
        "echo; echo '## ros2 topics'; (source /opt/ros/humble/setup.bash 2>/dev/null; "
        "timeout 10 ros2 topic list 2>&1 | head -80)"
    )
    ok, out = run(
        ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new",
         "-o", f"ConnectTimeout={SSH_TIMEOUT}", target, remote],
        timeout=SSH_TIMEOUT + 30,
    )
    if not ok:
        return Source(
            name, False,
            error=(f"ssh {target} failed (set up key auth; passwords are never "
                   f"stored here): {out.strip()[:200]}"))
    return _write_artifact(art_dir, f"jetson_{label}.txt", out, name)


def collect_motive(art_dir: Path, path: Path) -> Source:
    if not path.exists():
        return Source("motive", False, error=f"no such file: {path}")
    dest = art_dir / f"motive_{path.name}"
    try:
        shutil.copy2(path, dest)
    except OSError as exc:
        return Source("motive", False, error=f"copy failed: {exc}")
    return Source("motive", True, artifact=dest.name, sha256=sha256_of(dest))


# --------------------------------------------------------------------------- #
# gates
# --------------------------------------------------------------------------- #

def load_expected(path: Path) -> Dict[str, object]:
    with path.open(encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def load_baseline_params(baseline: Path) -> Dict[str, Dict[str, object]]:
    """Read the px4 parameter sets out of an earlier snapshot's manifest."""
    manifest = baseline / "manifest.json"
    if not manifest.exists():
        return {}
    data = json.loads(manifest.read_text(encoding="utf-8"))
    out: Dict[str, Dict[str, object]] = {}
    for source in data.get("sources", []):
        if source.get("name", "").startswith("px4:") and source.get("ok"):
            label = source["name"].split(":", 1)[1]
            out[label] = source.get("parsed", {}).get("values", {})
    return out


def evaluate_gates(px4_sets: Dict[str, Dict[str, object]],
                   expected: Dict[str, object],
                   baseline: Dict[str, Dict[str, object]]) -> List[GateResult]:
    results: List[GateResult] = []
    gates = expected.get("gates", {}) or {}
    must_equal = gates.get("must_equal", {}) or {}
    patterns = gates.get("baseline_compare", []) or []

    for label, values in px4_sets.items():
        for param, want in must_equal.items():
            got = values.get(param, "<absent>")
            results.append(GateResult("must_equal", label, param, want, got,
                                      values_equal(got, want)))

        prior = baseline.get(label)
        if not prior:
            continue
        watched = {
            key for key in set(values) | set(prior)
            if any(fnmatch.fnmatchcase(key, pat) for pat in patterns)
        }
        for param in sorted(watched):
            was = prior.get(param, "<absent>")
            now = values.get(param, "<absent>")
            if not values_equal(was, now):
                results.append(
                    GateResult("baseline_compare", label, param, was, now, False))
    return results


# --------------------------------------------------------------------------- #
# rollback generation
# --------------------------------------------------------------------------- #

ROLLBACK_PY = """#!/usr/bin/env python3
# GENERATED by flight_ops/snapshot.py -- do not edit.
# Restores the managed parameters to the values captured in snapshot {snap}
# for airframe "{label}", in STRICT REVERSE of the documented apply order.
#
# PX4 does not auto-persist MAVLink PARAM_SET, so this issues
# MAV_CMD_PREFLIGHT_STORAGE (param1=1) at the end. Every write is read back.
#
#   python3 {fname} --conn /dev/ttyACM0 --baud 57600 --yes
#
# If MAVLink is unavailable (MAV_2_CONFIG=0 means it is NOT on Ethernet), use
# the sibling rollback_console_{label}.txt instead: paste it into
# QGroundControl -> Analyze Tools -> MAVLink Console.

import argparse
import struct
import sys

# (name, value) in reverse apply order
RESTORE = {restore!r}

INT_NAMES = {int_names!r}

# Parameters whose MAV_PARAM_TYPE was not recorded, because the snapshot came
# from a capture format that does not carry types. Sending one of these with a
# guessed type risks PX4 rejecting or misinterpreting the write.
UNTYPED = {untyped!r}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", required=True, help="pymavlink connection string")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--yes", action="store_true", help="required; this writes to the FC")
    args = ap.parse_args()
    if not args.yes:
        print("refusing to write without --yes", file=sys.stderr)
        return 2
    if UNTYPED:
        print("REFUSING: no type information for %d parameter(s): %s"
              % (len(UNTYPED), ", ".join(sorted(UNTYPED))), file=sys.stderr)
        print("Use the sibling rollback_console_*.txt in the QGC MAVLink Console "
              "instead, or re-snapshot from a QGC .params export.", file=sys.stderr)
        return 2

    from pymavlink import mavutil

    link = mavutil.mavlink_connection(args.conn, baud=args.baud)
    link.wait_heartbeat(timeout=30)
    print("heartbeat from system %u component %u"
          % (link.target_system, link.target_component))

    failures = 0
    for name, value in RESTORE:
        if name in INT_NAMES:
            # int32 params must be bit-reinterpreted, not cast -- required above 2^24
            wire = struct.unpack("<f", struct.pack("<i", int(value)))[0]
            ptype = mavutil.mavlink.MAV_PARAM_TYPE_INT32
        else:
            wire, ptype = float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        link.mav.param_set_send(link.target_system, link.target_component,
                                name.encode(), wire, ptype)
        echo = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
        if echo is None:
            print("  %-20s NO ECHO" % name)
            failures += 1
            continue
        readback = echo.param_value
        if name in INT_NAMES:
            readback = struct.unpack("<i", struct.pack("<f", readback))[0]
        good = abs(float(readback) - float(value)) < 1e-4
        print("  %-20s -> %s %s" % (name, readback, "ok" if good else "MISMATCH"))
        failures += 0 if good else 1

    link.mav.command_long_send(
        link.target_system, link.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0, 0, 0, 0, 0, 0)
    print("storage command sent; %d failure(s)" % failures)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
"""


def write_rollback(snap_dir: Path, label: str, values: Dict[str, object],
                   mav_types: Dict[str, int], managed: List[str]) -> List[str]:
    """Emit rollback artifacts for one airframe. Returns filenames written."""
    present = [name for name in managed if name in values]
    missing = [name for name in managed if name not in values]
    restore = [(name, values[name]) for name in reversed(present)]
    # MAV_PARAM_TYPE 1..8 are the integer types; 9/10 are REAL32/REAL64.
    int_names = {name for name in present if mav_types.get(name, 9) < 9}
    # Only a QGC .params export carries type information. Without it every
    # parameter is assumed REAL32, which PX4 will reject for an int32 param --
    # so the generated pymavlink script is unsafe and the console path must be
    # used instead.
    untyped = [name for name in present if name not in mav_types]
    written: List[str] = []

    console = snap_dir / f"rollback_console_{label}.txt"
    lines = [
        f"# GENERATED by flight_ops/snapshot.py from snapshot {snap_dir.name}",
        f"# Airframe: {label}",
        "# Paste into QGroundControl -> Analyze Tools -> MAVLink Console.",
        "# STRICT REVERSE of the apply order in lab_config/expected_state.yaml.",
        "#",
        "# The lab QGC is older than PX4 v1.17: it will warn about unknown",
        "# parameters and reject valid enum values while the write succeeds.",
        "# Trust the read-back below, never the popup.",
        "",
    ]
    if untyped:
        lines += [
            "# NOTE: this snapshot came from a capture without type information",
            "#       (not a QGC .params export). This console path is unaffected --",
            "#       PX4 resolves the type itself -- but the sibling rollback",
            f"#       rollback_{label}.py is NOT safe to use. Prefer this file.",
            "",
        ]
    lines += [f"param set {name} {value}" for name, value in restore]
    lines += ["param save", ""]
    lines += [f"param show {name}" for name, _ in restore]
    if missing:
        lines += ["", "# NOT captured in this snapshot, so NOT restored:"]
        lines += [f"#   {name}" for name in missing]
    console.write_text("\n".join(lines) + "\n", encoding="utf-8")
    written.append(console.name)

    script_name = f"rollback_{label}.py"
    (snap_dir / script_name).write_text(
        ROLLBACK_PY.format(snap=snap_dir.name, label=label, fname=script_name,
                           restore=restore, int_names=int_names or set(),
                           untyped=set(untyped)),
        encoding="utf-8",
    )
    written.append(script_name)
    return written


# --------------------------------------------------------------------------- #
# reporting
# --------------------------------------------------------------------------- #

def render_summary(label: str, stamp: str, sources: List[Source],
                   gates: List[GateResult], rollbacks: Dict[str, List[str]]) -> str:
    lines = [f"# Snapshot {stamp} -- {label}", ""]
    lines += ["## Sources", "", "| source | status | artifact / error |", "|---|---|---|"]
    for source in sources:
        detail = source.artifact if source.ok else (source.error or "")
        lines.append(f"| `{source.name}` | {'ok' if source.ok else 'MISSING'} | {detail} |")
    lines += ["", "## Gates", ""]
    if not gates:
        lines.append("No PX4 parameter dump supplied, so no gates were evaluated.")
    else:
        failed = [gate for gate in gates if not gate.passed]
        lines.append(f"{len(gates) - len(failed)} passed, {len(failed)} failed.")
        lines += ["", "```"] + [gate.line() for gate in gates] + ["```"]
    lines += ["", "## Rollback", ""]
    if not rollbacks:
        lines.append("No parameter dump captured, so no rollback could be generated.")
    for airframe, files in rollbacks.items():
        lines.append(f"- **{airframe}**: " + ", ".join(f"`{name}`" for name in files))
    lines += ["",
              "Rollback restores the managed parameters in strict reverse of the apply "
              "order in `lab_config/expected_state.yaml`, to the values captured here.",
              ""]
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #

def parse_px4_arg(spec: str) -> Tuple[str, Path]:
    """Parse 'label:path'. A bare path is labelled by its stem."""
    if ":" in spec and not Path(spec).exists():
        label, _, raw = spec.partition(":")
        return label, Path(raw).expanduser()
    path = Path(spec).expanduser()
    return path.stem, path


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", default="snapshot",
                    help="short tag recorded in the manifest")
    ap.add_argument("--out-root", type=Path, default=DEFAULT_OUT_ROOT)
    ap.add_argument("--expected", type=Path, default=DEFAULT_EXPECTED)
    ap.add_argument("--px4-params", action="append", default=[], metavar="LABEL:PATH",
                    help="PX4 parameter dump (QGC .params export or 'param show -a')")
    ap.add_argument("--motive-project", type=Path, default=None)
    ap.add_argument("--baseline", type=Path, default=None,
                    help="earlier snapshot directory to diff managed parameters against")
    ap.add_argument("--check", action="store_true",
                    help="evaluate gates and exit non-zero if a must_equal gate fails")
    ap.add_argument("--local-only", action="store_true",
                    help="skip ssh and ROS graph collection")
    return ap


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        expected = load_expected(args.expected)
    except (OSError, yaml.YAMLError) as exc:
        print(f"cannot read {args.expected}: {exc}", file=sys.stderr)
        return 2

    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    snap_dir = args.out_root / stamp
    art_dir = snap_dir / "artifacts"
    try:
        art_dir.mkdir(parents=True, exist_ok=False)
    except OSError as exc:
        print(f"cannot create {art_dir}: {exc}", file=sys.stderr)
        return 2

    sources: List[Source] = [collect_git(art_dir), collect_env(art_dir),
                             collect_apt(art_dir)]
    if not args.local_only:
        sources.append(collect_ros_graph(art_dir))

    px4_sets: Dict[str, Dict[str, object]] = {}
    px4_types: Dict[str, Dict[str, int]] = {}
    for spec in args.px4_params:
        label, path = parse_px4_arg(spec)
        source = collect_px4(art_dir, label, path)
        sources.append(source)
        if source.ok:
            px4_sets[label] = source.parsed["values"]          # type: ignore[index]
            px4_types[label] = source.parsed["mav_types"]      # type: ignore[index]

    if not args.local_only:
        for host in expected.get("hosts", []) or []:
            sources.append(collect_jetson(art_dir, host))
    if args.motive_project:
        sources.append(collect_motive(art_dir, args.motive_project))

    baseline = load_baseline_params(args.baseline) if args.baseline else {}
    gates = evaluate_gates(px4_sets, expected, baseline)

    managed = list(expected.get("managed_params", []) or [])
    rollbacks: Dict[str, List[str]] = {}
    for label, values in px4_sets.items():
        rollbacks[label] = write_rollback(snap_dir, label, values,
                                          px4_types.get(label, {}), managed)

    manifest = {
        "label": args.label,
        "captured_utc": stamp,
        "expected_state": str(args.expected),
        "baseline": str(args.baseline) if args.baseline else None,
        "local_only": args.local_only,
        "sources": [vars(source) for source in sources],
        "gates": [vars(gate) for gate in gates],
        "rollback": rollbacks,
    }
    (snap_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, default=str), encoding="utf-8")
    (snap_dir / "SUMMARY.md").write_text(
        render_summary(args.label, stamp, sources, gates, rollbacks), encoding="utf-8")

    hard_failures = [g for g in gates if g.kind == "must_equal" and not g.passed]
    drift = [g for g in gates if g.kind == "baseline_compare" and not g.passed]

    print(f"snapshot: {snap_dir}")
    for source in sources:
        state = "ok  " if source.ok else "MISS"
        detail = source.artifact if source.ok else source.error
        print(f"  [{state}] {source.name:22s} {detail}")
    for gate in gates:
        if not gate.passed:
            print(f"  {gate.line()}")

    if hard_failures:
        print(f"\nREFUSE TO PROCEED: {len(hard_failures)} must_equal gate(s) failed.",
              file=sys.stderr)
        print("The shared hardware is not in the expected state -- another session may "
              "be live. Do not 'fix' it; coordinate first.", file=sys.stderr)
        return 1 if args.check else 0
    if drift:
        print(f"\n{len(drift)} managed parameter(s) drifted from the baseline. "
              "Review before proceeding.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
