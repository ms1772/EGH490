#!/usr/bin/env python3
"""Run every flight_ops self-test in one command.

Each safety-relevant tool in flight_ops carries a `--self-test` mode that
exercises its pure logic with no ROS graph, no hardware and no lab. This
runner finds them and runs them all, so "is the ground-station software still
good?" is one command rather than a habit.

Discovery is dynamic: any .py under flight_ops/ whose source mentions
`--self-test` is picked up. Adding a new tool with a self-test mode requires
no change here.

Usage:
    python3 flight_ops/run_selftests.py
    python3 flight_ops/run_selftests.py --verbose      # show each tool's output
    python3 flight_ops/run_selftests.py --only volume_guard,fake_mocap

Exit codes:
    0  every discovered self-test passed
    1  at least one failed
    2  nothing was discovered (treated as a failure -- a runner that finds
       nothing must not report success)
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

FLIGHT_OPS = Path(__file__).resolve().parent
REPO_ROOT = FLIGHT_OPS.parent
SELF_TEST_FLAG = "--self-test"
PER_TOOL_TIMEOUT_S = 300

# Count lines like "  [PASS] something" / "  [FAIL] something" so the summary
# reports assertions, not just tools.
_RESULT_LINE = re.compile(r"\[(PASS|FAIL)\]")


@dataclass
class ToolResult:
    name: str
    path: Path
    passed: bool
    returncode: int
    assertions_pass: int
    assertions_fail: int
    duration_s: float
    output: str
    error: Optional[str] = None


def discover(root: Path) -> List[Path]:
    """Every .py under flight_ops that advertises a --self-test mode."""
    found = []
    for path in sorted(root.rglob("*.py")):
        if path.name == Path(__file__).name:
            continue
        try:
            if SELF_TEST_FLAG in path.read_text(encoding="utf-8", errors="ignore"):
                found.append(path)
        except OSError:
            continue
    return found


def run_one(path: Path) -> ToolResult:
    name = path.stem
    started = time.monotonic()
    try:
        proc = subprocess.run(
            [sys.executable, str(path), SELF_TEST_FLAG],
            capture_output=True, text=True, timeout=PER_TOOL_TIMEOUT_S,
            cwd=str(REPO_ROOT),
        )
    except subprocess.TimeoutExpired:
        return ToolResult(name, path, False, -1, 0, 0,
                          time.monotonic() - started, "",
                          f"timed out after {PER_TOOL_TIMEOUT_S}s")
    except OSError as exc:
        return ToolResult(name, path, False, -1, 0, 0,
                          time.monotonic() - started, "", f"{type(exc).__name__}: {exc}")

    output = (proc.stdout or "") + (proc.stderr or "")
    hits = _RESULT_LINE.findall(output)
    return ToolResult(
        name=name,
        path=path,
        passed=proc.returncode == 0,
        returncode=proc.returncode,
        assertions_pass=hits.count("PASS"),
        assertions_fail=hits.count("FAIL"),
        duration_s=time.monotonic() - started,
        output=output,
    )


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--verbose", action="store_true",
                    help="print each tool's full output, not just the summary")
    ap.add_argument("--only", default="",
                    help="comma-separated tool stems to run (default: all discovered)")
    args = ap.parse_args(argv)

    tools = discover(FLIGHT_OPS)
    if args.only:
        wanted = {name.strip() for name in args.only.split(",") if name.strip()}
        tools = [path for path in tools if path.stem in wanted]

    if not tools:
        print("no self-tests discovered under flight_ops/", file=sys.stderr)
        return 2

    print(f"running {len(tools)} self-test suite(s)\n")
    results = []
    for path in tools:
        result = run_one(path)
        results.append(result)
        if args.verbose:
            print(f"----- {result.name} -----")
            print(result.output)
        mark = "PASS" if result.passed else "FAIL"
        detail = result.error or f"{result.assertions_pass} assertions"
        if result.assertions_fail:
            detail += f", {result.assertions_fail} FAILED"
        print(f"  [{mark}] {result.name:24s} {detail:34s} {result.duration_s:5.1f}s")

    failed = [r for r in results if not r.passed]
    total_pass = sum(r.assertions_pass for r in results)
    total_fail = sum(r.assertions_fail for r in results)

    print()
    print(f"{len(results) - len(failed)}/{len(results)} suites passed; "
          f"{total_pass} assertions passed, {total_fail} failed")

    if failed:
        print("\nfailing suites:", file=sys.stderr)
        for result in failed:
            print(f"  {result.path.relative_to(REPO_ROOT)} "
                  f"(exit {result.returncode})", file=sys.stderr)
            if not args.verbose:
                tail = [line for line in result.output.splitlines() if "FAIL" in line]
                for line in tail[-10:]:
                    print(f"      {line}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
