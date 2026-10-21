"""Resource guards for an unattended multi-hour campaign.

Checked BETWEEN TRIALS, not just once at startup. A preflight check tells you
the machine was healthy when you pressed go; it says nothing about hour 20,
which is exactly when a campaign runs out of disk or wedges on an orphaned
simulator. Every guard here is cheap enough to run before every trial.

Nothing in the existing harness calls `shutil.disk_usage` anywhere, and nothing
notices an `ign gazebo` that outlived its teardown. Those are the two failures
that quietly ruin a long run: the first fills the disk and every subsequent
trial fails to write its bag; the second leaves a stale simulator holding the
ROS graph so the next trial connects to the wrong world.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

# Bags are 21-33 MB each and a trial also writes logs and CSVs. 5 GB leaves
# room for roughly 150 more trials -- enough to finish any stage, and enough
# warning to stop cleanly rather than mid-write.
MIN_FREE_GB = 5.0

# WSL has ~7.8 GB total here, and Gazebo + three drones + the bridge is the
# bulk of it. Below this there is not enough headroom to start another trial
# without risking the OOM killer taking the simulator mid-flight.
MIN_AVAIL_RAM_MB = 800


@dataclass
class GuardResult:
    ok: bool
    name: str
    detail: str
    fatal: bool = False      # fatal => stop the stage; else retry/continue

    def __str__(self) -> str:
        return f"[{'ok' if self.ok else 'FAIL'}] {self.name}: {self.detail}"


def check_disk(path: Path, min_free_gb: float = MIN_FREE_GB) -> GuardResult:
    try:
        usage = shutil.disk_usage(str(path))
    except OSError as e:
        return GuardResult(False, f"disk({path})", f"unreadable: {e}", fatal=True)
    free_gb = usage.free / 1e9
    ok = free_gb >= min_free_gb
    return GuardResult(
        ok, f"disk({path})",
        f"{free_gb:.1f} GB free (need {min_free_gb:.0f})",
        fatal=not ok,
    )


def check_ram(min_avail_mb: float = MIN_AVAIL_RAM_MB) -> GuardResult:
    """Available (not free) memory -- the kernel's own estimate of what a new
    process could actually get, which is the number that matters here."""
    try:
        fields = {}
        for line in Path("/proc/meminfo").read_text().splitlines():
            k, _, rest = line.partition(":")
            fields[k.strip()] = rest.strip()
        avail_mb = float(fields["MemAvailable"].split()[0]) / 1024.0
    except Exception as e:                                   # noqa: BLE001
        return GuardResult(True, "ram", f"unreadable ({e}); not blocking")
    ok = avail_mb >= min_avail_mb
    return GuardResult(ok, "ram", f"{avail_mb:.0f} MB available "
                                  f"(need {min_avail_mb:.0f})")


SIM_ORPHAN_PATTERNS = [
    "ign gazebo", "ign-gazebo", "gz sim", "ros_gz_bridge",
    "as2_platform_gazebo", "deckga_execute",
]


def find_sim_orphans() -> List[str]:
    """Simulator processes that outlived a teardown.

    Deliberately does NOT match on this module's own name or path: the campaign
    driver is itself a python process whose command line mentions the repo, and
    a pattern broad enough to catch it would have the guard kill the campaign it
    is guarding. (A probe script of mine did exactly that with
    `pkill -f rtf_probe` matching its own filename.)
    """
    out: List[str] = []
    me = os.getpid()
    try:
        proc = subprocess.run(["ps", "-eo", "pid,cmd"], capture_output=True,
                              text=True, timeout=10)
    except Exception:                                        # noqa: BLE001
        return out
    for line in proc.stdout.splitlines()[1:]:
        line = line.strip()
        if not line:
            continue
        pid_s, _, cmd = line.partition(" ")
        try:
            pid = int(pid_s)
        except ValueError:
            continue
        if pid == me:
            continue
        if "campaign_driver" in cmd or "guards.py" in cmd:
            continue
        if any(p in cmd for p in SIM_ORPHAN_PATTERNS):
            out.append(f"{pid} {cmd[:90]}")
    return out


def check_no_sim_orphans() -> GuardResult:
    orphans = find_sim_orphans()
    if not orphans:
        return GuardResult(True, "orphans", "none")
    return GuardResult(False, "orphans",
                       f"{len(orphans)} simulator process(es) survived teardown: "
                       + "; ".join(orphans[:3]))


def kill_sim_orphans(grace_s: float = 3.0) -> int:
    """SIGKILL anything left over. Returns how many were signalled."""
    orphans = find_sim_orphans()
    killed = 0
    for entry in orphans:
        pid = int(entry.split()[0])
        try:
            os.kill(pid, 9)
            killed += 1
        except (ProcessLookupError, PermissionError):
            pass
    if killed:
        time.sleep(grace_s)
    return killed


def preflight(results_root: Path, home: Optional[Path] = None,
              *, need_sim: bool = False) -> List[GuardResult]:
    """Everything that must hold before a stage starts."""
    home = home or Path.home()
    checks = [check_disk(results_root), check_disk(home), check_ram()]

    for mod in ("numpy", "matplotlib", "scipy", "planner.timing",
                "planner.cost_matrix", "planner.kmedoids", "planner.balance"):
        try:
            __import__(mod)
            checks.append(GuardResult(True, f"import {mod}", "ok"))
        except ImportError as e:
            # scipy is optional but its absence silently degrades the stats:
            # aggregate._ci95 falls back to a hardcoded 1.96 and
            # _paired_pvalue returns NaN, with nothing recording that it did.
            fatal = mod != "scipy"
            checks.append(GuardResult(False, f"import {mod}", str(e), fatal=fatal))

    if need_sim:
        for tool in ("ros2", "ign"):
            found = shutil.which(tool)
            checks.append(GuardResult(bool(found), f"which {tool}",
                                      found or "NOT FOUND", fatal=not found))
        disp = os.environ.get("DISPLAY", "")
        checks.append(GuardResult(
            bool(disp), "DISPLAY", disp or "unset -- Gazebo will fail to render",
            fatal=not disp))
        checks.append(check_no_sim_orphans())

    return checks


def summarise(checks: List[GuardResult]) -> tuple:
    """(all_ok, any_fatal, text)."""
    lines = [str(c) for c in checks]
    all_ok = all(c.ok for c in checks)
    any_fatal = any((not c.ok) and c.fatal for c in checks)
    return all_ok, any_fatal, "\n".join(lines)
