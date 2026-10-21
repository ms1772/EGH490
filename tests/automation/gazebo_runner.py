"""Aerostack2 + Gazebo lifecycle for the test harness.

Two launch modes:
  - full_gui: spawns launch_as2.bash + launch_ground_station.bash via a controlling
              pty (so the trailing `tmux attach-session` doesn't fail), plus
              rviz_paths_node + rviz_obstacles_node. Pauses for manual Enter
              before the executor. Used by the Stage 4a smoke trial.
  - headless: spawns launch_as2.bash with stdin=DEVNULL (no pty), no ground
              station, no RViz nodes. Used by Stage 4b's 8-trial subset.

Both modes share readiness, executor (line-watcher completion), teardown.

WSL-only.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
import time

# Unix-only stdlib (this whole module only runs on WSL/Linux). Guard the imports
# so the module can at least be imported on Windows for sanity / linting.
try:
    import pty
    import select
    import signal
    _UNIX_OK = True
except ImportError:
    pty = None
    select = None
    signal = None
    _UNIX_OK = False


def _require_unix():
    if not _UNIX_OK:
        raise RuntimeError(
            "gazebo_runner requires a Unix host (WSL2 Ubuntu 22.04). "
            "Run from your WSL shell, not PowerShell."
        )
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
PROJECT_GAZEBO_DIR = REPO_ROOT / "aerostack_examples" / "02_examples_gazebo_project" / "project_gazebo"

READINESS_CLOCK_TIMEOUT_S = 90
READINESS_POSE_TIMEOUT_S = 60
POST_READY_SETTLE_S = 10
# Worst-case mission: n=100 wp at speed=1.0 m/s. Empirically the makespan can
# hit ~18 min for n=100 (drones not perfectly parallel). 30 min gives safe
# margin. Overridable from sweep_gazebo CLI.
# 3600, not 1800: the measured EASIEST condition already takes 1291 s, and
# the campaign's worst (n=100, obs=100, 3 UAVs) is several times that. A
# backstop that fires on a healthy trial destroys data that was about to
# land, which is worse than waiting. The per-trial wall-clock guard in the
# campaign driver is the real runaway protection.
EXECUTOR_BACKSTOP_S = 3600
EXECUTOR_SIGINT_GRACE_S = 10
EXECUTOR_SIGTERM_GRACE_S = 5
TEARDOWN_SETTLE_S = 3
# Recorded IN ADDITION to the pose topics when they exist. The point is that
# anything in the bag can be re-rendered offline afterwards, which is what
# "never re-run a simulation for a picture" actually requires: /tf + /tf_static
# allow frame-correct replay, and the two /deckga marker topics carry the
# planned paths and obstacle boxes that an RViz figure needs. Bags are 21-33 MB
# today and these add little.
#
# Filtered against `ros2 topic list` before use -- in headless mode the marker
# nodes are not launched, so naming them unconditionally would ask rosbag2 to
# record topics that have no publisher.
BAG_EXTRA_TOPICS = [
    "/tf",
    "/tf_static",
    "/deckga/markers",
    "/deckga/obstacles",
]

DEFAULT_POSE_TOPICS = [
    "/drone0/self_localization/pose",
    "/drone1/self_localization/pose",
    "/drone2/self_localization/pose",
]

# --- real-time factor -------------------------------------------------------
# The world template pins the sim to real time:
#   /opt/ros/humble/share/as2_gazebo_assets/worlds/empty.sdf.jinja:6-7
#     <max_step_size>0.004</max_step_size>
#     <real_time_factor>1.0</real_time_factor>
# That is a configured throttle, not a CPU limit -- an empty world with three
# quadrotors does not come close to saturating 20 cores.
#
# It can be lifted at RUNTIME, with no file edited anywhere: the world loads
# ignition-gazebo-user-commands-system, which serves /world/<name>/set_physics.
# Measured on a bare world: RTF 1.0 -> 10.0 immediately, service returns
# data: true.
#
# NOTE real_time_factor: 0.0 is NOT "unlimited" -- it is silently ignored and
# the sim stays at 1.0. Always pass an explicit positive value.
#
# Whether the FULL stack tolerates it is a different question and must be
# measured, not assumed: config/config.yaml:11,25 run the platform and
# controller at cmd_freq 50 Hz against use_sim_time, so at RTF N they must
# sustain 50*N Hz of wall-clock work. If they cannot, tracking degrades and the
# results change. See the Stage 0 calibration.
GZ_WORLD_NAME = "empty"
DEFAULT_RTF = 1.0
RTF_SETTLE_S = 2.0
PHYSICS_MAX_STEP_SIZE = 0.004


@dataclass
class GazeboLifecycle:
    """Tracks every child process we spawn so teardown can kill them all."""
    pty_procs: List[subprocess.Popen] = field(default_factory=list)
    bg_procs: List[subprocess.Popen] = field(default_factory=list)
    pose_topics: List[str] = field(default_factory=list)
    bag_proc: Optional[subprocess.Popen] = None
    bag_path: Optional[Path] = None


def _spawn_with_pty(cmd: List[str], cwd: Path, log_path: Path) -> subprocess.Popen:
    """Spawn a command with a controlling pty so blocking calls like
    `tmux attach-session` don't fail with 'no current client'. Stdout/stderr
    are tee'd to log_path via a child process that reads the pty master fd."""
    _require_unix()
    master_fd, slave_fd = pty.openpty()
    proc = subprocess.Popen(
        cmd, cwd=str(cwd),
        stdin=slave_fd, stdout=slave_fd, stderr=slave_fd,
        start_new_session=True, close_fds=True,
    )
    os.close(slave_fd)
    # Background reader writes pty output to logfile.
    pid = os.fork()
    if pid == 0:  # child
        try:
            with open(log_path, "ab") as logf:
                while True:
                    rlist, _, _ = select.select([master_fd], [], [], 1.0)
                    if not rlist:
                        if proc.poll() is not None:
                            break
                        continue
                    try:
                        data = os.read(master_fd, 4096)
                    except OSError:
                        break
                    if not data:
                        break
                    logf.write(data)
                    logf.flush()
        finally:
            os._exit(0)
    return proc


def _spawn_bg(cmd: List[str], cwd: Path, log_path: Path) -> subprocess.Popen:
    """Spawn a command with stdin=DEVNULL (no pty), stdout/stderr -> log_path."""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    logf = log_path.open("ab")
    return subprocess.Popen(
        cmd, cwd=str(cwd),
        stdin=subprocess.DEVNULL, stdout=logf, stderr=subprocess.STDOUT,
        start_new_session=True, close_fds=True,
    )


def _wait_topic_once(topic: str, timeout_s: float, qos: str = "best_effort") -> bool:
    """Return True if `ros2 topic echo --once <topic>` produces output within timeout."""
    try:
        proc = subprocess.run(
            ["ros2", "topic", "echo", "--once", topic, "--qos-reliability", qos],
            capture_output=True, text=True, timeout=timeout_s,
        )
        return proc.returncode == 0 and bool(proc.stdout.strip())
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def wait_for_clock(timeout_s: float = READINESS_CLOCK_TIMEOUT_S, poll_s: float = 2.0) -> bool:
    """Poll /clock until one message arrives or timeout. Returns True on success."""
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout_s:
        if _wait_topic_once("/clock", timeout_s=poll_s + 1):
            return True
        time.sleep(poll_s)
    return False


def set_real_time_factor(rtf: float, world: str = GZ_WORLD_NAME,
                         timeout_ms: int = 3000) -> bool:
    """Ask Gazebo to run at `rtf` x real time. Returns True if it accepted.

    Must be called AFTER the world is up (wait_for_clock) -- the service does
    not exist before then.
    """
    _require_unix()
    if rtf is None or float(rtf) <= 0.0:
        raise ValueError(
            f"real_time_factor must be a positive number, got {rtf!r}. "
            f"0.0 does not mean 'unlimited' -- Gazebo ignores it and stays at 1.0."
        )
    req = f"real_time_factor: {float(rtf)}, max_step_size: {PHYSICS_MAX_STEP_SIZE}"
    try:
        proc = subprocess.run(
            ["ign", "service", "-s", f"/world/{world}/set_physics",
             "--reqtype", "ignition.msgs.Physics",
             "--reptype", "ignition.msgs.Boolean",
             "--timeout", str(timeout_ms), "--req", req],
            capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 5,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False
    return "data: true" in proc.stdout


def measure_real_time_factor(world: str = GZ_WORLD_NAME, samples: int = 4,
                             gap_s: float = 1.0) -> Optional[float]:
    """Median achieved RTF from /world/<world>/stats, or None if unavailable.

    Median rather than mean: the first sample after a physics change is often
    a transient, and one outlier should not decide whether an RTF is accepted.
    """
    _require_unix()
    vals: List[float] = []
    for _ in range(samples):
        try:
            proc = subprocess.run(
                ["ign", "topic", "-e", "-t", f"/world/{world}/stats", "-n", "1"],
                capture_output=True, text=True, timeout=8,
            )
        except (subprocess.TimeoutExpired, FileNotFoundError):
            break
        m = re.search(r"real_time_factor:\s*([0-9.]+)", proc.stdout)
        if m:
            vals.append(float(m.group(1)))
        time.sleep(gap_s)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def discover_pose_topics(num_uavs: int = 3) -> List[str]:
    """List ros2 topics, return any '*/self_localization/pose' for the expected drones.
    Falls back to DEFAULT_POSE_TOPICS if grep returns nothing."""
    try:
        proc = subprocess.run(
            ["ros2", "topic", "list"], capture_output=True, text=True, timeout=15,
        )
        topics = proc.stdout.splitlines()
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return list(DEFAULT_POSE_TOPICS[:num_uavs])
    found = [t.strip() for t in topics if "self_localization/pose" in t]
    if not found:
        return list(DEFAULT_POSE_TOPICS[:num_uavs])
    found.sort()
    return found[:num_uavs] if len(found) >= num_uavs else found


def wait_for_pose(pose_topic: str, timeout_s: float = READINESS_POSE_TIMEOUT_S,
                  poll_s: float = 2.0) -> bool:
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout_s:
        if _wait_topic_once(pose_topic, timeout_s=poll_s + 1):
            return True
        time.sleep(poll_s)
    return False


def _bag_home_dir(run_ts: str, trial_id: str) -> Path:
    """Bag is staged in $HOME to avoid DrvFs slowness on /mnt/c."""
    home = Path(os.environ.get("HOME", "/tmp"))
    p = home / "deckga_bags" / run_ts / trial_id
    p.parent.mkdir(parents=True, exist_ok=True)
    return p / "poses.bag"


def _filter_existing_topics(candidates: List[str], timeout_s: float = 10.0) -> List[str]:
    """Keep only candidates that currently have a publisher.

    Asking rosbag2 to record a topic nobody publishes is not fatal, but it is
    not verified to be harmless either, and the bag recorder failing would void
    a trial's entire dataset. Checking costs one `ros2 topic list`.
    """
    try:
        proc = subprocess.run(["ros2", "topic", "list"], capture_output=True,
                              text=True, timeout=timeout_s)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []
    live = {t.strip() for t in proc.stdout.splitlines() if t.strip()}
    return [t for t in candidates if t in live]


def start_bag_recorder(topics: List[str], run_ts: str, trial_id: str,
                       log_path: Path) -> tuple[subprocess.Popen, Path]:
    bag_dir = _bag_home_dir(run_ts, trial_id)
    if bag_dir.exists():
        shutil.rmtree(bag_dir, ignore_errors=True)
    bag_dir.parent.mkdir(parents=True, exist_ok=True)
    extra = _filter_existing_topics(BAG_EXTRA_TOPICS)
    all_topics = list(topics) + [t for t in extra if t not in topics]
    cmd = ["ros2", "bag", "record", "-o", str(bag_dir), *all_topics]
    proc = _spawn_bg(cmd, cwd=Path.cwd(), log_path=log_path)
    return proc, bag_dir


def stop_bag_recorder(proc: subprocess.Popen, grace_s: float = 10.0) -> None:
    """SIGINT (not SIGTERM/SIGKILL) so rosbag2 flushes its writer cleanly."""
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except (ProcessLookupError, PermissionError):
        return
    try:
        proc.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass


def launch_full_gui(trial_pkl: Path, log_dir: Path) -> GazeboLifecycle:
    """Full GUI: launch_as2 + ground_station via pty, RViz nodes as bg procs."""
    log_dir.mkdir(parents=True, exist_ok=True)
    lc = GazeboLifecycle()

    # 1. Aerostack2 sim
    p = _spawn_with_pty(
        ["bash", "launch_as2.bash", "-m"],
        cwd=PROJECT_GAZEBO_DIR,
        log_path=log_dir / "launch_as2.log",
    )
    lc.pty_procs.append(p)

    # 2. Ground station / RViz config
    time.sleep(2.0)  # give the sim a head-start before the GS attaches
    p = _spawn_with_pty(
        ["bash", "launch_ground_station.bash", "-m", "-t", "-v"],
        cwd=PROJECT_GAZEBO_DIR,
        log_path=log_dir / "launch_ground_station.log",
    )
    lc.pty_procs.append(p)

    # 3. Path + obstacle visualisers (own ROS2 nodes -- no pty needed)
    p = _spawn_bg(
        [sys.executable, str(REPO_ROOT / "deckga_ros2" / "rviz_paths_node.py"),
         "--deckga_pkl", str(trial_pkl),
         "--path_key", "quicknav_paths"],
        cwd=REPO_ROOT,
        log_path=log_dir / "rviz_paths_node.log",
    )
    lc.bg_procs.append(p)
    p = _spawn_bg(
        [sys.executable, str(REPO_ROOT / "deckga_ros2" / "rviz_obstacles_node.py"),
         "--deckga_pkl", str(trial_pkl)],
        cwd=REPO_ROOT,
        log_path=log_dir / "rviz_obstacles_node.log",
    )
    lc.bg_procs.append(p)

    return lc


def launch_headless(log_dir: Path) -> GazeboLifecycle:
    """Headless: launch_as2 ONLY (no GS, no RViz) but still via pty.

    tmuxinator inside launch_as2.bash needs a controlling terminal — without a
    pty it errors with 'open terminal failed: not a terminal' and no sessions
    start. So 'headless' means skipping the GS + RViz nodes, not skipping the
    pty.
    """
    _require_unix()
    log_dir.mkdir(parents=True, exist_ok=True)
    lc = GazeboLifecycle()
    p = _spawn_with_pty(
        ["bash", "launch_as2.bash", "-m"],
        cwd=PROJECT_GAZEBO_DIR,
        log_path=log_dir / "launch_as2.log",
    )
    lc.pty_procs.append(p)
    return lc


def write_discovered_topics(run_dir: Path, topics: List[str], time_to_ready_s: float) -> Path:
    """Stage 4a writes this; Stage 4b reads it to size its timeouts and topic names."""
    gz_dir = run_dir / "gazebo"
    gz_dir.mkdir(parents=True, exist_ok=True)
    path = gz_dir / "discovered_topics.json"
    path.write_text(json.dumps({
        "pose_topics": topics,
        "time_to_ready_s": time_to_ready_s,
    }, indent=2))
    return path


def read_discovered_topics(run_dir: Path) -> Optional[dict]:
    path = run_dir / "gazebo" / "discovered_topics.json"
    if not path.exists():
        return None
    return json.loads(path.read_text())


def run_executor(deckga_pkl: Path, log_dir: Path, trial_id: str, num_uavs: int,
                 path_key: str = "quicknav_paths", speed: float = 1.0,
                 takeoff_z: float = 0.5,
                 backstop_s: float = EXECUTOR_BACKSTOP_S) -> tuple[int, str]:
    """Run deckga_execute.py with a line-watcher that SIGINTs on the completion
    marker. Returns (exit_code, status) where status in
    {'ok', 'executor_timeout', 'executor_failed'}.

    backstop_s overrides EXECUTOR_BACKSTOP_S for this call.
    """
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / "executor.log"
    completion_marker = f"[LOG] Wrote UAV{num_uavs - 1} segments CSV"
    # CRITICAL: `-u` forces unbuffered stdout. Without it Python block-buffers
    # the completion-marker prints, the line-watcher never sees them, and the
    # backstop fires at 1200s on a mission that actually finished at 1060s.
    cmd = [
        sys.executable, "-u", str(REPO_ROOT / "deckga_ros2" / "deckga_execute.py"),
        "--deckga_pkl", str(deckga_pkl),
        "--path_key", path_key,
        "--log_dir", str(log_dir),
        "--run_tag", trial_id,
        "--num_uavs", str(num_uavs),
        "--speed", str(speed),
        "--takeoff_z", str(takeoff_z),
        "--ensure_reach", "--first_wp_wait",
    ]
    with log_path.open("ab") as logf:
        logf.write(f"$ {' '.join(cmd)}\n".encode())
    proc = subprocess.Popen(
        cmd, cwd=REPO_ROOT,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        start_new_session=True, close_fds=True,
        bufsize=1, text=True,
    )

    t0 = time.perf_counter()
    completion_seen = False
    failed_early = False
    try:
        with log_path.open("a", encoding="utf-8") as logf:
            while True:
                if proc.stdout is None:
                    break
                line = proc.stdout.readline()
                if line:
                    logf.write(line)
                    logf.flush()
                    if completion_marker in line:
                        completion_seen = True
                        break
                else:
                    rc = proc.poll()
                    if rc is not None:
                        if not completion_seen:
                            failed_early = (rc != 0)
                        break
                if time.perf_counter() - t0 > backstop_s:
                    break
    except Exception as e:
        with log_path.open("a", encoding="utf-8") as logf:
            logf.write(f"\n[harness] line-watcher exception: {e}\n")

    # If still alive, SIGINT then escalate
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass
        try:
            proc.wait(timeout=EXECUTOR_SIGINT_GRACE_S)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=EXECUTOR_SIGTERM_GRACE_S)
            except (subprocess.TimeoutExpired, ProcessLookupError, PermissionError):
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass

    # Drain remaining stdout
    try:
        rest = proc.stdout.read() if proc.stdout else ""
        if rest:
            with log_path.open("a", encoding="utf-8") as logf:
                logf.write(rest)
    except Exception:
        pass

    elapsed = time.perf_counter() - t0
    rc = proc.returncode if proc.returncode is not None else -1
    if completion_seen:
        return rc, "ok"
    if failed_early:
        return rc, "executor_failed"
    if elapsed > backstop_s:
        return rc, "executor_timeout"
    return rc, "executor_failed"


def _pkill_excluding_self(pattern: str, match_full_cmd: bool = True,
                           log_path: Optional[Path] = None) -> List[int]:
    """Send SIGKILL to processes matching `pattern`, EXCLUDING our own pid and
    our process group.

    `stop.bash` uses `pkill -9 -f 'gz'` and `pkill -9 -f 'gazebo'` which kill
    sweep_gazebo.py itself (its cmdline contains "gazebo"). We replicate
    stop.bash's intent here without the suicide.
    """
    if not _UNIX_OK:
        return []
    my_pid = os.getpid()
    my_pgid = os.getpgrp()
    pgrep_args = ["pgrep", "-f", pattern] if match_full_cmd else ["pgrep", "-x", pattern]
    try:
        proc = subprocess.run(pgrep_args, capture_output=True, text=True, timeout=5)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []
    killed = []
    for line in proc.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            pid = int(line)
        except ValueError:
            continue
        if pid == my_pid:
            continue
        try:
            # Skip processes in our own process group (parents and siblings)
            if os.getpgid(pid) == my_pgid:
                continue
        except (ProcessLookupError, PermissionError):
            continue
        try:
            os.kill(pid, signal.SIGKILL)
            killed.append(pid)
        except (ProcessLookupError, PermissionError):
            pass
    if log_path is not None and killed:
        with log_path.open("a", encoding="utf-8") as logf:
            logf.write(f"[teardown] pattern={pattern!r} killed pids={killed}\n")
    return killed


def teardown(lifecycle: GazeboLifecycle, log_dir: Path) -> dict:
    """Scoped teardown — equivalent to stop.bash but does NOT suicide on
    'pkill -f gazebo' by excluding our own pid/process group.

    Steps mirror stop.bash:
      1. tmux kill known session names (drone0, drone1, drone2, ground_station)
      2. SIGKILL ign-gazebo-* / gz / ruby (the simulator processes)
      3. SIGKILL ros_gz_bridge, rviz2, our python visualisers, deckga_execute
      4. Verify clean via pgrep
    """
    _require_unix()
    report = {"orphans_killed": []}
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / "teardown.log"

    # 1. Known tmux sessions
    for ns in ("drone0", "drone1", "drone2", "ground_station"):
        try:
            subprocess.run(["tmux", "kill-session", "-t", ns],
                           capture_output=True, timeout=5)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass

    # 2. Simulator processes (Ignition Fortress)
    sim_patterns = [
        "ign-gazebo",          # ign-gazebo, ign-gazebo-server
        "ign gazebo",          # `ign gazebo` CLI invocation
        "gz sim",              # newer gz CLI
        "/ruby ",              # ruby invoked for sdformat
        "ros_gz_bridge",       # ros2 <-> gz bridge
        "rviz2",               # standalone RViz
        "rviz_paths_node",     # our path visualiser
        "rviz_obstacles_node", # our obstacle visualiser
        "deckga_execute",      # our executor (running as subprocess)
    ]
    all_killed = []
    for pat in sim_patterns:
        killed = _pkill_excluding_self(pat, match_full_cmd=True, log_path=log_path)
        all_killed.extend(killed)
    report["orphans_killed"] = all_killed

    # 3. Kill any processes we spawned that are still alive via group
    for p in lifecycle.pty_procs + lifecycle.bg_procs:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    # 4. Final post-check (informational only; report doesn't fail on residue)
    try:
        pg = subprocess.run(
            ["pgrep", "-fa", "ign-gazebo|ros_gz_bridge|deckga_execute"],
            capture_output=True, text=True, timeout=5,
        )
        if pg.stdout.strip():
            with log_path.open("a", encoding="utf-8") as logf:
                logf.write(f"\n[teardown] residue after kill round:\n{pg.stdout}\n")
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    time.sleep(TEARDOWN_SETTLE_S)
    return report


def copy_bag(src: Path, dst: Path) -> bool:
    """Copy bag dir from $HOME staging area to results/. Returns True on success."""
    if not src.exists():
        return False
    try:
        if dst.exists():
            shutil.rmtree(dst)
        shutil.copytree(src, dst)
        return True
    except Exception as e:
        print(f"[gazebo_runner] bag copy failed: {e}", file=sys.stderr)
        return False
