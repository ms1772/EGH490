"""Visual capture during a live Gazebo trial.

Everything here must run INSIDE a trial's window. `gazebo_runner.teardown`
SIGKILLs `ign-gazebo`, `gz sim`, `rviz2` and the marker nodes between trials, so
a capture fired afterwards finds nothing to photograph.

NAMING IS A SAFETY CONSTRAINT, not a style choice. Teardown's kill list is
matched with `pgrep -f` against the FULL command line
(`gazebo_runner.py:464-479`): `ign-gazebo`, `ign gazebo`, `gz sim`, `/ruby `,
`ros_gz_bridge`, `rviz2`, `rviz_paths_node`, `rviz_obstacles_node`,
`deckga_execute`. Any helper whose argv contains one of those gets killed along
with the simulator. This module is called `capture.py` and shells out to
`ign service`, whose argv is `ign service -s /gui/screenshot ...` -- note it
does NOT contain the substring "ign gazebo", so it survives. Do not rename this
to anything matching that list, and do not add those words to its arguments.

(Learned concretely: a probe script named rtf_probe.sh killed itself with
`pkill -f "rtf_probe"`.)

WHY THE SCREENSHOT SERVICE IS FREE
----------------------------------
Gazebo's GUI is running during EVERY trial, including "headless" mode --
`launch_headless` only skips the ground station and the two RViz marker nodes,
and still runs the same `launch_as2.bash -m`. The `Screenshot` plugin is
already in both gui.config files, advertising `/gui/screenshot`. So capture
needs a trigger, not any new rendering.
"""
from __future__ import annotations

import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

SCREENSHOT_SERVICE = "/gui/screenshot"
SERVICE_TIMEOUT_MS = 4000


def screenshot(out_path: Path, timeout_ms: int = SERVICE_TIMEOUT_MS) -> bool:
    """Ask the running Gazebo GUI to save a PNG at `out_path`. True on success.

    THE `data:` FIELD IS A DIRECTORY, NOT A FILE PATH. Measured against a live
    Gazebo 6.17.1:

        data: "/tmp/shotdir"     -> /tmp/shotdir/2026-10-11T17:50:30.275.png
        data: "/tmp/shot_b.png"  -> nothing written
        data: ""                 -> nothing written

    Every one of those returned `data: true`. The reply means "request
    accepted", not "image saved", so passing a filename fails completely
    silently -- which is why this asks for a directory, then finds the file
    Gazebo actually created and renames it to what the caller wanted.
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if out_path.exists():
        out_path.unlink()

    # Its own directory, so the newest-PNG pick below cannot grab a frame some
    # other capture wrote a moment earlier.
    staging = out_path.parent / f".shot_{out_path.stem}"
    staging.mkdir(parents=True, exist_ok=True)
    for stale in staging.glob("*.png"):
        stale.unlink()

    try:
        subprocess.run(
            ["ign", "service", "-s", SCREENSHOT_SERVICE,
             "--reqtype", "ignition.msgs.StringMsg",
             "--reptype", "ignition.msgs.Boolean",
             "--timeout", str(timeout_ms),
             "--req", f'data: "{staging}"'],
            capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 5,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False

    # The GUI writes asynchronously, so poll rather than assume.
    for _ in range(24):
        pngs = sorted(staging.glob("*.png"))
        if pngs and pngs[-1].stat().st_size > 0:
            pngs[-1].replace(out_path)
            for leftover in staging.glob("*.png"):
                leftover.unlink()
            try:
                staging.rmdir()
            except OSError:
                pass
            return True
        time.sleep(0.25)
    return False


def service_available(timeout_s: float = 5.0) -> bool:
    try:
        proc = subprocess.run(["ign", "service", "-l"], capture_output=True,
                              text=True, timeout=timeout_s)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False
    return SCREENSHOT_SERVICE in proc.stdout


@dataclass
class FrameSeries:
    """Periodic screenshots, for assembling a video afterwards.

    Used instead of a live screen recorder when one is unavailable or
    unreliable: the frames are ordinary PNGs, so the video can be built later
    with ffmpeg or OpenCV and needs no live session at all. That is the only
    form of "video" that satisfies never re-running a simulation for a picture.
    """
    out_dir: Path
    period_s: float = 0.5
    prefix: str = "frame"
    frames: List[Path] = field(default_factory=list)
    _next_at: float = 0.0

    def tick(self) -> Optional[Path]:
        """Capture if enough time has passed. Call freely; it rate-limits."""
        now = time.perf_counter()
        if now < self._next_at:
            return None
        self._next_at = now + self.period_s
        path = self.out_dir / f"{self.prefix}_{len(self.frames):05d}.png"
        if screenshot(path):
            self.frames.append(path)
            return path
        return None


def frames_to_video(frame_dir: Path, out_mp4: Path, fps: int = 10,
                    pattern: str = "frame_%05d.png") -> bool:
    """Assemble a PNG sequence into mp4. Needs no live simulator."""
    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    try:
        proc = subprocess.run(
            ["ffmpeg", "-y", "-framerate", str(fps),
             "-i", str(frame_dir / pattern),
             "-c:v", "libx264", "-pix_fmt", "yuv420p", str(out_mp4)],
            capture_output=True, text=True, timeout=600,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False
    return proc.returncode == 0 and out_mp4.exists()


def record_screen(out_mp4: Path, duration_s: float, display: str = ":0",
                  size: str = "1920x1080", fps: int = 15):
    """Start an ffmpeg x11grab recording. Returns the Popen, or None.

    Non-blocking so the caller can keep driving the trial. Stop it with
    `stop_recording`.
    """
    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    try:
        return subprocess.Popen(
            ["ffmpeg", "-y", "-f", "x11grab", "-video_size", size,
             "-framerate", str(fps), "-i", display,
             "-t", str(duration_s), "-c:v", "libx264", "-preset", "ultrafast",
             "-pix_fmt", "yuv420p", str(out_mp4)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        return None


def stop_recording(proc, grace_s: float = 10.0) -> bool:
    """SIGINT so ffmpeg writes its moov atom; a SIGKILL leaves an unplayable file."""
    if proc is None:
        return False
    try:
        proc.send_signal(2)
        proc.wait(timeout=grace_s)
    except Exception:                                   # noqa: BLE001
        try:
            proc.kill()
        except Exception:
            pass
        return False
    return True


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="PNG path for a single shot")
    ap.add_argument("--check", action="store_true",
                    help="only report whether the screenshot service is up")
    args = ap.parse_args()

    if args.check:
        up = service_available()
        print(f"{SCREENSHOT_SERVICE}: {'available' if up else 'NOT available'}")
        return 0 if up else 1

    ok = screenshot(Path(args.out))
    print(f"screenshot -> {args.out}: {'ok' if ok else 'FAILED'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
