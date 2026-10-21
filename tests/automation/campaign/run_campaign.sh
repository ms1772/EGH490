#!/bin/bash
# Campaign entry point. Run from the repo root INSIDE WSL.
#
# Everything the campaign needs from the environment is set here, because the
# driver inherits it and the stages inherit it from the driver. Each export
# below was a failure once:
#
#   ROS setup      -- without it `ros2`/`ign` are absent, the Gazebo stage fails
#                     its preflight, and the campaign stops at s5.
#   DISPLAY=:0     -- unset under a detached shell gives "Couldn't open X
#                     display :0" and a black Gazebo with no screenshot surface.
#   LIBGL_ALWAYS_SOFTWARE -- WSL has no usable GPU path for this renderer.
#   MPLBACKEND=Agg -- the planner imports pyplot; without a headless backend it
#                     can block or fail with no display.
#
# Do NOT run anything else on this machine while this is going. Measured: a
# concurrent test suite pushed a Gazebo trial's achieved RTF to 0.969 and its
# makespan 8% above the historical value for the identical plan.
set -u
set -o pipefail

REPO="/mnt/c/Users/mitch/Multi-UAV Project/ROS2_MultiUAV_3D-main"
cd "$REPO" || { echo "cannot cd to $REPO"; exit 2; }

set +u
source /opt/ros/humble/setup.bash
set -u

export DISPLAY="${DISPLAY:-:0}"
export LIBGL_ALWAYS_SOFTWARE=1
export MPLBACKEND=Agg
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

RUN_DIR="${1:-$REPO/tests/automation/results/campaign_$(date +%Y%m%d-%H%M%S)}"
JOBS="${JOBS:-8}"
RTF="${RTF:-1.0}"

mkdir -p "$RUN_DIR"
echo "run_dir : $RUN_DIR"
echo "jobs    : $JOBS   (offline stages only; Gazebo is always serial)"
echo "rtf     : $RTF"
echo "log     : $RUN_DIR/campaign/campaign.log"
echo

# Unbuffered so the log is useful while it runs, not just afterwards.
exec python3 -u tests/automation/campaign/campaign_driver.py --run-dir "$RUN_DIR" --jobs "$JOBS" --rtf "$RTF"
