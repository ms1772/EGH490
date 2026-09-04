#!/bin/bash
# Detached overnight runner for sweep_gazebo. nohup'd so it survives the parent
# bash/wsl session exit. Writes progress to /tmp/sweep_gazebo.log and a marker
# to /tmp/sweep_gazebo.done on completion.

set +e
source /opt/ros/humble/setup.bash 2>/dev/null
cd "/mnt/c/Users/mitch/Multi-UAV Project/ROS2_MultiUAV_3D-main"

LOG=/tmp/sweep_gazebo.log
DONE=/tmp/sweep_gazebo.done
PIDFILE=/tmp/sweep_gazebo.pid

# Clean previous markers
rm -f "$DONE" "$PIDFILE"
echo "$$" > "$PIDFILE"

echo "===== sweep_gazebo started at $(date) =====" > "$LOG"
echo "Command: python3 tests/automation/sweep_gazebo.py --matrix gazebo_subset --mode headless --offline-run-dir tests/automation/results/20260520-163723" >> "$LOG"

python3 tests/automation/sweep_gazebo.py \
    --matrix gazebo_subset \
    --mode headless \
    --offline-run-dir tests/automation/results/20260520-163723 \
    >> "$LOG" 2>&1
rc=$?

echo "" >> "$LOG"
echo "===== sweep_gazebo finished at $(date) =====" >> "$LOG"
echo "EXIT=$rc" >> "$LOG"
touch "$DONE"
