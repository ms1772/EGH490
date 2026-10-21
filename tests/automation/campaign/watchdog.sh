#!/bin/bash
# Self-healing watchdog for the campaign. Detached, like the campaign itself.
#
# WHY THIS IS NOT A NOTIFIER
# --------------------------
# A watcher that merely reports a failure is worth little here: the session
# only reads it when it next polls, so the detection latency is the polling
# interval either way. What matters over an unattended 16 h run is that a
# recoverable failure RECOVERS without anyone present. So this takes action.
#
# It is safe to restart the campaign because every layer is resumable: the
# driver skips stages with a `.done` marker, `sweep_gazebo` skips trials with
# an ok row, and non-ok rows are purged on resume so they retry. Restarting
# therefore costs at most the trial that was in flight.
#
# It must be setsid-detached for the same reason the campaign is: Claude Code
# reaps its own tracked background shells when host memory runs low, and a
# Gazebo trial pushes the host there routinely. A tracked watchdog would be
# killed precisely when it is most needed.
set -u

REPO="/mnt/c/Users/mitch/Multi-UAV Project/ROS2_MultiUAV_3D-main"
RUN_DIR="${1:-$REPO/tests/automation/results/campaign_20261011}"
LOG="$RUN_DIR/campaign/watchdog.log"
BEAT="$RUN_DIR/campaign/watchdog_alive.txt"
PERIOD="${PERIOD:-120}"
MAX_RESTARTS="${MAX_RESTARTS:-5}"
# Completion is read from the driver's OWN final log line, not from a stage
# marker. `s7_report` is required=False, so if it failed the campaign would
# still finish cleanly but write no marker -- and a marker-based test would
# then restart a completed campaign over and over.
CAMPAIGN_LOG="$RUN_DIR/campaign/campaign.log"

mkdir -p "$RUN_DIR/campaign"
log() { echo "$(date -Is) $*" >> "$LOG"; }
log "watchdog start (period=${PERIOD}s, max_restarts=$MAX_RESTARTS, run_dir=$RUN_DIR)"

restarts=0
while true; do
  sleep "$PERIOD"
  date -Is > "$BEAT"

  if grep -q "campaign COMPLETE" "$CAMPAIGN_LOG" 2>/dev/null; then
    log "driver logged completion -- watchdog exiting"
    exit 0
  fi

  if pgrep -f "[c]ampaign_driver" >/dev/null 2>&1; then
    continue
  fi

  # Driver is gone but the campaign is not finished.
  if [ "$restarts" -ge "$MAX_RESTARTS" ]; then
    log "driver absent and restart budget ($MAX_RESTARTS) exhausted -- NOT restarting again."
    log "something is failing repeatedly; leaving the run dir untouched for inspection."
    exit 1
  fi
  restarts=$((restarts + 1))

  # A driver that died mid-trial can leave the simulator running, and a fresh
  # trial launching into a live Gazebo would be contaminated at best. Clear it
  # before restarting. These are the same patterns gazebo_runner.teardown uses.
  killed=0
  for pat in "ign-gazebo" "ign gazebo" "gz sim" "ros_gz_bridge" "rviz2" "deckga_execute" "parameter_bridge"; do
    if pgrep -f "$pat" >/dev/null 2>&1; then
      pkill -f "$pat" 2>/dev/null && killed=$((killed + 1))
    fi
  done
  tmux kill-server 2>/dev/null || true
  sleep 10
  log "driver absent; cleared $killed orphan pattern(s); restart #$restarts"

  setsid nohup bash "$REPO/tests/automation/campaign/run_campaign.sh" "$RUN_DIR" \
      >> "$RUN_DIR/campaign/restart_${restarts}.out" 2>&1 < /dev/null &

  # Give the new driver time to come up before the next liveness check, so a
  # slow start is not mistaken for another death.
  sleep 90
done
