#!/usr/bin/env bash
#
# session_stop.sh -- tear down everything session_bringup.sh started.
#
# Also reachable as `flight_ops/session_bringup.sh --stop`.
#
# WHY THIS IS NOT JUST `tmux kill-server`
#
# Three reasons, in order of how much time each one has cost somebody:
#
# 1. `ros2 launch` is a supervisor. Killing its pane kills the launch service,
#    not necessarily the nodes it spawned. A node that survives its session
#    keeps publishing on the same topics as the next launch, and the symptom
#    -- two platforms fighting over one namespace -- looks exactly like a
#    hardware fault. So the panes get Ctrl-C first (which launch forwards as a
#    clean SIGINT to its children), and only then are the sessions killed.
#
# 2. `tmux kill-server` would also kill tmux sessions this project did not
#    start. Only sessions matching the o134_ prefix are touched.
#
# 3. A stale node holding a VehicleCommand publisher is a safety issue, not a
#    tidiness issue.
#
# The final sweep is targeted at the executables this project launches. It is
# listed explicitly below rather than hidden behind a wildcard so that anyone
# can see exactly what gets killed before running it.
#
# NOTE ON rviz2 AND robot_state_publisher: this sweep is by command line, and
# those two do not carry anything that distinguishes this project's copies
# from anyone else's. RViz is started inside as2_visualization; every namespace
# also gets a robot_state_publisher. If you keep an unrelated RViz or
# robot_state_publisher running on this machine, use --no-sweep and close the
# panes by hand. Everything else in the list is specific to this stack.
#
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION_PREFIX="o134_"

if [[ -t 1 ]]; then C_GRN=$'\033[1;32m'; C_CYN=$'\033[1;36m'; C_OFF=$'\033[0m'
else C_GRN=""; C_CYN=""; C_OFF=""; fi
info() { printf '%s[stop]%s %s\n' "${C_CYN}" "${C_OFF}" "$*"; }
good() { printf '%s[stop]%s %s\n' "${C_GRN}" "${C_OFF}" "$*"; }

usage() {
  cat <<'EOF'
session_stop.sh -- tear down the o134_* tmux sessions and their ROS nodes

  --list        show what would be stopped, then exit
  --no-sweep    kill the tmux sessions but do NOT pkill leftover nodes.
                Use when you are debugging a node you started by hand.
  -h, --help    this text
EOF
}

LIST_ONLY="false"
SWEEP="true"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --list)      LIST_ONLY="true" ;;
    --no-sweep)  SWEEP="false" ;;
    -h|--help)   usage; exit 0 ;;
    *)           usage >&2; exit 2 ;;
  esac
  shift
done

# --------------------------------------------------------------------------
# which sessions
# --------------------------------------------------------------------------
mapfile -t SESSIONS < <(tmux list-sessions -F '#{session_name}' 2>/dev/null \
                          | grep "^${SESSION_PREFIX}" || true)

if [[ "${LIST_ONLY}" == "true" ]]; then
  if [[ ${#SESSIONS[@]} -eq 0 ]]; then
    info "no ${SESSION_PREFIX}* tmux sessions are running"
  else
    info "would stop: ${SESSIONS[*]}"
  fi
  exit 0
fi

# If this script is being run from INSIDE one of the sessions it is about to
# kill, killing that session takes the script with it. Do it last.
CURRENT_SESSION=""
if [[ -n "${TMUX:-}" ]]; then
  CURRENT_SESSION="$(tmux display-message -p '#S' 2>/dev/null || true)"
fi

# --------------------------------------------------------------------------
# 1. Ctrl-C every pane, so `ros2 launch` shuts its children down cleanly
# --------------------------------------------------------------------------
for session in "${SESSIONS[@]}"; do
  info "interrupting panes in ${session}"
  while read -r pane; do
    [[ -n "${pane}" ]] || continue
    tmux send-keys -t "${pane}" C-c 2>/dev/null || true
  done < <(tmux list-panes -s -t "${session}" -F '#{pane_id}' 2>/dev/null || true)
done

# Give launch a moment to propagate SIGINT and let the nodes run their
# destructors. Two seconds is enough for every node in this stack; the sweep
# below catches anything that is not.
[[ ${#SESSIONS[@]} -eq 0 ]] || sleep 2

# --------------------------------------------------------------------------
# 2. kill the sessions
# --------------------------------------------------------------------------
for session in "${SESSIONS[@]}"; do
  [[ "${session}" == "${CURRENT_SESSION}" ]] && continue
  tmux kill-session -t "${session}" 2>/dev/null && info "killed ${session}" || true
done

# --------------------------------------------------------------------------
# 3. sweep up anything that outlived its pane
# --------------------------------------------------------------------------
if [[ "${SWEEP}" == "true" ]]; then
  PATTERNS=(
    # Aerostack2 nodes this project launches
    "as2_platform_pixhawk_node"
    "as2_platform_multirotor_simulator_node"
    "as2_state_estimator_node"
    "as2_motion_controller_node"
    "takeoff_behavior_node"
    "land_behavior_node"
    "go_to_behavior_node"
    "follow_path_behavior_node"
    "generate_polynomial_trajectory_behavior_node"
    "as2_alphanumeric_viewer_node"
    # as2_visualization spawns these two per namespace. They are named after
    # what they do, not after the package, so they must be listed by hand or
    # they outlive the session and keep republishing markers into the next one.
    "as2_visualization"
    "marker_publisher"
    # flight_ops nodes
    "flight_ops/nodes/volume_guard.py"
    "flight_ops/nodes/fake_mocap.py"
    "flight_ops/nodes/vrpn_to_rigidbodies.py"
    # visualisation
    "rviz2"
  )
  for pat in "${PATTERNS[@]}"; do
    if pkill -f "${pat}" 2>/dev/null; then info "swept ${pat}"; fi
  done
fi

# --------------------------------------------------------------------------
# 4. and finally the session this script is running in, if any
# --------------------------------------------------------------------------
if [[ -n "${CURRENT_SESSION}" && "${CURRENT_SESSION}" == ${SESSION_PREFIX}* ]]; then
  good "everything else is down; killing this session (${CURRENT_SESSION}) last"
  tmux kill-session -t "${CURRENT_SESSION}" 2>/dev/null || true
  exit 0
fi

REMAINING="$(tmux list-sessions -F '#{session_name}' 2>/dev/null | grep -c "^${SESSION_PREFIX}" || true)"
if [[ "${REMAINING}" == "0" ]]; then
  good "all ${SESSION_PREFIX}* sessions are down"
else
  info "${REMAINING} ${SESSION_PREFIX}* session(s) still listed -- re-run to be sure"
fi
exit 0
