#!/usr/bin/env bash
#
# session_bringup.sh -- S7. THE launcher. One script, byte-identical off-site
#                       and in O-134.
#
# ============================================================================
# THE RULE THIS SCRIPT EXISTS TO ENFORCE
# ============================================================================
#
#   No event happens for the first time inside O-134.
#   A lab session executes a script that has already been executed.
#
# Which means: NOTHING IS EDITED IN THE LAB. Not this file, not anything under
# lab_config/o134_project/. Everything that varies between a bench run and a
# flight is a COMMAND-LINE FLAG, and every flag has already been exercised on
# rig R1 before anyone walks into the room. If you find yourself opening an
# editor in O-134, the session is already off the rails.
#
# The two modes are the same chain with one component swapped:
#
#   --sim        rig R1. as2_platform_multirotor_simulator stands in for the
#                Pixhawk, flight_ops/nodes/fake_mocap.py stands in for
#                OptiTrack. Runs entirely on the ground station, no hardware.
#   --hardware   O-134. as2_platform_pixhawk against a real FMU, real mocap.
#
# Everything downstream of the platform -- state estimator, controller,
# behaviors, volume guard, viewers -- is launched from the SAME configuration
# in both modes. That is what makes the rig worth running.
#
# ============================================================================
# USAGE
# ============================================================================
#
#   flight_ops/session_bringup.sh --sim
#   flight_ops/session_bringup.sh --sim --drones drone0,drone1,drone2
#   flight_ops/session_bringup.sh --hardware --drones drone0 --guard-arm
#   flight_ops/session_bringup.sh --stop
#
# Run --help for the full flag list.
#
# ============================================================================
# NO PERSISTENT HOOKS
# ============================================================================
# This script appends nothing to ~/.bashrc, installs no systemd unit and
# writes no crontab. Every environment change it makes lives inside the tmux
# panes it starts and dies with them. `--stop` leaves the machine as it was.
# ============================================================================

set -euo pipefail

# --------------------------------------------------------------------------
# where things are
# --------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_DIR="${SCRIPT_DIR}/lab_config/o134_project"
DRONES_YAML="${PROJECT_DIR}/config/drones.yaml"
TMUXINATOR_DIR="${PROJECT_DIR}/tmuxinator"
PREFLIGHT="${SCRIPT_DIR}/preflight_check.py"
STOP_SCRIPT="${SCRIPT_DIR}/session_stop.sh"

# Overridable, but with a default that matches the machine this was built on.
AS2_WS="${AS2_WS:-${HOME}/as2_o134_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

SESSION_PREFIX="o134_"
GROUND_SESSION="${SESSION_PREFIX}ground"

# --------------------------------------------------------------------------
# defaults
# --------------------------------------------------------------------------
MODE=""                      # sim | hardware -- no default, must be explicit
DRONES_CSV="drone0"          # build-up discipline: multi-drone is opt-in
ROLE="all"                   # drone | ground | all
SKIP_PREFLIGHT="false"
GUARD_ARM="false"
WANT_RVIZ="true"
WANT_VIEWER="true"
DOMAIN_ID=""                 # from drones.yaml meta unless overridden
LOG_LEVEL="info"
SETTLE_S="12"
ATTACH="false"
PRINT_PLAN="false"
PREFLIGHT_EXTRA=()

# --------------------------------------------------------------------------
# output helpers -- a lab script that mumbles is a lab script nobody reads
# --------------------------------------------------------------------------
if [[ -t 1 ]]; then
  C_RED=$'\033[1;31m'; C_YLW=$'\033[1;33m'; C_GRN=$'\033[1;32m'
  C_CYN=$'\033[1;36m'; C_OFF=$'\033[0m'
else
  C_RED=""; C_YLW=""; C_GRN=""; C_CYN=""; C_OFF=""
fi

say()   { printf '%s\n' "$*"; }
info()  { printf '%s[bringup]%s %s\n' "${C_CYN}" "${C_OFF}" "$*"; }
good()  { printf '%s[bringup]%s %s\n' "${C_GRN}" "${C_OFF}" "$*"; }
warn()  { printf '%s[bringup] WARNING:%s %s\n' "${C_YLW}" "${C_OFF}" "$*" >&2; }
die()   { printf '%s[bringup] FATAL:%s %s\n' "${C_RED}" "${C_OFF}" "$*" >&2; exit 2; }

rule()  { printf '%s\n' "----------------------------------------------------------------------"; }

loud_banner() {
  # $1 = colour, rest = lines
  local colour="$1"; shift
  printf '%s' "${colour}"
  printf '======================================================================\n'
  local line
  for line in "$@"; do printf '  %s\n' "${line}"; done
  printf '======================================================================\n'
  printf '%s' "${C_OFF}"
}

usage() {
  cat <<'EOF'
session_bringup.sh -- the single O-134 / rig-R1 launcher

MODE (exactly one is required -- there is deliberately no default)
  --sim                 Rig R1. as2_platform_multirotor_simulator plus
                        flight_ops/nodes/fake_mocap.py. No hardware needed.
  --hardware            O-134. as2_platform_pixhawk against a real FMU.

WHAT TO BRING UP
  --drones LIST         Comma-separated Aerostack2 namespaces.
                        Default: drone0. Multi-drone is OPT-IN because the
                        campaign builds up one airframe at a time.
                        Every name must appear in
                        lab_config/o134_project/config/drones.yaml.
  --role ROLE           drone | ground | all.  Default: all.
                        On a Jetson use --role drone --drones <that airframe>.
                        On the ground station use --role ground.
                        On rig R1 use the default: everything on one box.
  --no-rviz             Do not start RViz (ground role).
  --no-viewer           Do not start the alphanumeric viewers (ground role).

SAFETY
  --guard-arm           Make volume_guard's LAND and DISARM stages REAL.
                        Without it the guard runs in DRY RUN and only logs
                        what it would have sent. This is the bench default
                        and it is deliberately awkward to turn off.
  --skip-preflight      Do not run flight_ops/preflight_check.py at all.
                        Prints a very loud warning. Use only when you already
                        know the stack is broken and are debugging it.

PLUMBING
  --domain N            ROS_DOMAIN_ID. Default: the value in drones.yaml
                        (meta.ros_domain_id), which is 0 -- the domain the
                        flight controllers are gated to in
                        lab_config/expected_state.yaml. Override for bench
                        isolation only.
  --log-level LEVEL     debug | info | warn | error. Default: info.
  --settle SECONDS      How long to wait after launch before running the
                        post-bringup preflight. Default: 12.
  --preflight-arg ARG   Extra argument passed through to preflight_check.py.
                        Repeatable. This is how QGC parameter dumps reach the
                        px4_params check, e.g.
                          --preflight-arg --px4-params
                          --preflight-arg drone0:~/dumps/droneA.params
  --attach              Attach to the first session when everything is up.
                        Default: leave the sessions detached and return.
  --print-plan          Print every command that would be run, then exit.

TEARDOWN
  --stop                Tear down every o134_* tmux session and exit.
                        Equivalent to flight_ops/session_stop.sh.
  --status              List the o134_* sessions that are running, and exit.

  -h, --help            This text.

EXIT CODES
  0  the stack is up and the post-bringup preflight was GREEN
  1  the stack is up but the post-bringup preflight was RED -- DO NOT FIT
     PROPS. The stack is left running on purpose so the failure can be read.
  2  the launcher refused to start: bad arguments, missing packages, or the
     pre-bringup gate failed. Nothing was launched.
EOF
}

# --------------------------------------------------------------------------
# argument parsing
# --------------------------------------------------------------------------
set_mode() {
  [[ -z "${MODE}" ]] || die "--sim and --hardware are mutually exclusive"
  MODE="$1"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim)            set_mode sim ;;
    --hardware)       set_mode hardware ;;
    --drones)         DRONES_CSV="${2:?--drones needs a value}"; shift ;;
    --drones=*)       DRONES_CSV="${1#*=}" ;;
    --role)           ROLE="${2:?--role needs a value}"; shift ;;
    --role=*)         ROLE="${1#*=}" ;;
    --no-rviz)        WANT_RVIZ="false" ;;
    --no-viewer)      WANT_VIEWER="false" ;;
    --guard-arm)      GUARD_ARM="true" ;;
    --skip-preflight) SKIP_PREFLIGHT="true" ;;
    --domain)         DOMAIN_ID="${2:?--domain needs a value}"; shift ;;
    --domain=*)       DOMAIN_ID="${1#*=}" ;;
    --log-level)      LOG_LEVEL="${2:?--log-level needs a value}"; shift ;;
    --log-level=*)    LOG_LEVEL="${1#*=}" ;;
    --settle)         SETTLE_S="${2:?--settle needs a value}"; shift ;;
    --settle=*)       SETTLE_S="${1#*=}" ;;
    --preflight-arg)  PREFLIGHT_EXTRA+=("${2:?--preflight-arg needs a value}"); shift ;;
    --attach)         ATTACH="true" ;;
    --print-plan)     PRINT_PLAN="true" ;;
    --stop)           exec "${STOP_SCRIPT}" ;;
    --status)
      say "o134 tmux sessions:"
      tmux list-sessions 2>/dev/null | grep "^${SESSION_PREFIX}" || say "  (none)"
      exit 0 ;;
    -h|--help)        usage; exit 0 ;;
    *)                usage >&2; die "unknown argument: $1" ;;
  esac
  shift
done

[[ -n "${MODE}" ]] || { usage >&2; die "one of --sim or --hardware is required. There is no default: launching the wrong platform against real hardware is not a mistake this script will make on your behalf."; }

case "${ROLE}" in
  drone|ground|all) ;;
  *) die "--role must be drone, ground or all (got '${ROLE}')" ;;
esac

[[ -f "${DRONES_YAML}" ]] || die "missing ${DRONES_YAML}"
[[ -d "${TMUXINATOR_DIR}" ]] || die "missing ${TMUXINATOR_DIR}"
[[ -f "${PREFLIGHT}" ]] || die "missing ${PREFLIGHT}"

# --------------------------------------------------------------------------
# read config/drones.yaml -- the single source of per-aircraft truth
# --------------------------------------------------------------------------
# Emits one "ns|fmu_prefix|target_system_id|rigid_body_name" line per
# REQUESTED drone, in the requested order, and fails loudly if a requested
# namespace is not in the table. A typo in --drones must not silently produce
# an aircraft with default values.
read_drone_table() {
  python3 - "${DRONES_YAML}" "${DRONES_CSV}" <<'PY'
import sys, yaml

path, requested = sys.argv[1], sys.argv[2]
with open(path, "r", encoding="utf-8") as fh:
    doc = yaml.safe_load(fh) or {}
table = doc.get("drones") or {}
if not isinstance(table, dict) or not table:
    sys.exit("drones.yaml has no 'drones:' mapping")

names = [n.strip() for n in requested.split(",") if n.strip()]
if not names:
    sys.exit("--drones resolved to an empty list")
if len(set(names)) != len(names):
    sys.exit("--drones repeats a namespace")

missing = [n for n in names if n not in table]
if missing:
    sys.exit("--drones names %s, which is not in %s. Known: %s"
             % (", ".join(missing), path, ", ".join(sorted(table))))

required = ("fmu_prefix", "target_system_id", "rigid_body_name")
for ns in names:
    row = table[ns] or {}
    absent = [k for k in required if row.get(k) in (None, "")]
    if absent:
        sys.exit("drones.yaml: %s is missing %s" % (ns, ", ".join(absent)))
    # A rigid body name that does not match Motive is defect D1. There is
    # nothing this script can do about a wrong name, but it can refuse an
    # empty or whitespace one, which is the case the plugin cannot report
    # usefully because it never gets constructed.
    body = str(row["rigid_body_name"]).strip()
    if not body:
        sys.exit("drones.yaml: %s has a blank rigid_body_name" % ns)
    print("%s|%s|%s|%s" % (ns, row["fmu_prefix"], row["target_system_id"], body))

meta = doc.get("meta") or {}
print("__meta__|%s" % meta.get("ros_domain_id", 0))
PY
}

TABLE="$(read_drone_table)" || die "could not read ${DRONES_YAML} (see the message above)"

DRONE_NS=()
DRONE_FMU=()
DRONE_SYS=()
DRONE_BODY=()
YAML_DOMAIN="0"
while IFS='|' read -r a b c d; do
  [[ -n "${a}" ]] || continue
  if [[ "${a}" == "__meta__" ]]; then YAML_DOMAIN="${b}"; continue; fi
  DRONE_NS+=("${a}"); DRONE_FMU+=("${b}"); DRONE_SYS+=("${c}"); DRONE_BODY+=("${d}")
done <<< "${TABLE}"

[[ ${#DRONE_NS[@]} -gt 0 ]] || die "no drones selected"
[[ -n "${DOMAIN_ID}" ]] || DOMAIN_ID="${YAML_DOMAIN}"

# Normalise the requested list to exactly what the table produced, so every
# consumer downstream (viewers, guard, preflight) sees the same order.
DRONES_CSV="$(IFS=,; echo "${DRONE_NS[*]}")"

# --------------------------------------------------------------------------
# derived per-mode values
# --------------------------------------------------------------------------
# mocap_topic is per MODE, not per aircraft:
#   hardware  one VRPN client + bridge per Jetson, publishing into that
#             drone's own namespace (deploy/README.md section 2, Option B),
#             so a wifi dropout starves one aircraft, not three.
#   sim       one fake_mocap.py for the whole rig on the ground station,
#             which is also what volume_guard and RViz consume.
mocap_topic_for() {
  if [[ "${MODE}" == "hardware" ]]; then printf '/%s/mocap/rigid_bodies' "$1"
  else printf '/mocap/rigid_bodies'; fi
}

# --------------------------------------------------------------------------
# the environment
# --------------------------------------------------------------------------
source_ros() {
  # set +u first, always: the ROS setup scripts dereference unset variables.
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  if [[ -f "${AS2_WS}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${AS2_WS}/install/setup.bash"
  fi
  set -u
  export ROS_DOMAIN_ID="${DOMAIN_ID}"
}

require_pkg() {
  ros2 pkg prefix "$1" >/dev/null 2>&1 \
    || die "ROS 2 package '$1' is not on AMENT_PREFIX_PATH. ${2:-}"
}

# --------------------------------------------------------------------------
# summary
# --------------------------------------------------------------------------
print_summary() {
  rule
  say "  mode              ${MODE}"
  say "  role              ${ROLE}"
  say "  drones            ${DRONES_CSV}"
  say "  ROS_DOMAIN_ID     ${DOMAIN_ID}"
  say "  workspace         ${AS2_WS}"
  say "  project           ${PROJECT_DIR}"
  say "  volume guard      $([[ "${GUARD_ARM}" == "true" ]] && echo 'ARMED (land and disarm are REAL)' || echo 'dry run (logs only)')"
  say "  preflight         $([[ "${SKIP_PREFLIGHT}" == "true" ]] && echo 'SKIPPED' || echo 'enabled')"
  rule
  local i
  for i in "${!DRONE_NS[@]}"; do
    printf '  %-8s fmu_prefix=%-8s target_system_id=%-3s rigid_body=%-10s mocap=%s\n' \
      "${DRONE_NS[$i]}" "${DRONE_FMU[$i]}" "${DRONE_SYS[$i]}" "${DRONE_BODY[$i]}" \
      "$(mocap_topic_for "${DRONE_NS[$i]}")"
  done
  rule
}

# --------------------------------------------------------------------------
# volume_guard arguments, derived from the ONE table
# --------------------------------------------------------------------------
# THE fmu-prefix PROBLEM, AND WHY THE ANSWER IS A REMAP
#
# PX4's UXRCE_DDS_NS_IDX puts each flight controller's topics under /uav_N.
# volume_guard's --fmu-prefix resolves '{ns}' with the AEROSTACK2 namespace
# ("drone0"), never with "uav_0", and the substitution is guarded by a literal
# `"{ns}" in prefix` test, so no format trick (e.g. '/uav_{ns[5]}') reaches
# the formatter either. There is no string that expresses drone0 -> /uav_0.
#
# So the mapping is done one layer down, by ROS 2 static remapping, which
# rewrites the topic AFTER volume_guard has named it and needs no change to
# volume_guard.py:
#
#     --fmu-prefix '/{ns}'
#     --ros-args -r /drone0/fmu/in/vehicle_command:=/uav_0/fmu/in/vehicle_command
#
# Verified: the node's publishers land on /uav_0|1|2/fmu/in/vehicle_command.
#
# CAVEAT, because it will bite someone at 2 a.m.: volume_guard's own log line
# and its /volume_guard/status payload report the PRE-remap name
# (/drone0/fmu/...), because that is the name it asked for. `ros2 node info
# /volume_guard` reports the real one. Trust the graph, not the log.
build_guard_args() {
  local bodies=() systems=() remaps=() i
  for i in "${!DRONE_NS[@]}"; do
    bodies+=("${DRONE_NS[$i]}:${DRONE_BODY[$i]}")
    systems+=("${DRONE_NS[$i]}:${DRONE_SYS[$i]}")
    if [[ "${MODE}" == "hardware" ]]; then
      remaps+=("-r" "/${DRONE_NS[$i]}/fmu/in/vehicle_command:=${DRONE_FMU[$i]}/fmu/in/vehicle_command")
    fi
  done

  GUARD_ARGS="--drones ${DRONES_CSV}"
  GUARD_ARGS+=" --rigid-bodies $(IFS=,; echo "${bodies[*]}")"
  GUARD_ARGS+=" --target-systems $(IFS=,; echo "${systems[*]}")"
  # Single-quoted so the braces reach Python untouched.
  GUARD_ARGS+=" --fmu-prefix '/{ns}'"
  if [[ "${GUARD_ARM}" == "true" ]]; then GUARD_ARGS+=" --arm"; else GUARD_ARGS+=" --dry-run"; fi

  GUARD_ROS_ARGS=""
  if [[ ${#remaps[@]} -gt 0 ]]; then GUARD_ROS_ARGS="${remaps[*]}"; fi
}

# --------------------------------------------------------------------------
# preflight
# --------------------------------------------------------------------------
# TWO GATES, BECAUSE preflight_check.py MEASURES A RUNNING SYSTEM
#
# S4 checks mocap rate, the TF tree, live PX4 parameters, the DDS link, the
# battery and the Aerostack2 node set. Seven of its nine checks are
# meaningless before the stack exists, so a single "run it, then decide
# whether to launch" gate would fail on every launch, forever, and would be
# switched off within a week.
#
#   PRE   runs before anything is launched: preflight_check.py --self-test,
#         139 assertions, no ROS required. It answers "is the gate itself
#         trustworthy, and does this checkout hang together". If it fails,
#         NOTHING IS LAUNCHED -- a broken gate is worse than no gate, because
#         a crew will read its GREEN.
#
#   POST  runs after the stack has settled: the real props-on verdict. It
#         cannot un-launch anything, so it does not try. It prints the table,
#         says PREFLIGHT RED, and exits 1 with the stack STILL RUNNING, which
#         is the only state in which the failure can be diagnosed.
#
# --skip-preflight skips both.
run_pre_gate() {
  info "PRE gate: preflight_check.py --self-test"
  if python3 "${PREFLIGHT}" --self-test >/tmp/o134_preflight_selftest.$$ 2>&1; then
    good "PRE gate PASSED -- $(grep -c '\[PASS\]' /tmp/o134_preflight_selftest.$$) assertions, $(tail -1 /tmp/o134_preflight_selftest.$$)"
    rm -f /tmp/o134_preflight_selftest.$$
    return 0
  fi
  cat /tmp/o134_preflight_selftest.$$ >&2
  rm -f /tmp/o134_preflight_selftest.$$
  loud_banner "${C_RED}" \
    "PRE-BRINGUP GATE FAILED" \
    "" \
    "preflight_check.py --self-test did not pass, so the props-on gate" \
    "itself cannot be trusted. NOTHING HAS BEEN LAUNCHED." \
    "" \
    "Fix the tool, or re-run with --skip-preflight if you are deliberately" \
    "debugging it. Do not fly against a gate that cannot check itself." >&2
  exit 2
}

# Run S4 against the running stack and return its verdict.
#
# In --hardware the fleet-wide invocation cannot cover four of the checks,
# because preflight_check.py takes ONE --mocap-topic and ONE --fmu-prefix and
# this topology has one of each PER AIRCRAFT (per-Jetson mocap bridges,
# per-FC /uav_N namespaces). The deploy README calls this out as a known gap.
# The launcher closes it the only way it can without touching the tool: run
# those four checks once per drone, with that drone's own topic and prefix.
# NOTE ON THE SEPARATOR: preflight_check.py's select_checks() splits --only and
# --skip on COMMAS, not whitespace. A space-separated list is rejected with
# "unknown check(s): ..." and exit 2, which reads like a broken tool rather
# than a typo. (flight_ops/deploy/README.md contains a space-separated example
# that does not work; it is not this script's to fix.)
run_post_gate() {
  local rc=0 per_drone_only="mocap_rate,rigid_bodies,pose_delta,dds_link"

  if [[ "${MODE}" == "sim" ]]; then
    # No FMU and no battery on rig R1, so those two checks are skipped
    # explicitly rather than left to FAIL and be ignored -- an ignored FAIL
    # trains a crew to ignore FAILs.
    info "POST gate: preflight_check.py (rig R1 subset)"
    python3 "${PREFLIGHT}" \
      --drones "${DRONES_CSV}" \
      --mocap-topic "/mocap/rigid_bodies" \
      --skip "dds_link,battery,px4_params" \
      "${PREFLIGHT_EXTRA[@]+"${PREFLIGHT_EXTRA[@]}"}" || rc=$?
    return "${rc}"
  fi

  info "POST gate: preflight_check.py (fleet-wide checks)"
  python3 "${PREFLIGHT}" \
    --drones "${DRONES_CSV}" \
    --skip "${per_drone_only}" \
    "${PREFLIGHT_EXTRA[@]+"${PREFLIGHT_EXTRA[@]}"}" || rc=$?

  local i sub=0
  for i in "${!DRONE_NS[@]}"; do
    info "POST gate: preflight_check.py (${DRONE_NS[$i]} mocap + DDS link)"
    sub=0
    python3 "${PREFLIGHT}" \
      --drones "${DRONE_NS[$i]}" \
      --only "${per_drone_only}" \
      --mocap-topic "$(mocap_topic_for "${DRONE_NS[$i]}")" \
      --fmu-prefix "${DRONE_FMU[$i]}" \
      "${PREFLIGHT_EXTRA[@]+"${PREFLIGHT_EXTRA[@]}"}" || sub=$?
    [[ ${sub} -eq 0 ]] || rc=1
  done
  return "${rc}"
}

# --------------------------------------------------------------------------
# tmux
# --------------------------------------------------------------------------
session_exists() { tmux has-session -t "$1" 2>/dev/null; }

refuse_if_running() {
  local live=() s
  for s in "$@"; do session_exists "${s}" && live+=("${s}"); done
  if [[ ${#live[@]} -gt 0 ]]; then
    die "these tmux sessions are already up: ${live[*]}
Two stacks in one ROS domain fight over every topic and the symptoms look
like a hardware fault. Tear the old one down first:
    ${STOP_SCRIPT}"
  fi
}

start_drone_session() {
  local i="$1" ns="${DRONE_NS[$i]}" session="${SESSION_PREFIX}${DRONE_NS[$i]}"
  local cmd=(tmuxinator start -n "${session}" -p "${TMUXINATOR_DIR}/drone.yaml"
    "drone_namespace=${ns}"
    "mode=${MODE}"
    "project_dir=${PROJECT_DIR}"
    "repo_root=${REPO_ROOT}"
    "ws_setup=${AS2_WS}/install/setup.bash"
    "ros_domain_id=${DOMAIN_ID}"
    "fmu_prefix=${DRONE_FMU[$i]}"
    "target_system_id=${DRONE_SYS[$i]}"
    "rigid_body_name=${DRONE_BODY[$i]}"
    "mocap_topic=$(mocap_topic_for "${ns}")"
    "log_level=${LOG_LEVEL}")
  if [[ "${PRINT_PLAN}" == "true" ]]; then printf '  %q ' "${cmd[@]}"; printf '\n'; return 0; fi
  info "starting ${session}"
  "${cmd[@]}" >/dev/null
}

start_ground_session() {
  build_guard_args
  local mocap_bodies="${DRONE_BODY[*]}"
  local cmd=(tmuxinator start -n "${GROUND_SESSION}" -p "${TMUXINATOR_DIR}/ground_station.yaml"
    "drones_comma=${DRONES_CSV}"
    "mode=${MODE}"
    "project_dir=${PROJECT_DIR}"
    "repo_root=${REPO_ROOT}"
    "ws_setup=${AS2_WS}/install/setup.bash"
    "ros_domain_id=${DOMAIN_ID}"
    "guard_args=${GUARD_ARGS}"
    "guard_ros_args=${GUARD_ROS_ARGS}"
    "mocap_bodies=${mocap_bodies}"
    "rviz=${WANT_RVIZ}"
    "viewer=${WANT_VIEWER}")
  if [[ "${PRINT_PLAN}" == "true" ]]; then printf '  %q ' "${cmd[@]}"; printf '\n'; return 0; fi
  info "starting ${GROUND_SESSION}"
  "${cmd[@]}" >/dev/null
}

# ==========================================================================
# main
# ==========================================================================
print_summary

if [[ "${MODE}" == "hardware" ]]; then
  loud_banner "${C_YLW}" \
    "HARDWARE MODE -- MUST BE DETERMINED BEFORE THE FIRST ARMED FLIGHT" \
    "" \
    "MBD-1  config/platform_pixhawk.yaml : max_thrust is still the UPSTREAM" \
    "       DEFAULT 15.0 N, not a measured X500 V2 value. It scales the" \
    "       whole newtons-to-PX4-setpoint mapping, and getting it wrong" \
    "       reads exactly like a tuning problem. Card I-04." \
    "MBD-2  config/drones.yaml : target_system_id must equal each airframe's" \
    "       MAV_SYS_ID, read back in QGC. Not assumed, not written by us." \
    "MBD-3  config/platform_pixhawk.yaml : external_odom_timeout_s 0.1 s has" \
    "       never been measured against the real OptiTrack chain's jitter." \
    "MBD-4  config/pid_speed_controller.yaml : gains are the Aerostack2" \
    "       Gazebo reference values. Never flown on this airframe." \
    "MBD-5  config/volume_guard.yaml : box_center_m assumes the mocap origin" \
    "       is at mid-height. If Motive's origin is on the floor this must" \
    "       be [0, 0, 2] or half the fence is in the ground." \
    "MBD-6  config/drones.yaml : rigid_body_name assumes Motive streams" \
    "       'drone0/1/2'. Confirm the exact strings, case included." \
    "MBD-7  config/mocap_pose_guarded.yaml : earth_to_map_* are identity." \
    "       Correct unless the lab frame is deliberately offset -- and then" \
    "       it must change for EVERY aircraft in the session." \
    "" \
    "AND: UXRCE_DDS_NS_IDX must be 0/1/2 on the three FCs, written in QGC," \
    "read back, and REBOOTED. Nothing in this repository sets it." \
    "" \
    "See lab_config/o134_project/README.md for the full list."
fi

if [[ "${GUARD_ARM}" == "true" ]]; then
  loud_banner "${C_RED}" \
    "VOLUME GUARD IS ARMED" \
    "" \
    "LAND and FORCE-DISARM are REAL. A latched DISARM cuts the motors in" \
    "the air and THE AIRCRAFT WILL FALL. This is correct for a net strike" \
    "at speed and wrong for a marginal trip -- the ladder and the dwell" \
    "times are what separate them, and they are set on the run-sheet card." \
    "" \
    "Reset a latched trip with:" \
    "  ros2 service call /volume_guard/reset_all std_srvs/srv/Trigger"
elif [[ "${MODE}" == "hardware" ]]; then
  warn "the volume guard is in DRY RUN: it will log a LAND or DISARM and send NOTHING."
  warn "For a flight with props on, pass --guard-arm. This is not a default on purpose."
fi

if [[ "${SKIP_PREFLIGHT}" == "true" ]]; then
  loud_banner "${C_RED}" \
    "PREFLIGHT SKIPPED -- --skip-preflight WAS PASSED" \
    "" \
    "flight_ops/preflight_check.py is the props-on gate. It has not run." \
    "Nothing has verified the mocap rate, the TF tree, the PX4 parameter" \
    "read-back, the DDS link, the battery or the Aerostack2 node set." \
    "" \
    "DO NOT FIT PROPELLERS ON THE STRENGTH OF THIS LAUNCH." \
    "This flag exists to debug a stack that is already known to be broken."
fi

# Environment before anything that needs ros2 on PATH.
source_ros
info "ROS 2 sourced; ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

command -v tmux >/dev/null 2>&1 || die "tmux is not installed"
command -v tmuxinator >/dev/null 2>&1 || die "tmuxinator is not installed"

# Package availability, checked before launching rather than discovered as a
# dead tmux pane thirty seconds later.
if [[ "${ROLE}" != "ground" ]]; then
  require_pkg as2_mocap_guarded \
    "Build it: cd ${AS2_WS} && colcon build --packages-select as2_mocap_guarded"
  require_pkg as2_motion_controller
  require_pkg as2_behaviors_motion
  require_pkg as2_behaviors_trajectory_generation
  if [[ "${MODE}" == "hardware" ]]; then
    require_pkg as2_platform_pixhawk \
      "It is built from source with the O134 patch; see flight_ops/patches/."
  else
    require_pkg as2_platform_multirotor_simulator
  fi
fi
if [[ "${ROLE}" != "drone" ]]; then
  [[ -f "${REPO_ROOT}/flight_ops/nodes/volume_guard.py" ]] || die "missing volume_guard.py"
  if [[ "${MODE}" == "sim" ]]; then
    [[ -f "${REPO_ROOT}/flight_ops/nodes/fake_mocap.py" ]] || die "missing fake_mocap.py"
  fi
  [[ "${WANT_VIEWER}" != "true" ]] || require_pkg as2_alphanumeric_viewer
  [[ "${WANT_RVIZ}" != "true" ]] || require_pkg as2_visualization
fi

[[ "${SKIP_PREFLIGHT}" == "true" ]] || run_pre_gate

# Sessions we are about to create.
SESSIONS=()
if [[ "${ROLE}" != "ground" ]]; then
  for ns in "${DRONE_NS[@]}"; do SESSIONS+=("${SESSION_PREFIX}${ns}"); done
fi
[[ "${ROLE}" == "drone" ]] || SESSIONS+=("${GROUND_SESSION}")

if [[ "${PRINT_PLAN}" == "true" ]]; then
  say ""
  say "Commands that would be run:"
  [[ "${ROLE}" == "ground" ]] || for i in "${!DRONE_NS[@]}"; do start_drone_session "${i}"; done
  [[ "${ROLE}" == "drone" ]] || start_ground_session
  say ""
  say "Sessions: ${SESSIONS[*]}"
  exit 0
fi

refuse_if_running "${SESSIONS[@]}"

if [[ "${ROLE}" != "ground" ]]; then
  for i in "${!DRONE_NS[@]}"; do
    start_drone_session "${i}"
    sleep 0.4   # let tmuxinator finish writing its project before the next
  done
fi
if [[ "${ROLE}" != "drone" ]]; then
  start_ground_session
fi

good "sessions up: ${SESSIONS[*]}"
info "settling for ${SETTLE_S}s before the post-bringup preflight"
sleep "${SETTLE_S}"

VERDICT=0
PREFLIGHT_RC=0
if [[ "${SKIP_PREFLIGHT}" == "true" ]]; then
  warn "post-bringup preflight skipped."
else
  rule
  run_post_gate || PREFLIGHT_RC=$?
  rule
  # Normalised to 1. Exit 2 from this script means "refused to start, nothing
  # was launched"; by here the stack IS running, so reusing 2 would be a lie
  # to whatever is reading the exit code.
  [[ "${PREFLIGHT_RC}" -eq 0 ]] || VERDICT=1
fi

if [[ "${VERDICT}" -eq 0 ]]; then
  if [[ "${SKIP_PREFLIGHT}" == "true" ]]; then
    warn "the stack is up but NOTHING HAS BEEN VERIFIED."
  else
    good "PREFLIGHT GREEN -- the stack is up and every non-skipped check passed."
  fi
else
  loud_banner "${C_RED}" \
    "PREFLIGHT RED -- DO NOT FIT PROPELLERS" \
    "" \
    "preflight_check.py exited ${PREFLIGHT_RC} (1 = a check FAILED," \
    "2 = it could not verify anything: no ROS, bad arguments, all skipped)." \
    "" \
    "The stack has been LEFT RUNNING on purpose: a torn-down stack cannot" \
    "be diagnosed. Read the FAIL rows above, fix, then:" \
    "  ${STOP_SCRIPT}" \
    "and launch again." >&2
fi

say ""
say "  attach:  tmux attach -t ${SESSIONS[0]}"
say "  list:    tmux list-sessions | grep '^${SESSION_PREFIX}'"
say "  stop:    ${STOP_SCRIPT}"
say ""

if [[ "${ATTACH}" == "true" ]]; then
  exec tmux attach-session -t "${SESSIONS[0]}"
fi

exit "${VERDICT}"
