#!/usr/bin/env bash
#
# mocap_bringup.sh -- OptiTrack (VRPN) -> ROS 2 mocap chain, one command.
#
#   Motive (VRPN server)  --VRPN-->  vrpn_mocap client_node  --PoseStamped-->
#   flight_ops/nodes/vrpn_to_rigidbodies.py  --mocap4r2_msgs/RigidBodies-->
#   as2_state_estimator (mocap_pose_guarded) / volume_guard / preflight
#
# Runs on a Jetson (per aircraft, namespaced) and on the ground station
# (--gs, unnamespaced). Same file on both. deploy/README.md section 2,
# Option B.
#
#   Jetson:  mocap_bringup.sh --server 131.181.135.150 --drone drone0
#              client   /drone0/vrpn_mocap/client   -> /drone0/vrpn_mocap/<tracker>/pose
#              bridge   -> /drone0/mocap/rigid_bodies  (+ /drone0/mocap/health)
#
#   GS:      mocap_bringup.sh --server 131.181.135.150 --gs
#              client   /vrpn_mocap/client           -> /vrpn_mocap/<tracker>/pose
#              bridge   -> /mocap/rigid_bodies         (+ /mocap/health)
#            which is what volume_guard.py and the launcher's ground role
#            consume (config/volume_guard.yaml: mocap_topic /mocap/rigid_bodies).
#
# MEASURED ON THE GROUND STATION, 2026-09-11, ros-humble-vrpn-mocap 1.1.0
# against a local VRPN server (not assumed from the README):
#   * executable is `client_node` (there is no `client`); default node name
#     is `client`, so `-r __ns:=/drone0/vrpn_mocap` gives /drone0/vrpn_mocap/client.
#   * topic is <node namespace>/<tracker>/pose, geometry_msgs/PoseStamped,
#     QoS BEST_EFFORT / KEEP_LAST(5) / VOLATILE (param sensor_data_qos: true).
#     With `-r __ns:=/drone0` alone the topic would be /drone0/drone0/pose and
#     the bridge template in the README would match NOTHING, silently.
#   * against a dead or silent server the client LOGS NOTHING, creates no
#     topics and sits there. Hence the data check below: the only evidence
#     of a live link is a message on the output topic.
#   * the client picks up a server that appears later (~1 s, refresh_freq) and
#     recovers from a server restart without being restarted. So if Motive
#     streaming is switched on AFTER this script ran, just re-run --status.
#
# NO PERSISTENT HOOKS. Nothing is written outside a tmux session and
# /tmp/o134_mocap_<name>/. --stop leaves the machine as it was.
#
# INDEPENDENT OF THE AS2 STACK, ON PURPOSE. The tmux session is mocap_<name>,
# not o134_*, and the bridge is launched as <deploy>/../nodes/vrpn_to_rigidbodies.py
# -- NOT realpath'd -- so that flight_ops/session_stop.sh's sweep pattern
# "flight_ops/nodes/vrpn_to_rigidbodies.py" does not match it (verified on the
# GS: session_stop.sh ran, the chain stayed up, --status still GREEN). Prove
# the mocap link once, then restart the AS2 stack as often as you like without
# losing it. The only thing that stops this chain is `--stop` here.
#
# Idempotent: re-running with the same flags is a no-op plus a data check;
# with different flags it restarts; with a dead process it restarts.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE="${SCRIPT_DIR}/../nodes/vrpn_to_rigidbodies.py"   # keep the ../ -- see the header
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
AS2_WS="${AS2_WS:-${HOME}/as2_o134_ws}"

# --------------------------------------------------------------------------
# defaults
# --------------------------------------------------------------------------
SERVER=""                 # REQUIRED for a start. Never hardcoded: confirm in Motive on the day.
PORT="3883"
DRONE="drone0"            # Aerostack2 namespace; also the default body name
TRACKERS=""               # Motive rigid-body name(s) as streamed; default = DRONE
BODIES=""                 # rigid_body_name(s) written into RigidBodies; default = DRONE
GS_MODE="false"           # --gs: unnamespaced /mocap/rigid_bodies for the ground station
FRAME_ID="map"
STALE_MS="100"            # = volume_guard mocap_timeout_s and platform external_odom_timeout_s
UPDATE_HZ="100.0"
WAIT_S="10"               # data-check window
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
ACTION="start"            # start | status | stop | plan
ATTACH="false"
NO_CHECK="false"

if [[ -t 1 ]]; then
  C_RED=$'\033[1;31m'; C_YLW=$'\033[1;33m'; C_GRN=$'\033[1;32m'; C_CYN=$'\033[1;36m'; C_OFF=$'\033[0m'
else
  C_RED=""; C_YLW=""; C_GRN=""; C_CYN=""; C_OFF=""
fi
say()  { printf '%s\n' "$*"; }
info() { printf '%s[mocap]%s %s\n' "${C_CYN}" "${C_OFF}" "$*"; }
good() { printf '%s[mocap]%s %s\n' "${C_GRN}" "${C_OFF}" "$*"; }
warn() { printf '%s[mocap] WARNING:%s %s\n' "${C_YLW}" "${C_OFF}" "$*" >&2; }
die()  { printf '%s[mocap] FATAL:%s %s\n' "${C_RED}" "${C_OFF}" "$*" >&2; exit 2; }
loud() {
  printf '%s' "${C_RED}"
  printf '======================================================================\n'
  local line; for line in "$@"; do printf '  %s\n' "${line}"; done
  printf '======================================================================\n'
  printf '%s' "${C_OFF}"
}

usage() {
  cat <<'EOF'
mocap_bringup.sh -- start / check / stop the VRPN -> RigidBodies mocap chain

  mocap_bringup.sh --server IP [--port 3883] [--drone drone0] [--tracker NAME]
  mocap_bringup.sh --server IP --gs
  mocap_bringup.sh --status [--drone drone0 | --gs]
  mocap_bringup.sh --stop   [--drone drone0 | --gs]

WHERE
  --server IP        Motive / VRPN server address. REQUIRED to start. Read it
                     off Motive's Streaming pane on the day; never assume it.
  --port N           VRPN port. Default 3883.

WHAT
  --drone NS         Aerostack2 namespace (default drone0). The bridge publishes
                     /NS/mocap/rigid_bodies and /NS/mocap/health, the client
                     lives in /NS/vrpn_mocap. Must be in config/drones.yaml.
  --gs               Ground-station copy: NO namespace. Publishes
                     /mocap/rigid_bodies and /mocap/health, which is what
                     volume_guard.py (config/volume_guard.yaml) consumes.
  --tracker NAME     Rigid-body name exactly as Motive streams it (default: the
                     --drone value). Comma list allowed. If Motive streams e.g.
                     'RigidBody1', pass --tracker RigidBody1 and the bridge
                     renames it to --body without touching drones.yaml.
  --body NAME        rigid_body_name written into RigidBodies (default: the
                     --drone value). Must equal drones.yaml rigid_body_name.
                     Comma list, one per --tracker.
  --frame-id F       frame_id on the poses (default map).
  --stale-ms N       bridge staleness gate (default 100 = volume_guard and
                     platform timeouts). A body older than this is DROPPED.

CHECK
  --wait S           data-check window after start / for --status (default 10).
  --no-check         start and return without waiting for data.
  --status           check the running chain and the data, exit 0 if healthy,
                     1 if no data, 2 if not running.

PLUMBING
  --domain N         ROS_DOMAIN_ID (default: $ROS_DOMAIN_ID or 0).
  --attach           attach to the tmux session after starting.
  --print-plan       print the resolved commands and exit.
  --stop             stop the chain for this --drone / --gs. Idempotent.
  -h, --help

EXIT CODES
  0  chain up and data flowing (or stopped / plan printed)
  1  chain up but NO DATA in the window -- Motive/address problem, chain left
     running so it connects the moment streaming is switched on
  2  refused: bad arguments, missing package, or (--status) not running
EOF
}

# --------------------------------------------------------------------------
# argument parsing
# --------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --server)     SERVER="${2:?--server needs a value}"; shift ;;
    --server=*)   SERVER="${1#*=}" ;;
    --port)       PORT="${2:?--port needs a value}"; shift ;;
    --port=*)     PORT="${1#*=}" ;;
    --drone)      DRONE="${2:?--drone needs a value}"; shift ;;
    --drone=*)    DRONE="${1#*=}" ;;
    --tracker)    TRACKERS="${2:?--tracker needs a value}"; shift ;;
    --tracker=*)  TRACKERS="${1#*=}" ;;
    --body)       BODIES="${2:?--body needs a value}"; shift ;;
    --body=*)     BODIES="${1#*=}" ;;
    --gs)         GS_MODE="true" ;;
    --frame-id)   FRAME_ID="${2:?--frame-id needs a value}"; shift ;;
    --stale-ms)   STALE_MS="${2:?--stale-ms needs a value}"; shift ;;
    --wait)       WAIT_S="${2:?--wait needs a value}"; shift ;;
    --no-check)   NO_CHECK="true" ;;
    --domain)     DOMAIN_ID="${2:?--domain needs a value}"; shift ;;
    --attach)     ATTACH="true" ;;
    --print-plan) ACTION="plan" ;;
    --status)     ACTION="status" ;;
    --stop)       ACTION="stop" ;;
    -h|--help)    usage; exit 0 ;;
    *)            usage >&2; die "unknown argument: $1" ;;
  esac
  shift
done

[[ "${DRONE}" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]] || die "--drone '${DRONE}' is not a valid ROS namespace"
[[ "${PORT}" =~ ^[0-9]+$ ]] || die "--port must be a number (got '${PORT}')"
[[ "${WAIT_S}" =~ ^[0-9]+$ ]] || die "--wait must be an integer number of seconds"
[[ "${STALE_MS}" =~ ^[0-9]+$ ]] || die "--stale-ms must be an integer"
[[ -z "${TRACKERS}" ]] && TRACKERS="${DRONE}"
[[ -z "${BODIES}" ]] && BODIES="${DRONE}"

IFS=',' read -r -a TRACKER_ARR <<< "${TRACKERS}"
IFS=',' read -r -a BODY_ARR <<< "${BODIES}"
[[ ${#TRACKER_ARR[@]} -eq ${#BODY_ARR[@]} ]] \
  || die "--tracker has ${#TRACKER_ARR[@]} entries but --body has ${#BODY_ARR[@]}; they pair one-to-one"

# --------------------------------------------------------------------------
# derived names -- the whole topology is these five strings
# --------------------------------------------------------------------------
if [[ "${GS_MODE}" == "true" ]]; then
  NAME="gs"
  CLIENT_NS="/vrpn_mocap"
  OUT_TOPIC="/mocap/rigid_bodies"
  HEALTH_TOPIC="/mocap/health"           # bridge default; no remap
else
  NAME="${DRONE}"
  CLIENT_NS="/${DRONE}/vrpn_mocap"
  OUT_TOPIC="/${DRONE}/mocap/rigid_bodies"
  HEALTH_TOPIC="/${DRONE}/mocap/health"  # remapped from the bridge's fixed /mocap/health
fi
SESSION="mocap_${NAME}"                  # NOT o134_* on purpose: session_stop.sh must not own it
RUN_DIR="/tmp/o134_mocap_${NAME}"
CONFIG_FILE="${RUN_DIR}/config"          # what the running chain was started with
INPUT_TEMPLATE="${CLIENT_NS}/{tracker}/pose"

# --status / --stop without --server: reuse the server the chain was started with.
if [[ -z "${SERVER}" && -f "${CONFIG_FILE}" ]]; then
  SERVER="$(sed -n 's/^server=//p' "${CONFIG_FILE}" | head -1)"
  [[ -n "${SERVER}" ]] && info "using --server ${SERVER} from ${CONFIG_FILE}"
fi

config_sig() {
  printf 'server=%s\nport=%s\ntrackers=%s\nbodies=%s\nframe_id=%s\nstale_ms=%s\ndomain=%s\nrmw=%s\ngs=%s\n' \
    "${SERVER}" "${PORT}" "${TRACKERS}" "${BODIES}" "${FRAME_ID}" "${STALE_MS}" "${DOMAIN_ID}" "${RMW}" "${GS_MODE}"
}

py_list() {  # a,b,c -> ['a','b','c']
  local out="[" first="true" x
  for x in "$@"; do
    [[ "${first}" == "true" ]] || out+=","
    out+="'${x}'"; first="false"
  done
  printf '%s]' "${out}"
}
TRACKER_LIST="$(py_list "${TRACKER_ARR[@]}")"
BODY_LIST="$(py_list "${BODY_ARR[@]}")"

# pgrep patterns. They must never match this script's own command line, and
# they do not: they contain the resolved topic strings, which appear only in
# the node argv.
CLIENT_PAT="vrpn_mocap/client_node .*__ns:=${CLIENT_NS}( |\$)"
BRIDGE_PAT="vrpn_to_rigidbodies.py .*output_topic:=${OUT_TOPIC}( |\$)"

client_alive() { pgrep -f -- "${CLIENT_PAT}" >/dev/null 2>&1; }
bridge_alive() { pgrep -f -- "${BRIDGE_PAT}" >/dev/null 2>&1; }
session_up()   { tmux has-session -t "${SESSION}" 2>/dev/null; }

# --------------------------------------------------------------------------
# environment
# --------------------------------------------------------------------------
# The same prelude is used in-process (for ros2 / the check) and baked into
# the pane runners, so what the panes run is exactly what was checked.
env_prelude() {
  cat <<EOF
set +u
source "${ROS_SETUP}"
if [ -f "${AS2_WS}/setup_env.sh" ]; then source "${AS2_WS}/setup_env.sh"; \
elif [ -f "${AS2_WS}/install/setup.bash" ]; then source "${AS2_WS}/install/setup.bash"; fi
export ROS_DOMAIN_ID="${DOMAIN_ID}"
export RMW_IMPLEMENTATION="${RMW}"
${CYCLONEDDS_URI:+export CYCLONEDDS_URI=$(printf '%q' "${CYCLONEDDS_URI}")}
set -u
EOF
}

source_env() {
  # shellcheck disable=SC1090
  eval "$(env_prelude)"
}

# --------------------------------------------------------------------------
# the two commands
# --------------------------------------------------------------------------
resolve_client_bin() {
  local prefix
  prefix="$(ros2 pkg prefix vrpn_mocap 2>/dev/null)" \
    || die "ROS 2 package 'vrpn_mocap' is not installed. On this machine: sudo apt-get install ros-humble-vrpn-mocap"
  CLIENT_BIN="${prefix}/lib/vrpn_mocap/client_node"
  [[ -x "${CLIENT_BIN}" ]] || die "expected ${CLIENT_BIN} (from ros-humble-vrpn-mocap) -- not found"
}

client_cmd() {
  printf '%q --ros-args -r __ns:=%s -p server:=%s -p port:=%s -p frame_id:=%s -p update_freq:=%s -p sensor_data_qos:=true' \
    "${CLIENT_BIN}" "${CLIENT_NS}" "${SERVER}" "${PORT}" "${FRAME_ID}" "${UPDATE_HZ}"
}

bridge_cmd() {
  local remap=""
  [[ "${GS_MODE}" == "true" ]] || remap=" -r /mocap/health:=${HEALTH_TOPIC}"
  printf 'python3 %q --ros-args -p trackers:=%q -p rigid_body_names:=%q -p input_topic_template:=%q -p output_topic:=%s -p frame_id:=%s -p stale_timeout_s:=%s%s' \
    "${BRIDGE}" "${TRACKER_LIST}" "${BODY_LIST}" "${INPUT_TEMPLATE}" "${OUT_TOPIC}" "${FRAME_ID}" \
    "$(awk "BEGIN{printf \"%.3f\", ${STALE_MS}/1000.0}")" "${remap}"
}

print_summary() {
  say "----------------------------------------------------------------------"
  say "  role            $([[ "${GS_MODE}" == "true" ]] && echo 'GROUND STATION (unnamespaced)' || echo "aircraft ${DRONE}")"
  say "  VRPN server     ${SERVER:-<not given>}:${PORT}"
  say "  tracker(s)      ${TRACKERS}   -> rigid_body_name(s) ${BODIES}"
  say "  client          ${CLIENT_NS}/client   publishes ${INPUT_TEMPLATE}"
  say "  bridge          ${OUT_TOPIC}   health ${HEALTH_TOPIC}   stale ${STALE_MS} ms"
  say "  frame_id        ${FRAME_ID}"
  say "  ROS_DOMAIN_ID   ${DOMAIN_ID}   RMW ${RMW}"
  say "  tmux session    ${SESSION}      logs ${RUN_DIR}/{client,bridge}.log"
  say "----------------------------------------------------------------------"
}

# --------------------------------------------------------------------------
# reachability -- VRPN servers (Motive included) listen on TCP and UDP <port>
# --------------------------------------------------------------------------
probe_server() {
  local rc=0
  timeout 2 bash -c "exec 3<>/dev/tcp/${SERVER}/${PORT}" 2>/dev/null || rc=$?
  case "${rc}" in
    0)   good "TCP ${SERVER}:${PORT} accepts connections (a VRPN server is listening)"; return 0 ;;
    124) warn "TCP ${SERVER}:${PORT}: no answer in 2 s -- wrong address, wrong network, or the Motive PC is firewalled"; return 1 ;;
    *)   warn "TCP ${SERVER}:${PORT}: connection refused -- Motive is reachable but VRPN streaming is probably OFF"; return 1 ;;
  esac
}

# --------------------------------------------------------------------------
# the data check -- the only evidence of a live link
# --------------------------------------------------------------------------
data_check() {
  local window="$1"
  python3 - "${OUT_TOPIC}" "${CLIENT_NS}" "${window}" "${SERVER:-?}" "${PORT}" "${TRACKERS}" "${BODIES}" <<'PY'
import json, re, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mocap4r2_msgs.msg import RigidBodies
from std_msgs.msg import String

out_topic, client_ns, window, server, port, trackers, bodies = sys.argv[1:8]
window = float(window)
want_bodies = set(bodies.split(","))

rclpy.init()
n = Node("mocap_bringup_check")
qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
stamps, seen = [], set()
health = {}
def on_rb(m):
    stamps.append(time.monotonic())
    for b in m.rigidbodies: seen.add(b.rigid_body_name)
def on_health(m):
    try: health.update(json.loads(m.data))
    except Exception: pass
n.create_subscription(RigidBodies, out_topic, on_rb, qos)
health_topic = out_topic.rsplit("/", 1)[0] + "/health"
n.create_subscription(String, health_topic, on_health, 10)

t0 = time.monotonic()
while time.monotonic() - t0 < window:
    rclpy.spin_once(n, timeout_sec=0.1)

# Which trackers has the VRPN client ever heard from? Every tracker Motive
# sends becomes a PUBLISHER on <client_ns>/<name>/pose, whether or not it is
# the one we asked for. Publishers only: the bridge's own subscription makes
# the wanted topic exist regardless, so topic names alone prove nothing.
pat = re.compile("^" + re.escape(client_ns) + r"/([^/]+)/pose$")
published = sorted(m.group(1) for t, _ in n.get_topic_names_and_types()
                   for m in [pat.match(t)] if m and n.count_publishers(t) > 0)
n.destroy_node(); rclpy.shutdown()

want_trackers = set(trackers.split(","))
def line(s): print("  " + s)
if len(stamps) >= 2:
    span = stamps[-1] - stamps[0]
    rate = (len(stamps) - 1) / span if span > 0 else 0.0
    gap = max(b - a for a, b in zip(stamps, stamps[1:])) * 1000.0
    print(f"DATA OK  {out_topic}: {len(stamps)} msgs in {window:.0f} s  ->  {rate:.1f} Hz, max gap {gap:.0f} ms")
    line(f"bodies in the stream: {sorted(seen)}   (wanted: {sorted(want_bodies)})")
    line(f"VRPN trackers the client has heard from: {published}")
    rc = 0
    if not want_bodies <= seen:
        print(f"MISSING BODY: wanted {sorted(want_bodies - seen)} but the stream carries {sorted(seen)}")
        line("check --tracker against the exact rigid-body name in Motive (case included)")
        rc = 1
    if rate < 90.0:
        line(f"WARNING: {rate:.1f} Hz is below ~100 Hz. Motive frame rate? wifi? preflight_check.py mocap_rate is the gate.")
    if gap > 100.0:
        line(f"WARNING: max gap {gap:.0f} ms exceeds the 100 ms staleness gates -- expect dropouts to trip volume_guard")
    sys.exit(rc)

print(f"NO VRPN DATA within {window:.0f} s from {server}:{port} on {out_topic}")
if want_trackers & set(published):
    line(f"the VRPN client DID hear from {sorted(want_trackers & set(published))} earlier, but nothing arrived in this window.")
    line("The stream has STOPPED, not failed to start. In this order:")
    line("  1. Motive: is the rigid body still tracked (markers visible, not greyed / not disabled)?")
    line("  2. Motive: Streaming pane -> VRPN Streaming Engine still ON? Motive still running?")
    line("  3. the network between this machine and the Motive PC (see the TCP probe line above)")
    line("no restart needed: the client reconnects by itself. Fix, then re-run --status.")
elif published:
    line(f"the VRPN client IS receiving trackers {published}, but none is named in --tracker '{trackers}'")
    line("-> rename the rigid body in Motive to match, or pass --tracker <name> (and --body to keep drones.yaml happy)")
else:
    line("the VRPN client has heard from NO tracker at all. In this order:")
    line("  1. Motive: Streaming pane -> VRPN Streaming Engine is ON (Broadcast Frame Data enabled)")
    line("  2. Motive: the rigid body exists, is enabled, and its markers are visible (tracked, not greyed)")
    line("  3. the address: --server must be the Motive PC's IP as shown in the Streaming pane; port 3883")
    line("  4. this machine can reach it: ping <ip>; a firewall on the Motive PC blocks silently")
    line("the client logs NOTHING for a dead server. Fix Motive, then re-run --status; no restart needed.")
sys.exit(1)
PY
}

# --------------------------------------------------------------------------
# actions
# --------------------------------------------------------------------------
do_stop() {
  local was="false"
  if session_up; then
    was="true"
    info "interrupting ${SESSION}"
    while read -r pane; do
      [[ -n "${pane}" ]] || continue
      tmux send-keys -t "${pane}" C-c 2>/dev/null || true
    done < <(tmux list-panes -s -t "${SESSION}" -F '#{pane_id}' 2>/dev/null || true)
    sleep 1
  fi
  if client_alive || bridge_alive; then
    was="true"
    pkill -INT -f -- "${CLIENT_PAT}" 2>/dev/null || true
    pkill -INT -f -- "${BRIDGE_PAT}" 2>/dev/null || true
    local i
    for i in 1 2 3 4 5 6; do
      client_alive || bridge_alive || break
      sleep 0.5
    done
    if client_alive || bridge_alive; then
      warn "still alive after SIGINT; sending SIGKILL"
      pkill -KILL -f -- "${CLIENT_PAT}" 2>/dev/null || true
      pkill -KILL -f -- "${BRIDGE_PAT}" 2>/dev/null || true
    fi
  fi
  if session_up; then tmux kill-session -t "${SESSION}" 2>/dev/null || true; fi
  if [[ "${was}" == "true" ]]; then good "${SESSION} stopped; ${OUT_TOPIC} has no publisher from this machine"
  else info "${SESSION} was not running (nothing to do)"; fi
}

do_status() {
  print_summary
  local up="true"
  if session_up; then info "tmux session ${SESSION}: up"; else warn "tmux session ${SESSION}: NOT running"; up="false"; fi
  if client_alive; then info "vrpn_mocap client_node (${CLIENT_NS}): alive"; else warn "vrpn_mocap client_node (${CLIENT_NS}): DEAD"; up="false"; fi
  if bridge_alive; then info "vrpn_to_rigidbodies.py (${OUT_TOPIC}): alive"; else warn "vrpn_to_rigidbodies.py (${OUT_TOPIC}): DEAD"; up="false"; fi
  if [[ "${up}" != "true" ]]; then
    loud "MOCAP CHAIN NOT RUNNING on this machine (${NAME})" "" \
         "start it:  $(basename "$0") --server <motive-ip> $([[ "${GS_MODE}" == "true" ]] && echo '--gs' || echo "--drone ${DRONE}")" \
         "logs, if it died:  ${RUN_DIR}/client.log  ${RUN_DIR}/bridge.log" >&2
    exit 2
  fi
  [[ -n "${SERVER}" ]] && probe_server || true
  info "watching ${OUT_TOPIC} for ${WAIT_S} s"
  if data_check "${WAIT_S}"; then
    good "MOCAP OK (${NAME})"
    exit 0
  fi
  loud "NO MOCAP DATA (${NAME}) -- see the lines above. The chain is left running;" \
       "fix Motive / the address, then:  $(basename "$0") --status $([[ "${GS_MODE}" == "true" ]] && echo '--gs' || echo "--drone ${DRONE}")" >&2
  exit 1
}

write_runner() {  # $1 = path, $2 = label, $3 = command
  {
    printf '#!/usr/bin/env bash\n# generated by mocap_bringup.sh -- %s\n' "$2"
    env_prelude
    printf 'mkdir -p %q\n' "${RUN_DIR}"
    printf 'echo "[mocap_bringup] %s: $(date -u +%%FT%%TZ)"\n' "$2"
    printf 'echo "[mocap_bringup] %s"\n' "$(printf '%s' "$3" | sed 's/"/\\"/g')"
    printf '%s 2>&1 | tee -a %q\n' "$3" "${RUN_DIR}/$2.log"
    printf 'rc=${PIPESTATUS[0]}\n'
    printf 'echo "[mocap_bringup] %s EXITED rc=$rc -- read %s, then re-run mocap_bringup.sh"\n' "$2" "${RUN_DIR}/$2.log"
    printf 'exec sleep infinity\n'
  } > "$1"
  chmod 0755 "$1"
}

do_start() {
  [[ -n "${SERVER}" ]] || { usage >&2; die "--server <motive-ip> is required to start. Read it off Motive's Streaming pane; it is deliberately not defaulted."; }
  [[ "${SERVER}" =~ ^[A-Za-z0-9._-]+$ ]] || die "--server '${SERVER}' does not look like an address"
  command -v tmux >/dev/null 2>&1 || die "tmux is not installed"
  [[ -f "${BRIDGE}" ]] || die "bridge not found at ${BRIDGE}"
  resolve_client_bin
  python3 -c "import rclpy, mocap4r2_msgs.msg" 2>/dev/null \
    || die "python3 cannot import rclpy / mocap4r2_msgs (sudo apt-get install ros-humble-mocap4r2-msgs, and source ROS)"

  print_summary
  if [[ "${GS_MODE}" == "true" ]]; then
    info "ground-station copy: this feed is what volume_guard.py and session_bringup.sh --role ground read (/mocap/rigid_bodies)"
  fi

  local same_config="false"
  [[ -f "${CONFIG_FILE}" ]] && [[ "$(cat "${CONFIG_FILE}")" == "$(config_sig)" ]] && same_config="true"

  if session_up && client_alive && bridge_alive && [[ "${same_config}" == "true" ]]; then
    good "${SESSION} is already up with both processes alive and the same flags -- not restarting"
  else
    if session_up || client_alive || bridge_alive; then
      if [[ "${same_config}" == "true" ]]; then
        warn "${SESSION} is partly up (a process died or was swept by session_stop.sh) -- restarting it"
      else
        warn "${SESSION} is up with DIFFERENT flags (see ${CONFIG_FILE}) -- restarting it with these"
      fi
      do_stop
    fi
    mkdir -p "${RUN_DIR}"
    config_sig > "${CONFIG_FILE}"
    write_runner "${RUN_DIR}/run_client.sh" "client" "$(client_cmd)"
    write_runner "${RUN_DIR}/run_bridge.sh" "bridge" "$(bridge_cmd)"
    info "starting ${SESSION}"
    tmux new-session -d -s "${SESSION}" -n client -x 220 -y 50 "bash ${RUN_DIR}/run_client.sh"
    tmux new-window -t "${SESSION}" -n bridge "bash ${RUN_DIR}/run_bridge.sh"
    tmux select-window -t "${SESSION}:client"
    # The tmux server's GLOBAL environment is what every later session on this
    # machine inherits -- including session_bringup.sh's panes, whose
    # pre_window sets ROS_DOMAIN_ID but not RMW_IMPLEMENTATION or
    # CYCLONEDDS_URI. On a Jetson nothing in ~/.bashrc sets those (rule 6), so
    # if this script happens to be the one that creates the tmux server, make
    # sure the middleware choice travels with it. Lives only as long as the
    # tmux server; nothing on disk.
    tmux set-environment -g RMW_IMPLEMENTATION "${RMW}"
    tmux set-environment -g ROS_DOMAIN_ID "${DOMAIN_ID}"
    if [[ -n "${CYCLONEDDS_URI:-}" ]]; then tmux set-environment -g CYCLONEDDS_URI "${CYCLONEDDS_URI}"; fi
    info "tmux global env: RMW_IMPLEMENTATION=${RMW} ROS_DOMAIN_ID=${DOMAIN_ID} CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>}"
    local i
    for i in 1 2 3 4 5 6 7 8 9 10; do
      client_alive && bridge_alive && break
      sleep 0.5
    done
    client_alive || { tmux capture-pane -p -t "${SESSION}:client" 2>/dev/null | tail -15 >&2; die "client_node did not start -- see above and ${RUN_DIR}/client.log"; }
    bridge_alive || { tmux capture-pane -p -t "${SESSION}:bridge" 2>/dev/null | tail -15 >&2; die "vrpn_to_rigidbodies.py did not start -- see above and ${RUN_DIR}/bridge.log"; }
    good "both processes alive"
  fi

  say ""
  say "  attach:  tmux attach -t ${SESSION}      (Ctrl-b d to detach; Ctrl-b n next window)"
  say "  status:  $(basename "$0") --status $([[ "${GS_MODE}" == "true" ]] && echo '--gs' || echo "--drone ${DRONE}")"
  say "  stop:    $(basename "$0") --stop   $([[ "${GS_MODE}" == "true" ]] && echo '--gs' || echo "--drone ${DRONE}")"
  say ""

  probe_server || true
  if [[ "${NO_CHECK}" == "true" ]]; then
    warn "--no-check: not waiting for data. Nothing has proved the link yet."
  else
    info "waiting up to ${WAIT_S} s for data on ${OUT_TOPIC}"
    if ! data_check "${WAIT_S}"; then
      loud "NO VRPN DATA within ${WAIT_S} s from ${SERVER}:${PORT} (${NAME})" "" \
           "check Motive streaming is ON and the address is right (see the lines above)." \
           "The chain is LEFT RUNNING: the client picks the server up by itself once" \
           "streaming starts. Then:  $(basename "$0") --status $([[ "${GS_MODE}" == "true" ]] && echo '--gs' || echo "--drone ${DRONE}")" >&2
      exit 1
    fi
    good "MOCAP OK (${NAME}): ${OUT_TOPIC} is live"
  fi
  if [[ "${ATTACH}" == "true" ]]; then exec tmux attach-session -t "${SESSION}"; fi
  exit 0
}

do_plan() {
  [[ -n "${SERVER}" ]] || SERVER="<motive-ip>"
  CLIENT_BIN="$(ros2 pkg prefix vrpn_mocap 2>/dev/null || echo '<prefix>')/lib/vrpn_mocap/client_node"
  print_summary
  say "  tmux new-session -d -s ${SESSION} -n client  'bash ${RUN_DIR}/run_client.sh'"
  say "  tmux new-window  -t ${SESSION}    -n bridge  'bash ${RUN_DIR}/run_bridge.sh'"
  say ""
  say "  run_client.sh:"; say "    $(client_cmd)"
  say "  run_bridge.sh:"; say "    $(bridge_cmd)"
  say ""
  say "  then: data check on ${OUT_TOPIC} for ${WAIT_S} s (exit 1 if nothing arrives)"
  exit 0
}

# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
case "${ACTION}" in
  stop)   do_stop; exit 0 ;;
  plan)   source_env; do_plan ;;
  status) source_env; do_status ;;
  start)  source_env; do_start ;;
esac
