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
#   flight_ops/session_bringup.sh --hardware --role drone --drones drone0     # on the Jetson
#   flight_ops/session_bringup.sh --hardware --role ground --drones drone0 \
#         --mocap-server <motive-ip> --jetson-peer <jetson-ip> --guard-arm    # on the GS
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
TMUX_PROJECT_PY="${SCRIPT_DIR}/tmux_project.py"
MOCAP_BRINGUP="${SCRIPT_DIR}/deploy/mocap_bringup.sh"
GUARD_YAML="${PROJECT_DIR}/config/volume_guard.yaml"

# Overridable, but with a default that matches the machine this was built on.
AS2_WS="${AS2_WS:-${HOME}/as2_o134_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

SESSION_PREFIX="o134_"
GROUND_SESSION="${SESSION_PREFIX}ground"

# Ground-station Cyclone DDS configuration for hardware mode, and the session
# copy the launcher renders from it (interface + peers rewritten from flags).
# The copy lives in /tmp so a second shell can export the identical URI.
GS_CYCLONE_XML="${PROJECT_DIR}/config_ground_station/cyclonedds_gs.xml"
RENDERED_CYCLONE_XML="/tmp/o134_cyclonedds_gs.xml"

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
TMUX_RUNNER="auto"           # auto | tmuxinator | python -- see choose_runner()

# The mocap chain (hardware only). --mocap-server starts deploy/mocap_bringup.sh
# BEFORE the guard so /mocap/rigid_bodies exists when the guard looks for it.
MOCAP_SERVER=""              # Motive / VRPN address; never defaulted, read it off Motive
MOCAP_PORT="3883"
MOCAP_TRACKERS=""            # Motive rigid-body names; default: drones.yaml rigid_body_name

# --guard-arm gate: the guard's feed must be live at THIS rate for THIS long
# at the moment of launch, or the launcher refuses. 50 Hz is half the 100 Hz
# the chain is built for; below it the 100 ms mocap_timeout trips on jitter.
GUARD_FEED_MIN_HZ="50"
GUARD_FEED_WINDOW_S="3"

# DDS, hardware mode, ground role. See setup_middleware() for the rules.
CYCLONE_CONFIG=""            # "" = auto, "none" = leave the environment alone, or a file
GS_IFACE="eth1"              # the GS's fleet interface (WSL2 mirrored: the laptop's wifi)
# THE JETSON ADDRESS IS A DHCP LEASE. 10.88.38.136 is what jetsonorinnx
# (drone0) held on 2026-09-11 and it is the value cyclonedds_gs.xml ships
# with. When the lease moves, pass --jetson-peer; do not edit files in O-134.
JETSON_PEERS_CSV="10.88.38.136"
RMW="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

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
                        REFUSED in --hardware unless the guard's mocap topic
                        (/mocap/rigid_bodies, config/volume_guard.yaml) has a
                        live publisher at >= 50 Hz over 3 s, carrying every
                        rigid body in --drones, measured right before launch.
                        Measured 2026-09-11: with no GS-side feed the guard
                        reaches DISARM 7 s after every ground bringup; armed,
                        that force-disarms a landed aircraft. volume_guard.py
                        itself also exits 2 if no feed arrives inside its
                        startup grace, so the gate is enforced twice.

MOCAP (hardware only)
  --mocap-server IP     Start the OptiTrack chain (deploy/mocap_bringup.sh:
                        vrpn_mocap client + vrpn_to_rigidbodies.py) BEFORE the
                        stack. Ground role: the GS copy on /mocap/rigid_bodies
                        that volume_guard reads. Drone role: this aircraft's
                        copy on /<ns>/mocap/rigid_bodies that the estimator
                        reads. Read the address off Motive's Streaming pane;
                        it is deliberately not defaulted. The chain lives in
                        its own tmux session (mocap_gs / mocap_<ns>), survives
                        --stop, and is stopped with
                          deploy/mocap_bringup.sh --stop --gs
  --mocap-port N        VRPN port. Default 3883.
  --mocap-tracker LIST  Rigid-body names exactly as Motive streams them, one
                        per --drones entry, comma-separated. Default: the
                        rigid_body_name column of drones.yaml (MBD-6).
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
  --tmux-runner R       auto | tmuxinator | python. What turns the two
                        project files in lab_config/o134_project/tmuxinator/
                        into tmux sessions. auto (default) uses tmuxinator
                        when it is installed and flight_ops/tmux_project.py
                        otherwise -- a Jetson has no Ruby and no sudo, so it
                        is always the latter there. Same files, same panes;
                        parity is checked by tmux_project.py --self-test.

DDS (hardware mode, ground role -- the GS side of the wifi link)
  --jetson-peer IPS     Comma-separated addresses of the aircraft, written
                        into the Cyclone Peers block for this session.
                        Default 10.88.38.136 (drone0's DHCP lease on
                        2026-09-11 -- check it on the day: `hostname -I`
                        on the Jetson). A wrong peer degrades discovery to
                        multicast-only, which on this wifi means 0 topics.
  --gs-iface IFACE      The GS interface Cyclone is pinned to. Default eth1
                        (the laptop's wifi as WSL2 mirrored networking
                        presents it). Refused if the interface does not
                        exist.
  --cyclone-config X    FILE: render the session copy from this file instead
                        of lab_config/o134_project/config_ground_station/
                        cyclonedds_gs.xml.  none: do not touch CYCLONEDDS_URI
                        at all (the panes inherit this shell's value).
                        Default: the project file in --hardware for the
                        ground/all roles; `none` for --sim and for the drone
                        role, where ~/as2_o134_ws/setup_env.sh already
                        points at the Jetson's own cyclonedds.xml (and if it
                        is unset there, that file is used directly rather
                        than Cyclone's defaults).

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
    --mocap-server)   MOCAP_SERVER="${2:?--mocap-server needs a value}"; shift ;;
    --mocap-server=*) MOCAP_SERVER="${1#*=}" ;;
    --mocap-port)     MOCAP_PORT="${2:?--mocap-port needs a value}"; shift ;;
    --mocap-port=*)   MOCAP_PORT="${1#*=}" ;;
    --mocap-tracker)  MOCAP_TRACKERS="${2:?--mocap-tracker needs a value}"; shift ;;
    --mocap-tracker=*) MOCAP_TRACKERS="${1#*=}" ;;
    --skip-preflight) SKIP_PREFLIGHT="true" ;;
    --domain)         DOMAIN_ID="${2:?--domain needs a value}"; shift ;;
    --domain=*)       DOMAIN_ID="${1#*=}" ;;
    --log-level)      LOG_LEVEL="${2:?--log-level needs a value}"; shift ;;
    --log-level=*)    LOG_LEVEL="${1#*=}" ;;
    --settle)         SETTLE_S="${2:?--settle needs a value}"; shift ;;
    --settle=*)       SETTLE_S="${1#*=}" ;;
    --preflight-arg)  PREFLIGHT_EXTRA+=("${2:?--preflight-arg needs a value}"); shift ;;
    --jetson-peer)    JETSON_PEERS_CSV="${2:?--jetson-peer needs a value}"; shift ;;
    --jetson-peer=*)  JETSON_PEERS_CSV="${1#*=}" ;;
    --gs-iface)       GS_IFACE="${2:?--gs-iface needs a value}"; shift ;;
    --gs-iface=*)     GS_IFACE="${1#*=}" ;;
    --cyclone-config) CYCLONE_CONFIG="${2:?--cyclone-config needs a value}"; shift ;;
    --cyclone-config=*) CYCLONE_CONFIG="${1#*=}" ;;
    --attach)         ATTACH="true" ;;
    --print-plan)     PRINT_PLAN="true" ;;
    --tmux-runner)    TMUX_RUNNER="${2:?--tmux-runner needs a value}"; shift ;;
    --tmux-runner=*)  TMUX_RUNNER="${1#*=}" ;;
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

case "${TMUX_RUNNER}" in
  auto|tmuxinator|python) ;;
  *) die "--tmux-runner must be auto, tmuxinator or python (got '${TMUX_RUNNER}')" ;;
esac

[[ -f "${DRONES_YAML}" ]] || die "missing ${DRONES_YAML}"
[[ -d "${TMUXINATOR_DIR}" ]] || die "missing ${TMUXINATOR_DIR}"
[[ -f "${PREFLIGHT}" ]] || die "missing ${PREFLIGHT}"
[[ -f "${TMUX_PROJECT_PY}" ]] || die "missing ${TMUX_PROJECT_PY}"

if [[ -n "${MOCAP_SERVER}" ]]; then
  [[ "${MODE}" == "hardware" ]] || die "--mocap-server is for --hardware only. In --sim, fake_mocap.py publishes /mocap/rigid_bodies from inside the ground session."
  [[ -f "${MOCAP_BRINGUP}" ]] || die "missing ${MOCAP_BRINGUP}"
  [[ "${MOCAP_SERVER}" =~ ^[A-Za-z0-9._-]+$ ]] || die "--mocap-server '${MOCAP_SERVER}' does not look like an address"
  [[ "${MOCAP_PORT}" =~ ^[0-9]+$ ]] || die "--mocap-port must be a number"
fi

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
# middleware -- which Cyclone config every pane runs under, decided ONCE here
# --------------------------------------------------------------------------
# WHY THE LAUNCHER OWNS THIS
#
# The tmux server's global environment is what every pane inherits, and it is
# set by whichever process happened to create the server -- this script,
# deploy/mocap_bringup.sh, or a crew member's `tmux` an hour earlier. So the
# panes' RMW_IMPLEMENTATION and CYCLONEDDS_URI are NOT reliably this shell's.
# The launcher therefore computes MIDDLEWARE_ENV, a shell fragment, and both
# tmuxinator projects run it in pre_window: the panes see exactly what this
# shell decided, whoever started the server.
#
# THE RULES
#   --hardware, role ground/all   render lab_config/.../cyclonedds_gs.xml to
#                                 /tmp/o134_cyclonedds_gs.xml with --gs-iface
#                                 and --jetson-peer applied, export it.
#                                 (Measured 2026-09-11: without DontRoute the
#                                 GS sees 0-1 of the 65 /uav_0/fmu topics.)
#   --hardware, role drone        inherit. ~/as2_o134_ws/setup_env.sh has
#                                 already pointed CYCLONEDDS_URI at the
#                                 Jetson's own cyclonedds.xml; if the shell
#                                 never sourced it, use that file directly.
#   --sim                         inherit. Rig R1 is one box; the default
#                                 Cyclone config is right for it and a pinned
#                                 interface that is down would refuse to start.
#   --cyclone-config FILE|none    overrides the above.
MIDDLEWARE_ENV=""
MIDDLEWARE_NOTE=""

render_cyclone_config() {
  # $1 = source xml, $2 = destination. Rewrites the interface pin and the
  # Peers block; everything else (DontRoute, the participant cap, the
  # comments) is carried verbatim so the session copy reads like the source.
  python3 - "$1" "$2" "${GS_IFACE}" "${JETSON_PEERS_CSV}" "${MODE} ${ROLE}" <<'PY'
import datetime, re, sys
src, dst, iface, peers_csv, context = sys.argv[1:6]
peers = [p.strip() for p in peers_csv.split(",") if p.strip()]
if not peers:
    sys.exit("--jetson-peer resolved to an empty list")
for p in peers:
    if not re.fullmatch(r"[A-Za-z0-9._:-]+", p):
        sys.exit("--jetson-peer '%s' does not look like an address" % p)
if not re.fullmatch(r"[A-Za-z0-9._-]+", iface):
    sys.exit("--gs-iface '%s' does not look like an interface name" % iface)
with open(src, "r", encoding="utf-8") as fh:
    text = fh.read()
n = len(re.findall(r'<NetworkInterface\s+name="[^"]*"', text))
if n != 1:
    sys.exit("%s: expected exactly one <NetworkInterface name=...>, found %d" % (src, n))
text = re.sub(r'(<NetworkInterface\s+name=")[^"]*(")',
              lambda m: m.group(1) + iface + m.group(2), text)
peers_xml = ("<Peers>\n" + "".join('        <Peer address="%s"/>\n' % p for p in peers)
             + "      </Peers>")
text, n = re.subn(r"<Peers>.*?</Peers>", peers_xml, text, flags=re.S)
if n != 1:
    sys.exit("%s: expected exactly one <Peers> block, found %d" % (src, n))
stamp = datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
header = ("<!-- SESSION COPY rendered %s by session_bringup.sh (%s)\n"
          "     from %s\n"
          "     interface %s; peers %s.\n"
          "     Do not edit: change the source or the launcher flags. -->\n"
          % (stamp, context, src, iface, ", ".join(peers)))
lines = text.split("\n", 1)
if lines[0].startswith("<?xml"):
    text = lines[0] + "\n" + header + (lines[1] if len(lines) > 1 else "")
else:
    text = header + text
with open(dst, "w", encoding="utf-8") as fh:
    fh.write(text)
PY
}

setup_middleware() {
  local source_xml="" uri=""
  case "${CYCLONE_CONFIG}" in
    none) source_xml="" ;;
    "")   if [[ "${MODE}" == "hardware" && "${ROLE}" != "drone" ]]; then source_xml="${GS_CYCLONE_XML}"; fi ;;
    *)    source_xml="${CYCLONE_CONFIG}" ;;
  esac

  if [[ -n "${source_xml}" ]]; then
    [[ -f "${source_xml}" ]] || die "Cyclone config not found: ${source_xml}"
    if ! ip -brief link show "${GS_IFACE}" >/dev/null 2>&1; then
      die "--gs-iface ${GS_IFACE} does not exist on this machine. Interfaces: $(ip -brief link show 2>/dev/null | awk '{print $1}' | tr '\n' ' ')
Cyclone pinned to a missing interface refuses to create a single participant."
    fi
    render_cyclone_config "${source_xml}" "${RENDERED_CYCLONE_XML}" \
      || die "could not render ${source_xml} (see the message above)"
    uri="file://${RENDERED_CYCLONE_XML}"
    export CYCLONEDDS_URI="${uri}"
    MIDDLEWARE_NOTE="rendered from ${source_xml}: iface ${GS_IFACE}, peers ${JETSON_PEERS_CSV}"
  else
    uri="${CYCLONEDDS_URI:-}"
    if [[ -z "${uri}" && "${MODE}" == "hardware" && -f "${AS2_WS}/cyclonedds.xml" ]]; then
      # A Jetson whose operator did not `source setup_env.sh`. The file
      # jetson_setup.sh wrote is right there; Cyclone's defaults are not
      # (they may bind the FC link, and cap the host at ten participants).
      uri="file://${AS2_WS}/cyclonedds.xml"
      export CYCLONEDDS_URI="${uri}"
      warn "CYCLONEDDS_URI was unset; using ${uri} (source ${AS2_WS}/setup_env.sh next time)"
      MIDDLEWARE_NOTE="${AS2_WS}/cyclonedds.xml (CYCLONEDDS_URI was unset in this shell)"
    else
      MIDDLEWARE_NOTE="inherited from this shell (${uri:-CYCLONEDDS_URI unset -> Cyclone defaults})"
    fi
  fi
  export RMW_IMPLEMENTATION="${RMW}"

  # The fragment the panes run. Single-quoted so a path survives the trip
  # through tmux send-keys and an interactive shell; a quote in the value
  # would break that, so refuse one rather than ship a broken pre_window.
  [[ "${uri}" != *"'"* && "${RMW}" != *"'"* ]] || die "CYCLONEDDS_URI / RMW_IMPLEMENTATION must not contain a single quote"
  MIDDLEWARE_ENV="export RMW_IMPLEMENTATION='${RMW}'"
  if [[ -n "${uri}" ]]; then
    MIDDLEWARE_ENV+=" && export CYCLONEDDS_URI='${uri}'"
  else
    # Explicit, so a stale value in the tmux server's environment (left by
    # whoever created the server) cannot leak into the panes.
    MIDDLEWARE_ENV+=" && unset CYCLONEDDS_URI"
  fi

  # The ros2 CLI daemon caches the graph under the environment it was started
  # with. Restart it under this one so `ros2 topic list` in the crew's shell
  # shows the fleet the panes see. (hz/echo build their own node and need the
  # same export in that shell -- printed below.)
  if [[ -n "${source_xml}" && "${PRINT_PLAN}" != "true" ]]; then
    ros2 daemon stop >/dev/null 2>&1 || true
    ros2 daemon start >/dev/null 2>&1 || true
  fi
}

print_middleware() {
  rule
  say "  RMW_IMPLEMENTATION  ${RMW}"
  say "  CYCLONEDDS_URI      ${CYCLONEDDS_URI:-<unset>}"
  say "  cyclone config      ${MIDDLEWARE_NOTE}"
  if [[ "${CYCLONEDDS_URI:-}" == "file://${RENDERED_CYCLONE_XML}" ]]; then
    say "  other shells        export CYCLONEDDS_URI=${CYCLONEDDS_URI}; ros2 daemon stop; ros2 daemon start"
  fi
  rule
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
  if [[ "${MODE}" == "hardware" ]]; then
    say "  mocap chain       $([[ -n "${MOCAP_SERVER}" ]] && echo "started here from VRPN ${MOCAP_SERVER}:${MOCAP_PORT}" || echo 'NOT started here (--mocap-server) -- must already be up')"
  fi
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
# the mocap chain (hardware) and the --guard-arm feed gate
# --------------------------------------------------------------------------
# WHY THE FEED IS CHECKED HERE AND NOT LEFT TO THE GUARD
#
# In --hardware nothing in the ground session publishes /mocap/rigid_bodies:
# the estimator's feed is per aircraft (/<ns>/mocap/rigid_bodies, on the
# Jetson) and the guard's is the GS-side copy, which only exists if someone
# started it. On 2026-09-11 nobody had, and the guard -- correctly, by its
# own rules -- walked its mocap_timeout ladder to DISARM 7 s after launch.
# In dry run that is a log line. With --guard-arm it is a force-disarm of a
# landed aircraft on every bringup. So:
#
#   1. --mocap-server starts the chain first (deploy/mocap_bringup.sh), and
#   2. --guard-arm is refused unless the guard's topic is measured live --
#      a publisher, >= GUARD_FEED_MIN_HZ over GUARD_FEED_WINDOW_S, every
#      rigid body present -- right before the guard is started.
#
# volume_guard.py enforces the same rule from the inside (exit 2 if no feed
# by the end of its startup grace). Two independent checks, one failure mode.
mocap_bodies_csv() { local IFS=,; echo "${DRONE_BODY[*]}"; }

mocap_cmd_ground() {
  local bodies; bodies="$(mocap_bodies_csv)"
  printf '%q --server %q --port %q --gs --tracker %q --body %q --wait 10' \
    "${MOCAP_BRINGUP}" "${MOCAP_SERVER}" "${MOCAP_PORT}" "${MOCAP_TRACKERS:-${bodies}}" "${bodies}"
}

mocap_cmd_drone() {
  local i="$1" tracker
  if [[ -n "${MOCAP_TRACKERS}" ]]; then
    IFS=',' read -r -a _trk <<< "${MOCAP_TRACKERS}"
    tracker="${_trk[$i]:-${DRONE_BODY[$i]}}"
  else
    tracker="${DRONE_BODY[$i]}"
  fi
  printf '%q --server %q --port %q --drone %q --tracker %q --body %q --wait 10' \
    "${MOCAP_BRINGUP}" "${MOCAP_SERVER}" "${MOCAP_PORT}" "${DRONE_NS[$i]}" "${tracker}" "${DRONE_BODY[$i]}"
}

run_mocap_chain() {  # $1 = command line (already quoted), $2 = label
  info "mocap chain (${2}): ${1}"
  local rc=0
  eval "${1}" || rc=$?
  case "${rc}" in
    0) good "mocap chain (${2}) up with data" ;;
    1) warn "mocap chain (${2}) is up but NO DATA arrived. Motive streaming / address / body name."
       warn "The chain is left running and reconnects by itself. With --guard-arm the launcher will now refuse." ;;
    *) die "mocap_bringup.sh (${2}) refused (exit ${rc}); see its message above" ;;
  esac
}

start_mocap_chains() {
  [[ -n "${MOCAP_SERVER}" ]] || return 0
  local i
  if [[ "${ROLE}" != "ground" ]]; then
    for i in "${!DRONE_NS[@]}"; do run_mocap_chain "$(mocap_cmd_drone "${i}")" "${DRONE_NS[$i]}"; done
  fi
  if [[ "${ROLE}" != "drone" ]]; then
    run_mocap_chain "$(mocap_cmd_ground)" "ground station"
  fi
}

# The guard's own topic, read from the file the guard will be given, so the
# gate measures what the guard will subscribe to and not a copy of the name.
guard_mocap_topic() {
  python3 - "${GUARD_YAML}" <<'PY' 2>/dev/null || printf '/mocap/rigid_bodies'
import sys, yaml
doc = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
params = ((doc.get("/**") or {}).get("ros__parameters") or {})
print(params.get("mocap_topic") or "/mocap/rigid_bodies")
PY
}

guard_feed_gate() {
  [[ "${GUARD_ARM}" == "true" ]] || return 0
  [[ "${ROLE}" != "drone" ]] || return 0
  if [[ "${MODE}" == "sim" ]]; then
    info "--guard-arm in --sim: fake_mocap.py starts inside the ground session, so the launch-time feed gate does not apply; volume_guard.py's own grace check (exit 2 without a feed) still does."
    return 0
  fi
  local topic; topic="$(guard_mocap_topic)"
  info "--guard-arm: measuring the guard's feed ${topic} for ${GUARD_FEED_WINDOW_S} s (need a publisher, >= ${GUARD_FEED_MIN_HZ} Hz, bodies $(mocap_bodies_csv))"
  local rc=0
  python3 - "${topic}" "${GUARD_FEED_WINDOW_S}" "${GUARD_FEED_MIN_HZ}" "$(mocap_bodies_csv)" <<'PY' || rc=$?
import sys, time
topic, window, min_hz, bodies_csv = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
wanted = [b for b in bodies_csv.split(",") if b]
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from mocap4r2_msgs.msg import RigidBodies
except ImportError as exc:
    print(f"  cannot probe {topic}: {exc} (source the workspace)")
    sys.exit(1)
rclpy.init()
node = Node("o134_guard_feed_gate")
count = [0]
seen = set()
def on_msg(m):
    count[0] += 1
    for b in m.rigidbodies:
        seen.add(str(b.rigid_body_name))
qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                 reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
node.create_subscription(RigidBodies, topic, on_msg, qos)
t0 = time.monotonic()
while time.monotonic() - t0 < window:
    rclpy.spin_once(node, timeout_sec=0.05)
elapsed = time.monotonic() - t0
pubs = node.count_publishers(topic)
node.destroy_node(); rclpy.shutdown()
rate = count[0] / elapsed if elapsed > 0 else 0.0
missing = [b for b in wanted if b not in seen]
print(f"  {topic}: publishers={pubs} messages={count[0]} in {elapsed:.1f} s = {rate:.1f} Hz; bodies seen={sorted(seen)}")
problems = []
if pubs < 1:
    problems.append(f"no publisher on {topic} (nothing is producing the guard's feed on this machine)")
if rate < min_hz:
    problems.append(f"{rate:.1f} Hz is below the {min_hz:g} Hz minimum" + (" -- a publisher exists but sends nothing: Motive streaming off, wrong address, or wrong tracker name" if pubs >= 1 and count[0] == 0 else ""))
if missing:
    problems.append(f"rigid bod{'y' if len(missing) == 1 else 'ies'} {missing} never appeared (Motive name vs drones.yaml rigid_body_name, MBD-6)")
if problems:
    for p in problems:
        print(f"  REFUSED: {p}")
    sys.exit(1)
print(f"  OK: live at {rate:.1f} Hz with {sorted(seen)}")
PY
  if [[ ${rc} -ne 0 ]]; then
    loud_banner "${C_RED}" \
      "--guard-arm REFUSED: the volume guard has no live mocap feed" \
      "" \
      "An ARMED guard with no feed walks its mocap_timeout ladder to a" \
      "FORCE-DISARM 7 s after launch, aircraft on the ground or not." \
      "Nothing has been launched." \
      "" \
      "Start the feed first:  --mocap-server <motive-ip>   (or" \
      "  deploy/mocap_bringup.sh --server <motive-ip> --gs), confirm" \
      "  deploy/mocap_bringup.sh --status --gs   says MOCAP OK, then relaunch." \
      "Or drop --guard-arm for a dry-run bringup." >&2
    exit 2
  fi
  good "--guard-arm accepted: the guard's feed is live"
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
# WHICH PROGRAM TURNS A PROJECT FILE INTO A SESSION
#
# tmuxinator where it exists (the ground station), flight_ops/tmux_project.py
# where it does not (every Jetson: no Ruby, no sudo, no rubygems from the
# lab). Both take `start -n SESSION -p FILE key=value...`, both render the
# same ERB and type the same lines into the same panes; tmux_project.py's
# self-test pins that, and the two were compared line-for-line on the GS on
# 2026-09-11. What matters is that the project FILES are the single source
# and nobody has to retype a launch command on the aircraft.
RUNNER=()
RUNNER_NOTE=""
choose_runner() {
  local want="${TMUX_RUNNER}"
  if [[ "${want}" == "auto" ]]; then
    if command -v tmuxinator >/dev/null 2>&1; then want="tmuxinator"; else want="python"; fi
  fi
  case "${want}" in
    tmuxinator)
      command -v tmuxinator >/dev/null 2>&1 || die "--tmux-runner tmuxinator, but tmuxinator is not installed"
      RUNNER=(tmuxinator start)
      RUNNER_NOTE="tmuxinator $(tmuxinator version 2>/dev/null | head -1 || true)" ;;
    python)
      python3 "${TMUX_PROJECT_PY}" --self-test >/dev/null 2>&1         || die "flight_ops/tmux_project.py --self-test failed; run it to see why"
      RUNNER=(python3 "${TMUX_PROJECT_PY}" start)
      RUNNER_NOTE="flight_ops/tmux_project.py$(command -v tmuxinator >/dev/null 2>&1 || printf ' (tmuxinator is not installed)')" ;;
  esac
}

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
  local cmd=("${RUNNER[@]}" -n "${session}" -p "${TMUXINATOR_DIR}/drone.yaml"
    "drone_namespace=${ns}"
    "mode=${MODE}"
    "project_dir=${PROJECT_DIR}"
    "repo_root=${REPO_ROOT}"
    "ws_setup=${AS2_WS}/install/setup.bash"
    "ros_domain_id=${DOMAIN_ID}"
    "middleware_env=${MIDDLEWARE_ENV}"
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
  local cmd=("${RUNNER[@]}" -n "${GROUND_SESSION}" -p "${TMUXINATOR_DIR}/ground_station.yaml"
    "drones_comma=${DRONES_CSV}"
    "mode=${MODE}"
    "project_dir=${PROJECT_DIR}"
    "repo_root=${REPO_ROOT}"
    "ws_setup=${AS2_WS}/install/setup.bash"
    "ros_domain_id=${DOMAIN_ID}"
    "middleware_env=${MIDDLEWARE_ENV}"
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
setup_middleware
print_middleware

command -v tmux >/dev/null 2>&1 || die "tmux is not installed"
choose_runner
info "tmux runner: ${RUNNER_NOTE}"

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
  if [[ -n "${MOCAP_SERVER}" ]]; then
    [[ "${ROLE}" == "ground" ]] || for i in "${!DRONE_NS[@]}"; do say "  $(mocap_cmd_drone "${i}")"; done
    [[ "${ROLE}" == "drone" ]] || say "  $(mocap_cmd_ground)"
  fi
  if [[ "${GUARD_ARM}" == "true" && "${ROLE}" != "drone" && "${MODE}" == "hardware" ]]; then
    say "  [gate] --guard-arm: refuse unless $(guard_mocap_topic) is live at >= ${GUARD_FEED_MIN_HZ} Hz over ${GUARD_FEED_WINDOW_S} s with bodies $(mocap_bodies_csv)"
  fi
  [[ "${ROLE}" == "ground" ]] || for i in "${!DRONE_NS[@]}"; do start_drone_session "${i}"; done
  [[ "${ROLE}" == "drone" ]] || start_ground_session
  say ""
  say "Sessions: ${SESSIONS[*]}"
  exit 0
fi

refuse_if_running "${SESSIONS[@]}"

# The feed before the guard, and the gate before anything is launched.
start_mocap_chains
guard_feed_gate

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
if [[ -n "${MOCAP_SERVER}" ]]; then
  say "  mocap:   ${MOCAP_BRINGUP} --status $([[ "${ROLE}" == "drone" ]] && echo "--drone ${DRONE_NS[0]}" || echo '--gs')   (--stop to end it; --stop above leaves it running)"
fi
say ""

if [[ "${ATTACH}" == "true" ]]; then
  exec tmux attach-session -t "${SESSIONS[0]}"
fi

exit "${VERDICT}"
