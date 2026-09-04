#!/usr/bin/env bash
#
# verify_jetson.sh -- the onboard-computer equivalent of preflight_check.py.
#
# RUN THIS ON THE JETSON, after jetson_setup.sh.
#
# One command, one PASS/FAIL table, one verdict. It reads only: it never writes
# to the flight controller, never changes a parameter, never installs anything.
# The single exception is --with-fmu, which starts a MicroXRCEAgent it then
# kills again, because there is no other way to prove the uXRCE-DDS session.
#
# It follows the same contract as flight_ops/preflight_check.py:
#
#   * every check is independent and degrades to a FAIL with a readable reason
#     rather than a traceback;
#   * a check reports what it MEASURED, not just a boolean -- the measurement is
#     the point;
#   * an all-skipped run is NOT green. A gate that returns zero having checked
#     nothing is worse than no gate.
#
# Checks (--list-checks):
#
#   arch           aarch64
#   os             Ubuntu 22.04 jammy (the only release with Humble debs)
#   ros_install    /opt/ros/humble present
#   ros_env        ROS_DISTRO / RMW_IMPLEMENTATION / ROS_DOMAIN_ID / CYCLONEDDS_URI
#   apt_packages   every required line of packages.txt is installed
#   aerostack2     ros-humble-aerostack2 present, version reported
#   python_deps    pymavlink, fastcrc, pyserial importable
#   workspace      px4_msgs + as2_platform_pixhawk built, node binary present
#   platform_patch the O134 kill-switch and odom-staleness changes are in the source
#   xrce_agent     MicroXRCEAgent on PATH and runnable
#   fc_link        enP8p1s0 holds 10.41.10.1 and 10.41.10.2 answers
#   route          the default route is NOT the phantom 10.41.10.254
#   fleet_iface    the Cyclone interface exists, is UP, and matches cyclonedds.xml
#   gs_reach       the ground station answers            (needs --gs-host)
#   time_sync      the clock is disciplined
#   udp_port       UDP 8888 is free (or already held by our own agent)
#   fmu_topics     the /fmu/ topic set is visible        (needs --with-fmu)
#   fmu_endpoints  those topics have MATCHED endpoints   (needs --with-fmu)
#
# --fmu-prefix: three aircraft on one ROS_DOMAIN_ID with BARE /fmu/ topics all
# publish and subscribe to the SAME names. Set PX4's UXRCE_DDS_NS_IDX per
# airframe and pass the resulting prefix here (e.g. --fmu-prefix /uav_0), or
# pass "{ns}" to substitute --drone-ns. See README trap 7.5.
#
# Usage:
#   ./verify_jetson.sh --drone-ns drone0
#   ./verify_jetson.sh --drone-ns drone0 --with-fmu --gs-host 10.88.51.10
#   ./verify_jetson.sh --only "workspace xrce_agent"
#   ./verify_jetson.sh --json /tmp/verify_drone0.json
#
# Exit codes:
#   0  every non-skipped check passed  -- JETSON GREEN
#   1  at least one check FAILED       -- JETSON RED
#   2  nothing could be verified       -- JETSON UNVERIFIED
#
set -uo pipefail

readonly EXPECTED_ARCH="aarch64"
readonly EXPECTED_CODENAME="jammy"
readonly EXPECTED_ROS_DISTRO="humble"
readonly EXPECTED_RMW="rmw_cyclonedds_cpp"
readonly EXPECTED_DOMAIN="0"

readonly FC_IP="10.41.10.2"
readonly FC_LINK_IP="10.41.10.1"
readonly FC_IFACE_DEFAULT="enP8p1s0"
readonly FC_PHANTOM_GW="10.41.10.254"
readonly FLEET_IFACE_DEFAULT="wlP1p1s0"
readonly XRCE_PORT="8888"

# PX4 v1.17 dds_topics.yaml -- unversioned names, no _v1 suffix. Same list as
# preflight_check.py's DEFAULT_FMU_TOPICS, deliberately.
readonly FMU_TOPICS_OUT="/fmu/out/vehicle_odometry /fmu/out/vehicle_control_mode /fmu/out/battery_status /fmu/out/sensor_combined /fmu/out/timesync_status"
readonly FMU_TOPICS_IN="/fmu/in/trajectory_setpoint /fmu/in/offboard_control_mode /fmu/in/vehicle_command /fmu/in/vehicle_visual_odometry"

readonly ALL_CHECKS="arch os ros_install ros_env apt_packages aerostack2 python_deps workspace platform_patch xrce_agent fc_link route fleet_iface gs_reach time_sync udp_port fmu_topics fmu_endpoints"

PASS="PASS"; FAIL="FAIL"; SKIP="SKIP"
EXIT_GREEN=0; EXIT_FAIL=1; EXIT_UNVERIFIED=2

# --------------------------------------------------------------------------- #
# options
# --------------------------------------------------------------------------- #
WS="${O134_WS:-${HOME}/as2_o134_ws}"
XRCE_PREFIX="${XRCE_PREFIX:-${HOME}/xrce_agent}"
DRONE_NS="${AS2_DRONE_NS:-}"
FC_IFACE="${O134_FC_IFACE:-${FC_IFACE_DEFAULT}}"
FLEET_IFACE="${O134_FLEET_IFACE:-${FLEET_IFACE_DEFAULT}}"
GS_HOST=""
ONLY=""
SKIP_LIST=""
# Fleet-wide /fmu/ namespace. Empty is PX4's default (bare /fmu/...). Three
# aircraft on one ROS_DOMAIN_ID with bare topics COLLIDE -- see README trap 7.5.
# Accepts a literal prefix ("/uav_0") or "{ns}", substituted with --drone-ns,
# matching preflight_check.py and volume_guard.py's --fmu-prefix.
FMU_PREFIX="${O134_FMU_PREFIX:-}"
WITH_FMU=0
NO_SOURCE=0
JSON_OUT=""
FMU_WINDOW=8

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGES_FILE="${SCRIPT_DIR}/packages.txt"

usage() { sed -n '2,60p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    --drone-ns)    DRONE_NS="${2:?}"; shift 2 ;;
    --ws)          WS="${2:?}"; shift 2 ;;
    --xrce-prefix) XRCE_PREFIX="${2:?}"; shift 2 ;;
    --fc-iface)    FC_IFACE="${2:?}"; shift 2 ;;
    --fleet-iface) FLEET_IFACE="${2:?}"; shift 2 ;;
    --gs-host)     GS_HOST="${2:?}"; shift 2 ;;
    --fmu-prefix)  FMU_PREFIX="${2:?}"; shift 2 ;;
    --only)        ONLY="${2:?}"; shift 2 ;;
    --skip)        SKIP_LIST="${2:?}"; shift 2 ;;
    --with-fmu)    WITH_FMU=1; shift ;;
    --fmu-window)  FMU_WINDOW="${2:?}"; shift 2 ;;
    --no-source)   NO_SOURCE=1; shift ;;
    --json)        JSON_OUT="${2:?}"; shift 2 ;;
    --list-checks) printf '%s\n' ${ALL_CHECKS}; exit 0 ;;
    -h|--help)     usage; exit 0 ;;
    *) printf 'unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

# --------------------------------------------------------------------------- #
# results table
# --------------------------------------------------------------------------- #
US=$'\037'
ROWS=()

row() { # row <check> <target> <status> <detail...>
  local check="$1" target="$2" status="$3"; shift 3
  ROWS+=("${check}${US}${target}${US}${status}${US}$*")
}

want() {
  local c="$1"
  if [ -n "${ONLY}" ]; then case " ${ONLY} " in *" ${c} "*) return 0 ;; *) return 1 ;; esac; fi
  if [ -n "${SKIP_LIST}" ]; then case " ${SKIP_LIST} " in *" ${c} "*) return 1 ;; esac; fi
  return 0
}

render_table() {
  # Aligned, and DETAIL is never truncated -- the reason a check failed is the
  # only thing worth reading at the bench.
  local w1=5 w2=6 w3=6   # len("CHECK"), len("TARGET"), len("STATUS")
  local r c t s d
  for r in "${ROWS[@]}"; do
    IFS="${US}" read -r c t s d <<<"${r}"
    [ ${#c} -gt "${w1}" ] && w1=${#c}
    [ ${#t} -gt "${w2}" ] && w2=${#t}
    [ ${#s} -gt "${w3}" ] && w3=${#s}
  done
  printf '%-*s  %-*s  %-*s  %s\n' "${w1}" "CHECK" "${w2}" "TARGET" "${w3}" "STATUS" "DETAIL"
  printf '%s  %s  %s  %s\n' \
    "$(printf '%*s' "${w1}" '' | tr ' ' '-')" \
    "$(printf '%*s' "${w2}" '' | tr ' ' '-')" \
    "$(printf '%*s' "${w3}" '' | tr ' ' '-')" \
    "$(printf '%*s' 40 '' | tr ' ' '-')"
  for r in "${ROWS[@]}"; do
    IFS="${US}" read -r c t s d <<<"${r}"
    printf '%-*s  %-*s  %-*s  %s\n' "${w1}" "${c}" "${w2}" "${t}" "${w3}" "${s}" "${d}"
  done
}

count_status() {
  local want_s="$1" n=0 r c t s d
  for r in "${ROWS[@]}"; do
    IFS="${US}" read -r c t s d <<<"${r}"
    [ "${s}" = "${want_s}" ] && n=$((n + 1))
  done
  printf '%s' "${n}"
}

# --------------------------------------------------------------------------- #
# environment: source the operator's setup_env.sh so the checks see what a
# session sees. `set +u` first -- the ROS setup scripts read unset variables.
# --------------------------------------------------------------------------- #
SOURCE_NOTE="not attempted"
if [ "${NO_SOURCE}" -eq 0 ]; then
  if [ -f "${WS}/setup_env.sh" ]; then
    set +u
    # shellcheck disable=SC1090,SC1091
    if . "${WS}/setup_env.sh" >/dev/null 2>&1; then SOURCE_NOTE="sourced ${WS}/setup_env.sh"
    else SOURCE_NOTE="setup_env.sh exists but failed to source"; fi
    set -u
  else
    SOURCE_NOTE="no ${WS}/setup_env.sh -- run jetson_setup.sh --only env"
  fi
fi
[ -n "${DRONE_NS}" ] || DRONE_NS="${AS2_DRONE_NS:-<unset>}"

# Resolve "{ns}" the way preflight_check.py and volume_guard.py do.
case "${FMU_PREFIX}" in
  *'{ns}'*) FMU_PREFIX="$(printf '%s' "${FMU_PREFIX}" | sed "s|{ns}|${DRONE_NS}|g")" ;;
esac
fmu_topic() { printf '%s%s' "${FMU_PREFIX}" "$1"; }

# --------------------------------------------------------------------------- #
# checks
# --------------------------------------------------------------------------- #

check_arch() {
  local a; a="$(uname -m)"
  if [ "${a}" = "${EXPECTED_ARCH}" ]; then
    row arch "${a}" "${PASS}" "$(uname -srm)"
  else
    row arch "${a}" "${FAIL}" "expected ${EXPECTED_ARCH}; this is not a Jetson Orin NX (are you on the ground station?)"
  fi
}

check_os() {
  local codename="unknown" pretty="unknown"
  if [ -r /etc/os-release ]; then
    # shellcheck disable=SC1091
    codename="$( . /etc/os-release && printf '%s' "${UBUNTU_CODENAME:-${VERSION_CODENAME:-unknown}}" )"
    # shellcheck disable=SC1091
    pretty="$( . /etc/os-release && printf '%s' "${PRETTY_NAME:-unknown}" )"
  fi
  local l4t=""
  [ -r /etc/nv_tegra_release ] && l4t=" | $(head -1 /etc/nv_tegra_release | cut -c1-48)"
  if [ "${codename}" = "${EXPECTED_CODENAME}" ]; then
    row os "${codename}" "${PASS}" "${pretty}${l4t}"
  else
    row os "${codename}" "${FAIL}" "${pretty} -- ROS 2 Humble has apt binaries for ${EXPECTED_CODENAME} only"
  fi
}

check_ros_install() {
  if [ -f "/opt/ros/${EXPECTED_ROS_DISTRO}/setup.bash" ]; then
    local n; n="$(dpkg-query -W -f='${binary:Package}\n' 2>/dev/null | grep -c "^ros-${EXPECTED_ROS_DISTRO}-" || true)"
    row ros_install "/opt/ros/${EXPECTED_ROS_DISTRO}" "${PASS}" "${n} ros-${EXPECTED_ROS_DISTRO}-* packages installed"
  else
    local others; others="$(ls -1 /opt/ros 2>/dev/null | tr '\n' ' ')"
    row ros_install "/opt/ros/${EXPECTED_ROS_DISTRO}" "${FAIL}" "absent; /opt/ros holds: ${others:-nothing}"
  fi
}

check_ros_env() {
  local problems=""
  [ "${ROS_DISTRO:-}" = "${EXPECTED_ROS_DISTRO}" ] || problems="${problems} ROS_DISTRO='${ROS_DISTRO:-unset}'"
  [ "${RMW_IMPLEMENTATION:-}" = "${EXPECTED_RMW}" ] || problems="${problems} RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION:-unset}'"
  [ "${ROS_DOMAIN_ID:-}" = "${EXPECTED_DOMAIN}" ] || problems="${problems} ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-unset}'"
  if [ -z "${CYCLONEDDS_URI:-}" ]; then
    problems="${problems} CYCLONEDDS_URI=unset"
  else
    local f="${CYCLONEDDS_URI#file://}"
    [ -r "${f}" ] || problems="${problems} CYCLONEDDS_URI points at unreadable '${f}'"
  fi
  # The RMW library must actually exist, not merely be named.
  if ! ls "/opt/ros/${EXPECTED_ROS_DISTRO}/lib/librmw_cyclonedds_cpp.so" >/dev/null 2>&1; then
    problems="${problems} librmw_cyclonedds_cpp.so not installed"
  fi
  if [ -z "${problems}" ]; then
    row ros_env "${DRONE_NS}" "${PASS}" "${SOURCE_NOTE}; distro=${ROS_DISTRO} rmw=${RMW_IMPLEMENTATION} domain=${ROS_DOMAIN_ID}"
  else
    row ros_env "${DRONE_NS}" "${FAIL}" "${SOURCE_NOTE};${problems}"
  fi
}

check_apt_packages() {
  if [ ! -f "${PACKAGES_FILE}" ]; then
    row apt_packages "packages.txt" "${SKIP}" "not found at ${PACKAGES_FILE}"
    return 0
  fi
  local req opt missing_req="" missing_opt="" nreq=0 p
  req="$(sed -e 's/#.*$//' "${PACKAGES_FILE}" | grep -v '^[[:space:]]*$' | grep -v '^@optional' | awk '{print $1}')"
  opt="$(sed -e 's/#.*$//' "${PACKAGES_FILE}" | grep '^@optional' | awk '{print $2}')"
  for p in ${req}; do
    nreq=$((nreq + 1))
    dpkg-query -W -f='${db:Status-Status}' "${p}" 2>/dev/null | grep -q '^installed$' || missing_req="${missing_req} ${p}"
  done
  for p in ${opt}; do
    dpkg-query -W -f='${db:Status-Status}' "${p}" 2>/dev/null | grep -q '^installed$' || missing_opt="${missing_opt} ${p}"
  done
  if [ -n "${missing_req}" ]; then
    row apt_packages "${nreq} required" "${FAIL}" "missing:${missing_req}"
  elif [ -n "${missing_opt}" ]; then
    row apt_packages "${nreq} required" "${PASS}" "all required present; optional not installed:${missing_opt}"
  else
    row apt_packages "${nreq} required" "${PASS}" "all required and all optional present"
  fi
}

check_aerostack2() {
  local v
  v="$(dpkg-query -W -f='${Version}' ros-humble-aerostack2 2>/dev/null || true)"
  if [ -z "${v}" ]; then
    row aerostack2 "ros-humble-aerostack2" "${FAIL}" "not installed -- as2_platform_pixhawk cannot link without as2_core"
    return 0
  fi
  local arch; arch="$(dpkg-query -W -f='${Architecture}' ros-humble-aerostack2 2>/dev/null || echo '?')"
  local core=""
  [ -d "/opt/ros/${EXPECTED_ROS_DISTRO}/share/as2_core" ] && core="as2_core present"
  # The ground station is pinned at 1.1.3; a different version onboard means the
  # platform patch was reasoned against a different as2_core API.
  case "${v}" in
    1.1.3*) row aerostack2 "${arch}" "${PASS}" "${v} (matches the ground station); ${core:-as2_core MISSING}" ;;
    *)      row aerostack2 "${arch}" "${FAIL}" "${v} -- the ground station and the platform patch assume 1.1.3; see flight_ops/patches/PATCHES.md" ;;
  esac
}

check_python_deps() {
  local missing="" found=""
  local mod
  for mod in pymavlink fastcrc serial; do
    local ver
    ver="$(python3 - "${mod}" <<'PY' 2>/dev/null
import importlib, sys
m = importlib.import_module(sys.argv[1])
print(getattr(m, "__version__", "?"))
PY
)"
    if [ -n "${ver}" ]; then found="${found} ${mod}=${ver}"; else missing="${missing} ${mod}"; fi
  done
  if [ -n "${missing}" ]; then
    row python_deps "python3" "${FAIL}" "not importable:${missing} (aarch64 wheels; installed offline)"
  else
    row python_deps "python3" "${PASS}" "${found# }"
  fi
}

check_workspace() {
  local node="${WS}/install/as2_platform_pixhawk/lib/as2_platform_pixhawk/as2_platform_pixhawk_node"
  local plugin_so="${WS}/install/as2_mocap_guarded/lib/libmocap_pose_guarded.so"
  local plugin_idx="${WS}/install/as2_mocap_guarded/share/ament_index/resource_index/as2_state_estimator__pluginlib__plugin/as2_mocap_guarded"
  local problems=""
  [ -d "${WS}/src/px4_msgs" ] || problems="${problems} src/px4_msgs absent;"
  [ -d "${WS}/src/as2_platform_pixhawk" ] || problems="${problems} src/as2_platform_pixhawk absent;"
  [ -d "${WS}/src/as2_mocap_guarded" ] || problems="${problems} src/as2_mocap_guarded absent;"
  [ -f "${WS}/install/setup.bash" ] || problems="${problems} install/setup.bash absent (never built);"
  [ -d "${WS}/install/px4_msgs" ] || problems="${problems} px4_msgs not built;"
  [ -x "${node}" ] || problems="${problems} platform node binary absent;"
  [ -f "${plugin_so}" ] || problems="${problems} libmocap_pose_guarded.so absent;"
  # The .so alone is not enough: without the resource-index entry the stock
  # as2_state_estimator's ClassLoader never finds mocap_pose_guarded::Plugin,
  # and the drone quietly runs the plugin that publishes an ORIGIN POSE on a
  # rigid-body name mismatch. That is the whole reason S1 exists.
  [ -f "${plugin_idx}" ] || problems="${problems} pluginlib resource-index entry for as2_mocap_guarded absent;"
  if [ -n "${problems}" ]; then
    row workspace "${WS}" "${FAIL}" "${problems}"
    return 0
  fi
  local nmsg plat_ver guard_ver
  nmsg="$(ls -1 "${WS}/install/px4_msgs/share/px4_msgs/msg"/*.msg 2>/dev/null | wc -l || echo 0)"
  plat_ver="$(grep -m1 -oP '(?<=<version>)[^<]+' "${WS}/src/as2_platform_pixhawk/package.xml" 2>/dev/null || echo '?')"
  guard_ver="$(grep -m1 -oP '(?<=<version>)[^<]+' "${WS}/src/as2_mocap_guarded/package.xml" 2>/dev/null || echo '?')"
  row workspace "${WS}" "${PASS}" "px4_msgs ${nmsg} msg defs; as2_platform_pixhawk ${plat_ver}; as2_mocap_guarded ${guard_ver} (.so + pluginlib index); platform node present"
}

check_platform_patch() {
  local f="${WS}/src/as2_platform_pixhawk/src/pixhawk_platform.cpp"
  if [ ! -f "${f}" ]; then
    row platform_patch "pixhawk_platform.cpp" "${FAIL}" "source not found at ${f}"
    return 0
  fi
  local missing=""
  grep -q "PX4_FORCE_DISARM_MAGIC"     "${f}" || missing="${missing} kill-switch-force-disarm(change5)"
  grep -q "isExternalOdomFresh"        "${f}" || missing="${missing} odom-staleness-gate(change6)"
  grep -q "latitude_deg"               "${f}" || missing="${missing} SensorGps-rename(change1)"
  # Change 5 removed the publisher, its create_publisher call and the include.
  # If any of those is back the kill switch is a silent no-op again -- PX4 v1.17
  # does not subscribe to /fmu/in/manual_control_switches. Match the CODE, not
  # the word: the patch leaves a comment naming that topic, on purpose.
  local regressed=""
  local hdr="${WS}/src/as2_platform_pixhawk/include/as2_platform_pixhawk/pixhawk_platform.hpp"
  grep -q "px4_manual_control_switches_pub_" "${f}" 2>/dev/null \
    && regressed=" REGRESSED: the manual_control_switches publisher is back in pixhawk_platform.cpp"
  grep -q "px4_manual_control_switches_pub_" "${hdr}" 2>/dev/null \
    && regressed="${regressed} REGRESSED: the publisher member is back in pixhawk_platform.hpp"
  grep -q "include.*manual_control_switches" "${f}" "${hdr}" 2>/dev/null \
    && regressed="${regressed} REGRESSED: the ManualControlSwitches include is back"
  if [ -n "${missing}" ] || [ -n "${regressed}" ]; then
    row platform_patch "O134 patch" "${FAIL}" "missing:${missing:- none}${regressed}"
  else
    row platform_patch "O134 patch" "${PASS}" "kill switch force-disarms; external-odom staleness gate present; GPS fields renamed"
  fi
}

check_xrce_agent() {
  local bin=""
  if command -v MicroXRCEAgent >/dev/null 2>&1; then bin="$(command -v MicroXRCEAgent)"
  elif [ -x "${XRCE_PREFIX}/bin/MicroXRCEAgent" ]; then bin="${XRCE_PREFIX}/bin/MicroXRCEAgent"
  fi
  if [ -z "${bin}" ]; then
    row xrce_agent "MicroXRCEAgent" "${FAIL}" "not on PATH and not at ${XRCE_PREFIX}/bin -- it is NOT an apt package; jetson_setup.sh --only agent builds it"
    return 0
  fi
  # Run it, and test the exit status of the binary itself -- not of a pipeline
  # whose last stage is always successful.
  if ! LD_LIBRARY_PATH="${XRCE_PREFIX}/lib:${LD_LIBRARY_PATH:-}" "${bin}" --help >/dev/null 2>&1; then
    local why
    why="$(LD_LIBRARY_PATH="${XRCE_PREFIX}/lib:${LD_LIBRARY_PATH:-}" "${bin}" --help 2>&1 | head -2 | tr '\n' ' ')"
    row xrce_agent "${bin}" "${FAIL}" "will not run: ${why:-no output}; try: ldd ${bin}"
    return 0
  fi
  local miss
  miss="$(LD_LIBRARY_PATH="${XRCE_PREFIX}/lib:${LD_LIBRARY_PATH:-}" ldd "${bin}" 2>/dev/null | grep -c 'not found' || true)"
  miss="${miss//[^0-9]/}"
  if [ "${miss:-0}" -gt 0 ]; then
    row xrce_agent "${bin}" "${FAIL}" "${miss} shared libraries not found (source setup_env.sh, which sets LD_LIBRARY_PATH)"
  else
    row xrce_agent "${bin}" "${PASS}" "runs; all shared libraries resolve; invoke as: MicroXRCEAgent udp4 -p ${XRCE_PORT}"
  fi
}

check_fc_link() {
  local addr problems=""
  addr="$(ip -brief addr show "${FC_IFACE}" 2>/dev/null || true)"
  if [ -z "${addr}" ]; then
    row fc_link "${FC_IFACE}" "${FAIL}" "interface does not exist; present: $(ip -brief link show 2>/dev/null | awk '{print $1}' | tr '\n' ' ')"
    return 0
  fi
  printf '%s' "${addr}" | grep -q "${FC_LINK_IP}" || problems="${problems} ${FC_IFACE} does not hold ${FC_LINK_IP};"
  local ping_detail
  if ping -c 2 -W 2 -I "${FC_IFACE}" "${FC_IP}" >/dev/null 2>&1; then
    local rtt
    rtt="$(ping -c 3 -W 2 -I "${FC_IFACE}" "${FC_IP}" 2>/dev/null | awk -F'/' '/rtt|round-trip/{print $5" ms avg"}')"
    ping_detail="${FC_IP} replies (${rtt:-rtt n/a})"
  else
    problems="${problems} no reply from ${FC_IP};"
    ping_detail=""
  fi
  if [ -n "${problems}" ]; then
    row fc_link "${FC_IFACE}" "${FAIL}" "${problems} is the flight controller powered and is UXRCE_DDS_CFG=1000 (Ethernet)?"
  else
    row fc_link "${FC_IFACE}" "${PASS}" "$(printf '%s' "${addr}" | awk '{print $3}') -> ${ping_detail}"
  fi
}

check_route() {
  local def; def="$(ip route show default 2>/dev/null | tr '\n' '; ' || true)"
  if [ -z "${def}" ]; then
    row route "default" "${FAIL}" "no default route at all -- no apt, no NTP, no internet"
    return 0
  fi
  if printf '%s' "${def}" | grep -q "${FC_PHANTOM_GW}"; then
    row route "default" "${FAIL}" "default route via the PHANTOM gateway ${FC_PHANTOM_GW} (nothing is there; the FC link is point-to-point). All outbound traffic is black-holed. ${def}"
  else
    row route "default" "${PASS}" "${def}"
  fi
}

check_fleet_iface() {
  if ! ip -brief link show "${FLEET_IFACE}" >/dev/null 2>&1; then
    row fleet_iface "${FLEET_IFACE}" "${FAIL}" "interface does not exist; present: $(ip -brief link show 2>/dev/null | awk '{print $1}' | tr '\n' ' ')"
    return 0
  fi
  local state addr problems=""
  state="$(ip -brief link show "${FLEET_IFACE}" | awk '{print $2}')"
  addr="$(ip -brief addr show "${FLEET_IFACE}" | awk '{print $3}')"
  [ "${state}" = "UP" ] || problems="${problems} link is ${state};"
  [ -n "${addr}" ] || problems="${problems} no IPv4 address;"
  # cyclonedds.xml must name THIS interface, or Cyclone is bound elsewhere.
  local cfg="${CYCLONEDDS_URI#file://}"
  if [ -n "${cfg}" ] && [ -r "${cfg}" ]; then
    if grep -q "NetworkInterface name=\"${FLEET_IFACE}\"" "${cfg}"; then
      problems="${problems}"
    else
      problems="${problems} cyclonedds.xml does not pin ${FLEET_IFACE} (Cyclone may bind ${FC_IFACE} and never reach the ground station);"
    fi
  else
    problems="${problems} no readable CYCLONEDDS_URI config;"
  fi
  if [ -n "${problems}" ]; then
    row fleet_iface "${FLEET_IFACE}" "${FAIL}" "${problems}"
  else
    row fleet_iface "${FLEET_IFACE}" "${PASS}" "UP ${addr}; pinned in $(basename "${cfg}")"
  fi
}

check_gs_reach() {
  if [ -z "${GS_HOST}" ]; then
    row gs_reach "-" "${SKIP}" "no --gs-host given"
    return 0
  fi
  if ping -c 2 -W 2 "${GS_HOST}" >/dev/null 2>&1; then
    local rtt
    rtt="$(ping -c 3 -W 2 "${GS_HOST}" 2>/dev/null | awk -F'/' '/rtt|round-trip/{print $5" ms avg"}')"
    row gs_reach "${GS_HOST}" "${PASS}" "replies (${rtt:-rtt n/a})"
  else
    row gs_reach "${GS_HOST}" "${FAIL}" "no ICMP reply -- it may simply drop ping, but check the wifi association first"
  fi
}

check_time_sync() {
  local sync="" src="" detail=""
  if command -v timedatectl >/dev/null 2>&1; then
    sync="$(timedatectl show -p NTPSynchronized --value 2>/dev/null || true)"
    src="$(timedatectl show -p TimeUSec --value 2>/dev/null || true)"
    detail="NTPSynchronized=${sync:-unknown}"
  fi
  if command -v chronyc >/dev/null 2>&1; then
    local off
    off="$(chronyc tracking 2>/dev/null | awk -F': *' '/System time/{print $2}')"
    [ -n "${off}" ] && detail="${detail}; chrony system time ${off}"
  fi
  # The measurement that actually matters is agreement with the ground station.
  if [ -n "${GS_HOST}" ] && command -v ssh >/dev/null 2>&1; then
    local remote local_t delta
    remote="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "${GS_HOST}" 'date +%s' 2>/dev/null || true)"
    remote="${remote//[^0-9]/}"
    if [ -n "${remote}" ]; then
      local_t="$(date +%s)"
      delta=$(( local_t - remote )); [ "${delta}" -lt 0 ] && delta=$(( -delta ))
      detail="${detail}; offset vs ${GS_HOST}: ${delta} s"
      if [ "${delta}" -gt 2 ]; then
        row time_sync "clock" "${FAIL}" "${detail} -- more than 2 s from the ground station; ROS timestamps will not agree"
        return 0
      fi
    else
      detail="${detail}; ssh to ${GS_HOST} for a clock comparison did not work (BatchMode: set up key auth)"
    fi
  fi
  if [ "${sync}" = "yes" ]; then
    row time_sync "clock" "${PASS}" "${detail}"
  elif [ -z "${sync}" ]; then
    row time_sync "clock" "${SKIP}" "timedatectl unavailable; ${detail:-nothing measured}"
  else
    row time_sync "clock" "${FAIL}" "${detail} -- install/enable chrony (it is in packages.txt) and re-check"
  fi
}

check_udp_port() {
  local holder=""
  if command -v ss >/dev/null 2>&1; then
    holder="$(ss -lunp 2>/dev/null | awk -v p=":${XRCE_PORT}" '$5 ~ p {print $0}' | head -1)"
  elif command -v netstat >/dev/null 2>&1; then
    holder="$(netstat -lunp 2>/dev/null | awk -v p=":${XRCE_PORT}" '$4 ~ p {print $0}' | head -1)"
  else
    row udp_port "udp/${XRCE_PORT}" "${SKIP}" "neither ss nor netstat available"
    return 0
  fi
  if [ -z "${holder}" ]; then
    row udp_port "udp/${XRCE_PORT}" "${PASS}" "free -- MicroXRCEAgent can bind it"
  elif printf '%s' "${holder}" | grep -q 'MicroXRCEAgent'; then
    row udp_port "udp/${XRCE_PORT}" "${PASS}" "already held by a running MicroXRCEAgent"
  else
    row udp_port "udp/${XRCE_PORT}" "${FAIL}" "held by something else: ${holder}"
  fi
}

# --- the two checks that need the DDS session up ---------------------------
FMU_AGENT_PID=""
FMU_TOPIC_LIST=""

fmu_bring_up() {
  # Returns 0 when a topic list was obtained, 1 otherwise. Starts an agent only
  # if one is not already running, and kills only what it started.
  command -v ros2 >/dev/null 2>&1 || { FMU_ERR="ros2 CLI not on PATH -- source setup_env.sh"; return 1; }
  local bin=""
  if command -v MicroXRCEAgent >/dev/null 2>&1; then bin="$(command -v MicroXRCEAgent)"
  elif [ -x "${XRCE_PREFIX}/bin/MicroXRCEAgent" ]; then bin="${XRCE_PREFIX}/bin/MicroXRCEAgent"; fi

  if pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
    FMU_NOTE="using the MicroXRCEAgent already running (pid $(pgrep -x MicroXRCEAgent | head -1))"
  elif [ -n "${bin}" ]; then
    LD_LIBRARY_PATH="${XRCE_PREFIX}/lib:${LD_LIBRARY_PATH:-}" \
      "${bin}" udp4 -p "${XRCE_PORT}" >"/tmp/verify_jetson_agent.$$.log" 2>&1 &
    FMU_AGENT_PID=$!
    FMU_NOTE="started MicroXRCEAgent pid ${FMU_AGENT_PID} for ${FMU_WINDOW} s"
    sleep 3
    if ! kill -0 "${FMU_AGENT_PID}" 2>/dev/null; then
      FMU_ERR="the agent exited immediately: $(head -3 "/tmp/verify_jetson_agent.$$.log" | tr '\n' ' ')"
      FMU_AGENT_PID=""
      return 1
    fi
  else
    FMU_ERR="no MicroXRCEAgent to start"
    return 1
  fi

  # Discovery needs a moment, and PX4 only creates its topics once the client
  # has a session.
  sleep "${FMU_WINDOW}"
  FMU_TOPIC_LIST="$(ros2 topic list 2>/dev/null || true)"
  [ -n "${FMU_TOPIC_LIST}" ] || { FMU_ERR="ros2 topic list returned nothing"; return 1; }
  return 0
}

fmu_tear_down() {
  if [ -n "${FMU_AGENT_PID}" ]; then
    kill "${FMU_AGENT_PID}" 2>/dev/null || true
    wait "${FMU_AGENT_PID}" 2>/dev/null || true
    rm -f "/tmp/verify_jetson_agent.$$.log"
  fi
}

check_fmu_topics() {
  if [ "${WITH_FMU}" -eq 0 ]; then
    row fmu_topics "-" "${SKIP}" "not requested; re-run with --with-fmu and the flight controller powered"
    return 0
  fi
  if [ -z "${FMU_TOPIC_LIST}" ]; then
    row fmu_topics "-" "${FAIL}" "${FMU_ERR:-no topic list}"
    return 0
  fi
  local missing="" t full
  for t in ${FMU_TOPICS_OUT} ${FMU_TOPICS_IN}; do
    full="$(fmu_topic "${t}")"
    printf '%s\n' "${FMU_TOPIC_LIST}" | grep -qx "${full}" || missing="${missing} ${full}"
  done
  local n_fmu
  n_fmu="$(printf '%s\n' "${FMU_TOPIC_LIST}" | grep -c "^${FMU_PREFIX}/fmu/" || true)"
  n_fmu="${n_fmu//[^0-9]/}"
  if [ -n "${missing}" ]; then
    local others
    others="$(printf '%s\n' "${FMU_TOPIC_LIST}" | grep '/fmu/' | head -3 | tr '\n' ' ')"
    row fmu_topics "${FMU_PREFIX:-<bare>}/fmu/" "${FAIL}" \
      "${n_fmu:-0} topics under this prefix; missing:${missing}; /fmu/ topics that DO exist: ${others:-none} -- if those carry a namespace, pass --fmu-prefix (${FMU_NOTE:-})"
  else
    row fmu_topics "${FMU_PREFIX:-<bare>}/fmu/" "${PASS}" "${n_fmu} topics under this prefix; all 9 required present (${FMU_NOTE:-})"
  fi
}

check_fmu_endpoints() {
  # A topic can EXIST because the agent advertised it while the flight
  # controller never matched. Existence is not the check; matched endpoints are.
  if [ "${WITH_FMU}" -eq 0 ]; then
    row fmu_endpoints "-" "${SKIP}" "not requested; needs --with-fmu"
    return 0
  fi
  if [ -z "${FMU_TOPIC_LIST}" ]; then
    row fmu_endpoints "-" "${FAIL}" "${FMU_ERR:-no topic list}"
    return 0
  fi
  local unmatched="" ok_count=0 t info raw n
  local full
  for t in ${FMU_TOPICS_OUT}; do
    full="$(fmu_topic "${t}")"
    info="$(ros2 topic info "${full}" 2>/dev/null || true)"
    raw="$(printf '%s' "${info}" | awk -F': *' '/Publisher count/{print $2}')"
    n="${raw//[^0-9]/}"          # `ros2 topic info` on a missing topic prints nothing
    if [ "${n:-0}" -ge 1 ]; then ok_count=$((ok_count + 1)); else unmatched="${unmatched} ${full}(pub=${raw:-none})"; fi
  done
  for t in ${FMU_TOPICS_IN}; do
    full="$(fmu_topic "${t}")"
    info="$(ros2 topic info "${full}" 2>/dev/null || true)"
    raw="$(printf '%s' "${info}" | awk -F': *' '/Subscription count/{print $2}')"
    n="${raw//[^0-9]/}"
    if [ "${n:-0}" -ge 1 ]; then ok_count=$((ok_count + 1)); else unmatched="${unmatched} ${full}(sub=${raw:-none})"; fi
  done
  # And prove data actually moves on the one topic that always should.
  local hz=""
  hz="$(timeout 6 ros2 topic hz "$(fmu_topic /fmu/out/timesync_status)" 2>/dev/null | awk '/average rate/{print $3; exit}')"
  if [ -n "${unmatched}" ]; then
    row fmu_endpoints "matched" "${FAIL}" "${ok_count}/9 matched; NOT matched:${unmatched}. The uXRCE-DDS session exists but the endpoints did not pair -- check UXRCE_DDS_AG_IP=170461697, UXRCE_DDS_PRT=8888, UXRCE_DDS_KEY=1, UXRCE_DDS_DOM_ID=0, and UXRCE_DDS_NS_IDX against the --fmu-prefix in use (${FMU_PREFIX:-<bare>})."
  else
    row fmu_endpoints "matched" "${PASS}" "9/9 matched; timesync_status ${hz:-no rate measured} Hz"
  fi
}

# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
printf '%s\n' "=============================================================="
printf 'O134 Jetson verification   %s\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
printf 'host %s   ns %s   ws %s\n' "$(hostname)" "${DRONE_NS}" "${WS}"
printf '%s\n' "=============================================================="
printf '\n'

# Bring the DDS session up once, before the two checks that need it.
FMU_ERR=""; FMU_NOTE=""
if [ "${WITH_FMU}" -eq 1 ] && { want fmu_topics || want fmu_endpoints; }; then
  printf 'starting the uXRCE-DDS session for the /fmu/ checks (~%s s)...\n\n' "$((FMU_WINDOW + 3))"
  fmu_bring_up || true
fi
trap fmu_tear_down EXIT

for c in ${ALL_CHECKS}; do
  if want "${c}"; then
    "check_${c}" || row "${c}" "-" "${FAIL}" "the check itself raised an error (this is a bug in verify_jetson.sh)"
  fi
done

fmu_tear_down
trap - EXIT

render_table
printf '\n'

n_pass="$(count_status "${PASS}")"
n_fail="$(count_status "${FAIL}")"
n_skip="$(count_status "${SKIP}")"

if [ "${n_fail}" -gt 0 ]; then
  code="${EXIT_FAIL}"
  verdict="JETSON RED         ${n_pass} passed, ${n_fail} failed, ${n_skip} skipped   DO NOT PUT THIS AIRCRAFT ON THE CARD"
elif [ "${n_pass}" -eq 0 ]; then
  code="${EXIT_UNVERIFIED}"
  verdict="JETSON UNVERIFIED  ${n_pass} passed, ${n_fail} failed, ${n_skip} skipped   NOTHING WAS CHECKED"
else
  code="${EXIT_GREEN}"
  verdict="JETSON GREEN       ${n_pass} passed, ${n_fail} failed, ${n_skip} skipped"
  [ "${WITH_FMU}" -eq 0 ] && verdict="${verdict}   (no --with-fmu: the uXRCE-DDS session is UNPROVEN)"
  # GREEN on a hand-picked subset is not GREEN on the aircraft. Say so, so that
  # a screenshot of this line cannot be mistaken for a full pass.
  [ -n "${ONLY}" ]      && verdict="${verdict}   (SUBSET: --only '${ONLY}' -- not a full verification)"
  [ -n "${SKIP_LIST}" ] && verdict="${verdict}   (SUBSET: --skip '${SKIP_LIST}' -- not a full verification)"
fi
printf '%s\n' "${verdict}"

if [ -n "${JSON_OUT}" ]; then
  {
    printf '{\n  "host": "%s",\n  "drone_ns": "%s",\n  "utc": "%s",\n  "verdict": "%s",\n' \
      "$(hostname)" "${DRONE_NS}" "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$(printf '%s' "${verdict}" | sed 's/"/\\"/g')"
    printf '  "pass": %s, "fail": %s, "skip": %s,\n  "checks": [\n' "${n_pass}" "${n_fail}" "${n_skip}"
    first=1
    for r in "${ROWS[@]}"; do
      IFS="${US}" read -r c t s d <<<"${r}"
      [ "${first}" -eq 1 ] || printf ',\n'
      first=0
      printf '    {"check": "%s", "target": "%s", "status": "%s", "detail": "%s"}' \
        "${c}" "$(printf '%s' "${t}" | sed 's/"/\\"/g')" "${s}" "$(printf '%s' "${d}" | sed 's/\\/\\\\/g; s/"/\\"/g')"
    done
    printf '\n  ]\n}\n'
  } > "${JSON_OUT}"
  printf 'json written: %s\n' "${JSON_OUT}"
fi

exit "${code}"
