#!/usr/bin/env bash
#
# jetson_setup.sh -- bring one Jetson Orin NX from a stock JetPack image to a
# flight-ready O134 onboard computer.
#
# RUN THIS ON THE JETSON. Not on the ground station.
#
#   ssh jetson@10.88.51.230
#   tar -xzf o134_bundle_<stamp>.tar.gz -C ~
#   ~/o134_bundle_<stamp>/flight_ops/deploy/jetson_setup.sh --drone-ns drone0
#
# What it does, in seven phases. Each phase is independently re-runnable and
# reports what it FOUND before it changes anything:
#
#   0 preflight   architecture, OS release, user, sudo, free disk
#   1 network     interfaces, routes, DNS, FC reachability, internet reachability
#   2 apt         ROS 2 apt source + every package in packages.txt
#   3 workspace   unpack px4_msgs + patched as2_platform_pixhawk into ~/as2_o134_ws
#   4 build       colcon build the workspace
#   5 agent       build Micro-XRCE-DDS-Agent from bundled source
#   6 env         write setup_env.sh, cyclonedds.xml and a rollback script
#
# DESIGN RULES THIS SCRIPT OBEYS
#
#   * It never assumes. Architecture, OS, existing ROS, and network are all
#     measured and printed, and a mismatch is a loud stop, not a guess.
#   * It is idempotent. A phase that is already satisfied prints "already done"
#     and moves on. Re-running after a failure is the normal recovery.
#   * NO PERSISTENT HOOKS. Nothing is appended to ~/.bashrc, no systemd unit is
#     installed, no cron job is created. The operator sources
#     ~/as2_o134_ws/setup_env.sh once per session, by hand.
#   * No password appears anywhere in this repository. `sudo` will prompt you
#     at the terminal; that is deliberate. Set up ssh KEY auth to the Jetson
#     before you start (ssh-copy-id), never a password in a script.
#   * Everything it creates lives under two removable directories:
#     ~/as2_o134_ws and ~/xrce_agent, plus the apt packages it records in
#     ~/as2_o134_ws/.deploy/apt_installed_by_deploy.txt.
#
# Exit codes:
#   0  every requested phase completed
#   1  a phase failed -- read the last PHASE line, fix, re-run
#   2  the machine is not a valid target (wrong arch / wrong Ubuntu / no sudo)
#
set -euo pipefail

# --------------------------------------------------------------------------- #
# constants -- the known environment, from flight_ops/lab_config and the FC
# --------------------------------------------------------------------------- #
readonly EXPECTED_ARCH="aarch64"
readonly EXPECTED_CODENAME="jammy"
readonly EXPECTED_ROS_DISTRO="humble"

readonly FC_IP="10.41.10.2"          # PX4 over the point-to-point Ethernet link
readonly FC_LINK_IP="10.41.10.1"     # this Jetson's address on that link
readonly FC_IFACE_DEFAULT="enP8p1s0"
readonly FC_PHANTOM_GW="10.41.10.254"  # advertised, does not exist, eats the default route
readonly XRCE_PORT="8888"

readonly FLEET_IFACE_DEFAULT="wlP1p1s0"   # wifi -- the only route with internet
readonly ROS_DOMAIN_ID_DEFAULT="0"
readonly RMW_DEFAULT="rmw_cyclonedds_cpp"

readonly XRCE_TAG="v2.4.2"           # matched to PX4 v1.17.0's uxrce_dds_client
readonly PX4_MSGS_BRANCH="release/1.17"
readonly PLATFORM_BASE_COMMIT="2b00b77dcd2a4e3f7f607ef043bc8cb85e215b88"

readonly ALL_PHASES="preflight network apt workspace build agent env"

# --------------------------------------------------------------------------- #
# options
# --------------------------------------------------------------------------- #
WS="${HOME}/as2_o134_ws"
XRCE_PREFIX="${HOME}/xrce_agent"
BUNDLE_DIR=""
DRONE_NS=""
FC_IFACE="${FC_IFACE_DEFAULT}"
FLEET_IFACE="${FLEET_IFACE_DEFAULT}"
GS_HOST=""
PEERS=""
# Fleet-wide /fmu/ topic namespace, set on the FC by PX4's UXRCE_DDS_NS_IDX.
# Empty means bare /fmu/..., which COLLIDES across three aircraft on one
# ROS_DOMAIN_ID. See README trap 7.5. Written into setup_env.sh so the platform
# node, volume_guard and verify_jetson.sh all read one value.
FMU_PREFIX=""
ONLY=""
SKIP=""
OFFLINE=0
DRY_RUN=0
ASSUME_YES=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGES_FILE="${SCRIPT_DIR}/packages.txt"

# Filled in by hard_guards(), read by phase_preflight().
GUARD_CODENAME=""
GUARD_PRETTY=""
NET_OK=0

usage() {
  sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  cat <<'EOF'

Options:
  --drone-ns NS        REQUIRED. Aerostack2 namespace for this aircraft
                       (drone0 | drone1 | drone2). Recorded in setup_env.sh.
  --bundle DIR         Directory produced by bundle_for_jetson.sh. Defaults to
                       the parent of this script's parent (i.e. the unpacked
                       bundle root) when that looks like a bundle.
  --ws DIR             Workspace root. Default ~/as2_o134_ws
  --xrce-prefix DIR    Micro-XRCE-DDS-Agent install prefix. Default ~/xrce_agent
  --fc-iface NAME      Ethernet interface to the flight controller. Default enP8p1s0
  --fleet-iface NAME   Interface Cyclone DDS binds to. Default wlP1p1s0
  --gs-host ADDR       Ground station address; added as a Cyclone unicast peer
                       and probed in phase 1.
  --peer ADDR          Extra Cyclone unicast peer (repeatable; the other drones).
  --fmu-prefix PFX     Namespace PX4 publishes its /fmu/ topics under, from the
                       FC's UXRCE_DDS_NS_IDX (e.g. /uav_0). Default: bare /fmu/,
                       which is only safe with ONE aircraft on the domain.
                       Exported as O134_FMU_PREFIX by setup_env.sh.
  --only  "p1 p2"      Run only these phases.
  --skip  "p1 p2"      Run everything except these phases.
  --offline            Do not touch the network for downloads. Phase 2 installs
                       only from bundled .deb files; fails loudly if absent.
  --dry-run            Print what each phase would do; change nothing.
  --yes                Do not pause for confirmation before the apt phase.
  -h, --help           This text.

Phases: preflight network apt workspace build agent env

Examples:
  # normal first run on drone0, bundle unpacked at ~/o134_bundle_20260904T0900Z
  ~/o134_bundle_20260904T0900Z/flight_ops/deploy/jetson_setup.sh --drone-ns drone0 \
      --gs-host 10.88.51.10 --peer 10.88.51.231 --peer 10.88.51.232

  # rebuild the workspace only, after editing a patch
  ./jetson_setup.sh --drone-ns drone0 --only "workspace build"

  # look before you leap
  ./jetson_setup.sh --drone-ns drone0 --dry-run
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --drone-ns)     DRONE_NS="${2:?--drone-ns needs a value}"; shift 2 ;;
    --bundle)       BUNDLE_DIR="${2:?--bundle needs a value}"; shift 2 ;;
    --ws)           WS="${2:?--ws needs a value}"; shift 2 ;;
    --xrce-prefix)  XRCE_PREFIX="${2:?--xrce-prefix needs a value}"; shift 2 ;;
    --fc-iface)     FC_IFACE="${2:?--fc-iface needs a value}"; shift 2 ;;
    --fleet-iface)  FLEET_IFACE="${2:?--fleet-iface needs a value}"; shift 2 ;;
    --gs-host)      GS_HOST="${2:?--gs-host needs a value}"; shift 2 ;;
    --fmu-prefix)   FMU_PREFIX="${2:?--fmu-prefix needs a value}"; shift 2 ;;
    --peer)         PEERS="${PEERS} ${2:?--peer needs a value}"; shift 2 ;;
    --only)         ONLY="${2:?--only needs a value}"; shift 2 ;;
    --skip)         SKIP="${2:?--skip needs a value}"; shift 2 ;;
    --offline)      OFFLINE=1; shift ;;
    --dry-run)      DRY_RUN=1; shift ;;
    --yes|-y)       ASSUME_YES=1; shift ;;
    -h|--help)      usage; exit 0 ;;
    *) printf 'unknown option: %s\n\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

# --------------------------------------------------------------------------- #
# output helpers -- one voice, greppable
# --------------------------------------------------------------------------- #
if [ -t 1 ]; then
  C_OK=$'\033[32m'; C_BAD=$'\033[31m'; C_WARN=$'\033[33m'; C_DIM=$'\033[2m'; C_OFF=$'\033[0m'
else
  C_OK=""; C_BAD=""; C_WARN=""; C_DIM=""; C_OFF=""
fi

LOGFILE=""
log()   { printf '[setup] %s\n' "$*"; [ -n "${LOGFILE}" ] && printf '[setup] %s\n' "$*" >>"${LOGFILE}" || true; }
ok()    { printf '  %sOK%s    %s\n' "${C_OK}" "${C_OFF}" "$*"; }
warn()  { printf '  %sWARN%s  %s\n' "${C_WARN}" "${C_OFF}" "$*"; }
bad()   { printf '  %sFAIL%s  %s\n' "${C_BAD}" "${C_OFF}" "$*"; }
info()  { printf '  %s.%s     %s\n' "${C_DIM}" "${C_OFF}" "$*"; }
die()   { printf '\n[setup] %sSTOP%s  %s\n' "${C_BAD}" "${C_OFF}" "$*" >&2; exit "${2:-1}"; }

phase_banner() {
  printf '\n%s\n' "=============================================================="
  printf 'PHASE %s\n' "$1"
  printf '%s\n' "=============================================================="
}

run() {
  # Execute, or print under --dry-run. Every mutating command goes through this.
  if [ "${DRY_RUN}" -eq 1 ]; then
    printf '  %sDRY%s   %s\n' "${C_DIM}" "${C_OFF}" "$*"
    return 0
  fi
  "$@"
}

want_phase() {
  local p="$1"
  if [ -n "${ONLY}" ]; then
    case " ${ONLY} " in *" ${p} "*) return 0 ;; *) return 1 ;; esac
  fi
  if [ -n "${SKIP}" ]; then
    case " ${SKIP} " in *" ${p} "*) return 1 ;; esac
  fi
  return 0
}

confirm() {
  [ "${ASSUME_YES}" -eq 1 ] && return 0
  [ "${DRY_RUN}" -eq 1 ] && return 0
  printf '  %s [y/N] ' "$1"
  local reply=""
  read -r reply || true
  case "${reply}" in y|Y|yes|YES) return 0 ;; *) return 1 ;; esac
}

# --------------------------------------------------------------------------- #
# packages.txt parsing
# --------------------------------------------------------------------------- #
required_packages() {
  # stdout: one required package name per line
  sed -e 's/#.*$//' -e 's/[[:space:]]*$//' "${PACKAGES_FILE}" \
    | grep -v '^[[:space:]]*$' \
    | grep -v '^@optional' \
    | awk '{print $1}'
}

optional_packages() {
  sed -e 's/#.*$//' -e 's/[[:space:]]*$//' "${PACKAGES_FILE}" \
    | grep '^@optional' \
    | awk '{print $2}'
}

pkg_installed() { dpkg-query -W -f='${db:Status-Status}' "$1" 2>/dev/null | grep -q '^installed$'; }

# --------------------------------------------------------------------------- #
# hard guards -- run BEFORE any phase, and independently of --only / --skip.
#
# These used to live inside phase_preflight, which meant `--only build` on the
# ground station skipped the architecture check and went straight to colcon.
# A guard you can opt out of is not a guard.
# --------------------------------------------------------------------------- #
hard_guards() {
  local arch; arch="$(uname -m)"
  if [ "${arch}" != "${EXPECTED_ARCH}" ]; then
    printf '  %sFAIL%s  architecture is %s, expected %s\n' "${C_BAD}" "${C_OFF}" "${arch}" "${EXPECTED_ARCH}"
    die "This script runs ON the Jetson. You appear to be on the ground station.
      If you meant to build a delivery bundle, run flight_ops/deploy/bundle_for_jetson.sh instead." 2
  fi
  if [ "$(id -u)" -eq 0 ]; then
    die "Do not run this as root. Run as the 'jetson' user; sudo is invoked per command
      so that the workspace ends up owned by the operator, not by root." 2
  fi
  [ -n "${DRONE_NS}" ] || die "--drone-ns is required (drone0 | drone1 | drone2).
      It is written into setup_env.sh and it is how the three aircraft stay distinct." 2

  # OS release. ROS 2 Humble has apt binaries for jammy ONLY. JetPack 5 ships
  # Ubuntu 20.04 (focal) and there is no Humble deb for it -- a hard stop with
  # three real options, not something to work around silently.
  GUARD_CODENAME=""; GUARD_PRETTY=""
  if [ -r /etc/os-release ]; then
    # shellcheck disable=SC1091
    GUARD_CODENAME="$( . /etc/os-release && printf '%s' "${UBUNTU_CODENAME:-${VERSION_CODENAME:-unknown}}" )"
    # shellcheck disable=SC1091
    GUARD_PRETTY="$( . /etc/os-release && printf '%s' "${PRETTY_NAME:-unknown}" )"
  fi
  if [ "${GUARD_CODENAME}" != "${EXPECTED_CODENAME}" ]; then
    printf '  %sFAIL%s  Ubuntu codename is %s, expected %s (22.04)\n' \
      "${C_BAD}" "${C_OFF}" "'${GUARD_CODENAME}'" "'${EXPECTED_CODENAME}'"
    die "ROS 2 Humble has apt binaries for jammy only. On this image you have three options:
        1. Flash JetPack 6.x (Ubuntu 22.04) -- the intended path, and the only one
           that keeps this kit's verification meaningful.
        2. Run the stack in a container (e.g. dustynv/ros:humble-ros-base-l4t-*).
           Then every step below happens INSIDE the container and the network
           namespace has to be host-mode for the FC link to work.
        3. Build ROS 2 Humble from source on focal. Hours, and the resulting
           tree will not match the ground station's apt build. Not recommended.
      Nothing was changed." 2
  fi

  # Bundle discovery, here rather than in phase_preflight so that `--only
  # workspace` finds the bundle it is standing in.
  if [ -z "${BUNDLE_DIR}" ]; then
    local candidate; candidate="$(cd "${SCRIPT_DIR}/../.." && pwd)"
    [ -f "${candidate}/BUNDLE_INFO.txt" ] && BUNDLE_DIR="${candidate}"
  fi
}

# --------------------------------------------------------------------------- #
# PHASE 0 -- preflight. Measure the machine and report it.
# --------------------------------------------------------------------------- #
phase_preflight() {
  phase_banner "0  preflight -- what machine is this?"

  ok "architecture $(uname -m)"

  # The codename was already enforced by hard_guards(); report what it found.
  info "os-release          : ${GUARD_PRETTY}"
  ok "Ubuntu 22.04 (${GUARD_CODENAME})"

  # JetPack / L4T identification, purely informational but it belongs in the log.
  if [ -r /etc/nv_tegra_release ]; then
    info "L4T                 : $(head -1 /etc/nv_tegra_release)"
  else
    warn "/etc/nv_tegra_release absent -- this may not be a Jetson"
  fi
  if [ -r /proc/device-tree/model ]; then
    info "board               : $(tr -d '\0' < /proc/device-tree/model)"
  fi

  info "user                : $(id -un) (uid $(id -u))"

  # sudo, without ever storing a password.
  if sudo -n true 2>/dev/null; then
    ok "sudo available (cached credential)"
  elif [ "${DRY_RUN}" -eq 1 ]; then
    info "sudo                : not checked under --dry-run"
  else
    log "sudo will prompt now so later phases do not stall mid-build."
    if sudo -v; then ok "sudo available"; else die "no sudo -- cannot install packages" 2; fi
  fi

  # Disk. ROS base + aerostack2 + a colcon build of px4_msgs is ~6 GB.
  local avail_kb avail_gb
  avail_kb="$(df -Pk "${HOME}" | awk 'NR==2{print $4}')"
  avail_gb=$(( avail_kb / 1024 / 1024 ))
  info "free on \$HOME       : ${avail_gb} GiB"
  if [ "${avail_gb}" -lt 12 ]; then
    warn "under 12 GiB free. ROS 2 base + Aerostack2 + a px4_msgs build needs ~8-10 GiB."
  else
    ok "disk space"
  fi

  # Existing ROS. Report, never clobber.
  if [ -d /opt/ros ]; then
    local found; found="$(ls -1 /opt/ros 2>/dev/null | tr '\n' ' ')"
    info "existing /opt/ros   : ${found}"
    if [ -f "/opt/ros/${EXPECTED_ROS_DISTRO}/setup.bash" ]; then
      ok "ROS 2 ${EXPECTED_ROS_DISTRO} already installed"
    else
      warn "a ROS distro is installed but not ${EXPECTED_ROS_DISTRO}: '${found}'"
      warn "phase 2 will ADD humble alongside it. Sourcing two distros in one shell breaks things --"
      warn "setup_env.sh sources humble only, so use it and nothing else."
    fi
  else
    info "existing /opt/ros   : none"
  fi

  # Bundle: discovered by hard_guards(); report it.
  if [ -n "${BUNDLE_DIR}" ] && [ -f "${BUNDLE_DIR}/BUNDLE_INFO.txt" ]; then
    ok "bundle              : ${BUNDLE_DIR}"
    sed 's/^/        /' "${BUNDLE_DIR}/BUNDLE_INFO.txt" | head -20
  else
    warn "no bundle found. Phases 'workspace' and 'agent' need one."
    warn "Produce it on the ground station:  flight_ops/deploy/bundle_for_jetson.sh"
  fi

  ok "drone namespace     : ${DRONE_NS}"
}

# --------------------------------------------------------------------------- #
# PHASE 1 -- network. The single most common reason a Jetson "cannot reach
# github". Diagnose, report, do NOT silently rewrite the operator's routing.
# --------------------------------------------------------------------------- #
phase_network() {
  phase_banner "1  network -- where can this machine actually go?"

  info "interfaces:"
  ip -brief addr show 2>/dev/null | sed 's/^/        /' || warn "ip(8) unavailable"

  info "routes:"
  ip route show 2>/dev/null | sed 's/^/        /' || true

  # 1a. the FC link
  if ip -brief addr show "${FC_IFACE}" 2>/dev/null | grep -q "${FC_LINK_IP}"; then
    ok "${FC_IFACE} carries ${FC_LINK_IP}"
  else
    warn "${FC_IFACE} does not carry ${FC_LINK_IP}."
    warn "  Expected the point-to-point link to PX4: Jetson ${FC_LINK_IP}, FC ${FC_IP}."
    warn "  Set it for this session only (no persistent hook):"
    warn "    sudo ip addr add ${FC_LINK_IP}/24 dev ${FC_IFACE} && sudo ip link set ${FC_IFACE} up"
  fi

  if [ "${DRY_RUN}" -eq 0 ] && ping -c 2 -W 2 -I "${FC_IFACE}" "${FC_IP}" >/dev/null 2>&1; then
    ok "flight controller ${FC_IP} responds on ${FC_IFACE}"
  else
    warn "no ICMP reply from ${FC_IP} on ${FC_IFACE}."
    warn "  This is EXPECTED if the FC is unpowered. It is NOT expected with the FC on."
    warn "  PX4 v1.17 answers ping once the Ethernet netman has configured the link."
  fi

  # 1b. THE PHANTOM GATEWAY. 10.41.10.254 is advertised on the point-to-point
  # link but nothing is there. If NetworkManager installs it as the default
  # route, every outbound packet -- apt, github, DNS -- is black-holed. This is
  # almost certainly why the Jetsons "could not reach github.com".
  local defroutes; defroutes="$(ip route show default 2>/dev/null || true)"
  if printf '%s' "${defroutes}" | grep -q "${FC_PHANTOM_GW}"; then
    bad "DEFAULT ROUTE POINTS AT THE PHANTOM GATEWAY ${FC_PHANTOM_GW}"
    printf '        %s\n' "${defroutes}"
    warn "  Nothing lives at ${FC_PHANTOM_GW}: the FC link is point-to-point with no router."
    warn "  While this route is installed there is NO internet, whatever the wifi says."
    warn "  Session-local fix (reverts on reboot, leaves no persistent hook):"
    warn "    sudo ip route del default via ${FC_PHANTOM_GW} dev ${FC_IFACE}"
    warn "  Durable fix, if the operator wants it (this DOES persist -- record it in the snapshot):"
    warn "    nmcli connection modify <fc-conn> ipv4.never-default yes ipv4.gateway ''"
    warn "    nmcli connection up <fc-conn>"
  elif [ -n "${defroutes}" ]; then
    ok "default route does not go via ${FC_PHANTOM_GW}"
    printf '        %s\n' "${defroutes}"
  else
    warn "no default route at all -- there will be no internet in phase 2"
  fi

  # 1c. the fleet interface Cyclone will bind to
  if ip -brief link show "${FLEET_IFACE}" >/dev/null 2>&1; then
    local state; state="$(ip -brief link show "${FLEET_IFACE}" | awk '{print $2}')"
    if [ "${state}" = "UP" ]; then
      ok "fleet interface ${FLEET_IFACE} is UP"
      ip -brief addr show "${FLEET_IFACE}" | sed 's/^/        /'
    else
      bad "fleet interface ${FLEET_IFACE} exists but is ${state}"
      warn "  Cyclone DDS is pinned to this interface by cyclonedds.xml. Down means no ROS 2"
      warn "  traffic reaches the ground station, and every /fmu/ topic will look local-only."
    fi
  else
    bad "fleet interface ${FLEET_IFACE} does not exist"
    warn "  Available: $(ip -brief link show 2>/dev/null | awk '{print $1}' | tr '\n' ' ')"
    warn "  Re-run with --fleet-iface <name>."
  fi

  # 1d. can we reach the world? Determines whether phase 2 can use apt.
  NET_OK=0
  if [ "${OFFLINE}" -eq 1 ]; then
    info "internet            : not probed (--offline)"
  elif [ "${DRY_RUN}" -eq 1 ]; then
    info "internet            : not probed (--dry-run)"
  else
    if getent hosts packages.ros.org >/dev/null 2>&1; then
      ok "DNS resolves packages.ros.org"
      if curl -fsS --max-time 20 -o /dev/null "http://packages.ros.org/ros2/ubuntu/dists/${EXPECTED_CODENAME}/Release" 2>/dev/null; then
        ok "packages.ros.org reachable over HTTP"
        NET_OK=1
      else
        bad "DNS works but packages.ros.org is unreachable over HTTP"
        warn "  Classic symptom of the phantom-gateway default route above."
      fi
    else
      bad "cannot resolve packages.ros.org"
      warn "  Check /etc/resolv.conf and that the default route is via ${FLEET_IFACE}."
    fi
  fi
  export NET_OK

  # 1e. the ground station
  if [ -n "${GS_HOST}" ] && [ "${DRY_RUN}" -eq 0 ]; then
    if ping -c 2 -W 2 "${GS_HOST}" >/dev/null 2>&1; then
      ok "ground station ${GS_HOST} responds"
    else
      warn "ground station ${GS_HOST} does not respond to ICMP (it may simply drop it)"
    fi
  fi
}

# --------------------------------------------------------------------------- #
# PHASE 2 -- apt. ROS 2 source, then packages.txt.
# --------------------------------------------------------------------------- #
ros_source_present() { [ -f /etc/apt/sources.list.d/ros2.list ] && [ -f /usr/share/keyrings/ros-archive-keyring.gpg ]; }

phase_apt() {
  phase_banner "2  apt -- ROS 2 source and packages.txt"

  [ -f "${PACKAGES_FILE}" ] || die "packages.txt not found next to this script: ${PACKAGES_FILE}"

  # If phase 'network' was skipped (--only apt), NET_OK is still the global 0,
  # which would silently push this phase down the offline path. Probe here.
  if [ "${NET_OK}" -eq 0 ] && [ "${OFFLINE}" -eq 0 ] && [ "${DRY_RUN}" -eq 0 ]; then
    if curl -fsS --max-time 20 -o /dev/null \
         "http://packages.ros.org/ros2/ubuntu/dists/${EXPECTED_CODENAME}/Release" 2>/dev/null; then
      NET_OK=1
      info "packages.ros.org reachable"
    else
      warn "packages.ros.org unreachable -- falling back to bundled .deb files if present"
      warn "  Run phase 'network' for the full diagnosis (the phantom gateway is the usual cause)."
    fi
  fi

  mkdir -p "${WS}/.deploy"
  local deploy_dir="${WS}/.deploy"
  if [ "${DRY_RUN}" -eq 0 ]; then
    dpkg-query -W -f='${binary:Package}\n' > "${deploy_dir}/dpkg_before.txt" 2>/dev/null || true
  fi

  # --- 2a. the ROS 2 apt source -------------------------------------------
  if ros_source_present; then
    ok "ROS 2 apt source already configured"
    info "  $(cat /etc/apt/sources.list.d/ros2.list)"
  else
    log "configuring the ROS 2 apt source"
    local keyfile="/usr/share/keyrings/ros-archive-keyring.gpg"
    local bundled_key=""
    [ -n "${BUNDLE_DIR}" ] && [ -f "${BUNDLE_DIR}/apt/ros.key" ] && bundled_key="${BUNDLE_DIR}/apt/ros.key"

    if [ -n "${bundled_key}" ]; then
      info "using the archive key shipped in the bundle (fetched on the ground station)"
      run sudo install -m 0644 "${bundled_key}" "${keyfile}"
    elif [ "${OFFLINE}" -eq 1 ]; then
      die "--offline and no ${BUNDLE_DIR:-<bundle>}/apt/ros.key. Re-bundle with the key,
      or drop the key in by hand:  sudo install -m0644 ros.key ${keyfile}"
    else
      [ "${NET_OK:-0}" -eq 1 ] || die "no route to packages.ros.org and no bundled key -- fix phase 1 first"
      run sudo curl -fsSL "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" -o "${keyfile}"
    fi

    run sudo add-apt-repository -y universe || warn "add-apt-repository universe failed (often already enabled)"
    local line="deb [arch=$(dpkg --print-architecture) signed-by=${keyfile}] http://packages.ros.org/ros2/ubuntu ${EXPECTED_CODENAME} main"
    if [ "${DRY_RUN}" -eq 1 ]; then
      printf '  %sDRY%s   write /etc/apt/sources.list.d/ros2.list: %s\n' "${C_DIM}" "${C_OFF}" "${line}"
    else
      printf '%s\n' "${line}" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    fi
    ok "ROS 2 apt source written"
  fi

  # Report the key's expiry. The ROS archive key rotated in 2025 and an expired
  # key produces an apt error that reads like a network fault.
  if [ -r /usr/share/keyrings/ros-archive-keyring.gpg ] && command -v gpg >/dev/null 2>&1; then
    local exp
    exp="$(gpg --show-keys --with-colons /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null \
           | awk -F: '/^pub/{print $7; exit}')"
    if [ -n "${exp}" ] && [ "${exp}" != "0" ]; then
      info "ROS archive key expires: $(date -u -d "@${exp}" '+%Y-%m-%d' 2>/dev/null || printf '%s' "${exp}")"
      if [ "${exp}" -lt "$(date +%s)" ]; then
        bad "the ROS archive key has EXPIRED -- apt will refuse the repo. Fetch a fresh ros.key."
      fi
    fi
  fi

  # --- 2b. packages --------------------------------------------------------
  local required optional missing_req missing_opt
  required="$(required_packages)"
  optional="$(optional_packages)"

  missing_req=""
  for p in ${required}; do pkg_installed "${p}" || missing_req="${missing_req} ${p}"; done
  missing_opt=""
  for p in ${optional}; do pkg_installed "${p}" || missing_opt="${missing_opt} ${p}"; done

  if [ -z "${missing_req# }" ] && [ -z "${missing_opt# }" ]; then
    ok "every package in packages.txt is already installed -- nothing to do"
    return 0
  fi
  info "missing (required)  :${missing_req:- none}"
  info "missing (optional)  :${missing_opt:- none}"

  # Offline path: install from the .deb set the bundle carries.
  if [ "${OFFLINE}" -eq 1 ] || [ "${NET_OK:-0}" -eq 0 ]; then
    local debdir="${BUNDLE_DIR:-}/debs"
    if [ -n "${BUNDLE_DIR}" ] && [ -d "${debdir}" ] && ls "${debdir}"/*.deb >/dev/null 2>&1; then
      log "installing from bundled .deb files (offline)"
      confirm "install $(ls -1 "${debdir}"/*.deb | wc -l) .deb files with apt-get?" \
        || die "declined"
      run sudo apt-get install -y --no-install-recommends "${debdir}"/*.deb
    else
      die "No network and no bundled .deb set at ${debdir}.
      Two ways forward, pick one and say so on the card:
        (a) put the Jetson on wifi with a working default route (see phase 1),
            then re-run without --offline. This is the normal path.
        (b) re-run bundle_for_jetson.sh on the ground station with --with-debs
            --jetson-status <dump from this Jetson>, and re-deliver."
    fi
  else
    confirm "run apt-get update and install${missing_req}${missing_opt} ?" || die "declined"
    run sudo apt-get update
    # shellcheck disable=SC2086
    [ -n "${missing_req# }" ] && run sudo apt-get install -y ${missing_req}
    for p in ${missing_opt}; do
      run sudo apt-get install -y "${p}" || warn "optional package '${p}' not installed -- continuing"
    done
  fi

  # --- 2c. verify, and record what WE installed for the rollback ------------
  if [ "${DRY_RUN}" -eq 0 ]; then
    local still=""
    for p in ${required}; do pkg_installed "${p}" || still="${still} ${p}"; done
    if [ -n "${still# }" ]; then
      bad "still missing after install:${still}"
      die "apt did not deliver the required set. Read the apt output above."
    fi
    ok "all required packages present"

    dpkg-query -W -f='${binary:Package}\n' > "${deploy_dir}/dpkg_after.txt" 2>/dev/null || true
    if [ -f "${deploy_dir}/dpkg_before.txt" ]; then
      comm -13 <(sort -u "${deploy_dir}/dpkg_before.txt") <(sort -u "${deploy_dir}/dpkg_after.txt") \
        >> "${deploy_dir}/apt_installed_by_deploy.txt" 2>/dev/null || true
      sort -u -o "${deploy_dir}/apt_installed_by_deploy.txt" "${deploy_dir}/apt_installed_by_deploy.txt" 2>/dev/null || true
      info "packages newly installed by this kit: $(wc -l < "${deploy_dir}/apt_installed_by_deploy.txt" 2>/dev/null || echo 0)"
      info "  recorded in ${deploy_dir}/apt_installed_by_deploy.txt (the rollback reads this)"
    fi
  fi

  # rosdep: initialise if it never has been. Failure here is not fatal --
  # nothing in this workspace resolves through rosdep at build time.
  if [ "${DRY_RUN}" -eq 0 ] && command -v rosdep >/dev/null 2>&1; then
    if [ ! -d /etc/ros/rosdep/sources.list.d ]; then
      run sudo rosdep init || warn "rosdep init failed (harmless if you are offline)"
    fi
    rosdep update --rosdistro "${EXPECTED_ROS_DISTRO}" >/dev/null 2>&1 \
      && ok "rosdep database updated" \
      || warn "rosdep update failed (offline?) -- not fatal for this workspace"
  fi
}

# --------------------------------------------------------------------------- #
# PHASE 3 -- workspace. Unpack the source the bundle carries.
# --------------------------------------------------------------------------- #
phase_workspace() {
  phase_banner "3  workspace -- ${WS}/src"

  run mkdir -p "${WS}/src" "${WS}/.deploy"

  [ -n "${BUNDLE_DIR}" ] || die "phase 'workspace' needs --bundle DIR (the unpacked bundle root)"
  [ -d "${BUNDLE_DIR}/src" ] || die "no src/ inside the bundle: ${BUNDLE_DIR}/src"

  # Checksums first. A tarball that survived scp over a flaky wifi link but
  # arrived corrupt would otherwise show up as an incomprehensible compile error.
  if [ -f "${BUNDLE_DIR}/MANIFEST.sha256" ]; then
    log "verifying bundle checksums"
    if [ "${DRY_RUN}" -eq 1 ]; then
      info "would run: sha256sum -c MANIFEST.sha256"
    elif ( cd "${BUNDLE_DIR}" && sha256sum -c --quiet MANIFEST.sha256 ); then
      ok "MANIFEST.sha256 verifies -- $(wc -l < "${BUNDLE_DIR}/MANIFEST.sha256") files"
    else
      die "BUNDLE CHECKSUM MISMATCH. Do not build this. Re-deliver the tarball:
      the copy on this Jetson does not match the one the ground station produced."
    fi
  else
    warn "no MANIFEST.sha256 in the bundle -- integrity unverified"
  fi

  local pkg
  for pkg in px4_msgs as2_platform_pixhawk as2_mocap_guarded; do
    local srcdir="${BUNDLE_DIR}/src/${pkg}"
    [ -d "${srcdir}" ] || die "bundle is missing src/${pkg}"
    if [ -d "${WS}/src/${pkg}" ]; then
      info "${pkg}: already present in the workspace"
      if [ "${DRY_RUN}" -eq 0 ] && ! diff -rq "${srcdir}" "${WS}/src/${pkg}" \
           --exclude=.git --exclude=build --exclude=install --exclude=log >/dev/null 2>&1; then
        warn "${pkg} in the workspace DIFFERS from the bundle."
        if confirm "replace ${WS}/src/${pkg} with the bundled copy?"; then
          run rm -rf "${WS}/src/${pkg}"
          run cp -a "${srcdir}" "${WS}/src/${pkg}"
          ok "${pkg} replaced from the bundle"
        else
          warn "keeping the workspace copy -- you are now building something the bundle does not describe"
        fi
      else
        ok "${pkg} matches the bundle"
      fi
    else
      run cp -a "${srcdir}" "${WS}/src/${pkg}"
      ok "${pkg} unpacked"
    fi
  done

  # Provenance. The bundle records what it shipped; assert it here so a wrong
  # px4_msgs branch is caught before it becomes a runtime message mismatch.
  if [ -f "${BUNDLE_DIR}/BUNDLE_INFO.txt" ]; then
    grep -q "px4_msgs.*${PX4_MSGS_BRANCH}" "${BUNDLE_DIR}/BUNDLE_INFO.txt" \
      && ok "bundle declares px4_msgs ${PX4_MSGS_BRANCH}" \
      || warn "bundle does not declare px4_msgs branch ${PX4_MSGS_BRANCH} -- check BUNDLE_INFO.txt"
    grep -q "${PLATFORM_BASE_COMMIT}" "${BUNDLE_DIR}/BUNDLE_INFO.txt" \
      && ok "bundle declares as2_platform_pixhawk base ${PLATFORM_BASE_COMMIT:0:7}" \
      || warn "bundle does not declare platform base commit ${PLATFORM_BASE_COMMIT:0:7}"
  fi

  # The local patch must actually be in the tree we are about to build. The
  # bundle ships the PATCHED working tree (the Jetson has no github), so the
  # test is a content test, not a git test.
  local marker="${WS}/src/as2_platform_pixhawk/src/pixhawk_platform.cpp"
  if [ -f "${marker}" ]; then
    if grep -q "PX4_FORCE_DISARM_MAGIC" "${marker}" && grep -q "isExternalOdomFresh" "${marker}"; then
      ok "O134 platform patch is present in the source (kill switch + odom staleness gate)"
    else
      bad "as2_platform_pixhawk source does NOT carry the O134 patch"
      die "The kill switch would be a silent no-op and the external-odometry staleness gate
      would be absent. Re-bundle from a ground station whose working tree is patched
      (flight_ops/patches/apply_platform_patch.sh)."
    fi
  fi

  # flight_ops nodes this Jetson runs.
  if [ -d "${BUNDLE_DIR}/flight_ops" ]; then
    run mkdir -p "${WS}/flight_ops"
    run cp -a "${BUNDLE_DIR}/flight_ops/." "${WS}/flight_ops/"
    ok "flight_ops payload copied to ${WS}/flight_ops"
  fi
  if [ -f "${BUNDLE_DIR}/BUNDLE_INFO.txt" ]; then
    run cp -a "${BUNDLE_DIR}/BUNDLE_INFO.txt" "${WS}/.deploy/bundle_info.txt"
  fi

  # aarch64 python wheels, if the bundle carries them.
  if [ -d "${BUNDLE_DIR}/wheels" ] && ls "${BUNDLE_DIR}/wheels"/*.whl >/dev/null 2>&1; then
    log "installing bundled aarch64 wheels (--user, no network)"
    run python3 -m pip install --user --no-index --find-links "${BUNDLE_DIR}/wheels" \
        pymavlink fastcrc pyserial || warn "wheel install failed -- check python3 -m pip list"
  fi
}

# --------------------------------------------------------------------------- #
# PHASE 4 -- build.
# --------------------------------------------------------------------------- #
phase_build() {
  phase_banner "4  build -- colcon"

  [ -d "${WS}/src/px4_msgs" ] || die "no ${WS}/src/px4_msgs -- run phase 'workspace' first"
  [ -f "/opt/ros/${EXPECTED_ROS_DISTRO}/setup.bash" ] || die "ROS 2 ${EXPECTED_ROS_DISTRO} is not installed -- run phase 'apt' first"

  if [ "${DRY_RUN}" -eq 1 ]; then
    info "would build px4_msgs then as2_platform_pixhawk with:"
    info "  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
    return 0
  fi

  # `set +u` is mandatory: the ROS setup scripts dereference unset variables and
  # would abort this script under `set -u`. Restore it immediately after.
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${EXPECTED_ROS_DISTRO}/setup.bash" || { set -u; die "cannot source ROS 2 ${EXPECTED_ROS_DISTRO}"; }
  if [ -f "/opt/ros/${EXPECTED_ROS_DISTRO}/share/aerostack2/local_setup.bash" ]; then
    # shellcheck disable=SC1091
    source "/opt/ros/${EXPECTED_ROS_DISTRO}/share/aerostack2/local_setup.bash" \
      || { set -u; die "cannot source aerostack2 -- is ros-humble-aerostack2 installed?"; }
  else
    set -u
    die "ros-humble-aerostack2 is not installed (no share/aerostack2/local_setup.bash).
      as2_platform_pixhawk will not configure without as2_core. Run phase 'apt'."
  fi
  set -u

  ok "ROS 2 ${ROS_DISTRO:-?} sourced; aerostack2 $(ros2 pkg xml as2_core 2>/dev/null | grep -m1 -oP '(?<=<version>)[^<]+' || echo '?')"

  # Build in two steps so a px4_msgs failure is not buried in platform errors.
  # -DBUILD_TESTING=OFF: ament_lint_auto is not part of ros-base, so the test
  # block cannot configure. Same flag the ground station uses.
  local common=(--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF)

  log "building px4_msgs (this is the long one -- ~20-40 min on an Orin NX)"
  ( cd "${WS}" && PYTHONNOUSERSITE=1 colcon build --packages-select px4_msgs "${common[@]}" ) \
    || die "px4_msgs failed. Log: ${WS}/log/latest_build/px4_msgs/stdout_stderr.log"
  ok "px4_msgs built"

  log "building as2_platform_pixhawk"
  ( cd "${WS}" && PYTHONNOUSERSITE=1 colcon build --packages-select as2_platform_pixhawk "${common[@]}" ) \
    || die "as2_platform_pixhawk failed. Log: ${WS}/log/latest_build/as2_platform_pixhawk/stdout_stderr.log"

  local node="${WS}/install/as2_platform_pixhawk/lib/as2_platform_pixhawk/as2_platform_pixhawk_node"
  [ -x "${node}" ] || die "build reported success but ${node} is missing"
  ok "as2_platform_pixhawk built: ${node}"

  # S1: the hardened state-estimator plugin. It is a pluginlib library loaded by
  # the STOCK as2_state_estimator node, so what has to exist is the .so and the
  # resource-index entry -- there is no executable to look for.
  log "building as2_mocap_guarded (S1 state-estimator plugin)"
  ( cd "${WS}" && PYTHONNOUSERSITE=1 colcon build --packages-select as2_mocap_guarded "${common[@]}" ) \
    || die "as2_mocap_guarded failed. Log: ${WS}/log/latest_build/as2_mocap_guarded/stdout_stderr.log"

  local plugin_so="${WS}/install/as2_mocap_guarded/lib/libmocap_pose_guarded.so"
  local plugin_idx="${WS}/install/as2_mocap_guarded/share/ament_index/resource_index/as2_state_estimator__pluginlib__plugin/as2_mocap_guarded"
  [ -f "${plugin_so}" ] || die "build reported success but ${plugin_so} is missing"
  [ -f "${plugin_idx}" ] || die "libmocap_pose_guarded.so exists but the pluginlib resource-index entry does not:
      ${plugin_idx}
      Without it the stock as2_state_estimator's ClassLoader cannot find
      mocap_pose_guarded::Plugin, and the drone silently falls back to a plugin
      that publishes an ORIGIN POSE on a rigid-body name mismatch."
  ok "as2_mocap_guarded built: libmocap_pose_guarded.so + pluginlib index entry"

  [ -f "${WS}/install/setup.bash" ] || die "no ${WS}/install/setup.bash after a successful build"
  ok "workspace overlay ready"
}

# --------------------------------------------------------------------------- #
# PHASE 5 -- Micro-XRCE-DDS-Agent.
#
# There is NO apt package for this on any architecture in the ROS 2 Humble
# repositories -- the buildfarm has no microxrcedds_agent or micro_ros_agent
# job. So it is built from the source the bundle carries, with
# UAGENT_SUPERBUILD=OFF so that CMake does NOT try to git-clone Fast-CDR and
# Fast-DDS at configure time. Those come from ROS 2 Humble, which is already
# installed: find_package(fastcdr 1) and find_package(fastrtps 2) are satisfied
# by ros-humble-fastcdr (1.0.x) and ros-humble-fastrtps (2.6.x).
# That is what makes this step work with no internet at all.
# --------------------------------------------------------------------------- #
phase_agent() {
  phase_banner "5  agent -- Micro-XRCE-DDS-Agent ${XRCE_TAG}"

  local bin="${XRCE_PREFIX}/bin/MicroXRCEAgent"
  if [ -x "${bin}" ]; then
    ok "already built: ${bin}"
    if [ "${DRY_RUN}" -eq 0 ]; then
      set +u
      # shellcheck disable=SC1091
      source "/opt/ros/${EXPECTED_ROS_DISTRO}/setup.bash" >/dev/null 2>&1 || true
      set -u
      LD_LIBRARY_PATH="${XRCE_PREFIX}/lib:${LD_LIBRARY_PATH:-}" "${bin}" --help >/dev/null 2>&1 \
        && ok "the binary runs" \
        || warn "the binary is present but would not run -- delete ${XRCE_PREFIX} and re-run this phase"
    fi
    return 0
  fi

  local srcdir=""
  if [ -n "${BUNDLE_DIR}" ] && [ -d "${BUNDLE_DIR}/xrce/Micro-XRCE-DDS-Agent" ]; then
    srcdir="${BUNDLE_DIR}/xrce/Micro-XRCE-DDS-Agent"
  elif [ -d "${XRCE_PREFIX}/src/Micro-XRCE-DDS-Agent" ]; then
    srcdir="${XRCE_PREFIX}/src/Micro-XRCE-DDS-Agent"
  else
    die "no Micro-XRCE-DDS-Agent source.
      It is not available from apt on ANY architecture for Humble, so it must be
      bundled. Re-run bundle_for_jetson.sh on the ground station (it clones ${XRCE_TAG}),
      or, if this Jetson has working internet:
        git clone -b ${XRCE_TAG} --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \\
            ${XRCE_PREFIX}/src/Micro-XRCE-DDS-Agent
      and re-run this phase."
  fi
  info "source              : ${srcdir}"

  if [ "${DRY_RUN}" -eq 1 ]; then
    info "would cmake -DUAGENT_SUPERBUILD=OFF -DUAGENT_USE_SYSTEM_{FASTCDR,FASTDDS,LOGGER}=ON"
    return 0
  fi

  run mkdir -p "${XRCE_PREFIX}/src" "${XRCE_PREFIX}/build"
  if [ "${srcdir}" != "${XRCE_PREFIX}/src/Micro-XRCE-DDS-Agent" ]; then
    run rm -rf "${XRCE_PREFIX}/src/Micro-XRCE-DDS-Agent"
    run cp -a "${srcdir}" "${XRCE_PREFIX}/src/Micro-XRCE-DDS-Agent"
  fi

  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${EXPECTED_ROS_DISTRO}/setup.bash" || { set -u; die "cannot source ROS 2"; }
  set -u

  log "configuring (system Fast-DDS from ROS 2 ${EXPECTED_ROS_DISTRO}, no network)"
  ( cd "${XRCE_PREFIX}/build" && cmake "${XRCE_PREFIX}/src/Micro-XRCE-DDS-Agent" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="${XRCE_PREFIX}" \
      -DUAGENT_SUPERBUILD=OFF \
      -DUAGENT_USE_SYSTEM_FASTCDR=ON \
      -DUAGENT_USE_SYSTEM_FASTDDS=ON \
      -DUAGENT_USE_SYSTEM_LOGGER=ON \
      -DUAGENT_BUILD_EXECUTABLE=ON \
      -DUAGENT_BUILD_TESTS=OFF ) \
    || die "cmake configure failed.
      The usual cause is that ROS 2 was not on CMAKE_PREFIX_PATH -- confirm
      /opt/ros/${EXPECTED_ROS_DISTRO}/setup.bash was sourced and that
      ros-humble-fastrtps and ros-humble-fastcdr are installed. libspdlog-dev,
      libfmt-dev and libssl-dev must also be present (they are in packages.txt).
      If it still fails, the fallback is the superbuild, which NEEDS INTERNET:
        cmake ... -DUAGENT_SUPERBUILD=ON"

  log "compiling"
  ( cd "${XRCE_PREFIX}/build" && make -j"$(nproc)" ) || die "agent build failed"
  ( cd "${XRCE_PREFIX}/build" && make install ) || die "agent install failed"

  [ -x "${bin}" ] || die "build succeeded but ${bin} is missing"
  LD_LIBRARY_PATH="${XRCE_PREFIX}/lib:${LD_LIBRARY_PATH:-}" "${bin}" --help >/dev/null 2>&1 \
    || die "${bin} will not run -- check LD_LIBRARY_PATH and ldd ${bin}"
  ok "MicroXRCEAgent built and runs: ${bin}"
  info "  it will be started by hand, without sudo:  MicroXRCEAgent udp4 -p ${XRCE_PORT}"
}

# --------------------------------------------------------------------------- #
# PHASE 6 -- environment. A file the operator SOURCES. No persistent hooks.
# --------------------------------------------------------------------------- #
phase_env() {
  phase_banner "6  env -- setup_env.sh, cyclonedds.xml, rollback.sh"

  run mkdir -p "${WS}/.deploy"

  # ---- cyclonedds.xml -----------------------------------------------------
  # Two things this fixes, both of which bite on this exact rig:
  #  1. Cyclone picks ONE interface by itself. With both the wifi and the
  #     point-to-point FC link up, it can pick enP8p1s0 -- whose only peer is a
  #     flight controller that speaks XRCE, not DDS. Every ROS 2 topic then
  #     becomes invisible to the ground station with no error anywhere.
  #  2. AllowMulticast=spdp keeps multicast for discovery only and sends user
  #     data unicast. Multicast on wifi is transmitted at the lowest basic rate
  #     and is where a 100 Hz stream goes to die.
  local peers_xml=""
  local p
  for p in ${GS_HOST} ${PEERS}; do
    peers_xml="${peers_xml}
        <Peer address=\"${p}\"/>"
  done
  [ -n "${peers_xml}" ] || peers_xml="
        <!-- no --gs-host / --peer given: discovery is multicast-only, which is
             the fragile case on wifi. Add peers and re-run phase 'env'. -->"

  if [ "${DRY_RUN}" -eq 1 ]; then
    info "would write ${WS}/cyclonedds.xml pinned to ${FLEET_IFACE}"
  else
    cat > "${WS}/cyclonedds.xml" <<XMLEOF
<?xml version="1.0" encoding="UTF-8" ?>
<!-- Generated by flight_ops/deploy/jetson_setup.sh. Edit and re-run phase 'env'
     rather than hand-editing, so all three aircraft stay identical. -->
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <!-- Pin the fleet interface. Without this Cyclone may choose ${FC_IFACE},
           the point-to-point link to the flight controller, and no ROS 2 topic
           will ever reach the ground station. -->
      <Interfaces>
        <NetworkInterface name="${FLEET_IFACE}" priority="default" multicast="default"/>
      </Interfaces>
      <!-- Multicast for discovery only; user data goes unicast. On wifi,
           multicast is sent at the lowest basic rate and drops first. -->
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>${peers_xml}
      </Peers>
    </Discovery>
    <Tracing>
      <Verbosity>warning</Verbosity>
      <OutputFile>stderr</OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>
XMLEOF
    ok "wrote ${WS}/cyclonedds.xml (interface ${FLEET_IFACE})"
  fi

  # ---- setup_env.sh -------------------------------------------------------
  if [ "${DRY_RUN}" -eq 1 ]; then
    info "would write ${WS}/setup_env.sh"
  else
    cat > "${WS}/setup_env.sh" <<ENVEOF
# ~/as2_o134_ws/setup_env.sh
#
# SOURCE this once per shell, by hand:
#     source ~/as2_o134_ws/setup_env.sh
#
# It is deliberately NOT in ~/.bashrc. Rule 6 of the shared-hardware discipline:
# no persistent hooks. A login shell on this Jetson must look like a stock
# JetPack login shell to anyone else who uses the aircraft.
#
# Generated by flight_ops/deploy/jetson_setup.sh on $(date -u '+%Y-%m-%dT%H:%M:%SZ')
# for ${DRONE_NS}. Regenerate with:  jetson_setup.sh --drone-ns ${DRONE_NS} --only env

# The ROS setup scripts dereference unset variables. Turn off 'nounset' while
# they run, then put it back exactly as we found it.
_o134_had_u=0
case "\$-" in *u*) _o134_had_u=1 ;; esac
set +u

. /opt/ros/${EXPECTED_ROS_DISTRO}/setup.bash
. /opt/ros/${EXPECTED_ROS_DISTRO}/share/aerostack2/local_setup.bash
if [ -f "${WS}/install/setup.bash" ]; then
  . "${WS}/install/setup.bash"
else
  echo "setup_env.sh: WARNING ${WS}/install/setup.bash is missing -- the workspace is not built" >&2
fi

if [ "\${_o134_had_u}" = "1" ]; then set -u; fi
unset _o134_had_u

# --- middleware -------------------------------------------------------------
export RMW_IMPLEMENTATION=${RMW_DEFAULT}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_DEFAULT}
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://${WS}/cyclonedds.xml"

# --- this aircraft ----------------------------------------------------------
export AS2_DRONE_NS=${DRONE_NS}
export O134_WS=${WS}
export O134_FC_IP=${FC_IP}
# Namespace PX4 publishes /fmu/ under (UXRCE_DDS_NS_IDX). Empty = bare /fmu/.
# Pass the same value to as2_platform_pixhawk's fmu_prefix parameter and to
# volume_guard.py's --fmu-prefix, so all three agree by construction.
export O134_FMU_PREFIX="${FMU_PREFIX}"
export O134_FC_IFACE=${FC_IFACE}
export O134_FLEET_IFACE=${FLEET_IFACE}

# --- Micro-XRCE-DDS-Agent (built from source; not an apt package) -----------
export XRCE_PREFIX=${XRCE_PREFIX}
case ":\${PATH}:" in *":${XRCE_PREFIX}/bin:"*) ;; *) PATH="${XRCE_PREFIX}/bin:\${PATH}"; export PATH ;; esac
case ":\${LD_LIBRARY_PATH:-}:" in *":${XRCE_PREFIX}/lib:"*) ;; *) LD_LIBRARY_PATH="${XRCE_PREFIX}/lib:\${LD_LIBRARY_PATH:-}"; export LD_LIBRARY_PATH ;; esac

echo "O134 env: \${AS2_DRONE_NS}  ROS_DISTRO=\${ROS_DISTRO}  RMW=\${RMW_IMPLEMENTATION}  DOMAIN=\${ROS_DOMAIN_ID}"
echo "          agent: MicroXRCEAgent udp4 -p ${XRCE_PORT}"
echo "          fleet iface: ${FLEET_IFACE} (CYCLONEDDS_URI is set)"
echo "          fmu prefix : ${FMU_PREFIX:-(bare /fmu/ -- unsafe with 3 aircraft)}"
ENVEOF
    chmod 0644 "${WS}/setup_env.sh"
    ok "wrote ${WS}/setup_env.sh"
    if [ -z "${FMU_PREFIX}" ]; then
      warn "O134_FMU_PREFIX is EMPTY, so PX4 topics are bare /fmu/... ."
      warn "  That is correct for ONE aircraft on the domain and WRONG for three:"
      warn "  all three would publish and subscribe to the same topic names, so a"
      warn "  disarm aimed at one reaches all of them and each platform would fuse"
      warn "  the others' odometry. Set UXRCE_DDS_NS_IDX on the FC and re-run:"
      warn "    jetson_setup.sh --drone-ns ${DRONE_NS} --only env --fmu-prefix /uav_N"
    else
      ok "fmu prefix          : ${FMU_PREFIX}"
    fi
  fi

  # ---- rollback.sh --------------------------------------------------------
  # Rule 4 of the shared-hardware discipline: explicit rollback, reverse order,
  # exact command per step. Generated, not written by hand, so it matches what
  # actually happened on THIS Jetson.
  if [ "${DRY_RUN}" -eq 1 ]; then
    info "would write ${WS}/.deploy/rollback.sh"
  else
    cat > "${WS}/.deploy/rollback.sh" <<'ROLLEOF'
#!/usr/bin/env bash
#
# rollback.sh -- undo jetson_setup.sh, in strict reverse order.
#
# Generated on the Jetson by jetson_setup.sh. It removes ONLY what this kit
# added. It prints every step and asks before each one; nothing happens under
# --dry-run (the default).
#
#   ./rollback.sh            # show what would be removed
#   ./rollback.sh --apply    # actually remove it
#
set -euo pipefail
APPLY=0
[ "${1:-}" = "--apply" ] && APPLY=1
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${HERE}/.." && pwd)"

step() {
  printf '\n--- %s\n' "$1"; shift
  printf '    %s\n' "$*"
  if [ "${APPLY}" -eq 1 ]; then
    printf '    run? [y/N] '; read -r r || true
    case "${r}" in y|Y) eval "$*" ;; *) printf '    skipped\n' ;; esac
  fi
}

# 6 -> 0, reverse of the setup phases.
step "6 env"       "rm -f '${WS}/setup_env.sh' '${WS}/cyclonedds.xml'"
step "5 agent"     "rm -rf \"\${XRCE_PREFIX:-\$HOME/xrce_agent}\""
step "4 build"     "rm -rf '${WS}/build' '${WS}/install' '${WS}/log'"
step "3 workspace" "rm -rf '${WS}/src' '${WS}/flight_ops'"

if [ -s "${HERE}/apt_installed_by_deploy.txt" ]; then
  printf '\n--- 2 apt: packages this kit installed (%s)\n' "$(wc -l < "${HERE}/apt_installed_by_deploy.txt")"
  printf '    sudo apt-get purge -y $(tr "\\n" " " < %s)\n' "${HERE}/apt_installed_by_deploy.txt"
  printf '    sudo apt-get autoremove -y\n'
  printf '    NOTE: read that list before running it. It is a dpkg diff taken\n'
  printf '          across the apt phase, so anything else installed in the same\n'
  printf '          window is in it too.\n'
  if [ "${APPLY}" -eq 1 ]; then
    printf '    run? [y/N] '; read -r r || true
    case "${r}" in
      y|Y) xargs -a "${HERE}/apt_installed_by_deploy.txt" sudo apt-get purge -y
           sudo apt-get autoremove -y ;;
      *)   printf '    skipped\n' ;;
    esac
  fi
else
  printf '\n--- 2 apt: no record of packages installed by this kit\n'
fi

step "2 apt source" "sudo rm -f /etc/apt/sources.list.d/ros2.list /usr/share/keyrings/ros-archive-keyring.gpg"
step "0 workspace root" "rmdir '${WS}' 2>/dev/null || echo '    (not empty -- left in place on purpose)'"

printf '\nNothing was ever added to ~/.bashrc, systemd or cron, so there is nothing else to undo.\n'
[ "${APPLY}" -eq 1 ] || printf 'This was a dry run. Re-run with --apply to act.\n'
ROLLEOF
    chmod 0755 "${WS}/.deploy/rollback.sh"
    ok "wrote ${WS}/.deploy/rollback.sh (dry-run by default)"
  fi
}

# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
main() {
  mkdir -p "${WS}/.deploy" 2>/dev/null || true
  LOGFILE="${WS}/.deploy/setup.log"
  : >>"${LOGFILE}" 2>/dev/null || LOGFILE=""

  printf '%s\n' "=============================================================="
  printf 'O134 Jetson setup   %s\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
  printf 'host %s   ns %s   ws %s\n' "$(hostname)" "${DRONE_NS:-<unset>}" "${WS}"
  [ "${DRY_RUN}" -eq 1 ] && printf 'DRY RUN -- nothing will be changed\n'
  [ "${OFFLINE}" -eq 1 ] && printf 'OFFLINE -- no downloads will be attempted\n'
  printf '%s\n' "=============================================================="

  # Unconditional. Not subject to --only / --skip: a guard you can opt out of
  # is not a guard.
  hard_guards

  local ran=""
  local ph
  for ph in ${ALL_PHASES}; do
    if want_phase "${ph}"; then
      "phase_${ph}"
      ran="${ran} ${ph}"
    else
      printf '\n[setup] phase %s -- skipped by request\n' "${ph}"
    fi
  done

  printf '\n%s\n' "=============================================================="
  printf 'phases completed:%s\n' "${ran:- none}"
  if [ "${DRY_RUN}" -eq 1 ]; then
    printf 'DRY RUN -- nothing was changed.\n'
  else
    cat <<NEXTEOF
Next, in this order:

  1. source ${WS}/setup_env.sh
  2. ${SCRIPT_DIR}/verify_jetson.sh --drone-ns ${DRONE_NS}
     (add --with-fmu once the flight controller is powered, to prove the
      uXRCE-DDS session and the /fmu/ endpoint matching)
  3. Only when verify_jetson.sh prints JETSON GREEN, put this Jetson on the
     aircraft card as proven.

Rollback, any time:  ${WS}/.deploy/rollback.sh        (dry run)
                     ${WS}/.deploy/rollback.sh --apply
NEXTEOF
  fi
  printf '%s\n' "=============================================================="
}

main "$@"
