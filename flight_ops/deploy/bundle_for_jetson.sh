#!/usr/bin/env bash
#
# bundle_for_jetson.sh -- build one tarball that carries everything a Jetson
# needs, so that setup on the aircraft requires no access to github.
#
# RUN THIS ON THE WSL GROUND STATION (Ubuntu 22.04). Not on a Jetson.
#
#   flight_ops/deploy/bundle_for_jetson.sh
#   scp o134_bundle_<stamp>.tar.gz jetson@10.88.51.230:~/     # printed at the end
#
# WHY A BUNDLE AT ALL
#
# The Jetsons could not reach github.com over the wired route, and the wired
# route is the one that is always up. The proven pattern on this project is:
# fetch and stage on the WSL side, scp in. This script makes that repeatable
# instead of remembered.
#
# What it does NOT solve: apt. Debian packages are architecture-specific and
# the ground station is amd64, so the ROS 2 arm64 packages cannot be built or
# meaningfully staged here without extra machinery. The normal path is to give
# the Jetson wifi with a working default route for the ten minutes the apt
# phase takes (see the phantom-gateway note in the README). --with-debs exists
# for the case where that is impossible; read its warning before relying on it.
#
# WHAT GOES IN
#
#   src/px4_msgs/                    branch release/1.17, the exact tree the GS built
#   src/as2_platform_pixhawk/        base 2b00b77 WITH the O134 patch already applied
#   xrce/Micro-XRCE-DDS-Agent/       tag v2.4.2 -- there is no apt package for this
#   flight_ops/deploy/               this kit, so the Jetson can re-run it
#   flight_ops/nodes/                vrpn_to_rigidbodies.py (each Jetson runs its own)
#   flight_ops/patches/              the patch and its evidence, for provenance
#   flight_ops/lab_config/           the gates and the indoor parameter set, for reference
#   apt/ros.key                      the ROS archive key, so the Jetson needs no curl
#   wheels/                          aarch64 python wheels          (--with-wheels)
#   debs/                            arm64 .deb closure             (--with-debs)
#   BUNDLE_INFO.txt                  commits, versions, host, UTC stamp
#   MANIFEST.sha256                  sha256 of every file, checked on the Jetson
#
# Exit codes:
#   0  bundle written
#   1  a prerequisite was missing or a stage failed
#   2  bad arguments / wrong machine
#
set -euo pipefail

readonly PX4_MSGS_BRANCH="release/1.17"
readonly PX4_MSGS_EXPECT_COMMIT="86d8239e962f6939e05c3737784f60c02fa884db"
readonly PLATFORM_BASE_COMMIT="2b00b77dcd2a4e3f7f607ef043bc8cb85e215b88"
readonly XRCE_TAG="v2.4.2"
readonly XRCE_REPO="https://github.com/eProsima/Micro-XRCE-DDS-Agent.git"
readonly ROS_KEY_URL="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"

readonly JETSON_PY_TAG="cp310"          # jammy ships python 3.10
readonly JETSON_PLATFORM="manylinux2014_aarch64"
readonly WHEELS="pymavlink==2.4.49 fastcrc==0.3.6 pyserial==3.5"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

WS="${HOME}/as2_o134_ws"
OUT_DIR="${PWD}"
STAMP="$(date -u '+%Y%m%dT%H%M%SZ')"
NAME=""
WITH_WHEELS=0
WITH_DEBS=0
JETSON_STATUS=""
JETSON_HOST="jetson@10.88.51.230"
KEEP_STAGE=0
NO_XRCE=0

usage() {
  sed -n '2,45p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  cat <<'EOF'

Options:
  --ws DIR              Ground-station workspace. Default ~/as2_o134_ws
  --out DIR             Where to write the tarball. Default: current directory
  --name NAME           Bundle basename. Default o134_bundle_<UTC stamp>
  --jetson HOST         Used only to print the scp line. Default jetson@10.88.51.230
  --with-wheels         Also download aarch64 wheels for pymavlink/fastcrc/pyserial
  --with-debs           Also stage the arm64 .deb closure. REQUIRES --jetson-status.
                        UNVERIFIED -- read the warning it prints.
  --jetson-status FILE  A copy of the Jetson's /var/lib/dpkg/status, for --with-debs:
                          scp jetson@10.88.51.230:/var/lib/dpkg/status ./jetson_dpkg_status
  --no-xrce             Skip the Micro-XRCE-DDS-Agent clone (you already have one)
  --keep-stage          Leave the staging directory behind for inspection
  -h, --help            This text
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --ws)             WS="${2:?}"; shift 2 ;;
    --out)            OUT_DIR="${2:?}"; shift 2 ;;
    --name)           NAME="${2:?}"; shift 2 ;;
    --jetson)         JETSON_HOST="${2:?}"; shift 2 ;;
    --with-wheels)    WITH_WHEELS=1; shift ;;
    --with-debs)      WITH_DEBS=1; shift ;;
    --jetson-status)  JETSON_STATUS="${2:?}"; shift 2 ;;
    --no-xrce)        NO_XRCE=1; shift ;;
    --keep-stage)     KEEP_STAGE=1; shift ;;
    -h|--help)        usage; exit 0 ;;
    *) printf 'unknown option: %s\n\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

[ -n "${NAME}" ] || NAME="o134_bundle_${STAMP}"

if [ -t 1 ]; then C_OK=$'\033[32m'; C_BAD=$'\033[31m'; C_WARN=$'\033[33m'; C_OFF=$'\033[0m'
else C_OK=""; C_BAD=""; C_WARN=""; C_OFF=""; fi
log()  { printf '[bundle] %s\n' "$*"; }
ok()   { printf '  %sOK%s    %s\n' "${C_OK}" "${C_OFF}" "$*"; }
warn() { printf '  %sWARN%s  %s\n' "${C_WARN}" "${C_OFF}" "$*"; }
die()  { printf '\n[bundle] %sSTOP%s  %s\n' "${C_BAD}" "${C_OFF}" "$*" >&2; exit "${2:-1}"; }

# --------------------------------------------------------------------------- #
# 0. is this the right machine, and are the prerequisites here?
# --------------------------------------------------------------------------- #
log "checking the ground station"

arch="$(uname -m)"
if [ "${arch}" = "aarch64" ]; then
  die "This is aarch64 -- you appear to be ON a Jetson.
      bundle_for_jetson.sh runs on the WSL ground station and produces the tarball
      that jetson_setup.sh consumes. Did you mean jetson_setup.sh?" 2
fi
ok "architecture ${arch} (ground station)"

for tool in git tar sha256sum awk sed; do
  command -v "${tool}" >/dev/null 2>&1 || die "'${tool}' is not installed"
done
ok "git, tar, sha256sum present"

[ -d "${REPO_ROOT}/flight_ops" ] || die "cannot find flight_ops -- REPO_ROOT resolved to ${REPO_ROOT}"
[ -d "${WS}/src" ] || die "no workspace at ${WS}/src. Pass --ws, or build the ground station workspace first."
ok "repo ${REPO_ROOT}"
ok "workspace ${WS}"

# --- px4_msgs provenance ---------------------------------------------------
PX4_SRC="${WS}/src/px4_msgs"
[ -d "${PX4_SRC}" ] || die "no ${PX4_SRC}"
px4_commit="$(git -C "${PX4_SRC}" rev-parse HEAD 2>/dev/null || echo unknown)"
px4_branch="$(git -C "${PX4_SRC}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
px4_desc="$(git -C "${PX4_SRC}" describe --tags --always 2>/dev/null || echo unknown)"
if [ "${px4_commit}" = "${PX4_MSGS_EXPECT_COMMIT}" ]; then
  ok "px4_msgs ${px4_desc} @ ${px4_commit:0:8} (${px4_branch})"
else
  warn "px4_msgs is at ${px4_commit:0:8}, expected ${PX4_MSGS_EXPECT_COMMIT:0:8} (${PX4_MSGS_BRANCH})."
  warn "  The bundle will ship what is here. Every Jetson will then run message"
  warn "  definitions that differ from what PATCHES.md was verified against."
fi
if [ -n "$(git -C "${PX4_SRC}" status --porcelain 2>/dev/null)" ]; then
  warn "px4_msgs has uncommitted local changes -- they WILL be shipped:"
  git -C "${PX4_SRC}" status --short | sed 's/^/        /'
fi

# --- as2_platform_pixhawk provenance and, critically, the patch ------------
PLAT_SRC="${WS}/src/as2_platform_pixhawk"
[ -d "${PLAT_SRC}" ] || die "no ${PLAT_SRC}"
plat_commit="$(git -C "${PLAT_SRC}" rev-parse HEAD 2>/dev/null || echo unknown)"
if [ "${plat_commit}" = "${PLATFORM_BASE_COMMIT}" ]; then
  ok "as2_platform_pixhawk base commit ${plat_commit:0:8}"
else
  warn "as2_platform_pixhawk HEAD is ${plat_commit:0:8}, expected base ${PLATFORM_BASE_COMMIT:0:8}"
fi

PATCH_FILE="${REPO_ROOT}/flight_ops/patches/as2_platform_pixhawk_o134.patch"
if [ -f "${PATCH_FILE}" ]; then
  if git -C "${PLAT_SRC}" apply --check --reverse "${PATCH_FILE}" >/dev/null 2>&1; then
    ok "O134 patch IS applied to the working tree (git apply --check --reverse)"
  else
    die "The O134 patch is NOT cleanly applied to ${PLAT_SRC}.
      Shipping this tree would put unpatched flight software on three aircraft:
      the kill switch would be a silent no-op and the external-odometry
      staleness gate would be absent. Fix it first:
        ${REPO_ROOT}/flight_ops/patches/apply_platform_patch.sh ${WS}"
  fi
else
  warn "no patch file at ${PATCH_FILE} -- provenance cannot be asserted"
fi
# Belt and braces: assert the actual content, not just the patch bookkeeping.
plat_cpp="${PLAT_SRC}/src/pixhawk_platform.cpp"
if [ -f "${plat_cpp}" ]; then
  grep -q "PX4_FORCE_DISARM_MAGIC" "${plat_cpp}" || die "kill-switch force-disarm (change 5) not present in ${plat_cpp}"
  grep -q "isExternalOdomFresh"    "${plat_cpp}" || die "external-odometry staleness gate (change 6) not present in ${plat_cpp}"
  ok "patch content verified in pixhawk_platform.cpp"
fi

# --------------------------------------------------------------------------- #
# 1. stage
# --------------------------------------------------------------------------- #
STAGE_PARENT="$(mktemp -d "${TMPDIR:-/tmp}/o134_bundle.XXXXXX")"
STAGE="${STAGE_PARENT}/${NAME}"
cleanup() {
  if [ "${KEEP_STAGE}" -eq 1 ]; then
    printf '[bundle] staging kept: %s\n' "${STAGE}"
  else
    rm -rf "${STAGE_PARENT}"
  fi
}
trap cleanup EXIT

mkdir -p "${STAGE}/src" "${STAGE}/flight_ops" "${STAGE}/apt" "${STAGE}/xrce"
log "staging in ${STAGE}"

# --- source packages. Copy the WORKING TREE (the patch lives there, not in a
#     commit), minus git metadata and build products.
copy_pkg() {
  local src="$1" dst="$2"
  tar -C "$(dirname "${src}")" \
      --exclude='.git' --exclude='build' --exclude='install' --exclude='log' \
      --exclude='__pycache__' --exclude='*.pyc' \
      -cf - "$(basename "${src}")" | tar -C "$(dirname "${dst}")" -xf -
}
copy_pkg "${PX4_SRC}"  "${STAGE}/src/px4_msgs"
ok "src/px4_msgs staged ($(du -sh "${STAGE}/src/px4_msgs" | awk '{print $1}'))"
copy_pkg "${PLAT_SRC}" "${STAGE}/src/as2_platform_pixhawk"
ok "src/as2_platform_pixhawk staged (patched)"

# --- as2_mocap_guarded (S1). The project's OWN package: a hardened fork of the
#     stock mocap_pose state-estimator plugin. It is what each drone actually
#     loads, so it has to be on every Jetson. It lives in the repo under
#     deckga_ros2/ and is copied (not symlinked, not a git repo) into the
#     workspace, so provenance here is a content comparison, not a commit.
GUARDED_REPO="${REPO_ROOT}/deckga_ros2/as2_mocap_guarded"
GUARDED_WS="${WS}/src/as2_mocap_guarded"
guarded_src=""
guarded_note=""
if [ -d "${GUARDED_WS}" ]; then
  guarded_src="${GUARDED_WS}"
  if [ -d "${GUARDED_REPO}" ]; then
    if diff -rq --exclude=build --exclude=install --exclude=log --exclude=.git \
         "${GUARDED_REPO}" "${GUARDED_WS}" >/dev/null 2>&1; then
      guarded_note="workspace copy, identical to deckga_ros2/as2_mocap_guarded"
    else
      guarded_note="workspace copy, DIFFERS from deckga_ros2/as2_mocap_guarded"
      warn "${GUARDED_WS} differs from the repo copy. Shipping the WORKSPACE copy"
      warn "  (it is what the ground station actually built). Reconcile them:"
      diff -rq --exclude=build --exclude=install --exclude=log --exclude=.git \
        "${GUARDED_REPO}" "${GUARDED_WS}" 2>&1 | sed 's/^/        /' | head -10
    fi
  else
    guarded_note="workspace copy (no repo copy to compare against)"
  fi
elif [ -d "${GUARDED_REPO}" ]; then
  guarded_src="${GUARDED_REPO}"
  guarded_note="repo copy (not present in ${WS}/src -- the ground station has not built it)"
  warn "as2_mocap_guarded is not in ${WS}/src. Shipping the repo copy, which the"
  warn "  ground station has therefore never compiled. It WILL be compiled on the Jetson."
else
  die "as2_mocap_guarded not found in ${WS}/src or ${REPO_ROOT}/deckga_ros2/.
      It is the S1 state-estimator plugin -- without it the drones fall back to the
      stock mocap_pose plugin, which publishes an ORIGIN POSE on a rigid-body name
      mismatch. That is the defect the fork exists to fix. Do not ship without it."
fi
[ -f "${guarded_src}/src/mocap_pose_guarded.cpp" ] \
  || die "${guarded_src} has no src/mocap_pose_guarded.cpp -- that is not the S1 package"
copy_pkg "${guarded_src}" "${STAGE}/src/as2_mocap_guarded"
ok "src/as2_mocap_guarded staged (${guarded_note})"

# --- flight_ops payload. Only what a Jetson uses or needs for provenance.
for d in deploy nodes patches lab_config; do
  if [ -d "${REPO_ROOT}/flight_ops/${d}" ]; then
    mkdir -p "${STAGE}/flight_ops/${d}"
    tar -C "${REPO_ROOT}/flight_ops" \
        --exclude='__pycache__' --exclude='*.pyc' \
        -cf - "${d}" | tar -C "${STAGE}/flight_ops" -xf -
    ok "flight_ops/${d} staged"
  else
    warn "flight_ops/${d} does not exist in the repo -- skipped"
  fi
done
chmod +x "${STAGE}"/flight_ops/deploy/*.sh 2>/dev/null || true
chmod +x "${STAGE}"/flight_ops/patches/*.sh 2>/dev/null || true

# --- the ROS archive key, so the Jetson never needs to reach raw.githubusercontent
if command -v curl >/dev/null 2>&1 && curl -fsSL --max-time 30 "${ROS_KEY_URL}" -o "${STAGE}/apt/ros.key" 2>/dev/null; then
  ok "apt/ros.key fetched ($(wc -c < "${STAGE}/apt/ros.key") bytes)"
  if command -v gpg >/dev/null 2>&1; then
    exp="$(gpg --show-keys --with-colons "${STAGE}/apt/ros.key" 2>/dev/null | awk -F: '/^pub/{print $7; exit}')"
    if [ -n "${exp}" ] && [ "${exp}" != "0" ]; then
      printf '        key expires %s\n' "$(date -u -d "@${exp}" '+%Y-%m-%d' 2>/dev/null || printf '%s' "${exp}")"
    fi
  fi
elif [ -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  cp /usr/share/keyrings/ros-archive-keyring.gpg "${STAGE}/apt/ros.key"
  warn "could not fetch the key; copied this machine's /usr/share/keyrings/ros-archive-keyring.gpg instead"
else
  rmdir "${STAGE}/apt" 2>/dev/null || true
  warn "no ROS archive key staged -- the Jetson will need to curl it itself"
fi

# --- Micro-XRCE-DDS-Agent. NOT an apt package on any architecture for Humble
#     (the buildfarm has no microxrcedds_agent or micro_ros_agent job), so the
#     source has to travel with the bundle.
if [ "${NO_XRCE}" -eq 1 ]; then
  warn "--no-xrce: the agent source is NOT in this bundle. jetson_setup.sh phase 'agent' will fail."
else
  log "cloning Micro-XRCE-DDS-Agent ${XRCE_TAG}"
  if git clone --quiet --depth 1 --branch "${XRCE_TAG}" "${XRCE_REPO}" \
       "${STAGE}/xrce/Micro-XRCE-DDS-Agent" 2>/dev/null; then
    xrce_commit="$(git -C "${STAGE}/xrce/Micro-XRCE-DDS-Agent" rev-parse HEAD)"
    rm -rf "${STAGE}/xrce/Micro-XRCE-DDS-Agent/.git"
    ok "xrce/Micro-XRCE-DDS-Agent ${XRCE_TAG} @ ${xrce_commit:0:8}"
  else
    xrce_commit="clone-failed"
    warn "clone failed. The Jetson has no way to get this by itself -- fix the ground"
    warn "station's network and re-run, or place a copy at ${STAGE}/xrce/ by hand."
  fi
fi

# --- aarch64 python wheels -------------------------------------------------
if [ "${WITH_WHEELS}" -eq 1 ]; then
  log "downloading aarch64 wheels (${WHEELS})"
  mkdir -p "${STAGE}/wheels"
  # shellcheck disable=SC2086
  if python3 -m pip download --quiet --only-binary=:all: \
       --platform "${JETSON_PLATFORM}" --python-version 310 \
       --implementation cp --abi "${JETSON_PY_TAG}" \
       -d "${STAGE}/wheels" ${WHEELS}; then
    ok "wheels: $(ls -1 "${STAGE}/wheels" | tr '\n' ' ')"
  else
    warn "wheel download failed. pyserial is pure-python (py3-none-any) and pymavlink"
    warn "may need --platform any; the Jetsons already have these installed, so this"
    warn "is a convenience, not a blocker."
  fi
fi

# --- arm64 .deb closure ----------------------------------------------------
stage_debs() {
  cat <<'DEBWARN'

  ------------------------------------------------------------------------
  --with-debs is UNVERIFIED. It builds an isolated apt root that pretends to
  be the Jetson (using its /var/lib/dpkg/status), resolves the arm64 package
  closure against ports.ubuntu.com and packages.ros.org, and downloads it.

  It is fragile by nature: the answer depends on the Jetson's exact installed
  state, and JetPack pins some system libraries to NVIDIA versions that the
  resolver does not know about. Treat what it produces as a best effort.

  THE ROBUST PATH IS THE OTHER ONE: give the Jetson wifi with a working
  default route for ten minutes and let apt do its job. See the README's
  phantom-gateway note -- that is usually what "no internet" really is.
  ------------------------------------------------------------------------

DEBWARN
  [ -n "${JETSON_STATUS}" ] || die "--with-debs requires --jetson-status FILE.
      Get it with:  scp ${JETSON_HOST}:/var/lib/dpkg/status ./jetson_dpkg_status"
  [ -r "${JETSON_STATUS}" ] || die "cannot read ${JETSON_STATUS}"
  grep -q '^Package:' "${JETSON_STATUS}" || die "${JETSON_STATUS} is not a dpkg status file
      (expected /var/lib/dpkg/status, not the output of dpkg-query -W)"

  local root="${STAGE_PARENT}/aptroot"
  mkdir -p "${root}/etc/apt/apt.conf.d" "${root}/etc/apt/preferences.d" \
           "${root}/etc/apt/sources.list.d" "${root}/etc/apt/trusted.gpg.d" \
           "${root}/var/lib/apt/lists/partial" "${root}/var/lib/dpkg" \
           "${root}/var/cache/apt/archives/partial" "${root}/var/log/apt"
  cp "${JETSON_STATUS}" "${root}/var/lib/dpkg/status"
  : > "${root}/var/lib/dpkg/available"

  # arm64 lives on ports.ubuntu.com, not archive.ubuntu.com.
  cat > "${root}/etc/apt/sources.list" <<'SRCEOF'
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy main universe restricted multiverse
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-updates main universe restricted multiverse
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-security main universe restricted multiverse
deb [arch=arm64] http://packages.ros.org/ros2/ubuntu jammy main
SRCEOF
  cp /usr/share/keyrings/*.gpg "${root}/etc/apt/trusted.gpg.d/" 2>/dev/null || true
  [ -f "${STAGE}/apt/ros.key" ] && cp "${STAGE}/apt/ros.key" "${root}/etc/apt/trusted.gpg.d/ros.asc" 2>/dev/null || true

  local aptopts=(
    -o "Dir=${root}"
    -o "Dir::State=${root}/var/lib/apt"
    -o "Dir::State::status=${root}/var/lib/dpkg/status"
    -o "Dir::Cache=${root}/var/cache/apt"
    -o "Dir::Etc=${root}/etc/apt"
    -o "Dir::Etc::sourcelist=${root}/etc/apt/sources.list"
    -o "Dir::Etc::sourceparts=${root}/etc/apt/sources.list.d"
    -o "Dir::Etc::trusted=${root}/etc/apt/trusted.gpg"
    -o "Dir::Etc::trustedparts=${root}/etc/apt/trusted.gpg.d"
    -o "APT::Architecture=arm64"
    -o "APT::Architectures=arm64"
    -o "APT::Get::AllowUnauthenticated=false"
    -o "Debug::NoLocking=true"
  )

  log "updating the isolated arm64 apt root (this downloads index files only)"
  apt-get "${aptopts[@]}" update >/dev/null 2>&1 \
    || die "apt-get update failed inside the isolated root. Check the ground station's network."

  local pkgs
  pkgs="$(sed -e 's/#.*$//' "${SCRIPT_DIR}/packages.txt" \
          | grep -v '^[[:space:]]*$' | sed 's/^@optional[[:space:]]*//' | awk '{print $1}' | tr '\n' ' ')"
  log "resolving the closure for: ${pkgs}"

  local uris
  # shellcheck disable=SC2086
  uris="$(apt-get "${aptopts[@]}" install -y --print-uris --no-install-recommends ${pkgs} 2>/dev/null \
          | awk -F"'" '/^'"'"'http/{print $2}')" || true
  if [ -z "${uris}" ]; then
    warn "the resolver returned no URIs. Either everything is already installed on the"
    warn "Jetson (check ${JETSON_STATUS}) or the resolution failed. Nothing staged."
    return 0
  fi
  mkdir -p "${STAGE}/debs"
  local n=0
  local u
  for u in ${uris}; do
    if curl -fsSL --max-time 120 -O --output-dir "${STAGE}/debs" "${u}" 2>/dev/null; then
      n=$((n + 1))
    else
      warn "failed to download ${u}"
    fi
  done
  ok "debs staged: ${n} files, $(du -sh "${STAGE}/debs" 2>/dev/null | awk '{print $1}')"
  cat > "${STAGE}/debs/INSTALL_ORDER.txt" <<'ORDEREOF'
These .deb files were resolved against a snapshot of THIS Jetson's dpkg status.
Install them all at once so apt can order them itself:

    sudo apt-get install -y --no-install-recommends ./*.deb

jetson_setup.sh does exactly that when run with --offline. If apt complains
about unmet dependencies, the Jetson's installed state has drifted from the
status file used to resolve the closure -- re-take the status file and re-bundle.
ORDEREOF
}
[ "${WITH_DEBS}" -eq 1 ] && stage_debs

# --------------------------------------------------------------------------- #
# 2. provenance and checksums
# --------------------------------------------------------------------------- #
cat > "${STAGE}/BUNDLE_INFO.txt" <<INFOEOF
O134 Jetson deployment bundle
=============================
name              : ${NAME}
built (UTC)       : $(date -u '+%Y-%m-%dT%H:%M:%SZ')
built on          : $(hostname) / $(uname -srm)
built by          : $(id -un)
ground station OS : $( . /etc/os-release 2>/dev/null && printf '%s' "${PRETTY_NAME:-unknown}" )
source workspace  : ${WS}
repo              : ${REPO_ROOT}

src/px4_msgs
  branch          : ${px4_branch}
  describe        : ${px4_desc}
  commit          : ${px4_commit}
  expected branch : ${PX4_MSGS_BRANCH}
  expected commit : ${PX4_MSGS_EXPECT_COMMIT}

src/as2_platform_pixhawk
  base commit     : ${plat_commit}
  expected base   : ${PLATFORM_BASE_COMMIT}
  local patch     : flight_ops/patches/as2_platform_pixhawk_o134.patch
  patch applied   : yes (verified by git apply --check --reverse AND by content)

src/as2_mocap_guarded
  source          : ${guarded_src}
  provenance      : ${guarded_note}
  version         : $(grep -m1 -oP '(?<=<version>)[^<]+' "${guarded_src}/package.xml" 2>/dev/null || echo '?')
  note            : S1. Hardened fork of the stock mocap_pose plugin. Loaded by
                    the stock as2_state_estimator via plugin_name: mocap_pose_guarded.
                    It does NOT modify the installed as2_state_estimator.

xrce/Micro-XRCE-DDS-Agent
  tag             : ${XRCE_TAG}
  commit          : ${xrce_commit:-not staged}
  note            : NOT available from apt for Humble on any architecture.
                    Built on the Jetson with UAGENT_SUPERBUILD=OFF against the
                    Fast-DDS that ROS 2 Humble already provides, so no network
                    is needed at build time.

aerostack2
  ground station  : $(dpkg-query -W -f='${Version} (${Architecture})' ros-humble-aerostack2 2>/dev/null || echo 'not installed here')
  on the Jetson   : installed from apt as the ros-humble-as2-* COMPONENT packages
                    listed in flight_ops/deploy/packages.txt. The ros-humble-aerostack2
                    metapackage is no longer in the jammy arm64 index (observed
                    2026-09-11); the components are, at 1.1.3.

extras
  wheels          : $( [ -d "${STAGE}/wheels" ] && ls -1 "${STAGE}/wheels" | wc -l || echo 0 ) file(s)
  debs            : $( [ -d "${STAGE}/debs" ] && ls -1 "${STAGE}/debs"/*.deb 2>/dev/null | wc -l || echo 0 ) file(s)

On the Jetson
  tar -xzf ${NAME}.tar.gz -C ~
  ~/${NAME}/flight_ops/deploy/jetson_setup.sh --drone-ns droneN
  ~/${NAME}/flight_ops/deploy/verify_jetson.sh --drone-ns droneN
INFOEOF
ok "BUNDLE_INFO.txt written"

cat > "${STAGE}/README_FIRST.txt" <<FIRSTEOF
Read flight_ops/deploy/README.md in this bundle. The three-line version:

  1. tar -xzf ${NAME}.tar.gz -C ~
  2. ~/${NAME}/flight_ops/deploy/jetson_setup.sh --drone-ns drone0 \\
         --gs-host <ground station ip> --peer <other jetson> --peer <other jetson>
  3. source ~/as2_o134_ws/setup_env.sh
     ~/${NAME}/flight_ops/deploy/verify_jetson.sh --drone-ns drone0 --with-fmu

Nothing is added to .bashrc, systemd or cron. To undo everything:
     ~/as2_o134_ws/.deploy/rollback.sh            # dry run
     ~/as2_o134_ws/.deploy/rollback.sh --apply
FIRSTEOF

log "computing checksums"
( cd "${STAGE}" && find . -type f ! -name MANIFEST.sha256 -print0 \
    | sort -z | xargs -0 sha256sum > MANIFEST.sha256 )
n_files="$(wc -l < "${STAGE}/MANIFEST.sha256")"
ok "MANIFEST.sha256: ${n_files} files"

# --------------------------------------------------------------------------- #
# 3. tar it
# --------------------------------------------------------------------------- #
mkdir -p "${OUT_DIR}"
TARBALL="${OUT_DIR}/${NAME}.tar.gz"
log "writing ${TARBALL}"
tar -C "${STAGE_PARENT}" -czf "${TARBALL}" "${NAME}"
# Write the checksum with a BARE filename, not the build path: it is checked on
# the Jetson, in whatever directory the tarball lands in.
( cd "${OUT_DIR}" && sha256sum "${NAME}.tar.gz" > "${NAME}.tar.gz.sha256" )
size="$(du -h "${TARBALL}" | awk '{print $1}')"
ok "${TARBALL} (${size})"
ok "${TARBALL}.sha256"

# --------------------------------------------------------------------------- #
# 4. how to deliver it
# --------------------------------------------------------------------------- #
cat <<DELIVEREOF

==============================================================
Deliver it. Key authentication only -- no password goes in this
repository, and none should be typed into a script.

  # once per Jetson, if you have not already:
  ssh-copy-id ${JETSON_HOST}

  # copy the bundle and its checksum
  scp ${TARBALL} ${TARBALL}.sha256 ${JETSON_HOST}:~/

  # verify the transfer ON THE JETSON before unpacking
  ssh ${JETSON_HOST} 'sha256sum -c ~/$(basename "${TARBALL}").sha256'

  # unpack and run
  ssh ${JETSON_HOST} 'tar -xzf ~/$(basename "${TARBALL}") -C ~'
  ssh -t ${JETSON_HOST} '~/${NAME}/flight_ops/deploy/jetson_setup.sh --drone-ns drone0'

Repeat for the other two aircraft with --drone-ns drone1 / drone2.
The bundle is identical for all three; only --drone-ns differs.
==============================================================
DELIVEREOF
