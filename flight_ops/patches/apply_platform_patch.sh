#!/usr/bin/env bash
#
# apply_platform_patch.sh
#
# Reproduce the O134 build of as2_platform_pixhawk against the installed
# Aerostack2 1.1.3 (binary/apt) and px4_msgs release/1.17 (source, version 2.0.1).
#
#   - clones as2_platform_pixhawk into the workspace if it is not there yet
#   - checks out the pinned base commit
#   - applies as2_platform_pixhawk_o134.patch
#   - builds only as2_platform_pixhawk
#   - reports PASS / FAIL and exits non-zero on failure
#
# Idempotent: safe to re-run. If the patch is already applied the checkout is left
# alone; otherwise the working tree is force-reset to the base commit first.
#
# It deliberately does NOT touch px4_msgs: that package is expected to be already
# built and installed in the workspace (see PATCHES.md).
#
# Usage:
#   ./apply_platform_patch.sh [WORKSPACE_DIR]
# Default WORKSPACE_DIR is ~/as2_o134_ws
#
set -uo pipefail

BASE_COMMIT="2b00b77dcd2a4e3f7f607ef043bc8cb85e215b88"
REPO_URL="https://github.com/aerostack2/as2_platform_pixhawk.git"
PKG="as2_platform_pixhawk"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH_FILE="${SCRIPT_DIR}/as2_platform_pixhawk_o134.patch"

WS="${1:-$HOME/as2_o134_ws}"
SRC="${WS}/src/${PKG}"

log()  { printf '[apply_platform_patch] %s\n' "$*"; }
fail() { printf '[apply_platform_patch] FAIL: %s\n' "$*" >&2; exit 1; }

[ -f "${PATCH_FILE}" ] || fail "patch not found: ${PATCH_FILE}"

# ---------------------------------------------------------------- 1. sources
mkdir -p "${WS}/src"

if [ ! -d "${SRC}/.git" ]; then
  log "cloning ${REPO_URL} -> ${SRC}"
  git clone "${REPO_URL}" "${SRC}" || fail "git clone failed"
else
  log "using existing checkout ${SRC}"
fi

# Make sure the pinned commit is present even in a shallow / stale clone.
if ! git -C "${SRC}" cat-file -e "${BASE_COMMIT}^{commit}" 2>/dev/null; then
  log "fetching full history to reach ${BASE_COMMIT}"
  git -C "${SRC}" fetch --unshallow origin 2>/dev/null || git -C "${SRC}" fetch origin
fi
git -C "${SRC}" cat-file -e "${BASE_COMMIT}^{commit}" 2>/dev/null \
  || fail "base commit ${BASE_COMMIT} not found in ${SRC}"

# ------------------------------------------------------------- 2. apply patch
if git -C "${SRC}" apply --check --reverse "${PATCH_FILE}" 2>/dev/null; then
  log "patch already applied, leaving the working tree as it is"
else
  log "checking out base commit ${BASE_COMMIT} (discards local modifications)"
  git -C "${SRC}" checkout --quiet --force "${BASE_COMMIT}" || fail "git checkout failed"
  log "applying ${PATCH_FILE}"
  git -C "${SRC}" apply --check "${PATCH_FILE}" || fail "git apply --check failed"
  git -C "${SRC}" apply "${PATCH_FILE}" || fail "git apply failed"
  log "patch applied"
fi

# ----------------------------------------------------------- 3. sanity checks
if [ ! -d "${WS}/src/px4_msgs" ]; then
  log "WARNING: ${WS}/src/px4_msgs is missing."
  log "         Expected px4_msgs branch release/1.17 (version 2.0.1), already built."
fi

# ------------------------------------------------------------------- 4. build
# 'set +u' is required: the ROS setup scripts read unset variables.
# '-DBUILD_TESTING=OFF' is required: ament_lint_auto is not installed on this machine.
# 'PYTHONNOUSERSITE=1' keeps colcon off the user site-packages.
log "building ${PKG} in ${WS}"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash                              || fail "cannot source ROS 2 humble"
# shellcheck disable=SC1091
source /opt/ros/humble/share/aerostack2/local_setup.bash        || fail "cannot source aerostack2"
set -u

pushd "${WS}" > /dev/null || fail "cannot cd to ${WS}"
PYTHONNOUSERSITE=1 colcon build \
  --packages-select "${PKG}" \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
BUILD_RC=$?
popd > /dev/null || true

NODE_BIN="${WS}/install/${PKG}/lib/${PKG}/${PKG}_node"

echo
if [ "${BUILD_RC}" -eq 0 ] && [ -e "${NODE_BIN}" ]; then
  log "PASS: ${PKG} built successfully"
  log "      node: ${NODE_BIN}"
  log "      run:  ros2 run ${PKG} ${PKG}_node --ros-args -p external_odom_timeout_s:=0.1 ..."
  exit 0
fi

log "FAIL: colcon exit code ${BUILD_RC}"
if [ ! -e "${NODE_BIN}" ]; then
  log "      node executable not found at ${NODE_BIN}"
fi
log "      see ${WS}/log/latest_build/${PKG}/stdout_stderr.log"
exit 1
