#!/usr/bin/env bash
# Build the O-134 mission end to end: inputs -> plan -> geometry gate.
#
#   bash flight_ops/lab_config/o134_mission/make_mission.sh
#   bash flight_ops/lab_config/o134_mission/make_mission.sh --offset 0.5 0 0
#
# Takes ~1.5-2.5 min: the GA runs its full numIter regardless of how few
# waypoints it is given, so this is not something to start on the flight line.
# Build it the day before; re-run it on the day only if the measured room
# forces an --offset.
#
# It deliberately runs under /opt/ros/humble's python. deckga_execute.py will
# read this pickle with that same interpreter (numpy 1.21) on flight day, and a
# pickle written by a numpy 2.x python is unreadable to it -- the failure is
# "ModuleNotFoundError: No module named 'numpy._core'" and it happens at load
# time, with the aircraft on the pad.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../../.." && pwd)"

OFFSET=(0 0 0)
while [[ $# -gt 0 ]]; do
  case "$1" in
    --offset) OFFSET=("$2" "$3" "$4"); shift 4 ;;
    -h|--help) sed -n '2,14p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

set +u
# shellcheck disable=SC1091
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
set -u

cd "${REPO}"
PY=python3
echo "=== python: $(${PY} -c 'import sys,numpy;print(sys.version.split()[0], "numpy", numpy.__version__)')"

echo
echo "=== 1/3  mission inputs ==="
${PY} "${HERE}/mission_spec.py" --out_dir "${HERE}" --offset "${OFFSET[@]}"

START_POINTS="$(${PY} "${HERE}/mission_spec.py" --print_only --offset "${OFFSET[@]}" \
                | sed -n 's/^--start_points "\(.*\)"$/\1/p')"
if [[ -z "${START_POINTS}" ]]; then
  echo "could not derive --start_points from mission_spec.py" >&2
  exit 1
fi
# NOTE the "=" in --start_points=... below. The first start point's x is
# negative, so the value begins with "-" and argparse reads it as a flag if
# it is passed as a separate word: "error: argument --start_points: expected
# one argument", with the mission half built.
echo "start_points: ${START_POINTS}"

echo
echo "=== 2/3  DECK-GA + QuickNav (this is the slow part) ==="
${PY} DECK_GA_QuickNav.py \
  --points_pkl "${HERE}/o134_points.pkl" \
  --obstacles_pkl "${HERE}/o134_obstacles.pkl" \
  --num_uavs 3 \
  --start_points="${START_POINTS}" \
  --no_plot \
  --out_pkl "${HERE}/o134_mission.pkl"

echo
echo "=== 3/3  offline geometry gate ==="
${PY} "${HERE}/check_mission.py" --pkl "${HERE}/o134_mission.pkl" --verbose
