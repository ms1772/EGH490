#!/usr/bin/env bash
# show_rviz.sh - open the DECK-GA + QuickNav visualisation in RViz.
#
# Starts the path + obstacle publisher nodes, opens RViz, then automatically
# stops the publishers again when RViz closes. It reads only, changes no
# files, and kills only the processes it starts.
#
# Run from the Ubuntu-22.04 WSL distro:
#   bash deckga_ros2/show_rviz.sh             # QuickNav (avoided) paths + obstacles
#   bash deckga_ros2/show_rviz.sh --baseline  # also overlay DECK-GA (no-avoidance) paths
#   bash deckga_ros2/show_rviz.sh --pkl PATH  # use a different output pkl
#   bash deckga_ros2/show_rviz.sh --check     # run all preflight checks, launch nothing

# Re-exec under bash if started with sh/dash (this script uses bash arrays).
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi

ROS_SETUP="/opt/ros/humble/setup.bash"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKL="$SCRIPT_DIR/data/deckga_quicknav_output.pkl"
PATH_KEY="quicknav_paths"
SHOW_BASELINE=0
CHECK_ONLY=0

err() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

# ---- args --------------------------------------------------------------------
while [ $# -gt 0 ]; do
  case "$1" in
    --baseline) SHOW_BASELINE=1; shift ;;
    --check)    CHECK_ONLY=1; shift ;;
    --pkl)      [ -n "${2:-}" ] || err "--pkl needs a path"; PKL="$2"; shift 2 ;;
    -h|--help)
      cat <<'USAGE'
show_rviz.sh - open the DECK-GA + QuickNav visualisation in RViz.
  bash deckga_ros2/show_rviz.sh             QuickNav (avoided) paths + obstacles
  bash deckga_ros2/show_rviz.sh --baseline  also overlay DECK-GA (no-avoidance) paths
  bash deckga_ros2/show_rviz.sh --pkl PATH  use a different output pkl
  bash deckga_ros2/show_rviz.sh --check     run preflight checks only
USAGE
      exit 0 ;;
    *) err "unknown option: $1 (try --help)" ;;
  esac
done

# ---- preflight (fail early and clearly, before launching anything) ----------
[ -f "$ROS_SETUP" ] || err "ROS 2 not found at $ROS_SETUP - are you in the Ubuntu-22.04 WSL distro?"
# shellcheck source=/dev/null
source "$ROS_SETUP"

command -v python3 >/dev/null 2>&1 || err "python3 not on PATH"
command -v rviz2   >/dev/null 2>&1 || err "rviz2 not found (needs ros-humble-desktop)"
python3 -c "import rclpy" >/dev/null 2>&1 || err "rclpy not importable even after sourcing ROS"

NODE_PATHS="$SCRIPT_DIR/rviz_paths_node.py"
NODE_OBS="$SCRIPT_DIR/rviz_obstacles_node.py"
[ -f "$NODE_PATHS" ] || err "missing node script: $NODE_PATHS"
[ -f "$NODE_OBS" ]   || err "missing node script: $NODE_OBS"
[ -f "$PKL" ]        || err "data pkl not found: $PKL
       Generate one first, e.g. with DECK_GA_QuickNav.py --out_pkl <path>."

# Confirm the pkl actually holds the keys the nodes need.
python3 - "$PKL" "$PATH_KEY" "$SHOW_BASELINE" <<'PY'
import pickle, sys
pkl, key, want_base = sys.argv[1], sys.argv[2], sys.argv[3] == "1"
try:
    with open(pkl, "rb") as f:
        d = pickle.load(f)
except Exception as e:
    sys.exit("ERROR: cannot read pkl: %s" % e)
if not isinstance(d, dict):
    sys.exit("ERROR: pkl is not a dict")
need = [key, "obstacle_xyz", "obs_size"] + (["deckga_paths"] if want_base else [])
missing = [k for k in need if k not in d]
if missing:
    sys.exit("ERROR: pkl missing keys %s. Present: %s" % (missing, list(d.keys())))
print("  pkl OK: %d path(s), %d obstacle(s)" % (len(d[key]), len(d["obstacle_xyz"])))
PY
[ $? -eq 0 ] || exit 1

[ -n "${DISPLAY:-}" ] || echo "WARNING: \$DISPLAY is empty - RViz may not open a window (WSLg normally sets it)."

echo "  ROS:      $ROS_SETUP"
echo "  pkl:      $PKL"
echo "  path_key: $PATH_KEY   baseline: $SHOW_BASELINE"

if [ "$CHECK_ONLY" -eq 1 ]; then
  echo "ALL CHECKS PASSED (--check: nothing launched)."
  exit 0
fi

# ---- launch with guaranteed cleanup -----------------------------------------
PIDS=()
cleanup() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    [ -n "$pid" ] && kill "$pid" 2>/dev/null
  done
}
trap cleanup EXIT INT TERM

echo "Starting publisher nodes..."
python3 "$NODE_PATHS" --deckga_pkl "$PKL" --path_key "$PATH_KEY" & PIDS+=("$!")
python3 "$NODE_OBS"   --deckga_pkl "$PKL"                        & PIDS+=("$!")
if [ "$SHOW_BASELINE" -eq 1 ]; then
  python3 "$NODE_PATHS" --deckga_pkl "$PKL" --path_key deckga_paths \
          --topic /deckga/markers_baseline & PIDS+=("$!")
fi

sleep 1  # let the publishers come up before RViz subscribes

echo ""
echo "RViz is opening. Configure it (first time only):"
echo "  - Global Options -> Fixed Frame = earth"
echo "  - Add -> By topic -> /deckga/markers      (MarkerArray)  = $PATH_KEY paths"
echo "  - Add -> By topic -> /deckga/obstacles    (MarkerArray)  = obstacle cubes"
[ "$SHOW_BASELINE" -eq 1 ] && \
echo "  - Add -> By topic -> /deckga/markers_baseline (MarkerArray) = DECK-GA, no avoidance"
echo ""
echo "Close the RViz window (or press Ctrl-C here) to stop the publishers automatically."
echo ""

rviz2
# rviz2 exits -> EXIT trap runs cleanup() -> publisher nodes stopped. No orphans.
