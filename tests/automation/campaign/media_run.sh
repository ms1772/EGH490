#!/bin/bash
# Dedicated media run: thesis figures and the mission video.
#
# WHY THIS IS SEPARATE FROM THE CAMPAIGN
# --------------------------------------
# Two reasons, both measured:
#
# 1. Window size. The campaign's screenshots come out 581x797 -- the 3D viewport
#    inside a 1000x845 window -- which is small for a figure. Enlarging it is a
#    one-line config change, but LIBGL_ALWAYS_SOFTWARE=1 means SOFTWARE
#    rasterisation, so a 1600x1000 viewport is ~4x the pixels and ~4x the
#    per-frame CPU cost. The GUI is up on every one of the 80 campaign trials,
#    so enlarging it there would load all of them for the benefit of four.
#
# 2. Frame capture. A frame series fires `ign service` subprocesses DURING the
#    mission. The campaign's captured trials are also data rows feeding the
#    paired comparison, so they take stills only (outside the flight window,
#    perturbing nothing). Video needs frames during flight.
#
# So: the campaign optimises for clean data, this run optimises for pictures,
# and the rows it produces are DISCARDED. Point --offline-run-dir at the
# campaign so it flies exactly the same plans.
#
# gui.config is a shared user config. It is snapshotted into the run dir and
# restored by an EXIT trap, so an interrupt or a failure still puts it back.
set -u

REPO="/mnt/c/Users/mitch/Multi-UAV Project/ROS2_MultiUAV_3D-main"
GUI="$HOME/.ignition/gazebo/6/gui.config"
RUN_DIR="${1:-$REPO/tests/automation/results/media_$(date +%Y%m%d-%H%M%S)}"
OFFLINE="${OFFLINE:-$REPO/tests/automation/results/campaign_20261011}"
WIDTH="${WIDTH:-1600}"
HEIGHT="${HEIGHT:-1000}"
# A0 vs A4 at the largest fleet in the densest Gazebo condition: the comparison
# the thesis makes, in the scene where avoidance is most visible.
TRIALS="${TRIALS:-gzc_n100_o20_s0_A0_published,gzc_n100_o20_s0_A4_qn_minmax_p0}"
PERIOD="${PERIOD:-2.0}"

mkdir -p "$RUN_DIR"
if [ ! -f "$GUI" ]; then
  echo "no gui.config at $GUI -- launch Gazebo once to create it"; exit 2
fi
cp "$GUI" "$RUN_DIR/gui.config.backup"
restore_gui() {
  cp "$RUN_DIR/gui.config.backup" "$GUI" && echo "[media] restored $GUI"
}
trap restore_gui EXIT INT TERM

python3 - "$GUI" "$WIDTH" "$HEIGHT" <<'PY'
import re, sys
path, w, h = sys.argv[1], sys.argv[2], sys.argv[3]
s = open(path, encoding="utf-8").read()
s, nw = re.subn(r"<width>\d+</width>", f"<width>{w}</width>", s, count=1)
s, nh = re.subn(r"<height>\d+</height>", f"<height>{h}</height>", s, count=1)
if not (nw and nh):
    raise SystemExit("could not find <width>/<height> in gui.config")
open(path, "w", encoding="utf-8").write(s)
print(f"[media] gui.config window -> {w}x{h}")
PY

set +u
source /opt/ros/humble/setup.bash
set -u
export DISPLAY="${DISPLAY:-:0}"
export LIBGL_ALWAYS_SOFTWARE=1
export MPLBACKEND=Agg
cd "$REPO" || exit 2

echo "[media] run_dir=$RUN_DIR"
echo "[media] trials=$TRIALS  frame period=${PERIOD}s"

IFS=',' read -r -a IDS <<< "$TRIALS"
for id in "${IDS[@]}"; do
  echo "[media] ===== $id ====="
  # One invocation per trial: sweep_gazebo takes --only-trial (singular), and
  # running the whole matrix here would fly all 80 again.
  python3 -u tests/automation/sweep_gazebo.py \
    --matrix campaign_gazebo --mode headless --no-prompt \
    --offline-run-dir "$OFFLINE" --run-dir "$RUN_DIR" \
    --only-trial "$id" --rtf 1.0 --executor-timeout 3600 \
    --capture-trials "$id" --capture-period "$PERIOD"
  echo "[media] $id exit $?"
done

# Assemble each frame series. Done after the flights, from plain PNGs, so no
# live session is needed and a failed encode costs nothing.
for id in "${IDS[@]}"; do
  d="$RUN_DIR/media/$id"
  n=$(ls "$d"/${id}_frame_*.png 2>/dev/null | wc -l)
  if [ "$n" -gt 0 ]; then
    out="$RUN_DIR/media/${id}.mp4"
    # pad to EVEN dimensions. libx264 refuses an odd width or height, and the
    # enlarged capture window produces 1181x952 -- odd width -- so a plain
    # encode fails with "width not divisible by 2" only AFTER both flights are
    # finished, which is the most expensive moment to discover it. pad rather
    # than scale, so no pixel is resampled; it adds at most one row/column.
    ffmpeg -y -loglevel error -framerate 10 -pattern_type glob \
      -i "$d/${id}_frame_*.png" \
      -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" \
      -c:v libx264 -pix_fmt yuv420p -crf 20 "$out" \
      && echo "[media] wrote $out ($n frames)" \
      || echo "[media] ffmpeg failed for $id"
  else
    echo "[media] no frames for $id"
  fi
done

echo "[media] stills and video under $RUN_DIR/media/"
