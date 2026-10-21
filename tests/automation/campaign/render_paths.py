#!/usr/bin/env python3
"""Planned-vs-flown 3-D figures, rendered from saved artefacts only.

NEEDS NO SIMULATOR. Inputs are a planner pickle (planned paths, obstacles,
start points) and a trial's bag (flown poses). Both are permanent artefacts of
the campaign, so these figures can be regenerated or restyled indefinitely
without re-flying anything -- which is the whole point of recording the bag.

WHY MATPLOTLIB RATHER THAN RVIZ
-------------------------------
The plan allowed either. RViz needs a live X session, the right .rviz config,
two marker nodes republishing from the pickle, and an ImageMagick `import
-window` capture whose window name has to match -- several moving parts, each
able to fail silently into a blank image. Note also that the harness's own
`rviz2_config.rviz` has ZERO `/deckga/*` displays, so a screenshot of it shows
drones and TF but neither paths nor obstacles; the config that does work is
`flight_ops/lab_config/o134_project/config_ground_station/o134.rviz`.

This renders the same information from the same data with no live session, and
produces a vector-friendly figure at a chosen DPI instead of a screen-resolution
PNG. The RViz route remains available for an interactive look.

COLOURS, for the record: obstacles are RED in `rviz_obstacles_node.py:116-119`,
not cyan as the harness's prompt text claims.
"""
from __future__ import annotations

import argparse
import pickle
import sys
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib
matplotlib.use("Agg")                 # before pyplot; there is no display here
import matplotlib.pyplot as plt       # noqa: E402
import numpy as np                    # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

UAV_COLOURS = ["#1f77b4", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b"]
OBSTACLE_COLOUR = "#d62728"           # red, matching rviz_obstacles_node
FLOWN_RATE_HZ = 5.0                   # plenty for a figure; full rate is noise


def _load_pickle(path: Path) -> dict:
    with path.open("rb") as f:
        return pickle.load(f)


def _as_xyz(value) -> np.ndarray:
    """(N,3) float array from a pickle field that may be None, list or ndarray.

    `value or []` is WRONG here and raises: these fields are numpy arrays, and
    `or` evaluates the array's truth value, which is ambiguous for more than one
    element. The None check has to be explicit.
    """
    if value is None:
        return np.zeros((0, 3), dtype=float)
    arr = np.asarray(value, dtype=float)
    if arr.size == 0:
        return np.zeros((0, 3), dtype=float)
    return arr.reshape(-1, 3)


def _cube(ax, centre, half, colour=OBSTACLE_COLOUR, alpha=0.18):
    """Draw an axis-aligned cube. The planner's obstacles are AABBs of
    half-size `obs_size`, so this is the exact collision geometry, not a
    stylised marker."""
    c = np.asarray(centre, dtype=float)
    h = float(half)
    # 6 faces as quads
    for axis in range(3):
        for sign in (-1.0, 1.0):
            idx = [i for i in range(3) if i != axis]
            corners = []
            for a in (-1.0, 1.0):
                for b in (-1.0, 1.0):
                    p = np.zeros(3)
                    p[axis] = sign * h
                    p[idx[0]] = a * h
                    p[idx[1]] = b * h
                    corners.append(c + p)
            # order the 4 corners into a proper quad
            quad = [corners[0], corners[1], corners[3], corners[2]]
            from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: PLC0415
            poly = Poly3DCollection([quad], alpha=alpha)
            poly.set_facecolor(colour)
            poly.set_edgecolor("none")
            ax.add_collection3d(poly)


def _flown_tracks(bag: Path, num_uavs: int) -> Dict[int, np.ndarray]:
    from tests.automation.collision_audit import read_bag_poses  # noqa: PLC0415
    out: Dict[int, np.ndarray] = {}
    for i in range(num_uavs):
        topic = f"/drone{i}/self_localization/pose"
        try:
            samples = read_bag_poses(bag, topic, downsample_hz=FLOWN_RATE_HZ)
        except Exception as e:                                # noqa: BLE001
            print(f"  [warn] {topic}: {e}")
            continue
        if samples:
            out[i] = np.asarray([p for _, p in samples], dtype=float).reshape(-1, 3)
    return out


def render(pickle_path: Path, bag_path: Optional[Path], out_png: Path,
           path_key: str = "quicknav_paths", dpi: int = 200,
           title: Optional[str] = None, elev: float = 22.0,
           azim: float = -58.0,
           figsize: tuple = (11, 8.5),
           legend_anchor=None) -> Path:
    data = _load_pickle(pickle_path)
    # Explicit None check for the same reason as _as_xyz: this is a list of
    # arrays today, but `or []` would raise the moment it became an ndarray.
    raw_paths = data.get(path_key)
    paths: List[np.ndarray] = [np.asarray(p, dtype=float).reshape(-1, 3)
                               for p in (raw_paths if raw_paths is not None else [])]
    if not paths:
        raise SystemExit(f"{pickle_path} has no '{path_key}'")
    obstacles = _as_xyz(data.get("obstacle_xyz"))
    obs_size = data.get("obs_size")
    if isinstance(obs_size, (list, tuple, np.ndarray)):
        obs_half = float(np.asarray(obs_size).reshape(-1)[0]) if len(obs_size) else 5.0
    else:
        obs_half = float(obs_size if obs_size is not None else 5.0)
    starts = _as_xyz(data.get("start_points"))

    flown = _flown_tracks(bag_path, len(paths)) if bag_path and bag_path.exists() else {}

    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection="3d")

    for oc in obstacles:
        _cube(ax, oc, obs_half)

    # Flown UNDERNEATH, planned dashed on top. The two nearly coincide -- which
    # is the result -- so drawing the solid line last would hide the dashed one
    # and make the figure look like it only shows one thing.
    for i, p in enumerate(paths):
        col = UAV_COLOURS[i % len(UAV_COLOURS)]
        if i in flown:
            q = flown[i]
            ax.plot(q[:, 0], q[:, 1], q[:, 2], "-", color=col, linewidth=2.6,
                    alpha=0.35, label=f"UAV {i} flown")
    for i, p in enumerate(paths):
        col = UAV_COLOURS[i % len(UAV_COLOURS)]
        ax.plot(p[:, 0], p[:, 1], p[:, 2], "--", color=col, linewidth=1.4,
                label=f"UAV {i} planned")
        ax.scatter(p[:, 0], p[:, 1], p[:, 2], color=col, s=10, depthshade=False)

    if starts.size:
        ax.scatter(starts[:, 0], starts[:, 1], starts[:, 2], marker="^",
                   s=110, c="black", depthshade=False, label="start points")

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    arm = data.get("arm") or "?"
    ax.set_title(title or
                 f"{pickle_path.parent.name}  --  arm {arm}"
                 f"{'  (planned vs flown)' if flown else '  (planned)'}")
    ax.view_init(elev=elev, azim=azim)

    # TRUE equal aspect, via the box shape rather than by padding every axis to
    # the largest span. The flight volume is ~200 x 200 x 50 m, so forcing one
    # shared radius gave z a +/-100 m range: the paths were squashed into the
    # lower third and most of the frame was empty. Setting `box_aspect` to the
    # real data ranges keeps a metre equal in every direction AND uses the whole
    # figure, so a vertical detour is neither exaggerated nor flattened.
    stack = [p for p in paths if p.size]
    if obstacles.size:
        stack.append(obstacles)
    if starts.size:
        stack.append(starts)
    pts = np.vstack(stack)
    lo = pts.min(axis=0)
    hi = pts.max(axis=0)
    pad = np.maximum((hi - lo) * 0.05, 1.0)
    lo, hi = lo - pad, hi + pad
    lo[2] = max(0.0, lo[2])                      # the ground is a real boundary
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(tuple(np.maximum(hi - lo, 1e-6)))

    # A flat box_aspect leaves the drawn content well below the axes top, so a
    # legend pinned to "upper left" floats above it with a band of white in
    # between. legend_anchor lets the caller drop it onto the content.
    ax.legend(loc="upper left", fontsize=8, ncol=2, framealpha=0.85,
              bbox_to_anchor=legend_anchor)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_png, dpi=dpi)
    plt.close(fig)
    return out_png


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-dir", required=True,
                    help="campaign run dir (holds trials/ and gazebo/)")
    ap.add_argument("--trials", default="",
                    help="comma-separated trial_ids; default every trial that "
                         "has both a pickle and a bag")
    ap.add_argument("--out-dir", default=None,
                    help="default <run_dir>/media/figures")
    ap.add_argument("--path-key", default="quicknav_paths")
    ap.add_argument("--dpi", type=int, default=200)
    args = ap.parse_args()

    run_dir = Path(args.run_dir).resolve()
    out_dir = Path(args.out_dir).resolve() if args.out_dir else run_dir / "media" / "figures"

    wanted = [t.strip() for t in args.trials.split(",") if t.strip()]
    if not wanted:
        # Prefer trials that have a bag, so the figure shows planned AND flown.
        wanted = sorted(p.parent.name
                        for p in (run_dir / "gazebo").glob("*/poses.bag")
                        ) if (run_dir / "gazebo").exists() else []
        if not wanted:
            wanted = sorted(p.parent.name for p in
                            (run_dir / "trials").glob("*/deckga_quicknav_output.pkl"))

    n_ok = 0
    for tid in wanted:
        pkl = run_dir / "trials" / tid / "deckga_quicknav_output.pkl"
        if not pkl.exists():
            print(f"  [skip] {tid}: no pickle")
            continue
        bag = run_dir / "gazebo" / tid / "poses.bag"
        out = out_dir / f"{tid}_paths.png"
        try:
            render(pkl, bag if bag.exists() else None, out,
                   path_key=args.path_key, dpi=args.dpi)
        except Exception as e:                                # noqa: BLE001
            print(f"  [fail] {tid}: {type(e).__name__}: {e}")
            continue
        n_ok += 1
        print(f"  [ok] {out}  ({'planned+flown' if bag.exists() else 'planned only'})")
    print(f"rendered {n_ok} figure(s) into {out_dir}")
    return 0 if n_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
