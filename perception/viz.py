"""Visualization for the perception pipeline.

Two outputs:
    save_matplotlib_snapshot(...)   : always written. Uses Agg backend so this
                                       is safe on headless WSL / CI.
    show_open3d_viewer(...)         : opt-in via --viz. Requires display
                                       (WSLg or X server).
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable

import numpy as np

# Headless-safe matplotlib backend.
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Line3DCollection  # noqa: E402


def _aabb_edges(centre: np.ndarray, half: np.ndarray) -> np.ndarray:
    """Return 12 edges of an AABB as (12, 2, 3) array (start, end, xyz)."""
    cx, cy, cz = centre
    sx, sy, sz = half
    corners = np.array([
        [cx - sx, cy - sy, cz - sz],
        [cx + sx, cy - sy, cz - sz],
        [cx + sx, cy + sy, cz - sz],
        [cx - sx, cy + sy, cz - sz],
        [cx - sx, cy - sy, cz + sz],
        [cx + sx, cy - sy, cz + sz],
        [cx + sx, cy + sy, cz + sz],
        [cx - sx, cy + sy, cz + sz],
    ])
    edge_idx = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    return np.array([[corners[a], corners[b]] for a, b in edge_idx])


def save_matplotlib_snapshot(
    points: np.ndarray,
    centres: np.ndarray,
    sizes: np.ndarray,
    out_path: str | Path,
    *,
    title: str = "Adaptive AABB extraction",
    max_points: int = 50000,
) -> None:
    """Save a 3D PNG of (subsampled) points + AABB wireframes."""
    out_path = Path(out_path).expanduser()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    pts = np.asarray(points, dtype=float)
    if len(pts) > max_points:
        rng = np.random.default_rng(0)
        idx = rng.choice(len(pts), size=max_points, replace=False)
        pts = pts[idx]

    fig = plt.figure(figsize=(8, 6), dpi=150)
    ax = fig.add_subplot(111, projection="3d")
    if len(pts) > 0:
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=pts[:, 2], cmap="viridis", s=0.5, alpha=0.4)

    if len(centres) > 0:
        all_edges = []
        for c, h in zip(centres, sizes):
            all_edges.extend(_aabb_edges(c, h).tolist())
        lc = Line3DCollection(all_edges, colors="red", linewidths=1.0)
        ax.add_collection3d(lc)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"{title} — {len(centres)} obstacles")
    fig.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)


def show_open3d_viewer(
    points: np.ndarray,
    centres: np.ndarray,
    sizes: np.ndarray,
    *,
    window_name: str = "Adaptive AABB extraction",
) -> None:
    """Interactive Open3D viewer. Requires display.

    Falls back gracefully (prints a message, no exception) if Open3D can't
    open a window (e.g. truly headless WSL without WSLg / X server).
    """
    try:
        import open3d as o3d
    except ImportError:
        print("[viz] Open3D not available — skipping interactive viewer.")
        return

    geometries = []

    pcd = o3d.geometry.PointCloud()
    if len(points) > 0:
        pcd.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
        geometries.append(pcd)

    for c, h in zip(centres, sizes):
        cx, cy, cz = (float(v) for v in c)
        sx, sy, sz = (float(v) for v in h)
        aabb = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array([cx - sx, cy - sy, cz - sz]),
            max_bound=np.array([cx + sx, cy + sy, cz + sz]),
        )
        aabb.color = (1.0, 0.0, 0.0)
        geometries.append(aabb)

    if not geometries:
        print("[viz] Nothing to show.")
        return

    # Some headless setups raise when draw_geometries can't open a window;
    # we treat that as informational, not a pipeline failure.
    if os.environ.get("PERCEPTION_VIZ_SAFE") == "1":
        try:
            o3d.visualization.draw_geometries(geometries, window_name=window_name)
        except Exception as e:
            print(f"[viz] Open3D viewer unavailable ({type(e).__name__}: {e}).")
    else:
        o3d.visualization.draw_geometries(geometries, window_name=window_name)
