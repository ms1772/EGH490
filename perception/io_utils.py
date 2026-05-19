"""I/O for the perception pipeline.

Readers:
    load_cloud(path) -> (M, 3) float ndarray
        Dispatches by file extension. Supports .pcd .ply .xyz (Open3D),
        .las .laz (laspy), .npy .bin (numpy).

Writers:
    write_obstacles_dict(path, centers, sizes, meta)
        Writes the dict-pkl schema consumed by DECK_GA_QuickNav.py and
        deckga_ros2/rviz_obstacles_node.py.

Loaders for legacy:
    load_obstacles_any(path, fallback_size)
        Auto-detects dict vs legacy Nx3 array. Used by tests; the real
        consumer (DECK_GA_QuickNav.py) inlines its own copy of this logic
        to avoid importing perception.
"""

from __future__ import annotations

import pickle
from pathlib import Path
from typing import Any, Dict, Tuple

import numpy as np


_OPEN3D_EXTS = {".pcd", ".ply", ".xyz", ".xyzn", ".xyzrgb", ".pts"}
_LAS_EXTS = {".las", ".laz"}
_NUMPY_EXTS = {".npy"}
_BIN_EXTS = {".bin"}


def load_cloud(path: str | Path) -> np.ndarray:
    """Load a point cloud, return Nx3 float64 ndarray of XYZ.

    Extension dispatch:
        .pcd / .ply / .xyz / .xyzn / .xyzrgb / .pts  -> Open3D
        .las / .laz                                  -> laspy
        .npy                                         -> numpy.load
        .bin                                         -> raw float32 [x,y,z,...] dump
    """
    p = Path(path).expanduser()
    if not p.exists():
        raise FileNotFoundError(f"Point cloud not found: {p}")

    ext = p.suffix.lower()

    if ext in _OPEN3D_EXTS:
        import open3d as o3d  # local import — keeps perception/io_utils.py importable for legacy loader
        pcd = o3d.io.read_point_cloud(str(p))
        pts = np.asarray(pcd.points, dtype=float)
        if pts.size == 0:
            raise ValueError(f"Open3D loaded an empty cloud from {p}")
        return pts

    if ext in _LAS_EXTS:
        import laspy
        las = laspy.read(str(p))
        # laspy 2.x exposes .x .y .z as scaled float arrays
        pts = np.column_stack([np.asarray(las.x, dtype=float),
                               np.asarray(las.y, dtype=float),
                               np.asarray(las.z, dtype=float)])
        if pts.size == 0:
            raise ValueError(f"laspy loaded an empty cloud from {p}")
        return pts

    if ext in _NUMPY_EXTS:
        arr = np.load(str(p))
        arr = np.asarray(arr, dtype=float)
        if arr.ndim != 2 or arr.shape[1] < 3:
            raise ValueError(f".npy cloud must be Nx>=3. Got shape {arr.shape}")
        return arr[:, :3]

    if ext in _BIN_EXTS:
        # KITTI-style: float32 with [x,y,z,intensity,...]. We default to 4 cols.
        raw = np.fromfile(str(p), dtype=np.float32)
        # Try 4-col first (KITTI), then 3-col.
        for cols in (4, 3):
            if raw.size % cols == 0:
                arr = raw.reshape(-1, cols)
                return arr[:, :3].astype(float)
        raise ValueError(
            f".bin cloud size {raw.size} not divisible by 4 or 3 floats. "
            "Specify the layout upstream."
        )

    raise ValueError(
        f"Unsupported point-cloud extension: {ext!r}. "
        f"Supported: {sorted(_OPEN3D_EXTS | _LAS_EXTS | _NUMPY_EXTS | _BIN_EXTS)}"
    )


def write_obstacles_dict(
    path: str | Path,
    centers: np.ndarray,
    sizes: np.ndarray,
    meta: Dict[str, Any],
) -> None:
    """Write the dict-pkl schema consumed by DECK_GA_QuickNav.py."""
    p = Path(path).expanduser()
    p.parent.mkdir(parents=True, exist_ok=True)

    centers = np.asarray(centers, dtype=float)
    sizes = np.asarray(sizes, dtype=float)
    if centers.ndim != 2 or centers.shape[1] != 3:
        raise ValueError(f"centers must be (N,3). Got {centers.shape}")
    if sizes.ndim != 2 or sizes.shape[1] != 3 or sizes.shape[0] != centers.shape[0]:
        raise ValueError(
            f"sizes must be (N,3) matching centers (N,{centers.shape[0]}). Got {sizes.shape}"
        )

    obj = {"centers": centers, "sizes": sizes, "meta": dict(meta)}
    with p.open("wb") as f:
        pickle.dump(obj, f)


def write_obstacles_txt(path: str | Path, centers: np.ndarray, sizes: np.ndarray) -> None:
    """Sidecar 6-column CSV for human inspection: cx,cy,cz,sx,sy,sz."""
    p = Path(path).expanduser()
    p.parent.mkdir(parents=True, exist_ok=True)
    centers = np.asarray(centers, dtype=float)
    sizes = np.asarray(sizes, dtype=float)
    rows = np.hstack([centers, sizes])
    header = "cx,cy,cz,sx,sy,sz"
    np.savetxt(p, rows, delimiter=",", header=header, comments="", fmt="%.4f")


def load_obstacles_any(
    path: str | Path,
    fallback_size: float = 5.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """Auto-detect dict-pkl vs legacy Nx3 obstacle file.

    Returns (centers (N,3), sizes (N,3) always per-axis half-sizes).
    Legacy Nx3 → sizes filled with fallback_size on every axis.
    """
    p = Path(path).expanduser()
    with p.open("rb") as f:
        obj = pickle.load(f)

    if isinstance(obj, dict) and "centers" in obj and "sizes" in obj:
        centers = np.asarray(obj["centers"], dtype=float)
        sizes = np.asarray(obj["sizes"], dtype=float)
        if sizes.ndim == 1:
            sizes = np.tile(sizes[:, None], (1, 3))
        return centers, sizes

    arr = np.asarray(obj, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f"Legacy obstacles must be Nx3. Got {arr.shape} from {p}")
    return arr, np.full((len(arr), 3), float(fallback_size))
