"""Ground / floor removal for the perception pipeline.

Three methods, selected by the caller:
    ransac       : Open3D segment_plane. Default. Seeded for determinism.
    z_threshold  : drop points with z <= ground_z.
    none         : passthrough.

All methods return (filtered_xyz (M,3), info_dict). info_dict contains
diagnostics: which method ran, how many points were removed, the RANSAC
plane model when applicable.
"""

from __future__ import annotations

from typing import Any, Dict, Tuple

import numpy as np


def remove_ground(
    points: np.ndarray,
    method: str = "ransac",
    *,
    ransac_distance: float = 0.2,
    ground_z: float | None = None,
    seed: int = 42,
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """Remove the ground plane from a point cloud.

    Parameters
    ----------
    points : (M, 3) ndarray
    method : 'ransac' | 'z_threshold' | 'none'
    ransac_distance : RANSAC inlier threshold in metres
    ground_z : float | None — required for z_threshold; if None for ransac, auto-pick.
    seed : int — RANSAC determinism.

    Returns
    -------
    filtered : (K, 3) ndarray (K <= M)
    info     : dict with keys 'method', 'n_in', 'n_out', and method-specific extras.
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"points must be (M,3), got {pts.shape}")

    n_in = len(pts)
    method = method.lower()

    if method == "none":
        return pts.copy(), {"method": "none", "n_in": n_in, "n_out": n_in}

    if method == "z_threshold":
        if ground_z is None:
            # Auto pick: 5th percentile of z, plus a small skim margin.
            ground_z = float(np.percentile(pts[:, 2], 5)) + 0.05
        mask = pts[:, 2] > float(ground_z)
        out = pts[mask]
        return out, {
            "method": "z_threshold",
            "n_in": n_in,
            "n_out": int(len(out)),
            "ground_z": float(ground_z),
        }

    if method == "ransac":
        import open3d as o3d

        # Seed Open3D's RNG for determinism.
        o3d.utility.random.seed(int(seed))

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=float(ransac_distance),
            ransac_n=3,
            num_iterations=1000,
        )

        inlier_mask = np.zeros(n_in, dtype=bool)
        inlier_mask[np.asarray(inliers, dtype=int)] = True
        out = pts[~inlier_mask]

        a, b, c, d = (float(x) for x in plane_model)
        # Plane normal vertical-ness: |c| close to 1 means horizontal floor.
        normal_z = float(abs(c))

        info: Dict[str, Any] = {
            "method": "ransac",
            "n_in": n_in,
            "n_out": int(len(out)),
            "plane_model": [a, b, c, d],
            "normal_z": normal_z,
        }
        if normal_z < 0.7:
            # Plane normal more than ~45° off vertical. Likely a wall or non-floor surface.
            info["warning"] = (
                f"RANSAC found a plane with normal_z={normal_z:.2f} (cos angle from vertical). "
                "Likely not the ground. Consider --ground-method z_threshold."
            )
        return out, info

    raise ValueError(f"Unknown ground method: {method!r}. Use ransac | z_threshold | none.")
