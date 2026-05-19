"""Adaptive AABB obstacle extraction from a point cloud.

Algorithm
---------
1. Voxelize at r_voxel using canonical voxel centres
   (np.floor(p / r_voxel) -> integer indices -> centres at (i + 0.5) * r_voxel).
2. Merge by exact Chebyshev clustering with threshold r_voxel + 2*s_safety.
   Two voxels merge iff their inflated AABBs touch on every axis.
3. For each merged cluster, compute fill_ratio. If >= tau_fill, emit one
   AABB inflated per-axis by s_safety. Otherwise recursively bisect the
   cluster's voxel set along its longest axis at the median voxel-centre
   coordinate, up to max_split_depth.
4. Emit each leaf cluster as a centre (3,) + per-axis half-sizes (3,) AABB.

The output is always (centers (N,3), sizes (N,3)) where sizes are per-axis
half-sizes, ready for the DECK-GA + QuickNav dict-pkl writer.
"""

from __future__ import annotations

from typing import List, Tuple

import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components


def voxelize(points: np.ndarray, r_voxel: float) -> np.ndarray:
    """Return canonical voxel centres for occupied voxels of `points`.

    A voxel is occupied if at least one input point falls inside it.
    Centres are at (i + 0.5) * r_voxel for integer voxel index i.
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"points must be (M,3), got {pts.shape}")
    if r_voxel <= 0:
        raise ValueError(f"r_voxel must be > 0, got {r_voxel}")

    indices = np.floor(pts / r_voxel).astype(np.int64)
    unique_indices = np.unique(indices, axis=0)
    centres = (unique_indices.astype(float) + 0.5) * r_voxel
    return centres


def chebyshev_clusters(voxel_centres: np.ndarray, merge_threshold: float) -> np.ndarray:
    """Cluster voxel centres by Chebyshev distance <= merge_threshold.

    Uses scipy.cKDTree.query_pairs(p=inf) for exact Chebyshev neighbour
    queries, then scipy.sparse.csgraph.connected_components.

    Returns
    -------
    labels : (M,) int array — cluster id per voxel.
    """
    n = len(voxel_centres)
    if n == 0:
        return np.zeros(0, dtype=int)
    if n == 1:
        return np.zeros(1, dtype=int)

    tree = cKDTree(voxel_centres)
    pairs = tree.query_pairs(r=float(merge_threshold), p=np.inf, output_type="ndarray")

    # Build a symmetric sparse graph with self-loops so connected_components
    # works correctly for isolated voxels too.
    if len(pairs) == 0:
        rows = np.arange(n)
        cols = np.arange(n)
    else:
        rows = np.concatenate([pairs[:, 0], pairs[:, 1], np.arange(n)])
        cols = np.concatenate([pairs[:, 1], pairs[:, 0], np.arange(n)])

    data = np.ones(len(rows), dtype=np.int8)
    mat = csr_matrix((data, (rows, cols)), shape=(n, n))
    _, labels = connected_components(mat, directed=False)
    return labels.astype(int)


def _cluster_aabb_and_fill(
    cluster_voxels: np.ndarray, r_voxel: float
) -> Tuple[np.ndarray, np.ndarray, float]:
    """Return tight AABB (min, max corners) and fill ratio for a voxel set.

    AABB encloses the full voxel extents (centres ± r_voxel/2 per axis).
    Fill ratio = (n_voxels * r_voxel^3) / V_aabb.
    """
    mins = cluster_voxels.min(axis=0) - r_voxel / 2.0
    maxs = cluster_voxels.max(axis=0) + r_voxel / 2.0
    dims = maxs - mins
    v_aabb = float(np.prod(dims))
    v_occ = float(len(cluster_voxels) * (r_voxel ** 3))
    fill = v_occ / v_aabb if v_aabb > 0 else 1.0
    return mins, maxs, fill


def _emit_inflated(
    mins: np.ndarray, maxs: np.ndarray, s_safety: float
) -> Tuple[np.ndarray, np.ndarray]:
    """Convert tight (mins, maxs) AABB into inflated (centre, half_sizes)."""
    centre = 0.5 * (mins + maxs)
    half_sizes = 0.5 * (maxs - mins) + s_safety
    return centre, half_sizes


def _split_cluster_recursive(
    cluster_voxels: np.ndarray,
    r_voxel: float,
    s_safety: float,
    tau_fill: float,
    max_depth: int,
    depth: int = 0,
    min_voxels: int = 8,
) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Recursive median-axis split. Returns list of (centre, half_sizes) tuples."""
    mins, maxs, fill = _cluster_aabb_and_fill(cluster_voxels, r_voxel)

    if (
        fill >= tau_fill
        or depth >= max_depth
        or len(cluster_voxels) <= min_voxels
    ):
        return [_emit_inflated(mins, maxs, s_safety)]

    # Bisect along the longest axis at the median voxel coordinate.
    dims = maxs - mins
    axis = int(np.argmax(dims))
    median = float(np.median(cluster_voxels[:, axis]))

    left_mask = cluster_voxels[:, axis] <= median
    right_mask = ~left_mask

    # If the split is degenerate (all voxels on one side because of ties), bail.
    if left_mask.all() or right_mask.all():
        return [_emit_inflated(mins, maxs, s_safety)]

    left_aabbs = _split_cluster_recursive(
        cluster_voxels[left_mask], r_voxel, s_safety, tau_fill, max_depth, depth + 1, min_voxels
    )
    right_aabbs = _split_cluster_recursive(
        cluster_voxels[right_mask], r_voxel, s_safety, tau_fill, max_depth, depth + 1, min_voxels
    )
    return left_aabbs + right_aabbs


def extract_obstacles(
    points: np.ndarray,
    *,
    r_voxel: float,
    s_safety: float,
    tau_fill: float = 0.05,
    max_split_depth: int = 3,
    min_voxels_per_leaf: int = 8,
) -> Tuple[np.ndarray, np.ndarray]:
    """Full pipeline: voxelize -> cluster -> fill check -> recursive split.

    Returns
    -------
    centers : (N, 3) ndarray
    sizes   : (N, 3) ndarray  — per-axis half-sizes including s_safety inflation
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"points must be (M,3), got {pts.shape}")
    if len(pts) == 0:
        return np.zeros((0, 3)), np.zeros((0, 3))

    voxel_centres = voxelize(pts, r_voxel)
    if len(voxel_centres) == 0:
        return np.zeros((0, 3)), np.zeros((0, 3))

    merge_threshold = r_voxel + 2.0 * s_safety
    labels = chebyshev_clusters(voxel_centres, merge_threshold)

    centres: List[np.ndarray] = []
    sizes: List[np.ndarray] = []

    for cid in np.unique(labels):
        mask = labels == cid
        cluster_voxels = voxel_centres[mask]
        leaves = _split_cluster_recursive(
            cluster_voxels,
            r_voxel=r_voxel,
            s_safety=s_safety,
            tau_fill=tau_fill,
            max_depth=max_split_depth,
            depth=0,
            min_voxels=min_voxels_per_leaf,
        )
        for c, h in leaves:
            centres.append(c)
            sizes.append(h)

    if not centres:
        return np.zeros((0, 3)), np.zeros((0, 3))

    return np.vstack(centres), np.vstack(sizes)
