#!/usr/bin/env python3
"""
QuickNav_detection_3D.py

Detects which segments of a 3D route intersect axis-aligned obstacle cubes.
Ported from dipraj-debnath/DECKGA_QuickNav_3D (no changes to algorithm logic).
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def obstacle_detection(route, obstacle_xyz, obs_size, visualize=True, eps=1e-6):
    """
    Detect route segments that intersect obstacle cubes.

    Parameters
    ----------
    route        : np.ndarray, shape (N, 3)
    obstacle_xyz : np.ndarray, shape (M, 3)  — obstacle centres
    obs_size     : scalar or np.ndarray (M,) — half-size of each obstacle cube
    visualize    : bool — show matplotlib plot
    eps          : float — tolerance for endpoint equality

    Returns
    -------
    Obstacle_route : np.ndarray (K, 6)  — [Ax Ay Az Bx By Bz] per intersecting segment
    new_ob         : np.ndarray (K, 3)  — obstacle centres that caused intersections
    obs_route      : np.ndarray (K, 2)  — 1-based [start_idx, end_idx] of each segment
    """
    def approx_equal(a, b):
        return np.all(np.abs(a - b) < eps)

    if isinstance(obs_size, (int, float)):
        obs_size = np.full(len(obstacle_xyz), obs_size)

    Obstacle_route = []
    new_ob = []
    obs_route = []

    for i in range(len(obstacle_xyz)):
        center = obstacle_xyz[i]
        size = obs_size[i]
        pmin = center - size
        pmax = center + size

        for j in range(len(route) - 1):
            A = route[j]
            B = route[j + 1]
            if line_intersects_cube(A, B, pmin, pmax):
                Obstacle_route.append(np.concatenate((A, B)))
                new_ob.append(center)
                obs_route.append([j + 1, j + 2])

    Obstacle_route = np.array(Obstacle_route) if Obstacle_route else np.zeros((0, 6))
    new_ob = np.array(new_ob) if new_ob else np.zeros((0, 3))
    obs_route = np.array(obs_route) if obs_route else np.zeros((0, 2))

    if visualize:
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(route[:, 0], route[:, 1], route[:, 2], "-o", linewidth=2, markersize=8, label="Route")
        ax.scatter(obstacle_xyz[:, 0], obstacle_xyz[:, 1], obstacle_xyz[:, 2],
                   c="r", marker="x", s=100, label="Obstacles")
        for i in range(len(obstacle_xyz)):
            draw_cube(ax, obstacle_xyz[i], obs_size[i])
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Route with Obstacles")
        ax.legend()
        plt.grid(True)
        plt.show()

    return Obstacle_route, new_ob, obs_route


def line_intersects_cube(p1, p2, cube_min, cube_max, eps=1e-4):
    """
    Slab-based AABB ray intersection test.

    Returns True if the segment p1→p2 penetrates the INTERIOR of the axis-aligned
    box [cube_min, cube_max]. The box is shrunk by `eps` on every axis so that
    segments touching a face, edge, or corner (e.g. QuickNav detour corners that
    sit on an obstacle's surface by construction) are NOT counted as intersections.
    """
    cube_min = np.asarray(cube_min, dtype=float) + eps
    cube_max = np.asarray(cube_max, dtype=float) - eps
    if np.any(cube_min >= cube_max):
        return False  # obstacle smaller than 2*eps on some axis -> no interior

    d = p2 - p1
    tmin, tmax = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-6:
            if p1[i] < cube_min[i] or p1[i] > cube_max[i]:
                return False
        else:
            ood = 1.0 / d[i]
            t1 = (cube_min[i] - p1[i]) * ood
            t2 = (cube_max[i] - p1[i]) * ood
            tmin_temp = min(t1, t2)
            tmax_temp = max(t1, t2)
            tmin = max(tmin, tmin_temp)
            tmax = min(tmax, tmax_temp)
            if tmin > tmax:
                return False
    return True


def draw_cube(ax, center, size):
    """Draw a wireframe box on a matplotlib 3D axes.

    `size` may be a scalar (cube) or a 3-vector (per-axis half-sizes).
    """
    if np.isscalar(size):
        sx = sy = sz = float(size)
    else:
        sx, sy, sz = (float(v) for v in size)

    verts = np.array([
        [center[0] + x * sx, center[1] + y * sy, center[2] + z * sz]
        for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)
    ])
    faces = [
        [verts[0], verts[1], verts[3], verts[2]],
        [verts[4], verts[5], verts[7], verts[6]],
        [verts[0], verts[1], verts[5], verts[4]],
        [verts[2], verts[3], verts[7], verts[6]],
        [verts[1], verts[3], verts[7], verts[5]],
        [verts[0], verts[2], verts[6], verts[4]],
    ]
    ax.add_collection3d(
        Poly3DCollection(faces, facecolors="cyan", linewidths=1, edgecolors="g", alpha=0.3)
    )
