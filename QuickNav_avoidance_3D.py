#!/usr/bin/env python3
"""
QuickNav_avoidance_3D.py

Given intersecting segments and obstacle cubes, computes shortest detour paths
by evaluating 8 corner-pair combinations across all 6 faces of each cube.
Ported from dipraj-debnath/DECKGA_QuickNav_3D (no changes to algorithm logic).
"""

import numpy as np

from QuickNav_detection_3D import line_intersects_cube


def obstacle_avoid(Obstacle_route, new_ob, obstacle_xyz, obs_size):
    """
    Compute avoidance detour paths for each detected obstacle intersection.

    Parameters
    ----------
    Obstacle_route : np.ndarray (K, 6)  — [Ax Ay Az Bx By Bz] per intersecting segment
    new_ob         : np.ndarray (K, 3)  — detected obstacle centres
    obstacle_xyz   : np.ndarray (M, 3)  — (unused here; kept for API symmetry)
    obs_size       : np.ndarray (K,) or scalar — half-size per obstacle

    Returns
    -------
    Obstacle_avoid_route : np.ndarray (4, 3, K)
        For each obstacle k, a 4-point path [start, corner1, corner2, end].
    """
    num_obs = len(new_ob)
    Obstacle_avoid_route = np.zeros((4, 3, num_obs))

    for i in range(num_obs):
        x0, y0, z0 = Obstacle_route[i, 0:3]
        x1, y1, z1 = Obstacle_route[i, 3:6]
        cx, cy, cz = new_ob[i]
        s = obs_size[i]

        if np.isscalar(s):
            sx = sy = sz = s
        elif isinstance(s, (np.ndarray, list, tuple)) and len(s) == 3:
            sx, sy, sz = s
        else:
            sx = sy = sz = s

        # 8 cube corners
        corners = np.array([
            [cx - sx, cy - sy, cz - sz],  # 0
            [cx + sx, cy - sy, cz - sz],  # 1
            [cx + sx, cy + sy, cz - sz],  # 2
            [cx - sx, cy + sy, cz - sz],  # 3
            [cx - sx, cy - sy, cz + sz],  # 4
            [cx + sx, cy - sy, cz + sz],  # 5
            [cx + sx, cy + sy, cz + sz],  # 6
            [cx - sx, cy + sy, cz + sz],  # 7
        ])

        # Faces defined by corner indices (each face: 4 corners)
        faces = {
            "bottom": [0, 1, 2, 3],
            "top":    [4, 5, 6, 7],
            "front":  [0, 1, 5, 4],
            "back":   [2, 3, 7, 6],
            "left":   [0, 3, 7, 4],
            "right":  [1, 2, 6, 5],
        }

        all_paths = []
        for face_indices in faces.values():
            c = [corners[idx] for idx in face_indices]
            # 8 corner-pair combinations per face
            all_paths.extend([
                [[x0, y0, z0], c[0], c[1], [x1, y1, z1]],
                [[x0, y0, z0], c[1], c[0], [x1, y1, z1]],
                [[x0, y0, z0], c[2], c[3], [x1, y1, z1]],
                [[x0, y0, z0], c[3], c[2], [x1, y1, z1]],
                [[x0, y0, z0], c[0], c[3], [x1, y1, z1]],
                [[x0, y0, z0], c[3], c[0], [x1, y1, z1]],
                [[x0, y0, z0], c[1], c[2], [x1, y1, z1]],
                [[x0, y0, z0], c[2], c[1], [x1, y1, z1]],
            ])

        # Filter out candidates where start→c1 or c2→end clips through this obstacle.
        # These would land the drone inside or back through the box on approach/exit.
        pmin = np.array([cx - sx, cy - sy, cz - sz])
        pmax = np.array([cx + sx, cy + sy, cz + sz])
        start = np.array([x0, y0, z0])
        end = np.array([x1, y1, z1])

        valid_paths = [
            p for p in all_paths
            if not line_intersects_cube(start, np.array(p[1]), pmin, pmax)
            and not line_intersects_cube(np.array(p[2]), end, pmin, pmax)
        ]
        # Fall back to all candidates if every approach/exit happens to clip the box
        if not valid_paths:
            valid_paths = all_paths

        # Select shortest 4-point detour from valid candidates
        best_path = None
        best_dist = np.inf
        for path in valid_paths:
            dist = sum(
                np.linalg.norm(np.array(path[j + 1]) - np.array(path[j]))
                for j in range(3)
            )
            if dist < best_dist:
                best_dist = dist
                best_path = path

        Obstacle_avoid_route[:, :, i] = best_path

    return Obstacle_avoid_route
