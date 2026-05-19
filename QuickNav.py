#!/usr/bin/env python3
"""
QuickNav.py

Iterative obstacle detection + avoidance loop for 3D paths.
Ported from dipraj-debnath/DECKGA_QuickNav_3D (no changes to algorithm logic).

Main entry point: apply_quicknav()
"""

import numpy as np

from QuickNav_detection_3D import obstacle_detection
from QuickNav_avoidance_3D import obstacle_avoid


def apply_quicknav(route, obstacle_xyz, obs_size, max_iterations=10):
    """
    Iteratively apply QuickNav obstacle detection and avoidance to a 3D path.

    Parameters
    ----------
    route         : np.ndarray (N, 3) — original route
    obstacle_xyz  : np.ndarray (M, 3) — obstacle centres
    obs_size      : np.ndarray (M,) or scalar — obstacle half-sizes
    max_iterations: int — maximum avoidance iterations

    Returns
    -------
    route : np.ndarray — final obstacle-free route (or best attempt)
    """
    def deduplicate_preserve_order(arr, eps=1e-6):
        result = []
        for p in arr:
            if not any(np.allclose(p, r, atol=eps) for r in result):
                result.append(p)
        return np.array(result)

    iteration = 0

    while iteration < max_iterations:
        iteration += 1
        Obstacle_route, new_ob, obs_route = obstacle_detection(
            route, obstacle_xyz, obs_size, visualize=False
        )

        if len(new_ob) == 0:
            break

        # Remove duplicate (overlapping) obstacle detections
        qq = True
        while qq and len(new_ob) > 1:
            qq = False
            r = len(new_ob)
            j = 0
            while j < r - 1:
                gg = True
                for i in range(min(6, Obstacle_route.shape[1])):
                    if not np.isclose(Obstacle_route[j, i], Obstacle_route[j + 1, i], atol=1e-6):
                        gg = False
                        break
                if gg:
                    Obstacle_route = np.delete(Obstacle_route, j + 1, axis=0)
                    new_ob = np.delete(new_ob, j + 1, axis=0)
                    obs_route = np.delete(obs_route, j + 1, axis=0)
                    r -= 1
                    qq = True
                else:
                    j += 1

        # Match detected obstacles back to their sizes
        matched_sizes = []
        for detected_ob in new_ob:
            idx = np.where((obstacle_xyz == detected_ob).all(axis=1))[0]
            matched_sizes.append(obs_size[idx[0]] if len(idx) > 0 else 1.0)

        # Pass the FULL obstacle list so obstacle_avoid can filter detour
        # candidates against every obstacle, not just the one being detoured.
        try:
            Obstacle_avoid_route = obstacle_avoid(
                Obstacle_route, new_ob, obstacle_xyz, obs_size,
                current_sizes=np.array(matched_sizes),
            )
        except Exception as e:
            print(f"[QuickNav] obstacle_avoid raised {type(e).__name__}: {e}; aborting.")
            break

        # Bug-D fix part 1: sort by segment index so the splice loop, which
        # uses a monotonic `index += 2` correction, processes detours in
        # left-to-right order. obstacle_detection iterates obstacles-then-
        # segments, so without this sort an upstream detour can overwrite an
        # earlier-spliced one.
        sort_idx = np.argsort(obs_route[:, 0].astype(int))
        obs_route = obs_route[sort_idx]
        Obstacle_avoid_route = Obstacle_avoid_route[:, :, sort_idx]

        # Bug-D fix part 2: when multiple obstacles share a single segment, the
        # splice logic cannot install more than one detour per segment without
        # overwriting. Keep only the FIRST detection per unique segment and let
        # the next outer iteration handle the rest (the spliced detour breaks
        # the original segment into shorter ones, so further obstacles are
        # detected against those shorter pieces).
        _, first_per_seg = np.unique(obs_route[:, 0].astype(int), return_index=True)
        first_per_seg = np.sort(first_per_seg)
        obs_route = obs_route[first_per_seg]
        Obstacle_avoid_route = Obstacle_avoid_route[:, :, first_per_seg]

        # Splice avoidance detours into route
        Final = route.copy()
        r_obs = len(obs_route)
        index = 0

        for i in range(r_obs):
            rr_final = len(Final)
            start_idx = int(obs_route[i, 0]) - 1 + index
            end_idx = int(obs_route[i, 1]) - 1 + index
            if start_idx < 0 or end_idx >= rr_final:
                continue
            avoid_path = Obstacle_avoid_route[:, :, i]
            if obs_route[i, 0] == 1:
                FF = Final[end_idx + 1:, :]
                Final = np.vstack([avoid_path, FF])
                index += 2
            elif (end_idx + 1) >= rr_final:
                Final = np.vstack([Final[:start_idx], avoid_path])
                index += 2
            else:
                part1 = Final[:start_idx]
                part2 = avoid_path
                part3 = Final[end_idx + 1:]
                Final = np.vstack([part1, part2, part3])
                index += 2

        if np.array_equal(route, Final):
            break
        route = Final
    else:
        # while-loop exhausted without break: max_iterations hit without convergence.
        # Surface unresolved collisions so the caller sees the failure rather than
        # silently flying an unsafe path.
        Obstacle_route, new_ob, obs_route = obstacle_detection(
            route, obstacle_xyz, obs_size, visualize=False
        )
        if len(new_ob) > 0:
            print(
                f"[QuickNav] WARNING: max_iterations={max_iterations} reached with "
                f"{len(new_ob)} unresolved obstacle intersection(s). "
                f"Returned path is NOT collision-free."
            )

    route = deduplicate_preserve_order(route)
    return route
