"""Synthetic octomap .bt writer for test fixtures.

Mirrors the parser convention in perception/octomap_to_obstacles.py:
- Root at world origin, OCTOMAP_MAX_DEPTH = 16.
- Child index i in [0,8) -> sign bits (i&1, (i>>1)&1, (i>>2)&1), 0 -> -, 1 -> +.
- Inner node = 2 bytes, 8 x 2-bit child codes (00 empty, 01 free, 10 occ, 11 inner).

Self-test (round-trip parser) is wired through
perception/tests/test_octomap_to_obstacles.py.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np

from perception.octomap_to_obstacles import OCTOMAP_MAX_DEPTH, _CHILD_SIGNS


def _xyz_to_leaf_index(p: np.ndarray, resolution: float) -> Tuple[int, int, int]:
    """World coord -> integer leaf-grid index (centred at origin)."""
    ix = int(np.floor(p[0] / resolution))
    iy = int(np.floor(p[1] / resolution))
    iz = int(np.floor(p[2] / resolution))
    return ix, iy, iz


def _box_to_leaf_indices(centre: np.ndarray, half_size: float, resolution: float):
    """Yield integer leaf indices inside a cube centred at `centre` with given half-size."""
    n_per_side = max(1, int(round(2 * half_size / resolution)))
    half_n = n_per_side / 2.0
    for ix in range(-int(np.floor(half_n)), int(np.ceil(half_n))):
        for iy in range(-int(np.floor(half_n)), int(np.ceil(half_n))):
            for iz in range(-int(np.floor(half_n)), int(np.ceil(half_n))):
                wx = centre[0] / resolution + ix
                wy = centre[1] / resolution + iy
                wz = centre[2] / resolution + iz
                yield int(np.floor(wx)), int(np.floor(wy)), int(np.floor(wz))


def _child_index_for(leaf_key: Tuple[int, int, int],
                     parent_origin_key: Tuple[int, int, int],
                     child_half_n: int) -> int:
    """For a leaf grid key, decide which child slot it lives in for a parent
    whose own children cover `child_half_n` leaves per axis."""
    # parent_origin_key is the (negative) corner of the parent in leaf units.
    # Children split the parent in half along each axis.
    bx = (leaf_key[0] - parent_origin_key[0]) >= child_half_n
    by = (leaf_key[1] - parent_origin_key[1]) >= child_half_n
    bz = (leaf_key[2] - parent_origin_key[2]) >= child_half_n
    return (1 if bx else 0) | (2 if by else 0) | (4 if bz else 0)


def write_synth_bt(
    path: Path,
    boxes: Iterable[Tuple[Tuple[float, float, float], float]],
    resolution: float = 0.5,
) -> Dict:
    """Write a .bt file with `boxes` = list of (centre_xyz, half_size).

    Returns metadata dict with leaf_centres (Nx3 float), resolution.
    """
    # 1) Enumerate target leaf-grid indices for every box.
    leaf_keys: set = set()
    for centre, half in boxes:
        c = np.asarray(centre, dtype=float)
        for k in _box_to_leaf_indices(c, half, resolution):
            leaf_keys.add(k)

    leaf_keys_sorted = sorted(leaf_keys)
    if not leaf_keys_sorted:
        leaf_centres = np.zeros((0, 3))
    else:
        leaf_centres = np.array(
            [[(kx + 0.5) * resolution, (ky + 0.5) * resolution, (kz + 0.5) * resolution]
             for (kx, ky, kz) in leaf_keys_sorted],
            dtype=float,
        )

    # 2) Build the tree bottom-up by recursive partitioning of the leaf-key set.
    #    Inner node serialisation uses iterative DFS so we can write each parent's
    #    2-byte word before its children. We'll do recursion (depth=16) which is fine.
    payload = bytearray()
    root_half_n = 2 ** (OCTOMAP_MAX_DEPTH - 1)  # children of root span half_n^3 leaves each

    def _serialise(keys: List[Tuple[int, int, int]],
                   parent_origin_key: Tuple[int, int, int],
                   parent_child_half_n: int):
        # parent_origin_key is the (-,-,-) corner of THIS node in leaf units;
        # parent_child_half_n is half the side length of THIS node in leaf units.
        word = 0
        # Bucket keys into 8 children.
        buckets: List[List[Tuple[int, int, int]]] = [[] for _ in range(8)]
        for k in keys:
            ci = _child_index_for(k, parent_origin_key, parent_child_half_n)
            buckets[ci].append(k)

        # Decide child code per slot. We write the parent word first, then iterate
        # in 0..7 order writing recursed inner nodes immediately after.
        child_codes = [0] * 8
        for i in range(8):
            if not buckets[i]:
                child_codes[i] = 0b00
                continue
            # If the child cell is a single leaf voxel (half_n == 1) AND it contains
            # exactly that leaf, mark as occupied leaf. Otherwise recurse.
            child_half_n = parent_child_half_n // 2
            if child_half_n == 0:
                # Parent is itself a single voxel -> bucket has exactly the parent key
                child_codes[i] = 0b10
                continue
            # Compute this child's origin in leaf units.
            sx, sy, sz = _CHILD_SIGNS[i]
            child_origin = (
                parent_origin_key[0] + (parent_child_half_n if sx > 0 else 0),
                parent_origin_key[1] + (parent_child_half_n if sy > 0 else 0),
                parent_origin_key[2] + (parent_child_half_n if sz > 0 else 0),
            )
            # Only emit 0b10 at child_half_n == 0 (single leaf voxel). At higher
            # levels a 0b10 would be a lie unless the entire child subtree were
            # fully occupied; we keep the writer simple by always recursing.
            child_codes[i] = 0b11

        for i, c in enumerate(child_codes):
            word |= (c & 0b11) << (2 * i)
        payload.append(word & 0xFF)
        payload.append((word >> 8) & 0xFF)

        # Recurse for 11 children.
        for i in range(8):
            if child_codes[i] != 0b11:
                continue
            child_half_n = parent_child_half_n // 2
            sx, sy, sz = _CHILD_SIGNS[i]
            child_origin = (
                parent_origin_key[0] + (parent_child_half_n if sx > 0 else 0),
                parent_origin_key[1] + (parent_child_half_n if sy > 0 else 0),
                parent_origin_key[2] + (parent_child_half_n if sz > 0 else 0),
            )
            _serialise(buckets[i], child_origin, child_half_n)

    # Root origin is the (-,-,-) corner of the root cube in leaf units.
    # Root half-extent in metres = root_half_n * resolution. Root spans 2*root_half_n leaves.
    root_origin = (-root_half_n, -root_half_n, -root_half_n)
    _serialise(leaf_keys_sorted, root_origin, root_half_n)

    # 3) Write file with header.
    p = Path(path).expanduser()
    p.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# Octomap OcTree binary file\n"
        "id OcTree\n"
        f"size {len(leaf_keys_sorted)}\n"
        f"res {resolution}\n"
        "data\n"
    ).encode("ascii")
    with p.open("wb") as f:
        f.write(header)
        f.write(payload)

    return {
        "path": str(p),
        "resolution": resolution,
        "leaf_centres": leaf_centres,
        "n_leaves": len(leaf_keys_sorted),
        "boxes": list(boxes),
    }


# Default fixture: three 1m cubes
DEFAULT_BOXES = [
    ((20.0, 10.0, 5.0), 0.5),
    ((50.0, 30.0, 5.0), 0.5),
    ((80.0, 15.0, 5.0), 0.5),
]
DEFAULT_RESOLUTION = 0.5
DEFAULT_FIXTURE_PATH = Path(__file__).parent / "synth_three_boxes.bt"


def ensure_default_fixture() -> Dict:
    """Write the default 3-box fixture if missing; return its metadata."""
    return write_synth_bt(DEFAULT_FIXTURE_PATH, DEFAULT_BOXES, DEFAULT_RESOLUTION)
