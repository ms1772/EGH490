"""OctoMap .bt -> obstacle AABBs.

Loads an OctoMap binary file (.bt), walks the tree to collect occupied leaf
voxel centres, then reuses perception/adaptive_aabb.extract_obstacles to
produce the same (centers, sizes) dict expected by DECK_GA_QuickNav.py.

Format reference: https://octomap.github.io/octomap/doc/binary_format.html

Header is ASCII, terminated by `data\\n`:
    # Octomap OcTree binary file
    id OcTree
    size <num_nodes>
    res <leaf_resolution>
    data

Then a binary tree. Each inner node is exactly 2 bytes = 16 bits = 8 children
* 2 bits. The 2-bit codes (per child, in Morton order 0..7):
    0b00 = unknown / no child
    0b01 = free leaf
    0b10 = occupied leaf
    0b11 = inner node (recurse, additional 2 bytes follow for that child)

Child index i in [0,8) maps to (x,y,z) sign bits (i&1, (i>>1)&1, (i>>2)&1),
where 0 -> negative offset, 1 -> positive offset of `half_extent/2` from
parent centre. This is the convention used by OctoMap's OcTreeKey arithmetic
and is mirrored exactly by the synth fixture writer so the parser round-trips.

The tree root is assumed centred at the world origin with half-extent
`(2**root_depth - 1) * resolution / 2` for root_depth = 16 (OctoMap's
hard-coded maximum). The synth fixture also uses depth=16 so it is wire-format
compatible with real .bt files in the limit (most nodes are empty children
encoded as `00` in the parent word, costing 2 bits).

If an `occupied leaf` marker appears at a non-leaf depth, the entire subtree
underneath is treated as uniformly occupied and "exploded" into its constituent
leaf voxels. We cap explode-per-marker at 64**3 voxels to avoid runaway memory;
callers hitting the cap should downsample their .bt upstream.

.ot files (full serialisation with log-odds) are NOT supported by this loader;
they require the octomap C++ library bindings. Raises NotImplementedError.
"""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Tuple

import numpy as np

from perception.adaptive_aabb import extract_obstacles

OCTOMAP_MAX_DEPTH = 16
MAX_EXPLODE_PER_SIDE = 64  # voxels per axis when expanding a non-leaf occupied marker
SAFETY_CAP_LEAVES = 5_000_000  # hard ceiling on parsed leaves; aborts cleanly


# Child sign vectors: index i in [0,8) -> (sx, sy, sz) in {-1, +1}.
# Bit 0 -> x, bit 1 -> y, bit 2 -> z. 0 -> -1, 1 -> +1.
_CHILD_SIGNS = np.array(
    [
        [-1 if not (i & 1) else +1,
         -1 if not (i & 2) else +1,
         -1 if not (i & 4) else +1]
        for i in range(8)
    ],
    dtype=np.int8,
)


def _read_header(buf: bytes) -> Tuple[float, int]:
    """Parse the ASCII header. Returns (resolution, data_section_start)."""
    sep = b"data\n"
    idx = buf.find(sep)
    if idx < 0:
        raise ValueError("octomap .bt header missing 'data\\n' separator")
    header = buf[:idx].decode("ascii", errors="replace")

    if ".ot" in header.lower() and ".bt" not in header.lower() and "binary" not in header.lower():
        raise NotImplementedError(
            "octomap .ot files require the octomap-python library; "
            "this loader only supports .bt (binary)."
        )

    resolution = None
    for line in header.splitlines():
        line = line.strip()
        if line.startswith("res"):
            parts = line.split()
            if len(parts) >= 2:
                try:
                    resolution = float(parts[1])
                except ValueError:
                    raise ValueError(f"octomap .bt header has un-parseable resolution: {line!r}")
    if resolution is None or resolution <= 0:
        raise ValueError(
            f"octomap .bt header missing or invalid `res` line. Header was:\n{header!r}"
        )
    return resolution, idx + len(sep)


def _explode_to_leaves(centre: np.ndarray, half_extent: float, resolution: float) -> np.ndarray:
    """Expand a uniformly-occupied cube into its constituent leaf-resolution centres."""
    n_per_side = max(1, int(round(2 * half_extent / resolution)))
    if n_per_side > MAX_EXPLODE_PER_SIDE:
        raise ValueError(
            f"octomap occupied marker covers {n_per_side}^3 leaf voxels "
            f"(half_extent={half_extent}, res={resolution}); refusing to explode "
            f"(cap {MAX_EXPLODE_PER_SIDE}^3). Downsample the .bt upstream."
        )
    if n_per_side == 1:
        return centre.reshape(1, 3)
    offsets = (np.arange(n_per_side) - (n_per_side - 1) / 2.0) * resolution
    gx, gy, gz = np.meshgrid(offsets, offsets, offsets, indexing="ij")
    grid = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
    return centre + grid


def load_octree_centres(path: Path) -> Tuple[np.ndarray, float]:
    """Parse a .bt file and return (occupied_leaf_centres (N,3) float64, resolution).

    Tree root is assumed at world origin, depth OCTOMAP_MAX_DEPTH.
    """
    p = Path(path).expanduser()
    if not p.exists():
        raise FileNotFoundError(f"octomap file not found: {p}")
    if p.suffix.lower() == ".ot":
        raise NotImplementedError(
            "octomap .ot files require octomap-python; this loader supports .bt only."
        )
    buf = p.read_bytes()
    resolution, data_off = _read_header(buf)

    root_half = (2 ** OCTOMAP_MAX_DEPTH) * resolution / 2.0
    centres = []
    # Iterative DFS. Stack entries: (centre_xyz: np.ndarray (3,), half_extent: float, depth: int)
    # We immediately read the node's 2 bytes when popped.
    pos = [data_off]  # mutable cursor

    def _read_node_bytes() -> int:
        if pos[0] + 2 > len(buf):
            raise ValueError(
                f"octomap .bt truncated at byte {pos[0]} (need 2 more bytes for inner node)"
            )
        b0 = buf[pos[0]]
        b1 = buf[pos[0] + 1]
        pos[0] += 2
        # Little-endian assembly: bits 0..7 are children 0..3, bits 8..15 are children 4..7,
        # each child uses 2 bits. OctoMap writes children 0..7 sequentially packed.
        return b0 | (b1 << 8)

    # Recursive DFS — must follow children in 0..7 order to match writer's
    # serialisation order. Using true recursion keeps the byte cursor in sync.
    def _walk(c: np.ndarray, h: float, d: int):
        word = _read_node_bytes()
        child_half = h / 2.0
        for i in range(8):
            code = (word >> (2 * i)) & 0b11
            if code == 0b00:
                continue
            child_centre = c + _CHILD_SIGNS[i].astype(np.float64) * child_half
            if code == 0b01:
                continue  # free leaf
            if code == 0b10:
                leaves = _explode_to_leaves(child_centre, child_half, resolution)
                centres.append(leaves)
                if sum(len(x) for x in centres) > SAFETY_CAP_LEAVES:
                    raise ValueError(
                        f"octomap parse exceeded safety cap of {SAFETY_CAP_LEAVES} leaves"
                    )
                continue
            # 0b11 — inner node, recurse to read its bytes next.
            if d + 1 > OCTOMAP_MAX_DEPTH:
                raise ValueError(
                    f"octomap recursion exceeded max depth {OCTOMAP_MAX_DEPTH} at byte {pos[0]}"
                )
            _walk(child_centre, child_half, d + 1)

    import sys as _sys
    _prev = _sys.getrecursionlimit()
    _sys.setrecursionlimit(max(_prev, OCTOMAP_MAX_DEPTH * 8 + 100))
    try:
        _walk(np.zeros(3, dtype=np.float64), root_half, 0)
    finally:
        _sys.setrecursionlimit(_prev)

    if pos[0] != len(buf):
        # Not strictly fatal — some writers append trailing newline. Warn via shape only.
        pass

    if not centres:
        return np.zeros((0, 3), dtype=np.float64), resolution
    return np.vstack(centres).astype(np.float64), resolution


def octomap_to_aabbs(
    path: Path,
    *,
    s_safety: float = 0.5,
    tau_fill: float = 0.05,
    max_split_depth: int = 3,
) -> Tuple[np.ndarray, np.ndarray, dict]:
    """Full pipeline: parse .bt -> adaptive AABB extraction.

    Returns (centers (N,3), sizes (N,3), meta).
    sizes are per-axis half-sizes including s_safety inflation.
    """
    centres, resolution = load_octree_centres(path)
    if len(centres) == 0:
        meta = {
            "source": "octomap",
            "input_file": str(path),
            "resolution": resolution,
            "params": {"s_safety": s_safety, "tau_fill": tau_fill, "max_split_depth": max_split_depth},
            "n_occupied_leaves": 0,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        return np.zeros((0, 3)), np.zeros((0, 3)), meta

    aabb_centres, aabb_sizes = extract_obstacles(
        centres,
        r_voxel=resolution,
        s_safety=s_safety,
        tau_fill=tau_fill,
        max_split_depth=max_split_depth,
    )
    meta = {
        "source": "octomap",
        "input_file": str(path),
        "resolution": resolution,
        "params": {"s_safety": s_safety, "tau_fill": tau_fill, "max_split_depth": max_split_depth},
        "n_occupied_leaves": int(len(centres)),
        "n_aabbs": int(len(aabb_centres)),
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }
    return aabb_centres, aabb_sizes, meta
