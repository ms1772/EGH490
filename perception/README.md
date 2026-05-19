# perception — LiDAR → Obstacle AABB pipeline

Offline, sensor-agnostic, file-driven obstacle extraction for the DECK-GA + QuickNav planner.

The pipeline takes any point-cloud file and produces a dict pickle that drops straight into `DECK_GA_QuickNav.py --obstacles_pkl`. Switching between hand-authored obstacles and lidar-extracted obstacles is a single CLI argument.

## Quickstart

```bash
# 1. Install (first run downloads ~200 MB Open3D wheel)
pip install -r perception/requirements.txt

# 2. Extract obstacles from a point cloud
python perception/lidar_to_obstacles.py \
    --input  path/to/survey.las \
    --output data/obstacles/obstacles_lidar.pkl \
    --voxel 0.3 --safety 0.5 --auto-center --seed 42

# 3. Run the planner pointing at the new obstacles
python DECK_GA_QuickNav.py \
    --obstacles_pkl data/obstacles/obstacles_lidar.pkl \
    --points_pkl   data/points/points_current.pkl
```

## Supported input formats

`.pcd` `.ply` `.xyz` (Open3D) · `.las` `.laz` (laspy) · `.npy` `.bin` (numpy)

## Algorithm — Adaptive AABB Extraction

1. Load cloud (strip to xyz only).
2. Translate (`--translate x y z` or `--auto-center`).
3. Ground removal (`--ground-method ransac` default; see caveat below).
4. Voxelize at `r_voxel` using canonical centres (`np.floor(p/r) + 0.5`).
5. Merge by **exact Chebyshev clustering** (`scipy.cKDTree.query_pairs(p=inf)` + `connected_components`) with threshold `r_voxel + 2·s_safety`. Two voxels merge iff their inflated AABBs touch.
6. **Fill-ratio check**: `fill = (n_voxels · r_voxel³) / V_aabb`. If `fill ≥ τ_fill` (default 0.05), emit one inflated AABB.
7. **Median-axis split** otherwise: bisect along the longest axis at the median voxel-centre coordinate, recurse up to `max_split_depth=3`.

## Voxel resolution recommendations

| Scene scale | `--voxel` | Notes |
|---|---|---|
| O-134 indoor (~8×6×4 m) | 0.05–0.15 | Small obstacles, want tight fit |
| Outdoor pad (~50×50×30 m) | 0.2–0.5 | Trees, building corners |
| Survey site (~200×200×50 m) | 0.3–1.0 | Coarse — anything thinner explodes memory |
| Km-scale | 0.5–1.0 | Downsample upstream first |

Default uses `clip(scene_extent / 1000, 0.1, 1.0)`. Always overridable via `--voxel`.

## Caveats

- **RANSAC wall edge case.** If the dominant plane in the cloud isn't the floor (e.g. the cloud is mostly a wall), `--ground-method ransac` removes the wrong thing. Inspect Open3D output with `--viz`. Fall back to `--ground-method z_threshold --ground-z <floor_z>` for non-horizontal-dominant scans.
- **Vegetation split limitation.** Canopies have intrinsic point density too low for clean trunk/canopy separation by fill-ratio. Buildings, vehicles, rocks split well; trees become single conservative bounding boxes (or partially split, never cleanly trunk + canopy). Tune `--fill-threshold` if needed.
- **Coordinate frame.** Extracted obstacles inherit the input cloud's frame. If your waypoints live near the origin but the cloud is in UTM, use `--auto-center` or `--translate x y z`. A loud warning fires if the obstacle bbox is > 50 m from the optional `--waypoints-pkl` bbox.
- **AABB axis alignment.** Angled poles, beams, etc. appear fatter than they are in the AABB. Safe but conservative.
- **Headless / WSL.** `--no-viz` is the default. For interactive Open3D viewer (`--viz`), WSL needs WSLg or an X server. Matplotlib snapshots are saved either way via the Agg backend.

## Output format

A dict pickle, drop-in for `DECK_GA_QuickNav.py`:

```python
{
    'centers': np.ndarray (N, 3),       # cx, cy, cz per obstacle
    'sizes':   np.ndarray (N, 3),       # per-axis HALF-sizes sx, sy, sz
    'meta':    {
        'source':       'lidar',
        'input_cloud':  '...',
        'params':       {'voxel':..., 'safety':..., 'fill_threshold':..., ...},
        'safety_used':  float,
        'translate':    [x, y, z],
        'cloud_extent': [(xmin,xmax), (ymin,ymax), (zmin,zmax)],
        'timestamp':    'ISO 8601',
    },
}
```

`DECK_GA_QuickNav.py` auto-detects this dict format vs the legacy Nx3 centres-only format.

## Tests

```bash
python -m pytest perception/tests/ -v
```
