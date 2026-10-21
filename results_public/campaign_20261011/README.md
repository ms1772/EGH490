# Campaign 2026-10-11: makespan reformulation of DECK-GA

These are the result tables behind the EGH490-2 final report, *Minimising Mission Time in Multi-UAV Path Planning: A Makespan Reformulation of DECK-GA* (M. Solomon, QUT, 2026).

They are copied unchanged, apart from line endings, from `tests/automation/results/campaign_20261011/`, which git ignores because that directory also holds about 40 GB of rosbags, media and cost caches. The procedure that produced them is in `tests/automation/campaign/RUNBOOK.md`.

| File | Rows | What it holds |
|---|---|---|
| `offline_master.csv` | 1,460 | One row per offline planning trial. Every row has `status = ok`. |
| `gazebo_master.csv` | 80 | One row per simulated flight: Gazebo, Aerostack2 and PX4 software-in-the-loop, 3 UAVs. Every row has `status = ok`. |
| `summary.csv` | — | Per-condition means, standard deviations and counts, aggregated from `offline_master.csv` by `tests/automation/aggregate.py`. |
| `arm_vs_ref_summary.csv` | — | Paired arm-vs-A0 contrasts at **run level**, from `aggregate.py`. |

## Trial groups (`trial_id` prefix in `offline_master.csv`)

| Prefix | Rows | Content |
|---|---|---|
| `cmp_` | 780 | 2×2 ablation at 3 UAVs. Each of the 60 scenes (10 waypoint layouts at 6 nested obstacle densities) has 13 runs: A0 once (it is deterministic) plus A1–A4 with 3 planner seeds each. |
| `flt_` | 600 | Fleet-size sweep, A0 against A4, 1–6, 8, 10 and 12 UAVs. |
| `gzc_` | 80 | The plans flown in Gazebo: 16 scenes (4 layouts at 0, 5, 10 and 20 obstacles) for each of 5 arms. |

## Arms

| Arm | Clustering | Edge cost | Objective |
|---|---|---|---|
| `A0_published` | DCKmeans (published DECK-GA) | straight line | min-sum |
| `A1_eu_minsum` | k-medoids | straight line | min-sum |
| `A2_eu_minmax` | k-medoids | straight line | min-max |
| `A3_qn_minsum` | k-medoids | QuickNav detour | min-sum |
| `A4_qn_minmax` | k-medoids | QuickNav detour | min-max |

## Key columns

**Scene keys**

- `n_points`, `n_obstacles`, `seed`: identify the scene. `seed` is the waypoint-layout seed.
- `planner_seed`: blank for A0.
- `num_uavs`

**Distances**

- `distance_*_total_m` and `distance_*_uavK_m`: planned path lengths.
  - `baseline` means before QuickNav avoidance.
  - `avoided` means after it.
- These are planned geometry, not measured flight distance.

**Makespan and balance**

- `max_tour_avoided_m`: the longest planned tour.
- `makespan_avoided_s`: the longest tour divided by the time model's effective speed. Divide by `time_model_v_eff` to get back to metres.
- `balance_ratio_avoided`: the longest tour divided by the mean tour. This is descriptive only.

**Planner-side checks**

- `residual_collisions_*`: intersections between the planned paths and the obstacle boxes, as checked by the planner.

**Gazebo columns**

- `planned_makespan_s`: the longest planned tour at the commanded 1 m/s.
- `mission_makespan_s`: the flown makespan, i.e. when the last UAV completes its final waypoint.
- `audit_*`: an independent check of the logged poses against the virtual obstacle boxes, with a 0.5 m tolerance for counting an intersection.
- `rtf_achieved`: the real-time factor actually achieved.

## Statistical note

`arm_vs_ref_summary.csv` reports run-level paired *t*-tests. Those overstate the evidence. A0 is reused across the three planner seeds, and the six obstacle densities of a layout share one waypoint file.

The report therefore averages within each waypoint layout first, then tests over the 10 layouts (*n* = 10). Use that unit for any inference.

## Integrity (SHA-256)

These checksums are for the files as stored in this repository. Git normalises line endings to LF, so a Windows checkout with CRLF endings will not match until it is converted back to LF.

```
a242f788582e78cccfa3682091a9d8788c4cf350c144163c0a1ce74bb793564a  offline_master.csv
ac4f4b022cec6da93665c7e30f75a3a37f76d7156cc1832a6d41bb6c47ef23b8  gazebo_master.csv
af5399a8fd3e7e2af585eadba556f95a0ffaf85c9553e0c41a44ff2945d55eab  summary.csv
26e896586484910b2805c2c0da2d77d42158af9480f050c0828a7d5e5e6d2c5e  arm_vs_ref_summary.csv
```
