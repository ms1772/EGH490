# Campaign runbook

Operating instructions for the unattended simulation campaign. Every stage
states its command, its expected duration, what "done" looks like, and what to
do when it is not done.

The campaign is designed to need **no decisions while it runs**. Anything that
would require one is either decided here in advance or made into a hard failure.

---

## Settled by measurement (do not re-litigate)

| Question | Answer | Evidence |
|---|---|---|
| Real-time factor | **RTF 1** | Requesting 2.0 achieved 1.179 and 3.0 achieved 1.342, both *faithful* (sim-normalised makespan +1.1% / +10.2%, flown geometry +0.3% / +0.8%). RTF 2 passes every gate, but achieved RTF is load-dependent across an unattended run and the gradient between 1.18 and 1.34 is steep, so a *varying* time base injects load-correlated noise into makespan -- the headline metric -- where a uniform bias would cancel. 2.2 h of ~16 h is not worth that. See `rtf_acceptance_measured.md`. |
| True headless (`-s`) | **Not used** | It removes the GUI and therefore `/gui/screenshot`, which is how every figure is captured. Its only benefit was speed, moot at RTF 1 (the bottleneck is IPC -- 231k interrupts/s at ~30% CPU idle -- not rendering). |
| Offline parallelism | **`--jobs 8`** | The sweep is single-threaded and pins one core of twenty. Grouped **by scene** so a scene's trials share their cost matrix instead of racing to rebuild it. 780 trials: ~8 h serial, ~1 h at 8 workers. |
| Gazebo parallelism | **Always serial** | A concurrent test suite pushed a trial's achieved RTF to 0.969 and its makespan 8% above the historical value for the identical plan. |
| Python environment | **WSL `python3` (3.10.12)** throughout | The Gazebo stage unpickles planner output in WSL; Windows here is numpy 2.4.4 against WSL's 1.21.5, and 2.x pickles are not reliably readable by 1.21. All 107 project files are compile-checked under 3.10 -- a backslash in an f-string expression is legal on Windows 3.14 and a SyntaxError on 3.10. |
| Timing metrics | Measure with `--jobs 1` | `planning_time_*` is subprocess wall time, so a parallel run is not a clean timing measurement. The metrics the campaign exists for are deterministic and unaffected. |

### Two metrics that are not what their names suggest

* **`executed_uavN_dist_m` / `executed_total_dist_m` / `planned_makespan_s`** are
  **planned geometry, not measurement**. `deckga_execute.py:355` sums commanded
  waypoints and never reads a pose; `--speed` defaults to 1.0 m/s so
  `planned_makespan_s` is numerically a length. Measured identical to 15
  significant figures across RTF 1, 2 and 3. For real flown distance or tracking
  deviation use `campaign/bag_distance.py`, which integrates bag poses at 2 Hz
  (a full-rate integral is 4.9x inflated by estimator jitter).
* **`mission_makespan_s`** is **wall-clock**, not sim time -- every executor
  timestamp is `time.perf_counter()`. Multiply by achieved RTF to compare across
  real-time factors. At RTF 1 the distinction is immaterial, which is one more
  reason to stay there.

---

## Before you start

```bash
# from WSL, repo root
cd "/mnt/c/Users/mitch/Multi-UAV Project/ROS2_MultiUAV_3D-main"
source /opt/ros/humble/setup.bash
export DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1 MPLBACKEND=Agg
```

Three things that are easy to get wrong and expensive to discover late:

1. **Nothing else may run during a Gazebo stage.** Measured: running the test
   suite alongside one trial dropped its achieved RTF to 0.969 and inflated its
   makespan by 8% against the identical plan. The RTF acceptance threshold is
   2%. Concurrent work does not slow the campaign, it corrupts it.
2. **`DISPLAY` must be set in the shell that launches the campaign**, not just
   in an interactive login. A detached run without it produces
   `RenderingAPIException: Couldn't open X display :0` and every trial fails.
3. **The machine must stay awake.** Closing the lid suspends WSL; the campaign
   resumes cleanly afterwards but the wall-clock estimate stops meaning
   anything.

Check the machine is idle before any Gazebo stage:

```bash
ps -eo pid,cmd | grep -E "ign gazebo|deckga_execute|ros2 bag" | grep -v grep
# expect no output
```

---

## Stage 0 — RTF calibration (~45 min)

**Decides whether the campaign is ~30 h or ~12 h.** Runs the identical trial at
several real-time factors and accepts the fastest that does not change the
flight.

```bash
python3 tests/automation/campaign/rtf_calibrate.py \
  --offline-run-dir tests/automation/results/20260520-163723 \
  --run-dir tests/automation/results/rtf_cal_<ts> \
  --trial-id gz_n30_o20_s0 \
  --rtfs 1,2,3,5
```

**Done when** `rtf_calibration.md` exists and names a chosen factor.

**Acceptance** (applied automatically, per factor, against the 1.0x control):
per-UAV executed distance within 2%, sim-time makespan within 2%, residual
collisions identical, and achieved RTF at least 80% of requested.

**If no factor above 1.0 passes:** run the campaign at 1.0. Do not override.
A rejected factor means the aircraft flew a different path, which would
contaminate every number the campaign produces.

**Why sim-time makespan and not wall clock:** wall time is *supposed* to shrink.
If sim time shrinks too, the aircraft flew a shorter path — that is the failure
being tested for, not the success.

---

## Stage 1 — Pilot (~5 min)

```bash
python3 tests/automation/sweep_offline.py --matrix smoke --run-dir <run_dir>
```

**Done when** it exits 0. The smoke matrix runs its own assertions and exits 2
on failure.

**This is the gate.** Nothing past it launches until the pilot passes and its
`report.md` has been looked at. A broken planner that reaches the Gazebo stage
costs a night.

---

## Stage 2 — Cost-matrix pre-warm (no longer a separate stage)

The cache is wired into the sweep and **self-warms**: the first trial of a
(scene, cost-kind) builds the matrix and every later arm and planner seed hits
it. Verified on the pilot -- one miss per scene, hits thereafter, with identical
detour-edge counts across all three planner seeds and both obstacle-aware arms.

What *does* need its own stage is **`s4_gazebo_plans`**: the Gazebo matrix uses
`gzc_*` trial ids and shares **none** with the offline matrix's `cmp_*`, so its
80 plans must be built explicitly. Without that stage every Gazebo trial aborts
as `missing_pickle`.

## (historical) Stage 2 notes

Builds the pairwise QuickNav cost matrix once per scene. The matrix depends only
on the scene, so one build serves every arm and planner seed of it — 15 reuses.

Measured build cost: n=30/M=20 0.8 s · n=100/M=20 14 s · n=100/M=50 96 s ·
**n=100/M=100 519 s**.

**Done when** `cost_cache/` holds one `.npz` per scene.

**The sweep then runs with `require_hit=True`**, so a cache miss is a hard error
rather than 900 trials each silently rebuilding a 519-second matrix.

---

## Stages 3-4 — Offline sweeps (~1 h at `--jobs 8`)

```bash
python3 tests/automation/sweep_offline.py --matrix <name> --run-dir <run_dir>
```

**Done when** `offline_master.csv` has one row per trial with `status=ok`.

**Resume:** re-run the same command. `_purge_non_ok_rows` drops every non-ok row
so it is retried; `status=ok` trials are skipped by `trial_id`.

**Expected non-fatal statuses:**

| status | meaning | action |
|---|---|---|
| `skipped_overlap` | a waypoint fell inside an obstacle after 4 reseeds | should not occur now that obstacle generation uses rejection sampling; if it does, the density is too high for the obstacle size |
| `harness_error` | a bug in the harness, not the planner | traceback is in `trials/<id>/harness_error.txt`; the sweep continues |
| `subprocess_error` | the planner raised | read `trials/<id>/planner.log` |

---

## Stage 5 — Gazebo (~16 h at RTF 1, serial)

```bash
python3 tests/automation/sweep_gazebo.py \
  --matrix <name> --mode headless --no-prompt \
  --offline-run-dir <run_dir> --run-dir <run_dir> \
  --rtf <from stage 0> --executor-timeout 3600
```

**`--no-prompt` is mandatory.** Without it, a detached run hits
`_wait_for_manual_confirmation`, stdin is at EOF, it returns False, and **every
trial aborts instantly**.

**Order matters:** n=30 conditions before n=100. If the night goes wrong, the
cheap half is already banked.

**Done when** `gazebo_master.csv` has one row per trial. Check
`rtf_achieved` is close to `rtf_requested` on every row — a machine that could
not deliver the factor produces a slower, not a wrong, run, but it should be
visible rather than assumed.

**Per-trial failure policy** (automatic): retry twice, then record and continue.
A stage aborts only if more than 30% of its trials fail, which indicates
something systemic rather than one bad scene.

---

## Stage 6 — Capture and figures

Three artefacts, deliberately produced three different ways.

| artefact | how | simulator? | when |
|---|---|---|---|
| Gazebo stills | `capture.py`, fired inside the trial window | yes, inline | automatic on the 4 capture trials during S5 |
| Mission video | `campaign/media_run.sh` | yes, dedicated run | after S5 |
| Planned-vs-flown 3-D | `campaign/render_paths.py` | **no** | any time after S5 |

```bash
# Figures from saved artefacts. No Gazebo, no X, no live session.
python3 tests/automation/campaign/render_paths.py --run-dir <run_dir>

# Video + large-window stills. Flies 2 dedicated trials; its rows are DISCARDED.
bash tests/automation/campaign/media_run.sh
```

**Why the video is a separate run.** Capture during a mission fires `ign
service` subprocesses, and the 4 captured campaign trials are also data rows in
the paired comparison -- so they take stills only, outside the flight window,
perturbing nothing. Likewise the campaign keeps the default 1000x845 window:
`LIBGL_ALWAYS_SOFTWARE=1` means software rasterisation, so a 1600x1000 viewport
is ~4x the per-frame CPU cost on all 80 trials for the benefit of four.
`media_run.sh` enlarges the window, snapshots `gui.config` into its run dir and
restores it from an EXIT trap, so an interrupt still puts the shared config back.

**Prefer `render_paths.py` to RViz.** It reads the planner pickle and the bag
directly and writes a 2200x1700 figure at 200 dpi, against RViz's
screen-resolution screenshot; it needs no live X session, no marker nodes and
no window-name match, each of which can fail silently into a blank image.
Validated against a real bag: planned and flown overlap closely, as expected at
1 m/s.

If you do want RViz, it must use
`flight_ops/lab_config/o134_project/config_ground_station/o134.rviz`, **not**
the harness's `rviz2_config.rviz` — that one has zero `/deckga/*` displays and
would show drones and TF but no planned paths or obstacles. Note the harness's
own prompt text claims "paths in green, obstacles as cyan cubes"; obstacles are
in fact **red** (`rviz_obstacles_node.py:116-119`).

---

## Fleet-size sweep (optional, offline, ~20 min)

```bash
python3 tests/automation/sweep_offline.py --matrix campaign_fleet --jobs 8     --run-dir <run_dir>
```

240 trials: A0 and A4 only, over uavs {1,2,3} x n {30,100} x obs {20,100} x 5
scenes. Asks whether the makespan advantage grows, shrinks or holds with fleet
size -- a different question from the 2x2 ablation, which is why it does not
need all five arms. Each fleet size has its own start points
(`config.FLEET_START_POINTS`); they are NOT a prefix of the three-UAV default,
because starting the single-UAV case in a corner would make the comparison
partly about where the aircraft began. `k=1` and `k=2` are verified working.

Offline only, so run it whenever -- it never needs the simulator.

---

## Stage 7 — Aggregate and report (~5 min)

```bash
python3 tests/automation/aggregate.py --run-dir <run_dir>
python3 tests/automation/report.py    --run-dir <run_dir>
```

---

## Running the whole thing

```bash
# The entry point. Sets ROS, DISPLAY, LIBGL_ALWAYS_SOFTWARE and MPLBACKEND --
# each of which caused a failure once when it was missing.
nohup bash tests/automation/campaign/run_campaign.sh \
  tests/automation/results/<ts> > /dev/null 2>&1 &

# JOBS and RTF are environment overrides; the defaults are 8 and 1.0.
```

Stage order is `s1_pilot, s3_offline_method, s4_gazebo_plans, s5_gazebo,
s7_aggregate, s7_report`. Screenshot trials default to A0 and A4 at both
fleet sizes in the densest Gazebo condition, derived from the resolved
matrix rather than hardcoded.

The driver logs to `<run_dir>/campaign/campaign.log` (appended, never
truncated) and writes `<run_dir>/campaign/heartbeat.json` every 60 s.

**To check on it without disturbing anything:**

```bash
cat <run_dir>/campaign/heartbeat.json
tail -20 <run_dir>/campaign/campaign.log
ls <run_dir>/campaign/*.done
```

**To restart after an interruption:** re-run the same command. The driver skips
stages with a `.done` marker and the sweeps skip completed trials.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Every Gazebo trial `aborted` immediately | full_gui without `--no-prompt`; stdin EOF | add `--no-prompt` |
| Every trial `launch_timeout` | Gazebo cannot render | `export LIBGL_ALWAYS_SOFTWARE=1`, check `DISPLAY` |
| `rtf_achieved` far below requested | machine loaded, or the factor is beyond it | stop other work; re-run Stage 0 |
| Trials slow down over hours | orphaned simulators accumulating | `guards.kill_sim_orphans()`; the driver does this between stages |
| `n_detour_edges == 0` with obstacles present | cost matrix built in the wrong frame | the planner raises on this; it should be impossible |
| Disk fills mid-run | bags accumulating | guards abort the stage cleanly at 5 GB free; bags stage on WSL ext4 (949 GB), results on `/mnt/c` (53 GB) |
| A stage exits 0 but no rows | matrix filtered to nothing | check `--only-trial` / matrix name |

### Reverting the system changes

```bash
wsl -u root -- apt-get remove -y ffmpeg imagemagick
wsl -u root -- apt-get autoremove -y
```

`graphviz` was already installed before this campaign — do **not** remove it.
Full before/after state is in `installed_packages.txt`.
