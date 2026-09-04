# Automation harness

Automated test sweeps for the DECK-GA + QuickNav pipeline. Produces the §4
report metrics (path distance, planning time, detections, avoidances, residual
collisions) across a parameter matrix, plus an optional Gazebo execution subset.

## Layout

```
tests/automation/
  config.py              # MATRICES + TrialSpec
  metrics.py             # pure functions (distance, detect, avoid, residual)
  sweep_offline.py       # Stage 0 smoke + Stage 1 full matrix
  aggregate.py           # Stage 2 (mean/std per condition)
  report.py              # Stage 3 (markdown + plots)
  gazebo_runner.py       # Stage 4 lifecycle (full_gui/headless)
  sweep_gazebo.py        # Stage 4 driver
  collision_audit.py     # Stage 4a.5 (post-flight bag audit)
  fixtures/              # smoke_overlap fixtures, empty obstacle pkl
  tests/                 # pytest unit tests (no ROS dependency)
  results/               # gitignored output (per-run timestamped dir)
```

## Preconditions (WSL2 Ubuntu 22.04)

- ROS2 Humble sourced (`source /opt/ros/humble/setup.bash`)
- Aerostack2 + Gazebo Fortress installed
- `python3` with `numpy`, `matplotlib`, optionally `scipy` and `pyyaml`
- `pytest` (`pip3 install pytest`)
- `rosbag2_py` (bundled with `ros-humble-desktop`)

Stages 0–3 (offline, no Gazebo) run anywhere with Python + numpy. Stages 4a/4b
(Gazebo) require the full WSL2 stack.

## Order of operations

**1. Run unit + regression tests first.**
```bash
pytest tests/test_quicknav_fixes.py perception/tests/ tests/automation/ -q
```

**2. Offline smoke** (3 trials, ~6 min). This validates the planner pipeline
end-to-end, including the bug-A overlapping-obstacle fixture.
```bash
python tests/automation/sweep_offline.py --matrix smoke
```
On success the script prints `=== SMOKE PASS ===`. Note the timestamped run
dir it prints — you'll feed it to the next steps.

**3. Aggregate + report on the smoke output** (to confirm the report toolchain works).
```bash
python tests/automation/aggregate.py --run-dir tests/automation/results/<ts>
python tests/automation/report.py --run-dir tests/automation/results/<ts>
```

**4. Gazebo smoke (full GUI, ONE trial).** You watch live in RViz; the harness
records a ros2 bag; auto-runs `collision_audit.py` afterwards.
```bash
python tests/automation/sweep_gazebo.py --matrix gazebo_smoke --mode full_gui \
    --offline-run-dir tests/automation/results/<ts>
```
Verify in RViz that 3 drones spawn, paths render in green, obstacles in cyan,
then press Enter to launch the executor. After landing, the audit prints a
per-UAV pass/fail summary to stdout and writes `collision_audit.json`.

**REVIEW THE AUDIT** before continuing. Any `intersection` counts > 0 mean a
drone went meaningfully into an obstacle — investigate before running the full sweep.

**5. Full offline matrix** (80 trials, several hours).
```bash
python tests/automation/sweep_offline.py --matrix report
```
Resumable: if interrupted, re-invoke with `--run-dir <ts>` and it skips
already-completed trials.

**6. Gazebo subset (headless, 8 trials).** Re-uses the `discovered_topics.json`
written by step 4 to size the readiness timeout and pose-topic discovery.
```bash
python tests/automation/sweep_gazebo.py --matrix gazebo_subset --mode headless \
    --offline-run-dir tests/automation/results/<ts>
```
Fallback if headless launch repeatedly fails:
```bash
python tests/automation/sweep_gazebo.py --matrix gazebo_subset --mode full_gui \
    --no-prompt --offline-run-dir tests/automation/results/<ts>
```

**7. Re-aggregate + re-render the report** with Gazebo data joined in.
```bash
python tests/automation/aggregate.py --run-dir tests/automation/results/<ts>
python tests/automation/report.py --run-dir tests/automation/results/<ts>
```
The report now includes the "Executed vs Planned" section.

## Failure modes (what to look for in logs)

| Symptom | Diagnostic file | Mitigation |
|---|---|---|
| `status=skipped_overlap` | `trials/<id>/planner.log` (stderr has the conflict point) | seed unlucky; sweep continues |
| `status=executor_timeout` | `gazebo/<id>/executor.log` | drone failed to arm or behaviour server crashed; check log |
| `status=launch_timeout` | `gazebo/<id>/launch_as2.log` | tmux/Gazebo didn't come up; check that `LIBGL_ALWAYS_SOFTWARE=1` is set |
| `audit_verdict=FAIL` | `gazebo/<id>/collision_audit.json` | drone went into an obstacle; check incident list for obstacle_idx + timestamp |
| `[QuickNav] WARNING: max_iterations` | `trials/<id>/planner.log` | unsolvable scene; QuickNav gave up; reported in report.md failures section |
