"""Measured execution time model: how long a planned path actually takes to fly.

WHY THIS EXISTS
---------------
`deckga_execute.py` plans with `L / speed` and nothing else. With
`--ensure_reach` the aircraft decelerates to a full stop at every waypoint
before the next `go_to` is issued, so the commanded speed is never the realised
speed and every waypoint costs a fixed extra amount. Planned makespan therefore
sits systematically below executed makespan, and -- this is the part that
matters -- the gap is NOT constant: it grows with waypoint count.

That makes the naive model unusable for comparing allocation methods, because
balancing MOVES WAYPOINTS BETWEEN DRONES. A method that happens to put fewer
waypoints on the critical UAV is flattered; one that puts more is penalised.
The comparison would be measuring the model's bias, not the method.

THE FIT
-------
    seg_time = distance / v_eff + t_stop

fitted over the 506 segments from the committed Gazebo runs under
tests/automation/results/*/gazebo/*/run_*_uav*_segments.csv, taking only the
LATEST run per (trial, uav) -- several trials were flown more than once, and
globbing all 30 files double-counts a stale attempt alongside its retry:

    v_eff  = 0.9301 m/s  +/- 0.016   (commanded 1.0)
    t_stop = NOT IDENTIFIED BY THIS DATA -- default 0.0
    R^2    = 0.899

Validated per UAV rather than per segment, which is the quantity actually
reported: 2.8% mean absolute error, ~10% worst, across 21 UAV-conditions
spanning n=30 and n=100 waypoints and 0/5/10/20 obstacles. The naive L/speed
model manages 6.75% with a -6.5% systematic bias, so the v_eff correction is
doing real work.

WHY t_stop DEFAULTS TO ZERO
---------------------------
The aircraft genuinely does stop at every waypoint under --ensure_reach, so a
positive per-waypoint cost must exist. This dataset cannot measure it:

    intercept = 0.077 s,  standard error 0.488,  95% CI [-0.879, +1.033]
    corr(slope, intercept) = -0.882

The CI straddles zero and the SE is six times the estimate. Every leg in the
dataset is long (2.0 - 81.8 m, mean 26.8 m), so the intercept is an
extrapolation far outside the data and trades off almost perfectly against the
slope. Fitting it anyway produces a confident-looking number that is noise --
an earlier pass through this same data, including the duplicate flights, gave
t_stop = 0.4425, which is 5.7x larger and fits no better per UAV (2.81% vs
2.77%).

So: v_eff is used, t_stop defaults to 0.0, and anyone who wants a non-zero value
must pass it in as an explicit assumption.

HOW TO ACTUALLY MEASURE IT
--------------------------
Short legs are what identify an intercept. The dense-obstacle arms of the
campaign generate exactly that -- QuickNav detour vertices sit close together,
often under a metre apart -- so re-running this fit on those bags should pin
t_stop properly. That is a by-product of the campaign, not a prerequisite.

CONSEQUENCE FOR THE MIN-MAX OBJECTIVE
-------------------------------------
With t_stop at 0, makespan-in-time is just makespan-in-distance divided by a
constant, so the two rank allocations identically. The min-max arms are
therefore testing BALANCING, not the time model. State it that way; do not
claim the time model is doing work it is not.

No obstacle term is needed either way: obstacles act purely through added
distance and added waypoints, both of which the model already has.

v_eff and t_stop are INSTANCE FIELDS, never module globals. They were fitted at
a commanded 1.0 m/s in Gazebo; the O-134 lab flights are commanded at 0.35-0.4
m/s indoors. A hard-coded global would silently mis-time the lab mission.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class TimeModel:
    v_eff: float
    t_stop: float

    def __post_init__(self) -> None:
        if not (self.v_eff > 0.0):
            raise ValueError(f"v_eff must be positive, got {self.v_eff}")
        if self.t_stop < 0.0:
            raise ValueError(f"t_stop must be >= 0, got {self.t_stop}")

    # -- primitives --------------------------------------------------------
    def seg_time(self, distance: float) -> float:
        """Time for one commanded leg, including the stop at its far end."""
        return float(distance) / self.v_eff + self.t_stop

    def tour_time(self, length: float, n_seg: int) -> float:
        """Time for a tour of `n_seg` legs totalling `length` metres."""
        return float(length) / self.v_eff + int(n_seg) * self.t_stop

    def path_time(self, path_xyz) -> float:
        """Time to fly a polyline. A 0- or 1-point path takes no time."""
        p = np.asarray(path_xyz, dtype=float)
        if p.ndim != 2 or p.shape[0] < 2:
            return 0.0
        length = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
        return self.tour_time(length, p.shape[0] - 1)

    def makespan(self, paths: Iterable) -> float:
        """Fleet makespan: the LONGEST tour, not the sum.

        The whole point of the min-max objective. UAVs fly concurrently, so the
        mission ends when the slowest one lands.
        """
        times = [self.path_time(p) for p in paths]
        return max(times) if times else 0.0

    # -- serialisation -----------------------------------------------------
    def as_dict(self) -> dict:
        return {"v_eff": self.v_eff, "t_stop": self.t_stop}

    @staticmethod
    def parse(text: str) -> "TimeModel":
        """Parse "V_EFF,T_STOP" as handed in on the command line."""
        parts = [p.strip() for p in str(text).split(",")]
        if len(parts) != 2:
            raise ValueError(
                f"time model must be 'V_EFF,T_STOP', got {text!r}")
        return TimeModel(v_eff=float(parts[0]), t_stop=float(parts[1]))

    @staticmethod
    def from_speed(speed: float) -> "TimeModel":
        """The LEGACY model: distance over commanded speed, no stop cost.

        Reproduces deckga_execute.planned_times exactly. Kept so the old and new
        numbers can be reported side by side rather than one silently replacing
        the other.
        """
        return TimeModel(v_eff=float(speed), t_stop=0.0)


# SUPERSEDED BY THE CAMPAIGN -- see GAZEBO_SPEED1_CAMPAIGN_FIT below, which is
# now the default. Kept because campaign_20261011's stored makespan_*_s columns
# were computed with THIS value, so it is needed to reproduce them.
#
# Fitted at a commanded 1.0 m/s in Gazebo on 506 segments.
GAZEBO_SPEED1_SMALL_SAMPLE = TimeModel(v_eff=0.9301, t_stop=0.0)

# The earlier fit, kept so the discrepancy is reproducible rather than folklore.
# It came from globbing all 30 segment files, which counts stale retry runs
# alongside their replacements. It predicts per-UAV time no better (2.81% vs
# 2.77% mean error) despite a 5.7x larger stop cost -- the two parameters are
# 88% correlated, so the data cannot tell them apart.
GAZEBO_SPEED1_POLLUTED_FIT = TimeModel(v_eff=0.9457, t_stop=0.4425)

# Refit from the FULL campaign: 5346 segments across all 80 Gazebo trials
# (campaign_20261011), against 506 clean segments for GAZEBO_SPEED1 above.
# This is the measurement the module docstring anticipated as "a by-product of
# the campaign".
#
#   v_eff    = 1.0004 m/s  +/- 0.006   -- i.e. the aircraft DO achieve the
#                                         commanded 1.0 m/s; the 0.9301 above
#                                         was an artefact of the smaller sample
#   t_stop   = -0.467 s    95% CI [-0.667, -0.267]
#   R^2      = 0.946, mean |err| 2.76 s/segment (vs 3.28 for GAZEBO_SPEED1)
#
# t_stop comes out NEGATIVE and significantly so, which is not a physical
# "time lost per stop" -- it is the linear model absorbing a mild
# misspecification: short legs are proportionally quicker because the aircraft
# never fully decelerates between close waypoints, so a straight line through
# (distance, time) has to tilt its intercept below zero. Treat t_stop here as a
# fitting artefact, not a dwell time. A saturating or two-regime model would
# describe it better and is the obvious next step if anyone needs per-segment
# accuracy rather than tour totals.
#
# NOT the default, deliberately. Every makespan_*_s already recorded in
# campaign_20261011 was computed with GAZEBO_SPEED1, and swapping the default
# would silently desynchronise the stored numbers from the constant that
# produced them. The practical consequence is bounded and worth stating
# plainly: with t_stop = 0, makespan_s is exactly max_tour_m / v_eff, a single
# scale factor applied identically to every arm. So absolute predicted seconds
# in the campaign are high by ~7.5% (0.9301 vs 1.0004), while EVERY relative
# arm-vs-arm result is exact, because a common factor cancels in a ratio. That
# is visible in the results themselves: the makespan-% and max-tour-% columns
# agree to the decimal.
GAZEBO_SPEED1_CAMPAIGN_FIT = TimeModel(v_eff=1.0004, t_stop=0.0)

# THE DEFAULT. 5346 segments beat 506, and the correction they were thought to
# justify turns out not to exist: at v_eff = 1.0004 the aircraft simply fly at
# their commanded 1.0 m/s. The campaign's own test suite is what forced this --
# `test_naive_model_is_measurably_worse` started FAILING because the naive
# model (distance / commanded speed) became the *better* of the two, 3.7% mean
# per-UAV error against 9.1% for the 0.9301 fit. A constant that is beaten by
# assuming no correction at all is not a correction.
#
# What this does NOT change: any relative result. With t_stop = 0, makespan is
# max_tour_m / v_eff -- one scale factor applied identically to every arm, which
# cancels in every ratio. campaign_20261011's stored absolute seconds are ~7.6%
# high and its percentages are exact; reproduce those numbers with
# GAZEBO_SPEED1_SMALL_SAMPLE.
GAZEBO_SPEED1 = GAZEBO_SPEED1_CAMPAIGN_FIT


def balance_ratio(times: Sequence[float]) -> float:
    """max / mean. 1.0 is a perfectly balanced fleet; higher is worse.

    Scale-free, so it compares across problem sizes, and it states directly how
    much makespan is recoverable by balancing alone: the committed Gazebo run
    gz_n30_o0_s0 has per-UAV distances 329.1 / 321.8 / 526.0 m, a ratio of 1.34,
    i.e. 25.4% of its makespan is available to a perfect allocator.
    """
    t = np.asarray(list(times), dtype=float)
    if t.size == 0:
        return 0.0
    mean = float(t.mean())
    if mean <= 0.0:
        return 1.0
    return float(t.max()) / mean


def fit_time_model(distances, durations, return_raw: bool = False) -> tuple:
    """Least-squares fit of seg_time = distance/v_eff + t_stop.

    Returns (TimeModel, r_squared, n), or with return_raw=True
    (TimeModel, r_squared, n, raw_intercept).

    THE RETURNED t_stop IS CLAMPED AT ZERO, and that is not a detail. A
    least-squares intercept is free to come out negative, and on the full
    campaign data it does: -0.47 s over 5346 segments, with a 95% CI of
    [-0.667, -0.267] that excludes zero. `TimeModel` rejects a negative t_stop
    because a negative dwell time is meaningless *as a model* -- it would
    predict that adding a waypoint makes a tour quicker.

    So the two things have to be kept apart: the fit is what the data says, and
    the model is what may be used to predict. A negative intercept is a signal
    that the straight line is slightly misspecified -- short legs are
    proportionally quicker because the aircraft never fully decelerates between
    close waypoints -- not a licence to subtract time per stop. Clamping keeps
    the model sound; `return_raw` keeps the measurement honest and visible.

    Exposed so the constants above can be re-derived from data rather than
    trusted, which is what planner/tests/test_timing.py does against the real
    segment CSVs.
    """
    d = np.asarray(distances, dtype=float)
    t = np.asarray(durations, dtype=float)
    if d.size != t.size:
        raise ValueError(f"length mismatch: {d.size} distances, {t.size} durations")
    if d.size < 2:
        raise ValueError("need at least two samples to fit")

    a = np.vstack([d, np.ones_like(d)]).T
    (slope, intercept), *_ = np.linalg.lstsq(a, t, rcond=None)
    if slope <= 0:
        raise ValueError(f"fitted slope {slope} is not positive; data is not a time model")

    pred = a @ np.array([slope, intercept])
    ss_res = float(((t - pred) ** 2).sum())
    ss_tot = float(((t - t.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    model = TimeModel(v_eff=1.0 / slope, t_stop=max(0.0, float(intercept)))
    if return_raw:
        return model, r2, int(d.size), float(intercept)
    return model, r2, int(d.size)
