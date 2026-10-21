"""Makespan-balanced reassignment: minimise the LONGEST tour, not the total.

DECK-GA's clustering minimises within-cluster spread, which is not the fleet
objective. UAVs fly concurrently, so the mission ends when the slowest one
lands. Measured on the committed Gazebo run gz_n30_o0_s0, the three executed
distances were 329.1 / 321.8 / 526.0 m: makespan 526.0 against a balanced-split
mean of 392.3, so 25.4% of that mission's duration was recoverable by
reallocation alone, invisible in any total-distance report.

WHY STRICT DESCENT DOES NOT WORK HERE
-------------------------------------
A plain hill-climber on a min-max objective stalls almost immediately. Move one
waypoint off the critical UAV and the receiver usually becomes critical at
essentially the SAME makespan -- a plateau, not a local optimum. So this accepts
the best admissible neighbour even when it does not improve, walks the plateau,
and keeps `best` separate from `current`.

Returning `best` rather than `current` is what makes "never worse than the
starting allocation" true by construction, on the estimator.

Cycling is prevented three ways: a tabu list on (waypoint, source UAV) with
aspiration so it cannot block a genuine improvement; `best` decoupled from
`current`, so a cycle costs time but cannot corrupt the result; and an absolute
epsilon so float noise never registers as progress.

ESTIMATOR CONSISTENCY IS A HARD RULE
------------------------------------
The same estimator scores the incumbent and every candidate. Scoring the
incumbent with the real GA and candidates with the cheap estimator would make
the incumbent look better than every alternative by construction, and this
module would become a no-op that reports success. The estimator's absolute error
(~4% against the GA) matters far less than the fact that its bias largely
cancels between two allocations differing by one waypoint.

Practical consequence: do not chase improvements below about 2% of a UAV's tour
cost. They are inside the estimator's noise.

STARTS ARE DISTINCT PER UAV
---------------------------
Which cluster goes to which UAV changes the answer, because each aircraft starts
somewhere different. For k <= 6 every permutation is enumerated exactly (6
evaluations at k=3), re-run after each accepted move -- moving waypoints changes
which start suits which cluster, and optimising against a stale pairing silently
wastes most of the available gain.
"""
from __future__ import annotations

import itertools
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from planner.tour import estimate_cost


@dataclass
class BalanceConfig:
    max_iters: int = 300
    max_no_improve: int = 40
    max_seconds: float = 20.0
    tabu_tenure: int = 7
    allow_swap: bool = True
    swap_candidates: int = 5
    move_candidates: int = 12      # cheap pre-ranking, see _rank_relocations
    eps_s: float = 1e-6
    closed_tours: bool = True


@dataclass
class BalanceResult:
    labels: np.ndarray
    uav_of_cluster: np.ndarray
    times: np.ndarray
    makespan: float
    makespan0: float
    n_moves: int
    n_evals: int
    terminated: str
    history: list = field(default_factory=list)

    @property
    def improvement(self) -> float:
        if self.makespan0 <= 0:
            return 0.0
        return (self.makespan0 - self.makespan) / self.makespan0


class _Evaluator:
    """Costs one allocation, and counts how often it was asked."""

    def __init__(self, cost, time_model, start_nodes, k, closed):
        self.cost = cost
        self.tm = time_model
        self.starts = np.asarray(start_nodes, dtype=int)
        self.k = int(k)
        self.closed = bool(closed)
        self.n_evals = 0

    def cluster_time(self, labels, cluster, uav) -> float:
        members = np.flatnonzero(labels == cluster)
        self.n_evals += 1
        if members.size == 0:
            return 0.0
        return estimate_cost(self.cost, members.tolist(), int(self.starts[uav]),
                             self.tm, closed=self.closed).time_s

    def all_times(self, labels, uav_of_cluster) -> np.ndarray:
        return np.array([self.cluster_time(labels, j, uav_of_cluster[j])
                         for j in range(self.k)], dtype=float)


def best_pairing(ev: _Evaluator, labels, objective: str = "minmax") -> tuple:
    """Exact cluster -> UAV assignment, by enumeration.

    k! is 6 at k=3 and 120 at k=5, each evaluation ~1.5 ms, so exhaustive search
    is simply cheaper than reasoning about a heuristic.

    `objective` MUST match the arm's own objective, which is why it is a
    parameter rather than fixed at makespan. The pairing is itself an
    optimisation, so a "minsum" arm whose clusters were handed to UAVs by a
    makespan-optimal permutation is not a min-sum method at all -- it is
    partially min-max, and a 2x2 ablation built on it would attribute only the
    waypoint reassignment to the objective and understate the rest. Keeping
    them aligned means each arm optimises one objective everywhere.

    Defaults to "minmax" so `balance_makespan`'s own calls are unchanged.
    """
    k = ev.k
    if k > 6:
        order = np.arange(k)
        return order, ev.all_times(labels, order)

    if objective not in ("minmax", "minsum"):
        raise ValueError(f"objective must be minmax or minsum, got {objective!r}")

    def score(times: np.ndarray) -> float:
        # max -> fleet completion time; sum -> total flight time, the quantity
        # the published formulation actually minimises.
        return float(times.max()) if objective == "minmax" else float(times.sum())

    best_perm, best_times, best_score = None, None, np.inf
    for perm in itertools.permutations(range(k)):
        p = np.array(perm, dtype=int)
        times = ev.all_times(labels, p)
        s = score(times)
        if s < best_score - 1e-12:
            best_perm, best_times, best_score = p, times, s
    return best_perm, best_times


def _rank_relocations(cost, labels, src, dst_list, medoids, cap):
    """Cheaply shortlist which waypoints are worth trying to move.

    Evaluating every (waypoint, destination) pair with the tour estimator is far
    too slow inside a search loop. Ranking by `C[p, medoid_dst] - C[p, medoid_src]`
    costs one matrix lookup per candidate and puts the genuinely misplaced
    waypoints -- close to another cluster, far from their own -- at the front.
    """
    members = np.flatnonzero(labels == src)
    if members.size <= 1:
        return []
    out = []
    for dst in dst_list:
        delta = cost.length[members, medoids[dst]] - cost.length[members, medoids[src]]
        order = np.argsort(delta)[:cap]
        out.extend((int(members[i]), int(dst), float(delta[i])) for i in order)
    out.sort(key=lambda t: t[2])
    return out[:cap]


def balance_makespan(cost, time_model, labels0, start_nodes, *,
                     medoids=None, cfg: Optional[BalanceConfig] = None,
                     rng=None) -> BalanceResult:
    """Reassign waypoints to minimise the fleet's worst tour time."""
    cfg = cfg or BalanceConfig()
    labels = np.asarray(labels0, dtype=int).copy()
    starts = np.asarray(start_nodes, dtype=int)
    k = int(starts.size)

    ev = _Evaluator(cost, time_model, starts, k, cfg.closed_tours)

    if medoids is None:
        medoids = np.array(
            [int(np.flatnonzero(labels == j)[0]) if (labels == j).any() else 0
             for j in range(k)], dtype=int)
    else:
        medoids = np.asarray(medoids, dtype=int)

    uav_of_cluster, times = best_pairing(ev, labels)
    makespan0 = float(times.max())

    best_labels = labels.copy()
    best_pair = uav_of_cluster.copy()
    best_times = times.copy()
    best_ms = makespan0

    tabu: dict = {}
    history = [makespan0]
    n_moves = 0
    no_improve = 0
    terminated = "converged"
    t0 = time.perf_counter()

    for it in range(cfg.max_iters):
        if time.perf_counter() - t0 > cfg.max_seconds:
            terminated = "time_limit"
            break

        src = int(np.argmax(times))
        others = [j for j in range(k) if j != src]
        if not others:
            terminated = "single_cluster"
            break

        # Only a receiver that is currently below the makespan can help.
        receivers = [j for j in others if times[j] < times[src] - cfg.eps_s]
        if not receivers:
            receivers = others

        candidates = []
        for p, dst, _ in _rank_relocations(cost, labels, src, receivers,
                                           medoids, cfg.move_candidates):
            candidates.append(("relocate", p, dst, None))

        if cfg.allow_swap:
            src_members = np.flatnonzero(labels == src)
            for dst in receivers:
                dst_members = np.flatnonzero(labels == dst)
                if src_members.size == 0 or dst_members.size == 0:
                    continue
                for p in src_members[:cfg.swap_candidates]:
                    near = dst_members[np.argsort(cost.length[p, dst_members])[:2]]
                    for q in near:
                        candidates.append(("swap", int(p), int(dst), int(q)))

        if not candidates:
            terminated = "no_moves"
            break

        scored = []
        for kind, p, dst, q in candidates:
            trial = labels.copy()
            if kind == "relocate":
                trial[p] = dst
            else:
                trial[p], trial[q] = dst, src
            if not (trial == src).any():
                continue          # never empty a cluster
            t_src = ev.cluster_time(trial, src, uav_of_cluster[src])
            t_dst = ev.cluster_time(trial, dst, uav_of_cluster[dst])
            cand_times = times.copy()
            cand_times[src], cand_times[dst] = t_src, t_dst
            scored.append((float(cand_times.max()), float(cand_times.sum()),
                           kind, p, dst, q, cand_times))

        if not scored:
            terminated = "no_moves"
            break

        # Tie-break on the fleet sum so the search cannot stall on a plateau.
        scored.sort(key=lambda s: (s[0], s[1]))

        chosen = None
        for s in scored:
            is_tabu = tabu.get((s[3], src), -1) > it
            if (not is_tabu) or s[0] < best_ms - cfg.eps_s:   # aspiration
                chosen = s
                break
        if chosen is None:
            terminated = "all_tabu"
            break

        ms, _, kind, p, dst, q, cand_times = chosen
        if kind == "relocate":
            labels[p] = dst
        else:
            labels[p], labels[q] = dst, src
        tabu[(p, src)] = it + cfg.tabu_tenure
        n_moves += 1

        # Re-pair: moving waypoints changes which start suits which cluster.
        uav_of_cluster, times = best_pairing(ev, labels)
        history.append(float(times.max()))

        if times.max() < best_ms - cfg.eps_s:
            best_ms = float(times.max())
            best_labels = labels.copy()
            best_pair = uav_of_cluster.copy()
            best_times = times.copy()
            no_improve = 0
        else:
            no_improve += 1
            if no_improve >= cfg.max_no_improve:
                terminated = "no_improvement"
                break

    return BalanceResult(
        labels=best_labels,
        uav_of_cluster=best_pair,
        times=best_times,
        makespan=best_ms,
        makespan0=makespan0,
        n_moves=n_moves,
        n_evals=ev.n_evals,
        terminated=terminated,
        history=history,
    )
