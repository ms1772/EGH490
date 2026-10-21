import numpy as np


# Default for the flip-mutation defect described at the mutation block below.
# False == corrected. Exposed as a module constant so a caller can flip the
# whole pipeline without threading a keyword through four call sites.
FLIP_BROKEN_DEFAULT = False


class GAResult:
    """Richer return for callers that pass return_route=True."""

    __slots__ = ("route_points", "route_indices", "opt_cost", "dist_history", "n_iter")

    def __init__(self, route_points, route_indices, opt_cost, dist_history, n_iter):
        self.route_points = route_points
        self.route_indices = route_indices
        self.opt_cost = opt_cost
        self.dist_history = dist_history
        self.n_iter = n_iter


def ga_3d_pathplanning(xyz, *, dmat=None, num_iter=None, pop_size=16,
                       rng=None, return_route=False, verbose=True,
                       flip_broken=None):
    """GA tour optimiser over a cost matrix.

    Called with a single positional argument it behaves exactly as it always
    has, which is what keeps DECK_GA.py:205, DECK_GA_new.py:33 and
    deckga_ros2/generate_deckga_output.py:101 working untouched.

    dmat
        Optional precomputed (N, N) cost matrix. None builds the Euclidean one
        from `xyz` using the original np.meshgrid expression -- deliberately not
        scipy.spatial.distance.cdist, whose rounding differs and would break the
        bitwise-equivalence tests. Supplying a non-Euclidean matrix is the whole
        point of the obstacle-aware arms: the search loop touches only `dmat`
        and integer permutations, never coordinates.

        The matrix MUST be symmetric. The flip operator is a segment reversal,
        whose cost-neutrality on the reversed interior is only true when
        C[i,j] == C[j,i]. It must also be finite everywhere: an np.inf entry
        would leave globalMin at inf and optRoute unbound.
    num_iter
        None -> 1e5, the original.
    rng
        None -> the legacy global np.random calls, preserving the accidental
        reproducibility that DCKmeans' np.random.seed(42) confers. A Generator
        makes seeding explicit. Note the two paths draw DIFFERENT values from
        the same conceptual stream, so a Generator run is never comparable to a
        global-stream run -- do not claim it is.
    return_route
        False -> the original (N+1, 3) point array. True -> GAResult, carrying
        the route INDICES and the optimised cost, so a caller using a
        non-Euclidean dmat can report what was actually minimised rather than
        re-measuring the straight-line length of the returned points.
    """
    if flip_broken is None:
        flip_broken = FLIP_BROKEN_DEFAULT

    _perm = np.random.permutation if rng is None else rng.permutation
    _randint = ((lambda lo, hi, size: np.random.randint(lo, hi, size=size))
                if rng is None else
                (lambda lo, hi, size: rng.integers(lo, hi, size=size)))

    # Initialize the variables
    N = xyz.shape[0]  # Number of points
    popSize = int(pop_size)
    numIter = int(1e5) if num_iter is None else int(num_iter)

    # Generate distance matrix in 3D
    if dmat is None:
        a = np.meshgrid(np.arange(N), np.arange(N))
        dmat = np.sqrt(np.sum((xyz[a[0], :] - xyz[a[1], :])**2, axis=2))
    else:
        dmat = np.asarray(dmat, dtype=float)
        if dmat.shape != (N, N):
            raise ValueError(f"dmat must be ({N}, {N}), got {dmat.shape}")
        if not np.all(np.isfinite(dmat)):
            raise ValueError(
                "dmat contains non-finite entries; use a large finite penalty "
                "for infeasible pairs instead (np.inf leaves optRoute unbound)"
            )

    # Sanity checks for population size
    popSize = 4 * int(np.ceil(popSize / 4))
    numIter = max(1, round(numIter))

    # Initialize population
    pop = np.zeros((popSize, N), dtype=int)
    pop[0, :] = np.arange(N)  # Start with sequential route
    for k in range(1, popSize):
        pop[k, :] = _perm(N)  # Random routes for the population

    # GA variables
    globalMin = np.inf
    distHistory = np.zeros(numIter)
    # Bound up front. It is always assigned on the first generation (any finite
    # cost beats inf), but leaving it implicit meant an all-inf dmat produced an
    # UnboundLocalError 100k iterations later instead of a clear failure.
    optRoute = pop[0, :]

    for iter in range(numIter):
        # Vectorised closed-tour length for the whole population at once.
        #
        # BIT-IDENTICAL to the scalar double loop it replaces, which matters:
        # the next two lines take an argmin over these values, so a difference
        # in the last bits can select a different individual, change how many
        # RNG draws the generation consumes, and diverge the entire run.
        #
        # np.cumsum is a plain left-to-right accumulation over float64, so it
        # reproduces `d = closing_edge; d += seg0; d += seg1; ...` exactly.
        # ndarray.sum(axis=1) does NOT -- it uses pairwise summation and would
        # add the closing edge last instead of first. Measured over 400 random
        # (N=35, popSize=16) cases: cumsum 0/400 mismatches, .sum(1) 400/400.
        cols = np.concatenate(
            [dmat[pop[:, -1], pop[:, 0]][:, None], dmat[pop[:, :-1], pop[:, 1:]]],
            axis=1,
        )
        totalDist = np.cumsum(cols, axis=1)[:, -1]

        minDist = np.min(totalDist)
        index = np.argmin(totalDist)
        distHistory[iter] = minDist

        if minDist < globalMin:
            globalMin = minDist
            optRoute = pop[index, :]

        # Genetic Algorithm Operators: mutation and crossover
        randomOrder = _perm(popSize)

        if not flip_broken:
            # Vectorised generation step, bit-identical to the scalar loop below.
            #
            # The four per-group (I, J) draws stay a Python loop on purpose: it
            # costs nothing at G=4 and it preserves the RNG stream order exactly,
            # which is what makes the bit-identity test against the scalar path
            # meaningful. Everything else operates on all groups at once.
            #
            # Only reachable with the flip fixed, because the broken path needs
            # the np.unique repair, whose firing pattern is inherently serial.
            G = popSize // 4
            IJ = np.empty((G, 2), dtype=int)
            for g in range(G):
                IJ[g] = np.sort(_randint(0, N, 2))

            rtes = pop[randomOrder].reshape(G, 4, N)
            dists = totalDist[randomOrder].reshape(G, 4)
            best = rtes[np.arange(G), np.argmin(dists, axis=1), :]      # (G, N)

            Icol = IJ[:, 0][:, None]
            Jcol = IJ[:, 1][:, None]
            col = np.arange(N)[None, :]
            inseg = (col >= Icol) & (col <= Jcol)

            kids = np.empty((G, 4, N), dtype=int)
            kids[:, 0, :] = best                                        # clone
            # flip: within [I, J] read position I+J-x, i.e. a mirrored gather
            kids[:, 1, :] = np.take_along_axis(
                best, np.where(inseg, Icol + Jcol - col, col), axis=1)
            # swap: exchange the two endpoints
            rows = np.arange(G)
            sw = best.copy()
            sw[rows, IJ[:, 0]] = best[rows, IJ[:, 1]]
            sw[rows, IJ[:, 1]] = best[rows, IJ[:, 0]]
            kids[:, 2, :] = sw
            # slide: np.roll(sub, -1) is a modular gather over the sub-block
            seglen = Jcol - Icol + 1
            kids[:, 3, :] = np.take_along_axis(
                best, np.where(inseg, Icol + ((col - Icol + 1) % seglen), col), axis=1)

            pop = kids.reshape(popSize, N)
            continue

        newPop = np.zeros((popSize, N), dtype=int)
        for p in range(0, popSize, 4):
            rtes = pop[randomOrder[p:p+4], :]
            dists = totalDist[randomOrder[p:p+4]]
            idx = np.argmin(dists)
            bestOf4Route = rtes[idx, :]
            routeInsertionPoints = np.sort(_randint(0, N, 2))
            I, J = routeInsertionPoints

            # Mutation operations (flip, swap, slide)
            tmpPop = np.zeros((4, N), dtype=int)
            tmpPop[0, :] = bestOf4Route

            # Flip mutation -- 2-opt segment reversal.
            #
            # The parent MUST be copied before the slice is reversed. The
            # original did not, so every index outside [I, J] kept the 0 from
            # the np.zeros above, the child was not a permutation, and the
            # integrity guard further down replaced it with a uniform random
            # restart. Measured at N=35: that fired on ~96.9% of draws, making
            # ~24% of every generation random noise and removing 2-opt -- the
            # move that makes a GA competitive on TSP -- almost entirely.
            # Compare the swap and slide blocks below: both .copy() first.
            #
            # Fixing it improved tour length by up to 5.5% at N=35 and changed
            # nothing at N=11, which is why it went unnoticed: the repo's own
            # tests only ever exercised small instances.
            #
            # flip_broken=True reproduces the defect exactly, so the before/after
            # comparison stays runnable after this is merged. Note that `np.sort`
            # on routeInsertionPoints guarantees I <= J, which is why the
            # original's `elif I > J` branch was dead code and is gone.
            if flip_broken:
                if I < J:
                    tmpPop[1, I:J+1] = bestOf4Route[I:J+1][::-1]
                else:
                    tmpPop[1, :] = bestOf4Route
            else:
                tmpPop[1, :] = bestOf4Route.copy()
                tmpPop[1, I:J+1] = bestOf4Route[I:J+1][::-1]

            # Swap mutation
            tmpPop[2, :] = bestOf4Route.copy()
            tmpPop[2, [I, J]] = bestOf4Route[[J, I]]

            # Slide mutation
            tmpPop[3, :] = bestOf4Route.copy()
            tmpPop[3, I:J+1] = np.roll(bestOf4Route[I:J+1], shift=-1)

            # Population integrity.
            #
            # With the flip fixed, all four operators (clone, segment reversal,
            # two-element swap, cyclic roll of a sub-block) are permutation-
            # preserving by construction -- verified over 80,000 random children
            # with zero failures -- so this repair is dead code and the
            # np.unique calls cost 6-9 s of every 1e5-iteration run. It is kept
            # only for the broken path, where it is the mechanism that turned
            # the defect into random restarts rather than a crash.
            if flip_broken:
                for i in range(4):
                    if len(np.unique(tmpPop[i, :])) != N:
                        tmpPop[i, :] = _perm(N)

            newPop[p:p+4, :] = tmpPop
        pop = newPop  # Update the population

    # Final optimized route, rotated to begin at xyz[0] (the UAV start, which
    # every caller prepends) with that point repeated at the end.
    if return_route:
        # Index-based rotation. The coordinate-matching version below does
        # np.where(... == xyz[0])[0][0], i.e. exact float equality on a triple;
        # nothing forbids a waypoint coinciding with a start point, and on a
        # collision it silently picks the wrong rotation (or raises IndexError
        # on [0][0] if no row matches).
        zero_at = int(np.flatnonzero(optRoute == 0)[0])
        idx = np.roll(optRoute, -zero_at)
        route_indices = np.concatenate([idx, idx[:1]])
        rearranged_route = xyz[route_indices, :]
    else:
        final_route = xyz[optRoute, :]
        start_point = np.where(np.all(final_route == xyz[0], axis=1))[0][0]
        rearranged_route = np.concatenate((final_route[start_point:], final_route[:start_point+1]))

    # Print GA Input and Output
    if verbose:
        print("\n--- GA Input ---")
        print("Waypoints:")
        print(xyz)
        print("\n--- GA Output ---")
        print("Optimized Route:")
        print(rearranged_route)

    if return_route:
        return GAResult(
            route_points=rearranged_route,
            route_indices=route_indices,
            opt_cost=float(globalMin),
            dist_history=distHistory,
            n_iter=numIter,
        )
    return rearranged_route
