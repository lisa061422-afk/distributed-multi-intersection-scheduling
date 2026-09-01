"""
Python equivalent of MAIN_Parallel_Compute.m — ADMM core loop.

Agent layout (0-indexed) is parameterized by ``const``:
  0 .. n_int-1                  : intersection agents (decision tree)
  n_int .. n_int+n_road-1       : road agents          (analytical)
  n_int+n_road  (= terminal_0)  : terminal agent       (delay cost)

Sizes (n_int / n_road / n_agents / terminal_id_0idx) MUST be present in
``const`` — see ``_agent_sizes``. Construction helpers populate them.

Parallel execution uses concurrent.futures.ProcessPoolExecutor.
Sequential fallback available via const['useParallel'] = False.

All arrays 0-indexed (n=0..N-1, kn=0).
"""

import os
import time
import copy
import numpy as np
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed

from .decision_tree import ini_admm_decision_tree
from .road_agent import update_road_agent
from .terminal_agent import update_terminal_agent


def _agent_sizes(const):
    """Return (n_int, n_road, n_agents, terminal_0) from const.

    All four keys must be present in ``const``; otherwise raises KeyError.
    Construction helpers (``generate_random_config``, ``load_config`` in
    ``run_validation.py``) populate these. Failing loud here prevents a class
    of bugs where a const dict built for a non-4-int topology silently runs
    with the old 4-int sizes.
    """
    required = ('n_int', 'n_road', 'n_agents', 'terminal_id_0idx')
    missing = [k for k in required if k not in const]
    if missing:
        raise KeyError(
            f'const is missing topology size keys: {missing}. '
            f'Build const via generate_random_config(...) or load_config(...) '
            f'so the sizes get populated, or set them explicitly.')
    return (const['n_int'], const['n_road'],
            const['n_agents'], const['terminal_id_0idx'])


# ===========================================================================
# Parallel worker — must be at module level for Windows spawn pickling
# ===========================================================================

_parallel_pool = None         # persistent pool across ADMM calls
_parallel_pool_root = None    # warehouse root used to create pool


def _worker_init(warehouse_root):
    """Set sys.path and pre-import modules in each worker process."""
    import sys
    if warehouse_root not in sys.path:
        sys.path.insert(0, warehouse_root)
    # Pre-import to avoid cold-import latency on first task
    import python.decision_tree  # noqa: F401


def _parallel_int_task(packed):
    """Run one intersection agent in a worker process."""
    (agent_i, entries, valid_systems,
     x_prev, y_prev, xi_bar, yi_bar,
     ai_x, ai_y, cache, const) = packed
    import time as _time
    from python.decision_tree import ini_admm_decision_tree as _dt
    t0 = _time.time()
    x_out, y_out, best_alpha, best_gamma, best_idx, _, cache_new = _dt(
        agent_i, entries, x_prev, y_prev, xi_bar, yi_bar,
        valid_systems, ai_x, ai_y, const, cache)
    return {
        'kind': 'intersection',
        'agent_i': agent_i,
        'valid_systems': valid_systems,
        'best_x': x_out,
        'best_y': y_out,
        'best_alpha': best_alpha,
        'best_gamma': best_gamma,
        'cache': cache_new,
        'elapsed': _time.time() - t0,
    }


def _noop_task(_):
    """Warm-up task: triggers module import in worker on first call."""
    import python.decision_tree  # noqa: F401
    return None


def _get_pool():
    """Return the module-level ProcessPoolExecutor, creating it on first call."""
    global _parallel_pool, _parallel_pool_root
    root = str(Path(__file__).parent.parent)
    if _parallel_pool is None or _parallel_pool_root != root:
        if _parallel_pool is not None:
            _parallel_pool.shutdown(wait=False)
        n_workers = min(4, os.cpu_count() or 4)
        _parallel_pool = ProcessPoolExecutor(
            max_workers=n_workers,
            initializer=_worker_init,
            initargs=(root,))
        _parallel_pool_root = root
    return _parallel_pool


def warmup_parallel_pool():
    """
    Pre-warm the ProcessPoolExecutor: spawn workers and pre-import modules.
    Call this once at MPC startup so the first run_admm_core call pays no
    pool-creation penalty.  Safe to call multiple times (no-op if already warm).
    """
    pool = _get_pool()
    list(pool.map(_noop_task, range(os.cpu_count() or 4)))


# ===========================================================================
# Initialisation
# ===========================================================================

def init_xy_prev(alpha_tilde, pathInfo_agent_chain, pathInfo_c,
                 Dt, N, perturb_scale=0.0,
                 n_agents=9, terminal_id_0idx=8):
    """
    Equivalent of MATLAB initEarliestXYPrev_fromChain.
    All 0-indexed.  kn=0 (one task per vehicle per agent).

    Parameters
    ----------
    n_agents          : total agent count (default 9 = 4 ints + 4 roads + terminal)
    terminal_id_0idx  : 0-indexed agent ID of the terminal (default 8)

    Returns x_prev, y_prev, x_prev_bar, y_prev_bar
      x_prev[agent][n] = np.array([value])  or None
    """
    n_nonterm = n_agents - 1   # number of non-terminal agents (ints + roads)
    x_prev = [[None] * N for _ in range(n_agents)]
    y_prev = [[None] * N for _ in range(n_nonterm)]   # no y for terminal

    for n in range(N):
        kn = 0
        chain = pathInfo_agent_chain[n][kn]   # list of agent IDs (0-indexed)
        cseq  = pathInfo_c[n][kn]             # intersection durations
        t_alpha = float(alpha_tilde[n][kn])

        delta = float(perturb_scale) * np.random.rand() if perturb_scale > 0 else 0.0

        passed_int_time  = 0.0
        passed_road_time = 0.0
        int_count = 0

        for posi, ag in enumerate(chain):
            if ag == terminal_id_0idx:
                t_terminal = t_alpha + passed_int_time + passed_road_time + delta
                x_prev[terminal_id_0idx][n] = np.array([t_terminal])
                continue

            if posi % 2 == 0:   # intersection (even positions: 0, 2, 4, ...)
                if int_count >= len(cseq):
                    raise ValueError(
                        f'Vehicle {n}: int_count={int_count} exceeds len(cseq)={len(cseq)}')
                t_arr = t_alpha + passed_int_time + passed_road_time + delta
                t_dep = t_arr + cseq[int_count]
                x_prev[ag][n] = np.array([t_arr])
                y_prev[ag][n] = np.array([t_dep])
                passed_int_time += cseq[int_count]
                int_count += 1
            else:               # road (odd positions: 1, 3, 5, ...)
                t_road_start = t_alpha + passed_int_time + passed_road_time + delta
                t_road_end   = t_road_start + Dt
                x_prev[ag][n] = np.array([t_road_start])
                y_prev[ag][n] = np.array([t_road_end])
                passed_road_time += Dt

    # bar copies
    x_prev_bar = [[x_prev[ag][n].copy() if x_prev[ag][n] is not None else None
                   for n in range(N)] for ag in range(n_agents)]
    y_prev_bar = [[y_prev[ag][n].copy() if y_prev[ag][n] is not None else None
                   for n in range(N)] for ag in range(n_nonterm)]

    return x_prev, y_prev, x_prev_bar, y_prev_bar


# ===========================================================================
# Residual computation
# ===========================================================================

def compute_r(x_prev, y_prev, r_local, const):
    """Equivalent of MATLAB compute_r."""
    pathInfo_agent_chain = const['pathInfo_agent_chain']
    N = const['N']
    kn = 0
    r_val = 0.0

    for n in range(N):
        chain = pathInfo_agent_chain[n][kn]
        for pos in range(1, len(chain)):
            prev_ag = chain[pos - 1]
            curr_ag = chain[pos]
            xc = x_prev[curr_ag][n]
            yp = y_prev[prev_ag][n]
            if xc is not None and yp is not None:
                r_val += float((xc[kn] - yp[kn]) ** 2)

    return r_val + r_local


# ===========================================================================
# Agent update helpers (sequential wrappers)
# ===========================================================================

def _agent_update_intersection(const, agent_i, entries, valid_systems,
                                x_prev, y_prev,
                                xi_prev_bar, yi_prev_bar,
                                ai_x, ai_y, cache_ai):
    """Run one intersection agent update. Returns result dict."""
    t0 = time.time()
    x_out, y_out, best_alpha, best_gamma, best_idx, NODES, cache_new = \
        ini_admm_decision_tree(
            agent_i, entries, x_prev, y_prev,
            xi_prev_bar, yi_prev_bar,
            valid_systems, ai_x, ai_y, const, cache_ai)
    return {
        'kind': 'intersection',
        'agent_i': agent_i,
        'valid_systems': valid_systems,
        'best_x': x_out,
        'best_y': y_out,
        'best_alpha': best_alpha,
        'best_gamma': best_gamma,
        'cache': cache_new,
        'elapsed': time.time() - t0,
    }


def _agent_update_road(const, agent_i, entries, valid_systems,
                       xi_prev, yi_prev, xi_prev_bar, yi_prev_bar,
                       ai_x, ai_y):
    """Run one road agent update."""
    t0 = time.time()
    x_road, y_road = update_road_agent(
        agent_i, entries, valid_systems,
        xi_prev, yi_prev, xi_prev_bar, yi_prev_bar,
        ai_x, ai_y, const)
    return {
        'kind': 'road',
        'agent_i': agent_i,
        'valid_systems': valid_systems,
        'x_road': x_road,
        'y_road': y_road,
        'elapsed': time.time() - t0,
    }


def _agent_update_terminal(const, x9_prev, x9_prev_bar, a9_x, terminal_id_0idx=8):
    """Run terminal agent update."""
    t0 = time.time()
    x9_new, delay_cost = update_terminal_agent(x9_prev, x9_prev_bar, a9_x, const)
    return {
        'kind': 'terminal',
        'agent_i': terminal_id_0idx,
        'x9_new': x9_new,
        'delay_cost': delay_cost,
        'elapsed': time.time() - t0,
    }


# ===========================================================================
# ADMM core loop
# ===========================================================================

def run_admm_core(const, agent_participation,
                  x_init=None, y_init=None, ax_init=None, ay_init=None):
    """
    Equivalent of MATLAB run_admm_core.

    Parameters
    ----------
    const               : dict  (all hyperparameters + config)
    agent_participation : list[9] of list[N]  entries per agent
    x_init              : list[9][N] or None  warm-start primal x
    y_init              : list[8][N] or None  warm-start primal y
    ax_init             : list[9][N] or None  hot-start dual a_x
    ay_init             : list[9][N] or None  hot-start dual a_y

    Returns
    -------
    x_prev, y_prev, local_tree_cache, residual_r, residual_s,
    delay_costs, k, T_admm, a_x, a_y
    """
    N        = const['N']
    max_iter = const['max_iter']
    tol_r    = const['tol_r']
    tol_s    = const['tol_s']
    rho1     = const['rho1']
    rho2     = const['rho2']
    use_parallel     = const.get('useParallel', False)
    use_adaptive_rho = const.get('use_adaptive_rho', False)
    verbose          = const.get('verbose', True)
    pathInfo_agent_chain = const['pathInfo_agent_chain']
    kn = 0

    # Topology sizes
    n_int, n_road, n_agents, terminal_0 = _agent_sizes(const)
    n_nonterm = n_int + n_road   # non-terminal agents

    # ── Initialisation ───────────────────────────────────────────────
    local_tree_cache = [None] * n_agents

    if x_init is not None and y_init is not None:
        # Warm-start: use provided primal variables; clamp to alpha_tilde
        x_prev = [[x_init[ag][n].copy() if x_init[ag][n] is not None else None
                   for n in range(N)] for ag in range(n_agents)]
        y_prev = [[y_init[ag][n].copy() if y_init[ag][n] is not None else None
                   for n in range(N)] for ag in range(n_nonterm)]
        # Clamp: x_prev[ag][n] >= alpha_tilde[n] (feasibility under updated alpha)
        for n in range(N):
            lb = float(const['alpha_tilde'][n][0])
            for ag in range(n_agents):
                if x_prev[ag][n] is not None:
                    x_prev[ag][n] = np.array([max(float(x_prev[ag][n][0]), lb)])
        x_prev_bar = [[x_prev[ag][n].copy() if x_prev[ag][n] is not None else None
                       for n in range(N)] for ag in range(n_agents)]
        y_prev_bar = [[y_prev[ag][n].copy() if y_prev[ag][n] is not None else None
                       for n in range(N)] for ag in range(n_nonterm)]
    else:
        x_prev, y_prev, x_prev_bar, y_prev_bar = init_xy_prev(
            const['alpha_tilde'], pathInfo_agent_chain,
            const['pathInfo_c'], const['Dt'], N,
            const.get('randInitScale', 0.0),
            n_agents=n_agents, terminal_id_0idx=terminal_0)

    # Dual variables: carry from previous step (hot-start) or reset to zero
    if ax_init is not None and ay_init is not None:
        a_x = [[ax_init[ag][n].copy() if ax_init[ag][n] is not None else np.array([0.0])
                for n in range(N)] for ag in range(n_agents)]
        a_y = [[ay_init[ag][n].copy() if ay_init[ag][n] is not None else np.array([0.0])
                for n in range(N)] for ag in range(n_agents)]
    else:
        a_x = [[np.array([0.0]) for n in range(N)] for _ in range(n_agents)]
        a_y = [[np.array([0.0]) for n in range(N)] for _ in range(n_agents)]
    a_x_new = copy.deepcopy(a_x)
    a_y_new = copy.deepcopy(a_y)

    residual_r  = np.zeros(max_iter)
    residual_s  = np.zeros(max_iter)
    delay_costs = np.zeros(max_iter)

    T0 = time.time()

    # Boyd 2011 §3.4.3 over/under-relaxation factor (1.0 = standard ADMM).
    # α<1 (under-relaxation) damps oscillation on non-convex problems;
    # α∈(1,2] (over-relaxation) accelerates convergence on convex problems.
    alpha_relax = float(const.get('alpha_relax', 1.0))
    do_relax    = (alpha_relax != 1.0)

    for k in range(max_iter):
        t_iter = time.time()
        x_last = copy.deepcopy(x_prev)
        y_last = copy.deepcopy(y_prev)

        # Snapshot consensus targets z^k before Step 1 mutates them — needed
        # for relaxed primal x_relaxed = α·x + (1-α)·z^k (Boyd Eq. 3.21).
        if do_relax:
            x_bar_prev = copy.deepcopy(x_prev_bar)
            y_bar_prev = copy.deepcopy(y_prev_bar)

        # ── Step 1: dual variable update ────────────────────────────
        for n in range(N):
            chain = pathInfo_agent_chain[n][kn]

            # First agent in chain: x_bar = x/2
            ag0 = chain[0]
            if x_prev_bar[ag0][n] is not None:
                x_prev_bar[ag0][n] = np.array(
                    [(float(x_prev[ag0][n][kn]) + 0.0) / 2.0])

            for pos in range(1, len(chain)):
                prev_ag = chain[pos - 1]
                curr_ag = chain[pos]

                # Skip terminal for y update (terminal has no y variable)
                xc = x_prev[curr_ag][n]
                yp = y_prev[prev_ag][n] if prev_ag != terminal_0 else None

                if xc is None or yp is None:
                    continue

                # Boyd over-relaxation: use α·primal + (1-α)·z^k in dual /
                # consensus updates. α=1 reduces to standard ADMM exactly.
                if do_relax:
                    xb_old = x_bar_prev[curr_ag][n]
                    yb_old = (y_bar_prev[prev_ag][n]
                              if prev_ag != terminal_0 else None)
                    xc_v = (alpha_relax * float(xc[kn])
                            + (1 - alpha_relax) * float(xb_old[kn])
                            if xb_old is not None else float(xc[kn]))
                    yp_v = (alpha_relax * float(yp[kn])
                            + (1 - alpha_relax) * float(yb_old[kn])
                            if yb_old is not None else float(yp[kn]))
                else:
                    xc_v = float(xc[kn])
                    yp_v = float(yp[kn])

                a_x_new[curr_ag][n] = np.array([
                    float(a_x[curr_ag][n][kn])
                    + rho1 * (xc_v - yp_v)])
                x_prev_bar[curr_ag][n] = np.array([
                    (xc_v + yp_v) / 2.0])

                a_y_new[prev_ag][n] = np.array([
                    float(a_y[prev_ag][n][kn])
                    + rho1 * (yp_v - xc_v)])
                if prev_ag != terminal_0:
                    y_prev_bar[prev_ag][n] = np.array([
                        (yp_v + xc_v) / 2.0])

        # ── Step 2: local agent updates ──────────────────────────────
        meta    = [None] * n_agents
        futures = {}   # future → agent_i

        # Intersection agents 0..n_int-1: dispatch to pool or run inline
        for agent_i in range(n_int):
            entries = agent_participation[agent_i]
            valid_systems = [n for n in range(N) if entries[n]]
            if not valid_systems:
                meta[agent_i] = {'kind': 'intersection', 'agent_i': agent_i,
                                 'valid_systems': [], 'best_x': np.zeros(N),
                                 'best_y': np.zeros(N), 'best_alpha': [None]*N,
                                 'best_gamma': [None]*N, 'cache': None,
                                 'elapsed': 0.0}
            elif use_parallel:
                packed = (agent_i, entries, valid_systems,
                          x_prev, y_prev,
                          x_prev_bar[agent_i], y_prev_bar[agent_i],
                          a_x_new[agent_i], a_y_new[agent_i],
                          local_tree_cache[agent_i], const)
                futures[_get_pool().submit(_parallel_int_task, packed)] = agent_i
            else:
                if verbose and k % 10 == 0:
                    print(f'  [k={k+1}] agent {agent_i} starting...')
                meta[agent_i] = _agent_update_intersection(
                    const, agent_i, entries, valid_systems,
                    x_prev, y_prev,
                    x_prev_bar[agent_i], y_prev_bar[agent_i],
                    a_x_new[agent_i], a_y_new[agent_i],
                    local_tree_cache[agent_i])

        # Road agents n_int..n_nonterm-1: always sequential (analytical, < 1ms each)
        for agent_i in range(n_int, n_nonterm):
            entries = agent_participation[agent_i]
            valid_systems = [n for n in range(N) if entries[n]]
            if not valid_systems:
                meta[agent_i] = {'kind': 'road', 'agent_i': agent_i,
                                 'valid_systems': [],
                                 'x_road': np.zeros(N), 'y_road': np.zeros(N),
                                 'elapsed': 0.0}
            else:
                meta[agent_i] = _agent_update_road(
                    const, agent_i, entries, valid_systems,
                    x_prev[agent_i], y_prev[agent_i],
                    x_prev_bar[agent_i], y_prev_bar[agent_i],
                    a_x_new[agent_i], a_y_new[agent_i])

        # Terminal agent: always sequential
        meta[terminal_0] = _agent_update_terminal(
            const, x_prev[terminal_0], x_prev_bar[terminal_0], a_x_new[terminal_0],
            terminal_id_0idx=terminal_0)

        # Collect parallel intersection results
        for fut in as_completed(futures):
            meta[futures[fut]] = fut.result()

        # ── Merge results ────────────────────────────────────────────
        r_local = 0.0
        for agent_i in range(n_agents):
            res = meta[agent_i]
            if verbose and k % 10 == 0:
                print(f'  [k={k+1}] Agent {agent_i} time={res.get("elapsed",0):.3f}s')

            if res['kind'] == 'intersection':
                local_tree_cache[agent_i] = res['cache']
                for n in res['valid_systems']:
                    x_prev[agent_i][n] = np.array([res['best_x'][n]])
                    y_prev[agent_i][n] = np.array([res['best_y'][n]])
                    ba = res['best_alpha'][n]
                    bg = res['best_gamma'][n]
                    if ba is not None and bg is not None:
                        xv = res['best_x'][n]
                        yv = res['best_y'][n]
                        r_local += (float(xv) - float(ba)) ** 2
                        r_local += (float(yv) - float(bg)) ** 2

            elif res['kind'] == 'road':
                for n in res['valid_systems']:
                    x_prev[agent_i][n] = np.array([res['x_road'][n]])
                    y_prev[agent_i][n] = np.array([res['y_road'][n]])

            elif res['kind'] == 'terminal':
                delay_costs[k] = res['delay_cost']
                for n in range(N):
                    x_prev[terminal_0][n] = np.array([res['x9_new'][n]])

        # ── NaN guard ────────────────────────────────────────────────
        for agent_i in range(n_agents):
            for n in range(N):
                xc = x_prev[agent_i][n]
                xl = x_last[agent_i][n]
                if (xc is not None and xl is not None
                        and np.any(np.isnan(xc)) and not np.any(np.isnan(xl))):
                    print(f'[NaN] k={k+1} agent={agent_i} n={n} x_prev replaced')
                    x_prev[agent_i][n] = xl.copy()
                if agent_i != terminal_0:
                    yc = y_prev[agent_i][n]
                    yl = y_last[agent_i][n]
                    if (yc is not None and yl is not None
                            and np.any(np.isnan(yc)) and not np.any(np.isnan(yl))):
                        print(f'[NaN] k={k+1} agent={agent_i} n={n} y_prev replaced')
                        y_prev[agent_i][n] = yl.copy()

        # ── Step 3: residuals ────────────────────────────────────────
        r = compute_r(x_prev, y_prev, r_local, const)
        s = 0.0
        for agent_i in range(n_nonterm):
            entries = agent_participation[agent_i]
            vs = [n for n in range(N) if entries[n]]
            for n in vs:
                xc = x_prev[agent_i][n]; xl = x_last[agent_i][n]
                yc = y_prev[agent_i][n]; yl = y_last[agent_i][n]
                if xc is not None and xl is not None:
                    s += float(np.sum((xc - xl) ** 2))
                if yc is not None and yl is not None:
                    s += float(np.sum((yc - yl) ** 2))
        for n in range(N):
            xc = x_prev[terminal_0][n]; xl = x_last[terminal_0][n]
            if xc is not None and xl is not None:
                s += float(np.sum((xc - xl) ** 2))

        residual_r[k] = r
        residual_s[k] = s
        a_x = a_x_new
        a_y = a_y_new
        a_x_new = copy.deepcopy(a_x)
        a_y_new = copy.deepcopy(a_y)

        # ── Adaptive ρ (Boyd 2011) ────────────────────────────────────
        if use_adaptive_rho:
            mu = 10.0; tau = 2.0
            if r > mu * s:
                scale = tau
            elif s > mu * r:
                scale = 1.0 / tau
            else:
                scale = 1.0
            if scale != 1.0:
                ratio = 1.0 / scale
                for ag in range(n_agents):
                    for n in range(N):
                        a_x[ag][n] = a_x[ag][n] * ratio
                        if ag != terminal_0:
                            a_y[ag][n] = a_y[ag][n] * ratio
                rho1 *= scale
                rho2 *= scale
                const = {**const, 'rho1': rho1, 'rho2': rho2}
                if verbose:
                    print(f'[AdapRho] k={k+1}  r/s={r/max(s,1e-9):.1f}  rho→{rho1:.3f}')

        iter_elapsed = time.time() - t_iter
        if verbose:
            print(f'[Iter {k+1}] r={r:.4f}  s={s:.4f}  time={iter_elapsed:.2f}s')

        first_iter_timeout = const.get('first_iter_timeout', None)
        if k == 0 and first_iter_timeout is not None and iter_elapsed > first_iter_timeout:
            print(f'[Early exit] iter 1 took {iter_elapsed:.1f}s > '
                  f'{first_iter_timeout}s threshold — abandoning seed.')
            residual_r  = residual_r[:k + 1]
            residual_s  = residual_s[:k + 1]
            delay_costs = delay_costs[:k + 1]
            k += 1
            break

        if r < tol_r and s < tol_s:
            if verbose:
                print(f'Converged at iteration {k+1}')
            residual_r  = residual_r[:k + 1]
            residual_s  = residual_s[:k + 1]
            delay_costs = delay_costs[:k + 1]
            k += 1
            break
    else:
        # Loop completed without convergence: align k with array length.
        k = max_iter

    T_admm = time.time() - T0
    if verbose:
        print(f'ADMM elapsed {T_admm:.1f} s')
    return (x_prev, y_prev, local_tree_cache,
            residual_r, residual_s, delay_costs, k, T_admm,
            a_x, a_y)
