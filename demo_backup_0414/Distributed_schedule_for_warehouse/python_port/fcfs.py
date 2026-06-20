"""
Centralized FCFS baseline using the proper branch-and-bound solver from
``python_port/centralized/fcfs_solver.py`` (ported from MATLAB
``expand_array_global2.m`` lines 111-251).

The centralized solver expects a different ``const`` shape than this
project's distributed C-ADMM uses, so this module provides an adapter that
re-packs ``const`` + ``agent_participation`` into the centralized format,
runs ``search_fcfs``, and pulls per-vehicle gamma/beta out of the leaf
node for plotting and downstream export.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from python_port.centralized.fcfs_solver import search_fcfs
from python_port.centralized.search import search_dfs_bb
from python_port.centralized.node import Node
from python_port.centralized.vehicle import Vehicle, TaskProfile  # noqa  (re-exported via shim below)


# ───────────────────────────────────────────────────────────────────────
# Vehicle / TaskProfile shim — centralized solver imports them via
# ``from .vehicle import ...`` inside its own helpers; we mirror the
# structures here so the solver runs against this package without us
# needing to copy vehicle.py wholesale.
# ───────────────────────────────────────────────────────────────────────


@dataclass
class FcfsResult:
    alpha: List[List[float]]
    beta:  List[List[float]]
    gamma: List[List[float]]
    delays: List[float]
    total_delay: float
    t_term: List[float]
    int_schedule: List[List[Dict]]
    elapsed: float
    nodes_explored: int


# ───────────────────────────────────────────────────────────────────────
# Adapter: distributed const → centralized const + root Node
# ───────────────────────────────────────────────────────────────────────

def _build_centralized_inputs(const, ap, Smax: int = 3):
    """Convert distributed-ADMM const into centralized-FCFS const + root Node.

    Returns (cent_const, root, int_seq_0idx_per_vehicle, int_durs_per_vehicle).
    """
    N        = const['N']
    Dt       = float(const['Dt'])
    n_int    = const['n_int']
    alpha_tilde = const['alpha_tilde']                # list[N] of np.ndarray
    chains   = [const['pathInfo_agent_chain'][n][0] for n in range(N)]
    pinfo_c  = [const['pathInfo_c'][n][0] for n in range(N)]
    pathInfo = const.get('pathInfo')
    IntSpaceDB = const['IntSpaceDB']

    # Per-int "space_per_int" — pull from first int_db entry. The
    # centralized framework assumes uniform space_per_int across ints.
    space_per_int = int(IntSpaceDB[0]['numSpaces'])
    Mtot = n_int * space_per_int

    # Build per-vehicle int_seq (1-indexed) and route_id (1-indexed)
    int_seq_per_n: List[List[int]]   = []
    int_seq0_per_n: List[List[int]]  = []   # 0-indexed for plotting
    int_durs_per_n: List[List[float]] = []
    route_ids_per_n: List[List[int]] = []
    NI = []
    alpha0_per_n = []
    for n in range(N):
        ch = chains[n]
        int_chain_0 = [a for a in ch[:-1] if a < n_int]
        int_chain_1 = [a + 1 for a in int_chain_0]
        int_seq_per_n.append(int_chain_1)
        int_seq0_per_n.append(int_chain_0)
        int_durs_per_n.append(list(pinfo_c[n]))
        if pathInfo is not None and 'routeId' in pathInfo[n]:
            route_ids_per_n.append([int(r) for r in pathInfo[n]['routeId']])
        else:
            route_ids_per_n.append([1] * len(int_chain_1))
        NI.append(len(int_chain_1))
        alpha0_per_n.append(float(alpha_tilde[n][0]))

    # Build task_cache[n][ti] = TaskProfile
    task_cache: List[List[TaskProfile]] = []
    for n in range(N):
        per_v: List[TaskProfile] = []
        for ti in range(NI[n]):
            int_id_1idx   = int_seq_per_n[n][ti]
            route_id_1idx = route_ids_per_n[n][ti]
            int_db = IntSpaceDB[int_id_1idx - 1]
            # In python_port, routeSpace/routeDur are dicts keyed by
            # 1-indexed route id (see topology._four_way_int_db).
            rs = int_db['routeSpace']
            rd = int_db['routeDur']
            local_spaces = list(rs[route_id_1idx] if isinstance(rs, dict)
                                else rs[route_id_1idx - 1])
            local_durs   = list(rd[route_id_1idx] if isinstance(rd, dict)
                                else rd[route_id_1idx - 1])
            global_spaces = [int(s) + (int_id_1idx - 1) * space_per_int
                             for s in local_spaces]
            L = len(local_durs)
            if L > Smax:
                raise ValueError(
                    f'vehicle {n} task {ti}: route uses {L} spaces but Smax={Smax}')
            C   = np.zeros(Smax, dtype=float)
            Map = np.zeros(Smax, dtype=int)
            C[:L]   = local_durs
            Map[:L] = global_spaces
            per_v.append(TaskProfile(C=C, Map=Map,
                                      int_id=int_id_1idx, route_id=route_id_1idx,
                                      L=L))
        task_cache.append(per_v)

    NI_arr = np.array(NI, dtype=int)
    cent_const = {
        'N':             N,
        'NI':            NI_arr,
        'Smax':          Smax,
        'num_int':       n_int,
        'space_per_int': space_per_int,
        'Mtot':          Mtot,
        'Dt':            Dt,
        'task_cache':    task_cache,
        'use_weak_rule': False,   # FCFS doesn't branch on contention; weak rule moot
        'priority_n':    0,
    }

    # Root Node
    d0 = np.array(alpha0_per_n, dtype=float)
    r0 = np.zeros((Smax, N))
    o0 = np.zeros(N)
    ni0 = np.zeros(N, dtype=int)
    tw0 = 0.0
    U_c0 = np.zeros((N, Mtot), dtype=int)
    U0   = np.zeros((N, Mtot), dtype=int)
    g0 = 0.0
    f0 = 0.0
    gamma0 = [np.full(NI[n], np.nan) for n in range(N)]
    alpha0 = [np.full(NI[n], np.nan) for n in range(N)]
    x0     = [[[] for _ in range(NI[n])] for n in range(N)]
    speed0 = [0.0 for _ in range(N)]
    ra_reset0 = np.zeros((Smax, N))
    pair_lock0   = -1 * np.ones((N, N), dtype=int)
    reset_since0 = np.zeros(N)

    root = Node(
        idx=0,
        d=d0, r=r0, o=o0, tw=tw0, ni=ni0, parent=-1,
        U_c=U_c0, U_temp=U0, g=g0, gamma=gamma0, f=f0, speed=speed0,
        ra_reset=ra_reset0, x=x0, alpha=alpha0,
        pair_lock=pair_lock0, reset_since=reset_since0,
    )
    return cent_const, root, int_seq0_per_n, int_durs_per_n


# ───────────────────────────────────────────────────────────────────────
# Public entry point
# ───────────────────────────────────────────────────────────────────────

def run_fcfs(const, agent_participation,
             deadline_s: float | None = 60.0,
             verbose: bool = False) -> FcfsResult:
    """Run centralized FCFS. Returns FcfsResult with gamma/beta per vehicle."""
    t0 = time.time()
    cent_const, root, int_chain0, int_durs = _build_centralized_inputs(
        const, agent_participation)

    nodes, leaves, best_idx, best_g, _ = search_fcfs(
        root, cent_const, deadline=deadline_s, verbose=verbose)

    if best_idx < 0:
        raise RuntimeError(
            f'FCFS search did not produce a leaf (deadline {deadline_s}s hit).')

    # ── Extract per-vehicle gamma/beta from the leaf node ──────────────
    leaf = nodes[best_idx]
    gamma_pv: List[List[float]] = []
    beta_pv:  List[List[float]] = []
    alpha_pv: List[List[float]] = []
    for n in range(const['N']):
        g_arr = leaf.gamma[n]
        a_arr = leaf.alpha[n]
        if g_arr is None or a_arr is None:
            raise RuntimeError(f'FCFS leaf missing gamma/alpha for vehicle {n}')
        durs = int_durs[n]
        gamma_pv.append([float(x) for x in g_arr])
        beta_pv.append([float(g - d) for g, d in zip(g_arr, durs)])
        alpha_pv.append([float(a) for a in a_arr])

    # ── Per-vehicle terminal time + delay ──────────────────────────────
    Dt = float(const['Dt'])
    deadline = const['deadline']
    chains = [const['pathInfo_agent_chain'][n][0] for n in range(const['N'])]
    n_int = const['n_int']
    t_term = [0.0]*const['N']
    delays = [0.0]*const['N']
    for n in range(const['N']):
        chain = chains[n]
        ints  = [a for a in chain[:-1] if a < n_int]
        n_total_agents = len(chain) - 1   # exclude terminal
        n_roads = n_total_agents - len(ints)
        # trailing road count (after last int): n_roads - (len(ints)-1) if ints>=1 else 0
        trailing = max(0, n_roads - max(0, len(ints) - 1))
        if not ints:
            t_term[n] = float(const['alpha_tilde'][n][0])
        else:
            t_term[n] = gamma_pv[n][-1] + Dt * trailing
        delays[n] = max(0.0, t_term[n] - float(deadline[n][0]))

    # ── Per-int schedule for plotting ──────────────────────────────────
    int_schedule: List[List[Dict]] = [[] for _ in range(n_int)]
    for n in range(const['N']):
        for k, ag in enumerate(int_chain0[n]):
            if k >= len(beta_pv[n]):
                continue
            int_schedule[ag].append({
                'n':     n,
                'alpha': alpha_pv[n][k],
                'beta':  beta_pv[n][k],
                'gamma': gamma_pv[n][k],
                'k':     k,
            })
    for ag in range(n_int):
        int_schedule[ag].sort(key=lambda d: d['beta'])

    return FcfsResult(
        alpha=alpha_pv, beta=beta_pv, gamma=gamma_pv,
        delays=delays, total_delay=float(sum(delays)), t_term=t_term,
        int_schedule=int_schedule,
        elapsed=time.time() - t0,
        nodes_explored=len(nodes),
    )


# ───────────────────────────────────────────────────────────────────────
# Plotting (unchanged from the naive version)
# ───────────────────────────────────────────────────────────────────────


def _make_colors(N):
    import matplotlib.pyplot as plt
    cmap = plt.get_cmap('tab20', max(N, 20))
    return [cmap(i % cmap.N) for i in range(N)]


class _FakeNode:
    """Per-int adapter so _local_panel can read gamma like an ADMM cache."""
    __slots__ = ('gamma',)
    def __init__(self, gamma):
        self.gamma = gamma   # list[N] of scalar (NaN for vehicles not at this int)


def plot_fcfs_local_panel(const, result: FcfsResult, title_prefix: str = 'FCFS'):
    """REUSE the optimal-side renderer (`run_results._local_panel`) so the
    FCFS plot is visually identical to optimal's fig5. Per-int we synthesize
    a fake-node whose gamma[n] is vehicle n's exit time at that int."""
    import math
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
    from python_port.run_results import _local_panel

    N      = const['N']
    n_int  = const['n_int']
    chains = [const['pathInfo_agent_chain'][n][0] for n in range(N)]

    # _local_panel reads `x_prev[prev_road_agent][n][0] + Dt` as genTime
    # for non-first ints. So x_prev must be indexed by ALL agents (incl
    # road agents), with road_agent[n] = γ_prev_int (vehicle exit time of
    # the previous int). Then prev_road + Dt gives arrival at curr int.
    n_agents = const['n_agents']
    x_prev = [[None]*N for _ in range(n_agents)]
    for n in range(N):
        chain = chains[n]
        # Walk through the chain and at each int (after the first) set the
        # previous road's x_prev so genTime=γ_prev_int+Dt = arrival time.
        int_count = 0
        for posi, ag in enumerate(chain[:-1]):
            if ag < n_int:
                if int_count >= 1 and posi >= 1:
                    prev_road = chain[posi - 1]
                    if int_count - 1 < len(result.gamma[n]):
                        x_prev[prev_road][n] = [float(result.gamma[n][int_count - 1])]
                int_count += 1

    rows = max(1, int(math.ceil(math.sqrt(n_int))))
    cols = max(1, int(math.ceil(n_int / rows)))
    fig = plt.figure(figsize=(7.5 * cols, 5.0 * rows), facecolor='white')
    fig.suptitle(f'{title_prefix} Local Schedules (Per Intersection)',
                 fontsize=14, y=0.985)
    gs = GridSpec(rows, cols, figure=fig,
                  wspace=0.18, hspace=0.22,
                  left=0.06, right=0.97, top=0.93, bottom=0.06)

    for ag in range(n_int):
        gamma_vec = [float('nan')] * N
        valid_systems = []
        for n in range(N):
            ints = [a for a in chains[n][:-1] if a < n_int]
            if ag in ints:
                k = ints.index(ag)
                if k < len(result.gamma[n]):
                    gamma_vec[n] = float(result.gamma[n][k])
                    valid_systems.append(n)

        if not valid_systems:
            ax = fig.add_subplot(gs[ag // cols, ag % cols])
            ax.text(0.5, 0.5, f'Intersection {ag + 1}: no traffic',
                    ha='center', va='center', fontsize=12)
            ax.set_xticks([]); ax.set_yticks([])
            continue

        node = _FakeNode(gamma_vec)
        _local_panel(fig, gs[ag // cols, ag % cols],
                     NODES=[node], path_nodes=[0],
                     agent_i=ag, valid_systems=valid_systems, const=const,
                     x_prev=x_prev, Dt=const['Dt'],
                     title=f'Intersection {ag + 1}')
    return fig


def run_optimal_bnb(const, agent_participation,
                    deadline_s: float | None = 300.0,
                    verbose: bool = False,
                    use_fcfs_bound: bool = True) -> FcfsResult:
    """Run centralized DFS branch-and-bound for the TRUE optimal schedule.

    Same return shape as run_fcfs (FcfsResult) so the export pipeline can
    treat them uniformly. Uses search_dfs_bb which explores the full tree
    with bounding pruning — guaranteed optimal cost (no ADMM convergence
    issues).

    use_fcfs_bound: when True (default), runs FCFS first and feeds its
    cost as the initial best_g for BnB → much more aggressive pruning,
    typically completes in a fraction of the time vs ∞ initial bound.
    """
    t0 = time.time()
    cent_const, root, int_chain0, int_durs = _build_centralized_inputs(
        const, agent_participation)

    initial_best_g = float('inf')
    if use_fcfs_bound:
        from python_port.centralized.fcfs_solver import search_fcfs
        # Need a separate root since search_fcfs mutates node fields.
        import copy as _copy
        root_for_fcfs = _copy.deepcopy(root)
        f_nodes, _, f_idx, f_g, _ = search_fcfs(
            root_for_fcfs, cent_const, deadline=30.0, verbose=False)
        if f_idx >= 0 and f_g < initial_best_g:
            initial_best_g = f_g

    t_start = time.time()
    nodes, leaves, best_idx, best_g, log = search_dfs_bb(
        root, cent_const, deadline=deadline_s, verbose=verbose,
        initial_best_g=initial_best_g)
    bnb_elapsed = time.time() - t_start
    deadline_hit = (deadline_s is not None and bnb_elapsed >= deadline_s * 0.95)

    if deadline_hit:
        # Strict policy: timeout = no valid optimal result. Caller must lower
        # N or extend deadline.
        raise TimeoutError(
            f'Optimal BnB exceeded deadline {deadline_s}s '
            f'(elapsed={bnb_elapsed:.1f}s, expanded={len(nodes)} nodes, '
            f'best_g_found={best_g:.4f}). '
            f'Optimality not proven; rejecting partial result.')

    if best_idx < 0:
        raise RuntimeError(
            f'Optimal BnB found no leaf (expanded {len(nodes)} nodes).')

    leaf = nodes[best_idx]
    gamma_pv: List[List[float]] = []
    beta_pv:  List[List[float]] = []
    alpha_pv: List[List[float]] = []
    for n in range(const['N']):
        g_arr = leaf.gamma[n]
        a_arr = leaf.alpha[n]
        if g_arr is None or a_arr is None:
            raise RuntimeError(f'Optimal BnB leaf missing gamma/alpha for vehicle {n}')
        durs = int_durs[n]
        gamma_pv.append([float(x) for x in g_arr])
        beta_pv.append([float(g - d) for g, d in zip(g_arr, durs)])
        alpha_pv.append([float(a) for a in a_arr])

    Dt = float(const['Dt'])
    deadline = const['deadline']
    chains = [const['pathInfo_agent_chain'][n][0] for n in range(const['N'])]
    n_int = const['n_int']
    t_term = [0.0]*const['N']
    delays = [0.0]*const['N']
    for n in range(const['N']):
        chain = chains[n]
        ints  = [a for a in chain[:-1] if a < n_int]
        n_total_agents = len(chain) - 1
        n_roads = n_total_agents - len(ints)
        trailing = max(0, n_roads - max(0, len(ints) - 1))
        if not ints:
            t_term[n] = float(const['alpha_tilde'][n][0])
        else:
            t_term[n] = gamma_pv[n][-1] + Dt * trailing
        delays[n] = max(0.0, t_term[n] - float(deadline[n][0]))

    int_schedule: List[List[Dict]] = [[] for _ in range(n_int)]
    for n in range(const['N']):
        for k, ag in enumerate(int_chain0[n]):
            if k >= len(beta_pv[n]):
                continue
            int_schedule[ag].append({
                'n':     n,
                'alpha': alpha_pv[n][k],
                'beta':  beta_pv[n][k],
                'gamma': gamma_pv[n][k],
                'k':     k,
            })
    for ag in range(n_int):
        int_schedule[ag].sort(key=lambda d: d['beta'])

    return FcfsResult(
        alpha=alpha_pv, beta=beta_pv, gamma=gamma_pv,
        delays=delays, total_delay=float(sum(delays)), t_term=t_term,
        int_schedule=int_schedule,
        elapsed=time.time() - t0,
        nodes_explored=len(nodes),
    )


def build_result_from_admm(const, x_prev, y_prev) -> FcfsResult:
    """Wrap ADMM's converged x_prev (β at each int) / y_prev (γ at each int)
    into the FcfsResult shape so it can be plotted with `plot_fcfs_local_panel`
    (same renderer as FCFS, ensures visual parity)."""
    N        = const['N']
    n_int    = const['n_int']
    Dt       = float(const['Dt'])
    deadline = const['deadline']
    chains   = [const['pathInfo_agent_chain'][n][0] for n in range(N)]

    alpha_pv: List[List[float]] = []
    beta_pv:  List[List[float]] = []
    gamma_pv: List[List[float]] = []
    for n in range(N):
        ints = [a for a in chains[n][:-1] if a < n_int]
        a_list, b_list, g_list = [], [], []
        prev_gamma = None
        for k, ag in enumerate(ints):
            if k == 0:
                alpha_k = float(const['alpha_tilde'][n][0])
            else:
                alpha_k = (prev_gamma + Dt) if prev_gamma is not None else float('nan')
            beta_k  = float(x_prev[ag][n][0]) if x_prev[ag][n] is not None else float('nan')
            gamma_k = float(y_prev[ag][n][0]) if y_prev[ag][n] is not None else float('nan')
            a_list.append(alpha_k); b_list.append(beta_k); g_list.append(gamma_k)
            prev_gamma = gamma_k
        alpha_pv.append(a_list); beta_pv.append(b_list); gamma_pv.append(g_list)

    t_term = [0.0] * N
    delays = [0.0] * N
    for n in range(N):
        ints = [a for a in chains[n][:-1] if a < n_int]
        n_total_agents = len(chains[n]) - 1
        n_roads = n_total_agents - len(ints)
        trailing = max(0, n_roads - max(0, len(ints) - 1))
        t_term[n] = (gamma_pv[n][-1] + Dt * trailing) if gamma_pv[n] else float('nan')
        delays[n] = max(0.0, t_term[n] - float(deadline[n][0]))

    int_schedule: List[List[Dict]] = [[] for _ in range(n_int)]
    for n in range(N):
        ints = [a for a in chains[n][:-1] if a < n_int]
        for k, ag in enumerate(ints):
            int_schedule[ag].append({
                'n': n, 'alpha': alpha_pv[n][k],
                'beta':  beta_pv[n][k], 'gamma': gamma_pv[n][k], 'k': k,
            })
    for ag in range(n_int):
        int_schedule[ag].sort(key=lambda d: d['beta'])

    return FcfsResult(
        alpha=alpha_pv, beta=beta_pv, gamma=gamma_pv,
        delays=delays, total_delay=float(sum(delays)), t_term=t_term,
        int_schedule=int_schedule,
        elapsed=0.0, nodes_explored=0,
    )


def print_optimal_summary(const, result: FcfsResult):
    print(f'\n=== Optimal (centralized DFS-BnB) ===')
    print(f'  search elapsed = {result.elapsed:.3f}s')
    print(f'  nodes explored = {result.nodes_explored}')
    print(f'  total delay    = {result.total_delay:.4f}')


def print_fcfs_summary(const, result: FcfsResult):
    print(f'\n=== FCFS baseline (centralized BnB) ===')
    print(f'  search elapsed = {result.elapsed:.3f}s')
    print(f'  nodes explored = {result.nodes_explored}')
    print(f'  total delay    = {result.total_delay:.4f}')
    print('\nPer-vehicle terminal time vs deadline (FCFS):')
    deadline = const['deadline']
    for n in range(const['N']):
        ddl = float(deadline[n][0])
        t   = result.t_term[n]
        d   = result.delays[n]
        print(f'  n={n}: t_term={t:7.3f}  ddl={ddl:7.3f}  delay={d:6.3f}')
