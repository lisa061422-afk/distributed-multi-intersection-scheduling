"""
Random scenario config generator — Python port of:
  generateTrafficSystem.m, makeIntSpaceDB.m, buildIntSpaceDB.m,
  getRoadAgent.m, export_config_for_python.m

All internal indexing follows MATLAB convention (1-indexed agents,
1-indexed route labels) so the generated JSON is byte-compatible with
validation_config.json and load_config() needs no changes.

The network spec (route_dict, road agents, IntSpaceDB) is supplied by a
``Topology`` object — see topology.py. Defaults to ``topology.DEFAULT``
(four_int) so existing callers keep working.
"""

import json
import numpy as np
from pathlib import Path

from python import topology as _topo


# Legacy alias for callers that imported get_road_agent from this module.
def get_road_agent(i, j, topology=None):
    """Return 1-indexed road agent ID between 1-indexed intersections i and j."""
    t = topology if topology is not None else _topo.DEFAULT
    return t.get_road_agent(i, j)


# ===========================================================================
# Random config generator
# ===========================================================================

from . import defaults as _D

def generate_random_config(N, seed=None, max_per_int=None,
                            T_val=_D.T_VAL, T_ent=_D.T_ENT, Dt=_D.DT,
                            v_max=_D.V_MAX, detect_range=_D.DETECT_RANGE,
                            rho1=_D.RHO1, rho2=_D.RHO2, weight=_D.WEIGHT,
                            max_iter=_D.MAX_ITER, tol_r=_D.TOL_R, tol_s=_D.TOL_S,
                            topology=None):
    """
    Generate a random N-vehicle scenario.

    Parameters
    ----------
    N            : int    number of vehicles (e.g. 10-15)
    seed         : int    RNG seed (None = random)
    max_per_int  : int    max vehicles per intersection
    T_val        : float  headway between vehicles at same entrance (s)
    T_ent        : float  stagger per entrance rank (s)
    Dt           : float  road traversal time (s)
    v_max        : float  vehicle approach speed (m/s)
    detect_range : float  detection zone diameter (m)
    topology     : Topology, optional. Defaults to topology.DEFAULT (four_int).
    ...          : ADMM hyperparameters

    Returns
    -------
    const              : dict  (same format as load_config output)
    agent_participation: list[n_agents][N]
    """
    t = topology if topology is not None else _topo.DEFAULT
    route_dict   = t.route_dict
    int_db_list  = t.int_db
    n_int        = t.n_int
    n_agents     = t.n_agents
    terminal_1   = t.terminal_id_1idx
    W_phys       = 1.6   # merging zone width (m), used for base_time

    rng = np.random.default_rng(seed)

    # ── Sample vehicles ────────────────────────────────────────────────
    # 1-indexed; index 0 unused. Length n_int+1 so int_load[i] for i in 1..n_int.
    int_load = np.zeros(n_int + 1, dtype=int)
    entrance_rank = {}   # entrance → next entryIndex (0-based)
    vehicles = []

    candidates = t.candidate_od_pairs()

    for _ in range(N):
        rng.shuffle(candidates)
        chosen = None
        chosen_variant = None
        for (ent, ext) in candidates:
            entry = route_dict[(ent, ext)]
            variants = list(entry.get('variants') or [{'ints': entry['ints'],
                                                       'route_id': entry['route_id']}])
            # Try equal-length alternative paths in random order before
            # rejecting this OD pair.
            order = list(range(len(variants)))
            rng.shuffle(order)
            for idx in order:
                v = variants[idx]
                if all(int_load[i] < max_per_int for i in v['ints']):
                    chosen = (ent, ext)
                    chosen_variant = v
                    break
            if chosen is not None:
                break
        if chosen is None:
            raise ValueError(
                f'Cannot assign vehicle {len(vehicles)+1}: all routes (incl. '
                f'equal-length variants) exceed max_per_int={max_per_int}. '
                f'Try fewer vehicles or a larger limit.')
        ent, ext = chosen
        ints = chosen_variant['ints']
        rids = chosen_variant['route_id']
        for i in ints:
            int_load[i] += 1
        if ent not in entrance_rank:
            entrance_rank[ent] = 0
        entry_idx = entrance_rank[ent]
        entrance_rank[ent] += 1
        vehicles.append({'entrance': ent, 'exit': ext, 'entry_idx': entry_idx,
                         'ints': ints, 'route_id': rids})

    # ── Compute entrance rank offsets (unique entrances, order of first appearance) ──
    seen = {}
    ent_rank_offset = {}
    rank = 0
    for v in vehicles:
        e = v['entrance']
        if e not in seen:
            seen[e] = rank
            rank += 1
        ent_rank_offset[e] = seen[e]

    # ── Build path info ────────────────────────────────────────────────
    base_time = (detect_range / 2.0 - W_phys / 2.0) / v_max

    alpha_tilde_list = []
    deadline_list    = []
    chains_list      = []   # 1-indexed agent chains (incl terminal at end)
    cseqs_list       = []   # intersection durations
    route_ids_list   = []

    for v in vehicles:
        ent, ext, entry_idx = v['entrance'], v['exit'], v['entry_idx']
        ints = v['ints']         # chosen variant from sampling step
        rids = v['route_id']

        # alpha_tilde
        alpha = base_time + ent_rank_offset[ent] * T_ent + entry_idx * T_val
        alpha_tilde_list.append(alpha)

        # intersection durations — read from this intersection's IntSpaceDB
        dur = [sum(int_db_list[ints[k] - 1]['routeDur'][rids[k]])
               for k in range(len(ints))]
        cseqs_list.append(dur)

        # agent chain (1-indexed): int, road, int, road, ..., int, terminal
        chain = []
        for k in range(len(ints)):
            chain.append(ints[k])
            if k < len(ints) - 1:
                chain.append(t.get_road_agent(ints[k], ints[k+1]))
        chain.append(terminal_1)
        chains_list.append(chain)
        route_ids_list.append(rids)

        # deadline
        c_total  = sum(dur)
        num_roads = len(ints) - 1
        deadline_list.append(alpha + c_total + Dt * num_roads)

    # ── Build Python const dict (0-indexed, matching load_config output) ──
    alpha_tilde_py = [np.array([a]) for a in alpha_tilde_list]
    deadline_py    = [np.array([d]) for d in deadline_list]

    # chains: convert 1-indexed → 0-indexed
    chains_py = [[a - 1 for a in ch] for ch in chains_list]

    pathInfo_agent_chain = [[ch] for ch in chains_py]
    pathInfo_c           = [[cseq] for cseq in cseqs_list]
    pathInfo             = [{'routeId': rids} for rids in route_ids_list]

    # agent_participation (n_agents agents, 0-indexed)
    terminal_0 = terminal_1 - 1
    agent_participation = [[False] * N for _ in range(n_agents)]
    for n, ch in enumerate(chains_py):
        for ag in ch[:-1]:   # exclude terminal
            agent_participation[ag][n] = True
    agent_participation[terminal_0] = [True] * N   # terminal always

    const = {
        'N':          N,
        'Dt':         Dt,
        'rho1':       rho1,
        'rho2':       rho2,
        'weight':     weight,
        'max_iter':   max_iter,
        'tol_r':      tol_r,
        'tol_s':      tol_s,
        'priority_n': 0,
        'use_pruning': True,
        'use_weak_rule': True,
        'timeout_int_s': 30,
        'useTBound': True,
        'use_quadprog': True,
        'use_adaptive_rho': False,
        'randInitScale': 0.0,
        'useParallel': False,
        # Topology sizes (0-indexed agent layout: 0..n_int-1 = ints,
        # n_int..n_int+n_road-1 = roads, n_int+n_road = terminal).
        'n_int':            n_int,
        'n_road':           t.n_road,
        'n_agents':         n_agents,
        'terminal_id_0idx': terminal_1 - 1,
        'alpha_tilde': alpha_tilde_py,
        'deadline':    deadline_py,
        'pathInfo_agent_chain': pathInfo_agent_chain,
        'pathInfo_c':           pathInfo_c,
        'pathInfo':             pathInfo,
        'IntSpaceDB': list(int_db_list),
        # Per-vehicle config (entrance / exit / entry_idx) — mirrors MATLAB
        # const.config{n}, used by demo .js exporter to populate vehicle metadata.
        'config': [
            {'entrance': v['entrance'], 'exit': v['exit'],
             'entry_idx': v['entry_idx']}
            for v in vehicles
        ],
    }

    return const, agent_participation


# ===========================================================================
# Save / load JSON (same format as validation_config.json)
# ===========================================================================

def save_config_json(const, agent_participation, filepath, topology=None):
    """Save config to JSON in the same format as export_config_for_python.m."""
    t = topology if topology is not None else _topo.DEFAULT
    n_int      = t.n_int
    n_nonterm  = t.n_int + t.n_road   # 0-indexed agents 0..n_nonterm-1
    int_db_list = t.int_db

    N   = const['N']
    out = {
        'N':            N,
        'Dt':           const['Dt'],
        'rho1':         const['rho1'],
        'rho2':         const['rho2'],
        'weight':       const['weight'],
        'max_iter':     const['max_iter'],
        'tol_r':        const['tol_r'],
        'tol_s':        const['tol_s'],
        'randInitScale': const.get('randInitScale', 0.0),
        'alpha_tilde':  [float(const['alpha_tilde'][n][0]) for n in range(N)],
        'deadline':     [float(const['deadline'][n][0])    for n in range(N)],
        # agent chains: back to 1-indexed for JSON
        'agent_chains': [[a + 1 for a in const['pathInfo_agent_chain'][n][0]]
                         for n in range(N)],
        'path_c':       [const['pathInfo_c'][n][0] for n in range(N)],
        'route_ids':    [const['pathInfo'][n]['routeId'] for n in range(N)],
        # agent_participation: n_nonterm x N (0-indexed non-terminal agents)
        'agent_participation': [
            [int(agent_participation[ag][n]) for n in range(N)]
            for ag in range(n_nonterm)
        ],
        # IntSpaceDB: one entry per intersection (routeSpace/routeDur as lists)
        'IntSpaceDB': [
            {
                'numSpaces': int_db_list[i]['numSpaces'],
                'numRoutes': int_db_list[i]['numRoutes'],
                'routeSpace': [int_db_list[i]['routeSpace'][r+1]
                               for r in range(int_db_list[i]['numRoutes'])],
                'routeDur':   [int_db_list[i]['routeDur'][r+1]
                               for r in range(int_db_list[i]['numRoutes'])],
            }
            for i in range(n_int)
        ],
    }
    Path(filepath).parent.mkdir(parents=True, exist_ok=True)
    with open(filepath, 'w') as f:
        json.dump(out, f)
