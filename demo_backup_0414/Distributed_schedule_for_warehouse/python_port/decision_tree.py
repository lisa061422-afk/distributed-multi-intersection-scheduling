"""
Equivalent of MATLAB INi_Admm_DecisionTree.m

Key change: A* (min-f priority queue) replaced with BFS (collections.deque).
BFS explores nodes in breadth-first order — simpler, no heuristic needed,
Python deque is highly optimised.

All arrays 0-indexed (n=0..N-1, s=0..S-1).
"""

import time
import copy
import numpy as np
from collections import deque

from .node import Node, make_node
from .tree_utils import (
    next_sig_m, tw1_basedon_u_temp,
    traverse_columns, resetting_rule,
    prune_nodes_by_ni, update_vtemp_x,
)
from .in_admm import in_admm


# ---------------------------------------------------------------------------

def _expand_node(NODES, node_count, OPEN_queue, LEAF,
                 c_node_idx, ctx, const):
    """
    Equivalent of MATLAB expand_array_IN.
    Pops nothing from OPEN_queue — caller already did that.
    Adds child nodes to OPEN_queue and returns updated state.
    """
    n_pruned_out = 0
    N = const['N']
    M = ctx['M']
    S = ctx['S']
    NI_agent = ctx['NI_agent']
    valid_systems = ctx['valid_systems']
    Cmat = ctx['Cmat']
    MapMat = ctx['MapMat']
    ddl = ctx['ddl']
    arrival_ref = ctx['arrival_ref']
    priority_n = const.get('priority_n', 0)
    use_weak_rule = const.get('use_weak_rule', True)

    node = NODES[c_node_idx]
    l = node.idx
    d = node.d.copy()
    r = node.r.copy()
    o = node.o.copy()
    tw = node.tw
    ni = node.ni.copy()
    g = node.g
    gamma = copy.deepcopy(node.gamma)
    speed = list(node.speed)
    x = copy.deepcopy(node.x)
    alpha = copy.deepcopy(node.alpha)
    pair_lock = node.pair_lock.copy()
    reset_since = node.reset_since.copy()
    parent_node_index = l

    # ── advance each vehicle's deadline counter ──────────────────────
    da = np.zeros(N)
    ra = np.zeros((S, N))
    oa = np.zeros(N)
    ni2 = np.zeros(N, dtype=int)

    for n in valid_systems:
        if d[n] <= 1e-5:
            if np.sum(r[:, n]) > 1e-5:
                # deadline fired but task still running
                ni2[n] = ni[n]
                da[n] = 16.111
                ra[:, n] = r[:, n]
                oa[n] = o[n]
            elif ni[n] + 1 <= NI_agent[n]:
                ni2[n] = ni[n] + 1
                da[n] = 16.111
                ra[:, n] = Cmat[:, n]
                oa[n] = 0.0
            else:
                ni2[n] = NI_agent[n] + 1
                da[n] = np.inf
                ra[:, n] = -np.inf
                oa[n] = 0.0
        else:
            ni2[n] = ni[n]
            da[n] = d[n]
            ra[:, n] = r[:, n]
            oa[n] = o[n]

    # ── termination check ────────────────────────────────────────────
    if np.all(ni2[valid_systems] == np.array(NI_agent)[valid_systems] + 1):
        LEAF.append(l)
        return NODES, node_count, OPEN_queue, LEAF, n_pruned_out

    all_gamma_done = all(
        gamma[n] is not None and not np.isnan(gamma[n])
        for n in valid_systems
    )
    if all_gamma_done:
        LEAF.append(l)
        return NODES, node_count, OPEN_queue, LEAF, n_pruned_out

    # ── expand node ──────────────────────────────────────────────────
    def _add_child(d2, r2, o2, tw1, ra_reset, U_c, U_temp, x_new,
                   pair_lock_new, reset_since_new):
        nonlocal node_count
        new_node = make_node(
            node_count, d2, r2, o2, tw1, ni2, parent_node_index,
            U_c, U_temp, g, gamma, speed, ra, ra_reset, x_new,
            Cmat, valid_systems, alpha, arrival_ref, const,
            pair_lock_new, reset_since_new)
        node_count += 1
        if node_count >= len(NODES):
            NODES.extend([None] * len(NODES))
        NODES[node_count] = new_node
        OPEN_queue.append(node_count)
        return new_node

    if np.all(ra[:, valid_systems] <= 1e-5):
        # No active tasks — single branch
        U_temp_empty = np.zeros((N, M), dtype=int)
        d2, r2, o2, tw1 = next_sig_m(tw, da, ra, oa, U_temp_empty,
                                      valid_systems, ctx, const)
        ra_reset = -np.ones((S, N))
        _add_child(d2, r2, o2, tw1, ra_reset,
                   np.zeros((N, M), dtype=int), U_temp_empty,
                   x, pair_lock, reset_since)

    elif np.any(ra[:, valid_systems] > 1e-5):
        da = np.round(da, 6)
        ra = np.round(ra, 6)
        active_sys = [n for n in valid_systems
                      if np.any(ra[:, n] > 1e-5)]

        U_c = np.zeros((N, M), dtype=int)
        for n in active_sys:
            s_idx = np.where(ra[:, n] > 1e-5)[0]
            if len(s_idx) > 0:
                s0 = int(s_idx[0])
                m1 = int(MapMat[s0, n])   # 1-indexed space label
                if 0 < m1 <= M:
                    U_c[n, m1 - 1] = s0 + 1   # store 1-indexed slot

        col_cnt = np.sum(U_c[active_sys, :] > 0, axis=0)

        if np.any(col_cnt > 1):
            V_valid, n_pr, cb_updates = traverse_columns(U_c, priority_n,
                                                          pair_lock, ra)
            n_pruned_out += n_pr

            for U_temp_v in V_valid:
                branches = resetting_rule(
                    ra, ra.copy(), U_temp_v, U_c,
                    tw, da, NODES, l, x, ni2,
                    ctx, const, pair_lock, reset_since)

                for br in branches:
                    ra_t2, V_t2, x_t2, pl_upd, rs_upd2, tw1_rr = br

                    if use_weak_rule:
                        new_pl = pl_upd.copy()
                        cont_m = np.where(np.sum(U_c > 0, axis=0) >= 2)[0]
                        for m_col in cont_m:
                            w_rows = np.where(V_t2[:, m_col] > 0)[0]
                            if len(w_rows) == 0:
                                continue
                            w = int(w_rows[0])
                            losers = np.where(U_c[:, m_col] > 0)[0]
                            losers = losers[losers != w]
                            mask = ((new_pl[w, losers] == 0) |
                                    (new_pl[w, losers] == w))
                            valid_l = losers[mask]
                            if len(valid_l) > 0:
                                new_pl[w, valid_l] = w
                                new_pl[valid_l, w] = w
                        cb_mask = cb_updates != 0
                        new_pl[cb_mask] = cb_updates[cb_mask]
                    else:
                        new_pl = np.zeros((N, N))

                    d2, r2, o2, tw1 = next_sig_m(
                        tw, da, ra_t2, oa, V_t2,
                        valid_systems, ctx, const, tw1_rr)
                    x_t2_upd = update_vtemp_x(
                        x_t2, V_t2, ni2, r2, tw1, Cmat, N)

                    ra_reset = ra_t2.copy()
                    _add_child(d2, r2, o2, tw1, ra_reset,
                               U_c, V_t2, x_t2_upd, new_pl, rs_upd2)

        else:
            # No contention
            U_temp_v = U_c.copy()
            d2, r2, o2, tw1 = next_sig_m(tw, da, ra, oa, U_temp_v,
                                          valid_systems, ctx, const)
            ra_reset = -np.ones((S, N))
            new_pl = pair_lock.copy() if use_weak_rule else np.zeros((N, N))
            x_clean = update_vtemp_x(x, U_temp_v, ni2, r2, tw1, Cmat, N)
            _add_child(d2, r2, o2, tw1, ra_reset,
                       U_c, U_temp_v, x_clean, new_pl, reset_since)

    return NODES, node_count, OPEN_queue, LEAF, n_pruned_out


# ---------------------------------------------------------------------------

def ini_admm_decision_tree(agent_i, entries,
                            x_prev, y_prev,
                            xi_prev_bar, yi_prev_bar,
                            valid_systems, ai_x, ai_y,
                            const, local_tree_cache=None):
    """
    Equivalent of MATLAB INi_Admm_DecisionTree with BFS instead of A*.

    Parameters (all 0-indexed vehicles)
    ------------------------------------
    agent_i         : int   0-indexed agent ID
    entries         : list[N]  non-empty = vehicle visits this agent
    x_prev          : list[9] of list[N]  x_prev[agent][n] = array
    y_prev          : list[9] of list[N]
    xi_prev_bar     : list[N]  averaged x per vehicle
    yi_prev_bar     : list[N]  averaged y per vehicle
    valid_systems   : list of int  0-indexed
    ai_x, ai_y      : list[N]  dual variables
    const           : dict
    local_tree_cache: dict or None  (cached tree, reused across ADMM iterations)

    Returns
    -------
    x_out, y_out, best_alpha, best_gamma, best_idx, NODES, local_tree_cache
    """
    IntSpaceDB = const['IntSpaceDB']
    N = const['N']
    Dt = const['Dt']
    alpha_tilde = const['alpha_tilde']
    pathInfo_agent_chain = const['pathInfo_agent_chain']
    pathInfo = const['pathInfo']
    pathInfo_c = const['pathInfo_c']
    use_pruning = const.get('use_pruning', True)
    use_t_bound = const.get('useTBound', True)
    timeout_s = const.get('timeout_int_s', 30)

    kn = 0    # 0-indexed task index (kn=1 in MATLAB)
    T_cst = 16.111
    S = 3

    M = IntSpaceDB[agent_i]['numSpaces']
    Cmat = np.zeros((S, N))
    MapMat = np.zeros((S, N), dtype=int)

    NI_agent = np.zeros(N, dtype=int)
    NI_agent[valid_systems] = 1

    ddl = [None] * N
    arrival_ref = np.full(N, np.nan)

    for n in valid_systems:
        ddl[n] = np.nan
        chain = pathInfo_agent_chain[n][kn]   # list of agent IDs (0-indexed)
        try:
            posi = chain.index(agent_i)       # 0-indexed position in chain
        except ValueError:
            continue

        pos_int = posi // 2                   # which intersection (0-indexed)
        route_idx = pathInfo[n]['routeId'][pos_int]   # route label

        space_seq = IntSpaceDB[agent_i]['routeSpace'][route_idx]
        dur_seq   = IntSpaceDB[agent_i]['routeDur'][route_idx]
        L = min(S, len(dur_seq))
        Cmat[:L, n] = dur_seq[:L]
        MapMat[:L, n] = space_seq[:L]

        if posi == 0:
            ddl[n] = float(alpha_tilde[n][kn])
            arrival_ref[n] = float(alpha_tilde[n][kn])
        else:
            prev_road = chain[posi - 1]
            prev_int  = chain[posi - 2]
            x1 = float(x_prev[prev_road][n][kn]) + Dt
            x2 = float(x_prev[agent_i][n][kn])
            lam = 0.5
            ddl[n] = x1 + lam * (x2 - x1)
            arrival_ref[n] = float(y_prev[prev_int][n][kn]) + Dt

    # ── Build ctx struct ─────────────────────────────────────────────
    ctx = {
        'agent_i': agent_i,
        'M': M, 'S': S,
        'NI_agent': NI_agent,
        'entries': entries,
        'valid_systems': valid_systems,
        'Cmat': Cmat,
        'MapMat': MapMat,
        'ddl': ddl,
        'arrival_ref': arrival_ref,
    }

    # T_bound: worst-case sequential deadline
    valid_arrivals = arrival_ref[valid_systems]
    valid_arrivals = valid_arrivals[~np.isnan(valid_arrivals)]
    T_bound = (float(np.max(valid_arrivals)) if len(valid_arrivals) > 0 else 0.0) \
              + float(np.sum(Cmat[:, valid_systems]))
    print(f'  [T_bound] Agent {agent_i}: {T_bound:.2f}')

    # ── Initialise tree ──────────────────────────────────────────────
    d_init = np.zeros(N)
    for n in valid_systems:
        d_init[n] = ddl[n] if ddl[n] is not None and not np.isnan(ddl[n]) else 0.0

    r_init = np.zeros((S, N))
    o_init = np.zeros(N)
    ni_init = np.zeros(N, dtype=int)
    gamma_init = [None] * N
    alpha_init = [None] * N
    x_init = [[] for _ in range(N)]

    NODES = [None] * 2000   # 1-based; index 0 unused
    root = Node(
        idx=1, d=d_init, r=r_init, o=o_init, tw=0.0, ni=ni_init,
        parent=0,
        U_c=np.zeros((N, M), dtype=int),
        U_temp=np.zeros((N, M), dtype=int),
        g=0.0, gamma=gamma_init, f=0.0,
        speed=[],
        ra_reset=np.zeros((S, N)),
        x=x_init, alpha=alpha_init,
        pair_lock=np.zeros((N, N)),
        reset_since=np.zeros(N),
    )
    NODES[1] = root
    node_count = 1

    OPEN_queue = deque([1])   # BFS queue (node indices)
    LEAF = []
    total_pruned = 0
    t_start = time.time()

    # ── BFS loop ─────────────────────────────────────────────────────
    while OPEN_queue:
        c_node_idx = OPEN_queue.popleft()   # BFS: FIFO

        NODES, node_count, OPEN_queue, LEAF, np_out = _expand_node(
            NODES, node_count, OPEN_queue, LEAF, c_node_idx, ctx, const)
        total_pruned += np_out

        # Pruning: keep only min-g among nodes with same ni
        if use_pruning and len(OPEN_queue) > 1:
            pruned_list = prune_nodes_by_ni(NODES, list(OPEN_queue))
            OPEN_queue = deque(pruned_list)

        # T_bound pruning
        if use_t_bound:
            new_q = deque()
            for idx in OPEN_queue:
                if NODES[idx].tw > T_bound:
                    total_pruned += 1
                    print(f'  [T_bound] Agent {agent_i}: pruned node (tw={NODES[idx].tw:.2f} > {T_bound:.2f})')
                else:
                    new_q.append(idx)
            if not new_q and not LEAF:
                LEAF.append(c_node_idx)
            OPEN_queue = new_q

        # Timeout
        if time.time() - t_start > timeout_s:
            print(f'  [BFS] Agent {agent_i}: timeout {timeout_s}s, forcing termination.')
            OPEN_queue.clear()
            if not LEAF:
                LEAF.append(1)
            break

    # Trim NODES list
    NODES = NODES[:node_count + 1]

    print(f'  [Agent {agent_i}] nodes={node_count}  leaves={len(LEAF)}'
          f'  pruned={total_pruned}  t={time.time()-t_start:.2f}s')

    # ── Solve QP over all leaves ─────────────────────────────────────
    x_out, y_out, best_alpha, best_gamma, best_idx, NODES = in_admm(
        NODES, LEAF, agent_i, entries,
        x_prev[agent_i], y_prev[agent_i],
        xi_prev_bar, yi_prev_bar,
        ai_x, ai_y,
        valid_systems, MapMat, Cmat, const)

    # ── Reconstruct optimal path ─────────────────────────────────────
    result_nodes = []
    cur = best_idx
    while cur > 0:
        result_nodes.insert(0, cur)
        cur = NODES[cur].parent

    # Update local_tree_cache
    if local_tree_cache is None:
        local_tree_cache = {}
    local_tree_cache['NODES'] = NODES
    local_tree_cache['Path'] = result_nodes
    local_tree_cache['best_idx'] = best_idx
    local_tree_cache['cost'] = NODES[best_idx].g if best_idx else 0.0
    local_tree_cache['Cmat'] = Cmat
    local_tree_cache['MapMat'] = MapMat
    local_tree_cache['valid_systems'] = valid_systems
    local_tree_cache['entries'] = entries

    return x_out, y_out, best_alpha, best_gamma, best_idx, NODES, local_tree_cache
