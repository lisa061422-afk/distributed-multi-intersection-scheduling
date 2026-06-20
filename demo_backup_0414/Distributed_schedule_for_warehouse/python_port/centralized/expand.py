"""
expand_array_global_pure: stateless tree expansion (read-only on `nodes`).

Returns (children, mark_leaf) where children is a list of Node packets with
.idx == -1 (the caller assigns global indices when appending to NODES).
"""
from __future__ import annotations

from typing import List, Tuple

import numpy as np

from .helpers import EPS, BIG_M, next_sig_m_global
from .node import Node, clone_arr_list, clone_x, make_node
from .resetting_rule import resetting_rule_global
from .traverse_columns import traverse_columns


def expand_pure(nodes: List[Node], c_idx: int, const: dict) -> Tuple[List[Node], bool]:
    N         = const["N"]
    NI        = const["NI"]
    Smax      = const["Smax"]
    Mtot      = const["Mtot"]
    task_cache = const["task_cache"]
    use_weak   = const.get("use_weak_rule", True)
    priority_n = const.get("priority_n", 0)

    c = nodes[c_idx]
    l     = c.idx
    d     = c.d
    r     = c.r
    tw    = c.tw
    ni    = c.ni
    g     = c.g
    gamma = c.gamma
    speed = c.speed
    x     = c.x
    alpha = c.alpha
    o     = c.o

    pair_lock   = c.pair_lock
    reset_since = c.reset_since

    # ---- initialise updated states at tw ----
    da  = np.full(N, np.inf)
    ra  = np.zeros((Smax, N))
    oa  = np.zeros(N)
    ni2 = ni.copy()
    U_c = np.zeros((N, Mtot), dtype=int)

    alpha = clone_arr_list(alpha)

    for n in range(N):
        if ni[n] >= NI[n] and np.all(r[:, n] <= EPS):
            ni2[n]  = NI[n]
            da[n]   = np.inf
            ra[:, n] = 0
            oa[n]   = 0
            continue

        if d[n] <= EPS and ni[n] < NI[n]:
            next_task = ni[n] + 1
            ti = next_task - 1
            task = task_cache[n][ti]

            ni2[n]  = next_task
            da[n]   = BIG_M
            ra[:, n] = task.C
            oa[n]   = 0

            # alpha[n] is np.ndarray length NI(n); guard NaNs
            if alpha[n] is None:
                alpha[n] = np.full(NI[n], np.nan)
            elif alpha[n].shape[0] < NI[n]:
                pad = np.full(NI[n] - alpha[n].shape[0], np.nan)
                alpha[n] = np.concatenate([alpha[n], pad])
            if np.isnan(alpha[n][ti]):
                alpha[n][ti] = tw
        else:
            ni2[n]  = ni[n]
            da[n]   = d[n]
            ra[:, n] = r[:, n]
            oa[n]   = o[n]

    # ---- terminate if all done ----
    all_done = True
    for n in range(N):
        if not (ni2[n] >= NI[n] and np.all(ra[:, n] <= EPS)):
            all_done = False
            break
    if all_done:
        return [], True

    # ---- find active systems and build U_c ----
    active = [n for n in range(N) if np.any(ra[:, n] > EPS)]
    for n in active:
        s_idx_arr = np.where(ra[:, n] > EPS)[0]
        if s_idx_arr.size == 0:
            continue
        s_idx = int(s_idx_arr[0])  # 0-indexed sub-task row
        ti = ni2[n] - 1
        task = task_cache[n][ti]
        m1 = int(task.Map[s_idx])      # 1-indexed global space
        U_c[n, m1 - 1] = s_idx + 1     # store 1-indexed sub-task

    # ---- ctx: Cvec, MapVec for current ni2 ----
    Cvec   = np.zeros((Smax, N))
    MapVec = np.zeros((Smax, N), dtype=int)
    for n in range(N):
        if ni2[n] >= 1 and ni2[n] <= NI[n]:
            ti = ni2[n] - 1
            task = task_cache[n][ti]
            L = task.L
            Cvec[:L, n]   = task.C[:L]
            MapVec[:L, n] = task.Map[:L]
    ctx = {"Cvec": Cvec, "MapVec": MapVec}

    # =======================================================
    # CASE 1: no active tasks → propagate
    # =======================================================
    if not active or np.all(ra <= EPS):
        U_temp = np.zeros((N, Mtot), dtype=int)
        d2, r2, o2, tw1 = next_sig_m_global(tw, da, ra, oa, U_temp)
        ra_reset = -1 * np.ones((Smax, N))
        new_node = make_node(l, d2, r2, o2, tw1, ni2, U_c, U_temp,
                             g, gamma, speed, ra, ra_reset, x, alpha,
                             const, pair_lock, reset_since)
        return [new_node], False

    # =======================================================
    # CASE 2: active tasks exist
    # =======================================================
    col_cnt = (U_c > 0).sum(axis=0)

    # ---- no contention ----
    if np.all(col_cnt <= 1):
        U_temp = U_c.copy()
        d2, r2, o2, tw1 = next_sig_m_global(tw, da, ra, oa, U_temp)
        ra_reset = -1 * np.ones((Smax, N))
        x_clean = _update_vtemp_x(x, U_temp, ni2, r2, tw1, Cvec, N)
        new_node = make_node(l, d2, r2, o2, tw1, ni2, U_c, U_temp,
                             g, gamma, speed, ra, ra_reset, x_clean, alpha,
                             const, pair_lock, reset_since)
        return [new_node], False

    # ---- contention ----
    U_valid, _, cb_updates = traverse_columns(U_c, priority_n, pair_lock, ra)

    children: List[Node] = []
    child_sigs: List[tuple] = []

    for U_temp_i in U_valid:
        ra_temp = ra.copy()
        branches = resetting_rule_global(ra, ra_temp, U_temp_i, U_c, tw, da,
                                         nodes, c_idx, x, ni2, ctx, const,
                                         pair_lock, reset_since)
        for br in branches:
            ra_t2  = br["ra_temp"]
            U_t2   = br["U_temp"]
            x2     = br["x"]
            pl_upd = br["pair_lock"]
            rs2    = br["reset_since"]

            ra_reset = ra_t2.copy()

            if use_weak:
                new_pair_lock = pl_upd.copy()
                contended = np.where((U_c > 0).sum(axis=0) >= 2)[0]
                for m_col in contended:
                    winners = np.where(U_t2[:, m_col] > 0)[0]
                    if winners.size == 0:
                        continue
                    w = int(winners[0])
                    losers = np.where(U_c[:, m_col] > 0)[0]
                    losers = losers[losers != w]
                    for ll in losers:
                        if new_pair_lock[w, ll] == -1 or new_pair_lock[w, ll] == w:
                            new_pair_lock[w, ll] = w
                            new_pair_lock[ll, w] = w
                cb_mask = cb_updates != -1
                new_pair_lock[cb_mask] = cb_updates[cb_mask]
            else:
                new_pair_lock = -1 * np.ones((N, N), dtype=int)

            d2, r2, o2, tw1 = next_sig_m_global(tw, da, ra_t2, oa, U_t2)
            x2 = _update_vtemp_x(x2, U_t2, ni2, r2, tw1, Cvec, N)

            sig = (round(tw1, 8),
                   tuple(ni2.tolist()),
                   tuple(np.round(d2, 8).tolist()),
                   tuple(np.round(o2, 8).tolist()),
                   tuple(np.round(r2.flatten(), 8).tolist()),
                   tuple(U_t2.flatten().tolist()))
            if sig in child_sigs:
                continue
            child_sigs.append(sig)

            new_node = make_node(l, d2, r2, o2, tw1, ni2, U_c, U_t2,
                                 g, gamma, speed, ra, ra_reset, x2, alpha,
                                 const, new_pair_lock, rs2)
            children.append(new_node)

    return children, False


# ---------------------------------------------------------------------------
def _update_vtemp_x(x_in: list, U_temp: np.ndarray, ni2: np.ndarray,
                    ra_new: np.ndarray, tw_new: float,
                    Cvec: np.ndarray, N: int) -> list:
    """Refresh x records for the active sub-task chosen in U_temp."""
    x_out = clone_x(x_in)
    for n in range(N):
        if ni2[n] < 1:
            continue
        ti = ni2[n] - 1
        if ti >= len(x_out[n]):
            continue
        recs = x_out[n][ti]
        if recs is None:
            recs = []

        cols = np.where(U_temp[n, :] > 0)[0]
        m_idx = int(cols[0]) if cols.size else None  # 0-indexed column

        if m_idx is not None:
            s_cur = int(U_temp[n, m_idx])  # 1-indexed sub-task
            has_rec = any(r[2] == s_cur for r in recs)
            if has_rec:
                recs = [r for r in recs if r[2] != s_cur]
                t_end = tw_new + ra_new[s_cur - 1, n]
                recs.append((t_end - Cvec[s_cur - 1, n], t_end, s_cur, m_idx + 1))
                x_out[n][ti] = recs
        else:
            if not recs:
                continue
            new_recs = []
            for r in recs:
                t_start_old, t_end_old, s_ri, m_ri = r
                if ra_new[s_ri - 1, n] > EPS:
                    t_end_new = tw_new + ra_new[s_ri - 1, n]
                    new_recs.append((t_end_new - Cvec[s_ri - 1, n], t_end_new, s_ri, m_ri))
                else:
                    new_recs.append(r)
            x_out[n][ti] = new_recs
    return x_out
