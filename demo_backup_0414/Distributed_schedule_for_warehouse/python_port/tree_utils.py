"""
Tree utility functions — Python equivalents of MATLAB helpers used during
decision-tree expansion.

All arrays 0-indexed (n=0..N-1, s=0..S-1).
Space labels m are 1-indexed integers (physical IDs stored in MapMat values).
"""

import numpy as np
import copy
from collections import defaultdict


# ===========================================================================
# NextSigM  (next time-window event)
# ===========================================================================

def next_sig_m(tw, da, ra, oa, U_temp, valid_systems, ctx, const, tw1_in=None):
    """
    Equivalent of MATLAB NextSigM.
    Advances time by the minimum of remaining deadlines / slot durations.

    Returns (dt, rt, ot, tw1)  — all 0-indexed.
    """
    N = const['N']
    S = ctx['S']

    rt = np.zeros((S, N))
    ot = np.zeros(N)
    dt = np.zeros(N)

    if tw1_in is not None and tw1_in != 0:
        tw1 = float(tw1_in)
        Lw = tw1 - tw
    elif np.all(U_temp == 0):
        pos = da > 1e-5
        Lw = float(np.min(da[pos])) if np.any(pos) else 0.0
        tw1 = round(tw + Lw, 6)
    else:
        remain_time = np.zeros(N)
        for n in valid_systems:
            row = U_temp[n, :]
            if np.any(row):
                m_idx = int(np.argmax(row > 0))
                s_temp = int(U_temp[n, m_idx]) - 1   # convert 1-indexed slot to 0-indexed
                remain_time[n] = ra[s_temp, n]

        pos_da = (da > 1e-5) & (da < 1000)
        pos_rt = remain_time > 1e-5
        candidates = []
        if np.any(pos_da):
            candidates.append(float(np.min(da[pos_da])))
        if np.any(pos_rt):
            candidates.append(float(np.min(remain_time[pos_rt])))
        Lw = min(candidates) if candidates else 0.0
        tw1 = round(tw + Lw, 6)

    t = Lw
    for n in valid_systems:
        dt[n] = da[n] - t
        if np.any(U_temp[n, :]):
            m_idx = int(np.argmax(U_temp[n, :] > 0))
            s_temp = int(U_temp[n, m_idx]) - 1   # 0-indexed slot
            rt[:, n] = ra[:, n]
            rt[s_temp, n] = ra[s_temp, n] - t
            ot[n] = oa[n] + t
        else:
            rt[:, n] = ra[:, n]
            ot[n] = oa[n] + np.sign(np.sum(ra[:, n])) * t

    return dt, rt, ot, tw1


# ===========================================================================
# tw1_basedon_u_temp
# ===========================================================================

def tw1_basedon_u_temp(U_temp, ra, da, tw, const):
    """Equivalent of MATLAB tw1_basedon_U_temp."""
    N = const['N']
    if np.all(U_temp == 0):
        pos = da > 1e-5
        Lw = float(np.min(da[pos])) if np.any(pos) else 0.0
    else:
        remain_time = np.zeros(N)
        for n in range(N):
            if np.any(U_temp[n, :]):
                m_idx = int(np.argmax(U_temp[n, :] > 0))
                s_temp = int(U_temp[n, m_idx]) - 1   # 0-indexed
                remain_time[n] = ra[s_temp, n]
        pos_da = (da > 1e-5) & (da < 1000)
        pos_rt = remain_time > 1e-5
        candidates = []
        if np.any(pos_da):
            candidates.append(float(np.min(da[pos_da])))
        if np.any(pos_rt):
            candidates.append(float(np.min(remain_time[pos_rt])))
        Lw = min(candidates) if candidates else 0.0
    return float(tw + Lw)


# ===========================================================================
# trace_valid_nodes
# ===========================================================================

def trace_valid_nodes(curr_idx, t_curr, t1, NODES):
    """
    Trace ancestor nodes from curr_idx back to the first node with tw <= t1.
    Returns list of node indices (1-based).
    """
    valid_nodes = [curr_idx]
    while t_curr > t1 + 1e-5:
        node = NODES[curr_idx]
        parent_idx = node.parent
        if parent_idx == 0:
            break
        t_parent = NODES[parent_idx].tw
        valid_nodes.append(parent_idx)
        curr_idx = parent_idx
        t_curr = t_parent
    return valid_nodes


# ===========================================================================
# find_node_ta_bar
# ===========================================================================

def find_node_ta_bar(l, tw, t_avail, NODES):
    """
    Find the largest ancestor tw <= t_avail.
    Returns (ta_bar_node, ta_bar).
    """
    ta_bar_node = l
    ta_bar = tw
    while ta_bar > t_avail + 1e-5:
        node = NODES[ta_bar_node]
        ta_bar_node = node.parent
        ta_bar = NODES[ta_bar_node].tw
    return ta_bar_node, ta_bar


# ===========================================================================
# check_resc_occupation
# ===========================================================================

def check_resc_occupation(NODES, valid_nodes, resc1, l, V_temp, n,
                          tw1, tw, reset_since):
    """
    Equivalent of MATLAB check_resc_occupation.
    Returns (last_valid_idx, t_avail).
    resc1: space label (1-indexed integer).
    n: vehicle index (0-indexed).
    """
    N = V_temp.shape[0]
    others = [p for p in range(N) if p != n]

    last_valid_idx = -1
    t_avail = tw1

    if np.all(V_temp[others, resc1 - 1] == 0):   # resc1 is 1-indexed → col resc1-1
        last_valid_idx = l
        t_avail = tw

        for i in range(len(valid_nodes) - 1):
            idx = valid_nodes[i]
            node = NODES[idx]
            V = node.U_temp            # historical space assignment (N, M)
            tw_idx = node.tw

            any_full_occ = False
            reset_cap = np.inf

            for j in others:
                col = resc1 - 1        # space label → 0-indexed column
                if col >= V.shape[1] or V[j, col] == 0:
                    continue
                rs_j = reset_since[j] if j < len(reset_since) else 0.0
                if rs_j > 0 and rs_j <= tw_idx:
                    continue
                elif rs_j > 0 and rs_j > tw_idx:
                    reset_cap = min(reset_cap, rs_j)
                else:
                    any_full_occ = True

            if not any_full_occ and np.isinf(reset_cap):
                last_valid_idx = idx
                parent_idx = NODES[last_valid_idx].parent
                if parent_idx > 0:
                    t_avail = NODES[parent_idx].tw
            elif not any_full_occ and np.isfinite(reset_cap):
                t_avail = reset_cap
                break
            else:
                break

    return last_valid_idx, t_avail


# ===========================================================================
# check_x
# ===========================================================================

def check_x(t_avail, s, m, x, n, Cmat, N):
    """
    Equivalent of MATLAB check_x.
    s: 0-indexed slot, m: 1-indexed space label.
    x[p] = list of (t_start, t_end, slot, space_label) records.
    Returns (t_avail, blocker).
    """
    blocker = -1
    for p in range(N):
        if p == n or x[p] is None or len(x[p]) == 0:
            continue
        for rec in reversed(x[p]):
            t_start, t_end, slot, resc = rec
            if resc == m and not (t_end <= t_avail or
                                   t_start >= t_avail + Cmat[s, n]):
                t_avail = t_end
                blocker = p
    return t_avail, blocker


# ===========================================================================
# update_vtemp_x
# ===========================================================================

def update_vtemp_x(x_in, U_temp, ni2, ra_new, tw_new, Cmat, N):
    """
    Equivalent of MATLAB update_vtemp_x (local function in expand_array_IN).
    x[n] = list of (t_start, t_end, s, m) records (flat, kn=0 assumed).
    """
    x_out = copy.deepcopy(x_in)
    for n in range(N):
        if x_out[n] is None:
            continue
        m_idx = -1
        for col in range(U_temp.shape[1]):
            if U_temp[n, col] > 0:
                m_idx = col
                break
        if m_idx >= 0:
            s_cur_1idx = int(U_temp[n, m_idx])   # 1-indexed slot stored in U_temp
            s_cur = s_cur_1idx - 1                # 0-indexed
            m_label = m_idx + 1                   # space label (1-indexed)
            # Remove existing record with same space label
            x_out[n] = [(ts, te, sl, rm) for (ts, te, sl, rm) in x_out[n]
                        if rm != m_label]
            t_end_new = tw_new + ra_new[s_cur, n]
            x_out[n].append((t_end_new - Cmat[s_cur, n], t_end_new,
                             s_cur_1idx, m_label))
        else:
            # No active slot — update existing records
            updated = []
            for (ts, te, slot_1idx, rm) in x_out[n]:
                s0 = slot_1idx - 1   # 0-indexed
                if ra_new[s0, n] > 1e-5:
                    t_end_new = tw_new + ra_new[s0, n]
                    updated.append((t_end_new - Cmat[s0, n], t_end_new,
                                   slot_1idx, rm))
                else:
                    updated.append((ts, te, slot_1idx, rm))
            x_out[n] = updated
    return x_out


# ===========================================================================
# Cycle detection (Kahn's algorithm)
# ===========================================================================

def _cycle_exists(pair_lock, rows):
    """Return True if the pair_lock sub-graph on `rows` contains a cycle."""
    n = len(rows)
    in_deg = np.zeros(n, dtype=int)
    for i in range(n):
        for j in range(n):
            if i != j and pair_lock[rows[i], rows[j]] == rows[j]:
                in_deg[i] += 1
    queue = list(np.where(in_deg == 0)[0])
    processed = 0
    while queue:
        v = queue.pop(0)
        processed += 1
        for u in range(n):
            if pair_lock[rows[v], rows[u]] == rows[v]:
                in_deg[u] -= 1
                if in_deg[u] == 0:
                    queue.append(u)
    return processed < n


# ===========================================================================
# space_variants  (local helper for resetting_rule)
# ===========================================================================

def _space_variants(V_temp, m, n, pair_lock, reset_since):
    """
    Generate V_temp variants when vehicle n needs space m (1-indexed label).
    Returns list of (V_new, displaced_vehicle_or_None).
    """
    col = m - 1   # 0-indexed column
    if col >= V_temp.shape[1]:
        return [(V_temp, None)]

    occ = [p for p in range(V_temp.shape[0])
           if V_temp[p, col] > 0 and p != n]
    if not occ:
        return [(V_temp, None)]

    np_ = occ[0]
    lock_val = pair_lock[n, np_]
    variants = []

    if lock_val == 0 or lock_val == np_:
        variants.append((V_temp.copy(), None))

    if lock_val == n:
        V_B = V_temp.copy()
        V_B[np_, col] = 0
        variants.append((V_B, np_))

    # If lock unknown and np_ was previously reset, n gets priority
    if (lock_val == 0
            and np_ < len(reset_since) and reset_since[np_] > 0):
        V_B = V_temp.copy()
        V_B[np_, col] = 0
        return [(V_B, np_)]   # displace only

    if not variants:
        variants = [(V_temp.copy(), None)]
    return variants


def _relock_vtemp(pl, V_temp, n, m):
    """Record that n lost to the current V_temp occupier of space m (1-indexed)."""
    if m <= 0:
        return pl
    col = m - 1
    if col >= V_temp.shape[1]:
        return pl
    occ = [p for p in range(V_temp.shape[0])
           if V_temp[p, col] > 0 and p != n]
    if occ:
        o = occ[0]
        if pl[n, o] == 0 or pl[n, o] == o:
            pl = pl.copy()
            pl[n, o] = o
            pl[o, n] = o
    return pl


# ===========================================================================
# traverse_columns
# ===========================================================================

def traverse_columns(U_c, priority_n, pair_lock, ra):
    """
    Equivalent of MATLAB traverse_columns.
    Enumerates valid selector matrices over contended columns.
    Returns (U_valid list, n_pruned, cb_updates matrix).
    """
    N, M = U_c.shape
    if pair_lock is None:
        pair_lock = np.zeros((N, N))
    if ra is None:
        ra = np.zeros((1, N))

    n_pruned = 0
    cb_updates = np.zeros_like(pair_lock)
    U_base = np.zeros_like(U_c)
    contended_cols = []

    for m in range(M):
        rows = np.where(U_c[:, m] > 0)[0]

        if len(rows) == 0:
            continue
        elif len(rows) == 1:
            U_base[rows[0], m] = U_c[rows[0], m]
        else:
            # Priority override
            if priority_n > 0 and U_c[priority_n, m] > 0:
                U_base[priority_n, m] = U_c[priority_n, m]
                n_pruned += len(rows) - 1
                continue

            # Weak rule: find candidate that beats all others via pair_lock
            winner_lock = -1
            for ci, candidate in enumerate(rows):
                s_cand = int(U_c[candidate, m])
                if s_cand < 1 or s_cand > ra.shape[0]:
                    continue
                if ra[s_cand - 1, candidate] <= 1e-5:
                    continue
                beats_all = all(
                    pair_lock[candidate, other] == candidate
                    for other in rows if other != candidate
                )
                if beats_all:
                    winner_lock = candidate
                    break

            if winner_lock >= 0:
                U_base[winner_lock, m] = U_c[winner_lock, m]
                n_pruned += len(rows) - 1
                continue

            # Cycle break
            if _cycle_exists(pair_lock, rows):
                win_count = np.zeros(len(rows), dtype=int)
                for ci, ri in enumerate(rows):
                    for oi, ro in enumerate(rows):
                        if ci != oi and pair_lock[ri, ro] == ri:
                            win_count[ci] += 1
                max_w = np.max(win_count)
                candidates = rows[win_count == max_w]
                winner_cb = int(np.min(candidates))
                U_base[winner_cb, m] = U_c[winner_cb, m]
                n_pruned += len(rows) - 1
                losers_cb = rows[rows != winner_cb]
                cb_updates[winner_cb, losers_cb] = winner_cb
                cb_updates[losers_cb, winner_cb] = winner_cb
                continue

            contended_cols.append(m)

    if not contended_cols:
        return [U_base], n_pruned, cb_updates

    def recurse(U_temp, k, result):
        if k >= len(contended_cols):
            result.append(U_temp.copy())
            return
        m = contended_cols[k]
        rows = np.where(U_c[:, m] > 0)[0]
        for n_r in rows:
            U_next = U_temp.copy()
            U_next[:, m] = 0
            U_next[n_r, m] = U_c[n_r, m]
            recurse(U_next, k + 1, result)

    U_valid = []
    recurse(U_base, 0, U_valid)
    return U_valid, n_pruned, cb_updates


# ===========================================================================
# resetting_rule
# ===========================================================================

def resetting_rule(ra, ra_temp, V_temp, V_c, tw, da, NODES, l,
                   x, ni2, ctx, const, pair_lock=None, reset_since=None):
    """
    Equivalent of MATLAB resetting_rule.
    Returns list of branch dicts: each = (ra_temp, V_temp, x, pair_lock, reset_since, tw1).
    """
    N = const['N']
    MapMat = ctx['MapMat']
    Cmat = ctx['Cmat']

    if pair_lock is None:
        pair_lock = np.zeros((N, N))
    if reset_since is None:
        reset_since = np.zeros(N)

    tw1 = tw1_basedon_u_temp(V_temp, ra, da, tw, const)
    interrupted = [n for n in range(N)
                   if np.any(ra[:, n] > 1e-5) and not np.any(V_temp[n, :] > 0)]

    branches = [(ra_temp, V_temp, x, pair_lock, reset_since, tw1)]

    for n in interrupted:
        next_branches = []
        for b in branches:
            ra_b, V_b, x_b, pl_b, rs_b, _ = b
            x_b = copy.deepcopy(x_b)
            x_b[n] = []              # discard stale reservations
            rs_b = rs_b.copy()
            rs_b[n] = tw

            s_int_cols = np.where(V_c[n, :] > 0)[0]
            if len(s_int_cols) == 0:
                next_branches.append((ra_b, V_b, x_b, pl_b, rs_b, tw1))
                continue

            s_int = int(V_c[n, s_int_cols[0]])   # 1-indexed slot

            # ── sub 1: interrupted at slot 1 ────────────────────────
            if s_int == 1:
                ra_b2 = ra_b.copy()
                ra_b2[:, n] = Cmat[:, n]
                pl_b2 = _relock_vtemp(pl_b.copy(), V_b, n, int(MapMat[0, n]))
                next_branches.append((ra_b2, V_b, x_b, pl_b2, rs_b, tw1))

            # ── sub 2: interrupted at slot 2 ────────────────────────
            elif s_int == 2:
                a1 = int(MapMat[0, n])
                t1_start = tw1 - Cmat[0, n]
                valid_a1 = trace_valid_nodes(l, tw, t1_start, NODES)

                vars2 = _space_variants(V_b, a1, n, pl_b, rs_b)
                for V_vi, displaced in vars2:
                    ra_vi = ra_b.copy()
                    x_vi = copy.deepcopy(x_b)
                    pl_vi = pl_b.copy()
                    rs_vi = rs_b.copy()

                    if displaced is not None:
                        np2 = displaced
                        ra_vi[:, np2] = Cmat[:, np2]
                        x_vi[np2] = []
                        rs_vi[np2] = tw
                        if pl_vi[n, np2] == 0 or pl_vi[n, np2] == n:
                            pl_vi[n, np2] = n
                            pl_vi[np2, n] = n
                        x_vi[n].append((tw1 - Cmat[0, n], tw1, 1, a1))
                        ra_vi[0, n] = 0
                        ra_vi[1:, n] = Cmat[1:, n]
                    else:
                        _, tb1 = check_resc_occupation(
                            NODES, valid_a1, a1, l, V_vi, n, tw1, tw, rs_vi)
                        tb1, blkr = check_x(tb1, 0, a1, x_vi, n, Cmat, N)
                        if blkr >= 0 and blkr != n:
                            if pl_vi[n, blkr] == 0 or pl_vi[n, blkr] == blkr:
                                pl_vi[n, blkr] = blkr
                                pl_vi[blkr, n] = blkr
                        pl_vi = _relock_vtemp(pl_vi, V_vi, n, a1)
                        if tb1 <= t1_start + 1e-5:
                            x_vi[n].append((tw1 - Cmat[0, n], tw1, 1, a1))
                            ra_vi[0, n] = 0
                        else:
                            x_vi[n].append((tb1, tb1 + Cmat[0, n], 1, a1))
                            ra_vi[0, n] = tb1 + Cmat[0, n] - tw1
                        ra_vi[1:, n] = Cmat[1:, n]

                    next_branches.append((ra_vi, V_vi, x_vi, pl_vi, rs_vi, tw1))

            # ── sub 3: interrupted at slot 3 ────────────────────────
            elif s_int == 3:
                a2 = int(MapMat[1, n])
                b1 = int(MapMat[0, n])
                t22 = tw1 - Cmat[1, n]
                valid_a2 = trace_valid_nodes(l, tw, t22, NODES)

                vars_a2 = _space_variants(V_b, a2, n, pl_b, rs_b)
                for V_v2, disp2 in vars_a2:
                    ra_v2 = ra_b.copy()
                    x_v2 = copy.deepcopy(x_b)
                    pl_v2 = pl_b.copy()
                    rs_v2 = rs_b.copy()

                    if disp2 is not None:
                        np2 = disp2
                        ra_v2[:, np2] = Cmat[:, np2]
                        x_v2[np2] = []
                        rs_v2[np2] = tw
                        if pl_v2[n, np2] == 0 or pl_v2[n, np2] == n:
                            pl_v2[n, np2] = n
                            pl_v2[np2, n] = n
                        ta2_avail = tw1
                    else:
                        _, ta2 = check_resc_occupation(
                            NODES, valid_a2, a2, l, V_v2, n, tw1, tw, rs_v2)
                        ta2_avail = max(ta2, t22)
                        ta2_avail, blkr2 = check_x(ta2_avail, 1, a2, x_v2, n, Cmat, N)
                        if blkr2 >= 0 and blkr2 != n:
                            if pl_v2[n, blkr2] == 0 or pl_v2[n, blkr2] == blkr2:
                                pl_v2[n, blkr2] = blkr2
                                pl_v2[blkr2, n] = blkr2
                        pl_v2 = _relock_vtemp(pl_v2, V_v2, n, a2)

                    t21 = ta2_avail - Cmat[0, n]
                    ta_bar_node, ta_bar = find_node_ta_bar(l, tw, ta2_avail, NODES)
                    valid_b1 = trace_valid_nodes(ta_bar_node, ta_bar, t21, NODES)

                    vars_b1 = _space_variants(V_v2, b1, n, pl_v2, rs_v2)
                    for V_v1, disp1 in vars_b1:
                        ra_v1 = ra_v2.copy()
                        x_v1 = copy.deepcopy(x_v2)
                        pl_v1 = pl_v2.copy()
                        rs_v1 = rs_v2.copy()

                        if disp1 is not None:
                            np1 = disp1
                            ra_v1[:, np1] = Cmat[:, np1]
                            x_v1[np1] = []
                            rs_v1[np1] = tw
                            if pl_v1[n, np1] == 0 or pl_v1[n, np1] == n:
                                pl_v1[n, np1] = n
                                pl_v1[np1, n] = n
                            x_v1[n].append((t21, ta2_avail, 1, b1))
                            x_v1[n].append((ta2_avail, tw1, 2, a2))
                            ra_v1[1, n] = ta2_avail + Cmat[1, n] - tw1
                            ra_v1[2, n] = Cmat[2, n]
                        else:
                            _, tb1 = check_resc_occupation(
                                NODES, valid_b1, b1, ta_bar_node, V_v1,
                                n, tw1, tw, rs_v1)
                            tb1, blkrb = check_x(tb1, 0, b1, x_v1, n, Cmat, N)
                            if blkrb >= 0 and blkrb != n:
                                if pl_v1[n, blkrb] == 0 or pl_v1[n, blkrb] == blkrb:
                                    pl_v1[n, blkrb] = blkrb
                                    pl_v1[blkrb, n] = blkrb
                            pl_v1 = _relock_vtemp(pl_v1, V_v1, n, b1)

                            if tb1 <= t21 + 1e-5:
                                x_v1[n].append((t21, ta2_avail, 1, b1))
                                x_v1[n].append((ta2_avail, tw1, 2, a2))
                                ra_v1[1, n] = ta2_avail + Cmat[1, n] - tw1
                                ra_v1[2, n] = Cmat[2, n]
                            else:
                                valid_b1_2 = trace_valid_nodes(l, tw, t21, NODES)
                                _, tb1_2 = check_resc_occupation(
                                    NODES, valid_b1_2, b1, l, V_v1,
                                    n, tw1, tw, rs_v1)
                                x_v1[n].append(
                                    (tb1_2, min(tb1_2 + Cmat[0, n], tw1), 1, b1))
                                x_v1[n].append(
                                    (min(tb1_2 + Cmat[0, n], tw1), tw1, 2, a2))
                                ra_v1[0, n] = max(0.0, tb1_2 + Cmat[0, n] - tw1)
                                ra_v1[1, n] = min(
                                    tb1_2 + Cmat[0, n] - tw1 + Cmat[1, n], Cmat[1, n])
                                ra_v1[2, n] = Cmat[2, n]

                        next_branches.append((ra_v1, V_v1, x_v1, pl_v1, rs_v1, tw1))

        branches = next_branches

    return branches


# ===========================================================================
# prune_nodes_by_ni
# ===========================================================================

def prune_nodes_by_ni(NODES, OPEN):
    """
    Equivalent of MATLAB prune_nodes_by_ni.
    For nodes with all remaining times near 0, keep only the one with min f
    among those with the same ni vector.
    Returns pruned OPEN list.
    """
    ni_groups = defaultdict(list)   # key: ni tuple → list of (idx, f)

    for idx in OPEN:
        node = NODES[idx]
        if np.all(node.r <= 1e-5):
            key = tuple(node.ni.tolist())
            ni_groups[key].append((idx, node.f))

    to_remove = set()
    for key, group in ni_groups.items():
        if len(group) >= 2:
            best_i = int(np.argmin([f for _, f in group]))
            for i, (idx, _) in enumerate(group):
                if i != best_i:
                    to_remove.add(idx)

    return [idx for idx in OPEN if idx not in to_remove]
