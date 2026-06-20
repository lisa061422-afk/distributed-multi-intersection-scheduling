"""
resetting_rule_global: handle interrupted vehicles, return list of branches.

Port of resetting_rule_global.m.  Returns a list of dicts:
   ra_temp, U_temp, x, pair_lock, reset_since, tw1
"""
from __future__ import annotations

from typing import List

import numpy as np

from .helpers import (EPS, tw1_basedon_U_temp, trace_valid_nodes,
                      find_node_ta_bar, check_x, check_resc_occupation)
from .node import clone_x


def resetting_rule_global(ra: np.ndarray, ra_temp: np.ndarray,
                          U_temp: np.ndarray, U_c: np.ndarray,
                          tw: float, da: np.ndarray, nodes: List, l: int,
                          x: list, ni2: np.ndarray, ctx: dict, const: dict,
                          pair_lock: np.ndarray,
                          reset_since: np.ndarray) -> List[dict]:
    N      = const["N"]
    Cvec   = ctx["Cvec"]    # (Smax, N)
    MapVec = ctx["MapVec"]
    task_cache = const["task_cache"]
    Smax   = Cvec.shape[0]

    tw1 = tw1_basedon_U_temp(U_temp, ra, da, tw)

    # interrupted = ra has work AND U_temp row is all zero
    interrupted = []
    for n in range(N):
        if np.any(ra[:, n] > EPS) and not np.any(U_temp[n, :]):
            interrupted.append(n)

    branches = [{
        "ra_temp":     ra_temp.copy(),
        "U_temp":      U_temp.copy(),
        "x":           clone_x(x),
        "pair_lock":   pair_lock.copy(),
        "reset_since": reset_since.copy(),
        "tw1":         tw1,
    }]

    for n in interrupted:
        next_branches: List[dict] = []
        for b in branches:
            ra_b = b["ra_temp"].copy()
            V_b  = b["U_temp"].copy()
            x_b  = clone_x(b["x"])
            pl_b = b["pair_lock"].copy()
            rs_b = b["reset_since"].copy()

            # discard stale reservations for this interrupted task
            ti_n = ni2[n] - 1
            if 0 <= ti_n < len(x_b[n]):
                x_b[n][ti_n] = []
            rs_b[n] = tw

            # locate first nonzero in U_c[n,:]
            cols = np.where(U_c[n, :] > 0)[0]
            if cols.size == 0:
                next_branches.append({
                    "ra_temp": ra_b, "U_temp": V_b, "x": x_b,
                    "pair_lock": pl_b, "reset_since": rs_b, "tw1": tw1,
                })
                continue
            s_int = int(U_c[n, cols[0]])

            # ----- sub 1 -----
            if s_int == 1:
                ra_b[:, n] = 0
                ra_b[:Smax, n] = Cvec[:, n]
                pl_b = _relock_Vtemp(pl_b, V_b, n, int(MapVec[0, n]))
                next_branches.append({
                    "ra_temp": ra_b, "U_temp": V_b, "x": x_b,
                    "pair_lock": pl_b, "reset_since": rs_b, "tw1": tw1,
                })

            # ----- sub 2 -----
            elif s_int == 2:
                a1       = int(MapVec[0, n])
                t1_start = tw1 - Cvec[0, n]
                valid_a1 = trace_valid_nodes(l, tw, t1_start, nodes)

                vars_ = _space_variants(V_b, a1, n, pl_b, rs_b)
                for V_vi, displaced in vars_:
                    ra_vi = ra_b.copy()
                    x_vi  = clone_x(x_b)
                    pl_vi = pl_b.copy()
                    rs_vi = rs_b.copy()

                    if displaced is not None:
                        np_ = displaced
                        ra_vi[:, np_] = 0
                        ra_vi[:Smax, np_] = Cvec[:, np_]
                        ti_np = ni2[np_] - 1
                        if 0 <= ti_np < len(x_vi[np_]):
                            x_vi[np_][ti_np] = []
                        rs_vi[np_] = tw
                        if pl_vi[n, np_] == -1 or pl_vi[n, np_] == n:
                            pl_vi[n, np_] = n
                            pl_vi[np_, n] = n
                        x_vi[n][ti_n].append((tw1 - Cvec[0, n], tw1, 1, a1))
                        ra_vi[0, n] = 0
                        ra_vi[1:Smax, n] = Cvec[1:Smax, n]
                    else:
                        _, tb1 = check_resc_occupation(nodes, valid_a1, a1, l,
                                                       V_vi, n, tw1, tw, rs_vi)
                        tb1, blkr = check_x(tb1, 1, a1, x_vi, n, ni2, task_cache)
                        if blkr > 0 and blkr != n:
                            if pl_vi[n, blkr] == -1 or pl_vi[n, blkr] == blkr:
                                pl_vi[n, blkr] = blkr
                                pl_vi[blkr, n] = blkr
                        pl_vi = _relock_Vtemp(pl_vi, V_vi, n, a1)

                        if tb1 <= t1_start + EPS:
                            x_vi[n][ti_n].append((tw1 - Cvec[0, n], tw1, 1, a1))
                            ra_vi[0, n] = 0
                        else:
                            x_vi[n][ti_n].append((tb1, tb1 + Cvec[0, n], 1, a1))
                            ra_vi[0, n] = tb1 + Cvec[0, n] - tw1
                        ra_vi[1:Smax, n] = Cvec[1:Smax, n]

                    next_branches.append({
                        "ra_temp": ra_vi, "U_temp": V_vi, "x": x_vi,
                        "pair_lock": pl_vi, "reset_since": rs_vi, "tw1": tw1,
                    })

            # ----- sub 3 -----
            elif s_int == 3:
                a2 = int(MapVec[1, n])
                b1 = int(MapVec[0, n])
                t22      = tw1 - Cvec[1, n]
                valid_a2 = trace_valid_nodes(l, tw, t22, nodes)

                vars_a2 = _space_variants(V_b, a2, n, pl_b, rs_b)
                for V_v2, disp2 in vars_a2:
                    ra_v2 = ra_b.copy()
                    x_v2  = clone_x(x_b)
                    pl_v2 = pl_b.copy()
                    rs_v2 = rs_b.copy()

                    if disp2 is not None:
                        np2 = disp2
                        ra_v2[:, np2] = 0
                        ra_v2[:Smax, np2] = Cvec[:, np2]
                        ti_np2 = ni2[np2] - 1
                        if 0 <= ti_np2 < len(x_v2[np2]):
                            x_v2[np2][ti_np2] = []
                        rs_v2[np2] = tw
                        if pl_v2[n, np2] == -1 or pl_v2[n, np2] == n:
                            pl_v2[n, np2] = n
                            pl_v2[np2, n] = n
                        ta2_avail = tw1
                    else:
                        _, ta2 = check_resc_occupation(nodes, valid_a2, a2, l,
                                                      V_v2, n, tw1, tw, rs_v2)
                        ta2_avail = max(ta2, t22)
                        ta2_avail, blkr2 = check_x(ta2_avail, 2, a2, x_v2, n,
                                                   ni2, task_cache)
                        if blkr2 > 0 and blkr2 != n:
                            if pl_v2[n, blkr2] == -1 or pl_v2[n, blkr2] == blkr2:
                                pl_v2[n, blkr2] = blkr2
                                pl_v2[blkr2, n] = blkr2
                        pl_v2 = _relock_Vtemp(pl_v2, V_v2, n, a2)

                    t21 = ta2_avail - Cvec[0, n]
                    ta_bar_node, ta_bar = find_node_ta_bar(l, tw, ta2_avail, nodes)
                    valid_b1 = trace_valid_nodes(ta_bar_node, ta_bar, t21, nodes)

                    vars_b1 = _space_variants(V_v2, b1, n, pl_v2, rs_v2)
                    for V_v1, disp1 in vars_b1:
                        ra_v1 = ra_v2.copy()
                        x_v1  = clone_x(x_v2)
                        pl_v1 = pl_v2.copy()
                        rs_v1 = rs_v2.copy()

                        if disp1 is not None:
                            np1 = disp1
                            ra_v1[:, np1] = 0
                            ra_v1[:Smax, np1] = Cvec[:, np1]
                            ti_np1 = ni2[np1] - 1
                            if 0 <= ti_np1 < len(x_v1[np1]):
                                x_v1[np1][ti_np1] = []
                            rs_v1[np1] = tw
                            if pl_v1[n, np1] == -1 or pl_v1[n, np1] == n:
                                pl_v1[n, np1] = n
                                pl_v1[np1, n] = n
                            x_v1[n][ti_n].append((t21, ta2_avail, 1, b1))
                            x_v1[n][ti_n].append((ta2_avail, tw1, 2, a2))
                            ra_v1[1, n] = ta2_avail + Cvec[1, n] - tw1
                            ra_v1[2, n] = Cvec[2, n]
                        else:
                            _, tb1 = check_resc_occupation(nodes, valid_b1, b1,
                                                           ta_bar_node, V_v1, n,
                                                           tw1, tw, rs_v1)
                            tb1, blkrb = check_x(tb1, 1, b1, x_v1, n, ni2, task_cache)
                            if blkrb > 0 and blkrb != n:
                                if pl_v1[n, blkrb] == -1 or pl_v1[n, blkrb] == blkrb:
                                    pl_v1[n, blkrb] = blkrb
                                    pl_v1[blkrb, n] = blkrb
                            pl_v1 = _relock_Vtemp(pl_v1, V_v1, n, b1)

                            if tb1 <= t21 + EPS:
                                x_v1[n][ti_n].append((t21, ta2_avail, 1, b1))
                                x_v1[n][ti_n].append((ta2_avail, tw1, 2, a2))
                                ra_v1[1, n] = ta2_avail + Cvec[1, n] - tw1
                                ra_v1[2, n] = Cvec[2, n]
                            else:
                                valid_b1_2 = trace_valid_nodes(l, tw, t21, nodes)
                                _, tb1_2 = check_resc_occupation(nodes, valid_b1_2,
                                                                 b1, l, V_v1, n,
                                                                 tw1, tw, rs_v1)
                                end_b1 = min(tb1_2 + Cvec[0, n], tw1)
                                x_v1[n][ti_n].append((tb1_2, end_b1, 1, b1))
                                x_v1[n][ti_n].append((end_b1, tw1, 2, a2))
                                ra_v1[0, n] = max(0.0, tb1_2 + Cvec[0, n] - tw1)
                                ra_v1[1, n] = min(tb1_2 + Cvec[0, n] - tw1 + Cvec[1, n], Cvec[1, n])
                                ra_v1[2, n] = Cvec[2, n]

                        next_branches.append({
                            "ra_temp": ra_v1, "U_temp": V_v1, "x": x_v1,
                            "pair_lock": pl_v1, "reset_since": rs_v1, "tw1": tw1,
                        })

        branches = next_branches

    return branches


# ---------------------------------------------------------------------------
def _space_variants(V_temp: np.ndarray, m: int, n: int,
                    pair_lock: np.ndarray, reset_since: np.ndarray):
    """Generate (V_temp_variant, displaced_n_or_None) tuples for n needing space m.

    Mirrors space_variants() inside resetting_rule.m (distributed reference).
    """
    occ = np.where(V_temp[:, m - 1] > 0)[0]
    occ = occ[occ != n]
    if occ.size == 0:
        return [(V_temp.copy(), None)]
    np_ = int(occ[0])
    lock_val = int(pair_lock[n, np_])
    out = []

    if lock_val == -1 or lock_val == np_:
        out.append((V_temp.copy(), None))

    if lock_val == n:
        V_B = V_temp.copy()
        V_B[np_, m - 1] = 0
        out.append((V_B, np_))

    if lock_val == -1 and reset_since.size > np_ and reset_since[np_] > 0:
        V_B = V_temp.copy()
        V_B[np_, m - 1] = 0
        out = [(V_B, np_)]   # displace only

    if not out:
        out = [(V_temp.copy(), None)]
    return out


def _relock_Vtemp(pl: np.ndarray, V_temp: np.ndarray, n: int, m: int) -> np.ndarray:
    """Record that n lost to V_temp's current occupier of space m (1-indexed)."""
    if m <= 0:
        return pl
    occ = np.where(V_temp[:, m - 1] > 0)[0]
    occ = occ[occ != n]
    if occ.size == 0:
        return pl
    o = int(occ[0])
    if pl[n, o] == -1 or pl[n, o] == o:
        pl[n, o] = o
        pl[o, n] = o
    return pl
