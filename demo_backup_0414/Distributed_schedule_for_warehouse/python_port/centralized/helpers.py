"""
Small utility functions shared by the centralized port.

Ports of:
  tw1_basedon_U_temp.m
  trace_valid_nodes.m
  find_node_ta_bar.m
  check_x.m
  check_resc_occupation.m
  NextSigM_global.m
"""
from __future__ import annotations

from typing import List

import numpy as np

EPS = 1e-5
BIG_M = 1000.0


# ---------------------------------------------------------------------------
def tw1_basedon_U_temp(U_temp: np.ndarray, ra: np.ndarray,
                       da: np.ndarray, tw: float) -> float:
    """Compute the next significant moment given U_temp + remaining times."""
    N = U_temp.shape[0]
    if not np.any(U_temp):
        pos_d = da[da > EPS]
        if pos_d.size == 0:
            return tw
        return tw + float(np.min(pos_d))

    remain_time = np.full(N, np.inf)
    for n in range(N):
        m = np.where(U_temp[n, :] != 0)[0]
        if m.size == 0:
            continue
        s_temp = U_temp[n, m[0]]
        # s_temp is 1-indexed sub-task; ra row index is 0-indexed
        remain_time[n] = ra[s_temp - 1, n]

    pos_d = da[(da > EPS) & (da < BIG_M)]
    pos_r = remain_time[(remain_time > EPS) & np.isfinite(remain_time)]

    if pos_d.size == 0 and pos_r.size == 0:
        return tw
    if pos_d.size == 0:
        Lw = float(np.min(pos_r))
    elif pos_r.size == 0:
        Lw = float(np.min(pos_d))
    else:
        Lw = min(float(np.min(pos_d)), float(np.min(pos_r)))
    return tw + Lw


# ---------------------------------------------------------------------------
def trace_valid_nodes(curr_idx: int, t_curr: float, t1: float,
                      nodes: List) -> List[int]:
    """Walk the parent chain backward until tw <= t1.  Returns indices visited
    starting at curr_idx (newest first).  curr_idx is 0-indexed into nodes."""
    valid = [curr_idx]
    while t_curr > t1 + EPS:
        nd = nodes[curr_idx]
        parent = nd.parent
        if parent < 0:
            return valid
        t_parent = nodes[parent].tw
        valid.append(parent)
        curr_idx = parent
        t_curr   = t_parent
    return valid


def find_node_ta_bar(l: int, tw: float, t_avail: float,
                     nodes: List) -> tuple[int, float]:
    """Walk back until tw <= t_avail.  Returns (node_idx, tw_at_that_node)."""
    ta_bar_node = l
    ta_bar      = tw
    while ta_bar > t_avail + EPS:
        parent = nodes[ta_bar_node].parent
        if parent < 0:
            return ta_bar_node, ta_bar
        ta_bar_node = parent
        ta_bar      = nodes[ta_bar_node].tw
    return ta_bar_node, ta_bar


# ---------------------------------------------------------------------------
def check_x(t_avail: float, s: int, m: int, x: List, n: int,
            ni2: np.ndarray, task_cache: List[List]) -> tuple[float, int]:
    """Push t_avail forward past any conflicting reservation in x for space m
    using sub-task s of vehicle n.  Returns (new t_avail, blocker | 0).

    s is 1-indexed sub-task; m is 1-indexed global space id.
    Reservation tuples: (t_start, t_end, s_other, m_other).
    """
    N = len(x)
    ti_n = ni2[n] - 1   # 0-indexed task id
    Cvec = task_cache[n][ti_n].C  # length Smax

    blocker = 0
    Cs = Cvec[s - 1]
    for p in range(N):
        if p == n or not x[p]:
            continue
        for ti in range(len(x[p])):
            recs = x[p][ti]
            if not recs:
                continue
            for row in recs:
                t_start, t_end, _s_other, m_other = row
                if m_other != m:
                    continue
                # overlap with [t_avail, t_avail + Cs]
                if not (t_end <= t_avail or t_start >= t_avail + Cs):
                    t_avail = t_end
                    blocker = p
    return t_avail, blocker


# ---------------------------------------------------------------------------
def check_resc_occupation(nodes: List, valid_nodes: List[int], resc1: int,
                          l: int, U_temp: np.ndarray, n: int,
                          tw1: float, tw: float,
                          reset_since: np.ndarray | None) -> tuple[int, float]:
    """Find earliest time space `resc1` is free for vehicle n, scanning the
    parent chain.  Returns (last_valid_idx, t_avail).
    """
    N_sys = U_temp.shape[0]
    others = [j for j in range(N_sys) if j != n]
    last_valid_idx = -1
    t_avail = tw1

    if reset_since is None:
        reset_since = np.zeros(N_sys)

    if all(U_temp[j, resc1 - 1] == 0 for j in others):
        last_valid_idx = l
        t_avail        = tw

        for i in range(len(valid_nodes) - 1):
            idx = valid_nodes[i]
            U   = nodes[idx].U_temp
            tw_idx = nodes[idx].tw

            any_full_occ = False
            reset_cap    = float("inf")
            for j in others:
                if U[j, resc1 - 1] == 0:
                    continue
                if reset_since[j] > 0 and reset_since[j] <= tw_idx:
                    continue
                if reset_since[j] > 0 and reset_since[j] > tw_idx:
                    reset_cap = min(reset_cap, float(reset_since[j]))
                else:
                    any_full_occ = True

            if not any_full_occ and reset_cap == float("inf"):
                last_valid_idx = idx
                parent = nodes[last_valid_idx].parent
                if parent < 0:
                    t_avail = 0.0
                else:
                    t_avail = nodes[parent].tw
            elif not any_full_occ and reset_cap < float("inf"):
                t_avail = reset_cap
                break
            else:
                break

    return last_valid_idx, t_avail


# ---------------------------------------------------------------------------
def next_sig_m_global(tw: float, da: np.ndarray, ra: np.ndarray,
                      oa: np.ndarray, U_temp: np.ndarray):
    """Advance state from tw to the next significant moment.

    Returns: d2, r2, o2, tw1
    """
    Smax, N = ra.shape
    r2 = np.zeros_like(ra)
    o2 = np.zeros_like(oa)
    d2 = np.zeros_like(da)

    if not np.any(U_temp):
        pos_d = da[(da > EPS) & np.isfinite(da)]
        if pos_d.size == 0:
            return da.copy(), ra.copy(), oa.copy(), tw
        Lw = float(np.min(pos_d))
    else:
        remain_time = np.full(N, np.inf)
        for n in range(N):
            cols = np.where(U_temp[n, :] != 0)[0]
            if cols.size == 0:
                continue
            s_temp = U_temp[n, cols[0]]
            remain_time[n] = ra[s_temp - 1, n]

        pos_d = da[(da > EPS) & (da < BIG_M)]
        pos_r = remain_time[(remain_time > EPS) & np.isfinite(remain_time)]

        if pos_d.size == 0 and pos_r.size == 0:
            return da.copy(), ra.copy(), oa.copy(), tw
        if pos_d.size == 0:
            Lw = float(np.min(pos_r))
        elif pos_r.size == 0:
            Lw = float(np.min(pos_d))
        else:
            Lw = min(float(np.min(pos_d)), float(np.min(pos_r)))

    tw1 = tw + Lw
    t   = Lw

    for n in range(N):
        d2[n] = da[n] - t
        cols = np.where(U_temp[n, :] != 0)[0]
        if cols.size > 0:
            s_temp = U_temp[n, cols[0]]
            r2[:, n] = ra[:, n]
            r2[s_temp - 1, n] = ra[s_temp - 1, n] - t
            o2[n] = oa[n] + t
        else:
            r2[:, n] = ra[:, n]
            o2[n] = oa[n] + np.sign(np.sum(ra[:, n])) * t

    # numerical cleanup
    r2[np.abs(r2) <= EPS] = 0
    d2[np.abs(d2) <= EPS] = 0
    o2[np.abs(o2) <= EPS] = 0
    return d2, r2, o2, tw1
