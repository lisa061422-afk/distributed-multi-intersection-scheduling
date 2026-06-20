"""
FCFS (First-Come-First-Served) baseline scheduler in the centralized framework.

Mirrors the FCFS branch logic in MATLAB ``expand_array_global2.m`` lines
111-251:
    1) Tasks already executing continue to execute.
    2) Released-but-not-started tasks remain waiting.
    3) A waiting task can start only if every currently-executing task in the
       same intersection has released all shared conflict spaces.
    4) Among simultaneously admissible waiting tasks, order:
       first by alpha (release time of the current sub-task), then by system
       index.

The implementation reuses the expand_pure setup (state propagation, U_c
construction, ctx) but replaces the branching contention logic with a single
deterministic U_temp pick. No WeakRule, no pair_lock, no preemption.

Output shape matches search_dfs_bb so the comparison harness can swap solvers
freely: ``(nodes, leaves, best_idx, best_g, log)``.
"""
from __future__ import annotations

import math
import time
from typing import List, Tuple

import numpy as np

from .helpers import EPS, BIG_M, next_sig_m_global
from .node import Node, clone_arr_list, make_node
from .expand import _update_vtemp_x
from .resetting_rule import resetting_rule_global


def _expand_fcfs_one(nodes: List[Node], c_idx: int, const: dict
                     ) -> Tuple[Node | None, bool]:
    """Single-branch expansion. Returns (child, is_leaf)."""
    N         = const["N"]
    NI        = const["NI"]
    Smax      = const["Smax"]
    Mtot      = const["Mtot"]
    task_cache = const["task_cache"]
    space_per_int = const["space_per_int"]

    c     = nodes[c_idx]
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

    # ---- propagate state to tw (mirrors expand_pure) ----
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
        return None, True

    # ---- find active systems and build U_c ----
    active = [n for n in range(N) if np.any(ra[:, n] > EPS)]
    s_idx_per_n: dict[int, int] = {}    # 0-indexed sub-task row of n's first nonzero ra
    for n in active:
        s_idx_arr = np.where(ra[:, n] > EPS)[0]
        if s_idx_arr.size == 0:
            continue
        s_idx = int(s_idx_arr[0])
        s_idx_per_n[n] = s_idx
        ti = ni2[n] - 1
        task = task_cache[n][ti]
        m1 = int(task.Map[s_idx])      # 1-indexed global space
        U_c[n, m1 - 1] = s_idx + 1     # 1-indexed sub-task

    # ---- ctx for x update ----
    Cvec   = np.zeros((Smax, N))
    MapVec = np.zeros((Smax, N), dtype=int)
    for n in range(N):
        if ni2[n] >= 1 and ni2[n] <= NI[n]:
            ti = ni2[n] - 1
            task = task_cache[n][ti]
            L = task.L
            Cvec[:L, n]   = task.C[:L]
            MapVec[:L, n] = task.Map[:L]

    # ---- no active or no contention: trivial single child ----
    col_cnt = (U_c > 0).sum(axis=0)
    if not active or np.all(ra <= EPS):
        U_temp = np.zeros((N, Mtot), dtype=int)
        d2, r2, o2, tw1 = next_sig_m_global(tw, da, ra, oa, U_temp)
        ra_reset = -1 * np.ones((Smax, N))
        new_node = make_node(l, d2, r2, o2, tw1, ni2, U_c, U_temp,
                             g, gamma, speed, ra, ra_reset, x, alpha,
                             const, pair_lock, reset_since)
        return new_node, False

    if np.all(col_cnt <= 1):
        U_temp = U_c.copy()
        d2, r2, o2, tw1 = next_sig_m_global(tw, da, ra, oa, U_temp)
        ra_reset = -1 * np.ones((Smax, N))
        x_clean = _update_vtemp_x(x, U_temp, ni2, r2, tw1, Cvec, N)
        new_node = make_node(l, d2, r2, o2, tw1, ni2, U_c, U_temp,
                             g, gamma, speed, ra, ra_reset, x_clean, alpha,
                             const, pair_lock, reset_since)
        return new_node, False

    # ---- contention: per-column FCFS pick + resetting_rule first-branch ----
    # FCFS rule: for each contended column, the winner is the vehicle with the
    # smallest alpha[n][current_task_index] (release time of its current task);
    # ties broken by system index. Uncontended columns pass through unchanged.
    # Then resetting_rule_global handles displaced/interrupted vehicles; FCFS
    # commits to its first branch (no exploration).
    U_temp_pick = np.zeros_like(U_c)

    def _alpha_key(n: int) -> Tuple[float, int]:
        ti = ni2[n] - 1
        a = alpha[n]
        if a is None or ti >= a.shape[0] or np.isnan(a[ti]):
            return (math.inf, n)
        return (float(a[ti]), n)

    for m_col in range(Mtot):
        rows = np.where(U_c[:, m_col] > 0)[0]
        if rows.size == 0:
            continue
        if rows.size == 1:
            U_temp_pick[rows[0], m_col] = U_c[rows[0], m_col]
            continue
        # contended — FCFS winner
        winner = min(rows.tolist(), key=_alpha_key)
        U_temp_pick[winner, m_col] = U_c[winner, m_col]

    # Hand off to resetting_rule_global for interrupted-vehicle bookkeeping.
    # Take its first branch — FCFS does not explore alternatives.
    ctx = {"Cvec": Cvec, "MapVec": MapVec}
    ra_temp_init = ra.copy()
    branches = resetting_rule_global(ra, ra_temp_init, U_temp_pick, U_c, tw, da,
                                     nodes, c_idx, x, ni2, ctx, const,
                                     pair_lock, reset_since)
    if not branches:
        return None, True

    br = branches[0]
    ra_t2  = br["ra_temp"]
    U_t2   = br["U_temp"]
    x2     = br["x"]
    pl_upd = br["pair_lock"]
    rs2    = br["reset_since"]

    d2, r2, o2, tw1 = next_sig_m_global(tw, da, ra_t2, oa, U_t2)

    if tw1 <= tw + 1e-12:
        return None, True

    x2 = _update_vtemp_x(x2, U_t2, ni2, r2, tw1, Cvec, N)
    ra_reset = ra_t2.copy()
    new_node = make_node(l, d2, r2, o2, tw1, ni2, U_c, U_t2,
                         g, gamma, speed, ra, ra_reset, x2, alpha,
                         const, pl_upd, rs2)
    return new_node, False


def search_fcfs(root: Node, const: dict,
                deadline: float | None = None,
                verbose: bool = True
                ) -> Tuple[List[Node], List[int], int, float, list]:
    """Single-chain FCFS search — same return shape as search_dfs_bb."""
    nodes: List[Node] = [root]
    root.idx = 0
    log: list = []
    cur = 0
    step = 0
    t0 = time.perf_counter()
    deadline_hit = False

    while True:
        step += 1
        if deadline is not None and (time.perf_counter() - t0) > deadline:
            deadline_hit = True
            break

        child, is_leaf = _expand_fcfs_one(nodes, cur, const)
        if is_leaf:
            best_g = nodes[cur].g
            msg = f"[FCFS] leaf at node {cur}, g = {best_g:.6f}"
            log.append(msg)
            if verbose:
                print(msg)
            return nodes, [cur], cur, best_g, log

        if child is None:
            # Stall — treat as leaf with current cost
            best_g = nodes[cur].g
            log.append(f"[FCFS] stall at node {cur}, g = {best_g:.6f}")
            if verbose:
                print(log[-1])
            return nodes, [cur], cur, best_g, log

        child.idx = len(nodes)
        nodes.append(child)
        cur = child.idx

    suffix = " (deadline hit)" if deadline_hit else ""
    summary = f"[FCFS] done. expanded={step}, nodes={len(nodes)}{suffix}"
    log.append(summary)
    if verbose:
        print(summary)
    return nodes, [], -1, math.inf, log
