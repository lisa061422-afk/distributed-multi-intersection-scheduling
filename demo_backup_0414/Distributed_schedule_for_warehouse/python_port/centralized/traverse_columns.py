"""
traverse_columns: enumerate U_temp candidates over contended columns.

Port of traverse_columns.m (centralized version with WeakRule + cycle-break).
"""
from __future__ import annotations

from typing import List, Tuple

import numpy as np

EPS = 1e-5


def traverse_columns(U_c: np.ndarray, priority_n: int = 0,
                     pair_lock: np.ndarray | None = None,
                     ra: np.ndarray | None = None
                     ) -> Tuple[List[np.ndarray], int, np.ndarray]:
    """
    Returns
    -------
    U_valid    : list of N×M selector matrices (one per branch)
    n_pruned   : number of branches skipped by WeakRule / cycle-break
    cb_updates : N×N matrix of forced pair_lock overwrites from cycle-break
    """
    N, M = U_c.shape
    if pair_lock is None:
        pair_lock = -1 * np.ones((N, N), dtype=int)
    # cb_updates uses the same sentinel: -1 means "no cycle-break override".
    cb_updates = -1 * np.ones_like(pair_lock)

    U_base = np.zeros_like(U_c)
    contended_cols: List[int] = []
    n_pruned = 0

    for m in range(M):
        rows = np.where(U_c[:, m] > 0)[0]

        if rows.size == 0:
            continue

        if rows.size == 1:
            U_base[rows[0], m] = U_c[rows[0], m]
            continue

        # priority override
        if priority_n > 0 and U_c[priority_n - 1, m] > 0:
            U_base[priority_n - 1, m] = U_c[priority_n - 1, m]
            n_pruned += rows.size - 1
            continue

        # WeakRule auto-win
        winner_lock = -1
        if ra is not None:
            for ci in range(rows.size):
                cand = rows[ci]
                s_cand = int(U_c[cand, m])
                if s_cand < 1 or s_cand > ra.shape[0] or ra[s_cand - 1, cand] <= EPS:
                    continue
                beats_all = True
                for oi in range(rows.size):
                    other = rows[oi]
                    if other == cand:
                        continue
                    if pair_lock[cand, other] != cand:
                        beats_all = False
                        break
                if beats_all:
                    winner_lock = cand
                    break

        if winner_lock >= 0:
            U_base[winner_lock, m] = U_c[winner_lock, m]
            n_pruned += rows.size - 1
            continue

        # cycle-break
        if _cycle_exists(pair_lock, rows):
            win_count = np.zeros(rows.size, dtype=int)
            for ci in range(rows.size):
                for oi in range(rows.size):
                    if ci != oi and pair_lock[rows[ci], rows[oi]] == rows[ci]:
                        win_count[ci] += 1
            max_w = win_count.max()
            cands = rows[win_count == max_w]
            winner_cb = int(cands.min())
            U_base[winner_cb, m] = U_c[winner_cb, m]
            n_pruned += rows.size - 1
            losers_cb = rows[rows != winner_cb]
            for ll in losers_cb:
                cb_updates[winner_cb, ll] = winner_cb
                cb_updates[ll, winner_cb] = winner_cb
            continue

        contended_cols.append(m)

    if not contended_cols:
        return [U_base.copy()], n_pruned, cb_updates

    out: List[np.ndarray] = []
    _recurse_contended(U_c, U_base, contended_cols, 0, out)
    return out, n_pruned, cb_updates


def _recurse_contended(U_c: np.ndarray, U_temp: np.ndarray,
                       cols: List[int], k: int, out: List[np.ndarray]) -> None:
    if k == len(cols):
        out.append(U_temp.copy())
        return
    m = cols[k]
    rows = np.where(U_c[:, m] > 0)[0]
    for n in rows:
        U_next = U_temp.copy()
        U_next[:, m] = 0
        U_next[n, m] = U_c[n, m]
        _recurse_contended(U_c, U_next, cols, k + 1, out)


def _cycle_exists(pair_lock: np.ndarray, rows: np.ndarray) -> bool:
    n = rows.size
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
