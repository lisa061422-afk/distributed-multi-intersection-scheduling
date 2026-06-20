"""
Node dataclass + new-node builder for the centralized scheduling tree.

Mirrors MATLAB NODES{idx}{...}:
   .idx        node index (0-indexed)
   .d          (N,) deadline remaining
   .r          (Smax, N) sub-task remaining time
   .o          (N,) overtime
   .tw         scalar
   .ni         (N,) int   current task counter
   .parent     int   parent index (-1 if root)
   .U_c        (N, M)
   .U_temp     (N, M)
   .g          float
   .gamma      list[N] of np.ndarray(NI(n))   (None entries → NaN)
   .f          float
   .speed      list[N] of float
   .ra_reset   (Smax, N)
   .x          list[N] of list[NI(n)] of list of (t_start, t_end, s, m) tuples
   .alpha      list[N] of np.ndarray(NI(n))   (per-task generation time)
   .pair_lock  (N, N)
   .reset_since (N,)
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# ---------------------------------------------------------------------------
# Structural clone helpers (replace copy.deepcopy hot-path uses).
# These exploit the fact that:
#   * gamma / alpha entries are np.ndarray  →  arr.copy() is a fast C call
#   * x[n][ti] entries are immutable tuples →  share them, only rebuild the
#     outer list spine that the algorithm mutates in place.
# ---------------------------------------------------------------------------
def clone_arr_list(lst):
    """Copy each np.ndarray in `lst` (or pass through None)."""
    return [None if a is None else a.copy() for a in lst]


def clone_x(x):
    """Clone x = list[N] of list[NI(n)] of list-of-tuple records.

    The records themselves are tuples (immutable, safely shared); only the
    outer per-vehicle and per-task list spines are rebuilt so the caller can
    mutate `x[n][ti] = []` or `.append(...)` without aliasing the parent.
    """
    return [[list(per_task) for per_task in per_vehicle] for per_vehicle in x]


@dataclass
class Node:
    idx: int
    d: np.ndarray
    r: np.ndarray
    o: np.ndarray
    tw: float
    ni: np.ndarray
    parent: int
    U_c: np.ndarray
    U_temp: np.ndarray
    g: float
    gamma: list
    f: float
    speed: list
    ra_reset: np.ndarray
    x: list
    alpha: list
    pair_lock: np.ndarray
    reset_since: np.ndarray


def make_node(parent_idx: int,
              d2: np.ndarray, r2: np.ndarray, o2: np.ndarray,
              tw1: float, ni2: np.ndarray,
              U_c: np.ndarray, U_temp: np.ndarray,
              g_prev: float, gamma_in: list, speed: list,
              ra: np.ndarray, ra_reset: np.ndarray,
              x_in: list, alpha_in: list,
              const, pair_lock: np.ndarray, reset_since: np.ndarray) -> Node:
    """Build a new Node packet.  Updates gamma/g/d2 the same way NewNode_global.m
    does: when a vehicle finishes its current task at tw1, set gamma[n][ni-1]=tw1,
    add (start_time - alpha[n][ni-1]) to g, and reset d[n] to const['Dt'].

    The returned Node has idx = parent_idx-placeholder (-1).  The caller (BFS or
    DFS driver) assigns the global index after merge.
    """
    N        = const["N"]
    NI       = const["NI"]
    task_cache = const["task_cache"]
    Dt       = const["Dt"]

    gamma = clone_arr_list(gamma_in)
    alpha = clone_arr_list(alpha_in)
    x     = clone_x(x_in)
    d2    = d2.copy()

    g_n_total = 0.0
    for n in range(N):
        if np.sum(ra[:, n]) > 1e-5 and np.sum(r2[:, n]) <= 1e-5 and ni2[n] >= 1:
            ti = ni2[n] - 1
            task = task_cache[n][ti]
            gamma[n][ti] = tw1
            c_total = float(np.sum(task.C[task.C > 0]))
            start_time = tw1 - c_total
            g_n_total += start_time - alpha[n][ti]
            if ni2[n] < NI[n]:
                d2[n] = Dt
            else:
                d2[n] = np.inf

    g_new = g_prev + g_n_total
    f_new = g_new

    return Node(
        idx=-1,
        d=d2,
        r=r2.copy(),
        o=o2.copy(),
        tw=float(tw1),
        ni=ni2.copy(),
        parent=parent_idx,
        U_c=U_c.copy(),
        U_temp=U_temp.copy(),
        g=g_new,
        gamma=gamma,
        f=f_new,
        speed=list(speed),
        ra_reset=ra_reset.copy(),
        x=x,
        alpha=alpha,
        pair_lock=pair_lock.copy(),
        reset_since=reset_since.copy(),
    )
