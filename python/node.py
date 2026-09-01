"""
Node dataclass for the intersection scheduling decision tree.

Indexing conventions (0-based throughout):
  - vehicle n  : 0 .. N-1   (MATLAB: 1..N)
  - slot s     : 0 .. S-1   (MATLAB: 1..S)
  - space label m : 1-indexed integer value stored in MapMat (physical ID, not array index)
  - task kn    : always 0   (MATLAB: kn=1; each vehicle has exactly one task per agent)

Node fields correspond to MATLAB NODES{idx}{k}:
  {1}  idx        node index (1-based root to keep parent-link arithmetic simple)
  {2}  d          deadline remaining   shape (N,)
  {3}  r          slot remaining times  shape (S, N)
  {4}  o          overtime              shape (N,)
  {5}  tw         time window start     float
  {6}  ni         task counter          shape (N,) int
  {7}  parent     parent node index (0 = root has no parent)
  {8}  U_c        contention matrix     shape (N, M)
  {9}  U_temp     selected assignment   shape (N, M)
  {10} g          g-cost                float
  {11} gamma      list[N] of float|None  exit time estimate per vehicle
  {12} f          f-cost (= g, h=0 for BFS)
  {13} speed      speed records (unused)
  {14} ra_reset   reset remaining times shape (S, N)
  {15} x          list[N] of list[(t_start,t_end,s,m)]  resource records
  {16} alpha      list[N] of float|None  entry time per vehicle
  {17} pair_lock  weak-rule matrix      shape (N, N)
  {18} reset_since reset timestamps     shape (N,)
"""

from dataclasses import dataclass
import numpy as np
import copy


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


def make_node(node_count, d2, r2, o2, tw1, ni2, parent_idx,
              U_c, U_temp, g, gamma, speed, ra, ra_reset, x,
              Cmat, valid_systems, alpha, arrival_ref, const,
              pair_lock=None, reset_since=None):
    """
    Equivalent of MATLAB NewNode.m.
    All arrays 0-indexed.  gamma/alpha/x are deep-copied before mutation.
    """
    N = const['N']
    S = Cmat.shape[0]

    gamma = copy.deepcopy(gamma)
    alpha = copy.deepcopy(alpha)

    g_n = np.zeros(N)
    for n in valid_systems:
        if np.sum(ra[:, n]) > 1e-5 and np.sum(r2[:, n]) <= 1e-5:
            gamma[n] = tw1
            alpha[n] = tw1 - np.sum(Cmat[:, n])
            if arrival_ref[n] is not None and not np.isnan(arrival_ref[n]):
                g_n[n] = alpha[n] - arrival_ref[n]

    g_new = g + float(np.sum(g_n))
    f_new = g_new

    if pair_lock is None:
        pair_lock = np.zeros((N, N))
    if reset_since is None:
        reset_since = np.zeros(N)

    return Node(
        idx=node_count + 1,
        d=d2.copy(), r=r2.copy(), o=o2.copy(),
        tw=float(tw1), ni=ni2.copy(),
        parent=parent_idx,
        U_c=U_c.copy(), U_temp=U_temp.copy(),
        g=g_new, gamma=gamma, f=f_new,
        speed=list(speed) if speed else [],
        ra_reset=ra_reset.copy(),
        x=copy.deepcopy(x), alpha=alpha,
        pair_lock=pair_lock.copy(),
        reset_since=reset_since.copy(),
    )
