"""
Equivalent of MATLAB buildMutualExclusionConstraints_numeric.m

Returns (Aineq, bineq) such that   Aineq @ x_vec <= bineq
where x_vec is a compact vector indexed over valid_systems via idx_map.

All array dimensions use 0-based indexing:
  n   : vehicle index  0..N-1
  s   : slot index     0..S-1
  m   : space label    1-indexed integer (physical ID stored in MapMat)
"""

import numpy as np


def build_mutual_exclusion_constraints(
        MapMat, Cmat, valid_systems, gamma, idx_map, C_vec, eps_gap=1e-4):
    """
    Parameters
    ----------
    MapMat       : ndarray (S, N)   space labels per slot per vehicle (0 = unused)
    Cmat         : ndarray (S, N)   execution durations
    valid_systems: list of int      0-indexed vehicle IDs present at this agent
    gamma        : list[N]          gamma[n] = float exit-time estimate, or None
    idx_map      : ndarray (N,)     idx_map[n] = column in x_vec (-1 if absent)
    C_vec        : ndarray (N,)     C_n = gamma_n - alpha_n per vehicle
    eps_gap      : float            ordering slack (default 1e-4)

    Returns
    -------
    Aineq : ndarray (nc, nv)
    bineq : ndarray (nc,)
    """
    S, N = MapMat.shape
    nv = len(valid_systems)

    # Build ghat: estimated completion time per vehicle
    ghat = np.zeros(N)
    for n in valid_systems:
        if gamma[n] is not None and not np.isnan(gamma[n]):
            ghat[n] = float(gamma[n])

    # Collect all space labels used by valid systems
    all_spaces = []
    for n in valid_systems:
        seq = MapMat[:, n]
        all_spaces.extend(seq[seq > 0].astype(int).tolist())

    if not all_spaces:
        return np.zeros((0, nv)), np.zeros(0)

    unique_spaces, counts = np.unique(all_spaces, return_counts=True)
    contended = unique_spaces[counts >= 2]

    rows_A = []
    rows_b = []

    for mm in contended:
        # Collect (vehicle, slot_idx) pairs using space mm
        occ = []
        for n in valid_systems:
            s_list = np.where(MapMat[:, n] == mm)[0]  # 0-indexed slots
            for s in s_list:
                occ.append((n, int(s)))

        K = len(occ)
        if K <= 1:
            continue

        s_hat = np.zeros(K)
        tail_val = np.zeros(K)

        for k, (n, s) in enumerate(occ):
            last_nonzero = np.where(MapMat[:, n] > 0)[0]
            Sn = int(last_nonzero[-1]) if len(last_nonzero) > 0 else 0
            # tail_sum = sum of Cmat rows s+1 .. Sn  (0-indexed, inclusive)
            tail_sum = float(np.sum(Cmat[s + 1:Sn + 1, n]))
            tail_val[k] = tail_sum
            e_hat = ghat[n] - tail_sum
            s_hat[k] = e_hat - Cmat[s, n]

        order = np.argsort(s_hat)

        for r in range(K - 1):
            k1 = order[r];     n1, s1 = occ[k1]
            k2 = order[r + 1]; n2, s2 = occ[k2]

            # constraint: x(n1) - x(n2) <= rhs
            rhs = ((C_vec[n2] - tail_val[k2] - Cmat[s2, n2])
                   - (C_vec[n1] - tail_val[k1]) - eps_gap)

            row = np.zeros(nv)
            i1 = int(idx_map[n1])
            i2 = int(idx_map[n2])
            if i1 >= 0:
                row[i1] = 1.0
            if i2 >= 0:
                row[i2] = -1.0

            rows_A.append(row)
            rows_b.append(float(rhs))

    if not rows_A:
        return np.zeros((0, nv)), np.zeros(0)

    return np.array(rows_A), np.array(rows_b)
