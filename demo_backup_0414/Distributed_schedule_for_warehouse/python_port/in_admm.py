"""
Equivalent of MATLAB IN_Admm.m

Solves the intersection agent's local QP over all decision-tree leaves,
returns the leaf with minimum cost together with its optimal x/y schedule.

QP formulation (per leaf):
  min  sum_n [ rho1*(xn - x_bar_n)^2 + a_x_n*xn
             + rho1*(yn - y_bar_n)^2 + a_y_n*yn
             + rho2*(xn - alpha_n)^2 + rho2*(yn - gamma_n)^2 ]
  s.t. xn >= alpha_tilde_n   (lower bound)
       mutual-exclusion ordering constraints (Aineq @ x <= bineq)

where yn = xn + C_n  (C_n = gamma_n - alpha_n, substituted analytically).

Fast path: analytical closed-form x* = -f/(2*A), clip to lb.
           Only call numerical QP when mutual-exclusion is violated.

All arrays 0-indexed.
"""

import numpy as np
from typing import List, Optional

from .constraints import build_mutual_exclusion_constraints


# --------------------------------------------------------------------------
def _solve_qp_scipy(A_coef: float, f_vec: np.ndarray,
                    lb_vec: np.ndarray,
                    Aineq: Optional[np.ndarray],
                    bineq: Optional[np.ndarray]):
    """
    Solve:  min  0.5 * A_coef * ||x||^2 + f_vec @ x
    s.t.    x >= lb_vec
            Aineq @ x <= bineq   (optional)

    Returns (x_vec, exitflag)  exitflag>0 means success.
    Uses scipy SLSQP — adequate for small nv (<=20).
    """
    from scipy.optimize import minimize, LinearConstraint, Bounds

    nv = len(f_vec)
    H = A_coef * np.eye(nv)

    def obj(x):
        return 0.5 * float(x @ H @ x) + float(f_vec @ x)

    def grad(x):
        return H @ x + f_vec

    bounds = Bounds(lb=lb_vec, ub=np.full(nv, np.inf))
    constraints = []
    if Aineq is not None and Aineq.shape[0] > 0:
        constraints.append(LinearConstraint(Aineq, lb=-np.inf, ub=bineq))

    x0 = np.maximum(-f_vec / A_coef, lb_vec)
    res = minimize(obj, x0, jac=grad, method='SLSQP',
                   bounds=bounds, constraints=constraints,
                   options={'ftol': 1e-10, 'disp': False, 'maxiter': 500})

    if res.success or res.status == 8:   # 8 = positive dir derivative (near boundary)
        return res.x, 1
    return None, -1


# --------------------------------------------------------------------------
def in_admm(NODES, LEAF: List[int], agent_i: int, entries,
            xi_prev, yi_prev, xi_prev_bar, yi_prev_bar,
            ai_x, ai_y, valid_systems, MapMat, Cmat, const):
    """
    Equivalent of MATLAB IN_Admm.

    Parameters (all 0-indexed vehicles)
    ------------------------------------
    NODES        : list of Node objects (index 1-based; NODES[0]=None, NODES[k]=Node)
    LEAF         : list of leaf node indices
    agent_i      : this agent's index (0-based)
    entries      : list[N]  entries[n] = non-empty means vehicle n visits this agent
    xi_prev      : list[N]  xi_prev[n][0] = previous x value (kn=0)
    yi_prev      : list[N]  yi_prev[n][0] = previous y value
    xi_prev_bar  : list[N]  xi_prev_bar[n][0] = averaged x
    yi_prev_bar  : list[N]  yi_prev_bar[n][0] = averaged y
    ai_x         : list[N]  dual variable for x
    ai_y         : list[N]  dual variable for y
    valid_systems: list of int  0-indexed vehicle IDs
    MapMat       : ndarray (S, N)
    Cmat         : ndarray (S, N)
    const        : dict

    Returns
    -------
    best_x, best_y, best_alpha, best_gamma, best_idx, NODES
    """
    rho1 = const['rho1']
    rho2 = const['rho2']
    N = const['N']
    alpha_tilde = const['alpha_tilde']   # list[N] of float

    kn = 0   # 0-indexed task (kn=1 in MATLAB)

    best_cost = np.inf
    best_x = None
    best_y = None
    best_gamma = None
    best_alpha = None
    best_idx = None

    A_coef = 4.0 * (rho1 + rho2)   # diagonal Hessian coefficient
    nv = len(valid_systems)

    # Build compact index map: valid_systems[i] → position i in x_vec
    idx_map = np.full(N, -1, dtype=int)
    for i, n in enumerate(valid_systems):
        idx_map[n] = i

    for leaf_idx in LEAF:
        node = NODES[leaf_idx]
        gamma = node.gamma   # list[N]
        alpha = node.alpha   # list[N]

        # --- Build f_vec, lb_vec, C_vec ---
        f_vec = np.zeros(nv)
        lb_vec = np.full(nv, -np.inf)
        C_vec = np.zeros(N)

        skip_leaf = False
        for i, n in enumerate(valid_systems):
            if not entries[n]:
                continue
            if (xi_prev_bar[n] is None or alpha[n] is None or gamma[n] is None):
                print(f'[IN_Admm] Agent {agent_i} sys {n} leaf {leaf_idx}: '
                      f'empty bar/alpha/gamma — skipping system.')
                continue

            x_bar    = float(xi_prev_bar[n][kn])
            y_bar    = float(yi_prev_bar[n][kn])
            a_x      = float(ai_x[n][kn])
            a_y      = float(ai_y[n][kn])
            alpha_kn = float(alpha[n])
            gamma_kn = float(gamma[n])
            C_n      = gamma_kn - alpha_kn
            C_vec[n] = C_n

            f_vec[i] = (a_x + a_y
                        + 2 * rho1 * (C_n - x_bar - y_bar)
                        - 4 * rho2 * alpha_kn)
            lb_vec[i] = float(alpha_tilde[n][kn])

        # --- Analytical closed-form ---
        x_cl = np.maximum(-f_vec / A_coef, lb_vec)

        # Build mutual-exclusion constraints
        Aineq, bineq = build_mutual_exclusion_constraints(
            MapMat, Cmat, valid_systems, gamma, idx_map, C_vec, eps_gap=1e-4)

        if Aineq.shape[0] == 0 or np.all(Aineq @ x_cl <= bineq + 1e-8):
            # Analytical solution is feasible
            x_vec = x_cl
            exitflag = 1
        else:
            # Fallback to numerical QP
            x_vec, exitflag = _solve_qp_scipy(A_coef, f_vec, lb_vec, Aineq, bineq)

        if exitflag <= 0 or x_vec is None:
            x_opt = np.zeros(N)
            y_opt = np.zeros(N)
            for n in valid_systems:
                x_opt[n] = float(xi_prev[n][kn])
                y_opt[n] = float(yi_prev[n][kn])
            cost = np.inf
        else:
            x_opt = np.zeros(N)
            for i, n in enumerate(valid_systems):
                x_opt[n] = x_vec[i]
            y_opt = x_opt.copy()
            for n in valid_systems:
                y_opt[n] += C_vec[n]

            if (np.any(np.isnan(x_opt[valid_systems]))
                    or np.any(np.isnan(y_opt[valid_systems]))):
                print(f'[IN_Admm] Agent {agent_i} leaf {leaf_idx}: NaN — fallback.')
                for n in valid_systems:
                    x_opt[n] = float(xi_prev[n][kn])
                    y_opt[n] = float(yi_prev[n][kn])
                cost = np.inf
            else:
                cost = 0.0
                for n in valid_systems:
                    xn = x_opt[n]; yn = y_opt[n]
                    cost += (rho1 * (xn - float(xi_prev_bar[n][kn])) ** 2
                             + float(ai_x[n][kn]) * xn
                             + rho1 * (yn - float(yi_prev_bar[n][kn])) ** 2
                             + float(ai_y[n][kn]) * yn
                             + rho2 * (xn - float(alpha[n])) ** 2
                             + rho2 * (yn - float(gamma[n])) ** 2)

        if not np.isnan(cost) and cost < best_cost:
            best_cost = cost
            best_x = x_opt
            best_y = y_opt
            best_gamma = gamma
            best_alpha = alpha
            best_idx = leaf_idx

    # Fallback if all leaves infeasible
    if best_x is None:
        print(f'[IN_Admm] Agent {agent_i}: all {len(LEAF)} leaves infeasible — fallback.')
        best_x = np.zeros(N)
        best_y = np.zeros(N)
        best_alpha = [None] * N
        best_gamma = [None] * N
        best_idx = LEAF[0]
        for n in valid_systems:
            best_x[n] = float(xi_prev[n][kn])
            best_y[n] = float(yi_prev[n][kn])
            best_alpha[n] = NODES[best_idx].alpha[n]
            best_gamma[n] = NODES[best_idx].gamma[n]

    return best_x, best_y, best_alpha, best_gamma, best_idx, NODES
