"""
Equivalent of MATLAB updateAgent9.m

Closed-form KKT solution for the terminal agent (agent 9, index 8 in 0-based).
Problem per vehicle n:
  min  rho1*(x - x_bar)^2 + a_x*x + weight * max(x - ddl, 0)
  s.t. x >= 0

Convex piecewise-quadratic — three regions solved analytically.
All arrays 0-indexed.
"""

import numpy as np


def update_terminal_agent(x9_prev, x9_prev_bar, a9_x, const):
    """
    Parameters
    ----------
    x9_prev     : list[N]  x9_prev[n][0] = previous x (kn=0)
    x9_prev_bar : list[N]  averaged x
    a9_x        : list[N]  dual variable for x
    const       : dict

    Returns
    -------
    x9_new    : ndarray (N,)
    delay_cost: float
    """
    N = const['N']
    rho1 = const['rho1']
    deadline = const['deadline']   # list[N] of array; deadline[n][0]
    weight = const['weight']
    kn = 0   # 0-indexed task

    x9_new = np.zeros(N)

    for n in range(N):
        if x9_prev_bar[n] is None:
            continue
        x_bar = float(x9_prev_bar[n][kn])
        a_x   = float(a9_x[n][kn])
        ddl_n = float(deadline[n][kn])

        # Unconstrained minimiser in left region (x <= ddl)
        x_A = x_bar - a_x / (2.0 * rho1)
        # Unconstrained minimiser in right region (x >= ddl)
        x_B = x_bar - (a_x + weight) / (2.0 * rho1)

        if x_A <= ddl_n:
            x_opt = x_A
        elif x_B >= ddl_n:
            x_opt = x_B
        else:
            x_opt = ddl_n   # kink is optimal

        x9_new[n] = max(x_opt, 0.0)

    # Delay cost
    ddl_vec = np.array([float(deadline[n][kn]) for n in range(N)])
    dif_val = np.maximum(x9_new - ddl_vec, 0.0)
    delay_cost = float(np.nansum(dif_val))

    return x9_new, delay_cost
