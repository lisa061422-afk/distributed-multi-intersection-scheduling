"""
Equivalent of MATLAB updateRoadAgent.m

Closed-form KKT solution for each road agent.
Problem per vehicle n:
  min  rho1*(x - x_bar)^2 + a_x*x + rho1*(y - y_bar)^2 + a_y*y
  s.t. y >= x + Dt,   x >= 0

Three KKT cases solved analytically (no solver needed).
All arrays 0-indexed.
"""

import numpy as np


def update_road_agent(agent_i, entries, valid_systems,
                      xi_prev, yi_prev,
                      xi_prev_bar, yi_prev_bar,
                      ai_x, ai_y, const):
    """
    Parameters
    ----------
    agent_i       : int   0-indexed agent ID (road agent)
    entries       : list[N]  non-empty = vehicle visits this agent
    valid_systems : list of int  0-indexed vehicle IDs
    xi_prev       : list[N]  xi_prev[n][0] = previous x (kn=0)
    yi_prev       : list[N]  yi_prev[n][0] = previous y
    xi_prev_bar   : list[N]  averaged x
    yi_prev_bar   : list[N]  averaged y
    ai_x, ai_y    : list[N]  dual variables
    const         : dict

    Returns
    -------
    x_road : ndarray (N,)
    y_road : ndarray (N,)
    """
    N = const['N']
    rho1 = const['rho1']
    Dt = const['Dt']
    kn = 0   # 0-indexed task

    x_road = np.zeros(N)
    y_road = np.zeros(N)

    for n in valid_systems:
        if not entries[n]:
            continue

        x_bar = float(xi_prev_bar[n][kn])
        y_bar = float(yi_prev_bar[n][kn])
        a_x   = float(ai_x[n][kn])
        a_y   = float(ai_y[n][kn])

        x_unc = x_bar - a_x / (2.0 * rho1)
        y_unc = y_bar - a_y / (2.0 * rho1)

        if x_unc >= 0.0 and y_unc >= x_unc + Dt:
            # Case 1: both constraints inactive
            x_n = x_unc
            y_n = y_unc

        elif x_unc >= 0.0:
            # Case 2: y = x + Dt active
            x_n = (x_bar + y_bar - Dt) / 2.0 - (a_x + a_y) / (4.0 * rho1)
            if x_n < 0.0:
                x_n = 0.0
                y_n = Dt
            else:
                y_n = x_n + Dt

        else:
            # Case 3: x = 0 active
            x_n = 0.0
            y_n = max(y_unc, Dt)

        x_road[n] = x_n
        y_road[n] = y_n

    return x_road, y_road
