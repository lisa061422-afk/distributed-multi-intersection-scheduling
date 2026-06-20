"""
Export schedule results to the Traffic_Demo HTML's expected .js format.

Ports MATLAB's:
  * export_demo_json.m       → export_optimal_js
  * export_demo_json_fcfs.m  → export_fcfs_js

Both produce a single ``var SCHEDULE_DATA = {...};`` file matching the
schema the demo's ``applySetup()`` expects (see warehouse_amr_demo_test.html).
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable, List


def _theoretical_deadline(const, n: int) -> float:
    """alpha_tilde[0] + sum(routeDur per int) + (NI-1)*Dt — the zero-wait
    finish time used as a reference deadline in the demo bar."""
    alpha0 = float(const['alpha_tilde'][n][0])
    Dt     = float(const['Dt'])
    chain  = const['pathInfo_agent_chain'][n][0]
    n_int  = const['n_int']
    int_agents = [a for a in chain[:-1] if a < n_int]
    rids   = list(const['pathInfo'][n].get('routeId', []))
    IntSpaceDB = const['IntSpaceDB']

    deadline = alpha0
    for k, ag in enumerate(int_agents):
        rId = int(rids[k]) if k < len(rids) else 1
        rd  = IntSpaceDB[ag]['routeDur']
        dur_seq = rd[rId] if isinstance(rd, dict) else rd[rId - 1]
        deadline += float(sum(dur_seq))
        if k < len(int_agents) - 1:
            deadline += Dt
    return deadline


def _vehicle_meta(const, n: int) -> dict:
    cfg = const['config'][n] if 'config' in const else {}
    return {
        'id':       n + 1,                       # demo uses 1-indexed
        'entrance': int(cfg.get('entrance', 0)),
        'exit':     int(cfg.get('exit',     0)),
        'alpha0':   float(const['alpha_tilde'][n][0]),
        'deadline': _theoretical_deadline(const, n),
    }


def _all_alpha0(const) -> List[float]:
    return [float(const['alpha_tilde'][n][0]) for n in range(const['N'])]


def _write_js(result: dict, output_file: Path) -> Path:
    output_file = Path(output_file).with_suffix('.js')
    output_file.parent.mkdir(parents=True, exist_ok=True)
    with output_file.open('w') as f:
        f.write('var SCHEDULE_DATA = ')
        json.dump(result, f, indent=2)
        f.write(';\n')
    return output_file


# ───────────────────────────────────────────────────────────────────────
# Optimal export (CR-MPC C-ADMM result)
# ───────────────────────────────────────────────────────────────────────

def export_optimal_js(const, x_prev, y_prev, scenario_name: str,
                      output_file, policy_name: str = 'optimal',
                      priority_n: int = 0) -> Path:
    """Mirror of export_demo_json.m. x_prev/y_prev come from run_admm_core."""
    N = const['N']
    n_int = const['n_int']
    Dt = float(const['Dt'])
    pathInfo = const['pathInfo']
    chains = [const['pathInfo_agent_chain'][n][0] for n in range(N)]

    # t_cut = min release - 4s
    alphas = _all_alpha0(const)
    t_cut = min(alphas) - 4.0

    # t_final = max γ across all vehicles' last intersection
    t_finals = []
    for n in range(N):
        chain = chains[n]
        # Last int agent = chain[-2] (chain ends with terminal at -1)
        # MATLAB: chain(end-1). chain layout: int road int ... int terminal.
        last_ag = chain[-2]
        if last_ag < n_int and y_prev[last_ag][n] is not None:
            t_finals.append(float(y_prev[last_ag][n][0]))
    t_final = max(t_finals) if t_finals else 0.0

    vehicles = []
    for n in range(N):
        meta = _vehicle_meta(const, n)
        chain = chains[n]
        int_agents = [a for a in chain[:-1] if a < n_int]
        rids = list(pathInfo[n].get('routeId', []))

        kf = []
        for k, ag in enumerate(int_agents):
            beta_v  = float(x_prev[ag][n][0]) if x_prev[ag][n] is not None else 0.0
            gamma_v = float(y_prev[ag][n][0]) if y_prev[ag][n] is not None else 0.0
            rId     = int(rids[k]) if k < len(rids) else 1
            # Demo expects 1-indexed agent id
            kf.append([ag + 1, beta_v, gamma_v, rId])

        vehicles.append({**meta, 'keyframes': kf})

    result = {
        'scenario':       scenario_name,
        'policy':         policy_name,
        'priorityRobot':  int(priority_n),
        't_cut':          t_cut,
        't_final':        t_final,
        'Dt':             Dt,
        'vehicles':       vehicles,
    }
    out = _write_js(result, output_file)
    print(f'Exported optimal: {out}  (N={N}, t_cut={t_cut:.2f}, t_final={t_final:.2f})')
    return out


# ───────────────────────────────────────────────────────────────────────
# FCFS export
# ───────────────────────────────────────────────────────────────────────

def export_fcfs_js(const, fcfs_result, scenario_name: str,
                   output_file, policy_name: str = 'fcfs') -> Path:
    """Mirror of export_demo_json_fcfs.m. fcfs_result is FcfsResult from fcfs.run_fcfs."""
    N = const['N']
    n_int = const['n_int']
    Dt = float(const['Dt'])
    pathInfo = const['pathInfo']
    chains = [const['pathInfo_agent_chain'][n][0] for n in range(N)]

    alphas = _all_alpha0(const)
    t_cut = min(alphas) - 4.0

    t_finals = []
    for n in range(N):
        if fcfs_result.gamma[n]:
            t_finals.append(float(fcfs_result.gamma[n][-1]))
    t_final = max(t_finals) if t_finals else 0.0

    vehicles = []
    for n in range(N):
        meta = _vehicle_meta(const, n)
        chain = chains[n]
        int_agents = [a for a in chain[:-1] if a < n_int]
        rids = list(pathInfo[n].get('routeId', []))

        kf = []
        for k, ag in enumerate(int_agents):
            beta_v  = float(fcfs_result.beta[n][k])  if k < len(fcfs_result.beta[n])  else 0.0
            gamma_v = float(fcfs_result.gamma[n][k]) if k < len(fcfs_result.gamma[n]) else 0.0
            rId     = int(rids[k]) if k < len(rids) else 1
            kf.append([ag + 1, beta_v, gamma_v, rId])

        vehicles.append({**meta, 'keyframes': kf})

    result = {
        'scenario':       scenario_name,
        'policy':         policy_name,
        'priorityRobot':  0,
        't_cut':          t_cut,
        't_final':        t_final,
        'Dt':             Dt,
        'vehicles':       vehicles,
    }
    out = _write_js(result, output_file)
    print(f'Exported FCFS:    {out}  (N={N}, t_cut={t_cut:.2f}, t_final={t_final:.2f})')
    return out
