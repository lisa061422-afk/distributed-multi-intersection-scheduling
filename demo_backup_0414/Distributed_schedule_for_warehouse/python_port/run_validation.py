"""
Numerical validation: load config exported by MATLAB export_config_for_python.m,
run the Python ADMM, and print key metrics for comparison.

Usage (from warehouse root):
    cd python_port
    python run_validation.py

Expects validation_config.json in the same directory.
"""

import json
import sys
import time
import numpy as np
from pathlib import Path

# Make sure the package is importable when run directly
sys.path.insert(0, str(Path(__file__).parent.parent))

from python_port.admm_core import run_admm_core


# ===========================================================================
# JSON → Python config adapter
# ===========================================================================

def load_config(json_path: str) -> dict:
    """
    Load MATLAB-exported JSON and convert to Python 0-indexed format.

    MATLAB uses 1-indexed agent IDs (1..9) and vehicle IDs (1..N).
    Python port uses 0-indexed throughout.
    """
    with open(json_path, encoding='utf-8') as f:
        raw = json.load(f)

    N = int(raw['N'])

    # ── Scalars ──────────────────────────────────────────────────────
    const = {
        'N':          N,
        'Dt':         float(raw['Dt']),
        'rho1':       float(raw['rho1']),
        'rho2':       float(raw['rho2']),
        'weight':     float(raw['weight']),
        'max_iter':   int(raw['max_iter']),
        'tol_r':      float(raw['tol_r']),
        'tol_s':      float(raw['tol_s']),
        'priority_n': 0,
        'use_pruning': True,
        'use_weak_rule': True,
        'timeout_int_s': 30,
        'useTBound': True,
        'use_quadprog': True,
        'use_adaptive_rho': False,
        'randInitScale': float(raw.get('randInitScale', 0.0)),
        'useParallel': False,
    }

    # ── alpha_tilde, deadline (1-indexed list, 0-indexed access via [n][0]) ─
    alpha_tilde_raw = raw['alpha_tilde']
    deadline_raw    = raw['deadline']
    alpha_tilde = [np.array([float(alpha_tilde_raw[n])]) for n in range(N)]
    deadline    = [np.array([float(deadline_raw[n])])    for n in range(N)]
    const['alpha_tilde'] = alpha_tilde
    const['deadline']    = deadline

    # MATLAB jsonencode collapses single-element arrays to scalars — normalise here
    def to_list(v):
        return v if isinstance(v, list) else [v]

    # ── agent_chains: convert 1-indexed MATLAB agent IDs → 0-indexed ────
    chains_raw = raw['agent_chains']
    pathInfo_agent_chain = []
    for n in range(N):
        chain_1idx = [int(x) for x in to_list(chains_raw[n])]
        chain_0idx = [a - 1 for a in chain_1idx]   # 1-indexed → 0-indexed
        pathInfo_agent_chain.append([chain_0idx])   # [kn=0]
    const['pathInfo_agent_chain'] = pathInfo_agent_chain

    # ── path_c: intersection durations per vehicle ───────────────────
    cseqs_raw = raw['path_c']
    pathInfo_c = []
    for n in range(N):
        cseq = [float(x) for x in to_list(cseqs_raw[n])]
        pathInfo_c.append([cseq])   # [kn=0]
    const['pathInfo_c'] = pathInfo_c

    # ── route_ids: per-vehicle route labels (1-indexed, kept as labels) ─
    route_ids_raw = raw['route_ids']
    route_ids = []
    for n in range(N):
        route_ids.append([int(x) for x in to_list(route_ids_raw[n])])

    # pathInfo: list[N] of dicts with 'routeId' (0-indexed per-intersection list)
    pathInfo = [{'routeId': route_ids[n]} for n in range(N)]
    const['pathInfo'] = pathInfo

    # ── IntSpaceDB: 4 intersections, 0-indexed ────────────���──────────
    int_db_raw = raw['IntSpaceDB']
    # int_db_raw is a list of 4 dicts (0-indexed after MATLAB's 1-indexed export)
    IntSpaceDB = []
    for ag in range(4):
        db_raw = int_db_raw[ag]
        num_routes = int(db_raw['numRoutes'])
        db = {
            'numSpaces': int(db_raw['numSpaces']),
            'numRoutes': num_routes,
            'routeSpace': {},   # keyed by 1-indexed route label
            'routeDur':   {},
        }
        rs_raw = db_raw['routeSpace']   # list of num_routes arrays
        rd_raw = db_raw['routeDur']
        for r_idx in range(num_routes):
            r_label = r_idx + 1   # 1-indexed route label
            db['routeSpace'][r_label] = [int(x) for x in to_list(rs_raw[r_idx])]
            db['routeDur'][r_label]   = [float(x) for x in to_list(rd_raw[r_idx])]
        IntSpaceDB.append(db)
    const['IntSpaceDB'] = IntSpaceDB

    # ── agent_participation: 9 agents × N vehicles ───────────────────
    ap_mat = raw['agent_participation']   # 8 x N from MATLAB (0-indexed rows 0..7)
    agent_participation = []
    for ag in range(9):
        if ag < 8:
            row = [bool(int(ap_mat[ag][n])) for n in range(N)]
        else:
            # Terminal (agent 8): participates in all vehicles
            row = [True] * N
        agent_participation.append(row)

    return const, agent_participation


# ===========================================================================
# Main validation run
# ===========================================================================

def main():
    json_path = Path(__file__).parent / 'validation_config.json'
    if not json_path.exists():
        print(f'ERROR: {json_path} not found.')
        print('Run export_config_for_python.m in MATLAB first.')
        sys.exit(1)

    print(f'Loading config from {json_path}')
    const, agent_participation = load_config(str(json_path))
    N = const['N']

    print(f'\n=== Python ADMM Validation ===')
    print(f'N={N}  max_iter={const["max_iter"]}  rho1={const["rho1"]}  Dt={const["Dt"]}')
    print(f'Chains (0-indexed):')
    for n in range(min(N, 3)):
        print(f'  Vehicle {n}: {const["pathInfo_agent_chain"][n][0]}')
    print(f'  alpha_tilde[0..2]: {[round(float(const["alpha_tilde"][n][0]),4) for n in range(min(3,N))]}')
    print()

    t0 = time.time()
    (x_prev, y_prev, cache,
     res_r, res_s, delay_costs,
     k_conv, T_admm) = run_admm_core(const, agent_participation)

    print(f'\n=== Results ===')
    print(f'Converged at k={k_conv}   elapsed={T_admm:.2f}s')
    print(f'Delay cost (terminal): {delay_costs[k_conv-1]:.6f}')
    print(f'Final residuals: r={res_r[k_conv-1]:.6f}  s={res_s[k_conv-1]:.6f}')
    print(f'Residual r history: {[round(float(x),4) for x in res_r[:min(k_conv,10)]]}')

    print(f'\nFinal x_prev (agent 0, intersection, first 5 vehicles):')
    for n in range(min(5, N)):
        xv = x_prev[0][n]
        yv = y_prev[0][n]
        if xv is not None:
            print(f'  n={n}: x={float(xv[0]):.4f}  y={float(yv[0]):.4f}')
        else:
            print(f'  n={n}: (not participating)')

    print(f'\nFinal x_prev (terminal agent 8, first 5 vehicles):')
    for n in range(min(5, N)):
        xv = x_prev[8][n]
        if xv is not None:
            ddl = float(const['deadline'][n][0])
            delay = max(float(xv[0]) - ddl, 0)
            print(f'  n={n}: x={float(xv[0]):.4f}  deadline={ddl:.4f}  delay={delay:.4f}')


if __name__ == '__main__':
    main()
