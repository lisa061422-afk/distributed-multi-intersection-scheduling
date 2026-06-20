"""
One-command demo case exporter.

Runs ADMM (optimal) + FCFS for a given (spec, N, seed) and writes the two
.js files the Traffic_Demo HTML loads:
    schedules/{group}_{scene}_optimal.js
    schedules/{group}_{scene}_fcfs.js

Usage (from repo root):

    python python_port/export_demo_case.py \\
        --spec python_port/warehouse_2x2.py \\
        --N 20 --seed 42 --Dt 3.0 --T_val 2.0 \\
        --group 20r --scene S1 \\
        --demo_dir "C:/Users/robin/OneDrive/Documents/Github_file/demo_backup_0414/Traffic_Demo/schedules"
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from python_port import defaults as _D
from python_port.topology import manual_topology
from python_port.generate_config import generate_random_config
from python_port.admm_core import run_admm_core
from python_port.fcfs import run_fcfs, run_optimal_bnb
from python_port.export_to_demo import export_optimal_js, export_fcfs_js


def _load_spec(spec_path):
    p = Path(spec_path).resolve()
    g = {'__file__': str(p)}
    exec(p.read_text(encoding='utf-8'), g)
    return manual_topology(coords=g['coords'], ports=g['ports'],
                           name=g.get('name', p.stem))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--spec',    required=True, help='manual topology .py spec')
    ap.add_argument('--N',       type=int,   required=True)
    ap.add_argument('--seed',    type=int,   required=True)
    ap.add_argument('--Dt',      type=float, default=_D.DT)
    ap.add_argument('--T_val',   type=float, default=_D.T_VAL)
    ap.add_argument('--T_ent',   type=float, default=_D.T_ENT)
    ap.add_argument('--max_iter',type=int,   default=_D.MAX_ITER)
    ap.add_argument('--rho',     type=float, default=_D.RHO1)
    ap.add_argument('--alpha',   type=float, default=_D.ALPHA_RELAX)
    ap.add_argument('--max_per_int', type=int, default=_D.MAX_PER_INT)
    ap.add_argument('--solver', choices=['admm', 'bnb'], default='bnb',
                    help="Optimal solver: 'admm' (distributed C-ADMM) or "
                         "'bnb' (centralized DFS branch-and-bound, default).")
    ap.add_argument('--bnb_deadline', type=float, default=_D.BNB_DEADLINE_S,
                    help='Time limit (s) for BnB search (default 300).')
    ap.add_argument('--group',   required=True, help='e.g. 20r')
    ap.add_argument('--scene',   required=True, help='e.g. S1')
    ap.add_argument('--demo_dir',required=True,
                    help='absolute path to Traffic_Demo/schedules')
    args = ap.parse_args()

    print(f'Building topology from {args.spec}...')
    t = _load_spec(args.spec)

    print(f'Generating config: N={args.N}, seed={args.seed}, '
          f'Dt={args.Dt}, T_val={args.T_val}')
    cap = args.max_per_int if args.max_per_int is not None else args.N
    const, ap2 = generate_random_config(
        N=args.N, seed=args.seed + 1000,
        max_per_int=cap, Dt=args.Dt,
        T_val=args.T_val, T_ent=args.T_ent,
        max_iter=args.max_iter, topology=t)
    const['useParallel'] = True
    const['verbose']     = False
    const['rho1']        = args.rho
    const['rho2']        = args.rho
    const['alpha_relax'] = args.alpha

    demo_dir = Path(args.demo_dir).resolve()
    if not demo_dir.exists():
        raise FileNotFoundError(f'demo_dir not found: {demo_dir}')
    scenario_str = f'{args.group} · {args.scene}'
    out_opt  = demo_dir / f'{args.group}_{args.scene}_optimal.js'
    out_fcfs = demo_dir / f'{args.group}_{args.scene}_fcfs.js'

    if args.solver == 'admm':
        print('\n=== Running optimal (C-ADMM, distributed) ===')
        res = run_admm_core(const, ap2)
        x_prev, y_prev, _, res_r, res_s, delay_costs, k, T_admm, _, _ = res
        converged = (float(res_r[k-1]) < const['tol_r']) and \
                    (float(res_s[k-1]) < const['tol_s'])
        print(f'  converged={converged}  iter={k}  elapsed={T_admm:.2f}s  '
              f'delay_cost={float(delay_costs[k-1]):.4f}')
        if not converged:
            print('  WARNING: ADMM did not converge — exporting anyway, '
                  'but the schedule may be infeasible.')

        print('\n=== Running FCFS (centralized BnB) ===')
        fcfs_res = run_fcfs(const, ap2, deadline_s=120.0, verbose=False)
        print(f'  elapsed={fcfs_res.elapsed:.3f}s  nodes={fcfs_res.nodes_explored}  '
              f'total_delay={fcfs_res.total_delay:.4f}')

        print('\n=== Exporting .js ===')
        export_optimal_js(const, x_prev, y_prev,
                          scenario_name=f'{scenario_str} · Optimal',
                          output_file=out_opt, policy_name='optimal')
    else:
        print('\n=== Running optimal (centralized DFS-BnB) ===')
        opt_res = run_optimal_bnb(const, ap2, deadline_s=args.bnb_deadline,
                                   verbose=False)
        print(f'  elapsed={opt_res.elapsed:.3f}s  nodes={opt_res.nodes_explored}  '
              f'total_delay={opt_res.total_delay:.4f}')

        print('\n=== Running FCFS (centralized BnB) ===')
        fcfs_res = run_fcfs(const, ap2, deadline_s=120.0, verbose=False)
        print(f'  elapsed={fcfs_res.elapsed:.3f}s  nodes={fcfs_res.nodes_explored}  '
              f'total_delay={fcfs_res.total_delay:.4f}')

        print('\n=== Exporting .js ===')
        # Reuse the FCFS exporter shape (same FcfsResult dataclass).
        export_fcfs_js(const, opt_res,
                       scenario_name=f'{scenario_str} · Optimal',
                       output_file=out_opt, policy_name='optimal')

    export_fcfs_js(const, fcfs_res,
                   scenario_name=f'{scenario_str} · FCFS',
                   output_file=out_fcfs, policy_name='fcfs')

    print(f'\nDONE. Files:\n  {out_opt}\n  {out_fcfs}')


if __name__ == '__main__':
    main()
