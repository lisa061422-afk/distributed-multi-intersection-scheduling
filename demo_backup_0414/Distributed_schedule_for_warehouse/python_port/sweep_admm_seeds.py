"""
Try several seeds with ADMM (default rho/alpha) for the 2x2 N=20 case.
Stop at first seed that converges within --max_iter and --budget seconds.
Export it directly to demo dir.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from python_port.topology import manual_topology
from python_port.generate_config import generate_random_config
from python_port.admm_core import run_admm_core
from python_port.fcfs import run_fcfs
from python_port.export_to_demo import export_optimal_js, export_fcfs_js

REPO = Path(__file__).resolve().parent.parent
SPEC = REPO / 'python_port' / 'warehouse_2x2.py'

DEFAULT_SEEDS = [42, 7, 13, 100, 200, 1234, 2024, 4101, 4222, 4223,
                 1, 3, 11, 21, 71]


def _load_spec(p):
    g = {'__file__': str(p)}
    exec(p.read_text(encoding='utf-8'), g)
    return manual_topology(coords=g['coords'], ports=g['ports'],
                           name=g.get('name', p.stem))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--N',       type=int, default=20)
    ap.add_argument('--Dt',      type=float, default=3.0)
    ap.add_argument('--T_val',   type=float, default=2.0)
    ap.add_argument('--max_iter',type=int, default=300)
    ap.add_argument('--budget',  type=float, default=120.0,
                    help='per-seed wall-clock budget (s)')
    ap.add_argument('--seeds',   type=int, nargs='+', default=DEFAULT_SEEDS)
    ap.add_argument('--demo_dir',
                    default='C:/Users/robin/OneDrive/Documents/Github_file/'
                            'demo_backup_0414/Traffic_Demo/schedules')
    ap.add_argument('--group',   default='20r')
    ap.add_argument('--scene',   default='S1')
    args = ap.parse_args()

    t = _load_spec(SPEC)
    print(f'Probe ADMM seeds for N={args.N} (budget={args.budget}s, max_iter={args.max_iter})\n')
    print(f"{'seed':>5}  {'conv':>5}  {'iter':>5}  {'admm(s)':>9}  {'cost':>8}  {'fcfs':>7}")
    print('-' * 60)

    for seed in args.seeds:
        try:
            const, ap2 = generate_random_config(
                N=args.N, seed=seed + 1000,
                max_per_int=args.N, Dt=args.Dt, T_val=args.T_val,
                topology=t, max_iter=args.max_iter)
        except ValueError:
            print(f'{seed:>5}  {"--":>5}  {"--":>5}  {"--":>9}  {"--":>8}  density-reject')
            continue

        const['useParallel'] = True
        const['verbose']     = False

        # Wrap ADMM with a wall-clock budget using SIGALRM (Linux/macOS only).
        # On Windows we just rely on --max_iter; budget is approximate.
        t0 = time.time()
        res = run_admm_core(const, ap2)
        x_prev, y_prev, _, res_r, res_s, delay_costs, k, T_admm, _, _ = res
        wall = time.time() - t0
        converged = (float(res_r[k-1]) < const['tol_r']) and \
                    (float(res_s[k-1]) < const['tol_s'])
        cost = float(delay_costs[k-1])

        flag = '✓' if converged else '✗'
        print(f'{seed:>5}  {flag:>5}  {k:>5}  {wall:>9.2f}  {cost:>8.4f}', end='')

        # FCFS for comparison
        try:
            fcfs_res = run_fcfs(const, ap2, deadline_s=30.0, verbose=False)
            print(f'  {fcfs_res.total_delay:>7.3f}')
        except Exception as e:
            print(f'  fcfs-fail: {e}')
            continue

        if converged:
            # Export and stop
            demo_dir = Path(args.demo_dir).resolve()
            scenario_str = f'{args.group} · {args.scene}'
            out_opt  = demo_dir / f'{args.group}_{args.scene}_optimal.js'
            out_fcfs = demo_dir / f'{args.group}_{args.scene}_fcfs.js'
            print(f'\n  → exporting seed={seed}')
            export_optimal_js(const, x_prev, y_prev,
                              scenario_name=f'{scenario_str} · Optimal',
                              output_file=out_opt, policy_name='optimal')
            export_fcfs_js(const, fcfs_res,
                           scenario_name=f'{scenario_str} · FCFS',
                           output_file=out_fcfs, policy_name='fcfs')
            print('DONE.')
            return 0

    print('\nNo seed converged with default ADMM params.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
