"""
Find a seed for which centralized DFS-BnB completes within --budget seconds.

For each candidate seed:
  1. Generate config (skip if density-reject)
  2. Run FCFS (fast)
  3. Run BnB with --budget timeout
  4. If BnB completed (deadline NOT hit) → success, optionally export

The first successful seed (or top --top_n) is reported.

Usage from repo root:
    python python_port/sweep_bnb_seeds.py --N 20 --budget 120
    python python_port/sweep_bnb_seeds.py --N 20 --budget 120 --export
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from python_port.topology import manual_topology
from python_port.generate_config import generate_random_config
from python_port.fcfs import run_fcfs, run_optimal_bnb

REPO = Path(__file__).resolve().parent.parent
SPEC = REPO / 'python_port' / 'warehouse_2x2.py'

DEFAULT_SEEDS = [42, 7, 13, 100, 200, 1234, 2024, 4101, 4222, 4223,
                 1, 3, 11, 21, 71, 314, 999, 5, 17, 33]


def _load_spec(spec_path):
    p = Path(spec_path).resolve()
    g = {'__file__': str(p)}
    exec(p.read_text(encoding='utf-8'), g)
    return manual_topology(coords=g['coords'], ports=g['ports'],
                           name=g.get('name', p.stem))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--N',       type=int, default=20)
    ap.add_argument('--Dt',      type=float, default=3.0)
    ap.add_argument('--T_val',   type=float, default=2.0)
    ap.add_argument('--budget',  type=float, default=120.0,
                    help='BnB time budget per seed (s)')
    ap.add_argument('--seeds',   type=int, nargs='+', default=DEFAULT_SEEDS)
    ap.add_argument('--export',  action='store_true',
                    help='Export the first successful seed to demo dir')
    ap.add_argument('--demo_dir',
                    default='C:/Users/robin/OneDrive/Documents/Github_file/'
                            'demo_backup_0414/Traffic_Demo/schedules')
    ap.add_argument('--group',   default='20r')
    ap.add_argument('--scene',   default='S1')
    args = ap.parse_args()

    print(f'Building topology...')
    t = _load_spec(SPEC)

    print(f'Sweep: N={args.N}, Dt={args.Dt}, T_val={args.T_val}, '
          f'budget={args.budget}s')
    print(f'Seeds ({len(args.seeds)}): {args.seeds}\n')
    print(f"{'seed':>5}  {'fcfs':>7}  {'bnb':>9}  {'bnb_t':>7}  {'nodes':>8}  status")
    print('-' * 70)

    first_success = None
    for seed in args.seeds:
        try:
            const, ap2 = generate_random_config(
                N=args.N, seed=seed + 1000,
                max_per_int=args.N,  # no density cap
                Dt=args.Dt, T_val=args.T_val,
                topology=t, max_iter=300)
        except ValueError:
            print(f'{seed:>5}  {"--":>7}  {"--":>9}  {"--":>7}  {"--":>8}  density-reject')
            continue

        # FCFS first
        try:
            fcfs_res = run_fcfs(const, ap2, deadline_s=60.0, verbose=False)
        except Exception as e:
            print(f'{seed:>5}  err   {"--":>9}  {"--":>7}  {"--":>8}  fcfs-fail: {e}')
            continue

        # BnB with budget
        t0 = time.time()
        try:
            opt_res = run_optimal_bnb(const, ap2, deadline_s=args.budget, verbose=False)
            bnb_t = time.time() - t0
            bnb_done = (bnb_t < args.budget * 0.95)
            beats_fcfs = (opt_res.total_delay < fcfs_res.total_delay - 1e-6)
            if bnb_done:
                status = 'OK'
            elif beats_fcfs:
                status = f'partial(<FCFS)'
            else:
                status = 'deadline-hit'
            print(f'{seed:>5}  {fcfs_res.total_delay:>7.3f}  '
                  f'{opt_res.total_delay:>9.3f}  '
                  f'{bnb_t:>7.2f}  {opt_res.nodes_explored:>8}  {status}')
            # Accept first OK; or, fall back to first partial-better-than-FCFS
            if bnb_done and first_success is None:
                first_success = (seed, const, ap2, fcfs_res, opt_res, 'OK')
                print(f'    → first successful seed: {seed}')
                break
            elif beats_fcfs and first_success is None:
                # tentative; keep looking for an OK but record this
                first_success = (seed, const, ap2, fcfs_res, opt_res, 'partial')
        except Exception as e:
            print(f'{seed:>5}  {fcfs_res.total_delay:>7.3f}  err  {"--":>7}  {"--":>8}  bnb-fail: {e}')
            continue

    if first_success is None:
        print('\nNo seed completed within budget. '
              'Increase --budget or try lower N.')
        return 1

    if not args.export:
        print('\n(Re-run with --export to write .js files)')
        return 0

    seed, const, ap2, fcfs_res, opt_res, kind = first_success
    print(f'\nUsing seed={seed} ({kind})')
    from python_port.export_to_demo import export_fcfs_js
    demo_dir = Path(args.demo_dir).resolve()
    out_opt  = demo_dir / f'{args.group}_{args.scene}_optimal.js'
    out_fcfs = demo_dir / f'{args.group}_{args.scene}_fcfs.js'
    scenario_str = f'{args.group} · {args.scene}'

    print(f'\nExporting seed={seed} to demo dir...')
    export_fcfs_js(const, opt_res,
                   scenario_name=f'{scenario_str} · Optimal',
                   output_file=out_opt, policy_name='optimal')
    export_fcfs_js(const, fcfs_res,
                   scenario_name=f'{scenario_str} · FCFS',
                   output_file=out_fcfs, policy_name='fcfs')
    print('DONE.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
