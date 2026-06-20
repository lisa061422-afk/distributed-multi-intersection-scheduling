"""
Find the largest N for which centralized BnB completes within --budget,
on the 2x2 warehouse spec. Uses strict timeout (no partial results).
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


def _load_spec(p):
    g = {'__file__': str(p)}
    exec(p.read_text(encoding='utf-8'), g)
    return manual_topology(coords=g['coords'], ports=g['ports'],
                           name=g.get('name', p.stem))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--N_max', type=int, default=20)
    ap.add_argument('--N_min', type=int, default=8)
    ap.add_argument('--seed',  type=int, default=42)
    ap.add_argument('--budget', type=float, default=60.0)
    ap.add_argument('--Dt', type=float, default=3.0)
    ap.add_argument('--T_val', type=float, default=2.0)
    args = ap.parse_args()

    print(f'Probe largest N where BnB completes within {args.budget}s')
    print(f'(seed={args.seed}, Dt={args.Dt}, T_val={args.T_val})\n')
    print(f"{'N':>4}  {'fcfs':>7}  {'bnb':>9}  {'bnb_t':>7}  {'nodes':>9}  status")
    print('-' * 65)

    t = _load_spec(SPEC)
    largest_ok = None

    for N in range(args.N_max, args.N_min - 1, -1):
        try:
            const, ap2 = generate_random_config(
                N=N, seed=args.seed + 1000,
                max_per_int=N, Dt=args.Dt, T_val=args.T_val,
                topology=t, max_iter=300)
        except ValueError as e:
            print(f'{N:>4}  {"--":>7}  {"--":>9}  {"--":>7}  {"--":>9}  density-reject')
            continue

        try:
            fcfs_res = run_fcfs(const, ap2, deadline_s=30.0, verbose=False)
        except Exception as e:
            print(f'{N:>4}  err  {"--":>9}  {"--":>7}  {"--":>9}  fcfs-fail')
            continue

        t0 = time.time()
        try:
            opt_res = run_optimal_bnb(const, ap2, deadline_s=args.budget, verbose=False)
            bnb_t = time.time() - t0
            print(f'{N:>4}  {fcfs_res.total_delay:>7.3f}  '
                  f'{opt_res.total_delay:>9.3f}  '
                  f'{bnb_t:>7.2f}  {opt_res.nodes_explored:>9}  OK')
            if largest_ok is None:
                largest_ok = (N, opt_res.total_delay, fcfs_res.total_delay,
                              bnb_t, opt_res.nodes_explored)
                print(f'    → largest N that BnB completes: {N}')
                break
        except TimeoutError:
            bnb_t = time.time() - t0
            print(f'{N:>4}  {fcfs_res.total_delay:>7.3f}  '
                  f'{"timeout":>9}  {bnb_t:>7.2f}  {"--":>9}  TIMEOUT')

    if largest_ok is None:
        print(f'\nNo N in [{args.N_min}..{args.N_max}] completes within {args.budget}s.')
        return 1

    N, opt_g, fcfs_g, bnb_t, nodes = largest_ok
    print(f'\n=== Result ===')
    print(f'  use N = {N}')
    print(f'  BnB time = {bnb_t:.2f}s, nodes = {nodes}')
    print(f'  Optimal cost = {opt_g:.4f}')
    print(f'  FCFS cost    = {fcfs_g:.4f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
