"""
Scaling experiment: sweep n_int and N across multiple seeds, record
ADMM convergence iterations / wall-clock / delay cost.

Output:
  - CSV file with one row per (n_int, N, seed)
  - PNG plot: T_solve vs n_int (mean ± std across seeds), one curve per N
  - PNG plot: k_conv vs n_int

Usage (from warehouse root):
    python python_port/run_scaling.py
    python python_port/run_scaling.py --n_int 2,3,5,7 --N 6,10,14 --seeds 3
    python python_port/run_scaling.py --out my_results.csv --plot
    python python_port/run_scaling.py --parallel       # use ProcessPoolExecutor
    python python_port/run_scaling.py --max_iter 60    # cap each ADMM run

A row is marked "DNF" (did-not-finish) if ADMM hits max_iter without
converging — the residuals are still recorded but k_conv is the cap.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from python_port.topology import random_topology
from python_port.generate_config import generate_random_config
from python_port.admm_core import run_admm_core, warmup_parallel_pool


def _parse_int_list(s: str):
    return [int(x.strip()) for x in s.split(',') if x.strip()]


def run_one(n_int: int, N: int, seed: int, max_iter: int, Dt: float,
            use_parallel: bool, max_per_int: int = None,
            rho: float = 1.0, alpha: float = 1.0,
            adaptive_rho: bool = False):
    """Run a single (n_int, N, seed) cell. Returns dict of metrics."""
    t0 = time.time()
    try:
        topo = random_topology(n_int=n_int, seed=seed)
    except Exception as e:
        return {'error': f'topology: {type(e).__name__}: {e}'}

    if max_per_int is None:
        max_per_int = max(N, n_int * 2)

    try:
        const, ap = generate_random_config(
            N=N, seed=seed + 1000, max_per_int=max_per_int, Dt=Dt,
            max_iter=max_iter, topology=topo)
    except Exception as e:
        return {'error': f'gen_vehicles: {type(e).__name__}: {e}'}

    const['useParallel']      = use_parallel
    const['verbose']          = False
    const['rho1']             = rho
    const['rho2']             = rho
    const['alpha_relax']      = alpha
    const['use_adaptive_rho'] = adaptive_rho

    t_solve_start = time.time()
    try:
        res = run_admm_core(const, ap)
    except Exception as e:
        return {'error': f'admm: {type(e).__name__}: {e}'}
    t_solve = time.time() - t_solve_start

    x_prev, y_prev, _, res_r, res_s, delay_costs, k, T_admm, _, _ = res
    converged = (res_r[k - 1] < const['tol_r']) and (res_s[k - 1] < const['tol_s'])

    return {
        'n_int':     n_int,
        'n_road':    topo.n_road,
        'n_ports':   topo.n_ports,
        'n_OD':      len(topo.route_dict),
        'N':         N,
        'seed':      seed,
        'k_conv':    k,
        'converged': int(converged),
        'T_solve':   t_solve,
        'cost':      float(delay_costs[k - 1]),
        'r_final':   float(res_r[k - 1]),
        's_final':   float(res_s[k - 1]),
        'wall_total': time.time() - t0,
        'error':     '',
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--n_int', type=str, default='2,3,5,7,10',
                   help='Comma-separated list of n_int values to sweep')
    p.add_argument('--N',     type=str, default='6,10,14',
                   help='Comma-separated list of N (vehicles) values to sweep')
    p.add_argument('--seeds', type=int, default=3,
                   help='Number of seeds per (n_int, N) cell')
    p.add_argument('--seed_base', type=int, default=11)
    p.add_argument('--max_iter',  type=int, default=200,
                   help='Max ADMM iterations per cell (default 200)')
    p.add_argument('--Dt',        type=float, default=2.0)
    p.add_argument('--parallel',  action='store_true',
                   help='Run intersection updates in parallel (1 process pool reused across cells)')
    p.add_argument('--out',  type=str, default='scaling_results.csv',
                   help='Output CSV path (relative to warehouse root)')
    p.add_argument('--plot', action='store_true',
                   help='Generate PNG plots alongside the CSV (requires matplotlib)')
    args = p.parse_args()

    n_int_list = _parse_int_list(args.n_int)
    N_list     = _parse_int_list(args.N)

    print(f'Sweep grid: n_int={n_int_list}, N={N_list}, seeds={args.seeds}')
    print(f'             max_iter={args.max_iter}, Dt={args.Dt}, '
          f'parallel={args.parallel}')
    print()

    if args.parallel:
        print('Warming up parallel pool...')
        warmup_parallel_pool()
        print()

    rows = []
    total_cells = len(n_int_list) * len(N_list) * args.seeds
    cell_idx = 0

    print(f'  {"n_int":>5} {"N":>4} {"seed":>5} | '
          f'{"k":>4} {"T(s)":>7} {"cost":>9} {"OD":>4} {"conv":>4}')
    print('  ' + '-' * 60)

    for n_int in n_int_list:
        for N in N_list:
            for s in range(args.seeds):
                cell_idx += 1
                seed = args.seed_base + s
                row = run_one(n_int=n_int, N=N, seed=seed,
                              max_iter=args.max_iter, Dt=args.Dt,
                              use_parallel=args.parallel)
                if 'error' in row and row.get('error'):
                    print(f'  {n_int:>5} {N:>4} {seed:>5} | ERROR: {row["error"]}')
                    rows.append({**row, 'n_int': n_int, 'N': N, 'seed': seed})
                    continue
                conv_marker = 'OK' if row['converged'] else 'DNF'
                print(f'  {row["n_int"]:>5} {row["N"]:>4} {row["seed"]:>5} | '
                      f'{row["k_conv"]:>4} {row["T_solve"]:>7.2f} '
                      f'{row["cost"]:>9.4f} {row["n_OD"]:>4} {conv_marker:>4}')
                rows.append(row)

    # ── Write CSV ────────────────────────────────────────────────────
    out_path = Path(args.out).resolve()
    fieldnames = ['n_int', 'n_road', 'n_ports', 'n_OD',
                  'N', 'seed', 'k_conv', 'converged',
                  'T_solve', 'cost', 'r_final', 's_final',
                  'wall_total', 'error']
    with open(out_path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in fieldnames})
    print(f'\nWrote CSV: {out_path}')

    # ── Aggregate summary table ──────────────────────────────────────
    print('\nSummary (mean ± std across seeds):')
    print(f'  {"n_int":>5} {"N":>4} | {"k_conv":>15} {"T_solve(s)":>15} {"cost":>15}')
    print('  ' + '-' * 60)
    import statistics as stats
    for n_int in n_int_list:
        for N in N_list:
            cells = [r for r in rows if r.get('n_int') == n_int
                     and r.get('N') == N and not r.get('error')]
            if not cells:
                continue
            ks = [r['k_conv']  for r in cells]
            ts = [r['T_solve'] for r in cells]
            cs = [r['cost']    for r in cells]
            def mstd(xs):
                m = stats.mean(xs)
                sd = stats.stdev(xs) if len(xs) > 1 else 0.0
                return f'{m:>7.2f}+/-{sd:>5.2f}'
            print(f'  {n_int:>5} {N:>4} | {mstd(ks):>15} '
                  f'{mstd(ts):>15} {mstd(cs):>15}')

    # ── Optional plots ──────────────────────────────────────────────
    if args.plot:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            print('matplotlib not installed; skipping plots')
            return
        import statistics as stats

        # Plot 1: T_solve vs n_int, one curve per N
        fig, ax = plt.subplots(figsize=(7, 5))
        for N in N_list:
            xs, ms, sds = [], [], []
            for n_int in n_int_list:
                cells = [r for r in rows if r.get('n_int') == n_int
                         and r.get('N') == N and not r.get('error')]
                if not cells:
                    continue
                ts = [r['T_solve'] for r in cells]
                xs.append(n_int)
                ms.append(stats.mean(ts))
                sds.append(stats.stdev(ts) if len(ts) > 1 else 0.0)
            ax.errorbar(xs, ms, yerr=sds, marker='o',
                        capsize=3, label=f'N={N}')
        ax.set_xlabel('n_int (number of intersections)')
        ax.set_ylabel('ADMM solve time (s)')
        ax.set_title('Distributed ADMM: wall-clock scaling')
        ax.set_yscale('log')
        ax.grid(True, which='both', alpha=0.3)
        ax.legend()
        fig.tight_layout()
        out_png = out_path.with_suffix('.t_solve.png')
        fig.savefig(out_png, dpi=120)
        print(f'Wrote plot: {out_png}')

        # Plot 2: k_conv vs n_int
        fig, ax = plt.subplots(figsize=(7, 5))
        for N in N_list:
            xs, ms, sds = [], [], []
            for n_int in n_int_list:
                cells = [r for r in rows if r.get('n_int') == n_int
                         and r.get('N') == N and not r.get('error')]
                if not cells:
                    continue
                ks = [r['k_conv'] for r in cells]
                xs.append(n_int)
                ms.append(stats.mean(ks))
                sds.append(stats.stdev(ks) if len(ks) > 1 else 0.0)
            ax.errorbar(xs, ms, yerr=sds, marker='o',
                        capsize=3, label=f'N={N}')
        ax.set_xlabel('n_int')
        ax.set_ylabel('iterations to converge')
        ax.set_title('Distributed ADMM: convergence iterations')
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        out_png = out_path.with_suffix('.k_conv.png')
        fig.savefig(out_png, dpi=120)
        print(f'Wrote plot: {out_png}')


if __name__ == '__main__':
    main()
