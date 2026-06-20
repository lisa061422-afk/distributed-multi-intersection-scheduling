"""
Generic seed sweep for 2x2 warehouse runs at arbitrary N + max_per_int.

Same status taxonomy as sweep_seeds_20r.py, but N is configurable.

Run from repo root:
    python python_port/sweep_seeds_generic.py --N 12 --max_per_int 8
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SPEC = 'python_port/warehouse_2x2.py'
DEFAULT_SEEDS = [42, 7, 13, 100, 200, 1234, 2024, 4101, 4222, 4223,
                 1, 3, 11, 21, 71, 314, 999]


def run_one(seed, N, first_iter_timeout, max_iter, max_per_int):
    cmd = [
        sys.executable, '-u', 'python_port/run_arbitrary_demo.py',
        '--mode', 'manual', '--spec', SPEC,
        '--N', str(N), '--seed', str(seed),
        '--Dt', '3.0', '--T_val', '2.0',
        '--max_iter', str(max_iter),
        '--max_per_int', str(max_per_int),
        '--parallel', '--quiet',
        '--first_iter_timeout', str(first_iter_timeout),
    ]
    t0 = time.time()
    proc = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, timeout=600)
    wall = time.time() - t0
    out = proc.stdout

    info = {'seed': seed, 'wall': wall, 'rc': proc.returncode,
            'iters': None, 'admm_elapsed': None, 'delay': None, 'reason': ''}

    density_reject = ('[Density reject]' in out) or proc.returncode == 2
    iter1_slow = '[Early exit]' in out
    converged  = '=== Converged ===' in out
    no_conv    = '=== Did not converge ===' in out and not iter1_slow

    if density_reject:
        info['status'] = 'density-reject'
        for line in out.splitlines():
            if '[Density reject]' in line:
                info['reason'] = line.split(']', 1)[1].strip()[:60]
                break
    elif iter1_slow:
        info['status'] = 'iter1-slow'
    elif converged:
        info['status'] = 'converged'
    elif no_conv:
        info['status'] = 'no-conv'
    else:
        info['status'] = f'unknown(rc={proc.returncode})'

    for line in out.splitlines():
        s = line.strip()
        if s.startswith('iterations'):
            try: info['iters'] = int(s.split('=')[1].split('/')[0].strip())
            except Exception: pass
        elif s.startswith('elapsed'):
            try: info['admm_elapsed'] = float(s.split('=')[1].replace('s','').strip())
            except Exception: pass
        elif s.startswith('delay cost'):
            try: info['delay'] = float(s.split('=')[1].strip())
            except Exception: pass
    return info


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--N', type=int, required=True)
    ap.add_argument('--seeds', type=int, nargs='+', default=DEFAULT_SEEDS)
    ap.add_argument('--first_iter_timeout', type=float, default=10.0)
    ap.add_argument('--total_budget', type=float, default=30.0)
    ap.add_argument('--max_per_int', type=int, required=True)
    ap.add_argument('--max_iter', type=int, default=300)
    args = ap.parse_args()

    print(f'Sweep: N={args.N}  2x2 warehouse  Dt=3  T_val=2  max_per_int={args.max_per_int}')
    print(f'first_iter_timeout={args.first_iter_timeout}s  '
          f'total_budget={args.total_budget}s  max_iter={args.max_iter}')
    print(f'Seeds ({len(args.seeds)}): {args.seeds}\n')
    print(f"{'seed':>5}  {'iters':>5}  {'admm(s)':>8}  {'delay':>7}  {'status':<14}  reason")
    print('-' * 75)

    good = []
    for seed in args.seeds:
        info = run_one(seed, args.N, args.first_iter_timeout, args.max_iter, args.max_per_int)
        st = info['status']
        if st == 'converged' and (info['admm_elapsed'] or 1e9) > args.total_budget:
            st = 'slow-conv'
        if st == 'converged':
            good.append(info)
        print(f"{seed:>5}  {info['iters'] or 0:>5}  "
              f"{info['admm_elapsed'] or 0:>8.2f}  "
              f"{info['delay'] or 0:>7.3f}  {st:<14}  {info['reason']}")

    print('\n=== Best candidates (converged, within budget) ===')
    if not good:
        print('  None.')
    else:
        good.sort(key=lambda x: (x['delay'], x['admm_elapsed']))
        for g in good[:5]:
            print(f"  seed={g['seed']:>5}  iters={g['iters']:>3}  "
                  f"admm={g['admm_elapsed']:.2f}s  delay={g['delay']:.3f}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
