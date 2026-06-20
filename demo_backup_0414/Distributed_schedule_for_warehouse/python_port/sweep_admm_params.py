"""Quick ADMM parameter sweep for the 2x2 N=20 case.

Tries 4 (rho, alpha, adaptive_rho) combos against the same seed and reports
convergence + iters + elapsed.

Usage:  python python_port/sweep_admm_params.py
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SPEC = 'python_port/warehouse_2x2.py'

CONFIGS = [
    # (label, rho, alpha, adaptive) — baseline (rho=1) skipped, known to diverge
    ('rho=0.3  alpha=1.0  adaptive=off',  0.3, 1.0, False),
    ('rho=0.3  alpha=0.7  adaptive=off',  0.3, 0.7, False),
    ('rho=0.3  alpha=0.7  adaptive=on',   0.3, 0.7, True),
    ('rho=0.5  alpha=1.0  adaptive=on',   0.5, 1.0, True),
]

def run_one(rho, alpha, adaptive, N=20, seed=42, max_iter=300):
    cmd = [
        sys.executable, '-u', 'python_port/run_arbitrary_demo.py',
        '--mode', 'manual', '--spec', SPEC,
        '--N', str(N), '--seed', str(seed),
        '--Dt', '3.0', '--T_val', '2.0',
        '--max_iter', str(max_iter),
        '--rho', str(rho), '--alpha', str(alpha),
        '--parallel', '--quiet',
    ]
    if adaptive:
        cmd.append('--adaptive_rho')
    t0 = time.time()
    proc = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True,
                          timeout=900)
    wall = time.time() - t0
    out = proc.stdout
    converged = '=== Converged ===' in out
    iters = None
    delay = None
    elapsed_admm = None
    for line in out.splitlines():
        s = line.strip()
        if s.startswith('iterations'):
            try: iters = int(s.split('=')[1].split('/')[0].strip())
            except Exception: pass
        elif s.startswith('elapsed'):
            try: elapsed_admm = float(s.split('=')[1].replace('s','').strip())
            except Exception: pass
        elif s.startswith('delay cost'):
            try: delay = float(s.split('=')[1].strip())
            except Exception: pass
    return converged, iters, elapsed_admm, delay, wall


def main():
    print(f"Sweep: 2x2 warehouse, N=20, seed=42, Dt=3, T_val=2")
    print(f"{'config':<35}  {'conv':>5}  {'iters':>5}  {'admm(s)':>9}  {'delay':>8}")
    print('-' * 75)
    for label, rho, alpha, adaptive in CONFIGS:
        conv, iters, elapsed, delay, _ = run_one(rho, alpha, adaptive)
        flag = '✓' if conv else '✗'
        print(f"{label:<35}  {flag:>5}  {iters or 0:>5}  "
              f"{elapsed or 0:>9.2f}  {delay or 0:>8.4f}")

if __name__ == '__main__':
    main()
