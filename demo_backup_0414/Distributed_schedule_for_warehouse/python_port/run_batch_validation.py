"""
Batch validation: 20 random scenarios (N=10-15), Python ADMM timing.

Saves each scenario as configs/scenario_XX.json so MATLAB can read them too.
Run batch_validate_matlab.m in MATLAB to get MATLAB timings for comparison.

Usage (from warehouse root):
    python python_port/run_batch_validation.py
"""

import sys
import time
import json
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from python_port.generate_config import generate_random_config, save_config_json
from python_port.admm_core import run_admm_core, warmup_parallel_pool

RESULTS_JSON = Path(__file__).parent / 'batch_results.json'
SEQ_TIMEOUT  = 20.0   # skip scenario if sequential takes longer (s)

# ===========================================================================
# Scenario parameters
# ===========================================================================
DT_VAL   = 3.0   # road traversal time (s)
T_VAL    = 2.0   # headway between vehicles at same entrance (s)

# (N, seed, max_per_int)  — all N=10, 20 seeds
SCENARIOS = [
    (10, 101, 6), (10, 102, 6), (10, 103, 6), (10, 104, 6), (10, 105, 6),
    (10, 201, 6), (10, 202, 6), (10, 203, 6), (10, 204, 6), (10, 205, 6),
    (10, 301, 6), (10, 302, 6), (10, 303, 6), (10, 304, 6), (10, 305, 6),
    (10, 401, 6), (10, 402, 6), (10, 403, 6), (10, 404, 6), (10, 405, 6),
]

CONFIG_DIR = Path(__file__).parent / 'configs'


def main():
    CONFIG_DIR.mkdir(exist_ok=True)

    print('Pre-warming parallel pool...')
    warmup_parallel_pool()
    print('Pool ready.\n')

    print(f'{"#":>3}  {"N":>4}  {"seed":>6}  {"max/int":>7}  '
          f'{"k":>5}  {"cost":>10}  {"T_seq(s)":>10}  {"T_par(s)":>10}  {"speedup":>8}')
    print('-' * 75)

    results = []

    for i, (N, seed, max_int) in enumerate(SCENARIOS):
        # ── Generate config ──────────────────────────────────────────
        try:
            const, ap = generate_random_config(N, seed=seed, max_per_int=max_int,
                                               Dt=DT_VAL, T_val=T_VAL)
        except ValueError as e:
            print(f'{i+1:>3}  N={N} seed={seed}  SKIP: {e}')
            continue

        # Save JSON for MATLAB
        json_path = CONFIG_DIR / f'scenario_{i+1:02d}.json'
        save_config_json(const, ap, json_path)

        # ── Sequential run ────────────────────────────────────────────
        const_seq = {**const, 'useParallel': False, 'verbose': False}
        t0 = time.time()
        (_, _, _, _, _, dc_seq, k_seq, _, _, _) = run_admm_core(const_seq, ap)
        T_seq = time.time() - t0

        if T_seq > SEQ_TIMEOUT:
            print(f'{i+1:>3}  {N:>4}  {seed:>6}  {max_int:>7}  '
                  f'{"--":>5}  {"--":>10}  {T_seq:>10.2f}  {"SKIP":>10}  {"--":>8}')
            continue

        # ── Parallel run (pool already warm) ─────────────────────────
        const_par = {**const, 'useParallel': True, 'verbose': False}
        t0 = time.time()
        (_, _, _, _, _, dc_par, k_par, _, _, _) = run_admm_core(const_par, ap)
        T_par = time.time() - t0

        speedup = T_seq / max(T_par, 0.001)
        cost = float(dc_par[-1])

        results.append({
            'i': i + 1, 'N': N, 'seed': seed, 'max_int': max_int,
            'k': k_par, 'cost': cost,
            'T_seq': T_seq, 'T_par': T_par, 'speedup': speedup,
            'json': str(json_path),
        })

        print(f'{i+1:>3}  {N:>4}  {seed:>6}  {max_int:>7}  '
              f'{k_par:>5}  {cost:>10.4f}  {T_seq:>10.2f}  {T_par:>10.2f}  {speedup:>8.2f}x')

    # ── Summary ───────────────────────────────────────────────────────
    if results:
        avg_seq = np.mean([r['T_seq'] for r in results])
        avg_par = np.mean([r['T_par'] for r in results])
        avg_spd = np.mean([r['speedup'] for r in results])
        max_par = max(r['T_par'] for r in results)

        print('-' * 75)
        print(f'{"Avg":>3}  {"":>4}  {"":>6}  {"":>7}  '
              f'{"":>5}  {"":>10}  {avg_seq:>10.2f}  {avg_par:>10.2f}  {avg_spd:>8.2f}x')
        print()
        print(f'Average parallel T_solve : {avg_par:.2f} s')
        print(f'Max     parallel T_solve : {max_par:.2f} s')
        print()
        if max_par < 2.0:
            print(f'  -> Online MPC feasible with dt >= 2.0 s')
        elif max_par < 3.0:
            print(f'  -> Online MPC feasible with dt >= 3.0 s')
        else:
            print(f'  -> Recommend dt >= {max_par:.1f} s for safe MPC margin')
        print()
        print(f'Scenario JSONs saved to: {CONFIG_DIR}')
        print('Run batch_validate_matlab.m in MATLAB to compare.')

        # Save results to JSON for PPTX generation
        summary = {
            'avg_seq': float(avg_seq), 'avg_par': float(avg_par),
            'avg_speedup': float(avg_spd), 'max_par': float(max_par),
        }
        with open(RESULTS_JSON, 'w') as f:
            json.dump({'results': results, 'summary': summary}, f, indent=2)
        print(f'Results saved to: {RESULTS_JSON}')


if __name__ == '__main__':
    main()
