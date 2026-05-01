"""
MPC online demo.

Runs three scenarios on seed=201 N=10 config:
  A) Static ADMM (single solve at t=0)
  B) MPC, no disturbance (re-plans every 0.5s, deterministic motion)
  C) MPC, with speed disturbance std=0.1 m/s (re-plans every 0.5s)

Usage (from warehouse root):
    python python_port/run_mpc_demo.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))


def main():
    import time
    import numpy as np
    from python_port.run_validation import load_config
    from python_port.admm_core import run_admm_core, warmup_parallel_pool
    from python_port.mpc_loop import MPCScheduler

    # ── Load config ───────────────────────────────────────────────────
    json_path = Path(__file__).parent / 'validation_config.json'
    print(f'Loading config from {json_path}')
    const, agent_participation = load_config(str(json_path))
    N = const['N']
    print(f'N={N}  max_iter={const["max_iter"]}  Dt={const["Dt"]}')

    # Pre-warm pool once
    print('\nWarming up parallel pool...')
    t0 = time.time()
    warmup_parallel_pool()
    print(f'Pool ready in {time.time()-t0:.2f}s')

    # ── Scenario A: Static ADMM at t=0 ───────────────────────────────
    print('\n' + '='*60)
    print('A) Static ADMM (single solve, t=0)')
    print('='*60)
    const_static = {**const, 'useParallel': True}
    t0 = time.time()
    x_s, y_s, _, _, _, dc_s, k_s, T_s, _, _ = run_admm_core(const_static, agent_participation)
    T_static = time.time() - t0
    terminal_0 = const.get('terminal_id_0idx', const.get('n_agents', 9) - 1)
    terminal_static = [float(x_s[terminal_0][n][0]) for n in range(N)]
    delays_static = [max(0.0, terminal_static[n] - float(const['deadline'][n][0]))
                     for n in range(N)]
    print(f'Converged k={k_s}  T_solve={T_s:.2f}s  total_delay={dc_s[k_s-1]:.4f}')
    print(f'  terminal[0..4]: {[round(x,3) for x in terminal_static[:5]]}')
    print(f'  delays[0..4]:   {[round(d,3) for d in delays_static[:5]]}')

    # ── Scenario B: MPC, no disturbance ──────────────────────────────
    print('\n' + '='*60)
    print('B) MPC receding-horizon, no disturbance (dt=0.5s, 6 steps)')
    print('='*60)
    sched_B = MPCScheduler(const, agent_participation)
    hist_B = sched_B.simulate(
        n_steps=6, dt=0.5, disturbance_std=0.0,
        use_parallel=True, verbose=True, seed=0)

    delays_B = [h['delay_cost'] for h in hist_B]
    T_B = [h['T_solve'] for h in hist_B]
    print(f'\n  Delay cost per step: {[round(d,4) for d in delays_B]}')
    print(f'  T_solve per step:   {[round(t,2) for t in T_B]} s')
    print(f'  Final delay cost: {delays_B[-1]:.4f}  (static: {dc_s[k_s-1]:.4f})')

    # ── Scenario C: MPC with speed disturbance ────────────────────────
    print('\n' + '='*60)
    print('C) MPC receding-horizon, disturbance std=0.1 m/s (dt=0.5s, 6 steps)')
    print('='*60)
    sched_C = MPCScheduler(const, agent_participation)
    hist_C = sched_C.simulate(
        n_steps=6, dt=0.5, disturbance_std=0.1,
        use_parallel=True, verbose=True, seed=42)

    delays_C = [h['delay_cost'] for h in hist_C]
    T_C = [h['T_solve'] for h in hist_C]
    print(f'\n  Delay cost per step: {[round(d,4) for d in delays_C]}')

    # Show alpha_tilde evolution under disturbance
    print('\n  alpha_tilde and positions evolving under disturbance:')
    print(f'  {"t":>5}  {"alpha0":>8}  {"alpha1":>8}  {"pos0":>8}  {"pos1":>8}')
    for h in hist_C:
        print(f'  {h["t"]:>5.1f}  '
              f'{h["alpha_tilde"][0]:>8.3f}  {h["alpha_tilde"][1]:>8.3f}  '
              f'{h["positions"][0]:>8.3f}  {h["positions"][1]:>8.3f}')
    a0_init = float(const['alpha_tilde'][0][0])
    a1_init = float(const['alpha_tilde'][1][0])
    print(f'  (initial alpha_tilde: a0={a0_init:.3f}  a1={a1_init:.3f})')

    # ── Summary ───────────────────────────────────────────────────────
    print('\n' + '='*60)
    print('Summary')
    print('='*60)
    print(f'  Static ADMM:          T_solve={T_s:.2f}s  cost={dc_s[k_s-1]:.4f}')
    print(f'  MPC no disturbance:   avg T={sum(T_B)/len(T_B):.2f}s/step  '
          f'final cost={delays_B[-1]:.4f}')
    print(f'  MPC with disturbance: avg T={sum(T_C)/len(T_C):.2f}s/step  '
          f'final cost={delays_C[-1]:.4f}')
    avg_T = sum(T_B) / len(T_B)
    print(f'\n  T_solve={avg_T:.2f}s → feasible MPC time step: dt ≥ {avg_T:.1f}s')
    print(f'  (e.g. dt=2.0s is practical for v_max=1.0 m/s AMRs)')
    print(f'\n  Use warmup_parallel_pool() at system startup to eliminate'
          f' first-call latency.')


if __name__ == '__main__':
    main()
