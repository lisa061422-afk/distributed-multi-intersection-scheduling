"""
End-to-end demo for arbitrary 4-way topologies.

Random mode  : generate a random connected graph with n_int intersections,
                random vehicles, run ADMM, print convergence.
Manual mode  : import a coords / ports spec from a Python file and do the same.

Usage (from the warehouse root):

    python python_port/run_arbitrary_demo.py --mode random --n_int 6 --N 8 --seed 42
    python python_port/run_arbitrary_demo.py --mode manual --spec my_topology.py --N 8

A manual spec file is a plain Python module that defines two module-level
variables:

    coords = {1: (0, 1), 2: (1, 1), 3: (1, 0), 4: (0, 0)}
    ports  = [(1,'N'),(1,'W'),(2,'N'),(2,'E'),
              (3,'E'),(3,'S'),(4,'S'),(4,'W')]

(See ``manual_topology`` in topology.py for the full direction conventions.)
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Make the package importable when run as `python python_port/run_arbitrary_demo.py`
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from python_port import defaults as _D
from python_port.topology import (manual_topology, random_topology, Topology,
                                    plot_topology)
from python_port.generate_config import generate_random_config
from python_port.admm_core import run_admm_core
from python_port.pick_topology import pick_topology_interactive


def _load_manual_spec(spec_path: str) -> Topology:
    """Import a Python file and read its ``coords`` / ``ports`` (and optional
    ``excluded_edges`` / ``name``) symbols.

    excluded_edges format: iterable of {i, j} pairs. Example:
        excluded_edges = [{1, 4}, (2, 3)]   # don't connect I1-I4 or I2-I3
    """
    import importlib.util
    spec_path = Path(spec_path).resolve()
    if not spec_path.exists():
        raise FileNotFoundError(f'spec file not found: {spec_path}')
    spec = importlib.util.spec_from_file_location('manual_spec', str(spec_path))
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if not hasattr(mod, 'coords') or not hasattr(mod, 'ports'):
        raise ValueError(
            f'{spec_path} must define module-level `coords` and `ports`')
    name = getattr(mod, 'name', spec_path.stem)
    excluded = getattr(mod, 'excluded_edges', None)
    return manual_topology(coords=mod.coords, ports=mod.ports,
                            excluded_edges=excluded, name=name)


def _print_topology(t: Topology) -> None:
    print(f'Topology "{t.name}":')
    print(f'  n_int   = {t.n_int}')
    print(f'  n_road  = {t.n_road}')
    print(f'  n_ports = {t.n_ports}')
    print(f'  n_agents (incl terminal) = {t.n_agents}')
    print(f'  # OD pairs in route_dict = {len(t.route_dict)}')


def _print_vehicles(const: dict) -> None:
    print('Vehicles (0-indexed agent chains):')
    for n in range(const['N']):
        chain = const['pathInfo_agent_chain'][n][0]
        alpha = float(const['alpha_tilde'][n][0])
        ddl   = float(const['deadline'][n][0])
        print(f'  n={n}: chain={chain}  alpha_tilde={alpha:.2f}  ddl={ddl:.2f}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--mode',  choices=['random', 'manual', 'interactive'],
                   default='manual',
                   help='random | manual | interactive (default: manual)')
    p.add_argument('--n_int', type=int, default=_D.N_INT_DEFAULT,
                   help='[random mode] number of intersections')
    p.add_argument('--n_ports', type=int, default=None,
                   help='[random mode] number of external ports (default: 2*n_int)')
    p.add_argument('--spec',  type=str,
                   default=str(Path(__file__).resolve().parent / 'warehouse_2x2.py'),
                   help='[manual mode] path to a .py file with `coords` and `ports` '
                        '(default: warehouse_2x2.py)')
    p.add_argument('--grid_size', type=int, default=_D.GRID_SIZE,
                   help='[interactive mode] grid size for the picker')
    p.add_argument('--N',     type=int, default=_D.N_DEFAULT,
                   help='Number of vehicles')
    p.add_argument('--seed',  type=int, default=_D.SEED_DEFAULT)
    p.add_argument('--max_iter', type=int, default=_D.MAX_ITER,
                   help='Max ADMM iterations')
    p.add_argument('--Dt',    type=float, default=_D.DT,
                   help='Road traversal time (uniform across all roads for now)')
    p.add_argument('--T_val', type=float, default=_D.T_VAL,
                   help='Headway (s) between same-entrance vehicles. Paper '
                        'uses 2.0; smaller = more contention = harder to '
                        'converge.')
    p.add_argument('--T_ent', type=float, default=_D.T_ENT,
                   help='Stagger (s) between vehicles from different entrances.')
    p.add_argument('--rho', type=float, default=_D.RHO1,
                   help='ADMM penalty rho1=rho2. For big networks '
                        '(n_int>=15) try 0.3 or 0.1 if it diverges.')
    p.add_argument('--adaptive_rho', action='store_true',
                   help='Enable Boyd-2011 adaptive rho schedule (auto-tunes rho '
                        'based on r/s ratio). Useful when default diverges.')
    p.add_argument('--alpha', type=float, default=_D.ALPHA_RELAX,
                   help='Boyd-2011 over/under-relaxation factor (1.0 = standard '
                        'ADMM). alpha in (0,1) damps oscillation; (1,2) accelerates.')
    p.add_argument('--parallel', action=argparse.BooleanOptionalAction, default=True,
                   help='Run ADMM intersection updates in parallel (default: on). '
                        'Use --no-parallel to disable.')
    p.add_argument('--quiet', action='store_true',
                   help='Suppress per-iteration ADMM logs')
    p.add_argument('--plot',  action=argparse.BooleanOptionalAction, default=True,
                   help='Show plots in GUI windows (default: on). '
                        'Use --no-plot to disable.')
    p.add_argument('--save_plots', type=str, default='my_plots',
                   help='Save plots as PNGs to this directory (default: my_plots/). '
                        'Pass empty string "" to disable.')
    p.add_argument('--first_iter_timeout', type=float, default=None,
                   help='If first ADMM iteration exceeds this many seconds, '
                        'abandon the run (used for seed sweeping).')
    p.add_argument('--max_per_int', type=int, default=None,
                   help='Hard cap on vehicles per intersection during random '
                        'config generation. Generation FAILS (ValueError, exit 2) '
                        'if cap cannot be met — caller should mark the seed '
                        'invalid. Default = N (no cap).')
    args = p.parse_args()

    # ── Build topology ──────────────────────────────────────────────
    if args.mode == 'random':
        t = random_topology(n_int=args.n_int, n_ports=args.n_ports, seed=args.seed)
    elif args.mode == 'manual':
        if not args.spec:
            p.error('--spec is required for --mode manual')
        t = _load_manual_spec(args.spec)
    elif args.mode == 'interactive':
        t, _, _ = pick_topology_interactive(grid_size=args.grid_size)
    _print_topology(t)

    # ── Generate vehicles ───────────────────────────────────────────
    cap = args.max_per_int if args.max_per_int is not None else args.N
    try:
        const, ap = generate_random_config(
            N=args.N, seed=args.seed + 1000,
            max_per_int=cap, Dt=args.Dt,
            T_val=args.T_val, T_ent=args.T_ent,
            max_iter=args.max_iter, topology=t)
    except ValueError as e:
        print(f'[Density reject] {e}')
        sys.exit(2)
    print()
    _print_vehicles(const)

    # ── Run ADMM ────────────────────────────────────────────────────
    const['useParallel']      = args.parallel
    const['verbose']          = not args.quiet
    const['rho1']             = args.rho
    const['rho2']             = args.rho
    const['use_adaptive_rho'] = args.adaptive_rho
    const['alpha_relax']      = args.alpha
    const['first_iter_timeout'] = args.first_iter_timeout
    if args.rho != 1.0 or args.adaptive_rho or args.alpha != 1.0:
        print(f'  rho={args.rho}  adaptive_rho={args.adaptive_rho}  alpha={args.alpha}')
    print('\nRunning ADMM...')
    t0 = time.time()
    res = run_admm_core(const, ap)
    x_prev, y_prev, _, res_r, res_s, delay_costs, k, T_admm, _, _ = res
    elapsed = time.time() - t0

    converged = (res_r[k-1] < const['tol_r']) and (res_s[k-1] < const['tol_s'])
    print(f'\n{"=== Converged" if converged else "=== Did not converge"} ===')
    print(f'  iterations    = {k} / {const["max_iter"]}')
    print(f'  elapsed       = {elapsed:.2f}s')
    print(f'  delay cost    = {delay_costs[k-1]:.4f}')
    print(f'  final r, s    = {res_r[k-1]:.4f}, {res_s[k-1]:.4f}')
    print(f'  tol_r, tol_s  = {const["tol_r"]}, {const["tol_s"]}')

    # Per-vehicle delay summary
    terminal_0 = const['terminal_id_0idx']
    print('\nPer-vehicle terminal time vs deadline:')
    for n in range(const['N']):
        xv  = x_prev[terminal_0][n]
        ddl = float(const['deadline'][n][0])
        if xv is not None:
            t_term = float(xv[0])
            delay  = max(t_term - ddl, 0.0)
            print(f'  n={n}: t_term={t_term:7.3f}  ddl={ddl:7.3f}  delay={delay:6.3f}')

    # ── FCFS baseline (always runs after optimal, fast) ────────────
    from python_port.fcfs import run_fcfs, plot_fcfs_local_panel, print_fcfs_summary
    fcfs_res = run_fcfs(const, ap)   # default H=0; pathInfo_c already encodes
                                      # the vehicle's full service duration at
                                      # each intersection.
    print_fcfs_summary(const, fcfs_res)

    # ── Auto-save self-contained batch dir ──────────────────────────
    # batches/seed_<N>_<seed>/ contains:
    #   data.pkl          — full state for re-export / replay
    #   figs/*.png        — all plots from this run
    #   demo/*.js         — demo-ready optimal.js + fcfs.js
    import pickle
    from python_port.export_to_demo import export_optimal_js, export_fcfs_js
    from python_port.export_tree_to_demo import (export_tree_static_js,
                                                   export_tree_interactive_js)
    batch_dir = (Path(__file__).resolve().parent / 'batches' /
                 f'seed_N{args.N}_s{args.seed}')
    batch_dir.mkdir(parents=True, exist_ok=True)
    (batch_dir / 'figs').mkdir(exist_ok=True)
    (batch_dir / 'demo').mkdir(exist_ok=True)

    local_tree_cache = res[2]
    with open(batch_dir / 'data.pkl', 'wb') as f:
        pickle.dump({
            'mode': args.mode, 'spec': getattr(args, 'spec', None),
            'N': args.N, 'seed': args.seed,
            'Dt': args.Dt, 'T_val': args.T_val, 'T_ent': args.T_ent,
            'const': const, 'ap': ap,
            'x_prev': x_prev, 'y_prev': y_prev,
            'fcfs_res': fcfs_res,
            'local_tree_cache': local_tree_cache,
            'converged': converged, 'k': k, 'T_admm': T_admm,
        }, f)
    if converged:
        scenario = f'N{args.N}_s{args.seed}'
        export_optimal_js(const, x_prev, y_prev,
                          scenario_name=f'{scenario} · Optimal',
                          output_file=batch_dir / 'demo' / 'optimal.js',
                          policy_name='optimal')
        export_fcfs_js(const, fcfs_res,
                       scenario_name=f'{scenario} · FCFS',
                       output_file=batch_dir / 'demo' / 'fcfs.js',
                       policy_name='fcfs')
        # Tree exports — group/scene placeholders, caller can rename when copying
        export_tree_static_js(local_tree_cache,
                              group=f'N{args.N}', scene=f's{args.seed}',
                              out_path=batch_dir / 'demo' / 'tree_optimal.js')
        export_tree_interactive_js(local_tree_cache, const, x_prev,
                                     group=f'N{args.N}', scene=f's{args.seed}',
                                     out_path=batch_dir / 'demo' / 'interactive_tree.js')
    print(f'\n[batch] saved → {batch_dir}')
    # Redirect plot save into the batch dir if user didn't override.
    if args.save_plots == 'my_plots':
        args.save_plots = str(batch_dir / 'figs')

    # ── Optional plotting ──────────────────────────────────────────
    if args.plot or args.save_plots:
        import matplotlib
        if args.save_plots and not args.plot:
            matplotlib.use('Agg')   # headless
        import matplotlib.pyplot as plt
        from python_port.run_results import plot_admm_results

        fig0, ax0 = plt.subplots(figsize=(7, 7))
        plot_topology(t, ax=ax0)

        figs = plot_admm_results(
            const, x_prev, y_prev,
            res[2],            # local_tree_cache
            res_r, res_s, delay_costs, k)

        # FCFS local panel — same renderer (_local_panel) as fig5
        fig_fcfs = plot_fcfs_local_panel(const, fcfs_res, title_prefix='FCFS')

        if args.save_plots:
            from pathlib import Path as _P
            outdir = _P(args.save_plots).resolve()
            outdir.mkdir(parents=True, exist_ok=True)
            fig0.savefig(outdir / 'fig0_topology.png', dpi=120, bbox_inches='tight')
            for i, f in enumerate(figs, start=1):
                f.savefig(outdir / f'fig{i}.png', dpi=120, bbox_inches='tight')
            fig_fcfs.savefig(outdir / 'fig6_fcfs_local.png', dpi=120, bbox_inches='tight')
            print(f'\nSaved plots to: {outdir}')
        if args.plot:
            plt.show()


if __name__ == '__main__':
    main()
