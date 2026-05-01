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
                   required=True)
    p.add_argument('--n_int', type=int, default=6,
                   help='[random mode] number of intersections')
    p.add_argument('--n_ports', type=int, default=None,
                   help='[random mode] number of external ports (default: 2*n_int)')
    p.add_argument('--spec',  type=str, default=None,
                   help='[manual mode] path to a .py file with `coords` and `ports`')
    p.add_argument('--grid_size', type=int, default=6,
                   help='[interactive mode] grid size for the picker (default 6)')
    p.add_argument('--N',     type=int, default=8,
                   help='Number of vehicles')
    p.add_argument('--seed',  type=int, default=42)
    p.add_argument('--max_iter', type=int, default=200,
                   help='Max ADMM iterations (default 200)')
    p.add_argument('--Dt',    type=float, default=5.0,
                   help='Road traversal time (uniform across all roads for now)')
    p.add_argument('--T_val', type=float, default=2.0,
                   help='Headway (s) between same-entrance vehicles. Paper '
                        'uses 2.0; smaller = more contention = harder to '
                        'converge.')
    p.add_argument('--T_ent', type=float, default=0.0,
                   help='Stagger (s) between vehicles from different entrances.')
    p.add_argument('--rho', type=float, default=1.0,
                   help='ADMM penalty ρ1=ρ2 (default 1.0). For big networks '
                        '(n_int>=15) try 0.3 or 0.1 if it diverges.')
    p.add_argument('--adaptive_rho', action='store_true',
                   help='Enable Boyd-2011 adaptive ρ schedule (auto-tunes ρ '
                        'based on r/s ratio). Useful when default diverges.')
    p.add_argument('--alpha', type=float, default=1.0,
                   help='Boyd-2011 over/under-relaxation factor (default 1.0 '
                        '= standard ADMM). α∈(0,1) damps oscillation on big '
                        'networks; α∈(1,2) accelerates on convex problems.')
    p.add_argument('--parallel', action='store_true',
                   help='Run ADMM intersection updates in parallel')
    p.add_argument('--quiet', action='store_true',
                   help='Suppress per-iteration ADMM logs')
    p.add_argument('--plot',  action='store_true',
                   help='Show topology + 5 ADMM result figures (matplotlib GUI)')
    p.add_argument('--save_plots', type=str, default=None,
                   help='Save plots as PNGs to this directory (headless, no GUI)')
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
    # max_per_int set to N so a single intersection isn't a bottleneck for sampling
    const, ap = generate_random_config(
        N=args.N, seed=args.seed + 1000,
        max_per_int=args.N, Dt=args.Dt,
        T_val=args.T_val, T_ent=args.T_ent,
        max_iter=args.max_iter, topology=t)
    print()
    _print_vehicles(const)

    # ── Run ADMM ────────────────────────────────────────────────────
    const['useParallel']      = args.parallel
    const['verbose']          = not args.quiet
    const['rho1']             = args.rho
    const['rho2']             = args.rho
    const['use_adaptive_rho'] = args.adaptive_rho
    const['alpha_relax']      = args.alpha
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

        if args.save_plots:
            from pathlib import Path as _P
            outdir = _P(args.save_plots).resolve()
            outdir.mkdir(parents=True, exist_ok=True)
            fig0.savefig(outdir / 'fig0_topology.png', dpi=120, bbox_inches='tight')
            for i, f in enumerate(figs, start=1):
                f.savefig(outdir / f'fig{i}.png', dpi=120, bbox_inches='tight')
            print(f'\nSaved plots to: {outdir}')
        if args.plot:
            plt.show()


if __name__ == '__main__':
    main()
