"""
直接 Run 这个文件就能跑（VS Code 里 F5 / 右上角 ▶️）。

改下面 CONFIG 区里的参数，存盘后再 Run 就用新参数。

不用敲任何 PowerShell 命令。
"""

# =============================================================================
#  CONFIG —— 改这一段
# =============================================================================

# ── 模式：选一个 ──
MODE = 'random'   # 'interactive' | 'random' | 'manual'

# ── 通用参数 ──
N         = 15           # 车辆数
SEED      = 43          # RNG 种子
MAX_ITER  = 200         # ADMM 最大迭代
DT        = 2.0         # road 通过时间 (s)
PARALLEL  = False       # 路口 agent 并行（True 在大问题上更快）
VERBOSE   = True        # 打印每轮 ADMM 残差（False 就静默跑，看似卡住）

# ── ADMM 调参 ──
RHO          = 1.0      # 惩罚 ρ；大网络散开时调小（0.3 / 0.1）
ALPHA        = 1.0      # over/under-relaxation；<1 抑制振荡
ADAPTIVE_RHO = False    # 自动调 ρ

# ── 各模式特定参数 ──
RANDOM_N_INT   = 6                              # 仅 random 模式
RANDOM_N_PORTS = None                           # None = 自动 = 2*n_int
MANUAL_SPEC    = 'python/example_manual_topo.py'   # 仅 manual 模式
PICKER_GRID    = 6                              # 仅 interactive 模式

# ── 输出 ──
PLOT       = True       # 弹 matplotlib 窗口
SAVE_PLOTS = None       # None 不存；或写路径 'C:/.../plots/' 存 PNG

# =============================================================================
#  以下是粘合代码，一般不用改
# =============================================================================

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from python.topology import (manual_topology, random_topology,
                             plot_topology)
from python.generate_config import generate_random_config
from python.admm_core import run_admm_core
from python.pick_topology import pick_topology_interactive


def _load_manual_spec(spec_path):
    import importlib.util
    spec_path = Path(spec_path).resolve()
    if not spec_path.exists():
        raise FileNotFoundError(f'spec 文件没找到: {spec_path}')
    spec = importlib.util.spec_from_file_location('manual_spec', str(spec_path))
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return manual_topology(
        coords=mod.coords, ports=mod.ports,
        excluded_edges=getattr(mod, 'excluded_edges', None),
        name=getattr(mod, 'name', spec_path.stem))


def main():
    # ── 1. 建拓扑 ─────────────────────────────────────────────────
    if MODE == 'random':
        t = random_topology(n_int=RANDOM_N_INT, n_ports=RANDOM_N_PORTS, seed=SEED)
    elif MODE == 'manual':
        t = _load_manual_spec(MANUAL_SPEC)
    elif MODE == 'interactive':
        t, _, _ = pick_topology_interactive(grid_size=PICKER_GRID)
    else:
        raise ValueError(f'未知 MODE: {MODE!r}')

    print(f'\n=== Topology "{t.name}" ===')
    print(f'  n_int={t.n_int}, n_road={t.n_road}, n_ports={t.n_ports}, '
          f'#OD={len(t.route_dict)}')

    # ── 2. 生成车辆 + ADMM const ────────────────────────────────
    const, ap = generate_random_config(
        N=N, seed=SEED + 1000, max_per_int=N,
        Dt=DT, max_iter=MAX_ITER, topology=t)

    print(f'\n=== Vehicles ===')
    for n in range(const['N']):
        chain = const['pathInfo_agent_chain'][n][0]
        alpha_n = float(const['alpha_tilde'][n][0])
        ddl     = float(const['deadline'][n][0])
        print(f'  n={n}: chain={chain}  α̃={alpha_n:.2f}  ddl={ddl:.2f}')

    # ── 3. 跑 ADMM ─────────────────────────────────────────────
    const['useParallel']      = PARALLEL
    const['verbose']          = VERBOSE
    const['rho1']             = RHO
    const['rho2']             = RHO
    const['alpha_relax']      = ALPHA
    const['use_adaptive_rho'] = ADAPTIVE_RHO

    print(f'\n=== Running ADMM (ρ={RHO}, α={ALPHA}, '
          f'adaptive={ADAPTIVE_RHO}, max_iter={MAX_ITER}) ===')
    t0 = time.time()
    res = run_admm_core(const, ap)
    x_prev, y_prev, ltc, res_r, res_s, delay_costs, k, _, _, _ = res
    elapsed = time.time() - t0

    converged = (res_r[k-1] < const['tol_r']) and (res_s[k-1] < const['tol_s'])
    status = 'Converged' if converged else 'Did NOT converge'
    print(f'\n=== {status} ===')
    print(f'  iterations    = {k} / {MAX_ITER}')
    print(f'  elapsed       = {elapsed:.2f}s')
    print(f'  delay cost    = {delay_costs[k-1]:.4f}')
    print(f'  final r, s    = {res_r[k-1]:.4f}, {res_s[k-1]:.4f}')

    terminal_0 = const['terminal_id_0idx']
    print(f'\n=== Per-vehicle terminal time ===')
    for n in range(const['N']):
        xv  = x_prev[terminal_0][n]
        ddl = float(const['deadline'][n][0])
        if xv is not None:
            t_term = float(xv[0])
            delay  = max(t_term - ddl, 0.0)
            print(f'  n={n}: t_term={t_term:7.3f}  ddl={ddl:7.3f}  '
                  f'delay={delay:6.3f}')

    # ── 4. 画图 ────────────────────────────────────────────────
    if PLOT or SAVE_PLOTS:
        import matplotlib
        if SAVE_PLOTS and not PLOT:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from python.run_results import plot_admm_results

        fig0, ax0 = plt.subplots(figsize=(7, 7))
        plot_topology(t, ax=ax0)

        figs = plot_admm_results(
            const, x_prev, y_prev, ltc,
            res_r, res_s, delay_costs, k)

        if SAVE_PLOTS:
            outdir = Path(SAVE_PLOTS).resolve()
            outdir.mkdir(parents=True, exist_ok=True)
            fig0.savefig(outdir / 'fig0_topology.png', dpi=120, bbox_inches='tight')
            for i, f in enumerate(figs, start=1):
                f.savefig(outdir / f'fig{i}.png', dpi=120, bbox_inches='tight')
            print(f'\n图已存到: {outdir}')
        if PLOT:
            plt.show()


if __name__ == '__main__':
    main()
