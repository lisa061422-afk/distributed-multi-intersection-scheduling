"""
Run ADMM and display all results matching MATLAB plot_C_ADMM2.m exactly.

Usage (from warehouse root, or open in VS Code and press Run):
    python python_port/run_results.py

Figures:
  1. ADMM Residuals (r, s) — log scale
  2. Delay cost over iterations
  3. Gantt — Merging Zones 1-4  (height proportional to vehicles per agent)
  4. Gantt — Roads 1-4 + Terminal
  5. Local panels — per-intersection space allocation (M1..M5) + delay hatches
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec, GridSpecFromSubplotSpec

from python_port.run_validation import load_config
from python_port.admm_core import run_admm_core, warmup_parallel_pool


# ===========================================================================
# Style constants (matching MATLAB plot_C_ADMM2.m)
# ===========================================================================
FS_AX  = 20   # axis tick labels + axis labels
FS_TIT = 18   # subplot titles
FS_TXT = 16   # bar text
BAR_H  = 0.72
BAR_EDGE_COLOR = (0.53, 0.81, 0.92)   # light blue edge

# ── Local-panel constants (matching plot_local_schedule_final_into_panel_1.m) ──
SPACE_COLORS = np.array([
    [0.16, 0.95, 0.16],   # M1 green
    [0.98, 0.84, 0.00],   # M2 yellow
    [1.00, 0.55, 0.00],   # M3 orange
    [0.12, 0.66, 0.84],   # M4 blue
    [0.95, 0.10, 0.10],   # M5 red
])
LINE_COLOR     = (0.35, 0.75, 0.95)   # light blue baseline
GEN_LINE_COLOR = (0.55, 0.10, 0.75)   # purple gen line
FS_LOCAL       = 11                   # font size inside local panels


def _draw_delay_hatch(ax, x1, x2, y1, y2,
                      color=(0.75, 0.75, 0.75), spacing=0.18):
    """Diagonal hatch (slope +1) inside [x1,x2]×[y1,y2]."""
    if x2 <= x1 or y2 <= y1:
        return
    H = y2 - y1
    c = x1 - H
    while c <= x2:
        xa = max(x1, c); xb = min(x2, c + H)
        if xb > xa:
            ya = y1 + (xa - c); yb = y1 + (xb - c)
            ax.plot([xa, xb], [ya, yb], '-', color=color,
                    linewidth=0.8, clip_on=True)
        c += spacing
    ax.plot([x1, x2, x2, x1, x1], [y1, y1, y2, y2, y1],
            '-', color=(0.82, 0.82, 0.82), linewidth=0.5)


def _local_panel(fig, gs_cell, NODES, path_nodes, agent_i, valid_systems, const,
                 x_prev=None, Dt=None,
                 show_gen_line=True, show_delay_patch=True, title=None):
    """
    Per-intersection local schedule (Nv vehicle rows on top + Ns space rows
    on bottom).  Equivalent of MATLAB plot_local_schedule_final_into_panel_1.

    agent_i    : 0-indexed intersection (0..3)
    path_nodes : list of node indices forming the optimal path (last one = leaf)
    """
    IntSpaceDB = const['IntSpaceDB']
    pathInfo   = const['pathInfo']
    pathInfo_agent_chain = const['pathInfo_agent_chain']
    alpha_tilde = const['alpha_tilde']
    if Dt is None:
        Dt = const['Dt']

    if not path_nodes:
        return []

    last_node  = NODES[path_nodes[-1]]
    gamma_last = last_node.gamma

    db = IntSpaceDB[agent_i]
    Ns = db['numSpaces']

    def _to_list(v):
        return v if isinstance(v, list) else [v]

    items = []
    for n in valid_systems:
        chain = pathInfo_agent_chain[n][0]
        if agent_i not in chain:
            continue
        posi      = chain.index(agent_i)
        route_idx = posi // 2

        rids = pathInfo[n]['routeId']
        if route_idx >= len(rids):
            continue
        rId = int(rids[route_idx])

        sp_raw = db['routeSpace'].get(rId)
        du_raw = db['routeDur'].get(rId)
        if sp_raw is None or du_raw is None:
            continue
        spaces = _to_list(sp_raw)
        durs   = _to_list(du_raw)
        if not spaces or not durs:
            continue

        tf_n = gamma_last[n] if n < len(gamma_last) else None
        if tf_n is None or not np.isfinite(tf_n):
            continue
        tf = float(tf_n)
        t0 = tf - sum(durs)

        segStart, segEnd = [], []
        cur = t0
        for d in durs:
            segStart.append(cur); segEnd.append(cur + d); cur += d

        genTime = float('nan')
        if posi == 0:
            at = alpha_tilde[n]
            try:
                genTime = float(at[0])
            except (TypeError, IndexError):
                genTime = float(at)
        elif x_prev is not None:
            prev_road = chain[posi - 1]
            if (prev_road < len(x_prev) and n < len(x_prev[prev_road])
                    and x_prev[prev_road][n] is not None):
                genTime = float(x_prev[prev_road][n][0]) + Dt

        items.append({
            'veh': n, 'k': route_idx + 1, 'rId': rId,
            'spaces': spaces, 'dur': durs,
            't0': t0, 'tf': tf,
            'segStart': segStart, 'segEnd': segEnd,
            'genTime': genTime,
        })

    if not items:
        return []

    items.sort(key=lambda it: it['veh'])
    Nv  = len(items)
    Nax = Nv + Ns

    all_t_min = [it['t0'] for it in items]
    all_t_max = [it['tf'] for it in items]
    all_gen   = [it['genTime'] for it in items if np.isfinite(it['genTime'])]
    xmin = min(all_t_min + all_gen) - 0.4 if all_gen else min(all_t_min) - 0.4
    xmax = max(all_t_max) + 0.4

    sub_gs = GridSpecFromSubplotSpec(Nax, 1, subplot_spec=gs_cell, hspace=0.05)
    ax_list = []

    # ── vehicle rows (top) ────────────────────────────────────────────
    for r, it in enumerate(items):
        ax = fig.add_subplot(sub_gs[r])
        ax.set_xlim(xmin, xmax); ax.set_ylim(0, 1.2)
        ax.set_yticks([]); ax.set_facecolor('white')
        ax.grid(True, alpha=0.16)
        for sp in ax.spines.values():
            sp.set_linewidth(1.0)

        edges = sorted(set([xmin, xmax] + it['segStart'] + it['segEnd']))
        occ   = [int(any(s <= t < e for s, e in zip(it['segStart'], it['segEnd'])))
                 for t in edges]
        ax.step(edges, occ, where='post', color=LINE_COLOR, linewidth=1.5)

        if show_delay_patch and np.isfinite(it['genTime']) and it['t0'] > it['genTime']:
            _draw_delay_hatch(ax, it['genTime'], it['t0'], 0, 1.0)

        for q, sp_id in enumerate(it['spaces']):
            x = it['segStart'][q]; w = it['segEnd'][q] - it['segStart'][q]
            ax.add_patch(mpatches.Rectangle(
                (x, 0), w, 1.0,
                facecolor=SPACE_COLORS[sp_id - 1], edgecolor='none'))

        if show_gen_line and np.isfinite(it['genTime']):
            ax.axvline(it['genTime'], linestyle='--',
                       color=GEN_LINE_COLOR, linewidth=1.6)

        ax.text(xmin - 0.005 * (xmax - xmin), 0.5,
                f'N{it["veh"] + 1}-K{it["k"]}',
                ha='right', va='center', fontsize=FS_LOCAL, clip_on=False)

        if r == 0 and title is not None:
            ax.set_title(title, fontsize=FS_TIT, pad=6)
        if r < Nax - 1:
            ax.set_xticklabels([])
        ax_list.append(ax)

    # ── space rows (bottom) ───────────────────────────────────────────
    for s_id in range(1, Ns + 1):
        r  = Nv + s_id - 1
        ax = fig.add_subplot(sub_gs[r])
        ax.set_xlim(xmin, xmax); ax.set_ylim(0, 1.2)
        ax.set_yticks([]); ax.set_facecolor('white')
        ax.grid(True, alpha=0.16)
        for sp in ax.spines.values():
            sp.set_linewidth(1.0)

        ax.plot([xmin, xmax], [0, 0], color=LINE_COLOR, linewidth=1.5)

        for it in items:
            for q, sp_id in enumerate(it['spaces']):
                if sp_id == s_id:
                    x = it['segStart'][q]; w = it['segEnd'][q] - it['segStart'][q]
                    ax.add_patch(mpatches.Rectangle(
                        (x, 0), w, 1.0, facecolor='none',
                        edgecolor=SPACE_COLORS[s_id - 1], linewidth=2.0))
                    ax.text(x + w / 2, 0.5, f'N{it["veh"] + 1}',
                            ha='center', va='center',
                            fontsize=FS_LOCAL, color='k', clip_on=True)

        ax.text(xmin - 0.001 * (xmax - xmin), 0.5, f'M{s_id}',
                ha='right', va='center', fontsize=FS_LOCAL, clip_on=False)

        if r < Nax - 1:
            ax.set_xticklabels([])
        else:
            ax.set_xlabel('Time (seconds)', fontsize=FS_LOCAL)
        ax_list.append(ax)

    return ax_list


def _make_colors(N):
    """
    Approximate MATLAB lines(N) brightened: colors = 0.7*lines + 0.3.
    Uses matplotlib's tab10/tab20 as base (closest to MATLAB lines).
    """
    matlab_lines = np.array([
        [0.00, 0.00, 1.00],
        [0.00, 0.50, 0.00],
        [1.00, 0.00, 0.00],
        [0.00, 0.75, 0.75],
        [0.75, 0.00, 0.75],
        [0.75, 0.75, 0.00],
        [0.25, 0.25, 0.25],
    ])
    # Tile for N > 7
    base = np.tile(matlab_lines, (int(np.ceil(N / 7)), 1))[:N]
    colors = 0.7 * base + 0.3 * np.ones_like(base)
    colors = np.clip(colors, 0, 1)
    # MATLAB special-cases N>=8: lighten row 8
    if N >= 8:
        colors[7] = 0.5 * colors[7] + 0.5 * np.array([1, 1, 1])
    return colors   # shape (N, 3), 0-indexed: colors[n] for vehicle n (0-indexed)


# ===========================================================================
# Data collection helpers (matching MATLAB collect_used_vehicles_for_agent)
# ===========================================================================

def _collect_agent_data(ag, x_prev, y_prev, pathInfo_agent_chain, N, kn=0,
                         terminal_id_0idx=8):
    """
    Returns lists of (veh_id_0idx, posK, st, en) for vehicles that actually
    appear on agent ag and have valid x/y values.
    ag is 0-indexed (0..n_agents-1). terminal_id_0idx is the ID of the
    terminal agent (which has no y variable); defaults to 8 for the
    original 4-int network.
    """
    used_veh = []
    pos_order = []
    st_list = []
    en_list = []

    for n in range(N):
        chain = pathInfo_agent_chain[n][kn]
        if ag not in chain:
            continue
        posi = chain.index(ag)          # 0-indexed position in chain
        posK = (posi + 2) // 2          # visit count (matches MATLAB ceil((posi+1)/2))

        xv = x_prev[ag][n]
        if xv is None or np.isnan(xv[0]):
            continue
        st = float(xv[0])

        if ag != terminal_id_0idx:
            yv = y_prev[ag][n]
            if yv is None or np.isnan(yv[0]):
                continue
            en = float(yv[0])
            if en <= st:
                continue
        else:
            en = st + 0.08   # terminal: short visible bar

        used_veh.append(n)
        pos_order.append(posK)
        st_list.append(st)
        en_list.append(en)

    return used_veh, pos_order, st_list, en_list


def _global_xlim(data_list):
    """
    Compute global [tmin, tmax] with 2% padding (matching MATLAB compute_global_xlim_from_data).
    data_list: list of (used_veh, pos_order, st_list, en_list) tuples.
    """
    all_st, all_en = [], []
    for _, _, st_list, en_list in data_list:
        all_st.extend(st_list)
        all_en.extend(en_list)
    if not all_st:
        return 0.0, 10.0
    t_min = min(all_st)
    t_max = max(all_en)
    pad = max(0.2, 0.02 * (t_max - t_min))
    return max(0.0, t_min - pad), t_max + pad


# ===========================================================================
# Bar drawing
# ===========================================================================

def _draw_bar(ax, st, en, row_idx, veh_id_0, posK, colors, show_text=True):
    """
    Draw one Gantt bar.
    row_idx : 1-based local row (1 = bottom, nLocal = top)
    veh_id_0: 0-indexed vehicle id (for color lookup)
    """
    dur = en - st
    y0  = row_idx - BAR_H / 2
    ax.add_patch(mpatches.FancyBboxPatch(
        (st, y0), dur, BAR_H,
        boxstyle='square,pad=0',
        facecolor=colors[veh_id_0],
        edgecolor=BAR_EDGE_COLOR,
        linewidth=1.0))
    if show_text and dur > 0:
        ax.text(st + dur / 2, row_idx,
                f'N{veh_id_0 + 1}-K{posK}',   # 1-indexed vehicle label (matches MATLAB)
                ha='center', va='center',
                fontsize=FS_TXT, color='k')


# ===========================================================================
# Plot helpers
# ===========================================================================

def _gantt_figure(group_data, group_titles, fig_size, suptitle, xlabel_bottom,
                  colors, bottom_margin=0.08, top_margin=0.06, gap_frac=0.04):
    """
    Build a Gantt figure with subplots whose heights are proportional to
    the number of vehicles that use each agent (matching MATLAB layout).

    group_data : list of (used_veh, pos_order, st_list, en_list)
    group_titles: list of str
    """
    n_rows = len(group_data)
    n_local = [max(len(d[0]), 1) for d in group_data]
    total_rows = sum(n_local)

    tmin, tmax = _global_xlim(group_data)

    # Height ratios for GridSpec
    fig = plt.figure(figsize=fig_size, facecolor='white')
    gs = GridSpec(
        n_rows, 1,
        figure=fig,
        height_ratios=n_local,
        hspace=gap_frac * n_rows / (1 - top_margin - bottom_margin),
        left=0.08, right=0.97,
        top=1 - top_margin, bottom=bottom_margin)

    for i, (used_veh, pos_order, st_list, en_list) in enumerate(group_data):
        ax = fig.add_subplot(gs[i])
        ax.set_facecolor('white')
        ax.tick_params(labelsize=FS_AX)
        for spine in ax.spines.values():
            spine.set_linewidth(1.0)

        nL = n_local[i]

        if not used_veh:
            ax.set_xlim(tmin, tmax)
            ax.set_ylim(0.5, 1.5)
            ax.set_yticks([1])
            ax.set_yticklabels(['-'], fontsize=FS_AX)
        else:
            for j, (n, posK, st, en) in enumerate(zip(used_veh, pos_order, st_list, en_list)):
                local_row = nL - j   # j=0 → top row = nL; last j → row 1
                _draw_bar(ax, st, en, local_row, n, posK, colors, show_text=True)

            ax.set_xlim(tmin, tmax)
            ax.set_ylim(0.5, nL + 0.5)
            ax.set_yticks(range(1, nL + 1))
            # yticklabels: flip so top ytick matches top vehicle
            ax.set_yticklabels(
                [f'N{n + 1}' for n in used_veh],   # already in draw order (top→bottom)
                fontsize=FS_AX)

        ax.set_title(group_titles[i], fontsize=FS_TIT)
        ax.grid(True)
        ax.box = True

        if i < n_rows - 1:
            ax.set_xticklabels([])
        else:
            ax.set_xlabel(xlabel_bottom, fontsize=FS_AX)

    fig.suptitle(suptitle, fontsize=FS_TIT, y=1 - top_margin / 2)
    return fig


# ===========================================================================
# Main
# ===========================================================================

def main():
    # ── Config mode switch ──────────────────────────────────────────
    MODE = 'random'        # 'manual' or 'random'

    # Random-only params (ignored in manual mode)
    RAND_N           = 15
    RAND_SEED        = 203
    RAND_MAX_PER_INT = 8

    # ── Optional overrides ──────────────────────────────────────────
    # Whatever you put here OVERRIDES the JSON. JSON is the default source
    # of truth — only add a key here when you want to force a different value.
    # Physical params (Dt, T_val, ...) belong in JSON's "physical_params",
    # NOT here — putting them here desyncs deadline (precomputed) from ADMM.
    OVERRIDE = {
        'useParallel': True,   # JSON doesn't store this — set here
        # ── Below are ALL in JSON. Uncomment only to force a different value: ──
        # 'rho1':              1.0,
        # 'rho2':              1.0,
        # 'weight':            1.5,
        # 'max_iter':          200,
        # 'tol_r':             1e-2,
        # 'tol_s':             1e-2,
        # 'use_pruning':       True,
        # 'use_weak_rule':     True,
        # 'use_adaptive_rho':  False,
        # 'timeout_int_s':     30,
    }

    if MODE == 'manual':
        json_path = Path(__file__).parent / 'validation_config.json'
        if not json_path.exists():
            print(f'ERROR: {json_path} not found.')
            sys.exit(1)
        print(f'Loading manual config from {json_path}')
        const, agent_participation = load_config(str(json_path))
    elif MODE == 'random':
        from python_port.generate_config import generate_random_config
        print(f'Generating random config: N={RAND_N}, seed={RAND_SEED}, '
              f'max_per_int={RAND_MAX_PER_INT}')
        const, agent_participation = generate_random_config(
            RAND_N, seed=RAND_SEED, max_per_int=RAND_MAX_PER_INT,
            Dt=2.0, T_val=2.0)
    else:
        print(f'ERROR: unknown MODE {MODE!r} (use "manual" or "random")')
        sys.exit(1)

    # Apply ADMM-hyperparam overrides
    for k_, v_ in OVERRIDE.items():
        const[k_] = v_
    print(f'Effective: Dt={const["Dt"]}  rho1={const["rho1"]}  weight={const["weight"]}  '
          f'max_iter={const["max_iter"]}  tol_r={const["tol_r"]}  tol_s={const["tol_s"]}  '
          f'useParallel={const["useParallel"]}')

    N          = const['N']
    terminal_0 = const['terminal_id_0idx']

    # ── Pre-warm parallel pool (only if running parallel) ─────────────
    if const['useParallel']:
        print('Warming up parallel pool...')
        warmup_parallel_pool()
    else:
        print('Sequential mode — skipping pool warmup.')

    # ── Run ADMM ─────────────────────────────────────────────────────
    const_run = {**const, 'verbose': False}
    t_wall = time.time()
    (x_prev, y_prev, local_tree_cache,
     residual_r, residual_s, delay_costs,
     k, _, _, _) = run_admm_core(const_run, agent_participation)
    t_wall = time.time() - t_wall

    # ── Print metrics ─────────────────────────────────────────────────
    sep = '=' * 55
    print()
    print(sep)
    print('ADMM Results')
    print(sep)
    print(f'  Computation time  : {t_wall:.3f} s')
    print(f'  Iterations        : {k}')
    print(f'  Final primal r    : {residual_r[-1]:.6f}')
    print(f'  Final dual   s    : {residual_s[-1]:.6f}')
    print()
    print(f'  {"Vehicle":>8}  {"Arrival":>10}  {"Deadline":>10}  {"Delay":>8}')
    total_delay = 0.0
    for n in range(N):
        xv  = x_prev[terminal_0][n]
        ddl = float(const['deadline'][n][0])
        arr = float(xv[0]) if xv is not None else float('nan')
        delay = max(arr - ddl, 0.0)
        total_delay += delay
        print(f'  {"N"+str(n+1):>8}  {arr:>10.4f}  {ddl:>10.4f}  {delay:>8.4f}')
    print(f'  {"-"*46}')
    print(f'  {"Overall delay":>8}  {"":>10}  {"":>10}  {total_delay:>8.4f}')
    print(f'  ADMM internal delay_cost (final) : {delay_costs[k-1]:.4f}')
    print(f'  diff (table − ADMM) : {total_delay - delay_costs[k-1]:+.4e}')
    print(sep)

    plot_admm_results(
        const, x_prev, y_prev, local_tree_cache,
        residual_r, residual_s, delay_costs, k)
    plt.show()


def plot_admm_results(const, x_prev, y_prev, local_tree_cache,
                       residual_r, residual_s, delay_costs, k):
    """Build figures from an ADMM run. Returns list of Figures.

    If ADMM converged (final r < tol_r AND final s < tol_s), returns 5 figures:
      [residuals, delay_cost, int_gantt, road_gantt, local_panels].
    If it did not converge, returns only the 2 diagnostic figures
      [residuals, delay_cost] — the schedule is incomplete so the Gantt /
    local-panel plots would be misleading.
    """
    N          = const['N']
    n_int      = const['n_int']
    n_road     = const['n_road']
    n_agents   = const['n_agents']
    terminal_0 = const['terminal_id_0idx']
    tol_r      = const['tol_r']
    tol_s      = const['tol_s']
    kn = 0

    converged = (float(residual_r[k - 1]) < tol_r) and \
                (float(residual_s[k - 1]) < tol_s)

    colors = _make_colors(N)
    iters  = np.arange(1, k + 1)

    # ── Figure 1: Residuals ───────────────────────────────────────────
    fig1 = plt.figure(figsize=(7.8, 5.2), facecolor='white')
    ax1  = fig1.add_axes([0.12, 0.14, 0.82, 0.76])
    ax1.semilogy(iters, residual_r, 'b-',  linewidth=2, label='Primal Residual')
    ax1.semilogy(iters, residual_s, 'r--', linewidth=2, label='Dual Residual')
    ax1.set_xlabel('Iteration', fontsize=FS_AX)
    ax1.set_ylabel('Residual (log scale)', fontsize=FS_AX)
    ax1.set_title('ADMM Residuals', fontsize=FS_TIT)
    ax1.tick_params(labelsize=FS_AX)
    ax1.legend(loc='best', fontsize=FS_TXT)
    ax1.grid(True)

    # ── Figure 2: Delay cost ──────────────────────────────────────────
    fig2 = plt.figure(figsize=(7.8, 5.2), facecolor='white')
    ax2  = fig2.add_axes([0.12, 0.14, 0.82, 0.76])
    ax2.plot(iters, delay_costs, 'b-o', linewidth=2, markersize=5)
    ax2.set_xlabel('Iteration', fontsize=FS_AX)
    ax2.set_ylabel('Total Time Delay Cost', fontsize=FS_AX)
    ax2.set_title('Delay Cost over Iterations', fontsize=FS_TIT)
    ax2.tick_params(labelsize=FS_AX)
    ax2.grid(True)

    # If ADMM did not converge, the schedule is not feasible / not aligned —
    # the Gantt charts and per-intersection panels would be misleading. Skip
    # them and return only the diagnostic plots so the caller can see WHY.
    if not converged:
        print(f'[plot_admm_results] ADMM did not converge '
              f'(r={float(residual_r[k-1]):.4f} >= tol_r={tol_r}, '
              f'or s={float(residual_s[k-1]):.4f} >= tol_s={tol_s}); '
              f'skipping Gantt + local-panel figures.')
        return [fig1, fig2]

    # ── Collect Gantt data ────────────────────────────────────────────
    pathInfo_agent_chain = const['pathInfo_agent_chain']

    int_data  = [_collect_agent_data(ag, x_prev, y_prev, pathInfo_agent_chain, N, kn,
                                      terminal_id_0idx=terminal_0)
                 for ag in range(n_int)]
    road_data = [_collect_agent_data(ag, x_prev, y_prev, pathInfo_agent_chain, N, kn,
                                      terminal_id_0idx=terminal_0)
                 for ag in range(n_int, n_agents)]

    # ── Figure 3: Intersection agents ────────────────────────────────
    fig3 = _gantt_figure(
        int_data,
        [f'Merging Zone {i+1}' for i in range(n_int)],
        fig_size=(15, 7.2),
        suptitle='',
        xlabel_bottom='Time(seconds)',
        colors=colors,
        gap_frac=0.04)

    # ── Figure 4: Road agents + Terminal ─────────────────────────────
    fig4 = _gantt_figure(
        road_data,
        [f'Road {i+1}' for i in range(n_road)] + ['Terminal'],
        fig_size=(15, 7.6),
        suptitle='',
        xlabel_bottom='Time(seconds)',
        colors=colors,
        gap_frac=0.022)

    # ── Figure 5: Local panels per intersection ──────────────────────
    import math
    rows = max(1, int(math.ceil(math.sqrt(n_int))))
    cols = max(1, int(math.ceil(n_int / rows)))
    fig5 = plt.figure(figsize=(7.5 * cols, 5.0 * rows), facecolor='white')
    fig5.suptitle('Optimal Local Schedules (Per Intersection)',
                  fontsize=FS_TIT, y=0.985)
    outer_gs = GridSpec(rows, cols, figure=fig5,
                        wspace=0.18, hspace=0.22,
                        left=0.06, right=0.97, top=0.93, bottom=0.06)
    for ag in range(n_int):
        cell  = outer_gs[ag // cols, ag % cols]
        cache = local_tree_cache[ag] if ag < len(local_tree_cache) else None
        if not cache:
            ax = fig5.add_subplot(cell)
            ax.text(0.5, 0.5, f'Intersection {ag + 1}: no cache',
                    ha='center', va='center', fontsize=12)
            ax.set_xticks([]); ax.set_yticks([])
            continue
        nodes_list = cache.get('NODES')
        path_list  = cache.get('Path')
        valid_systems = cache.get('valid_systems', list(range(N)))
        if not nodes_list or not path_list:
            ax = fig5.add_subplot(cell)
            ax.text(0.5, 0.5, f'Intersection {ag + 1}: empty path',
                    ha='center', va='center', fontsize=12)
            ax.set_xticks([]); ax.set_yticks([])
            continue
        _local_panel(fig5, cell, nodes_list, path_list, ag, valid_systems, const,
                     x_prev=x_prev, Dt=const['Dt'],
                     title=f'Intersection {ag + 1}')

    return [fig1, fig2, fig3, fig4, fig5]


if __name__ == '__main__':
    main()
