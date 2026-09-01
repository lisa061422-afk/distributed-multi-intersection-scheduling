"""
Interactive topology picker.

Open a matplotlib window with an integer grid. Click to place intersections
and ports; close to commit. Returns a ``Topology`` ready to feed into
``generate_random_config`` / ``run_admm_core``.

Controls
--------
  Left-click on a grid cell    : toggle intersection at that cell
  Right-click near an int side : toggle port on that side (N/E/S/W)
  Press 'r'                    : reset (clear everything)
  Press 'q' (or close window)  : commit and return Topology

Edges between orthogonal-neighbor intersections are derived automatically.
A port that becomes blocked by a newly added neighbor is auto-removed.

Usage from a script:

    from python.pick_topology import pick_topology_interactive
    t = pick_topology_interactive(grid_size=6)

Or via run_arbitrary_demo.py:

    python python/main.py
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Tuple, Dict, List

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from python.topology import manual_topology, _DIR_DELTA, Topology


_OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}


def pick_topology_interactive(grid_size: int = 6,
                                name: str = 'picked',
                                figsize: Tuple[float, float] = (8, 8)
                                ) -> Tuple[Topology, Dict[int, Tuple[int, int]],
                                            List[Tuple[int, str]]]:
    """Open a matplotlib picker. Returns (Topology, coords, ports).

    Parameters
    ----------
    grid_size : int    grid is grid_size × grid_size cells (default 6)
    name      : str    Topology.name on the returned object
    figsize   : tuple  matplotlib figure size

    Returns
    -------
    topology : Topology
    coords   : dict {int_id_1idx: (col, row)}
    ports    : list of (int_id_1idx, direction)

    Raises
    ------
    RuntimeError if no intersections / no ports were placed, or if the picked
    graph is disconnected (the latter via manual_topology).
    """
    import matplotlib.pyplot as plt

    coords: Dict[int, Tuple[int, int]] = {}
    ports: List[Tuple[int, str]] = []
    excluded_edges: set = set()        # set of frozenset({i, j})
    next_int_id = [1]

    fig, ax = plt.subplots(figsize=figsize)
    fig.canvas.manager.set_window_title('Pick topology — left-click=int, right-click=port, q=commit')

    def status_text():
        n_int = len(coords)
        n_port = len(ports)
        coord2id = {tuple(c): i for i, c in coords.items()}
        edge_count = 0
        skipped = 0
        for i, (cx, cy) in coords.items():
            for d, (dx, dy) in _DIR_DELTA.items():
                nb = coord2id.get((cx + dx, cy + dy))
                if nb is None or nb <= i:
                    continue
                if frozenset({i, nb}) in excluded_edges:
                    skipped += 1
                else:
                    edge_count += 1
        sk = f'  skipped={skipped}' if skipped else ''
        return f'n_int={n_int}  n_road={edge_count}  n_ports={n_port}{sk}'

    def redraw():
        ax.clear()
        # Background grid
        ax.set_xlim(-0.6, grid_size - 0.4)
        ax.set_ylim(-0.6, grid_size - 0.4)
        ax.set_xticks(range(grid_size))
        ax.set_yticks(range(grid_size))
        ax.grid(True, alpha=0.3, zorder=0)
        ax.set_aspect('equal')

        title = (
            'Quickest: LEFT-CLICK a few intersections, then press Q.\n'
            'RIGHT-CLICK near int side = toggle port. '
            'MIDDLE-CLICK on edge midpoint = remove/restore that edge. '
            'R=reset.')
        ax.set_title(title + '\n' + status_text(), fontsize=9)

        # Edges (auto-derived from grid adjacency)
        coord2id = {tuple(c): i for i, c in coords.items()}
        for i, (cx, cy) in coords.items():
            for d, (dx, dy) in _DIR_DELTA.items():
                nb = coord2id.get((cx + dx, cy + dy))
                if nb is None or nb <= i:
                    continue
                nx, ny = coords[nb]
                if frozenset({i, nb}) in excluded_edges:
                    # Skipped: dashed grey + small red X at midpoint
                    ax.plot([cx, nx], [cy, ny], color='lightgray',
                            lw=1.5, ls='--', zorder=1, alpha=0.6)
                    ax.scatter([(cx + nx) / 2], [(cy + ny) / 2],
                               s=80, marker='x', c='crimson',
                               linewidth=2, zorder=2)
                else:
                    ax.plot([cx, nx], [cy, ny], 'k-', lw=2, zorder=1)

        # Intersections
        for i, (cx, cy) in coords.items():
            ax.scatter(cx, cy, s=620, c='steelblue', edgecolors='navy',
                       linewidth=1.5, zorder=3)
            ax.text(cx, cy, f'I{i}', color='white', ha='center', va='center',
                    fontsize=11, fontweight='bold', zorder=4)

        # Ports
        for k, (int_id, d) in enumerate(ports, start=1):
            cx, cy = coords[int_id]
            dx, dy = _DIR_DELTA[d]
            px, py = cx + 0.32 * dx, cy + 0.32 * dy
            ax.scatter(px, py, s=140, c='orange', marker='s',
                       edgecolors='darkorange', linewidth=1, zorder=3)
            ax.text(px, py, f'P{k}', fontsize=7, ha='center', va='center', zorder=4)

        # Hint markers: ghost squares on free directions of placed ints, to show
        # where right-click can add a port
        for i, (cx, cy) in coords.items():
            for d, (dx, dy) in _DIR_DELTA.items():
                # skip if neighbor exists
                if (cx + dx, cy + dy) in coord2id:
                    continue
                if (i, d) in ports:
                    continue
                px, py = cx + 0.32 * dx, cy + 0.32 * dy
                ax.scatter(px, py, s=40, c='lightgray', marker='s',
                           edgecolors='gray', linewidth=0.5, alpha=0.6, zorder=2)

        fig.canvas.draw_idle()

    def on_click(event):
        if event.xdata is None or event.ydata is None or event.inaxes is not ax:
            return
        x, y = event.xdata, event.ydata

        if event.button == 1:                        # ── LEFT: toggle int ──
            cx, cy = round(x), round(y)
            if not (0 <= cx < grid_size and 0 <= cy < grid_size):
                return
            coord2id = {tuple(c): i for i, c in coords.items()}
            existing = coord2id.get((cx, cy))
            if existing is not None:
                # Delete intersection + any ports on it
                del coords[existing]
                ports[:] = [p for p in ports if p[0] != existing]
            else:
                new_id = next_int_id[0]
                coords[new_id] = (cx, cy)
                next_int_id[0] += 1
                # Remove neighbor ports that now collide with this new edge
                for d, (dx, dy) in _DIR_DELTA.items():
                    nb = coord2id.get((cx + dx, cy + dy))
                    if nb is not None:
                        opp = _OPP[d]
                        ports[:] = [p for p in ports if p != (nb, opp)]

        elif event.button == 2:                      # ── MIDDLE: toggle edge ──
            # Find edge whose midpoint is closest to the click (within 0.35).
            coord2id = {tuple(c): i for i, c in coords.items()}
            best_edge = None
            best_d2 = 0.35 ** 2
            for i, (cx, cy) in coords.items():
                for d, (dx, dy) in _DIR_DELTA.items():
                    nb = coord2id.get((cx + dx, cy + dy))
                    if nb is None or nb <= i:
                        continue
                    nx, ny = coords[nb]
                    mx, my = (cx + nx) / 2, (cy + ny) / 2
                    d2 = (x - mx) ** 2 + (y - my) ** 2
                    if d2 < best_d2:
                        best_d2 = d2
                        best_edge = frozenset({i, nb})
            if best_edge is None:
                return
            if best_edge in excluded_edges:
                excluded_edges.discard(best_edge)
            else:
                excluded_edges.add(best_edge)

        elif event.button == 3:                      # ── RIGHT: toggle port ──
            # Find the placed int closest to the click (within radius 0.6)
            best = None
            best_d2 = float('inf')
            for i, (icx, icy) in coords.items():
                d2 = (x - icx) ** 2 + (y - icy) ** 2
                if d2 < best_d2 and d2 < 0.6 ** 2:
                    best_d2 = d2
                    best = i
            if best is None:
                return
            icx, icy = coords[best]
            dx, dy = x - icx, y - icy
            if abs(dx) > abs(dy):
                direction = 'E' if dx > 0 else 'W'
            else:
                direction = 'N' if dy > 0 else 'S'
            # If that direction is already occupied by an edge, ignore
            coord2id = {tuple(c): i for i, c in coords.items()}
            ddx, ddy = _DIR_DELTA[direction]
            if (icx + ddx, icy + ddy) in coord2id:
                return
            key = (best, direction)
            if key in ports:
                ports.remove(key)
            else:
                ports.append(key)

        redraw()

    def on_key(event):
        if event.key in ('q', 'enter'):
            plt.close(fig)
        elif event.key == 'r':
            coords.clear()
            ports.clear()
            excluded_edges.clear()
            next_int_id[0] = 1
            redraw()

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)
    redraw()
    plt.show()

    # ── After window closes ─────────────────────────────────────────
    if not coords:
        raise RuntimeError(
            'No intersections placed. Left-click on grid cells to add '
            'intersections, then press q to commit.')

    # Auto-fill: if the user didn't right-click any ports, fill EVERY free
    # directional slot with a port. This is the most common case — the user
    # just clicks intersections and presses q.
    if not ports:
        coord2id = {tuple(c): i for i, c in coords.items()}
        for i, (cx, cy) in coords.items():
            for d, (dx, dy) in _DIR_DELTA.items():
                if (cx + dx, cy + dy) not in coord2id:
                    ports.append((i, d))
        print(f'[auto-fill] no ports were picked manually — '
              f'placed {len(ports)} ports on all free intersection sides')

    if len(ports) < 2:
        raise RuntimeError(
            f'Need at least 2 ports for any OD pair; only {len(ports)} placed. '
            f'(Right-click near an intersection side to add a port, '
            f'or place more intersections so that some sides are free.)')

    # Renumber intersections to be contiguous 1..n_int
    # (user may have deleted some)
    sorted_ids = sorted(coords.keys())
    relabel = {old: new for new, old in enumerate(sorted_ids, start=1)}
    coords_clean = {relabel[o]: c for o, c in coords.items()}
    ports_clean = [(relabel[i], d) for (i, d) in ports]
    # Relabel excluded_edges; drop any edge whose endpoints got deleted
    excluded_clean = []
    for e in excluded_edges:
        i, j = tuple(e)
        if i in relabel and j in relabel:
            excluded_clean.append(frozenset({relabel[i], relabel[j]}))

    # Print spec so user can save it for next time
    print('\n' + '=' * 50)
    print(f'Picked: n_int={len(coords_clean)}, n_ports={len(ports_clean)}, '
          f'excluded_edges={len(excluded_clean)}')
    print('Spec (paste into a .py file to reuse):')
    print('=' * 50)
    print(f'name = {name!r}')
    print(f'coords = {coords_clean!r}')
    print(f'ports = {ports_clean!r}')
    if excluded_clean:
        # Print as list-of-sets for readability
        ee = [set(e) for e in excluded_clean]
        print(f'excluded_edges = {ee!r}')
    print('=' * 50 + '\n')

    topo = manual_topology(coords=coords_clean, ports=ports_clean,
                            excluded_edges=excluded_clean or None,
                            name=name)
    return topo, coords_clean, ports_clean


# Self-test entry point — run this file directly to play with the picker.
if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--grid', type=int, default=6)
    args = p.parse_args()
    t, coords, ports = pick_topology_interactive(grid_size=args.grid)
    print(f'Topology built: {t.name}  '
          f'(n_int={t.n_int}, n_road={t.n_road}, n_ports={t.n_ports}, '
          f'#OD={len(t.route_dict)})')
