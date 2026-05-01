"""
Topology spec — single source of truth for the road network.

A Topology object captures everything that depends on the *physical layout*
of the warehouse / traffic network:
  - how many intersection nodes there are, and how many external ports
  - which intersections are connected by a road agent (and that road's ID)
  - the (entrance, exit) → (intersection sequence, per-intersection route label)
    dictionary used by both random and manual config builders
  - the per-intersection IntSpaceDB (route → list of conflict spaces, durations)

Agent ID convention (1-indexed throughout this module, matching the MATLAB port):
  1 .. n_int                       intersection agents
  n_int+1 .. n_int+n_road          road agents
  n_int+n_road+1                   terminal (single)

Step 1 only ships ``four_int()`` — the existing 4-intersection topology used
by validation_config.json. Step 2 will add ``five_int()`` and arbitrary 4-way
manual topologies built from an adjacency list.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, FrozenSet, Tuple, List, Dict


# ---------------------------------------------------------------------------
# Physical constants for the standard 4-way merging zone
# (mirror generate_config.py — kept here as the single source of truth)
# ---------------------------------------------------------------------------
_W      = 1.6    # merging zone width (m)
_V_INT  = 0.8    # AMR speed inside zone (m/s)
_C1 = math.pi * _W / (8 * _V_INT)   # ≈ 0.7854 s
_C2 = _W / (2 * _V_INT)              # = 1.0 s
_C3 = _C1


# ---------------------------------------------------------------------------
# Topology dataclass
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Topology:
    name: str
    n_int: int
    n_road: int
    n_ports: int
    # routeDict: (entrance_port, exit_port) → {'ints': [int_ids], 'route_id': [route_labels]}
    # All IDs and labels are 1-indexed (port IDs, intersection IDs, route labels).
    route_dict: Mapping[Tuple[int, int], Dict[str, List[int]]]
    # road_agent: undirected edge between two intersections → road agent ID (1-indexed).
    # Keyed by frozenset({i, j}) so lookup is symmetric.
    road_agent: Mapping[FrozenSet[int], int]
    # int_db: length = n_int. Each entry is one intersection's IntSpaceDB:
    #   {'numSpaces': int, 'numRoutes': int,
    #    'routeSpace': {label_1idx: [space_ids]},
    #    'routeDur':   {label_1idx: [durations]}}
    int_db: List[dict]
    # Optional grid embedding used when this Topology was built from coords.
    # ``coords[i_1idx] = (col, row)``. None for topologies not on a grid.
    coords: Mapping[int, Tuple[int, int]] = None
    # Optional port placement table. ``port_locs[port_id_1idx] = (int_id, dir)``
    # where dir is 'N'|'E'|'S'|'W'. None if not tracked.
    port_locs: Mapping[int, Tuple[int, str]] = None
    # Pairs of grid-adjacent intersections that should NOT be connected by a
    # road, even though they're orthogonal grid neighbors. Each entry is a
    # frozenset({i, j}) of 1-indexed intersection IDs. None ↔ empty set.
    excluded_edges: FrozenSet[FrozenSet[int]] = None

    # -- derived ---------------------------------------------------------
    @property
    def n_agents(self) -> int:
        """Total agents = intersections + roads + 1 terminal."""
        return self.n_int + self.n_road + 1

    @property
    def terminal_id_1idx(self) -> int:
        """1-indexed agent ID of the terminal."""
        return self.n_int + self.n_road + 1

    @property
    def terminal_id_0idx(self) -> int:
        """0-indexed agent ID of the terminal."""
        return self.n_int + self.n_road

    # -- accessors -------------------------------------------------------
    def get_road_agent(self, i: int, j: int) -> int:
        """Return 1-indexed road agent ID between 1-indexed intersections i and j."""
        key = frozenset({i, j})
        if key not in self.road_agent:
            raise ValueError(f'No road between intersection {i} and {j}')
        return self.road_agent[key]

    def candidate_od_pairs(self) -> List[Tuple[int, int]]:
        """Sorted list of (entrance, exit) pairs available for random sampling."""
        return sorted(self.route_dict.keys())


# ---------------------------------------------------------------------------
# 4-way intersection IntSpaceDB template
# (unchanged from generate_config.py:_make_one_int_db — 12 routes, 5 spaces)
# ---------------------------------------------------------------------------
def _four_way_int_db() -> dict:
    """Build the IntSpaceDB for one regular 4-way intersection.

    12 routes (4 entries × 3 turn directions), 5 conflict spaces.
    Returns a dict with 1-indexed routeSpace/routeDur keys.
    """
    import numpy as np

    Map = np.array([
        [1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4],
        [5, 2, 0, 5, 3, 0, 5, 4, 0, 5, 1, 0],
        [3, 0, 0, 4, 0, 0, 1, 0, 0, 2, 0, 0],
    ], dtype=float)
    C = np.array([
        [_C1, _C2, _C3, _C1, _C2, _C3, _C1, _C2, _C3, _C1, _C2, _C3],
        [_C1, _C2,   0, _C1, _C2,   0, _C1, _C2,   0, _C1, _C2,   0],
        [_C1,   0,   0, _C1,   0,   0, _C1,   0,   0, _C1,   0,   0],
    ])
    num_routes = 12
    route_space: Dict[int, List[int]] = {}
    route_dur:   Dict[int, List[float]] = {}
    for r_idx in range(num_routes):
        r_label = r_idx + 1
        col_map = Map[:, r_idx]
        col_c   = C[:, r_idx]
        mask = col_map != 0
        route_space[r_label] = [int(x) for x in col_map[mask]]
        route_dur[r_label]   = [float(x) for x in col_c[mask]]
    return {
        'numSpaces': 5,
        'numRoutes': num_routes,
        'routeSpace': route_space,
        'routeDur':   route_dur,
    }


# ---------------------------------------------------------------------------
# four_int factory — the original 4-intersection topology
# (mirrors generate_config.py:_ROUTE_DICT, _ROAD_AGENT, _INT_DB)
# ---------------------------------------------------------------------------
_FOUR_INT_ROUTE_DICT: Dict[Tuple[int, int], Dict[str, List[int]]] = {
    # entrance 1
    (1, 2): {'ints': [1, 2],     'route_id': [3, 12]},
    (1, 3): {'ints': [1, 2],     'route_id': [3, 11]},
    (1, 4): {'ints': [1, 2, 3],  'route_id': [3, 10, 3]},
    (1, 5): {'ints': [1, 2, 3],  'route_id': [3, 10, 2]},
    (1, 6): {'ints': [1, 4],     'route_id': [2, 2]},
    (1, 7): {'ints': [1, 4],     'route_id': [2, 1]},
    (1, 8): {'ints': [1],        'route_id': [1]},
    # entrance 2
    (2, 1): {'ints': [2, 1],     'route_id': [1, 4]},
    (2, 3): {'ints': [2],        'route_id': [3]},
    (2, 4): {'ints': [2, 3],     'route_id': [2, 3]},
    (2, 5): {'ints': [2, 3],     'route_id': [2, 2]},
    (2, 6): {'ints': [2, 3, 4],  'route_id': [2, 1, 6]},
    (2, 7): {'ints': [2, 3, 4],  'route_id': [2, 1, 5]},
    (2, 8): {'ints': [2, 1],     'route_id': [1, 5]},
    # entrance 3
    (3, 1): {'ints': [2, 1],     'route_id': [5, 4]},
    (3, 2): {'ints': [2],        'route_id': [4]},
    (3, 4): {'ints': [2, 3],     'route_id': [6, 3]},
    (3, 5): {'ints': [2, 3],     'route_id': [6, 2]},
    (3, 6): {'ints': [2, 3, 4],  'route_id': [6, 1, 6]},
    (3, 7): {'ints': [2, 3, 4],  'route_id': [6, 1, 5]},
    (3, 8): {'ints': [2, 1],     'route_id': [5, 5]},
    # entrance 4
    (4, 1): {'ints': [3, 4, 1],  'route_id': [5, 4, 8]},
    (4, 2): {'ints': [3, 2],     'route_id': [4, 8]},
    (4, 3): {'ints': [3, 2],     'route_id': [4, 7]},
    (4, 5): {'ints': [3],        'route_id': [6]},
    (4, 6): {'ints': [3, 4],     'route_id': [5, 6]},
    (4, 7): {'ints': [3, 4],     'route_id': [5, 5]},
    (4, 8): {'ints': [3, 4, 1],  'route_id': [5, 4, 9]},
    # entrance 5
    (5, 1): {'ints': [3, 4, 1],  'route_id': [9, 4, 8]},
    (5, 2): {'ints': [3, 2],     'route_id': [8, 8]},
    (5, 3): {'ints': [3, 2],     'route_id': [8, 7]},
    (5, 4): {'ints': [3],        'route_id': [7]},
    (5, 6): {'ints': [3, 4],     'route_id': [9, 6]},
    (5, 7): {'ints': [3, 4],     'route_id': [9, 5]},
    (5, 8): {'ints': [3, 4, 1],  'route_id': [9, 4, 9]},
    # entrance 6
    (6, 1): {'ints': [4, 1],     'route_id': [8, 8]},
    (6, 2): {'ints': [4, 1, 2],  'route_id': [8, 7, 12]},
    (6, 3): {'ints': [4, 1, 2],  'route_id': [8, 7, 11]},
    (6, 4): {'ints': [4, 3],     'route_id': [7, 11]},
    (6, 5): {'ints': [4, 3],     'route_id': [7, 10]},
    (6, 7): {'ints': [4],        'route_id': [9]},
    (6, 8): {'ints': [4, 1],     'route_id': [8, 9]},
    # entrance 7
    (7, 1): {'ints': [4, 1],     'route_id': [12, 8]},
    (7, 2): {'ints': [4, 1, 2],  'route_id': [12, 7, 12]},
    (7, 3): {'ints': [4, 1, 2],  'route_id': [12, 7, 11]},
    (7, 4): {'ints': [4, 3],     'route_id': [11, 11]},
    (7, 5): {'ints': [4, 3],     'route_id': [11, 10]},
    (7, 6): {'ints': [4],        'route_id': [10]},
    (7, 8): {'ints': [4, 1],     'route_id': [12, 9]},
    # entrance 8
    (8, 1): {'ints': [1],        'route_id': [12]},
    (8, 2): {'ints': [1, 2],     'route_id': [11, 12]},
    (8, 3): {'ints': [1, 2],     'route_id': [11, 11]},
    (8, 4): {'ints': [1, 2, 3],  'route_id': [11, 10, 3]},
    (8, 5): {'ints': [1, 2, 3],  'route_id': [11, 10, 2]},
    (8, 6): {'ints': [1, 4],     'route_id': [10, 2]},
    (8, 7): {'ints': [1, 4],     'route_id': [10, 1]},
}

# Roads in the 4-int topology: I1-I2 (5), I2-I3 (6), I3-I4 (7), I1-I4 (8).
# Total agents: 4 ints (1-4) + 4 roads (5-8) + terminal (9) = 9.
_FOUR_INT_ROADS: Dict[FrozenSet[int], int] = {
    frozenset({1, 4}): 8,
    frozenset({1, 2}): 5,
    frozenset({2, 3}): 6,
    frozenset({3, 4}): 7,
}


def four_int() -> Topology:
    """The original 4-intersection topology (validation_config.json's network).

    Layout:    I1 ── I2     ports 1=I1-N, 2=I1-W, 3=I2-N, 4=I2-E,
                │     │            5=I3-E, 6=I3-S, 7=I4-S, 8=I4-W
                I4 ── I3
    """
    int_db_one = _four_way_int_db()
    return Topology(
        name='four_int',
        n_int=4,
        n_road=4,
        n_ports=8,
        route_dict=_FOUR_INT_ROUTE_DICT,
        road_agent=_FOUR_INT_ROADS,
        int_db=[int_db_one] * 4,
        coords={1: (0, 1), 2: (1, 1), 3: (1, 0), 4: (0, 0)},
        port_locs={1: (1, 'N'), 2: (1, 'W'),
                    3: (2, 'N'), 4: (2, 'E'),
                    5: (3, 'E'), 6: (3, 'S'),
                    7: (4, 'S'), 8: (4, 'W')},
    )


# ---------------------------------------------------------------------------
# manual_topology — build a 4-way Topology from a grid spec
# ---------------------------------------------------------------------------
#
# Direction conventions (col grows East, row grows North):
#   E = (+1,  0)    W = (-1,  0)
#   N = ( 0, +1)    S = ( 0, -1)
#
# Each intersection has 4 directional slots (N, E, S, W). A slot may hold
# either an edge (to an orthogonal grid-neighbor intersection) or an external
# port, but not both. The standard 4-way IntSpaceDB is reused.
#
# Route-label encoding (matches generateTrafficSystem_5int.m comments):
#   1, 2, 3   = S-entry: left(W),  straight(N), right(E)
#   4, 5, 6   = E-entry: left(S),  straight(W), right(N)
#   7, 8, 9   = N-entry: left(E),  straight(S), right(W)
#  10,11,12   = W-entry: left(N),  straight(E), right(S)

_DIR_DELTA = {'E': (1, 0), 'W': (-1, 0), 'N': (0, 1), 'S': (0, -1)}

# (entry_dir, exit_dir) → 1-indexed routeId in the 12-route 4-way template.
_ROUTE_ID_TABLE: Dict[Tuple[str, str], int] = {
    ('S', 'W'): 1,  ('S', 'N'): 2,  ('S', 'E'): 3,
    ('E', 'S'): 4,  ('E', 'W'): 5,  ('E', 'N'): 6,
    ('N', 'E'): 7,  ('N', 'S'): 8,  ('N', 'W'): 9,
    ('W', 'N'): 10, ('W', 'E'): 11, ('W', 'S'): 12,
}


def manual_topology(coords: Dict[int, Tuple[int, int]],
                    ports: List[Tuple[int, str]],
                    name: str = 'manual',
                    excluded_edges=None) -> Topology:
    """Build a Topology from a grid spec.

    Parameters
    ----------
    coords : dict {int_id_1idx: (col, row)}
        Integer grid coordinates for each intersection. Keys must be
        contiguous 1..n_int with no duplicate coords.
    ports  : list of (int_id_1idx, direction)
        External ports. Each direction must be one of {'N','E','S','W'} and
        must be a free slot at that intersection (not already used by an edge
        to a neighbor or a previous port).
    name   : str
        Topology name (for debugging/labelling).
    excluded_edges : iterable of {i, j}, optional
        Pairs of intersections that should NOT be connected even though they
        are orthogonal grid neighbors. Each entry can be a tuple, set, or
        frozenset of two 1-indexed int IDs. The graph must remain connected
        after exclusions (else ValueError).

    Returns
    -------
    Topology
        With route_dict, road_agent map, and per-int IntSpaceDB filled in.
        All intersections use the standard 4-way IntSpaceDB.

    Notes
    -----
    Edges are derived automatically: any pair of intersections at orthogonal
    grid neighbors is treated as connected by a road agent (unless listed in
    ``excluded_edges``). Non-orthogonal placements yield no edge.

    The graph must be connected (BFS from int 1 must reach all intersections);
    otherwise ValueError is raised.
    """
    # Normalise excluded_edges to a frozenset of frozensets
    if excluded_edges is None:
        excluded_set: FrozenSet[FrozenSet[int]] = frozenset()
    else:
        excluded_set = frozenset(frozenset(e) for e in excluded_edges)
    # ── 1. Validate coords ───────────────────────────────────────────
    n_int = len(coords)
    if sorted(coords.keys()) != list(range(1, n_int + 1)):
        raise ValueError(
            f'coords keys must be contiguous 1..n_int; got {sorted(coords.keys())}')
    coord2int: Dict[Tuple[int, int], int] = {}
    for i, c in coords.items():
        if tuple(c) in coord2int:
            raise ValueError(f'Intersections {coord2int[tuple(c)]} and {i} share coord {c}')
        coord2int[tuple(c)] = i

    # ── 2. Derive grid-adjacency edges ───────────────────────────────
    # adjacency[i][dir] = ('int', neighbor_id) | ('port', port_id) | None
    adjacency: Dict[int, Dict[str, object]] = {
        i: {'N': None, 'S': None, 'E': None, 'W': None} for i in coords
    }
    edges_seen: Dict[FrozenSet[int], int] = {}
    edge_order: List[Tuple[int, int]] = []
    for i, (cx, cy) in coords.items():
        for d, (dx, dy) in _DIR_DELTA.items():
            nb = coord2int.get((cx + dx, cy + dy))
            if nb is None:
                continue
            # Honour user's excluded_edges — keep the slot empty (NOT marked as
            # 'int', NOT marked as 'port') so route_dict ignores this direction.
            if frozenset({i, nb}) in excluded_set:
                continue
            adjacency[i][d] = ('int', nb)
            key = frozenset({i, nb})
            if key not in edges_seen:
                edges_seen[key] = len(edge_order)
                edge_order.append((i, nb))
    n_road = len(edge_order)

    # Road agent IDs follow intersections: 1..n_int = ints, n_int+1..n_int+n_road = roads
    road_agent: Dict[FrozenSet[int], int] = {
        frozenset({i, j}): n_int + 1 + idx
        for idx, (i, j) in enumerate(edge_order)
    }

    # ── 3. Place ports ───────────────────────────────────────────────
    port_int: Dict[int, Tuple[int, str]] = {}   # port_id_1idx → (int_id, dir)
    for k, (int_id, dir_) in enumerate(ports, start=1):
        if int_id not in adjacency:
            raise ValueError(f'Port {k} references unknown intersection {int_id}')
        if dir_ not in _DIR_DELTA:
            raise ValueError(f'Port {k}: bad direction {dir_!r} (expected N/E/S/W)')
        if adjacency[int_id][dir_] is not None:
            raise ValueError(
                f'Port {k}: direction {dir_} of int {int_id} is already used '
                f'(slot = {adjacency[int_id][dir_]})')
        adjacency[int_id][dir_] = ('port', k)
        port_int[k] = (int_id, dir_)
    n_ports = len(port_int)

    # ── 4. Connectivity check (BFS from int 1) ───────────────────────
    seen = {1}
    frontier = [1]
    while frontier:
        cur = frontier.pop(0)
        for slot in adjacency[cur].values():
            if slot is not None and slot[0] == 'int':
                nb = slot[1]
                if nb not in seen:
                    seen.add(nb)
                    frontier.append(nb)
    if len(seen) != n_int:
        missing = sorted(set(coords) - seen)
        raise ValueError(
            f'Graph not connected: {missing} unreachable from intersection 1')

    # ── 5. Build route_dict via BFS shortest paths between ports ─────
    def _shortest_int_path(src: int, dst: int) -> List[int]:
        if src == dst:
            return [src]
        parent: Dict[int, int] = {src: src}
        q = [src]
        while q:
            cur = q.pop(0)
            if cur == dst:
                break
            for slot in adjacency[cur].values():
                if slot is None or slot[0] != 'int':
                    continue
                nb = slot[1]
                if nb not in parent:
                    parent[nb] = cur
                    q.append(nb)
        if dst not in parent:
            return []
        path = [dst]
        while path[-1] != src:
            path.append(parent[path[-1]])
        return list(reversed(path))

    def _dir_to_neighbor(at_int: int, neighbor: int) -> str:
        for d, slot in adjacency[at_int].items():
            if slot is not None and slot[0] == 'int' and slot[1] == neighbor:
                return d
        raise RuntimeError(f'No edge from int {at_int} to {neighbor}')

    route_dict: Dict[Tuple[int, int], Dict[str, List[int]]] = {}
    for p_in in range(1, n_ports + 1):
        int_a, entry_dir_at_a = port_int[p_in]
        for p_out in range(1, n_ports + 1):
            if p_in == p_out:
                continue
            int_z, exit_dir_at_z = port_int[p_out]
            ints_path = _shortest_int_path(int_a, int_z)
            if not ints_path:
                continue   # disconnected (shouldn't happen after connectivity check)

            rids: List[int] = []
            for k_int, cur in enumerate(ints_path):
                # Entry direction at cur
                if k_int == 0:
                    entry_dir = entry_dir_at_a
                else:
                    entry_dir = _dir_to_neighbor(cur, ints_path[k_int - 1])
                # Exit direction at cur
                if k_int == len(ints_path) - 1:
                    exit_dir = exit_dir_at_z
                else:
                    exit_dir = _dir_to_neighbor(cur, ints_path[k_int + 1])
                if (entry_dir, exit_dir) not in _ROUTE_ID_TABLE:
                    rids = []
                    break   # U-turn or invalid — skip this OD pair
                rids.append(_ROUTE_ID_TABLE[(entry_dir, exit_dir)])
            if rids:
                route_dict[(p_in, p_out)] = {'ints': list(ints_path), 'route_id': rids}

    # ── 6. IntSpaceDB: one 4-way template per intersection ───────────
    int_db = [_four_way_int_db() for _ in range(n_int)]

    return Topology(
        name=name,
        n_int=n_int,
        n_road=n_road,
        n_ports=n_ports,
        route_dict=route_dict,
        road_agent=road_agent,
        int_db=int_db,
        coords=dict(coords),
        port_locs=dict(port_int),
        excluded_edges=excluded_set if excluded_set else None,
    )


# ---------------------------------------------------------------------------
# random_topology — generate a random connected 4-way grid graph
# ---------------------------------------------------------------------------
def random_topology(n_int: int,
                    n_ports: int = None,
                    seed: int = None,
                    n_int_max: int = 20,
                    name: str = None) -> Topology:
    """Generate a random connected 4-way topology with ``n_int`` intersections.

    Algorithm
    ---------
    1. Place intersection 1 at (0, 0).
    2. For i = 2..n_int: pick a random already-placed intersection that still
       has a free orthogonal slot; place i in that direction. This guarantees
       a connected spanning tree on a grid.
    3. (Implicit) Any incidental orthogonal neighbors get auto-connected as
       extra edges via ``manual_topology``'s grid-adjacency logic, so the final
       graph may contain cycles — useful for testing networks denser than trees.
    4. Collect every free directional slot across all intersections, shuffle,
       and pick the first ``n_ports`` as external ports. By default,
       ``n_ports = min(2 * n_int, free_slot_count)``.

    Parameters
    ----------
    n_int     : 1..n_int_max — number of intersections to generate.
    n_ports   : optional desired port count. Clamped to the number of free
                slots actually available (a 1-int network has at most 4 ports).
    seed      : RNG seed for reproducibility.
    n_int_max : sanity upper bound (default 20). Larger graphs likely make ADMM
                slow; raise the bound if you really want to go bigger.

    Returns
    -------
    Topology — built via ``manual_topology`` so all downstream behaviour is
    identical to manual mode.
    """
    import random
    if n_int < 1:
        raise ValueError(f'n_int must be >= 1, got {n_int}')
    if n_int > n_int_max:
        raise ValueError(
            f'n_int={n_int} exceeds n_int_max={n_int_max}; '
            f'pass n_int_max=N to override')

    rng = random.Random(seed)

    coords: Dict[int, Tuple[int, int]] = {1: (0, 0)}
    used_positions = {(0, 0)}

    for i in range(2, n_int + 1):
        # Shuffle existing intersections; for each, try directions in random
        # order. Stop at the first free slot found.
        bases = list(coords.keys())
        rng.shuffle(bases)
        placed = False
        for base_id in bases:
            cx, cy = coords[base_id]
            dirs = ['N', 'E', 'S', 'W']
            rng.shuffle(dirs)
            for d in dirs:
                dx, dy = _DIR_DELTA[d]
                pos = (cx + dx, cy + dy)
                if pos not in used_positions:
                    coords[i] = pos
                    used_positions.add(pos)
                    placed = True
                    break
            if placed:
                break
        if not placed:
            # All existing nodes are surrounded — impossible for n_int <= ~25
            # on a planar grid, but keep a defensive error.
            raise RuntimeError(
                f'Could not place intersection {i}: all placed nodes are '
                f'surrounded. This should not happen for reasonable n_int.')

    # Build a temporary adjacency to find every free slot
    coord2int = {pos: i for i, pos in coords.items()}
    free_slots: List[Tuple[int, str]] = []
    for i, (cx, cy) in coords.items():
        for d, (dx, dy) in _DIR_DELTA.items():
            if (cx + dx, cy + dy) not in coord2int:
                free_slots.append((i, d))
    rng.shuffle(free_slots)

    # Default port count: 2 per int, capped by free slot availability.
    if n_ports is None:
        n_ports = min(2 * n_int, len(free_slots))
    n_ports = min(max(n_ports, 2), len(free_slots))   # need >= 2 for any OD pair

    ports = free_slots[:n_ports]

    if name is None:
        name = f'random_{n_int}int_{n_ports}port_seed{seed}'
    return manual_topology(coords=coords, ports=ports, name=name)


# ---------------------------------------------------------------------------
# plot_topology — quick visual sanity check of a Topology
# ---------------------------------------------------------------------------
def plot_topology(t: Topology, ax=None, title: str = None):
    """Draw a Topology: intersection nodes, road edges, and external ports.

    Requires matplotlib. Returns the matplotlib axis used (so the caller can
    further annotate or save).
    """
    import matplotlib.pyplot as plt

    if ax is None:
        fig, ax = plt.subplots(figsize=(7, 7))

    # ── Coords: prefer the ones stored on Topology (set by manual/random
    # builders); fall back to BFS-inferred grid layout, then circular. ──
    if t.coords is not None:
        coords = dict(t.coords)
    else:
        coords = _infer_grid_coords(t) or _circular_layout(t.n_int)

    # ── Edges (real) ─────────────────────────────────────────────────
    for edge_set, road_id in t.road_agent.items():
        i, j = sorted(edge_set)
        xi, yi = coords[i]
        xj, yj = coords[j]
        ax.plot([xi, xj], [yi, yj], 'k-', linewidth=2, zorder=1)
        ax.text((xi + xj) / 2, (yi + yj) / 2, f'R{road_id}',
                fontsize=8, color='gray',
                ha='center', va='center',
                bbox=dict(boxstyle='round,pad=0.15', fc='white', ec='none', alpha=0.8),
                zorder=2)

    # ── Excluded edges (would-be neighbors that user opted out of) ───
    if t.excluded_edges:
        for edge_set in t.excluded_edges:
            i, j = sorted(edge_set)
            if i not in coords or j not in coords:
                continue
            xi, yi = coords[i]
            xj, yj = coords[j]
            ax.plot([xi, xj], [yi, yj], color='lightgray', linewidth=2,
                    linestyle='--', zorder=1, alpha=0.7)
            ax.text((xi + xj) / 2, (yi + yj) / 2, '(skipped)',
                    fontsize=7, color='gray', style='italic',
                    ha='center', va='center', alpha=0.8, zorder=2)

    # ── Intersections ────────────────────────────────────────────────
    for i, (x, y) in coords.items():
        ax.scatter([x], [y], s=600, c='steelblue', edgecolors='navy',
                   linewidth=1.5, zorder=3)
        ax.text(x, y, f'I{i}', fontsize=11, color='white',
                ha='center', va='center', fontweight='bold', zorder=4)

    # ── Ports: prefer stored port_locs; otherwise draw on every free slot ──
    port_offset = 0.32
    if t.port_locs is not None:
        for port_id, (int_id, d) in t.port_locs.items():
            x, y = coords[int_id]
            dx, dy = _DIR_DELTA[d]
            px, py = x + dx * port_offset, y + dy * port_offset
            ax.scatter([px], [py], s=140, c='orange', marker='s',
                       edgecolors='darkorange', linewidth=1, zorder=3)
            ax.text(px, py, f'P{port_id}', fontsize=7, color='black',
                    ha='center', va='center', zorder=4)
    else:
        used_dirs = {i: set() for i in coords}
        for edge_set in t.road_agent:
            i_a, i_b = sorted(edge_set)
            ax_, ay_ = coords[i_a]
            bx_, by_ = coords[i_b]
            if   bx_ > ax_: used_dirs[i_a].add('E'); used_dirs[i_b].add('W')
            elif bx_ < ax_: used_dirs[i_a].add('W'); used_dirs[i_b].add('E')
            elif by_ > ay_: used_dirs[i_a].add('N'); used_dirs[i_b].add('S')
            elif by_ < ay_: used_dirs[i_a].add('S'); used_dirs[i_b].add('N')
        for i, (x, y) in coords.items():
            for d in ('N', 'E', 'S', 'W'):
                if d in used_dirs[i]:
                    continue
                dx, dy = _DIR_DELTA[d]
                px, py = x + dx * port_offset, y + dy * port_offset
                ax.scatter([px], [py], s=120, c='orange', marker='s',
                           edgecolors='darkorange', linewidth=1, zorder=3)

    # ── Cosmetics ───────────────────────────────────────────────────
    ax.set_aspect('equal')
    ax.set_title(title or f'Topology "{t.name}":  '
                 f'n_int={t.n_int}, n_road={t.n_road}, n_ports={t.n_ports}, '
                 f'#OD={len(t.route_dict)}',
                 fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.margins(0.18)
    ax.set_xticks([]); ax.set_yticks([])
    return ax


def _infer_grid_coords(t: Topology):
    """Try to recover (col, row) integer coords by BFS from int 1.

    Works for topologies built via ``manual_topology`` or ``random_topology``
    where edges are exactly orthogonal grid neighbors. Returns None if the
    edge structure can't be embedded on a grid (i.e., some int has more than
    4 neighbors, or a triangle exists).
    """
    coords: Dict[int, Tuple[int, int]] = {1: (0, 0)}
    # adjacency from road_agent
    adj: Dict[int, List[int]] = {i: [] for i in range(1, t.n_int + 1)}
    for edge_set in t.road_agent:
        i, j = sorted(edge_set)
        adj[i].append(j); adj[j].append(i)

    if any(len(v) > 4 for v in adj.values()):
        return None

    # BFS placing each new node at one of the 4 free neighbor slots
    used = {(0, 0): 1}
    q = [1]
    while q:
        cur = q.pop(0)
        cx, cy = coords[cur]
        free_dirs = [d for d in ('N', 'E', 'S', 'W')
                     if (cx + _DIR_DELTA[d][0],
                         cy + _DIR_DELTA[d][1]) not in used]
        # Try to place each unvisited neighbor at any free slot
        for nb in adj[cur]:
            if nb in coords:
                continue
            # If nb is already constrained by another already-placed neighbor
            # of nb being adjacent on the grid, that would force the slot.
            # Simple heuristic: take the first free direction.
            if not free_dirs:
                return None
            d = free_dirs.pop(0)
            dx, dy = _DIR_DELTA[d]
            pos = (cx + dx, cy + dy)
            coords[nb] = pos
            used[pos] = nb
            q.append(nb)

    if len(coords) != t.n_int:
        return None
    return coords


def _circular_layout(n_int: int):
    import math
    return {i + 1: (math.cos(2 * math.pi * i / n_int),
                    math.sin(2 * math.pi * i / n_int))
            for i in range(n_int)}


# ---------------------------------------------------------------------------
# Module-level default — convenience for callers that don't yet take a Topology
# ---------------------------------------------------------------------------
DEFAULT: Topology = four_int()
