"""
Pure-Python equivalent of export_config_for_python.m (manual mode).
Generates validation_config.json from the hardcoded manual_10r S1 config —
no MATLAB needed.

Usage (from the repository root or python/):
    python python/build_manual_config.py
"""

import json
import re
import sys
from pathlib import Path

# Make the package importable when this file is run directly as a script
# (e.g. `python python/build_manual_config.py` from the repo root).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from python import topology as _topo


# ── manual_10r S1 (mirrors MAIN_Parallel_Compute.m configMode='manual') ──
CONFIG_RAW = [
    {'entrance': 1, 'exits': [7, 3]},
    {'entrance': 4, 'exits': [1, 3]},
    {'entrance': 6, 'exits': [1]},
    {'entrance': 7, 'exits': [4]},
    {'entrance': 5, 'exits': [3]},
    {'entrance': 3, 'exits': [2, 4]},
    {'entrance': 2, 'exits': [5]},
]

# ── Default values (only used if missing from JSON; mirrors MAIN_Parallel_Compute.m) ──
DEFAULTS = {
    # ── Physical (used to derive alpha_tilde / deadline) ──
    'physical_params': {
        'Dt':           2.0,      # road travel time between intersections (s)
        'v_max_phys':   1.5,      # AMR speed on road (m/s)
        'detect_range': 7.6,      # detection range (m)
        'T_val':        2.0,      # headway @ same entrance (s)
        'T_ent':        0.0,      # stagger @ different entrances (s)
        'W':            1.6,      # merging zone width (m)
    },
    # ── ADMM hyperparams ──
    'rho1':          1.0,
    'rho2':          1.0,
    'weight':        1.5,
    'max_iter':      50,
    'tol_r':         1e-2,
    'tol_s':         1e-2,
    'randInitScale': 0,
    'seed':          0,
}


# Route dictionary and road-agent lookup come from topology.py.
# A thin shim adapts the topology's ('ints','route_id') keys to this module's
# legacy ('int','routeId') keys without copying the data.
def _ROUTE_DICT_VIEW(topology=None):
    """Return a {(ent,ext): {'int':[..], 'routeId':[..]}} view over a topology."""
    t = topology if topology is not None else _topo.DEFAULT
    return {od: {'int': r['ints'], 'routeId': r['route_id']}
            for od, r in t.route_dict.items()}


def get_road_agent(i, j, topology=None):
    """1-indexed int IDs → 1-indexed road agent ID."""
    t = topology if topology is not None else _topo.DEFAULT
    return t.get_road_agent(i, j)


def _compact_json(obj):
    """Dump JSON with primitive arrays + simple dicts collapsed to one line."""
    s = json.dumps(obj, indent=2, ensure_ascii=False)

    # Collapse arrays whose contents are only primitives.
    arr_pattern = re.compile(
        r'\[\s*((?:-?\d[\d.eE+\-]*|true|false|null)'
        r'(?:\s*,\s*(?:-?\d[\d.eE+\-]*|true|false|null))*)\s*\]'
    )
    # Collapse dicts whose values are only primitives or short strings.
    dict_pattern = re.compile(
        r'\{\s*("[^"\n]+":\s*(?:-?\d[\d.eE+\-]*|true|false|null|"[^"\n]*")'
        r'(?:\s*,\s*"[^"\n]+":\s*(?:-?\d[\d.eE+\-]*|true|false|null|"[^"\n]*"))*)\s*\}'
    )
    prev = None
    while prev != s:
        prev = s
        s = arr_pattern.sub(
            lambda m: '[' + ', '.join(t.strip() for t in m.group(1).split(',')) + ']',
            s,
        )
        s = dict_pattern.sub(
            lambda m: '{' + ', '.join(p.strip() for p in m.group(1).split(',')) + '}',
            s,
        )
    return s


def _expand_config_raw(config_raw):
    """Group → per-vehicle list with entryIndex."""
    out = []
    for grp in config_raw:
        for j, ext in enumerate(grp['exits']):
            out.append({'entrance':   grp['entrance'],
                        'exit':       ext,
                        'entryIndex': j + 1})
    return out


def derive_fields(vehicles, IntSpaceDB, physical_params, topology=None):
    """
    Pure function: vehicles + IntSpaceDB + physical_params → derived fields.
    Used by both build_manual_config.py (write JSON) and load_config (auto regen on load).

    Parameters
    ----------
    vehicles, IntSpaceDB, physical_params : as before
    topology : Topology, optional. Defaults to topology.DEFAULT (= four_int).
               Supplies route_dict and the road-agent lookup. IntSpaceDB stays
               separate because the JSON file is its source of truth.

    Returns dict with keys:
      alpha_tilde, deadline, initial_position,
      agent_chains, path_c, route_ids, agent_participation
    """
    t = topology if topology is not None else _topo.DEFAULT
    route_dict   = t.route_dict
    terminal_id  = t.terminal_id_1idx
    n_nonterm    = t.n_int + t.n_road   # number of non-terminal agents

    Dt           = physical_params['Dt']
    v_max        = physical_params['v_max_phys']
    detect_range = physical_params['detect_range']
    T_val        = physical_params['T_val']
    T_ent        = physical_params['T_ent']
    W_phys       = physical_params['W']

    N = len(vehicles)

    # ── route_ids, agent_chain, path_c per vehicle ──
    route_ids    = [None] * N
    agent_chains = [None] * N
    path_c       = [None] * N
    for n, v in enumerate(vehicles):
        key = (v['entrance'], v['exit'])
        if key not in route_dict:
            raise KeyError(f'No route defined for ent={key[0]} exit={key[1]}')
        rt      = route_dict[key]
        int_seq = rt['ints']       # 1-indexed
        rids    = rt['route_id']   # 1-indexed

        dur = []
        for k_int, ag in enumerate(int_seq):
            rd_entry = IntSpaceDB[ag - 1]['routeDur'][rids[k_int] - 1]
            if not isinstance(rd_entry, list):
                rd_entry = [rd_entry]
            dur.append(sum(rd_entry))

        chain = []
        for i in range(len(int_seq) - 1):
            road = t.get_road_agent(int_seq[i], int_seq[i + 1])
            chain.extend([int_seq[i], road])
        chain.extend([int_seq[-1], terminal_id])

        route_ids[n]    = rids
        agent_chains[n] = chain
        path_c[n]       = dur

    # ── alpha_tilde, deadline, initial_position ──
    base    = (detect_range / 2 - W_phys / 2) / v_max
    headway = T_val

    # Entrance rank = order of first appearance in vehicle list
    ent_rank = {}
    for v in vehicles:
        ent_rank.setdefault(v['entrance'], len(ent_rank))

    alpha_tilde, deadline, initial_position = [], [], []
    for n, v in enumerate(vehicles):
        a = base + ent_rank[v['entrance']] * T_ent + (v['entryIndex'] - 1) * headway
        alpha_tilde.append(a)
        num_roads = (len(agent_chains[n]) - 1) // 2
        deadline.append(a + sum(path_c[n]) + Dt * num_roads)
        initial_position.append(-(detect_range / 2 - W_phys / 2 + a * v_max))

    # ── agent_participation: (n_int + n_road) × N ──
    ap = [[0] * N for _ in range(n_nonterm)]
    for n in range(N):
        for ag in agent_chains[n][:-1]:
            ap[ag - 1][n] = 1

    return {
        'alpha_tilde':         alpha_tilde,
        'deadline':            deadline,
        'initial_position':    initial_position,
        'agent_chains':        agent_chains,
        'path_c':              path_c,
        'route_ids':           route_ids,
        'agent_participation': ap,
    }


def main():
    """Generate / regenerate validation_config.json from scratch."""
    here = Path(__file__).parent
    existing_json = here / 'validation_config.json'

    # IntSpaceDB is fixed; reuse from existing JSON if present
    if existing_json.exists():
        with open(existing_json, encoding='utf-8') as f:
            old = json.load(f)
        IntSpaceDB = old['IntSpaceDB']
    else:
        raise FileNotFoundError(f'Need an existing {existing_json} to read IntSpaceDB.')

    # Source of truth: 'vehicles' field in JSON if present, else CONFIG_RAW
    if 'vehicles' in old and old['vehicles']:
        vehicles = [{'entrance':   int(v['entrance']),
                     'exit':       int(v['exit']),
                     'entryIndex': int(v['entryIndex'])} for v in old['vehicles']]
        print(f"Using 'vehicles' from JSON ({len(vehicles)} vehicles)")
    else:
        vehicles = _expand_config_raw(CONFIG_RAW)
        print(f'Using CONFIG_RAW from script ({len(vehicles)} vehicles, manual_10r S1)')

    # Source of truth for params: JSON if present, else DEFAULTS
    pp = {**DEFAULTS['physical_params'], **(old.get('physical_params') or {})}

    derived = derive_fields(vehicles, IntSpaceDB, pp)

    out = {
        'N': len(vehicles),
        'Dt': pp['Dt'],
        'rho1':          old.get('rho1',          DEFAULTS['rho1']),
        'rho2':          old.get('rho2',          DEFAULTS['rho2']),
        'weight':        old.get('weight',        DEFAULTS['weight']),
        'max_iter':      old.get('max_iter',      DEFAULTS['max_iter']),
        'tol_r':         old.get('tol_r',         DEFAULTS['tol_r']),
        'tol_s':         old.get('tol_s',         DEFAULTS['tol_s']),
        'randInitScale': old.get('randInitScale', DEFAULTS['randInitScale']),
        'seed':          old.get('seed',          DEFAULTS['seed']),
        'physical_params': pp,
        'vehicles': [{'n':          n + 1,
                      'entrance':   v['entrance'],
                      'exit':       v['exit'],
                      'entryIndex': v['entryIndex']}
                     for n, v in enumerate(vehicles)],
        **derived,
        'IntSpaceDB': IntSpaceDB,
    }

    with open(existing_json, 'w', encoding='utf-8') as f:
        f.write(_compact_json(out))

    print(f'Wrote {existing_json}')
    print(f'  N = {len(vehicles)}   Dt = {pp["Dt"]}')
    print(f'  alpha_tilde = {[round(x, 3) for x in derived["alpha_tilde"]]}')
    print(f'  deadline    = {[round(x, 3) for x in derived["deadline"]]}')


if __name__ == '__main__':
    main()
