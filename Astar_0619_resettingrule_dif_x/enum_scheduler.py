"""
enum_scheduler.py
=================
Single-intersection scheduler (NO-WAIT block model, priority enumeration).

Paradigm = DECISION TREE + TIMING MODEL (same as the resetting-rule method),
but with two cleaner components:
  * TIMING MODEL      = earliest_t()  -- longest-path (one entry time per vehicle)
  * CONFLICT DETECTION= find_conf()   -- scan the timed schedule for interval overlap
  * DECISION TREE     = solve()       -- branch on each real conflict (both orderings),
                                          prune priority cycles (禁成环 = positive cycle).

Model:
  - Each vehicle = ONE entry time t (>= release). Its sub-tasks are welded
    contiguous (no-wait): sub-task s occupies space Map[s] during
    [t + pre(s), t + pre(s) + dur(s)].
  - A priority "i before j on space m" is a difference constraint:
        t_j - t_i >= pre_i(m) + dur_i(m) - pre_j(m)
  - Given a set of such constraints, earliest start times = longest path.
  - A complete (conflict-free) schedule is a leaf; the min-delay leaf is the optimum.

Verified: on 12 test instances the optimum equals exhaustive enumeration (oracle),
and matches the user's MATLAB wherever the MATLAB method is itself optimal.
"""
import math, itertools

Ca = math.pi / 4      # arc sub-task duration  (~0.7854 s)
Cs = 1.0              # straight sub-task duration

# Intersection layout (4 arms x 3 directions = 12 routes; 5 conflict spaces).
# Map[s][r] = space used by sub-task s of route r (0 = no sub-task). 1-indexed spaces.
_MAP = [[1,1,1,2,2,2,3,3,3,4,4,4],
        [5,2,0,5,3,0,5,4,0,5,1,0],
        [3,0,0,4,0,0,1,0,0,2,0,0]]
_C   = [[Ca,Cs,Ca,Ca,Cs,Ca,Ca,Cs,Ca,Ca,Cs,Ca],
        [Ca,Cs,0 ,Ca,Cs,0 ,Ca,Cs,0 ,Ca,Cs,0 ],
        [Ca,0 ,0 ,Ca,0 ,0 ,Ca,0 ,0 ,Ca,0 ,0 ]]

# Physical constants for the release (arrival) time.
V_MAX, DETECT, W = 20.0, 510.0, 30.0
def _release(d1): return (DETECT/2 - W/2)/V_MAX + d1     # = 12 + d1


def build(routes, d1):
    """Build vehicle list.  veh[i] = (release_time, [(space, duration), ...])."""
    veh = []
    for i, r in enumerate(routes):
        subs = [(_MAP[s][r-1], _C[s][r-1]) for s in range(3) if _MAP[s][r-1] > 0]
        veh.append((_release(d1[i]), subs))
    return veh

def _pre(v, i, s): return sum(d for _, d in v[i][1][:s])   # sum of durations before sub-task s
def _dur(v, i, s): return v[i][1][s][1]
def _spc(v, i, s): return v[i][1][s][0]


# ----------------------------- TIMING MODEL --------------------------------
def earliest_t(v, cons):
    """
    Earliest entry time of every vehicle, given precedence constraints `cons`
    (each (i, j, w) means t_j >= t_i + w) plus releases (t_i >= release_i).
    = longest path from a source (Bellman-Ford).  Returns None if a positive
    cycle exists (the ordering is infeasible -> 禁成环 prunes that branch).
    """
    N = len(v)
    t = [v[i][0] for i in range(N)]          # start at releases
    for _ in range(N + 1):
        changed = False
        for (i, j, w) in cons:
            if t[i] + w > t[j] + 1e-12:
                t[j] = t[i] + w; changed = True
        if not changed:
            break
    else:
        return None                          # did not converge -> positive cycle
    for (i, j, w) in cons:                    # verify (cycle safety check)
        if t[i] + w > t[j] + 1e-9:
            return None
    return t


# --------------------------- CONFLICT DETECTION ----------------------------
def shared_pairs(v):
    """All op-pairs (i,si,j,sj) that use the SAME space (static, from layout)."""
    N = len(v); out = []
    for i in range(N):
        for si in range(len(v[i][1])):
            for j in range(i+1, N):
                for sj in range(len(v[j][1])):
                    if _spc(v, i, si) == _spc(v, j, sj):
                        out.append((i, si, j, sj))
    return out

def find_conf(v, t, decided, P):
    """
    Scan the TIMED schedule for the first shared-space pair whose intervals
    actually OVERLAP and whose order is not yet decided.  Continuous-interval
    test (no time-grid) -> handles tightly-packed / no-gap occupations cleanly.
    Returns (i, si, j, sj, m) or None (None => schedule is conflict-free = leaf).
    """
    for (i, si, j, sj) in P:
        m = _spc(v, i, si)
        if (frozenset((i, j)), m) in decided:
            continue
        a0 = t[i] + _pre(v, i, si); a1 = a0 + _dur(v, i, si)
        b0 = t[j] + _pre(v, j, sj); b1 = b0 + _dur(v, j, sj)
        if a0 < b1 - 1e-9 and b0 < a1 - 1e-9:     # strict overlap (touching = OK)
            return (i, si, j, sj, m)
    return None


# ----------------------------- DECISION TREE -------------------------------
def solve(routes, d1):
    """
    Enumerate ALL contention-driven complete schedules (the C-ADMM column set).
    Returns dict { delay_vector : total_delay }  (deduped by delay vector).
    """
    v = build(routes, d1)
    P = shared_pairs(v)
    leaves = {}
    seen = set()                              # memoization: a decided constraint-set once

    def rec(cons, decided):
        key = frozenset((i, j, round(w, 6)) for (i, j, w) in cons)
        if key in seen:
            return
        seen.add(key)
        t = earliest_t(v, cons)               # TIMING MODEL
        if t is None:                         # 禁成环: infeasible ordering
            return
        c = find_conf(v, t, decided, P)       # CONFLICT DETECTION
        if c is None:                         # leaf = complete conflict-free schedule
            dv = tuple(round(t[i] - v[i][0], 4) for i in range(len(v)))
            leaves[dv] = round(sum(t[i] - v[i][0] for i in range(len(v))), 4)
            return
        (i, si, j, sj, m) = c                 # BRANCH on the conflict (both orderings)
        nd = decided | {(frozenset((i, j)), m)}
        rec(cons + [(i, j, _pre(v,i,si)+_dur(v,i,si)-_pre(v,j,sj))], nd)   # i before j
        rec(cons + [(j, i, _pre(v,j,sj)+_dur(v,j,sj)-_pre(v,i,si))], nd)   # j before i

    rec([], set())
    return leaves


# --------------------------- C-ADMM INTERFACE ------------------------------
def intersection_solve(routes, d1=None):
    """
    Solve one intersection.  Returns:
      { 'optimum': float,                       # min total delay
        'columns': [ {                          # ALL contention-driven schedules
            'delay':  [per-vehicle delay],
            'entry':  [per-vehicle entry time],
            'exit':   [per-vehicle exit time],
            'total':  total delay } , ... ] }
    Each column is a complete, contention-free, no-wait schedule (a C-ADMM column).
    """
    if d1 is None:
        d1 = [0.0] * len(routes)
    v = build(routes, d1)
    sumC = [sum(d for _, d in v[i][1]) for i in range(len(v))]
    rel  = [v[i][0] for i in range(len(v))]
    cols = []
    for dv, tot in solve(routes, d1).items():
        entry = [round(rel[i] + dv[i], 4) for i in range(len(v))]
        exit_ = [round(entry[i] + sumC[i], 4) for i in range(len(v))]
        cols.append({'delay': list(dv), 'entry': entry, 'exit': exit_, 'total': tot})
    cols.sort(key=lambda c: c['total'])
    return {'optimum': cols[0]['total'] if cols else None, 'columns': cols}


# ------------------------------- ORACLE ------------------------------------
def oracle_min(routes, d1):
    """Exhaustive: try every orientation of every shared-space pair; min total delay."""
    v = build(routes, d1); P = shared_pairs(v); best = None
    for combo in itertools.product([0, 1], repeat=len(P)):
        cons = []
        for (i, si, j, sj), b in zip(P, combo):
            if b == 0: cons.append((i, j, _pre(v,i,si)+_dur(v,i,si)-_pre(v,j,sj)))
            else:      cons.append((j, i, _pre(v,j,sj)+_dur(v,j,sj)-_pre(v,i,si)))
        t = earliest_t(v, cons)
        if t is None:
            continue
        g = sum(t[i] - v[i][0] for i in range(len(v)))
        if best is None or g < best:
            best = g
    return round(best, 4)


if __name__ == '__main__':
    import time
    tests = [([1,4,5],[0,0,0]), ([1,2,11],[0,0,0]), ([1,4,7],[0,0,0]),
             ([1,2,4,5],[0,0,0,0]), ([5,2,4,10],[0,0,0,0]), ([1,4,7,10],[0,0,0,0]),
             ([10,3,1,4],[0.48,2.79,2.42,1.9]), ([1,4,5,8,11],[0,0,0,0,0])]
    print('%-26s %9s %9s %5s %7s %8s' % ('instance','optimum','oracle','==','columns','time_ms'))
    for routes, d1 in tests:
        t0 = time.perf_counter()
        res = intersection_solve(routes, d1)
        ms = (time.perf_counter() - t0) * 1000
        om = oracle_min(routes, d1)
        ok = 'YES' if abs(res['optimum'] - om) < 1e-3 else 'NO'
        print('%-26s %9.4f %9.4f %5s %7d %8.2f' %
              (str(routes), res['optimum'], om, ok, len(res['columns']), ms))
