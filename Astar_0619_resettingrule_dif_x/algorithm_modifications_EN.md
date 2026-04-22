# WeakRule ON Algorithm: Core Modifications

## Mod 1: Pairwise Priority Lock (pair_lock)

**Problem:** Two systems repeatedly competing for the same resource generate a new branch on every encounter, causing search tree explosion and cycling.

**Modification:** Introduce an N×N matrix `pair_lock(i,j) = winner` that records the outcome of each pairwise competition.

**Auto-win condition:** Candidate c auto-wins space m (no branching) if and only if:
1. `pair_lock(c, o) = c` holds for every other contender o on space m
2. c is actively executing its current sub-task (`ra > 0`)

**Completeness guarantee:** All priority combinations are enumerated at the *first* competition (each pair generates two branches). Subsequent lock enforcement only prevents redundant re-branching on the *same* path; sibling branches remain unaffected. When a third system enters the competition with `pair_lock = 0`, new branches are automatically created.

**Key design decision:** The lock is never cleared on reset. Clearing locks was the root cause of cycling in the prior version.

---

## Mod 2: Co-Decision of x and V_temp (space_variants)

**Problem:** In the original method, the rescheduled sub-task occupation (x) depends unilaterally on the current V_temp occupant, making it impossible to find optimal solutions that require reassigning V_temp entries.

**Modification:** Introduce `space_variants`, which generates multiple branches when a reset system n needs to re-enter space m currently occupied by system np:

| pair_lock(n, np) | Branches generated |
|---|---|
| 0 (never competed) | both wait and displace |
| = np (np won before) | wait only |
| = n (n won before) | displace only |

**Displace branch:** np is evicted from V_temp and fully reset; n acquires priority on space m. The rescheduled start time of n's sub-task (x) is set to align with the moment m becomes available, eliminating idle gaps.

**Constraint:** Only the pending V_temp requests of other systems are modified. The V_valid assignments already committed by `traverse_columns` in the main branching loop remain untouched.

---

## Mod 3: reset_since — Marking Stale Occupation Records

**Problem:** After system n is reset, its historical V occupation records are no longer valid for the current branch. However, V cannot be directly modified since it is shared across ancestor nodes.

**Modification:** Each node carries a vector `reset_since[n]`:
- When n is reset at time tw: set `reset_since[n] = tw`
- Clear stale reservations: `x{n}{k} = {}`

**Usage:** `check_resc_occupation` ignores any V entry for system n with a timestamp earlier than `reset_since[n]`.

**Final schedule reconstruction:** A complete path's schedule is assembled from:
- `gamma[n]`: authoritative occupation intervals recorded when each sub-task completes
- `x{n}{k}`: reservation intervals generated during resets, of the form `{t_start, t_end, sub_index, space}`

The two are merged and filtered by `reset_since` to produce the valid scheduling timeline for each system.

---

## Mod 4: T_bound Pruning

**Problem:** The open list accumulates many nodes whose schedules are provably worse than a naive serial baseline, wasting search time.

**Modification:** Compute a serial worst-case upper bound:

$$T_{\text{bound}} = \max_n \tilde{\alpha}_n + \sum_{n=1}^{N}\sum_{s=1}^{S} C_{s,n}$$

where $\tilde{\alpha}_n = d1_n + \frac{R/2 - W/2}{v_{\max}}$ is the earliest arrival time of system n at the merge zone.

**Pruning rule:** Any node in OPEN with `tw > T_bound` is immediately removed.

**Interpretation:** $T_{\text{bound}}$ represents the completion time if the last-arriving system waits for all other systems' sub-tasks to execute serially before it begins. Any valid cooperative schedule must finish before this bound; nodes exceeding it cannot lead to competitive solutions.
