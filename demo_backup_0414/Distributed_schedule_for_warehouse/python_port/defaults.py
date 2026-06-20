"""
Single source of truth for default scenario / algorithm parameters.

ALL CLI argparse defaults, generator function defaults, and wrapper
defaults should import from this module instead of hard-coding values.
That way changing a default in one place updates the whole pipeline.
"""

# ─── Physical / scenario timing ─────────────────────────────────────────
DT           = 4.0    # road traversal time (s)
T_VAL        = 2.0    # headway between vehicles at same entrance (s)
T_ENT        = 0.0    # stagger between different entrances (s)
V_MAX        = 1.0    # vehicle approach speed (m/s)
DETECT_RANGE = 7.6    # detection zone diameter (m)
W_PHYS       = 1.6    # merging-zone width (m), used for base_time

# ─── Vehicle / config generation ────────────────────────────────────────
N_DEFAULT       = 20     # default fleet size
SEED_DEFAULT    = 47    # default RNG seed
MAX_PER_INT     = None  # per-int cap (None → uses N, no cap)

# ─── Topology ───────────────────────────────────────────────────────────
N_INT_DEFAULT   = 6     # random topology default # of intersections
GRID_SIZE       = 6     # interactive picker default grid size

# ─── ADMM hyperparameters ───────────────────────────────────────────────
RHO1             = 1.0
RHO2             = 1.0
WEIGHT           = 1.5
ALPHA_RELAX      = 1.0
USE_ADAPTIVE_RHO = True
USE_PARALLEL     = True
USE_PRUNING      = True
USE_WEAK_RULE    = True
USE_QUADPROG     = True
USE_T_BOUND      = True
TIMEOUT_INT_S    = 30
RAND_INIT_SCALE  = 0.0
PRIORITY_N       = 0
MAX_ITER         = 300
TOL_R            = 1e-2
TOL_S            = 1e-2

# ─── BnB ────────────────────────────────────────────────────────────────
SMAX             = 3       # max sub-tasks per task in centralized framework
BNB_DEADLINE_S   = 300.0   # default BnB time limit
USE_FCFS_BOUND   = True    # warm-start optimal BnB with FCFS upper bound
