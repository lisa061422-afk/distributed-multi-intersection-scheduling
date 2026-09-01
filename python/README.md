# Python distributed scheduler

This directory contains the Python implementation of the distributed C-ADMM
multi-intersection scheduling algorithm. It is the Python counterpart of the
MATLAB scheduler in `matlab/`.

Given robot routes, this code schedules conflict-free crossing orders and times
across multiple intersections.

## Quick start

Run commands from the repository root:

```powershell
pip install -r python/requirements.txt
python python/main.py
```

Edit the configuration block at the top of `main.py` to select random,
interactive, or manual topology input. Two reusable manual maps are included:

- `warehouse_2x2.py` — four-intersection 2-by-2 warehouse map.
- `example_manual_topo.py` — five-intersection example map.

Additional checks:

```powershell
python python/run_validation.py
python python/run_mpc_demo.py
```

## Included source

- `admm_core.py` — distributed C-ADMM iteration and optional parallel workers.
- `decision_tree.py`, `in_admm.py` — local decision-tree and QP subproblem.
- `road_agent.py`, `terminal_agent.py` — distributed agent updates.
- `topology.py`, `generate_config.py` — topology and scenario construction.
- `mpc_loop.py` — receding-horizon scheduling wrapper.
- `main.py` — direct runnable entry point.

Only the scheduler and its required configuration/validation files are kept.
PPT/PPTX generators and documents, cached bytecode, experiment logs, batch
sweeps, generated plots, web-demo exporters, and the separate centralized
FCFS/branch-and-bound implementation are intentionally excluded.
