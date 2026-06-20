"""
Minimal Vehicle / TaskProfile dataclasses needed by the centralized FCFS
solver. The original ``vehicle.py`` from Four_int_031226 also bundles
generation/builder helpers tied to its own traffic_system module; we strip
those out because this project's adapter (``python_port/fcfs.py``) builds
the task_cache itself from the distributed-side ``IntSpaceDB``.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np


@dataclass
class Vehicle:
    entrance: int
    exit: int
    entry_index: int
    int_seq: List[int]
    sub_dir: List[int]
    route_id: List[int]
    NI: int
    alpha0: float


@dataclass
class TaskProfile:
    """C, Map padded to length Smax. Only first L entries are valid."""
    C: np.ndarray   # shape (Smax,)
    Map: np.ndarray # shape (Smax,) — 1-indexed global space ids
    int_id: int
    route_id: int
    L: int
