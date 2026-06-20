"""
DFS+B&B and BFS search loops on top of expand_pure.

Both build a list[Node] in-place; nodes are appended as they are discovered.
DFS_BB tracks best_g during the run for branch-and-bound pruning and is
anytime (returns best-so-far on deadline).
"""
from __future__ import annotations

import math
import time
from typing import List, Tuple

from .expand import expand_pure
from .node import Node


def search_dfs_bb(root: Node, const: dict,
                  deadline: float | None = None,
                  verbose: bool = True,
                  initial_best_g: float = math.inf
                  ) -> Tuple[List[Node], List[int], int, float, list]:
    """Run DFS + branch-and-bound from `root`.

    initial_best_g: if a feasible upper bound is known (e.g. from FCFS),
    pass it here so BnB prunes aggressively from the start.

    Returns (nodes, leaves, best_idx, best_g, log_lines).
    """
    nodes: List[Node] = [root]
    root.idx = 0
    leaves: List[int] = []
    best_g = float(initial_best_g)
    best_idx = -1
    log: list = []
    if math.isfinite(best_g):
        log.append(f"[DFS_BB] initial bound from FCFS: best_g = {best_g:.6f}")
        if verbose:
            print(log[-1])

    stack: List[int] = [0]
    n_pruned = 0
    step = 0
    t0 = time.perf_counter()
    deadline_hit = False

    while stack:
        c_idx = stack.pop()
        step += 1

        if deadline is not None and (time.perf_counter() - t0) > deadline:
            deadline_hit = True
            break

        # bound prune
        if nodes[c_idx].g >= best_g - 1e-9:
            n_pruned += 1
            continue

        children, is_leaf = expand_pure(nodes, c_idx, const)

        if is_leaf:
            leaves.append(c_idx)
            leaf_g = nodes[c_idx].g
            if leaf_g < best_g:
                best_g = leaf_g
                best_idx = c_idx
                msg = f"[DFS_BB] step {step}: new best_g = {best_g:.6f} at node {c_idx} (#leaf={len(leaves)})"
                log.append(msg)
                if verbose:
                    print(msg)
            continue

        # sort children by g ascending; push reversed so smallest-g is popped first
        if not children:
            continue
        order = sorted(range(len(children)), key=lambda k: children[k].g, reverse=True)
        for k in order:
            child = children[k]
            if child.g >= best_g - 1e-9:
                n_pruned += 1
                continue
            child.idx = len(nodes)
            nodes.append(child)
            stack.append(child.idx)

    suffix = " (deadline hit, suboptimal)" if deadline_hit else ""
    summary = (f"[DFS_BB] done. expanded={step}, pruned={n_pruned}, "
               f"leaves={len(leaves)}, best_g={best_g:.6f}{suffix}")
    log.append(summary)
    if verbose:
        print(summary)
    return nodes, leaves, best_idx, best_g, log


def search_bfs(root: Node, const: dict, verbose: bool = True
               ) -> Tuple[List[Node], List[int], int, float, list]:
    """Level-by-level BFS.  Returns same tuple as search_dfs_bb."""
    nodes: List[Node] = [root]
    root.idx = 0
    leaves: List[int] = []
    best_g = math.inf
    best_idx = -1
    log: list = []

    level_open = [0]
    step = 0

    while level_open:
        step += 1
        next_open: List[int] = []
        for c_idx in level_open:
            children, is_leaf = expand_pure(nodes, c_idx, const)
            if is_leaf:
                leaves.append(c_idx)
                leaf_g = nodes[c_idx].g
                if leaf_g < best_g:
                    best_g = leaf_g
                    best_idx = c_idx
                continue
            for child in children:
                child.idx = len(nodes)
                nodes.append(child)
                next_open.append(child.idx)

        if verbose:
            print(f"[BFS] level {step}: #nodes={len(nodes)}, |OPEN|={len(next_open)}, |LEAF|={len(leaves)}")
        level_open = next_open

    summary = f"[BFS] done. levels={step}, nodes={len(nodes)}, leaves={len(leaves)}, best_g={best_g:.6f}"
    log.append(summary)
    if verbose:
        print(summary)
    return nodes, leaves, best_idx, best_g, log


def extract_path(nodes: List[Node], leaf_idx: int) -> List[int]:
    """Trace from leaf back to root via parent pointers."""
    out = []
    cur = leaf_idx
    while cur >= 0:
        out.append(cur)
        cur = nodes[cur].parent
    return list(reversed(out))
