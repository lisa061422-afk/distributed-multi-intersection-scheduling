---
title: "A No-Wait Single-Intersection Scheduler by Priority Enumeration: A Decision Tree with a Longest-Path Timing Model"
---

# Problem Formulation

Let $\mathcal{N}=\{1,\dots,N\}$ index the vehicles to be scheduled through the merging zone, and let $\mathcal{M}$ be the set of conflict points (*spaces*). Vehicle $n$ traverses an ordered sequence of sub-tasks $s\in\mathcal{S}_n=\{1,\dots,S_n\}$; sub-task $(n,s)$ occupies conflict point $\sigma(n,s)\in\mathcal{M}$ for a fixed duration $c_{n,s}>0$. Define the prefix offset and the total traversal length

$$\pi_{n,s}=\sum_{k=1}^{s-1}c_{n,k},\qquad C_{n}=\sum_{s\in\mathcal{S}_n}c_{n,s},$$

and let $\alpha_n$ denote the earliest admissible entry (release) time of vehicle $n$.

## No-wait motion

Each vehicle crosses the intersection without stopping, so its sub-tasks are contiguous. The schedule of vehicle $n$ is therefore parameterized by a *single* entry time $t_n\ge\alpha_n$, under which sub-task $(n,s)$ occupies $\sigma(n,s)$ over the interval

$$I_{n,s}(t_n)=\left[\, t_n+\pi_{n,s},\;\; t_n+\pi_{n,s}+c_{n,s}\,\right].$$

## Scheduling problem

Two sub-tasks conflict iff they use the same conflict point; collect such pairs in

$$\mathcal{K}=\left\{\, \{(n,s),(n',s')\}:\sigma(n,s)=\sigma(n',s'),\;n\neq n'\,\right\}.$$

A schedule is *feasible* iff conflicting occupations never overlap. Minimizing the total delay yields

$$(\mathrm{P})\qquad \min_{t\in\mathbb{R}^{N}}\; \sum_{n\in\mathcal{N}}\left(t_n-\alpha_n\right)$$

subject to

$$t_n\ge\alpha_n,\quad n\in\mathcal{N},$$
$$I_{n,s}(t_n)\cap I_{n',s'}(t_{n'})=\varnothing,\quad \forall\{(n,s),(n',s')\}\in\mathcal{K}.$$

Since motion is no-wait, the exit time of $n$ is $t_n+C_n$, so $\sum_n(t_n-\alpha_n)$ equals the total completion delay.

# Disjunctive Form and the Selection

Each non-overlap constraint is a disjunction. For $\{(n,s),(n',s')\}\in\mathcal{K}$, the orientation *"$(n,s)$ before $(n',s')$"* reads

$$t_{n'}-t_{n}\;\ge\;w_{n\to n'}\;:=\;\pi_{n,s}+c_{n,s}-\pi_{n',s'},$$

and the reverse orientation has weight $w_{n'\to n}=\pi_{n',s'}+c_{n',s'}-\pi_{n,s}$. A *selection* $\xi$ orients every conflict pair, i.e. it is a set of weighted precedence arcs $i\xrightarrow{w}j$ on the vehicle nodes. Adjoining the release arcs $0\xrightarrow{\alpha_n}n$ from a source node $0$ yields the precedence graph $G_\xi=\left(\{0\}\cup\mathcal{N},\,A_\xi\right)$.

# Timing Model

**Proposition 1 (Earliest-entry timing).** *If $G_\xi$ is acyclic, then problem $(\mathrm{P})$ restricted to the orientation $\xi$ has a unique pointwise-minimal solution $t^\star(\xi)$, characterized by the longest-path recursion*

$$t^\star_n(\xi)=\max\left(\alpha_n,\; \max_{\,i:\,(i\to n)\in A_\xi}\left(t^\star_i(\xi)+w_{i\to n}\right)\right),\quad n\in\mathcal{N},$$

*computed by the fixed-point (Bellman–Ford) iteration $\mathsf{EarliestT}(\xi)$. If $G_\xi$ contains a positive cycle, $\xi$ is infeasible.*

The recursion above is the **timing model** $\mathsf{EarliestT}:\xi\mapsto t^\star(\xi)$: it maps an ordering to the unique contention-free, no-wait schedule that starts every vehicle as early as the chosen precedences permit. Its cost is

$$J(\xi)\;=\;\sum_{n\in\mathcal{N}}\left(t^\star_n(\xi)-\alpha_n\right).$$

# Reduction to a Finite Search

**Proposition 2.** $\min(\mathrm{P})=\min_{\xi\in\Xi}J(\xi)$, *where $\Xi$ is the finite set of acyclic selections.*

The optimum is attained by a semi-active schedule; hence it suffices to search over orderings, the timing model supplying the optimal entry times of each.

# Decision-Tree Search

A node of the search tree is a *partial* selection $\hat\xi$ with decided set $D\subseteq\mathcal{K}$. The search interleaves the timing model with conflict detection. The conflict oracle $\mathsf{FindConflict}(t,D)$ returns the first pair $\{(n,s),(n',s')\}\in\mathcal{K}\setminus D$ whose intervals overlap at the current schedule $t$, i.e. satisfying

$$t_n+\pi_{n,s}<t_{n'}+\pi_{n',s'}+c_{n',s'}\;\;\wedge\;\;t_{n'}+\pi_{n',s'}<t_n+\pi_{n,s}+c_{n,s},$$

and returns $\textsc{none}$ if no such pair exists.

**Algorithm 1: No-wait enumeration scheduler.** $\mathsf{Recurse}(\hat\xi, D)$:

1. $t\gets\mathsf{EarliestT}(\hat\xi)$ &nbsp;&nbsp; *(timing model, Prop. 1)*
2. **if** $t=\textsc{none}$: **return** &nbsp;&nbsp; *(positive cycle $\Rightarrow$ infeasible — prune)*
3. $\kappa\gets\mathsf{FindConflict}(t,D)$ &nbsp;&nbsp; *(first overlapping, undecided pair)*
4. **if** $\kappa=\textsc{none}$: record leaf $t^\star\gets t$; **return** &nbsp;&nbsp; *(conflict-free complete schedule)*
5. $\{(n,s),(n',s')\}\gets\kappa$
6. $\mathsf{Recurse}\!\left(\hat\xi\cup\{n\xrightarrow{w_{n\to n'}}n'\},\; D\cup\{\kappa\}\right)$ &nbsp;&nbsp; *(branch: $(n,s)$ before $(n',s')$)*
7. $\mathsf{Recurse}\!\left(\hat\xi\cup\{n'\xrightarrow{w_{n'\to n}}n\},\; D\cup\{\kappa\}\right)$ &nbsp;&nbsp; *(branch: $(n',s')$ before $(n,s)$)*

Initialize with $\mathsf{Recurse}(\varnothing,\varnothing)$.

The set of leaves enumerates $\{t^\star(\xi):\xi\in\Xi\}$, i.e. all contention-free complete schedules; the minimum-cost leaf solves $(\mathrm{P})$. A positive-cycle test in $\mathsf{EarliestT}$ prunes infeasible orientations, so only acyclic selections are expanded.
