"""
Receding-horizon MPC scheduler for warehouse AMR intersection management.

Each MPC step:
  1. Update alpha_tilde from current vehicle positions
  2. Recompute deadlines
  3. Solve ADMM (sequential or parallel)
  4. Return optimal arrival/departure schedule

Physical model
--------------
  - Vehicles approach their first intersection at speed v_max (m/s)
  - alpha_tilde[n] = time vehicle n enters the detection zone of its
    first intersection (= zone_half_width / v_max + departure_stagger)
  - initial_pos[n] = -(zone_half_width + alpha_tilde[n] * v_max)
  - At time t: pos[n] = initial_pos[n] + integral(v_actual[n], t)

MPC update rule
---------------
  - If pos[n] < zone_entry: alpha_tilde[n] = t + (zone_entry - pos[n]) / v_max
  - If pos[n] >= zone_entry (vehicle already in zone): alpha_tilde[n] = t (now)
  - deadline[n] = alpha_tilde[n] + path_span[n]   (path_span is constant)
"""

import time
import copy
import numpy as np
from .admm_core import run_admm_core, warmup_parallel_pool


class MPCScheduler:
    """
    Receding-horizon ADMM scheduler.

    Parameters
    ----------
    const              : dict   base const from load_config or manual setup
    agent_participation: list   agent_participation[ag][n] truthy ↔ vehicle n uses agent ag
    v_max              : float  vehicle speed (m/s); default 1.0
    detect_range       : float  detection range (m); default 7.6
    W                  : float  merging zone width (m); default 1.6
    """

    def __init__(self, const, agent_participation,
                 v_max=1.0, detect_range=7.6, W=1.6):
        self._const_base = const
        self._ap         = agent_participation
        self._N          = const['N']
        self._v_max      = v_max
        self._detect_range = detect_range
        self._W          = W

        # Zone entry position (< 0, vehicle must reach this to enter detection zone)
        self._zone_entry = -(detect_range / 2.0 - W / 2.0)   # e.g. -3.0 m

        # Per-vehicle constant: deadline - alpha_tilde (traversal span, doesn't change)
        self._path_span = np.array([
            float(const['deadline'][n][0]) - float(const['alpha_tilde'][n][0])
            for n in range(self._N)
        ])

        # Initial vehicle positions derived from alpha_tilde and v_max
        self._pos0 = np.array([
            -(detect_range / 2.0 - W / 2.0 + float(const['alpha_tilde'][n][0]) * v_max)
            for n in range(self._N)
        ])

        # Mutable position state (updated by simulate())
        self._pos = self._pos0.copy()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def warmup(self):
        """Pre-warm the parallel pool once at system startup."""
        warmup_parallel_pool()

    def reset(self):
        """Reset vehicle positions to initial state."""
        self._pos = self._pos0.copy()

    def alpha_tilde_from_positions(self, t_current, positions=None):
        """
        Compute alpha_tilde for all vehicles given current positions.

        Parameters
        ----------
        t_current : float   current wall time (s)
        positions : array_like of length N, or None → use internal state

        Returns
        -------
        list[N] of np.ndarray([alpha])
        """
        pos = np.asarray(positions) if positions is not None else self._pos
        alpha = np.where(
            pos >= self._zone_entry,
            t_current,                                                  # already in zone
            t_current + (self._zone_entry - pos) / self._v_max         # time to zone
        )
        return [np.array([float(a)]) for a in alpha]

    def step(self, t_current, positions=None, use_parallel=True):
        """
        One MPC step: update alpha_tilde from positions, solve ADMM.

        Parameters
        ----------
        t_current   : float        current time (s)
        positions   : array_like   current vehicle positions (m); None → internal state
        use_parallel: bool         use ProcessPoolExecutor for intersection agents

        Returns
        -------
        x_prev      : list[9][N]   optimal arrival times per agent per vehicle
        y_prev      : list[8][N]   optimal departure times
        delay_cost  : float        total delay cost at convergence
        T_solve     : float        ADMM wall-clock solve time (s)
        k           : int          ADMM iterations to convergence
        alpha_tilde : list[N]      alpha_tilde used in this step
        """
        alpha_new = self.alpha_tilde_from_positions(t_current, positions)
        deadline_new = [
            np.array([float(alpha_new[n][0]) + self._path_span[n]])
            for n in range(self._N)
        ]

        const_step = {
            **self._const_base,
            'alpha_tilde':  alpha_new,
            'deadline':     deadline_new,
            'useParallel':  use_parallel,
        }

        x_prev, y_prev, _, _, _, delay_costs, k, T_solve = run_admm_core(
            const_step, self._ap)

        return x_prev, y_prev, float(delay_costs[k - 1]), T_solve, k, alpha_new

    def simulate(self, n_steps, dt=0.5, disturbance_std=0.0,
                 use_parallel=True, verbose=True, seed=None):
        """
        Run the MPC loop for n_steps time steps.

        At each step:
          - Advance vehicle positions by v_actual * dt
            where v_actual[n] ~ N(v_max, disturbance_std)
          - Re-solve ADMM with updated alpha_tilde
          - Record results

        Parameters
        ----------
        n_steps         : int    number of MPC steps to run
        dt              : float  time step (s)
        disturbance_std : float  std of speed noise (m/s); 0 → deterministic
        use_parallel    : bool
        verbose         : bool
        seed            : int or None   RNG seed for reproducible disturbances

        Returns
        -------
        history : list of dicts, one per step:
            't', 'delay_cost', 'T_solve', 'k',
            'alpha_tilde', 'terminal_x', 'positions'
        """
        if seed is not None:
            rng = np.random.default_rng(seed)
        else:
            rng = np.random.default_rng()

        self.reset()
        history = []
        t = 0.0

        for step_idx in range(n_steps):
            # ── Solve ───────────────────────────────────────────────
            pos_snapshot = self._pos.copy()
            x_prev, y_prev, delay_cost, T_solve, k, alpha_new = self.step(
                t, positions=pos_snapshot, use_parallel=use_parallel)

            # ── Terminal arrival times ───────────────────────────────
            terminal_x = np.array([
                float(x_prev[8][n][0]) if x_prev[8][n] is not None else np.nan
                for n in range(self._N)
            ])

            if verbose:
                print(f'[MPC t={t:.1f}s] k={k}  cost={delay_cost:.4f}  '
                      f'T_solve={T_solve:.2f}s  '
                      f'terminal[0..2]={terminal_x[:3].round(3).tolist()}')

            history.append({
                't':           t,
                'delay_cost':  delay_cost,
                'T_solve':     T_solve,
                'k':           k,
                'alpha_tilde': [float(a[0]) for a in alpha_new],
                'terminal_x':  terminal_x,
                'positions':   pos_snapshot.copy(),
            })

            # ── Advance state ────────────────────────────────────────
            if disturbance_std > 0:
                v_actual = self._v_max + rng.normal(0.0, disturbance_std, self._N)
                v_actual = np.clip(v_actual, 0.1, None)   # no negative speeds
            else:
                v_actual = np.full(self._N, self._v_max)

            self._pos += v_actual * dt
            t += dt

        return history
