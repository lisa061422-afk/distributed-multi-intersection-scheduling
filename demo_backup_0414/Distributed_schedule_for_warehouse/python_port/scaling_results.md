# Scaling Convergence Sweep Results

Random topology + random vehicle assignment, ADMM convergence rate vs network size.

## Sweep 1 — 2026-04-29

**Setup**
- N (vehicles) = 6
- Dt = 2.0 s
- T_val = 2.0 s (paper-faithful headway)
- max_iter = 200
- tol_r = tol_s = 0.01

**Parameter sets compared**
- `default` : ρ = 1.0, α = 1.0, adaptive_rho off (paper baseline)
- `tuned`   : ρ = 0.3, α = 0.7, adaptive_rho on

**Convergence rate** (✓ converged within 200 iter, 5 seeds per cell)

| n_int | default rate | tuned rate | seeds (default) | seeds (tuned) | notes |
|------:|:------------:|:----------:|:----------------|:--------------|:------|
|     4 |     80%      |    80%     | OK OK -- OK OK  | OK OK -- OK OK | seed=33 fails both |
|     6 |     80%      |    80%     | -- OK OK OK OK  | -- OK OK OK OK | seed=11 fails both |
|     8 |     80%      |  **100%**  | OK OK OK -- OK  | OK OK OK OK OK | tuning helps at boundary |
|    10 |     40%      |    40%     | OK -- -- -- OK  | OK -- -- -- OK | tuning ineffective |
|    12 |     40%      |    40%     | OK -- -- -- OK  | OK -- -- -- OK | tuning ineffective |
|    15 |     60%      |    60%     | -- OK -- OK OK  | -- OK -- OK OK | not strictly monotonic |

Seeds tested: 11, 22, 33, 42, 99 (in column order).

## Findings

1. **Reliable region: n_int ≤ 8.** ≥80% with paper defaults; 100% at n=8 with tuning.
2. **Cliff at n_int = 10–12.** 40% rate; **parameter tuning does NOT help in this region**.
3. **Non-monotonic beyond n=12.** n=15 gives 60% — suggests instance-dependent rather than strict size limit. Need more seeds to characterize tail.
4. **Even n=4 not 100%.** Random vehicle assignments occasionally produce hard instances even at paper baseline scale.
5. **Tuning sweet spot is the cliff edge (n=8).** Deeper failures (n≥10) are structural — see root cause below.

## Root cause of structural failure (n_int ≥ 10)

`[IN_Admm] Agent X sys Y leaf Z: empty bar/alpha/gamma — skipping system.`

In [`python_port/node.py:74`](node.py#L74) (mirrors MATLAB `NewNode.m`), `alpha[n]` and
`gamma[n]` are assigned **only at the transition where vehicle n's residual
execution time goes from positive to zero**. When T-bound pruning leaves a
single surviving leaf in which some vehicle never reaches that transition
(e.g., it remains queued behind another vehicle for the entire scheduled
window), that vehicle's `alpha`/`gamma` stay `None` and `IN_Admm`
skips it.

The skip leaves that vehicle's primal stale at this agent while neighbours
keep updating, so its dual variable accumulates linearly across iterations
→ slow divergence dominated by period-2 oscillation.

This is **not** a Python port bug — the MATLAB source contains the same
skip logic. It is an algorithmic corner case that becomes more frequent
in larger / more heterogeneous topologies.

## Reproduction

```powershell
# Single n_int / seed run with verbose output
python python_port/run_arbitrary_demo.py --mode random --n_int 8 --N 6 --seed 42 `
    --max_iter 200 --Dt 2.0 --rho 0.3 --alpha 0.7 --adaptive_rho --plot

# Programmatic sweep (edit n_ints / seeds list inline)
python -c "
from python_port.run_scaling import run_one
for n_int in [4,6,8,10,12,15]:
    for seed in [11,22,33,42,99]:
        r = run_one(n_int=n_int, N=6, seed=seed, max_iter=200, Dt=2.0,
                    use_parallel=False, rho=0.3, alpha=0.7, adaptive_rho=True)
        print(n_int, seed, 'OK' if r.get('converged') else '--')
"
```

## TODO / future runs

- [ ] Densify n_int = 10..15 with 10+ seeds each to characterize cliff statistically
- [ ] Sweep N (vehicles) at fixed n_int — does N matter independent of n_int?
- [ ] Try N proportional to n_int (e.g., N = n_int) — realistic "city scales with cars"
- [ ] Attempt fix: modify `node.py:74` so `alpha`/`gamma` are populated at all leaves where `ni2 == NI_agent + 1`, not just on the residual transition. Verify 4-int baseline preserved.
