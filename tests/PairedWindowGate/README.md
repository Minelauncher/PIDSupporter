# Transient-sensitive empirical gate audit — 2026-10-27

Research only. Production, installed DLL and Workshop are unchanged. Thirty
production source hashes were checked before and after the experiment.

The requested exact-model accumulation failure is reproducible and repairable.
Replacing a raw short-window FRF discrepancy with an equation residual reduces
false rejections and average selection loss, while retaining the latest Spirit
working PID and blocking its aggressive ungated PID. It does **not** dominate the
old gate in every case and is not a certified robust-stability test.

## Cause and variants

The old radius is the maximum of `abs(Y_window/U_window - G_model)` over
5-second Hann windows. Even a perfect model need not equal this finite-window
ratio: start/end transients and windowing contribute to the discrepancy. Taking
the maximum over additional records can preserve a spurious large discrepancy.
This is a known FRF measurement issue, distinct from model identification error:
[Schoukens, Rolain and Pintelon, 2006](https://www.sciencedirect.com/science/article/pii/S0005109805002803).

All variants retain the ten frequencies 0.2, 0.6, ..., 3.8 Hz, the first two thirds
of each record, maximum aggregation, nominal Schur stability and threshold one.
There is no added gain cap, radius multiplier or deletion of adverse windows.

1. **Legacy:** original window FRF discrepancy.
2. **Paired:** free rollout from the first n observed output samples; transform
   measured minus simulated output using identical windows. No output correction.
3. **Warmup (exploratory):** same, with only the n initial-state coefficients fitted
   over the first 2 seconds. This uses the existing IV history duration.
4. **Equation (exploratory):** first form the residual of the fixed model equation,
   then convert its window spectrum to an output-equivalent discrepancy.

For the physical affine ARX model

$$y_k=\sum_{j=1}^n a_j y_{k-j}+\sum_{j=1}^n b_j u_{k-j}+c,$$

the equation residual and experimental radius are

$$e_k=y_k-\sum a_jy_{k-j}-\sum b_ju_{k-j}-c,\qquad
w(\omega)=\max_s\left|\frac{E_s(\omega)}{A(e^{-i\omega h})U_s(\omega)}\right|.$$

The first n residual samples are zero because pre-record histories are not known.
The affine offset stays fixed; no per-record disturbance or trend is fitted away.
The gate checks `w * abs(C/(1+G*C)) < 1` at the same frequencies. Actual saturated
input is used; candidate rollouts retain the production PID saturation/integrator.

For an exact affine ARX model, the equation residual is zero for arbitrary initial
history after n samples. The test fixture gives old maximum radius **1.783**, new
**2.95e-13**. A deliberately 10% wrong numerator still gives **0.150**, so the fix
does not simply erase all model mismatch.

This does **not** prove that the new radius bounds an unknown plant. Converting a
windowed equation residual through nominal A is approximate; a weakly excited
mode, cancellation, correlated disturbance or unobserved frequency can still be
missed. Past observed outputs in this *diagnostic* are not future output feedback
in candidate rollout. The small-gain-shaped inequality is only a guarantee if
the necessary full-band uncertainty and stability assumptions actually hold.

## Protocol and exploration disclosure

- Same 72 synthetic cases: six plant families × six physical conditions × two
  seeds; no measurement noise. Physical disturbances/nonlinearity/coupling remain.
- Frozen existing IV model for each single30, repeat2, repeat4 dataset: 216
  datasets. This isolates the gate from changes in model fitting/order selection.
- Repeated records keep the prepared PID and carry state forward. This is an
  information accumulation comparison, not iterative PID convergence.
- Current C# search, hierarchy, up to 27 joint neighbors and evaluation budget
  remain unchanged. Four gates × 216 datasets, plus five real datasets × five
  methods including off: **889 searches**.
- Truth is used only after search for synthetic scoring and Duster BP scoring.
  The bounds/search input JSON contains only records and frozen identified models.
- A common finite candidate pool includes the original grid, incumbent and every
  arm's recommendations. Nine smooth-SP operating conditions, both signs and
  local PID neighbors are evaluated from the same first-30-second physical state.
  “Best” always means this finite pool; it is not the global optimum.
- Good candidate = physical cost within 10% of this pool best. This tolerance is
  a **reporting definition**, never an acceptance threshold or search input.
- Final thirds are held out from radius construction and audited after selection.
- Paired was the initial planned correction. It rejected the known-good latest
  Spirit PID, leading to warmup, which also rejected it, then equation. All arms
  are reported. Equation is an exploratory result on these logs, not an untouched
  external test. No plant truth was used to tune the formula or threshold.

## Synthetic results

Cost ratio to common-pool best; lower is better, 1 is best. Baseline ratios differ
from the earlier duration report because this pool also includes all new choices.

| Records | Legacy | Paired | Warmup | Equation |
|---|---:|---:|---:|---:|
| 1 × 30 s, mean loss | 1.546 | 1.453 | 1.466 | 1.503 |
| 2 × 30 s, mean loss | 2.760 | 1.823 | 1.848 | 1.780 |
| 4 × 30 s, mean loss | 1.868 | 1.607 | 1.598 | 1.666 |
| 1 × 30 s, worse than incumbent /72 | 5 | 4 | 4 | 6 |
| 2 × 30 s, worse than incumbent /72 | 11 | 8 | 7 | 9 |
| 4 × 30 s, worse than incumbent /72 | 6 | 5 | 3 | 7 |
| 1 × 30 s, good candidates rejected /867 | 74 | 39 | 45 | 40 |
| 2 × 30 s, good candidates rejected /867 | 78 | 42 | 50 | 38 |
| 4 × 30 s, good candidates rejected /867 | 85 | 44 | 49 | 39 |

The 867 candidates are pooled case/candidate pairs, not independent trials.
Equation retains an oracle candidate in 59/72 cases in each arm, versus legacy
46/46/43. In p063 repeat2, the old recommendation `[.542,7.9,.5]` has loss **30.918**
in this expanded pool; equation `[1,.1,6.8]` has loss **1.020**. The earlier 16.83×
number compared against the old single30 recommendation, a different denominator.
This near-exact-model failure was substantially repaired, also in p061.

Equation-selected neighborhoods pass their final-third diagnostic in 64/65/60
cases respectively; failures are still present. This is residual consistency,
not plant-uncertainty coverage. Search budget was reached in 6/4/3 equation runs
(legacy 8/10/10); reported recommendations are the budgeted production search
result, not an exhaustively optimized gain region. Computation time under parallel
offline loading is not a game frame-time measurement.

## Real logs and BP evaluation

PID tuples are `(P, Ti, Td)`.

| Dataset | Gate off | Legacy | Equation |
|---|---|---|---|
| Duster 1 | (.168,1.1,.2) | (.168,1.1,.2) | (.168,1.1,.2) |
| Duster 2, cumulative | (.163,1,.2) | (.175,1.1,.2) | (.175,1.1,.2) |
| Old Spirit 1 | (.988,2.4,.4) | (.074,2.3,.8) | (.081,2.9,.8) |
| Old Spirit 2, cumulative | (1,48.5,.4) | (.088,3.9,.4) | (.078,3.5,.5) |
| Latest Spirit | (.997,.1,1.9) | (.321,.6,.5) | (.294,.5,.7) |

Latest Spirit equation accepts the known-working `(.321,.6,.5)` and incumbent,
while rejecting the aggressive `(.995,.1,1.9)` and `(1,.1,0)` probes. Paired and
warmup reject the known-working candidate, illustrating why synthetic improvement
alone is insufficient. The three Spirit ungated recommendations are also checked
against the changed gate in the saved report/verification.

Duster BP uses frozen physics/context only after selection, with the same
one-step command timing, smooth target and joint neighborhood as the previous
BP scorer. Equation's recommendations and BP costs exactly match legacy:
0.00219517 and 0.000297286. Paired/warmup are 6.82% worse in Duster 1 and unchanged
in Duster 2. BP is an approximate reconstructed plant, not the actual game.
Spirit has no equivalent validated truth scorer here: blocking an aggressive PID
and retaining an observed working PID do not establish the new recommendation's
actual closed-loop performance.

## Reproduce / artifacts

Run from repository root with the existing research Python environment:

```powershell
research_lti/.venv_ren/Scripts/python.exe tests/PairedWindowGate/prepare.py
dotnet build tests/PairedWindowGate/Study.csproj -c Release
dotnet tests/PairedWindowGate/bin/Release/net10.0/Study.dll tests/output/paired-window-gate-20261027 0 1 legacy,paired,warmup,equation
dotnet tests/PairedWindowGate/bin/Release/net10.0/Study.dll tests/output/paired-window-gate-20261027 0 1 off
research_lti/.venv_ren/Scripts/python.exe tests/PairedWindowGate/grade_all.py
research_lti/.venv_ren/Scripts/python.exe tests/PairedWindowGate/verify.py
research_lti/.venv_ren/Scripts/python.exe tests/PairedWindowGate/report.py
```

Run `grade_bp.py` from the parent workspace, before `report.py`. Existing local
duration fixtures, real recordings, BP contexts and historical evaluator helpers
are prerequisites; this is not a standalone simulator distribution. Search files
are resumable: use a new output directory when changing a variant.

`results.json` preserves all case-level scores and game search/probe results.
Full input, search, grading and hash manifests are in
`tests/output/paired-window-gate-20261027/`. Verification checks all 216 old radii
against C# production (maximum absolute discrepancy 2.47e-11), 80 Python/C# gate
probes, exact/wrong-model fixtures and unchanged production sources.

**Decision:** equation residual is a promising correction to the identified
transient artifact, not an across-the-board safety upgrade. Keep the deployed mod
unchanged as requested; do not claim guaranteed coverage or monotone convergence.
