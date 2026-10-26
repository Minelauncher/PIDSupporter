# Collection duration and accumulation results

Current 1.1.5 tuning policies; 72 synthetic cases; no measurement noise. Production source, installed mod and Workshop were not modified.

All methods use the same prepared PID. Repeated collections do not apply intermediate recommendations. The main table evaluates all recommendations from the same first-30-second endpoint.

| Collection | Mean cost / finite-pool best | Median | Worst | Worse than working PID | Better / worse than single30 |
|---|---:|---:|---:|---:|---:|
| single30 | 1.477 | 1.012 | 6.717 | 5/72 | 0 / 0 |
| repeat2 | 2.306 | 1.098 | 17.021 | 11/72 | 18 / 31 |
| continuous60 | 1.594 | 1.021 | 7.397 | 8/72 | 15 / 20 |
| repeat4 | 1.733 | 1.238 | 7.824 | 5/72 | 13 / 34 |
| continuous120 | 1.484 | 1.020 | 5.373 | 4/72 | 14 / 16 |

A ratio of 1 means matching the best center in the shared finite pool, not a global optimum. Tracking costs include the same joint neighborhoods for every method; changes within 1% count as similar. No nominal or neighborhood threshold failures occurred.

The 30-second reference mean differs from the earlier 1.457 because the common pool now includes the extra recommendations from all five arms. This changes the reference best without changing the data.

## Main findings

- Longer collection does not consistently reduce selection loss under the unchanged policy. Single30 has the lowest primary mean (1.477); continuous120 is effectively tied (1.484).
- Repeating 30-second collection twice is worse on average (2.306), with 31 worse and 18 better cases than single30. Four repeats recover partly (1.733), but remain worse in 34 cases and better in 13.
- Continuous120 reduces the worst finite-pool ratio from 6.717 to 5.373 and the number worse than the working PID from 5 to 4; this is not a broad improvement across conditions.
- At equal total time, repeat2 beats continuous60 in 20 cases and loses in 23; repeat4 beats continuous120 in 13 and loses in 30.
- Exact nominal PID evaluation also worsens in more cases after repeat2: 15 worse than the working PID versus 8 for single30. The result is not solely a neighborhood-scoring artifact.

## Evaluation-start sensitivity

The second check evaluates every recommendation from the same continuous120 endpoint instead. Mean finite-pool ratios:
- single30: 1.384
- repeat2: 2.244
- continuous60: 1.497
- repeat4: 1.693
- continuous120: 1.356

The repeated-collection regression persists. Continuous120 has a small advantage at this later endpoint (1.356 versus 1.384), so the small ordering difference between single30 and continuous120 is state-dependent.

## Coupled disturbance subset (12 cases)

| Arm | Mean finite-pool ratio | Worse than working PID |
|---|---:|---:|
| single30 | 3.390 | 4 |
| repeat2 | 4.183 | 5 |
| continuous60 | 3.973 | 7 |
| repeat4 | 3.178 | 1 |
| continuous120 | 3.271 | 2 |

## A verified source of accumulated-data regression

The clean third-order resonance cases p061 and p063 expose a gate limitation, rather than failed identification. Their selected models stay third order with maximum absolute frequency-response error against truth around 0.3–3.4 × 10^-6 on the checked frequencies. Truth was accessed only after selection for this diagnosis.

For p063, the previous good PID [1, 4, 0.5] has maximum empirical-gate rho 0.292 with one record, but 1.338 after the second record, crossing the rejection threshold of 1. The new recommendation is [0.542, 7.9, 0.5], and its primary cost is 16.83 times the single30 recommendation. The controlling discrepancy is at 0.6 Hz in the first 5-second window of the second record. The input Fourier magnitude there is 1.905, so it is not simply an almost-zero input denominator.

The gate uses the maximum discrepancy of finite-window response estimates. Initial transients/window effects can be treated as plant uncertainty even when the fitted plant is accurate. Adding windows cannot shrink a fixed-model maximum. A new phase and record start can therefore exclude a genuinely good PID rather than improve its ranking. This explains these two cases, not every regression in the study; latest-state initialization, validation changes and nonlinear/coupled behavior also differ across arms.

## Scope and reproducibility

- Source hashes confirm no production edits. All 72 first30 U/Y/T/PID/injected-SP/instrument arrays match the prior production acquisition harness.
- The 60/120-second whitelist and collection-duration adaptations exist only in research copies. Each continuous arm preserves the production train/validation/final proportions and fixed frequency; no richer excitation policy is added.
- Repeated runs retain physical/controller state but restart the waveform phase/sign and endpoint fades; their realized input energy is not matched to a continuous run.
- Search evaluation limits are unchanged. Budget-hit flags occur, but no search reaches the 10-second wall-clock limit in this run; reported hits are the evaluation-count budget.
- This is a fixed-collection-PID information study, not repeated apply/re-tune convergence. Results do not establish monotonic improvement with more data.

See [protocol](README.md), [summary](results/summary.json), [paired results](results/paired-results.json), and [gate diagnosis](results/gate-diagnosis.json).
