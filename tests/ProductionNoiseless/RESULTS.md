# Noiseless production tuner versus historical FRIT — 2026-10-26

The current implementation is substantially better overall on these 72 cases, but is not uniformly better. No runtime mod code was changed.

## Paired result

Both methods used the identical 30-second current-policy recording and current collection PID. FRIT used the historical C# methods from commit `715d44b` and the original MathNet library; source extraction was checked against Git. This is not a comparison of the two historical collection policies.

| Metric | Current production | Historical FRIT |
|---|---:|---:|
| Improved versus collection PID (>1%) | 61 / 72 | 24 / 72 |
| Worsened (>1%) | 5 / 72 | 48 / 72 |
| Similar (within 1%) | 6 / 72 | 0 / 72 |
| Median tracking-cost ratio to collection PID | 0.506 | 2.062 |
| Nominal PID: improved / worsened | 49 / 8 | 20 / 47 |
| Nominal PID: median ratio | 0.617 | 2.075 |
| Nominal threshold failures | 0 | 0 |
| Neighborhood threshold failures | 0 | 2 |

Current beats FRIT in 53 cases, loses in 18 and is similar in 1 under the joint-neighborhood criterion. Its median cost ratio to FRIT is 0.201. At the exact recommended PID, current wins 53, loses 13 and is similar in 6. Thus the overall result is not solely an advantage of scoring neighborhoods.

## By condition

| Condition (12 cases each) | Current improved / worse | FRIT improved / worse | Current beats / loses to FRIT |
|---|---:|---:|---:|
| baseline | 12 / 0 | 1 / 11 | 12 / 0 |
| coupled_disturbance | 2 / 4 | 8 / 4 | 4 / 8 |
| delay_300ms | 11 / 1 | 0 / 12 | 12 / 0 |
| four_times_slower | 12 / 0 | 3 / 9 | 12 / 0 |
| nonzero_initial | 12 / 0 | 1 / 11 | 12 / 0 |
| weak_actuator | 12 / 0 | 11 / 1 | 1 / 10 |

The coupled-disturbance condition remains the clearest weakness: current improves only 2, worsens 4 and stays similar in 6. FRIT improves 8 there, but two FRIT neighborhoods cross the evaluation excursion threshold. The current method therefore cannot be described as having solved coupling. In the weak-actuator condition FRIT usually wins narrowly (median current/FRIT ratio 1.018), while current remains modestly better than the collection PID under neighborhood scoring.

## Finite reference pool and failure penalties

The reference pool contains 121 baseline centers plus each case’s collection PID, original PID and both recommendations, deduplicated. Current mean ratio to the shared finite-pool best is 1.457, worst 6.717. These are not ratios to a global optimum. FRIT has two neighborhood threshold penalties (p070, p071); their large sentinel values must not be interpreted as physical billion-fold losses. Among the other 70 cases, FRIT median ratio is 5.063 and mean 9.753. No nominal recommendation crossed the threshold in either method.

## Interpretation and limitations

- Historical FRIT estimated its automatic bandwidth at the lowest available spectral bin in 59/72 records and applied a post-optimization D cap in 34/72. These are observations, not a causal ablation. They suggest checking target-bandwidth matching and post-cap objective mismatch before attributing every loss to FRIT theory itself.
- All historical runs returned a finite recommendation. The historical optimizer reported `stopped`, not a convergence certificate; the original acceptance behavior was retained.
- Six families, two excitation seeds, one original PID and one collection per case are a limited scope. No sensor noise was added; nonlinear behavior and physical coupling remained.
- Nine smooth-SP evaluation tasks span three amplitudes and three speeds. Endpoint state, controller state, input limits, integral handling and disturbance phase are shared. Plant truth is used only for simulation/evaluation.
- Existing production preparation, fixed square excitation, identification and robust gate were used together. These results do not isolate the contribution of the gate.
- No in-game performance or frame-time claim follows from this console experiment. No new global optimality, convergence or robust-stability guarantee is claimed.

See [protocol and reproduction](README.md), [summary](results/summary.json), and [paired case results](results/paired-results.json).
