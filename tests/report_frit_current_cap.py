from pathlib import Path
import json
p=Path('PIDSupporter/tests/output/frit-current-cap-20261025');data=json.loads((p/'results.json').read_text())
lines=['FRIT / current IV / no-response-cap IV: matched Duster BP comparison','',
'Two records: autotune-20261023-162529.jsonl. Frozen recommendations, no BP truth used for recommendations. No mod changes.',
'Current means frozen native IV audit with normalized response cap10, orders1-3, min validation selection, current neighborhood PID search. No-cap changes only that response cap; stable pole constraint retained. Primary record2 uses both records; single-record result also listed.',
'FRIT is the existing reproduction of Git715d44b automatic bandwidth target sweep and D cap. SciPy LM replaces MathNet LM; not binary-identical legacy execution. FRIT used its original full30s calculation; IV uses0-15s training and15-20s validation. This is an operational-method comparison, not an equal training-window estimator comparison.',
'BP physics initial state comes from record endpoint; other-axis commands held at final values. 15s smooth +/- SP, tau1.25s, amplitude max(half excitation, 0.1 training output std), saturation and integral clamp. Same initial controller state for all choices. One-step application delay. No added measurement noise.',
'Metric: nominal tracking MSE normalized by SP amplitude, worse of +/- direction. Robust metric is worst tracking cost over center and existing axis neighbors. This is the established axis-neighbor benchmark, not an exhaustive simultaneous-neighbor robustness certificate.',
'Oracle: best robust center in union of historical ten centers, incumbents, and all method recommendations (including single-record control), with neighborhoods. A finite candidate best, not global plant optimum. New recommendations evaluated directly, not snapped to historical grid.',
'Failure means response leaves20*target-amplitude envelope or nonfinite. Its1e12 penalty is not a physically meaningful regret magnitude.',
'', 'Results:']
for r in data:
 lines.append(f"Record {r['record']}; incumbent={r['incumbent']['pid']}; centers={len(r['centers'])}; best robust={r['best']:.12g}")
 for v in r['methods']:
  lines.append(f"  {v['method']}: PID={v['pid']}; nominal/old={v['nominalVersusIncumbent']:.9g}; robust/old={v['versusIncumbent']:.9g}; robust regret={v['regret']:.9g}; failure={v['failed']}")
lines += ['', 'Primary result:',
'FRIT reproduction worsens both records (2/2); current IV worsens second pooled record (1/2); no-response-cap IV worsens neither (0/2). Both nominal and robust comparisons agree for these primary choices.',
'Record1 current->no cap robust BP cost improves46.8%; record2 pooled improves81.0%. Against incumbent, no-cap nominal tracking MSE improves99.62% and49.00%, respectively. These are tracking-cost improvements, not percentages of every operational performance measure.',
'Current single-record control on record2 nominal cost worsens14.8% but robust cost improves3.4%; pooled current worsens both. Thus pooling and score interpretation matter.',
'No-cap robust regret versus union-best is0.00472% for record1 and0% for record2. This does not demonstrate global optimality: the union includes the recommendations themselves.',
'This supports removing the response cap as a narrow Duster fix, but two records/one conditional BP scenario cannot establish fleet-wide superiority over FRIT or safety in game.',
'FRIT large losses apply to the reproduced legacy implementation, not FRIT theory in general. Its very small final D term is a plausible contributor; this comparison alone does not isolate that causal mechanism.']
(p/'REPORT.txt').write_text('\n'.join(lines),encoding='utf-8');Path('PIDSupporter/tests/frit_current_cap_RESULTS.txt').write_text('\n'.join(lines),encoding='utf-8');print('\n'.join(lines[-10:]))
