from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('PIDSupporter/tests/output/delay-ablation-20261025');dec=json.loads((p/'decisions.json').read_text());search=[json.loads(l) for l in (p/'search.jsonl').read_text().splitlines()];bp=json.loads((p/'duster-bp.json').read_text());proxy=json.loads((p/'spirit-holdout-proxy.json').read_text());logged=json.loads((p/'logged.json').read_text());original=[v['value'] for v in logged if v['kind']=='pidSearch'];fits=[json.loads(l) for l in (p/'fits.jsonl').read_text().splitlines()]
for i in range(4):
 base=next(x for x in search if x['record']==i and x['stage']=='baseline');np.testing.assert_allclose(base['result']['pid'],original[i]['Pid'],atol=1e-12)
assert len(fits)==72 and len(search)==16
lines=['Delay / frequency validation / model-consensus ablation','', (p/'protocol.txt').read_text(),
'Stage2 uses frequency validation as the primary selection score, with original time score only as tie-break; this is NOT a joint time+frequency Pareto guarantee. All time scores remain reported. Each order/delay retains its native time-selected pole structure. Stage3 retains models within10% of frequency minimum, not statistically calibrated confidence sets.',
'Train spectra use0-15s Welch200-sample windows, 50%overlap; validation15-20s is only one5s window. Final20-30s is kept out of selection. Frequency score is max across pooled records of RMS complex relative transfer mismatch at eligible odd harmonics0.6-3.8Hz. Effective S_zy/S_zu may contain excitation-correlated coupling; it is not isolated plant truth.',
'Native C# grid+DE+local search, slider quantization, saturation/integral rules, axis-neighborhood robust hierarchical cost retained. Stage3 maximizes cost over models and existing PID neighbors. Baseline reproduces all four logged recommended PIDs exactly.',
'', 'Selections and final diagnostics:']
for i in range(4):
 lines.append(('Duster' if i<2 else 'Spirit')+f' record{i%2+1}')
 for s in [v for v in search if v['record']==i]:
  stage=s['stage'];detail=dec[i]['stages'].get(stage);lines.append(f" {stage}: PID={s['result']['pid']}, models={s['models']}, seconds={s['result']['seconds']:.3f}, selected={detail if detail else dec[i]['consensus']}")
lines += ['', 'Duster evaluation-only BP at latest observed endpoint; fresh delivered-command context reconstructed from latest physics logs. Other commands held fixed; corrected stock BP and measured propulsion efficiency from previous work retained. Conditional simulator, not new flight truth.']
for b in bp:
 base=next(v for v in bp if v['record']==b['record'] and v['stage']=='baseline');lines.append(f" Duster{b['record']+1} {b['stage']}: robust={b['robust']:.9g}, ratio to original recommended={b['robust']/base['robust']:.6f}, ratio to collection PID={b['versusIncumbent']:.6f}")
lines += ['', 'Spirit final-segment stability PROXY only: scalar gain+ideal-delay frequency correction fitted to heldout20-30s, after choices frozen. Integer+linear-interpolated fractional delay used for feedback poles. This is not an independently validated Spirit BP or real-flight test, so no actual PID regret is claimed.']
for s in proxy:lines.append(json.dumps(s))
lines += ['', 'Findings:',
'All three modifications avoid P~1 on both Spirit records: delay-only P=.251/.680; frequency-primary P=.363/.481; consensus P=.363/.615. Smaller P alone is not proof of better control.',
'Delay-only selected150ms (upper search boundary) in both Spirit runs. Final fast-band error remains .744/.901; second is worse than baseline .637. Thus merely adding delay candidates can fit time error without finding credible fast dynamics.',
'Frequency-primary selected order3/delay50ms for Spirit, final frequency error .857->.130 and .637->.402. However free rollout worsens .880->3.551 and1.356->8.592. Closed-output final error improves .275->.195 and.185->.127. This is a real objective tradeoff, not all-around model recovery.',
'Consensus has only1 retained model on Spirit1/Duster1, so those results do not test multi-model benefits. Spirit2 retains order3/delay50ms andorder2/delay75ms; changes PID to.615,43.4,.3. No demonstrated actual superiority over single frequency model; two-model agreement is not plant uncertainty coverage.',
'Duster2 recommended PID remains .162,1,.2 in every stage. Duster1 changes from.183,1.2,.2 to.114,.4,.4 (delay) or.095,.4,.5 (frequency/consensus). Matched BP robust cost increases35.2%/49.0% relative to original recommendation, although all remain >99% better than the collection incumbent on this smooth-SP benchmark.',
'Spirit final-frequency proxy places original recommendations outside linear stability boundary (radii1.091/1.023), all three new recommendations inside. This supports the mechanism but does not prove actual gameplay recovery.',
'Conclusion: frequency validation helps detect the Spirit fast-dynamics mismatch; unconditional frequency-first model selection loses some Duster performance and worsens Spirit free rollout. Ensemble adds no demonstrated practical benefit yet. No runtime, installed DLL, or Workshop changes.',
'Validation limitations: only four records/two crafts, short spectral windows, a fixed .6-3.8Hz diagnostic band, chosen10% ensemble tolerance, and finite delay search. No universal convergence/optimality/safety claim.']
text='\n'.join(lines);(p/'REPORT.txt').write_text(text,encoding='utf-8');Path('PIDSupporter/tests/delay_ablation_RESULTS.txt').write_text(text,encoding='utf-8')
fig,axes=plt.subplots(1,2,figsize=(11,4),constrained_layout=True);stages=['baseline','delay_only','frequency_validation','consensus'];labels=['Current','Delay','Frequency','Consensus']
for i in [2,3]:axes[0].plot(labels,[next(v for v in search if v['record']==i and v['stage']==s)['result']['pid'][0] for s in stages],marker='o',label=f'Spirit {i-1}')
axes[0].set(ylabel='Recommended P',title='Recommendations, not measured performance');axes[0].legend()
for i in [0,1]:
 vals=[next(v for v in bp if v['record']==i and v['stage']==s)['robust'] for s in stages];axes[1].plot(labels,np.array(vals)/vals[0],marker='o',label=f'Duster {i+1}')
axes[1].set(ylabel='BP cost / current recommendation cost',title='Conditional Duster BP: lower is better');axes[1].legend();fig.savefig(p/'comparison.png',dpi=150);plt.close(fig)
print('PASS: 72 fits, 16 searches; all four baseline PIDs match game log. Report saved.')
