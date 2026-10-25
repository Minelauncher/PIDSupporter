from duster_shared_disturbance import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fits=json.loads((OUT/'fits.json').read_text());grades=json.loads((OUT/'grades.json').read_text());sens=json.loads((OUT/'sensitivity.json').read_text())
for group,pool in zip(fits,[[R[0]],[R[1]],R]):
 for m in group['variants']:
  assert max(abs(np.roots(m['den'])))<1
  for j,r in enumerate(pool):
   yf,_=predict(r,m,j);yc,uc=predict(r,m,j,True);np.testing.assert_allclose(yf[:160],yc[:160],atol=1e-6,rtol=1e-6);np.testing.assert_allclose(uc[:160],r['U'][:160])
selected=fits[2]['variants'][fits[2]['selected']]
lines=['Duster two-record common plant / per-record disturbance experiment','',
'Protocol: each record 0-15 s training, 15-20 s selection, 20-30 s untouched final. No measurement noise added. No mod changes.',
'Model: y_j = (B/A)u_j + homogeneous initial transient_j + c_j + slope_j*t. A and B shared; initial state and nuisance terms separate. Constant-only nuisance also tested.',
'Estimator: separable output-error least squares (variable projection), NOT IV or refined BJ. Poles optimized, numerator and nuisance terms solved jointly by normalized least squares. Thus no closed-loop unbiasedness guarantee is claimed.',
'Orders1-3: stable real pole for order1, stable conjugate pair for order2, conjugate pair plus positive real pole for order3. Decay bounds .05..300 s (.025..300 for third pole); frequency .005..5 Hz. No response cap, no ridge. This restricted pole family does not cover all stable order3 systems.',
'Multi-start training-cost minimum per structure; model/constant-vs-affine chosen by minimum worst freeY/closedY/closedU normalized validation RMSE across records. Not 1-SE. Same scoring for separate fits.',
'Scales from first15s of each record. Known measured inputs used for free rollout. Closed replay uses measured u for first4s, then recorded SP and existing saturated PID; no output corrections. Nuisance terms extrapolated from training only.',
'PID comparison: same existing ten centers per record, same neighbors, smooth +/- targets, saturation/integral handling and recent output histories. No DE expansion in this diagnostic. Future nuisance trend extrapolated; hold-at-end sensitivity also tested.',
'BP reference is a fresh evaluation of the same historical candidate locations, held other-axis commands; not new game truth. One-step command application matches the current model evaluation. No BP costs enter training/selection. Fits frozen by hash before grading.',
'',f"Selected shared model: order {selected['order']}, {selected['kind']}, pole parameters decay_s/frequency_Hz/third_decay_s {selected['poleParameters']}",
'', 'Results (final normalized RMSE: freeY, closedY, closedU; BP regret relative to best tested center):']
for group in grades:
 lines.append(f"Record {group['record']}")
 for row in group['rows']:
  lines.append(f"  {row['label']}: order={row['order']}, final={row['final']}, PID={row['pid']}, BPregret={row['bpRegret']:.6f}, rank={row['rankCorrelation']:.6f}")
lines += ['', 'Pole sensitivity (training record weights changed, final data not used):']
for s in sens:lines.append(f"weights={s['weights']}: {s['model']['poleParameters']}")
lines += ['', 'Interpretation:',
'Shared fitting improves second-record free rollout over ordinary no-cap pooled IV: 1.76064 -> 0.89577 (49.1%). Separate nuisance-only refit with frozen old pooled plant already reaches 1.11265; plant re-estimation adds about19.5% improvement over that control.',
'It does NOT improve every metric: second-record closed Y/U errors worsen from 0.00120/0.02006 to 0.02370/0.11952. First-record closed replay likewise worsens versus no-cap IV. The better free rollout is not enough to claim a better control model.',
'Against the separately fitted output-error models, shared free error changes record1 .27611 -> .22737 and record2 4.02796 -> .89577. See matched BP regrets above; historical two-step BP grades are not used in final results.',
'Against original no-cap IV, the recommended candidate is unchanged in both records. Thus the shared approach does not improve BP selection on this fixed candidate set. No claim is made about unrestricted DE optimization.',
'Shared poles are identical across records by construction; that alone is not evidence of stability. Reweighting records by a factor2 keeps frequency near .092-.096Hz but decay changes 2.17-4.51s and third decay .38-1.09s. Frequency is more repeatable, exact pole is still uncertain.',
'The chosen slow decay3.22s/frequency.09175Hz differs substantially from prior no-cap pooled decay11.87s/frequency.08147Hz and single-record2 decay11.29s/frequency.03390Hz. Allocation between nuisance and plant changes the fitted poles materially.',
'Conclusion: separate disturbance helps long-horizon recorded-input prediction but does not establish better PID choice or uniquely identify real slow poles. Two records and a simple affine nuisance model do not fully resolve closed-loop coupling and time-varying operation.',
'Checks: all fitted poles stable; free and closed implementation agree for first4s with measured input; final segment not used for fitting or selection; future other-axis observations unused.']
(OUT/'REPORT.txt').write_text('\n'.join(lines),encoding='utf-8')
fig,axs=plt.subplots(2,1,figsize=(11,7),constrained_layout=True)
tr=np.load(OUT/'traces.npz')
for i,ax in enumerate(axs):
 t=np.arange(1200)*DT;ax.plot(t,R[i]['Y'],color='black',label='Observed')
 for label in ['separate_disturbance','shared_disturbance',('single' if i==0 else 'pooled')+'_nocap_nuisance_only']:ax.plot(t,tr[f'{i+1}_{label}'],label=label)
 ax.axvline(15,color='gray',linestyle=':');ax.axvspan(20,30,color='gray',alpha=.15);ax.set(title=f'Record {i+1}: free rollout, no output correction',xlabel='Seconds',ylabel='Roll (deg)');ax.legend(fontsize=8)
fig.savefig(OUT/'comparison.png',dpi=150);plt.close(fig)
print('Checks passed; selected',selected['order'],selected['kind'],selected['poleParameters'])
