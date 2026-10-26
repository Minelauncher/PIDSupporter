from production_noiseless_common import *
ROOT=BASE/'output/production-noiseless-20261026';cases=read(ROOT/'cases-evaluation-only.json');rows=[read(ROOT/'graded'/(c['id']+'.json')) for c in cases]
manifest=read(ROOT/'frozen.json');errors=[]
for c in cases:
 path=ROOT/'runs'/(c['id']+'.json');assert hashlib.sha256(path.read_bytes()).hexdigest()==manifest[path.name];run=read(path);r=run['record']
 if not r:continue
 f=np.float32;kp,ti,td=map(f,r['Pid']);inte=f(r['Integrals'][0]);prev=f(r['R'][0]-r['Y'][0]);maxerr=0
 for k in range(1,len(r['U'])):
  h=f(r['T'][k]-r['T'][k-1]);e=f(r['R'][k]-r['Y'][k]);acc=f(inte+e*h)
  if f(acc/ti*kp)>1:acc=f(ti/kp)
  elif f(acc/ti*kp)<-1:acc=f(-ti/kp)
  if ti>=250 or h>.5:acc=f(0)
  u=float(np.clip(f(kp*f(e+acc/ti+f((e-prev)/h)*td)),-1,1));maxerr=max(maxerr,abs(u-r['U'][k]));inte=acc;prev=e
 assert maxerr<1e-6,(c['id'],maxerr)
 assert len(r['U'])==600 and max(abs(np.array(r['U'])))<=1
 errors.append(maxerr)
def summarize(rs):
 rel=np.array([r['operationVsWorking'] for r in rs]);loss=np.array([r['operationRatio'] for r in rs]);return dict(cases=len(rs),recommended=sum(r['accepted'] for r in rs),improved=int(sum(rel<.99)),worse=int(sum(rel>1.01)),withinOnePercent=int(sum((rel>=.99)&(rel<=1.01))),medianVsWorking=float(np.median(rel)),worstVsWorking=float(rel.max()),medianBestRatio=float(np.median(loss)),meanBestRatio=float(loss.mean()),worstBestRatio=float(loss.max()),betterThanNoAction=sum(r['operationNominalVsNoAction']<.99 for r in rs),newFailureCases=sum(r['operationFailureTasks']>r['workingFailureTasks'] for r in rs),matchedWorse=sum(r['matchedVsWorking']>1.01 for r in rs),matchedMedianVsWorking=float(np.median([r['matchedVsWorking'] for r in rs])))
summary=dict(all=summarize(rows),conditions={c:summarize([r for r in rows if r['condition']==c]) for c in dict.fromkeys(r['condition'] for r in rows)},fitSeconds=dict(median=float(np.median([r['fitSeconds'] for r in rows if r['fitSeconds'] is not None]))),preparationSeconds=dict(min=min(r['preparationSeconds'] for r in rows),max=max(r['preparationSeconds'] for r in rows)),maxRecordedPidReplayError=max(errors),exactSourceHashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in (BASE.parent/'PIDSupporterCore').glob('*.cs') if f.name in ['NativeAutoTune.cs','NativeLinear.cs','NativeLinearAccumulation.cs','NativeLinearConstraints.cs','NumericalSvd.cs','AutoTuneSession.cs','AdaptiveExcitation.cs','PidSearch.cs','EmpiricalFrequencyGate.cs','AutoTuneHistory.cs']})
(ROOT/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
bad=[r for r in rows if r['operationVsWorking']>1.01]
lines=['Current production pipeline, noise-free synthetic operating envelope — 2026-10-26',json.dumps(read(ROOT/'protocol.json'),indent=2),json.dumps(summary,indent=2),
'Unmodified production source is linked into the harness. Only game/Unity host objects are substituted. Production seed field is set for deterministic excitation. Actual preparation,30s collection,independent carrier,1..3-order IV selection,empirical frequency gate,DE and27-point hierarchy all execute. No research1SE,refined IV,separate delay,residual-radius or set-membership code substitutes the algorithm.',
'Truth coefficients are confined to the synthetic measurement generator and grading code. The production fitter receives only the copied TuneRecord. Choices are hashed before grading. All72 recordings have600 post-limit samples and independent float PID replay agrees within1e-6.',
'Performance primary is same endpoint state,integral,error,queue and phase as collected record. Each candidate sees the same endpoint. Matched task is the mod smooth SP with tau1.25 and current delta; operational score averages9 amplitude/speed conditions, each taking worst sign, with worst27-neighbor cost. This broader score is deliberately not the same objective as the production two-sign test.',
'Reference best is finite121-center legacy grid extended by current/original/recommended PID, with joint neighborhoods. Ratio1 means best among that finite pool; it is not continuous global optimality. Zero-action comparison uses nominal PID, not neighborhood worst case, and retains natural dynamics/disturbances.',
'Noisy sensors are absent, but physical disturbances, nonlinearity, changing input effectiveness between cases and latent coupling remain. One original PID(.4,2.5,.2),two excitation seeds per condition,six plant families. One-shot per fresh window; accumulated multi-run behavior is not tested here.',
'Bad relative-to-working cases:',json.dumps(bad,indent=2),
'Tuning runtime is a headless host measurement, not a game frame-time benchmark. Production safety/stability guarantees are not established. No algorithm changes were made in response to results. Documentation-only1.1.4 preserves1.1.3 DLLs.']
(ROOT/'report.txt').write_text('\n\n'.join(lines),encoding='utf-8')
