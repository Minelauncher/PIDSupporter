from pathlib import Path
import sys,json,numpy as np,time
sys.path.insert(0,str(Path('PIDSupporter/tests').resolve()))
# Existing corrected BP physics, used only after fitted models and recommendations are frozen.
code=Path('PIDSupporter/tests/duster_candidate_delayed.py').read_text();exec(code.split('results=[];')[0],globals())
outroot=Path('PIDSupporter/tests/output/response-cap-crosscheck-20261025');fits=[json.loads(l) for l in (outroot/'fits.jsonl').read_text().splitlines()];old=json.loads((ROOT/'duster-candidate-landscape-delayed.json').read_text());output=[];start=time.perf_counter()
for number in [1,2]:
 r=records[number-1];ctx=dict(np.load(ROOT/f'duster-candidate-context-{number}.npz'));app=ctx['app'];last=app[-1];q=Rotation.from_quat(last[17:21]);w=(last[14:17]+H*q.apply(q.inv().apply(last[8:11])/I))*(1-H*last[29]);q0=Rotation.from_rotvec(H*w)*q;v0=last[11:14]+H*last[5:8]/mass;w0=w
 selected=[m for m in fits if m['id']==f'duster_old_{number}'];centers=[tuple(m['result']['pid']) for m in selected];allp=sorted(set(p for c in centers for p in neighbors(c)));cache={tuple(v['pid']):v['bp'] for v in old[number-1]['points']}
 cachefile=outroot/f'bp-new-points-{number}.json'
 if cachefile.exists():cache.update({tuple(v['pid']):v['bp'] for v in json.loads(cachefile.read_text())})
 def measure(p):
  delta=max(.5*r['Amplitude'],.1*r['ys']);base=r['Y'][-1];bias=np.mean(r['U'][-40:]);metrics=[]
  for sign in [-1,1]:
   rot=q0;v=v0.copy();w=w0.copy();lastu=r['U'][-1];inte=r['InitialIntegral'];prev=r['InitialError'];tr=sl=ef=0;failed=False
   for k in range(600):
    rot,v,w=physics(rot,v,w,lastu,ctx);yp=-rot.as_euler('zxy',degrees=True)[0];target=base+sign*delta*(1-np.exp(-k*H/1.25));error=target-yp;uv,inte=pidstep(p,error,prev,inte,H);prev=error
    if not np.isfinite(yp) or abs(yp-base)>20*delta:tr=600e12;failed=True;break
    tr+=(error/delta)**2;sl+=((uv-lastu)/(2*H))**2;ef+=((uv-bias)/2)**2;lastu=uv
   metrics.append([tr/600,sl/600,ef/600,float(failed)])
  return dict(zip(['tracking','slew','effort','failed'],map(float,np.max(metrics,axis=0))))
 for i,p in enumerate(allp):
  if p not in cache:cache[p]=measure(p);cachefile.write_text(json.dumps([dict(pid=list(k),bp=v) for k,v in cache.items()],indent=2))
  if i%5==0:print('BP',number,i+1,len(allp),'seconds',round(time.perf_counter()-start),flush=True)
 best=min([v['bp']['robust'] for v in old[number-1]['centers']]+[max(cache[q]['tracking'] for q in neighbors(p)) for p in centers])
 for m in selected:
  p=tuple(m['result']['pid']);cost=max(cache[q]['tracking'] for q in neighbors(p));output.append(dict(id=m['id'],cumulative=m['cumulative'],cap=m['cap'],pid=p,robust=cost,bestUnion=best,regret=cost/best-1,nominal=cache[p]['tracking'],failed=bool(cache[p].get('failed',False))))
 (outroot/'new-de-bp-grades.json').write_text(json.dumps(output,indent=2));print('RUN_DONE',number,output[-len(selected):],flush=True)
