"""Fresh BP candidate costs with same one-step command application as model evaluator."""
from pathlib import Path
import sys,json,numpy as np,time
sys.path.insert(0,str(Path('PIDSupporter/tests').resolve()))
code=Path('PIDSupporter/tests/duster_candidate_delayed.py').read_text();exec(code.split('results=[];')[0],globals())
dest=Path('PIDSupporter/tests/output/duster-shared-disturbance-20261025');old=json.loads((ROOT/'duster-candidate-landscape-delayed.json').read_text());start=time.perf_counter()
for i,r in enumerate(records):
 ctx=dict(np.load(ROOT/f'duster-candidate-context-{i+1}.npz'));last=ctx['app'][-1];q=Rotation.from_quat(last[17:21]);w=(last[14:17]+H*q.apply(q.inv().apply(last[8:11])/I))*(1-H*last[29]);q0=Rotation.from_rotvec(H*w)*q;v0=last[11:14]+H*last[5:8]/mass;w0=w
 file=dest/f'bp-matched-{i+1}.json';cache={tuple(v['pid']):v['bp'] for v in json.loads(file.read_text())} if file.exists() else {}
 for j,point in enumerate(old[i]['points']):
  p=tuple(point['pid'])
  if p in cache:continue
  delta=max(.5*r['Amplitude'],.1*r['ys']);base=r['Y'][-1];bias=np.mean(r['U'][-40:]);metrics=[]
  for sign in [-1,1]:
   rot=q0;v=v0.copy();w=w0.copy();lastu=r['U'][-1];inte=r['InitialIntegral'];prev=r['InitialError'];tr=sl=ef=0;failed=False
   for k in range(600):
    rot,v,w=physics(rot,v,w,lastu,ctx);yp=-rot.as_euler('zxy',degrees=True)[0];target=base+sign*delta*(1-np.exp(-k*H/1.25));error=target-yp;uv,inte=pidstep(p,error,prev,inte,H);prev=error
    if not np.isfinite(yp) or abs(yp-base)>20*delta:tr=600e12;failed=True;break
    tr+=(error/delta)**2;sl+=((uv-lastu)/(2*H))**2;ef+=((uv-bias)/2)**2;lastu=uv
   metrics.append([tr/600,sl/600,ef/600,float(failed)])
  cache[p]=dict(zip(['tracking','slew','effort','failed'],map(float,np.max(metrics,axis=0))));file.write_text(json.dumps([dict(pid=list(p),bp=v) for p,v in cache.items()],indent=2))
  if j%10==0:print('BP',i+1,j+1,'/',len(old[i]['points']),'elapsed',round(time.perf_counter()-start),flush=True)
