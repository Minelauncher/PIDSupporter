from pathlib import Path
import json,numpy as np
from scipy.stats import spearmanr
root=Path('PIDSupporter/tests/output/response-cap-crosscheck-20261025');fits=[json.loads(l) for l in (root/'fits.jsonl').read_text().splitlines()]
bprows=json.loads(Path('PIDSupporter/tests/output/bp-replay/duster-candidate-landscape-delayed.json').read_text())
def measure(r,m,points):
 p=np.asarray(points,dtype=np.float32);n=len(p);o=m['expandedOrder'];a=np.array(m['c']);dt=r['dt'];delta=max(.5*r['Amplitude'],.1*r['ys']);base=r['Y'][-1];bias=np.mean(r['U'][-round(1/dt):]);ret=[]
 for sign in [-1,1]:
  yh=np.tile((np.array(r['Y'][-o:][::-1])-m['ym'])/m['ys'],(n,1));uh=np.tile((np.array(r['U'][-o:][::-1])-m['um'])/m['us'],(n,1));inte=np.full(n,r['InitialIntegral']);prev=np.full(n,r['InitialError']);last=np.full(n,r['U'][-1]);tr=np.zeros(n);sl=tr.copy();ef=tr.copy();failed=np.zeros(n,bool)
  for k in range(round(15/dt)):
   yn=yh@a[:o]+uh@a[o:2*o]+a[-1];yy=m['ym']+m['ys']*yn;target=base+sign*delta*(1-np.exp(-k*dt/1.25));e=(target-yy).astype(np.float32);inte=(inte.astype(np.float32)+e*np.float32(dt)).astype(np.float32);inte=np.clip(inte,-p[:,1]/p[:,0],p[:,1]/p[:,0]);inte=np.where(p[:,1]>=250,0,inte).astype(np.float32);uv=np.clip(p[:,0]*(e+inte/p[:,1]+p[:,2]*(e-prev.astype(np.float32))/np.float32(dt)),-1,1);prev=e
   failed|=(abs(yy-base)>20*delta)|~np.isfinite(yy);tr+=(e/delta)**2;sl+=((uv-last)/(2*dt))**2;ef+=((uv-bias)/2)**2;last=uv;yn=np.clip(np.nan_to_num(yn),-1e10,1e10);yh=np.column_stack([yn,yh[:,:-1]]);uh=np.column_stack([(uv-m['um'])/m['us'],uh[:,:-1]])
  tr/=round(15/dt);tr[failed]=1e12;ret.append(np.column_stack([tr,sl/round(15/dt),ef/round(15/dt)]))
 return np.max(ret,axis=0)
def neighbors(p):
 arr=[tuple(p)]
 for j in range(3):
  if (j==1 and p[j]>=250) or (j==2 and p[j]==0):continue
  for sign in [-1,1]:
   q=list(p);q[j]=round(np.clip(q[j]+sign*max([.001,.1,.1][j],q[j]*.1),[.001,.1,0][j],[1,249.9,100][j]),3 if j==0 else 1);arr.append(tuple(q))
 return list(dict.fromkeys(arr))
def choose(s):
 ids=np.where(s[:,0]<=s[:,0].min()*1.01+1e-12)[0];ids=ids[s[ids,1]<=s[ids,1].min()*1.01+1e-12];return min(ids,key=lambda i:(s[i,2],s[i,3]))
out=[]
for m in fits:
 if not m['id'].startswith('duster_old') or 'failed' in m:continue
 run=int(m['id'][-1]);ref=bprows[run-1];r=json.loads((root/'records'/f'{m["id"]}.json').read_text());points=[v['pid'] for v in ref['points']];cost=measure(r,m,points);ix={tuple(p):i for i,p in enumerate(points)};scores=[]
 for center in ref['centers']:
  pid=center['pid'];indices=[ix[q] for q in neighbors(pid)];scores.append([cost[indices,0].max(),*cost[ix[tuple(pid)]]])
 scores=np.array(scores);pick=int(choose(scores));truth=np.array([[v['bp']['robust'],v['bp']['tracking'],v['bp']['slew'],v['bp']['effort']] for v in ref['centers']]);oracle=int(choose(truth));best=truth[:,0].min();row=dict(id=m['id'],cumulative=m['cumulative'],cap=m['cap'],pid=ref['centers'][pick]['pid'],regret=float(truth[pick,0]/best-1),bpCost=float(truth[pick,0]),bpBest=float(best),spearman=float(spearmanr(scores[:,0],truth[:,0]).statistic),candidateCount=len(scores),note='Selection among frozen existing BP-evaluated candidates; not fresh game truth or evaluation of unconstrained DE recommendation.');out.append(row);print(row,flush=True)
(root/'duster-bp-grades.json').write_text(json.dumps(out,indent=2))
real=[v for v in fits if not v['id'].startswith('synt_')]
for v in real:
 print(v['id'],'pool',v['cumulative'],'cap',v['cap'],'final',v.get('FinalFree'),v.get('FinalY'),v.get('FinalU'),'gain',v.get('gain'),'pid',v.get('result',{}).get('pid'))
