from duster_shared_disturbance import *
from scipy.stats import spearmanr
assert json.loads((OUT/'frozen.json').read_text())['sha256']==hashlib.sha256((OUT/'fits.json').read_bytes()).hexdigest()
source=Path('PIDSupporter/tests/grade_cap_real.py').read_text();tree=ast.parse(source)
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef)],type_ignores=[]),'candidate helpers','exec'),globals())
def candidate(r,m,j,points,hold=False):
 p=np.asarray(points,dtype=np.float32);N=len(p);n=m['order'];a=-np.array(m['den'][1:]);b=np.array(m['b']);nuis=np.array(m['nuisance'][j][n:]);dt=DT
 def disturbance(k):return nuis[0]+(nuis[1]*k*dt/15 if len(nuis)>1 else 0)
 delta=max(.5*r['Amplitude'],.1*r['ys']);base=r['Y'][-1];bias=np.mean(r['U'][-40:]);ret=[]
 for sign in [-1,1]:
  yh=np.tile([r['Y'][-1-k]-disturbance(1199-k) for k in range(n)],(N,1));uh=np.tile(r['U'][-n:][::-1],(N,1));inte=np.full(N,r['InitialIntegral']);prev=np.full(N,r['InitialError']);last=np.full(N,r['U'][-1]);tr=np.zeros(N);sl=tr.copy();ef=tr.copy();failed=np.zeros(N,bool)
  for k in range(600):
   yp=yh@a+uh@b;yy=yp+disturbance(1199 if hold else 1200+k);target=base+sign*delta*(1-np.exp(-k*dt/1.25));e=(target-yy).astype(np.float32);inte=(inte.astype(np.float32)+e*np.float32(dt)).astype(np.float32);inte=np.clip(inte,-p[:,1]/p[:,0],p[:,1]/p[:,0]);inte=np.where(p[:,1]>=250,0,inte).astype(np.float32);uv=np.clip(p[:,0]*(e+inte/p[:,1]+p[:,2]*(e-prev.astype(np.float32))/np.float32(dt)),-1,1);prev=e
   failed|=(abs(yy-base)>20*delta)|~np.isfinite(yy);tr+=(e/delta)**2;sl+=((uv-last)/(2*dt))**2;ef+=((uv-bias)/2)**2;last=uv;yp=np.clip(np.nan_to_num(yp),-1e10,1e10);yh=np.column_stack([yp,yh[:,:-1]]);uh=np.column_stack([uv,uh[:,:-1]])
  tr/=600;tr[failed]=1e12;ret.append(np.column_stack([tr,sl/600,ef/600]))
 return np.max(ret,axis=0)
def fixed_nuisance(r,old,kind):
 n=old['expandedOrder'];a=np.array(old['c'][:n]);b=np.array(old['c'][n:2*n])*old['ys']/old['us'];den=np.r_[1,-a];U,V=design(r,den,kind,600);target=np.array(r['Y'][:600])-U@b;s=np.maximum(np.linalg.norm(V,axis=0),1e-15);z=np.linalg.lstsq(V/s,target,rcond=None)[0]/s
 return dict(order=n,den=den.tolist(),b=b.tolist(),kind=kind,nuisance=[z.tolist()])
fits=json.loads((OUT/'fits.json').read_text());old=[m for m in map(json.loads,(BASE/'fits.jsonl').read_text().splitlines()) if m['id'].startswith('duster_old')];bp=json.loads(Path('PIDSupporter/tests/output/bp-replay/duster-candidate-landscape-delayed.json').read_text());output=[];traces={}
for i,r in enumerate(R):
 ref=bp[i]
 matched=OUT/f'bp-matched-{i+1}.json'
 if matched.exists():
  cache={tuple(v['pid']):v['bp'] for v in json.loads(matched.read_text())}
  assert all(tuple(v['pid']) in cache for v in ref['points']), 'BP grading incomplete'
  for point in ref['points']:point['bp']=cache[tuple(point['pid'])]
  for center in ref['centers']:
   val=dict(cache[tuple(center['pid'])]);val['robust']=max(cache[q]['tracking'] for q in neighbors(center['pid']));center['bp']=val
 points=[v['pid'] for v in ref['points']];ix={tuple(p):k for k,p in enumerate(points)};variants=[]
 for label,f,j in [('separate_disturbance',fits[i],0),('shared_disturbance',fits[2],i)]:
  m=f['variants'][f['selected']];variants.append((label,m,j,None))
  if label=='shared_disturbance':variants.append((label+'_hold_future_disturbance',m,j,True))
 for m in old:
  if m['id']!=f'duster_old_{i+1}':continue
  label=('pooled' if m['cumulative'] else 'single')+('_cap' if m['cap'] else '_nocap')
  variants.append((label,None,0,m))
  if not m['cap']:
   # Fixed old dynamics + nuisance control separates refitting plant from refitting disturbance.
   nm=fixed_nuisance(r,m,'affine');nm['evaluation']=evaluate([r],nm);variants.append((label+'_nuisance_only',nm,0,None))
 rows=[]
 for label,m,j,extra in variants:
  cost=measure(r,extra,points) if m is None else candidate(r,m,j,points,extra is True)
  scores=np.array([[cost[[ix[q] for q in neighbors(c['pid'])],0].max(),*cost[ix[tuple(c['pid'])]]] for c in ref['centers']]);pick=int(choose(scores));truth=np.array([c['bp']['robust'] for c in ref['centers']]);selected=ref['centers'][pick]
  row=dict(label=label,order=extra['order'] if m is None else m['order'],pid=selected['pid'],bpCost=float(truth[pick]),bpRegret=float(truth[pick]/min(truth)-1),rankCorrelation=float(spearmanr(scores[:,0],truth).statistic),predictedCost=float(scores[pick,0]),final=[extra['FinalFree'],extra['FinalY'],extra['FinalU']] if m is None else m['evaluation'][j]['final'])
  if m is not None:
   row['poles']=[[float(z.real),float(z.imag)] for z in np.roots(m['den'])];row['gain']=float(sum(m['b'])/sum(m['den']));row['disturbanceCoefficients']=m['nuisance'][j][m['order']:];traces[f'{i+1}_{label}']=m['evaluation'][j]['free']
  rows.append(row);print(i+1,label,'PID',row['pid'],'regret',round(row['bpRegret'],4),'final',row['final'],flush=True)
 output.append(dict(record=i+1,candidateCount=len(ref['centers']),rows=rows))
(OUT/'grades.json').write_text(json.dumps(output,indent=2));np.savez(OUT/'traces.npz',**traces)
