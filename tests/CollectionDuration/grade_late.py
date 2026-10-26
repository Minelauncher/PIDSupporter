import sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from production_noiseless_common import *
def grade_all(case,run,extra_centers=()):
 spec=case['plant'];r=run.get('record');endpoint=run['endpoint'];working=run['workingPid'];result=run.get('result');picked=result['pid'] if result and result['accepted'] else working
 centers=read(SRC/'grid.json')['centers']+[working,case['originalPid'],picked]+list(extra_centers);centers=list(dict.fromkeys(tuple(p) for p in centers));points=sorted({p for c in centers for p in neighbors(c)});ix={p:i for i,p in enumerate(points)};groups=[[ix[p] for p in neighbors(c)] for c in centers];ps=np.array(points+[(0.,250.,0.)]);B=len(ps)
 delta=max(.5*run['amplitude'],.1*float(np.std(r['Y'][:len(r['Y'])//2]))) if r else .3;base=r['Y'][-1] if r else case['baseSP'];tasks=[('matched',delta,1.25)]+[('operation',amp,tau) for amp in [.1,.3,.6] for tau in [.5,1.25,2.]];allcost=[];allpeak=[];allsat=[]
 for kind,amp,tau in tasks:
  costs=[];peaks=[];sats=[]
  for sign in [-1,1]:
   x=np.tile(endpoint['X'],(B,1));q=np.tile(endpoint['Q'],(B,1));cross=np.full(B,endpoint['Cross']);last=np.full(B,endpoint['lastU']);inte=np.full(B,endpoint['integral']);prev=np.full(B,endpoint['error']);total=np.zeros(B);peak=np.zeros(B);sat=np.zeros(B);failed=np.zeros(B,dtype=bool)
   for k in range(300):
    q=np.column_stack([q[:,1:],last]);x,yy=advance(spec,x,q[:,0]*case['gain']);x=np.clip(np.nan_to_num(x,nan=1e8,posinf=1e8,neginf=-1e8),-1e8,1e8)
    t=endpoint['Time']+(k+1)*.05
    if case['coupled']:cross+=.05*(-.7*cross+.4*yy+.8*np.sin(2*np.pi*.11*t)+.4*np.sin(2*np.pi*.23*t));yy=yy-.8*cross
    target=base+sign*amp*(1-np.exp(-k*.05/tau));failed|=~np.isfinite(yy)|(abs(yy-base)>20*amp);yy=np.clip(np.nan_to_num(yy,nan=1e8,posinf=1e8,neginf=-1e8),-1e8,1e8);error=target-yy
    # Same float PID order and contribution clamp as NativeAutoTune.PidStep.
    p=ps.astype(np.float32);e=error.astype(np.float32);h=np.float32(.05);acc=inte.astype(np.float32)+e*h;ti=p[:,1];kp=p[:,0];limit=np.divide(ti,kp,out=np.full(B,np.inf,dtype=np.float32),where=kp!=0);acc=np.clip(acc,-limit,limit);acc=np.where(ti>=250,0,acc).astype(np.float32);u=np.clip(kp*(e+acc/ti+((e-prev.astype(np.float32))/h)*p[:,2]),-1,1);u[-1]=0;inte=acc;prev=e;last=u;total+=((target-yy)/amp)**2/300;peak=np.maximum(peak,abs(yy-base));sat+=(abs(u)>=.99999)/300
   total[failed]=1e12;costs.append(total);peaks.append(peak);sats.append(sat)
  allcost.append(np.max(costs,axis=0));allpeak.append(np.max(peaks,axis=0));allsat.append(np.max(sats,axis=0))
 cc=np.array(allcost);ss=np.array(allsat);idx=centers.index(tuple(picked));wi=centers.index(tuple(working));oi=centers.index(tuple(case['originalPid']));matched=np.array([cc[0,g].max() for g in groups]);oper=np.array([cc[1:,g].max(axis=1).mean() for g in groups]);pi=ix[tuple(picked)];wpi=ix[tuple(working)];opi=ix[tuple(case['originalPid'])]
 out={}
 for j,p in enumerate(centers):
  q=ix[tuple(p)];out[tuple(p)]=dict(cost=float(oper[j]),oracle=float(oper.min()),loss=float(oper[j]/max(oper.min(),1e-12)),vsWorking=float(oper[j]/max(oper[wi],1e-12)),nominalVsWorking=float(cc[1:,q].mean()/max(cc[1:,wpi].mean(),1e-12)),nominalFailures=int(sum(cc[1:,q]>=1e10)),matchedVsWorking=float(matched[j]/max(matched[wi],1e-12)),saturation=float(ss[1:,q].mean()))
 return out


ROOT=BASE/'output/collection-duration-20261027-verified'
ARMS=['single30','repeat2','continuous60','repeat4','continuous120']
out=ROOT/'graded-late';out.mkdir(exist_ok=True)
for c in read(ROOT/'cases-evaluation-only.json'):
 dest=out/(c['id']+'.json')
 if dest.exists():continue
 runs=[read(ROOT/'runs'/(c['id']+'-'+a+'.json')) for a in ARMS];pids=[r['result']['pid'] if r['result']['accepted'] else r['workingPid'] for r in runs]
 with np.errstate(over='ignore',invalid='ignore',divide='ignore'):scores=grade_all(c,runs[-1],pids)
 row=dict(id=c['id'],condition=c['condition'],arms={a:dict(scores[tuple(p)],pid=p) for a,p in zip(ARMS,pids)})
 dest.write_text(json.dumps(row,indent=2));print(c['id'],flush=True)
