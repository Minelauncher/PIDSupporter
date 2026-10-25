"""Common plant, separate deterministic nuisance forcing; no BP in fitting."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import json,ast,hashlib
import numpy as np
from scipy.signal import lfilter
from scipy.optimize import least_squares
BASE=Path('PIDSupporter/tests/output/response-cap-crosscheck-20261025')
OUT=Path('PIDSupporter/tests/output/duster-shared-disturbance-20261025');OUT.mkdir(exist_ok=True)
R=[json.loads((BASE/f'records/duster_old_{i}.json').read_text()) for i in [1,2]]
DT=.025
def denom(x,n):
 v=np.exp(x)
 if n==1: roots=[np.exp(-DT/v[0])]
 else:
  z=np.exp((-1/v[0]+2j*np.pi*v[1])*DT);roots=[z,z.conjugate()]
  if n==3:roots.append(np.exp(-DT/v[2]))
 return np.poly(roots).real
def design(r,den,kind,N):
 n=len(den)-1;u=np.asarray(r['U'][:N]);t=np.arange(N)*DT
 U=np.column_stack([lfilter(np.r_[np.zeros(j+1),1],den,u) for j in range(n)])
 Z=np.column_stack([lfilter([1],den,np.zeros(N),zi=np.eye(n)[j])[0] for j in range(n)])
 D=np.ones((N,1)) if kind=='constant' else np.column_stack([np.ones(N),t/15])
 return U,np.column_stack([Z,D])
def fit(pool,n,kind,weights=None):
 weights=np.ones(len(pool)) if weights is None else np.asarray(weights)
 def solve(x,ret=False):
  den=denom(x,n);cols=n+len(pool)*(n+(1 if kind=='constant' else 2));Xs=[];Ys=[]
  for j,r in enumerate(pool):
   U,V=design(r,den,kind,600);X=np.zeros((600,cols));X[:,:n]=U;X[:,n+j*V.shape[1]:n+(j+1)*V.shape[1]]=V;s=np.std(r['Y'][:600]);Xs.append(X/s*np.sqrt(weights[j]));Ys.append(np.array(r['Y'][:600])/s*np.sqrt(weights[j]))
  X=np.vstack(Xs);y=np.concatenate(Ys);scale=np.maximum(np.linalg.norm(X,axis=0),1e-15);coef=np.linalg.lstsq(X/scale,y,rcond=None)[0]/scale;err=X@coef-y
  return (den,coef,err) if ret else err
 lower=np.log([.05] if n==1 else [.05,.005]+([.025] if n==3 else []));upper=np.log([300] if n==1 else [300,5]+([300] if n==3 else []))
 starts=[[1],[10],[100]] if n==1 else [[tau,f]+([.2] if n==3 else []) for tau,f in [(1,.3),(10,.03),(10,.08),(50,.1)]]
 opts=[least_squares(solve,np.log(v),bounds=(lower,upper),max_nfev=180,ftol=1e-9,xtol=1e-9,gtol=1e-9) for v in starts];opt=min(opts,key=lambda v:np.sum(v.fun**2));den,c,err=solve(opt.x,True);w=n+(1 if kind=='constant' else 2)
 return dict(order=n,kind=kind,den=den.tolist(),b=c[:n].tolist(),nuisance=[c[n+j*w:n+(j+1)*w].tolist() for j in range(len(pool))],poleParameters=np.exp(opt.x).tolist(),trainRMSE=float(np.sqrt(np.mean(err**2))),converged=bool(opt.success),starts=[dict(cost=float(np.mean(v.fun**2)),parameters=np.exp(v.x).tolist(),success=bool(v.success)) for v in opts])
def predict(r,m,j,closed=False):
 den=np.array(m['den']);n=m['order'];coef=np.array(m['nuisance'][j]);N=len(r['Y']);U,V=design(r,den,m['kind'],N)
 if not closed:return U@np.array(m['b'])+V@coef,np.asarray(r['U'])
 # Same four-second measured-input warmup as the existing replay, no output corrections.
 d=V[:,n:]@coef[n:];state=coef[:n].copy();uh=[0.]*n;yy=[];uu=[];prev=r['InitialError'];inte=r['InitialIntegral'];kp,ti,td=r['Pid']
 for k in range(N):
  yp=float(np.array(m['b'])@uh+state[0]);state=np.r_[state[1:],0]-den[1:]*yp;y=yp+d[k];e=r['R'][k]-y
  if k<160:u=r['U'][k];inte=r['Integrals'][k] if 'Integrals' in r else inte
  else:
   inte=np.clip(inte+e*DT,-ti/kp,ti/kp) if ti<250 else 0;u=float(np.clip(kp*(e+inte/ti+td*(e-prev)/DT),-1,1))
  prev=e;uh=[u]+uh[:-1];yy.append(y);uu.append(u)
 return np.array(yy),np.array(uu)
def evaluate(pool,m):
 ev=[]
 for j,r in enumerate(pool):
  yf,_=predict(r,m,j);yc,uc=predict(r,m,j,True);err=np.array([(yf-r['Y'])/np.std(r['Y'][:600]),(yc-r['Y'])/np.std(r['Y'][:600]),(uc-r['U'])/np.std(r['U'][:600])]);ev.append(dict(validation=np.sqrt(np.mean(err[:,600:800]**2,axis=1)).tolist(),final=np.sqrt(np.mean(err[:,800:]**2,axis=1)).tolist(),free=yf.tolist(),closed=yc.tolist(),u=uc.tolist()))
 return ev
def main():
 rows=[]
 for label,pool in [('single1',[R[0]]),('single2',[R[1]]),('shared',R)]:
  variants=[]
  for kind in ['constant','affine']:
   for n in [1,2,3]:
    m=fit(pool,n,kind);m['evaluation']=evaluate(pool,m);m['score']=float(np.max([v['validation'] for v in m['evaluation']]));variants.append(m);print(label,kind,n,'poles',m['poleParameters'],'score',m['score'],flush=True)
  pick=min(range(len(variants)),key=lambda i:(variants[i]['score'],variants[i]['order']));rows.append(dict(label=label,variants=variants,selected=pick))
 (OUT/'fits.json').write_text(json.dumps(rows,indent=2))
 # Weight sensitivity is training-only, and never used to choose the final model.
 shared=rows[-1]['variants'][rows[-1]['selected']];sensitivity=[]
 for weights in [[.5,1],[1,.5]]:
  m=fit(R,shared['order'],shared['kind'],weights);sensitivity.append(dict(weights=weights,model=m));print('weight sensitivity',weights,m['poleParameters'],flush=True)
 (OUT/'sensitivity.json').write_text(json.dumps(sensitivity,indent=2));(OUT/'frozen.json').write_text(json.dumps({'sha256':hashlib.sha256((OUT/'fits.json').read_bytes()).hexdigest()}))
if __name__=='__main__':main()
