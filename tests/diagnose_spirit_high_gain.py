from pathlib import Path
import json,numpy as np
from scipy.signal import csd,welch
root=Path.home()/'OneDrive/Documents/From The Depths/Mods/PIDSupporter/Diagnostics';events=[json.loads(l) for l in (root/'autotune-20261025-105401.jsonl').read_text(encoding='utf-8-sig').splitlines()]
records=[e['value'] for e in events if e['kind']=='record'][2:];models=[e['value'] for e in events if e['kind']=='linearAccumulatedSelected'][2:];contexts=[e['value'] for e in events if e['kind']=='operatingContext'][2:]
out=Path('PIDSupporter/tests/output/spirit-high-gain-20261025');out.mkdir(exist_ok=True);dt=.025
def pars(m):
 n=m['order'];c=np.array(m['coefficients']);return c[:n],c[n:2*n]*m['Ys']/m['Us'],m['Ym']*(1-c[:n].sum())-m['Um']*c[n:2*n].sum()*m['Ys']/m['Us']+c[-1]*m['Ys']
def radius(m,p,delay=1):
 a,b,_=pars(m);kp,ti,td=p;cn=kp*np.array([1+dt/ti+td/dt,-1-2*td/dt,td/dt]);den=np.convolve(np.r_[1,-a],[1,-1]);num=np.convolve(np.r_[np.zeros(delay),b],cn);L=max(len(den),len(num));poly=np.pad(den,(0,L-len(den)))+np.pad(num,(0,L-len(num)));return float(max(abs(np.roots(poly))))
def sim(m,r,p=None,delay=1):
 a,b,c=pars(m);n=len(a);N=len(r['Y']);y=np.array(r['Y']);u=np.array(r['U']);inte=r['InitialIntegral'];prev=r['InitialError'];start=max(n,n+delay-1)
 for k in range(start,N):
  y[k]=a@y[k-n:k][::-1]+b@u[k-delay-n+1:k-delay+1][::-1]+c
  if p is not None:
   e=r['R'][k]-y[k]
   if k<160:inte=r['Integrals'][k]
   else:
    kp,ti,td=p;inte=np.clip(inte+e*dt,-ti/kp,ti/kp) if ti<250 else 0;u[k]=np.clip(kp*(e+inte/ti+td*(e-prev)/dt),-1,1)
   prev=e
 return dict(yNRMSE=float(np.sqrt(np.mean((y[800:]-r['Y'][800:])**2))/np.std(r['Y'][:600])),uNRMSE=float(np.sqrt(np.mean((u[800:]-r['U'][800:])**2))/np.std(r['U'][:600])),saturation=float(np.mean(abs(u[160:])>=.999999)),yRms=float(np.std(y[160:])))
result=[]
for j,(r,m,ctx) in enumerate(zip(records,models,contexts)):
 y=np.array(r['Y']);u=np.array(r['U']);z=np.array(r['InstrumentSP']);t=np.array(r['T']);a,b,c=pars(m);n=len(a);ix=np.arange(n,len(y));res=y[ix]-np.array([a@y[k-n:k][::-1]+b@u[k-n:k][::-1]+c for k in ix]);f,Szu=csd(z,u,fs=40,nperseg=400);_,Szy=csd(z,y,fs=40,nperseg=400);_,Szz=welch(z,fs=40,nperseg=400);_,Suu=welch(u,fs=40,nperseg=400);_,Syy=welch(y,fs=40,nperseg=400)
 freq=[]
 for hz in [.2,.6,1,1.4,1.8,2.6,3,4.2,5,6.2]:
  k=np.argmin(abs(f-hz));v=np.exp(-2j*np.pi*f[k]*dt);G=sum(b[h]*v**(h+1) for h in range(n))/(1-sum(a[h]*v**(h+1) for h in range(n)));obs=Szy[k]/Szu[k];freq.append(dict(hz=float(f[k]),modelMag=float(abs(G)),observedMag=float(abs(obs)),phaseErrorDeg=float(np.angle(G/obs,deg=True)),cohU=float(abs(Szu[k])**2/(Szz[k]*Suu[k])),cohY=float(abs(Szy[k])**2/(Szz[k]*Syy[k]))))
 ph=ctx['physics'];pt=np.array([s['t'] for s in ph]);features=[];names=[]
 for axis in ['Pitch','Yaw']:
  raw=np.array([next(x['rawU'] for x in s['otherAxes'] if x['axis']==axis) for s in ph]);features.append(np.interp(t[ix]-.2,pt,np.clip(raw,-1,1)));names.append(axis+' clipped request past0.2s')
 for h in range(3):features.append(np.interp(t[ix]-.2,pt,[s['localAngularVelocityRadPerSec'][h] for s in ph]));names.append('omega'+str(h)+' past0.2s')
 X=np.column_stack(features);train=ix<600;test=ix>=800
 def regress(X):
  mean=X[train].mean(0);scale=np.maximum(X[train].std(0),1e-12);D=np.column_stack([np.ones(len(X)),(X-mean)/scale]);coef=np.linalg.lstsq(D[train],res[train],rcond=None)[0];err=res-D@coef;return dict(trainRMSE=float(np.sqrt(np.mean(err[train]**2))),finalRMSE=float(np.sqrt(np.mean(err[test]**2))),finalRelative=float(np.linalg.norm(err[test])/np.linalg.norm(res[test])))
 # Separate contextual correlation from a plain slow drift control.
 residual=dict(rawFinal=float(np.sqrt(np.mean(res[test]**2))),drift=regress(t[ix,None]),otherContext=regress(X),correlations={name:float(np.corrcoef(col,res)[0,1]) for name,col in zip(names,features)})
 row=dict(run=j+1,frequency=freq,residual=residual,replays=[],poleTests=[])
 for modelidx,mm in enumerate(models):row['replays'].append(dict(model=modelidx+1,free=sim(mm,r),closed=sim(mm,r,r['Pid'])))
 for p in [r['Pid'],[.988,2.5,.4],[.999,51.1,.4]]:
  for delay in [1,2,3,4]:row['poleTests'].append(dict(pid=p,delay=delay,radius=radius(m,p,delay),replay=sim(m,r,p,delay)))
 result.append(row);print(json.dumps(row),flush=True)
(out/'results.json').write_text(json.dumps(result,indent=2))
