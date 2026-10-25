from pathlib import Path
import json,numpy as np
from scipy.signal import coherence,welch
src=Path.home()/'OneDrive/Documents/From The Depths/Mods/PIDSupporter/Diagnostics/autotune-20261025-105401.jsonl'
ev=[json.loads(l) for l in src.read_text(encoding='utf-8-sig').splitlines()]
out=Path('PIDSupporter/tests/output/latest-spirit-excitation-20261025');out.mkdir(exist_ok=True)
records=[x['value'] for x in ev if x['kind']=='record'];contexts=[x['value'] for x in ev if x['kind']=='operatingContext'];plans=[x['value'] for x in ev if x['kind']=='collectionPlanFrozen'];models=[x['value'] for x in ev if x['kind']=='linearAccumulatedSelected'];rows=[]
for i,(r,c,p,m) in enumerate(zip(records,contexts,plans,models)):
 t=np.array(r['T']);t=t-t[0];u=np.array(r['U']);y=np.array(r['Y']);z=np.array(r['InstrumentSP']);sp=np.array(r['InjectedSP']);dt=np.median(np.diff(t));fs=1/dt;sel=(t>=5)&(t<25)
 def harmonic(v):
  X=np.column_stack([np.ones(sel.sum()),t[sel]-15]+[f(2*np.pi*hz*t[sel]) for hz in [.2,.6,1,1.4,1.8] for f in [np.sin,np.cos]]);coef=np.linalg.lstsq(X,v[sel],rcond=None)[0];return [float(np.hypot(*coef[2+2*k:4+2*k])) for k in range(5)]
 f,cu=coherence(z,u,fs=40,nperseg=400);_,cy=coherence(z,y,fs=40,nperseg=400)
 fu,pu=welch(u,fs=40,nperseg=400);fy,py=welch(y,fs=40,nperseg=400)
 bands=[]
 for hz in [.2,.6,1,1.4,1.8]:
  k=int(np.argmin(abs(f-hz)));bands.append(dict(hz=hz,coherenceU=float(cu[k]),coherenceY=float(cy[k])))
 n=600;lag=80;ix=np.arange(80,n);Z=np.column_stack([z[ix-j] for j in range(81)]);Z-=Z.mean(0);Z/=np.maximum(Z.std(0),1e-12);Q,s,_=np.linalg.svd(Z,full_matrices=False);Q=Q[:,s>max(Z.shape)*np.finfo(float).eps*s[0]];info=[]
 for order in [1,2,3]:
  X=np.column_stack([y[ix-j-1] for j in range(order)]+[u[ix-j-1] for j in range(order)]);X-=X.mean(0);X/=np.maximum(X.std(0),1e-12);sv=np.linalg.svd(Q.T@X/np.sqrt(len(ix)),compute_uv=False);info.append(dict(order=order,minimum=float(sv[-1]),condition=float(sv[0]/sv[-1])))
 ph=c['physics'];omega=np.array([v['localAngularVelocityRadPerSec'] for v in ph]);axes={}
 for axis in ['Pitch','Yaw']:
  v=[next((a['rawU'] for a in s['otherAxes'] if a['axis']==axis),np.nan) for s in ph];axes[axis]=dict(rawUStd=float(np.nanstd(v)),rawUPeak=float(np.nanmax(abs(np.array(v)))))
 row=dict(run=i+1,name=c['blueprintName'],runId=c['runId'],pid=r['Pid'],amplitude=p['amplitude'],cap=p['cap'],fractionOfCap=p['amplitude']/p['cap'],spRms=float(np.std(sp)),uRms=float(np.std(u)),yRms=float(np.std(y)),uPeak=float(max(abs(u))),saturation=float(np.mean(abs(u)>=.999999)),harmonicU=harmonic(u),harmonicY=harmonic(y),harmonicSP=harmonic(sp),coherences=bands,information=info,highFrequencyU=float(np.sum(pu[fu>=3])/np.sum(pu)),peakUHz=float(fu[np.argmax(pu[1:])+1]),localAngularRateRms=np.sqrt(np.mean(omega**2,axis=0)).tolist(),otherAxes=axes,closedY=m['FinalY'],closedU=m['FinalU'],freeY=m['FinalFree'])
 rows.append(row);print(json.dumps(row),flush=True)
(out/'metrics.json').write_text(json.dumps(rows,indent=2))
