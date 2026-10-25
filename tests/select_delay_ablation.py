from pathlib import Path
import json,numpy as np,hashlib
from scipy.signal import csd,welch
p=Path('PIDSupporter/tests/output/delay-ablation-20261025');r=[json.loads((p/f'record{i}.json').read_text()) for i in range(4)];fits=[json.loads(l) for l in (p/'fits.jsonl').read_text().splitlines()];hz=np.arange(.6,3.81,.4)
def freq(r,start,end):
 z=np.array(r['InstrumentSP'])[start:end];u=np.array(r['U'])[start:end];y=np.array(r['Y'])[start:end];f,zu=csd(z,u,fs=40,nperseg=200);_,zy=csd(z,y,fs=40,nperseg=200);_,zz=welch(z,fs=40,nperseg=200);_,uu=welch(u,fs=40,nperseg=200);_,yy=welch(y,fs=40,nperseg=200);ix=np.array([np.argmin(abs(f-h)) for h in hz]);return zy[ix]/zu[ix],np.minimum(abs(zu[ix])**2/(zz[ix]*uu[ix]),abs(zy[ix])**2/(zz[ix]*yy[ix]))
data=[]
for rr in r:
 tr,co=freq(rr,0,600);va,_=freq(rr,600,800);fi,_=freq(rr,800,1200);data.append(dict(train=tr,mask=(co>=.8)&np.isfinite(tr)&(abs(tr)>1e-8),val=va,final=fi))
def G(m):
 n=m['expandedOrder'];c=np.array(m['c']);z=np.exp(-2j*np.pi*hz*.025);return sum(c[n+j]*z**(j+1) for j in range(n))/(1-sum(c[j]*z**(j+1) for j in range(n)))*m['ys']/m['us']
jobs=[];decisions=[]
for i in range(4):
 pool=range(0 if i<2 else 2,i+1);variants=[f for f in fits if f['record']==i and 'failed' not in f and f['model']['gain']>0]
 for f in variants:
  pred=G(f['model']);f['frequencyValidation']=float(max(np.sqrt(np.mean(abs((pred[d['mask']]-d['val'][d['mask']])/d['val'][d['mask']])**2)) for d in [data[j] for j in pool]));d=data[i];f['frequencyFinal']=float(np.sqrt(np.mean(abs((pred[d['mask']]-d['final'][d['mask']])/d['final'][d['mask']])**2)))
 baseline=min([f for f in variants if f['delay']==1],key=lambda f:f['score']);time=min(variants,key=lambda f:f['score']);fr=min(variants,key=lambda f:(f['frequencyValidation'],f['score']));ensemble=[f for f in variants if f['frequencyValidation']<=1.1*fr['frequencyValidation']];ensemble.sort(key=lambda f:f['frequencyValidation'])
 for name,ms in [('baseline',[baseline]),('delay_only',[time]),('frequency_validation',[fr]),('consensus',ensemble)]:jobs.append(dict(record=i,stage=name,models=[v['model'] for v in ms],score=max(v['score'] for v in ms)))
 row=dict(record=i,eligibleHz=hz[data[i]['mask']].tolist(),stages={k:dict(delay=v['delay'],order=v['order'],timeValidation=v['score'],frequencyValidation=v['frequencyValidation'],frequencyFinal=v['frequencyFinal'],final=v['final']) for k,v in [('baseline',baseline),('delay_only',time),('frequency_validation',fr)]},consensus=[dict(delay=v['delay'],order=v['order'],frequencyValidation=v['frequencyValidation'],frequencyFinal=v['frequencyFinal'],final=v['final']) for v in ensemble]);decisions.append(row);print(row,flush=True)
(p/'jobs.json').write_text(json.dumps(jobs,indent=2));(p/'decisions.json').write_text(json.dumps(decisions,indent=2));(p/'scored-fits.json').write_text(json.dumps(fits,indent=2));(p/'frozen.json').write_text(json.dumps({'jobsSha256':hashlib.sha256((p/'jobs.json').read_bytes()).hexdigest()}))
