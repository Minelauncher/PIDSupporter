from pathlib import Path
import json,hashlib,copy
import numpy as np
from bounds import bounds,accept

BASE=Path(__file__).resolve().parents[1];ROOT=BASE/'output/paired-window-gate-20261027'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
original=read(ROOT/'source-hashes.json')
assert all(hashlib.sha256((BASE.parent/'PIDSupporterCore'/k).read_bytes()).hexdigest()==v for k,v in original.items())
errors=[];probes=0
for f in (ROOT/'jobs').glob('*.json'):
 j=read(f)
 if j['kind']=='synthetic':
  old=read(BASE/'output/collection-duration-20261027-verified/runs'/f.name)
  legacy=next(e['value']['radius'] for e in old['events'] if e['kind']=='empiricalFrequencyBounds')
  errors.append(float(np.max(abs(np.array(legacy)-j['bounds']['radii']['legacy']))))
 for method in ['legacy','paired','warmup','equation']:
  s=read(ROOT/'search'/(j['id']+'-'+method+'.json'))
  if s['probe']:
   expected=accept(j['model'],[p['pid'] for p in s['probe']],j['bounds'],method)
   assert np.array_equal(expected,[p['accepted'] for p in s['probe']]);probes+=len(expected)
assert max(errors)<1e-8
spirit=[]
for cid in ['old_2','old_3','latest_spirit']:
 j=read(ROOT/'jobs'/(cid+'.json'));p=read(ROOT/'search'/(cid+'-off.json'))['result']['pid']
 spirit.append(dict(id=cid,pid=p,equationAccepted=bool(accept(j['model'],[p],j['bounds'],'equation')[0])))
assert not any(x['equationAccepted'] for x in spirit)
(ROOT/'spirit-off-probes.json').write_text(json.dumps(spirit,indent=2))
# Algebraic diagnostic: generate outputs from an arbitrary fixed identified model,
# nonzero initial output history. This is a test fixture, never used by selection.
j=read(ROOT/'jobs/p063-repeat2.json');m=j['model'];n=m['order'];c=np.array(m['coefficients']);b=c[n:2*n]*m['Ys']/m['Us'];offset=m['Ym']*(1-sum(c[:n]))-m['Um']*sum(b)+m['Ys']*c[-1]
records=copy.deepcopy(j['records'])
for r in records:
 y=np.zeros(len(r['Y']));y[:n]=np.linspace(.2,.3,n);u=r['U']
 for k in range(n,len(y)):y[k]=sum(c[h]*y[k-1-h]+b[h]*u[k-1-h] for h in range(n))+offset
 r['Y']=y.tolist()
bb=bounds(records,m);wrong=copy.deepcopy(m)
wrong['coefficients'][n:2*n]=(np.array(wrong['coefficients'][n:2*n])*1.1).tolist()
wrongbb=bounds(records,wrong)
assert max(bb['radii']['equation'])<1e-9
assert max(wrongbb['radii']['equation'])>1e-3
out=dict(productionSourcesUnchanged=len(original),legacyRadiusMaxAbsError=max(errors),syntheticDatasets=len(errors),csharpPythonProbeMatches=probes,exactModel=dict(legacyMax=max(bb['radii']['legacy']),equationMax=max(bb['radii']['equation'])),tenPercentWrongNumeratorEquationMax=max(wrongbb['radii']['equation']))
(ROOT/'verification.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
