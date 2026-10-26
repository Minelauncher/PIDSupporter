import sys,json,hashlib
from pathlib import Path
import numpy as np
BASE=Path(__file__).resolve().parents[1]
ROOT=BASE/'output/collection-duration-20261027-verified'
ARMS=['single30','repeat2','continuous60','repeat4','continuous120']
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
cases=read(ROOT/'cases-evaluation-only.json')
rows=[read(ROOT/'graded'/(c['id']+'.json')) for c in cases]
def stats(rows):
 out={}
 for arm in ARMS:
  a=[r['arms'][arm] for r in rows];loss=np.array([x['loss'] for x in a]);w=np.array([x['vsWorking'] for x in a]);v=np.array([r['arms'][arm]['cost']/r['arms']['single30']['cost'] for r in rows]);nom=np.array([x['nominalVsWorking'] for x in a])
  out[arm]=dict(n=len(a),meanLoss=float(np.mean(loss)),medianLoss=float(np.median(loss)),worstLoss=float(max(loss)),worseWorking=int(sum(w>1.01)),betterWorking=int(sum(w<.99)),medianVsWorking=float(np.median(w)),worstVsWorking=float(max(w)),betterThan30=int(sum(v<.99)),worseThan30=int(sum(v>1.01)),medianVs30=float(np.median(v)),accepted=sum(x['accepted'] for x in a),nominalWorseWorking=int(sum(nom>1.01)),nominalMedianVsWorking=float(np.median(nom)),nominalFailures=sum(x['nominalFailures']>0 for x in a),neighborhoodFailures=sum(x['cost']>=1e10 for x in a),secondsMedian=float(np.median([x['seconds'] for x in a])),budgetHits=sum(bool(x['search'] and x['search']['BudgetReached']) for x in a),orders={str(o):sum(x['order']==o for x in a) for o in [None,1,2,3]},reasons={key:sum(x['reason']==key for x in a) for key in set(x['reason'] for x in a)})
 return out
report=dict(overall=stats(rows),conditions={k:stats([r for r in rows if r['condition']==k]) for k in sorted({r['condition'] for r in rows})})
report['equalTime']={}
for a,b in [('repeat2','continuous60'),('repeat4','continuous120')]:
 v=np.array([r['arms'][a]['cost']/r['arms'][b]['cost'] for r in rows]);report['equalTime'][a+'_vs_'+b]=dict(better=int(sum(v<.99)),worse=int(sum(v>1.01)),median=float(np.median(v)))
report['collection']={}
for arm in ARMS:
 energy=[];sat=[];peak=[];radii=[]
 for c in cases:
  r=read(ROOT/'runs'/(c['id']+'-'+arm+'.json'));u=np.concatenate([x['U'] for x in r['records']]);energy.append(float(np.sum(u*u)*.05));sat.append(float(np.mean(abs(u)>=.99999)));peak.append(float(max(abs(u))));freq=[e['value'] for e in r['events'] if e['kind']=='empiricalFrequencyBounds'];radii.append(max(freq[0]['radius']) if freq else None)
 report['collection'][arm]=dict(medianInputEnergy=float(np.median(energy)),medianSaturation=float(np.median(sat)),maxSaturation=max(sat),maxPeak=max(peak))
report['worstChanges']={a:sorted([dict(id=r['id'],condition=r['condition'],ratio=r['arms'][a]['cost']/r['arms']['single30']['cost'],old=r['arms']['single30']['pid'],new=r['arms'][a]['pid'],reason=r['arms'][a]['reason']) for r in rows],key=lambda x:x['ratio'],reverse=True)[:5] for a in ARMS[1:]}
late=[]
for row in rows:
 p=ROOT/'graded-late'/(row['id']+'.json')
 if p.exists():
  l=read(p);late.append(dict(row,arms={a:dict(row['arms'][a],**l['arms'][a]) for a in ARMS}))
if len(late)==len(rows):report['lateCommonEndpoint']=stats(late)
(ROOT/'summary.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report['overall'],indent=2));print(json.dumps(report['equalTime'],indent=2))
