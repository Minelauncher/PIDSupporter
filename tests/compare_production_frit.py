"""Frozen current records, historical C#/MathNet FRIT, common truth-only scoring."""
from grade_production_noiseless import *
from copy import deepcopy

if __name__=='__main__':
 out=ROOT/'frit-comparison';out.mkdir(exist_ok=True)
 cases=read(ROOT/'cases-evaluation-only.json')
 for c in cases:
  dest=out/(c['id']+'.json')
  if dest.exists():continue
  run=read(ROOT/'runs'/(c['id']+'.json'));f=read(ROOT/'frit'/(c['id']+'.json'));fr=deepcopy(run)
  fr['result']=dict(f,reason='legacy_frit' if f['accepted'] else 'legacy_frit_failed')
  current=run['result']['pid'] if run['result']['accepted'] else run['workingPid']
  with np.errstate(over='ignore',invalid='ignore',divide='ignore'):
   a=grade(c,run,[f['pid']]);b=grade(c,fr,[current])
  dest.write_text(json.dumps(dict(id=c['id'],current=a,frit=b,legacy=f),indent=2))
  print(c['id'],a['operationVsWorking'],b['operationVsWorking'],flush=True)
 rows=[read(out/(c['id']+'.json')) for c in cases]
 def stats(rr):
  result={}
  for method in ['current','frit']:
   vals=[r[method] for r in rr];v=np.array([x['operationVsWorking'] for x in vals]);loss=np.array([x['operationRatio'] for x in vals])
   nominal=np.array([x['operationNominalVsWorking'] for x in vals])
   result[method]=dict(n=len(vals),improved=int(sum(v<.99)),worse=int(sum(v>1.01)),unchanged=int(sum((v>=.99)&(v<=1.01))),medianVsWorking=float(np.median(v)),worstVsWorking=float(max(v)),meanBestRatio=float(np.mean(loss)),worstBestRatio=float(max(loss)),failures=sum(x['operationFailureTasks']>0 for x in vals),nominalImproved=int(sum(nominal<.99)),nominalWorse=int(sum(nominal>1.01)),nominalMedianVsWorking=float(np.median(nominal)))
  ratio=np.array([r['current']['operationCost']/r['frit']['operationCost'] for r in rr])
  result['currentVsFrit']=dict(better=int(sum(ratio<.99)),worse=int(sum(ratio>1.01)),similar=int(sum((ratio>=.99)&(ratio<=1.01))),median=float(np.median(ratio)))
  return result
 report=dict(overall=stats(rows),conditions={cond:stats([r for r in rows if r['current']['condition']==cond]) for cond in sorted(set(c['condition'] for c in cases))},fritFailures=sum(not r['legacy']['accepted'] for r in rows))
 (ROOT/'frit-comparison-summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
