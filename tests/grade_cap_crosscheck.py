from pathlib import Path
import sys,json,hashlib
import numpy as np
from scipy.stats import spearmanr
sys.path.insert(0,str(Path('PIDSupporter/tests').resolve()))
import synthetic_one_se_validation as s
root=Path('PIDSupporter/tests/output/response-cap-crosscheck-20261025')
rows=[json.loads(x) for x in (root/'fits.jsonl').read_text().splitlines()];grid=s.read('grid.json');meta={x['id']:x for x in s.read('mapping-evaluation-only.json')};specs={x['id']:x for x in s.read('plants-evaluation-only.json')};truth={};out=[]
# Fits and choice rules are already frozen; truth is consulted only from this evaluation stage onward.
(root/'fit-sha256.txt').write_text(hashlib.sha256((root/'fits.jsonl').read_bytes()).hexdigest())
for row in rows:
 if not row['id'].startswith('synt_') or 'failed' in row:continue
 cid=row['id'][5:];o=row['order'];big=row['expandedOrder'];c=row['c'];m=dict(order=o,c=c[:o]+c[big+row['delay']-1:big+row['delay']-1+o]+[c[-1]],um=row['um'],ym=row['ym'],us=row['us'],ys=row['ys'])
 pred=s.predict_or_truth(m,None,grid['points'],0);chosen,ps=s.choose(pred,grid);inc=len(grid['centers'])-1
 invalid=(row['gain']<=0 or not np.isfinite(row['gain']));noImprovement=ps[chosen,0]>=.95*ps[inc,0];effective=inc if invalid or noImprovement else chosen
 plant=meta[cid]['plant']
 if plant not in truth:
  costs=s.predict_or_truth(None,specs[plant],grid['points'],0);oracle,ts=s.choose(costs,grid);truth[plant]=(costs,oracle,ts)
 costs,oracle,ts=truth[plant];base=float(ts[:,0].min());actual=float(ts[effective,0]);regret=actual/max(base,1e-12)-1
 rr=dict(id=cid,plant=plant,cap=row['cap'],order=o,gain=row['gain'],validation=row['validation'],final=[row['FinalFree'],row['FinalY'],row['FinalU']],invalidGain=invalid,retained=invalid or noImprovement,pid=grid['centers'][effective],regret=regret,cost=actual,oracleCost=base,incumbentCost=float(ts[inc,0]),worseThanIncumbent=bool(actual>ts[inc,0]*1.01),divergent=bool(actual>=1e10),predictedDivergentFraction=float(np.mean(pred[:,0]>=1e10)),boundaryPid=any([grid['centers'][effective][0] in [.001,1],grid['centers'][effective][1]==.1,grid['centers'][effective][2]==100]))
 out.append(rr);print('GRADE',cid,row['cap'],round(regret,4),rr['pid'],flush=True)
(root/'synthetic-grades.json').write_text(json.dumps(out,indent=2,default=lambda v:v.item()))
for cap in [True,False]:
 a=[v for v in out if v['cap']==cap];print('SUMMARY',cap,len(a),'regret mean/median/max',np.mean([v['regret'] for v in a]),np.median([v['regret'] for v in a]),max(v['regret'] for v in a),'negative',sum(v['invalidGain'] for v in a),'worse',sum(v['worseThanIncumbent'] for v in a),'diverged',sum(v['divergent'] for v in a))
