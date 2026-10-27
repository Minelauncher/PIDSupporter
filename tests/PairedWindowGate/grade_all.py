from pathlib import Path
import sys,json,time,hashlib
sys.path.insert(0,str(Path(__file__).resolve().parents[1]));from production_noiseless_common import *
from bounds import accept_many
# Reuse the frozen physical evaluator without running its collection loop.
exec((BASE/'CollectionDuration/grade.py').read_text().split("ROOT=BASE/")[0])
ROOT=BASE/'output/paired-window-gate-20261027';SOURCE=BASE/'output/collection-duration-20261027-verified'
ARMS=['single30','repeat2','repeat4'];METHODS=['legacy','paired','warmup','equation'];out=ROOT/'graded-all';out.mkdir(exist_ok=True)
for case in read(SOURCE/'cases-evaluation-only.json'):
 cid=case['id'];dest=out/(cid+'.json')
 if dest.exists():continue
 paths=[ROOT/'search'/(cid+'-'+a+'-'+m+'.json') for a in ARMS for m in METHODS];deadline=time.time()+3600
 while not all(p.exists() for p in paths):
  if time.time()>deadline:raise TimeoutError(cid)
  time.sleep(1)
 picked={};jobs={}
 for a in ARMS:
  jobs[a]=read(ROOT/'jobs'/(cid+'-'+a+'.json'))
  for m in METHODS:
   r=read(ROOT/'search'/(cid+'-'+a+'-'+m+'.json'));picked[a,m]=r['result']['pid'] if r['result']['accepted'] else jobs[a]['record']['Pid']
 baseline=read(SOURCE/'runs'/(cid+'-single30.json'))
 with np.errstate(over='ignore',invalid='ignore',divide='ignore'):scores=grade_all(case,baseline,list(picked.values()))
 centers=list(scores);best=min(v['cost'] for v in scores.values());good=np.array([scores[c]['cost']<=1.1*best for c in centers]);points=sorted({p for c in centers for p in neighbors(c)});ix={p:i for i,p in enumerate(points)};groups=[[ix[p] for p in neighbors(c)] for c in centers]
 row=dict(id=cid,condition=case['condition'],plant=case['plant']['id'],arms={})
 for a in ARMS:
  job=jobs[a];masks=accept_many(job['model'],points,job['bounds']);allowed={m:np.array([masks[m][g].all() for g in groups]) for m in masks};row['arms'][a]={}
  for m in METHODS:
   p=tuple(picked[a,m]);j=centers.index(p);s=read(ROOT/'search'/(cid+'-'+a+'-'+m+'.json'));v=dict(scores[p],pid=p,accepted=s['result']['accepted'],reason=s['result']['reason'],goodTotal=int(sum(good)),goodRejected=int(sum(good&~allowed[m])),oracleSurvives=bool(any(allowed[m][k] and scores[c]['cost']<=best*(1+1e-9) for k,c in enumerate(centers))),selectedNeighborhoodPasses=bool(allowed[m][j]),selectedHeldoutPasses=bool(allowed[{'legacy':'heldout','paired':'heldout','warmup':'heldoutWarmup','equation':'heldoutEquation'}[m]][j]),seconds=s['result']['seconds'],searchBudgetReached=s['search'] and s['search']['BudgetReached']);row['arms'][a][m]=v
 dest.write_text(json.dumps(row,indent=2));print(cid,[(a,round(row['arms'][a]['legacy']['loss'],3),round(row['arms'][a]['paired']['loss'],3)) for a in ARMS],flush=True)
(ROOT/'search-frozen.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'search').glob('*.json')},indent=2))
