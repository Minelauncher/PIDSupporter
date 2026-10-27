from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import json,shutil
import numpy as np
from synthetic_gate_ablation import neighbors
base=Path(__file__).resolve().parents[1];dest=base/'output/paired-window-gate-20261027';source=base/'output/delay-ablation-20261025'
runs=[json.loads(p.read_text()) for p in (dest/'search').glob('old_*.json')];runs=[r for r in runs if r['id'] in ['old_0','old_1']]
assert len(runs)==10
for i in [1,2]:
 shutil.copy2(source/f'duster-candidate-context-{i}.npz',dest/f'duster-candidate-context-{i}.npz')
 cache={}
 for folder in ['delay-ablation-20261025','robust-joint-20261025','robust-search-20261025','tracking-requirement-20261025','residual-radius-20261026']:
  f=base/'output'/folder/f'bp-matched-{i}.json'
  if f.exists():
   for v in json.loads(f.read_text()):cache[tuple(v['pid'])]=v
 (dest/f'bp-matched-{i}.json').write_text(json.dumps(list(cache.values())))
code=(base/'grade_shared_bp_matched.py').read_text()
code=code.replace("dest=Path('PIDSupporter/tests/output/duster-shared-disturbance-20261025')","dest=Path('PIDSupporter/tests/output/paired-window-gate-20261027')")
code=code.replace('for i,r in enumerate(records):',"records=[json.loads((Path('PIDSupporter/tests/output/delay-ablation-20261025')/f'record{i}.json').read_text()) for i in range(2)]\nfrom synthetic_gate_ablation import neighbors\nfor i,r in enumerate(records):\n centers=[r['Pid']]+[run['result']['pid'] for run in runs if run['id']==f'old_{i}']\n old[i]['points']=[dict(pid=list(p)) for p in sorted({q for p in centers for q in neighbors(p)})]\n")
code=code.replace("ROOT/f'duster-candidate-context-{i+1}.npz'","dest/f'duster-candidate-context-{i+1}.npz'")
__file__=str(base/'grade_radius_bp.py')
exec(compile(code,'radius BP evaluation','exec'),globals())
out=[]
from synthetic_gate_ablation import neighbors
for run in runs:
 i=int(run['id'][-1]);cache={tuple(v['pid']):v['bp']['tracking'] for v in json.loads((dest/f'bp-matched-{i+1}.json').read_text())}
 def score(p):return max(cache[q] for q in neighbors(p))
 legacy=next(v for v in runs if v['id']==run['id'] and v['method']=='legacy');out.append(dict(id=run['id'],method=run['method'],pid=run['result']['pid'],cost=score(run['result']['pid']),vsLegacy=score(run['result']['pid'])/score(legacy['result']['pid']),vsIncumbent=score(run['result']['pid'])/score(records[i]['Pid'])))
(dest/'duster-bp.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
