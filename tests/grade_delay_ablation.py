from pathlib import Path
import sys,json,numpy as np
sys.path.insert(0,str(Path('PIDSupporter/tests').resolve()))
dest=Path('PIDSupporter/tests/output/delay-ablation-20261025')
# Reconstruct evaluation-only Duster context from this run's measured actuator records.
src=Path('PIDSupporter/tests/duster_skin_closed_loop.py').read_text().split('    variants=[];')[0]
src=src.replace('autotune-20261023-162529.jsonl','autotune-20261025-105401.jsonl').replace('zip(contexts,records)','zip(contexts[:2],records[:2])').replace("ROOT/f'duster-candidate-context-{number}.npz'","dest/f'duster-candidate-context-{number}.npz'")
sys.argv.append('--export-candidates');exec(compile(src,'latest Duster context','exec'),globals())
# Reuse corrected BP equations, but initialize from newly reconstructed context.
src=Path('PIDSupporter/tests/grade_shared_bp_matched.py').read_text()
src=src.replace("dest=Path('PIDSupporter/tests/output/duster-shared-disturbance-20261025')","dest=Path('PIDSupporter/tests/output/delay-ablation-20261025')")
src=src.replace('for i,r in enumerate(records):',"""searches=[json.loads(l) for l in (dest/'search.jsonl').read_text().splitlines()]
records=[json.loads((dest/f'record{i}.json').read_text()) for i in range(2)]
for i,r in enumerate(records):
 centers=[r['Pid']]+[v['result']['pid'] for v in searches if v['record']==i and v['result']['pid'] is not None]
 old[i]['centers']=[dict(pid=list(p)) for p in sorted(set(map(tuple,centers)))]
 old[i]['points']=[dict(pid=list(p)) for p in sorted({q for p in centers for q in neighbors(p)})]
""")
src=src.replace("ROOT/f'duster-candidate-context-{i+1}.npz'","dest/f'duster-candidate-context-{i+1}.npz'")
exec(compile(src,'latest Duster BP grading','exec'),globals())
grades=[]
for i,r in enumerate(records):
 cache={tuple(v['pid']):v['bp'] for v in json.loads((dest/f'bp-matched-{i+1}.json').read_text())}
 def cost(p):return max(cache[q]['tracking'] for q in neighbors(p))
 inc=cost(r['Pid'])
 for row in searches:
  if row['record']!=i:continue
  p=row['result']['pid'];grades.append(dict(record=i,stage=row['stage'],pid=p,robust=cost(p),versusIncumbent=cost(p)/inc,nominal=cache[tuple(p)]['tracking']))
(dest/'duster-bp.json').write_text(json.dumps(grades,indent=2));print(json.dumps(grades,indent=2))
