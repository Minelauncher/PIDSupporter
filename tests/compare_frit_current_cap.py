"""Frozen recommendations, fresh matched BP grading; never changes runtime."""
from pathlib import Path
import json,hashlib
DEST=Path('PIDSupporter/tests/output/frit-current-cap-20261025');DEST.mkdir(exist_ok=True)
BASE=Path('PIDSupporter/tests/output/response-cap-crosscheck-20261025')
legacy=json.loads(Path('PIDSupporter/tests/output/bp-replay/duster-legacy-frit-compare.json').read_text())
fits=[json.loads(l) for l in (BASE/'fits.jsonl').read_text().splitlines()]
recommendations=[]
for i in [1,2]:
 rows=[dict(method='FRIT',pid=legacy[i-1]['selected']['pid'])]
 for m in fits:
  if m['id']!=f'duster_old_{i}':continue
  rows.append(dict(method=('current' if m['cap'] else 'no_response_cap')+('_pooled' if m['cumulative'] else '_single'),pid=m['result']['pid']))
 recommendations.append(rows)
(DEST/'recommendations.json').write_text(json.dumps(recommendations,indent=2))
(DEST/'frozen.json').write_text(json.dumps({'sha256':hashlib.sha256((DEST/'recommendations.json').read_bytes()).hexdigest(),'source':'FRIT port of715d44b, SciPy LM not binary-identical MathNet. IV frozen native cap-crosscheck fits.'}))
# Reuse the freshly matched one-step BP evaluator; copy only its same-protocol cache.
for i in [1,2]:
 target=DEST/f'bp-matched-{i}.json'
 if not target.exists():target.write_bytes(Path(f'PIDSupporter/tests/output/duster-shared-disturbance-20261025/bp-matched-{i}.json').read_bytes())
src=Path('PIDSupporter/tests/grade_shared_bp_matched.py').read_text()
src=src.replace("dest=Path('PIDSupporter/tests/output/duster-shared-disturbance-20261025')","dest=DEST")
src=src.replace("for i,r in enumerate(records):", """for i,r in enumerate(records):
 centers=[dict(label='incumbent',pid=r['Pid'])]+old[i]['centers']+[dict(label=x['method'],pid=x['pid']) for x in recommendations[i]]
 unique={tuple(c['pid']) for c in centers};old[i]['centers']=[dict(pid=list(p)) for p in sorted(unique)]
 old[i]['points']=[dict(pid=list(p)) for p in sorted({p for c in unique for p in neighbors(c)})]
""")
exec(compile(src,'matched BP comparison','exec'),globals())
summary=[]
for i,r in enumerate(records):
 cache={tuple(v['pid']):v['bp'] for v in json.loads((DEST/f'bp-matched-{i+1}.json').read_text())}
 def grade(p):
  vals=[cache[q]['tracking'] for q in neighbors(p)];return dict(nominal=cache[tuple(p)]['tracking'],robust=max(vals),failed=bool(cache[tuple(p)]['failed']))
 centers=[dict(pid=c['pid'],**grade(c['pid'])) for c in old[i]['centers']];best=min(v['robust'] for v in centers);bestNominal=min(v['nominal'] for v in centers);inc=grade(r['Pid']);rows=[]
 for rec in recommendations[i]:
  v=grade(rec['pid']);rows.append(dict(**rec,**v,versusIncumbent=v['robust']/inc['robust'],nominalVersusIncumbent=v['nominal']/inc['nominal'],regret=v['robust']/best-1,nominalRegret=v['nominal']/bestNominal-1))
 summary.append(dict(record=i+1,incumbent=dict(pid=r['Pid'],**inc),best=best,bestNominal=bestNominal,centers=centers,methods=rows))
(DEST/'results.json').write_text(json.dumps(summary,indent=2))
for run in summary:
 print('RECORD',run['record'],'incumbent',run['incumbent'],flush=True)
 for row in run['methods']:print(row,flush=True)
