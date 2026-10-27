from pathlib import Path
import json,hashlib
import numpy as np
BASE=Path(__file__).resolve().parents[1];ROOT=BASE/'output/paired-window-gate-20261027'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
rows=[read(p) for p in sorted((ROOT/'graded-all').glob('*.json'))];assert len(rows)==72
summary={}
for a in ['single30','repeat2','repeat4']:
 summary[a]={}
 for m in ['legacy','paired','warmup','equation']:
  v=[r['arms'][a][m] for r in rows]
  summary[a][m]=dict(meanLoss=float(np.mean([x['loss'] for x in v])),medianLoss=float(np.median([x['loss'] for x in v])),maxLoss=max(x['loss'] for x in v),worse=sum(x['vsWorking']>1.000001 for x in v),goodRejected=sum(x['goodRejected'] for x in v),goodTotal=sum(x['goodTotal'] for x in v),oracleSurvives=sum(x['oracleSurvives'] for x in v),recommendations=sum(x['accepted'] for x in v),heldoutPass=sum(x['selectedHeldoutPasses'] for x in v),budgetReached=sum(bool(x['searchBudgetReached']) for x in v))
game=[read(p) for p in sorted((ROOT/'search').glob('*.json')) if not p.name.startswith('p')]
results=dict(protocol=read(ROOT/'protocol.json'),summary=summary,game=game,spiritUngatedProbes=read(ROOT/'spirit-off-probes.json'),dusterBP=read(ROOT/'duster-bp.json'),verification=read(ROOT/'verification.json'),cases=rows)
(ROOT/'summary.json').write_text(json.dumps(summary,indent=2))
(Path(__file__).parent/'results.json').write_text(json.dumps(results,indent=2))
(ROOT/'search-frozen.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'search').glob('*.json')},indent=2))
print('Saved 72-case results, 25 game searches, BP scores and verification.')
