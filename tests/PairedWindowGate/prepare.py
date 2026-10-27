from pathlib import Path
import json,hashlib,sys
from bounds import bounds
BASE=Path(__file__).resolve().parents[1];ROOT=BASE/'output/paired-window-gate-20261027';jobs=ROOT/'jobs';jobs.mkdir(exist_ok=True)
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
def save(j):
 j['bounds']=bounds(j['records'],j['model']);(jobs/(j['id']+'.json')).write_text(json.dumps(j))
source=BASE/'output/collection-duration-20261027-verified'
for c in read(source/'cases-evaluation-only.json'):
 for arm in ['single30','repeat2','repeat4']:
  r=read(source/'runs'/(c['id']+'-'+arm+'.json'));save(dict(id=c['id']+'-'+arm,kind='synthetic',caseId=c['id'],arm=arm,record=r['record'],records=r['records'],model=r['model'],score=r['model']['validation']))
game=read(BASE/'output/residual-radius-20261026/game-cases.json')
for i,c in enumerate(game):
 old=c['model'];m=dict(order=old['expandedOrder'],coefficients=old['c'],Um=old['um'],Ym=old['ym'],Us=old['us'],Ys=old['ys']);records=[c['record']]
 if i in [1,3]:records=[game[i-1]['record'],c['record']]
 save(dict(id=c['id'],kind='game',craft=c['craft'],record=c['record'],records=records,model=m,score=c['score']))
(ROOT/'protocol.json').write_text(json.dumps(dict(primary='paired free-rollout residual, same windows as measured response',initialization='Only first n measured outputs/input histories, n=model order. No fitted initial state, no trend or disturbance fitting, no output feedback correction.',aggregation='Unchanged maximum over all first-2/3 5-second windows, same10 frequencies and threshold1. No gain caps, radius scaling or dropping windows.',selection='Frozen same identified model for both gates; current C# search and nominal Schur test unchanged.',holdout='Final third residual spectra audited only after selection; never used to pick gate settings.',synthetic='72 cases x single30/repeat2/repeat4; no measurement noise; plant truth only in grading.',game='Two Duster, two older Spirit, one latest Spirit log. Historical selected models frozen; no future other-axis observations.',limitations='Empirical residual envelope, not guaranteed uncertainty coverage. Same model affine offset for all records; mismatch is not fitted away. Perfect finite-window equality is necessary, not sufficient, for robustness.'),indent=2))
(ROOT/'jobs-frozen.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in jobs.glob('*.json')},indent=2));print('Prepared',len(list(jobs.glob('*.json'))),'data-only jobs')

# Preserve exploration chronology when regenerating data-only jobs.
p=ROOT/"protocol.json";j=read(p);j["exploratoryVariants"]={'warmup': 'After primary rejected known-good Spirit, initial state only fitted to first 2s (existing IV history length), same free residual envelope.', 'equation': 'After warmup still rejected known-good Spirit, use A(q)y-B(q)u-offset residual divided spectrally by nominal A and U. Same windows, maximum, frequencies, threshold. No truth used in radius or search. Finite-window inversion approximate; no certified bound.'};p.write_text(json.dumps(j,indent=2))
