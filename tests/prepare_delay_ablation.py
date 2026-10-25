from pathlib import Path
import json
src=Path('PIDSupporter/tests/ResponseCapAudit');dst=Path('PIDSupporter/tests/DelayAblation');dst.mkdir(exist_ok=True)
for f in src.glob('*.cs'):
 if f.name!='Audit.cs':(dst/f.name).write_bytes(f.read_bytes())
(dst/'Audit.csproj').write_bytes((src/'Audit.csproj').read_bytes())
f=dst/'NativeAutoTune.cs';s=f.read_text();s=s.replace('for (int sign = -1; sign <= 1; sign += 2)','foreach(var model in Audit.Ensemble ?? new[]{net})\n                for (int sign = -1; sign <= 1; sign += 2)');s=s.replace('Array.Copy(warm, x, x.Length);','var state=model.RecentState(r);');s=s.replace('double yp = net.Next(x);','double yp = model.Next(state);').replace('net.Push(x, yp, next)','model.Push(state, yp, next)');f.write_text(s)
out=Path('PIDSupporter/tests/output/delay-ablation-20261025');out.mkdir(exist_ok=True)
log=Path.home()/'OneDrive/Documents/From The Depths/Mods/PIDSupporter/Diagnostics/autotune-20261025-105401.jsonl';events=[json.loads(l) for l in log.read_text(encoding='utf-8-sig').splitlines()];recs=[e['value'] for e in events if e['kind']=='record'];ctx=[e['value'] for e in events if e['kind']=='operatingContext']
for i,r in enumerate(recs):(out/f'record{i}.json').write_text(json.dumps(r))
(out/'logged.json').write_text(json.dumps([e for e in events if e['kind'] in ['pidSearch','linearAccumulatedSelected','result']],indent=2))
(out/'protocol.txt').write_text('Latest Duster records0,1 and Spirit2,3. Separate pools per craft. Train0-15s, select15-20s, final20-30s. Delays1..6 samples (25..150ms), orders1..3. Native pole-only IV fitter unchanged. Stage1 minimum existing time validation. Stage2 minimum max-across-record frequency relative complex RMS over training-visible odd harmonics0.6..3.8Hz (both training coherence>=.8); time score tie break. Stage3 all models within10% of best frequency score, worst-model native PID search. 10% is an explicit sensitivity policy, not a confidence interval. No BP/final segment for selection. No mod change.\n')
