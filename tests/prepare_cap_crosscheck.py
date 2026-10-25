from pathlib import Path
import json,shutil
root=Path('PIDSupporter/tests/output/response-cap-crosscheck-20261025');root.mkdir(exist_ok=True);(root/'records').mkdir(exist_ok=True)
logs=Path.home()/'OneDrive/Documents/From The Depths/Mods/PIDSupporter/Diagnostics'
manifest=[]
for label,name in [('duster_old','autotune-20261023-162529.jsonl'),('spirit','autotune-20261021-122141.jsonl'),('duster_square','autotune-20261025-085341.jsonl')]:
 events=[json.loads(l) for l in (logs/name).read_text(encoding='utf-8-sig').splitlines()];rs=[e['value'] for e in events if e['kind']=='record']
 for i,r in enumerate(rs):
  id=f'{label}_{i+1}';(root/'records'/f'{id}.json').write_text(json.dumps(r));manifest.append(dict(id=id,group=label,delay=1,file=f'{id}.json',pool=[f'{label}_{j+1}.json' for j in range(i+1)]))
synt=Path('PIDSupporter/tests/output/synthetic-one-se-20261025')
# Select noise-free original cases before learning; no plant coefficients enter the fitting manifest.
meta=json.loads((synt/'mapping-evaluation-only.json').read_text())
for d in meta:
 if d['sigma']!=0:continue
 r=json.loads((synt/(d['id']+'.json')).read_text());r.update(T=[(i+1)*r['dt'] for i in range(len(r['U']))],Duration=30,InjectedSP=r['R'],BaseSP=[0]*len(r['U']),Amplitude=.6,Lower=-1,Upper=1,InitialIntegral=r['Integrals'][-1],InitialError=r['R'][-1]-r['Y'][-1]);id='synt_'+d['id'];(root/'records'/f'{id}.json').write_text(json.dumps(r));manifest.append(dict(id=id,group='synthetic',delay=r['knownCommandDelay'],file=f'{id}.json',pool=[f'{id}.json']))
(root/'manifest.json').write_text(json.dumps(manifest,indent=2))
runner=root/'runner';runner.mkdir(exist_ok=True)
old=Path('PIDSupporter/tests/ResponseCapAudit')
for name in ['NativeLinear.cs','NativeLinearAccumulation.cs','NativeLinearConstraints.cs','NativeAutoTune.cs','PidSearch.cs','NumericalSvd.cs','Result.cs','Audit.csproj','Audit.cs']:
 s=(old/name).read_text(encoding='utf-8-sig')
 if name=='NativeLinearAccumulation.cs':
  s=s.replace('r.U[i-1-j]','r.U[i-Audit.Delay-j]')
  s=s.replace('var m=new NativeLinear{Order=order,Coefficients=coefficients,Um=meansU[k],Ym=meansY[k],Us=us,Ys=ys};','var m=Audit.MakeModel(order,coefficients,meansU[k],meansY[k],us,ys);')
 (runner/name).write_text(s,encoding='utf-8')
print('Records',len(manifest),'real',sum(m['group']!='synthetic' for m in manifest))
