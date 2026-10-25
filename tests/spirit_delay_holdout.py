"""Evaluation-only final-segment correction proxy. Not game/BP ground truth."""
exec(__import__('pathlib').Path(__file__).with_name('spirit_delay_diagnostic.py').read_text().split('rows=[]')[0])
from scipy.optimize import least_squares
p=Path('PIDSupporter/tests/output/delay-ablation-20261025');search=[json.loads(l) for l in (p/'search.jsonl').read_text().splitlines()];rows=[]
for i,(r,m) in enumerate(zip(records,models)):
 a,b,c=pars(m);n=len(a);z=np.exp(-2j*np.pi*hz*dt);G=sum(b[j]*z**(j+1) for j in range(n))/(1-sum(a[j]*z**(j+1) for j in range(n)));target=estimate(r,20,30)
 def loss(x):
  e=(x[0]*np.exp(-2j*np.pi*hz*x[1])*G-target)/abs(target);return np.r_[e.real,e.imag]
 opt=least_squares(loss,[1.5,.05],bounds=([.1,0],[10,.2]));gain,delay=opt.x
 def rad(pid):
  kp,ti,td=pid;cn=kp*np.array([1+dt/ti+td/dt,-1-2*td/dt,td/dt]);steps=delay/dt;d=int(steps);frac=steps-d;B=np.convolve(np.r_[np.zeros(1+d),gain*b],[1-frac,frac]);den=np.convolve(np.r_[1,-a],[1,-1]);num=np.convolve(B,cn);L=max(len(den),len(num));return float(max(abs(np.roots(np.pad(den,(0,L-len(den)))+np.pad(num,(0,L-len(num)))))))
 row=dict(record=i+2,proxyGain=float(gain),proxyAdditionalDelay=float(delay),proxyFitError=float(np.sqrt(np.mean(opt.fun**2)*2)),stages=[dict(stage=v['stage'],pid=v['result']['pid'],poleRadius=rad(v['result']['pid'])) for v in search if v['record']==i+2]);rows.append(row);print(json.dumps(row),flush=True)
(p/'spirit-holdout-proxy.json').write_text(json.dumps(rows,indent=2))
