"""Train-only input-linked frequency correction; diagnostic, not a deployed estimator."""
exec(__import__('pathlib').Path(__file__).with_name('diagnose_spirit_high_gain.py').read_text().split('result=[]')[0])
from scipy.optimize import least_squares
hz=np.arange(.6,3.81,.4)
def estimate(r,start,end):
 sel=slice(round(start/dt),round(end/dt));z=np.array(r['InstrumentSP'])[sel];u=np.array(r['U'])[sel];y=np.array(r['Y'])[sel]
 f,zu=csd(z,u,fs=40,nperseg=400);_,zy=csd(z,y,fs=40,nperseg=400);ix=[np.argmin(abs(f-h)) for h in hz]
 return zy[ix]/zu[ix]

rows=[]
for i,(r,m) in enumerate(zip(records,models)):
 a,b,c=pars(m);n=len(a);v=np.exp(-2j*np.pi*hz*dt);G=sum(b[j]*v**(j+1) for j in range(n))/(1-sum(a[j]*v**(j+1) for j in range(n)));train=estimate(r,0,15);test=estimate(r,20,30)
 def fun(x):
  e=(x[0]*np.exp(-2j*np.pi*hz*x[1])*G-train)/abs(train);return np.r_[e.real,e.imag]
 fit=least_squares(fun,[1,.03],bounds=([.1,0],[10,.2]));gain,delay=fit.x;new=gain*np.exp(-2j*np.pi*hz*delay)*G
 def rad(p, gain=gain, delay=delay):
  kp,ti,td=p;cn=kp*np.array([1+dt/ti+td/dt,-1-2*td/dt,td/dt]);steps=delay/dt;integer=int(steps);frac=steps-integer;B=np.convolve(np.r_[np.zeros(1+integer),gain*b],[1-frac,frac]);den=np.convolve(np.r_[1,-a],[1,-1]);num=np.convolve(B,cn);L=max(len(den),len(num));return float(max(abs(np.roots(np.pad(den,(0,L-len(den)))+np.pad(num,(0,L-len(num)))))))
 row=dict(record=i+1,gainFactor=float(gain),additionalDelaySeconds=float(delay),trainComplexRelativeRMSEBefore=float(np.sqrt(np.mean(abs((G-train)/train)**2))),trainComplexRelativeRMSEAfter=float(np.sqrt(np.mean(abs((new-train)/train)**2))),finalComplexRelativeRMSEBefore=float(np.sqrt(np.mean(abs((G-test)/test)**2))),finalComplexRelativeRMSEAfter=float(np.sqrt(np.mean(abs((new-test)/test)**2))),pids=[dict(pid=p,originalRadius=radius(m,p),correctedRadius=rad(p),gainOnlyRadius=rad(p,delay=0),delayOnlyRadius=rad(p,gain=1)) for p in [[.0625,250,.5],[.247,2.5,.4],[.988,2.5,.4],[.999,51.1,.4]]],frequencies=hz.tolist())
 rows.append(row);print(json.dumps(row),flush=True)
(out/'delay-correction.json').write_text(json.dumps(rows,indent=2))
