from pathlib import Path
import json,numpy as np
root=Path(__file__).resolve().parents[1]/'output/collection-duration-20261027-verified'
read=lambda p:json.loads(p.read_text())
cases={x['id']:x for x in read(root/'cases-evaluation-only.json')};out=[]
for cid in ['p061','p063']:
 base=read(root/'runs'/(cid+'-single30.json'));pid=base['result']['pid']
 for arm in ['single30','repeat2','repeat4','continuous120']:
  r=read(root/'runs'/(cid+'-'+arm+'.json'));m=r['model'];n=m['order'];co=np.array(m['coefficients']);gate=next(e['value'] for e in r['events'] if e['kind']=='empiricalFrequencyBounds');f=np.array(gate['frequencies']);z=np.exp(-2j*np.pi*f*.05)
  g=sum(co[n+j]*z**(j+1) for j in range(n))/(1-sum(co[j]*z**(j+1) for j in range(n)))*m['Ys']/m['Us'];kp,ti,td=pid
  cn=np.array([kp*(1+.05/ti+td/.05),kp*(-1-2*td/.05),kp*td/.05]);cc=sum(c*z**i for i,c in enumerate(cn))/(1-z);rho=np.array(gate['radius'])*abs(cc/(1+g*cc));j=int(np.argmax(rho))
  # Plant truth is accessed only here, after all choices are frozen.
  p=cases[cid]['plant'];A=np.array(p['A']);B=np.array(p['B']);C=np.array(p['C']);truth=np.array([zz*(C@np.linalg.solve(np.eye(len(A))-zz*A,B)) for zz in z]);worst=None
  for ri,record in enumerate(r['records']):
   count=100;blocks=(len(record['U'])*2//3)//count
   for block in range(blocks):
    win=.5-.5*np.cos(2*np.pi*np.arange(count)/count);ex=np.exp(-2j*np.pi*f[j]*.05*np.arange(count));u=np.array(record['U'][block*count:(block+1)*count]);y=np.array(record['Y'][block*count:(block+1)*count]);U=np.sum((u-u.mean())*win*ex);Y=np.sum((y-y.mean())*win*ex);err=abs(Y/U-g[j]);v=dict(record=ri+1,block=block,frequency=float(f[j]),inputFourier=float(abs(U)),observedError=float(err))
    if worst is None or err>worst['observedError']:worst=v
  row=dict(id=cid,arm=arm,order=n,validation=m['validation'],actualModelMaxFrequencyError=float(max(abs(g-truth))),previousPid=pid,previousPidMaxRho=float(max(rho)),worstWindow=worst);out.append(row);print(cid,arm,'model error',row['actualModelMaxFrequencyError'],'rho',max(rho),'window',worst)
(root/'gate-diagnosis.json').write_text(json.dumps(out,indent=2))
