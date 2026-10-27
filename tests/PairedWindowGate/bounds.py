import numpy as np

def bounds(records,m):
 n=m['order'];a=np.array(m['coefficients']);b=a[n:2*n]*m['Ys']/m['Us'];offset=m['Ym']*(1-sum(a[:n]))-m['Um']*sum(b)+m['Ys']*a[-1]
 dt=(records[-1]['T'][-1]-records[-1]['T'][0])/(len(records[-1]['T'])-1);hz=np.arange(10)*.4+.2;hz=hz[hz<.5/dt];z=np.exp(-2j*np.pi*hz*dt);g=sum(b[j]*z**(j+1) for j in range(n))/(1-sum(a[j]*z**(j+1) for j in range(n)))
 legacy=[];paired=[];warmup=[];equation=[];hold=[];hold_warmup=[];hold_equation=[];diagnostics=[]
 for r in records:
  u=np.array(r['U']);y=np.array(r['Y']);N=len(y);pred=np.empty(N);pred[:n]=y[:n]
  # Observe only the first n samples. No future output correction or initial-state fit.
  for k in range(n,N):pred[k]=sum(a[j]*pred[k-1-j]+b[j]*u[k-1-j] for j in range(n))+offset
  residual=y-pred;eq=np.zeros(N)
  for k in range(n,N):eq[k]=y[k]-sum(a[j]*y[k-1-j]+b[j]*u[k-1-j] for j in range(n))-offset
  aden=1-sum(a[j]*z**(j+1) for j in range(n))
  count=round(5/dt);t=np.arange(count);kernel=(.5-.5*np.cos(2*np.pi*t/count))*np.exp(-2j*np.pi*hz[:,None]*dt*t)
  forced=np.zeros(N);H=np.zeros((N,n));H[:n]=np.eye(n)
  for k in range(n,N):
   forced[k]=sum(a[j]*forced[k-1-j]+b[j]*u[k-1-j] for j in range(n))+offset
   H[k]=sum(a[j]*H[k-1-j] for j in range(n))
  stop=min(N//2,max(n+1,round(2/dt)));initial=np.linalg.lstsq(H[n:stop],y[n:stop]-forced[n:stop],rcond=None)[0];pred_warmup=forced+H@initial;ew=y-pred_warmup
  diagnostics.append(dict(initialSamples=n,initialSeconds=n*dt,freeRms=float(np.sqrt(np.mean(residual[n:]**2))),holdoutRms=float(np.sqrt(np.mean(residual[N*2//3:]**2)))))
  for block in range(N//count):
   lo=block*count;hi=lo+count;ft=[]
   for values in [r['InstrumentSP'],u,y,residual,ew,eq]:
    v=np.asarray(values[lo:hi]);ft.append(kernel@(v-v.mean()))
   zz,uu,yy,ee,ee_warmup,ee_equation=ft
   if np.any(abs(zz.conj()*uu)<=1e-24):raise ValueError('Unresolved excitation-linked input')
   if hi<=N*2//3:legacy.append(abs(yy/uu-g));paired.append(abs(ee/uu));warmup.append(abs(ee_warmup/uu));equation.append(abs(ee_equation/(aden*uu)))
   else:hold.append(abs(ee/uu));hold_warmup.append(abs(ee_warmup/uu));hold_equation.append(abs(ee_equation/(aden*uu)))
 return dict(dt=dt,hz=hz.tolist(),radii=dict(legacy=np.max(legacy,axis=0).tolist(),paired=np.max(paired,axis=0).tolist(),warmup=np.max(warmup,axis=0).tolist(),equation=np.max(equation,axis=0).tolist()),holdout=hold and np.array(hold).tolist(),holdoutWarmup=hold_warmup and np.array(hold_warmup).tolist(),holdoutEquation=hold_equation and np.array(hold_equation).tolist(),diagnostics=diagnostics)

def schur(a):
 a=np.array(a,dtype=float)
 for n in range(len(a)-1,0,-1):
  scale=abs(a[0])
  if not np.isfinite(scale) or scale==0:return False
  a=a/scale
  if abs(a[n])>=abs(a[0]):return False
  a=a[0]*a[:n]-a[n]*a[n:0:-1]
 return bool(np.isfinite(a[0]) and a[0]!=0)

def accept(m,points,bb,method):
 n=m['order'];a=np.array(m['coefficients']);den=np.r_[1,-a[:n]];num=np.r_[0,a[n:2*n]*m['Ys']/m['Us']];z=np.exp(-2j*np.pi*np.array(bb['hz'])*bb['dt']);poly=np.polynomial.polynomial.polyval;g=poly(z,num)/poly(z,den);w=np.array(bb['radii'][method]);out=[]
 for kp,ti,td in points:
  h=bb['dt'];cn=kp*(np.array([1+td/h,-td/h]) if ti>=250 else np.array([1+h/ti+td/h,-1-2*td/h,td/h]));cd=[1] if ti>=250 else [1,-1]
  d=np.convolve(den,cd);v=np.convolve(num,cn);l=max(len(d),len(v));stable=schur(np.pad(d,(0,l-len(d)))+np.pad(v,(0,l-len(v))));c=poly(z,cn)/poly(z,cd)
  out.append(bool(stable and np.max(w*abs(c/(1+g*c)))<1))
 return np.array(out)

def accept_many(m,points,bb):
 n=m['order'];a=np.array(m['coefficients']);den=np.r_[1,-a[:n]];num=np.r_[0,a[n:2*n]*m['Ys']/m['Us']];z=np.exp(-2j*np.pi*np.array(bb['hz'])*bb['dt']);poly=np.polynomial.polynomial.polyval;g=poly(z,num)/poly(z,den);names=['legacy','paired','heldout'];ws=[np.array(bb['radii']['legacy']),np.array(bb['radii']['paired']),np.max(bb['holdout'],axis=0)];out=[]
 names += ['warmup','equation','heldoutWarmup','heldoutEquation'];ws += [np.array(bb['radii'][k]) for k in ['warmup','equation']]+[np.max(bb[k],axis=0) for k in ['holdoutWarmup','holdoutEquation']]
 for kp,ti,td in points:
  h=bb['dt'];cn=kp*(np.array([1+td/h,-td/h]) if ti>=250 else np.array([1+h/ti+td/h,-1-2*td/h,td/h]));cd=[1] if ti>=250 else [1,-1]
  d=np.convolve(den,cd);v=np.convolve(num,cn);l=max(len(d),len(v));stable=schur(np.pad(d,(0,l-len(d)))+np.pad(v,(0,l-len(v))));c=poly(z,cn)/poly(z,cd);s=abs(c/(1+g*c));out.append([bool(stable and np.max(w*s)<1) for w in ws])
 out=np.array(out);return {name:out[:,i] for i,name in enumerate(names)}
