import os
os.environ["OPENBLAS_NUM_THREADS"]="1"
from pathlib import Path
import json, hashlib, itertools
import numpy as np
BASE=Path(__file__).resolve().parent
SRC=BASE/"ProductionNoiseless/fixtures"
def read(p):return json.loads(p.read_text(encoding="utf-8"))

def neighbors(p):
 out=[]
 for signs in itertools.product([-1,0,1],repeat=3):
  if (p[1]>=250 and signs[1]) or (p[2]==0 and signs[2]):continue
  q=list(p)
  for j,s in enumerate(signs):
   if s:q[j]=round(np.clip(p[j]+s*max([.001,.1,.1][j],.1*p[j]),[.001,.1,0][j],[1,249.9,100][j]),3 if j==0 else 1)
  out.append(tuple(q))
 return list(dict.fromkeys([tuple(p)]+out))
def advance(spec,x,u):
    if spec['type']=='nonlinear':
        v=x[:,0];x[:,0]=spec['p1']*v+(1-spec['p1'])*(u-.2*v*abs(v));x[:,1]=spec['p2']*x[:,1]+(1-spec['p2'])*x[:,0];return x,x[:,1].copy()
    x=x@np.array(spec['A']).T+u[:,None]*np.array(spec['B']);return x,x@np.array(spec['C'])
