from production_noiseless_common import *
from scipy.linalg import fractional_matrix_power
ROOT=BASE/'output/production-noiseless-20261026';ROOT.mkdir(exist_ok=True)
plants=read(SRC/'plants-evaluation-only.json');cases=[]
for plant in plants:
 for condition in ['baseline','nonzero_initial','delay_300ms','weak_actuator','four_times_slower','coupled_disturbance']:
  for seed in [826,827]:
   p=json.loads(json.dumps(plant));speed=.25 if condition=='four_times_slower' else 1
   if p['type']=='linear':
    A=np.array(p['A']);B=np.array(p['B']);An=np.real_if_close(fractional_matrix_power(A,speed)).real;Bn=(np.eye(len(A))-An)@np.linalg.solve(np.eye(len(A))-A,B);p['A']=An.tolist();p['B']=Bn.tolist()
   else:p['p1']**=speed;p['p2']**=speed
   cases.append(dict(id=f'p{len(cases):03d}',plant=p,condition=condition,seed=seed,delay=6 if condition=='delay_300ms' else 1,gain=.2 if condition=='weak_actuator' else 1,initial=.5 if condition=='nonzero_initial' else 0,coupled=condition=='coupled_disturbance',originalPid=[.4,2.5,.2],baseSP=.3,dt=.05))
(ROOT/'cases-evaluation-only.json').write_text(json.dumps(cases,indent=2))
(ROOT/'protocol.json').write_text(json.dumps(dict(cases=len(cases),noise=0,production='Linked production AutoTuneSession, measurement prefix/postfix, AdaptiveExcitation, NativeLinear, history, PidSearch and EmpiricalFrequencyGate; no source edits or overrides. Fake host only supplies clock/PID state and synthetic measurements. Excitation seed injected by reflection for reproducibility.',conditions=['baseline','nonzero initial state','300ms command delay','20% input effectiveness','4x slower dynamics','physical coupled-output disturbance'],preparation='Actual repeating4s gain observation; test harness stops at180s if still preparing, reports timeout. Production itself has no such timeout.',collection='Actual fixed30s square-wave SP policy, limits[-1,1], independent carrier instrument, amplitude after preparation. One collection per fresh window; no accumulation in this sweep.',selection='Current full production identification and27-neighbor DE/hierarchy/frequency gate. No frozen model or research1SE/delay separation substituted.',grading='After freezing choices: same endpoint plant state, controller integral/error, command queue and physical disturbance phase for every candidate. Nine smooth-SP tasks (amplitudes .1,.3,.6 times tau .5,1.25,2), each worst of +/- sign over15s; separate matched-amplitude tau1.25 task and zero-action baseline. Compare pre-preparation PID, collection PID, recommendation, and finite grid reference best including all recommendations. Truth only simulator/evaluator; no coefficients or states passed to tuner.',scope='Synthetic plant families, not FTD physics/frame benchmark. No sensor noise; disturbances/nonlinearity remain. Slider limits unchanged. External truth unavailable to production tuner.'),indent=2))
print(len(cases))
