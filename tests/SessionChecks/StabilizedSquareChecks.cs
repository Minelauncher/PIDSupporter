using System;using System.IO;using System.Linq;using System.Collections.Generic;using System.Text.Json;using System.Threading;using PIDSupporter;
internal static class StabilizedSquareChecks
{
 static double SpeedFactor=1,DisturbanceScale=1;
 const double Dt=.05;const int N=600;
 sealed class State {internal double X,C,U,I,E;}
 sealed class Data {internal TuneRecord Record=null!;internal double[] X=new double[N],Cross=new double[N];internal double OtherSat;}
 static void Step(State s,string mode,double exo){double beta=1.1;double other=mode=="exact"?.7*(beta*s.X+exo-s.C):NativeAutoTune.Clamp(.7*(beta*s.X+exo-s.C),-1,1);double act=(mode=="both"||mode=="tanh")?Math.Tanh(1.5*s.U)/1.5:s.U;double dx=-.35*s.X+2*act-((mode=="both"||mode=="cubic")?.025*s.X*s.X*s.X:0);s.C+=Dt*SpeedFactor*(-.15*s.C+2*other);s.X+=Dt*SpeedFactor*dx;}
 static double Exo(double t,int seed)=>DisturbanceScale*(.8*Math.Sin(2*Math.PI*.11*t+.63*seed)+.4*Math.Sin(2*Math.PI*.23*t+.37*seed));
 static double[] Wave(double[] pid,int seed){var random=new Random(70000+seed);var phases=Enumerable.Range(0,60).Select(_=>random.NextDouble()*2*Math.PI).ToArray();var weights=Enumerable.Range(1,60).Select(k=>{double hz=k/30.0,w=2*Math.PI*hz;return Math.Max(.5*Math.Max(0,Math.Min(1,(.4-hz)/.2)),1/Math.Sqrt(1+Math.Pow(w*pid[2]-1/(w*pid[1]),2)));}).ToArray();var r=new double[N];for(int i=0;i<N;i++){double t=i*Dt,edge=Math.Min(t,30-t),env=edge>=1?1:.5*(1-Math.Cos(Math.PI*edge));for(int k=0;k<60;k++)r[i]+=3*env*weights[k]*Math.Sin(2*Math.PI*(k+1)*t/30+phases[k])/20;}return r;}
 sealed class Prepared {internal State S=null!;internal double[] Pid=null!;internal double Seconds;internal int Reductions;internal bool Passed;internal List<object> Windows=new();internal double[] Snapshot=null!;}
 static Prepared Ready=null!;
 static State Copy(State s)=>new State{X=s.X,C=s.C,U=s.U,I=s.I,E=s.E};
 static double[] Snapshot(State s)=>new[]{s.X,s.C,s.U,s.I,s.E};
 static void Prepare(string mode,double[] initialPid,int seed){
  var s=new State();var p=(double[])initialPid.Clone();var prep=new Prepared();int consecutive=0,steps=0;
  for(int w=0;w<8;w++){
   var u=new double[100];var y=new double[100];
   for(int k=0;k<100;k++,steps++){if(steps>0)Step(s,mode,Exo(steps*Dt,seed));y[k]=s.X-.8*s.C;double err=-y[k];s.U=Controller(p,err,s.E,ref s.I,Dt,true,steps==0);s.E=err;u[k]=s.U;}
   var tail=u.Skip(40).ToArray();double high=Math.Sqrt(Enumerable.Range(41,59).Average(k=>Math.Pow((u[k]-u[k-1])/2,2)));
   double sat=tail.Count(v=>Math.Abs(v)>.99999)/(double)tail.Length,yr=Rms(y.Skip(40).ToArray());
   bool ok=high<=.2 && sat<=.2 && yr<=2 && double.IsFinite(yr);
   prep.Windows.Add(new{seconds=steps*Dt,kp=p[0],highFrequencyRms=high,saturation=sat,yRms=yr,passed=ok});
   consecutive=ok?consecutive+1:0;
   if(consecutive>=2){prep.Passed=true;break;}
   if(!ok && w<7){double old=p[0];p[0]*=.7;s.I*=old/p[0];prep.Reductions++;}
  }
  prep.Seconds=steps*Dt;prep.Pid=p;Ready=prep;
  Pilot(s,mode,p,seed);
  prep.S=Copy(s);prep.Snapshot=Snapshot(s);
 }
 static double ChosenAmplitude,BoundU,BoundY,GU,GY,BU,BY;
 static bool Feasible;
 static double[] Pilot(State s,string mode,double[] pid,int seed) {
  var u=new double[200];var y=new double[200];var z=new double[200];
  for(int i=0;i<200;i++){Step(s,mode,Exo(Ready.Seconds+i*Dt,seed));z[i]=Math.Sin(2*Math.PI*.2*i*Dt)>=0?1:-1;
   y[i]=s.X-.8*s.C;double err=z[i]-y[i];s.U=Controller(pid,err,s.E,ref s.I,Dt,true,false);s.E=err;u[i]=s.U;
  }
  // Estimate response locked to the square-wave harmonics, separately per period.
  // Bounds are deterministic empirical envelopes, not probabilistic confidence bounds.
  double[] Envelope(double[] v){double gain=0,residual=0;
   for(int cycle=0;cycle<2;cycle++){var x=new double[100,7];var target=new double[100];
    for(int k=0;k<100;k++){double t=k*Dt;x[k,0]=1;int col=1;foreach(int h in new[]{1,3,5}){x[k,col++]=Math.Sin(2*Math.PI*.2*h*t);x[k,col++]=Math.Cos(2*Math.PI*.2*h*t);}target[k]=v[cycle*100+k];}
    var co=NativeLinear.LeastSquares(x,target,new WorkBudget(CancellationToken.None))!;
    double linked=0,unlinked=0;for(int k=0;k<100;k++){double pred=0;for(int j=1;j<7;j++)pred+=x[k,j]*co[j];linked+=pred*pred;unlinked+=Math.Pow(target[k]-pred,2);}
    gain=Math.Max(gain,Math.Sqrt(linked/100));residual=Math.Max(residual,Math.Sqrt(unlinked/100));
   }return new[]{gain,residual};
  }
  var ue=Envelope(u);var ye=Envelope(y);GU=ue[0];BU=ue[1];GY=ye[0];BY=ye[1];
  // Maximize a^2 (linked excitation power), subject to empirical RMS envelopes.
  // u_RMS <= .5, y_RMS <= 1; local amplitude trust interval [0,2] around pilot a=1.
  double a=Math.Min(2,Math.Min((.5-BU)/Math.Max(GU,1e-12),(1-BY)/Math.Max(GY,1e-12)));
  Feasible=a>0;ChosenAmplitude=Math.Max(0,a);BoundU=BU+ChosenAmplitude*GU;BoundY=BY+ChosenAmplitude*GY;
  return new[]{ChosenAmplitude};
 }
 static Data Collect(string mode,double[] pid,int seed,double[] wave,double target,double fixedAmplitude=0){var s=Copy(Ready.S);if(!pid.SequenceEqual(Ready.Pid))throw new Exception("PID mismatch");var d=new Data();var r=new TuneRecord{U=new double[N],Y=new double[N],R=new double[N],T=new double[N],Integrals=new double[N],BaseSP=new double[N],InjectedSP=new double[N],InstrumentSP=new double[N],Pid=(double[])pid.Clone(),Lower=-1,Upper=1,Amplitude=12,Speed=.5};d.Record=r;double amplitude=fixedAmplitude<0?ChosenAmplitude:fixedAmplitude>0?fixedAmplitude:1,energy=0;
 for(int i=0;i<N;i++){
  // Uses ONLY completed input samples; never reads current/future plant output.
  if(fixedAmplitude==0&&i>0&&i%20==0){double recent=0;for(int j=i-20;j<i;j++)recent+=r.U[j]*r.U[j]*Dt;double desired=Math.Max(0,(target-energy)/(30-i*Dt));double factor=Math.Sqrt(desired/Math.Max(recent,1e-8));amplitude=NativeAutoTune.Clamp(amplitude*NativeAutoTune.Clamp(factor,.8,1.25),.125,8);}
  double t=i*Dt;Step(s,mode,Exo(t+Ready.Seconds+10,seed));double y=s.X-.8*s.C,reference=amplitude*wave[i],e=reference-y;s.U=Controller(pid,e,s.E,ref s.I,Dt,true,false);s.E=e;energy+=s.U*s.U*Dt;
  r.U[i]=s.U;r.Y[i]=y;r.R[i]=r.InjectedSP[i]=r.InstrumentSP[i]=reference;r.T[i]=(i+1)*Dt;r.Integrals[i]=s.I;d.X[i]=amplitude;d.Cross[i]=energy;
 }r.InitialIntegral=s.I;r.InitialError=s.E;return d;}
 static double Rms(double[] v)=>Math.Sqrt(v.Average(x=>x*x));
 static PidSearch.Motion Cost(string mode,NativeLinear? model,double[] pid,int seed){double tracking=0,slew=0,effort=0;
 foreach(double speed in new[]{.5,1.5,3.0})foreach(double amp in new[]{.2,.5,1.0}){var s=new State();var state=model==null?null:new double[2*model.Order];double sp=0,last=0;
 for(int i=0;i<N;i++){if(i>0&&model==null)Step(s,mode,Exo(i*Dt,seed+1000));double y=model==null?s.X-.8*s.C:model.Next(state!);sp+=(amp-sp)*(1-Math.Exp(-Dt/speed));double e=sp-y;double u=Controller(pid,e,s.E,ref s.I,Dt,true,i==0);s.E=e;s.U=u;if(model!=null)model.Push(state!,y,u);tracking+=e*e;slew+=Math.Pow((u-last)/Dt,2);effort+=u*u;last=u;if(!double.IsFinite(tracking)||tracking>1e12)return new PidSearch.Motion{Tracking=1e12,Slew=1e12,Effort=1e12};}}
 return new PidSearch.Motion{Tracking=tracking/(9*N),Slew=slew/(9*N),Effort=effort/(9*N)};}
 static int Choose(double[][] candidates,PidSearch.Motion[] costs){var rows=candidates.Select((p,i)=>new PidSearch.Ranked{Pid=p,Robust=costs[i].Tracking,Nominal=costs[i].Tracking,Slew=costs[i].Slew,Effort=costs[i].Effort}).ToList();return Array.IndexOf(candidates,PidSearch.ChooseHierarchy(rows).Pid);}

 internal static bool CollectionClip=true;
 internal static double Controller(double[] pid,double error,double previous,ref double integral,double dt,bool clip,bool first){
  double bounded=NativeAutoTune.PidStep(pid,error,previous,ref integral,dt,-1,1,first);
  if(clip||first||dt==0)return bounded;
  float kp=(float)pid[0],ti=Math.Max((float)pid[1],1e-9f),td=(float)pid[2],e=(float)error;
  return kp*(e+(float)integral/ti+((e-(float)previous)/(float)dt)*td);
 }

 static double[] Shape(double[] pid,int seed,string shape){
  var original=Wave(pid,seed);double peak=original.Max(Math.Abs);var a=new double[N];
  if(shape=="multisine")a=original;
  else {double phase=new Random(820000+seed).NextDouble();double value=0,alpha=1-Math.Exp(-Dt/.25);
   for(int i=-600;i<N;i++){double t=i*Dt;double target=Math.Sin(2*Math.PI*(.2*t+phase))>=0?1:-1;value+=alpha*(target-value);if(i>=0){double edge=Math.Min(t,30-t),env=edge>=1?1:.5*(1-Math.Cos(Math.PI*edge));a[i]=env*(shape=="square"?target:value);}}
  }
  double mean=a.Average();for(int i=0;i<N;i++)a[i]-=mean;double scale=(new Random(912345+seed).Next(2)==0?-1:1)/a.Max(Math.Abs);for(int i=0;i<N;i++)a[i]*=scale;return a;
 }

 static double Fixed(string folder,double d){using var doc=JsonDocument.Parse(File.ReadAllText(Path.Combine(folder,"frozen-choice.json")));return doc.RootElement.GetProperty("choices").EnumerateArray().First(x=>x.GetProperty("D").GetDouble()==d).GetProperty("amplitude").GetDouble();}
 internal static void Run(string phase,string folder,int seeds){Directory.CreateDirectory(folder);if(phase=="bias"){Bias(folder,seeds);return;}bool calibration=phase=="calibration";using var output=new StreamWriter(Path.Combine(folder,phase+".jsonl"));int count=0;
 foreach(string mode in new[]{"linear","tanh","cubic","both"})foreach(double derivative in new[]{.2,1.0})for(int index=0;index<seeds;index++){
  int seed=(calibration?500:600)+index;SpeedFactor=calibration?(index%2==0?.9:1.1):(index%2==0?.75:1.25);DisturbanceScale=calibration?1:(index%4<2?.7:1.3);
  double[] originalPid={.4,2,derivative};Prepare(mode,originalPid,seed);var pid=Ready.Pid;var grid=(from p in new[]{.1,.3,.6,1.0} from ti in new[]{.5,2,8,250.0} from td in new[]{0.0,.2,.8} select new[]{p,ti,td}).ToList();grid.Add(pid);var candidates=grid.ToArray();int baseline=candidates.Length-1;PidSearch.Motion[]? truth=null;
  double[] settings=calibration?new[]{.25,.5,.75,1,1.5,2,3,4}:new[]{Fixed(folder,derivative),0,-1};
  foreach(double amplitude in settings){string strategy=amplitude<0?"designed":amplitude==0?"adaptive":"fixed";var wave=Shape(pid,seed,"square");var d=Collect(mode,pid,seed,wave,7.5,amplitude);var r=d.Record;
   foreach(string instrument in calibration?new[]{"injected"}:new[]{"injected","carrier"}){
    r.InstrumentSP=instrument=="carrier"?(double[])wave.Clone():(double[])r.InjectedSP.Clone();
    if(r.U.Any(u=>Math.Abs(u)>1)||r.T[N-1]!=30)throw new Exception("collection invariant");
    if(!Ready.Passed || (amplitude<0&&!Feasible)){
     truth??=candidates.Select(p=>Cost(mode,null,p,seed)).ToArray();double opt=truth.Min(c=>c.Tracking);
     output.WriteLine(JsonSerializer.Serialize(new{mode,derivative,seed,strategy,amplitude,instrument,stabilized=Ready.Passed,kp=pid[0],preSeconds=Ready.Seconds,reductions=Ready.Reductions,preWindows=Ready.Windows,startState=Ready.Snapshot,speed=SpeedFactor,disturbance=DisturbanceScale,order=0,gain=0.0,noFit=true,actualAmplitude=d.X.Average(),feasible=Feasible,predictedURms=BoundU,predictedYRms=BoundY,actualYRms=Rms(r.Y),GU,GY,BU,BY,score=1e12,closedY=(double?)null,freeY=(double?)null,uEnergy=r.U.Sum(u=>u*u)*Dt,saturation=r.U.Count(u=>Math.Abs(u)>.99999)/(double)N,uPeak=r.U.Max(Math.Abs),rawRegret=truth[baseline].Tracking/opt-1,regret=truth[baseline].Tracking/opt-1,relativeCurrent=1.0,rawRelativeCurrent=1.0,rawWorse=false,guardWorse=false,rawPid=pid,selectedPid=pid}));
     output.Flush();count++;continue;
    }
    var selection=NativeLinear.Select(r,Dt,new WorkBudget(CancellationToken.None),_=>{});var m=selection.Model;
    var predict=new NativeLinear{Order=m.Order,Coefficients=(double[])m.Coefficients.Clone(),Us=m.Us,Ys=m.Ys};predict.Coefficients[2*predict.Order]=0;
    var predicted=candidates.Select(p=>Cost(mode,predict,p,seed)).ToArray();int raw=Choose(candidates,predicted);bool rejected=m.DcGain()<=0||!double.IsFinite(selection.Score)||predicted[baseline].Tracking>=1e10;int selected=rejected||predicted[raw].Tracking>=.95*predicted[baseline].Tracking?baseline:raw;
    truth??=candidates.Select(p=>Cost(mode,null,p,seed)).ToArray();double best=truth.Min(c=>c.Tracking);
    output.WriteLine(JsonSerializer.Serialize(new{mode,derivative,seed,strategy,amplitude,instrument,stabilized=Ready.Passed,kp=pid[0],preSeconds=Ready.Seconds,reductions=Ready.Reductions,preWindows=Ready.Windows,startState=Ready.Snapshot,speed=SpeedFactor,disturbance=DisturbanceScale,order=m.Order,gain=m.DcGain(),actualAmplitude=d.X.Average(),feasible=Feasible,predictedURms=BoundU,predictedYRms=BoundY,actualYRms=Rms(r.Y),GU,GY,BU,BY,score=selection.Score,closedY=selection.FinalY*Std(r.Y.Take(300).ToArray()),freeY=selection.FinalFree*Std(r.Y.Take(300).ToArray()),uEnergy=r.U.Sum(u=>u*u)*Dt,saturation=r.U.Count(u=>Math.Abs(u)>.99999)/(double)N,uPeak=r.U.Max(Math.Abs),rawRegret=truth[raw].Tracking/best-1,regret=truth[selected].Tracking/best-1,relativeCurrent=truth[selected].Tracking/truth[baseline].Tracking,rawRelativeCurrent=truth[raw].Tracking/truth[baseline].Tracking,rawWorse=truth[raw].Tracking>truth[baseline].Tracking*1.000001,guardWorse=truth[selected].Tracking>truth[baseline].Tracking*1.000001,rawPid=candidates[raw],selectedPid=candidates[selected]}));output.Flush();count++;
   }
  }Console.WriteLine($"{phase} {count}: {mode} D={derivative} seed={seed}");
 }
 }
 static double[] FitExact(TuneRecord r){var budget=new WorkBudget(CancellationToken.None);var q=NativeLinear.Instruments(r.InstrumentSP!,40,300,40,budget);int rank=q.GetLength(1);var x=new double[rank,5];var y=new double[rank];
 for(int i=40;i<300;i++){double[] row={r.Y[i-1],r.Y[i-2],r.U[i-1],r.U[i-2],1};for(int j=0;j<rank;j++){double z=q[i-40,j];y[j]+=z*r.Y[i];for(int k=0;k<5;k++)x[j,k]+=z*row[k];}}
 var scale=new double[5];for(int k=0;k<5;k++){for(int j=0;j<rank;j++)scale[k]+=x[j,k]*x[j,k];scale[k]=Math.Max(1e-12,Math.Sqrt(scale[k]));for(int j=0;j<rank;j++)x[j,k]/=scale[k];}
 var co=NativeLinear.LeastSquares(x,y,budget)??throw new Exception("singular exact IV");for(int k=0;k<5;k++)co[k]/=scale[k];return co;
 }
 static double[] Exact(double speed){double a=1-.05*speed*.35,d=1-.05*speed*1.55,e=.05*speed*1.54,b=.1*speed;return new[]{a+d,-a*d,b,-b*(d+.8*e),0.0};}
 static void Bias(string folder,int seeds){
  foreach(double speed in new[]{.75,1.25}){SpeedFactor=speed;DisturbanceScale=0;double[] p={.4,2,.2};Prepare("exact",p,999);p=Ready.Pid;var t=Collect("exact",p,999,Shape(p,999,"square"),7.5,1).Record;
   if(FitExact(t).Zip(Exact(speed),(x,y)=>Math.Abs(x-y)).Max()>1e-7)throw new Exception("exact fixture mismatch");}
  DisturbanceScale=1;using var output=new StreamWriter(Path.Combine(folder,"bias.jsonl"));
  foreach(double speed in new[]{.75,1.25})foreach(double derivative in new[]{.2,1.0})for(int k=0;k<seeds;k++){
   SpeedFactor=speed;int seed=2000+k;double[] initial={.4,2,derivative};Prepare("exact",initial,seed);var pid=Ready.Pid;var wave=Shape(pid,seed,"square");
   foreach(double amp in new[]{Fixed(folder,derivative),0,-1}){var r=Collect("exact",pid,seed,wave,7.5,amp).Record;
    foreach(string instrument in new[]{"injected","carrier"}){r.InstrumentSP=instrument=="carrier"?(double[])wave.Clone():(double[])r.InjectedSP.Clone();
     bool noFit=!Ready.Passed||(amp<0&&!Feasible);double[]? co=noFit?null:FitExact(r);
     var exact=Exact(speed);var residual=new double[260];for(int i=40;i<300;i++)residual[i-40]=r.Y[i]-exact[0]*r.Y[i-1]-exact[1]*r.Y[i-2]-exact[2]*r.U[i-1]-exact[3]*r.U[i-2];
     double mean=residual.Average(),er=Std(residual);var moments=new List<double>();var rawMoments=new List<double>();
     foreach(int lag in new[]{0,1,5,10,20,40}){var z=Enumerable.Range(40,260).Select(i=>r.InstrumentSP[i-lag]).ToArray();double zm=z.Average(),zs=Std(z);rawMoments.Add(z.Zip(residual,(zz,v)=>zz*v).Average());moments.Add(z.Zip(residual,(zz,v)=>(zz-zm)*(v-mean)).Average()/Math.Max(1e-12,zs*er));}
     output.WriteLine(JsonSerializer.Serialize(new{speed,derivative,seed,strategy=amp<0?"designed":amp==0?"adaptive":"fixed",amplitude=amp,instrument,noFit,kp=pid[0],stabilized=Ready.Passed,reductions=Ready.Reductions,preSeconds=Ready.Seconds,exact,coefficients=co,moments,rawMoments,uEnergy=r.U.Sum(u=>u*u)*Dt}));output.Flush();
    }
   }if(k%32==31)Console.WriteLine($"bias speed={speed} D={derivative} seeds={k+1}");
  }
 }
 static double Std(double[] v){double mean=v.Average();return Math.Sqrt(v.Average(x=>(x-mean)*(x-mean)));}
}
