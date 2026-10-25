using System;using System.IO;using System.Linq;using System.Collections.Generic;using System.Text.Json;using System.Threading;using PIDSupporter;
internal static class PilotAmplitudeChecks
{
 const double Dt=.05;const int N=600;
 sealed class State {internal double X,C,U,I,E;}
 sealed class Data {internal TuneRecord Record=null!;internal double[] X=new double[N],Cross=new double[N];internal double OtherSat;}
 static void Step(State s,string mode,double exo){double beta=1.1;double other=NativeAutoTune.Clamp(.7*(beta*s.X+exo-s.C),-1,1);double act=(mode=="both"||mode=="tanh")?Math.Tanh(1.5*s.U)/1.5:s.U;double dx=-.35*s.X+2*act-((mode=="both"||mode=="cubic")?.025*s.X*s.X*s.X:0);s.C+=Dt*(-.15*s.C+2*other);s.X+=Dt*dx;}
 static double Exo(double t,int seed)=>.8*Math.Sin(2*Math.PI*.11*t+.63*seed)+.4*Math.Sin(2*Math.PI*.23*t+.37*seed);
 static double[] Wave(double[] pid,int seed){var random=new Random(70000+seed);var phases=Enumerable.Range(0,60).Select(_=>random.NextDouble()*2*Math.PI).ToArray();var weights=Enumerable.Range(1,60).Select(k=>{double hz=k/30.0,w=2*Math.PI*hz;return Math.Max(.5*Math.Max(0,Math.Min(1,(.4-hz)/.2)),1/Math.Sqrt(1+Math.Pow(w*pid[2]-1/(w*pid[1]),2)));}).ToArray();var r=new double[N];for(int i=0;i<N;i++){double t=i*Dt,edge=Math.Min(t,30-t),env=edge>=1?1:.5*(1-Math.Cos(Math.PI*edge));for(int k=0;k<60;k++)r[i]+=3*env*weights[k]*Math.Sin(2*Math.PI*(k+1)*t/30+phases[k])/20;}return r;}
 static Data Collect(string mode,double[] pid,int seed,double[] wave,double multiplier,State initial){var s=Copy(initial);var d=new Data();var r=new TuneRecord{U=new double[N],Y=new double[N],R=new double[N],T=new double[N],Integrals=new double[N],BaseSP=new double[N],InjectedSP=new double[N],InstrumentSP=new double[N],Pid=(double[])pid.Clone(),Lower=CollectionClip?-1:-1e6,Upper=CollectionClip?1:1e6,Amplitude=12,Speed=.5};d.Record=r;
 for(int i=0;i<N;i++){double t=30+i*Dt;double other=NativeAutoTune.Clamp(.7*(1.1*s.X+Exo(t,seed)-s.C),-1,1);if(Math.Abs(other)>.99999)d.OtherSat+=1.0/N;Step(s,mode,Exo(t,seed));double y=s.X-.8*s.C,reference=multiplier*wave[i],e=reference-y;s.U=Controller(pid,e,s.E,ref s.I,Dt,true,false);s.E=e;r.U[i]=s.U;r.Y[i]=y;r.R[i]=r.InjectedSP[i]=r.InstrumentSP[i]=reference;r.T[i]=(i+1)*Dt;r.Integrals[i]=s.I;d.X[i]=s.X;d.Cross[i]=-.8*s.C;}
 r.InitialIntegral=s.I;r.InitialError=s.E;return d;}
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

 static State Copy(State s)=>new State{X=s.X,C=s.C,U=s.U,I=s.I,E=s.E};
 sealed class Observation {public int amplitude;public double[] u=Array.Empty<double>(),y=Array.Empty<double>(),z=Array.Empty<double>();}
 sealed class PilotScore {public int amplitude;public double u,y,gain,residualY;}
 static readonly double[] Frequencies={.4,.8,1.2,1.6};
 static (State,Observation[]) Pilot(string mode,double[] pid,int seed){
  var s=new State();var obs=new List<Observation>();var rng=new Random(910000+seed);var phases=Frequencies.Select(_=>rng.NextDouble()*2*Math.PI).ToArray();
  foreach(int amp in new[]{1,2,4,4,2,1}){
   var o=new Observation{amplitude=amp,u=new double[100],y=new double[100],z=new double[100]};int block=obs.Count;
   for(int j=0;j<100;j++){
    double t=(block*100+j)*Dt;if(block*100+j>0)Step(s,mode,Exo(t,seed));double target=0;
    for(int k=0;k<4;k++){double w=2*Math.PI*Frequencies[k];target+=.3/Math.Sqrt(1+Math.Pow(w*pid[2]-1/(w*pid[1]),2))*Math.Sin(w*j*Dt+phases[k]);}
    target*=amp;double y=s.X-.8*s.C,e=target-y;s.U=Controller(pid,e,s.E,ref s.I,Dt,true,block*100+j==0);s.E=e;
    o.u[j]=s.U;o.y[j]=y;o.z[j]=target;
   }obs.Add(o);
  }return(s,obs.ToArray());
 }
 // Pure data-only policy. Plant type/state/coefficients are not arguments.
 static (int,PilotScore[]) Decide(Observation[] obs){
  var scores=new List<PilotScore>();
  foreach(int amp in new[]{1,2,4}){
   var us=new List<double>();var ys=new List<double>();var residuals=new List<double>();
   foreach(var o in obs.Where(o=>o.amplitude==amp)){
    var x=new double[80,10];for(int j=0;j<80;j++){double t=(j+20)*Dt;x[j,0]=1;x[j,1]=t;for(int k=0;k<4;k++){x[j,2+2*k]=Math.Sin(2*Math.PI*Frequencies[k]*t);x[j,3+2*k]=Math.Cos(2*Math.PI*Frequencies[k]*t);}}
    double Measure(double[] signal,out double residual){var target=signal.Skip(20).ToArray();var c=NativeLinear.LeastSquares(x,target,new WorkBudget(CancellationToken.None))!;double energy=0,err=0;for(int k=2;k<10;k++)energy+=c[k]*c[k]/2;for(int j=0;j<80;j++){double fit=0;for(int k=0;k<10;k++)fit+=x[j,k]*c[k];err+=Math.Pow(target[j]-fit,2);}residual=Math.Sqrt(err/80);return Math.Sqrt(energy);}
    us.Add(Measure(o.u,out _));ys.Add(Measure(o.y,out double er));residuals.Add(er);
   }
   double u=us.Average(),y=ys.Average();scores.Add(new PilotScore{amplitude=amp,u=u,y=y,gain=y/Math.Max(u,1e-12),residualY=residuals.Average()});
  }
  // Frozen heuristic: retain >=80% of the 1x pilot response/input ratio,
  // then maximize driven output among eligible amplitudes. No oracle threshold tuning.
  double floor=.8*scores[0].gain;var chosen=scores.Where(v=>v.gain>=floor&&double.IsFinite(v.gain)).OrderByDescending(v=>v.y).ThenBy(v=>v.amplitude).FirstOrDefault();
  return(chosen?.amplitude??1,scores.ToArray());
 }
 internal static void Run(string folder,int seeds){Directory.CreateDirectory(folder);using var output=new StreamWriter(Path.Combine(folder,"results.jsonl"));using var pilots=new StreamWriter(Path.Combine(folder,"pilots.jsonl"));int count=0;
 foreach(string mode in new[]{"linear","tanh","cubic","both"})foreach(double derivative in new[]{.2,1.0})for(int index=0;index<seeds;index++){
  int seed=100+index;double[] pid={.4,2,derivative};var pre=Pilot(mode,pid,seed);var decision=Decide(pre.Item2);
  pilots.WriteLine(JsonSerializer.Serialize(new{mode,derivative,seed,chosen=decision.Item1,scores=decision.Item2,observations=pre.Item2},new JsonSerializerOptions{IncludeFields=true}));pilots.Flush();
  var grid=(from p in new[]{.1,.3,.6,1.0} from ti in new[]{.5,2,8,250.0} from td in new[]{0.0,.2,.8} select new[]{p,ti,td}).ToList();grid.Add(pid);var candidates=grid.ToArray();int baseline=candidates.Length-1;var wave=Wave(pid,seed);PidSearch.Motion[]? truth=null;
  foreach(int multiplier in new[]{1,2,4}){
   var d=Collect(mode,pid,seed,wave,multiplier,pre.Item1);var r=d.Record;
   if(r.U.Any(u=>Math.Abs(u)>1)||r.T[N-1]!=30)throw new Exception("collection invariant");
   var selection=NativeLinear.Select(r,Dt,new WorkBudget(CancellationToken.None),_=>{});var m=selection.Model;
   var predict=new NativeLinear{Order=m.Order,Coefficients=(double[])m.Coefficients.Clone(),Us=m.Us,Ys=m.Ys};predict.Coefficients[2*predict.Order]=0;
   var predicted=candidates.Select(p=>Cost(mode,predict,p,seed)).ToArray();int raw=Choose(candidates,predicted);bool rejected=m.DcGain()<=0||!double.IsFinite(selection.Score)||predicted[baseline].Tracking>=1e10;int selected=rejected||predicted[raw].Tracking>=.95*predicted[baseline].Tracking?baseline:raw;
   truth??=candidates.Select(p=>Cost(mode,null,p,seed)).ToArray();double best=truth.Min(c=>c.Tracking);
   output.WriteLine(JsonSerializer.Serialize(new{mode,derivative,seed,multiplier,adaptiveSelected=decision.Item1==multiplier,order=m.Order,gain=m.DcGain(),score=selection.Score,closedY=selection.FinalY*Std(r.Y.Take(300).ToArray()),freeY=selection.FinalFree*Std(r.Y.Take(300).ToArray()),saturation=r.U.Count(u=>Math.Abs(u)>.99999)/(double)N,uRms=Rms(r.U),uPeak=r.U.Max(Math.Abs),rawPid=candidates[raw],selectedPid=candidates[selected],rawRegret=truth[raw].Tracking/best-1,regret=truth[selected].Tracking/best-1,rawWorse=truth[raw].Tracking>truth[baseline].Tracking*1.000001,guardWorse=truth[selected].Tracking>truth[baseline].Tracking*1.000001,bestCost=best,currentCost=truth[baseline].Tracking}));output.Flush();count++;
  }
  Console.WriteLine($"{count} fits: {mode} D={derivative} seed={seed}; chose{decision.Item1}");
 }
 }
 static double Std(double[] v){double mean=v.Average();return Math.Sqrt(v.Average(x=>(x-mean)*(x-mean)));}
}
