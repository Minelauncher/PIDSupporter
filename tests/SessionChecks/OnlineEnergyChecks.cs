using System;using System.IO;using System.Linq;using System.Collections.Generic;using System.Text.Json;using System.Threading;using PIDSupporter;
internal static class OnlineEnergyChecks
{
 const double Dt=.05;const int N=600;
 sealed class State {internal double X,C,U,I,E;}
 sealed class Data {internal TuneRecord Record=null!;internal double[] X=new double[N],Cross=new double[N];internal double OtherSat;}
 static void Step(State s,string mode,double exo){double beta=1.1;double other=NativeAutoTune.Clamp(.7*(beta*s.X+exo-s.C),-1,1);double act=(mode=="both"||mode=="tanh")?Math.Tanh(1.5*s.U)/1.5:s.U;double dx=-.35*s.X+2*act-((mode=="both"||mode=="cubic")?.025*s.X*s.X*s.X:0);s.C+=Dt*(-.15*s.C+2*other);s.X+=Dt*dx;}
 static double Exo(double t,int seed)=>.8*Math.Sin(2*Math.PI*.11*t+.63*seed)+.4*Math.Sin(2*Math.PI*.23*t+.37*seed);
 static double[] Wave(double[] pid,int seed){var random=new Random(70000+seed);var phases=Enumerable.Range(0,60).Select(_=>random.NextDouble()*2*Math.PI).ToArray();var weights=Enumerable.Range(1,60).Select(k=>{double hz=k/30.0,w=2*Math.PI*hz;return Math.Max(.5*Math.Max(0,Math.Min(1,(.4-hz)/.2)),1/Math.Sqrt(1+Math.Pow(w*pid[2]-1/(w*pid[1]),2)));}).ToArray();var r=new double[N];for(int i=0;i<N;i++){double t=i*Dt,edge=Math.Min(t,30-t),env=edge>=1?1:.5*(1-Math.Cos(Math.PI*edge));for(int k=0;k<60;k++)r[i]+=3*env*weights[k]*Math.Sin(2*Math.PI*(k+1)*t/30+phases[k])/20;}return r;}
 static Data Collect(string mode,double[] pid,int seed,double[] wave,double target){var s=new State();var d=new Data();var r=new TuneRecord{U=new double[N],Y=new double[N],R=new double[N],T=new double[N],Integrals=new double[N],BaseSP=new double[N],InjectedSP=new double[N],InstrumentSP=new double[N],Pid=(double[])pid.Clone(),Lower=-1,Upper=1,Amplitude=12,Speed=.5};d.Record=r;double amplitude=1,energy=0;
 for(int i=0;i<N;i++){
  // Uses ONLY completed input samples; never reads current/future plant output.
  if(i>0&&i%20==0){double recent=0;for(int j=i-20;j<i;j++)recent+=r.U[j]*r.U[j]*Dt;double desired=Math.Max(0,(target-energy)/(30-i*Dt));double factor=Math.Sqrt(desired/Math.Max(recent,1e-8));amplitude=NativeAutoTune.Clamp(amplitude*NativeAutoTune.Clamp(factor,.8,1.25),.125,8);}
  double t=i*Dt;if(i>0)Step(s,mode,Exo(t,seed));double y=s.X-.8*s.C,reference=amplitude*wave[i],e=reference-y;s.U=Controller(pid,e,s.E,ref s.I,Dt,true,i==0);s.E=e;energy+=s.U*s.U*Dt;
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
  double mean=a.Average();for(int i=0;i<N;i++)a[i]-=mean;double scale=peak/a.Max(Math.Abs);for(int i=0;i<N;i++)a[i]*=scale;return a;
 }
 internal static void Run(string folder,int seeds){Directory.CreateDirectory(folder);using var output=new StreamWriter(Path.Combine(folder,"results.jsonl"));int count=0;
 foreach(string mode in new[]{"linear","tanh","cubic","both"})foreach(double derivative in new[]{.2,1.0})for(int index=0;index<seeds;index++){
  int seed=100+index;double[] pid={.4,2,derivative};var grid=(from p in new[]{.1,.3,.6,1.0} from ti in new[]{.5,2,8,250.0} from td in new[]{0.0,.2,.8} select new[]{p,ti,td}).ToList();grid.Add(pid);var candidates=grid.ToArray();int baseline=candidates.Length-1;PidSearch.Motion[]? truth=null;
  foreach(int multiplier in new[]{1,2,4})foreach(string shape in new[]{"multisine","square","smooth"}){
   var wave=Shape(pid,seed,shape);double targetEnergy=multiplier==1?2.4:multiplier==2?7.5:15;
   var d=Collect(mode,pid,seed,wave,targetEnergy);var r=d.Record;
   foreach(string instrument in new[]{"injected","carrier"}){
   r.InstrumentSP=instrument=="carrier"?(double[])wave.Clone():(double[])r.InjectedSP.Clone();
   if(r.U.Any(u=>Math.Abs(u)>1)||r.T[N-1]!=30)throw new Exception("collection invariant");
   int instrumentRank=0;AutoTuneDiagnostics.Sink=(kind,value)=>{if(kind=="linearInstruments")instrumentRank=(int)value.GetType().GetProperty("rank")!.GetValue(value)!;};
   var selection=NativeLinear.Select(r,Dt,new WorkBudget(CancellationToken.None),_=>{});AutoTuneDiagnostics.Sink=null;var m=selection.Model;
   var predict=new NativeLinear{Order=m.Order,Coefficients=(double[])m.Coefficients.Clone(),Us=m.Us,Ys=m.Ys};predict.Coefficients[2*predict.Order]=0;
   var predicted=candidates.Select(p=>Cost(mode,predict,p,seed)).ToArray();int raw=Choose(candidates,predicted);bool rejected=m.DcGain()<=0||!double.IsFinite(selection.Score)||predicted[baseline].Tracking>=1e10;int selected=rejected||predicted[raw].Tracking>=.95*predicted[baseline].Tracking?baseline:raw;
   truth??=candidates.Select(p=>Cost(mode,null,p,seed)).ToArray();double best=truth.Min(c=>c.Tracking);
   output.WriteLine(JsonSerializer.Serialize(new{mode,derivative,seed,multiplier,shape,instrument,targetEnergy,finalAmplitude=d.X[N-1],trainEnergy=r.U.Take(300).Sum(u=>u*u)*Dt,order=m.Order,gain=m.DcGain(),instrumentRank,score=selection.Score,closedY=selection.FinalY*Std(r.Y.Take(300).ToArray()),freeY=selection.FinalFree*Std(r.Y.Take(300).ToArray()),saturation=r.U.Count(u=>Math.Abs(u)>.99999)/(double)N,uEnergy=r.U.Sum(u=>u*u)*Dt,uRms=Rms(r.U),uPeak=r.U.Max(Math.Abs),spRms=Rms(r.R),spPeak=r.R.Max(Math.Abs),yRms=Rms(r.Y),rawPid=candidates[raw],selectedPid=candidates[selected],rawRegret=truth[raw].Tracking/best-1,regret=truth[selected].Tracking/best-1,rawWorse=truth[raw].Tracking>truth[baseline].Tracking*1.000001,guardWorse=truth[selected].Tracking>truth[baseline].Tracking*1.000001,bestCost=best,currentCost=truth[baseline].Tracking}));output.Flush();count++;
   if(mode=="both"&&seed==100&&multiplier==2&&instrument=="carrier")File.WriteAllText(Path.Combine(folder,$"trace-D{derivative}-{shape}.json"),JsonSerializer.Serialize(new{r.R,r.U,r.Y,amplitudes=d.X,energy=d.Cross}));
  }
  }
  Console.WriteLine($"{count} fits: {mode} D={derivative} seed={seed}");
 }
 }
 static double Std(double[] v){double mean=v.Average();return Math.Sqrt(v.Average(x=>(x-mean)*(x-mean)));}
}
