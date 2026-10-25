using System;using System.IO;using System.Linq;using System.Collections.Generic;using System.Text.Json;using System.Threading;using PIDSupporter;
internal static class QuietOtherAxesChecks
{
 const double Dt=.05;const int N=600;
 sealed class State {internal double X,C,U,I,E;}
 sealed class Data {internal TuneRecord Record=null!;internal double[] X=new double[N],Cross=new double[N];internal double OtherSat;}
 static void Step(State s,string mode,double exo){double beta=mode=="independent"?0:1.1;double other=NativeAutoTune.Clamp(.7*(beta*s.X+exo-s.C),-1,1);double act=mode=="nonlinear"?Math.Tanh(1.5*s.U)/1.5:s.U;double dx=-.35*s.X+2*act-(mode=="nonlinear"?.025*s.X*s.X*s.X:0);s.C+=Dt*(-.15*s.C+2*other);s.X+=Dt*dx;}
 static double Exo(double t,int seed)=>.8*Math.Sin(2*Math.PI*.11*t+.63*seed)+.4*Math.Sin(2*Math.PI*.23*t+.37*seed);
 static double[] Wave(double[] pid,int seed){var random=new Random(70000+seed);var phases=Enumerable.Range(0,60).Select(_=>random.NextDouble()*2*Math.PI).ToArray();var weights=Enumerable.Range(1,60).Select(k=>{double hz=k/30.0,w=2*Math.PI*hz;return Math.Max(.5*Math.Max(0,Math.Min(1,(.4-hz)/.2)),1/Math.Sqrt(1+Math.Pow(w*pid[2]-1/(w*pid[1]),2)));}).ToArray();var r=new double[N];for(int i=0;i<N;i++){double t=i*Dt,edge=Math.Min(t,30-t),env=edge>=1?1:.5*(1-Math.Cos(Math.PI*edge));for(int k=0;k<60;k++)r[i]+=3*env*weights[k]*Math.Sin(2*Math.PI*(k+1)*t/30+phases[k])/20;}return r;}
 static Data Collect(string mode,double[] pid,int seed,double[] wave,double motionScale){var s=new State();var d=new Data();var r=new TuneRecord{U=new double[N],Y=new double[N],R=new double[N],T=new double[N],Integrals=new double[N],BaseSP=new double[N],InjectedSP=new double[N],InstrumentSP=new double[N],Pid=(double[])pid.Clone(),Lower=-1,Upper=1,Amplitude=12,Speed=.5};d.Record=r;
 for(int i=0;i<N;i++){double t=i*Dt;double other=NativeAutoTune.Clamp(.7*((mode=="independent"?0:1.1)*s.X+motionScale*Exo(t,seed)-s.C),-1,1);if(Math.Abs(other)>.99999)d.OtherSat+=1.0/N;if(i>0)Step(s,mode,motionScale*Exo(t,seed));double y=s.X-.8*s.C,reference=wave[i],e=reference-y;s.U=NativeAutoTune.PidStep(pid,e,s.E,ref s.I,Dt,-1,1,i==0);s.E=e;r.U[i]=s.U;r.Y[i]=y;r.R[i]=r.InjectedSP[i]=r.InstrumentSP[i]=reference;r.T[i]=(i+1)*Dt;r.Integrals[i]=s.I;d.X[i]=s.X;d.Cross[i]=-.8*s.C;}
 r.InitialIntegral=s.I;r.InitialError=s.E;return d;}
 static double Rms(double[] v)=>Math.Sqrt(v.Average(x=>x*x));
 static PidSearch.Motion Cost(string mode,NativeLinear? model,double[] pid,int seed,double background=1){double tracking=0,slew=0,effort=0;
 foreach(double speed in new[]{.5,1.5,3.0})foreach(double amp in new[]{.2,.5,1.0}){var s=new State();var state=model==null?null:new double[2*model.Order];double sp=0,last=0;
 for(int i=0;i<N;i++){if(i>0&&model==null)Step(s,mode,background*Exo(i*Dt,seed+1000));double y=model==null?s.X-.8*s.C:model.Next(state!);sp+=(amp-sp)*(1-Math.Exp(-Dt/speed));double e=sp-y;double u=NativeAutoTune.PidStep(pid,e,s.E,ref s.I,Dt,-1,1,i==0);s.E=e;s.U=u;if(model!=null)model.Push(state!,y,u);tracking+=e*e;slew+=Math.Pow((u-last)/Dt,2);effort+=u*u;last=u;if(!double.IsFinite(tracking)||tracking>1e12)return new PidSearch.Motion{Tracking=1e12,Slew=1e12,Effort=1e12};}}
 return new PidSearch.Motion{Tracking=tracking/(9*N),Slew=slew/(9*N),Effort=effort/(9*N)};}
 static int Choose(double[][] candidates,PidSearch.Motion[] costs){var rows=candidates.Select((p,i)=>new PidSearch.Ranked{Pid=p,Robust=costs[i].Tracking,Nominal=costs[i].Tracking,Slew=costs[i].Slew,Effort=costs[i].Effort}).ToList();return Array.IndexOf(candidates,PidSearch.ChooseHierarchy(rows).Pid);}
 internal static void Run(string folder,int seeds){Directory.CreateDirectory(folder);using var output=new StreamWriter(Path.Combine(folder,"results.jsonl"));int count=0;
 foreach(string mode in new[]{"independent","reactive","nonlinear"})foreach(double derivative in new[]{.2,1.0})for(int seed=0;seed<seeds;seed++){
  double[] pid={.4,2,derivative};var grid=(from p in new[]{.1,.3,.6,1.0} from ti in new[]{.5,2,8,250.0} from td in new[]{0.0,.2,.8} select new[]{p,ti,td}).ToList();grid.Add(pid);var candidates=grid.ToArray();int baseline=candidates.Length-1;
  var truth=candidates.Select(p=>Cost(mode,null,p,seed)).ToArray();double trueBest=truth.Min(c=>c.Tracking);var wave=Wave(pid,seed);
  foreach(double motionScale in new[]{1.0,.5,.25,0}){
   var d=Collect(mode,pid,seed,wave,motionScale);var r=d.Record;
   if(r.U.Any(u=>Math.Abs(u)>1)||r.T[N-1]!=30)throw new Exception("collection invariant");
   var selection=NativeLinear.Select(r,Dt,new WorkBudget(CancellationToken.None),_=>{});var model=selection.Model;
   // Incremental model at a common zero operating point; same evaluation references at all amplitudes.
   var predict=new NativeLinear{Order=model.Order,Coefficients=(double[])model.Coefficients.Clone(),Us=model.Us,Ys=model.Ys};predict.Coefficients[2*predict.Order]=0;
   var predicted=candidates.Select(p=>Cost(mode,predict,p,seed)).ToArray();int raw=Choose(candidates,predicted);bool rejected=model.DcGain()<=0||!double.IsFinite(selection.FinalWorst)||predicted[baseline].Tracking>=1e10;int selected=rejected||predicted[raw].Tracking>=.95*predicted[baseline].Tracking?baseline:raw;
   var matchedTruth=motionScale==1?truth:candidates.Select(p=>Cost(mode,null,p,seed,motionScale)).ToArray();double matchedBest=matchedTruth.Min(c=>c.Tracking);
   double[] Rank(double[] values)=>values.Select(v=>(double)values.Count(w=>w<v)+.5*values.Count(w=>w==v)).ToArray();var pr=Rank(predicted.Select(c=>c.Tracking).ToArray());var tr=Rank(truth.Select(c=>c.Tracking).ToArray());double pm=pr.Average(),tm=tr.Average();double rankCorrelation=pr.Zip(tr,(a,b)=>(a-pm)*(b-tm)).Sum()/Math.Sqrt(pr.Sum(a=>Math.Pow(a-pm,2))*tr.Sum(a=>Math.Pow(a-tm,2)));
   var row=new{mode,derivative,seed,motionScale,order=model.Order,gain=model.DcGain(),selection=selection.Score,free=selection.FinalFree,closedY=selection.FinalY,closedU=selection.FinalU,freePhysical=selection.FinalFree*Std(r.Y.Take(N/2).ToArray()),spRms=Rms(r.R),uPeak=r.U.Max(Math.Abs),uEnergy=r.U.Sum(v=>v*v)*Dt,saturation=r.U.Count(v=>Math.Abs(v)>.99999)/(double)N,otherSaturation=d.OtherSat,outputRms=Rms(r.Y),otherStateRms=Rms(d.Cross)/.8,otherRateRms=Math.Sqrt(Enumerable.Range(1,N-1).Average(i=>Math.Pow((d.Cross[i]-d.Cross[i-1])/(.8*Dt),2))),rankCorrelation,rejected,rawPid=candidates[raw],selectedPid=candidates[selected],regret=truth[selected].Tracking/trueBest-1,rawRegret=truth[raw].Tracking/trueBest-1,currentRegret=truth[baseline].Tracking/trueBest-1,worseCurrent=truth[selected].Tracking>truth[baseline].Tracking*1.000001,chosenCost=truth[selected].Tracking,currentCost=truth[baseline].Tracking,bestCost=trueBest,matchedRegret=matchedTruth[selected].Tracking/matchedBest-1,matchedCost=matchedTruth[selected].Tracking,matchedCurrent=matchedTruth[baseline].Tracking,matchedBest,matchedWorseCurrent=matchedTruth[selected].Tracking>matchedTruth[baseline].Tracking*1.000001};output.WriteLine(JsonSerializer.Serialize(row));output.Flush();
  }
  if(++count%4==0)Console.WriteLine($"Completed {count*4} fits: {mode}, D={derivative}, seed={seed}");
 }
 }
 static double Std(double[] v){double mean=v.Average();return Math.Sqrt(v.Average(x=>(x-mean)*(x-mean)));}
}
