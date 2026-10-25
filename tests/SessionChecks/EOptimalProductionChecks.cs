using System;
using System.Diagnostics;
using System.Threading;
using PIDSupporter;
internal static class EOptimalProductionChecks
{
 internal static void Run(){
  var weights=new double[60];for(int i=0;i<60;i++)weights[i]=1;
  double[] pid={.4,2,.2};var r=EOptimalExcitation.Wave(new[]{1.0,1,1,1},weights,.5,.05,321);
  var u=new double[r.Length];var y=new double[r.Length];double integral=0,previous=0;
  for(int i=0;i<r.Length;i++){y[i]=i==0?0:.95*y[i-1]+.05*u[i-1];double e=r[i]-y[i];u[i]=NativeAutoTune.PidStep(pid,e,previous,ref integral,.05,-1,1,false);previous=e;}
  var pr=new double[80];var pu=new double[80];var py=new double[80];Array.Copy(r,pr,80);Array.Copy(u,pu,80);Array.Copy(y,py,80);
  int events=0;AutoTuneDiagnostics.Sink=(kind,value)=>{if(kind!="eOptimalDesign")return;events++;var t=value.GetType();double score=(double)t.GetProperty("bestScore")!.GetValue(value)!,baseline=(double)t.GetProperty("baseline")!.GetValue(value)!;if(!double.IsFinite(score)||score<baseline||score<1e-8)throw new Exception("E criterion regression");Console.WriteLine(System.Text.Json.JsonSerializer.Serialize(value));};
  var watch=Stopwatch.StartNew();
  var a=EOptimalExcitation.Design(pr,pu,py,.05,Array.Empty<TuneRecord>(),pid,weights,.5,-1,1,new WorkBudget(CancellationToken.None));
  var record=new TuneRecord{U=u,Y=y,InjectedSP=r,InstrumentSP=r};
  var b=EOptimalExcitation.Design(pr,pu,py,.05,new[]{record},pid,weights,.5,-1,1,new WorkBudget(CancellationToken.None));
  // A third-order plant with physical forcing exercises the complete order ensemble.
  for(int i=0;i<r.Length;i++){
   y[i]=(i>0?1.2*y[i-1]+.02*u[i-1]:0)+(i>1?-.47*y[i-2]+.01*u[i-2]:0)+(i>2?.06*y[i-3]+.005*u[i-3]:0)+.0001*Math.Sin(i*.73);
   double e=r[i]-y[i];u[i]=NativeAutoTune.PidStep(pid,e,previous,ref integral,.05,-1,1,false);previous=e;
  }
  Array.Copy(u,pu,80);Array.Copy(y,py,80);
  EOptimalExcitation.Design(pr,pu,py,.05,new[]{record},pid,weights,.5,-1,1,new WorkBudget(CancellationToken.None));
  foreach(var design in new[]{a,b}){double energy=0;for(int k=0;k<60;k++)energy+=Math.Pow(design.Factors[AdaptiveExcitation.Band((k+1)/30.0)],2);if(Math.Abs(energy-60)>1e-8)throw new Exception("energy normalization");foreach(double v in EOptimalExcitation.Wave(design.Factors,weights,.5,.05,54321))if(Math.Abs(v)>.5)throw new Exception("SP cap");}
  var cts=new CancellationTokenSource();cts.Cancel();bool cancelled=false;try{EOptimalExcitation.Design(pr,pu,py,.05,new[]{record},pid,weights,.5,-1,1,new WorkBudget(cts.Token));}catch(OperationCanceledException){cancelled=true;}
  if(!cancelled||events!=3)throw new Exception("cancel/design events");AutoTuneDiagnostics.Sink=null;
  Console.WriteLine($"PASS: production E-optimal, orders1-3, accumulated information, nondecreasing criterion, nominal energy, SP cap, cancellation; three designs {watch.Elapsed.TotalSeconds:0.00}s with yielding budget.");
 }
}
