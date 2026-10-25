using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Threading;
using System.Diagnostics;
using System.Text.Json;
using PIDSupporter;
internal static class EOptimalChecks
{
 const double Dt=.05;const int N=600;
 static WorkBudget Budget()=>new WorkBudget(CancellationToken.None);
 sealed class Data {internal double[] R=null!,U=null!,Y=null!,D=null!;internal State End=null!;}
 sealed class State {internal double Y,U,I,E;internal State Copy()=>new State{Y=Y,U=U,I=I,E=E};}
 sealed class Design {internal double[] Allocation=null!,Wave=null!;internal double[,] Q=null!;}
 static double[] Weights(double[] pid)=>Enumerable.Range(1,60).Select(k=>{double w=2*Math.PI*k/30,react=w*pid[2]-1/(w*pid[1]);return Math.Max(.5*Math.Max(0,Math.Min(1,(.4-k/30.0)/.2)),1/Math.Sqrt(1+react*react));}).ToArray();
 static double[] Wave(double[] pid,double[] allocation,int seed,int n=600)
 {
  var random=new Random(seed);var weights=Weights(pid);var phases=Enumerable.Range(0,60).Select(_=>random.NextDouble()*2*Math.PI).ToArray();double energy=0,baseEnergy=weights.Sum(w=>w*w);
  for(int k=0;k<60;k++)energy+=Math.Pow(weights[k]*allocation[AdaptiveExcitation.Band((k+1)/30.0)],2);
  double scale=Math.Sqrt(baseEnergy/energy),cap=Math.Min(5,.2/pid[0]);var r=new double[n];
  for(int i=0;i<n;i++){double t=i*Dt,edge=Math.Min(t,n*Dt-t),envelope=edge>=1?1:.5*(1-Math.Cos(Math.PI*edge)),sum=0;for(int k=0;k<60;k++)sum+=weights[k]*allocation[AdaptiveExcitation.Band((k+1)/30.0)]*scale*Math.Sin(2*Math.PI*(k+1)*t/30+phases[k]);r[i]=cap*envelope*Math.Max(-1,Math.Min(1,sum/20));}return r;
 }
 static Data Simulate(double a,double b,double[] pid,double[] wave,double[] disturbance,State initial,double offset=0)
 {
  var s=initial.Copy();int n=wave.Length;var d=new Data{R=wave,U=new double[n],Y=new double[n],D=disturbance};
  for(int i=0;i<n;i++){s.Y=a*s.Y+b*s.U+offset+disturbance[i];double error=wave[i]-s.Y;s.U=NativeAutoTune.PidStep(pid,error,s.E,ref s.I,Dt,-1,1,false);s.E=error;d.Y[i]=s.Y;d.U[i]=s.U;}d.End=s;return d;
 }
 static (double[,],double[]) Project(Data d,int end,int lag)
 {
  var q=NativeLinear.Instruments(d.R,lag,end,lag,Budget());var x=new double[q.GetLength(1),3];var y=new double[q.GetLength(1)];
  for(int i=lag;i<end;i++)for(int j=0;j<q.GetLength(1);j++){double z=q[i-lag,j];x[j,0]+=z*d.Y[i-1];x[j,1]+=z*d.U[i-1];x[j,2]+=z;y[j]+=z*d.Y[i];}return(x,y);
 }
 static double[] Fit(Data d,int end,int lag,bool constrained)
 {
  var (x,y)=Project(d,end,lag);
  if(!constrained)return NativeLinear.LeastSquares(x,y,Budget())??throw new Exception("Rank deficient fit");
  return NativeLinear.FitConstrained(x,y,1,1,Dt,Budget()).OrderBy(c=>c.Loss).First().C;
 }
 static double Sigma(Data d,double[,] q,double us,double ys,int end=300,int lag=40)
 {
  int n=end-lag;double um=0,ym=0;for(int i=lag;i<end;i++){um+=d.U[i-1]/n;ym+=d.Y[i-1]/n;}
  double xx=0,yy=0,xy=0;
  for(int j=0;j<q.GetLength(1);j++){double x=0,y=0;for(int i=lag;i<end;i++){x+=q[i-lag,j]*(d.Y[i-1]-ym)/ys/Math.Sqrt(n);y+=q[i-lag,j]*(d.U[i-1]-um)/us/Math.Sqrt(n);}xx+=x*x;yy+=y*y;xy+=x*y;}
  double largest=.5*(xx+yy+Math.Sqrt((xx-yy)*(xx-yy)+4*xy*xy));return Math.Sqrt(Math.Max(0,largest>0?(xx*yy-xy*xy)/largest:0));
 }
 static double Score(Design design,List<double[]> models,double[] pid,double us,double ys)
 {
  double worst=double.PositiveInfinity;foreach(var c in models){var d=Simulate(c[0],c[1],pid,design.Wave,new double[N],new State());worst=Math.Min(worst,Sigma(d,design.Q,us,ys));}return worst;
 }
 static readonly double[][] Pids=(from p in new[]{.1,.3,.6,1.0} from ti in new[]{.5,2.0,8.0,250.0} from td in new[]{0.0,.2,.8} select new[]{p,ti,td}).ToArray();
 static double Cost(double[] model,double[] pid)
 {
  double cost=0;foreach(double speed in new[]{.5,1.5,3.0})foreach(double amp in new[]{.2,.5,1.0}){var s=new State();double sp=0;for(int i=0;i<300;i++){s.Y=model[0]*s.Y+model[1]*s.U+model[2];sp+=(amp-sp)*(1-Math.Exp(-Dt/speed));double e=sp-s.Y;s.U=NativeAutoTune.PidStep(pid,e,s.E,ref s.I,Dt,-1,1,false);s.E=e;cost+=e*e*Dt;if(!double.IsFinite(cost)||cost>1e12)return 1e12;}}return cost;
 }
 static double Std(double[] values){double mean=values.Average();return Math.Max(1e-6,Math.Sqrt(values.Average(v=>(v-mean)*(v-mean))));}
 internal static void Audit()
 {
  using var output=new StreamWriter("tests/output/eoptimal-fixed30/true-signal.jsonl");int count=0;var costs=new Dictionary<string,(double best,double current)>();
  foreach(var line in File.ReadLines("tests/output/eoptimal-fixed30/results.jsonl")){
   using var document=JsonDocument.Parse(line);var r=document.RootElement;var control=r.GetProperty("control").GetString();
   double[] pid={.4,2,control=="normalD"?.2:1};int seed=r.GetProperty("seed").GetInt32();
   var allocation=r.GetProperty("allocation").EnumerateArray().Select(v=>v.GetDouble()).ToArray();
   double a=r.GetProperty("a").GetDouble(),b=r.GetProperty("b").GetDouble(),us=r.GetProperty("us").GetDouble(),ys=r.GetProperty("ys").GetDouble();
   var wave=Wave(pid,allocation,50000+seed%16);var q=NativeLinear.Instruments(wave,40,300,40,Budget());
   var clean=Simulate(a,b,pid,wave,new double[N],new State());double trueSignalSigma=Sigma(clean,q,us,ys);
   string costKey=r.GetProperty("plant").GetString()+control;
   if(!costs.ContainsKey(costKey))costs[costKey]=(Pids.Min(p=>Cost(new[]{a,b,0.0},p)),Cost(new[]{a,b,0.0},pid));
   var cost=costs[costKey];bool guardRejected=r.GetProperty("fitted")[1].GetDouble()<=0;
   double guardedSelectionLoss=guardRejected?cost.current/cost.best-1:r.GetProperty("selectionLoss").GetDouble();
   bool guardedWorseCurrent=!guardRejected&&r.GetProperty("worseThanCurrent").GetBoolean();
   output.WriteLine(JsonSerializer.Serialize(new{plant=r.GetProperty("plant").GetString(),control,condition=r.GetProperty("condition").GetString(),method=r.GetProperty("method").GetString(),seed,trueSignalSigma,guardRejected,guardedSelectionLoss,guardedWorseCurrent}));
   if(++count%300==0)Console.WriteLine($"Signal-only information audited: {count}");
  }
 }
 internal static void Run()
 {
  Directory.CreateDirectory("tests/output/eoptimal-fixed30");using var output=new StreamWriter("tests/output/eoptimal-fixed30/results.jsonl");
  foreach(string control in new[]{"normalD","strongD"}){
   double[] pid={.4,2,control=="normalD"?.2:1};var allocations=new List<double[]>{new[]{1.0,1,1,1}};
   // Fixed finite candidate library, chosen before results; all tones remain excited.
   for(int b=0;b<4;b++)foreach(double factor in new[]{.25,.5,2.0,4.0}){var al=new[]{1.0,1,1,1};al[b]=factor;allocations.Add(al);}
   var library=allocations.Select(al=>{var w=Wave(pid,al,827);return new Design{Allocation=al,Wave=w,Q=NativeLinear.Instruments(w,40,300,40,Budget())};}).ToArray();
   foreach(string plant in new[]{"fast","slow"})foreach(string condition in new[]{"none","force-low","coupling-low","force","coupling"}){
    double a=plant=="fast"?.95:.99,b=1-a;var trueCosts=Pids.Select(p=>Cost(new[]{a,b,0.0},p)).ToArray();double best=trueCosts.Min(),current=Cost(new[]{a,b,0.0},pid);
    for(int seed=0;seed<32;seed++){
     // Matched +/- physical disturbances; independent fresh collection phases per base seed.
     var random=new Random(10000+seed%16);double phase=random.NextDouble()*2*Math.PI,force=0,z=0,v=0;var dd=new double[680];
     for(int i=0;i<dd.Length;i++){double innovation=Math.Sqrt(-2*Math.Log(Math.Max(1e-12,random.NextDouble())))*Math.Cos(2*Math.PI*random.NextDouble());force=.95*force+Math.Sqrt(1-.95*.95)*innovation;v+=Dt*(-.5*v-z+.8*Math.Sin(2*Math.PI*.18*i*Dt+phase));z+=Dt*v;dd[i]=(condition=="none"?0:condition.StartsWith("force")?b*.3*force:b*.6*z)*(condition.EndsWith("-low")?.1:1)*(seed<16?1:-1);}
     var prep=Simulate(a,b,pid,Wave(pid,new[]{1.0,1,1,1},826,80),dd[..80],new State());
     // Design receives only observed data and known PID; plant coefficients never enter it.
     var watch=Stopwatch.StartNew();var models=new List<double[]>{Fit(prep,80,20,true),Fit(prep,60,15,true),Fit(prep,80,10,true)};double us=Std(prep.U),ys=Std(prep.Y);
     var scores=library.Select(design=>Score(design,models,pid,us,ys)).ToArray();int chosen=Array.IndexOf(scores,scores.Max());double designMs=watch.Elapsed.TotalMilliseconds;
     var heuristic=AdaptiveExcitation.Analyze(prep.R,prep.U,prep.Y,Dt,Budget()).Factors;
     foreach(string method in new[]{"fixed","heuristic","eoptimal"}){
      var allocation=method=="fixed"?allocations[0]:method=="heuristic"?heuristic:library[chosen].Allocation;
      // Collection phases are drawn after allocation; the optimizer never sees this seed.
      var d=Simulate(a,b,pid,Wave(pid,allocation,50000+seed%16),dd[80..],prep.End);
      var projected=NativeLinear.Instruments(d.R,40,300,40,Budget());double actualSigma=Sigma(d,projected,us,ys);
      var fitted=Fit(d,300,40,true);var raw=Fit(d,300,40,false);var predicted=Pids.Select(p=>Cost(fitted,p)).ToArray();int selected=Array.IndexOf(predicted,predicted.Min());
      double freq=0,baseFreq=0;for(int k=1;k<=60;k++){var q=System.Numerics.Complex.Exp(new System.Numerics.Complex(0,2*Math.PI*k/30*Dt));var truth=b/(q-a);var estimate=fitted[1]/(q-fitted[0]);freq+=(estimate-truth).Magnitude*(estimate-truth).Magnitude;baseFreq+=truth.Magnitude*truth.Magnitude;}
      var moment=new double[3];for(int j=0;j<3;j++)for(int i=40;i<300;i++)moment[j]+=d.R[i-j*20]*d.D[i]/260;
      double rollout=0,y=d.Y[399];for(int i=400;i<N;i++){y=fitted[0]*y+fitted[1]*d.U[i-1]+fitted[2];rollout+=Math.Pow(y-d.Y[i],2)/200;}
      output.WriteLine(JsonSerializer.Serialize(new{plant,control,condition,seed,method,allocation,designMs,chosen,predictedSigma=method=="heuristic"?(double?)null:scores[method=="eoptimal"?chosen:0],baselinePredictedSigma=scores[0],actualSigma,us,ys,models,raw,fitted,a,b,moment,
       frequencyErrorPct=100*Math.Sqrt(freq/baseFreq),rolloutRmse=Math.Sqrt(rollout),pid=Pids[selected],selectionLoss=trueCosts[selected]/best-1,worseThanCurrent=trueCosts[selected]>current,
       energy=d.U.Sum(u=>u*u)*Dt,inputPeak=d.U.Max(Math.Abs),spPeak=d.R.Max(Math.Abs),saturation=d.U.Count(u=>Math.Abs(u)>=.99999)/(double)N,outputRms=Math.Sqrt(d.Y.Average(y=>y*y))}));
     }output.Flush();
    }
    Console.WriteLine($"Completed {plant}/{control}/{condition}");
   }
  }
 }
}
