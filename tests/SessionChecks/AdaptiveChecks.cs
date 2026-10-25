using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Threading;
using System.Diagnostics;
using System.Text.Json;
using PIDSupporter;
internal static class AdaptiveChecks
{
 const double Dt=.05;const int N=600;
 static WorkBudget Budget()=>new WorkBudget(CancellationToken.None);
 static double Normal(Random r)=>Math.Sqrt(-2*Math.Log(Math.Max(1e-12,r.NextDouble())))*Math.Cos(2*Math.PI*r.NextDouble());
 sealed class Data { public double[] U=new double[N],Y=new double[N],R=new double[N],Carrier=new double[N],D=new double[N]; public List<object> Updates=new List<object>(); }
 static double[] Weights(double[] pid)=>Enumerable.Range(1,60).Select(k=>{double w=2*Math.PI*k/30,react=w*pid[2]-(pid[1]>=250?0:1/(w*pid[1]));return Math.Max(.5*Math.Max(0,Math.Min(1,(.4-k/30.0)/.2)),1/Math.Sqrt(1+react*react));}).ToArray();
 static Data Collect(double a,double b,double[] pid,int seed,string disturbance,bool adaptive,double sign=1)
 {
  var d=new Data();var rng=new Random(seed);var phaseRng=new Random(826);var phases=Enumerable.Range(0,60).Select(_=>phaseRng.NextDouble()*2*Math.PI).ToArray();var weights=Weights(pid);
  var allocation=new[]{1.0,1,1,1};var old=(double[])allocation.Clone();double change=-10,integral=0,previous=0,force=0,other=0,otherVelocity=0,phase=rng.NextDouble()*2*Math.PI;
  double cap=Math.Min(5,.2/pid[0]);
  // Old fixed waveform uses the same integer harmonics, D weighting, taper and mean correction.
  double SinIntegral(double w)=>Math.Abs(w)<1e-10?1:Math.Sin(w)/w;
  double center=Enumerable.Range(0,60).Sum(k=>weights[k]*Math.Sin(phases[k])*(-SinIntegral(2*Math.PI*(k+1)/30)-.5*(SinIntegral(2*Math.PI*(k+1)/30-Math.PI)+SinIntegral(2*Math.PI*(k+1)/30+Math.PI))))/29;
  for(int i=0;i<N;i++){
   double t=i*Dt;
   if(adaptive&&(i==240||i==320)){
    int end=i-40,start=120;var sw=Stopwatch.StartNew();
    var r=AdaptiveExcitation.Analyze(d.Carrier[start..end],d.U[start..end],d.Y[start..end],Dt,Budget());
    old=(double[])allocation.Clone();double e=0,baseE=weights.Sum(w=>w*w);
    for(int k=0;k<60;k++)e+=Math.Pow(weights[k]*r.Factors[AdaptiveExcitation.Band((k+1)/30.0)],2);
    for(int j=0;j<4;j++)allocation[j]=r.Factors[j]*Math.Sqrt(baseE/e);change=t;
    d.Updates.Add(new{time=t,computeMs=sw.Elapsed.TotalMilliseconds,minimum=r.MinimumSingular,response=r.Response,allocation=(double[])allocation.Clone()});
   }
   double raw=0,baseRaw=0,blend=Math.Max(0,Math.Min(1,t-change));
   for(int k=0;k<60;k++){double v=weights[k]*Math.Sin(2*Math.PI*(k+1)*t/30+phases[k]);int band=AdaptiveExcitation.Band((k+1)/30.0);baseRaw+=v;raw+=v*(old[band]+blend*(allocation[band]-old[band]));}
   double edge=Math.Min(t,30-t),envelope=edge>=1?1:.5*(1-Math.Cos(Math.PI*edge));
   d.Carrier[i]=(baseRaw-center)/20;d.R[i]=cap*envelope*Math.Max(-1,Math.Min(1,(raw-center)/20));
   if(i>0){
    // Exact observed output; disturbance is physical additive force in the state update.
    force=.95*force+Math.Sqrt(1-.95*.95)*Normal(rng);
    otherVelocity+=Dt*(-.5*otherVelocity-other+.8*Math.Sin(2*Math.PI*.18*t+phase));other+=Dt*otherVelocity;
    d.D[i]=disturbance=="none"?0:disturbance.StartsWith("force")?(1-a)*.3*force:(1-a)*.6*other;
    d.D[i]*=sign*(disturbance.EndsWith("-low")?.1:1);
    d.Y[i]=a*d.Y[i-1]+b*d.U[i-1]+d.D[i];
   }
   double error=d.R[i]-d.Y[i];d.U[i]=NativeAutoTune.PidStep(pid,error,previous,ref integral,Dt,-1,1,i==0);previous=error;
  }
  return d;
 }
 static object Fit(Data d,double a,double b,string method,int seed,string disturbance,string plant,string control)
 {
  int start=method=="fixed30"?0:120,end=start+(N-start)/2,lag=40,first=start+lag;
  var source=method=="adaptive-carrier"?d.Carrier:d.R;
  var q=NativeLinear.Instruments(source,first,end,lag,Budget());int rank=q.GetLength(1);
  var x=new double[rank,3];var target=new double[rank];
  for(int i=first;i<end;i++)for(int k=0;k<rank;k++){double z=q[i-first,k];x[k,0]+=z*d.Y[i-1];x[k,1]+=z*d.U[i-1];x[k,2]+=z;target[k]+=z*d.Y[i];}
  var c=NativeLinear.LeastSquares(x,target,Budget())??new[]{double.NaN,double.NaN,double.NaN};
  // Raw (not self-normalized) moments against the known true residual, evaluation only.
  var moment=new double[lag+1];for(int j=0;j<=lag;j++)for(int i=first;i<end;i++)moment[j]+=source[i-j]*d.D[i]/(end-first);
  double free=0,pred=d.Y[end-1];for(int i=end;i<N;i++){pred=c[0]*pred+c[1]*d.U[i-1]+c[2];free+=Math.Pow(pred-d.Y[i],2)/(N-end);if(!double.IsFinite(free)||free>1e12){free=1e12;break;}}
  var candidates=(from p in new[]{.1,.3,.6,1.0} from ti in new[]{.5,2.0,8.0,250.0} from td in new[]{0.0,.2,.8} select new[]{p,ti,td}).ToArray();
  double Cost(double aa,double bb,double cc,double[] pid){double cost=0;foreach(double speed in new[]{.5,1.5,3.0})foreach(double amp in new[]{.2,.5,1.0}){double yy=0,uu=0,integ=0,prev=0,sp=0;for(int i=0;i<300;i++){yy=aa*yy+bb*uu+cc;sp+=(amp-sp)*(1-Math.Exp(-Dt/speed));double er=sp-yy;uu=NativeAutoTune.PidStep(pid,er,prev,ref integ,Dt,-1,1,i==0);prev=er;cost+=er*er*Dt;if(!double.IsFinite(cost)||cost>1e12)return 1e12;}}return cost;}
  var truth=candidates.Select(p=>Cost(a,b,0,p)).ToArray();var predicted=candidates.Select(p=>Cost(c[0],c[1],c[2],p)).ToArray();int selected=Array.IndexOf(predicted,predicted.Min());
  return new{method,seed,disturbance,plant,control,a,b,estimate=c,rank,moment,rolloutRmse=Math.Sqrt(free),pid=candidates[selected],selectionLoss=truth[selected]/truth.Min()-1,
   inputEnergy=d.U.Sum(v=>v*v)*Dt,inputPeak=d.U.Max(Math.Abs),spPeak=d.R.Max(Math.Abs),saturation=d.U.Count(v=>Math.Abs(v)>=.99999)/(double)N,outputRms=Math.Sqrt(d.Y.Average(v=>v*v)),updates=d.Updates};
 }
 internal static void Numerics()
 {
  var matrix=new double[100,4];double[] singular={1,.1,1e-8,1e-17};
  double[,] v={{.5,.5,.5,.5},{.5,-.5,.5,-.5},{.5,.5,-.5,-.5},{.5,-.5,-.5,.5}};
  for(int i=0;i<100;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)matrix[i,j]+=Math.Sqrt(.02)*Math.Cos(Math.PI*(i+.5)*(k+1)/100)*singular[k]*v[j,k];
  var svd=NumericalSvd.Decompose(matrix,Budget());if(svd.Rank!=3)throw new Exception("Mixed weak singular direction was lost");
  double error=0,orthogonal=0;
  for(int i=0;i<100;i++)for(int j=0;j<4;j++){double reconstructed=0;for(int k=0;k<4;k++)reconstructed+=svd.Rotated[i,k]*svd.V[j,k];error=Math.Max(error,Math.Abs(matrix[i,j]-reconstructed));}
  for(int j=0;j<4;j++)for(int k=j+1;k<4;k++)if(svd.S[j]>svd.Cutoff&&svd.S[k]>svd.Cutoff){double dot=0;for(int i=0;i<100;i++)dot+=svd.Rotated[i,j]*svd.Rotated[i,k]/(svd.S[j]*svd.S[k]);orthogonal=Math.Max(orthogonal,Math.Abs(dot));}
  if(error>1e-13||orthogonal>1e-12)throw new Exception("SVD reconstruction/orthogonality");
  Console.WriteLine($"Direct SVD check: rank3; 1e-8 direction retained; error {error:G3}, orthogonality {orthogonal:G3}");
 }
 internal static void Run()
 {
  Numerics();
  var diagonal=new double[,]{{1,0,0},{0,1e-8,0},{0,0,1e-17}};var svd=NumericalSvd.Decompose(diagonal,Budget());if(svd.Rank!=2)throw new Exception("Numerical rank cutoff");
  Directory.CreateDirectory("tests/output/adaptive-physical-disturbance");
  bool lowOnly=Environment.GetEnvironmentVariable("PID_SYNTH_LOW_ONLY")=="1";
  bool append=Environment.GetEnvironmentVariable("PID_SYNTH_APPEND_NEGATIVE")=="1";
  using var output=new StreamWriter("tests/output/adaptive-physical-disturbance/results.jsonl",append||lowOnly);
  int runs=0;foreach(string plant in new[]{"fast","slow"})foreach(string control in new[]{"normalD","strongD"})foreach(string disturbance in (lowOnly?new[]{"force-low","coupling-low"}:new[]{"none","force","coupling","force-low","coupling-low"})){
   double a=plant=="fast"?.95:.99,b=1-a;var pid=new[]{.4,2.0,control=="normalD"?.2:1.0};
   for(int seed=append?32:0;seed<64;seed++){
    var fixedData=Collect(a,b,pid,10000+seed%32,disturbance,false,seed<32?1:-1);var adapted=Collect(a,b,pid,10000+seed%32,disturbance,true,seed<32?1:-1);
    foreach(string method in new[]{"fixed30","fixed24","adaptive-injected","adaptive-carrier"}){var row=Fit(method.StartsWith("fixed")?fixedData:adapted,a,b,method,seed,disturbance,plant,control);output.WriteLine(JsonSerializer.Serialize(row));}output.Flush();runs++;
   }
   Console.WriteLine($"Completed {plant}/{control}/{disturbance}: {runs} paired seeds");
  }
 }
}
