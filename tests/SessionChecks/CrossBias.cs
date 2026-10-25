using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Collections.Generic;
using System.Reflection;
using System.Threading;
using PIDSupporter;
internal static class CrossBias
{
 const string Root="tests/output/cross-sp-iv-20261020";
 static double[] Arr(JsonElement e,string k)=>e.GetProperty(k).EnumerateArray().Select(x=>x.GetDouble()).ToArray();
 static double Noise(Random r)=>Math.Sqrt(-2*Math.Log(Math.Max(r.NextDouble(),1e-15)))*Math.Cos(2*Math.PI*r.NextDouble());
 static (double[] r,double[] u,double[] y,double[] e) Sim(double[] pid,double[] phase,double amp,double[] baseR,int seed,bool colored,bool adaptive,double[]? replay=null,bool crossMode=true)
 {
  var rng=new Random(seed);var exc=new AdaptiveExciter(pid,phase,amp);var cross=new CrossExciter(pid,phase,amp);var r=new double[600];var u=new double[600];var y=new double[600];var e=new double[600];double x=0,integral=0,prev=0,noise=0,lastNoise=0;
  for(int k=0;k<600;k++)
  {
   r[k]=replay!=null?replay[k]:adaptive?(crossMode?cross.Next(k,u,y):exc.Next(k,u,y)):baseR[k];
   noise=colored?.8*noise+.006*Noise(rng):.01*Noise(rng);
   if(k>0)x=.92*x+.08*u[k-1];y[k]=x+noise;e[k]=noise-.92*lastNoise;lastNoise=noise;
   double error=r[k]-y[k];u[k]=NativeAutoTune.PidStep(pid,error,prev,ref integral,.05,-1,1,false);prev=error;
  }
  return(r,u,y,e);
 }
 static double[] Fit(double[] z,double[] u,double[] y)
 {
  var budget=new WorkBudget(CancellationToken.None);
  var q=(double[,])typeof(AdaptiveIv).GetMethod("Instruments",BindingFlags.NonPublic|BindingFlags.Static)!.Invoke(null,new object[]{z,40,300,40,budget})!;
  int rank=q.GetLength(1);var xp=new double[rank,3];var yp=new double[rank];
  for(int i=40;i<300;i++)for(int k=0;k<rank;k++)
  {double v=q[i-40,k];yp[k]+=v*y[i];xp[k,0]+=v*y[i-1];xp[k,1]+=v*u[i-1];xp[k,2]+=v;}
  var a=new double[3,3];var b=new double[3];
  for(int j=0;j<3;j++){for(int k=0;k<rank;k++){b[j]+=xp[k,j]*yp[k]/260;for(int h=0;h<3;h++)a[j,h]+=xp[k,j]*xp[k,h]/260;}a[j,j]+=1e-8;}
  return(double[])typeof(AdaptiveIv).GetMethod("Solve",BindingFlags.NonPublic|BindingFlags.Static)!.Invoke(null,new object[]{a,b,budget})!;
 }
 internal static void Run()
 {
  var inputs=JsonDocument.Parse(File.ReadAllText("tests/output/phase-iv-20261020/design-input.json")).RootElement;var rows=new List<object>();
  foreach(int controller in new[]{0,2})foreach(bool colored in new[]{false,true})
  {
   var d=inputs[controller];var pid=Arr(d,"pid");var phase=Arr(d,"phases");var baseR=Arr(d,"wave");double amp=d.GetProperty("amplitude").GetDouble();
   for(int seed=0;seed<128;seed++)
   {
    var baseline=Sim(pid,phase,amp,baseR,9000+seed,colored,false);var adaptive=Sim(pid,phase,amp,baseR,9000+seed,colored,true);
    var fft=Sim(pid,phase,amp,baseR,9000+seed,colored,true,null,false);
    var replay=Sim(pid,phase,amp,baseR,19000+seed,colored,false,adaptive.r);
    foreach(string method in new[]{"baseline","fft_sp_iv","cross_sp_iv","cross_base_iv","replay_independent_noise"})
    {
     var s=method=="baseline"?baseline:method=="replay_independent_noise"?replay:method=="fft_sp_iv"?fft:adaptive;
     var z=method=="cross_base_iv"?baseR:s.r;var coef=Fit(z,s.u,s.y);
     var moments=new double[41];double erms=Math.Sqrt(s.e.Skip(40).Take(260).Average(x=>x*x));
     for(int lag=0;lag<=40;lag++)
     {double zm=z.Skip(40-lag).Take(260).Average(),zs=Math.Sqrt(z.Skip(40-lag).Take(260).Average(x=>(x-zm)*(x-zm)));for(int i=40;i<300;i++)moments[lag]+=(z[i-lag]-zm)*s.e[i]/(260*Math.Max(1e-12,zs*erms));}
     rows.Add(new{controller,colored,seed,method,coefficients=coef,moments,saturation=s.u.Count(x=>Math.Abs(x)>=.999999)/600.0});
    }
   }
   Console.WriteLine($"Bias audit controller={controller} colored={colored} finished");
  }
  File.WriteAllText(Root+"/bias-monte-carlo.json",JsonSerializer.Serialize(rows));
 }
}
