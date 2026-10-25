using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Text.Json;
using System.Threading;
using PIDSupporter;
internal static class EnsembleReview
{
 static double[] A(JsonElement e,string k)=>e.GetProperty(k).EnumerateArray().Select(v=>v.GetDouble()).ToArray();
 internal static void Run()
 {
  string folder="tests/output/ensemble-review-20261020";Directory.CreateDirectory(folder);
  var runs=JsonDocument.Parse(File.ReadAllText("tests/output/game-log-20261020-121006/runs.json")).RootElement;
  foreach(var run in runs.EnumerateArray())
  {
   if(!run.TryGetProperty("record",out var d))continue;
   int index=run.GetProperty("index").GetInt32();var start=run.GetProperty("start");
   if(File.Exists(folder+$"/run-{index}.json"))continue;
   var r=new TuneRecord{U=A(d,"U"),Y=A(d,"Y"),R=A(d,"R"),T=A(d,"T"),Pid=A(d,"Pid"),Integrals=A(d,"Integrals"),InjectedSP=A(d,"InjectedSP"),BaseSP=A(d,"BaseSP"),Speed=start.GetProperty("speed").GetDouble(),Amplitude=d.GetProperty("Amplitude").GetDouble(),Lower=d.GetProperty("Lower").GetDouble(),Upper=d.GetProperty("Upper").GetDouble(),InitialIntegral=d.GetProperty("InitialIntegral").GetDouble(),InitialError=d.GetProperty("InitialError").GetDouble()};
   var budget=new WorkBudget(CancellationToken.None);
   EnsembleLinear.Models.Clear();EnsembleLinear.Select(r,NativeAutoTune.Validate(r),budget,_=>{});
   var models=EnsembleLinear.Models.ToArray();
   var funcs=models.Select(m=>EnsembleMeasure.Create(r,m.Model,budget)).ToArray();
   var searches=new List<PidSearch.Result>();
   var pool=new Dictionary<(double,double,double),double[]>();
   void Add(double[] p){pool[(p[0],p[1],p[2])]=p;}
   Add(r.Pid);Add(A(run.GetProperty("result"),"pid"));
   foreach(var p in NativeAutoTune.Candidates(r.Pid))Add(p);
   for(int m=0;m<models.Length;m++){
    int j=m;var search=PidSearch.Run(r.Pid,p=>funcs[j](p).Tracking,budget,_=>{},false,funcs[j]);
    searches.Add(search);Add(search.Pid);Add(search.DeNominalPid);Add(search.GridPid);
    Console.WriteLine($"run {index} model {m+1}/18 order {models[m].Order} PID {string.Join(",",search.Pid)}");
   }
   // Common candidate support: independent seeded log-uniform slider-grid samples, all modes,
   // plus every model's optimized PID and the original recommendation.
   var random=new Random(20261020);
   for(int k=0;k<768;k++){
    int mode=k%4;
    Add(new[]{Math.Round(Math.Exp(Math.Log(.001)+random.NextDouble()*Math.Log(1000)),3),
     (mode&1)!=0?250:Math.Min(249.9,Math.Round(Math.Exp(Math.Log(.1)+random.NextDouble()*Math.Log(2499)),1)),
     (mode&2)!=0?0:Math.Round(Math.Exp(Math.Log(.1)+random.NextDouble()*Math.Log(1000)),1)});
   }
   var candidates=pool.Values.ToArray();var costs=new double[models.Length][];var nominal=new double[models.Length][];var slew=new double[models.Length][];var effort=new double[models.Length][];
   for(int m=0;m<models.Length;m++){
    int j=m;costs[m]=new double[candidates.Length];nominal[m]=new double[candidates.Length];slew[m]=new double[candidates.Length];effort[m]=new double[candidates.Length];
    for(int k=0;k<candidates.Length;k++){var v=funcs[m](candidates[k]);nominal[m][k]=v.Tracking;slew[m][k]=v.Slew;effort[m][k]=v.Effort;costs[m][k]=PidSearch.Neighborhood(candidates[k],p=>funcs[j](p).Tracking,false);}
   }
   var metadata=models.Select((m,i)=>new{m.Order,m.Ridge,m.Score,coefficients=m.Model.Coefficients,m.Model.Um,m.Model.Us,m.Model.Ym,m.Model.Ys,search=searches[i]}).ToArray();
   File.WriteAllText(folder+$"/run-{index}.json",JsonSerializer.Serialize(new{index,original=r.Pid,logged=A(run.GetProperty("result"),"pid"),metadata,candidates,costs,nominal,slew,effort},new JsonSerializerOptions{WriteIndented=false,NumberHandling=System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals}));
   Console.WriteLine($"DONE {index} candidates {candidates.Length}");
  }
 }
}
