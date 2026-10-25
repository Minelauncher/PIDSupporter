using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Text.Json;
using System.Reflection;
using System.Threading;
using System.Text.Json.Serialization;
using PIDSupporter;
internal static class RefinedIvReview
{
 const string Root="tests/output/refined-iv-20261020";
 static double[] A(JsonElement e,string k)=>e.GetProperty(k).EnumerateArray().Select(v=>v.GetDouble()).ToArray();
 internal sealed class Entry
 {
  public string Method {get;set;}="";
  public int Order {get;set;}
  public int Seed {get;set;}
  public double[] Coefficients {get;set;}=null!;
  public double[] Validation {get;set;}=null!;
  public double[] Final {get;set;}=null!;
  public double Score {get;set;}
  public PidSearch.Result? Search {get;set;}
  public string? Decision {get;set;}
  [JsonIgnore] public NativeLinear Model {get;set;}=null!;
 }
 internal static void Run()
 {
  var runs=JsonDocument.Parse(File.ReadAllText("tests/output/game-log-20261020-121006/runs.json")).RootElement;
  var fitted=JsonDocument.Parse(File.ReadAllText(Root+"/models.json")).RootElement;
  var errorMethod=typeof(NativeLinear).GetMethod("Errors",BindingFlags.Instance|BindingFlags.NonPublic)!;
  var options=new JsonSerializerOptions{NumberHandling=JsonNumberHandling.AllowNamedFloatingPointLiterals};
  foreach(var run in runs.EnumerateArray())
  {
   if(!run.TryGetProperty("record",out var d))continue;
   int index=run.GetProperty("index").GetInt32();var start=run.GetProperty("start");
   var r=new TuneRecord{U=A(d,"U"),Y=A(d,"Y"),R=A(d,"R"),T=A(d,"T"),Pid=A(d,"Pid"),Integrals=A(d,"Integrals"),InjectedSP=A(d,"InjectedSP"),BaseSP=A(d,"BaseSP"),Speed=start.GetProperty("speed").GetDouble(),Amplitude=d.GetProperty("Amplitude").GetDouble(),Lower=d.GetProperty("Lower").GetDouble(),Upper=d.GetProperty("Upper").GetDouble(),InitialIntegral=d.GetProperty("InitialIntegral").GetDouble(),InitialError=d.GetProperty("InitialError").GetDouble()};
   var budget=new WorkBudget(CancellationToken.None);double dt=NativeAutoTune.Validate(r);int n=r.U.Length;
   var old=JsonDocument.Parse(File.ReadAllText($"tests/output/ensemble-review-20261020/run-{index}.json")).RootElement;
   var entries=new List<Entry>();
   void Add(JsonElement m,string method,int order,int seed){
    var net=new NativeLinear{Order=order,Coefficients=A(m,"coefficients"),Um=m.GetProperty("Um").GetDouble(),Us=m.GetProperty("Us").GetDouble(),Ym=m.GetProperty("Ym").GetDouble(),Ys=m.GetProperty("Ys").GetDouble()};
    var validation=(double[])errorMethod.Invoke(net,new object[]{r,dt,n/2,n*2/3,budget})!;
    var final=(double[])errorMethod.Invoke(net,new object[]{r,dt,n*2/3,n,budget})!;
    entries.Add(new Entry{Method=method,Order=order,Seed=seed,Coefficients=net.Coefficients,Model=net,Validation=validation,Final=final,Score=validation.Max()});
   }
   int seed=0;
   foreach(var m in old.GetProperty("metadata").EnumerateArray())Add(m,"current",m.GetProperty("Order").GetInt32(),seed++);
   foreach(var m in fitted.EnumerateArray())
    if(m.GetProperty("index").GetInt32()==index && m.GetProperty("status").GetString()=="converged")
     Add(m,m.GetProperty("method").GetString()!,m.GetProperty("order").GetInt32(),m.GetProperty("seed").GetInt32());
   // Representatives chosen using validation only. All seed errors still exported.
   var reps=entries.Where(m=>NativeLinear.Finite(m.Score)).GroupBy(m=>(m.Method,m.Order)).Select(g=>g.OrderBy(m=>m.Score).First()).ToArray();
   var funcs=reps.Select(m=>EnsembleMeasure.Create(r,m.Model,budget)).ToArray();
   var pool=new Dictionary<(double,double,double),double[]>();
   void Put(double[] p){pool[(p[0],p[1],p[2])]=p;}
   foreach(var p in old.GetProperty("candidates").EnumerateArray())Put(p.EnumerateArray().Select(v=>v.GetDouble()).ToArray());
   for(int m=0;m<reps.Length;m++){
    int j=m;var s=PidSearch.Run(r.Pid,p=>funcs[j](p).Tracking,budget,_=>{},false,funcs[j]);
    reps[m].Search=s;reps[m].Decision=NativeAutoTune.DecisionReason(s.RobustCost,s.BaselineRobust);
    Put(s.Pid);Put(s.DeNominalPid);Put(s.GridPid);
    Console.WriteLine($"run {index} {reps[m].Method} order {reps[m].Order} PID {string.Join(",",s.Pid)} {reps[m].Decision}");
   }
   var candidates=pool.Values.ToArray();var costs=new double[reps.Length][];
   for(int m=0;m<reps.Length;m++){
    int j=m;costs[m]=candidates.Select(p=>PidSearch.Neighborhood(p,q=>funcs[j](q).Tracking,false)).ToArray();
   }
   File.WriteAllText(Root+$"/evaluation-{index}.json",JsonSerializer.Serialize(new{index,original=r.Pid,entries,reps,candidates,costs},options));
   Console.WriteLine($"DONE {index}");
  }
 }
}
